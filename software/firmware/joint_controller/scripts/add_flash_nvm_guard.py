#!/usr/bin/env python3
"""Configure linker layout and UF2 packaging for the selected flash link target."""

import json
from pathlib import Path
import re
import subprocess
import zlib

Import("env")

XIP_BASE_ADDR = 0x10000000
FLASH_MAP_MACROS = (
    "FLASH_RUNTIME_APP_BASE_OFFSET",
    "FLASH_RUNTIME_APP_LIMIT_OFFSET",
    "FLASH_BOOT_UPDATE_BASE_OFFSET",
    "FLASH_BOOT_UPDATE_SIZE_BYTES",
    "FLASH_BOOT_UPDATE_END_OFFSET",
    "FLASH_SLOT_A_BASE_OFFSET",
    "FLASH_SLOT_A_END_OFFSET",
    "FLASH_SLOT_B_BASE_OFFSET",
    "FLASH_APP_SLOT_SIZE_BYTES",
    "FLASH_NVM_END_OFFSET",
)


def _get_project_option(name: str, default: str) -> str:
    try:
        return env.GetProjectOption(name, default)
    except Exception:
        return default


def _parse_flash_map_macros(project_dir: Path) -> dict[str, int]:
    flash_map_path = project_dir / "include" / "flash_map.h"
    content = flash_map_path.read_text(encoding="utf-8")
    raw_definitions: dict[str, str] = {}
    for line in content.splitlines():
        match = re.match(r"#define\s+([A-Z0-9_]+)\s+(.+)$", line.strip())
        if match:
            raw_definitions[match.group(1)] = match.group(2).split("//", 1)[0].strip()

    cache: dict[str, int] = {}

    def evaluate_macro(name: str) -> int:
        if name in cache:
            return cache[name]
        if name not in raw_definitions:
            raise RuntimeError(f"Could not parse {name} from {flash_map_path}")

        expr = raw_definitions[name]
        expr = re.sub(r"\b(0x[0-9A-Fa-f]+|\d+)[uUlL]+\b", r"\1", expr)
        referenced = set(re.findall(r"\b[A-Z][A-Z0-9_]*\b", expr))
        for token in sorted(referenced, key=len, reverse=True):
            if token == name:
                continue
            if token in raw_definitions:
                expr = re.sub(rf"\b{token}\b", str(evaluate_macro(token)), expr)
        if re.search(r"[^0-9A-Fa-fxX\(\)\+\-\*/\s]", expr):
            raise RuntimeError(f"Unsupported expression while parsing {name}: {expr}")
        cache[name] = int(eval(expr, {"__builtins__": {}}, {}))
        return cache[name]

    values: dict[str, int] = {}
    for macro in FLASH_MAP_MACROS:
        values[macro] = evaluate_macro(macro)
    return values


def _resolve_link_target(project_dir: Path) -> dict[str, int | str]:
    flash_map = _parse_flash_map_macros(project_dir)
    target_name = _get_project_option("custom_flash_link_target", "runtime").strip().lower()

    if target_name == "runtime":
        offset = flash_map["FLASH_RUNTIME_APP_BASE_OFFSET"]
        length = flash_map["FLASH_RUNTIME_APP_LIMIT_OFFSET"] - flash_map["FLASH_RUNTIME_APP_BASE_OFFSET"]
    elif target_name == "boot_update":
        offset = flash_map["FLASH_BOOT_UPDATE_BASE_OFFSET"]
        length = flash_map["FLASH_BOOT_UPDATE_SIZE_BYTES"]
    elif target_name == "slot_a":
        offset = flash_map["FLASH_SLOT_A_BASE_OFFSET"]
        length = flash_map["FLASH_APP_SLOT_SIZE_BYTES"]
    elif target_name == "slot_b":
        offset = flash_map["FLASH_SLOT_B_BASE_OFFSET"]
        length = flash_map["FLASH_APP_SLOT_SIZE_BYTES"]
    else:
        raise RuntimeError(f"Unsupported custom_flash_link_target: {target_name}")

    return {
        "name": target_name,
        "origin_addr": XIP_BASE_ADDR + offset,
        "length": length,
        "tail_addr": XIP_BASE_ADDR + flash_map["FLASH_NVM_END_OFFSET"],
    }


def _find_tool_path(tool_name: str, fallback_candidates: tuple[str, ...]) -> str:
    discovered_path = env.WhereIs(tool_name)
    if discovered_path:
        return discovered_path

    home = Path.home()
    for candidate in fallback_candidates:
        candidate_path = home / candidate
        if candidate_path.exists():
            return str(candidate_path)

    raise RuntimeError(f"Could not locate required build tool: {tool_name}")


def _write_firmware_artifact_manifest(
    build_dir: Path, link_target: dict[str, int | str], bin_payload: bytes
) -> None:
    manifest_path = build_dir / "firmware_manifest.json"
    manifest = {
        "format_version": 1,
        "link_target": str(link_target["name"]),
        "flash_xip_address": f"0x{link_target['origin_addr']:08X}",
        "slot_size_bytes": int(link_target["length"]),
        "image_size_bytes": len(bin_payload),
        "image_crc32": f"0x{zlib.crc32(bin_payload) & 0xFFFFFFFF:08X}",
    }
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"📦 Wrote artifact manifest: {manifest_path}")


def generate_slot_artifacts(source, target, env):
    project_dir = Path(env.subst("$PROJECT_DIR"))
    build_dir = Path(env.subst("$BUILD_DIR"))
    link_target = _resolve_link_target(project_dir)
    bin_path = build_dir / f"{env.subst('$PROGNAME')}.bin"

    if not bin_path.exists():
        raise RuntimeError(f"Expected firmware BIN artifact was not produced: {bin_path}")

    bin_payload = bin_path.read_bytes()
    _write_firmware_artifact_manifest(build_dir, link_target, bin_payload)

    if link_target["name"] == "runtime":
        return

    uf2_path = build_dir / f"{env.subst('$PROGNAME')}.uf2"
    if uf2_path.exists():
        uf2_path.unlink()

    picotool_path = _find_tool_path(
        "picotool",
        (
            ".platformio/packages/tool-picotool-rp2040-earlephilhower/picotool",
        ),
    )
    uf2_family_args = str(env.BoardConfig().get("build.uf2family", "")).strip().split()
    cmd = [
        picotool_path,
        "uf2",
        "convert",
        str(bin_path),
        "-t",
        "bin",
        str(uf2_path),
        "-o",
        f"0x{link_target['origin_addr']:08X}",
    ]
    cmd.extend(uf2_family_args)
    subprocess.run(cmd, check=True)
    print(f"📦 Repacked slot UF2 from BIN for {link_target['name']}: {uf2_path}")


def _replace_once(content: str, pattern: str, replacement: str, description: str) -> str:
    patched, count = re.subn(pattern, replacement, content, count=1, flags=re.MULTILINE)
    if count != 1:
        raise RuntimeError(f"Could not patch {description} in generated linker script")
    return patched


def configure_generated_linker_script(source, target, env):
    project_dir = Path(env.subst("$PROJECT_DIR"))
    link_target = _resolve_link_target(project_dir)
    assert_text = (
        f"    ASSERT(__flash_binary_end <= 0x{link_target['origin_addr'] + link_target['length']:08X},\n"
        '        "ERROR: Application image overlaps reserved flash region")\n'
    )

    ld_path = Path(env.subst("$BUILD_DIR")) / "memmap_default.ld"
    if not ld_path.exists():
        raise RuntimeError(f"Generated linker script not found: {ld_path}")

    content = ld_path.read_text(encoding="utf-8")
    content = _replace_once(
        content,
        r"^\s*FLASH\(rx\)\s*:\s*ORIGIN = 0x[0-9A-Fa-f]+,\s*LENGTH = [0-9A-Za-z]+$",
        f"    FLASH(rx) : ORIGIN = 0x{link_target['origin_addr']:08X}, LENGTH = {link_target['length']}",
        "FLASH memory region",
    )
    content = _replace_once(
        content,
        r"^PROVIDE \( _EEPROM_start = [^;]+ \);$",
        f"PROVIDE ( _EEPROM_start = {link_target['tail_addr']} );",
        "_EEPROM_start symbol",
    )
    content = _replace_once(
        content,
        r"^PROVIDE \( _FS_start\s+= [^;]+ \);$",
        f"PROVIDE ( _FS_start     = {link_target['tail_addr']} );",
        "_FS_start symbol",
    )
    content = _replace_once(
        content,
        r"^PROVIDE \( _FS_end\s+= [^;]+ \);$",
        f"PROVIDE ( _FS_end       = {link_target['tail_addr']} );",
        "_FS_end symbol",
    )
    content = _replace_once(
        content,
        r"    \.partition : \{\n"
        r"        \. = __flash_binary_start \+ 0x2ff0;\n"
        r"        LONG\([^\n]+\)\n"
        r"        LONG\([^\n]+\)\n"
        r"        LONG\([^\n]+\)\n"
        r"        LONG\([^\n]+\)\n"
        r"    \} > FLASH",
        (
            "    .partition : {\n"
            "        . = __flash_binary_start + 0x2ff0;\n"
            f"        LONG({link_target['tail_addr']})\n"
            f"        LONG({link_target['tail_addr']})\n"
            f"        LONG({link_target['tail_addr']})\n"
            f"        LONG({link_target['length']})\n"
            "    } > FLASH"
        ),
        ".partition block",
    )

    if assert_text not in content:
        anchor = '    /* todo assert on extra code */\n}\n'
        if anchor not in content:
            raise RuntimeError(
                f"Could not find insertion anchor in generated linker script: {ld_path}"
            )
        content = content.replace(anchor, assert_text + "\n" + anchor, 1)

    ld_path.write_text(content, encoding="utf-8")
    print(
        "🛡️  Configured linker script "
        f"target={link_target['name']} origin=0x{link_target['origin_addr']:08X} "
        f"length={link_target['length']} ({ld_path})"
    )


env.AddPreAction("$BUILD_DIR/${PROGNAME}.elf", configure_generated_linker_script)
env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", generate_slot_artifacts)
