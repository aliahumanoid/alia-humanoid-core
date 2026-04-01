"""
Offline validator for Jetson controller config against firmware-generated joint_config.json.

Purpose:
  - catch host/Jetson topology drift before hardware is available
  - enforce direct-drive capability invariants for hybrid joints like HIP

This script does not require CAN hardware.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import yaml


def _load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def _fail(errors: list[str], message: str) -> None:
    errors.append(message)


def _validate_profile(profile_name: str, profile_cfg: dict, json_joint: dict, errors: list[str]) -> None:
    dofs = json_joint.get("dofs", [])
    dof_count = json_joint.get("dof_count")
    if dof_count != len(dofs):
        _fail(errors, f"{profile_name}: json dof_count={dof_count} but found {len(dofs)} DOF entries")
        return

    home = profile_cfg.get("home_position_deg", {})
    if not isinstance(home, dict):
        _fail(errors, f"{profile_name}: home_position_deg must be a mapping")
        return

    dof_overrides = profile_cfg.get("dof_overrides", {})
    if dof_overrides is None:
        dof_overrides = {}
    if not isinstance(dof_overrides, dict):
        _fail(errors, f"{profile_name}: dof_overrides must be a mapping")
        return

    valid_indices = {int(dof["index"]) for dof in dofs}
    for raw_idx, override_cfg in dof_overrides.items():
        idx = int(raw_idx)
        if idx not in valid_indices:
            _fail(errors, f"{profile_name}: dof_overrides[{idx}] does not match any DOF in joint_config.json")
            continue
        if not isinstance(override_cfg, dict):
            _fail(errors, f"{profile_name}: dof_overrides[{idx}] must be a mapping")
            continue
        gains_cfg = override_cfg.get("gains")
        if gains_cfg is not None and not isinstance(gains_cfg, dict):
            _fail(errors, f"{profile_name}: dof_overrides[{idx}].gains must be a mapping")
            continue
        if isinstance(gains_cfg, dict):
            for gain_name in ("outer", "inner"):
                block = gains_cfg.get(gain_name)
                if block is None:
                    continue
                if not isinstance(block, dict):
                    _fail(errors, f"{profile_name}: dof_overrides[{idx}].gains.{gain_name} must be a mapping")
                    continue
                missing = [k for k in ("kp", "ki", "kd") if k not in block]
                if missing:
                    _fail(errors, f"{profile_name}: dof_overrides[{idx}].gains.{gain_name} missing keys {missing}")

    for dof in dofs:
        idx = int(dof["index"])
        if idx not in [int(k) for k in home.keys()]:
            _fail(errors, f"{profile_name}: missing home_position_deg for DOF {idx} ({dof['name']})")
            continue

        home_deg = float(home[str(idx)] if str(idx) in home else home[idx])
        min_deg = float(dof["min_angle"])
        max_deg = float(dof["max_angle"])
        if not (min_deg <= home_deg <= max_deg):
            _fail(
                errors,
                f"{profile_name}: home_position_deg[{idx}]={home_deg:.1f}° outside limits [{min_deg:.1f}°, {max_deg:.1f}°]",
            )

        drive_type = dof.get("drive_type")
        motor_count = int(dof.get("motor_count", 0))

        if drive_type == "direct_drive":
            if motor_count != 1:
                _fail(errors, f"{profile_name} DOF {idx}: direct_drive must have motor_count=1, got {motor_count}")
            if dof.get("supports_pretension") is not False:
                _fail(errors, f"{profile_name} DOF {idx}: direct_drive must disable supports_pretension")
            if dof.get("supports_recalc_offset") is not False:
                _fail(errors, f"{profile_name} DOF {idx}: direct_drive must disable supports_recalc_offset")
            if dof.get("supports_auto_mapping") is not False:
                _fail(errors, f"{profile_name} DOF {idx}: direct_drive must disable supports_auto_mapping")
            if dof.get("supports_slack_diag") is not False:
                _fail(errors, f"{profile_name} DOF {idx}: direct_drive must disable supports_slack_diag")
            if dof.get("supports_retension_probe") is not False:
                _fail(errors, f"{profile_name} DOF {idx}: direct_drive must disable supports_retension_probe")
            if dof.get("supports_outer_impedance") is not True:
                _fail(errors, f"{profile_name} DOF {idx}: direct_drive must keep supports_outer_impedance=true")
        elif drive_type == "antagonistic_tendon":
            if motor_count != 2:
                _fail(errors, f"{profile_name} DOF {idx}: antagonistic_tendon must have motor_count=2, got {motor_count}")
        else:
            _fail(errors, f"{profile_name} DOF {idx}: unknown drive_type={drive_type!r}")


def validate(controller_yaml: Path, joint_json: Path) -> int:
    cfg = _load_yaml(controller_yaml)
    joint_cfg = _load_json(joint_json)

    errors: list[str] = []
    joints_yaml = cfg.get("joints", {})
    joints_json = joint_cfg.get("joints", {})

    if not joints_yaml:
        _fail(errors, "controller.yaml: missing joints section")
    if not joints_json:
        _fail(errors, "joint_config.json: missing joints section")

    default_joint = cfg.get("default_joint")
    if default_joint and default_joint not in joints_yaml:
        _fail(errors, f"default_joint={default_joint!r} not found in controller.yaml joints")

    for profile_name, profile_cfg in joints_yaml.items():
        if profile_name not in joints_json:
            _fail(errors, f"{profile_name}: not found in joint_config.json")
            continue
        _validate_profile(profile_name, profile_cfg, joints_json[profile_name], errors)

    if errors:
        print("Jetson config validation FAILED:")
        for err in errors:
            print(f"  - {err}")
        return 1

    print("Jetson config validation OK")
    print(f"  controller.yaml: {controller_yaml}")
    print(f"  joint_config.json: {joint_json}")
    print(f"  validated profiles: {', '.join(sorted(joints_yaml.keys()))}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate Jetson controller config against joint_config.json")
    parser.add_argument(
        "--yaml",
        type=Path,
        default=Path(__file__).parent / "config" / "controller.yaml",
        help="Path to controller.yaml",
    )
    parser.add_argument(
        "--json",
        type=Path,
        default=Path(__file__).parent.parent.parent / "joint_config.json",
        help="Path to joint_config.json",
    )
    args = parser.parse_args()
    return validate(args.yaml.resolve(), args.json.resolve())


if __name__ == "__main__":
    raise SystemExit(main())
