"""
Minimal CAN firmware updater for RP2350 joint controllers.

Default behavior:
- enter maintenance
- stream the candidate image to the inactive slot
- verify it in place

Optional activation performs the full staged flow:
- mark the verified candidate as pending
- reboot through the boot/update selector
- wait for the candidate image to announce itself
- confirm the candidate and exit maintenance

Optional rollback validation performs a deliberate failed first-boot flow:
- mark the verified candidate as pending
- reboot into the candidate slot
- intentionally skip confirmation
- reboot again and verify the controller returns to the previous stable slot
"""
from __future__ import annotations

import argparse
import asyncio
import binascii
import json
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from .can_bus import CanBus
from .config import ControllerConfig, JointControlConfig, load_config
from .protocol import (
    CAN_ID_FW_UPDATE_INFO,
    CAN_ID_FW_UPDATE_PROGRESS,
    CAN_ID_FW_UPDATE_STATUS,
    CAN_ID_FW_UPDATE_UID,
    FW_UPDATE_ERR_IMAGE_CRC_MISMATCH,
    FW_UPDATE_ERR_NONE,
    FW_UPDATE_ERR_VERIFY_FAILED,
    FW_UPDATE_EVT_ACTIVATE_OK,
    FW_UPDATE_EVT_BEGIN_ACCEPTED,
    FW_UPDATE_EVT_CANDIDATE_BOOT_OK,
    FW_UPDATE_EVT_CONFIRM_OK,
    FW_UPDATE_EVT_ERROR,
    FW_UPDATE_EVT_INFO_READY,
    FW_UPDATE_EVT_MAINTENANCE_EXITED,
    FW_UPDATE_EVT_MAINTENANCE_ENTERED,
    FW_UPDATE_EVT_PAGE_COMMITTED,
    FW_UPDATE_EVT_VERIFY_OK,
    FirmwareUpdateInfo,
    FirmwareUpdateProgress,
    FirmwareUpdateStatus,
    FirmwareUpdateUid,
    decode_fw_update_info,
    decode_fw_update_progress,
    decode_fw_update_status,
    decode_fw_update_uid,
    encode_fw_update_abort,
    encode_fw_update_activate,
    encode_fw_update_begin,
    encode_fw_update_confirm,
    encode_fw_update_end,
    encode_fw_update_enter_maintenance,
    encode_fw_update_exit_maintenance,
    encode_fw_update_get_info,
    encode_fw_update_meta_a,
    encode_fw_update_meta_b,
    encode_fw_update_page_begin,
    encode_fw_update_page_frag,
    encode_fw_update_reboot,
    encode_fw_update_verify,
)

logger = logging.getLogger("jetson_controller.fw_update")

FLASH_PAGE_SIZE = 256
PAGE_FRAGMENT_SIZE = 6
# Bench-validated default pacing for the current SLCAN -> MCP2515 transport.
FW_UPDATE_INTERFRAME_DELAY_S = 0.0005
FW_UPDATE_CTRL_DELAY_S = 0.00075
FW_UPDATE_PAGE_BEGIN_DELAY_S = 0.00075
FW_BOOT_RECEIVING = 2
FW_BOOT_VERIFIED = 3
FW_BOOT_PENDING_TEST = 4
FW_BOOT_CANDIDATE_RUNNING = 5
FW_IMAGE_SLOT_NONE = 0
EXPECTED_VERIFY_ERROR_CODES = {
    "image_crc_mismatch": FW_UPDATE_ERR_IMAGE_CRC_MISMATCH,
    "verify_failed": FW_UPDATE_ERR_VERIFY_FAILED,
}


@dataclass
class UpdateArtifact:
    manifest_path: Path
    image_path: Path
    target_slot: int
    image_size_bytes: int
    image_crc32: int
    image_bytes: bytes


@dataclass(frozen=True)
class UpdateTimingConfig:
    ctrl_gap_s: float = FW_UPDATE_CTRL_DELAY_S
    page_begin_gap_s: float = FW_UPDATE_PAGE_BEGIN_DELAY_S
    frag_gap_s: float = FW_UPDATE_INTERFRAME_DELAY_S


@dataclass
class UpdateSnapshot:
    uid: Optional[FirmwareUpdateUid] = None
    info: Optional[FirmwareUpdateInfo] = None
    progress: Optional[FirmwareUpdateProgress] = None
    status: Optional[FirmwareUpdateStatus] = None


def info_matches_candidate_boot(info: FirmwareUpdateInfo, target_slot: int) -> bool:
    return (
        info.active_slot != target_slot
        and info.pending_slot == target_slot
        and info.boot_state == FW_BOOT_CANDIDATE_RUNNING
        and info.candidate_awaiting_confirmation
    )


def info_matches_pending_activation(info: FirmwareUpdateInfo, target_slot: int) -> bool:
    return (
        info.active_slot != target_slot
        and info.pending_slot == target_slot
        and info.boot_state == FW_BOOT_PENDING_TEST
        and info.maintenance_active
        and not info.update_in_progress
    )


def info_matches_confirmed_slot(info: FirmwareUpdateInfo, target_slot: int) -> bool:
    return (
        info.active_slot == target_slot
        and info.pending_slot == FW_IMAGE_SLOT_NONE
        and not info.candidate_awaiting_confirmation
    )


def info_matches_verified_slot(info: FirmwareUpdateInfo, target_slot: int) -> bool:
    return (
        info.active_slot != target_slot
        and info.pending_slot == FW_IMAGE_SLOT_NONE
        and info.boot_state == FW_BOOT_VERIFIED
        and info.maintenance_active
        and not info.update_in_progress
        and not info.candidate_awaiting_confirmation
    )


def info_matches_rolled_back_slot(info: FirmwareUpdateInfo, stable_slot: int) -> bool:
    return (
        info.active_slot == stable_slot
        and info.pending_slot == FW_IMAGE_SLOT_NONE
        and not info.update_in_progress
        and not info.candidate_awaiting_confirmation
    )


def info_matches_receiving_session(info: FirmwareUpdateInfo, target_slot: int) -> bool:
    return (
        info.active_slot != target_slot
        and info.pending_slot == FW_IMAGE_SLOT_NONE
        and info.boot_state == FW_BOOT_RECEIVING
        and info.maintenance_active
        and info.update_in_progress
    )


def setup_logging(verbose: bool = False) -> None:
    root = logging.getLogger()
    root.setLevel(logging.DEBUG)
    handler = logging.StreamHandler()
    handler.setLevel(logging.DEBUG if verbose else logging.INFO)
    handler.setFormatter(
        logging.Formatter(
            "%(asctime)s.%(msecs)03d %(levelname)-7s [%(name)s] %(message)s",
            datefmt="%H:%M:%S",
        )
    )
    root.handlers.clear()
    root.addHandler(handler)
    logging.getLogger("can").setLevel(logging.WARNING)


def load_update_artifact(manifest_path: str) -> UpdateArtifact:
    manifest = Path(manifest_path).resolve()
    with manifest.open("r", encoding="utf-8") as fh:
        payload = json.load(fh)

    image_path = manifest.with_name("firmware.bin")
    image_bytes = image_path.read_bytes()

    link_target = str(payload["link_target"]).strip().lower()
    slot_map = {"slot_a": 1, "slot_b": 2}
    if link_target not in slot_map:
        raise ValueError(f"Manifest {manifest} is not a slot artifact: link_target={link_target}")

    image_size_bytes = int(payload["image_size_bytes"])
    image_crc32 = int(str(payload["image_crc32"]), 16)
    actual_crc32 = binascii.crc32(image_bytes[:image_size_bytes]) & 0xFFFFFFFF
    if len(image_bytes) < image_size_bytes:
        raise ValueError(
            f"Artifact {image_path} is shorter than manifest image_size_bytes "
            f"({len(image_bytes)} < {image_size_bytes})"
        )
    if actual_crc32 != image_crc32:
        raise ValueError(
            f"Artifact CRC32 mismatch for {image_path}: "
            f"manifest=0x{image_crc32:08X} actual=0x{actual_crc32:08X}"
        )

    return UpdateArtifact(
        manifest_path=manifest,
        image_path=image_path,
        target_slot=slot_map[link_target],
        image_size_bytes=image_size_bytes,
        image_crc32=image_crc32,
        image_bytes=image_bytes[:image_size_bytes],
    )


def maybe_corrupt_artifact(artifact: UpdateArtifact,
                           corrupt_byte_offset: Optional[int]) -> UpdateArtifact:
    if corrupt_byte_offset is None:
        return artifact

    if corrupt_byte_offset < 0 or corrupt_byte_offset >= artifact.image_size_bytes:
        raise ValueError(
            f"corrupt_byte_offset={corrupt_byte_offset} is outside image size "
            f"({artifact.image_size_bytes})"
        )

    mutated = bytearray(artifact.image_bytes)
    mutated[corrupt_byte_offset] ^= 0x01
    logger.warning(
        "Corrupting transmitted image byte at offset %d while keeping manifest CRC32=0x%08X",
        corrupt_byte_offset,
        artifact.image_crc32,
    )
    return UpdateArtifact(
        manifest_path=artifact.manifest_path,
        image_path=artifact.image_path,
        target_slot=artifact.target_slot,
        image_size_bytes=artifact.image_size_bytes,
        image_crc32=artifact.image_crc32,
        image_bytes=bytes(mutated),
    )


def consume_update_message(snapshot: UpdateSnapshot,
                           joint_id: int,
                           msg,
                           *,
                           allow_error_codes: Optional[set[int]] = None) -> UpdateSnapshot:
    arb_id = msg.arbitration_id
    data = bytes(msg.data)

    if arb_id == CAN_ID_FW_UPDATE_UID + joint_id and len(data) >= 8:
        snapshot.uid = decode_fw_update_uid(data, joint_id)
    elif arb_id == CAN_ID_FW_UPDATE_INFO + joint_id and len(data) >= 8:
        snapshot.info = decode_fw_update_info(data, joint_id)
    elif arb_id == CAN_ID_FW_UPDATE_PROGRESS + joint_id and len(data) >= 8:
        snapshot.progress = decode_fw_update_progress(data, joint_id)
    elif arb_id == CAN_ID_FW_UPDATE_STATUS + joint_id and len(data) >= 8:
        snapshot.status = decode_fw_update_status(data, joint_id)
        log_fn = logger.debug if snapshot.status.event_code == FW_UPDATE_EVT_PAGE_COMMITTED else logger.info
        log_fn(
            "FW status: %s boot=%s active=%s pending=%s err=%s value=%s",
            snapshot.status.event_name,
            snapshot.status.boot_state_name,
            snapshot.status.active_slot,
            snapshot.status.pending_slot,
            snapshot.status.error_name,
            snapshot.status.value,
        )
        if snapshot.status.event_code == FW_UPDATE_EVT_ERROR or snapshot.status.error_code != FW_UPDATE_ERR_NONE:
            if allow_error_codes is not None and snapshot.status.error_code in allow_error_codes:
                return snapshot
            raise RuntimeError(
                f"Firmware update error from joint {joint_id}: "
                f"{snapshot.status.error_name} (value={snapshot.status.value})"
            )
    return snapshot


async def wait_for_update_snapshot(can_bus: CanBus,
                                   joint_id: int,
                                   *,
                                   timeout_s: float,
                                   expect_event: Optional[int] = None,
                                   require_uid: bool = False,
                                   require_info: bool = False,
                                   allow_error_codes: Optional[set[int]] = None) -> UpdateSnapshot:
    deadline = asyncio.get_running_loop().time() + timeout_s
    snapshot = UpdateSnapshot()

    while asyncio.get_running_loop().time() < deadline:
        msg = await can_bus.recv(timeout=min(0.2, deadline - asyncio.get_running_loop().time()))
        if msg is None:
            continue

        snapshot = consume_update_message(
            snapshot,
            joint_id,
            msg,
            allow_error_codes=allow_error_codes,
        )

        event_ok = expect_event is None or (
            snapshot.status is not None and snapshot.status.event_code == expect_event
        )
        uid_ok = (not require_uid) or snapshot.uid is not None
        info_ok = (not require_info) or snapshot.info is not None
        if event_ok and uid_ok and info_ok:
            return snapshot

    raise TimeoutError(
        f"Timed out waiting for joint {joint_id} update response "
        f"(event={expect_event}, uid={require_uid}, info={require_info})"
    )


async def send_and_wait(can_bus: CanBus,
                        frame: tuple[int, bytes],
                        joint_id: int,
                        *,
                        event_code: int,
                        timeout_s: float = 2.0,
                        allow_error_codes: Optional[set[int]] = None) -> UpdateSnapshot:
    arb_id, data = frame
    await can_bus.send(arb_id, data)
    return await wait_for_update_snapshot(
        can_bus,
        joint_id,
        timeout_s=timeout_s,
        expect_event=event_code,
        allow_error_codes=allow_error_codes,
    )


async def send_with_gap(can_bus: CanBus, frame: tuple[int, bytes], *, delay_s: float = 0.0) -> None:
    await can_bus.send(*frame)
    if delay_s > 0.0:
        await asyncio.sleep(delay_s)


async def send_burst_with_gap(can_bus: CanBus,
                              frames: list[tuple[int, bytes]],
                              *,
                              delay_s: float = FW_UPDATE_INTERFRAME_DELAY_S) -> None:
    last_index = len(frames) - 1
    for index, frame in enumerate(frames):
        await send_with_gap(can_bus, frame, delay_s=delay_s if index < last_index else 0.0)


async def gather_update_info(can_bus: CanBus, joint_id: int) -> UpdateSnapshot:
    await can_bus.send(*encode_fw_update_get_info())
    return await wait_for_update_snapshot(
        can_bus,
        joint_id,
        timeout_s=2.0,
        expect_event=FW_UPDATE_EVT_INFO_READY,
        require_uid=True,
        require_info=True,
    )


async def wait_for_candidate_boot(can_bus: CanBus,
                                  joint_id: int,
                                  target_slot: int,
                                  *,
                                  timeout_s: float = 15.0) -> UpdateSnapshot:
    deadline = asyncio.get_running_loop().time() + timeout_s
    next_poll_at = asyncio.get_running_loop().time() + 0.25
    snapshot = UpdateSnapshot()

    while asyncio.get_running_loop().time() < deadline:
        now = asyncio.get_running_loop().time()
        if now >= next_poll_at:
            await can_bus.send(*encode_fw_update_get_info())
            next_poll_at = now + 0.5

        msg = await can_bus.recv(timeout=min(0.25, deadline - now))
        if msg is None:
            continue

        snapshot = consume_update_message(snapshot, joint_id, msg)

        if snapshot.status is not None and snapshot.status.event_code == FW_UPDATE_EVT_CANDIDATE_BOOT_OK:
            return snapshot

        if snapshot.info is not None:
            if info_matches_candidate_boot(snapshot.info, target_slot):
                return snapshot
            if (
                snapshot.info.pending_slot == FW_IMAGE_SLOT_NONE
                and snapshot.info.active_slot != target_slot
                and not snapshot.info.update_in_progress
            ):
                raise RuntimeError(
                    "Controller rebooted without entering candidate slot "
                    f"(active={snapshot.info.active_slot}, target={target_slot})"
                )

    raise TimeoutError(
        f"Timed out waiting for joint {joint_id} candidate boot "
        f"(target_slot={target_slot})"
    )


async def wait_for_rollback(can_bus: CanBus,
                            joint_id: int,
                            stable_slot: int,
                            *,
                            timeout_s: float = 15.0) -> UpdateSnapshot:
    deadline = asyncio.get_running_loop().time() + timeout_s
    next_poll_at = asyncio.get_running_loop().time() + 0.25
    snapshot = UpdateSnapshot()

    while asyncio.get_running_loop().time() < deadline:
        now = asyncio.get_running_loop().time()
        if now >= next_poll_at:
            await can_bus.send(*encode_fw_update_get_info())
            next_poll_at = now + 0.5

        msg = await can_bus.recv(timeout=min(0.25, deadline - now))
        if msg is None:
            continue

        snapshot = consume_update_message(snapshot, joint_id, msg)

        if snapshot.info is not None and info_matches_rolled_back_slot(snapshot.info, stable_slot):
            return snapshot

    raise TimeoutError(
        f"Timed out waiting for joint {joint_id} rollback "
        f"(stable_slot={stable_slot})"
    )


async def ensure_update_session_reset(can_bus: CanBus,
                                      joint_id: int,
                                      snapshot: UpdateSnapshot) -> UpdateSnapshot:
    info = snapshot.info
    if info is None:
        return snapshot

    if not info.update_in_progress and info.boot_state != 2:
        return snapshot

    logger.warning(
        "Controller reports update already in progress "
        "(boot=%s flags=0x%02X). Sending ABORT_UPDATE before retry.",
        info.boot_state,
        info.flags,
    )
    await can_bus.send(*encode_fw_update_abort())
    return await wait_for_update_snapshot(
        can_bus,
        joint_id,
        timeout_s=3.0,
        expect_event=FW_UPDATE_EVT_INFO_READY,
        require_uid=True,
        require_info=True,
    )


async def stream_image(can_bus: CanBus,
                       joint_id: int,
                       artifact: UpdateArtifact,
                       *,
                       timing: UpdateTimingConfig,
                       start_page_index: int = 0,
                       interrupt_after_pages: Optional[int] = None) -> tuple[bool, int]:
    page_count = (artifact.image_size_bytes + FLASH_PAGE_SIZE - 1) // FLASH_PAGE_SIZE

    if start_page_index < 0 or start_page_index > page_count:
        raise ValueError(
            f"start_page_index={start_page_index} outside valid range 0..{page_count}"
        )

    if start_page_index > 0:
        logger.info(
            "Resuming page stream at page %d/%d",
            start_page_index + 1,
            page_count,
        )

    bytes_streamed = 0
    for page_index in range(start_page_index, page_count):
        start = page_index * FLASH_PAGE_SIZE
        end = min(start + FLASH_PAGE_SIZE, artifact.image_size_bytes)
        page = artifact.image_bytes[start:end]
        page_seq = page_index & 0xFF
        is_final = end >= artifact.image_size_bytes

        await send_with_gap(
            can_bus,
            encode_fw_update_page_begin(page_index, page_seq, page, is_final_page=is_final),
            delay_s=timing.page_begin_gap_s,
        )
        for frag_index, frag_start in enumerate(range(0, len(page), PAGE_FRAGMENT_SIZE)):
            frag = page[frag_start:frag_start + PAGE_FRAGMENT_SIZE]
            is_last_frag = (frag_start + PAGE_FRAGMENT_SIZE) >= len(page)
            await send_with_gap(
                can_bus,
                encode_fw_update_page_frag(page_seq, frag_index, frag),
                delay_s=0.0 if is_last_frag else timing.frag_gap_s,
            )

        status = await wait_for_update_snapshot(
            can_bus,
            joint_id,
            timeout_s=2.0,
            expect_event=FW_UPDATE_EVT_PAGE_COMMITTED,
        )
        if status.status is None or status.status.value != (page_index & 0xFFFF):
            raise RuntimeError(
                f"Unexpected PAGE_COMMITTED acknowledgement for page {page_index}: {status.status}"
            )

        if (page_index + 1) % 32 == 0 or page_index + 1 == page_count:
            logger.info("Flashed page %d/%d", page_index + 1, page_count)
        bytes_streamed += len(page)

        if interrupt_after_pages is not None and (page_index + 1) >= interrupt_after_pages:
            logger.warning(
                "Intentionally interrupting update after committed page %d/%d",
                page_index + 1,
                page_count,
            )
            return False, bytes_streamed

    return True, bytes_streamed


async def reboot_into_candidate(can_bus: CanBus,
                                joint_cfg: JointControlConfig,
                                *,
                                target_slot: int) -> UpdateSnapshot:
    logger.info("Rebooting controller into candidate slot %d", target_slot)

    await send_with_gap(
        can_bus,
        encode_fw_update_reboot(),
        delay_s=0.0,
    )

    candidate = await wait_for_candidate_boot(
        can_bus,
        joint_cfg.joint_id,
        target_slot,
        timeout_s=20.0,
    )
    return candidate


async def activate_verified_candidate(can_bus: CanBus,
                                      joint_cfg: JointControlConfig,
                                      *,
                                      target_slot: int) -> None:
    candidate = await reboot_into_candidate(
        can_bus,
        joint_cfg,
        target_slot=target_slot,
    )

    if candidate.info is not None:
        logger.info(
            "Candidate image booted: active=%d pending=%d boot=%s flags=0x%02X",
            candidate.info.active_slot,
            candidate.info.pending_slot,
            candidate.info.boot_state,
            candidate.info.flags,
        )
    else:
        logger.info("Candidate image booted and emitted CANDIDATE_BOOT_OK")

    await send_and_wait(
        can_bus,
        encode_fw_update_confirm(target_slot),
        joint_cfg.joint_id,
        event_code=FW_UPDATE_EVT_CONFIRM_OK,
        timeout_s=3.0,
    )
    logger.info("Candidate slot confirmed as active")

    await send_and_wait(
        can_bus,
        encode_fw_update_exit_maintenance(),
        joint_cfg.joint_id,
        event_code=FW_UPDATE_EVT_MAINTENANCE_EXITED,
        timeout_s=3.0,
    )

    final = await gather_update_info(can_bus, joint_cfg.joint_id)
    if final.info is None:
        raise RuntimeError("Failed to retrieve final update state after confirmation")
    if not info_matches_confirmed_slot(final.info, target_slot):
        raise RuntimeError(
            "Unexpected final update state after confirmation: "
            f"active={final.info.active_slot} pending={final.info.pending_slot} "
            f"boot={final.info.boot_state} flags=0x{final.info.flags:02X}"
        )
    if final.info.maintenance_active:
        raise RuntimeError("Controller remained in maintenance after EXIT_MAINTENANCE")

    logger.info(
        "Activation flow complete: active_slot=%d pending_slot=%d boot=%d fw=%s",
        final.info.active_slot,
        final.info.pending_slot,
        final.info.boot_state,
        final.info.fw_version,
    )


async def validate_candidate_rollback(can_bus: CanBus,
                                      joint_cfg: JointControlConfig,
                                      *,
                                      target_slot: int,
                                      stable_slot: int) -> None:
    candidate = await reboot_into_candidate(
        can_bus,
        joint_cfg,
        target_slot=target_slot,
    )
    if candidate.info is not None:
        logger.info(
            "Candidate image booted for rollback validation: active=%d pending=%d boot=%s flags=0x%02X",
            candidate.info.active_slot,
            candidate.info.pending_slot,
            candidate.info.boot_state,
            candidate.info.flags,
        )
    else:
        logger.info("Candidate image booted for rollback validation")

    logger.info("Intentionally skipping CONFIRM_UPDATE and forcing a second reboot")
    await send_with_gap(
        can_bus,
        encode_fw_update_reboot(),
        delay_s=0.0,
    )

    rolled_back = await wait_for_rollback(
        can_bus,
        joint_cfg.joint_id,
        stable_slot,
        timeout_s=20.0,
    )
    if rolled_back.info is None:
        raise RuntimeError("Failed to observe rollback state after candidate reboot")

    logger.info(
        "Rollback observed: active=%d pending=%d boot=%d flags=0x%02X",
        rolled_back.info.active_slot,
        rolled_back.info.pending_slot,
        rolled_back.info.boot_state,
        rolled_back.info.flags,
    )

    if rolled_back.info.maintenance_active:
        await send_and_wait(
            can_bus,
            encode_fw_update_exit_maintenance(),
            joint_cfg.joint_id,
            event_code=FW_UPDATE_EVT_MAINTENANCE_EXITED,
            timeout_s=3.0,
        )

    final = await gather_update_info(can_bus, joint_cfg.joint_id)
    if final.info is None:
        raise RuntimeError("Failed to retrieve final update state after rollback")
    if not info_matches_rolled_back_slot(final.info, stable_slot):
        raise RuntimeError(
            "Unexpected final rollback state: "
            f"active={final.info.active_slot} pending={final.info.pending_slot} "
            f"boot={final.info.boot_state} flags=0x{final.info.flags:02X}"
        )
    if final.info.maintenance_active:
        raise RuntimeError("Controller remained in maintenance after rollback EXIT_MAINTENANCE")

    logger.info(
        "Rollback validation complete: active_slot=%d pending_slot=%d boot=%d fw=%s",
        final.info.active_slot,
        final.info.pending_slot,
        final.info.boot_state,
        final.info.fw_version,
    )


def log_update_snapshot_summary(snapshot: UpdateSnapshot, *, label: str) -> None:
    uid_text = snapshot.uid.uid.hex().upper() if snapshot.uid is not None else "UNKNOWN"
    if snapshot.info is None:
        logger.info("%s: uid=%s info=missing", label, uid_text)
        return

    progress_text = (
        str(snapshot.progress.next_page_index)
        if snapshot.progress is not None
        else "n/a"
    )
    logger.info(
        "%s: uid=%s active=%d pending=%d boot=%d maint=%s upd=%s confirm=%s next_page=%s fw=%s",
        label,
        uid_text,
        snapshot.info.active_slot,
        snapshot.info.pending_slot,
        snapshot.info.boot_state,
        snapshot.info.maintenance_active,
        snapshot.info.update_in_progress,
        snapshot.info.candidate_awaiting_confirmation,
        progress_text,
        snapshot.info.fw_version,
    )


async def abort_update_and_exit_maintenance(can_bus: CanBus,
                                            joint_cfg: JointControlConfig,
                                            *,
                                            stable_slot: int,
                                            reason: str) -> None:
    logger.info("%s", reason)
    await send_and_wait(
        can_bus,
        encode_fw_update_abort(),
        joint_cfg.joint_id,
        event_code=FW_UPDATE_EVT_INFO_READY,
        timeout_s=3.0,
    )

    info_after_abort = await gather_update_info(can_bus, joint_cfg.joint_id)
    if info_after_abort.info is None or not info_matches_rolled_back_slot(info_after_abort.info, stable_slot):
        raise RuntimeError(
            "Unexpected controller state after ABORT_UPDATE from verify failure"
        )

    if info_after_abort.info.maintenance_active:
        await send_and_wait(
            can_bus,
            encode_fw_update_exit_maintenance(),
            joint_cfg.joint_id,
            event_code=FW_UPDATE_EVT_MAINTENANCE_EXITED,
            timeout_s=3.0,
        )

    final = await gather_update_info(can_bus, joint_cfg.joint_id)
    if final.info is None or not info_matches_rolled_back_slot(final.info, stable_slot):
        raise RuntimeError("Controller did not return to stable slot after ABORT_UPDATE cleanup")
    if final.info.maintenance_active:
        raise RuntimeError("Controller remained in maintenance after ABORT_UPDATE cleanup")

    logger.info(
        "ABORT_UPDATE cleanup complete: active_slot=%d pending_slot=%d boot=%d fw=%s",
        final.info.active_slot,
        final.info.pending_slot,
        final.info.boot_state,
        final.info.fw_version,
    )


async def cleanup_after_expected_verify_failure(can_bus: CanBus,
                                                joint_cfg: JointControlConfig,
                                                *,
                                                stable_slot: int) -> None:
    await abort_update_and_exit_maintenance(
        can_bus,
        joint_cfg,
        stable_slot=stable_slot,
        reason="Cleaning up after expected verify failure with ABORT_UPDATE",
    )


async def run_info_only(config: ControllerConfig,
                        joint_cfg: JointControlConfig) -> None:
    can_bus = CanBus()
    try:
        await can_bus.connect(config.can_interface, config.can_channel, config.can_bitrate)
        snapshot = await gather_update_info(can_bus, joint_cfg.joint_id)
        log_update_snapshot_summary(snapshot, label="Controller state")
    finally:
        await can_bus.disconnect()


async def run_cleanup_session(config: ControllerConfig,
                              joint_cfg: JointControlConfig) -> None:
    can_bus = CanBus()
    try:
        await can_bus.connect(config.can_interface, config.can_channel, config.can_bitrate)
        snapshot = await gather_update_info(can_bus, joint_cfg.joint_id)
        if snapshot.uid is None or snapshot.info is None:
            raise RuntimeError("Failed to retrieve initial UID/info snapshot for cleanup")

        log_update_snapshot_summary(snapshot, label="Cleanup start")
        info = snapshot.info
        stable_slot = info.active_slot
        pending_slot = info.pending_slot

        if pending_slot != FW_IMAGE_SLOT_NONE and info_matches_candidate_boot(info, pending_slot):
            await validate_candidate_rollback(
                can_bus,
                joint_cfg,
                target_slot=pending_slot,
                stable_slot=stable_slot,
            )
            return

        if pending_slot != FW_IMAGE_SLOT_NONE and info_matches_pending_activation(info, pending_slot):
            await validate_candidate_rollback(
                can_bus,
                joint_cfg,
                target_slot=pending_slot,
                stable_slot=stable_slot,
            )
            return

        if info.update_in_progress or info.boot_state in (FW_BOOT_RECEIVING, FW_BOOT_VERIFIED):
            await abort_update_and_exit_maintenance(
                can_bus,
                joint_cfg,
                stable_slot=stable_slot,
                reason="Cleanup detected RECEIVING/VERIFIED state; aborting partial session",
            )
            return

        if info.maintenance_active:
            logger.info("Cleanup detected maintenance-only state; exiting maintenance")
            await send_and_wait(
                can_bus,
                encode_fw_update_exit_maintenance(),
                joint_cfg.joint_id,
                event_code=FW_UPDATE_EVT_MAINTENANCE_EXITED,
                timeout_s=3.0,
            )
            final = await gather_update_info(can_bus, joint_cfg.joint_id)
            log_update_snapshot_summary(final, label="Cleanup final")
            return

        logger.info("Cleanup found no pending update state to reset")
    finally:
        await can_bus.disconnect()


async def run_update(config: ControllerConfig,
                     joint_cfg: JointControlConfig,
                     artifact: UpdateArtifact,
                     *,
                     timing: UpdateTimingConfig,
                     activate: bool = False,
                     activate_only: bool = False,
                     validate_rollback: bool = False,
                     resume_receiving: bool = False,
                     interrupt_after_pages: Optional[int] = None,
                     expect_verify_error_code: Optional[int] = None) -> None:
    can_bus = CanBus()
    session_started = False

    try:
        await can_bus.connect(config.can_interface, config.can_channel, config.can_bitrate)

        initial = await gather_update_info(can_bus, joint_cfg.joint_id)
        if initial.uid is None or initial.info is None:
            raise RuntimeError("Failed to retrieve initial UID/info snapshot")

        resuming_receiving = (
            resume_receiving
            and initial.info is not None
            and info_matches_receiving_session(initial.info, artifact.target_slot)
        )

        if not resuming_receiving:
            initial = await ensure_update_session_reset(can_bus, joint_cfg.joint_id, initial)
            if initial.uid is None or initial.info is None:
                raise RuntimeError("Failed to retrieve reset update snapshot")
        elif initial.progress is None:
            initial = await gather_update_info(can_bus, joint_cfg.joint_id)
            if initial.uid is None or initial.info is None or initial.progress is None:
                raise RuntimeError("Failed to retrieve live receiving progress for resume")

        board_uid_crc32 = binascii.crc32(initial.uid.uid) & 0xFFFFFFFF
        logger.info(
            "Target joint=%s id=%d uid=%s active_slot=%d pending_slot=%d fw=%s",
            joint_cfg.name,
            joint_cfg.joint_id,
            initial.uid.uid.hex().upper(),
            initial.info.active_slot,
            initial.info.pending_slot,
            initial.info.fw_version,
        )
        logger.info(
            "Update pacing: ctrl=%.1f ms page_begin=%.1f ms frag=%.1f ms",
            timing.ctrl_gap_s * 1000.0,
            timing.page_begin_gap_s * 1000.0,
            timing.frag_gap_s * 1000.0,
        )
        stable_slot = initial.info.active_slot

        if artifact.target_slot == initial.info.active_slot:
            raise RuntimeError(
                f"Manifest targets slot {artifact.target_slot}, but that is currently active. "
                "Build/send the opposite slot artifact."
            )

        if resume_receiving and not resuming_receiving:
            raise RuntimeError(
                "Controller is not in a resumable BOOT_RECEIVING session for the requested target slot"
            )

        if activate_only and info_matches_candidate_boot(initial.info, artifact.target_slot):
            logger.info("Controller is already running candidate slot %d; leaving it unconfirmed",
                        artifact.target_slot)
            return

        if validate_rollback and info_matches_candidate_boot(initial.info, artifact.target_slot):
            logger.info("Controller is already running candidate slot %d; resuming rollback validation",
                        artifact.target_slot)
            await validate_candidate_rollback(
                can_bus,
                joint_cfg,
                target_slot=artifact.target_slot,
                stable_slot=stable_slot,
            )
            return

        if activate and info_matches_candidate_boot(initial.info, artifact.target_slot):
            logger.info("Controller is already running candidate slot %d; resuming confirm flow",
                        artifact.target_slot)
            await send_and_wait(
                can_bus,
                encode_fw_update_confirm(artifact.target_slot),
                joint_cfg.joint_id,
                event_code=FW_UPDATE_EVT_CONFIRM_OK,
                timeout_s=3.0,
            )
            await send_and_wait(
                can_bus,
                encode_fw_update_exit_maintenance(),
                joint_cfg.joint_id,
                event_code=FW_UPDATE_EVT_MAINTENANCE_EXITED,
                timeout_s=3.0,
            )
            final = await gather_update_info(can_bus, joint_cfg.joint_id)
            if final.info is None or not info_matches_confirmed_slot(final.info, artifact.target_slot):
                raise RuntimeError("Candidate confirmation resume did not converge to a stable slot")
            return

        if (activate or activate_only or validate_rollback) and info_matches_verified_slot(
            initial.info, artifact.target_slot
        ):
            logger.info("Controller already has slot %d verified; skipping rewrite and activating candidate",
                        artifact.target_slot)
            await send_and_wait(
                can_bus,
                encode_fw_update_activate(artifact.target_slot, 1),
                joint_cfg.joint_id,
                event_code=FW_UPDATE_EVT_ACTIVATE_OK,
                timeout_s=2.0,
            )
            if activate_only:
                final = await gather_update_info(can_bus, joint_cfg.joint_id)
                if final.info is None or not info_matches_pending_activation(final.info, artifact.target_slot):
                    raise RuntimeError(
                        "ACTIVATE_SLOT from verified state did not leave the controller in PENDING_TEST"
                    )
                logger.info(
                    "Candidate slot marked pending without reboot: active_slot=%d pending_slot=%d boot=%d fw=%s",
                    final.info.active_slot,
                    final.info.pending_slot,
                    final.info.boot_state,
                    final.info.fw_version,
                )
                return
            if validate_rollback:
                await validate_candidate_rollback(
                    can_bus,
                    joint_cfg,
                    target_slot=artifact.target_slot,
                    stable_slot=stable_slot,
                )
            else:
                await activate_verified_candidate(
                    can_bus,
                    joint_cfg,
                    target_slot=artifact.target_slot,
                )
            return

        if activate_only and info_matches_pending_activation(initial.info, artifact.target_slot):
            logger.info("Controller already has slot %d pending test; leaving reboot to the operator",
                        artifact.target_slot)
            return

        if validate_rollback and info_matches_pending_activation(initial.info, artifact.target_slot):
            logger.info("Controller already has slot %d pending test; resuming rollback validation",
                        artifact.target_slot)
            await validate_candidate_rollback(
                can_bus,
                joint_cfg,
                target_slot=artifact.target_slot,
                stable_slot=stable_slot,
            )
            return

        if activate and info_matches_pending_activation(initial.info, artifact.target_slot):
            logger.info("Controller already has slot %d pending test; resuming reboot/confirm flow",
                        artifact.target_slot)
            await activate_verified_candidate(
                can_bus,
                joint_cfg,
                target_slot=artifact.target_slot,
            )
            return

        start_page_index = 0
        if resuming_receiving:
            session_started = True
            if initial.progress is None:
                raise RuntimeError("Missing progress snapshot while resuming BOOT_RECEIVING session")
            start_page_index = initial.progress.next_page_index
            page_count = (artifact.image_size_bytes + FLASH_PAGE_SIZE - 1) // FLASH_PAGE_SIZE
            if start_page_index > page_count:
                raise RuntimeError(
                    f"Controller requested resume from page {start_page_index}, "
                    f"but artifact has only {page_count} pages"
                )
            logger.info(
                "Resuming live receiving session at page %d/%d",
                min(start_page_index + 1, page_count),
                page_count,
            )
        else:
            await send_and_wait(
                can_bus,
                encode_fw_update_enter_maintenance(require_idle=True, reject_if_startup_active=True),
                joint_cfg.joint_id,
                event_code=FW_UPDATE_EVT_MAINTENANCE_ENTERED,
            )

            fw_major, fw_minor, fw_patch = (
                initial.info.fw_major,
                initial.info.fw_minor,
                initial.info.fw_patch,
            )
            await send_burst_with_gap(
                can_bus,
                [
                    encode_fw_update_begin(
                        artifact.target_slot,
                        fw_major=fw_major,
                        fw_minor=fw_minor,
                        fw_patch=fw_patch,
                        allow_same_version=True,
                        candidate_boots_in_maintenance=True,
                    ),
                    encode_fw_update_meta_a(artifact.image_size_bytes, artifact.image_crc32),
                    encode_fw_update_meta_b(board_uid_crc32, 1, 0),
                ],
                delay_s=timing.ctrl_gap_s,
            )
            await wait_for_update_snapshot(
                can_bus,
                joint_cfg.joint_id,
                timeout_s=3.0,
                expect_event=FW_UPDATE_EVT_BEGIN_ACCEPTED,
            )
            session_started = True

        stream_started_at = asyncio.get_running_loop().time()
        stream_completed, streamed_bytes = await stream_image(
            can_bus,
            joint_cfg.joint_id,
            artifact,
            timing=timing,
            start_page_index=start_page_index,
            interrupt_after_pages=interrupt_after_pages,
        )
        stream_elapsed_s = max(asyncio.get_running_loop().time() - stream_started_at, 1e-6)

        if not stream_completed:
            logger.warning(
                "Leaving controller in BOOT_RECEIVING for resume validation "
                "(target_slot=%d)",
                artifact.target_slot,
            )
            logger.info(
                "Transferred %.1f KiB before intentional interruption in %.2f s (%.1f KiB/s)",
                streamed_bytes / 1024.0,
                stream_elapsed_s,
                (streamed_bytes / 1024.0) / stream_elapsed_s if stream_elapsed_s > 0.0 else 0.0,
            )
            return

        logger.info(
            "Streamed %d bytes in %.2f s (%.1f KiB/s)",
            streamed_bytes,
            stream_elapsed_s,
            (streamed_bytes / 1024.0) / stream_elapsed_s,
        )

        await can_bus.send(*encode_fw_update_end())

        if expect_verify_error_code is not None:
            verify_failure = await send_and_wait(
                can_bus,
                encode_fw_update_verify(artifact.target_slot),
                joint_cfg.joint_id,
                event_code=FW_UPDATE_EVT_ERROR,
                timeout_s=4.0,
                allow_error_codes={expect_verify_error_code},
            )
            if verify_failure.status is None or verify_failure.status.error_code != expect_verify_error_code:
                raise RuntimeError("Expected verify failure did not produce the requested error code")
            logger.info("Observed expected verify failure: %s", verify_failure.status.error_name)
            await cleanup_after_expected_verify_failure(
                can_bus,
                joint_cfg,
                stable_slot=stable_slot,
            )
            return

        await send_and_wait(
            can_bus,
            encode_fw_update_verify(artifact.target_slot),
            joint_cfg.joint_id,
            event_code=FW_UPDATE_EVT_VERIFY_OK,
            timeout_s=4.0,
        )

        logger.info(
            "CAN write + verify completed for joint=%s target_slot=%d image=%s",
            joint_cfg.name,
            artifact.target_slot,
            artifact.image_path,
        )

        if activate_only:
            await send_and_wait(
                can_bus,
                encode_fw_update_activate(artifact.target_slot, 1),
                joint_cfg.joint_id,
                event_code=FW_UPDATE_EVT_ACTIVATE_OK,
                timeout_s=2.0,
            )
            final = await gather_update_info(can_bus, joint_cfg.joint_id)
            if final.info is None or not info_matches_pending_activation(final.info, artifact.target_slot):
                raise RuntimeError("ACTIVATE_SLOT without reboot did not leave the controller in PENDING_TEST")
            logger.info(
                "Candidate slot marked pending without reboot: active_slot=%d pending_slot=%d boot=%d fw=%s",
                final.info.active_slot,
                final.info.pending_slot,
                final.info.boot_state,
                final.info.fw_version,
            )
            return

        if activate or validate_rollback:
            await send_and_wait(
                can_bus,
                encode_fw_update_activate(artifact.target_slot, 1),
                joint_cfg.joint_id,
                event_code=FW_UPDATE_EVT_ACTIVATE_OK,
                timeout_s=2.0,
            )
            logger.info("ACTIVATE_SLOT acknowledged")
            if validate_rollback:
                await validate_candidate_rollback(
                    can_bus,
                    joint_cfg,
                    target_slot=artifact.target_slot,
                    stable_slot=stable_slot,
                )
            else:
                await activate_verified_candidate(
                    can_bus,
                    joint_cfg,
                    target_slot=artifact.target_slot,
                )

    except Exception:
        if session_started:
            try:
                await can_bus.send(*encode_fw_update_abort())
            except Exception:
                logger.warning("Failed to send ABORT_UPDATE after error", exc_info=True)
        raise
    finally:
        await can_bus.disconnect()


def select_joint(config: ControllerConfig, joint_name: str) -> JointControlConfig:
    key = joint_name.strip().lower()
    for cfg in config.joints.values():
        if cfg.name.lower() == key:
            return cfg
    raise ValueError(f"Joint '{joint_name}' not found in config")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Write firmware to RP2350 joint controller via Host CAN")
    parser.add_argument("--config", default=None, help="Path to controller.yaml")
    parser.add_argument("--joint", required=True, help="Joint profile name from controller.yaml, e.g. hip_right")
    parser.add_argument("--manifest", default=None, help="Path to slot_a/slot_b firmware_manifest.json")
    activation_group = parser.add_mutually_exclusive_group()
    activation_group.add_argument("--activate", action="store_true",
                                  help="After verify, switch to the candidate slot, confirm it, and exit maintenance")
    activation_group.add_argument("--activate-only", action="store_true",
                                  help="After verify, mark the candidate pending test but leave reboot/confirmation to the operator")
    activation_group.add_argument("--validate-rollback", action="store_true",
                                  help="After verify, boot the candidate once without confirmation, then verify rollback to the previous stable slot")
    parser.add_argument("--info-only", action="store_true",
                        help="Query and print the controller firmware-update state without changing it")
    parser.add_argument("--cleanup-session", action="store_true",
                        help="Abort or roll back any partial update state, then exit maintenance when possible")
    parser.add_argument("--resume-receiving", action="store_true",
                        help="Resume an already-open BOOT_RECEIVING session using FW_UPDATE_PROGRESS.next_page_index")
    parser.add_argument("--interrupt-after-pages", type=int, default=None,
                        help="Intentionally stop after N committed pages and leave the controller in BOOT_RECEIVING")
    parser.add_argument("--corrupt-byte-offset", type=int, default=None,
                        help="Flip one transmitted image byte at the given offset without changing the manifest CRC")
    parser.add_argument("--expect-verify-error", choices=sorted(EXPECTED_VERIFY_ERROR_CODES.keys()),
                        default=None,
                        help="Treat the selected verify error as the expected outcome and clean up with ABORT_UPDATE")
    parser.add_argument("--ctrl-gap-ms", type=float, default=FW_UPDATE_CTRL_DELAY_S * 1000.0,
                        help="Delay between BEGIN/META control frames in milliseconds")
    parser.add_argument("--page-begin-gap-ms", type=float, default=FW_UPDATE_PAGE_BEGIN_DELAY_S * 1000.0,
                        help="Delay after each PAGE_BEGIN frame in milliseconds")
    parser.add_argument("--frag-gap-ms", type=float, default=FW_UPDATE_INTERFRAME_DELAY_S * 1000.0,
                        help="Delay between page fragment frames in milliseconds")
    parser.add_argument("--verbose", action="store_true", help="Enable debug logging")
    return parser


async def _async_main(args: argparse.Namespace) -> int:
    setup_logging(args.verbose)
    if args.info_only and args.cleanup_session:
        raise ValueError("--info-only and --cleanup-session are mutually exclusive")
    if (args.info_only or args.cleanup_session) and any([
        args.activate,
        args.activate_only,
        args.validate_rollback,
        args.resume_receiving,
        args.interrupt_after_pages is not None,
        args.corrupt_byte_offset is not None,
        args.expect_verify_error is not None,
    ]):
        raise ValueError("inspection/cleanup modes cannot be combined with update execution flags")
    if args.interrupt_after_pages is not None and args.interrupt_after_pages <= 0:
        raise ValueError("--interrupt-after-pages must be > 0")
    if args.ctrl_gap_ms < 0.0 or args.page_begin_gap_ms < 0.0 or args.frag_gap_ms < 0.0:
        raise ValueError("timing gaps must be >= 0 ms")
    if args.corrupt_byte_offset is not None and args.expect_verify_error is None:
        raise ValueError("--corrupt-byte-offset requires --expect-verify-error")
    config = load_config(args.config, selected_joints=[args.joint])
    joint_cfg = select_joint(config, args.joint)
    timing = UpdateTimingConfig(
        ctrl_gap_s=args.ctrl_gap_ms / 1000.0,
        page_begin_gap_s=args.page_begin_gap_ms / 1000.0,
        frag_gap_s=args.frag_gap_ms / 1000.0,
    )

    if args.info_only:
        await run_info_only(config, joint_cfg)
        return 0

    if args.cleanup_session:
        await run_cleanup_session(config, joint_cfg)
        return 0

    if args.expect_verify_error is not None and (args.activate or args.activate_only or args.validate_rollback):
        raise ValueError("--expect-verify-error cannot be combined with activation or rollback modes")
    if args.manifest is None:
        raise ValueError("--manifest is required unless --info-only or --cleanup-session is used")

    artifact = maybe_corrupt_artifact(
        load_update_artifact(args.manifest),
        args.corrupt_byte_offset,
    )
    await run_update(
        config,
        joint_cfg,
        artifact,
        timing=timing,
        activate=args.activate,
        activate_only=args.activate_only,
        validate_rollback=args.validate_rollback,
        resume_receiving=args.resume_receiving,
        interrupt_after_pages=args.interrupt_after_pages,
        expect_verify_error_code=(
            EXPECTED_VERIFY_ERROR_CODES[args.expect_verify_error]
            if args.expect_verify_error is not None
            else None
        ),
    )
    return 0


def main(argv: Optional[list[str]] = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    return asyncio.run(_async_main(args))


if __name__ == "__main__":
    raise SystemExit(main())
