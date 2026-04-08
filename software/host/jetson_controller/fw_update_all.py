"""
Sequential multi-board Host CAN firmware updater.

This is a thin orchestration layer over fw_update.py:
- preflight selected boards
- choose inactive-slot artifact per board
- run the existing single-board updater sequentially
- emit a JSON report
"""
from __future__ import annotations

import argparse
import asyncio
import json
import logging
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Optional

from .can_bus import CanBus
from .config import ControllerConfig, JointControlConfig, load_config
from .fw_update import (
    FW_BOOT_RECEIVING,
    FW_BOOT_VERIFIED,
    FW_IMAGE_SLOT_NONE,
    UpdateArtifact,
    UpdateSnapshot,
    UpdateTimingConfig,
    gather_update_info,
    load_update_artifact,
    run_cleanup_session,
    run_update,
    select_joint,
    setup_logging,
)

logger = logging.getLogger("jetson_controller.fw_update_all")

# Batch updates should use the fastest bench-validated profile by default,
# while still keeping a slower whole-board fallback for outliers.
BATCH_DEFAULT_CTRL_GAP_MS = 0.75
BATCH_DEFAULT_PAGE_BEGIN_GAP_MS = 0.75
BATCH_DEFAULT_FRAG_GAP_MS = 0.5
BATCH_DEFAULT_PAGE_RETRIES = 2
BATCH_DEFAULT_PAGE_RETRY_BACKOFF_MS = 150.0
BATCH_TURBO_CTRL_GAP_MS = 0.4
BATCH_TURBO_PAGE_BEGIN_GAP_MS = 0.4
BATCH_TURBO_FRAG_GAP_MS = 0.2
BATCH_SAFE_RETRY_CTRL_GAP_MS = 5.0
BATCH_SAFE_RETRY_PAGE_BEGIN_GAP_MS = 5.0
BATCH_SAFE_RETRY_FRAG_GAP_MS = 2.0


def _now_iso() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def _parse_joint_csv(raw: str) -> list[str]:
    joints: list[str] = []
    seen: set[str] = set()
    for token in raw.split(","):
        name = token.strip().lower()
        if not name or name in seen:
            continue
        joints.append(name)
        seen.add(name)
    if not joints:
        raise ValueError("Empty --joints selection")
    return joints


def _snapshot_to_dict(snapshot: UpdateSnapshot) -> dict[str, Any]:
    payload: dict[str, Any] = {}
    if snapshot.uid is not None:
        payload["uid"] = snapshot.uid.uid.hex().upper()
    if snapshot.info is not None:
        payload["info"] = {
            "active_slot": snapshot.info.active_slot,
            "pending_slot": snapshot.info.pending_slot,
            "boot_state": snapshot.info.boot_state,
            "maintenance_active": snapshot.info.maintenance_active,
            "update_in_progress": snapshot.info.update_in_progress,
            "candidate_awaiting_confirmation": snapshot.info.candidate_awaiting_confirmation,
            "attempts_remaining": snapshot.info.attempts_remaining,
            "fw_version": snapshot.info.fw_version,
        }
    if snapshot.progress is not None:
        payload["progress"] = {
            "next_page_index": snapshot.progress.next_page_index,
            "last_page_seq": snapshot.progress.last_page_seq,
            "last_frag_index": snapshot.progress.last_frag_index,
        }
    if snapshot.status is not None:
        payload["status"] = {
            "event_code": snapshot.status.event_code,
            "event_name": snapshot.status.event_name,
            "error_code": snapshot.status.error_code,
            "error_name": snapshot.status.error_name,
            "value": snapshot.status.value,
        }
    return payload


def _is_clean_stable(snapshot: UpdateSnapshot) -> bool:
    if snapshot.info is None:
        return False
    info = snapshot.info
    return (
        info.active_slot in (1, 2)
        and info.pending_slot == FW_IMAGE_SLOT_NONE
        and info.boot_state == 0
        and not info.maintenance_active
        and not info.update_in_progress
        and not info.candidate_awaiting_confirmation
    )


def _artifact_by_inactive_slot(active_slot: int,
                               slot_a_artifact: UpdateArtifact,
                               slot_b_artifact: UpdateArtifact) -> UpdateArtifact:
    if active_slot == 1:
        return slot_b_artifact
    if active_slot == 2:
        return slot_a_artifact
    raise ValueError(f"Unsupported active_slot={active_slot}; expected 1 or 2")


def _report_template(*,
                     requested_joints: list[str],
                     mode: str,
                     continue_on_error: bool,
                     timing: UpdateTimingConfig) -> dict[str, Any]:
    return {
        "started_at": _now_iso(),
        "finished_at": None,
        "mode": mode,
        "continue_on_error": continue_on_error,
        "timing_ms": {
            "ctrl_gap_ms": timing.ctrl_gap_s * 1000.0,
            "page_begin_gap_ms": timing.page_begin_gap_s * 1000.0,
            "frag_gap_ms": timing.frag_gap_s * 1000.0,
            "burst_percent": timing.burst_percent,
            "burst_pause_ms": timing.burst_pause_s * 1000.0,
            "page_retries": timing.page_retry_limit,
            "page_retry_backoff_ms": timing.page_retry_backoff_s * 1000.0,
        },
        "requested_joints": requested_joints,
        "results": [],
    }


def _write_report(path: Optional[Path], report: dict[str, Any]) -> None:
    if path is None:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_suffix(path.suffix + ".tmp")
    tmp_path.write_text(json.dumps(report, indent=2, sort_keys=False) + "\n", encoding="utf-8")
    tmp_path.replace(path)


async def _fetch_snapshot(config: ControllerConfig,
                          joint_cfg: JointControlConfig) -> UpdateSnapshot:
    can_bus = CanBus()
    try:
        await can_bus.connect(config.can_interface, config.can_channel, config.can_bitrate)
        snapshot = await gather_update_info(can_bus, joint_cfg.joint_id)
        if snapshot.uid is None or snapshot.info is None:
            raise RuntimeError("Failed to retrieve controller UID/info snapshot")
        return snapshot
    finally:
        await can_bus.disconnect()


async def _ensure_preflight_ready(config: ControllerConfig,
                                  joint_cfg: JointControlConfig,
                                  *,
                                  cleanup_before: bool) -> tuple[UpdateSnapshot, Optional[UpdateSnapshot]]:
    initial = await _fetch_snapshot(config, joint_cfg)
    if _is_clean_stable(initial):
        return initial, None

    if not cleanup_before:
        raise RuntimeError(
            "Controller is not in a clean stable preflight state "
            f"(boot={initial.info.boot_state if initial.info else 'n/a'})"
        )

    logger.info("Preflight cleanup for joint=%s", joint_cfg.name)
    await run_cleanup_session(config, joint_cfg)
    cleaned = await _fetch_snapshot(config, joint_cfg)
    if not _is_clean_stable(cleaned):
        raise RuntimeError(
            "Controller did not return to a clean stable state after preflight cleanup"
        )
    return cleaned, initial


def _summarize_results(results: list[dict[str, Any]]) -> dict[str, int]:
    summary = {"success": 0, "failed": 0, "dry_run": 0}
    for item in results:
        result = str(item.get("result", "")).lower()
        if result in summary:
            summary[result] += 1
    return summary


def _timing_to_dict(timing: UpdateTimingConfig) -> dict[str, float]:
    return {
        "ctrl_gap_ms": timing.ctrl_gap_s * 1000.0,
        "page_begin_gap_ms": timing.page_begin_gap_s * 1000.0,
        "frag_gap_ms": timing.frag_gap_s * 1000.0,
        "burst_percent": timing.burst_percent,
        "burst_pause_ms": timing.burst_pause_s * 1000.0,
        "page_retries": timing.page_retry_limit,
        "page_retry_backoff_ms": timing.page_retry_backoff_s * 1000.0,
    }


def _same_timing(a: UpdateTimingConfig, b: UpdateTimingConfig) -> bool:
    return (
        a.ctrl_gap_s == b.ctrl_gap_s
        and a.page_begin_gap_s == b.page_begin_gap_s
        and a.frag_gap_s == b.frag_gap_s
        and a.burst_percent == b.burst_percent
        and a.burst_pause_s == b.burst_pause_s
        and a.page_retry_limit == b.page_retry_limit
        and a.page_retry_backoff_s == b.page_retry_backoff_s
    )


def _should_retry_with_safe_timing(exc: Exception) -> bool:
    text = str(exc)
    return "FRAG_INDEX_MISMATCH" in text or "INVALID_STATE" in text


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Sequentially update multiple RP2350 joint controllers over one Host CAN channel"
    )
    parser.add_argument("--config", default=None, help="Path to controller.yaml")
    parser.add_argument("--joints", required=True, help="Comma-separated joint names from controller.yaml")
    parser.add_argument("--slot-a-manifest", required=True, help="Path to slot A firmware_manifest.json")
    parser.add_argument("--slot-b-manifest", required=True, help="Path to slot B firmware_manifest.json")
    parser.add_argument("--activate", action="store_true",
                        help="For each board, activate, reboot, confirm, and exit maintenance after verify")
    parser.add_argument("--cleanup-before", action="store_true",
                        help="Attempt cleanup-session during preflight if a board is not already clean")
    parser.add_argument("--continue-on-error", action="store_true",
                        help="Continue to the next board after a failure")
    parser.add_argument("--no-cleanup-after-failure", action="store_true",
                        help="Do not attempt cleanup-session on a failed board")
    parser.add_argument("--no-safe-retry", action="store_true",
                        help="Disable the automatic one-time retry with conservative timing after transport pacing errors")
    parser.add_argument("--report", default=None, help="Write JSON batch report to this path")
    parser.add_argument("--dry-run", action="store_true",
                        help="Run preflight and slot selection only; do not write firmware")
    parser.add_argument("--ctrl-gap-ms", type=float, default=BATCH_DEFAULT_CTRL_GAP_MS,
                        help="Delay between BEGIN/META control frames in milliseconds")
    parser.add_argument("--page-begin-gap-ms", type=float, default=BATCH_DEFAULT_PAGE_BEGIN_GAP_MS,
                        help="Delay after each PAGE_BEGIN frame in milliseconds")
    parser.add_argument("--frag-gap-ms", type=float, default=BATCH_DEFAULT_FRAG_GAP_MS,
                        help="Delay between page fragment frames in milliseconds")
    parser.add_argument("--burst-percent", type=float, default=0.0,
                        help="After this percentage of image pages, pause for --burst-pause-ms and repeat")
    parser.add_argument("--burst-pause-ms", type=float, default=0.0,
                        help="Pause duration between burst windows in milliseconds")
    parser.add_argument("--page-retries", type=int, default=BATCH_DEFAULT_PAGE_RETRIES,
                        help="Retry the current page this many times after recoverable page errors")
    parser.add_argument("--page-retry-backoff-ms", type=float, default=BATCH_DEFAULT_PAGE_RETRY_BACKOFF_MS,
                        help="Backoff before retrying a failed page in milliseconds")
    parser.add_argument("--turbo", action="store_true",
                        help="Use the aggressive 0.4/0.4/0.2 ms pacing preset with page retry")
    parser.add_argument("--verbose", action="store_true", help="Enable debug logging")
    return parser


async def _async_main(args: argparse.Namespace) -> int:
    setup_logging(args.verbose)

    requested_joints = _parse_joint_csv(args.joints)
    if args.ctrl_gap_ms < 0.0 or args.page_begin_gap_ms < 0.0 or args.frag_gap_ms < 0.0:
        raise ValueError("timing gaps must be >= 0 ms")
    if args.burst_percent < 0.0 or args.burst_pause_ms < 0.0:
        raise ValueError("burst settings must be >= 0")
    if (args.burst_percent == 0.0) != (args.burst_pause_ms == 0.0):
        raise ValueError("--burst-percent and --burst-pause-ms must be set together")
    if args.burst_percent >= 100.0:
        raise ValueError("--burst-percent must be < 100")
    if args.page_retries < 0 or args.page_retry_backoff_ms < 0.0:
        raise ValueError("page retry settings must be >= 0")
    if args.turbo:
        args.ctrl_gap_ms = BATCH_TURBO_CTRL_GAP_MS
        args.page_begin_gap_ms = BATCH_TURBO_PAGE_BEGIN_GAP_MS
        args.frag_gap_ms = BATCH_TURBO_FRAG_GAP_MS

    timing = UpdateTimingConfig(
        ctrl_gap_s=args.ctrl_gap_ms / 1000.0,
        page_begin_gap_s=args.page_begin_gap_ms / 1000.0,
        frag_gap_s=args.frag_gap_ms / 1000.0,
        burst_percent=args.burst_percent,
        burst_pause_s=args.burst_pause_ms / 1000.0,
        page_retry_limit=args.page_retries,
        page_retry_backoff_s=args.page_retry_backoff_ms / 1000.0,
    )
    cleanup_after_failure = not args.no_cleanup_after_failure
    safe_retry_timing = UpdateTimingConfig(
        ctrl_gap_s=BATCH_SAFE_RETRY_CTRL_GAP_MS / 1000.0,
        page_begin_gap_s=BATCH_SAFE_RETRY_PAGE_BEGIN_GAP_MS / 1000.0,
        frag_gap_s=BATCH_SAFE_RETRY_FRAG_GAP_MS / 1000.0,
        burst_percent=0.0,
        burst_pause_s=0.0,
        page_retry_limit=args.page_retries,
        page_retry_backoff_s=args.page_retry_backoff_ms / 1000.0,
    )

    config = load_config(args.config, selected_joints=requested_joints)
    slot_a_artifact = load_update_artifact(args.slot_a_manifest)
    slot_b_artifact = load_update_artifact(args.slot_b_manifest)

    if slot_a_artifact.target_slot != 1:
        raise ValueError(f"--slot-a-manifest must target slot A, got slot {slot_a_artifact.target_slot}")
    if slot_b_artifact.target_slot != 2:
        raise ValueError(f"--slot-b-manifest must target slot B, got slot {slot_b_artifact.target_slot}")

    report_path = Path(args.report).expanduser().resolve() if args.report else None
    report = _report_template(
        requested_joints=requested_joints,
        mode="activate" if args.activate else "verify_only",
        continue_on_error=args.continue_on_error,
        timing=timing,
    )
    _write_report(report_path, report)

    for joint_name in requested_joints:
        joint_cfg = select_joint(config, joint_name)
        entry: dict[str, Any] = {
            "joint": joint_cfg.name,
            "joint_id": joint_cfg.joint_id,
            "started_at": _now_iso(),
            "result": "failed",
        }
        started_at = time.perf_counter()

        try:
            preflight_snapshot, dirty_snapshot = await _ensure_preflight_ready(
                config,
                joint_cfg,
                cleanup_before=args.cleanup_before,
            )
            entry["initial"] = _snapshot_to_dict(dirty_snapshot or preflight_snapshot)
            if dirty_snapshot is not None:
                entry["preflight_cleanup"] = {
                    "applied": True,
                    "post_cleanup": _snapshot_to_dict(preflight_snapshot),
                }
            else:
                entry["preflight_cleanup"] = {"applied": False}

            artifact = _artifact_by_inactive_slot(
                preflight_snapshot.info.active_slot,
                slot_a_artifact,
                slot_b_artifact,
            )
            entry["target_slot"] = artifact.target_slot
            entry["manifest"] = str(artifact.manifest_path)
            entry["image_size_bytes"] = artifact.image_size_bytes
            entry["attempts"] = []

            if args.dry_run:
                entry["result"] = "dry_run"
                entry["final"] = _snapshot_to_dict(preflight_snapshot)
            else:
                attempt_timings = [timing]
                if not args.no_safe_retry and not _same_timing(timing, safe_retry_timing):
                    attempt_timings.append(safe_retry_timing)

                last_exc: Optional[Exception] = None
                for attempt_index, attempt_timing in enumerate(attempt_timings, start=1):
                    attempt_started_at = time.perf_counter()
                    attempt_entry = {
                        "attempt": attempt_index,
                        "timing_ms": _timing_to_dict(attempt_timing),
                    }
                    try:
                        await run_update(
                            config,
                            joint_cfg,
                            artifact,
                            timing=attempt_timing,
                            activate=args.activate,
                        )
                        attempt_entry["result"] = "success"
                        attempt_entry["elapsed_s"] = round(time.perf_counter() - attempt_started_at, 3)
                        entry["attempts"].append(attempt_entry)
                        last_exc = None
                        break
                    except Exception as exc:
                        last_exc = exc
                        attempt_entry["result"] = "failed"
                        attempt_entry["elapsed_s"] = round(time.perf_counter() - attempt_started_at, 3)
                        attempt_entry["error"] = str(exc)
                        entry["attempts"].append(attempt_entry)

                        can_retry = (
                            attempt_index < len(attempt_timings)
                            and _should_retry_with_safe_timing(exc)
                        )
                        if not can_retry:
                            raise

                        logger.warning(
                            "Retrying joint=%s with safe timing after transport error: %s",
                            joint_cfg.name,
                            exc,
                        )
                        await run_cleanup_session(config, joint_cfg)

                if last_exc is not None:
                    raise last_exc
                final_snapshot = await _fetch_snapshot(config, joint_cfg)
                entry["result"] = "success"
                entry["final"] = _snapshot_to_dict(final_snapshot)
        except Exception as exc:
            entry["error"] = str(exc)
            if cleanup_after_failure:
                try:
                    await run_cleanup_session(config, joint_cfg)
                    cleanup_snapshot = await _fetch_snapshot(config, joint_cfg)
                    entry["cleanup_after_failure"] = {
                        "applied": True,
                        "final": _snapshot_to_dict(cleanup_snapshot),
                    }
                except Exception as cleanup_exc:
                    entry["cleanup_after_failure"] = {
                        "applied": True,
                        "error": str(cleanup_exc),
                    }
            else:
                entry["cleanup_after_failure"] = {"applied": False}
        finally:
            entry["elapsed_s"] = round(time.perf_counter() - started_at, 3)
            entry["finished_at"] = _now_iso()
            report["results"].append(entry)
            _write_report(report_path, report)

        if entry["result"] == "success":
            logger.info(
                "Batch update success: joint=%s target_slot=%s elapsed=%.2f s",
                entry["joint"],
                entry.get("target_slot"),
                entry["elapsed_s"],
            )
            continue

        if entry["result"] == "dry_run":
            logger.info(
                "Batch dry-run: joint=%s would_target_slot=%s",
                entry["joint"],
                entry.get("target_slot"),
            )
            continue

        logger.error("Batch update failed for joint=%s: %s", entry["joint"], entry.get("error", "unknown error"))
        if not args.continue_on_error:
            break

    report["finished_at"] = _now_iso()
    report["summary"] = _summarize_results(report["results"])
    _write_report(report_path, report)

    logger.info(
        "Batch summary: success=%d failed=%d dry_run=%d",
        report["summary"]["success"],
        report["summary"]["failed"],
        report["summary"]["dry_run"],
    )
    return 0 if report["summary"]["failed"] == 0 else 1


def main(argv: Optional[list[str]] = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    return asyncio.run(_async_main(args))


if __name__ == "__main__":
    raise SystemExit(main())
