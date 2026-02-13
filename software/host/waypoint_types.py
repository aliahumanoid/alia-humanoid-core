"""
Waypoint contract and validation for the host waypoint pipeline.

This module is the single authority for waypoint semantics — angle limits,
velocity caps, monotonicity, batch size.  It does NOT depend on Flask,
CanManager, or any I/O layer, so it can be tested and used from Jetson
without the web UI.
"""
import logging
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
from uuid import uuid4

from config import JOINTS, MAX_ANGLES, MIN_ANGLES

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Safety constants (must match firmware)
# ---------------------------------------------------------------------------
MAX_VELOCITY_DEG_S: float = 150.0   # ABSOLUTE_MAX_VELOCITY_DEG_S in firmware
MAX_BATCH_SIZE: int = 1800           # Margin vs 2000 firmware buffer depth
ANGLE_RESOLUTION: int = 100          # int16 × 100 → 0.01° per count
T_OFFSET_MAX: int = 65535            # uint16 range
MIN_LEAD_MS: float = 15.0            # Waypoint with adjusted t_offset < this is "late"
MAX_CONSECUTIVE_LATE: int = 10       # Abort batch after this many consecutive late skips


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------
@dataclass
class WaypointEntry:
    """Single waypoint in a batch."""
    angles_deg: List[Optional[float]]   # max 3, None = unused DOF
    t_offset_ms: int                     # 0 .. 65535


@dataclass
class WaypointBatch:
    """A batch of waypoints destined for one joint."""
    joint: str
    entries: List[WaypointEntry]
    batch_id: str = field(default_factory=lambda: uuid4().hex[:8])


@dataclass
class ValidationError:
    """One validation failure in a batch."""
    index: Optional[int]    # waypoint index, None = batch-level error
    field: str              # e.g. "t_offset_ms", "angles_deg[0]", "batch"
    message: str


@dataclass
class ValidationResult:
    """Aggregated result of validate_batch()."""
    errors: List[ValidationError]
    warnings: List[str]

    @property
    def ok(self) -> bool:
        return len(self.errors) == 0


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def get_dof_limits(joint_name: str) -> Dict[int, Tuple[float, float]]:
    """Return {dof_index: (min_angle, max_angle)} for a joint.

    Handles both scalar (single-DOF) and dict (multi-DOF) formats
    from config.MIN_ANGLES / config.MAX_ANGLES.
    """
    joint_key = joint_name.upper()
    if joint_key not in JOINTS:
        return {}

    min_raw = MIN_ANGLES.get(joint_key)
    max_raw = MAX_ANGLES.get(joint_key)
    if min_raw is None or max_raw is None:
        return {}

    limits: Dict[int, Tuple[float, float]] = {}
    if isinstance(min_raw, dict):
        # Multi-DOF: {0: -50.0, 1: -25.0, ...}
        for dof_idx in min_raw:
            limits[dof_idx] = (min_raw[dof_idx], max_raw[dof_idx])
    else:
        # Single-DOF: scalar
        limits[0] = (float(min_raw), float(max_raw))

    return limits


def _quantize(angle: float) -> int:
    """Convert angle to CAN int16 representation (0.01° resolution)."""
    return round(angle * ANGLE_RESOLUTION)


# ---------------------------------------------------------------------------
# Batch builder  (dict list → WaypointBatch)
# ---------------------------------------------------------------------------
def build_batch(joint: str, waypoints_raw: list) -> WaypointBatch:
    """Convert a list of dicts (from JSON) into a WaypointBatch.

    Each dict is expected to have:
        - angles_deg: list of up to 3 angles (float or null)
        - t_offset_ms: int
    """
    entries: List[WaypointEntry] = []
    for wp in waypoints_raw:
        angles = wp.get("angles_deg", [None, None, None])
        # Normalise to exactly 3 slots
        while len(angles) < 3:
            angles.append(None)
        angles = angles[:3]

        t_offset = int(wp.get("t_offset_ms", 0))
        entries.append(WaypointEntry(angles_deg=angles, t_offset_ms=t_offset))

    return WaypointBatch(joint=joint.upper(), entries=entries)


def deduplicate_batch(batch: WaypointBatch) -> WaypointBatch:
    """Remove consecutive waypoints that quantise to the same int16 angles.

    CAN waypoints encode angles as int16 × 100 (0.01° resolution).  At low
    velocities, cosine S-curves produce adjacent angles that round to the
    same int16 value, creating "zero-steps" where the firmware sees no
    movement and the PID stalls.  Removing duplicates lets the firmware
    interpolate over a longer interval instead.

    Returns a new WaypointBatch with duplicates removed (preserves batch_id).
    """
    if len(batch.entries) <= 1:
        return batch

    kept: List[WaypointEntry] = [batch.entries[0]]

    for entry in batch.entries[1:]:
        prev = kept[-1]
        all_same = True
        has_any_active = False

        for dof_idx in range(3):
            curr_a = entry.angles_deg[dof_idx] if dof_idx < len(entry.angles_deg) else None
            prev_a = prev.angles_deg[dof_idx] if dof_idx < len(prev.angles_deg) else None

            if curr_a is None and prev_a is None:
                continue
            has_any_active = True
            if curr_a is None or prev_a is None:
                all_same = False
                break
            if _quantize(curr_a) != _quantize(prev_a):
                all_same = False
                break

        if not (has_any_active and all_same):
            kept.append(entry)

    return WaypointBatch(joint=batch.joint, entries=kept, batch_id=batch.batch_id)


def batch_to_dicts(batch: WaypointBatch) -> list:
    """Convert a WaypointBatch back to a list of dicts for send_waypoint_batch().

    This bridges the typed dataclass world (validation) with the dict-based
    CanManager API (backward-compatible for direct HTTP callers).
    """
    return [
        {"angles_deg": e.angles_deg, "t_offset_ms": e.t_offset_ms}
        for e in batch.entries
    ]


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------
def validate_batch(
    batch: WaypointBatch,
    dof_limits: Optional[Dict[int, Tuple[float, float]]] = None,
) -> ValidationResult:
    """Validate a WaypointBatch.

    Args:
        batch: The batch to validate.
        dof_limits: Per-DOF angle limits {dof_index: (min, max)}.
                    If None, looked up via get_dof_limits(batch.joint).

    Returns:
        ValidationResult with errors (hard failures) and warnings.
    """
    errors: List[ValidationError] = []
    warnings: List[str] = []

    # --- Batch-level checks ---
    joint_key = batch.joint.upper()
    if joint_key not in JOINTS:
        errors.append(ValidationError(
            index=None, field="joint",
            message=f"Unknown joint '{batch.joint}'",
        ))
        # Can't continue without joint config
        return ValidationResult(errors=errors, warnings=warnings)

    if not batch.entries:
        errors.append(ValidationError(
            index=None, field="batch",
            message="Batch is empty",
        ))
        return ValidationResult(errors=errors, warnings=warnings)

    if len(batch.entries) > MAX_BATCH_SIZE:
        errors.append(ValidationError(
            index=None, field="batch",
            message=f"Batch size {len(batch.entries)} exceeds limit {MAX_BATCH_SIZE}",
        ))
        # Still validate individual entries for early feedback

    # Resolve DOF limits
    if dof_limits is None:
        dof_limits = get_dof_limits(joint_key)

    dof_count = len(JOINTS[joint_key]["dofs"])

    # --- Per-entry checks ---
    prev_t: Optional[int] = None
    prev_angles: List[Optional[float]] = [None, None, None]
    prev_quantized: List[Optional[int]] = [None, None, None]

    for i, entry in enumerate(batch.entries):
        # t_offset_ms range
        if entry.t_offset_ms < 0 or entry.t_offset_ms > T_OFFSET_MAX:
            errors.append(ValidationError(
                index=i, field="t_offset_ms",
                message=f"t_offset_ms={entry.t_offset_ms} out of range [0, {T_OFFSET_MAX}]",
            ))

        # angles_deg length
        if len(entry.angles_deg) > 3:
            errors.append(ValidationError(
                index=i, field="angles_deg",
                message=f"angles_deg has {len(entry.angles_deg)} elements, max 3",
            ))

        # Angle limits per DOF
        for dof_idx, angle in enumerate(entry.angles_deg[:3]):
            if angle is None:
                continue
            if dof_idx >= dof_count:
                errors.append(ValidationError(
                    index=i, field=f"angles_deg[{dof_idx}]",
                    message=f"DOF{dof_idx} does not exist on {joint_key} (has {dof_count} DOFs)",
                ))
                continue
            if dof_idx in dof_limits:
                lo, hi = dof_limits[dof_idx]
                if angle < lo or angle > hi:
                    errors.append(ValidationError(
                        index=i, field=f"angles_deg[{dof_idx}]",
                        message=f"DOF{dof_idx} angle {angle:.2f}° out of range [{lo:.1f}, {hi:.1f}]",
                    ))

        # Monotonicity
        if prev_t is not None and entry.t_offset_ms <= prev_t:
            errors.append(ValidationError(
                index=i, field="t_offset_ms",
                message=(
                    f"t_offset_ms={entry.t_offset_ms} is not strictly increasing "
                    f"(previous={prev_t})"
                ),
            ))

        # Velocity between consecutive entries
        if prev_t is not None and entry.t_offset_ms > prev_t:
            dt_s = (entry.t_offset_ms - prev_t) / 1000.0
            if dt_s > 0.0:
                for dof_idx in range(min(len(entry.angles_deg), 3)):
                    curr_angle = entry.angles_deg[dof_idx] if dof_idx < len(entry.angles_deg) else None
                    prev_angle = prev_angles[dof_idx]
                    if curr_angle is not None and prev_angle is not None:
                        vel = abs(curr_angle - prev_angle) / dt_s
                        if vel > MAX_VELOCITY_DEG_S:
                            errors.append(ValidationError(
                                index=i, field=f"velocity_dof{dof_idx}",
                                message=(
                                    f"DOF{dof_idx} velocity {vel:.1f}°/s exceeds "
                                    f"limit {MAX_VELOCITY_DEG_S:.0f}°/s "
                                    f"(Δ={abs(curr_angle - prev_angle):.2f}° in {dt_s*1000:.0f}ms)"
                                ),
                            ))

        # Deduplication warning (quantised to same int16 on all active DOFs)
        curr_quantized: List[Optional[int]] = [None, None, None]
        for dof_idx in range(min(len(entry.angles_deg), 3)):
            angle = entry.angles_deg[dof_idx]
            curr_quantized[dof_idx] = _quantize(angle) if angle is not None else None

        if i > 0:
            all_same = True
            has_any_active = False
            for dof_idx in range(3):
                cq = curr_quantized[dof_idx]
                pq = prev_quantized[dof_idx]
                if cq is None and pq is None:
                    continue
                has_any_active = True
                if cq != pq:
                    all_same = False
                    break
            if has_any_active and all_same:
                warnings.append(
                    f"Waypoint {i}: quantises to same angle as waypoint {i-1} "
                    f"(zero-step, firmware will see no movement)"
                )

        # Update previous state
        prev_t = entry.t_offset_ms
        for dof_idx in range(3):
            if dof_idx < len(entry.angles_deg):
                prev_angles[dof_idx] = entry.angles_deg[dof_idx]
                prev_quantized[dof_idx] = curr_quantized[dof_idx]

    return ValidationResult(errors=errors, warnings=warnings)
