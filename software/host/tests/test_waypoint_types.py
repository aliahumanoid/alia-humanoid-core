"""
Unit tests for waypoint_types.py — pure validation, no Flask or CAN.

Tests cover:
- validate_batch: angle limits, monotonicity, velocity, batch size, edge cases
- build_batch: normalisation, batch_id generation
- get_dof_limits: single-DOF, multi-DOF, unknown joint
- deduplicate_batch: zero-step removal, identity preservation
"""
import sys
from pathlib import Path

# Ensure host/ is on sys.path so `from config import ...` works
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from waypoint_types import (
    WaypointBatch,
    WaypointEntry,
    build_batch,
    deduplicate_batch,
    batch_to_dicts,
    get_dof_limits,
    validate_batch,
    MAX_BATCH_SIZE,
    MAX_VELOCITY_DEG_S,
    T_OFFSET_MAX,
)


# -----------------------------------------------------------------------
# validate_batch
# -----------------------------------------------------------------------
class TestValidateBatch:
    """Tests for the main batch validation function."""

    def test_valid_single_dof_batch(self):
        batch = build_batch("KNEE_LEFT", [
            {"angles_deg": [10.0], "t_offset_ms": 100},
            {"angles_deg": [20.0], "t_offset_ms": 200},
            {"angles_deg": [30.0], "t_offset_ms": 300},
        ])
        result = validate_batch(batch)
        assert result.ok, f"Expected valid, got errors: {[e.message for e in result.errors]}"

    def test_valid_multi_dof_batch(self):
        batch = build_batch("ANKLE_RIGHT", [
            {"angles_deg": [0.0, 0.0], "t_offset_ms": 100},
            {"angles_deg": [5.0, 3.0], "t_offset_ms": 200},
        ])
        result = validate_batch(batch)
        assert result.ok, f"Errors: {[e.message for e in result.errors]}"

    def test_unknown_joint(self):
        batch = build_batch("NONEXISTENT", [
            {"angles_deg": [0.0], "t_offset_ms": 100},
        ])
        result = validate_batch(batch)
        assert not result.ok
        assert any("Unknown joint" in e.message for e in result.errors)

    def test_empty_batch(self):
        batch = WaypointBatch(joint="KNEE_LEFT", entries=[])
        result = validate_batch(batch)
        assert not result.ok
        assert any("empty" in e.message.lower() for e in result.errors)

    def test_batch_size_exceeds_limit(self):
        # Create batch with MAX_BATCH_SIZE + 1 entries
        entries = [
            {"angles_deg": [float(i % 100)], "t_offset_ms": i}
            for i in range(1, MAX_BATCH_SIZE + 2)
        ]
        batch = build_batch("KNEE_LEFT", entries)
        result = validate_batch(batch)
        assert not result.ok
        assert any("exceeds limit" in e.message for e in result.errors)

    def test_angle_out_of_range_high(self):
        # KNEE_LEFT max is 112.0°
        batch = build_batch("KNEE_LEFT", [
            {"angles_deg": [200.0], "t_offset_ms": 100},
        ])
        result = validate_batch(batch)
        assert not result.ok
        assert any("out of range" in e.message for e in result.errors)

    def test_angle_out_of_range_low(self):
        # KNEE_LEFT min is -2.0°
        batch = build_batch("KNEE_LEFT", [
            {"angles_deg": [-10.0], "t_offset_ms": 100},
        ])
        result = validate_batch(batch)
        assert not result.ok
        assert any("out of range" in e.message for e in result.errors)

    def test_angle_at_exact_limit_passes(self):
        # KNEE_LEFT range: [-2.0, 112.0]
        batch = build_batch("KNEE_LEFT", [
            {"angles_deg": [-2.0], "t_offset_ms": 100},
            {"angles_deg": [112.0], "t_offset_ms": 5000},
        ])
        result = validate_batch(batch)
        assert result.ok, f"Errors: {[e.message for e in result.errors]}"

    def test_monotonicity_violation(self):
        batch = build_batch("KNEE_LEFT", [
            {"angles_deg": [10.0], "t_offset_ms": 500},
            {"angles_deg": [20.0], "t_offset_ms": 400},  # Earlier than previous
        ])
        result = validate_batch(batch)
        assert not result.ok
        assert any("not strictly increasing" in e.message for e in result.errors)

    def test_monotonicity_equal_times_rejected(self):
        batch = build_batch("KNEE_LEFT", [
            {"angles_deg": [10.0], "t_offset_ms": 500},
            {"angles_deg": [20.0], "t_offset_ms": 500},  # Same time
        ])
        result = validate_batch(batch)
        assert not result.ok
        assert any("not strictly increasing" in e.message for e in result.errors)

    def test_velocity_exceeds_limit(self):
        # 100° in 10ms = 10000°/s >> 150°/s
        batch = build_batch("KNEE_LEFT", [
            {"angles_deg": [0.0], "t_offset_ms": 100},
            {"angles_deg": [100.0], "t_offset_ms": 110},
        ])
        result = validate_batch(batch)
        assert not result.ok
        assert any("velocity" in e.message.lower() for e in result.errors)

    def test_velocity_at_limit_passes(self):
        # 15° in 100ms = 150°/s exactly at limit
        batch = build_batch("KNEE_LEFT", [
            {"angles_deg": [0.0], "t_offset_ms": 100},
            {"angles_deg": [15.0], "t_offset_ms": 200},
        ])
        result = validate_batch(batch)
        assert result.ok, f"Errors: {[e.message for e in result.errors]}"

    def test_velocity_just_over_limit(self):
        # 15.1° in 100ms = 151°/s, just over
        batch = build_batch("KNEE_LEFT", [
            {"angles_deg": [0.0], "t_offset_ms": 100},
            {"angles_deg": [15.1], "t_offset_ms": 200},
        ])
        result = validate_batch(batch)
        assert not result.ok

    def test_t_offset_out_of_range_negative(self):
        batch = build_batch("KNEE_LEFT", [
            {"angles_deg": [10.0], "t_offset_ms": -1},
        ])
        result = validate_batch(batch)
        assert not result.ok
        assert any("out of range" in e.message for e in result.errors)

    def test_t_offset_out_of_range_too_high(self):
        batch = build_batch("KNEE_LEFT", [
            {"angles_deg": [10.0], "t_offset_ms": 70000},
        ])
        result = validate_batch(batch)
        assert not result.ok
        assert any("out of range" in e.message for e in result.errors)

    def test_dof_index_exceeds_joint_count(self):
        # KNEE_LEFT has 1 DOF (index 0). Sending DOF1 and DOF2 values
        batch = build_batch("KNEE_LEFT", [
            {"angles_deg": [10.0, 5.0, 3.0], "t_offset_ms": 100},
        ])
        result = validate_batch(batch)
        assert not result.ok
        assert any("does not exist" in e.message for e in result.errors)

    def test_none_angles_ignored(self):
        """None angles should not be validated."""
        batch = build_batch("ANKLE_RIGHT", [
            {"angles_deg": [0.0, None], "t_offset_ms": 100},
            {"angles_deg": [5.0, None], "t_offset_ms": 200},
        ])
        result = validate_batch(batch)
        assert result.ok

    def test_dedup_warning_generated(self):
        """Zero-step duplicates should generate a warning (not error)."""
        batch = build_batch("KNEE_LEFT", [
            {"angles_deg": [45.001], "t_offset_ms": 100},
            {"angles_deg": [45.002], "t_offset_ms": 200},
        ])
        result = validate_batch(batch)
        assert result.ok  # Not an error
        assert len(result.warnings) > 0
        assert "zero-step" in result.warnings[0].lower()

    def test_single_entry_batch_valid(self):
        batch = build_batch("KNEE_LEFT", [
            {"angles_deg": [50.0], "t_offset_ms": 500},
        ])
        result = validate_batch(batch)
        assert result.ok

    def test_multi_dof_angle_limits(self):
        # ANKLE_RIGHT DOF0: [-50, 25], DOF1: [-25, 25]
        batch = build_batch("ANKLE_RIGHT", [
            {"angles_deg": [-60.0, 0.0], "t_offset_ms": 100},  # DOF0 below min
        ])
        result = validate_batch(batch)
        assert not result.ok
        assert any("DOF0" in e.message and "out of range" in e.message for e in result.errors)


# -----------------------------------------------------------------------
# build_batch
# -----------------------------------------------------------------------
class TestBuildBatch:
    """Tests for JSON → WaypointBatch conversion."""

    def test_normalises_angles_to_3(self):
        batch = build_batch("KNEE_LEFT", [
            {"angles_deg": [10.0], "t_offset_ms": 100},
        ])
        assert len(batch.entries[0].angles_deg) == 3
        assert batch.entries[0].angles_deg[0] == 10.0
        assert batch.entries[0].angles_deg[1] is None
        assert batch.entries[0].angles_deg[2] is None

    def test_generates_batch_id(self):
        batch = build_batch("KNEE_LEFT", [
            {"angles_deg": [10.0], "t_offset_ms": 100},
        ])
        assert batch.batch_id
        assert len(batch.batch_id) == 8

    def test_joint_uppercased(self):
        batch = build_batch("knee_left", [
            {"angles_deg": [10.0], "t_offset_ms": 100},
        ])
        assert batch.joint == "KNEE_LEFT"

    def test_truncates_extra_angles(self):
        batch = build_batch("KNEE_LEFT", [
            {"angles_deg": [1.0, 2.0, 3.0, 4.0, 5.0], "t_offset_ms": 100},
        ])
        assert len(batch.entries[0].angles_deg) == 3

    def test_string_angle_raises_valueerror(self):
        """Non-numeric angle must raise ValueError, not pass through."""
        import pytest
        with pytest.raises(ValueError, match="not numeric"):
            build_batch("KNEE_LEFT", [
                {"angles_deg": ["abc"], "t_offset_ms": 100},
            ])

    def test_string_t_offset_raises_valueerror(self):
        """Non-numeric t_offset must raise ValueError."""
        import pytest
        with pytest.raises(ValueError, match="not a valid integer"):
            build_batch("KNEE_LEFT", [
                {"angles_deg": [10.0], "t_offset_ms": "not_a_number"},
            ])

    def test_numeric_string_angle_coerced(self):
        """String '10.5' should be coerced to float 10.5."""
        batch = build_batch("KNEE_LEFT", [
            {"angles_deg": ["10.5"], "t_offset_ms": 100},
        ])
        assert batch.entries[0].angles_deg[0] == 10.5

    def test_numeric_string_t_offset_coerced(self):
        """String '200' should be coerced to int 200."""
        batch = build_batch("KNEE_LEFT", [
            {"angles_deg": [10.0], "t_offset_ms": "200"},
        ])
        assert batch.entries[0].t_offset_ms == 200

    def test_bool_angle_coerced_to_float(self):
        """Boolean True/False should coerce to float (1.0/0.0)."""
        batch = build_batch("KNEE_LEFT", [
            {"angles_deg": [True], "t_offset_ms": 100},
        ])
        assert batch.entries[0].angles_deg[0] == 1.0

    def test_list_angle_raises_valueerror(self):
        """A list inside angles_deg should raise ValueError."""
        import pytest
        with pytest.raises(ValueError, match="not numeric"):
            build_batch("KNEE_LEFT", [
                {"angles_deg": [[1, 2]], "t_offset_ms": 100},
            ])


# -----------------------------------------------------------------------
# get_dof_limits
# -----------------------------------------------------------------------
class TestGetDofLimits:
    """Tests for DOF limit lookup."""

    def test_single_dof_joint(self):
        limits = get_dof_limits("KNEE_LEFT")
        assert 0 in limits
        lo, hi = limits[0]
        assert lo == -2.0
        assert hi == 112.0

    def test_multi_dof_joint(self):
        limits = get_dof_limits("ANKLE_RIGHT")
        assert 0 in limits
        assert 1 in limits
        # DOF0: plantar_dorsal [-50, 25]
        assert limits[0] == (-50.0, 25.0)
        # DOF1: inversion_eversion [-25, 25]
        assert limits[1] == (-25.0, 25.0)

    def test_three_dof_joint(self):
        limits = get_dof_limits("HIP_LEFT")
        assert len(limits) == 3

    def test_unknown_joint_returns_empty(self):
        limits = get_dof_limits("NONEXISTENT")
        assert limits == {}


# -----------------------------------------------------------------------
# deduplicate_batch
# -----------------------------------------------------------------------
class TestDeduplicateBatch:
    """Tests for zero-step deduplication."""

    def test_removes_zero_step_duplicates(self):
        batch = build_batch("KNEE_LEFT", [
            {"angles_deg": [45.001], "t_offset_ms": 100},
            {"angles_deg": [45.002], "t_offset_ms": 200},  # Same int16 (4500)
            {"angles_deg": [45.015], "t_offset_ms": 300},  # Different (4502)
        ])
        deduped = deduplicate_batch(batch)
        assert len(deduped.entries) == 2
        assert deduped.entries[0].t_offset_ms == 100
        assert deduped.entries[1].t_offset_ms == 300

    def test_preserves_different_angles(self):
        batch = build_batch("KNEE_LEFT", [
            {"angles_deg": [10.0], "t_offset_ms": 100},
            {"angles_deg": [20.0], "t_offset_ms": 200},
            {"angles_deg": [30.0], "t_offset_ms": 300},
        ])
        deduped = deduplicate_batch(batch)
        assert len(deduped.entries) == 3  # No duplicates to remove

    def test_single_entry_unchanged(self):
        batch = build_batch("KNEE_LEFT", [
            {"angles_deg": [10.0], "t_offset_ms": 100},
        ])
        deduped = deduplicate_batch(batch)
        assert len(deduped.entries) == 1

    def test_empty_batch_unchanged(self):
        batch = WaypointBatch(joint="KNEE_LEFT", entries=[])
        deduped = deduplicate_batch(batch)
        assert len(deduped.entries) == 0

    def test_preserves_batch_id(self):
        batch = build_batch("KNEE_LEFT", [
            {"angles_deg": [45.001], "t_offset_ms": 100},
            {"angles_deg": [45.002], "t_offset_ms": 200},
        ])
        deduped = deduplicate_batch(batch)
        assert deduped.batch_id == batch.batch_id

    def test_multi_dof_all_same(self):
        """Multi-DOF: both DOFs quantize same → remove."""
        batch = build_batch("ANKLE_RIGHT", [
            {"angles_deg": [10.001, 5.001], "t_offset_ms": 100},
            {"angles_deg": [10.002, 5.002], "t_offset_ms": 200},
        ])
        deduped = deduplicate_batch(batch)
        assert len(deduped.entries) == 1

    def test_multi_dof_one_different_kept(self):
        """Multi-DOF: DOF0 same but DOF1 different → keep."""
        batch = build_batch("ANKLE_RIGHT", [
            {"angles_deg": [10.001, 5.0], "t_offset_ms": 100},
            {"angles_deg": [10.002, 5.02], "t_offset_ms": 200},
        ])
        deduped = deduplicate_batch(batch)
        assert len(deduped.entries) == 2  # DOF1 changed (502 vs 500)


# -----------------------------------------------------------------------
# batch_to_dicts
# -----------------------------------------------------------------------
class TestBatchToDicts:
    """Tests for WaypointBatch → dict list conversion."""

    def test_round_trip(self):
        raw = [
            {"angles_deg": [10.0, None, None], "t_offset_ms": 100},
            {"angles_deg": [20.0, None, None], "t_offset_ms": 200},
        ]
        batch = build_batch("KNEE_LEFT", raw)
        dicts = batch_to_dicts(batch)
        assert len(dicts) == 2
        assert dicts[0]["angles_deg"] == [10.0, None, None]
        assert dicts[0]["t_offset_ms"] == 100
        assert dicts[1]["angles_deg"] == [20.0, None, None]
        assert dicts[1]["t_offset_ms"] == 200
