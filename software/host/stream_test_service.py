"""
Continuous waypoint streaming test service.

Emulates production gateway-like constraints (fixed-cadence publish,
per-joint backpressure, fault injection) while running on the Flask
host bench.  Uses ``WaypointClient`` + ``HttpTransport`` to exercise
the full HTTP → Flask → CanManager → CAN pipeline.

See ``software/docs/WEB_CONTINUOUS_STREAM_TEST_SPEC.md`` for the full
specification.

Phase 1 model: single joint, single active DOF, bounded cosine
oscillation with mandatory preposition ramp.

Usage::

    from stream_test_service import StreamTestService
    service = StreamTestService(socketio=sio, base_url="http://127.0.0.1:5001")
    result = service.start({
        "joint": "KNEE_LEFT",
        "active_dof": 0,
        "n_dof": 1,
        "min_deg": 20.0,
        "max_deg": 80.0,
        "start_at": "min",
        "frequency_hz": 0.5,
        "rate_hz": 50,
        "duration_s": 60,
        ...
    })
    # later
    service.stop(result["session_id"])
"""
from __future__ import annotations

import enum
import json
import logging
import math
import random
import threading
import time
import uuid
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional

import requests as http_requests

from waypoint_client import (
    BatchRecord,
    BatchState,
    ClientConfig,
    HttpTransport,
    RetryPolicy,
    WaypointClient,
)

logger = logging.getLogger("stream_test")


# ---------------------------------------------------------------------------
# Session State Machine
# ---------------------------------------------------------------------------
class SessionState(enum.Enum):
    """Stream test session lifecycle states.

    Flow: IDLE → STARTING → PREPOSITIONING → RUNNING → STOPPING → STOPPED
    Any state → FAILED on irrecoverable error.
    """
    IDLE = "IDLE"
    STARTING = "STARTING"
    PREPOSITIONING = "PREPOSITIONING"
    RUNNING = "RUNNING"
    STOPPING = "STOPPING"
    STOPPED = "STOPPED"
    FAILED = "FAILED"


# ---------------------------------------------------------------------------
# Trajectory Generator
# ---------------------------------------------------------------------------
class TrajectoryGenerator:
    """Generate bounded cosine oscillation waypoint chunks for a single DOF.

    Oscillates between ``min_deg`` and ``max_deg`` using a cosine wave,
    which guarantees zero velocity at both extremes and bounded angles
    by construction.

    Only the ``active_dof`` index gets a computed angle; all other DOFs
    in ``angles_deg`` are ``None`` (firmware ignores them).
    """

    def __init__(
        self,
        min_deg: float,
        max_deg: float,
        start_at: str,
        frequency_hz: float,
        rate_hz: float,
        horizon_ms: float,
        active_dof: int,
        n_dof: int,
    ):
        self._min_deg = min_deg
        self._max_deg = max_deg
        self._center = (min_deg + max_deg) / 2.0
        self._amplitude = (max_deg - min_deg) / 2.0
        self._start_at = start_at  # "min" or "max"
        self._freq = frequency_hz
        self._active_dof = active_dof
        self._n_dof = n_dof
        self._rate_hz = rate_hz
        self._horizon_ms = horizon_ms
        self._period_ms = 1000.0 / rate_hz

    def next_chunk(self, t_base_ms: float, lead_ms: float = 0.0) -> List[Dict[str, Any]]:
        """Generate a single waypoint at *t_base_ms* with *lead_ms* lookahead.

        One WP per scheduler tick — no overlap between consecutive calls.

        ``t_base_ms`` is the *absolute* session time used for cosine phase.
        ``lead_ms`` is the lead-time offset for the waypoint's
        ``t_offset_ms`` (relative to "now").

        Returns list of ``{"angles_deg": [...], "t_offset_ms": int}``.
        """
        t_s = t_base_ms / 1000.0

        # Bounded cosine oscillation:
        #   cos(0) = 1  → at t=0: center ± amplitude = extreme
        #   Derivative of cos at t=0 is 0 → zero initial velocity
        if self._start_at == "max":
            angle = self._center + self._amplitude * math.cos(
                2 * math.pi * self._freq * t_s
            )
        else:  # start_at == "min"
            angle = self._center - self._amplitude * math.cos(
                2 * math.pi * self._freq * t_s
            )

        # Build angles_deg: active DOF gets angle, others are None
        angles_deg: List[Optional[float]] = [None] * self._n_dof
        angles_deg[self._active_dof] = round(angle, 2)

        # t_offset_ms = lead time from "now" (uint16, max 65535)
        t_offset = max(1, min(65535, int(lead_ms)))

        return [{
            "angles_deg": angles_deg,
            "t_offset_ms": t_offset,
        }]


# ---------------------------------------------------------------------------
# Per-Joint Backpressure
# ---------------------------------------------------------------------------
class _JointBackpressure:
    """Track inflight batches for one joint."""

    def __init__(self, max_inflight: int):
        self._max_inflight = max_inflight
        self._inflight: int = 0
        self._consecutive_full: int = 0
        self._lock = threading.Lock()

    def can_send(self) -> bool:
        """Return True if there is room to send another chunk."""
        with self._lock:
            if self._inflight >= self._max_inflight:
                self._consecutive_full += 1
                return False
            self._consecutive_full = 0
            return True

    @property
    def should_drop(self) -> bool:
        """True if consecutive deferrals exceed threshold (>3)."""
        with self._lock:
            return self._consecutive_full > 3

    def on_send(self) -> None:
        with self._lock:
            self._inflight += 1

    def on_done(self) -> None:
        with self._lock:
            self._inflight = max(0, self._inflight - 1)

    @property
    def inflight(self) -> int:
        with self._lock:
            return self._inflight

    @property
    def fill(self) -> int:
        """Current queue fill level for UI gauges."""
        with self._lock:
            return self._inflight

    @property
    def max_capacity(self) -> int:
        with self._lock:
            return self._max_inflight


# ---------------------------------------------------------------------------
# Fault Injector
# ---------------------------------------------------------------------------
class _FaultInjector:
    """Pre-send hook that simulates fault profiles.

    Modes: ``none``, ``delay_jitter``, ``drop_rate``, ``burst_pause``,
    ``sync_stale``, ``conflict_spike``.
    """

    def __init__(self, profile: dict):
        self._mode = profile.get("mode", "none")
        seed = profile.get("seed")
        self._rng = random.Random(seed)
        # Mode-specific parameters
        self._drop_rate = profile.get("drop_rate", 0.01)
        self._delay_min_ms = profile.get("delay_min_ms", 1)
        self._delay_max_ms = profile.get("delay_max_ms", 20)
        self._burst_period = profile.get("burst_period", 100)
        self._burst_pause_ms = profile.get("burst_pause_ms", 500)

    def pre_send(self, tick: int) -> str:
        """Return action: 'send', 'delay', 'drop', or 'pause'."""
        if self._mode == "none":
            return "send"
        elif self._mode == "delay_jitter":
            return "delay"
        elif self._mode == "drop_rate":
            if self._rng.random() < self._drop_rate:
                return "drop"
            return "send"
        elif self._mode == "burst_pause":
            if tick > 0 and tick % self._burst_period == 0:
                return "pause"
            return "send"
        elif self._mode == "sync_stale":
            return "send"  # Handled by disabling sync refresh
        elif self._mode == "conflict_spike":
            return "send"  # Handled by sending duplicate batches
        return "send"

    @property
    def delay_ms(self) -> float:
        return self._rng.uniform(self._delay_min_ms, self._delay_max_ms)

    @property
    def pause_ms(self) -> float:
        return self._burst_pause_ms

    @property
    def suppress_sync(self) -> bool:
        return self._mode == "sync_stale"

    @property
    def conflict_spike(self) -> bool:
        return self._mode == "conflict_spike"


# ---------------------------------------------------------------------------
# Event Log
# ---------------------------------------------------------------------------
class _EventLog:
    """Bounded event buffer with monotonic sequence numbers."""

    MAX_EVENTS = 10000
    TRIM_TO = 5000

    def __init__(self):
        self._events: List[dict] = []
        self._seq: int = 0
        self._lock = threading.Lock()

    def emit(
        self,
        level: str,
        event: str,
        joint: str = "",
        detail: str = "",
    ) -> None:
        with self._lock:
            self._seq += 1
            entry = {
                "seq": self._seq,
                "ts": time.time(),
                "level": level,
                "event": event,
                "joint": joint,
                "detail": detail,
            }
            self._events.append(entry)
            if len(self._events) > self.MAX_EVENTS:
                self._events = self._events[-self.TRIM_TO:]

    def since(self, after_seq: int = 0) -> List[dict]:
        with self._lock:
            return [e for e in self._events if e["seq"] > after_seq]

    def all(self) -> List[dict]:
        with self._lock:
            return list(self._events)


# ---------------------------------------------------------------------------
# Firmware Telemetry Poller (background thread)
# ---------------------------------------------------------------------------
class _TelemetryPoller:
    """Background daemon that polls firmware telemetry on its own schedule.

    The publish loop reads the cached result without blocking.
    Modeled after CanManager._listen_loop daemon pattern.
    """

    def __init__(self, transport: Any, joint: str, interval_s: float = 1.0):
        self._transport = transport
        self._joint = joint
        self._interval_s = interval_s
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._cached: Optional[dict] = None
        self._thread = threading.Thread(
            target=self._poll_loop,
            name=f"telemetry-poller-{joint}",
            daemon=True,
        )

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def get_cached(self) -> Optional[dict]:
        """Non-blocking read of latest telemetry. Returns None if never received."""
        with self._lock:
            return self._cached

    def _poll_loop(self) -> None:
        while not self._stop.wait(timeout=self._interval_s):
            try:
                telem = self._transport.request_wp_telemetry(self._joint)
                if telem is not None:
                    with self._lock:
                        self._cached = telem
            except Exception:
                pass  # Best-effort, don't crash the poller


# ---------------------------------------------------------------------------
# Stream Metrics
# ---------------------------------------------------------------------------
class StreamMetrics:
    """Aggregate metrics for a streaming session (spec section 6.4)."""

    def __init__(self, target_rate_hz: float, joints: List[str],
                 max_inflight: int = 1):
        self._target_rate = target_rate_hz
        self._joints = joints
        self._max_inflight = max_inflight
        self._lock = threading.Lock()

        # Counters
        self._chunks_sent: int = 0          # enqueued (attempted)
        self._chunks_confirmed: int = 0     # COMPLETE or PARTIAL
        self._chunks_failed: int = 0        # FAILED_HARD
        self._chunks_dropped: int = 0
        self._chunks_deferred: int = 0
        self._waypoints_sent: int = 0
        self._waypoints_partial: int = 0
        self._waypoints_late: int = 0
        self._sync_refresh_count: int = 0
        self._last_error: Optional[str] = None

        # Drift tracking
        self._drift_samples: deque = deque(maxlen=1000)

        # Per-joint queue fill max
        self._queue_fill_max: Dict[str, int] = {j: 0 for j in joints}

        # HTTP status counts (from WaypointClient metrics)
        self._http_status_counts: Dict[int, int] = {}
        self._extra_status_counts: Dict[int, int] = {}  # conflict spike etc.
        self._retries: int = 0

        # Firmware telemetry (from 0x4D0 on-demand response)
        # Wrap-aware accumulation: CAN frame sends low 16 bits of uint32 counters
        self._fw_wp_accepted: int = 0
        self._fw_wp_dropped: int = 0
        self._fw_buffer_fill: int = 0
        self._fw_prev_accepted: int = 0
        self._fw_prev_dropped_full: int = 0
        self._fw_prev_dropped_guard: int = 0
        self._fw_accum_accepted: int = 0
        self._fw_accum_dropped: int = 0
        self._fw_telemetry_initialized: bool = False

        # Rate calculation
        self._start_time: float = time.monotonic()
        self._tick_count: int = 0

    def record_drift(self, drift_ms: float) -> None:
        with self._lock:
            self._drift_samples.append(abs(drift_ms))
            self._tick_count += 1

    def inc_sent(self, joint: str, wp_count: int) -> None:
        with self._lock:
            self._chunks_sent += 1
            self._waypoints_sent += wp_count

    def inc_dropped(self, joint: str) -> None:
        with self._lock:
            self._chunks_dropped += 1

    def inc_deferred(self, joint: str) -> None:
        with self._lock:
            self._chunks_deferred += 1

    def inc_partial(self, wp_count: int) -> None:
        with self._lock:
            self._waypoints_partial += wp_count

    def inc_confirmed(self) -> None:
        with self._lock:
            self._chunks_confirmed += 1

    def inc_failed(self) -> None:
        with self._lock:
            self._chunks_failed += 1

    def inc_late(self, count: int) -> None:
        with self._lock:
            self._waypoints_late += count

    def inc_sync_refresh(self) -> None:
        with self._lock:
            self._sync_refresh_count += 1

    def set_error(self, msg: str) -> None:
        with self._lock:
            self._last_error = msg

    def update_queue_fill(self, joint: str, fill: int) -> None:
        with self._lock:
            if fill > self._queue_fill_max.get(joint, 0):
                self._queue_fill_max[joint] = fill

    def record_http_status(self, code: int) -> None:
        """Record an extra HTTP status code (e.g. from conflict spike)."""
        with self._lock:
            self._extra_status_counts[code] = (
                self._extra_status_counts.get(code, 0) + 1
            )

    def merge_client_metrics(self, client_metrics: dict) -> None:
        """Pull HTTP status counts and retries from WaypointClient."""
        with self._lock:
            self._http_status_counts = client_metrics.get("status_counts", {})
            self._retries = client_metrics.get("retries", 0)

    def update_fw_telemetry(self, telem: dict) -> None:
        """Update firmware-side telemetry with uint16 wrap handling.

        CAN frame sends low 16 bits of uint32 counters.  At 1s poll interval
        and 300 wp/s max, delta per interval is ~300, well under 65535.
        """
        with self._lock:
            raw_acc = telem.get("wp_accepted", 0)
            raw_df = telem.get("wp_dropped_full", 0)
            raw_dg = telem.get("wp_dropped_guard", 0)

            if not self._fw_telemetry_initialized:
                # First poll: just record prev values, don't accumulate
                self._fw_prev_accepted = raw_acc
                self._fw_prev_dropped_full = raw_df
                self._fw_prev_dropped_guard = raw_dg
                self._fw_telemetry_initialized = True
                self._fw_buffer_fill = telem.get("buffer_fill", 0)
                return

            # Delta with 16-bit wrap handling
            d_acc = (raw_acc - self._fw_prev_accepted) & 0xFFFF
            d_df = (raw_df - self._fw_prev_dropped_full) & 0xFFFF
            d_dg = (raw_dg - self._fw_prev_dropped_guard) & 0xFFFF

            self._fw_prev_accepted = raw_acc
            self._fw_prev_dropped_full = raw_df
            self._fw_prev_dropped_guard = raw_dg

            self._fw_accum_accepted += d_acc
            self._fw_accum_dropped += d_df + d_dg

            self._fw_wp_accepted = self._fw_accum_accepted
            self._fw_wp_dropped = self._fw_accum_dropped
            self._fw_buffer_fill = telem.get("buffer_fill", 0)

    def snapshot(self) -> dict:
        with self._lock:
            elapsed = time.monotonic() - self._start_time
            actual_rate = self._tick_count / elapsed if elapsed > 0 else 0.0

            # p95 drift
            if self._drift_samples:
                sorted_d = sorted(self._drift_samples)
                idx = int(len(sorted_d) * 0.95)
                p95 = sorted_d[min(idx, len(sorted_d) - 1)]
            else:
                p95 = 0.0

            # Merge client + extra (conflict spike) HTTP status counts
            merged_status: Dict[int, int] = dict(self._http_status_counts)
            for code, cnt in self._extra_status_counts.items():
                merged_status[code] = merged_status.get(code, 0) + cnt

            # partial ratio: HTTP 207 count / attempted chunks
            partial_count = merged_status.get(207, 0)
            partial_ratio = partial_count / max(1, self._chunks_sent)

            # late ratio: real late waypoints / total waypoints sent
            late_ratio = self._waypoints_late / max(1, self._waypoints_sent)

            return {
                "target_rate_hz": self._target_rate,
                "actual_rate_hz": round(actual_rate, 1),
                "scheduler_drift_ms_p95": round(p95, 2),
                "chunks_sent": self._chunks_sent,
                "chunks_confirmed": self._chunks_confirmed,
                "chunks_failed": self._chunks_failed,
                "chunks_dropped": self._chunks_dropped,
                "chunks_deferred": self._chunks_deferred,
                "waypoints_sent": self._waypoints_sent,
                "waypoints_partial": self._waypoints_partial,
                "waypoints_late": self._waypoints_late,
                "http_status_counts": merged_status,
                "retries": self._retries,
                "queue_fill_max": dict(self._queue_fill_max),
                "max_inflight_per_joint": self._max_inflight,
                "partial_ratio": round(partial_ratio, 4),
                "late_ratio": round(late_ratio, 6),
                "sync_refresh_count": self._sync_refresh_count,
                "last_error": self._last_error,
                "fw_wp_accepted": self._fw_wp_accepted,
                "fw_wp_dropped": self._fw_wp_dropped,
                "fw_buffer_fill": self._fw_buffer_fill,
            }


# ---------------------------------------------------------------------------
# Stream Session (publish loop thread)
# ---------------------------------------------------------------------------
class _StreamSession:
    """Single streaming session — owns the publish loop thread."""

    def __init__(
        self,
        session_id: str,
        config: dict,
        base_url: str,
        socketio: Any = None,
    ):
        self.session_id = session_id
        self._config = config
        self._base_url = base_url
        self._socketio = socketio
        self._state = SessionState.STARTING
        self._started_at = time.time()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._stop_reason: Optional[str] = None

        # Single joint (Phase 1)
        joint = config["joints"][0]
        rate_hz = config["rate_hz"]
        horizon_ms = config.get("horizon_ms", 250)
        max_inflight = config.get("max_inflight_per_joint", 1)
        fault_profile = config.get("fault_profile", {"mode": "none"})

        active_dof = config.get("active_dof", 0)
        n_dof = config.get("n_dof", 1)

        self._trajectory: Dict[str, TrajectoryGenerator] = {
            joint: TrajectoryGenerator(
                min_deg=config["min_deg"],
                max_deg=config["max_deg"],
                start_at=config.get("start_at", "min"),
                frequency_hz=config.get("frequency_hz", 0.5),
                rate_hz=rate_hz,
                horizon_ms=horizon_ms,
                active_dof=active_dof,
                n_dof=n_dof,
            ),
        }
        self._backpressure: Dict[str, _JointBackpressure] = {
            joint: _JointBackpressure(max_inflight),
        }

        self._fault = _FaultInjector(fault_profile)
        self.metrics = StreamMetrics(rate_hz, [joint], max_inflight=max_inflight)
        self.events = _EventLog()

        # Batch completion tracking
        self._pending_batches: Dict[str, str] = {}  # batch_id → joint

    @property
    def state(self) -> SessionState:
        return self._state

    @state.setter
    def state(self, value: SessionState) -> None:
        old = self._state
        self._state = value
        self.events.emit("INFO", "state_change", detail=f"{old.value} -> {value.value}")
        self._emit_state()

    @property
    def uptime_s(self) -> float:
        return time.time() - self._started_at

    def start(self) -> None:
        """Launch the publish loop thread."""
        self._thread = threading.Thread(
            target=self._run,
            name=f"stream-test-{self.session_id}",
            daemon=True,
        )
        self._thread.start()

    def stop(self, reason: str = "operator_stop") -> None:
        """Signal the publish loop to stop and wait for cleanup."""
        self._stop_reason = reason
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=5.0)
        if self._state not in (SessionState.STOPPED, SessionState.FAILED):
            self.state = SessionState.STOPPED

    # -- Publish loop ------------------------------------------------------

    def _run(self) -> None:
        """Preposition + fixed-cadence publish loop with deadline tracking."""
        config = self._config
        rate_hz = config["rate_hz"]
        joint = config["joints"][0]  # single joint (Phase 1)
        duration_s = config.get("duration_s", 60)
        active_dof = config.get("active_dof", 0)
        n_dof = config.get("n_dof", 1)

        # Create per-session WaypointClient + HttpTransport
        transport = HttpTransport(self._base_url, timeout_s=10.0)
        client_config = ClientConfig(
            base_url=self._base_url,
            retry=RetryPolicy(
                base_backoff_409_s=0.05,
                base_backoff_502_s=0.2,
                base_backoff_500_s=0.5,
                base_backoff_timeout_s=0.2,
            ),
            max_queue_size=config.get("max_inflight_per_joint", 1),
        )
        client = WaypointClient(transport, config=client_config)
        telemetry_poller = _TelemetryPoller(transport, joint)

        try:
            # Initial time sync
            if not self._fault.suppress_sync:
                sync_ok = transport.send_time_sync()
                if sync_ok:
                    self.events.emit("INFO", "time_sync", detail="initial sync sent")
                else:
                    self.events.emit("WARN", "time_sync_fail", detail="initial sync failed")

            # ============================================================
            # PREPOSITIONING PHASE
            # ============================================================
            self.state = SessionState.PREPOSITIONING

            start_at = config.get("start_at", "min")
            prepos_target = config["min_deg"] if start_at == "min" else config["max_deg"]

            # 1. Read current encoder position via loopback HTTP
            try:
                enc_resp = http_requests.get(
                    f"{self._base_url}/can/encoder_angles",
                    params={"joint": joint},
                    timeout=2.0,
                )
                enc = enc_resp.json()
            except Exception as e:
                self.state = SessionState.FAILED
                self.events.emit("ERROR", "encoder_read_failed", joint, str(e))
                return

            if not enc.get("valid") or enc.get("age_ms", 9999) > 300:
                self.state = SessionState.FAILED
                self.events.emit(
                    "ERROR", "encoder_stale", joint,
                    f"age_ms={enc.get('age_ms')}, valid={enc.get('valid')}",
                )
                return

            encoder_angles = enc.get("angles_deg", [])
            if active_dof >= len(encoder_angles) or encoder_angles[active_dof] is None:
                self.state = SessionState.FAILED
                self.events.emit(
                    "ERROR", "encoder_null_dof", joint,
                    f"DOF {active_dof} has no reading",
                )
                return

            current_angle = encoder_angles[active_dof]

            # 2. Ramp to preposition target: 20 waypoints over 2s
            # Use linear profile for slow ramps, cosine S-curve for fast ones
            ramp_duration_s = 2.0
            ramp_steps = 20
            delta_t_ms = int(ramp_duration_s / ramp_steps * 1000)
            slow_threshold = 15.0  # deg/s — matches JS SLOW_MOTION_THRESHOLD_DEG_S
            ramp_peak_vel = abs(prepos_target - current_angle) * math.pi / (2 * ramp_duration_s)

            ramp_waypoints = []
            for i in range(ramp_steps):
                t = (i + 1) / ramp_steps  # 0.05 to 1.0
                if ramp_peak_vel < slow_threshold:
                    smooth_t = t  # Linear: constant velocity
                else:
                    smooth_t = 0.5 * (1 - math.cos(t * math.pi))  # Cosine S-curve
                angle = current_angle + (prepos_target - current_angle) * smooth_t

                angles = [None] * n_dof
                angles[active_dof] = round(angle, 2)
                ramp_waypoints.append({
                    "angles_deg": angles,
                    "t_offset_ms": (i + 1) * delta_t_ms,
                })

            # 3. Send ramp via WaypointClient (loopback)
            try:
                client.enqueue_batch(joint, ramp_waypoints)
                self.events.emit(
                    "INFO", "preposition_ramp_sent", joint,
                    f"target={prepos_target}, from={current_angle:.1f}, "
                    f"{ramp_steps} wp over {ramp_duration_s}s",
                )
            except Exception as e:
                self.state = SessionState.FAILED
                self.events.emit("ERROR", "preposition_send_failed", joint, str(e))
                return

            # 4. Wait for ramp to complete
            if self._stop_event.wait(timeout=ramp_duration_s + 0.5):
                return  # stopped during preposition

            # 5. Ready check: poll encoder, 3 consecutive ±1°, timeout 3.0s
            ready_count = 0
            ready_deadline = time.monotonic() + 3.0
            ready_reached = False
            while time.monotonic() < ready_deadline:
                if self._stop_event.is_set():
                    return
                try:
                    r = http_requests.get(
                        f"{self._base_url}/can/encoder_angles",
                        params={"joint": joint},
                        timeout=1.0,
                    )
                    edata = r.json()
                    if edata.get("valid") and edata.get("age_ms", 9999) <= 300:
                        ea = edata.get("angles_deg", [])
                        if active_dof < len(ea) and ea[active_dof] is not None:
                            actual = ea[active_dof]
                            if abs(actual - prepos_target) <= 1.0:
                                ready_count += 1
                                if ready_count >= 3:
                                    ready_reached = True
                                    break
                            else:
                                ready_count = 0
                        else:
                            ready_count = 0
                    else:
                        ready_count = 0
                except Exception:
                    ready_count = 0
                time.sleep(0.1)

            if not ready_reached:
                self.state = SessionState.FAILED
                self.events.emit(
                    "ERROR", "preposition_timeout", joint,
                    f"target={prepos_target}, not reached within 3s",
                )
                return

            self.events.emit(
                "INFO", "preposition_ready", joint,
                f"target={prepos_target}, ready ({ready_count} consecutive ±1°)",
            )

            # ============================================================
            # STREAMING PHASE
            # ============================================================
            # Fresh time sync right before streaming — preposition may
            # have consumed > MAX_SYNC_AGE_MS (2000ms).
            if not self._fault.suppress_sync:
                ok = transport.send_time_sync()
                if ok:
                    self.events.emit("INFO", "time_sync", detail="pre-stream refresh")
                else:
                    self.events.emit("WARN", "time_sync_fail", detail="pre-stream refresh failed")

            self.state = SessionState.RUNNING
            telemetry_poller.start()
            period_s = 1.0 / rate_hz
            next_tick = time.monotonic()
            tick = 0
            # Refresh time sync well within MAX_SYNC_AGE_MS (2000ms).
            # At 50 Hz → every 50 ticks (1s), at 100 Hz → every 100 ticks (1s).
            sync_interval_ticks = max(1, int(1.0 * rate_hz))
            metrics_interval_ticks = int(rate_hz)  # every 1s
            max_ticks = int(duration_s * rate_hz)
            sync_fail_start: Optional[float] = None

            while not self._stop_event.is_set() and tick < max_ticks:
                now = time.monotonic()
                drift = (now - next_tick) * 1000  # ms
                self.metrics.record_drift(drift)

                bp = self._backpressure[joint]

                # Backpressure check
                if not bp.can_send():
                    if bp.should_drop:
                        self.metrics.inc_dropped(joint)
                        self.events.emit(
                            "WARN", "backpressure_drop", joint,
                            "consecutive overload > 3 ticks",
                        )
                    else:
                        self.metrics.inc_deferred(joint)
                        self.events.emit(
                            "WARN", "backpressure_defer", joint,
                            "queue full, defer one chunk",
                        )
                else:
                    # Fault injection
                    action = self._fault.pre_send(tick)
                    if action == "drop":
                        self.metrics.inc_dropped(joint)
                        self.events.emit("WARN", "fault_drop", joint)
                    elif action == "pause":
                        pause_ms = self._fault.pause_ms
                        self.events.emit(
                            "WARN", "fault_pause", joint,
                            f"burst pause {pause_ms}ms",
                        )
                        self._stop_event.wait(timeout=pause_ms / 1000)
                        if self._stop_event.is_set():
                            break
                    else:
                        if action == "delay":
                            delay_ms = self._fault.delay_ms
                            time.sleep(delay_ms / 1000)

                        # Generate chunk
                        t_base_ms = tick * period_s * 1000
                        # Cumulative t_offset: each WP needs a unique,
                        # increasing offset relative to the firmware batch
                        # anchor (set when buffer was empty at tick 0).
                        # lead_ms = base lookahead + elapsed ticks.
                        base_lead_ms = config.get("horizon_ms", 250)
                        tick_period_ms = 1000.0 / rate_hz
                        cumulative_lead_ms = base_lead_ms + tick * tick_period_ms
                        chunk = self._trajectory[joint].next_chunk(
                            t_base_ms, lead_ms=cumulative_lead_ms,
                        )

                        # Send via WaypointClient
                        try:
                            batch_id = client.enqueue_batch(joint, chunk)
                            bp.on_send()
                            self.metrics.inc_sent(joint, len(chunk))
                            self._pending_batches[batch_id] = joint

                            # Conflict spike: send duplicate directly via
                            # transport to bypass WaypointClient serialisation
                            if self._fault.conflict_spike and tick % 10 == 0:
                                def _fire_conflict(j=joint, wp=chunk):
                                    try:
                                        resp = transport.send_batch(
                                            j, wp, f"conflict-{tick}",
                                        )
                                        code = resp.http_status
                                        self.metrics.record_http_status(code)
                                        if code == 409:
                                            self.events.emit(
                                                "INFO", "conflict_409", j,
                                                "parallel duplicate got expected 409",
                                            )
                                    except Exception:
                                        pass
                                threading.Thread(
                                    target=_fire_conflict, daemon=True,
                                ).start()
                        except Exception as exc:
                            self.metrics.set_error(str(exc))
                            self.events.emit(
                                "ERROR", "enqueue_failed", joint, str(exc),
                            )

                # Poll completed batches
                self._poll_completions(client)

                # Update queue fill metrics
                self.metrics.update_queue_fill(joint, bp.fill)

                # Periodic time sync (every ~1s)
                if (tick > 0
                        and tick % sync_interval_ticks == 0
                        and not self._fault.suppress_sync):
                    ok = transport.send_time_sync()
                    if ok:
                        self.metrics.inc_sync_refresh()
                        self.events.emit("INFO", "time_sync", detail="periodic refresh")
                        sync_fail_start = None
                    else:
                        if sync_fail_start is None:
                            sync_fail_start = time.monotonic()
                        elif time.monotonic() - sync_fail_start > 5.0:
                            self.state = SessionState.FAILED
                            self.events.emit(
                                "ERROR", "sync_timeout",
                                detail="sync failed continuously for >5s",
                            )
                            break

                # Periodic metrics merge + emit (every 1s)
                if tick % max(1, metrics_interval_ticks) == 0:
                    self.metrics.merge_client_metrics(client.get_metrics())
                    # Read cached firmware telemetry (non-blocking)
                    telem = telemetry_poller.get_cached()
                    if telem:
                        self.metrics.update_fw_telemetry(telem)
                    self._emit_metrics()

                tick += 1
                next_tick += period_s

                # Interruptible sleep until next tick
                sleep_time = next_tick - time.monotonic()
                if sleep_time > 0:
                    self._stop_event.wait(timeout=sleep_time)

            # Natural end or stop requested
            if self._state == SessionState.RUNNING:
                self.state = SessionState.STOPPING

        except Exception as exc:
            logger.exception("Stream session %s crashed", self.session_id)
            self.metrics.set_error(str(exc))
            self.events.emit("ERROR", "session_crash", detail=str(exc))
            self.state = SessionState.FAILED
        finally:
            # Stop telemetry poller
            telemetry_poller.stop()
            # Drain pending and shutdown client
            try:
                client.shutdown(timeout_s=5.0)
            except Exception:
                pass
            try:
                transport.close()
            except Exception:
                pass

            if self._state == SessionState.STOPPING:
                self.state = SessionState.STOPPED

            # Final metrics snapshot
            self.metrics.merge_client_metrics(client.get_metrics())
            self._emit_metrics()
            self.events.emit(
                "INFO", "session_end",
                detail=f"reason={self._stop_reason or 'duration_complete'}",
            )

    def _poll_completions(self, client: WaypointClient) -> None:
        """Check pending batches and update backpressure on completion."""
        completed = []
        for batch_id, joint in self._pending_batches.items():
            record = client.get_batch_status(batch_id)
            if record and record.completed_at is not None:
                self._backpressure[joint].on_done()
                if record.state == BatchState.FAILED_HARD:
                    self.metrics.inc_failed()
                else:
                    self.metrics.inc_confirmed()
                    result = record.result or {}
                    # Extract real late_count from can_manager response
                    late = result.get("late_count", 0)
                    if late > 0:
                        self.metrics.inc_late(late)
                    if record.state == BatchState.PARTIAL:
                        total = result.get("total", 0)
                        sent = result.get("sent", 0)
                        failed_wp = max(0, total - sent)
                        self.metrics.inc_partial(failed_wp)
                completed.append(batch_id)
        for bid in completed:
            del self._pending_batches[bid]

    def _emit_metrics(self) -> None:
        """Push metrics snapshot via SocketIO."""
        if self._socketio:
            try:
                self._socketio.emit(
                    "stream_test_metrics",
                    self.metrics.snapshot(),
                    namespace="/movement",
                )
            except Exception:
                pass

    def _emit_state(self) -> None:
        """Push state change via SocketIO."""
        if self._socketio:
            try:
                self._socketio.emit(
                    "stream_test_state",
                    {
                        "session_id": self.session_id,
                        "state": self._state.value,
                    },
                    namespace="/movement",
                )
            except Exception:
                pass


# ---------------------------------------------------------------------------
# StreamTestService (top-level facade)
# ---------------------------------------------------------------------------
class StreamTestService:
    """Manages stream test sessions.

    Thread-safe.  Only one active session at a time (v1).
    """

    def __init__(
        self,
        socketio: Any = None,
        base_url: str = "http://localhost:5001",
    ):
        self._socketio = socketio
        self._base_url = base_url
        self._lock = threading.Lock()
        self._session: Optional[_StreamSession] = None

    # -- Public API --------------------------------------------------------

    def start(self, config: dict) -> dict:
        """Start a new streaming session.

        Args:
            config: Session configuration (Phase 1: single joint, single DOF).

        Returns:
            ``{"session_id": str, "state": str}``

        Raises:
            ValueError: If config is invalid.
            RuntimeError: If a session is already running (409).
        """
        # --- Validate required fields (Phase 1: single joint) ---
        joint = config.get("joint")
        if not joint or not isinstance(joint, str):
            raise ValueError("'joint' must be a non-empty string")

        # Cross-check joint name and DOF count against firmware config
        joint_upper = joint.upper()
        try:
            from config import JOINTS as _JOINTS
            joint_entry = _JOINTS.get(joint_upper)
            if joint_entry is None:
                raise ValueError(f"Unknown joint '{joint_upper}'")
            real_dof_count = len(joint_entry["dofs"])
        except ImportError:
            real_dof_count = None  # graceful: config unavailable in tests

        rate_hz = config.get("rate_hz", 50)
        if rate_hz not in (50, 100):
            raise ValueError("'rate_hz' must be 50 or 100")

        # active_dof / n_dof
        active_dof = config.get("active_dof", 0)
        n_dof = config.get("n_dof", 1)
        if real_dof_count is not None and n_dof != real_dof_count:
            raise ValueError(
                f"n_dof={n_dof} does not match joint '{joint_upper}' "
                f"which has {real_dof_count} DOF(s)"
            )
        if not isinstance(active_dof, int) or active_dof < 0 or active_dof >= n_dof:
            raise ValueError(
                f"active_dof {active_dof} out of range [0, {n_dof})"
            )

        # min_deg / max_deg (coerce to float to handle JSON int/string)
        min_deg = config.get("min_deg")
        max_deg = config.get("max_deg")
        if min_deg is None or max_deg is None:
            raise ValueError("'min_deg' and 'max_deg' are required")
        try:
            min_deg = float(min_deg)
            max_deg = float(max_deg)
        except (TypeError, ValueError):
            raise ValueError("'min_deg' and 'max_deg' must be numeric")
        if not math.isfinite(min_deg) or not math.isfinite(max_deg):
            raise ValueError("'min_deg' and 'max_deg' must be finite numbers")
        if min_deg >= max_deg:
            raise ValueError("min_deg must be < max_deg")

        # safe_limits — use firmware limits if available, otherwise accept manual range
        safe = config.get("safe_limits")
        if not safe or not isinstance(safe, dict) or "min" not in safe or "max" not in safe:
            # No firmware safe limits — trust the manual min/max from UI
            logger.warning("No firmware safe_limits — using manual range [%s, %s]", min_deg, max_deg)
            safe = {"min": min_deg, "max": max_deg}
        if min_deg < safe["min"] or max_deg > safe["max"]:
            raise ValueError(
                f"Range [{min_deg}, {max_deg}] exceeds safe limits "
                f"[{safe['min']}, {safe['max']}]"
            )

        # start_at
        start_at = config.get("start_at", "min")
        if start_at not in ("min", "max"):
            raise ValueError("start_at must be 'min' or 'max'")

        # frequency_hz (coerce to float)
        freq = config.get("frequency_hz", 0.5)
        try:
            freq = float(freq)
        except (TypeError, ValueError):
            raise ValueError("'frequency_hz' must be numeric")
        if not math.isfinite(freq) or freq <= 0:
            raise ValueError("frequency_hz must be a positive finite number")

        with self._lock:
            if self._session and self._session.state in (
                SessionState.STARTING, SessionState.PREPOSITIONING,
                SessionState.RUNNING,
            ):
                raise RuntimeError("Session already active")

            session_id = self._make_session_id()
            session = _StreamSession(
                session_id=session_id,
                config={
                    "joints": [joint.upper()],
                    "rate_hz": rate_hz,
                    "duration_s": config.get("duration_s", 60),
                    "horizon_ms": config.get("horizon_ms", 250),
                    "buffer_depth_sim": config.get("buffer_depth_sim", 2),
                    "max_inflight_per_joint": config.get("max_inflight_per_joint", 1),
                    "active_dof": active_dof,
                    "n_dof": n_dof,
                    "min_deg": float(min_deg),
                    "max_deg": float(max_deg),
                    "start_at": start_at,
                    "frequency_hz": float(freq),
                    "fault_profile": config.get("fault_profile", {"mode": "none"}),
                },
                base_url=self._base_url,
                socketio=self._socketio,
            )
            self._session = session

        session.start()
        return {"session_id": session_id, "state": session.state.value}

    def stop(self, session_id: str, reason: str = "operator_stop") -> dict:
        """Stop the active session.  Idempotent."""
        with self._lock:
            session = self._session
        if session is None:
            return {"status": "success", "session_id": session_id, "state": "IDLE"}
        if session.session_id != session_id:
            raise ValueError(f"Unknown session_id: {session_id}")
        session.stop(reason=reason)
        return {
            "status": "success",
            "session_id": session_id,
            "state": session.state.value,
        }

    def get_status(self, session_id: str = None) -> dict:
        """Return current session status."""
        with self._lock:
            session = self._session
        if session is None:
            return {
                "session": None,
                "state": SessionState.IDLE.value,
            }
        if session_id and session.session_id != session_id:
            return {"session": None, "state": "UNKNOWN"}
        return {
            "session": {
                "session_id": session.session_id,
                "state": session.state.value,
                "started_at": session._started_at,
                "uptime_s": round(session.uptime_s, 1),
                "config": session._config,
            }
        }

    def get_metrics(self) -> dict:
        """Return metrics snapshot of active or last session."""
        with self._lock:
            session = self._session
        if session is None:
            return {}
        return session.metrics.snapshot()

    def get_events(self, session_id: str = None, after_seq: int = 0) -> List[dict]:
        """Return events since *after_seq*."""
        with self._lock:
            session = self._session
        if session is None:
            return []
        if session_id and session.session_id != session_id:
            return []
        return session.events.since(after_seq)

    # -- Internal ----------------------------------------------------------

    @staticmethod
    def _make_session_id() -> str:
        ts = time.strftime("%Y%m%d_%H%M%S")
        suffix = uuid.uuid4().hex[:4]
        return f"st_{ts}_{suffix}"
