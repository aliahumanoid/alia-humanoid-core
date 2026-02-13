"""
Continuous waypoint streaming test service.

Emulates production gateway-like constraints (fixed-cadence publish,
per-joint backpressure, fault injection) while running on the Flask
host bench.  Uses ``WaypointClient`` + ``HttpTransport`` to exercise
the full HTTP → Flask → CanManager → CAN pipeline.

See ``software/docs/WEB_CONTINUOUS_STREAM_TEST_SPEC.md`` for the full
specification.

Usage::

    from stream_test_service import StreamTestService
    service = StreamTestService(socketio=sio, base_url="http://127.0.0.1:5001")
    result = service.start({
        "joints": ["KNEE_LEFT"],
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
    """Stream test session lifecycle states."""
    IDLE = "IDLE"
    STARTING = "STARTING"
    RUNNING = "RUNNING"
    STOPPING = "STOPPING"
    STOPPED = "STOPPED"
    FAILED = "FAILED"


# ---------------------------------------------------------------------------
# Trajectory Generator
# ---------------------------------------------------------------------------
class TrajectoryGenerator:
    """Generate waypoint chunks for a single joint.

    Currently supports ``sinusoid`` trajectories.  Extensible to
    ``cosine``, ``step``, ``custom`` in future versions.
    """

    def __init__(
        self,
        traj_config: dict,
        rate_hz: float,
        horizon_ms: float,
        n_dof: int = 1,
    ):
        self._type = traj_config.get("type", "sinusoid")
        self._amplitude = traj_config.get("amplitude_deg", 8.0)
        self._offset = traj_config.get("offset_deg", 0.0)
        self._freq = traj_config.get("frequency_hz", 0.5)
        self._rate_hz = rate_hz
        self._horizon_ms = horizon_ms
        self._n_dof = n_dof
        self._period_ms = 1000.0 / rate_hz

    def next_chunk(self, t_base_ms: float) -> List[Dict[str, Any]]:
        """Generate a short chunk of 2-4 waypoints starting at *t_base_ms*.

        Returns list of ``{"angles_deg": [...], "t_offset_ms": int}``.
        """
        # Number of waypoints per chunk: cover horizon window
        n_wp = max(2, min(4, int(self._horizon_ms / self._period_ms)))
        chunk: List[Dict[str, Any]] = []
        for i in range(n_wp):
            t_ms = t_base_ms + i * self._period_ms
            t_s = t_ms / 1000.0
            if self._type == "sinusoid":
                angle = self._offset + self._amplitude * math.sin(
                    2 * math.pi * self._freq * t_s
                )
            else:
                angle = self._offset  # fallback: constant

            angles_deg = [round(angle, 2)] * self._n_dof
            chunk.append({
                "angles_deg": angles_deg,
                "t_offset_ms": max(1, int(t_ms)),
            })
        return chunk


# ---------------------------------------------------------------------------
# Per-Joint Backpressure
# ---------------------------------------------------------------------------
class _JointBackpressure:
    """Track inflight batches and simulated buffer depth for one joint."""

    def __init__(self, buffer_depth: int, max_inflight: int):
        self._buffer_depth = buffer_depth
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
# Stream Metrics
# ---------------------------------------------------------------------------
class StreamMetrics:
    """Aggregate metrics for a streaming session (spec section 6.4)."""

    def __init__(self, target_rate_hz: float, joints: List[str]):
        self._target_rate = target_rate_hz
        self._joints = joints
        self._lock = threading.Lock()

        # Counters
        self._chunks_sent: int = 0
        self._chunks_dropped: int = 0
        self._chunks_deferred: int = 0
        self._waypoints_sent: int = 0
        self._waypoints_partial: int = 0
        self._sync_refresh_count: int = 0
        self._last_error: Optional[str] = None

        # Drift tracking
        self._drift_samples: deque = deque(maxlen=1000)

        # Per-joint queue fill max
        self._queue_fill_max: Dict[str, int] = {j: 0 for j in joints}

        # HTTP status counts (from WaypointClient metrics)
        self._http_status_counts: Dict[int, int] = {}
        self._retries: int = 0

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

    def merge_client_metrics(self, client_metrics: dict) -> None:
        """Pull HTTP status counts and retries from WaypointClient."""
        with self._lock:
            self._http_status_counts = client_metrics.get("status_counts", {})
            self._retries = client_metrics.get("retries", 0)

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

            # late ratio from WaypointClient status counts
            total_wp = self._waypoints_sent
            late_count = self._http_status_counts.get(207, 0)  # partials as proxy
            late_ratio = late_count / max(1, self._chunks_sent)

            return {
                "target_rate_hz": self._target_rate,
                "actual_rate_hz": round(actual_rate, 1),
                "scheduler_drift_ms_p95": round(p95, 2),
                "chunks_sent": self._chunks_sent,
                "chunks_dropped": self._chunks_dropped,
                "chunks_deferred": self._chunks_deferred,
                "waypoints_sent": self._waypoints_sent,
                "waypoints_partial": self._waypoints_partial,
                "http_status_counts": dict(self._http_status_counts),
                "retries": self._retries,
                "queue_fill_max": dict(self._queue_fill_max),
                "late_ratio": round(late_ratio, 4),
                "sync_refresh_count": self._sync_refresh_count,
                "last_error": self._last_error,
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

        joints = config["joints"]
        rate_hz = config["rate_hz"]
        horizon_ms = config.get("horizon_ms", 250)
        buffer_depth = config.get("buffer_depth_sim", 2)
        max_inflight = config.get("max_inflight_per_joint", 1)
        traj_config = config.get("trajectory", {
            "type": "sinusoid",
            "amplitude_deg": 8.0,
            "offset_deg": 0.0,
            "frequency_hz": 0.5,
        })
        fault_profile = config.get("fault_profile", {"mode": "none"})

        # Per-joint components
        self._trajectory: Dict[str, TrajectoryGenerator] = {}
        self._backpressure: Dict[str, _JointBackpressure] = {}
        for joint in joints:
            self._trajectory[joint] = TrajectoryGenerator(
                traj_config, rate_hz, horizon_ms, n_dof=1,
            )
            self._backpressure[joint] = _JointBackpressure(
                buffer_depth, max_inflight,
            )

        self._fault = _FaultInjector(fault_profile)
        self.metrics = StreamMetrics(rate_hz, joints)
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
        """Fixed-cadence publish loop with deadline tracking."""
        config = self._config
        rate_hz = config["rate_hz"]
        joints = config["joints"]
        duration_s = config.get("duration_s", 60)

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
            max_queue_size=config.get("buffer_depth_sim", 2),
        )
        client = WaypointClient(transport, config=client_config)

        try:
            # Initial time sync
            if not self._fault.suppress_sync:
                sync_ok = transport.send_time_sync()
                if sync_ok:
                    self.events.emit("INFO", "time_sync", detail="initial sync sent")
                else:
                    self.events.emit("WARN", "time_sync_fail", detail="initial sync failed")

            self.state = SessionState.RUNNING
            period_s = 1.0 / rate_hz
            next_tick = time.monotonic()
            tick = 0
            sync_interval_ticks = int(30 * rate_hz)
            metrics_interval_ticks = int(rate_hz)  # every 1s
            max_ticks = int(duration_s * rate_hz)
            sync_fail_start: Optional[float] = None

            while not self._stop_event.is_set() and tick < max_ticks:
                now = time.monotonic()
                drift = (now - next_tick) * 1000  # ms
                self.metrics.record_drift(drift)

                for joint in joints:
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
                        continue

                    # Fault injection
                    action = self._fault.pre_send(tick)
                    if action == "drop":
                        self.metrics.inc_dropped(joint)
                        self.events.emit("WARN", "fault_drop", joint)
                        continue
                    elif action == "delay":
                        delay_ms = self._fault.delay_ms
                        time.sleep(delay_ms / 1000)
                    elif action == "pause":
                        pause_ms = self._fault.pause_ms
                        self.events.emit(
                            "WARN", "fault_pause", joint,
                            f"burst pause {pause_ms}ms",
                        )
                        self._stop_event.wait(timeout=pause_ms / 1000)
                        if self._stop_event.is_set():
                            break

                    # Generate chunk
                    t_base_ms = tick * period_s * 1000
                    chunk = self._trajectory[joint].next_chunk(t_base_ms)

                    # Send via WaypointClient
                    try:
                        batch_id = client.enqueue_batch(joint, chunk)
                        bp.on_send()
                        self.metrics.inc_sent(joint, len(chunk))
                        self._pending_batches[batch_id] = joint

                        # Conflict spike: send duplicate
                        if self._fault.conflict_spike and tick % 10 == 0:
                            try:
                                client.enqueue_batch(joint, chunk)
                            except Exception:
                                pass  # expected 409
                    except Exception as exc:
                        self.metrics.set_error(str(exc))
                        self.events.emit(
                            "ERROR", "enqueue_failed", joint, str(exc),
                        )

                # Poll completed batches
                self._poll_completions(client)

                # Update queue fill metrics
                for joint in joints:
                    bp = self._backpressure[joint]
                    self.metrics.update_queue_fill(joint, bp.fill)

                # Periodic time sync (every 30s)
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
                if record.state == BatchState.PARTIAL:
                    result = getattr(record, "result", None) or {}
                    self.metrics.inc_partial(0)
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
            config: Session configuration matching spec section 6.1.

        Returns:
            ``{"session_id": str, "state": str}``

        Raises:
            ValueError: If config is invalid.
            RuntimeError: If a session is already running (409).
        """
        # Validate required fields
        joints = config.get("joints")
        if not joints or not isinstance(joints, list):
            raise ValueError("'joints' must be a non-empty list")
        rate_hz = config.get("rate_hz", 50)
        if rate_hz not in (50, 100):
            raise ValueError("'rate_hz' must be 50 or 100")

        with self._lock:
            if self._session and self._session.state in (
                SessionState.STARTING, SessionState.RUNNING,
            ):
                raise RuntimeError("Session already active")

            session_id = self._make_session_id()
            session = _StreamSession(
                session_id=session_id,
                config={
                    "joints": [j.upper() for j in joints],
                    "rate_hz": rate_hz,
                    "duration_s": config.get("duration_s", 60),
                    "horizon_ms": config.get("horizon_ms", 250),
                    "buffer_depth_sim": config.get("buffer_depth_sim", 2),
                    "max_inflight_per_joint": config.get("max_inflight_per_joint", 1),
                    "trajectory": config.get("trajectory", {
                        "type": "sinusoid",
                        "amplitude_deg": 8.0,
                        "offset_deg": 0.0,
                        "frequency_hz": 0.5,
                    }),
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
