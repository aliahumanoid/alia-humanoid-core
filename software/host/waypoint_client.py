"""
Jetson-ready waypoint client for the Alia host waypoint pipeline.

Standalone module — no Flask, CanManager, or web UI dependencies.
Requires only: requests (for HttpTransport), threading, queue, logging, uuid,
time, enum, dataclasses.

The ``requests`` library is imported lazily inside HttpTransport methods so that
``import waypoint_client`` succeeds even without ``requests`` installed.

Usage::

    from waypoint_client import WaypointClient, HttpTransport

    client = WaypointClient(HttpTransport("http://10.0.0.1:5001"))
    batch_id = client.enqueue_batch("KNEE_LEFT", [
        {"angles_deg": [10.0], "t_offset_ms": 100},
        {"angles_deg": [20.0], "t_offset_ms": 200},
    ])
    # ... later ...
    record = client.get_batch_status(batch_id)
    client.shutdown()

See JETSON_WAYPOINT_API_CONTRACT.md for the full HTTP contract.
"""
from __future__ import annotations

import enum
import json
import logging
import queue
import random
import threading
import time
import uuid
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Protocol, runtime_checkable


logger = logging.getLogger("waypoint_client")


# ---------------------------------------------------------------------------
# Enums & Data Types
# ---------------------------------------------------------------------------
class BatchState(enum.Enum):
    """Per-joint batch lifecycle states."""
    IDLE = "IDLE"
    SENDING = "SENDING"
    PARTIAL = "PARTIAL"
    COMPLETE = "COMPLETE"
    RETRY_WAIT = "RETRY_WAIT"
    FAILED_HARD = "FAILED_HARD"


@dataclass(frozen=True)
class BatchResponse:
    """Normalised response from transport layer.

    ``http_status == 0`` is a sentinel for timeout / connection error.
    """
    http_status: int
    status: str                                         # "success" | "partial" | "error"
    message: str
    result: Optional[Dict[str, Any]] = None             # The 'result' object (200/207/502)
    validation_errors: Optional[List[Dict]] = None      # From 400 responses
    raw_body: Optional[Dict[str, Any]] = None           # Full body for debugging


@dataclass(frozen=True)
class HealthStatus:
    """Result of a backend health check."""
    can_connected: bool
    time_sync_age_ms: Optional[float]       # None = never synced
    server_reachable: bool
    detail: str = ""


@dataclass
class BatchRecord:
    """Tracking record for a submitted batch."""
    client_batch_id: str
    joint: str
    waypoint_count: int
    state: BatchState
    server_batch_id: Optional[str] = None
    attempts: int = 0
    created_at: float = field(default_factory=time.monotonic)
    completed_at: Optional[float] = None
    last_error: Optional[str] = None
    http_status: Optional[int] = None


@dataclass
class RetryPolicy:
    """Configurable retry parameters per HTTP status code."""
    max_retries_409: int = 3
    max_retries_502_503: int = 5
    max_retries_500: int = 2
    max_retries_timeout: int = 5
    base_backoff_409_s: float = 0.1         # 100 ms
    base_backoff_502_s: float = 0.5         # 500 ms
    base_backoff_500_s: float = 1.0         # 1 s
    base_backoff_timeout_s: float = 0.5
    jitter_factor: float = 0.3              # +/- 30 %


@dataclass
class ClientConfig:
    """Top-level client configuration."""
    base_url: str = "http://localhost:5001"
    retry: RetryPolicy = field(default_factory=RetryPolicy)
    max_queue_size: int = 100               # Per-joint queue depth
    auto_time_sync: bool = True             # Re-sync on 502/503 before retry


# ---------------------------------------------------------------------------
# Transport Protocol
# ---------------------------------------------------------------------------
@runtime_checkable
class WaypointTransport(Protocol):
    """Abstract transport for sending waypoint batches.

    Implementations:
    - HttpTransport: HTTP POST to Flask backend (production)
    - MockTransport: In-memory for testing
    """

    def send_batch(
        self,
        joint: str,
        waypoints: List[Dict[str, Any]],
        batch_id: str,
    ) -> BatchResponse:
        """Send a waypoint batch and return a normalised response."""
        ...

    def send_time_sync(self) -> bool:
        """Send time sync to the backend.  Returns True on success."""
        ...

    def check_health(self) -> HealthStatus:
        """Check backend health (CAN connected, sync fresh, reachable)."""
        ...


# ---------------------------------------------------------------------------
# HttpTransport
# ---------------------------------------------------------------------------
class HttpTransport:
    """HTTP transport calling the Flask backend via ``requests``."""

    def __init__(self, base_url: str, timeout_s: float = 30.0):
        self._base_url = base_url.rstrip("/")
        self._timeout = timeout_s
        self._session: Optional[Any] = None     # lazy requests.Session

    # -- Internal helpers --------------------------------------------------

    def _get_session(self):
        """Lazy-initialise ``requests.Session`` (avoids top-level import)."""
        if self._session is None:
            import requests                     # noqa: local import
            self._session = requests.Session()
            self._session.headers.update({"Content-Type": "application/json"})
        return self._session

    # -- WaypointTransport interface ---------------------------------------

    def send_batch(
        self,
        joint: str,
        waypoints: List[Dict[str, Any]],
        batch_id: str,
    ) -> BatchResponse:
        import requests                         # noqa: local import

        url = f"{self._base_url}/can/waypoint_batch"
        payload = {"joint": joint, "waypoints": waypoints}

        try:
            resp = self._get_session().post(url, json=payload, timeout=self._timeout)
            body = resp.json()
            return BatchResponse(
                http_status=resp.status_code,
                status=body.get("status", "error"),
                message=body.get("message", ""),
                result=body.get("result"),
                validation_errors=body.get("validation_errors"),
                raw_body=body,
            )
        except requests.Timeout:
            return BatchResponse(
                http_status=0,
                status="error",
                message=f"Request timeout after {self._timeout}s",
            )
        except requests.ConnectionError as exc:
            return BatchResponse(
                http_status=0,
                status="error",
                message=f"Connection error: {exc}",
            )

    def send_time_sync(self) -> bool:
        import requests                         # noqa: local import
        try:
            resp = self._get_session().post(
                f"{self._base_url}/can/time_sync",
                json={},
                timeout=5.0,
            )
            return resp.status_code == 200
        except (requests.Timeout, requests.ConnectionError):
            return False

    def check_health(self) -> HealthStatus:
        import requests                         # noqa: local import
        try:
            resp = self._get_session().get(
                f"{self._base_url}/can/status",
                timeout=5.0,
            )
            if resp.status_code != 200:
                return HealthStatus(
                    can_connected=False,
                    time_sync_age_ms=None,
                    server_reachable=True,
                    detail=f"HTTP {resp.status_code}",
                )
            body = resp.json()
            state = body.get("state", {})
            return HealthStatus(
                can_connected=state.get("connected", False),
                time_sync_age_ms=state.get("time_sync_age_ms"),
                server_reachable=True,
            )
        except (requests.Timeout, requests.ConnectionError) as exc:
            return HealthStatus(
                can_connected=False,
                time_sync_age_ms=None,
                server_reachable=False,
                detail=str(exc),
            )

    def close(self):
        if self._session:
            self._session.close()
            self._session = None


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------
class _Metrics:
    """Simple thread-safe metrics collector (no external deps)."""

    def __init__(self):
        self._lock = threading.Lock()
        self.batches_total: int = 0
        self.batches_partial: int = 0
        self.batches_failed: int = 0
        self.retries: int = 0
        self._send_times_ms: List[float] = []
        self._status_counts: Dict[int, int] = {}

    def record_send(self, resp: BatchResponse) -> None:
        with self._lock:
            code = resp.http_status
            self._status_counts[code] = self._status_counts.get(code, 0) + 1
            if resp.result and "elapsed_ms" in resp.result:
                self._send_times_ms.append(resp.result["elapsed_ms"])
                if len(self._send_times_ms) > 1000:
                    self._send_times_ms = self._send_times_ms[-500:]

    def inc_total(self) -> None:
        with self._lock:
            self.batches_total += 1

    def inc_partial(self) -> None:
        with self._lock:
            self.batches_partial += 1

    def inc_failed(self) -> None:
        with self._lock:
            self.batches_failed += 1

    def inc_retries(self) -> None:
        with self._lock:
            self.retries += 1

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            total_sends = sum(self._status_counts.values())
            mean_ms = (
                sum(self._send_times_ms) / len(self._send_times_ms)
                if self._send_times_ms else 0.0
            )
            return {
                "batches_total": self.batches_total,
                "batches_partial": self.batches_partial,
                "batches_failed": self.batches_failed,
                "retries": self.retries,
                "mean_send_ms": round(mean_ms, 1),
                "status_counts": dict(self._status_counts),
                "rate_409": (
                    self._status_counts.get(409, 0) / total_sends
                    if total_sends > 0 else 0.0
                ),
                "rate_502": (
                    self._status_counts.get(502, 0) / total_sends
                    if total_sends > 0 else 0.0
                ),
            }


# ---------------------------------------------------------------------------
# Per-Joint Worker
# ---------------------------------------------------------------------------
class _JointWorker:
    """Dedicated worker thread for a single joint.

    Owns a ``queue.Queue`` of ``(BatchRecord, waypoints)`` tuples and a daemon
    worker thread that is lazy-started on the first ``enqueue()`` call.
    """

    def __init__(
        self,
        joint: str,
        transport: WaypointTransport,
        config: ClientConfig,
        metrics: _Metrics,
        on_batch_done: Callable[[BatchRecord], None],
    ):
        self._joint = joint
        self._transport = transport
        self._config = config
        self._metrics = metrics
        self._on_batch_done = on_batch_done
        self._queue: queue.Queue = queue.Queue(maxsize=config.max_queue_size)
        self._cancel_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._startup_lock = threading.Lock()
        self._logger = logging.getLogger(f"waypoint_client.{joint}")

    # -- Public API --------------------------------------------------------

    def enqueue(self, record: BatchRecord, waypoints: List[Dict]) -> None:
        """Add a batch to the joint queue.  Starts worker if needed."""
        self._ensure_started()
        self._queue.put((record, waypoints), timeout=5.0)

    def cancel(self) -> None:
        """Signal worker to stop and drain pending batches."""
        self._cancel_event.set()
        while not self._queue.empty():
            try:
                record, _ = self._queue.get_nowait()
                record.state = BatchState.FAILED_HARD
                record.last_error = "Cancelled"
                record.completed_at = time.monotonic()
                self._on_batch_done(record)
            except queue.Empty:
                break

    def shutdown(self, timeout_s: float = 5.0) -> None:
        """Signal stop and join the worker thread."""
        self._cancel_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=timeout_s)

    # -- State machine -----------------------------------------------------

    def _classify_response(self, resp: BatchResponse, attempt: int) -> BatchState:
        """Map HTTP status code + attempt count to next state.

        Pure integer comparison — never string matching on message.
        """
        code = resp.http_status
        policy = self._config.retry

        if code == 200:
            return BatchState.COMPLETE
        elif code == 207:
            return BatchState.PARTIAL
        elif code == 400:
            return BatchState.FAILED_HARD
        elif code == 409:
            if attempt < policy.max_retries_409:
                return BatchState.RETRY_WAIT
            return BatchState.FAILED_HARD
        elif code in (502, 503):
            if attempt < policy.max_retries_502_503:
                return BatchState.RETRY_WAIT
            return BatchState.FAILED_HARD
        elif code == 500:
            if attempt < policy.max_retries_500:
                return BatchState.RETRY_WAIT
            return BatchState.FAILED_HARD
        elif code == 0:     # timeout / connection error
            if attempt < policy.max_retries_timeout:
                return BatchState.RETRY_WAIT
            return BatchState.FAILED_HARD
        else:
            return BatchState.FAILED_HARD

    def _backoff_seconds(self, code: int, attempt: int) -> float:
        """Progressive exponential backoff with jitter.

        Formula: base * 2^attempt * (1 +/- jitter_factor * random)
        """
        policy = self._config.retry

        if code == 409:
            base = policy.base_backoff_409_s
        elif code in (502, 503):
            base = policy.base_backoff_502_s
        elif code == 500:
            base = policy.base_backoff_500_s
        else:
            base = policy.base_backoff_timeout_s

        delay = base * (2 ** attempt)
        jitter = delay * policy.jitter_factor * (2 * random.random() - 1)
        return max(0.01, delay + jitter)

    # -- Internal ----------------------------------------------------------

    def _ensure_started(self) -> None:
        with self._startup_lock:
            need_new = (self._thread is None or not self._thread.is_alive())
            # If cancel was requested but old thread is still winding down,
            # wait for it so the new thread starts with a clean cancel_event.
            if not need_new and self._cancel_event.is_set():
                self._thread.join(timeout=2.0)
                # Only spawn new thread if old one actually exited;
                # otherwise keep the old thread to preserve serialisation.
                need_new = not self._thread.is_alive()
            if need_new:
                self._cancel_event.clear()
                self._thread = threading.Thread(
                    target=self._run,
                    name=f"wp-worker-{self._joint}",
                    daemon=True,
                )
                self._thread.start()

    def _run(self) -> None:
        """Worker loop: pull batches from queue, send with retry."""
        while not self._cancel_event.is_set():
            try:
                record, waypoints = self._queue.get(timeout=1.0)
            except queue.Empty:
                continue

            try:
                self._process_batch(record, waypoints)
            except Exception as exc:
                self._logger.exception(
                    "Unhandled error processing batch %s", record.client_batch_id
                )
                record.state = BatchState.FAILED_HARD
                record.last_error = str(exc)
            finally:
                record.completed_at = time.monotonic()
                self._on_batch_done(record)
                self._queue.task_done()

    def _process_batch(
        self, record: BatchRecord, waypoints: List[Dict]
    ) -> None:
        """Send a single batch with retry loop.  Updates *record* in-place."""
        record.state = BatchState.SENDING
        attempt = 0

        while not self._cancel_event.is_set():
            record.attempts = attempt + 1
            self._log_send(record, attempt)

            resp = self._transport.send_batch(
                record.joint, waypoints, record.client_batch_id
            )
            record.http_status = resp.http_status
            self._metrics.record_send(resp)

            next_state = self._classify_response(resp, attempt)
            record.state = next_state

            if next_state == BatchState.COMPLETE:
                record.server_batch_id = (resp.result or {}).get("batch_id")
                self._log_complete(record, resp)
                return

            elif next_state == BatchState.PARTIAL:
                record.server_batch_id = (resp.result or {}).get("batch_id")
                self._log_partial(record, resp)
                return

            elif next_state == BatchState.FAILED_HARD:
                record.last_error = resp.message
                self._log_failed(record, resp)
                return

            elif next_state == BatchState.RETRY_WAIT:
                # Re-sync time for transient CAN errors
                if resp.http_status in (502, 503) and self._config.auto_time_sync:
                    self._transport.send_time_sync()

                backoff = self._backoff_seconds(resp.http_status, attempt)
                self._log_retry(record, resp, attempt, backoff)
                self._metrics.inc_retries()

                # Interruptible sleep (returns True if cancel was set)
                if self._cancel_event.wait(timeout=backoff):
                    record.state = BatchState.FAILED_HARD
                    record.last_error = "Cancelled during retry wait"
                    return

                attempt += 1

    # -- Structured logging ------------------------------------------------

    def _log_send(self, record: BatchRecord, attempt: int) -> None:
        self._logger.info(json.dumps({
            "event": "batch_send",
            "client_batch_id": record.client_batch_id,
            "joint": record.joint,
            "waypoint_count": record.waypoint_count,
            "attempt": attempt + 1,
        }))

    def _log_complete(self, record: BatchRecord, resp: BatchResponse) -> None:
        self._logger.info(json.dumps({
            "event": "batch_complete",
            "client_batch_id": record.client_batch_id,
            "server_batch_id": record.server_batch_id,
            "joint": record.joint,
            "http_status": resp.http_status,
            "sent": (resp.result or {}).get("sent"),
            "total": (resp.result or {}).get("total"),
        }))

    def _log_partial(self, record: BatchRecord, resp: BatchResponse) -> None:
        self._logger.warning(json.dumps({
            "event": "batch_partial",
            "client_batch_id": record.client_batch_id,
            "joint": record.joint,
            "http_status": 207,
            "sent": (resp.result or {}).get("sent"),
            "total": (resp.result or {}).get("total"),
            "failed_indices": (resp.result or {}).get("failed_indices"),
        }))

    def _log_failed(self, record: BatchRecord, resp: BatchResponse) -> None:
        self._logger.error(json.dumps({
            "event": "batch_failed",
            "client_batch_id": record.client_batch_id,
            "joint": record.joint,
            "http_status": resp.http_status,
            "message": resp.message,
            "validation_errors": resp.validation_errors,
        }))

    def _log_retry(
        self, record: BatchRecord, resp: BatchResponse, attempt: int, backoff: float
    ) -> None:
        self._logger.warning(json.dumps({
            "event": "batch_retry",
            "client_batch_id": record.client_batch_id,
            "joint": record.joint,
            "http_status": resp.http_status,
            "attempt": attempt + 1,
            "backoff_s": round(backoff, 3),
        }))


# ---------------------------------------------------------------------------
# WaypointClient  (top-level facade)
# ---------------------------------------------------------------------------
class WaypointClient:
    """Jetson-side waypoint client with per-joint queues and retry.

    Thread-safe.  One instance per application.

    Usage::

        transport = HttpTransport("http://10.0.0.1:5001")
        client = WaypointClient(transport)

        client.enqueue_batch("KNEE_LEFT", [
            {"angles_deg": [10.0], "t_offset_ms": 100},
            {"angles_deg": [20.0], "t_offset_ms": 200},
        ])

        # Later ...
        client.shutdown()
    """

    def __init__(
        self,
        transport: WaypointTransport,
        config: Optional[ClientConfig] = None,
    ):
        self._transport = transport
        self._config = config or ClientConfig()
        self._metrics = _Metrics()
        self._workers: Dict[str, _JointWorker] = {}
        self._workers_lock = threading.Lock()
        self._batch_history: Dict[str, BatchRecord] = {}
        self._history_lock = threading.Lock()

    # -- Public API --------------------------------------------------------

    def enqueue_batch(
        self,
        joint: str,
        waypoints: List[Dict[str, Any]],
        batch_id: Optional[str] = None,
    ) -> str:
        """Enqueue a waypoint batch for async delivery.

        Args:
            joint: Joint name (e.g. ``"KNEE_LEFT"``).
            waypoints: List of ``{"angles_deg": [...], "t_offset_ms": int}``.
            batch_id: Optional client batch ID (auto-generated if None).

        Returns:
            client_batch_id (8-char UUID) for tracking.

        Raises:
            ValueError: If *joint* is empty.
            queue.Full: If the joint queue is at capacity.
        """
        if not joint:
            raise ValueError("joint must be non-empty")

        joint_upper = joint.upper()
        client_batch_id = batch_id or uuid.uuid4().hex[:8]

        record = BatchRecord(
            client_batch_id=client_batch_id,
            joint=joint_upper,
            waypoint_count=len(waypoints),
            state=BatchState.IDLE,
        )

        worker = self._get_or_create_worker(joint_upper)
        # put() first — if queue.Full, no side-effects in history/metrics
        worker.enqueue(record, waypoints)

        with self._history_lock:
            self._batch_history[client_batch_id] = record
        self._metrics.inc_total()

        return client_batch_id

    def cancel_joint(self, joint: str) -> None:
        """Cancel all pending batches for a joint."""
        joint_upper = joint.upper()
        with self._workers_lock:
            worker = self._workers.get(joint_upper)
        if worker:
            worker.cancel()

    def shutdown(self, timeout_s: float = 10.0) -> None:
        """Gracefully shut down all workers and close transport."""
        with self._workers_lock:
            workers = list(self._workers.values())
        per_worker = max(1.0, timeout_s / max(len(workers), 1))
        for w in workers:
            w.shutdown(timeout_s=per_worker)
        if hasattr(self._transport, "close"):
            self._transport.close()

    def get_batch_status(self, client_batch_id: str) -> Optional[BatchRecord]:
        """Query status of a submitted batch."""
        with self._history_lock:
            return self._batch_history.get(client_batch_id)

    def get_metrics(self) -> Dict[str, Any]:
        """Return current metrics snapshot."""
        return self._metrics.snapshot()

    # -- Internal ----------------------------------------------------------

    def _get_or_create_worker(self, joint: str) -> _JointWorker:
        with self._workers_lock:
            if joint not in self._workers:
                self._workers[joint] = _JointWorker(
                    joint=joint,
                    transport=self._transport,
                    config=self._config,
                    metrics=self._metrics,
                    on_batch_done=self._on_batch_done,
                )
            return self._workers[joint]

    def _on_batch_done(self, record: BatchRecord) -> None:
        """Callback from worker when batch reaches terminal state."""
        if record.state == BatchState.PARTIAL:
            self._metrics.inc_partial()
        elif record.state == BatchState.FAILED_HARD:
            self._metrics.inc_failed()

        logger.info(json.dumps({
            "event": "batch_done",
            "client_batch_id": record.client_batch_id,
            "joint": record.joint,
            "state": record.state.value,
            "attempts": record.attempts,
            "http_status": record.http_status,
            "elapsed_s": round(
                (record.completed_at or time.monotonic()) - record.created_at, 3
            ),
            "error": record.last_error,
        }))
