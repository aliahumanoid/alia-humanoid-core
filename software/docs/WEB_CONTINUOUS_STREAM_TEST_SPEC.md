# Web Continuous Waypoint Streaming - Technical Specification

**Version**: 1.0  
**Date**: 2026-02-13  
**Status**: Proposed (ready to implement)

---

## 1. Purpose

Define a robust, repeatable, web-driven test mode that is as close as possible to the production architecture:

- Production target: `Jetson → SPI → MCP2515 (CAN Expansion Board) → CAN → Joint controllers`
- Current bench path: `Web UI → Flask → CanManager → CAN → Joint controllers`
- Deferred gateway path: `Jetson → USB CDC → RP2350 Gateway → CAN → Joint controllers` (see CAN_SYSTEM_ARCHITECTURE.md §7.3)

This spec introduces a **continuous streaming test service** that emulates gateway-like constraints and backpressure while running in the current Flask host environment.

---

## 2. Scope

### In scope

- Continuous waypoint streaming at 50-100 Hz.
- Per-joint flow control and backpressure simulation.
- Live metrics and fault injection from web page.
- Deterministic test scenarios with pass/fail criteria.

### Out of scope

- Final real-time guarantees of RP2350 gateway (for example jitter +/-50 us).
- Firmware changes on joint controllers.
- Final USB CDC transport implementation (defined as migration target only).

---

## 3. System Under Test (SUT)

### 3.1 Control plane

- Web UI starts/stops a streaming session.
- Web UI reads live status, metrics, and events.

### 3.2 Data plane (bench)

- `StreamTestService` (server-side Python loop) generates waypoints continuously.
- Service sends waypoints through existing waypoint pipeline:
  - `WaypointClient` + `HttpTransport` -> `/can/waypoint_batch`
- Backend route and `CanManager` keep existing safety checks.

### 3.3 Gateway emulation layer

Server-side logic must emulate production constraints:

- limited in-flight work per joint (`max_inflight_per_joint`, default 1),
- fixed cadence publish loop (50/100 Hz),
- optional fault profiles (delay, drop, burst, sync issues).

---

## 4. Functional Requirements

### FR-1 Session lifecycle

Provide API to:

- start a stream session,
- stop session gracefully,
- query status,
- query metrics,
- read event log.

### FR-2 Continuous publish loop

For each active joint:

- generate target waypoints every `period_ms = 1000/rate_hz`,
- maintain monotonic `t_offset_ms`,
- publish in short rolling horizon windows (`horizon_ms`, default 250 ms).

### FR-3 Per-joint backpressure

- enforce per-joint queue cap (`max_queue_size`, default 2),
- reject or defer new chunks when queue is full,
- count and expose dropped/deferred chunks.

### FR-4 Safety gating

Before sending:

- require CAN connected,
- require valid and fresh time sync,
- enforce limits already validated by backend (`waypoint_types`).

### FR-5 Fault injection

Support runtime-selectable profiles:

- `none`,
- `delay_jitter` (random extra delay),
- `drop_rate` (probabilistic chunk drop),
- `burst_pause` (periodic pause),
- `sync_stale` (disable/slow sync refresh),
- `conflict_spike` (parallel same-joint submissions to provoke 409).

### FR-6 Observability

Expose live metrics at 1 Hz and event stream for:

- session transitions,
- warnings/errors,
- retry and HTTP status distribution,
- backpressure and drop counters.

---

## 5. Non-Functional Requirements

### NFR-1 Stability

- no deadlocks,
- no stuck `IDLE` batches in client history,
- clean stop within 2 s.

### NFR-2 Determinism (bench)

- scheduler drift monitor enabled,
- measured publish cadence error must be reported.

### NFR-3 Reproducibility

- every session stores full config snapshot,
- deterministic seed optional for fault profiles.

---

## 6. API Contract (Flask)

Base path: `/stream_test`

### 6.1 `POST /stream_test/start`

Request:

```json
{
  "joints": ["KNEE_LEFT"],
  "rate_hz": 100,
  "duration_s": 120,
  "horizon_ms": 250,
  "max_inflight_per_joint": 1,
  "trajectory": {
    "type": "sinusoid",
    "amplitude_deg": 8.0,
    "offset_deg": 0.0,
    "frequency_hz": 0.5
  },
  "fault_profile": {
    "mode": "none",
    "seed": 42
  }
}
```

Response:

```json
{
  "status": "success",
  "session_id": "st_20260213_205501_ab12",
  "state": "RUNNING"
}
```

### 6.2 `POST /stream_test/stop`

Request:

```json
{
  "session_id": "st_20260213_205501_ab12",
  "reason": "operator_stop"
}
```

Response:

```json
{
  "status": "success",
  "session_id": "st_20260213_205501_ab12",
  "state": "STOPPED"
}
```

### 6.3 `GET /stream_test/status`

Query:

- `session_id=<id>` optional; default current session.

Response:

```json
{
  "status": "success",
  "session": {
    "session_id": "st_20260213_205501_ab12",
    "state": "RUNNING",
    "started_at": 1739476501.12,
    "uptime_s": 37.4,
    "config": { "...": "..." }
  }
}
```

### 6.4 `GET /stream_test/metrics`

Response:

```json
{
  "status": "success",
  "metrics": {
    "target_rate_hz": 100,
    "actual_rate_hz": 99.4,
    "scheduler_drift_ms_p95": 1.7,
    "chunks_sent": 3720,
    "chunks_confirmed": 3706,
    "chunks_failed": 0,
    "chunks_dropped": 0,
    "chunks_deferred": 3,
    "waypoints_sent": 7440,
    "waypoints_partial": 0,
    "http_status_counts": {"200": 3698, "207": 8, "409": 12, "502": 2},
    "retries": 14,
    "queue_fill_max": {"KNEE_LEFT": 2},
    "max_inflight_per_joint": 1,
    "partial_ratio": 0.0022,
    "waypoints_late": 0,
    "late_ratio": 0.0,
    "sync_refresh_count": 2,
    "last_error": null,
    "fw_wp_accepted": 7440,
    "fw_wp_dropped": 0,
    "fw_buffer_fill": 12
  }
}
```

### 6.5 `GET /stream_test/events`

Query:

- `session_id=<id>`
- `after_seq=<int>` optional incremental cursor.

Response:

```json
{
  "status": "success",
  "events": [
    {
      "seq": 120,
      "ts": 1739476537.88,
      "level": "WARN",
      "event": "backpressure_defer",
      "joint": "KNEE_LEFT",
      "detail": "queue full, defer one chunk"
    }
  ]
}
```

---

## 7. Session State Machine

States:

- `IDLE`
- `STARTING`
- `RUNNING`
- `STOPPING`
- `STOPPED`
- `FAILED`

Transitions:

- `IDLE -> STARTING -> RUNNING`
- `RUNNING -> STOPPING -> STOPPED`
- any state -> `FAILED` on unrecoverable error.

Rules:

- only one active session at a time (v1),
- second `start` while running returns 409,
- `stop` is idempotent.

---

## 8. Scheduler and Backpressure Model

### 8.1 Publish cadence

- use monotonic clock (`time.monotonic`),
- fixed-step loop with deadline tracking,
- each tick computes per-joint target angle and small future window.

### 8.2 Chunk shape

- send short chunks only (2-4 waypoints),
- maintain rolling horizon `horizon_ms` (default 250 ms),
- each chunk has strictly increasing `t_offset_ms`.

### 8.3 Backpressure policy

Per joint, gating is based solely on `max_inflight_per_joint`:

- if in-flight >= `max_inflight_per_joint`: defer current chunk,
- default policy:
  - first overload: defer,
  - repeated overload (>3 ticks): drop oldest unsent chunk.

All defer/drop actions must emit event and increment counters.

### 8.4 Firmware telemetry (on-demand)

During a stream test, the host polls firmware waypoint buffer telemetry
every ~1s via CAN request/response (0x01C → 0x4D0+joint).  The response
contains `wp_accepted`, `wp_dropped_full`, `wp_dropped_guard`, and
`buffer_fill`.  These are exposed in the metrics snapshot as
`fw_wp_accepted`, `fw_wp_dropped`, `fw_buffer_fill`.

---

## 9. Time Sync Policy (bench approximation)

- send `/can/time_sync` at session start,
- refresh every 30 s during run,
- immediate refresh after first 502/503 in a 5 s window,
- if refresh fails continuously for >5 s: move session to `FAILED`.

---

## 10. Web UI Specification

Add panel: **Continuous Stream Test**

Controls:

- joint set selector,
- rate (`50` or `100` Hz),
- duration (`30s`, `120s`, `600s`, custom),
- horizon ms,
- fault profile selector,
- start/stop buttons.

Live indicators:

- state badge,
- uptime,
- target/actual Hz,
- p95 drift,
- retry count,
- 409/502/503 counters,
- queue fill gauge per joint,
- pass/fail badge against active scenario thresholds.

Data source:

- poll `status` and `metrics` each 1 s,
- consume `events` via polling (v1) or Socket.IO (optional v2).

---

## 11. Test Scenarios and Acceptance Criteria

### S0 Smoke

- 1 joint, 50 Hz, 60 s, no fault.
- Pass:
  - no crash,
  - no stuck session,
  - no dropped chunks.

### S1 Nominal 50 Hz

- 1 joint, 50 Hz, 10 min.
- Pass:
  - `actual_rate_hz >= 49.0`,
  - `chunks_dropped = 0`,
  - `partial_ratio <= 0.1%` (HTTP 207 count / chunks sent),
  - `late_ratio <= 0.1%` (late waypoints / waypoints sent),
  - `chunks_failed = 0`.

### S2 Nominal 100 Hz

- 1 joint, 100 Hz, 10 min.
- Pass:
  - `actual_rate_hz >= 98.0`,
  - `chunks_dropped = 0`,
  - `partial_ratio <= 0.3%` (HTTP 207 count / chunks sent),
  - `late_ratio <= 0.3%` (late waypoints / waypoints sent),
  - `chunks_failed = 0`.

### S3 Multi-joint

- 2 joints, 50 Hz each, 10 min.
- Pass:
  - no starvation across joints,
  - each joint `actual_rate_hz >= 49.0`,
  - no deadlock.

### S4 Fault resilience

- 1 joint, 100 Hz, 5 min, `delay_jitter` + `drop_rate=1%`.
- Pass:
  - service remains `RUNNING`,
  - retry behavior bounded,
  - recovery without manual restart.

### S5 Safety/stop behavior

- run nominal stream, trigger E-stop at random t.
- Pass:
  - stop command effective immediately,
  - stream transitions to `STOPPED` or `FAILED` with explicit reason,
  - restart possible after operator reset.

---

## 12. Go/No-Go for Real Robot Trial

Go if all true:

- S0..S3 pass,
- no P1 defects open,
- S4 and S5 pass in 2 consecutive runs,
- metrics/log export attached to run report.

No-Go if any:

- deadlock/stuck queue observed,
- unbounded retry storm,
- repeated partial/failed sends without recovery,
- missing stop determinism.

---

## 13. Implementation Mapping

Planned files:

- `software/host/stream_test_service.py` (new)
- `software/host/routes.py` (add `/stream_test/*`)
- `software/host/static/js/scripts.js` (new UI panel logic)
- `software/host/templates/index.html` (panel markup)
- `software/host/tests/test_stream_test_service.py` (new)
- `software/host/tests/test_stream_test_routes.py` (new)

Leverage existing modules:

- `software/host/waypoint_client.py`
- `software/host/waypoint_types.py`
- `software/host/can_manager.py`

---

## 14. Migration Path to Production

When gateway firmware is ready:

1. Keep `StreamTestService` logic and metrics unchanged.
2. Replace transport:
   - from `HttpTransport`
   - to `UsbCdcTransport` (new implementation of `WaypointTransport`).
3. Disable Flask-specific assumptions and keep same scenario suite.

Result:

- control/observability plane remains stable,
- data plane moves from HTTP bench path to production USB CDC path with minimal refactor.

---

## 15. Deliverables

For each test run store:

- session config JSON,
- metrics JSON snapshot every 1 s,
- event log JSONL,
- final pass/fail report (scenario + thresholds).

Suggested output directory:

- `software/host/logs/stream_test/<session_id>/`
