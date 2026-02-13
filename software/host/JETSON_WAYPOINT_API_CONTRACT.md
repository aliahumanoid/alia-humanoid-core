# Waypoint Batch API Contract v1.0

> **Owner:** alia-humanoid-core/software/host
> **Last updated:** 2026-02-13
> **Consumers:** Web UI (scripts.js), Jetson client (waypoint_client.py)

---

## Endpoint

```
POST /can/waypoint_batch
Content-Type: application/json
```

## Request Schema

```json
{
    "joint": "ANKLE_RIGHT",
    "waypoints": [
        {"angles_deg": [10.0, 5.0, null], "t_offset_ms": 100},
        {"angles_deg": [15.0, 7.0, null], "t_offset_ms": 200}
    ]
}
```

| Field | Type | Required | Notes |
|-------|------|----------|-------|
| `joint` | string | Yes | Case-insensitive, uppercased server-side |
| `waypoints` | array | Yes | Min 1, max 1800 entries |
| `waypoints[].angles_deg` | array[float\|null] | Yes | Up to 3 DOFs, null = unused DOF |
| `waypoints[].t_offset_ms` | int | Yes | Range [0, 65535], strictly increasing |

---

## Preflight Requirements

Before processing the batch, the server checks (in order):

1. **CAN bus connected** -- 503 if not
2. **Time sync fresh** -- 400 if never synced or age > 2000ms
3. **Batch valid** -- 400 if build fails (non-numeric, NaN/Inf)
4. **Validation** -- 400 if angle limits, velocity, monotonicity, batch size violated
5. **Deduplication** -- Removes zero-step quantization duplicates (non-fatal, logged)

---

## HTTP Status Codes

| Code | Condition | `status` field | Retry? |
|------|-----------|---------------|--------|
| **200** | All waypoints sent | `"success"` | No |
| **207** | Some waypoints sent | `"partial"` | No (proceed with warning) |
| **400** | Validation error, bad input, stale/missing sync | `"error"` | No (fix input) |
| **409** | Concurrent batch on same joint | `"error"` | Yes (short backoff + jitter) |
| **500** | Unexpected server exception | `"error"` | Yes (max 2 retries) |
| **502** | 0 waypoints sent (CAN layer failure) | `"error"` | Yes (backoff + re-sync) |
| **503** | CAN bus not connected / python-can missing | `"error"` | Yes (backoff + re-sync) |

---

## Response Schemas

### 200 -- Success

```json
{
    "status": "success",
    "message": "Batch abc12345: 100/100 waypoints sent to ANKLE_RIGHT",
    "result": {
        "total": 100,
        "sent": 100,
        "failed_indices": [],
        "skipped_indices": [],
        "elapsed_ms": 250.3,
        "timing_drift_ms": 1.2,
        "late_count": 0,
        "aborted": false,
        "joint": "ANKLE_RIGHT",
        "batch_id": "abc12345"
    }
}
```

### 207 -- Partial Success

```json
{
    "status": "partial",
    "message": "Batch abc12345: 85/100 waypoints sent to ANKLE_RIGHT",
    "result": {
        "total": 100,
        "sent": 85,
        "failed_indices": [42],
        "skipped_indices": [90, 91, 92, 93, 94, 95, 96, 97, 98, 99],
        "elapsed_ms": 220.1,
        "timing_drift_ms": 45.6,
        "late_count": 14,
        "aborted": true,
        "joint": "ANKLE_RIGHT",
        "batch_id": "abc12345"
    }
}
```

### 400 -- Validation Error (input)

```json
{
    "status": "error",
    "message": "Waypoint validation failed (2 error(s))",
    "validation_errors": [
        {"index": 0, "field": "angles_deg[0]", "message": "DOF0 angle 200.00 out of range [-50.0, 25.0]"},
        {"index": 3, "field": "t_offset_ms", "message": "t_offset_ms=400 is not strictly increasing (previous=500)"}
    ]
}
```

### 400 -- Missing/Stale Time Sync

```json
{
    "status": "error",
    "message": "Time sync too stale (2500.0ms > 2000ms). Send /can/time_sync first."
}
```

### 400 -- Build Error (non-numeric input)

```json
{
    "status": "error",
    "message": "Invalid waypoint data: Waypoint 0: angles_deg[0] = 'abc' is not numeric"
}
```

### 409 -- Concurrent Conflict

```json
{
    "status": "error",
    "message": "Waypoint batch already in progress for ANKLE_RIGHT"
}
```

### 502 -- Upstream Failure (0 sent)

```json
{
    "status": "error",
    "message": "Batch abc12345: 0/100 waypoints sent to ANKLE_RIGHT",
    "result": {
        "total": 100,
        "sent": 0,
        "failed_indices": [0],
        "skipped_indices": [],
        "elapsed_ms": 5.2,
        "timing_drift_ms": 0.0,
        "late_count": 0,
        "aborted": false,
        "joint": "ANKLE_RIGHT",
        "batch_id": "abc12345"
    }
}
```

### 503 -- CAN Unavailable

```json
{
    "status": "error",
    "message": "CAN bus not connected"
}
```

### 500 -- Server Error

```json
{
    "status": "error",
    "message": "Unable to send waypoint batch: <exception detail>"
}
```

---

## Result Object Fields

| Field | Type | Present in | Description |
|-------|------|-----------|-------------|
| `total` | int | 200, 207, 502 | Total waypoints in batch |
| `sent` | int | 200, 207, 502 | Successfully sent count |
| `failed_indices` | int[] | 200, 207, 502 | Waypoint indices with send exceptions |
| `skipped_indices` | int[] | 200, 207, 502 | Indices skipped (late waypoints) |
| `elapsed_ms` | float | 200, 207, 502 | Wall-clock batch send time |
| `timing_drift_ms` | float | 200, 207, 502 | Max offset loss due to elapsed time |
| `late_count` | int | 200, 207, 502 | Waypoints with adjusted_t_offset < 15ms |
| `aborted` | bool | 200, 207, 502 | True if MAX_CONSECUTIVE_LATE (10) hit |
| `joint` | string | 200, 207, 502 | Uppercased joint name |
| `batch_id` | string | 200, 207, 502 | Server-assigned 8-char batch ID |

---

## Companion Endpoints

### POST /can/time_sync

Sends a time synchronization frame to firmware.

**Request:** `{}` or `{"timestamp_ms": 1234567890}`
**Response (200):**
```json
{
    "status": "success",
    "result": {"timestamp_ms": 1234567890}
}
```

### GET /can/status

Returns CAN bus connection status.

**Response (200):**
```json
{
    "status": "connected",
    "bus_type": "socketcan",
    "channel": "can0"
}
```

---

## Constants (must match firmware)

| Constant | Value | Description |
|----------|-------|-------------|
| MAX_BATCH_SIZE | 1800 | Max waypoints per batch (safety margin vs 2000 firmware buffer) |
| MAX_VELOCITY_DEG_S | 150.0 | Absolute velocity cap per DOF |
| T_OFFSET_MAX | 65535 | uint16 range for t_offset_ms |
| ANGLE_RESOLUTION | 100 | int16 multiplier (0.01 deg per count) |
| MAX_SYNC_AGE_MS | 2000.0 | Time sync freshness threshold |
| MIN_LEAD_MS | 15.0 | Minimum adjusted t_offset to not be "late" |
| MAX_CONSECUTIVE_LATE | 10 | Abort batch after this many consecutive late skips |

---

## Client Retry Policy (Recommended)

| Code | Max Retries | Base Backoff | Strategy |
|------|-------------|-------------|----------|
| 409 | 3 | 100ms | Exponential + jitter |
| 500 | 2 | 1s | Exponential + jitter |
| 502 | 5 | 500ms | Exponential + jitter + re-sync time |
| 503 | 5 | 500ms | Exponential + jitter + re-sync time |
| timeout | 5 | 500ms | Exponential + jitter |
| 400 | 0 | -- | Never retry (fix input) |
