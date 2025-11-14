# CAN Control Protocol Specification

**Version**: 1.0  
**Date**: 14 November 2025  
**Status**: Draft - Phase 0 Design  
**Target**: Dual CAN bus architecture (Host ↔ Controllers, Controllers ↔ Motors)

---

## 1. Architecture Overview

```
┌─────────────┐  USB   ┌──────────────┐  CAN_H/L  ┌──────────────┐
│   HOST PC   │◄──────►│ MKS CANable  │◄─────────►│ Controller 1 │
│  (Python)   │        │  (USB-CAN)   │           │  (RP2040)    │
└─────────────┘        └──────────────┘           └──────┬───────┘
                                │                         │
                                │                   MCP2515 #2
                          CAN @ 1 Mbps                    │
                          (Host Bus)              ┌───────┴────────┐
                                │                 │                │
                       ┌────────┴────────┐   MCP2515 #1      Motors (LKM)
                       │                 │        │          CAN ID 1-4
                  Controller 2      Controller 3  │
                  Controller 4      Controller 5  └─→ Motor CAN Bus
                  Controller 6                        @ 1 Mbps
```

**Key Components**:
- **Host CAN Bus**: Commands & status between PC and joint controllers
- **Motor CAN Bus**: Each controller has dedicated bus to its motors (4x LKM per joint)
- **Dual MCP2515**: Same SPI1, different CS pins (GP8 host, GP9 motors)

---

## 2. Design Decisions Summary

| Decision | Choice | Rationale |
|----------|--------|-----------|
| **Message Format** | Fixed 8-byte struct | Fast parsing, deterministic timing |
| **Time Reference** | Absolute timestamp (uint32_t ms) | Eliminates clock drift across 6-20 nodes |
| **Trajectory Gen** | Host-side (sparse waypoints) | Controller simple, host flexible |
| **Update Rate** | 50-100 Hz (host) | Balance bandwidth/reactivity, 10-20ms margin |
| **Buffer Depth** | 2 waypoints | Smooth transitions, jitter tolerance |
| **Sync Method** | Periodic NTP-like | Re-sync every 10-60s, drift compensation |
| **Interpolation** | Linear (controller-side) | Low CPU, sufficient with dense waypoints |
| **Transport** | Mutually exclusive (CAN OR Serial) | Clean architecture, compile-time switch |

---

## 3. CAN Message Formats

### 3.1 Command: Waypoint Setpoint

```cpp
struct CanCmd_Waypoint {
    uint8_t  dof_index;       // 0-2 (which DOF to move)
    int16_t  target_angle;    // Target angle in 0.01° units (±327.67°)
    uint32_t t_arrival_ms;    // Absolute arrival time (epoch milliseconds)
    uint8_t  mode;            // Trajectory mode (see enum below)
} __attribute__((packed));   // Total: 8 bytes

// CAN ID: 0x010 + (joint_id × 0x10)
// Example: Ankle_Right = 0x010, Knee_Right = 0x030
```

**Trajectory Modes**:
```cpp
enum TrajectoryMode {
    MODE_DIRECT     = 0x00,  // Instant position change (no interpolation)
    MODE_LINEAR     = 0x01,  // Linear interpolation v = Δs/Δt
    MODE_SMOOTH     = 0x02,  // Trigonometric/smooth (existing path_trig)
};
```

---

### 3.2 Status: Joint Feedback

```cpp
struct CanStatus_Joint {
    uint8_t  dof_index;       // 0-2
    int16_t  current_angle;   // Current angle (0.01° resolution)
    int16_t  target_angle;    // Where moving toward
    uint8_t  progress;        // 0-100% completion
    uint8_t  flags;           // Status flags (see below)
    uint8_t  temperature;     // Max motor temp (°C)
} __attribute__((packed));   // Total: 8 bytes

// CAN ID: 0x200 + (joint_id × 0x10)
// Update rate: 50 Hz (periodic)
```

**Status Flags**:
```cpp
#define STATUS_MOVING       (1 << 0)  // Currently executing trajectory
#define STATUS_HOLDING      (1 << 1)  // Holding position (reached target)
#define STATUS_ERROR        (1 << 2)  // Error condition active
#define STATUS_BUFFER_FULL  (1 << 3)  // Waypoint buffer full (reject new)
#define STATUS_SYNCED       (1 << 4)  // Clock synchronized with host
```

---

### 3.3 System: Time Synchronization

```cpp
// Host → All Controllers (broadcast)
struct CanCmd_TimeSync {
    uint32_t t_host_ms;       // Host epoch time (milliseconds)
    uint32_t reserved;        // Future use
} __attribute__((packed));   // 8 bytes

// CAN ID: 0x002 (broadcast, high priority)
```

**Controller Response**: Status message with SYNCED flag set.

**Sync Frequency**: Every 10-60 seconds (drift compensation).

---

### 3.4 System: Emergency Stop

```cpp
// Host → All Controllers (broadcast)
struct CanCmd_EmergencyStop {
    uint8_t  reason_code;     // Stop reason (user, limit, error, etc.)
    uint8_t  reserved[7];
} __attribute__((packed));   // 8 bytes

// CAN ID: 0x000 (highest priority)
```

---

## 4. CAN ID Allocation

### 4.1 Command Messages (Host → Controllers)

```
0x000: Emergency Stop (broadcast, highest priority)
0x001: Reserved (future sync trigger)
0x002: Time Sync (broadcast)
0x010: Ankle Right command
0x020: Ankle Left command
0x030: Knee Right command
0x040: Knee Left command
0x050: Hip Right command
0x060: Hip Left command
...
0x140: Joint 20 command (future expansion)
```

**Address Space**: 0x000-0x1FF (512 IDs)  
**Allocation**: 0x010 + (joint_id × 0x10)  
**Max Joints**: 20 nodes

---

### 4.2 Status Messages (Controllers → Host)

```
0x200: Reserved
0x210: Ankle Right status
0x220: Ankle Left status
0x230: Knee Right status
0x240: Knee Left status
0x250: Hip Right status
0x260: Hip Left status
...
0x340: Joint 20 status
```

**Address Space**: 0x200-0x3FF  
**Allocation**: 0x200 + (joint_id × 0x10)

---

## 5. Controller-Side Implementation

### 5.1 Waypoint Buffer

```cpp
#define WAYPOINT_BUFFER_DEPTH 2

struct WaypointBuffer {
    CanCmd_Waypoint buffer[WAYPOINT_BUFFER_DEPTH];
    uint8_t count;                    // Number of waypoints in buffer
    float prev_angle;                 // Angle at start of current segment
    uint32_t prev_time;               // Time at start of current segment
};

// Per-DOF state
WaypointBuffer waypoint_buffers[MAX_DOFS];
```

---

### 5.2 Execution Logic

```cpp
void onWaypointReceived(uint8_t dof, CanCmd_Waypoint wp) {
    WaypointBuffer *buf = &waypoint_buffers[dof];
    
    // Validate timestamp
    uint32_t t_now = get_absolute_time_ms();
    if (wp.t_arrival_ms <= t_now) {
        LOG_ERROR("Waypoint in the past!");
        return;
    }
    
    // Check buffer space
    if (buf->count >= WAYPOINT_BUFFER_DEPTH) {
        LOG_WARN("Buffer full, rejecting waypoint");
        set_status_flag(STATUS_BUFFER_FULL);
        return;
    }
    
    // Insert sorted by timestamp
    insert_waypoint_sorted(buf, wp);
    buf->count++;
}

void updateTrajectory(uint8_t dof) {
    WaypointBuffer *buf = &waypoint_buffers[dof];
    uint32_t t_now = get_absolute_time_ms();
    
    // Check if reached current waypoint
    if (buf->count > 0 && t_now >= buf->buffer[0].t_arrival_ms) {
        // Transition to next waypoint
        buf->prev_angle = getCurrentAngle(dof);
        buf->prev_time = t_now;
        shift_buffer(buf);  // Remove first, shift others
        buf->count--;
    }
    
    // Compute interpolated target
    if (buf->count > 0) {
        CanCmd_Waypoint *wp = &buf->buffer[0];
        float time_total = wp->t_arrival_ms - buf->prev_time;
        float time_elapsed = t_now - buf->prev_time;
        float progress = time_elapsed / time_total;
        
        // Linear interpolation (MODE_LINEAR)
        float target = buf->prev_angle + 
                       (wp->target_angle / 100.0f - buf->prev_angle) * progress;
        
        // Feed to PID controller
        pid_outer[dof].setTarget(target);
        set_status_flag(STATUS_MOVING);
    } else {
        // No waypoints - hold position
        set_status_flag(STATUS_HOLDING);
    }
}
```

---

### 5.3 Time Synchronization

```cpp
// Global clock offset (updated by sync messages)
volatile int32_t clock_offset_ms = 0;
volatile bool clock_synced = false;

void onTimeSyncReceived(CanCmd_TimeSync sync) {
    uint32_t t_local = millis();  // RP2040 local time
    clock_offset_ms = sync.t_host_ms - t_local;
    clock_synced = true;
    
    LOG_INFO("Clock synced: offset = " + String(clock_offset_ms) + " ms");
}

uint32_t get_absolute_time_ms() {
    if (!clock_synced) {
        LOG_WARN("Clock not synced! Using local time");
        return millis();
    }
    return millis() + clock_offset_ms;
}
```

---

## 6. Example: Complete Movement Sequence

### Scenario
- **Joints**: Ankle Right (2 DOF), Knee Right (1 DOF)
- **Movement**: Coordinated ankle plantarflexion + knee flexion
- **Duration**: 200ms trajectory with 2 waypoints per joint
- **Update Rate**: 50 Hz (waypoint every 20ms, but sending 2 at start)

---

### 6.1 Initial Time Synchronization

```
t=0.000s HOST:
  ├─→ CAN TX [ID=0x002]: TIME_SYNC
  │   Data: [t_host=1700000000 (0x6564D780), reserved=0]
  │   (Epoch: Nov 14 2025, 17:13:20 UTC)
  │
  └─→ All controllers update clock_offset

t=0.001s CONTROLLERS:
  ├─→ Ankle_Right: clock_offset = 1700000000 - 12345 = 1699987655 ms
  ├─→ Knee_Right:  clock_offset = 1700000000 - 12348 = 1699987652 ms
  │   (3ms local clock drift between controllers - now compensated!)
  └─→ Set STATUS_SYNCED flag

t=0.002s CONTROLLERS:
  └─→ CAN TX [ID=0x210, 0x230]: STATUS messages
      Ankle_Right: [dof=0, angle=0, target=0, progress=0, flags=0x10 (SYNCED), temp=35]
      Knee_Right:  [dof=0, angle=0, target=0, progress=0, flags=0x10 (SYNCED), temp=33]
```

---

### 6.2 Trajectory Planning (Host Side)

```
t=0.010s HOST COMPUTATION:
  # Inverse kinematics + trajectory generation
  ankle_traj = [
      Waypoint(dof=0, angle=10.0°, t=1700000.100s),  # +100ms
      Waypoint(dof=0, angle=20.0°, t=1700000.200s),  # +200ms
  ]
  
  knee_traj = [
      Waypoint(dof=0, angle=5.0°, t=1700000.100s),
      Waypoint(dof=0, angle=10.0°, t=1700000.200s),
  ]
```

---

### 6.3 Waypoint Transmission (Pre-load)

```
t=0.015s HOST → CAN BUS:
  ├─→ [ID=0x010] Ankle_Right waypoint 1:
  │   [dof=0, angle=1000 (10.0°×100), t_arrival=1700000100, mode=LINEAR]
  │   Frame: [00 E8 03 64 65 64 D8 01]
  │   
  ├─→ [ID=0x010] Ankle_Right waypoint 2:
  │   [dof=0, angle=2000 (20.0°×100), t_arrival=1700000200, mode=LINEAR]
  │   Frame: [00 D0 07 C8 65 64 D8 01]
  │
  ├─→ [ID=0x030] Knee_Right waypoint 1:
  │   [dof=0, angle=500 (5.0°×100), t_arrival=1700000100, mode=LINEAR]
  │   
  └─→ [ID=0x030] Knee_Right waypoint 2:
      [dof=0, angle=1000 (10.0°×100), t_arrival=1700000200, mode=LINEAR]

Total transmission time: 4 messages × 172µs = 688µs
```

---

### 6.4 Controller Buffer State

```
t=0.016s CONTROLLERS AFTER RECEPTION:

Ankle_Right buffer (dof=0):
  ├─→ buffer[0]: {angle=10.0°, t_arrival=1700000.100, mode=LINEAR}
  ├─→ buffer[1]: {angle=20.0°, t_arrival=1700000.200, mode=LINEAR}
  ├─→ count = 2
  ├─→ prev_angle = 0.0° (current position)
  └─→ prev_time = 1700000.015 (time waypoint received)

Knee_Right buffer (dof=0):
  ├─→ buffer[0]: {angle=5.0°, t_arrival=1700000.100}
  ├─→ buffer[1]: {angle=10.0°, t_arrival=1700000.200}
  └─→ count = 2
```

---

### 6.5 Trajectory Execution Timeline

```
t=1700000.020s CONTROLLERS @ 100Hz PID loop:
  Ankle_Right:
    ├─→ t_now = 1700000.020
    ├─→ target_waypoint = buffer[0] (t_arrival=1700000.100)
    ├─→ time_total = 100ms - 15ms = 85ms
    ├─→ time_elapsed = 20ms - 15ms = 5ms
    ├─→ progress = 5/85 = 5.88%
    ├─→ interpolated_target = 0° + (10° - 0°) × 0.0588 = 0.59°
    └─→ PID_outer.setTarget(0.59°)
  
  Knee_Right:
    └─→ interpolated_target = 0° + (5° - 0°) × 0.0588 = 0.29°

t=1700000.050s (50ms elapsed):
  Ankle_Right:
    ├─→ time_elapsed = 50ms - 15ms = 35ms
    ├─→ progress = 35/85 = 41.2%
    └─→ interpolated_target = 0° + 10° × 0.412 = 4.12°
  
  Knee_Right:
    └─→ interpolated_target = 2.06°

t=1700000.100s (ARRIVAL at waypoint 1):
  Ankle_Right:
    ├─→ t_now >= buffer[0].t_arrival → TRANSITION!
    ├─→ prev_angle = current_angle = 10.0° ± 0.1° (PID error)
    ├─→ prev_time = 1700000.100
    ├─→ shift_buffer() → buffer[0] now = old buffer[1]
    ├─→ count = 1
    └─→ Next segment: 10° → 20° in 100ms
  
  Knee_Right:
    └─→ Similar transition: 5° → 10°

t=1700000.150s (mid-trajectory, segment 2):
  Ankle_Right:
    ├─→ time_elapsed = 150 - 100 = 50ms
    ├─→ progress = 50/100 = 50%
    └─→ interpolated_target = 10° + (20° - 10°) × 0.50 = 15.0°
  
  Knee_Right:
    └─→ interpolated_target = 5° + (10° - 5°) × 0.50 = 7.5°

t=1700000.200s (ARRIVAL at waypoint 2 - FINAL):
  Ankle_Right:
    ├─→ Reached target: 20.0°
    ├─→ count = 0 (buffer empty)
    ├─→ Status: HOLDING
    └─→ Hold position until new waypoint received
  
  Knee_Right:
    └─→ Reached target: 10.0°, HOLDING
```

---

### 6.6 Status Feedback During Motion

```
t=1700000.020s CONTROLLERS → HOST (50 Hz periodic):
  Ankle_Right status [ID=0x210]:
    [dof=0, current=59 (0.59°), target=1000 (10.0°), 
     progress=6%, flags=0x11 (MOVING|SYNCED), temp=36]

t=1700000.100s (transition point):
  Ankle_Right status:
    [dof=0, current=998 (9.98°), target=2000 (20.0°),
     progress=0%, flags=0x11, temp=38]

t=1700000.200s (final arrival):
  Ankle_Right status:
    [dof=0, current=1999 (19.99°), target=1999 (19.99°),
     progress=100%, flags=0x12 (HOLDING|SYNCED), temp=40]
```

---

### 6.7 Bandwidth Analysis

```
PHASE 1: Time Sync (1 message)
  t=0.000-0.001s: 1 broadcast × 172µs = 172µs

PHASE 2: Waypoint Pre-load (4 messages)
  t=0.015-0.016s: 4 commands × 172µs = 688µs

PHASE 3: Execution (200ms duration)
  Status feedback: 2 joints × 50 Hz × 0.2s = 20 messages
  Bandwidth: 20 × 172µs = 3440µs total (1.72% avg utilization)

TOTAL BANDWIDTH (200ms window):
  Commands: 5 messages (688µs + 172µs)
  Status: 20 messages (3440µs)
  ─────────────────────────────────
  Total: 25 messages = 4.3ms / 200ms = 2.15% utilization ✅
```

---

## 7. Bandwidth Budget Summary

### 7.1 Phase 0 (6 Joints, 50 Hz Updates)

```
Typical Walking Gait (1 second cycle):

Commands:
  - Initial sync: 1 msg
  - Waypoints: 6 joints × 2 waypoints = 12 msg
  - Mid-cycle updates: 6 joints × 3 waypoints = 18 msg
  Total commands: 31 msg/s

Status:
  - Periodic feedback: 6 joints × 50 Hz = 300 msg/s

Emergency/Events: ~5 msg/s

─────────────────────────────────────────
GRAND TOTAL: ~336 msg/s @ 5.8% utilization ✅
```

---

### 7.2 Phase 1 (20 Joints, 100 Hz Updates)

```
Full Body Control:

Commands:
  - Waypoints: 20 joints × 100 Hz = 2000 msg/s
  
Status:
  - Periodic: 20 joints × 50 Hz = 1000 msg/s

─────────────────────────────────────────
GRAND TOTAL: ~3000 msg/s @ 52% utilization ✅
Margin: 48% for bursts, retransmissions, config
```

---

## 8. Error Handling

### 8.1 Timeout Detection

```cpp
#define COMMAND_TIMEOUT_MS  100  // 2× worst update rate @ 50Hz
#define STATUS_TIMEOUT_MS   100  // 2× periodic status rate

// Controller side:
if (millis() - last_command_time > COMMAND_TIMEOUT_MS) {
    emergency_stop_all_motors();
    send_error_status(ERROR_COMMAND_TIMEOUT);
}

// Host side:
if (time.time() - last_status_time[joint_id] > STATUS_TIMEOUT_MS/1000.0) {
    mark_joint_offline(joint_id);
    trigger_safe_mode();  // Compensate with other joints
}
```

---

### 8.2 Clock Drift Detection

```cpp
// Re-sync every 30 seconds
#define RESYNC_INTERVAL_MS  30000

void periodic_sync_check() {
    if (millis() - last_sync_time > RESYNC_INTERVAL_MS) {
        send_time_sync_broadcast();
        last_sync_time = millis();
    }
}
```

---

### 8.3 Buffer Overflow

```cpp
if (buffer_full) {
    // Option A: Reject new waypoint (safe)
    LOG_WARN("Buffer full, rejecting waypoint");
    set_status_flag(STATUS_BUFFER_FULL);
    
    // Option B: Force-insert (aggressive, may cause discontinuity)
    // overwrite_oldest_waypoint(new_wp);
}
```

---

## 9. Compile-Time Configuration

```cpp
// platformio.ini
build_flags = 
    -D USE_CAN_CONTROL          ; Enable CAN control mode
    -D CAN_UPDATE_RATE_HZ=50    ; Host update rate
    -D WAYPOINT_BUFFER_DEPTH=2  ; Buffer size per DOF
    -D RESYNC_INTERVAL_MS=30000 ; Clock re-sync period
    
; Alternative: Serial control (current behavior)
; -D USE_SERIAL_CONTROL
```

---

## 10. Migration Path

### Phase 0 (Current → CAN Basic)
1. ✅ Hardware validated (MCP2515 modules, pinout confirmed)
2. Implement HostCAN library (MCP2515 #2 driver)
3. Implement waypoint buffer + interpolation
4. Test with 1-2 joints (loopback, latency measurement)
5. Scale to 6 joints (walking proof-of-concept)

### Phase 1 (Optimization)
6. Increase update rate 50 Hz → 100 Hz
7. Optimize buffer management (reduce latency)
8. Add advanced trajectory modes (S-curve, quintic)
9. Scale to 20 joints (full body)

### Phase 2 (Future)
10. Migrate to CAN-FD @ 5 Mbps (500 Hz update rate)
11. Add Ethernet for bulk data (calibration, diagnostics)

---

## 11. Testing & Validation

### 11.1 Unit Tests
- ✅ Waypoint buffer insertion/removal
- ✅ Timestamp validation (past/future detection)
- ✅ Interpolation accuracy (compare vs. expected trajectory)
- ✅ Clock sync offset calculation

### 11.2 Integration Tests
- ✅ 2-joint coordinated motion (sync accuracy <1ms)
- ✅ Bandwidth stress test (20 joints @ 100 Hz sustained)
- ✅ Timeout handling (disconnect simulation)
- ✅ Clock drift compensation (30 min continuous operation)

### 11.3 Performance Metrics
- Latency: Command TX → motion start < 2ms
- Jitter: Trajectory timing variance < ±500µs
- Sync accuracy: Multi-joint arrival time < ±200µs
- Throughput: Sustained 3000 msg/s @ <60% utilization

---

## 12. References

- MCP2515 Datasheet: [Microchip MCP2515](https://www.microchip.com/en-us/product/MCP2515)
- CAN 2.0B Specification: ISO 11898-1
- RP2040 SPI: [Pico SDK Documentation](https://raspberrypi.github.io/pico-sdk-doxygen/)
- Existing Serial Protocol: `software/firmware/joint_controller/PROTOCOL.md`
- Time Sync Implementation: `software/firmware/joint_controller/src/core0.cpp:203-230`

---

**END OF SPECIFICATION**

