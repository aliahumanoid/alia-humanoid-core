/**
 * @file waypoint_buffer.h
 * @brief Waypoint buffer manager for host-driven CAN control (Core1 only)
 *
 * All access is from Core1 — no cross-core locking needed.
 * Exception: Core0 writes during startup injection
 * (guarded by startup_injecting_waypoints with RELEASE/ACQUIRE barriers).
 */

#ifndef WAYPOINT_BUFFER_H
#define WAYPOINT_BUFFER_H

#include <Arduino.h>
#include <global.h>

// Waypoint buffer depth: number of waypoints that can be queued per DOF
// 
// Memory calculation:
//   sizeof(WaypointEntry) = 12 bytes (with padding)
//   Buffer per DOF = DEPTH * 12 bytes
//   
// Examples:
//   500 waypoints  = 6 KB per DOF   (original, supports ~60s trajectory)
//   2000 waypoints = 24 KB per DOF  (supports oscillation tests with many cycles)
//
// Pico 2 has 520 KB RAM, current usage ~30 KB (5.8%), plenty of headroom.
// With 3 DOFs × 24 KB = 72 KB total for waypoint buffers (14% of RAM)
// 
#ifndef WAYPOINT_BUFFER_DEPTH
#define WAYPOINT_BUFFER_DEPTH 2000
#endif

enum class WaypointState : uint8_t {
  IDLE = 0,
  MOVING,
  HOLDING
};

struct WaypointEntry {
  uint8_t dof_index;
  float target_angle_deg;
  uint32_t t_arrival_ms;
  uint8_t mode;
};

struct WaypointBuffer {
  WaypointEntry buffer[WAYPOINT_BUFFER_DEPTH];
  uint16_t head = 0;           // Index of next element to read (pop)
  uint16_t tail = 0;           // Index of next element to write (push)
  uint16_t count = 0;          // Number of elements in buffer
  float prev_angle_deg = 0.0f;
  uint32_t prev_time_ms = 0;
  WaypointState state = WaypointState::IDLE;
  // Tail tracking: last pushed values (for monotonicity & velocity checks)
  uint32_t last_pushed_time_ms = 0;
  float last_pushed_angle_deg = 0.0f;
};

void waypoint_buffers_init(uint8_t dof_count);
uint8_t waypoint_buffers_get_dof_count();

bool waypoint_buffer_push(uint8_t dof_index, const WaypointEntry &entry);
bool waypoint_buffer_peek(uint8_t dof_index, WaypointEntry &entry);
bool waypoint_buffer_pop(uint8_t dof_index);

void waypoint_buffer_clear(uint8_t dof_index);
void waypoint_buffer_reset_all();

uint16_t waypoint_buffer_count(uint8_t dof_index);
WaypointState waypoint_buffer_state(uint8_t dof_index);
void waypoint_buffer_set_state(uint8_t dof_index, WaypointState state);

float waypoint_buffer_prev_angle(uint8_t dof_index);
uint32_t waypoint_buffer_prev_time(uint8_t dof_index);
void waypoint_buffer_set_prev(uint8_t dof_index, float angle_deg, uint32_t time_ms);

// Tail tracking: last pushed entry values (monotonicity & velocity checks)
uint32_t waypoint_buffer_last_pushed_time(uint8_t dof_index);
float waypoint_buffer_last_pushed_angle(uint8_t dof_index);

#endif // WAYPOINT_BUFFER_H

