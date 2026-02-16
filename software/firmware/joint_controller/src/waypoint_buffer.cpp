#include "waypoint_buffer.h"

#include <algorithm>

// Telemetry counters (Core1 only)
WaypointTelemetry wp_telemetry = {0, 0, 0};

// ============================================================================
// WAYPOINT BUFFER — Single-core implementation (Core1 only)
// ============================================================================
// All waypoint buffer access happens exclusively on Core1:
//   - Push: pollHostCan() receives CAN waypoints on Core1
//   - Peek/Pop: executeWaypointMovement() consumes waypoints on Core1
//   - Clear: emergency stop handler on Core1
//
// No cross-core locking is needed. Previous implementation used
// critical_section_t (hardware spinlock) which added unnecessary overhead
// of ~10-20 CPU cycles per lock/unlock, thousands of times per second.
// ============================================================================

namespace {
WaypointBuffer g_buffers[MAX_DOFS];
uint8_t g_dof_count = 0;

bool is_valid_dof(uint8_t dof) {
  return dof < g_dof_count;
}

WaypointBuffer &buffer_for(uint8_t dof) {
  return g_buffers[dof];
}
} // namespace

void waypoint_buffers_init(uint8_t dof_count) {
  g_dof_count = min<uint8_t>(dof_count, MAX_DOFS);
  for (uint8_t i = 0; i < g_dof_count; ++i) {
    g_buffers[i].head           = 0;
    g_buffers[i].tail           = 0;
    g_buffers[i].count          = 0;
    g_buffers[i].prev_angle_deg      = 0.0f;
    g_buffers[i].prev_time_ms        = 0;
    g_buffers[i].state               = WaypointState::IDLE;
    g_buffers[i].last_pushed_time_ms  = 0;
    g_buffers[i].last_pushed_angle_deg = 0.0f;
  }
}

uint8_t waypoint_buffers_get_dof_count() {
  return g_dof_count;
}

bool waypoint_buffer_push(uint8_t dof_index, const WaypointEntry &entry) {
  if (!is_valid_dof(dof_index)) {
    return false;
  }

  WaypointBuffer &buf = buffer_for(dof_index);
  if (buf.count >= WAYPOINT_BUFFER_DEPTH) {
    wp_telemetry.wp_dropped_full++;
    return false;  // Buffer full
  }

  // Ring buffer push: write at tail, advance tail
  buf.buffer[buf.tail] = entry;
  buf.tail = (buf.tail + 1) % WAYPOINT_BUFFER_DEPTH;
  buf.count++;
  wp_telemetry.wp_accepted++;

  // Track last pushed entry for monotonicity & velocity checks
  buf.last_pushed_time_ms = entry.t_arrival_ms;
  buf.last_pushed_angle_deg = entry.target_angle_deg;

  if (buf.state == WaypointState::IDLE) {
    buf.state = WaypointState::MOVING;
  }
  return true;
}

bool waypoint_buffer_peek(uint8_t dof_index, WaypointEntry &entry) {
  if (!is_valid_dof(dof_index)) {
    return false;
  }

  WaypointBuffer &buf = buffer_for(dof_index);
  if (buf.count == 0) {
    return false;
  }
  // Ring buffer peek: read from head
  entry = buf.buffer[buf.head];
  return true;
}

bool waypoint_buffer_pop(uint8_t dof_index) {
  if (!is_valid_dof(dof_index)) {
    return false;
  }

  WaypointBuffer &buf = buffer_for(dof_index);
  if (buf.count == 0) {
    return false;
  }
  
  // Ring buffer pop: just advance head (O(1) instead of O(n) shift!)
  buf.head = (buf.head + 1) % WAYPOINT_BUFFER_DEPTH;
  buf.count--;
  
  if (buf.count == 0) {
    buf.state = WaypointState::HOLDING;
  }
  return true;
}

void waypoint_buffer_clear(uint8_t dof_index) {
  if (!is_valid_dof(dof_index)) {
    return;
  }
  WaypointBuffer &buf = buffer_for(dof_index);
  buf.head           = 0;
  buf.tail           = 0;
  buf.count          = 0;
  buf.prev_time_ms        = 0;
  buf.prev_angle_deg      = 0.0f;
  buf.state               = WaypointState::IDLE;
  buf.last_pushed_time_ms  = 0;
  buf.last_pushed_angle_deg = 0.0f;
}

void waypoint_buffer_reset_all() {
  for (uint8_t i = 0; i < g_dof_count; ++i) {
    g_buffers[i].head           = 0;
    g_buffers[i].tail           = 0;
    g_buffers[i].count          = 0;
    g_buffers[i].prev_time_ms        = 0;
    g_buffers[i].prev_angle_deg      = 0.0f;
    g_buffers[i].state               = WaypointState::IDLE;
    g_buffers[i].last_pushed_time_ms  = 0;
    g_buffers[i].last_pushed_angle_deg = 0.0f;
  }
}

uint16_t waypoint_buffer_count(uint8_t dof_index) {
  if (!is_valid_dof(dof_index)) {
    return 0;
  }
  return buffer_for(dof_index).count;
}

WaypointState waypoint_buffer_state(uint8_t dof_index) {
  if (!is_valid_dof(dof_index)) {
    return WaypointState::IDLE;
  }
  return buffer_for(dof_index).state;
}

void waypoint_buffer_set_state(uint8_t dof_index, WaypointState state) {
  if (!is_valid_dof(dof_index)) {
    return;
  }
  buffer_for(dof_index).state = state;
}

float waypoint_buffer_prev_angle(uint8_t dof_index) {
  if (!is_valid_dof(dof_index)) {
    return 0.0f;
  }
  return buffer_for(dof_index).prev_angle_deg;
}

uint32_t waypoint_buffer_prev_time(uint8_t dof_index) {
  if (!is_valid_dof(dof_index)) {
    return 0;
  }
  return buffer_for(dof_index).prev_time_ms;
}

void waypoint_buffer_set_prev(uint8_t dof_index, float angle_deg, uint32_t time_ms) {
  if (!is_valid_dof(dof_index)) {
    return;
  }
  WaypointBuffer &buf = buffer_for(dof_index);
  buf.prev_angle_deg  = angle_deg;
  buf.prev_time_ms    = time_ms;
}

uint32_t waypoint_buffer_last_pushed_time(uint8_t dof_index) {
  if (!is_valid_dof(dof_index)) {
    return 0;
  }
  return buffer_for(dof_index).last_pushed_time_ms;
}

float waypoint_buffer_last_pushed_angle(uint8_t dof_index) {
  if (!is_valid_dof(dof_index)) {
    return 0.0f;
  }
  return buffer_for(dof_index).last_pushed_angle_deg;
}
