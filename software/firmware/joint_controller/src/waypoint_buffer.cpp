#include "waypoint_buffer.h"

#include "pico/critical_section.h"
#include <algorithm>

namespace {
critical_section_t g_wp_lock;
bool g_wp_lock_init = false;

WaypointBuffer g_buffers[MAX_DOFS];
uint8_t g_dof_count = 0;

void ensure_lock() {
  if (!g_wp_lock_init) {
    critical_section_init(&g_wp_lock);
    g_wp_lock_init = true;
  }
}

inline void lock() {
  ensure_lock();
  critical_section_enter_blocking(&g_wp_lock);
}

inline void unlock() {
  if (!g_wp_lock_init) {
    return;
  }
  critical_section_exit(&g_wp_lock);
}

bool is_valid_dof(uint8_t dof) {
  return dof < g_dof_count;
}

WaypointBuffer &buffer_for(uint8_t dof) {
  return g_buffers[dof];
}
} // namespace

void waypoint_buffers_init(uint8_t dof_count) {
  lock();
  g_dof_count = min<uint8_t>(dof_count, MAX_DOFS);
  for (uint8_t i = 0; i < g_dof_count; ++i) {
    g_buffers[i].count          = 0;
    g_buffers[i].prev_angle_deg = 0.0f;
    g_buffers[i].prev_time_ms   = 0;
    g_buffers[i].state          = WaypointState::IDLE;
  }
  unlock();
}

uint8_t waypoint_buffers_get_dof_count() {
  return g_dof_count;
}

bool waypoint_buffer_push(uint8_t dof_index, const WaypointEntry &entry) {
  if (!is_valid_dof(dof_index)) {
    return false;
  }

  lock();
  WaypointBuffer &buf = buffer_for(dof_index);
  if (buf.count >= WAYPOINT_BUFFER_DEPTH) {
    unlock();
    return false;
  }
  buf.buffer[buf.count++] = entry;
  if (buf.state == WaypointState::IDLE) {
    buf.state = WaypointState::MOVING;
  }
  unlock();
  return true;
}

bool waypoint_buffer_peek(uint8_t dof_index, WaypointEntry &entry) {
  if (!is_valid_dof(dof_index)) {
    return false;
  }

  lock();
  WaypointBuffer &buf = buffer_for(dof_index);
  if (buf.count == 0) {
    unlock();
    return false;
  }
  entry = buf.buffer[0];
  unlock();
  return true;
}

bool waypoint_buffer_pop(uint8_t dof_index) {
  if (!is_valid_dof(dof_index)) {
    return false;
  }

  lock();
  WaypointBuffer &buf = buffer_for(dof_index);
  if (buf.count == 0) {
    unlock();
    return false;
  }
  for (uint8_t i = 1; i < buf.count; ++i) {
    buf.buffer[i - 1] = buf.buffer[i];
  }
  buf.count--;
  if (buf.count == 0) {
    buf.state = WaypointState::HOLDING;
  }
  unlock();
  return true;
}

void waypoint_buffer_clear(uint8_t dof_index) {
  if (!is_valid_dof(dof_index)) {
    return;
  }
  lock();
  WaypointBuffer &buf = buffer_for(dof_index);
  buf.count          = 0;
  buf.prev_time_ms   = 0;
  buf.prev_angle_deg = 0.0f;
  buf.state          = WaypointState::IDLE;
  unlock();
}

void waypoint_buffer_reset_all() {
  lock();
  for (uint8_t i = 0; i < g_dof_count; ++i) {
    g_buffers[i].count          = 0;
    g_buffers[i].prev_time_ms   = 0;
    g_buffers[i].prev_angle_deg = 0.0f;
    g_buffers[i].state          = WaypointState::IDLE;
  }
  unlock();
}

uint8_t waypoint_buffer_count(uint8_t dof_index) {
  if (!is_valid_dof(dof_index)) {
    return 0;
  }
  lock();
  const uint8_t count = buffer_for(dof_index).count;
  unlock();
  return count;
}

WaypointState waypoint_buffer_state(uint8_t dof_index) {
  if (!is_valid_dof(dof_index)) {
    return WaypointState::IDLE;
  }
  lock();
  WaypointState state = buffer_for(dof_index).state;
  unlock();
  return state;
}

void waypoint_buffer_set_state(uint8_t dof_index, WaypointState state) {
  if (!is_valid_dof(dof_index)) {
    return;
  }
  lock();
  buffer_for(dof_index).state = state;
  unlock();
}

float waypoint_buffer_prev_angle(uint8_t dof_index) {
  if (!is_valid_dof(dof_index)) {
    return 0.0f;
  }
  lock();
  float angle = buffer_for(dof_index).prev_angle_deg;
  unlock();
  return angle;
}

uint32_t waypoint_buffer_prev_time(uint8_t dof_index) {
  if (!is_valid_dof(dof_index)) {
    return 0;
  }
  lock();
  uint32_t t = buffer_for(dof_index).prev_time_ms;
  unlock();
  return t;
}

void waypoint_buffer_set_prev(uint8_t dof_index, float angle_deg, uint32_t time_ms) {
  if (!is_valid_dof(dof_index)) {
    return;
  }
  lock();
  WaypointBuffer &buf = buffer_for(dof_index);
  buf.prev_angle_deg  = angle_deg;
  buf.prev_time_ms    = time_ms;
  unlock();
}

