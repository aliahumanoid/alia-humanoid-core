/**
 * @file waypoint_buffer.h
 * @brief Thread-safe waypoint buffer manager for host-driven CAN control
 */

#ifndef WAYPOINT_BUFFER_H
#define WAYPOINT_BUFFER_H

#include <Arduino.h>
#include <global.h>

#ifndef WAYPOINT_BUFFER_DEPTH
#define WAYPOINT_BUFFER_DEPTH 8
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
  uint8_t count = 0;
  float prev_angle_deg = 0.0f;
  uint32_t prev_time_ms = 0;
  WaypointState state = WaypointState::IDLE;
};

void waypoint_buffers_init(uint8_t dof_count);
uint8_t waypoint_buffers_get_dof_count();

bool waypoint_buffer_push(uint8_t dof_index, const WaypointEntry &entry);
bool waypoint_buffer_peek(uint8_t dof_index, WaypointEntry &entry);
bool waypoint_buffer_pop(uint8_t dof_index);

void waypoint_buffer_clear(uint8_t dof_index);
void waypoint_buffer_reset_all();

uint8_t waypoint_buffer_count(uint8_t dof_index);
WaypointState waypoint_buffer_state(uint8_t dof_index);
void waypoint_buffer_set_state(uint8_t dof_index, WaypointState state);

float waypoint_buffer_prev_angle(uint8_t dof_index);
uint32_t waypoint_buffer_prev_time(uint8_t dof_index);
void waypoint_buffer_set_prev(uint8_t dof_index, float angle_deg, uint32_t time_ms);

#endif // WAYPOINT_BUFFER_H

