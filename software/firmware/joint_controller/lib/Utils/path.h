/**
 * @file path.h
 * @brief Trajectory generators for smooth motion planning
 * 
 * This module provides three types of velocity profile generators for
 * point-to-point motion planning. All profiles follow a three-phase
 * structure: acceleration → constant velocity → deceleration.
 * 
 * Trajectory Phases:
 * - Phase 1 (0 to t1): Acceleration from 0 to vmax
 * - Phase 2 (t1 to t2): Constant velocity cruise at vmax
 * - Phase 3 (t2 to t3): Deceleration from vmax to 0
 * 
 * Time Parameters:
 * - ta: Acceleration/deceleration duration
 * - tmax: Cruise phase duration (calculated automatically)
 * - t1 = ta
 * - t2 = ta + tmax
 * - t3 = 2*ta + tmax (total trajectory time)
 * 
 * Available Profiles:
 * 
 * 1. LINEAR (Trapezoidal):
 *    - Linear acceleration/deceleration
 *    - Constant jerk at transitions
 *    - Fast but can cause vibrations
 *    - Best for: high-speed moves where smoothness is less critical
 * 
 * 2. TRIGONOMETRIC (Smooth):
 *    - Sinusoidal acceleration/deceleration
 *    - Zero jerk at all transitions
 *    - Smooth but slower acceleration
 *    - Best for: precision moves, delicate payloads
 * 
 * 3. QUADRATIC:
 *    - Quadratic acceleration/deceleration
 *    - Moderate smoothness
 *    - Compromise between speed and smoothness
 *    - Best for: general-purpose motion
 * 
 * All functions generate:
 * - Position trajectory (x)
 * - Velocity profile (v)
 * - Acceleration profile (a)
 * - Time vector (t)
 * 
 * Example Usage:
 * @code
 * std::array<float, MAX_STEPS> t, x, v, a;
 * path_trig(0.0, 90.0, 45.0, 0.5, 100, t, x, v, a);
 * // Generates smooth trajectory from 0° to 90° with vmax=45°/s, ta=0.5s
 * @endcode
 */

#ifndef PATH_H
#define PATH_H

#include <global.h>
#include <array>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/**
 * @brief Generate linear (trapezoidal) velocity profile
 * 
 * Creates a trajectory with linear acceleration and deceleration phases.
 * This profile has constant jerk at phase transitions, which may cause
 * vibrations but allows faster motion.
 * 
 * Velocity profile:
 * - Acceleration: v(t) = (vmax/ta) * t
 * - Cruise: v(t) = vmax
 * - Deceleration: v(t) = vmax - (vmax/ta) * (t - t2)
 * 
 * @param xstart Starting position (e.g., degrees or mm)
 * @param xstop Target position (same units as xstart)
 * @param vmax Maximum velocity (units/second)
 * @param ta Acceleration/deceleration duration (seconds)
 * @param nstep Number of trajectory points to generate
 * @param[out] t Time array (seconds)
 * @param[out] x Position array (same units as xstart)
 * @param[out] v Velocity array (same units as vmax)
 * @param[out] a Acceleration array (units/second²)
 * 
 * @note If the distance is too short for the given vmax and ta,
 *       vmax is automatically reduced and tmax becomes 0
 */
void path_linear(float xstart, float xstop, float vmax, float ta, int nstep,
                 std::array<float, MAX_STEPS> &t, std::array<float, MAX_STEPS> &x,
                 std::array<float, MAX_STEPS> &v, std::array<float, MAX_STEPS> &a);

/**
 * @brief Generate trigonometric (smooth) velocity profile
 * 
 * Creates a trajectory with sinusoidal acceleration and deceleration phases.
 * This profile has zero jerk at all transitions, providing the smoothest
 * motion at the cost of slower acceleration.
 * 
 * Velocity profile:
 * - Acceleration: v(t) = (vmax/2) * [1 - cos(π*t/ta)]
 * - Cruise: v(t) = vmax
 * - Deceleration: v(t) = (vmax/2) * [1 + cos(π*(t-t2)/ta)]
 * 
 * @param xstart Starting position (e.g., degrees or mm)
 * @param xstop Target position (same units as xstart)
 * @param vmax Maximum velocity (units/second)
 * @param ta Acceleration/deceleration duration (seconds)
 * @param nstep Number of trajectory points to generate
 * @param[out] t Time array (seconds)
 * @param[out] x Position array (same units as xstart)
 * @param[out] v Velocity array (same units as vmax)
 * @param[out] a Acceleration array (units/second²)
 * 
 * @note Recommended for precision applications and delicate payloads
 */
void path_trig(float xstart, float xstop, float vmax, float ta, int nstep,
               std::array<float, MAX_STEPS> &t, std::array<float, MAX_STEPS> &x,
               std::array<float, MAX_STEPS> &v, std::array<float, MAX_STEPS> &a);

/**
 * @brief Generate quadratic velocity profile
 * 
 * Creates a trajectory with quadratic acceleration and deceleration phases.
 * This profile offers a compromise between the speed of linear profiles
 * and the smoothness of trigonometric profiles.
 * 
 * Velocity profile:
 * - Acceleration: v(t) = (2*vmax/ta)*t - (vmax/ta²)*t²
 * - Cruise: v(t) = vmax
 * - Deceleration: v(t) = vmax - (vmax/ta²)*(t-t2)²
 * 
 * @param xstart Starting position (e.g., degrees or mm)
 * @param xstop Target position (same units as xstart)
 * @param vmax Maximum velocity (units/second)
 * @param ta Acceleration/deceleration duration (seconds)
 * @param nstep Number of trajectory points to generate
 * @param[out] t Time array (seconds)
 * @param[out] x Position array (same units as xstart)
 * @param[out] v Velocity array (same units as vmax)
 * @param[out] a Acceleration array (units/second²)
 * 
 * @note The quadratic profile requires 4/3*ta for accel/decel phases
 *       instead of ta like the other profiles
 */
void path_quad(float xstart, float xstop, float vmax, float ta, int nstep,
               std::array<float, MAX_STEPS> &t, std::array<float, MAX_STEPS> &x,
               std::array<float, MAX_STEPS> &v, std::array<float, MAX_STEPS> &a);

#endif // PATH_H