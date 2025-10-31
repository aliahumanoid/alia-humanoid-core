/**
 * @file path.cpp
 * @brief Implementation of trajectory generators
 * 
 * See path.h for detailed documentation.
 */

#include "path.h"

// ===================================================================
// LINEAR (TRAPEZOIDAL) PROFILE
// ===================================================================

/**
 * Generate linear velocity profile with constant acceleration
 * 
 * Algorithm:
 * 1. Determine motion direction (sign of vmax)
 * 2. Calculate cruise phase duration (tmax)
 * 3. Compute phase transition times (t1, t2, t3)
 * 4. Generate velocity profile for each time step
 * 5. Integrate velocity to get position
 * 6. Differentiate velocity to get acceleration
 */
void path_linear(float xstart, float xstop, float vmax, float ta, int nstep,
                 std::array<float, MAX_STEPS> &t, std::array<float, MAX_STEPS> &x,
                 std::array<float, MAX_STEPS> &v, std::array<float, MAX_STEPS> &a) {
  float tmax, t1, t2, t3;

  // Step 1: Determine motion direction
  // If moving backward, negate vmax to maintain consistent math
  if (xstop < xstart)
    vmax = -vmax;

  // Step 2: Calculate cruise phase duration
  // If distance requires more time than 2*ta (accel+decel), add cruise phase
  // Otherwise, reduce vmax and skip cruise phase (triangular profile)
  if ((xstop - xstart) / vmax > ta)
    tmax = (xstop - xstart) / vmax - ta;
  else {
    tmax = 0;
    vmax = (xstop - xstart) / ta; // Reduce vmax for short moves
  }

  // Step 3: Define phase transition times
  t1 = ta;              // End of acceleration phase
  t2 = ta + tmax;       // End of cruise phase (start of deceleration)
  t3 = 2 * ta + tmax;   // End of trajectory

  // Step 4: Generate velocity profile
  for (int i = 0; i < nstep; i++) {
    // Linearly space time points from 0 to t3
    t[i] = t3 * i / (nstep - 1);
    
    // Compute velocity based on current phase
    if (t[i] <= t1)
      // Acceleration phase: v(t) = (vmax/ta) * t
      v[i] = vmax / ta * t[i];
    else if (t[i] <= t2)
      // Cruise phase: v(t) = vmax (constant)
      v[i] = vmax;
    else
      // Deceleration phase: v(t) = vmax - (vmax/ta) * (t - t2)
      v[i] = vmax - vmax / ta * (t[i] - t2);
  }

  // Step 5: Integrate velocity to get position (Euler integration)
  x[0] = xstart;
  a[0] = 0;
  for (int i = 1; i < nstep; i++) {
    // Position: x(i) = x(i-1) + v(i-1) * dt
    x[i] = x[i - 1] + (t[i] - t[i - 1]) * v[i - 1];
    
    // Step 6: Acceleration: a(i) = [v(i) - v(i-1)] / dt
    a[i] = (v[i] - v[i - 1]) / (t[i] - t[i - 1]);
  }
}

// ===================================================================
// TRIGONOMETRIC (SMOOTH) PROFILE
// ===================================================================

/**
 * Generate trigonometric velocity profile with sinusoidal acceleration
 * 
 * Uses cosine-based velocity profile for smooth, jerk-free motion.
 * The acceleration follows a sine wave, resulting in zero jerk at
 * all phase transitions.
 * 
 * Algorithm follows same structure as path_linear but with different
 * velocity equations during acceleration and deceleration.
 */
void path_trig(float xstart, float xstop, float vmax, float ta, int nstep,
               std::array<float, MAX_STEPS> &t, std::array<float, MAX_STEPS> &x,
               std::array<float, MAX_STEPS> &v, std::array<float, MAX_STEPS> &a) {
  float tmax, t1, t2, t3;

  // Determine motion direction
  if (xstop < xstart)
    vmax = -vmax;

  // Calculate cruise phase duration
  if ((xstop - xstart) / vmax > ta)
    tmax = (xstop - xstart) / vmax - ta;
  else {
    tmax = 0;
    vmax = (xstop - xstart) / ta; // Triangular profile for short moves
  }

  // Define phase transition times
  t1 = ta;              // End of acceleration
  t2 = ta + tmax;       // End of cruise
  t3 = 2 * ta + tmax;   // End of trajectory

  // Generate smooth velocity profile using cosine
  for (int i = 0; i < nstep; i++) {
    t[i] = t3 * i / (nstep - 1);
    
    if (t[i] <= t1)
      // Acceleration: v(t) = (vmax/2) * [1 - cos(π*t/ta)]
      // Starts at 0, smoothly accelerates to vmax
      // Derivative (acceleration) follows sine curve
      v[i] = vmax / 2 * (1 - cos(M_PI / ta * t[i]));
    else if (t[i] <= t2)
      // Cruise: constant velocity
      v[i] = vmax;
    else
      // Deceleration: v(t) = (vmax/2) * [1 + cos(π*(t-t2)/ta)]
      // Smoothly decelerates from vmax to 0
      v[i] = vmax / 2 * (1 + cos(M_PI / ta * (t[i] - t2)));
  }

  // Integrate velocity to position
  x[0] = xstart;
  a[0] = 0;
  for (int i = 1; i < nstep; i++) {
    x[i] = x[i - 1] + (t[i] - t[i - 1]) * v[i - 1];
    a[i] = (v[i] - v[i - 1]) / (t[i] - t[i - 1]);
  }
}

// ===================================================================
// QUADRATIC PROFILE
// ===================================================================

/**
 * Generate quadratic velocity profile with parabolic acceleration
 * 
 * Uses quadratic velocity curves for a compromise between the speed
 * of linear profiles and the smoothness of trigonometric profiles.
 * 
 * Key difference: Requires 4/3*ta for acceleration/deceleration instead
 * of ta like the other profiles. This is because the quadratic profile
 * needs more time to reach vmax while maintaining smooth acceleration.
 * 
 * Algorithm similar to path_linear but with parabolic velocity equations.
 */
void path_quad(float xstart, float xstop, float vmax, float ta, int nstep,
               std::array<float, MAX_STEPS> &t, std::array<float, MAX_STEPS> &x,
               std::array<float, MAX_STEPS> &v, std::array<float, MAX_STEPS> &a) {
  float tmax, t1, t2, t3;

  // Determine motion direction
  if (xstop < xstart)
    vmax = -vmax;

  // Calculate cruise phase duration
  // Note: Uses 4/3*ta instead of ta due to quadratic profile geometry
  if ((xstop - xstart) / vmax > 4.0f / 3.0f * ta)
    tmax = (xstop - xstart) / vmax - 4.0f / 3.0f * ta;
  else {
    tmax = 0;
    vmax = (xstop - xstart) / (4.0f / 3.0f * ta); // Adjust vmax for short moves
  }

  // Define phase transition times
  t1 = ta;              // End of acceleration
  t2 = ta + tmax;       // End of cruise
  t3 = 2 * ta + tmax;   // End of trajectory

  // Generate quadratic velocity profile
  for (int i = 0; i < nstep; i++) {
    t[i] = t3 * i / (nstep - 1);
    
    if (t[i] <= t1)
      // Acceleration: v(t) = (2*vmax/ta)*t - (vmax/ta²)*t²
      // Quadratic curve from 0 to vmax
      // Peak acceleration at t=0, decreasing to 0 at t=ta
      v[i] = (2 * vmax / ta) * t[i] - (vmax / (ta * ta)) * t[i] * t[i];
    else if (t[i] <= t2)
      // Cruise: constant velocity
      v[i] = vmax;
    else
      // Deceleration: v(t) = vmax - (vmax/ta²)*(t-t2)²
      // Quadratic curve from vmax to 0
      // Smooth deceleration with decreasing rate
      v[i] = vmax - (vmax / (ta * ta)) * (t[i] - t2) * (t[i] - t2);
  }

  // Integrate velocity to position
  x[0] = xstart;
  a[0] = 0;
  for (int i = 1; i < nstep; i++) {
    x[i] = x[i - 1] + (t[i] - t[i - 1]) * v[i - 1];
    a[i] = (v[i] - v[i - 1]) / (t[i] - t[i - 1]);
  }
}
