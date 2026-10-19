#pragma once

#include <cmath>
#include "robot_compat.h"  // firmware: "interfaces/isr.h" and "utils/core.h"
#include "velocity_profile.h"

/**
 * @file cubic_profile.h
 * @brief Angular velocity profile for smooth diagonal/curved turns.
 *
 * Copy of the mazerunner32 firmware vehicle/planner/cubic-profile.h, so that
 * the app runs the same code as the robot, with one addition: an optional
 * exact angle mode (set_exact_angle()).
 *
 * Computes omega at each tick as a cubic polynomial of the forward distance,
 * producing a smooth angular speed profile that starts and ends at zero.
 * The forward profile is read each tick — this profile does not own it.
 *
 * Firmware mode (default): the position advances by |v| dt, then omega is the
 * cubic sampled at the new position. The sum of omega * dt only approximates
 * the turn angle, and the tick that passes the end of the turn gives zero.
 *
 * Exact angle mode: the heading along the path has the closed form
 * Theta(s) = angle * (3 u^2 - 2 u^3), u = s / length. Each tick outputs
 * omega = (Theta(s1) - Theta(s0)) / dt for the step s0 to s1, the last step
 * stopping at the length, so the sum of omega * dt is the turn angle exactly
 * whatever the speed or the alignment of the end with a tick.
 *
 * Usage:
 *   1. Call init() with angle and path-length at the start of a turn.
 *   2. Call update() each control tick (after the forward profile's update()).
 *   3. Read getVelocity() for the angular speed to feed to the drive system.
 *   4. Check isComplete() to detect end of turn.
 *
 * All other VelocityProfile methods are either no-ops or delegate to reset().
 */
class CubicProfile : public VelocityProfile {
 public:
  explicit CubicProfile(float sample_time = CONTROL_LOOP_INTERVAL) : VelocityProfile(sample_time) {
    // Initialise other members if needed
  }

  /**
   * @brief Initialise for a new cubic turn.
   * @param distance     Arc length of the turn path (mm).
   * @param start_speed  Unused (turn inherits forward speed).
   * @param top_speed    Unused.
   * @param final_speed  Unused.
   * @param acceleration Unused.
   * @param jerk         Encodes the signed turn angle (degrees) for this profile.
   *
   * @note  The angle is passed via the @p jerk parameter so that the standard
   *        VelocityProfile::init() signature is preserved.  Use start_cubic_turn()
   *        in MotionDirector rather than calling init() directly.
   */
  ProfileStatus init(float distance, float start_speed, float top_speed, float final_speed, float acceleration, float jerk) override {
    (void)start_speed;
    (void)top_speed;
    (void)final_speed;
    (void)acceleration;
    float angle_deg = jerk;  // angle packed into jerk parameter
    m_angle = angle_deg;
    m_length = distance;
    m_position = 0;
    m_constant = 6.0f * (angle_deg * RADIANS_PER_DEGREE) / (m_length * m_length * m_length);
    m_omega = 0.0f;
    m_state = PS_ACTIVE;
    return ProfileStatus::SUCCESS;
  }

  /**
   * @brief Select how omega is worked out each tick; kept across init().
   * @param exact false: firmware behaviour, cubic sampled at the new position;
   *              true: heading difference over the tick, exact total angle
   */
  void set_exact_angle(bool exact) { m_exact = exact; }

  /** @brief Advance the cubic turn by one control tick. */
  ISR_SAFE void update(float current_speed) override {
    if (m_state != PS_ACTIVE) {
      return;
    }
    if (m_exact) {
      update_exact(current_speed);
      return;
    }
    m_position += fabsf(current_speed) * m_dt;
    float remaining = m_length - m_position;
    if (remaining > 0.0f) {
      float t = m_position * remaining;
      m_omega = (current_speed * m_constant * t) * DEGREES_PER_RADIAN;
    } else {
      m_omega = 0.0f;
      m_state = PS_FINISHED;
    }
  }

  void reset() override {
    m_omega = 0.0f;
    m_state = PS_IDLE;
  }

  bool isComplete() override { return m_state == PS_FINISHED; }

  /** @brief Angular velocity computed by update() (deg/s). */
  float getVelocity() override { return m_omega; }

  /** @brief Return the distance alongthe path */
  float getPosition() override { return m_position; }

  float getAcceleration() override { return 0.0f; }
  float getJerk() override { return 0.0f; }
  float speed_change_distance(float v_from, float v_to) const override {
    (void)v_from;
    (void)v_to;
    return 0.0f;
  }
  const char* getName() const override { return "CubicSpiral"; }

  /**
   * @brief Heading change from the start of the turn at a distance along the path.
   * @param s distance along the path, mm; clamped to 0..length
   * @return heading change, degrees, signed as the turn angle
   */
  float heading_at(float s) const {
    if (s <= 0.0f) {
      return 0.0f;
    }
    if (s >= m_length) {
      return m_angle;
    }
    float u = s / m_length;
    return m_angle * u * u * (3.0f - 2.0f * u);
  }

  /** @brief Path length of the turn, mm. */
  float length() const { return m_length; }

 private:
  /** @brief update() in exact angle mode; see the class description. */
  ISR_SAFE void update_exact(float current_speed) {
    float s0 = m_position;
    float s1 = s0 + fabsf(current_speed) * m_dt;
    if (s1 >= m_length) {
      s1 = m_length;
      m_state = PS_FINISHED;
    }
    float omega = (heading_at(s1) - heading_at(s0)) / m_dt;
    m_omega = current_speed < 0.0f ? -omega : omega;
    m_position = s1;
  }

  float m_position = 0.0f;
  float m_length = 0.0f;
  float m_angle = 0.0f;
  float m_constant = 0.0f;
  bool m_exact = false;
  volatile float m_omega = 0.0f;
  volatile uint8_t m_state = PS_IDLE;
};
