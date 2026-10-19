#pragma once

#include <cmath>
#include "robot_compat.h"  // firmware: "interfaces/isr.h" and "utils/core.h"
#include "velocity_profile.h"

/**
 * @file hybrid_turn_profile.h
 * @brief Angular velocity profile for hybrid (and sinusoid) smooth turns.
 *
 * Written to run unchanged in the mazerunner32 firmware alongside
 * CubicProfile; the app runs the same code (see cubic_profile.h).
 *
 * The path is set by its curvature as a function of distance s along it,
 * independent of speed. Five segments:
 *   1. linear rise of curvature        0 .. a
 *   2. quarter sine onto the arc       a .. lambda
 *   3. constant radius arc             lambda .. lambda + arc
 *   4, 5. mirror of 2 and 1            .. length
 * In units of the arc curvature 1/R, with f = blend, phi1 = asin(f),
 * c = cos(phi1):
 *   linear: kappa = f * x / a
 *   sine:   kappa = sin(phi1 + (x - a) / l)
 *   l = lambda / (f / c + pi/2 - phi1),  a = f * l / c
 * so the slope of curvature is continuous where the linear part meets the
 * sine, and the sine meets the arc with zero slope. Blend 0 is the pure
 * quarter sine (the sinusoid profile); near 1 it approaches the trapezoid.
 *
 * Heading along a transition, x from its zero curvature end:
 *   linear: Theta = f x^2 / (2 a R)
 *   sine:   Theta = f a / (2 R) + (l / R) (c - cos(phi1 + (x - a) / l))
 * Whole transition: k lambda / R, k = (f a / 2 + l c) / lambda. The arc
 * takes the rest of the angle, so arc = R * angle - 2 k lambda.
 *
 * Each tick the distance advances by |v| dt (v the commanded forward speed,
 * as CubicProfile) and the output is
 *   omega = (Theta(s1) - Theta(s0)) / dt
 * with the last step stopping at the end of the turn. The sum of omega * dt
 * is the turn angle exactly, whatever the speed, without aligning segment
 * ends with ticks. Cost per tick: one cosf in the sine segments.
 *
 * Usage:
 *   1. Call init_turn() at the start of a turn; init() is not supported.
 *   2. Call update() each control tick with the forward speed.
 *   3. Read getVelocity() for the angular speed (deg/s).
 *   4. Check isComplete() to detect end of turn.
 */
class HybridTurnProfile : public VelocityProfile {
 public:
  explicit HybridTurnProfile(float sample_time = CONTROL_LOOP_INTERVAL) : VelocityProfile(sample_time) {}

  /**
   * @brief Not supported: a hybrid turn needs more parameters than init() carries.
   * @return ProfileStatus::ERR_INVALID_PARAMS always; use init_turn()
   */
  ProfileStatus init(float distance, float start_speed, float top_speed, float final_speed, float acceleration, float jerk) override {
    (void)distance;
    (void)start_speed;
    (void)top_speed;
    (void)final_speed;
    (void)acceleration;
    (void)jerk;
    reset();
    return ProfileStatus::ERR_INVALID_PARAMS;
  }

  /**
   * @brief Set up and start a turn.
   *
   * All the trigonometry except one cosf per tick is done here.
   *
   * @param angle  turn angle, degrees; the sign sets the direction
   * @param radius radius of the constant arc, mm; > 0
   * @param lambda length of each transition, mm; >= 0. No more than
   *               R * |angle| / (2 k), or the transitions overlap
   * @param blend  fraction of the arc curvature where the linear part ends,
   *               clamped to 0 .. 0.95; 0 = sinusoid
   * @return SUCCESS, or ERR_INVALID_PARAMS (profile left idle) if the radius
   *         is not positive, lambda is negative or the transitions do not fit
   */
  ProfileStatus init_turn(float angle, float radius, float lambda, float blend) {
    reset();
    if (radius <= 0.0f || lambda < 0.0f) {
      return ProfileStatus::ERR_INVALID_PARAMS;
    }
    float f = blend < 0.0f ? 0.0f : (blend > 0.95f ? 0.95f : blend);
    float phi1 = asinf(f);
    float c = cosf(phi1);
    float l = lambda / (f / c + HALF_PI - phi1);
    float a = f * l / c;
    float abs_angle = fabsf(angle) * RADIANS_PER_DEGREE;
    float transition_theta = (f * a / 2.0f + l * c) / radius;
    float arc = radius * (abs_angle - 2.0f * transition_theta);
    if (arc < 0.0f) {
      // allow for float rounding when lambda is at its limit
      if (arc < -0.01f) {
        return ProfileStatus::ERR_INVALID_PARAMS;
      }
      arc = 0.0f;
    }
    m_sign = angle < 0.0f ? -1.0f : 1.0f;
    m_angle = abs_angle;
    m_radius = radius;
    m_lambda = lambda;
    m_f = f;
    m_phi1 = phi1;
    m_c = c;
    m_l = l;
    m_a = a;
    m_transition_theta = transition_theta;
    m_arc = arc;
    m_length = 2.0f * lambda + arc;
    m_position = 0.0f;
    m_omega = 0.0f;
    m_state = PS_ACTIVE;
    return ProfileStatus::SUCCESS;
  }

  /** @brief Advance the turn by one control tick. */
  ISR_SAFE void update(float current_speed) override {
    if (m_state != PS_ACTIVE) {
      return;
    }
    float s0 = m_position;
    float s1 = s0 + fabsf(current_speed) * m_dt;
    if (s1 >= m_length) {
      s1 = m_length;
      m_state = PS_FINISHED;
    }
    float omega = m_sign * (heading(s1) - heading(s0)) * DEGREES_PER_RADIAN / m_dt;
    m_omega = current_speed < 0.0f ? -omega : omega;
    m_position = s1;
  }

  void reset() override {
    m_omega = 0.0f;
    m_state = PS_IDLE;
  }

  bool isComplete() override { return m_state == PS_FINISHED; }

  /** @brief Angular velocity computed by update() (deg/s). */
  float getVelocity() override { return m_omega; }

  /** @brief Distance along the path since init_turn(), mm. */
  float getPosition() override { return m_position; }

  float getAcceleration() override { return 0.0f; }
  float getJerk() override { return 0.0f; }
  float speed_change_distance(float v_from, float v_to) const override {
    (void)v_from;
    (void)v_to;
    return 0.0f;
  }
  const char* getName() const override { return "HybridTurn"; }

  /**
   * @brief Heading change from the start of the turn at a distance along the path.
   * @param s distance along the path, mm; clamped to 0..length
   * @return heading change, degrees, signed as the turn angle
   */
  float heading_at(float s) const { return m_sign * heading(s) * DEGREES_PER_RADIAN; }

  /**
   * @brief Path curvature at a distance along the path.
   * @param s distance along the path, mm
   * @return curvature, 1/mm, signed as the turn angle; 0 outside the turn
   */
  float curvature_at(float s) const {
    if (s < 0.0f || s > m_length) {
      return 0.0f;
    }
    float x = s <= m_lambda + m_arc ? s : m_length - s;
    float k = 1.0f;
    if (x < m_lambda) {
      k = x <= m_a ? (m_a > 0.0f ? m_f * x / m_a : 0.0f) : sinf(m_phi1 + (x - m_a) / m_l);
    }
    return m_sign * k / m_radius;
  }

  /** @brief Path length of the turn, mm. */
  float length() const { return m_length; }

  /** @brief Length of each transition, mm. */
  float transition_length() const { return m_lambda; }

  /**
   * @brief Segment of the turn at the current position.
   * @return 1 entry transition, 2 constant arc, 3 exit transition
   */
  int phase() const {
    if (m_position < m_lambda) {
      return 1;
    }
    return m_position < m_lambda + m_arc ? 2 : 3;
  }

 private:
  /**
   * @brief Heading in the entry transition, x from its start.
   * @param x distance into the transition, mm; 0 .. lambda
   * @return heading change, rad, unsigned
   */
  float transition_heading(float x) const {
    if (x <= m_a) {
      return m_a > 0.0f ? m_f * x * x / (2.0f * m_a * m_radius) : 0.0f;
    }
    return (m_f * m_a / 2.0f + m_l * (m_c - cosf(m_phi1 + (x - m_a) / m_l))) / m_radius;
  }

  /**
   * @brief Heading change from the start of the turn.
   * @param s distance along the path, mm; clamped to 0..length
   * @return heading change, rad, unsigned
   */
  float heading(float s) const {
    if (s <= 0.0f) {
      return 0.0f;
    }
    if (s >= m_length) {
      return m_angle;
    }
    if (s < m_lambda) {
      return transition_heading(s);
    }
    if (s <= m_lambda + m_arc) {
      return m_transition_theta + (s - m_lambda) / m_radius;
    }
    return m_angle - transition_heading(m_length - s);
  }

  float m_position = 0.0f;          ///< distance along the path, mm
  float m_length = 0.0f;            ///< total path length, mm
  float m_angle = 0.0f;             ///< turn angle magnitude, rad
  float m_sign = 1.0f;              ///< turn direction, +1 or -1
  float m_radius = 1.0f;            ///< arc radius, mm
  float m_lambda = 0.0f;            ///< transition length, mm
  float m_arc = 0.0f;               ///< constant arc length, mm
  float m_f = 0.0f;                 ///< blend
  float m_phi1 = 0.0f;              ///< sine phase where the linear part ends, rad
  float m_c = 1.0f;                 ///< cos(phi1)
  float m_l = 0.0f;                 ///< sine length scale, mm
  float m_a = 0.0f;                 ///< linear part length, mm
  float m_transition_theta = 0.0f;  ///< heading change in one transition, rad
  volatile float m_omega = 0.0f;
  volatile uint8_t m_state = PS_IDLE;
};
