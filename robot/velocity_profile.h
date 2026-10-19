//
// Created by peter on 11/04/25.
//
// Mirror of the mazerunner32 firmware file vehicle/planner/velocityprofile.h.
// Keep the interface identical so the robot profiles in this directory can be
// moved to the firmware unchanged.
//

#pragma once

#include <cstdint>

#ifndef CONTROL_LOOP_INTERVAL
#define CONTROL_LOOP_INTERVAL 0.001f
#endif

/**
 * @file velocity_profile.h
 * @brief Abstract base class for all velocity profiles.
 *
 * Concrete profiles (SimpleTrapezoid, CubicProfile, etc.) inherit from this
 * class and are owned by a static ProfilePool in Vehicle.  MotionDirector
 * holds only VelocityProfile references — it never owns or constructs profiles.
 */

enum class ProfileStatus : int { SUCCESS = 0, WARN_SHORT_MOVE = 1, ERR_MICRO_MOVE = -1, ERR_IMPOSSIBLE_DECEL = -2, ERR_INVALID_PARAMS = -3 };

class VelocityProfile {
 public:
  /** @brief Motion-phase state machine values. */
  enum State : uint8_t {
    PS_IDLE = 0,
    PS_ACTIVE,
    PS_ACCELERATING,
    PS_CRUISING,
    PS_BRAKING,
    PS_FINISHED,
  };

  /** @brief Constructor requiring the system control period.
   * @param sample_time The time between update() calls in seconds (e.g., 0.001f).
   */
  explicit VelocityProfile(float sample_time = CONTROL_LOOP_INTERVAL) : m_dt(sample_time) {}

  virtual ~VelocityProfile() = default;

  /** @brief Current position (mm or deg) since last init(). */
  virtual float getPosition() = 0;

  /** @brief Current velocity (mm/s or deg/s). */
  virtual float getVelocity() = 0;

  /** @brief Current acceleration magnitude. */
  virtual float getAcceleration() = 0;

  /** @brief Current jerk (zero for trapezoidal profiles). */
  virtual float getJerk() = 0;

  /**
   * @brief Initialise and start the profile.
   * @param distance     Target distance/angle (signed; sign sets direction).
   * @param start_speed  Starting speed (unused by SimpleTrapezoid).
   * @param top_speed    Peak speed.
   * @param final_speed  Speed at end of profile.
   * @param acceleration Rate of speed change.
   * @param jerk         Rate of acceleration change (ignored by trapezoidal).
   * @return ProfileStatus indicating success or the reason for failure/mitigation.
   */
  virtual ProfileStatus init(float distance, float start_speed, float top_speed, float final_speed, float acceleration, float jerk) = 0;

  /** @brief Advance the profile by one control tick.
   * Called from systick.
   * profiles that do not need current speed can mark it (void)curent_speed in their definition
   */
  virtual void update(float current_speed = 0) = 0;

  /** @brief Reset position, speed and state to idle. */
  virtual void reset() = 0;

  /** @brief True when the profile has reached its target distance. */
  virtual bool isComplete() = 0;

  /**
   * @brief Distance needed to change speed from v_from to v_to.
   * Uses the profile's own configured dynamics (accel, jerk, etc.).
   * Valid for both acceleration and deceleration.
   */
  virtual float speed_change_distance(float v_from, float v_to) const = 0;

  /** @brief Returns the debug name defined in the concrete class. */
  virtual const char* getName() const = 0;

 protected:
  const float m_dt;
};
