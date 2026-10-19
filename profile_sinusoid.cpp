/**
 * @file profile_sinusoid.cpp
 * @brief Sinusoidal turn profile.
 */

#include <cmath>
#include "robot/hybrid_turn_profile.h"
#include "robot_sim.h"
#include "segmented_profile.h"

namespace {

/**
 *
 * Sinusoidal profile
 *
 * Like the trapezoidal turn, this turn has tyhree phases. The central constant
 * radius phase is the same but the transitions in and out have the angular
 * velocity follow a portion of a sinusoidal profile.
 *
 * This profile is intended to keep the forces on the tyres within its traction
 * circle at all times and provides a smooth transition as the robot reaches its
 * maximum angular velocity.
 *
 * Some builders specify a time for a reference turn rather than a distance
 * for this accelerating portion. that has some advantages but the time will
 * change as the robot tries to run the turn faster. A little bit of calculation
 * reveals that the distance always comes out the same so it is just easier
 * to specify a distance rather than a time and the resulting turn path
 * is easily made invariant with robot speed.
 *
 * After the constant arc, the robot 'unwinds' the turn with another sinusoidal
 * change to angular velocity
 *
 * Four parameters are needed to define the turn:
 *  - turn minimum radius - that is for the constant arc portion, in mm
 *  - total turn angle - in degrees
 *  - lambda - the robot transitions distance in mm
 *  - speed - the robot forward velocity. Constant through the turn
 *
 * Everything else is calculated from that.
 *
 * For this turn type only, setting the lambda to be the same as
 * the pi times mouse radius causes the peak lateral and forward wheel accelerations
 * to be about the same
 */
class SinusoidProfile : public SegmentedProfile {
 public:
  std::string name() const override { return "Sinusoid"; }

  std::string description() const override { return "Sinusoidal change of angular velocity in the transitions"; }

  /**
   * Speeds give a peak centripetal acceleration of 9800 mm/s/s:
   * speed = sqrt(9800 * radius), rounded to the nearest mm/s.
   */
  TurnDefaults defaults(Turn_t turn) const override {
    // offset, speed, {radius, lambda}
    static const TurnDefaults table[TURN_TYPE_COUNT] = {
        {150, 1048, {112, 96}},  // SS90F
        {160, 913, {85, 123}},   // SS180
        {160, 1038, {110, 65}},  // SD45
        {150, 902, {83, 90}},    // SD135
        {75, 1084, {120, 66}},   // DS45
        {105, 885, {80, 90}},    // DS135
        {105, 852, {74, 78}},    // DD90
        {75, 741, {56, 47}},     // SS90E
    };
    return table[turn];
  }

  /**
   * Runs the robot's HybridTurnProfile (robot/hybrid_turn_profile.h) with
   * blend 0, which is this profile exactly: phases on distance along the
   * path, not on heading, one tick per interval at constant speed.
   */
  void simulate(const TurnSetup &setup, float speed, const std::vector<float> &values, Track &track) const override {
    HybridTurnProfile profile(setup.interval);
    ProfileStatus status = profile.init_turn(setup.turn_angle, values[RADIUS], values[LAMBDA], 0.0f);
    simulate_robot_turn(profile, status == ProfileStatus::SUCCESS, [&] { return profile.phase(); }, setup, speed, track);
  }

 protected:
  float transition_angle(float lambda, float turn_omega, float speed, const std::vector<float> &values) const override {
    (void)values;
    return float(2 * lambda * turn_omega / PI / speed);
  }

  float ramp(float turn_omega, float x, float lambda, const std::vector<float> &values) const override {
    (void)values;
    return float(turn_omega * sin((PI / 2) * x / lambda));
  }

  float lambda_limit(const std::vector<float> &values) const override {
    (void)values;
    return float(PI / 4);
  }
};

}  // namespace

/// The single Sinusoidal profile instance, for the registry
const TurnProfile &sinusoid_profile() {
  static const SinusoidProfile profile;
  return profile;
}
