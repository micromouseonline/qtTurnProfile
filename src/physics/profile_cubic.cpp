/**
 * @file profile_cubic.cpp
 * @brief Cubic spiral turn profile.
 */

#include <cmath>
#include "robot/cubic_profile.h"
#include "robot_sim.h"
#include "turn_profile.h"

namespace {

/***
 *
 * CUBIC PROFILE
 *
 * The cubic profile is completely different to the other types. It is not
 * executed in phases and the robot describes a completely smooth path from
 * start to finish.
 *
 * In the simulation, Dalpha is not needed because we read the turn length
 * directly from the UI.
 *
 * If there is only the start and end points (x1,y1) and (x2,y2) respectively,
 * then the length is calculated from those. Let d be the euclidean distance
 * between the points:
 *
 *  distance =  sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1))
 *
 * then
 *
 * length = distance / dalpha(turnAngle)
 *
 * where turnAngle is in radians, positive is anti-clockwise
 *
 * the performance metrics can be calculated from the curvature equations
 *    turnDistance = length;
 *    turnOmega = (turnSpeed * k * length * length / 4);
 *    turnRadius = 4 * length / 6 / radians(turnAngle);
 *    turnAcceleration = turnSpeed * turnSpeed * 6 * radians(turnAngle) / 4 / length;
 *    turnAlpha = turnSpeed * k * length;
 *
 * Speed is constant throughout the turn.
 ***/
class CubicSpiralProfile : public TurnProfile {
 public:
  enum { LENGTH = 0 };

  std::string name() const override { return "Cubic Spiral"; }

  std::string description() const override { return "Curvature varies smoothly along the whole turn, no phases"; }

  std::vector<ParamDef> params() const override {
    return {
        {"length", "Length", "mm", 50, 500, 1, 0},
    };
  }

  /**
   * Speeds give a peak centripetal acceleration of 9800 mm/s/s at the
   * minimum radius, length / (1.5 * angle in radians).
   */
  TurnDefaults defaults(Turn_t turn) const override {
    // offset, speed, {length}
    static const TurnDefaults table[TURN_TYPE_COUNT] = {
        {150, 1016, {246}},  // SS90F
        {160, 877, {370}},   // SS180
        {160, 1056, {134}},  // SD45
        {150, 859, {266}},   // SD135
        {75, 1094, {142}},   // DS45
        {105, 844, {257}},   // DS135
        {105, 848, {173}},   // DD90
        {75, 718, {123}},    // SS90E
    };
    return table[turn];
  }

  /**
   * Runs the robot's CubicProfile (robot/cubic_profile.h) in firmware mode,
   * one tick per interval at constant speed. Phase 1 is the first half of the
   * length, phase 3 the second.
   */
  void simulate(const TurnSetup &setup, float speed, const std::vector<float> &values, Track &track) const override {
    float length = values[LENGTH];
    CubicProfile profile(setup.interval);
    profile.init(length, 0, 0, 0, 0, setup.turn_angle);
    simulate_robot_turn(profile, true, [&] { return profile.getPosition() < length / 2 ? 1 : 3; }, setup, speed, track);
  }

  /// Curvature is (6 * angle / L) * f(1 - f), f = s / L, largest at mid turn: a = v^2 * 1.5 * angle / L
  float peak_acceleration(const TurnSetup &setup, float speed, const std::vector<float> &values) const override {
    float angle = radians(setup.turn_angle);
    return speed * speed * 1.5f * angle / values[LENGTH];
  }

  float speed_for_acceleration(const TurnSetup &setup, float acceleration, const std::vector<float> &values) const override {
    float angle = radians(setup.turn_angle);
    return std::sqrt(acceleration * values[LENGTH] / (1.5f * angle));
  }
};

}  // namespace

/// The single cubic spiral profile instance, for the registry
const TurnProfile &cubic_profile() {
  static const CubicSpiralProfile profile;
  return profile;
}
