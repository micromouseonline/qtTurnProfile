/**
 * @file profile_hybrid.cpp
 * @brief Hybrid turn profile: linear then sinusoidal transitions.
 */

#include <algorithm>
#include <cmath>
#include "robot/hybrid_turn_profile.h"
#include "robot_sim.h"
#include "segmented_profile.h"

namespace {

/***
 *
 * HYBRID PROFILE
 *
 * Three phases like the sinusoid, but each transition starts with a linear
 * rise of omega (constant angular acceleration, as the trapezoid) and then
 * follows a quarter sine onto the arc. The sine part is the shape that keeps
 * the combined tyre load (centripetal plus wheel acceleration) constant;
 * the linear part keeps the wheel acceleration at its peak. With both at
 * their limits this is the fastest transition for given peak centripetal
 * acceleration, wheel acceleration and combined load (see docs/turn-analysis.md).
 *
 * Parameters:
 *  - radius: constant arc radius, mm
 *  - lambda: transition length (linear + sine), mm
 *  - blend f: fraction of the arc omega at which the linear part hands over
 *    to the sine. 0 gives the sinusoid profile exactly; near 1 the
 *    transition approaches the trapezoid.
 *
 * In units of the arc omega W, with phi1 = asin(f) and c = cos(phi1):
 *   linear part, 0 <= x <= a:   omega = W * f * x / a
 *   sine part,   a <= x <= lambda: omega = W * sin(phi1 + (x - a) / l)
 * The sine ends at phi = pi/2 (zero slope into the arc). Matching the slopes
 * at x = a (f / a = c / l) gives
 *   l = lambda / (f / c + pi/2 - phi1),   a = f * l / c
 * so the angular acceleration is continuous except at the turn entry and
 * exit, like the other segmented profiles.
 *
 * Angle turned in one transition, in units of W * lambda / v:
 *   k = (f * a / 2 + l * c) / lambda
 * (2 / pi for f = 0, the sinusoid; 1/2 as f -> 1, the trapezoid).
 ***/
class HybridProfile : public SegmentedProfile {
 public:
  enum { BLEND = 2 };

  std::string name() const override { return "Hybrid"; }

  std::string description() const override { return "Linear then sinusoidal change of angular velocity in the transitions"; }

  std::vector<ParamDef> params() const override {
    std::vector<ParamDef> defs = SegmentedProfile::params();
    defs.push_back({"blend", "Blend", "fraction of arc omega where the linear part ends", 0, 0.95, 0.01, 2});
    return defs;
  }

  /**
   * Fitted to the same footprint as the cubic defaults (same offset, exit on
   * the cubic's exit line): blend and lambda give the shortest time with
   * peak wheel acceleration and combined load no higher than the cubic's
   * (see docs/turn-analysis.md). Speeds give a peak centripetal acceleration of
   * 9800 mm/s/s: speed = sqrt(9800 * radius), rounded to the nearest mm/s.
   */
  TurnDefaults defaults(Turn_t turn) const override {
    // offset, speed, {radius, lambda, blend}
    static const TurnDefaults table[TURN_TYPE_COUNT] = {
        {150, 1071, {117, 73, 0.80f}},  // SS90F
        {160, 918, {86, 105, 0.85f}},   // SS180
        {160, 1093, {122, 52, 0.15f}},  // SD45
        {150, 902, {83, 81, 0.70f}},    // SD135
        {75, 1129, {130, 54, 0.00f}},   // DS45
        {105, 885, {80, 82, 0.70f}},    // DS135
        {105, 885, {80, 60, 0.50f}},    // DD90
        {75, 741, {56, 48, 0.15f}},     // SS90E
    };
    return table[turn];
  }

  /**
   * Runs the robot's HybridTurnProfile (robot/hybrid_turn_profile.h): phases
   * on distance along the path, not on heading, one tick per interval at
   * constant speed.
   */
  void simulate(const TurnSetup &setup, float speed, const std::vector<float> &values, Track &track) const override {
    HybridTurnProfile profile(setup.interval);
    ProfileStatus status = profile.init_turn(setup.turn_angle, values[RADIUS], values[LAMBDA], values[BLEND]);
    simulate_robot_turn(profile, status == ProfileStatus::SUCCESS, [&] { return profile.phase(); }, setup, speed, track);
  }

 protected:
  /// Shape of a transition, in units of lambda; see the class comment
  struct Shape {
    float f;     ///< blend, omega fraction at the end of the linear part
    float phi1;  ///< sine phase at the start of the sine part, rad
    float l;     ///< sine length scale / lambda
    float a;     ///< linear part length / lambda
    float k;     ///< transition angle / (W * lambda / v)
  };

  static Shape shape(const std::vector<float> &values) {
    Shape s{};
    s.f = std::clamp(values[BLEND], 0.0f, 0.95f);
    s.phi1 = std::asin(s.f);
    float c = std::cos(s.phi1);
    s.l = 1 / (s.f / c + float(PI) / 2 - s.phi1);
    s.a = s.f * s.l / c;
    s.k = s.f * s.a / 2 + s.l * c;
    return s;
  }

  float transition_angle(float lambda, float turn_omega, float speed, const std::vector<float> &values) const override {
    return shape(values).k * lambda * turn_omega / speed;
  }

  float ramp(float turn_omega, float x, float lambda, const std::vector<float> &values) const override {
    Shape s = shape(values);
    float u = x / lambda;
    if (u <= s.a) {
      return s.a > 0 ? turn_omega * s.f * u / s.a : 0;
    }
    return turn_omega * std::sin(s.phi1 + (u - s.a) / s.l);
  }

  float lambda_limit(const std::vector<float> &values) const override { return 1 / (2 * shape(values).k); }
};

}  // namespace

/// The single hybrid profile instance, for the registry
const TurnProfile &hybrid_profile() {
  static const HybridProfile profile;
  return profile;
}
