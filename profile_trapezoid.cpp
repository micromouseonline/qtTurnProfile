/**
 * @file profile_trapezoid.cpp
 * @brief Trapezoidal turn profile.
 */

#include "segmented_profile.h"
#include <cmath>

namespace {

/**
 *
 * The trapezoidal turn has the robot transition from straight-line motion to
 * a constant angular velocity arc by linearly increasing the angular velocity
 * over a fixed distance (lambda). During this portion of the turn, the robot
 * path follows a clothoid curve.
 *
 * Some builders specify a time for a reference turn rather than a distance
 * for this accelerating portion. that has some advantages but the time will
 * change as the robot tries to run the turn faster. A little bit of calculation
 * reveals that the distance always comes out the same so it is just easier
 * to specify a distance rather than a time and the resulting turn path
 * is easily made invariant with robot speed.
 *
 * After the constant arc, the robot 'unwinds' the turn with another linear
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
 * the mouse radius causes the peak lateral and forward wheel accelerations
 * to be about the same
 */
class TrapezoidProfile : public SegmentedProfile {
public:
    std::string name() const override { return "Trapezoid"; }

    std::string description() const override { return "Linear change of angular velocity in the transitions (clothoid entry and exit)"; }

    /**
     * Speeds give a peak centripetal acceleration of 9800 mm/s/s:
     * speed = sqrt(9800 * radius), rounded to the nearest mm/s.
     */
    TurnDefaults defaults(Turn_t turn) const override {
        // offset, speed, {radius, lambda}
        static const TurnDefaults table[TURN_TYPE_COUNT] = {
            {150, 970, {96, 100}},  // SS90F
            {160, 896, {82, 127}},  // SS180
            {160, 934, {89, 65}},  // SD45
            {150, 869, {77, 90}},  // SD135
            {75, 965, {95, 70}},  // DS45
            {105, 846, {73, 90}},  // DS135
            {105, 773, {61, 80}},  // DD90
            {75, 686, {48, 50}},  // SS90E
        };
        return table[turn];
    }

protected:
    float transition_angle(float lambda, float turn_omega, float speed,
                           const std::vector<float> &values) const override {
        (void) values;
        return lambda * turn_omega / (speed * 2);
    }

    float ramp(float turn_omega, float x, float lambda, const std::vector<float> &values) const override {
        (void) values;
        return turn_omega * x / lambda;
    }

    float lambda_limit(const std::vector<float> &values) const override {
        (void) values;
        return 1.0f;
    }

    /// 0.1 degrees, as in the original trapezoid simulation
    double tolerance() const override { return 0.1; }
};

}  // namespace

/// The single Trapezoidal profile instance, for the registry
const TurnProfile &trapezoid_profile() {
    static const TrapezoidProfile profile;
    return profile;
}
