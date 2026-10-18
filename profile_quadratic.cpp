/**
 * @file profile_quadratic.cpp
 * @brief Quadratic turn profile.
 */

#include "segmented_profile.h"
#include <cmath>

namespace {

/**
 *
 * The quadratic turn has three sections, or phases, like the trapezoidal turn.
 *
 * The difference is that the change in angular velocity is calculated as a
 * portion of a quadratic function. Like the trapezoidal version, the acceleration
 * is over a fixed distance.
 *
 * It is not clear that there is any practical advantage to this profile although
 * it does significantly ease the transition into and out of the constant radius
 * phase.
 *
 * Some builders specify a time for a reference turn rather than a distance
 * for this accelerating portion. that has some advantages but the time will
 * change as the robot tries to run the turn faster. A little bit of calculation
 * reveals that the distance always comes out the same so it is just easier
 * to specify a distance rather than a time and the resulting turn path
 * is easily made invariant with robot speed.
 *
 * After the constant arc, the robot 'unwinds' the turn with another quadratic
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
 * the distance between the wheels causes the peak lateral and forward
 * wheel accelerations to be about the same.
 */
class QuadraticProfile : public SegmentedProfile {
public:
    std::string name() const override { return "Quadratic"; }

    std::string description() const override { return "Quadratic change of angular velocity in the transitions"; }

    /**
     * Speeds give a peak centripetal acceleration of 9800 mm/s/s:
     * speed = sqrt(9800 * radius), rounded to the nearest mm/s.
     * SD45 lambda is 60: 65 exceeds the quadratic limit (64) at radius 110.
     */
    TurnDefaults defaults(Turn_t turn) const override {
        // offset, speed, {radius, lambda}
        static const TurnDefaults table[TURN_TYPE_COUNT] = {
            {150, 1048, {112, 100}},  // SS90F
            {160, 913, {85, 127}},  // SS180
            {160, 1038, {110, 60}},  // SD45
            {150, 902, {83, 90}},  // SD135
            {75, 1084, {120, 70}},  // DS45
            {105, 885, {80, 90}},  // DS135
            {105, 852, {74, 80}},  // DD90
            {75, 741, {56, 50}},  // SS90E
        };
        return table[turn];
    }

protected:
    float transition_angle(float lambda, float turn_omega, float speed) const override {
        return lambda * 2 * turn_omega / (speed * 3);
    }

    float ramp(float turn_omega, float x, float lambda) const override {
        float t = x / lambda;
        return float(turn_omega * t * (2.0 - t));
    }

    float lambda_limit() const override { return 0.75f; }
};

}  // namespace

/// The single Quadratic profile instance, for the registry
const TurnProfile &quadratic_profile() {
    static const QuadraticProfile profile;
    return profile;
}
