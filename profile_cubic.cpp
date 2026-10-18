/**
 * @file profile_cubic.cpp
 * @brief Cubic spiral turn profile.
 */

#include "turn_profile.h"
#include <cmath>

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
 *    turnRadius = 4 * length / 6 / RADIANS(turnAngle);
 *    turnAcceleration = turnSpeed * turnSpeed * 6 * RADIANS(turnAngle) / 4 / length;
 *    turnAlpha = turnSpeed * k * length;
 *
 * Speed is constant throughout the turn.
 ***/
class CubicProfile : public TurnProfile {
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
            {150, 1016, {248}},  // SS90F
            {160, 877, {370}},  // SS180
            {160, 1056, {134}},  // SD45
            {150, 859, {266}},  // SD135
            {75, 1094, {144}},  // DS45
            {105, 844, {257}},  // DS135
            {105, 848, {173}},  // DD90
            {75, 718, {124}},  // SS90E
        };
        return table[turn];
    }

    void simulate(const TurnSetup &setup, float speed, const std::vector<float> &values,
                  Track &track) const override {
        float turnSpeed = speed;
        float turnAngle = setup.turn_angle;
        float startAngle = setup.start_angle;

        float length = values[LENGTH];
        // k is the constant used to obtain the curvature of the path
        float k = 6.0f * (turnAngle) / (length * length * length);

        track.clear();
        RobotState robot_state(setup.start_x, setup.start_y, startAngle, turnSpeed, setup.interval);
        track.add_record(robot_state);
        // perform the turn;
        // unlike the other turn types, the cubic spiral is based on distance
        // rather than the turn angle.
        // TODO: is this a problem if there is wheel slip?
        while (robot_state.distance < length) {
            float frac = robot_state.distance / length;
            float omega = robot_state.speed * k * robot_state.distance * (length - robot_state.distance);
            robot_state.omega = omega;
            if (frac < 0.5) {
                robot_state.phase = 1;
            } else {
                robot_state.phase = 3;
            }
            robot_state.update();
            track.add_record(robot_state);
        }
        track.set_turn_end();
        simulate_leadout(robot_state, setup.leadout, track);
    }

    /// Curvature is (6 * angle / L) * f(1 - f), f = s / L, largest at mid turn: a = v^2 * 1.5 * angle / L
    float peak_acceleration(const TurnSetup &setup, float speed, const std::vector<float> &values) const override {
        float angle = RADIANS(setup.turn_angle);
        return speed * speed * 1.5f * angle / values[LENGTH];
    }

    float speed_for_acceleration(const TurnSetup &setup, float acceleration,
                                 const std::vector<float> &values) const override {
        float angle = RADIANS(setup.turn_angle);
        return std::sqrt(acceleration * values[LENGTH] / (1.5f * angle));
    }
};

}  // namespace

/// The single cubic spiral profile instance, for the registry
const TurnProfile &cubic_profile() {
    static const CubicProfile profile;
    return profile;
}
