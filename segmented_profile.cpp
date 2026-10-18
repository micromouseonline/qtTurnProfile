/**
 * @file segmented_profile.cpp
 * @brief Shared simulation and formulas for three phase turn profiles.
 */

#include "segmented_profile.h"
#include <cmath>

std::vector<ParamDef> SegmentedProfile::params() const {
    return {
            {"radius", "Radius", "mm", 15, 270, 1, 0},
            {"lambda", "Lambda", "mm", 0, 200, 1, 0},
    };
}

void SegmentedProfile::simulate(const TurnSetup &setup, float speed, const std::vector<float> &values,
                                Track &track) const {
    float turnRadius = values[RADIUS];  // the minimum radius portion
    float turnSpeed = speed;            // constant throughout the turn
    float turnAngle = setup.turn_angle;
    float lambda = values[LAMBDA];      // distance of angular acceleration phase
    float startAngle = setup.start_angle;

    // these are derived values needed to execute the turn
    float endAngle = startAngle + turnAngle;
    float turnOmega = degrees(turnSpeed / turnRadius);

    float transitionAngle = transition_angle(lambda, turnOmega, turnSpeed);
    // angle turned during the constant omega phase
    float theta2 = turnAngle - 2 * transitionAngle;
    float midArcLength = turnSpeed * theta2 / turnOmega;
    float turnDistance = lambda + midArcLength + lambda;

    track.clear();
    RobotState robot_state(setup.start_x, setup.start_y, startAngle, turnSpeed, setup.interval);
    robot_state.set_slip(setup.slip);
    track.add_record(robot_state);

    // In the robot, take care that rounding errors do not preventit from
    // exactly reaching the total angle as omega gets small. a residual error of
    // 0.1 degrees is good enough to have the robot no more than 0.3mm off track
    // after 180 mm of forward travel.
    // You can carry residual errors through to the subsequent straight for
    // correction if desired.
    // the distance limit ends the turn if step size leaves theta short of the end;
    // beyond turnDistance omega goes negative and the loop would never finish
    while (robot_state.theta < endAngle - tolerance() && robot_state.distance < turnDistance) {
        if (robot_state.theta < startAngle + transitionAngle) {
            robot_state.omega = ramp(turnOmega, robot_state.distance, lambda);
            robot_state.phase = 1;
        } else if (robot_state.theta < (endAngle - transitionAngle)) {
            robot_state.omega = turnOmega;
            robot_state.phase = 2;
        } else {
            robot_state.omega = ramp(turnOmega, turnDistance - robot_state.distance, lambda);
            robot_state.phase = 3;
        }
        robot_state.update();
        track.add_record(robot_state);
    }
    track.set_turn_end();
    simulate_leadout(robot_state, setup.leadout, track);
}

float SegmentedProfile::peak_acceleration(const TurnSetup &setup, float speed,
                                          const std::vector<float> &values) const {
    (void) setup;
    return speed * speed / values[RADIUS];
}

float SegmentedProfile::speed_for_acceleration(const TurnSetup &setup, float acceleration,
                                               const std::vector<float> &values) const {
    (void) setup;
    return std::sqrt(acceleration * values[RADIUS]);
}

float SegmentedProfile::max_lambda(float radius, float turn_angle) const {
    float angle = radians(turn_angle);
    return lambda_limit() * angle * radius;
}

bool SegmentedProfile::constrain(const TurnSetup &setup, std::vector<float> &values) const {
    int limit = int(max_lambda(values[RADIUS], setup.turn_angle));
    if (values[LAMBDA] > limit) {
        values[LAMBDA] = limit;
        return true;
    }
    return false;
}

std::vector<SummaryLine> SegmentedProfile::summary(const TurnSetup &setup, const std::vector<float> &values) const {
    return {{"Lambda Max", double(int(max_lambda(values[RADIUS], setup.turn_angle))), 0, "mm"}};
}
