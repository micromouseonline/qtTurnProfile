/**
 * @file turn_physics.cpp
 * @brief Turn profile simulation and analytic turn formulas.
 */

#include "turn_physics.h"
#include <cmath>
#include "robotstate.h"

namespace {

/**
 * @brief Record a straight leadout after the turn.
 *
 * @param robot_state state at the end of the turn; advanced in place
 * @param distance leadout distance, mm
 * @param track receives the records
 */
void simulate_leadout(RobotState &robot_state, float distance, Track &track) {
    robot_state.omega = 0;
    robot_state.phase = 4;
    double targetPos = robot_state.distance + distance;
    while (robot_state.distance < targetPos) {
        robot_state.update();
        track.add_record(robot_state);
    }
}

/**
 * @brief What distinguishes one segmented (three phase) profile from another.
 */
struct SegmentedShape {
    /// angle turned during one transition, degrees, given lambda, turn omega (deg/s) and speed
    float (*transition_angle)(float lambda, float turn_omega, float speed);
    /// omega during a transition at distance x from its zero-omega end
    float (*ramp)(float turn_omega, float x, float lambda);
    /// the turn ends when theta is within this many degrees of the end angle
    double tolerance;
};

/**
 * @brief Simulate a segmented turn: ramp up, constant arc, ramp down, then leadout.
 *
 * Phase 1 increases omega from zero to the arc value over lambda, phase 2
 * holds it, phase 3 mirrors phase 1. Phases switch on the heading reached.
 */
void simulate_segmented(const SegmentedShape &shape, const TurnInput &in, Track &track) {
    float turnRadius = in.radius;  // the minimum radius portion
    float turnSpeed = in.speed;    // constant throughout the turn
    float turnAngle = in.turn_angle;
    float lambda = in.lambda;      // distance of angular acceleration phase
    float startAngle = in.start_angle;

    // these are derived values needed to execute the turn
    float endAngle = startAngle + turnAngle;
    float turnOmega = DEGREES(turnSpeed / turnRadius);

    float transitionAngle = shape.transition_angle(lambda, turnOmega, turnSpeed);
    // angle turned during the constant omega phase
    float theta2 = turnAngle - 2 * transitionAngle;
    float midArcLength = turnSpeed * theta2 / turnOmega;
    float turnDistance = lambda + midArcLength + lambda;

    track.clear();
    RobotState robot_state(in.start_x, in.start_y, startAngle, turnSpeed, in.interval);
    track.add_record(robot_state);

    // In the robot, take care that rounding errors do not preventit from
    // exactly reaching the total angle as omega gets small. a residual error of
    // 0.1 degrees is good enough to have the robot no more than 0.3mm off track
    // after 180 mm of forward travel.
    // You can carry residual errors through to the subsequent straight for
    // correction if desired.
    // the distance limit ends the turn if step size leaves theta short of the end;
    // beyond turnDistance omega goes negative and the loop would never finish
    while (robot_state.theta < endAngle - shape.tolerance && robot_state.distance < turnDistance) {
        if (robot_state.theta < startAngle + transitionAngle) {
            robot_state.omega = shape.ramp(turnOmega, robot_state.distance, lambda);
            robot_state.phase = 1;
        } else if (robot_state.theta < (endAngle - transitionAngle)) {
            robot_state.omega = turnOmega;
            robot_state.phase = 2;
        } else {
            robot_state.omega = shape.ramp(turnOmega, turnDistance - robot_state.distance, lambda);
            robot_state.phase = 3;
        }
        robot_state.update();
        track.add_record(robot_state);
    }
    track.set_turn_end();
    simulate_leadout(robot_state, in.leadout, track);
}

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
const SegmentedShape trapezoidShape = {
    [](float lambda, float turn_omega, float speed) { return lambda * turn_omega / (speed * 2); },
    [](float turn_omega, float x, float lambda) { return turn_omega * x / lambda; },
    0.1,
};

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
const SegmentedShape quadraticShape = {
    [](float lambda, float turn_omega, float speed) { return lambda * 2 * turn_omega / (speed * 3); },
    [](float turn_omega, float x, float lambda) {
        float t = x / lambda;
        return float(turn_omega * t * (2.0 - t));
    },
    0.01,
};

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
const SegmentedShape sineShape = {
    [](float lambda, float turn_omega, float speed) { return float(2 * lambda * turn_omega / PI / speed); },
    [](float turn_omega, float x, float lambda) { return float(turn_omega * sin((PI / 2) * x / lambda)); },
    0.01,
};

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
 * This version has a parameter, gamma, that allows the mouse to
 * enter the turn more quickly - braking into and accelerating
 * out of he turn. the calculation retains the peak centripetal
 * acceleration at the given value. While the technique has promise,
 * it relies on the inner wheel being able to brake much harder
 * than the standard turn.
 *
 * Setting gamma at zero maintains a constant speed throughout.
 ***/
void simulate_cubic(const TurnInput &in, Track &track) {
    float turnSpeed = in.speed;
    float turnAngle = in.turn_angle;
    float startAngle = in.start_angle;

    float length = in.length;
    float gamma = in.gamma;
    // k is the constant used to obtain the curvature of the path
    float k = 6.0f * (turnAngle) / (length * length * length);

    track.clear();
    RobotState robot_state(in.start_x, in.start_y, startAngle, turnSpeed, in.interval);
    track.add_record(robot_state);
    // perform the turn;
    // unlike the other turn types, the cubic spiral is based on distance
    // rather than the turn angle.
    // TODO: is this a problem if there is wheel slip?
    while (robot_state.distance < length) {
        // calcualate the speed adjustment if used
        float frac = robot_state.distance / length;
        float q = 4.0 * gamma * (frac - 1) * frac + 1 + gamma;
        robot_state.speed = turnSpeed * q;

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
    simulate_leadout(robot_state, in.leadout, track);
}

}  // namespace

void simulate_turn(Profile_t profile, const TurnInput &in, Track &track) {
    switch (profile) {
        case PR_CUBIC:
            simulate_cubic(in, track);
            break;
        case PR_SINUSOID:
            simulate_segmented(sineShape, in, track);
            break;
        case PR_QUADRATIC:
            simulate_segmented(quadraticShape, in, track);
            break;
        case PR_TRAPEZOID:
        default:
            simulate_segmented(trapezoidShape, in, track);
            break;
    }
}

float max_lambda(Profile_t profile, float radius, float turn_angle) {
    float angle = RADIANS(turn_angle);
    switch (profile) {
        case PR_TRAPEZOID:
            return angle * radius;
        case PR_QUADRATIC:
            return 0.75f * angle * radius;
        case PR_SINUSOID:
            return float(PI / 4) * angle * radius;
        default:
            return 1e9f;
    }
}

float peak_acceleration(Profile_t profile, float speed, float radius, float length, float turn_angle) {
    float angle = RADIANS(turn_angle);
    switch (profile) {
        case PR_CUBIC:
            return speed * speed * 1.5f * angle / length;
        default:
            return speed * speed / radius;
    }
}

float speed_for_acceleration(Profile_t profile, float acceleration, float radius, float length, float turn_angle) {
    float angle = RADIANS(turn_angle);
    switch (profile) {
        case PR_CUBIC:
            return std::sqrt(acceleration * length / (1.5f * angle));
        default:
            return std::sqrt(acceleration * radius);
    }
}
