/**
 * @file turn_physics.h
 * @brief Turn profile simulation and analytic turn formulas, independent of Qt.
 *
 * Each profile simulates a turn at fixed time steps, recording the robot
 * state into a Track: the turn itself, then a straight leadout.
 *
 * Angles are in degrees and distances in mm. Headings use the simulation
 * convention of RobotState (positive angles turn clockwise on screen).
 */

#ifndef TURN_PHYSICS_H
#define TURN_PHYSICS_H

#include "track.h"

/// Turn profile types
enum Profile_t {
    PR_TRAPEZOID, PR_QUADRATIC, PR_SINUSOID, PR_CUBIC, PR_COUNT
};

/// Everything needed to simulate one turn
struct TurnInput {
    float start_x = 0;       ///< start position, mm
    float start_y = 0;       ///< start position, mm
    float start_angle = 0;   ///< start heading, degrees
    float turn_angle = 0;    ///< total turn angle, degrees
    float speed = 0;         ///< turn speed, mm/s
    float radius = 0;        ///< min radius of the constant arc, mm (segmented profiles)
    float lambda = 0;        ///< transition distance, mm (segmented profiles)
    float length = 0;        ///< spiral length, mm (cubic)
    float gamma = 0;         ///< speed variation fraction (cubic), 0 = constant speed
    float interval = 0.001f; ///< simulation time step, s
    float leadout = 100;     ///< straight distance recorded after the turn, mm
};

/**
 * @brief Simulate a turn and record it.
 *
 * The track is cleared first. Its turn end index marks the last turn record,
 * followed by the leadout records.
 *
 * @param profile turn profile
 * @param in turn parameters; fields not used by the profile are ignored
 * @param track receives the records
 */
void simulate_turn(Profile_t profile, const TurnInput &in, Track &track);

/**
 * @brief Largest lambda that leaves a non-negative constant radius arc.
 *
 * Each transition turns through k * lambda / radius radians, with k = 1/2
 * (trapezoid), 2/3 (quadratic) or 2/pi (sinusoid). Both transitions must fit
 * in the turn angle, so lambda <= angle * radius / (2k). A larger lambda makes
 * phase 3 overrun and the turn falls short.
 *
 * @param profile turn profile
 * @param radius constant arc radius, mm
 * @param turn_angle total turn angle, degrees
 * @return maximum lambda in mm; effectively unlimited for the cubic profile
 */
float max_lambda(Profile_t profile, float radius, float turn_angle);

/**
 * @brief Peak centripetal acceleration of a turn at a given speed.
 *
 * Each profile needs its own calculation:
 *  - trapezoid, quadratic, sinusoid: the peak is in the constant radius arc,
 *    a = v^2 / r. Valid while the arc exists (see max_lambda()).
 *  - cubic: curvature is (6 * angle / L) * f(1 - f), f = s / L, largest at
 *    mid turn, so a = v^2 * 1.5 * angle / L. Assumes gamma = 0 (constant speed).
 *
 * @param profile turn profile
 * @param speed turn speed, mm/s
 * @param radius constant arc radius, mm (segmented profiles)
 * @param length spiral length, mm (cubic)
 * @param turn_angle total turn angle, degrees
 * @return peak centripetal acceleration, mm/s/s
 */
float peak_acceleration(Profile_t profile, float speed, float radius, float length, float turn_angle);

/**
 * @brief Turn speed that gives a required peak centripetal acceleration.
 *
 * Inverse of peak_acceleration(). In the robot the speed is derived this way.
 *
 * @param profile turn profile
 * @param acceleration peak centripetal acceleration, mm/s/s
 * @param radius constant arc radius, mm (segmented profiles)
 * @param length spiral length, mm (cubic)
 * @param turn_angle total turn angle, degrees
 * @return turn speed, mm/s
 */
float speed_for_acceleration(Profile_t profile, float acceleration, float radius, float length, float turn_angle);

#endif  // TURN_PHYSICS_H
