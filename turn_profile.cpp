/**
 * @file turn_profile.cpp
 * @brief Shared turn geometry and leadout simulation.
 */

#include "turn_profile.h"

const TurnGeometry turnGeometry[TURN_TYPE_COUNT] = {
        {0, 0, 0, 90},     // SS90F
        {0, 0, 0, 180},    // SS180
        {0, 0, 0, 45},     // SD45
        {0, 0, 0, 135},    // SD135
        {90, 0, -45, 45},  // DS45
        {90, 0, -45, 135}, // DS135
        {90, 0, -45, 90},  // DD90
        {0, 0, 0, 90},     // SS90E
};

void simulate_leadout(RobotState &robot_state, float distance, Track &track) {
    robot_state.omega = 0;
    robot_state.phase = 4;
    double targetPos = robot_state.distance + distance;
    while (robot_state.distance < targetPos) {
        robot_state.update();
        track.add_record(robot_state);
    }
}
