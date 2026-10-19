/**
 * @file robot_sim.cpp
 * @brief Turn simulation driven by a robot angular profile.
 */

#include "robot_sim.h"

void simulate_robot_turn(VelocityProfile &profile, bool active, const std::function<int()> &phase, const TurnSetup &setup, float speed, Track &track) {
  track.clear();
  RobotState robot_state(setup.start_x, setup.start_y, setup.start_angle, speed, setup.interval);
  robot_state.set_slip(setup.slip);
  track.add_record(robot_state);
  if (active && speed > 0) {
    do {
      profile.update(speed);
      robot_state.omega = profile.getVelocity();
      robot_state.phase = phase();
      robot_state.update();
      track.add_record(robot_state);
    } while (!profile.isComplete());
  }
  track.set_turn_end();
  simulate_leadout(robot_state, setup.leadout, track);
}
