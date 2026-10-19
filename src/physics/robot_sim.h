/**
 * @file robot_sim.h
 * @brief Simulate a turn by running a robot angular profile tick by tick.
 *
 * Mirrors the firmware control loop (MotionDirector): each tick the angular
 * profile is updated with the forward speed, then its omega is applied for
 * one interval. The profile code is the code that runs on the robot (robot/).
 */

#ifndef ROBOT_SIM_H
#define ROBOT_SIM_H

#include <functional>
#include "robot/velocity_profile.h"
#include "turn_profile.h"

/**
 * @brief Run a started angular profile to completion and record the turn.
 *
 * Clears the track, records the start, then each tick: profile.update(speed),
 * omega = profile.getVelocity(), advance the robot state and record it, until
 * the profile is complete. Then marks the turn end and records the leadout.
 * The tick on which the profile completes is recorded as part of the turn.
 *
 * @param profile angular profile, already initialised for this turn with
 *        sample time setup.interval
 * @param active false if the profile init failed; then, or if the speed is
 *        not positive, only the start and the leadout are recorded
 * @param phase returns the phase to record for the current tick (1..3)
 * @param setup start pose, interval, leadout and slip model
 * @param speed constant forward speed, mm/s
 * @param track receives the records
 */
void simulate_robot_turn(VelocityProfile &profile, bool active, const std::function<int()> &phase, const TurnSetup &setup, float speed, Track &track);

#endif  // ROBOT_SIM_H
