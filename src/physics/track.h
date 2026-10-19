#ifndef TRACK_H
#define TRACK_H

/***
 * Represents the track made by the robot during a turn
 *
 **/

#include <vector>
#include "pose.h"
#include "robotstate.h"

class Track {
 public:
  Track();

  void clear();

  void add_record(RobotState robot_state);

  /// Mark the most recent record as the last record of the turn (before leadout).
  void set_turn_end() { m_finish_index = (m_record_count > 0) ? m_record_count - 1 : 0; }

  int record_count() const { return m_record_count; }

  bool empty() const { return m_record_count == 0; }

  int finish_index() const { return m_finish_index; }

  /// Distance at the end of the turn; 0 if the track is empty.
  float turn_length() const { return empty() ? 0 : m_distance[m_finish_index]; }

  /// Time at the end of the turn; 0 if the track is empty.
  float get_turn_time() const { return empty() ? 0 : m_time[m_finish_index]; }

  /// Speed at the last record (end of leadout); 0 if the track is empty.
  float get_exit_speed() const { return empty() ? 0 : m_speed.back(); }

  /// Pose at the end of the turn; origin if the track is empty.
  Pose get_exit_pose() const { return empty() ? Pose() : m_pose[m_finish_index]; }

  float get_max_alpha() const;

  float get_max_omega() const;

  float get_max_acceleration() const;

  /// Largest slip angle magnitude, degrees; 0 if the track is empty.
  float get_max_slip() const;

  float get_distance_at(int index) const { return m_distance[index]; }

  float get_omega_at(int index) const { return m_omega[index]; }

  float get_time_at(int index) const { return m_time[index]; }

  float get_speed_at(int index) const { return m_speed[index]; }

  float get_slip_at(int index) const { return m_slip[index]; }

  int get_phase_at(int index) const { return m_phase[index]; }

  Pose get_pose_at(int index) const { return m_pose[index]; }

 private:
  int m_record_count{};
  int m_finish_index{};
  std::vector<float> m_time;
  std::vector<float> m_distance;
  std::vector<Pose> m_pose;
  std::vector<float> m_alpha;
  std::vector<float> m_omega;
  std::vector<float> m_speed;
  std::vector<float> m_acceleration;
  std::vector<float> m_slip;
  std::vector<int> m_phase;
};

#endif  // TRACK_H
