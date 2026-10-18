#include "track.h"
#include "robotstate.h"
#include <algorithm>

Track::Track() {
    clear();
}

void Track::clear() {
    m_record_count = 0;
    m_finish_index = 0;
    m_time.clear();
    m_distance.clear();
    m_pose.clear();
    m_alpha.clear();
    m_omega.clear();
    m_speed.clear();
    m_acceleration.clear();
    m_phase.clear();
}

void Track::add_record(RobotState robot_state) {
    m_time.push_back(robot_state.time);
    m_distance.push_back(robot_state.distance);
    m_pose.push_back(Pose(robot_state.x, robot_state.y, robot_state.theta));
    m_alpha.push_back(robot_state.alpha);
    m_omega.push_back(robot_state.omega);
    m_speed.push_back(robot_state.speed);
    m_acceleration.push_back(robot_state.acceleration);
    m_phase.push_back(robot_state.phase);
    m_record_count++;
}

float Track::get_max_alpha() const {
    if (m_alpha.empty()) {
        return 0;
    }
    return *std::max_element(m_alpha.begin(), m_alpha.end());
}

float Track::get_max_omega() const {
    if (m_omega.empty()) {
        return 0;
    }
    return *std::max_element(m_omega.begin(), m_omega.end());
}

float Track::get_max_acceleration() const {
    if (m_acceleration.empty()) {
        return 0;
    }
    return *std::max_element(m_acceleration.begin(), m_acceleration.end());
}

