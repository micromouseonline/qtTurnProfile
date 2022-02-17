//
// Created by peter on 15/02/2022.
//

#ifndef QTTURNPROFILE_ROBOTSTATE_H
#define QTTURNPROFILE_ROBOTSTATE_H

#include <cmath>
#include "pose.h"

#define PI 3.14159265359
#define RADIANS(x) ((x)*PI / 180.0)
#define DEGREES(x) ((x)*180.0 / PI)

class RobotState {
public:
    RobotState() {
        reset();
    }

    RobotState(float start_x, float start_y, float start_angle, float start_speed, float interval) :
            x(start_x), y(start_y), theta(start_angle), speed(start_speed), m_interval(interval) {

    }

    explicit RobotState(float interval) {
        reset();
        set_interval(interval);
    }

    void reset() {
        time = 0;
        distance = 0;
        x = 0;
        y = 0;
        theta = 0;
        alpha = 0;
        omega = 0;
        speed = 0;
        acceleration = 0;
        phase = 0;
        m_oldOmega = 0;
        m_interval = 0;
    }

    void set_interval(float interval) {
        m_interval = interval;
    }

    void update() {
        float delta = speed * m_interval;
        time += m_interval;
        distance += delta;
        x += -delta * cos(PI / 2 + RADIANS(theta));
        y += -delta * sin(PI / 2 + RADIANS(theta));
        theta += omega * m_interval;
        pose.set_pose(x, y, theta);
        acceleration = speed * RADIANS(omega);
        alpha = omega - m_oldOmega;
        m_oldOmega = omega;
    }

    float time{};
    float distance{};
    float x{};
    float y{};
    float theta{};
    Pose pose{};
    float alpha{};
    float omega{};
    float speed{};
    float acceleration{};
    int phase{};
private:
    float m_oldOmega{};
    float m_interval{};
};


#endif //QTTURNPROFILE_ROBOTSTATE_H
