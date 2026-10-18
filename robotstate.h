//
// Created by peter on 15/02/2022.
//

#ifndef QTTURNPROFILE_ROBOTSTATE_H
#define QTTURNPROFILE_ROBOTSTATE_H

#include <cmath>
#include "angles.h"
#include "pose.h"

/// Standard gravity, mm/s/s
constexpr float GRAVITY = 9806.65f;

/**
 * @brief Simplified lateral tyre slip, shared by all profiles.
 *
 * Steady state, linear, no saturation: the slip angle between heading and
 * velocity is
 *
 *     slip = gradient * (a / g) / load_factor
 *
 * where a is the centripetal acceleration. Heading follows the commanded
 * omega exactly (gyro controlled); only the direction of travel changes, so
 * the path drifts to the outside of the turn.
 *
 * The gradient can be measured by running the robot straight across a slope
 * (no fan): gradient = measured crab angle / tan(slope angle).
 */
struct SlipParams {
    float gradient = 0;     ///< slip angle per g of lateral acceleration, deg/g; 0 = no slip
    float load_factor = 1;  ///< normal load / weight, (m*g + downforce) / (m*g); 1 = no fan
};

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
        slip = 0;
        phase = 0;
        m_oldOmega = 0;
        m_interval = 0;
    }

    void set_interval(float interval) {
        m_interval = interval;
    }

    /// Set the tyre slip model used by update()
    void set_slip(const SlipParams &slip_params) {
        m_slip = slip_params;
    }

    /**
     * @brief Advance one interval at the current speed and omega.
     *
     * The position moves along the velocity direction, theta - slip, where
     * slip is found from the centripetal acceleration (see SlipParams).
     */
    void update() {
        float delta = speed * m_interval;
        time += m_interval;
        distance += delta;
        slip = m_slip.gradient * (speed * float(radians(omega)) / GRAVITY) / m_slip.load_factor;
        float direction = theta - slip;
        x += -delta * cos(PI / 2 + radians(direction));
        y += -delta * sin(PI / 2 + radians(direction));
        theta += omega * m_interval;
        pose.set_pose(x, y, theta);
        acceleration = speed * radians(omega);
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
    float slip{};  ///< slip angle, degrees: heading minus velocity direction
    int phase{};
private:
    float m_oldOmega{};
    float m_interval{};
    SlipParams m_slip{};
};


#endif //QTTURNPROFILE_ROBOTSTATE_H
