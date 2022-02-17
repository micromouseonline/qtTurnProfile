#ifndef POSE_H
#define POSE_H

/***
 * Represents the pose of the robot which is its
 * position and heading in world coordinates
 */

class Pose {
public:
    Pose() : m_x(0), m_y(0), m_theta(0) {};

    Pose(float x, float y, float theta) : m_x(x), m_y(y), m_theta(theta) {};

    void set_pose(float x, float y, float theta) {
        m_x = x;
        m_y = y;
        m_theta = theta;
    }

    float x() const { return m_x; }

    float y() const { return m_y; }

    float theta() const { return m_theta; }

private:
    float m_x;
    float m_y;
    float m_theta;
};

#endif // POSE_H
