#ifndef GRAPHICMOUSE_H
#define GRAPHICMOUSE_H

#include <QDebug>
#include <QGraphicsItem>
#include <QPainter>
#include "pose.h"

class MouseItem : public QGraphicsItem {
public:
    MouseItem();

    QRectF boundingRect() const override;  // the scene needs to know this
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;

    void setPose(Pose pose);

    void setSensorAngle(qreal angle);

    void setVelocity(qreal speed, qreal direction = 0);

    qreal m_width = 80;  // the overall_width of the mouse
    qreal m_radius = 37;  // the distance between the mouse CG and the wheel
    qreal m_side_sensor_angle = 35;
    qreal m_front_sensor_angle = 80;

    /// velocity arrow length per unit speed: 1000 mm/s is drawn as 100 mm
    static constexpr qreal velocityScale = 0.1;

private:
    Pose m_pose{};
    qreal m_speed = 0;      ///< speed in mm/s
    qreal m_direction = 0;  ///< velocity direction relative to heading, degrees

    void drawVelocity(QPainter *painter) const;

    void
    drawSensor(QPainter *painter, QPoint &sensorOrigin, qreal sensorAngle) const;
};

#endif  // GRAPHICMOUSE_H
