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

    qreal m_width = 80;  // the overall_width of the mouse
    qreal m_radius = 37;  // the distance between the mouse CG and the wheel
    qreal m_side_sensor_angle = 35;
    qreal m_front_sensor_angle = 80;

private:
    Pose m_pose{};

    void
    drawSensor(QPainter *painter, QPoint &sensorOrigin, qreal sensorAngle) const;
};

#endif  // GRAPHICMOUSE_H
