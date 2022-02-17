#include "mouseitem.h"
#include <cmath>

#define PI 3.14159265359
#define RADIANS(x) ((x)*PI / 180.0)
#define DEGREES(x) ((x)*180.0 / PI)

MouseItem::MouseItem() {
    setSensorAngle(35);
}

QRectF MouseItem::boundingRect() const {
    return {-160, -150, 320, 300};
}

void MouseItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    (void) widget;
    (void) option;
    setPos(m_pose.x(), m_pose.y());
    setRotation(m_pose.theta());
    QPen pen;
    pen.setWidth(2);
    pen.setColor(Qt::cyan);
    // center mark
    painter->setPen(pen);
    painter->drawEllipse(-5, -5, 10, 10);
    // body
    int wheel_width = 8;
    int left_outer = -m_width / 2;
    int left_inner = left_outer + wheel_width + 2;
    int right_outer = m_width / 2;
    int right_inner = right_outer - wheel_width - 2;
    int recess_offs = 32;
    painter->drawArc(left_outer, -recess_offs - m_width / 2, m_width, m_width, 0, 180 * 16); // front arc
    painter->drawLine(left_outer, -recess_offs, left_inner, -recess_offs); // wheel step left
    painter->drawLine(right_outer, -recess_offs, right_inner, -recess_offs);           // wheel step right
    painter->drawLine(left_inner, -recess_offs, left_inner, recess_offs);   // wheel well left side
    painter->drawLine(right_inner, -recess_offs, right_inner, recess_offs);   // wheel well right side
    painter->drawLine(left_inner, recess_offs, right_inner, recess_offs);   // rear edge
    // sensors
    pen.setColor(Qt::green);
    painter->setPen(pen);
    painter->setPen(QColor(0, 255, 0, 192));
    // diagonals
    qreal halfAngle = 3;
    qreal sensorLength = 160;

    QPoint sensorOrigin;
    // side _sensors
    sensorOrigin = QPoint(12, -55);
    drawSensor(painter, sensorOrigin, m_side_sensor_angle);
    sensorOrigin = QPoint(-12, -55);
    drawSensor(painter, sensorOrigin, 180 - m_side_sensor_angle);
    // front sensors
    sensorOrigin = QPoint(30, -37);
    drawSensor(painter, sensorOrigin, m_front_sensor_angle);
    sensorOrigin = QPoint(-30, -37);
    drawSensor(painter, sensorOrigin, 180 - m_front_sensor_angle);

    // tyres
    pen.setWidthF(wheel_width);
    pen.setColor(QColor(255, 255, 255, 220));
    pen.setCapStyle(Qt::RoundCap);
    painter->setPen(pen);
    painter->drawLine(-m_radius, -24, -m_radius, -6);
    painter->drawLine(-m_radius, 4, -m_radius, 22);
    painter->drawLine(m_radius, -24, m_radius, -6);
    painter->drawLine(m_radius, 4, m_radius, 22);
}

void MouseItem::drawSensor(QPainter *painter, QPoint &sensorOrigin, qreal sensorAngle) const {
    float sensorLength = 160;
    float halfAngle = 3;
    qreal rayCos;
    qreal raySin;
    QPointF lineEnd;
    rayCos = sensorLength * cos(RADIANS(sensorAngle - halfAngle));
    raySin = sensorLength * sin(RADIANS(sensorAngle - halfAngle));
    lineEnd = sensorOrigin + QPointF(rayCos, -raySin);
    painter->drawLine(sensorOrigin, lineEnd);

    rayCos = sensorLength * cos(RADIANS(sensorAngle));
    raySin = sensorLength * sin(RADIANS(sensorAngle));
    lineEnd = sensorOrigin + QPointF(rayCos, -raySin);
    painter->drawLine(sensorOrigin, lineEnd);

    rayCos = sensorLength * cos(RADIANS(sensorAngle + halfAngle));
    raySin = sensorLength * sin(RADIANS(sensorAngle + halfAngle));
    lineEnd = sensorOrigin + QPointF(rayCos, -raySin);
    painter->drawLine(sensorOrigin, lineEnd);
}

void MouseItem::setPose(Pose pose) {
    m_pose = pose;

}

void MouseItem::setSensorAngle(qreal angle) {
    m_side_sensor_angle = angle;
}
