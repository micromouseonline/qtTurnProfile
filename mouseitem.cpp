#include "mouseitem.h"
#include <algorithm>
#include <cmath>
#include "angles.h"

MouseItem::MouseItem() {
  setSensorAngle(35);
}

QRectF MouseItem::boundingRect() const {
  // Sensor rays start at most hypot(12, 55) from the centre (side sensors) and
  // can point in any direction as the sensor angle changes, so cover a square
  // of that reach, plus pen width. The velocity arrow can be longer.
  qreal sensors = std::hypot(12.0, 55.0) + sensorLength + 2;
  qreal arrow = m_speed * velocityScale + 10;
  qreal r = std::max(sensors, arrow);
  return {-r, -r, 2 * r, 2 * r};
}

void MouseItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
  (void)widget;
  (void)option;
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
  painter->drawArc(left_outer, -recess_offs - m_width / 2, m_width, m_width, 0, 180 * 16);  // front arc
  painter->drawLine(left_outer, -recess_offs, left_inner, -recess_offs);                    // wheel step left
  painter->drawLine(right_outer, -recess_offs, right_inner, -recess_offs);                  // wheel step right
  painter->drawLine(left_inner, -recess_offs, left_inner, recess_offs);                     // wheel well left side
  painter->drawLine(right_inner, -recess_offs, right_inner, recess_offs);                   // wheel well right side
  painter->drawLine(left_inner, recess_offs, right_inner, recess_offs);                     // rear edge
  // sensors
  painter->setPen(QColor(0, 255, 0, 192));
  // diagonals
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

  drawVelocity(painter);
}

/**
 * @brief Draw the velocity vectors as arrows from the mouse centre.
 *
 * Two arrows of length m_speed * velocityScale: a faint one along the heading
 * (the velocity there would be without slip), then the actual velocity,
 * rotated from the heading by m_direction. Without slip they coincide.
 *
 * @param painter painter in item coordinates
 */
void MouseItem::drawVelocity(QPainter *painter) const {
  qreal length = m_speed * velocityScale;
  if (length < 1) {
    return;
  }
  drawArrow(painter, length, 0, QColor(255, 255, 255, 140));
  drawArrow(painter, length, m_direction, QColor(255, 140, 0));
}

/**
 * @brief Draw an arrow from the mouse centre.
 *
 * @param painter painter in item coordinates
 * @param length arrow length in mm
 * @param angle direction relative to the heading (-y in item coordinates),
 *        degrees, positive clockwise on screen
 * @param colour line colour
 */
void MouseItem::drawArrow(QPainter *painter, qreal length, qreal angle, const QColor &colour) const {
  QPen pen(colour);
  pen.setWidthF(2);
  pen.setCapStyle(Qt::RoundCap);
  painter->save();
  painter->setPen(pen);
  painter->rotate(angle);
  QPointF tip(0, -length);
  painter->drawLine(QPointF(0, 0), tip);
  painter->drawLine(tip, tip + QPointF(-5, 10));
  painter->drawLine(tip, tip + QPointF(5, 10));
  painter->restore();
}

void MouseItem::drawSensor(QPainter *painter, const QPoint &sensorOrigin, qreal sensorAngle) const {
  float halfAngle = 3;
  qreal rayCos;
  qreal raySin;
  QPointF lineEnd;
  rayCos = sensorLength * cos(radians(sensorAngle - halfAngle));
  raySin = sensorLength * sin(radians(sensorAngle - halfAngle));
  lineEnd = sensorOrigin + QPointF(rayCos, -raySin);
  painter->drawLine(sensorOrigin, lineEnd);

  rayCos = sensorLength * cos(radians(sensorAngle));
  raySin = sensorLength * sin(radians(sensorAngle));
  lineEnd = sensorOrigin + QPointF(rayCos, -raySin);
  painter->drawLine(sensorOrigin, lineEnd);

  rayCos = sensorLength * cos(radians(sensorAngle + halfAngle));
  raySin = sensorLength * sin(radians(sensorAngle + halfAngle));
  lineEnd = sensorOrigin + QPointF(rayCos, -raySin);
  painter->drawLine(sensorOrigin, lineEnd);
}

/**
 * @brief Move the mouse to a new pose in the scene.
 *
 * Position and rotation are applied here, not in paint(), so the item
 * geometry is correct before the scene repaints.
 *
 * @param pose position in mm and heading in degrees
 */
void MouseItem::setPose(Pose pose) {
  m_pose = pose;
  setPos(m_pose.x(), m_pose.y());
  setRotation(m_pose.theta());
}

/**
 * @brief Set the velocity shown by the arrow.
 *
 * @param speed speed in mm/s; arrow length is speed * velocityScale
 * @param direction velocity direction relative to the heading in degrees,
 *        positive clockwise on screen; 0 when there is no slip
 */
void MouseItem::setVelocity(qreal speed, qreal direction) {
  prepareGeometryChange();  // bounding rect depends on speed
  m_speed = speed;
  m_direction = direction;
}

void MouseItem::setSensorAngle(qreal angle) {
  m_side_sensor_angle = angle;
}
