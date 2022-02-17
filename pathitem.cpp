//
// Created by peter on 15/02/2022.
//

#include "pathitem.h"

void PathItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    QPen pen;
    pen.setWidth(10);
    pen.setColor(Qt::yellow);
    // center mark
    painter->setPen(pen);
    painter->drawLine(0, 0, 100, 200);
}

QRectF PathItem::boundingRect() const {
    return {0, 0, 200, 200};
}
