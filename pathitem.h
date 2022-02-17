//
// Created by peter on 15/02/2022.
//

#ifndef QTTURNPROFILE_PATHITEM_H
#define QTTURNPROFILE_PATHITEM_H

#include <QGraphicsItem>
#include <QPainter>

class PathItem : public QGraphicsItem {
public:
    PathItem() {};

    QRectF boundingRect() const override;  // the scene needs to know this

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;


private:

};


#endif //QTTURNPROFILE_PATHITEM_H
