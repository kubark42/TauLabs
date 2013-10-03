/**
******************************************************************************
*
* @file       mapline.h
* @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2012.
* @author     Tau Labs, http://taulabs.org Copyright (C) 2013.
* @brief      A graphicsItem representing a line connecting 2 map points
* @see        The GNU Public License (GPL) Version 3
* @defgroup   OPMapWidget
* @{
*
*****************************************************************************/
/*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
* or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
* for more details.
*
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/
#ifndef MAPLINE_H
#define MAPLINE_H

#include "mapgraphicitem.h"
#include "mappointitem.h"

namespace mapcontrol
{
class HomeItem;

class MapLine : public QObject, public QGraphicsLineItem
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)
public:
    enum GraphicItemTypes {TYPE_PATHSEGMENTLINE = 20, TYPE_WAYPOINTLINE = 21};

    MapLine(MapPointItem *from, MapPointItem *to, MapGraphicItem *map, QColor color=Qt::green);
    MapLine(HomeItem *from, MapPointItem *to, MapGraphicItem *map, QColor color=Qt::green);
    virtual int type() const = 0;
    QPainterPath shape() const;
    void setColor(const QColor &color)
        { myColor = color; }
private:
    QGraphicsItem * source;
    QGraphicsItem * destination;
    MapGraphicItem * my_map;
    QPolygonF arrowHead;
    QColor myColor;
protected:
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
public slots:
    void refreshLocations();
    void pointdeleted();
    void setOpacitySlot(qreal opacity);
};
}

namespace mapcontrol
{
/**
 * @brief The PathSegmentLine class This class is a simple line and only differentiates between q_graphics elements
 */
class PathSegmentLine : public MapLine
{
public:
    PathSegmentLine(MapPointItem *from, MapPointItem *to, MapGraphicItem *map, QColor color=Qt::magenta):
        MapLine(from, to, map, color){}

    enum { Type = UserType + TYPE_PATHSEGMENTLINE };
    int type() const {return Type;}
};
}


namespace mapcontrol
{
/**
 * @brief The PathSegmentLine class This class is a simple line and only differentiates between q_graphics elements
 */
class WayPointLine : public MapLine
{
public:
    WayPointLine(MapPointItem *from, MapPointItem *to, MapGraphicItem *map, QColor color=Qt::green):
        MapLine(from, to, map, color){}

    enum { Type = UserType + TYPE_WAYPOINTLINE };
    int type() const {return Type;}
};
}

#endif // MAPLINE_H
