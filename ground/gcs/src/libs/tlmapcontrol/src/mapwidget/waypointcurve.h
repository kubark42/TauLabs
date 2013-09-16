/**
******************************************************************************
*
* @file       waypointcurve.h
* @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2013
* @brief      A graphicsItem representing a curve connecting 2 waypoints
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
#ifndef WAYPOINTCURVE_H
#define WAYPOINTCURVE_H

#include "waypointitem.h"
#include "maparc.h"

namespace mapcontrol
{

/**
 * @brief The WayPointCurve class draws an arc between two graphics items of a given
 * radius and direction of curvature.  It will display a red straight line if the
 * radius is insufficient to connect the two waypoints
 */
class WayPointCurve:public MapArc
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)
public:
    enum { Type = UserType + TYPE_WAYPOINTCURVE };
    WayPointCurve(WayPointItem *start, WayPointItem *dest,
                  double radius, bool clockwise,
                  MapGraphicItem * map, QColor color=Qt::green);
    int type() const;

private:
    QPolygonF arrowHead;

protected:
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

public slots:
    //! Called if the start or end point is destroyed
    void waypointdeleted();
};
}

#endif // WAYPOINTCURVE_H
