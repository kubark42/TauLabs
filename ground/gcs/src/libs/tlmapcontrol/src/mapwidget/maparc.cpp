/**
******************************************************************************
*
* @file       maparc.cpp
* @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2013
* @brief      A graphicsItem representing an arc connecting 2 waypoints
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
#include "maparc.h"
#include <math.h>
#include "homeitem.h"

namespace mapcontrol
{

/**
 * @brief MapArc::MapArc Create the curve
 * @param start The starting location of the curve (will redraw if moved)
 * @param dest The ending location of the curve (will redraw if moved)
 * @param radius Radius of the curve
 * @param clockwise Whether to curve clockwise or counter (when going from start to finish) as view from above
 * @param map Handle to the map object
 * @param color Color of the curve
 */
MapArc::MapArc(MapPointItem *start, MapPointItem *dest, double curvature, bool clockwise, int numberOfOrbits, bool rank, MapGraphicItem *map, QColor color) :
    QGraphicsEllipseItem(map),
    my_map(map),
    myColor(color),
    m_start(start),
    m_dest(dest),
    m_curvature(curvature),
    m_clockwise(clockwise),
    m_numberOfOrbits(numberOfOrbits),
    m_rank(rank)
{
    connect(start, SIGNAL(relativePositionChanged(QPointF, MapPointItem*)), this, SLOT(refreshLocations()));
    connect(dest,  SIGNAL(relativePositionChanged(QPointF, MapPointItem*)), this, SLOT(refreshLocations()));
    connect(map, SIGNAL(childSetOpacity(qreal)),this,SLOT(setOpacitySlot(qreal)));
}


/**
 * @brief MapArc::refreshLocations Update the settings for the
 * arc when it is moved or the zoom changes
 */
void MapArc::refreshLocations()
{
    double pixels2meters = my_map->Projection()->GetGroundResolution(my_map->ZoomTotal(), m_start->Coord().Lat());
    double radius_px = fabs((1.0/m_curvature) / pixels2meters);

    // Pixel positions
    double oldPosition_px[2] = {m_start->pos().x(), m_start->pos().y()};
    double newPosition_px[2] = {m_dest->pos().x(),  m_dest->pos().y()};
    double arcCenter_px[2];

    arc_center_results ret = findArcCenter_px(oldPosition_px, newPosition_px, radius_px, m_curvature > 0, m_rank == true /*FIXME: THIS MUST NOT BE HARD CODED. PathSegmentActive::CURRENTARCRANK_MINOR is better, but it is impossible to include*/, arcCenter_px);
    if (ret != ARC_CENTER_FOUND) {
        qDebug() << "Arc center not found. Error code: 0x0" << ret;
//        Q_ASSERT(0);
//        return;
    }


    // Store the center
    center.setX(arcCenter_px[0]);
    center.setY(arcCenter_px[1]);

    double startAngle;
    double endAngle;
    if (m_rank == true || 1) { /*FIXME: THIS MUST NOT BE HARD CODED. PathSegmentActive::CURRENTARCRANK_MINOR is better, but it is impossible to include*/
        // Angles are left-handed vs. coordinate system
        startAngle = atan2(-(m_start->pos().y() - arcCenter_px[1]), m_start->pos().x() - arcCenter_px[0]);
        endAngle   = atan2(-(m_dest->pos().y() - arcCenter_px[1]),  m_dest->pos().x() - arcCenter_px[0]);
    } else {
        // Angles are left-handed vs. coordinate system
        startAngle = atan2(-(m_dest->pos().y() - arcCenter_px[1]),  m_dest->pos().x() - arcCenter_px[0]);
        endAngle   = atan2(-(m_start->pos().y() - arcCenter_px[1]), m_start->pos().x() - arcCenter_px[0]);
    }
    double span = endAngle - startAngle;

    // Compute the midpoint along the arc for the arrow
    // Angles are left-handed vs. coordinate system
    midpoint_angle = (startAngle + endAngle) / 2.0;
    midpoint.setX(arcCenter_px[0] + radius_px * cos(midpoint_angle));
    midpoint.setY(arcCenter_px[1] - radius_px * sin(midpoint_angle));
    if (m_clockwise)
        midpoint_angle = midpoint_angle + M_PI;

    if (m_clockwise) {
        while (span > 0)
            span = span - 2 * M_PI;
    } else {
        while (span < 0)
            span = span + 2 * M_PI;
    }

    setRect(arcCenter_px[0] - radius_px, arcCenter_px[1] - radius_px, 2.0*radius_px, 2.0*radius_px);
    setStartAngle(startAngle * RAD2DEG * 16.0);
    if (m_numberOfOrbits == 0)
        setSpanAngle(span * RAD2DEG * 16.0);
    else
        setSpanAngle(360.0 * 16.0); // Full circle
    update();
}

void MapArc::endpointdeleted()
{
    this->deleteLater();
}

void MapArc::setOpacitySlot(qreal opacity)
{
    setOpacity(opacity);
}


/**
 * @brief Compute the center of curvature of the arc, by calculating the intersection
 * of the two circles of radius R around the two points. Inspired by
 * http://www.mathworks.com/matlabcentral/newsreader/view_thread/255121
 * @param[in] start_point Starting point, in North-East coordinates
 * @param[in] end_point Ending point, in North-East coordinates
 * @param[in] radius Radius of the curve segment
 * @param[in] clockwise true if clockwise is the positive sense of the arc, false if otherwise
 * @param[in] minor true if minor arc, false if major arc
 * @param[out] center Center of circle formed by two points, in North-East coordinates
 * @return
 */
MapArc::arc_center_results MapArc::findArcCenter_px(double start_point[2], double end_point[2], double radius, bool clockwise, bool minor, double center[2])
{
    // Sanity check
    if(fabs(start_point[0] - end_point[0]) < 1e-6 && fabs(start_point[1] - end_point[1]) < 1e-6){
        // This means that the start point and end point are directly on top of each other. In the
        // case of coincident points, there is not enough information to define the circle
        center[0]=NAN;
        center[1]=NAN;
        return ARC_COINCIDENT_POINTS;
    }

    double m_x, m_y, p_x, p_y, d, d2;

    // Center between start and end
    m_x = (start_point[0] + end_point[0]) / 2;
    m_y = (start_point[1] + end_point[1]) / 2;

    // Normal vector to the line between start and end points
    if ((clockwise == true && minor == true) ||
            (clockwise == false && minor == false)) { //clockwise minor OR counterclockwise major
        p_x = -(end_point[1] - start_point[1]);
        p_y =  (end_point[0] - start_point[0]);
    } else { //counterclockwise minor OR clockwise major
        p_x =  (end_point[1] - start_point[1]);
        p_y = -(end_point[0] - start_point[0]);
    }

    // Work out how far to go along the perpendicular bisector. First check there is a solution.
    d2 = radius*radius / (p_x*p_x + p_y*p_y) - 0.25;
    if (d2 < 0) {
        if (d2 > -pow(radius*0.01, 2)) // Make a 1% allowance for roundoff error
            d2 = 0;
        else {
            center[0]=NAN;
            center[1]=NAN;
            return ARC_INSUFFICIENT_RADIUS; // In this case, the radius wasn't big enough to connect the two points
        }
    }

    d = sqrt(d2);

    if (fabs(p_x) < 1e-3 && fabs(p_y) < 1e-3) {
        center[0] = m_x;
        center[1] = m_y;
    } else {
        center[0] = m_x + p_x * d;
        center[1] = m_y + p_y * d;
    }

    return ARC_CENTER_FOUND;
}

}


namespace mapcontrol
{

/**
 * @brief PathSegmentCurve::PathSegmentCurve Create the curve
 * @param start The starting location of the curve (will redraw if moved)
 * @param dest The ending location of the curve (will redraw if moved)
 * @param radius Radius of the curve
 * @param clockwise Whether to curve clockwise or counter (when going from start to finish) as view from above
 * @param map Handle to the map object
 * @param color Color of the curve
 */
PathSegmentCurve::PathSegmentCurve(MapPointItem *start, MapPointItem *dest, double curvature, bool clockwise, int numberOfOrbits, bool rank, MapGraphicItem *map, QColor color) :
    MapArc(start, dest, curvature, clockwise, numberOfOrbits, rank, map, color)
{
    refreshLocations();
}

//! Return the type of the QGraphicsEllipseItem
int PathSegmentCurve::type() const
{
    // Enable the use of qgraphicsitem_cast with this item.
    return Type;
}

/**
 * @brief PathSegmentCurve::paint Draw the path arc
 * @param painter The painter for drawing
 */
void PathSegmentCurve::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(option);
    Q_UNUSED(widget);

    QPen myPen = pen();
    myPen.setColor(myColor);
    painter->setPen(myPen);
    painter->setBrush(myColor);

    qreal arrowSize = 10;
    QBrush brush=painter->brush();

    QPointF arrowP1 = midpoint + QPointF(sin(midpoint_angle + M_PI / 3) * arrowSize,
                                   cos(midpoint_angle + M_PI / 3) * arrowSize);
    QPointF arrowP2 = midpoint + QPointF(sin(midpoint_angle + M_PI - M_PI / 3) * arrowSize,
                                   cos(midpoint_angle + M_PI - M_PI / 3) * arrowSize);

    arrowHead.clear();
    arrowHead << midpoint << arrowP1 << arrowP2;
    painter->drawPolygon(arrowHead);
    painter->setBrush(brush);
    painter->drawArc(this->rect(), this->startAngle(), this->spanAngle());
}

void PathSegmentCurve::waypointdeleted()
{
    this->deleteLater();
}

}
