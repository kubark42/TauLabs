/**
******************************************************************************
*
* @file       pathsegmentendpointitem.h
* @author     Tau Labs, http://taulabs.org, Copyright (C) 2013
* @brief      A graphicsItem representing a path segement end point
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
#ifndef PATHSEGMENTENDPOINTITEM_H
#define PATHSEGMENTENDPOINTITEM_H

#include "mappointitem.h"

namespace mapcontrol
{
/**
* @brief A QGraphicsItem representing a WayPoint
*
* @class PathSegmentEndpointItem pathsegmentendpointitem.h "pathsegmentendpointitem.h"
*/
class PathSegmentEndpointItem: public MapPointItem
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)
public:
    enum { Type = UserType + TYPE_PATHSEGMENTENDPOINTITEM };
    /**
    * @brief Constructor
    *
    * @param coord coordinates in LatLng of the WayPoint
    * @param altitude altitude of the WayPoint
    * @param map pointer to map to use
    * @param description description fo the WayPoint
    * @return
    */
    PathSegmentEndpointItem(internals::PointLatLng const& coord, int const& altitude, MapGraphicItem *map, QString const& description = "");

    ~PathSegmentEndpointItem();

    /**
    * @brief Returns the WayPoint description
    *
    * @return QString
    */
    QString Description(){return description;}

    /**
    * @brief Sets the WayPoint description
    *
    * @param value
    */
    void SetDescription(QString const& value);

    /**
    * @brief Returns the WayPoint number
    *
    */
    int Number(){return number;}
    int numberAdjusted(){return number+1;}

    /**
    * @brief Sets WayPoint number
    *
    * @param value
    */
    void SetNumber(int const& value);

    /**
    * @brief Returns WayPoint LatLng coordinate
    *
    */
    virtual void SetCoord(internals::PointLatLng const& value);

//    /**
//    * @brief Used if WayPoint number is to be drawn on screen
//    *
//    */
//    bool ShowNumber(){return shownumber;}

//    /**
//    * @brief  Used to set if WayPoint number is to be drawn on screen
//    *
//    * @param value
//    */
//    void SetShowNumber(bool const& value);

    /**
    * @brief Sets the WayPoint altitude
    *
    * @return int
    */
    virtual void SetAltitude(const float &value);

    void setRelativeCoord(distBearingAltitude value);
    distBearingAltitude getRelativeCoord(){return relativeCoord;}
    int type() const;
    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
                QWidget *widget);
    void RefreshToolTip();
    QPixmap picture;
    QString customString(){return myCustomString;}
    void setCustomString(QString arg){myCustomString=arg;}
    void setFlag(GraphicsItemFlag flag, bool enabled);

    static int snumber;

protected:
    void mouseMoveEvent ( QGraphicsSceneMouseEvent * event );
    void mousePressEvent ( QGraphicsSceneMouseEvent * event );
    void mouseReleaseEvent ( QGraphicsSceneMouseEvent * event );
    void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event);

private:
    bool shownumber;
    bool isDragging;
    int number;

    QGraphicsSimpleTextItem* text;
    QGraphicsRectItem* textBG;
    QGraphicsSimpleTextItem* numberI;
    QGraphicsRectItem* numberIBG;
    QTransform transf;
    QString myCustomString;

public slots:
    /**
    * @brief Called when a WayPoint is deleted
    *
    * @param number number of the WayPoint that was deleted
    */
    void WPDeleted(int const& number,PathSegmentEndpointItem *waypoint);
    /**
    * @brief Called when a WayPoint is renumbered
    *
    * @param oldnumber the old  WayPoint number
    * @param newnumber the new WayPoint number
    * @param waypoint a pointer to the WayPoint renumbered
    */
    void WPRenumbered(int const& oldnumber,int const& newnumber,PathSegmentEndpointItem* waypoint);
    /**
    * @brief Called when a  WayPoint is inserted
    *
    * @param number the number of the  WayPoint
    * @param waypoint  a pointer to the WayPoint inserted
    */
    void WPInserted(int const& number,PathSegmentEndpointItem* waypoint);

    void onHomePositionChanged(internals::PointLatLng, float altitude);
    void RefreshPos();
    void setOpacitySlot(qreal opacity);
signals:
    /**
    * @brief fires when this WayPoint number changes (not fired if due to a auto-renumbering)
    *
    * @param oldnumber this WayPoint old number
    * @param newnumber this WayPoint new number
    * @param waypoint a pointer to this WayPoint
    */
    void WPNumberChanged(int const& oldnumber,int const& newnumber,PathSegmentEndpointItem* waypoint);

    /**
    * @brief Fired when the description, altitude or coordinates change
    *
    * @param waypoint a pointer to this WayPoint
    */

    /**
    * @brief Fired when the waypoint is dropped somewhere
    *
    * @param waypoint a pointer to this WayPoint
    */
    void WPDropped(PathSegmentEndpointItem* waypoint);

    void WPValuesChanged(PathSegmentEndpointItem* waypoint);
    void waypointdoubleclick(PathSegmentEndpointItem* waypoint);
    void manualCoordChange(PathSegmentEndpointItem *);
};
}
#endif // PATHSEGMENTENDPOINTITEM_H
