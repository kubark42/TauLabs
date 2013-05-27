/**
******************************************************************************
*
* @file       mappoint.h
* @author     Tau Labs, http://taulabs.org Copyright (C) 2013.
* @brief      A graphicsItem representing a MapItem
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
#ifndef MAPPOINT_H
#define MAPPOINT_H

#include <QGraphicsItem>
#include <QLabel>
#include <QObject>
#include <QPainter>
#include <QPoint>

#include "../internals/pointlatlng.h"
#include "mapgraphicitem.h"
#include "physical_constants.h"

namespace mapcontrol
{

struct distBearingAltitude
{
    double distance;
    double bearing;
    distBearingAltitude() : distance(0.0), bearing(0.0) { }
};

/**
* @brief A QGraphicsItem representing a MapPoint
*
* @class MapPoint mappoint.h "mappoint.h"
*/
class MapPoint:public QObject,public QGraphicsItem
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)
public:
    enum GraphicItemTypes {TYPE_WAYPOINTITEM = 1, TYPE_HOMEITEM = 4};

    /**
    * @brief Returns the MapPoint description
    *
    * @return QString
    */
    QString Description(){return description;}

    /**
    * @brief Sets the MapPoint description
    *
    * @param value
    */
    void SetDescription(QString const& value);

    /**
    * @brief Returns MapPoint LatLng coordinate
    *
    */
    internals::PointLatLng Coord(){return coord;}

    /**
    * @brief  Sets MapPoint LatLng coordinate
    *
    * @param value
    */
    virtual void SetCoord(internals::PointLatLng const& value);

    /**
    * @brief Returns the MapPoint altitude
    *
    * @return int
    */
    float Altitude(){return altitude;}

    /**
    * @brief Sets the MapPoint Altitude
    *
    * @param value
    */
    virtual void SetAltitude(const float &value);
    void setRelativeCoord(distBearingAltitude value);
    distBearingAltitude getRelativeCoord(){return relativeCoord;}

protected:
    MapGraphicItem* map;

    internals::PointLatLng coord; //coordinates of this MapPoint
    float altitude;
    distBearingAltitude relativeCoord;
    QString description;

    double DistanceToPoint_2D(const internals::PointLatLng &coord);
    double DistanceToPoint_3D(const internals::PointLatLng &coord, const int &altitude);
private:

    QGraphicsSimpleTextItem* text;
    QGraphicsRectItem* textBG;
    QGraphicsSimpleTextItem* numberI;
    QGraphicsRectItem* numberIBG;
    QTransform transf;
    QString myCustomString;

public slots:
signals:
    void absolutePositionChanged(internals::PointLatLng coord, float altitude);
    void relativePositionChanged(QPointF point, MapPoint* mappoint);
    void aboutToBeDeleted(MapPoint *);
};
}
#endif // MAPPOINT_H
