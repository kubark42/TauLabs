/**
******************************************************************************
*
* @file       geofencevertexitem.h
* @author     Tau Labs, http://taulabs.org Copyright (C) 2013.
* @brief      A graphicsItem representing a geofence vertex
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
#ifndef GEOFENCEVERTEXITEM_H
#define GEOFENCEVERTEXITEM_H

#include "mappointitem.h"

namespace mapcontrol
{
class HomeItem;
/**
* @brief A QGraphicsItem representing a GeoFence
*
* @class GeoFenceVertexItem geofencevertexitem.h "geofencevertexitem.h"
*/
class GeoFenceVertexItem: public MapPointItem
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)
public:
    enum { Type = UserType + TYPE_GEOFENCEVERTEXITEM };
    enum geofencetype {GEO_ABSOLUTE, GEO_RELATIVE};
    /**
    * @brief Constructor
    *
    * @param coord coordinates in LatLng of the GeoFence
    * @param altitude altitude of the GeoFence
    * @param map pointer to map to use
    * @return
    */
    GeoFenceVertexItem(internals::PointLatLng const& coord,int const& altitude,MapGraphicItem* map,geofencetype type = GEO_ABSOLUTE);
    /**
    * @brief Constructor
    *
    * @param coord coordinates in LatLng of the GeoFence
    * @param altitude altitude of the GeoFence
    * @param description description fo the GeoFence
    * @param map pointer to map to use
    * @return
    */
    GeoFenceVertexItem(internals::PointLatLng const& coord,int const& altitude,QString const& description,MapGraphicItem* map,geofencetype type = GEO_ABSOLUTE);
    GeoFenceVertexItem(distBearingAltitude const& relativeCoord,QString const& description,MapGraphicItem* map);

    /**
    * @brief Returns the GeoFence description
    *
    * @return QString
    */
    QString Description(){return description;}
    /**
    * @brief Sets the GeoFence description
    *
    * @param value
    */
    void SetDescription(QString const& value);
    /**
    * @brief Returns true if GeoFence is Reached
    *
    * @return bool
    */
    bool Reached(){return reached;}
    /**
    * @brief  Sets if GeoFence is Reached
    *
    * @param value
    */
    void SetReached(bool const& value);
    /**
    * @brief Returns the GeoFence number
    *
    */
    int Number(){return number;}
    int numberAdjusted(){return number+1;}
    /**
    * @brief Sets GeoFence number
    *
    * @param value
    */
    void SetNumber(int const& value);
    /**
    * @brief Returns GeoFence LatLng coordinate
    *
    */

    virtual void SetCoord(internals::PointLatLng const& value);
    /**
    * @brief Used if GeoFence number is to be drawn on screen
    *
    */
    bool ShowNumber(){return shownumber;}
    /**
    * @brief  Used to set if GeoFence number is to be drawn on screen
    *
    * @param value
    */
    void SetShowNumber(bool const& value);
    /**
    * @brief Returns the GeoFence altitude
    *
    * @return int
    */

    virtual void SetAltitude(const float &value);
    void setRelativeCoord(distBearingAltitude value);
    distBearingAltitude getRelativeCoord(){return relativeCoord;}
    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
                QWidget *widget);
    void RefreshToolTip();
    QPixmap picture;
    QString customString(){return myCustomString;}
    void setCustomString(QString arg){myCustomString=arg;}
    void setFlag(GraphicsItemFlag flag, bool enabled);
~GeoFenceVertexItem();

    static int snumber;
    void setWPType(geofencetype type);
    geofencetype WPType(){return myType;}
protected:
    void mouseMoveEvent ( QGraphicsSceneMouseEvent * event );
    void mousePressEvent ( QGraphicsSceneMouseEvent * event );
    void mouseReleaseEvent ( QGraphicsSceneMouseEvent * event );
    void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event);
private:
    bool reached;
    bool shownumber;
    bool isDragging;
    int number;
    bool isMagic;

    QGraphicsSimpleTextItem* text;
    QGraphicsRectItem* textBG;
    QGraphicsSimpleTextItem* numberI;
    QGraphicsRectItem* numberIBG;
    QTransform transf;
    HomeItem * myHome;
    geofencetype myType;
    QString myCustomString;

public slots:
    /**
    * @brief Called when a GeoFence is deleted
    *
    * @param number number of the GeoFence that was deleted
    */
    void WPDeleted(int const& number,GeoFenceVertexItem *geofencevertex);
    /**
    * @brief Called when a GeoFence is renumbered
    *
    * @param oldnumber the old  GeoFence number
    * @param newnumber the new GeoFence number
    * @param geofencevertex a pointer to the GeoFence renumbered
    */
    void WPRenumbered(int const& oldnumber,int const& newnumber,GeoFenceVertexItem* geofencevertex);
    /**
    * @brief Called when a  GeoFence is inserted
    *
    * @param number the number of the  GeoFence
    * @param geofencevertex  a pointer to the GeoFence inserted
    */
    void WPInserted(int const& number,GeoFenceVertexItem* geofencevertex);

    void onHomePositionChanged(internals::PointLatLng, float altitude);
    void RefreshPos();
    void setOpacitySlot(qreal opacity);
signals:
    /**
    * @brief fires when this GeoFence number changes (not fired if due to a auto-renumbering)
    *
    * @param oldnumber this GeoFence old number
    * @param newnumber this GeoFence new number
    * @param geofencevertex a pointer to this GeoFence
    */
    void WPNumberChanged(int const& oldnumber,int const& newnumber,GeoFenceVertexItem* geofencevertex);

    /**
    * @brief Fired when the description, altitude or coordinates change
    *
    * @param geofencevertex a pointer to this GeoFence
    */

    /**
    * @brief Fired when the geofencevertex is dropped somewhere
    *
    * @param geofencevertex a pointer to this GeoFence
    */
    void WPDropped(GeoFenceVertexItem* geofencevertex);

    void WPValuesChanged(GeoFenceVertexItem* geofencevertex);
    void geofencevertexdoubleclick(GeoFenceVertexItem* geofencevertex);
    void manualCoordChange(GeoFenceVertexItem *);
};
}
#endif // GEOFENCEVERTEXITEM_H
