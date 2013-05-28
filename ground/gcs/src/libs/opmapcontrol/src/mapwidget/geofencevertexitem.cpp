/**
******************************************************************************
*
* @file       geofencevertexitem.cpp
* @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2012.
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
#include "geofencevertexitem.h"
#include "homeitem.h"

namespace mapcontrol
{
    GeoFenceVertexItem::GeoFenceVertexItem(const internals::PointLatLng &coord,int const& altitude, MapGraphicItem *map,geofencetype type) :
        reached(false),
        shownumber(true),
        isDragging(false),
        myType(type)
    {
        this->map = map;
        this->coord = coord;
        this->altitude = altitude;
        description = "";

        text=0;
        numberI=0;
        isMagic=false;
        picture.load(QString::fromUtf8(":/markers/images/marker.png"));
        number=GeoFenceVertexItem::snumber;
        ++GeoFenceVertexItem::snumber;
        this->setFlag(QGraphicsItem::ItemIsMovable,true);
        this->setFlag(QGraphicsItem::ItemIgnoresTransformations,true);
        this->setFlag(QGraphicsItem::ItemIsSelectable,true);
        SetShowNumber(shownumber);
        RefreshToolTip();
        RefreshPos();
        myHome=NULL;
        QList<QGraphicsItem *> list=map->childItems();
        foreach(QGraphicsItem * obj,list)
        {
            HomeItem* h = qgraphicsitem_cast <HomeItem*>(obj);
            if(h)
                myHome = h;
        }

        if(myHome)
        {
            map->Projection()->offSetFromLatLngs(myHome->Coord(), coord, relativeCoord.distance, relativeCoord.bearing);
            connect(myHome, SIGNAL(absolutePositionChanged(internals::PointLatLng, float)), this, SLOT(onHomePositionChanged(internals::PointLatLng, float)));
        }
        connect(this, SIGNAL(geofencevertexdoubleclick(GeoFenceVertexItem*)), map, SIGNAL(wpdoubleclicked(GeoFenceVertexItem*)));
        emit manualCoordChange(this);
        connect(map, SIGNAL(childRefreshPosition()), this, SLOT(RefreshPos()));
        connect(map, SIGNAL(childSetOpacity(qreal)), this, SLOT(setOpacitySlot(qreal)));
    }

    GeoFenceVertexItem::GeoFenceVertexItem(const internals::PointLatLng &coord,int const& altitude, const QString &description, MapGraphicItem *map,geofencetype type):
        reached(false),
        shownumber(true),
        isDragging(false),
        myType(type)
    {
        this->map = map;
        this->coord = coord;
        this->altitude = altitude;
        this->description = description;

        text=0;
        numberI=0;
        isMagic=false;
        picture.load(QString::fromUtf8(":/markers/images/marker.png"));
        number=GeoFenceVertexItem::snumber;
        ++GeoFenceVertexItem::snumber;
        this->setFlag(QGraphicsItem::ItemIsMovable,true);
        this->setFlag(QGraphicsItem::ItemIgnoresTransformations,true);
        this->setFlag(QGraphicsItem::ItemIsSelectable,true);
        SetShowNumber(shownumber);
        RefreshToolTip();
        RefreshPos();
        myHome=NULL;
        QList<QGraphicsItem *> list=map->childItems();
        foreach(QGraphicsItem * obj,list)
        {
            HomeItem* h=qgraphicsitem_cast <HomeItem*>(obj);
            if(h)
                myHome=h;
        }
        if(myHome)
        {
            map->Projection()->offSetFromLatLngs(myHome->Coord(),coord,relativeCoord.distance,relativeCoord.bearing);
            connect(myHome, SIGNAL(absolutePositionChanged(internals::PointLatLng, float)), this, SLOT(onHomePositionChanged(internals::PointLatLng, float)));
        }
        connect(this,SIGNAL(geofencevertexdoubleclick(GeoFenceVertexItem*)),map, SIGNAL(wpdoubleclicked(GeoFenceVertexItem*)));
        emit manualCoordChange(this);
        connect(map, SIGNAL(childRefreshPosition()),this,SLOT(RefreshPos()));
        connect(map, SIGNAL(childSetOpacity(qreal)),this,SLOT(setOpacitySlot(qreal)));
    }

    GeoFenceVertexItem::GeoFenceVertexItem(const distBearingAltitude &relativeCoordinates, const QString &description, MapGraphicItem *map):
        reached(false),
        shownumber(true),
        isDragging(false)
    {
        this->map = map;
        this->relativeCoord = relativeCoordinates;
        this->altitude = altitude;
        this->description = description;

        myHome=NULL;
        QList<QGraphicsItem *> list=map->childItems();
        foreach(QGraphicsItem * obj,list)
        {
            HomeItem* h=qgraphicsitem_cast <HomeItem*>(obj);
            if(h)
               myHome=h;
        }
        if(myHome)
        {
            connect(myHome, SIGNAL(absolutePositionChanged(internals::PointLatLng, float)), this, SLOT(onHomePositionChanged(internals::PointLatLng, float)));
            coord=map->Projection()->translate(myHome->Coord(),relativeCoord.distance,relativeCoord.bearing);
        }
        myType=GEO_RELATIVE;
        text=0;
        numberI=0;
        isMagic=false;
        picture.load(QString::fromUtf8(":/markers/images/marker.png"));
        number=GeoFenceVertexItem::snumber;
        ++GeoFenceVertexItem::snumber;
        this->setFlag(QGraphicsItem::ItemIsMovable,true);
        this->setFlag(QGraphicsItem::ItemIgnoresTransformations,true);
        this->setFlag(QGraphicsItem::ItemIsSelectable,true);
        SetShowNumber(shownumber);
        RefreshToolTip();
        RefreshPos();
        connect(this,SIGNAL(geofencevertexdoubleclick(GeoFenceVertexItem*)),map, SIGNAL(wpdoubleclicked(GeoFenceVertexItem*)));
        emit manualCoordChange(this);
        connect(map, SIGNAL(childRefreshPosition()),this,SLOT(RefreshPos()));
        connect(map, SIGNAL(childSetOpacity(qreal)),this,SLOT(setOpacitySlot(qreal)));
    }

    void GeoFenceVertexItem::setWPType(geofencetype type)
    {
        myType=type;
        emit WPValuesChanged(this);
        RefreshPos();
        RefreshToolTip();
        this->update();
    }

    QRectF GeoFenceVertexItem::boundingRect() const
    {
        return QRectF(-picture.width()/2,-picture.height(),picture.width(),picture.height());
    }
    void GeoFenceVertexItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
    {
        Q_UNUSED(option);
        Q_UNUSED(widget);
        painter->drawPixmap(-picture.width()/2,-picture.height(),picture);
        painter->setPen(Qt::green);
        if(this->isSelected())
            painter->drawRect(QRectF(-picture.width()/2,-picture.height(),picture.width()-1,picture.height()-1));
    }
    void GeoFenceVertexItem::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event)
    {
        if(event->button()==Qt::LeftButton)
        {
            emit geofencevertexdoubleclick(this);
        }
    }

    void GeoFenceVertexItem::mousePressEvent(QGraphicsSceneMouseEvent *event)
    {
        if(event->button()==Qt::LeftButton)
        {
            text=new QGraphicsSimpleTextItem(this);
            textBG=new QGraphicsRectItem(this);

            textBG->setBrush(Qt::yellow);

            text->setPen(QPen(Qt::red));
            text->setPos(10,-picture.height());
            textBG->setPos(10,-picture.height());
            text->setZValue(3);
            RefreshToolTip();
            isDragging=true;
        }
        QGraphicsItem::mousePressEvent(event);
    }
    void GeoFenceVertexItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
    {
        QGraphicsItem::mouseReleaseEvent(event);
        if(event->button()==Qt::LeftButton &&
                event->buttonDownScenePos(Qt::LeftButton) != event->lastScenePos())
        {
            if(text)
            {
                delete text;
                text=NULL;
            }
            if(textBG)
            {
                delete textBG;
                textBG=NULL;
            }
            coord=map->FromLocalToLatLng(this->pos().x(),this->pos().y());

            isDragging=false;
            RefreshToolTip();
            emit manualCoordChange(this);
            emit relativePositionChanged(this->pos(),this);
            emit WPValuesChanged(this);
            emit WPDropped(this);
        } else if(event->button()==Qt::LeftButton) {
            if(text)
            {
                delete text;
                text=NULL;
            }
            if(textBG)
            {
                delete textBG;
                textBG=NULL;
            }
            coord=map->FromLocalToLatLng(this->pos().x(),this->pos().y());

            isDragging=false;
            RefreshToolTip();
        }
    }
    void GeoFenceVertexItem::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
    {

        if(isDragging)
        {
            coord=map->FromLocalToLatLng(this->pos().x(),this->pos().y());
            QString coord_str = " " + QString::number(coord.Lat(), 'f', 6) + "   " + QString::number(coord.Lng(), 'f', 6);
            if(myHome)
            {
                map->Projection()->offSetFromLatLngs(myHome->Coord(),coord,relativeCoord.distance,relativeCoord.bearing);
            }
            QString relativeCoord_str = QString::number(relativeCoord.distance) + "m " + QString::number(relativeCoord.bearing*180/M_PI)+"deg";
            text->setText(coord_str+"\n"+relativeCoord_str);
            textBG->setRect(text->boundingRect());
            emit relativePositionChanged(this->pos(),this);
            emit WPValuesChanged(this);
        }
            QGraphicsItem::mouseMoveEvent(event);
    }
    void GeoFenceVertexItem::SetAltitude(const float &value)
    {
        if(altitude==value)
            return;
        altitude=value;
        RefreshToolTip();
        emit WPValuesChanged(this);
        this->update();
    }

    void GeoFenceVertexItem::setRelativeCoord(distBearingAltitude value)
    {
//        if(relativeCoord == value)
//            return;
        relativeCoord=value;
        if(myHome)
        {
            SetCoord(map->Projection()->translate(myHome->Coord(),relativeCoord.distance,relativeCoord.bearing));
        }
        RefreshPos();
        RefreshToolTip();
        emit WPValuesChanged(this);
        this->update();
    }

    void GeoFenceVertexItem::SetCoord(const internals::PointLatLng &value)
    {
        if(coord == value)
            return;
        coord=value;
        distBearingAltitude back=relativeCoord;
        if(myHome)
            map->Projection()->offSetFromLatLngs(myHome->Coord(),Coord(),back.distance,back.bearing);
        if(qAbs(back.bearing-relativeCoord.bearing)>0.01 || qAbs(back.distance-relativeCoord.distance)>0.1)
        {
            relativeCoord=back;
        }
        emit WPValuesChanged(this);
        RefreshPos();
        RefreshToolTip();
        this->update();
    }
    void GeoFenceVertexItem::SetDescription(const QString &value)
    {
        if(description==value)
            return;
        description=value;
        RefreshToolTip();
        emit WPValuesChanged(this);
        this->update();
    }
    void GeoFenceVertexItem::SetNumber(const int &value)
    {
        int oldnumber=number;
        number=value;
        RefreshToolTip();
        numberI->setText(QString::number(numberAdjusted()));
        numberIBG->setRect(numberI->boundingRect().adjusted(-2,0,1,0));
        this->update();
        emit WPNumberChanged(oldnumber,value,this);
    }
    void GeoFenceVertexItem::SetReached(const bool &value)
    {
        reached=value;
        emit WPValuesChanged(this);
        if(value)
            picture.load(QString::fromUtf8(":/markers/images/bigMarkerGreen.png"));
        else
        {
            if(!isMagic)
            {
                if(this->flags() & QGraphicsItem::ItemIsMovable==QGraphicsItem::ItemIsMovable)
                    picture.load(QString::fromUtf8(":/markers/images/marker.png"));
                else
                    picture.load(QString::fromUtf8(":/markers/images/waypoint_marker2.png"));
            }
            else
            {
                picture.load(QString::fromUtf8(":/opmap/images/waypoint_marker3.png"));
            }
        }
            this->update();

    }
    void GeoFenceVertexItem::SetShowNumber(const bool &value)
    {
        shownumber=value;
        if((numberI==0) && value)
        {
            numberI=new QGraphicsSimpleTextItem(this);
            numberIBG=new QGraphicsRectItem(this);
            numberIBG->setBrush(Qt::white);
            numberIBG->setOpacity(0.5);
            numberI->setZValue(3);
            numberI->setPen(QPen(Qt::blue));
            numberI->setPos(0,-13-picture.height());
            numberIBG->setPos(0,-13-picture.height());
            numberI->setText(QString::number(numberAdjusted()));
            numberIBG->setRect(numberI->boundingRect().adjusted(-2,0,1,0));
        }
        else if (!value && numberI)
        {
            delete numberI;
            delete numberIBG;
        }
        this->update();
    }
    void GeoFenceVertexItem::WPDeleted(const int &onumber,GeoFenceVertexItem *waypoint)
    {
        Q_UNUSED(waypoint);
        int n=number;
        if(number>onumber) SetNumber(--n);
    }
    void GeoFenceVertexItem::WPInserted(const int &onumber, GeoFenceVertexItem *waypoint)
    {
        if(Number()==-1)
            return;

        if(waypoint!=this)
        {
            if(onumber<=number) SetNumber(++number);
        }
    }

    void GeoFenceVertexItem::onHomePositionChanged(internals::PointLatLng homepos, float homeAltitude)
    {
        Q_UNUSED(homeAltitude);

        if(myType == GEO_RELATIVE)
        {
            coord=map->Projection()->translate(homepos,relativeCoord.distance,relativeCoord.bearing);
            emit WPValuesChanged(this);
            RefreshPos();
            RefreshToolTip();
            this->update();
        }
        else
        {
            if(myHome)
            {
                map->Projection()->offSetFromLatLngs(myHome->Coord(),coord,relativeCoord.distance,relativeCoord.bearing);
            }
            emit WPValuesChanged(this);
        }
    }

    void GeoFenceVertexItem::WPRenumbered(const int &oldnumber, const int &newnumber, GeoFenceVertexItem *waypoint)
    {
        if (waypoint!=this)
        {
            if(((oldnumber>number) && (newnumber<=number)))
            {
                SetNumber(++number);
            }
            else if (((oldnumber<number) && (newnumber>number)))
            {
                SetNumber(--number);
            }
            else if (newnumber==number)
            {
                SetNumber(++number);
            }
        }
    }

    GeoFenceVertexItem::~GeoFenceVertexItem()
    {
        emit aboutToBeDeleted(this);
        --GeoFenceVertexItem::snumber;
    }
    void GeoFenceVertexItem::RefreshPos()
    {
        core::Point point=map->FromLatLngToLocal(coord);
        this->setPos(point.X(),point.Y());
        emit relativePositionChanged(this->pos(),this);
    }

    void GeoFenceVertexItem::setOpacitySlot(qreal opacity)
    {
        setOpacity(opacity);
    }

    /**
     * @brief GeoFenceVertexItem::RefreshToolTip Set the tooltip for this waypoint
     * whenever it changes
     */
    void GeoFenceVertexItem::RefreshToolTip()
    {
        QString coord_str = " " + QString::number(coord.Lat(), 'f', 6) + "   " + QString::number(coord.Lng(), 'f', 6);
        QString relativeCoord_str = " Distance:" + QString::number(relativeCoord.distance) + " Bearing:" + QString::number(relativeCoord.bearing*180/M_PI);
        if(Number()!=-1)
            setToolTip(QString("WayPoint Number:%1\nDescription:%2\nCoordinate:%4\nFrom Home:%5\nAltitude:%6\n%7").arg(QString::number(numberAdjusted())).arg(description).arg(coord_str).arg(relativeCoord_str).arg(QString::number(altitude)).arg(myCustomString));
        else
            setToolTip(QString("Magic WayPoint\nCoordinate:%1\nFrom Home:%2\nAltitude:%3\n\n%4").arg(coord_str).arg(relativeCoord_str).arg(QString::number(altitude)).arg(myCustomString));
    }

    void GeoFenceVertexItem::setFlag(QGraphicsItem::GraphicsItemFlag flag, bool enabled)
    {
        if(isMagic)
        {
            QGraphicsItem::setFlag(flag,enabled);
            return;
        }
        else if(flag==QGraphicsItem::ItemIsMovable)
        {
            if(enabled)
                picture.load(QString::fromUtf8(":/markers/images/marker.png"));
            else
                picture.load(QString::fromUtf8(":/markers/images/waypoint_marker2.png"));
        }
        QGraphicsItem::setFlag(flag,enabled);
    }

    int GeoFenceVertexItem::snumber=0;
}
