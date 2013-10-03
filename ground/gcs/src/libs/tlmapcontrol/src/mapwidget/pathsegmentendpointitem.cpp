/**
******************************************************************************
*
* @file       pathsegmentendpointitem.cpp
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
#include "pathsegmentendpointitem.h"

namespace mapcontrol
{
    PathSegmentEndpointItem::PathSegmentEndpointItem(const internals::PointLatLng &coord,int const& altitude, MapGraphicItem *map, const QString &description):
        shownumber(true),
        isDragging(false)
    {
        if (fabs(coord.Lat()) > 90 || fabs(coord.Lng()) > 180) {
            Q_ASSERT(0);
            return;
        }

        this->map = map;
        this->coord = coord;
        this->altitude = altitude;
        this->description = description;

        text=0;
        numberI=0;
        picture.load(QString::fromUtf8(":/markers/images/location-marker.png"));
        number=PathSegmentEndpointItem::snumber;
        ++PathSegmentEndpointItem::snumber;
        this->setFlag(QGraphicsItem::ItemIsMovable,true);
        this->setFlag(QGraphicsItem::ItemIgnoresTransformations,true);
        this->setFlag(QGraphicsItem::ItemIsSelectable,true);
//        SetShowNumber(shownumber);
        RefreshToolTip();
        RefreshPos();

        connect(this,SIGNAL(waypointdoubleclick(PathSegmentEndpointItem*)),map,SIGNAL(wpdoubleclicked(PathSegmentEndpointItem*)));
        emit manualCoordChange(this);
        connect(map,SIGNAL(childRefreshPosition()),this,SLOT(RefreshPos()));
        connect(map,SIGNAL(childSetOpacity(qreal)),this,SLOT(setOpacitySlot(qreal)));
    }

    PathSegmentEndpointItem::~PathSegmentEndpointItem()
    {
        emit aboutToBeDeleted(this);
        --PathSegmentEndpointItem::snumber;
    }


    QRectF PathSegmentEndpointItem::boundingRect() const
    {
        return QRectF(-picture.width()/2,-picture.height(),picture.width(),picture.height());
    }
    void PathSegmentEndpointItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
    {
        Q_UNUSED(option);
        Q_UNUSED(widget);
        painter->drawPixmap(-picture.width()/2,-picture.height(),picture);
        painter->setPen(Qt::green);
        if(this->isSelected())
            painter->drawRect(QRectF(-picture.width()/2,-picture.height(),picture.width()-1,picture.height()-1));
    }
    void PathSegmentEndpointItem::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event)
    {
        if(event->button()==Qt::LeftButton)
        {
            emit waypointdoubleclick(this);
        }
    }

    void PathSegmentEndpointItem::mousePressEvent(QGraphicsSceneMouseEvent *event)
    {
//        if(event->button()==Qt::LeftButton)
//        {
//            text=new QGraphicsSimpleTextItem(this);
//            textBG=new QGraphicsRectItem(this);

//            textBG->setBrush(Qt::yellow);

//            text->setPen(QPen(Qt::red));
//            text->setPos(10,-picture.height());
//            textBG->setPos(10,-picture.height());
//            text->setZValue(3);
//            RefreshToolTip();
//            isDragging=true;
//        }
        QGraphicsItem::mousePressEvent(event);
    }
    void PathSegmentEndpointItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
    {
        QGraphicsItem::mouseReleaseEvent(event);
//        if(event->button()==Qt::LeftButton &&
//                event->buttonDownScenePos(Qt::LeftButton) != event->lastScenePos())
//        {
//            if(text)
//            {
//                delete text;
//                text=NULL;
//            }
//            if(textBG)
//            {
//                delete textBG;
//                textBG=NULL;
//            }
//            coord=map->FromLocalToLatLng(this->pos().x(),this->pos().y());

//            isDragging=false;
//            RefreshToolTip();
//            emit manualCoordChange(this);
//            emit relativePositionChanged(this->pos(),this);
//            emit WPValuesChanged(this);
//            emit WPDropped(this);
//        } else if(event->button()==Qt::LeftButton) {
//            if(text)
//            {
//                delete text;
//                text=NULL;
//            }
//            if(textBG)
//            {
//                delete textBG;
//                textBG=NULL;
//            }
//            coord=map->FromLocalToLatLng(this->pos().x(),this->pos().y());

//            isDragging=false;
//            RefreshToolTip();
//        }
    }
    void PathSegmentEndpointItem::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
    {

//        if(isDragging)
//        {
//            coord=map->FromLocalToLatLng(this->pos().x(),this->pos().y());
//            QString coord_str = " " + QString::number(coord.Lat(), 'f', 6) + "   " + QString::number(coord.Lng(), 'f', 6);

//            QString relativeCoord_str = QString::number(relativeCoord.distance) + "m " + QString::number(relativeCoord.bearing*180/M_PI)+"deg";
//            text->setText(coord_str+"\n"+relativeCoord_str);
//            textBG->setRect(text->boundingRect());
//            emit relativePositionChanged(this->pos(),this);
//            emit WPValuesChanged(this);
//        }
//            QGraphicsItem::mouseMoveEvent(event);
    }
    void PathSegmentEndpointItem::SetAltitude(const float &value)
    {
        if(altitude==value)
            return;
        altitude=value;
        RefreshToolTip();
        emit WPValuesChanged(this);
        this->update();
    }

    void PathSegmentEndpointItem::setRelativeCoord(distBearingAltitude value)
    {
//        if(relativeCoord == value)
//            return;
        relativeCoord=value;

        RefreshPos();
        RefreshToolTip();
        emit WPValuesChanged(this);
        this->update();
    }

    void PathSegmentEndpointItem::SetCoord(const internals::PointLatLng &value)
    {
        if(coord == value)
            return;
        coord=value;
        distBearingAltitude back=relativeCoord;

        if(qAbs(back.bearing-relativeCoord.bearing)>0.01 || qAbs(back.distance-relativeCoord.distance)>0.1)
        {
            relativeCoord=back;
        }
        emit WPValuesChanged(this);
        RefreshPos();
        RefreshToolTip();
        this->update();
    }
    void PathSegmentEndpointItem::SetDescription(const QString &value)
    {
        if(description==value)
            return;
        description=value;
        RefreshToolTip();
        emit WPValuesChanged(this);
        this->update();
    }
    void PathSegmentEndpointItem::SetNumber(const int &value)
    {
        int oldNumber=number;
        number=value;
//        RefreshToolTip();
//        numberI->setText(QString::number(numberAdjusted()));
//        numberIBG->setRect(numberI->boundingRect().adjusted(-2,0,1,0));
//        this->update();
//        emit WPNumberChanged(oldNumber,value,this);
    }

//    void PathSegmentEndpointItem::SetShowNumber(const bool &value)
//    {
//        shownumber=value;
//        if((numberI==0) && value)
//        {
//            numberI=new QGraphicsSimpleTextItem(this);
//            numberIBG=new QGraphicsRectItem(this);
//            numberIBG->setBrush(Qt::white);
//            numberIBG->setOpacity(0.5);
//            numberI->setZValue(3);
//            numberI->setPen(QPen(Qt::blue));
//            numberI->setPos(0,-13-picture.height());
//            numberIBG->setPos(0,-13-picture.height());
//            numberI->setText(QString::number(numberAdjusted()));
//            numberIBG->setRect(numberI->boundingRect().adjusted(-2,0,1,0));
//        }
//        else if (!value && numberI)
//        {
//            delete numberI;
//            delete numberIBG;
//        }
//        this->update();
//    }
    void PathSegmentEndpointItem::WPDeleted(const int &onumber,PathSegmentEndpointItem *waypoint)
    {
        Q_UNUSED(waypoint);
        int n=number;
        if(number>onumber) SetNumber(--n);
    }
    void PathSegmentEndpointItem::WPInserted(const int &onumber, PathSegmentEndpointItem *waypoint)
    {
        if(Number()==-1)
            return;

        if(waypoint!=this)
        {
            if(onumber<=number) SetNumber(++number);
        }
    }

    void PathSegmentEndpointItem::onHomePositionChanged(internals::PointLatLng homepos, float homeAltitude)
    {
        Q_UNUSED(homeAltitude);

        coord=map->Projection()->translate(homepos,relativeCoord.distance,relativeCoord.bearing);
        emit WPValuesChanged(this);
        RefreshPos();
        RefreshToolTip();
        this->update();
    }

    void PathSegmentEndpointItem::WPRenumbered(const int &oldnumber, const int &newnumber, PathSegmentEndpointItem *waypoint)
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
    int PathSegmentEndpointItem::type() const
    {
        // Enable the use of qgraphicsitem_cast with this item.
        return Type;
    }

    void PathSegmentEndpointItem::RefreshPos()
    {
        core::Point point=map->FromLatLngToLocal(coord);
        this->setPos(point.X(),point.Y());
        emit relativePositionChanged(this->pos(),this);
    }

    void PathSegmentEndpointItem::setOpacitySlot(qreal opacity)
    {
        setOpacity(opacity);
    }

    /**
     * @brief PathSegmentEndpointItem::RefreshToolTip Set the tooltip for this waypoint
     * whenever it changes
     */
    void PathSegmentEndpointItem::RefreshToolTip()
    {
        QString coord_str = " " + QString::number(coord.Lat(), 'f', 6) + "   " + QString::number(coord.Lng(), 'f', 6);
        QString relativeCoord_str = " Distance:" + QString::number(relativeCoord.distance) + " Bearing:" + QString::number(relativeCoord.bearing*180/M_PI);
        setToolTip(QString("Path segment number:%1\nCoordinate:%4\nFrom Home:%5\nAltitude:%6\n%7").arg(QString::number(numberAdjusted())).arg(description).arg(coord_str).arg(relativeCoord_str).arg(QString::number(altitude)).arg(myCustomString));
    }

    void PathSegmentEndpointItem::setFlag(QGraphicsItem::GraphicsItemFlag flag, bool enabled)
    {
        if(flag==QGraphicsItem::ItemIsMovable) {
            if(enabled)
                picture.load(QString::fromUtf8(":/markers/images/location-marker.png"));
            else
                picture.load(QString::fromUtf8(":/markers/images/waypoint_marker2.png"));
        }
        QGraphicsItem::setFlag(flag,enabled);
    }

    int PathSegmentEndpointItem::snumber=0;
}
