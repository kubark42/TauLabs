/**
 ******************************************************************************
 * @file       waypointmodelmapproxy.cpp
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2013
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2012.
 *
 * @addtogroup GCSPlugins GCS Plugins
 * @{
 * @addtogroup OPMapPlugin Tau Labs Map Plugin
 * @{
 * @brief Tau Labs map plugin
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

#include "waypointmodelmapproxy.h"
#include "pathsegmentdescriptor.h"

QTimer WayPointModelMapProxy::overlayRefreshTimer;

WayPointModelMapProxy::WayPointModelMapProxy(QObject *parent,TLMapWidget *map, WaypointDataModel *waypointModel,QItemSelectionModel *selectionModel):
    QObject(parent),
    myMap(map),
    waypointModel(waypointModel),
    selection(selectionModel)
{
    connect(waypointModel,SIGNAL(rowsInserted(const QModelIndex&,int,int)),this,SLOT(rowsInserted(const QModelIndex&,int,int)));
    connect(waypointModel,SIGNAL(rowsRemoved(const QModelIndex&,int,int)),this,SLOT(rowsRemoved(const QModelIndex&,int,int)));
    connect(selection,SIGNAL(currentRowChanged(QModelIndex,QModelIndex)),this,SLOT(currentRowChanged(QModelIndex,QModelIndex)));
    connect(waypointModel,SIGNAL(dataChanged(QModelIndex,QModelIndex)),this,SLOT(dataChanged(QModelIndex,QModelIndex)));
    connect(myMap,SIGNAL(selectedWPChanged(QList<WayPointItem*>)),this,SLOT(selectedWPChanged(QList<WayPointItem*>)));
    connect(myMap,SIGNAL(WPManualCoordChange(WayPointItem*)),this,SLOT(WPValuesChanged(WayPointItem*)));

    // Only update the overlay periodically. Otherwise we flood the graphics system
    overlayRefreshTimer.setInterval(50);
    connect(&overlayRefreshTimer, SIGNAL(timeout()), this, SLOT(overlayRefreshTimeout()));
}

/**
 * @brief WayPointModelMapProxy::WPValuesChanged The UI changed a waypoint, update the waypointModel
 * @param wp The handle to the changed waypoint
 */
void WayPointModelMapProxy::WPValuesChanged(WayPointItem * wp)
{
    QModelIndex index;
    index = waypointModel->index(wp->Number(),WaypointDataModel::LATPOSITION);
    if(!index.isValid())
        return;
    waypointModel->setData(index,wp->Coord().Lat(),Qt::EditRole);
    index = waypointModel->index(wp->Number(),WaypointDataModel::LNGPOSITION);
    waypointModel->setData(index,wp->Coord().Lng(),Qt::EditRole);

    index = waypointModel->index(wp->Number(),WaypointDataModel::ALTITUDE);
    waypointModel->setData(index,wp->Altitude(),Qt::EditRole);
}

/**
 * @brief WayPointModelMapProxy::currentRowChanged When a row is changed, highlight the waypoint
 * @param current The selected row
 * @param previous Unused
 */
void WayPointModelMapProxy::currentRowChanged(QModelIndex current, QModelIndex previous)
{
    Q_UNUSED(previous);

    QList<WayPointItem*> list;
    WayPointItem * wp=findWayPointNumber(current.row());
    if(!wp)
        return;
    list.append(wp);
    myMap->setSelectedWP(list);
}

/**
 * @brief WayPointModelMapProxy::selectedWPChanged When a list of waypoints are changed, select them in model
 * @param list The list of changed waypoints
 */
void WayPointModelMapProxy::selectedWPChanged(QList<WayPointItem *> list)
{
    selection->clearSelection();
    foreach(WayPointItem * wp,list)
    {
        QModelIndex index = waypointModel->index(wp->Number(),0);
        selection->setCurrentIndex(index,QItemSelectionModel::Select | QItemSelectionModel::Rows);
    }
}

/**
 * @brief WayPointModelMapProxy::overlayTranslate Map from path types types to Overlay types
 * @param type The map delegate type which is like a Waypoint::Mode
 * @return
 */
WayPointModelMapProxy::overlayType WayPointModelMapProxy::overlayTranslate(Waypoint::ModeOptions type)
{
    switch(type)
    {
    case Waypoint::MODE_FLYENDPOINT:
    case Waypoint::MODE_FLYVECTOR:
    case Waypoint::MODE_DRIVEENDPOINT:
    case Waypoint::MODE_DRIVEVECTOR:
        return OVERLAY_LINE;
        break;
    case Waypoint::MODE_FLYCIRCLERIGHT:
    case Waypoint::MODE_DRIVECIRCLERIGHT:
        return OVERLAY_CURVE_RIGHT;
        break;
    case Waypoint::MODE_FLYCIRCLELEFT:
    case Waypoint::MODE_DRIVECIRCLELEFT:
        return OVERLAY_CURVE_LEFT;
        break;
    default:
        break;
    }

    // Default value
    return OVERLAY_LINE;
}

/**
 * @brief WayPointModelMapProxy::createOverlay Create a graphical path component
 * @param from The starting location
 * @param to The ending location (for circles the radius)
 * @param type The type of path component
 * @param color
 */
void WayPointModelMapProxy::createOverlay(WayPointItem *from, WayPointItem *to,
                                  overlayType type, QColor color,
                                  double radius=0)
{
    if(from==NULL || to==NULL || from==to)
        return;
    switch(type)
    {
    case OVERLAY_LINE:
        myMap->lineCreate(from, to, color);
        break;
    case OVERLAY_CIRCLE_RIGHT:
        myMap->circleCreate(to, from, true, color);
        break;
    case OVERLAY_CIRCLE_LEFT:
        myMap->circleCreate(to, from, false, color);
        break;
    case OVERLAY_CURVE_RIGHT:
        myMap->curveCreate(from, to, radius, true,  0, PathSegmentDescriptor::ARCRANK_MINOR, color);
        break;
    case OVERLAY_CURVE_LEFT:
        myMap->curveCreate(from, to, radius, false, 0, PathSegmentDescriptor::ARCRANK_MINOR, color);
        break;
    default:
        break;
    }
}

/**
 * @brief WayPointModelMapProxy::createOverlay Create a graphical path component
 * @param from The starting location
 * @param to The ending location (for circles the radius) which is a HomeItem
 * @param type The type of path component
 * @param color
 */
void WayPointModelMapProxy::createOverlay(WayPointItem *from, HomeItem *to, overlayType type,QColor color)
{
    if(from==NULL || to==NULL) //FIXME: Is it not also necessary here to check from!=to ? If so, this function can be removed in favor of only one createOverlay
        return;
    switch(type)
    {
    case OVERLAY_LINE:
        myMap->lineCreate(to, from, color); // FIXME: Ugh, to and from and from and to. Very confusing
        break;
    case OVERLAY_CIRCLE_RIGHT:
        myMap->circleCreate(to, from, true, color);
        break;
    case OVERLAY_CIRCLE_LEFT:
        myMap->circleCreate(to, from, false, color);
        break;
    default:
        break;
    }
}

/**
 * @brief WayPointModelMapProxy::refreshOverlays Update the information from the model and
 * redraw all the components
 */
void WayPointModelMapProxy::refreshOverlays()
{
    // Reset the countdown. This makes it likely that the redrawing and model updates won't occur until
    // all UAVOs have been updated
    overlayRefreshTimer.start();
}

/**
 * @brief WayPointModelMapProxy::findWayPointNumber Return the graphial icon for the requested waypoint
 * @param number The waypoint number
 * @return The pointer to the graphical item or NULL
 */
WayPointItem *WayPointModelMapProxy::findWayPointNumber(int number)
{
    if(number<0)
        return NULL;
    return myMap->WPFind(number);
}

/**
 * @brief WayPointModelMapProxy::rowsRemoved Called whenever a row is removed from the model
 * @param parent Unused
 * @param first The first row removed
 * @param last The last row removed
 */
void WayPointModelMapProxy::rowsRemoved(const QModelIndex &parent, int first, int last)
{
    Q_UNUSED(parent);

    for(int x=last;x>first-1;x--)
    {
        myMap->WPDelete(x);
    }
    refreshOverlays();
}

/**
 * @brief WayPointModelMapProxy::dataChanged Update the display whenever the model information changes
 * @param topLeft The first waypoint and column changed
 * @param bottomRight The last waypoint and column changed
 */
void WayPointModelMapProxy::dataChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight)
{
    Q_UNUSED(bottomRight);

    // Abort if no corresponding graphical item
    WayPointItem *item = findWayPointNumber(topLeft.row());
    if(!item)
        return;

    internals::PointLatLng latlng;
    double altitude;
    QModelIndex index;
    QString desc;

    for (int x = topLeft.row(); x <= bottomRight.row(); x++) {
        for (int column = topLeft.column(); column <= bottomRight.column(); column++) {
            // Action depends on which columns were modified
            switch(column)
            {
            case WaypointDataModel::MODE:
                refreshOverlays();
                break;
            case WaypointDataModel::WPDESCRITPTION:
                index = waypointModel->index(x,WaypointDataModel::WPDESCRITPTION);
                desc = index.data(Qt::DisplayRole).toString();
                item->SetDescription(desc);
                break;
            case WaypointDataModel::LATPOSITION:
                latlng = item->Coord();
                index = waypointModel->index(x,WaypointDataModel::LATPOSITION);
                latlng.SetLat(index.data(Qt::DisplayRole).toDouble());
                item->SetCoord(latlng);
                break;
            case WaypointDataModel::LNGPOSITION:
                latlng=item->Coord();
                index = waypointModel->index(x,WaypointDataModel::LNGPOSITION);
                latlng.SetLng(index.data(Qt::DisplayRole).toDouble());
                item->SetCoord(latlng);
                break;
            case WaypointDataModel::ALTITUDE:
                index = waypointModel->index(x,WaypointDataModel::ALTITUDE);
                altitude = index.data(Qt::DisplayRole).toDouble();
                item->SetAltitude(altitude);
                break;
            case WaypointDataModel::MODE_PARAMS:
                // Make sure to update radius of arcs
                refreshOverlays();
                break;
            case WaypointDataModel::LOCKED:
                index = waypointModel->index(x,WaypointDataModel::LOCKED);
                item->setFlag(QGraphicsItem::ItemIsMovable,!index.data(Qt::DisplayRole).toBool());
                break;
            }
        }
    }
}

/**
 * @brief WayPointModelMapProxy::rowsInserted When rows are inserted in the model, add the corresponding graphical items
 * @param parent Unused
 * @param first The first row to update
 * @param last The last row to update
 */
void WayPointModelMapProxy::rowsInserted(const QModelIndex &parent, int first, int last)
{
    Q_UNUSED(parent);
    for(int x=first; x<last+1; x++)
    {
        QModelIndex index;
        internals::PointLatLng latlng;
        WayPointItem *item;
        double altitude;
        index = waypointModel->index(x,WaypointDataModel::WPDESCRITPTION);
        QString desc = index.data(Qt::DisplayRole).toString();
        index = waypointModel->index(x,WaypointDataModel::LATPOSITION);
        latlng.SetLat(index.data(Qt::DisplayRole).toDouble());
        index = waypointModel->index(x,WaypointDataModel::LNGPOSITION);
        latlng.SetLng(index.data(Qt::DisplayRole).toDouble());
        index = waypointModel->index(x,WaypointDataModel::ALTITUDE);
        altitude = index.data(Qt::DisplayRole).toDouble();
        item = myMap->WPInsert(latlng,altitude,desc,x);
    }
    refreshOverlays();
}

/**
 * @brief WayPointModelMapProxy::deleteWayPoint When a waypoint is deleted graphically, delete from the model
 * @param number The waypoint which was deleted
 */
void WayPointModelMapProxy::deleteWayPoint(int number)
{
    waypointModel->removeRow(number,QModelIndex());
}

/**
 * @brief WayPointModelMapProxy::createWayPoint When a waypoint is created graphically, insert into the end of the model
 * @param coord The coordinate the waypoint was created
 */
void WayPointModelMapProxy::createWayPoint(internals::PointLatLng coord)
{
    waypointModel->insertRow(waypointModel->rowCount(),QModelIndex());
    QModelIndex index = waypointModel->index(waypointModel->rowCount()-1,WaypointDataModel::LATPOSITION,QModelIndex());
    waypointModel->setData(index,coord.Lat(),Qt::EditRole);
    index = waypointModel->index(waypointModel->rowCount()-1,WaypointDataModel::LNGPOSITION,QModelIndex());
    waypointModel->setData(index,coord.Lng(),Qt::EditRole);
}

/**
 * @brief WayPointModelMapProxy::deleteAll When all the waypoints are deleted graphically, update the model
 */
void WayPointModelMapProxy::deleteAll()
{
    waypointModel->removeRows(0,waypointModel->rowCount(),QModelIndex());
}

/**
 * @brief ModelMapProxy::overlayRefreshTimeout On timeout, update the information from the model and
 * redraw all the components
 */
void WayPointModelMapProxy::overlayRefreshTimeout()
{
    myMap->deleteWaypointOverlays();
    if(waypointModel->rowCount()<1)
        return;
    WayPointItem *wp_current = NULL;
    WayPointItem *wp_next = NULL;
    overlayType wp_next_overlay;

    // Get first waypoint type before stepping through path
    wp_current = findWayPointNumber(0);
    overlayType wp_current_overlay = overlayTranslate((Waypoint::ModeOptions) waypointModel->data(waypointModel->index(0,WaypointDataModel::MODE),Qt::UserRole).toInt());
    createOverlay(wp_current,myMap->Home,wp_current_overlay,Qt::green);

    for(int x=0; x < waypointModel->rowCount(); ++x)
    {
        wp_current = findWayPointNumber(x);

        wp_next_overlay = overlayTranslate((Waypoint::ModeOptions) waypointModel->data(waypointModel->index(x+1,WaypointDataModel::MODE),Qt::UserRole).toInt());

        wp_next = findWayPointNumber(x+1);
        createOverlay(wp_current, wp_next, wp_next_overlay, Qt::green,
                      waypointModel->data(waypointModel->index(x+1,WaypointDataModel::MODE_PARAMS)).toFloat());
    }
}