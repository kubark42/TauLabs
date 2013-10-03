/**
 ******************************************************************************
 * @file       pathsegmentmodelmapproxy.cpp
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2013
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

#include "pathsegmentmodelmapproxy.h"
#include "utils/coordinateconversions.h"

QTimer PathSegmentModelMapProxy::overlayRefreshTimer;

PathSegmentModelMapProxy::PathSegmentModelMapProxy(QObject *parent,TLMapWidget *map, PathSegmentDataModel *pathSegmentModel, QItemSelectionModel *selectionModel):
    QObject(parent),
    myMap(map),
    pathSegmentModel(pathSegmentModel),
    selection(selectionModel)
{
    connect(pathSegmentModel, SIGNAL(rowsInserted(const QModelIndex&,int,int)), this, SLOT(rowsInserted(const QModelIndex&,int,int)));
    connect(pathSegmentModel, SIGNAL(rowsRemoved(const QModelIndex&,int,int)), this, SLOT(rowsRemoved(const QModelIndex&,int,int)));
    connect(pathSegmentModel, SIGNAL(dataChanged(QModelIndex,QModelIndex)), this, SLOT(dataChanged(QModelIndex,QModelIndex)));
    connect(selection, SIGNAL(currentRowChanged(QModelIndex,QModelIndex)), this, SLOT(currentRowChanged(QModelIndex,QModelIndex)));
    connect(myMap, SIGNAL(selectedWPChanged(QList<WayPointItem*>)), this, SLOT(selectedWPChanged(QList<WayPointItem*>)));
    connect(myMap, SIGNAL(WPManualCoordChange(WayPointItem*)), this, SLOT(PSDValuesChanged(MapPointItem*)));

    // Only update the overlay periodically. Otherwise we flood the graphics system
    overlayRefreshTimer.setInterval(50);
    connect(&overlayRefreshTimer, SIGNAL(timeout()), this, SLOT(overlayRefreshTimeout()));
}

/**
 * @brief PathSegmentModelMapProxy::PSDValuesChanged The UI changed a waypoint, update the pathSegmentDataModel
 * @param psd The handle to the changed path segment descriptor
 */
void PathSegmentModelMapProxy::PSDValuesChanged(MapPointItem * psd)
{
//    QModelIndex index;
//    index = pathSegmentModel->index(wp->Number(),PathSegmentDataModel::LATPOSITION);
//    if(!index.isValid())
//        return;
//    pathSegmentModel->setData(index,wp->Coord().Lat(),Qt::EditRole);

//    index = pathSegmentModel->index(wp->Number(),PathSegmentDataModel::LNGPOSITION);
//    pathSegmentModel->setData(index,wp->Coord().Lng(),Qt::EditRole);

//    index = pathSegmentModel->index(wp->Number(),PathSegmentDataModel::ALTITUDE);
//    waypointModel->setData(index,wp->Altitude(),Qt::EditRole);
}

/**
 * @brief PathSegmentModelMapProxy::currentRowChanged When a row is changed, highlight the waypoint
 * @param current The selected row
 * @param previous Unused
 */
void PathSegmentModelMapProxy::currentRowChanged(QModelIndex current, QModelIndex previous)
{
//    Q_UNUSED(previous);

//    QList<WayPointItem*> list;
//    WayPointItem * wp=findWayPointNumber(current.row());
//    if(!wp)
//        return;
//    list.append(wp);
//    myMap->setSelectedWP(list);
}

/**
 * @brief PathSegmentModelMapProxy::selectedWPChanged When a list of waypoints are changed, select them in model
 * @param list The list of changed waypoints
 */
void PathSegmentModelMapProxy::selectedWPChanged(QList<WayPointItem *> list)
{
//    selection->clearSelection();
//    foreach(WayPointItem * wp,list)
//    {
//        QModelIndex index = waypointModel->index(wp->Number(),0);
//        selection->setCurrentIndex(index,QItemSelectionModel::Select | QItemSelectionModel::Rows);
//    }
}

///**
// * @brief PathSegmentModelMapProxy::overlayTranslate Map from path types types to Overlay types
// * @param type The map delegate type which is like a Waypoint::Mode
// * @return
// */
//PathSegmentModelMapProxy::overlayType PathSegmentModelMapProxy::overlayTranslate(Waypoint::ModeOptions type)
//{
//    switch(type)
//    {
//    case Waypoint::MODE_FLYENDPOINT:
//    case Waypoint::MODE_FLYVECTOR:
//    case Waypoint::MODE_DRIVEENDPOINT:
//    case Waypoint::MODE_DRIVEVECTOR:
//        return OVERLAY_LINE;
//        break;
//    case Waypoint::MODE_FLYCIRCLERIGHT:
//    case Waypoint::MODE_DRIVECIRCLERIGHT:
//        return OVERLAY_CURVE_RIGHT;
//        break;
//    case Waypoint::MODE_FLYCIRCLELEFT:
//    case Waypoint::MODE_DRIVECIRCLELEFT:
//        return OVERLAY_CURVE_LEFT;
//        break;
//    default:
//        break;
//    }

//    // Default value
//    return OVERLAY_LINE;
//}

/**
 * @brief PathSegmentModelMapProxy::createOverlay Create a graphical path component
 * @param from The starting location
 * @param to The ending location (for circles the radius)
 * @param type The type of path component
 * @param color
 */
void PathSegmentModelMapProxy::createOverlay(PathSegmentEndpointItem *from, PathSegmentEndpointItem *to,
                                             double curvature, int numberOfOrbits, int arcRank,
                                             QColor color )
{
    if(from==NULL || to==NULL || from==to)
        return;

    if (curvature == 0) {
        myMap->lineCreate(from, to, color);
    }
    else
        myMap->curveCreate(from, to, 1.0/curvature, (curvature > 0), numberOfOrbits, arcRank, color);
}

/**
 * @brief PathSegmentModelMapProxy::refreshOverlays Starts a timer, which upon timeout will trigger a
 * refresh of the path segment overlays
 */
void PathSegmentModelMapProxy::refreshOverlays()
{
    // Reset the countdown. This makes it likely that the redrawing and model updates won't occur until
    // all UAVOs have been updated
    overlayRefreshTimer.start();
}

/**
 * @brief PathSegmentModelMapProxy::findWayPointNumber Return the graphial icon for the requested waypoint
 * @param number The waypoint number
 * @return The pointer to the graphical item or NULL
 */
PathSegmentEndpointItem *PathSegmentModelMapProxy::findEndPointNumber(int number)
{
    if(number<0)
        return NULL;
    return myMap->PSDFind(number);
}

/**
 * @brief PathSegmentModelMapProxy::rowsRemoved Called whenever a row is removed from the model
 * @param parent Unused
 * @param first The first row removed
 * @param last The last row removed
 */
void PathSegmentModelMapProxy::rowsRemoved(const QModelIndex &parent, int first, int last)
{
    Q_UNUSED(parent);

    for(int i=last; i>first-1; i--)
    {
//        // Delete the marker if one is already here.
//        PathSegmentEndpointItem* ww = findEndPointNumber(i);
//        if (ww)
//            ww->deleteLater();
////        myMap->WPDelete(x);
    }
    refreshOverlays();
}

/**
 * @brief PathSegmentModelMapProxy::dataChanged Update the display whenever the model information changes
 * @param topLeft The first waypoint and column changed
 * @param bottomRight The last waypoint and column changed
 */
void PathSegmentModelMapProxy::dataChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight)
{
    Q_UNUSED(bottomRight);

    // Abort if no corresponding graphical item
    PathSegmentEndpointItem *item = findEndPointNumber(topLeft.row());
    if(!item)
        return;

    bool updateLatLon_flag = false;
    bool updateArc_flag = false;

    for (int x = topLeft.row(); x <= bottomRight.row(); x++) {
        for (int column = topLeft.column(); column <= bottomRight.column(); column++) {
            // Action depends on which columns were modified
            switch(column)
            {
            case PathSegmentDataModel::NED_POS_NORTH:
            case PathSegmentDataModel::NED_POS_EAST:
            case PathSegmentDataModel::NED_POS_DOWN:
                updateLatLon_flag = true;
                break;
            case PathSegmentDataModel::ARC_RANK:
            case PathSegmentDataModel::CURVATURE:
                updateArc_flag = true;
                break;
            }
        }
        if (updateLatLon_flag) {
            double LLA_endpoint[3];
            double NED_endpoint[3] = {pathSegmentModel->data(pathSegmentModel->index(x, PathSegmentDataModel::NED_POS_NORTH)).toDouble(),
                                      pathSegmentModel->data(pathSegmentModel->index(x, PathSegmentDataModel::NED_POS_EAST)).toDouble(),
                                      pathSegmentModel->data(pathSegmentModel->index(x, PathSegmentDataModel::NED_POS_DOWN)).toDouble()};

            Utils::CoordinateConversions().NED2LLA_HomeLLA(pathSegmentModel->homeLLA, NED_endpoint, LLA_endpoint);
            internals::PointLatLng endpoint(LLA_endpoint[0], LLA_endpoint[1]);

            item->SetCoord(endpoint);
            refreshOverlays();
        }

        if (updateArc_flag)
            refreshOverlays();
    }
}

/**
 * @brief PathSegmentModelMapProxy::rowsInserted When rows are inserted in the model, add the corresponding graphical items
 * @param parent Unused
 * @param first The first row to update
 * @param last The last row to update
 */
void PathSegmentModelMapProxy::rowsInserted(const QModelIndex &parent, int first, int last)
{
    Q_UNUSED(parent);

    for(int i=first; i < last+1; i++) {
        double LLA_endpoint[3];
        double NED_endpoint[3] = {pathSegmentModel->data(pathSegmentModel->index(i, PathSegmentDataModel::NED_POS_NORTH)).toDouble(),
                               pathSegmentModel->data(pathSegmentModel->index(i, PathSegmentDataModel::NED_POS_EAST)).toDouble(),
                               pathSegmentModel->data(pathSegmentModel->index(i, PathSegmentDataModel::NED_POS_DOWN)).toDouble()};
        Utils::CoordinateConversions().NED2LLA_HomeLLA(pathSegmentModel->homeLLA, NED_endpoint, LLA_endpoint);

        internals::PointLatLng endpoint(LLA_endpoint[0], LLA_endpoint[1]);
        double altitude = LLA_endpoint[2];
        QString desc("");
        myMap->PathSegmentEndpointInsert(endpoint, altitude, desc, i);
    }
    refreshOverlays();
}

///**
// * @brief PathSegmentModelMapProxy::deleteWayPoint When a waypoint is deleted graphically, delete from the model
// * @param number The waypoint which was deleted
// */
//void PathSegmentModelMapProxy::deleteWayPoint(int number)
//{
//    waypointModel->removeRow(number,QModelIndex());
//}

///**
// * @brief PathSegmentModelMapProxy::createWayPoint When a waypoint is created graphically, insert into the end of the model
// * @param coord The coordinate the waypoint was created
// */
//void PathSegmentModelMapProxy::createWayPoint(internals::PointLatLng coord)
//{
//    waypointModel->insertRow(waypointModel->rowCount(),QModelIndex());
//    QModelIndex index = waypointModel->index(waypointModel->rowCount()-1,WaypointDataModel::LATPOSITION,QModelIndex());
//    waypointModel->setData(index,coord.Lat(),Qt::EditRole);
//    index = waypointModel->index(waypointModel->rowCount()-1,WaypointDataModel::LNGPOSITION,QModelIndex());
//    waypointModel->setData(index,coord.Lng(),Qt::EditRole);
//}

///**
// * @brief PathSegmentModelMapProxy::deleteAll When all the waypoints are deleted graphically, update the model
// */
//void PathSegmentModelMapProxy::deleteAll()
//{
//    waypointModel->removeRows(0,waypointModel->rowCount(),QModelIndex());
//}


/**
 * @brief PathSegmentModelMapProxy::overlayRefreshTimeout On timeout, update the information from the model and
 * redraw all the components
 */
void PathSegmentModelMapProxy::overlayRefreshTimeout()
{
    myMap->deletePathSegmentOverlays();
    if(pathSegmentModel->rowCount() < 1)
        return;

    PathSegmentEndpointItem *psd_current = NULL;
    PathSegmentEndpointItem *psd_next = NULL;


    for(int i=0; i < pathSegmentModel->rowCount()-1; i++)
    {
        psd_current = findEndPointNumber(i);
        psd_next = findEndPointNumber(i+1);

        createOverlay(psd_current, psd_next,
                      pathSegmentModel->data(pathSegmentModel->index(i+1, PathSegmentDataModel::CURVATURE)).toDouble(),
                      pathSegmentModel->data(pathSegmentModel->index(i+1, PathSegmentDataModel::NUM_ORBITS)).toInt(),
                      pathSegmentModel->data(pathSegmentModel->index(i+1, PathSegmentDataModel::ARC_RANK)).toInt(),
                      Qt::magenta);
    }
}
