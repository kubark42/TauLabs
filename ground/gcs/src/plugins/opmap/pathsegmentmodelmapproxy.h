/**
 ******************************************************************************
 * @file       pathsegmentmodelmapproxy.h
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
#ifndef PATHSEGMENTMODELMAPPROXY_H
#define PATHSEGMENTMODELMAPPROXY_H
#include <QWidget>
#include "tlmapcontrol/tlmapcontrol.h"
#include "pathsegmentdescriptor.h"
#include "QMutexLocker"
#include "QPointer"
#include <QItemSelectionModel>

#include "../pathplanner/flightdatamodel.h"

using namespace mapcontrol;

/**
 * @brief The PathSegmentModelMapProxy class maps from the @ref PathSegmentDataModel to the OPMap
 * and provides synchronization, both when the model changes updating the UI and
 * if it is modified in the UI, propagating changes to the model.
 */
class PathSegmentModelMapProxy:public QObject
{
    typedef enum {OVERLAY_LINE, OVERLAY_CURVE_RIGHT, OVERLAY_CURVE_LEFT, OVERLAY_CIRCLE_RIGHT, OVERLAY_CIRCLE_LEFT} overlayType;
    Q_OBJECT
public:
    explicit PathSegmentModelMapProxy(QObject *parent, TLMapWidget *map, PathSegmentDataModel *pathSegmentModel, QItemSelectionModel *selectionModel);

private slots:

    //! Data in the model is changed, update the UI
    void dataChanged ( const QModelIndex & topLeft, const QModelIndex & bottomRight );

    //! Rows inserted into the model, update the UI
    void rowsInserted ( const QModelIndex & parent, int first, int last );

    //! Rows removed from the model, update the UI
    void rowsRemoved ( const QModelIndex & parent, int first, int last );

    //! The UI changed a path segment descriptor, update the model
    void PSDValuesChanged(MapPointItem *wp);

    //! When a row is changed, highlight the waypoint
    void currentRowChanged(QModelIndex,QModelIndex);

    //! When a list of waypoints are changed, select them in model
    void selectedWPChanged(QList<WayPointItem*>);

    //! Upon timeout, refresh the overlay
    void overlayRefreshTimeout();

private:
    //! Get the handle to a waypoint graphical item
    PathSegmentEndpointItem *findEndPointNumber(int number);
//    overlayType overlayTranslate(Waypoint::ModeOptions type);
    void refreshOverlays();

    void createOverlay(PathSegmentEndpointItem *from, PathSegmentEndpointItem * to, double curvature, int numberOfOrbits, int arcRank, QColor color);
    TLMapWidget * myMap;
    PathSegmentDataModel *pathSegmentModel;
    QItemSelectionModel * selection;
    static QTimer overlayRefreshTimer;

};

#endif // PATHSEGMENTMODELMAPPROXY_H
