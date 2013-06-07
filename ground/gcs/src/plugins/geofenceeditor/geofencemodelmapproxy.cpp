/**
 ******************************************************************************
 *
 * @file       geofencemodelmapproxy.cpp
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2012.
 * @author     Tau Labs, http://taulabs.org Copyright (C) 2013.
 * @addtogroup GCSPlugins GCS Plugins
 * @{
 * @addtogroup OPMapPlugin OpenPilot Map Plugin
 * @{
 * @brief The OpenPilot Map plugin
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

#include "geofencemodelmapproxy.h"
//#include "../internals/pointlatlng.h"

const int GeofenceModelMapProxy::DEFAULT_LOWER_ALTITUDE = 100;
const int GeofenceModelMapProxy::DEFAULT_UPPER_ALTITUDE = 400;


GeofenceModelMapProxy::GeofenceModelMapProxy(QObject *parent, OPMapWidget *map, GeoFenceVerticesDataModel *verticesModel, QItemSelectionModel * verticesSelectionModel, GeoFenceFacesDataModel * facesModel, QItemSelectionModel *facesSelectionModel) :
    QObject(parent),
    myMap(map),
    myVerticesModel(verticesModel),
    myFacesModel(facesModel),
    verticesSelection(verticesSelectionModel),
    facesSelection(facesSelectionModel),
    currentPolygonId(-1)

{
    connect(myVerticesModel,SIGNAL(rowsInserted(const QModelIndex&,int,int)),this,SLOT(rowsInserted(const QModelIndex&,int,int)));
    connect(myVerticesModel,SIGNAL(rowsRemoved(const QModelIndex&,int,int)),this,SLOT(rowsRemoved(const QModelIndex&,int,int)));
    connect(verticesSelection,SIGNAL(currentRowChanged(QModelIndex,QModelIndex)),this,SLOT(currentRowChanged(QModelIndex,QModelIndex)));
    connect(myVerticesModel,SIGNAL(dataChanged(QModelIndex,QModelIndex)),this,SLOT(dataChanged(QModelIndex,QModelIndex)));
    connect(myMap,SIGNAL(SelectedVertexChanged(QList<GeoFenceVertexItem*>)),this,SLOT(selectedVertexChanged(QList<GeoFenceVertexItem*>)));
    connect(myMap,SIGNAL(VertexValuesChanged(GeoFenceVertexItem*)),this,SLOT(vertexValuesChanged(GeoFenceVertexItem*)));
}

GeoFenceVertexItem *GeofenceModelMapProxy::findVertexNumber(int number)
{
//    if(number<0)
//        return NULL;
//    return myMap->GFVertexFind(number);
}

void GeofenceModelMapProxy::createVertexPoint(internals::PointLatLng coord)
{
    myVerticesModel->insertRow(myVerticesModel->rowCount(),QModelIndex());
    QModelIndex index=myVerticesModel->index(myVerticesModel->rowCount()-1,GeoFenceVerticesDataModel::GEO_LATITUDE);
    myVerticesModel->setData(index,coord.Lat(),Qt::EditRole);
    index=myVerticesModel->index(myVerticesModel->rowCount()-1,GeoFenceVerticesDataModel::GEO_LONGITUDE);
    myVerticesModel->setData(index,coord.Lng(),Qt::EditRole);
//    index=myModel->index(myModel->rowCount()-1,GeoFenceVerticesDataModel::GEO_POLYGON_ID);
//    myModel->setData(index,currentPolygonId,Qt::EditRole);
}

void GeofenceModelMapProxy::deleteVertexPoint(int number)
{
    myVerticesModel->removeRow(number,QModelIndex());
}

void GeofenceModelMapProxy::deleteAll()
{
    myVerticesModel->removeRows(0, myVerticesModel->rowCount(), QModelIndex());
    myFacesModel->removeRows(0, myFacesModel->rowCount(), QModelIndex());
}

void GeofenceModelMapProxy::beginGeofencePolygon()
{
    myMap->setGeofencePolyMode(true);
    currentPolygonId++;
}

void GeofenceModelMapProxy::endGeofencePolygon(QMouseEvent* event)
{
//    Q_UNUSED(event);

//    // Complete polygon, copy to create a top and bottom for
//    // extruded polyhedron, convert to triangle mesh and open
//    // vertex editor.
//    int curPairId = 0;
//    int startingRowCount = myVerticesModel->rowCount();
//    for(int count = 0; count < startingRowCount; ++count){
//        // Get latitude and longitude for current point, this will be used for the
//        // new point.
//        internals::PointLatLng point;
//        QModelIndex index = myVerticesModel->index(count, GeoFenceVerticesDataModel::GEO_LATITUDE);
//        point.SetLat(index.data().toDouble());
//        index = myVerticesModel->index(count, GeoFenceVerticesDataModel::GEO_LONGITUDE);
//        point.SetLng(index.data().toDouble());

//        // Set the point's altitude
//        index = myVerticesModel->index(count, GeoFenceVerticesDataModel::GEO_ALTITUDE);
//        myVerticesModel->setData(index, DEFAULT_UPPER_ALTITUDE);

////        // Set the point's pair id and polygon id
////        index = myModel->index(count, GeoFenceVerticesDataModel::GEO_VERTEX_PAIR_ID);
////        myModel->setData(index, curPairId);
////        index = myModel->index(count, GeoFenceVerticesDataModel::GEO_POLYGON_ID);
////        myModel->setData(index, currentPolygonId);

//        // Add a new vertex
//        createVertexPoint(point);

//        // Set the new point's altitude, polygon id and pair id
//        index = myVerticesModel->index(myVerticesModel->rowCount() - 1, GeoFenceVerticesDataModel::GEO_ALTITUDE);
//        myVerticesModel->setData(index, DEFAULT_LOWER_ALTITUDE);
////        index = myModel->index(myModel->rowCount() - 1, GeoFenceVerticesDataModel::GEO_VERTEX_PAIR_ID);
////        myModel->setData(index, curPairId);
////        index = myModel->index(myModel->rowCount() - 1, GeoFenceVerticesDataModel::GEO_POLYGON_ID);
////        myModel->setData(index, currentPolygonId);

//        ++curPairId;
//    }

//    // Display the editor dialog to allow desired altitudes to be set
//    emit requestGeofenceEditDialog();
}

void GeofenceModelMapProxy::dataChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight)
{
//    Q_UNUSED(bottomRight);

//    GeoFenceVertexItem * item=findVertexNumber(topLeft.row());
//    if(!item)
//        return;
//    internals::PointLatLng latlng;
//    int x=topLeft.row();
//    double altitude;
//    QModelIndex index;
//    switch(topLeft.column())
//    {
//    case GeoFenceVerticesDataModel::GEO_LATITUDE:
//        latlng=item->Coord();
//        index=myVerticesModel->index(x,GeoFenceVerticesDataModel::GEO_LATITUDE);
//        latlng.SetLat(index.data(Qt::DisplayRole).toDouble());
//        item->SetCoord(latlng);
//        break;
//    case GeoFenceVerticesDataModel::GEO_LONGITUDE:
//        latlng=item->Coord();
//        index=myVerticesModel->index(x,GeoFenceVerticesDataModel::GEO_LONGITUDE);
//        latlng.SetLng(index.data(Qt::DisplayRole).toDouble());
//        item->SetCoord(latlng);
//        break;
//    case GeoFenceVerticesDataModel::GEO_ALTITUDE:
//        index=myVerticesModel->index(x,GeoFenceVerticesDataModel::GEO_ALTITUDE);
//        altitude=index.data(Qt::DisplayRole).toDouble();
//        item->SetAltitude(altitude);
//        break;
//    }
}

void GeofenceModelMapProxy::rowsInserted(const QModelIndex &parent, int first, int last)
{
//    Q_UNUSED(parent);
//    for(int x=first;x<last+1;x++)
//    {
//        QModelIndex index;
//        GeoFenceVertexItem * item;
//        internals::PointLatLng latlng;
//        double altitude;
//        index=myVerticesModel->index(x,GeoFenceVerticesDataModel::GEO_LATITUDE);
//        latlng.SetLat(index.data(Qt::DisplayRole).toDouble());
//        index=myVerticesModel->index(x,GeoFenceVerticesDataModel::GEO_LONGITUDE);
//        latlng.SetLng(index.data(Qt::DisplayRole).toDouble());
//        index=myVerticesModel->index(x,GeoFenceVerticesDataModel::GEO_ALTITUDE);
//        altitude=index.data(Qt::DisplayRole).toDouble();
//        item=myMap->VertexInsert(latlng, altitude, x);
//    }
//    refreshOverlays();
}

void GeofenceModelMapProxy::rowsRemoved(const QModelIndex &parent, int first, int last)
{
//    Q_UNUSED(parent);

//    for(int x=last;x>first-1;x--)
//    {
//        myMap->WPDelete(x);
//    }
//    refreshOverlays();
}

void GeofenceModelMapProxy::vertexValuesChanged(GeoFenceVertexItem *wp)
{
//    QModelIndex index;
//    index=myVerticesModel->index(wp->Number(),GeoFenceVerticesDataModel::GEO_LATITUDE);
//    myVerticesModel->setData(index,wp->Coord().Lat(),Qt::EditRole);
//    index=myVerticesModel->index(wp->Number(),GeoFenceVerticesDataModel::GEO_LONGITUDE);
//    myVerticesModel->setData(index,wp->Coord().Lng(),Qt::EditRole);

//    index=myVerticesModel->index(wp->Number(),GeoFenceVerticesDataModel::GEO_ALTITUDE);
//    myVerticesModel->setData(index,wp->Altitude(),Qt::EditRole);
}

void GeofenceModelMapProxy::currentRowChanged(QModelIndex current, QModelIndex previous)
{
//    Q_UNUSED(previous);

//    QList<GeoFenceVertexItem*> list;
//    GeoFenceVertexItem * wp=findVertexNumber(current.row());
//    if(!wp)
//        return;
//    list.append(wp);
//    myMap->setSelectedVertex(list);
}

void GeofenceModelMapProxy::selectedVertexChanged(QList<GeoFenceVertexItem *> list)
{
    verticesSelection->clearSelection();
    foreach(GeoFenceVertexItem * wp,list)
    {
        QModelIndex index=myVerticesModel->index(wp->Number(),0);
        verticesSelection->setCurrentIndex(index,QItemSelectionModel::Select | QItemSelectionModel::Rows);
    }
}

GeofenceModelMapProxy::overlayType GeofenceModelMapProxy::overlayTranslate(int type)
{
    Q_UNUSED(type);
}

void GeofenceModelMapProxy::createOverlay(GeoFenceVertexItem *from, GeoFenceVertexItem *to, GeofenceModelMapProxy::overlayType type, QColor color)
{
    Q_UNUSED(from);
    Q_UNUSED(to);
    Q_UNUSED(type);
    Q_UNUSED(color);
}

void GeofenceModelMapProxy::refreshOverlays()
{
}
