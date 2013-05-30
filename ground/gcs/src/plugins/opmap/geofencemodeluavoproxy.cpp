/**
 ******************************************************************************
 * @file       modeluavproxy.cpp
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2013
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2012.
 * @addtogroup GCSPlugins GCS Plugins
 * @{
 * @addtogroup Path Planner Plugin
 * @{
 * @brief The Path Planner plugin
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

#include <QDebug>
#include <QEventLoop>
#include <QTimer>
#include "geofencemodeluavoproxy.h"
#include "extensionsystem/pluginmanager.h"
#include <math.h>

#include "utils/coordinateconversions.h"
#include "homelocation.h"

//! Initialize the model uavo proxy
GeoFenceModelUavoProxy::GeoFenceModelUavoProxy(QObject *parent, GeoFenceVerticesDataModel *model):QObject(parent),myModel(model)
{
    ExtensionSystem::PluginManager *pm = ExtensionSystem::PluginManager::instance();
    objManager = pm->getObject<UAVObjectManager>();
    geofenceFaces = GeoFenceFaces::GetInstance(objManager);
    geofenceVertices = GeoFenceVertices::GetInstance(objManager);
}

/**
 * @brief GeoFenceModelUavoProxy::modelToObjects Cast from the internal representation of a path
 * to the UAV objects required to represent it
 */
void GeoFenceModelUavoProxy::modelToObjects()
{
    // Set metadata
    GeoFenceVertices *gfV = GeoFenceVertices::GetInstance(objManager, 0);
    GeoFenceFaces *gfF = GeoFenceFaces::GetInstance(objManager, 0);

    Q_ASSERT(gfF != NULL && gfV != NULL);
    if (gfF == NULL || gfV == NULL)
        return;

    // Make sure the objects are acked
    UAVObject::Metadata initialGFFMeta = gfF->getMetadata();
    UAVObject::Metadata meta = initialGFFMeta;
    UAVObject::SetFlightTelemetryAcked(meta, true);
    gfF->setMetadata(meta);

    UAVObject::Metadata initialGFVMeta = gfV->getMetadata();
    meta = initialGFVMeta;
    UAVObject::SetFlightTelemetryAcked(meta, true);
    gfV->setMetadata(meta);

    double homeLLA[3];
    double NED[3];
    double LLA[3];
    getHomeLocation(homeLLA);

    for(int x=0; x < myModel->rowCount(); ++x)
    {
        GeoFenceVertices *gfV = NULL;
        GeoFenceFaces *gfF = NULL;

        // Get the number of existing waypoints
        int faceInstances=objManager->getNumInstances(geofenceFaces->getObjID());
        int vertexInstances=objManager->getNumInstances(geofenceVertices->getObjID());

        // Create new instances of vertices if this is more than exist
        if(x>vertexInstances-1)
        {
            gfV=new GeoFenceVertices;
            gfV->initialize(x, gfV->getMetaObject());
            objManager->registerObject(gfV);
        }
        else
        {
            gfV=GeoFenceVertices::GetInstance(objManager, x);
        }

        // Create new instances of faces if this is more than exist
        if(x>faceInstances-1)
        {
            gfF=new GeoFenceFaces;
            gfF->initialize(x, gfF->getMetaObject());
            objManager->registerObject(gfF);
        }
        else
        {
            gfF=GeoFenceFaces::GetInstance(objManager, x);
        }

        GeoFenceVertices::DataFields geofenceVerticesData = gfV->getData();
        GeoFenceFaces::DataFields geofenceFacesData = gfF->getData();

        // Convert from LLA to NED for sending to the model
        LLA[0] = myModel->data(myModel->index(x, GeoFenceVerticesDataModel::GEO_LATITUDE)).toDouble();
        LLA[1] = myModel->data(myModel->index(x, GeoFenceVerticesDataModel::GEO_LONGITUDE)).toDouble();
        LLA[2] = myModel->data(myModel->index(x, GeoFenceVerticesDataModel::GEO_ALTITUDE)).toDouble();
        Utils::CoordinateConversions().LLA2NED_HomeLLA(LLA, homeLLA, NED);

        // Fetch the data from the internal model
        geofenceVerticesData.Vertex[GeoFenceVertices::VERTEX_NORTH] = NED[0];
        geofenceVerticesData.Vertex[GeoFenceVertices::VERTEX_EAST]  = NED[1];
        geofenceVerticesData.Vertex[GeoFenceVertices::VERTEX_DOWN]  = NED[2];

//        geofenceFacesData.Vertices[GeoFenceFaces::VERTICES_A] = myModel->data(myModel->index(x, GeoFenceFacesDataModel::VELOCITY)).toUInt();
//        geofenceFacesData.Vertices[GeoFenceFaces::VERTICES_B] = myModel->data(myModel->index(x, GeoFenceFacesDataModel::VELOCITY)).toUInt();
//        geofenceFacesData.Vertices[GeoFenceFaces::VERTICES_C] = myModel->data(myModel->index(x, GeoFenceFacesDataModel::VELOCITY)).toUInt();

        qDebug() << "Another x: " << x << " out of: "<< myModel->rowCount();

        // Send update
        bool ret = robustUpdate(geofenceVerticesData, geofenceFacesData, x);
        if (ret)
            qDebug() << "Successfully updated geo-fence: " << ret;
        else {
            qDebug() << "Geo-fence upload failed";
//            break;
        }
    }

    // Reset metadata
    gfV->setMetadata(initialGFVMeta);
    gfF->setMetadata(initialGFFMeta);
}

/**
 * @brief robustUpdate Upload a waypoint and check for an ACK or retry.
 * @param data The data to set
 * @param instance The instance id
 * @return True if set succeed, false otherwise
 */
bool GeoFenceModelUavoProxy::robustUpdate(GeoFenceVertices::DataFields geofenceVerticesData, GeoFenceFaces::DataFields geofenceFacesData, int instance)
{
    uint8_t success = 0;

    GeoFenceVertices *gfV = GeoFenceVertices::GetInstance(objManager, instance);
    connect(gfV, SIGNAL(transactionCompleted(UAVObject*, bool)), this, SLOT(geofenceTransactionCompleted(UAVObject *, bool)));
    for (int i = 0; i < 10 && success == 0; i++) {
            QEventLoop m_eventloop;
            QTimer::singleShot(500, &m_eventloop, SLOT(quit())); // Allow 500ms for the transaction to complete.
            connect(this, SIGNAL(geofenceTransactionSucceeded()), &m_eventloop, SLOT(quit()));
            connect(this, SIGNAL(geofenceTransactionFailed()), &m_eventloop, SLOT(quit()));
            geofenceTransactionResult.insert(instance, false);
            gfV->setData(geofenceVerticesData);
            gfV->updated();
            qDebug() << "Stupid loop";
            m_eventloop.exec();
            if (geofenceTransactionResult.value(instance)) {
                success++;
                continue;
            }
            else {
                // Wait a bit before next attempt
                QTimer::singleShot(500, &m_eventloop, SLOT(quit()));
                m_eventloop.exec();
            }
    }
    disconnect(gfV, SIGNAL(transactionCompleted(UAVObject*,bool)), this, SLOT(geofenceTransactionCompleted(UAVObject *, bool)));

    GeoFenceFaces *gfF = GeoFenceFaces::GetInstance(objManager, instance);
    connect(gfF, SIGNAL(transactionCompleted(UAVObject*,bool)), this, SLOT(geofenceTransactionCompleted(UAVObject *, bool)));
    for (int i = 0; i < 10 && success == 1; i++) {
            QEventLoop m_eventloop;
            QTimer::singleShot(500, &m_eventloop, SLOT(quit()));
            connect(this, SIGNAL(geofenceTransactionSucceeded()), &m_eventloop, SLOT(quit()));
            connect(this, SIGNAL(geofenceTransactionFailed()), &m_eventloop, SLOT(quit()));
            geofenceTransactionResult.insert(instance, false);
            gfF->setData(geofenceFacesData);
            gfF->updated();
            m_eventloop.exec();
            if (geofenceTransactionResult.value(instance)) {
                success++;
                continue;
            }
            else {
                // Wait a bit before next attempt
                QTimer::singleShot(500, &m_eventloop, SLOT(quit()));
                m_eventloop.exec();
            }
    }
    disconnect(gfF, SIGNAL(transactionCompleted(UAVObject*,bool)), this, SLOT(geofenceTransactionCompleted(UAVObject *, bool)));

    return success;
}

/**
 * @brief geofenceTransactionCompleted Map from the transaction complete to whether it
 * did or not
 */
void GeoFenceModelUavoProxy::geofenceTransactionCompleted(UAVObject *obj, bool success)
{
    geofenceTransactionResult.insert(obj->getInstID(), success);
    if (success) {
        qDebug() << "Success " << obj->getInstID();
        emit geofenceTransactionSucceeded();
    } else {
        qDebug() << "Failed transaction " << obj->getInstID();
        emit geofenceTransactionFailed();
    }
}

/**
 * @brief GeoFenceModelUavoProxy::objectsToModel Take the existing UAV objects and
 * update the GCS model accordingly
 */
void GeoFenceModelUavoProxy::objectsToModel()
{
    double homeLLA[3];
    getHomeLocation(homeLLA);
    double LLA[3];

    // Remove all rows
    myModel->removeRows(0,myModel->rowCount());

    // Add rows back
    for(int x=0; x < objManager->getNumInstances(GeoFenceVertices::OBJID); ++x)
    {
/*        // Convert from LLA to NED for sending to the model
        LLA[0] = myModel->data(myModel->index(x, GeoFenceVerticesDataModel::GEO_LATITUDE)).toDouble();
        LLA[1] = myModel->data(myModel->index(x, GeoFenceVerticesDataModel::GEO_LONGITUDE)).toDouble();
        LLA[2] = myModel->data(myModel->index(x, GeoFenceVerticesDataModel::GEO_ALTITUDE)).toDouble();
        Utils::CoordinateConversions().LLA2NED_HomeLLA(LLA, homeLLA, NED);

        // Fetch the data from the internal model
        geofenceVerticesData.Vertex[GeoFenceVertices::VERTEX_NORTH] = NED[0];
        geofenceVerticesData.Vertex[GeoFenceVertices::VERTEX_EAST]  = NED[1];
        geofenceVerticesData.Vertex[GeoFenceVertices::VERTEX_DOWN]  = NED[2];
*/

        GeoFenceVertices *gfV;
        GeoFenceVertices::DataFields gfVdata;

        gfV = GeoFenceVertices::GetInstance(objManager,x);

        Q_ASSERT(gfV);
        if(!gfV)
            continue;

        // Get the waypoint data from the object manager and prepare a row in the internal model
        gfVdata = gfV->getData();
        myModel->insertRow(x);

        // Compute the coordinates in LLA
        double NED[3] = {gfVdata.Vertex[0], gfVdata.Vertex[1], gfVdata.Vertex[2]};
        Utils::CoordinateConversions().NED2LLA_HomeLLA(homeLLA, NED, LLA);

        // Store the data
        myModel->setData(myModel->index(x, GeoFenceVerticesDataModel::GEO_LATITUDE), LLA[0]);
        myModel->setData(myModel->index(x, GeoFenceVerticesDataModel::GEO_LONGITUDE), LLA[1]);
        myModel->setData(myModel->index(x, GeoFenceVerticesDataModel::GEO_ALTITUDE), LLA[2]);
        myModel->setData(myModel->index(x, GeoFenceVerticesDataModel::GEO_VERTEX_ID), x);
    }
}

/**
 * @brief GeoFenceModelUavoProxy::getHomeLocation Take care of scaling the home location UAVO to
 * degrees (lat lon) and meters altitude
 * @param [out] home A 3 element double array to store resul in
 * @return True if successful, false otherwise
 */
bool GeoFenceModelUavoProxy::getHomeLocation(double *homeLLA)
{
    // Compute the coordinates in LLA
    HomeLocation *home = HomeLocation::GetInstance(objManager);
    if (home == NULL)
        return false;

    HomeLocation::DataFields homeLocation = home->getData();
    homeLLA[0] = homeLocation.Latitude / 1e7;
    homeLLA[1] = homeLocation.Longitude / 1e7;
    homeLLA[2] = homeLocation.Altitude;

    return true;
}

