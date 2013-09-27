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
#include "modeluavoproxy.h"
#include "extensionsystem/pluginmanager.h"
#include <math.h>

#include "utils/coordinateconversions.h"
#include "homelocation.h"

//! Initialize the model uavo proxy
ModelUavoProxy::ModelUavoProxy(QObject *parent, WaypointDataModel *waypointModel, PathSegmentDataModel *pathSegmentModel):
    QObject(parent),
    waypointModel(waypointModel),
    pathSegmentModel(pathSegmentModel)
{
    ExtensionSystem::PluginManager *pm = ExtensionSystem::PluginManager::instance();
    Q_ASSERT(pm != NULL);
    objManager = pm->getObject<UAVObjectManager>();
    Q_ASSERT(objManager != NULL);
    waypointObj = Waypoint::GetInstance(objManager);
    Q_ASSERT(waypointObj != NULL);
}

/**
 * @brief ModelUavoProxy::modelToObjects Cast from the internal representation of a path
 * to the UAV objects required to represent it
 */
void ModelUavoProxy::modelToObjects()
{
    // Get UAVOs and return if this fails
    Waypoint *waypoint = Waypoint::GetInstance(objManager,0);
    Q_ASSERT(waypoint);
    if (waypoint == NULL)
        return;

    PathSegmentDescriptor *pathSegmentDescriptor = PathSegmentDescriptor::GetInstance(objManager,0);
    Q_ASSERT(pathSegmentDescriptor);
    if (pathSegmentDescriptor == NULL)
        return;

    /* First handle waypoints... */

    // Make sure the waypoint object is acked
    UAVObject::Metadata initialMeta = waypoint->getMetadata();
    UAVObject::Metadata meta = initialMeta;
    UAVObject::SetFlightTelemetryAcked(meta, true);
    waypoint->setMetadata(meta);

    double homeLLA[3];
    double NED[3];
    double LLA[3];
    getHomeLocation(homeLLA);

    for(int x=0; x < waypointModel->rowCount(); ++x)
    {
        Waypoint *wp = NULL;

        // Get the number of existing waypoints
        int instances = Waypoint::getNumInstances(objManager);

        // Create new instances of waypoints if this is more than exist
        if(x > instances - 1)
        {
            wp = new Waypoint;
            wp->initialize(x,wp->getMetaObject());
            objManager->registerObject(wp);
        }
        else
        {
            wp = Waypoint::GetInstance(objManager,x);
        }

        Q_ASSERT(wp);
        Waypoint::DataFields waypointData = wp->getData();

        // Convert from LLA to NED for sending to the model
        LLA[0] = waypointModel->data(waypointModel->index(x,WaypointDataModel::LATPOSITION)).toDouble();
        LLA[1] = waypointModel->data(waypointModel->index(x,WaypointDataModel::LNGPOSITION)).toDouble();
        LLA[2] = waypointModel->data(waypointModel->index(x,WaypointDataModel::ALTITUDE)).toDouble();
        Utils::CoordinateConversions().LLA2NED_HomeLLA(LLA, homeLLA, NED);

        // Fetch the data from the internal model
        waypointData.Velocity=waypointModel->data(waypointModel->index(x,WaypointDataModel::VELOCITY)).toDouble();
        waypointData.Position[Waypoint::POSITION_NORTH] = NED[0];
        waypointData.Position[Waypoint::POSITION_EAST]  = NED[1];
        waypointData.Position[Waypoint::POSITION_DOWN]  = NED[2];
        waypointData.Mode = waypointModel->data(waypointModel->index(x,WaypointDataModel::MODE), Qt::UserRole).toInt();
        waypointData.ModeParameters = waypointModel->data(waypointModel->index(x,WaypointDataModel::MODE_PARAMS)).toDouble();

        if (robustUpdate(waypointData, x))
            qDebug() << "Successfully updated";
        else {
            qDebug() << "Upload failed";
            break;
        }
    }
    waypoint->setMetadata(initialMeta);

    /* ...then the path segment descriptors */
    // Make sure the path segment descriptor object is acked
    initialMeta = pathSegmentDescriptor->getMetadata();
    meta = initialMeta;
    UAVObject::SetFlightTelemetryAcked(meta, true);
    pathSegmentDescriptor->setMetadata(meta);

    for(int x=0; x < pathSegmentModel->rowCount(); ++x)
    {
        PathSegmentDescriptor *psd = NULL;

        // Get the number of existing waypoints
        int instances = PathSegmentDescriptor::getNumInstances(objManager);

        // Create new instances of waypoints if this is more than exist
        if(x > instances - 1)
        {
            psd = new PathSegmentDescriptor;
            psd->initialize(x, psd->getMetaObject());
            objManager->registerObject(psd);
        }
        else
        {
            psd = PathSegmentDescriptor::GetInstance(objManager,x);
        }

        Q_ASSERT(psd);
        PathSegmentDescriptor::DataFields pathSegmentDescriptorData = psd->getData();

        // Fetch the data from the internal model
        pathSegmentDescriptorData.SwitchingLocus[0] = pathSegmentModel->data(pathSegmentModel->index(x, PathSegmentDataModel::NED_POS_NORTH)).toDouble();
        pathSegmentDescriptorData.SwitchingLocus[1] = pathSegmentModel->data(pathSegmentModel->index(x, PathSegmentDataModel::NED_POS_EAST)).toDouble();
        pathSegmentDescriptorData.SwitchingLocus[2] = pathSegmentModel->data(pathSegmentModel->index(x, PathSegmentDataModel::NED_POS_DOWN)).toDouble();
        pathSegmentDescriptorData.PathCurvature = pathSegmentModel->data(pathSegmentModel->index(x, PathSegmentDataModel::CURVATURE)).toDouble();
        pathSegmentDescriptorData.NumberOfOrbits = pathSegmentModel->data(pathSegmentModel->index(x, PathSegmentDataModel::NUM_ORBITS)).toInt();
        pathSegmentDescriptorData.ArcRank = pathSegmentModel->data(pathSegmentModel->index(x, PathSegmentDataModel::ARC_RANK)).toInt();

        if (robustUpdate(pathSegmentDescriptorData, x))
            qDebug() << "Successfully updated";
        else {
            qDebug() << "Upload failed";
            break;
        }
    }
    pathSegmentDescriptor->setMetadata(initialMeta);
}

/**
 * @brief robustUpdate Upload a waypoint and check for an ACK or retry.
 * @param data The data to set
 * @param instance The instance id
 * @return True if set succeed, false otherwise
 */
bool ModelUavoProxy::robustUpdate(Waypoint::DataFields data, int instance)
{
    Waypoint *wp = Waypoint::GetInstance(objManager, instance);
    connect(wp, SIGNAL(transactionCompleted(UAVObject*,bool)), this, SLOT(waypointTransactionCompleted(UAVObject *, bool)));
    for (int i = 0; i < 10; i++) {
            QEventLoop m_eventloop;
            QTimer::singleShot(500, &m_eventloop, SLOT(quit()));
            connect(this, SIGNAL(waypointTransactionSucceeded()), &m_eventloop, SLOT(quit()));
            connect(this, SIGNAL(waypointTransactionFailed()), &m_eventloop, SLOT(quit()));
            waypointTransactionResult.insert(instance, false);
            wp->setData(data);
            wp->updated();
            m_eventloop.exec();
            if (waypointTransactionResult.value(instance)) {
                disconnect(wp, SIGNAL(transactionCompleted(UAVObject*,bool)), this, SLOT(waypointTransactionCompleted(UAVObject *, bool)));
                return true;
            }

            // Wait a second for next attempt
            QTimer::singleShot(500, &m_eventloop, SLOT(quit()));
            m_eventloop.exec();
    }

    disconnect(wp, SIGNAL(transactionCompleted(UAVObject*,bool)), this, SLOT(waypointTransactionCompleted(UAVObject *, bool)));

    // None of the attempt got an ack
    return false;
}


/**
 * @brief robustUpdate Upload a path segment descriptor and check for an ACK or retry.
 * @param data The data to set
 * @param instance The instance id
 * @return True if set succeed, false otherwise
 */
bool ModelUavoProxy::robustUpdate(PathSegmentDescriptor::DataFields data, int instance)
{
    PathSegmentDescriptor *psd = PathSegmentDescriptor::GetInstance(objManager, instance);
    connect(psd, SIGNAL(transactionCompleted(UAVObject*,bool)), this, SLOT(pathSegmentDescriptorTransactionCompleted(UAVObject *, bool)));
    for (int i = 0; i < 10; i++) {
            QEventLoop m_eventloop;
            QTimer::singleShot(500, &m_eventloop, SLOT(quit()));
            connect(this, SIGNAL(pathSegmentDescriptorTransactionSucceeded()), &m_eventloop, SLOT(quit()));
            connect(this, SIGNAL(pathSegmentDescriptorTransactionFailed()), &m_eventloop, SLOT(quit()));
            pathSegmentDescriptorTransactionResult.insert(instance, false);
            psd->setData(data);
            psd->updated();
            m_eventloop.exec();
            if (pathSegmentDescriptorTransactionResult.value(instance)) {
                disconnect(psd, SIGNAL(transactionCompleted(UAVObject*,bool)), this, SLOT(pathSegmentDescriptorTransactionCompleted(UAVObject *, bool)));
                return true;
            }

            // Wait a second for next attempt
            QTimer::singleShot(500, &m_eventloop, SLOT(quit()));
            m_eventloop.exec();
    }

    disconnect(psd, SIGNAL(transactionCompleted(UAVObject*,bool)), this, SLOT(pathSegmentDescriptorTransactionCompleted(UAVObject *, bool)));

    // None of the attempt got an ack
    return false;
}



/**
 * @brief pathSegmentDescriptorTransactionCompleted Map from the transaction complete to whether it
 * did or not
 */
void ModelUavoProxy::pathSegmentDescriptorTransactionCompleted(UAVObject *obj, bool success) {
    Q_ASSERT(obj->getObjID() == PathSegmentDescriptor::OBJID);
    pathSegmentDescriptorTransactionResult.insert(obj->getInstID(), success);
    if (success) {
        qDebug() << "Success " << obj->getInstID();
        emit pathSegmentDescriptorTransactionSucceeded();
    } else {
        qDebug() << "Failed transaction " << obj->getInstID();
        emit pathSegmentDescriptorTransactionFailed();
    }
}


/**
 * @brief waypointTransactionCompleted Map from the transaction complete to whether it
 * did or not
 */
void ModelUavoProxy::waypointTransactionCompleted(UAVObject *obj, bool success) {
    Q_ASSERT(obj->getObjID() == Waypoint::OBJID);
    waypointTransactionResult.insert(obj->getInstID(), success);
    if (success) {
        qDebug() << "Success " << obj->getInstID();
        emit waypointTransactionSucceeded();
    } else {
        qDebug() << "Failed transaction " << obj->getInstID();
        emit waypointTransactionFailed();
    }
}

/**
 * @brief ModelUavoProxy::objectsToModel Take the existing UAV objects and
 * update the GCS model accordingly
 */
void ModelUavoProxy::objectsToModel()
{
    double homeLLA[3];
    getHomeLocation(homeLLA);
    double LLA[3];

    /* First handle waypoints... */
    waypointModel->removeRows(0,waypointModel->rowCount());
    for(int x=0; x < Waypoint::getNumInstances(objManager) ; ++x) {
        Waypoint * wp;
        Waypoint::DataFields wpfields;

        wp = Waypoint::GetInstance(objManager,x);
        Q_ASSERT(wp);

        if(!wp)
            continue;

        // Get the waypoint data from the object manager and prepare a row in the internal model
        wpfields = wp->getData();
        waypointModel->insertRow(x);

        // Compute the coordinates in LLA
        double NED[3] = {wpfields.Position[0], wpfields.Position[1], wpfields.Position[2]};
        Utils::CoordinateConversions().NED2LLA_HomeLLA(homeLLA, NED, LLA);

        // Store the data
        waypointModel->setData(waypointModel->index(x, WaypointDataModel::LATPOSITION), LLA[0]);
        waypointModel->setData(waypointModel->index(x, WaypointDataModel::LNGPOSITION), LLA[1]);
        waypointModel->setData(waypointModel->index(x, WaypointDataModel::ALTITUDE), LLA[2]);
        waypointModel->setData(waypointModel->index(x, WaypointDataModel::VELOCITY), wpfields.Velocity);
        waypointModel->setData(waypointModel->index(x, WaypointDataModel::MODE), wpfields.Mode);
        waypointModel->setData(waypointModel->index(x, WaypointDataModel::MODE_PARAMS), wpfields.ModeParameters);
    }

    /* ...then the path segment descriptors */
    pathSegmentModel->removeRows(0,pathSegmentModel->rowCount());
    for(int x=0; x < PathSegmentDescriptor::getNumInstances(objManager); ++x) {
        PathSegmentDescriptor * psd;
        PathSegmentDescriptor::DataFields psdData;

        psd = PathSegmentDescriptor::GetInstance(objManager,x);

        if(!psd)
            continue;

        // Get the path segment descriptor from the object manager and prepare a row in the internal model
        psdData = psd->getData();
        pathSegmentModel->insertRow(x);

        // Store the data
        pathSegmentModel->setData(pathSegmentModel->index(x, PathSegmentDataModel::NED_POS_NORTH), psdData.SwitchingLocus[0]);
        pathSegmentModel->setData(pathSegmentModel->index(x, PathSegmentDataModel::NED_POS_EAST), psdData.SwitchingLocus[1]);
        pathSegmentModel->setData(pathSegmentModel->index(x, PathSegmentDataModel::NED_POS_DOWN), psdData.SwitchingLocus[2]);
        pathSegmentModel->setData(pathSegmentModel->index(x, PathSegmentDataModel::CURVATURE), psdData.PathCurvature);
        pathSegmentModel->setData(pathSegmentModel->index(x, PathSegmentDataModel::NUM_ORBITS), psdData.NumberOfOrbits);
        pathSegmentModel->setData(pathSegmentModel->index(x, PathSegmentDataModel::ARC_RANK), psdData.ArcRank);
    }
}

/**
 * @brief ModelUavoProxy::getHomeLocation Take care of scaling the home location UAVO to
 * degrees (lat lon) and meters altitude
 * @param [out] home A 3 element double array to store resul in
 * @return True if successful, false otherwise
 */
bool ModelUavoProxy::getHomeLocation(double *homeLLA)
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

