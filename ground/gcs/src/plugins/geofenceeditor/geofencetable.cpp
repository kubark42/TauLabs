/**
 ******************************************************************************
 * @file       geofencetable.cpp
 * @author     Tau Labs, http://taulabs.org Copyright (C) 2013.
 *
 * @addtogroup GCSPlugins GCS Plugins
 * @{
 * @addtogroup GeoFenceEditorGadgetPlugin GeoFence Editor Gadget Plugin
 * @{
 * @brief A gadget to edit a geofence mesh
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
#include <QBrush>
#include "geofencetable.h"
#include "extensionsystem/pluginmanager.h"

GeoFenceTable::GeoFenceTable(QObject *parent) :
    QAbstractTableModel(parent)
{
    ExtensionSystem::PluginManager *pm = ExtensionSystem::PluginManager::instance();
    Q_ASSERT(pm != NULL);
    objManager = pm->getObject<UAVObjectManager>();
    Q_ASSERT(objManager != NULL);
    geofenceObj = GeoFence::GetInstance(objManager);
    Q_ASSERT(geofenceObj != NULL);
    geofenceActiveObj = GeoFenceActive::GetInstance(objManager);
    Q_ASSERT(geofenceActiveObj != NULL);

    elements = 0;

    // Unfortunately there is no per object new instance signal yet
    connect(objManager, SIGNAL(newInstance(UAVObject*)),
            this, SLOT(doNewInstance(UAVObject*)));
    connect(geofenceActiveObj, SIGNAL(objectUpdated(UAVObject*)),
            this, SLOT(geofencesUpdated(UAVObject*)));

    int numWaypoints = objManager->getNumInstances(GeoFence::OBJID);
    for (int i = 0; i < numWaypoints; i++) {
        GeoFence *geofence = GeoFence::GetInstance(objManager, i);
        connect(geofence, SIGNAL(objectUpdated(UAVObject*)), this, SLOT(geofencesUpdated(UAVObject*)));
    }

    headers.clear();
    headers.append(QString("North"));
    headers.append(QString("East"));
    headers.append(QString("Down"));
}

int GeoFenceTable::rowCount(const QModelIndex & /*parent*/) const
{
    return elements;
}

int GeoFenceTable::columnCount(const QModelIndex &parent) const
{
    if (parent.isValid())
        return 0;

    return headers.length();
}

QVariant GeoFenceTable::data(const QModelIndex &index, int role) const
{
    if(role == Qt::DisplayRole) {
        GeoFence *obj = GeoFence::GetInstance(objManager, index.row());
        Q_ASSERT(obj);
        GeoFence::DataFields geofence = obj->getData();

        switch(index.column()) {
        case 0:
            return geofence.Position[GeoFence::POSITION_NORTH];
        case 1:
            return geofence.Position[GeoFence::POSITION_EAST];
        case 2:
            return geofence.Position[GeoFence::POSITION_DOWN];
        default:
            Q_ASSERT(0);
            return 0;
        }
    } else if (role == Qt::BackgroundRole) {
        GeoFenceActive::DataFields geofenceActive = geofenceActiveObj->getData();

        if(index.row() == geofenceActive.Index) {
            return QBrush(Qt::lightGray);
        } else
            return QVariant::Invalid;
    }
    else {
        return QVariant::Invalid;
    }

}

QVariant GeoFenceTable::headerData(int section, Qt::Orientation orientation, int role) const
{
    if(role == Qt::DisplayRole && orientation == Qt::Horizontal) {
        if(section < headers.length())
            return headers[section];
        return QVariant::Invalid;
    } else
        return QAbstractTableModel::headerData(section, orientation, role);
}

/**
  * Called for any new UAVO instance and when that is a GeoFence register
  * to update the table
  */
void GeoFenceTable::doNewInstance(UAVObject*obj)
{
    Q_ASSERT(obj);
    if (!obj)
        return;

    if (obj->getObjID() == GeoFence::OBJID)
        connect(obj, SIGNAL(objectUpdated(UAVObject*)),this,SLOT(geofencesUpdated(UAVObject*)));
}

/**
  * Called whenever the geofences are updated to inform
  * the view
  */
void GeoFenceTable::geofencesUpdated(UAVObject *)
{
     int elementsNow = objManager->getNumInstances(geofenceObj->getObjID());

    // Currently only support adding instances which is all the UAVO manager
    // supports
    if (elementsNow > elements) {
        beginInsertRows(QModelIndex(), elements, elementsNow-1);
        elements = elementsNow;
        endInsertRows();
    }

    QModelIndex i1 = index(0,0);
    QModelIndex i2 = index(elements-1, columnCount(QModelIndex()));
    emit dataChanged(i1, i2);
}

Qt::ItemFlags GeoFenceTable::flags(const QModelIndex &index) const
{
    return QAbstractTableModel::flags(index) | Qt::ItemIsEditable;
}

bool GeoFenceTable::setData ( const QModelIndex & index, const QVariant & value, int role = Qt::EditRole )
{
    Q_UNUSED(role);

    double val = value.toDouble();
    qDebug() << "New value " << val << " for column " << index.column();

    GeoFence *obj = GeoFence::GetInstance(objManager, index.row());
    Q_ASSERT(obj);
    GeoFence::DataFields geofence = obj->getData();

    switch(index.column()) {
    case 0:
        geofence.Position[GeoFence::POSITION_NORTH] = val;
        break;
    case 1:
        geofence.Position[GeoFence::POSITION_EAST] = val;
        break;
    case 2:
        geofence.Position[GeoFence::POSITION_DOWN] = val;
        break;
    default:
        return false;
    }

    obj->setData(geofence);
    obj->updated();
    qDebug() << "Set data for instance " << obj->getInstID();

    return true;
}
