/**
 ******************************************************************************
 * @file       geofencetable.h
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

#ifndef GEOFENCETABLE_H
#define GEOFENCETABLE_H

#include <QAbstractTableModel>
#include <QList>
#include <QString>
#include <geofencefaces.h>
#include <geofencevertices.h>

class GeoFenceTable : public QAbstractTableModel
{
    Q_OBJECT
public:
    explicit GeoFenceTable(QObject *parent = 0);

    // Get dimensionality of the data
    int rowCount(const QModelIndex &parent) const;
    int columnCount(const QModelIndex &parent) const;

    // Access data
    QVariant data(const QModelIndex &index, int role) const;
    QVariant headerData(int section, Qt::Orientation orientation, int role) const;

    // Functions to make the data editable
    Qt::ItemFlags flags(const QModelIndex &index) const;
    bool setData ( const QModelIndex & index, const QVariant & value, int role);
signals:

protected slots:
    void geofencesUpdated(UAVObject *);
    void doNewInstance(UAVObject*);
public slots:

private:
    UAVObjectManager *objManager;
    GeoFence *geofenceObj;
    GeoFenceActive *geofenceActiveObj;
    QList <QString> headers;
    int elements;
};

#endif // GEOFENCETABLE_H
