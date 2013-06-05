/**
 ******************************************************************************
 * @file       geofencedialog.cpp
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2013.
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

#include "ui_geofence_dialog.h"

#include "geofencedialog.h"
#include "geofencemodeluavoproxy.h"

#include <QFileDialog>
#include "extensionsystem/pluginmanager.h"

GeoFenceDialog::GeoFenceDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::GeoFenceDialog)
{
    ui->setupUi(this);
}

GeoFenceDialog::~GeoFenceDialog()
{
    delete ui;
}

void GeoFenceDialog::setModel(GeoFenceVerticesDataModel *verticesModel, QItemSelectionModel *verticesSelection, GeoFenceFacesDataModel *facesModel, QItemSelectionModel *facesSelection)
{
    geofenceVerticesDataModel = verticesModel;
    ui->tvGeoVerticesFence->setModel(geofenceVerticesDataModel);
    ui->tvGeoVerticesFence->setSelectionModel(verticesSelection);
    ui->tvGeoVerticesFence->setSelectionBehavior(QAbstractItemView::SelectRows);
    connect(geofenceVerticesDataModel, SIGNAL(rowsInserted(const QModelIndex&, int, int)), this, SLOT(rowsInserted(const QModelIndex&, int, int)));
//    ui->tvGeoFence->resizeColumnsToContents();

    geofenceFacesDataModel = facesModel;
    ui->tvGeoFacesFence->setModel(geofenceFacesDataModel);
    ui->tvGeoFacesFence->setSelectionModel(facesSelection);
    ui->tvGeoFacesFence->setSelectionBehavior(QAbstractItemView::SelectRows);
    connect(geofenceFacesDataModel, SIGNAL(rowsInserted(const QModelIndex&, int, int)), this, SLOT(rowsInserted(const QModelIndex&, int, int)));

    proxy = new GeoFenceModelUavoProxy(this, verticesModel, facesModel);

}

void GeoFenceDialog::rowsInserted(const QModelIndex &parent, int start, int end)
{
    Q_UNUSED(parent);
    Q_UNUSED(start);
    Q_UNUSED(end);
}


// SHOULD BE IN A WIDGET
//VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV//
void GeoFenceDialog::on_tbAdd_clicked()
{
    ui->tvGeoVerticesFence->model()->insertRow(ui->tvGeoVerticesFence->model()->rowCount());
}

void GeoFenceDialog::on_tbDelete_clicked()
{
    ui->tvGeoVerticesFence->model()->removeRow(ui->tvGeoVerticesFence->selectionModel()->currentIndex().row());
}

void GeoFenceDialog::on_tbInsert_clicked()
{
    ui->tvGeoVerticesFence->model()->insertRow(ui->tvGeoVerticesFence->selectionModel()->currentIndex().row());
}

void GeoFenceDialog::on_tbReadFromFile_clicked()
{
    if(!geofenceVerticesDataModel)
        return;
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"));
    geofenceVerticesDataModel->readFromFile(fileName);
}

void GeoFenceDialog::on_tbSaveToFile_clicked()
{
    if(!geofenceVerticesDataModel)
        return;
    QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"));
    geofenceVerticesDataModel->writeToFile(fileName);
}

/**
 * @brief GeoFenceDialog::on_tbDetails_clicked Display a dialog to show
 * and edit details of a vertex.  The waypoint selected initially will be the
 * highlighted one.
 */
void GeoFenceDialog::on_tbDetails_clicked()
{
    ExtensionSystem::PluginManager *pm = ExtensionSystem::PluginManager::instance();
    Q_ASSERT(pm);
    if (pm == NULL)
        return;

//    WaypointDialog *dialog =  pm->getObject<WaypointDialog>();
//    Q_ASSERT(dialog);
//    dialog->show();
}

/**
 * @brief GeoFenceDialog::on_tbSendToUAV_clicked Use the proxy to send
 * the data from the flight model to the UAV
 */
void GeoFenceDialog::on_tbSendToUAV_clicked()
{
    proxy->modelToObjects();
}

/**
 * @brief GeoFenceDialog::on_tbFetchFromUAV_clicked Use the flight model to
 * get data from the UAV
 */
void GeoFenceDialog::on_tbFetchFromUAV_clicked()
{
    proxy->objectsToModel();
    ui->tvGeoVerticesFence->resizeColumnsToContents();
}
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^//
