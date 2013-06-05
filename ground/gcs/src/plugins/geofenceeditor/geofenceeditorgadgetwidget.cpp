/**
 ******************************************************************************
 * @file       geofenceeditorgadgetwidget.cpp
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2013.
 * @addtogroup GCSPlugins GCS Plugins
 * @{
 * @addtogroup GeoFenceEditorGadgetPlugin Geo-fence Editor Gadget Plugin
 * @{
 * @brief A gadget to edit a list of waypoints
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
#include "geofenceeditorgadgetwidget.h"
//#include "waypointdialog.h"
//#include "waypointdelegate.h"
#include "ui_geofence_dialog.h"

#include <QDomDocument>
#include <QFileDialog>
#include <QMessageBox>
#include <QPushButton>
#include <QString>
#include <QStringList>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QWidget>

#include "extensionsystem/pluginmanager.h"

GeoFenceEditorGadgetWidget::GeoFenceEditorGadgetWidget(QWidget *parent) :
    QLabel(parent)
{
    ui = new Ui_GeoFenceDialog();
    ui->setupUi(this);

    ExtensionSystem::PluginManager *pm = ExtensionSystem::PluginManager::instance();
    GeoFenceVerticesDataModel *verticesModel = pm->getObject<GeoFenceVerticesDataModel>();
    GeoFenceFacesDataModel *facesModel = pm->getObject<GeoFenceFacesDataModel>();
    Q_ASSERT(verticesModel);
    Q_ASSERT(facesModel);

    QItemSelectionModel *verticesSelection = pm->getObject<QItemSelectionModel>();
    QItemSelectionModel *facesSelection = pm->getObject<QItemSelectionModel>();
    Q_ASSERT(verticesSelection);
    Q_ASSERT(facesSelection);
    setModel(verticesModel, verticesSelection, facesModel, facesSelection);
}

GeoFenceEditorGadgetWidget::~GeoFenceEditorGadgetWidget()
{
   // Do nothing
}


void GeoFenceEditorGadgetWidget::setModel(GeoFenceVerticesDataModel *verticesModel, QItemSelectionModel *verticesSelection, GeoFenceFacesDataModel *facesModel, QItemSelectionModel *facesSelection)
{
    verticesDataModel = verticesModel;
    ui->tvGeoVerticesFence->setModel(verticesDataModel);
    ui->tvGeoVerticesFence->setSelectionModel(verticesSelection);
    ui->tvGeoVerticesFence->setSelectionBehavior(QAbstractItemView::SelectRows);
    connect(verticesDataModel, SIGNAL(rowsInserted(const QModelIndex&, int, int)), this, SLOT(rowsInserted(const QModelIndex&, int, int)));
//    ui->tvGeoFence->resizeColumnsToContents();

    facesDataModel = facesModel;
    ui->tvGeoFacesFence->setModel(facesDataModel);
    ui->tvGeoFacesFence->setSelectionModel(facesSelection);
    ui->tvGeoFacesFence->setSelectionBehavior(QAbstractItemView::SelectRows);
    connect(facesDataModel, SIGNAL(rowsInserted(const QModelIndex&, int, int)), this, SLOT(rowsInserted(const QModelIndex&, int, int)));

    proxy = new GeoFenceModelUavoProxy(this, verticesDataModel, facesDataModel);
}


void GeoFenceEditorGadgetWidget::on_tbAdd_clicked()
{
//    ui->tableView->model()->insertRow(ui->tableView->model()->rowCount());
}

void GeoFenceEditorGadgetWidget::on_tbDelete_clicked()
{
//    ui->tableView->model()->removeRow(ui->tableView->selectionModel()->currentIndex().row());
}

void GeoFenceEditorGadgetWidget::on_tbInsert_clicked()
{
//    ui->tableView->model()->insertRow(ui->tableView->selectionModel()->currentIndex().row());
}

void GeoFenceEditorGadgetWidget::on_tbReadFromFile_clicked()
{
    if(!verticesDataModel || !facesDataModel)
        return;

    // Get file name from file picker dialog
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"));

    // Open file and read XML contents
    QFile file(fileName);
    file.open(QIODevice::ReadOnly);
    QDomDocument doc("Geofence");
    QByteArray array=file.readAll();
    QString error;
    if (!doc.setContent(array,&error)) {
        QMessageBox msgBox;
        msgBox.setText(tr("File Parsing Failed."));
        msgBox.setInformativeText(QString(tr("This file is not a correct XML file:%0")).arg(error));
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();
        return;
    }
    file.close();

    QDomElement root = doc.documentElement();

    if (root.isNull() || (root.tagName() != "dataModel")) {
        QMessageBox msgBox;
        msgBox.setText(tr("Wrong file contents"));
        msgBox.setInformativeText(tr("This file does not contain correct UAVSettings"));
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();
        return;
    }


    // Parse log for both data models
    verticesDataModel->readFromFile(root);
    facesDataModel->readFromFile(root);

    // TODO: Check if home is inside the geofence. If not, ask if the user would like to make the geofence relative to the home coordinates
}

void GeoFenceEditorGadgetWidget::on_tbSaveToFile_clicked()
{
//    if(!model)
//        return;
//    QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"));
//    model->writeToFile(fileName);
}

/**
 * @brief GeoFenceEditorGadgetWidget::on_tbDetails_clicked Display a dialog to show
 * and edit details of a waypoint.  The waypoint selected initially will be the
 * highlighted one.
 */
void GeoFenceEditorGadgetWidget::on_tbDetails_clicked()
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
 * @brief GeoFenceEditorGadgetWidget::on_tbSendToUAV_clicked Use the proxy to send
 * the data from the flight model to the UAV
 */
void GeoFenceEditorGadgetWidget::on_tbSendToUAV_clicked()
{
    proxy->modelToObjects();
}

/**
 * @brief GeoFenceEditorGadgetWidget::on_tbFetchFromUAV_clicked Use the flight model to
 * get data from the UAV
 */
void GeoFenceEditorGadgetWidget::on_tbFetchFromUAV_clicked()
{
    proxy->objectsToModel();
//    ui->tableView->resizeColumnsToContents();
}


/**
  * @}
  * @}
  */
