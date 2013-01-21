/**
 ******************************************************************************
 *
 * @file       geofencedialog.cpp
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2012.
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

#include "geofencedialog.h"
#include "ui_geofencedialog.h"

GeofenceDialog::GeofenceDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::GeofenceDialog)
{
    ui->setupUi(this);
}

GeofenceDialog::~GeofenceDialog()
{
    delete ui;
}

void GeofenceDialog::setModel(GeofenceDataModel *model, QItemSelectionModel *selection)
{
    dataModel=model;
    ui->tableView->setModel(model);
    ui->tableView->setSelectionModel(selection);
    ui->tableView->setSelectionBehavior(QAbstractItemView::SelectRows);
    connect(dataModel,SIGNAL(rowsInserted(const QModelIndex&,int,int)),this,SLOT(rowsInserted(const QModelIndex&,int,int)));
    ui->tableView->resizeColumnsToContents();
}

void GeofenceDialog::rowsInserted(const QModelIndex &parent, int start, int end)
{
}
