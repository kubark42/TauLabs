/**
 ******************************************************************************
 * @file       pathsegmentdialog.cpp
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2013
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2012.
 * @addtogroup GCSPlugins GCS Plugins
 * @{
 * @addtogroup PathPlanner Map Plugin
 * @{
 * @brief Path segment editor dialog
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

#include "pathsegmentdialog.h"
#include <waypointdelegate.h>
#include <waypoint.h>
#include "ui_waypoint_dialog.h"

PathSegmentDialog::PathSegmentDialog(QWidget *parent, QAbstractItemModel *model,QItemSelectionModel *selection) :
    QDialog(parent,Qt::Window), ui(new Ui_waypoint_dialog),
    model(model), itemSelection(selection)
{
    ui->setupUi(this);
    connect(ui->cbMode,SIGNAL(currentIndexChanged(int)),this,SLOT(setupModeWidgets()));

    // Connect up the buttons
    connect(ui->pushButtonOK, SIGNAL(clicked()), this, SLOT(onOkButton_clicked()));
    connect(ui->pushButtonCancel, SIGNAL(clicked()), this, SLOT(onCancelButton_clicked()));
    connect(ui->pushButtonPrevious, SIGNAL(clicked()), this, SLOT(onPreviousButton_clicked()));
    connect(ui->pushButtonNext, SIGNAL(clicked()), this, SLOT(onNextButton_clicked()));

    mapper = new QDataWidgetMapper(this);

    WaypointDelegate *delegate = new WaypointDelegate(this);
    delegate->loadComboBox(ui->cbMode);

    mapper->setItemDelegate(delegate);
    connect (mapper,SIGNAL(currentIndexChanged(int)),this,SLOT(currentIndexChanged(int)));
    mapper->setModel(model);
    mapper->setSubmitPolicy(QDataWidgetMapper::AutoSubmit);
    mapper->addMapping(ui->doubleSpinBoxLatitude,WaypointDataModel::LATPOSITION);
    mapper->addMapping(ui->doubleSpinBoxLongitude,WaypointDataModel::LNGPOSITION);
    mapper->addMapping(ui->doubleSpinBoxAltitude,WaypointDataModel::ALTITUDE);
    mapper->addMapping(ui->doubleSpinBoxNorth,WaypointDataModel::NED_NORTH);
    mapper->addMapping(ui->doubleSpinBoxEast,WaypointDataModel::NED_EAST);
    mapper->addMapping(ui->doubleSpinBoxDown,WaypointDataModel::NED_DOWN);
    mapper->addMapping(ui->lineEditDescription,WaypointDataModel::WPDESCRITPTION);
    mapper->addMapping(ui->doubleSpinBoxVelocity,WaypointDataModel::VELOCITY);
    mapper->addMapping(ui->cbMode,WaypointDataModel::MODE);
    mapper->addMapping(ui->dsb_modeParams,WaypointDataModel::MODE_PARAMS);
    mapper->addMapping(ui->checkBoxLocked,WaypointDataModel::LOCKED);

    // Make sure the model catches updates from the check box
    //connect(ui->checkBoxLocked,SIGNAL(toggled(bool)),mapper,SLOT(submit()));
    connect(ui->checkBoxLocked, SIGNAL(stateChanged(int)), mapper, SLOT(submit()));

    mapper->setCurrentIndex(selection->currentIndex().row());

    // Support locking the controls when locked
    enableEditWidgets();
    connect(model,SIGNAL(dataChanged(QModelIndex,QModelIndex)),this,SLOT(enableEditWidgets()));

    // This means whenever the model changes we show those changes.  Since the update is on
    // auto submit changes are still permitted.
    connect(model, SIGNAL(dataChanged(QModelIndex,QModelIndex)), mapper, SLOT(revert()));

    connect(itemSelection,SIGNAL(currentRowChanged(QModelIndex,QModelIndex)),this,SLOT(currentRowChanged(QModelIndex,QModelIndex)));

    setModal(true);
}

/**
 * @brief PathSegmentDialog::currentIndexChanged Called when the data widget selector index
 * changes
 * @param index The newly selected index
 */
void PathSegmentDialog::currentIndexChanged(int index)
{
    ui->lbNumber->setText(QString::number(index+1));
    QModelIndex idx=mapper->model()->index(index,0);
    if(index==itemSelection->currentIndex().row())
        return;
    itemSelection->clear();
    itemSelection->setCurrentIndex(idx,QItemSelectionModel::Select | QItemSelectionModel::Rows);
}

PathSegmentDialog::~PathSegmentDialog()
{
    delete ui;
}

/**
 * @brief PathSegmentDialog::setupModeWidgets Whenever the waypoint mode type changes
 * this updates the UI to display the available options (e.g. radius)
 */
void PathSegmentDialog::setupModeWidgets()
{
    int mode = ui->cbMode->itemData(ui->cbMode->currentIndex()).toInt();
    switch(mode)
    {
    case Waypoint::MODE_FLYCIRCLERIGHT:
    case Waypoint::MODE_FLYCIRCLELEFT:
    case Waypoint::MODE_DRIVECIRCLELEFT:
    case Waypoint::MODE_DRIVECIRCLERIGHT:
        ui->modeParams->setVisible(true);
        ui->modeParams->setText(tr("Radius"));
        ui->dsb_modeParams->setVisible(true);
        break;
    default:
        ui->modeParams->setVisible(false);
        ui->dsb_modeParams->setVisible(false);
        break;
    }
}

/**
 * @brief PathSegmentDialog::editWaypoint Edit the requested waypoint, show dialog if it is not showing
 * @param[in] number The waypoint to edit
 */
void PathSegmentDialog::editWaypoint(int number)
{
    if(!isVisible())
        show();
    if(isMinimized())
        showNormal();
    if(!isActiveWindow())
        activateWindow();
    raise();
    setFocus(Qt::OtherFocusReason);
    mapper->setCurrentIndex(number);
}

//! Close the dialog button, accept the changes
void PathSegmentDialog::onOkButton_clicked()
{
    mapper->submit();
    close();
}

//! Close the dialog button, revert any changes
void PathSegmentDialog::onCancelButton_clicked()
{
    mapper->revert();
    close();
}

//! User requests the previous waypoint
void PathSegmentDialog::onPreviousButton_clicked()
{
    mapper->toPrevious();
}

//! User requests the next waypoint
void PathSegmentDialog::onNextButton_clicked()
{
    mapper->toNext();
}

/**
 * @brief PathSegmentDialog::currentRowChanged When the selector changes pass the
 * update to the data mapper
 * @param current The newly selected index
 */
void PathSegmentDialog::currentRowChanged(QModelIndex current, QModelIndex previous)
{
    Q_UNUSED(previous);

    mapper->setCurrentIndex(current.row());
}

/**
 * @brief PathSegmentDialog::enableEditWidgets Enable or disable the controls based
 * on the lock control
 * @param[in] value True if they should be enabled, false to disable
 */
void PathSegmentDialog::enableEditWidgets()
{
    int row = itemSelection->currentIndex().row();
    bool value = false;
//    bool value = model->data(model->index(row,WaypointDataModel::LOCKED)).toBool(); // FIXME: Properly handle whether these are editable or not.
    QWidget * w;
    foreach(QWidget * obj,this->findChildren<QWidget *>())
    {
        w=qobject_cast<QComboBox*>(obj);
        if(w)
            w->setEnabled(!value);
        w=qobject_cast<QLineEdit*>(obj);
        if(w)
            w->setEnabled(!value);
        w=qobject_cast<QDoubleSpinBox*>(obj);
        if(w)
            w->setEnabled(!value);
        w=qobject_cast<QSpinBox*>(obj);
        if(w)
            w->setEnabled(!value);
    }
}
