/**
 ******************************************************************************
 * @file       geofenceeditorgadgetwidget.cpp
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
#include "geofenceeditorgadgetwidget.h"
#include "ui_geofenceeditor.h"

#include <QDebug>
#include <QString>
#include <QStringList>
#include <QtGui/QWidget>
#include <QtGui/QTextEdit>
#include <QtGui/QVBoxLayout>
#include <QtGui/QPushButton>

#include "extensionsystem/pluginmanager.h"

GeoFenceEditorGadgetWidget::GeoFenceEditorGadgetWidget(QWidget *parent) : QLabel(parent)
{
    m_geofenceeditor = new Ui_GeoFenceEditor();
    m_geofenceeditor->setupUi(this);

    geofenceTable = new GeoFenceTable(this);
    m_geofenceeditor->geofences->setModel(geofenceTable);

    ExtensionSystem::PluginManager *pm = ExtensionSystem::PluginManager::instance();
    Q_ASSERT(pm != NULL);
    UAVObjectManager *objManager = pm->getObject<UAVObjectManager>();
    Q_ASSERT(objManager != NULL);
    geofenceObj = GeoFence::GetInstance(objManager);
    Q_ASSERT(geofenceObj != NULL);

    // Connect the signals
    connect(m_geofenceeditor->buttonNewWaypoint, SIGNAL(clicked()),
            this, SLOT(addInstance()));
}

GeoFenceEditorGadgetWidget::~GeoFenceEditorGadgetWidget()
{
   // Do nothing
}

void GeoFenceEditorGadgetWidget::waypointChanged(UAVObject *)
{
}

void GeoFenceEditorGadgetWidget::geofenceActiveChanged(UAVObject *)
{
}

void GeoFenceEditorGadgetWidget::addInstance()
{
    ExtensionSystem::PluginManager *pm = ExtensionSystem::PluginManager::instance();
    Q_ASSERT(pm != NULL);
    UAVObjectManager *objManager = pm->getObject<UAVObjectManager>();
    Q_ASSERT(objManager != NULL);

    qDebug() << "Instances before: " << objManager->getNumInstances(geofenceObj->getObjID());
    GeoFence *obj = new GeoFence();
    quint32 newInstId = objManager->getNumInstances(geofenceObj->getObjID());
    obj->initialize(newInstId,obj->getMetaObject());
    objManager->registerObject(obj);
    qDebug() << "Instances after: " << objManager->getNumInstances(geofenceObj->getObjID());
}

/**
  * @}
  * @}
  */
