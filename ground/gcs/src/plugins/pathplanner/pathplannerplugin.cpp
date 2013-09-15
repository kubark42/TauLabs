/**
 ******************************************************************************
 * @file       pathplannerplugin.cpp
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2013
 * @addtogroup GCSPlugins GCS Plugins
 * @{
 * @addtogroup PathPlannerGadgetPlugin Waypoint Editor Gadget Plugin
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
#include "pathplannerplugin.h"
#include "pathplannergadgetfactory.h"
#include <QtPlugin>
#include <QStringList>
#include <extensionsystem/pluginmanager.h>


PathPlannerPlugin::PathPlannerPlugin()
{
   // Do nothing
}

PathPlannerPlugin::~PathPlannerPlugin()
{
   // Do nothing
}

/**
 * @brief PathPlannerPlugin::initialize Initialize the plugin which includes
 * creating the new data model
 */
bool PathPlannerPlugin::initialize(const QStringList& args, QString *errMsg)
{
    Q_UNUSED(args);
    Q_UNUSED(errMsg);

    // Create a factory for making gadgets
    mf = new PathPlannerGadgetFactory(this);
    addAutoReleasedObject(mf);

    // Create the waypoint data model for the trajectory
    waypointDataModel = new WaypointDataModel(this);
    addAutoReleasedObject(waypointDataModel);

    // Create the path segment data model for the trajectory
    pathSegmentDataModel = new PathSegmentDataModel(this);
    addAutoReleasedObject(pathSegmentDataModel);

    // Create a selector and add it to the plugin
    selection = new QItemSelectionModel(waypointDataModel);
    addAutoReleasedObject(selection);

    // Create a waypoint common dialog to be used by the map and path planner
    waypointDialog = new WaypointDialog(NULL, waypointDataModel, selection);
    addAutoReleasedObject(waypointDialog);

    // Create a path segment common dialog to be used by the map and path planner
    pathSegmentDialog = new PathSegmentDialog(NULL, pathSegmentDataModel, selection);
    addAutoReleasedObject(pathSegmentDialog);


    return true;
}

void PathPlannerPlugin::extensionsInitialized()
{
   // Do nothing
}

void PathPlannerPlugin::shutdown()
{
   // Do nothing
}
Q_EXPORT_PLUGIN(PathPlannerPlugin)

/**
  * @}
  * @}
  */
