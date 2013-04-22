/**
 ******************************************************************************
 * @file       pathplanner.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2012.
 * @author     Tau Labs, http://www.taulabs.org Copyright (C) 2013.
 * @brief      Executes a series of waypoints
 * @addtogroup OpenPilotModules OpenPilot Modules
 * @{
 * @addtogroup PathPlanner Path Planner Module
 * @brief Executes a series of waypoints
 * @{
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

#include "openpilot.h"
#include "physical_constants.h"
#include "paths.h"

#include "flightstatus.h"
#include "pathdesired.h"
#include "pathmanagersettings.h"
#include "pathmanagerstatus.h"
#include "pathplannersettings.h"
#include "pathsegmentdescriptor.h"
#include "pathplannerstatus.h"
#include "positionactual.h"
#include "waypoint.h"
#include "waypointactive.h"
#include "modulesettings.h"

// Private constants
#define STACK_SIZE_BYTES 1024
#define TASK_PRIORITY (tskIDLE_PRIORITY)
#define MAX_QUEUE_SIZE 2
// It is difficult to cleanly define how often the path planner should run. Generally, it is an
// iterative process, and so we would like to continue to refine the solution as long as there
// are spare processor cycles available. The upshot of this is that the best strategy is to run
// the process and add 1ms delays in the structure of the algorithms. This provides a break for
// other processes of the same priority, so that they can have a chance to run.
#define UPDATE_RATE_MS 100 // Cannot be greater than 200
#define IDLE_UPDATE_RATE_MS (200-UPDATE_RATE_MS)

// Private types
typedef enum {PATH_PLANNER_SUCCESS, PATH_PLANNER_PROCESSING, PATH_PLANNER_STUCK, PATH_PLANNER_INSUFFICIENT_MEMORY} PathPlannerStates;
enum guidanceTypes{NOMANAGER, RETURNHOME, HOLDPOSITION, PATHPLANNER};

// Private variables
static xTaskHandle taskHandle;
static xQueueHandle queue;
static PathPlannerSettingsData pathPlannerSettings;
static PathPlannerStatusData pathPlannerStatus;
//static WaypointActiveData waypointActive;
//static WaypointData waypoint;
static bool process_waypoints_flag;
static bool module_enabled;
static bool path_manager_status_updated;
static PathPlannerSettingsPlannerAlgorithmOptions plannerAlgorithm;
static uint8_t guidanceType = NOMANAGER;

// Private functions

static void pathPlannerTask(void *parameters);
static void settingsUpdated(UAVObjEvent * ev);
static void waypointsUpdated(UAVObjEvent * ev);
static void pathManagerStatusUpdated(UAVObjEvent * ev);
static void createPathBox();
static void createPathLogo();
static void createPathHoldPosition();
static void createPathReturnToHome();
static int8_t processWaypoints(PathPlannerSettingsPlannerAlgorithmOptions plannerAlgorithm);

PathPlannerStates direct_path_planner();

////! Store which waypoint has actually been pushed into PathDesired
//static int32_t active_waypoint = -1;
////! Store the previous waypoint which is used to determine the path trajectory
//static int32_t previous_waypoint = -1;
/**
 * Module initialization
 */
int32_t PathPlannerStart()
{
	if(module_enabled) {
		taskHandle = NULL;

		// Start VM thread
		xTaskCreate(pathPlannerTask, (signed char *)"PathPlanner", STACK_SIZE_BYTES/4, NULL, TASK_PRIORITY, &taskHandle);
		TaskMonitorAdd(TASKINFO_RUNNING_PATHPLANNER, taskHandle);
		return 0;
	}

	return -1;
}

/**
 * Module initialization
 */
int32_t PathPlannerInitialize()
{
	taskHandle = NULL;

#ifdef MODULE_PathPlanner_BUILTIN
	module_enabled = true;
#else
	uint8_t module_state[MODULESETTINGS_ADMINSTATE_NUMELEM];
	ModuleSettingsAdminStateGet(module_state);
	if (module_state[MODULESETTINGS_ADMINSTATE_PATHPLANNER] == MODULESETTINGS_ADMINSTATE_ENABLED) {
		module_enabled = true;
	} else {
		module_enabled = false;
	}
#endif

	if(module_enabled) {
		PathManagerSettingsInitialize();
		PathManagerStatusInitialize();
		PathPlannerSettingsInitialize();
		PathPlannerStatusInitialize();
		PathSegmentDescriptorInitialize();
		WaypointInitialize();
		WaypointActiveInitialize();

		// Create object queue
		queue = xQueueCreate(MAX_QUEUE_SIZE, sizeof(UAVObjEvent));

		// This variable must only be set during the initialization process. This
		// is due to the vast differences in RAM requirements between path planners
		plannerAlgorithm = pathPlannerSettings.PlannerAlgorithm;

		return 0;
	}

	return -1;
}

MODULE_INITCALL(PathPlannerInitialize, PathPlannerStart);

/**
 * Module task
 */
static void pathPlannerTask(void *parameters)
{
	// If the PathManagerStatus isn't available no manager is running and we should abort
	while (PathManagerStatusHandle() == NULL || !TaskMonitorQueryRunning(TASKINFO_RUNNING_PATHMANAGER)) {
		AlarmsSet(SYSTEMALARMS_ALARM_PATHPLANNER, SYSTEMALARMS_ALARM_CRITICAL);
		vTaskDelay(1000);
	}
	AlarmsClear(SYSTEMALARMS_ALARM_PATHPLANNER);

	PathPlannerSettingsConnectCallback(settingsUpdated);
	settingsUpdated(PathPlannerSettingsHandle());

	WaypointConnectCallback(waypointsUpdated);
	WaypointActiveConnectCallback(waypointsUpdated);

	PathManagerStatusConnectCallback(pathManagerStatusUpdated);

	// Main thread loop
	path_manager_status_updated = false;

	while (1)
	{
		FlightStatusData flightStatus;
		FlightStatusGet(&flightStatus);

		switch (flightStatus.FlightMode) {
			case FLIGHTSTATUS_FLIGHTMODE_RETURNTOHOME:
				if (guidanceType != RETURNHOME) {
					createPathReturnToHome();

					pathPlannerStatus.PathAvailability = PATHPLANNERSTATUS_PATHAVAILABILITY_NONE;
					PathPlannerStatusSet(&pathPlannerStatus);
					guidanceType = RETURNHOME;
				}
				break;
			case FLIGHTSTATUS_FLIGHTMODE_POSITIONHOLD:
				if (guidanceType != HOLDPOSITION) {
					createPathHoldPosition();

					pathPlannerStatus.PathAvailability = PATHPLANNERSTATUS_PATHAVAILABILITY_NONE;
					PathPlannerStatusSet(&pathPlannerStatus);
					guidanceType = HOLDPOSITION;
				}
				break;
			case FLIGHTSTATUS_FLIGHTMODE_PATHPLANNER:
				if (guidanceType != PATHPLANNER) {
					PathPlannerSettingsGet(&pathPlannerSettings);

					switch(pathPlannerSettings.PreprogrammedPath) {
						case PATHPLANNERSETTINGS_PREPROGRAMMEDPATH_NONE:
							// No path? In that case, burn some time and loop back to beginning. This is something that should be fixed as this takes the final form.
							process_waypoints_flag = true;
							guidanceType = NOMANAGER;
							vTaskDelay(IDLE_UPDATE_RATE_MS * portTICK_RATE_MS);

							break;
						case PATHPLANNERSETTINGS_PREPROGRAMMEDPATH_10M_BOX:
							createPathBox();

							pathPlannerStatus.PathAvailability = PATHPLANNERSTATUS_PATHAVAILABILITY_NONE;
							PathPlannerStatusSet(&pathPlannerStatus);
							break;
						case PATHPLANNERSETTINGS_PREPROGRAMMEDPATH_LOGO:
							createPathLogo();

							pathPlannerStatus.PathAvailability = PATHPLANNERSTATUS_PATHAVAILABILITY_NONE;
							PathPlannerStatusSet(&pathPlannerStatus);
							break;
					}

					guidanceType = PATHPLANNER;
				}
				break;
			default:
				// When not running the path manager, short circuit and wait
				process_waypoints_flag = true;
				guidanceType = NOMANAGER;

				pathPlannerStatus.PathAvailability = PATHPLANNERSTATUS_PATHAVAILABILITY_NONE;
				PathPlannerStatusSet(&pathPlannerStatus);

				vTaskDelay(IDLE_UPDATE_RATE_MS * portTICK_RATE_MS);

				continue;
		}

		vTaskDelay(UPDATE_RATE_MS * portTICK_RATE_MS);

		if(process_waypoints_flag)
		{
			int8_t ret;
			ret = processWaypoints(plannerAlgorithm);
			switch (ret) {
				case PATH_PLANNER_SUCCESS:
					{
						process_waypoints_flag = false;

						pathPlannerStatus.PathAvailability = PATHPLANNERSTATUS_PATHAVAILABILITY_PATHREADY;
						PathPlannerStatusSet(&pathPlannerStatus);
					}
					break;
				case PATH_PLANNER_PROCESSING:
					break;
				case PATH_PLANNER_STUCK:
					process_waypoints_flag = false;
					// Need to inform the FlightDirector that the planner cannot find a solution to the path
					break;
				case PATH_PLANNER_INSUFFICIENT_MEMORY:
					process_waypoints_flag = false;
					// Need to inform the FlightDirector that there isn't enough memory to continue. This could be because of refinement of the path, or because of too many waypoints
					break;
			}
		}
	}
}


int8_t processWaypoints(PathPlannerSettingsPlannerAlgorithmOptions plannerAlgorithm)
{
	switch(plannerAlgorithm)
	{
		case PATHPLANNERSETTINGS_PLANNERALGORITHM_DIRECT:
		{
			PathPlannerStates ret;
			ret = direct_path_planner();
			return ret;
		}
		break;
		case PATHPLANNERSETTINGS_PLANNERALGORITHM_DIRECTWITHFILLETING:
		break;
	default:
		break;
	}

	return PATH_PLANNER_PROCESSING;
}



PathPlannerStates direct_path_planner()
{
	// Check for memory before generating new path descriptors
	if(1) //There is enough memory
	{
		// Generate the path segment descriptors
		for (int i=UAVObjGetNumInstances(PathSegmentDescriptorHandle()); i<UAVObjGetNumInstances(WaypointHandle())+1; i++) {
			//TODO: Ensure there is enough memory before generating
			PathSegmentDescriptorCreateInstance();
		}
	}
	else
		return PATH_PLANNER_INSUFFICIENT_MEMORY;

	PathSegmentDescriptorData pathSegmentDescriptor;

	PositionActualData positionActual;
	PositionActualGet(&positionActual);

	pathSegmentDescriptor.SwitchingLocus[0] = positionActual.North;
	pathSegmentDescriptor.SwitchingLocus[1] = positionActual.East;
	pathSegmentDescriptor.SwitchingLocus[2] = positionActual.Down;
	pathSegmentDescriptor.FinalVelocity = 0;
	pathSegmentDescriptor.DesiredAcceleration = 0;
	pathSegmentDescriptor.Timeout = 0;
	pathSegmentDescriptor.NumberOfOrbits = 0;
	pathSegmentDescriptor.PathCurvature = 0;
	pathSegmentDescriptor.ArcRank = PATHSEGMENTDESCRIPTOR_ARCRANK_MINOR;
	PathSegmentDescriptorInstSet(0, &pathSegmentDescriptor);

	uint16_t offset = 1;

	for(int i=0; i<UAVObjGetNumInstances(WaypointHandle()); i++) {
		WaypointData waypoint;
		WaypointInstGet(i, &waypoint);

		bool path_is_circle = false;

		// Velocity is independent of path
		pathSegmentDescriptor.FinalVelocity = waypoint.Velocity;

		// Determine if the path is a straight line or if it arcs
		if (waypoint.Mode == WAYPOINT_MODE_CIRCLEPOSITIONRIGHT)
		{
			path_is_circle = true;
			pathSegmentDescriptor.NumberOfOrbits = 1e8;
			pathSegmentDescriptor.PathCurvature = 1.0f/waypoint.ModeParameters;
		}
		else if (waypoint.Mode == WAYPOINT_MODE_FLYCIRCLERIGHT)
		{
			pathSegmentDescriptor.NumberOfOrbits = 0;
			pathSegmentDescriptor.PathCurvature = 1.0f/waypoint.ModeParameters;
		}
		else if (waypoint.Mode == WAYPOINT_MODE_DRIVECIRCLERIGHT)
		{
			pathSegmentDescriptor.NumberOfOrbits = 0;
			pathSegmentDescriptor.PathCurvature = 1.0f/waypoint.ModeParameters;
		}
		else if (waypoint.Mode == WAYPOINT_MODE_CIRCLEPOSITIONLEFT)
		{
			path_is_circle = true;
			pathSegmentDescriptor.NumberOfOrbits = 1e8;
			pathSegmentDescriptor.PathCurvature = -1.0f/waypoint.ModeParameters;
		}
		else if (waypoint.Mode == WAYPOINT_MODE_FLYCIRCLELEFT)
		{
			pathSegmentDescriptor.NumberOfOrbits = 0;
			pathSegmentDescriptor.PathCurvature = -1.0f/waypoint.ModeParameters;
		}
		else if (waypoint.Mode == WAYPOINT_MODE_DRIVECIRCLELEFT)
		{
			pathSegmentDescriptor.NumberOfOrbits = 0;
			pathSegmentDescriptor.PathCurvature = -1.0f/waypoint.ModeParameters;
		}

		// In the case of pure circles, the given waypoint is for a circle center
		// so we have to convert it into a pair of switching loci.
		if ( !path_is_circle ) {
			pathSegmentDescriptor.SwitchingLocus[0] = waypoint.Position[0];
			pathSegmentDescriptor.SwitchingLocus[1] = waypoint.Position[1];
			pathSegmentDescriptor.SwitchingLocus[2] = waypoint.Position[2];
			pathSegmentDescriptor.Timeout = 60; // TODO: Calculate time

			PathSegmentDescriptorInstSet(i+offset, &pathSegmentDescriptor);
		}
		else{
			//
			PathSegmentDescriptorData pathSegmentDescriptor_old;
			PathSegmentDescriptorInstGet(i-1, &pathSegmentDescriptor_old);

			PathManagerSettingsData pathManagerSettings;
			PathManagerSettingsGet(&pathManagerSettings);

			float radius = 1.0f/pathSegmentDescriptor.PathCurvature;

			// Calculate the approach angle from the previous switching locus to the waypoint
			float approachTheta_rad = atan2f(waypoint.Position[1] - pathSegmentDescriptor_old.SwitchingLocus[1], waypoint.Position[0] - pathSegmentDescriptor_old.SwitchingLocus[0]);

			// Calculate distance from previous waypoint to circle perimeter. (Distance to perimeter is distance to circle center minus radius)
			float d = sqrt(powf(pathSegmentDescriptor.SwitchingLocus[0] - pathSegmentDescriptor_old.SwitchingLocus[0], 2) + powf(pathSegmentDescriptor.SwitchingLocus[1] - pathSegmentDescriptor_old.SwitchingLocus[1], 2)) - radius;

			if (d > pathManagerSettings.HalfPlaneAdvanceTiming * pathSegmentDescriptor.FinalVelocity) {
				// Go straight to position
				pathSegmentDescriptor.SwitchingLocus[0] = waypoint.Position[0] + cos(approachTheta_rad)*radius;
				pathSegmentDescriptor.SwitchingLocus[1] = waypoint.Position[1] + sin(approachTheta_rad)*radius;
				pathSegmentDescriptor.SwitchingLocus[2] = waypoint.Position[2];
				pathSegmentDescriptor.PathCurvature = 0;
				pathSegmentDescriptor.NumberOfOrbits = 0;
				pathSegmentDescriptor.Timeout = 60; // TODO: Calculate timeout
				PathSegmentDescriptorInstSet(i+offset, &pathSegmentDescriptor);

				// Increment offest counter
				offset++;

				// Orbit position
				PathSegmentDescriptorCreateInstance();
				pathSegmentDescriptor.SwitchingLocus[0] = waypoint.Position[0] -cos(approachTheta_rad) * radius;
				pathSegmentDescriptor.SwitchingLocus[1] = waypoint.Position[1] -sin(approachTheta_rad) * radius;
				pathSegmentDescriptor.SwitchingLocus[2] = waypoint.Position[2];
				pathSegmentDescriptor.PathCurvature = -1.0f/waypoint.ModeParameters;
				pathSegmentDescriptor.NumberOfOrbits = 1e8;
				pathSegmentDescriptor.Timeout = 60; // TODO: Calculate timeout
				PathSegmentDescriptorInstSet(i+offset, &pathSegmentDescriptor);
			}
			else {
				// Enter directly into circle
				pathSegmentDescriptor.SwitchingLocus[0] = waypoint.Position[0] + cos(approachTheta_rad)*radius;
				pathSegmentDescriptor.SwitchingLocus[1] = waypoint.Position[1] + sin(approachTheta_rad)*radius;
				pathSegmentDescriptor.SwitchingLocus[2] = waypoint.Position[2];
				pathSegmentDescriptor.PathCurvature = 0;
				pathSegmentDescriptor.NumberOfOrbits = 0;
				pathSegmentDescriptor.Timeout = d; // TODO: Calculate timeout
				PathSegmentDescriptorInstSet(i+offset, &pathSegmentDescriptor);

				// Increment offest counter
				offset++;

				// Orbit position
				PathSegmentDescriptorCreateInstance();
				pathSegmentDescriptor.SwitchingLocus[0] = waypoint.Position[0] -cos(approachTheta_rad) * radius;
				pathSegmentDescriptor.SwitchingLocus[1] = waypoint.Position[1] -sin(approachTheta_rad) * radius;
				pathSegmentDescriptor.SwitchingLocus[2] = waypoint.Position[2];
				pathSegmentDescriptor.PathCurvature = -1.0f/waypoint.ModeParameters;
				pathSegmentDescriptor.NumberOfOrbits = 1e8;
				pathSegmentDescriptor.Timeout = 60; // TODO: Calculate timeout
				PathSegmentDescriptorInstSet(i+offset, &pathSegmentDescriptor);
			}
		}
	}

	return PATH_PLANNER_SUCCESS;
}


/**
 * On changed waypoints, replan the path
 */
static void waypointsUpdated(UAVObjEvent * ev)
{
	process_waypoints_flag = true;
}

/**
 * When the PathStatus is updated indicate a new one is available to consume
 */
static void pathManagerStatusUpdated(UAVObjEvent * ev)
{
	path_manager_status_updated = true;
}

void settingsUpdated(UAVObjEvent * ev) {
	uint8_t preprogrammedPath = pathPlannerSettings.PreprogrammedPath;
	PathPlannerSettingsGet(&pathPlannerSettings);

	if (pathPlannerSettings.PreprogrammedPath != preprogrammedPath) {
		switch(pathPlannerSettings.PreprogrammedPath) {
			case PATHPLANNERSETTINGS_PREPROGRAMMEDPATH_NONE:
				break;
			case PATHPLANNERSETTINGS_PREPROGRAMMEDPATH_10M_BOX:
				createPathBox();
				break;
			case PATHPLANNERSETTINGS_PREPROGRAMMEDPATH_LOGO:
				createPathLogo();
				break;
			case PATHPLANNERSETTINGS_PREPROGRAMMEDPATH_RETURNTOHOME:
				createPathReturnToHome();
				break;
			case PATHPLANNERSETTINGS_PREPROGRAMMEDPATH_HOLDPOSITION:
				createPathHoldPosition();
				break;

		}
	}
}

static void createPathReturnToHome()
{
	WaypointData waypoint;

	float radius = 70;

	PositionActualData positionActual;
	PositionActualGet(&positionActual);

	waypoint.Position[0] = 0;
	waypoint.Position[1] = 0;
	waypoint.Position[2] = positionActual.Down - 10;
	waypoint.Mode = WAYPOINT_MODE_CIRCLEPOSITIONLEFT;
	waypoint.ModeParameters = radius;
	WaypointInstSet(0, &waypoint);

	pathPlannerStatus.NumberOfWaypoints = 1;
	PathPlannerStatusSet(&pathPlannerStatus);
}

static void createPathHoldPosition()
{
	WaypointData waypoint;

	PositionActualData positionActual;
	PositionActualGet(&positionActual);

	float radius = 70;

	waypoint.Position[0] = positionActual.North;
	waypoint.Position[1] = positionActual.East;
	waypoint.Position[2] = positionActual.Down - 10;
	waypoint.Mode = WAYPOINT_MODE_CIRCLEPOSITIONRIGHT;
	waypoint.ModeParameters = radius;
	WaypointInstSet(0, &waypoint);

	pathPlannerStatus.NumberOfWaypoints = 1;
	PathPlannerStatusSet(&pathPlannerStatus);
}

static void createPathBox()
{
	float scale = 8;

	pathPlannerStatus.NumberOfWaypoints = 7;
	PathPlannerStatusSet(&pathPlannerStatus);

	for (int i=UAVObjGetNumInstances(WaypointHandle()); i<pathPlannerStatus.NumberOfWaypoints; i++) {
		WaypointCreateInstance();
	}

	// Draw O
	WaypointData waypoint;
	waypoint.Velocity = 12; // Since for now this isn't directional just set a mag
	waypoint.Mode = WAYPOINT_MODE_FLYVECTOR;

	waypoint.Position[0] = 0;
	waypoint.Position[1] = 0;
	waypoint.Position[2] = -2500;
	WaypointInstSet(0, &waypoint);

	waypoint.Position[0] = 25*scale;
	waypoint.Position[1] = 25*scale;
	waypoint.Position[2] = -2500;
	WaypointInstSet(1, &waypoint);

	waypoint.Position[0] = -25*scale;
	waypoint.Position[1] = 25*scale;
//	waypoint.Mode = WAYPOINT_MODE_FLYCIRCLELEFT;
	//waypoint.Mode = WAYPOINT_MODE_FLYCIRCLERIGHT;
	waypoint.ModeParameters = 35;
	WaypointInstSet(2, &waypoint);

	waypoint.Position[0] = -25*scale;
	waypoint.Position[1] = -25*scale;
	WaypointInstSet(3, &waypoint);

	waypoint.Position[0] = 25*scale;
	waypoint.Position[1] = -25*scale;
	WaypointInstSet(4, &waypoint);

	waypoint.Position[0] = 25*scale;
	waypoint.Position[1] = 25*scale;
	WaypointInstSet(5, &waypoint);

	waypoint.Position[0] = 0;
	waypoint.Position[1] = 0;
	waypoint.Mode = WAYPOINT_MODE_CIRCLEPOSITIONLEFT;
	waypoint.ModeParameters = 35;
	WaypointInstSet(6, &waypoint);
}

static void createPathLogo()
{
	float scale = 2;

	pathPlannerStatus.NumberOfWaypoints = 43;
	PathPlannerStatusSet(&pathPlannerStatus);

	for (int i=UAVObjGetNumInstances(WaypointHandle()); i<pathPlannerStatus.NumberOfWaypoints; i++) {
		WaypointCreateInstance();
	}


	// Draw O
	WaypointData waypoint;
	waypoint.Velocity = 12; // Since for now this isn't directional just set a mag
	for(uint32_t i = 0; i < 20; i++) {
		waypoint.Position[1] = scale * 30 * cosf(i / 19.0 * 2 * PI);
		waypoint.Position[0] = scale * 50 * sinf(i / 19.0 * 2 * PI);
		waypoint.Position[2] = -2500;
		waypoint.Mode = WAYPOINT_MODE_FLYVECTOR;
	}

	// Draw P
	for(uint32_t i = 20; i < 35; i++) {
		waypoint.Position[1] = scale * (55 + 20 * cosf(i / 10.0 * PI - PI / 2));
		waypoint.Position[0] = scale * (25 + 25 * sinf(i / 10.0 * PI - PI / 2));
		waypoint.Position[2] = -2500;
		waypoint.Mode = WAYPOINT_MODE_FLYVECTOR;
	}

	waypoint.Position[1] = scale * 35;
	waypoint.Position[0] = scale * -50;
	waypoint.Position[2] = -2500;
	waypoint.Mode = WAYPOINT_MODE_FLYVECTOR;
	WaypointInstSet(36, &waypoint);

	// Draw Box
	waypoint.Position[1] = scale * 35;
	waypoint.Position[0] = scale * -60;
	waypoint.Position[2] = -2500;
	waypoint.Mode = WAYPOINT_MODE_FLYVECTOR;
	WaypointInstSet(37, &waypoint);

	waypoint.Position[1] = scale * 85;
	waypoint.Position[0] = scale * -60;
	waypoint.Position[2] = -2500;
	waypoint.Mode = WAYPOINT_MODE_FLYVECTOR;
	WaypointInstSet(38, &waypoint);

	waypoint.Position[1] = scale * 85;
	waypoint.Position[0] = scale * 60;
	waypoint.Position[2] = -2500;
	waypoint.Mode = WAYPOINT_MODE_FLYVECTOR;
	WaypointInstSet(39, &waypoint);

	waypoint.Position[1] = scale * -40;
	waypoint.Position[0] = scale * 60;
	waypoint.Position[2] = -2500;
	waypoint.Mode = WAYPOINT_MODE_FLYVECTOR;
	WaypointInstSet(40, &waypoint);

	waypoint.Position[1] = scale * -40;
	waypoint.Position[0] = scale * -60;
	waypoint.Position[2] = -2500;
	waypoint.Mode = WAYPOINT_MODE_FLYVECTOR;
	WaypointInstSet(41, &waypoint);

	waypoint.Position[1] = scale * 35;
	waypoint.Position[0] = scale * -60;
	waypoint.Position[2] = -2500;
	waypoint.Mode = WAYPOINT_MODE_FLYVECTOR;
	WaypointInstSet(42, &waypoint);
}

/**
 * @}
 * @}
 */
