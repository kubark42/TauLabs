/**
 ******************************************************************************
 * @file       pathmanager.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2012.
 * @author     Tau Labs, http://www.taulabs.org Copyright (C) 2013.
 * @brief      Executes a series of waypoints
 * @addtogroup OpenPilotModules OpenPilot Modules
 * @{
 * @addtogroup PathManager Path Manager Module
 * @brief The path manager switches between motion descriptors in order to maneuver
 * along the path
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
#include "misc_math.h"

#include "flightstatus.h"
#include "positionactual.h"
#include "waypoint.h"
#include "waypointactive.h"
#include "modulesettings.h"

#include "pathfollowerstatus.h"
#include "pathmanagerstatus.h"
#include "pathplannersettings.h"
#include "pathsegmentdescriptor.h"

#include "CoordinateConversions.h"

// Private constants
#define STACK_SIZE_BYTES 700
#define TASK_PRIORITY (tskIDLE_PRIORITY+1)
#define MAX_QUEUE_SIZE 2
#define UPDATE_RATE_MS 100
#define IDLE_UPDATE_RATE_MS (200-UPDATE_RATE_MS)
#define OVERSHOOT_TIMER_MS 1000

// Private types
enum guidanceTypes{NOMANAGER, RETURNHOME, HOLDPOSITION, PATHPLANNER};
static struct PreviousLocus {
	float Position[3];

	float Velocity;
} *previousLocus;

// Private variables
static bool module_enabled;
static xTaskHandle taskHandle;
static xQueueHandle queue;
static PathPlannerSettingsData pathPlannerSettings;
static PathManagerStatusData pathManagerStatus;
static PathSegmentDescriptorData pathSegmentDescriptor_current;
static portTickType segmentTimer;
static float angularDistanceCompleted;
static float oldPosition_NE[2];
float arcCenter_NE[2];

// Private functions
static bool checkGoalCondition();
static void checkOvershoot();
static void pathManagerTask(void *parameters);
static void settingsUpdated(UAVObjEvent * ev);
static void pathSegmentDescriptorsUpdated(UAVObjEvent * ev);
static float updateArcMeasure(float oldPosition_NE[2], float newPosition_NE[2], float arcCenter_NE[2]);

/**
 * Module initialization
 */
int32_t PathManagerStart()
{
	if(module_enabled) {
		taskHandle = NULL;

		// Start VM thread
		xTaskCreate(pathManagerTask, (signed char *)"PathManager", STACK_SIZE_BYTES/4, NULL, TASK_PRIORITY, &taskHandle);
		TaskMonitorAdd(TASKINFO_RUNNING_PATHMANAGER, taskHandle);
		return 0;
	}

	return -1;
}

/**
 * Module initialization
 */
int32_t PathManagerInitialize()
{
	taskHandle = NULL;

#ifdef MODULE_PathManager_BUILTIN
	module_enabled = true;
#else
	uint8_t module_state[MODULESETTINGS_ADMINSTATE_NUMELEM];
	ModuleSettingsAdminStateGet(module_state);
	if (module_state[MODULESETTINGS_ADMINSTATE_PATHMANAGER] == MODULESETTINGS_ADMINSTATE_ENABLED) {
		module_enabled = true;
	} else {
		module_enabled = false;
	}
#endif

	if(module_enabled) {
		PathFollowerStatusInitialize();
		PathManagerStatusInitialize();
		PathPlannerSettingsInitialize();
		PathSegmentDescriptorInitialize();

		// Create object queue
		queue = xQueueCreate(MAX_QUEUE_SIZE, sizeof(UAVObjEvent));

		// Allocate memory
		previousLocus = (struct PreviousLocus *) pvPortMalloc(sizeof(struct PreviousLocus));
		memset(previousLocus, 0, sizeof(struct PreviousLocus));
		return 0;
	}

	return -1;
}

MODULE_INITCALL(PathManagerInitialize, PathManagerStart);

/**
 * Module task
 */
static void pathManagerTask(void *parameters)
{
	// If PathFollowerStatus is not available then no follower is running and we cannot continue
	while (!TaskMonitorQueryRunning(TASKINFO_RUNNING_PATHFOLLOWER)) {
		AlarmsSet(SYSTEMALARMS_ALARM_PATHMANAGER, SYSTEMALARMS_ALARM_CRITICAL);
		vTaskDelay(1000);
	}
	AlarmsClear(SYSTEMALARMS_ALARM_PATHMANAGER);

	// Connect callbacks
//	PathPlannerSettingsConnectCallback(settingsUpdated);
	PathSegmentDescriptorConnectCallback(pathSegmentDescriptorsUpdated);

	// Force reload all settings
	settingsUpdated(NULL);

	// Initialize all main loop variables
	bool pathplanner_active = false;
	static portTickType lastSysTime;
	static portTickType overshootTimer;
	lastSysTime = xTaskGetTickCount();
	overshootTimer = xTaskGetTickCount();

	// REMOVE THESE TWO LINES
	// VVVVVVVVVVVVVVVVVVVV
	int bob  =0;
	PathSegmentDescriptorCreateInstance();
	// ^^^^^^^^^^^^^^^^^^^^

	// Main thread loop
	while (1)
	{

		// Wait
		vTaskDelayUntil(&lastSysTime, UPDATE_RATE_MS * portTICK_RATE_MS);

		// Check flight mode
		FlightStatusData flightStatus;
		FlightStatusGet(&flightStatus);
		switch (flightStatus.FlightMode) {
			case FLIGHTSTATUS_FLIGHTMODE_RETURNTOHOME:
				if(bob++ ==0){
					PathSegmentDescriptorCreateInstance();
					PathSegmentDescriptorCreateInstance();
					PathSegmentDescriptorCreateInstance();
					PathSegmentDescriptorCreateInstance();
					PathSegmentDescriptorCreateInstance();

					PathSegmentDescriptorData pathSegmentDescriptor;

					pathSegmentDescriptor.SwitchingLocus[0] = 100;
					pathSegmentDescriptor.SwitchingLocus[1] = 100;
					pathSegmentDescriptor.SwitchingLocus[2] = -350;
					pathSegmentDescriptor.FinalVelocity = 11.4f;
					pathSegmentDescriptor.Timeout = 60000;
					pathSegmentDescriptor.PathCurvature = 1/40.0f;
					pathSegmentDescriptor.ArcAngle = 1e11; //TODO: Define this really large floating-point value as a magic number
					pathSegmentDescriptor.ArcRank = PATHSEGMENTDESCRIPTOR_ARCRANK_MAJOR;
					PathSegmentDescriptorInstSet(6, &pathSegmentDescriptor);

					pathSegmentDescriptor.SwitchingLocus[0] = 100;
					pathSegmentDescriptor.SwitchingLocus[1] = 100;
					pathSegmentDescriptor.SwitchingLocus[2] = -350;
					pathSegmentDescriptor.FinalVelocity = 11.3f;
					pathSegmentDescriptor.Timeout = 60;
					pathSegmentDescriptor.SegmentType = PATHSEGMENTDESCRIPTOR_SEGMENTTYPE_LINE;
					pathSegmentDescriptor.PathCurvature = 0;
					PathSegmentDescriptorInstSet(5, &pathSegmentDescriptor);

					pathSegmentDescriptor.SwitchingLocus[0] = 100;
					pathSegmentDescriptor.SwitchingLocus[1] = 500;
					pathSegmentDescriptor.SwitchingLocus[2] = -350;
					pathSegmentDescriptor.FinalVelocity = 11.2f;
					pathSegmentDescriptor.Timeout = 60;
					pathSegmentDescriptor.SegmentType = PATHSEGMENTDESCRIPTOR_SEGMENTTYPE_LINE;
					pathSegmentDescriptor.PathCurvature = 0;
					PathSegmentDescriptorInstSet(4, &pathSegmentDescriptor);

					pathSegmentDescriptor.SwitchingLocus[0] = 500;
					pathSegmentDescriptor.SwitchingLocus[1] = 500;
					pathSegmentDescriptor.SwitchingLocus[2] = -350;
					pathSegmentDescriptor.FinalVelocity = 11.1f;
					pathSegmentDescriptor.Timeout = 60;
					pathSegmentDescriptor.SegmentType = PATHSEGMENTDESCRIPTOR_SEGMENTTYPE_LINE;
					pathSegmentDescriptor.PathCurvature = 0;
					PathSegmentDescriptorInstSet(3, &pathSegmentDescriptor);

					pathSegmentDescriptor.SwitchingLocus[0] = 500;
					pathSegmentDescriptor.SwitchingLocus[1] = 100;
					pathSegmentDescriptor.SwitchingLocus[2] = -350;
					pathSegmentDescriptor.FinalVelocity = 11.0f;
					pathSegmentDescriptor.Timeout = 60;
					pathSegmentDescriptor.SegmentType = PATHSEGMENTDESCRIPTOR_SEGMENTTYPE_LINE;
					pathSegmentDescriptor.PathCurvature = 0;
					PathSegmentDescriptorInstSet(2, &pathSegmentDescriptor);

					pathSegmentDescriptor.SwitchingLocus[0] = 100;
					pathSegmentDescriptor.SwitchingLocus[1] = 100;
					pathSegmentDescriptor.SwitchingLocus[2] = -300;
					pathSegmentDescriptor.FinalVelocity = 10.1f;
					pathSegmentDescriptor.Timeout = 60;
					pathSegmentDescriptor.SegmentType = PATHSEGMENTDESCRIPTOR_SEGMENTTYPE_LINE;
					pathSegmentDescriptor.PathCurvature = 0;
					PathSegmentDescriptorInstSet(1, &pathSegmentDescriptor);

					pathSegmentDescriptor.SwitchingLocus[0] = 0;
					pathSegmentDescriptor.SwitchingLocus[1] = 0;
					pathSegmentDescriptor.SwitchingLocus[2] = 0;
					pathSegmentDescriptor.FinalVelocity = 10.1f;
					pathSegmentDescriptor.Timeout = 60;
					pathSegmentDescriptor.SegmentType = PATHSEGMENTDESCRIPTOR_SEGMENTTYPE_LINE;
					pathSegmentDescriptor.PathCurvature = 0;
					PathSegmentDescriptorInstSet(0, &pathSegmentDescriptor);
				}
			case FLIGHTSTATUS_FLIGHTMODE_POSITIONHOLD:
			case FLIGHTSTATUS_FLIGHTMODE_PATHPLANNER:
				break;
			default:
				// When not running the path manager, short circuit and wait
				pathplanner_active = false;
				vTaskDelay(IDLE_UPDATE_RATE_MS * portTICK_RATE_MS);
				continue;
		}

		// Check if the path_manager was just activated
		if(pathplanner_active == false) {
			// Set previousLocus as initial waypoint
			PathSegmentDescriptorData pathSegmentDescriptor;
			PathSegmentDescriptorInstGet(0, &pathSegmentDescriptor);
			previousLocus->Position[0] = pathSegmentDescriptor.SwitchingLocus[0];
			previousLocus->Position[1] = pathSegmentDescriptor.SwitchingLocus[1];
			previousLocus->Position[2] = pathSegmentDescriptor.SwitchingLocus[2];
			previousLocus->Velocity = pathSegmentDescriptor.FinalVelocity;

			// This triggers the path follower
			pathManagerStatus.Status = PATHMANAGERSTATUS_STATUS_INPROGRESS;
			pathManagerStatus.ActiveSegment = 1;
			PathManagerStatusSet(&pathManagerStatus);

			PositionActualData positionActual;
			PositionActualGet(&positionActual);
			oldPosition_NE[0] = positionActual.North;
			oldPosition_NE[1] = positionActual.East;

			pathplanner_active = true;

			// Reset timer
			segmentTimer = xTaskGetTickCount();

			continue;
		}

		// Update arc measure traveled
		{
			PositionActualData positionActual;
			PositionActualGet(&positionActual);
			float newPosition_NE[2] = {positionActual.North, positionActual.East};
			arcCenterFromTwoPointsAndRadiusAndArcRank(oldPosition_NE, newPosition_NE, 1.0f/pathSegmentDescriptor_current.PathCurvature, arcCenter_NE, pathSegmentDescriptor_current.ArcAngle > 0, pathSegmentDescriptor_current.ArcRank == PATHSEGMENTDESCRIPTOR_ARCRANK_MINOR);
			angularDistanceCompleted  += updateArcMeasure(oldPosition_NE, newPosition_NE, arcCenter_NE);

			oldPosition_NE[0] = newPosition_NE[0];
			oldPosition_NE[1] = newPosition_NE[1];
		}


		// If the vehicle is sufficiently close to the goal, check if it has achieved the goal
		// of the active path segment. Sufficiently close is chosen to be an arbitrary angular
		// distance, as this is robust and sufficient to describe all paths, including infinite
		// straight lines and infinite number of orbits about a point.
		bool advanceSegment_flag = false;
		if(abs(angularDistanceCompleted - pathSegmentDescriptor_current.ArcAngle) > 30) //Replace magic number 30 by a #define value
			advanceSegment_flag = checkGoalCondition();

		// Check if we should advance the segment.
		if(advanceSegment_flag){
			PathSegmentDescriptorData pathSegmentDescriptor;
			PathSegmentDescriptorInstGet(pathManagerStatus.ActiveSegment, &pathSegmentDescriptor);

			previousLocus->Position[0] = pathSegmentDescriptor.SwitchingLocus[0];
			previousLocus->Position[1] = pathSegmentDescriptor.SwitchingLocus[1];
			previousLocus->Position[2] = pathSegmentDescriptor.SwitchingLocus[2];
			previousLocus->Velocity = pathSegmentDescriptor.FinalVelocity;

			// Advance segment
			pathManagerStatus.ActiveSegment++;
			pathManagerStatus.Status = PATHMANAGERSTATUS_STATUS_INPROGRESS;
			PathManagerStatusSet(&pathManagerStatus);

			// Reset timer
			segmentTimer = xTaskGetTickCount();
		}
		// Check if we have timed out
		else if (lastSysTime-segmentTimer > pathSegmentDescriptor_current.Timeout*1000*portTICK_RATE_MS){
			// TODO: Handle the buffer overflow in xTaskGetTickCount
			pathManagerStatus.Status = PATHMANAGERSTATUS_STATUS_TIMEDOUT;
			PathManagerStatusSet(&pathManagerStatus);
		}
		// Once every second or so, check for higher-level path planner failure
		else if (lastSysTime-overshootTimer > OVERSHOOT_TIMER_MS*portTICK_RATE_MS){
			checkOvershoot();
			overshootTimer = lastSysTime;
		}
	}
}

/**
 * On changed path plan.
 */
static void pathSegmentDescriptorsUpdated(UAVObjEvent * ev)
{
	// On new path, reset to path beginning.
	// This is somewhat brittle, since it means we can't resume a path
	PathSegmentDescriptorInstGet(0, &pathSegmentDescriptor_current);
}



static void settingsUpdated(UAVObjEvent * ev) {
	uint8_t preprogrammedPath = pathPlannerSettings.PreprogrammedPath;
	PathPlannerSettingsGet(&pathPlannerSettings);
	if (pathPlannerSettings.PreprogrammedPath != preprogrammedPath) {
		switch(pathPlannerSettings.PreprogrammedPath) {
			case PATHPLANNERSETTINGS_PREPROGRAMMEDPATH_NONE:
				break;
		}
	}
}


// This is not a strict end to the segment, as some amount of error will always
// creep in. Instead, come within either a preset distance or a preset time of
// the goal condition.
static bool checkGoalCondition()
{
	bool advanceSegment_flag = false;

	// Half-plane approach
	// From R. Beard and T. McLain, "Small Unmanned Aircraft: Theory and Practice", 2011, Section 11.1.
	// Note: The half-plane approach has difficulties when the plane and the two loci are close to colinear
	// and reversing in direction. That is to say, a plane at P is supposed to go to A and then B:
	//    B----------P------A
	if(1)
	{
		// Check if there is a switching locus after the present one
		if (pathManagerStatus.ActiveSegment + 1 < UAVObjGetNumInstances(PathSegmentDescriptorHandle())){
			PathSegmentDescriptorData pathSegmentDescriptor_future;
			PathSegmentDescriptorInstGet(pathManagerStatus.ActiveSegment, &pathSegmentDescriptor_current);
			PathSegmentDescriptorInstGet(pathManagerStatus.ActiveSegment+1, &pathSegmentDescriptor_future);

			// Line meeting line
			if(pathSegmentDescriptor_current.PathCurvature == 0 && pathSegmentDescriptor_future.PathCurvature == 0){
				float *swl_past = previousLocus->Position;
				float *swl_current = pathSegmentDescriptor_current.SwitchingLocus;
				float *swl_future  = pathSegmentDescriptor_future.SwitchingLocus;

				// Calculate normal vector from past to preset switching locus
				float q_current[3] = {swl_current[0] - swl_past[0], swl_current[1] - swl_past[1], swl_current[2] - swl_past[2]};
				float q_current_mag = VectorMagnitude(q_current); //Normalize
				float q_future [3] = {swl_future[0] - swl_current[0], swl_future[1] - swl_current[1], swl_future[2] - swl_current[2]};
				float q_future_mag = VectorMagnitude(q_future); //Normalize

				// Compute the half-plane as the plane formed by the line perpendicular to the sum of the approach and
				// departure vectors. See Fig 11.1 in reference.
				//
				// We're going to take a litle mathematical shortcut, by utilizing the fact that we don't need the actual
				// normalized normal vector, any normal vector will do. If a and b are vectors, then
				// a/|a|+b/|b| = 1/(|a||b|)*(a*|b| + b*|a|), which points in the same direction as (a*|b| + b*|a|)
				float halfPlane[3] = {q_future[0]*q_current_mag + q_current[0]*q_future_mag,
									  q_future[1]*q_current_mag + q_current[1]*q_future_mag,
									  q_future[2]*q_current_mag + q_current[2]*q_future_mag};

				if(1){
					// Test if the UAV is in the half plane, H. This is easy by taking advantage of simple vector
					// calculus: a.b = |a||b|cos(theta), but since |a|,|b| >=0, then a.b > 0 if and only if
					// cos(theta) > 0, which means that the UAV is in I or IV quadrants, i.e. is in the half plane.
					PositionActualData positionActual;
					PositionActualGet(&positionActual);
					float p[3] = {positionActual.North - swl_current[0], positionActual.East - swl_current[1], positionActual.Down - swl_current[2]};

					//TODO: Decide if this should be tested in 2D or 3D. In 3D, if there is a large difference in altitude the locus could fail to switch,
					// and in the case of a line follower this would mean the airplane would keep on flying forward for ever.
					if(p[0]*halfPlane[0] + p[1]*halfPlane[1] /*+ p[2]*halfPlane[2]*/ > 0){
						advanceSegment_flag = true;
					}
				}
				else if(0){
					// Test if UAV is within X seconds of the half plane
				}
			}
			// "Small Unmanned Aircraft: Theory and Practice" provides no guidance for the perpendicular
			// intersection of a line and an arc. However, it seems reasonable to consider the halfplane
			// as occurring at the first intersection between the vector and the arc.
			//
			// The nice part about this approach is that it works equally well for a tangent curve, as the intersection
			// occurs at the right place on the circle. In the case that due to numerical error the vector and arc do
			// not intersect, we will still test for crossing into the half plane defined of a line drawn between the arc's
			// center and the closest point on the vector to the arc.
			else if(pathSegmentDescriptor_current.PathCurvature == 0 && pathSegmentDescriptor_future.PathCurvature != 0)
			{
				// Test if the infinite vector touches the circle. Code inspired by the nicely prepared
				// answer here: http://stackoverflow.com/questions/1073336/circle-line-collision-detection

				float *swl_past = previousLocus->Position;
				float *swl_current = pathSegmentDescriptor_current.SwitchingLocus;
				float *swl_future  = pathSegmentDescriptor_future.SwitchingLocus;

				// Calculate normal vector from past to preset switching locus
				float q_current[3] = {swl_current[0] - swl_past[0], swl_current[1] - swl_past[1], swl_current[2] - swl_past[2]};
				float q_current_mag = VectorMagnitude(q_current); //Normalize
				float q_future [3] = {swl_future[0] - swl_current[0], swl_future[1] - swl_current[1], swl_future[2] - swl_current[2]};
				float q_future_mag = VectorMagnitude(q_future); //Normalize

				float f[3] = {swl_current[0] - arcCenter_NE[0], swl_current[1] - arcCenter_NE[1]}; // Vector from arc center to ray start
				float a = q_current[0]*q_current[0] + q_current[1]*q_current[1];
				float b = 2.0f*f[0]*q_current[0] + f[1]*q_current[1];
				float c = f[0]*f[0] + f[1]*f[1] - 1.0f/powf(pathSegmentDescriptor_future.PathCurvature, 2);

				float discriminant = b*b-4*a*c;
				if( discriminant < 0 )
				{
				  // no intersection
				}


				// Compute the half-plane as the plane formed by the line perpendicular to the sum of the approach and
				// departure vectors. See Fig 11.1 in reference.
				//
				// We're going to take a litle mathematical shortcut, by utilizing the fact that we don't need the actual
				// normalized normal vector, any normal vector will do. If a and b are vectors, then
				// a/|a|+b/|b| = 1/(|a||b|)*(a*|b| + b*|a|), which points in the same direction as (a*|b| + b*|a|)
				float halfPlane[3] = {q_future[0]*q_current_mag + q_current[0]*q_future_mag,
									  q_future[1]*q_current_mag + q_current[1]*q_future_mag,
									  q_future[2]*q_current_mag + q_current[2]*q_future_mag};

				if(1){
					// Test if the UAV is in the half plane, H. This is easy by taking advantage of simple vector
					// calculus: a.b = |a||b|cos(theta), but since |a|,|b| >=0, then a.b > 0 if and only if
					// cos(theta) > 0, which means that the UAV is in I or IV quadrants, i.e. is in the half plane.
					PositionActualData positionActual;
					PositionActualGet(&positionActual);
					float p[3] = {positionActual.North - swl_current[0], positionActual.East - swl_current[1], positionActual.Down - swl_current[2]};

					//TODO: Decide if this should be tested in 2D or 3D. In 3D, if there is a large difference in altitude the locus could fail to switch,
					// and in the case of a line follower this would mean the airplane would keep on flying forward for ever.
					if(p[0]*halfPlane[0] + p[1]*halfPlane[1] /*+ p[2]*halfPlane[2]*/ > 0){
						advanceSegment_flag = true;
					}
				}
				else if(0){
					// Test if UAV is within X seconds of the half plane
				}
			}
		}
		else{ // Since there are no further switching loci, this must be the waypoint.
			//Do nothing.
		}
	}
	// B-ball approach
	// From R. Beard and T. McLain, "Small Unmanned Aircraft: Theory and Practice", 2011, Section 11.1.
	else if(0){
		// This method is less robust to error, and is primarily included for completeness.
	}

	return advanceSegment_flag;
}

//Check to see if we've seriously overflown our destination. Since the path follower is simply following a
//motion descriptor,it has no concept of where the path ends. It will simply keep following it to infinity
//if we don't stop it.
//So while we don't know why the navigation manager failed, we know we don't want the plane flying off.
static void checkOvershoot()
{
	PathSegmentDescriptorInstGet(pathManagerStatus.ActiveSegment, &pathSegmentDescriptor_current);

	if (pathSegmentDescriptor_current.SegmentType == PATHSEGMENTDESCRIPTOR_SEGMENTTYPE_LINE) {
		PositionActualData positionActual;
		PositionActualGet(&positionActual);

		float p[3] = {positionActual.North, positionActual.East, positionActual.Down};
		float *c = pathSegmentDescriptor_current.SwitchingLocus;
		float *r = previousLocus->Position;

		// Calculate vector from initial to final point
		float q[3] = {c[0] - r[0], c[1] - r[1], c[2] - r[2]};

		//Compute the norm squared of the horizontal path length
		float pathLength=sqrtf(q[0]*q[0]+q[1]*q[1]);

		// Perform a quick vector dot product to test if we've gone past the waypoint.
		// Add in a distance equal to 5s of flight time for good measure to make sure we don't have any jitter.
		// TODO: THE MATH HERE IS WRONG.
		if (sqrtf(powf((p[0]-r[0])*q[0],2)+powf((p[1]-r[1])*q[1],2)) > pathLength+5.0f*pathSegmentDescriptor_current.FinalVelocity){
			//Whoops, we've really overflown our destination point, and haven't received any instructions.

			//Inform the FSM
			pathManagerStatus.Status = PATHMANAGERSTATUS_STATUS_CRITICAL;
			PathManagerStatusSet(&pathManagerStatus);

			//TODO: Declare an alarm
			//TODO: Start circling
		}
	}

}

//<<<UGH, This function is badly named. All it's doing is calculating the angle between two vectors>>>
//Calculate the angle between two vectors, using simple vector calculus.
static float updateArcMeasure(float oldPosition_NE[2], float newPosition_NE[2], float arcCenter_NE[2])
{
	float a[2] = {arcCenter_NE[0] - oldPosition_NE[0], arcCenter_NE[1] - oldPosition_NE[1]};
	float b[2] = {arcCenter_NE[0] - newPosition_NE[0], arcCenter_NE[1] - newPosition_NE[1]};

	// arccos(a.b/(|a||b|))
	float theta = acosf((a[0]*b[0] + a[1]*b[1])/(sqrtf(a[0]*a[0] + a[1]*a[1]) * sqrtf(b[0]*b[0] + b[1]*b[1])));
	return theta;
}


/**
 * @}
 * @}
 */
