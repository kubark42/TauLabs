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

#include "fixedwingairspeeds.h"

#include "pathfollowerstatus.h"
#include "pathmanagerstatus.h"
#include "pathmanagersettings.h"
#include "pathsegmentdescriptor.h"

#include "CoordinateConversions.h"

// Private constants
#define STACK_SIZE_BYTES 700
#define TASK_PRIORITY (tskIDLE_PRIORITY+1)
#define MAX_QUEUE_SIZE 2
#define UPDATE_RATE_MS 100
#define IDLE_UPDATE_RATE_MS (200-UPDATE_RATE_MS)
#define OVERSHOOT_TIMER_MS 1000
#define ANGULAR_PROXIMITY_THRESHOLD 30

#define sign(x) (x < 0 ? -1 : 1)

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
static FixedWingAirspeedsData fixedWingAirspeeds;
static PathManagerSettingsData pathManagerSettings;
static PathManagerStatusData pathManagerStatus;
static PathSegmentDescriptorData pathSegmentDescriptor_current;
static portTickType segmentTimer;
static float angularDistanceToComplete;
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
		PathManagerSettingsInitialize();
		PathSegmentDescriptorInitialize();

		FixedWingAirspeedsInitialize(); //TODO: This shouldn't really be here, as it's airframe specific

		// Create object queue
		queue = xQueueCreate(MAX_QUEUE_SIZE, sizeof(UAVObjEvent)); //TODO: Is this even necessary?

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
	PathManagerSettingsConnectCallback(settingsUpdated);
	FixedWingAirspeedsConnectCallback(settingsUpdated);
	PathSegmentDescriptorConnectCallback(pathSegmentDescriptorsUpdated);

	// Force reload all settings
	settingsUpdated(NULL);

	// Initialize all main loop variables
	bool pathplanner_active = false;
	int8_t arc_has_center = INSUFFICIENT_RADIUS;
	static portTickType lastSysTime;
	static portTickType overshootTimer;
	lastSysTime = xTaskGetTickCount();
	overshootTimer = xTaskGetTickCount();
	uint8_t guidanceType = NOMANAGER;
	uint8_t theta_roundoff_trim_count = 0;

	// Main thread loop
	while (1)
	{

		// Wait
		vTaskDelayUntil(&lastSysTime, UPDATE_RATE_MS * portTICK_RATE_MS);

		// Check flight mode
		FlightStatusData flightStatus;
		FlightStatusGet(&flightStatus);
		switch (flightStatus.FlightMode) {
#if 1
			case FLIGHTSTATUS_FLIGHTMODE_RETURNTOHOME:
				if(guidanceType != RETURNHOME){
					guidanceType = RETURNHOME;
					pathplanner_active = false;

					PathSegmentDescriptorData pathSegmentDescriptor;

					for (int i=UAVObjGetNumInstances(PathSegmentDescriptorHandle()); i<=3; i++){
						PathSegmentDescriptorCreateInstance();
					}

					PositionActualData positionActual;
					PositionActualGet(&positionActual);

					// First locus is current vehicle position
					pathSegmentDescriptor.SwitchingLocus[0] = positionActual.North;
					pathSegmentDescriptor.SwitchingLocus[1] = positionActual.East;
					pathSegmentDescriptor.SwitchingLocus[2] = positionActual.Down;
					pathSegmentDescriptor.FinalVelocity = 12.1f; //TODO: Relate this to the airspeeds
					pathSegmentDescriptor.DesiredAcceleration = 0;
					pathSegmentDescriptor.Timeout = 0;
					pathSegmentDescriptor.NumberOfOrbits = 0;
					pathSegmentDescriptor.PathCurvature = 0;
					pathSegmentDescriptor.ArcRank = PATHSEGMENTDESCRIPTOR_ARCRANK_MINOR;
					PathSegmentDescriptorInstSet(0, &pathSegmentDescriptor);

					// Calculate direction from home to initial segment
					float radius = 60;
					float approachTheta_rad = atan2f(pathSegmentDescriptor.SwitchingLocus[1], pathSegmentDescriptor.SwitchingLocus[0]);

					// Go straight back to home
					pathSegmentDescriptor.SwitchingLocus[0] = cos(approachTheta_rad) * radius;
					pathSegmentDescriptor.SwitchingLocus[1] = sin(approachTheta_rad) * radius;
					pathSegmentDescriptor.SwitchingLocus[2] = positionActual.Down - 10;
					pathSegmentDescriptor.FinalVelocity = 12.1f;
					pathSegmentDescriptor.DesiredAcceleration = 0;
					pathSegmentDescriptor.Timeout = 60;
					pathSegmentDescriptor.NumberOfOrbits = 0;
					pathSegmentDescriptor.PathCurvature = 0;
					pathSegmentDescriptor.ArcRank = PATHSEGMENTDESCRIPTOR_ARCRANK_MINOR;
					PathSegmentDescriptorInstSet(1, &pathSegmentDescriptor);

					// Orbit home
					pathSegmentDescriptor.SwitchingLocus[0] = -cos(approachTheta_rad) * radius;
					pathSegmentDescriptor.SwitchingLocus[1] = -sin(approachTheta_rad) * radius;
					pathSegmentDescriptor.SwitchingLocus[2] = positionActual.Down - 10;
					pathSegmentDescriptor.FinalVelocity = 12.1f;
					pathSegmentDescriptor.DesiredAcceleration = 0;
					pathSegmentDescriptor.Timeout = 60;
					pathSegmentDescriptor.PathCurvature = 1/radius;
					pathSegmentDescriptor.NumberOfOrbits = 1e8; //TODO: Define this really large floating-point value as a magic number
					pathSegmentDescriptor.ArcRank = PATHSEGMENTDESCRIPTOR_ARCRANK_MINOR;
					PathSegmentDescriptorInstSet(2, &pathSegmentDescriptor);
				}
				break;
			case FLIGHTSTATUS_FLIGHTMODE_POSITIONHOLD:
				if(guidanceType != HOLDPOSITION){
					guidanceType = HOLDPOSITION;
					pathplanner_active = false;

					PathSegmentDescriptorData pathSegmentDescriptor;

					for (int i=UAVObjGetNumInstances(PathSegmentDescriptorHandle()); i<=2; i++){
						PathSegmentDescriptorCreateInstance();
					}

					PositionActualData positionActual;
					PositionActualGet(&positionActual);

					float radius = 60;

					// First locus is current vehicle position
					pathSegmentDescriptor.SwitchingLocus[0] = positionActual.North - radius;
					pathSegmentDescriptor.SwitchingLocus[1] = positionActual.East;
					pathSegmentDescriptor.SwitchingLocus[2] = positionActual.Down;
					pathSegmentDescriptor.FinalVelocity = 12.1f; //TODO: Relate this to the airspeeds
					pathSegmentDescriptor.DesiredAcceleration = 0;
					pathSegmentDescriptor.Timeout = 0;
					pathSegmentDescriptor.NumberOfOrbits = 0;
					pathSegmentDescriptor.PathCurvature = 0;
					pathSegmentDescriptor.ArcRank = PATHSEGMENTDESCRIPTOR_ARCRANK_MINOR;
					PathSegmentDescriptorInstSet(0, &pathSegmentDescriptor);

					// Orbit current position
					pathSegmentDescriptor.SwitchingLocus[0] = positionActual.North + radius;
					pathSegmentDescriptor.SwitchingLocus[1] = positionActual.East;
					pathSegmentDescriptor.SwitchingLocus[2] = positionActual.Down - 10;
					pathSegmentDescriptor.FinalVelocity = 12.1f;
					pathSegmentDescriptor.DesiredAcceleration = 0;
					pathSegmentDescriptor.Timeout = 60;
					pathSegmentDescriptor.PathCurvature = 1/radius;
					pathSegmentDescriptor.NumberOfOrbits = 1e8; //TODO: Define this really large floating-point value as a magic number
					pathSegmentDescriptor.ArcRank = PATHSEGMENTDESCRIPTOR_ARCRANK_MINOR;
					PathSegmentDescriptorInstSet(1, &pathSegmentDescriptor);
				}
				break;
#endif //NO_PATHPLANNER
			case FLIGHTSTATUS_FLIGHTMODE_PATHPLANNER:
				if(guidanceType != PATHPLANNER){
					guidanceType = PATHPLANNER;
					pathplanner_active = false;

					PathSegmentDescriptorData pathSegmentDescriptor;

					for (int i=UAVObjGetNumInstances(PathSegmentDescriptorHandle()); i<=6; i++){
						PathSegmentDescriptorCreateInstance();
					}

					PositionActualData positionActual;
					PositionActualGet(&positionActual);

					// First locus is current vehicle position
					pathSegmentDescriptor.SwitchingLocus[0] = positionActual.North;
					pathSegmentDescriptor.SwitchingLocus[1] = positionActual.East;
					pathSegmentDescriptor.SwitchingLocus[2] = positionActual.Down;
					pathSegmentDescriptor.FinalVelocity = 12.1f; //TODO: Relate this to the airspeeds
					pathSegmentDescriptor.DesiredAcceleration = 0;
					pathSegmentDescriptor.Timeout = 0;
					pathSegmentDescriptor.NumberOfOrbits = 0;
					pathSegmentDescriptor.PathCurvature = 0;
					pathSegmentDescriptor.ArcRank = PATHSEGMENTDESCRIPTOR_ARCRANK_MINOR;
					PathSegmentDescriptorInstSet(0, &pathSegmentDescriptor);

					pathSegmentDescriptor.SwitchingLocus[0] = 100;
					pathSegmentDescriptor.SwitchingLocus[1] = 0;
					pathSegmentDescriptor.SwitchingLocus[2] = -2450;
					pathSegmentDescriptor.FinalVelocity = 12.1f;
					pathSegmentDescriptor.Timeout = 60;
					pathSegmentDescriptor.NumberOfOrbits = 0;
					pathSegmentDescriptor.PathCurvature = 0;
					PathSegmentDescriptorInstSet(1, &pathSegmentDescriptor);

					pathSegmentDescriptor.SwitchingLocus[0] = 100;
					pathSegmentDescriptor.SwitchingLocus[1] = 200;
					pathSegmentDescriptor.SwitchingLocus[2] = -2450;
					pathSegmentDescriptor.FinalVelocity = 12.1f;
					pathSegmentDescriptor.Timeout = 60;
					pathSegmentDescriptor.NumberOfOrbits = 0;
					pathSegmentDescriptor.PathCurvature = 0;
					PathSegmentDescriptorInstSet(2, &pathSegmentDescriptor);

					pathSegmentDescriptor.SwitchingLocus[0] = 100;
					pathSegmentDescriptor.SwitchingLocus[1] = 200+120;
					pathSegmentDescriptor.SwitchingLocus[2] = -2500;
					pathSegmentDescriptor.FinalVelocity = 12.4f;
					pathSegmentDescriptor.Timeout = 60;
					pathSegmentDescriptor.PathCurvature = 1/60.0f; // 70m radius
					pathSegmentDescriptor.NumberOfOrbits = 1;
					pathSegmentDescriptor.ArcRank = PATHSEGMENTDESCRIPTOR_ARCRANK_MINOR;
					PathSegmentDescriptorInstSet(3, &pathSegmentDescriptor);

					pathSegmentDescriptor.SwitchingLocus[0] = 100;
					pathSegmentDescriptor.SwitchingLocus[1] = 0;
					pathSegmentDescriptor.SwitchingLocus[2] = -2500;
					pathSegmentDescriptor.FinalVelocity = 12.1f;
					pathSegmentDescriptor.Timeout = 60;
					pathSegmentDescriptor.NumberOfOrbits = 0;
					pathSegmentDescriptor.PathCurvature = 0;
					PathSegmentDescriptorInstSet(4, &pathSegmentDescriptor);

					pathSegmentDescriptor.SwitchingLocus[0] = 600;
					pathSegmentDescriptor.SwitchingLocus[1] = 0;
					pathSegmentDescriptor.SwitchingLocus[2] = -2500;
					pathSegmentDescriptor.FinalVelocity = 12.2f;
					pathSegmentDescriptor.Timeout = 60;
					pathSegmentDescriptor.PathCurvature = 1/400.0f;
					pathSegmentDescriptor.NumberOfOrbits = 0;
					pathSegmentDescriptor.ArcRank = PATHSEGMENTDESCRIPTOR_ARCRANK_MINOR;
					PathSegmentDescriptorInstSet(5, &pathSegmentDescriptor);

					pathSegmentDescriptor.SwitchingLocus[0] = 800;
					pathSegmentDescriptor.SwitchingLocus[1] = 0;
					pathSegmentDescriptor.SwitchingLocus[2] = -2500;
					pathSegmentDescriptor.FinalVelocity = 12.3f;
					pathSegmentDescriptor.Timeout = 60;
					pathSegmentDescriptor.PathCurvature = 0;
					pathSegmentDescriptor.NumberOfOrbits = 0;
					PathSegmentDescriptorInstSet(6, &pathSegmentDescriptor);
				}
				break;
			default:
				// When not running the path manager, short circuit and wait
				pathplanner_active = false;
				guidanceType = NOMANAGER;
				vTaskDelay(IDLE_UPDATE_RATE_MS * portTICK_RATE_MS);
				continue;
		}

		bool advanceSegment_flag = false;

		// Update arc measure traveled
		if (pathSegmentDescriptor_current.PathCurvature != 0){
			PositionActualData positionActual;
			PositionActualGet(&positionActual);
			float newPosition_NE[2] = {positionActual.North, positionActual.East};
			if (arc_has_center == CENTER_FOUND){
				angularDistanceCompleted  += updateArcMeasure(oldPosition_NE, newPosition_NE, arcCenter_NE) * RAD2DEG;

				oldPosition_NE[0] = newPosition_NE[0];
				oldPosition_NE[1] = newPosition_NE[1];
			}

			// Every 128 samples, correct for roundoff error. Error doesn't accumulate too quickly, so
			// this trigger value can safely be made much higher, with the condition that the type of
			// theta_roundoff_trim_count be changed from uint8_t;
			if ((theta_roundoff_trim_count++ & 0x8F) == 0){
				theta_roundoff_trim_count = 0;

				float referenceTheta = updateArcMeasure(previousLocus->Position, newPosition_NE, arcCenter_NE) * RAD2DEG;

				while(referenceTheta-angularDistanceCompleted < -180){
					referenceTheta += 360;
				}
				while(referenceTheta-angularDistanceCompleted > 180){
					referenceTheta -= 360;
				}

				angularDistanceCompleted = referenceTheta;
			}
		}

		pathManagerStatus.StatusParameters[0] = angularDistanceCompleted;
		pathManagerStatus.StatusParameters[1] = angularDistanceToComplete;
		PathManagerStatusSet(&pathManagerStatus);

		// If the vehicle is sufficiently close to the goal, check if it has achieved the goal
		// of the active path segment. Sufficiently close is chosen to be an arbitrary angular
		// distance, as this is robust and sufficient to describe all paths, including infinite
		// straight lines and infinite number of orbits about a point.
		if( sign(pathSegmentDescriptor_current.PathCurvature) * (angularDistanceToComplete - angularDistanceCompleted) < ANGULAR_PROXIMITY_THRESHOLD)
			advanceSegment_flag = checkGoalCondition();
		else
			pathManagerStatus.StatusParameters[2] = rand();


		// Check if the path_manager was just activated
		if(pathplanner_active == false) {
			PathManagerStatusGet(&pathManagerStatus);
			pathManagerStatus.ActiveSegment = 0;
			pathManagerStatus.Status = PATHMANAGERSTATUS_STATUS_INPROGRESS;
			PathManagerStatusSet(&pathManagerStatus);

			advanceSegment_flag = true;
			pathplanner_active = true;

			// Reset timer
			segmentTimer = xTaskGetTickCount();
		}


		// Advance segment
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

			// Load current segment into global memory.
			PathSegmentDescriptorInstGet(pathManagerStatus.ActiveSegment, &pathSegmentDescriptor_current);

			// Reset angular distance
			angularDistanceCompleted = 0;

			// If the path is an arc, find the center and angular distance along arc
			if (pathSegmentDescriptor_current.PathCurvature != 0 ){
				// Determine if the arc has a center, and if so assign it to arcCenter_NE
				arc_has_center = arcCenterFromTwoPointsAndRadiusAndArcRank(previousLocus->Position, pathSegmentDescriptor_current.SwitchingLocus,
					1.0f/pathSegmentDescriptor_current.PathCurvature, arcCenter_NE,
					pathSegmentDescriptor_current.PathCurvature > 0, pathSegmentDescriptor_current.ArcRank == PATHSEGMENTDESCRIPTOR_ARCRANK_MINOR);

				pathManagerStatus.StatusParameters[8] = arcCenter_NE[0];
				pathManagerStatus.StatusParameters[9] = arcCenter_NE[1];

				// If the arc has a center, then set the initial position as the beginning of the arc, and calculate the angular
				// distance to be traveled along the arc
				if (arc_has_center == CENTER_FOUND){
					oldPosition_NE[0] = previousLocus->Position[0];
					oldPosition_NE[1] = previousLocus->Position[1];

					float tmpAngle = updateArcMeasure(previousLocus->Position, pathSegmentDescriptor_current.SwitchingLocus, arcCenter_NE) * RAD2DEG;
					if (sign(pathSegmentDescriptor_current.PathCurvature) * tmpAngle < 0)
					{
						tmpAngle = tmpAngle	+ 360*sign(pathSegmentDescriptor_current.PathCurvature);
					}
					angularDistanceToComplete = sign(pathSegmentDescriptor_current.PathCurvature) * pathSegmentDescriptor_current.NumberOfOrbits*360 + tmpAngle;
				}
				else{
					// TODO: This is really bad, and we need to handle these cases. We can probably handle them just by extending the vector until it reaches the arc center
					angularDistanceToComplete = 0;
				}
			}
			else{
				angularDistanceToComplete = 0;
			}

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
	if(pathManagerSettings.SwitchingStrategy == PATHMANAGERSETTINGS_SWITCHINGSTRATEGY_HALFPLANE) {
		// Check if there is a switching locus after the present one
		if (pathManagerStatus.ActiveSegment + 1 < UAVObjGetNumInstances(PathSegmentDescriptorHandle())){
			// Calculate vector from past to preset switching locus
			float *swl_past = previousLocus->Position;
			float *swl_current = pathSegmentDescriptor_current.SwitchingLocus;
			float q_current[3] = {swl_current[0] - swl_past[0], swl_current[1] - swl_past[1], 0};
			float q_current_mag = VectorMagnitude(q_current); //Normalize
			float q_future[3];
			float q_future_mag;

			PathSegmentDescriptorData pathSegmentDescriptor_future;
			PathSegmentDescriptorInstGet(pathManagerStatus.ActiveSegment+1, &pathSegmentDescriptor_future);

			// Line-line intersection
			if(pathSegmentDescriptor_current.PathCurvature == 0 && pathSegmentDescriptor_future.PathCurvature == 0){
				float *swl_future  = pathSegmentDescriptor_future.SwitchingLocus;

				// Calculate vector from preset to future switching locus
				q_future [0] = swl_future[0] - swl_current[0];
				q_future [1] = swl_future[1] - swl_current[1];
				q_future [2] = 0;
				q_future_mag = VectorMagnitude(q_future); //Normalize

			}
			// "Small Unmanned Aircraft: Theory and Practice" provides no guidance for the perpendicular
			// intersection of a line and an arc. However, it seems reasonable to consider the halfplane
			// as occurring at the intersection between the vector and the arc, with the frontier defined
			// as the half-angle between the arriving vector and the departing arc tangent, similar to in
			// the line-line case.
			//
			// The nice part about this approach is that it works equally well for a tangent curve, as the intersection
			// occurs at the tangent of the circle. In the case that due to numerical error the vector and arc do
			// not intersect, we will still test for crossing into the half plane defined of a line drawn between the arc's
			// center and the closest point on the vector to the arc.
			else if(pathSegmentDescriptor_current.PathCurvature == 0 && pathSegmentDescriptor_future.PathCurvature != 0)
			{
				// Calculate vector tangent to arc at preset switching locus. This comes from geometry that that tangent to a circle
				// is the perpendicular vector to the vector connecting the tangent point and the center of the circle. The vector R
				// is tangent_point - arc_center, so the perpendicular to R is <-lambda*Ry,lambda*Rx>, where lambda = +-1.

				bool clockwise = pathSegmentDescriptor_future.PathCurvature > 0;
				bool minor = pathSegmentDescriptor_future.ArcRank == PATHSEGMENTDESCRIPTOR_ARCRANK_MINOR;
				int8_t lambda;

				if ((clockwise == true && minor == true) ||
						(clockwise == false && minor == false)) { //clockwise minor OR counterclockwise major
					lambda = 1;
				} else { //counterclockwise minor OR clockwise major
					lambda = -1;
				}

				// Vector perpendicular to the vector from arc center to tangent point
				q_future[0] = -lambda*(swl_current[1] - arcCenter_NE[1]);
				q_future[1] = lambda*(swl_current[0] - arcCenter_NE[0]);
				q_future[2] = 0;
				q_future_mag = VectorMagnitude(q_future); //Normalize


				pathManagerStatus.StatusParameters[1] = rand();
				pathManagerStatus.StatusParameters[2] = arcCenter_NE[0];
				pathManagerStatus.StatusParameters[3] = arcCenter_NE[1];
				pathManagerStatus.StatusParameters[4] = swl_past[0];
				pathManagerStatus.StatusParameters[5] = swl_past[1];
				pathManagerStatus.StatusParameters[6] = q_future[0]+.01*pathManagerStatus.ActiveSegment;
				pathManagerStatus.StatusParameters[7] = q_future[1]+.03;
				pathManagerStatus.StatusParameters[8] = q_current[0]+.02;
				pathManagerStatus.StatusParameters[9] = q_current[1]+.04;
				PathManagerStatusSet(&pathManagerStatus);

			}
			// "Small Unmanned Aircraft: Theory and Practice" provides no guidance for the perpendicular
			// intersection of an arc and a line. However, it seems reasonable to consider the halfplane
			// occurring at the intersection between the arc and the vector. The halfplane's frontier
			// is perpendicular to the tangent at the end of the arc.
			else if(pathSegmentDescriptor_current.PathCurvature != 0 && pathSegmentDescriptor_future.PathCurvature == 0)
			{
				// Cheat by remarking that the plane defined by the radius is perfectly defined by the angle made
				// between the center and the end of the trajectory. So if the vehicle has traveled further than
				// the required angular distance, it has crossed this
				if (sign(pathSegmentDescriptor_current.PathCurvature) * (angularDistanceCompleted - angularDistanceToComplete) >= 0)
					advanceSegment_flag = true;

				return advanceSegment_flag;
			}
			else{
				// Shouldn't be able to get here. Something has gone wrong.
				// TODO.
				AlarmsSet(SYSTEMALARMS_ALARM_PATHMANAGER, SYSTEMALARMS_ALARM_CRITICAL);
				return false;
			}

			// Compute the half-plane frontier as the line perpendicular to the sum of the approach and
			// departure vectors. See Fig 11.1 in reference.
			//
			// We're going to take a litle mathematical shortcut, by utilizing the fact that we don't need the actual
			// normalized normal vector, any normal vector will do. If a and b are vectors, then
			// a/|a|+b/|b| = 1/(|a||b|)*(a*|b| + b*|a|), which points in the same direction as (a*|b| + b*|a|)
			float halfPlane[3] = {q_future[0]*q_current_mag + q_current[0]*q_future_mag,
								q_future[1]*q_current_mag + q_current[1]*q_future_mag,
								q_future[2]*q_current_mag + q_current[2]*q_future_mag};

			// Test if the UAV is in the half plane, H. This is easy by taking advantage of simple vector
			// calculus: a.b = |a||b|cos(theta), but since |a|,|b| >=0, then a.b > 0 if and only if
			// cos(theta) > 0, which means that the UAV is in I or IV quadrants, i.e. is in the half plane.
			PositionActualData positionActual;
			PositionActualGet(&positionActual);
			float p[2] = {positionActual.North - swl_current[0], positionActual.East - swl_current[1]};

			// If we want to switch based on nominal time to locus, add the normalized q_current times the speed times the timing advace
			if(pathManagerSettings.HalfPlaneAdvanceTiming != 0){
				for (int i=0; i<2; i++){
						p[i] += q_current[i]/q_current_mag * fixedWingAirspeeds.BestClimbRateSpeed * (pathManagerSettings.HalfPlaneAdvanceTiming/1000.0f);
				}
			}

			// Finally test a.b > 0
			if(p[0]*halfPlane[0] + p[1]*halfPlane[1] > 0){
				advanceSegment_flag = true;
			}
		}
		else{ // Since there are no further switching loci, this must be the waypoint.
			//Do nothing.
		}
	}
	// B-ball approach. This tests if the vehicle is within a threshold distance.
	// From R. Beard and T. McLain, "Small Unmanned Aircraft: Theory and Practice", 2011, Section 11.1.
	else if (pathManagerSettings.SwitchingStrategy == PATHMANAGERSETTINGS_SWITCHINGSTRATEGY_BBALL) {
		// This method is less robust to error than the half-plane. It is cheaper and simpler, but those are it's only two advantages
		PositionActualData positionActual;
		PositionActualGet(&positionActual);
		float d[3] = {positionActual.North - pathSegmentDescriptor_current.SwitchingLocus[0], positionActual.East - pathSegmentDescriptor_current.SwitchingLocus[1], 0};

		if (VectorMagnitude(d) < pathManagerSettings.BBallThresholdDistance)
		{
			advanceSegment_flag = true;
		}
	}
	else {
		// TODO: This is bad to get here. Make sure it's not possible.
	}

	return advanceSegment_flag;
}

//Check to see if we've seriously overflown our destination. Since the path follower is simply following a
//motion descriptor,it has no concept of where the path ends. It will simply keep following it to infinity
//if we don't stop it.
//So while we don't know why the navigation manager failed, we know we don't want the plane flying off.
static void checkOvershoot()
{
	// TODO: Check for overshoot with non-infinite arcs, too.
	if (pathSegmentDescriptor_current.PathCurvature == 0) {
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
		if (0 && sqrtf(powf((p[0]-r[0])*q[0],2)+powf((p[1]-r[1])*q[1],2)) > pathLength+5.0f*pathSegmentDescriptor_current.FinalVelocity){
			//Whoops, we've really overflown our destination point, and haven't received any instructions.

			//Inform the FSM
			pathManagerStatus.Status = PATHMANAGERSTATUS_STATUS_CRITICAL;
			PathManagerStatusSet(&pathManagerStatus);

			//TODO: Declare an alarm
			AlarmsSet(SYSTEMALARMS_ALARM_PATHMANAGER, SYSTEMALARMS_ALARM_CRITICAL);
			//TODO: Start circling
		}
	}

}

//<<<UGH, This function is badly named. All it's doing is calculating the angle between two vectors>>>
//Calculate the angle between two vectors, using simple vector calculus.
static float updateArcMeasure(float oldPosition_NE[2], float newPosition_NE[2], float arcCenter_NE[2])
{
	float a[2] = {oldPosition_NE[0] - arcCenter_NE[0], oldPosition_NE[1] - arcCenter_NE[1]};
	float b[2] = {newPosition_NE[0] - arcCenter_NE[0], newPosition_NE[1] - arcCenter_NE[1]};

	// We cannot directly use the vector calculus formula for cos(theta) and sin(theta) because each
	// is only defined on half the circle. Instead, we combine the two because tangent is defined across
	// [-pi,pi]. Use the definition of the cross-product for 2-D vectors, a x b = |a||b| sin(theta), and
	// the definition of the dot product, a.b = |a||b| cos(theta), and divide the first by the second,
	// yielding a x b / (a.b) = sin(theta)/cos(theta) == tan(theta)
	float theta = atan2f(a[0]*b[1] - a[1]*b[0],(a[0]*b[0] + a[1]*b[1]));
	return theta;
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
	if (ev == NULL || ev->obj == PathManagerSettingsHandle()) {
		PathManagerSettingsGet(&pathManagerSettings);
	}
	if (ev == NULL || ev->obj == FixedWingAirspeedsHandle()) {
		FixedWingAirspeedsGet(&fixedWingAirspeeds);
	}
}


/**
 * @}
 * @}
 */
