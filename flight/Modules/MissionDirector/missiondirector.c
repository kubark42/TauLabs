/**
 ******************************************************************************
 * @addtogroup TauLabsModules Tau Labs Modules
 * @{
 * @addtogroup MissionDirectorModule Mission director module
 * @{
 *
 * @file       missiondirector.c
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2013
 * @brief
 *
 * @see        The GNU Public License (GPL) Version 3
 *
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

/**
 * Input objects: @ref Accels, @ref MissionDirectorSettings
 * Output object: @ref MissionDirectorOutput
 *
 * This module executes on a timer trigger. When the module is
 * triggered it will update the data of VibrationAnalysiOutput, based on
 * the output of an FFT running on the accelerometer samples. 
 */

#include "openpilot.h"
#include "misc_math.h"
#include "physical_constants.h"

#include "airfieldsettings.h"
#include "missiondirectorsettings.h"
#include "missiondirectorstatus.h"
#include "missiondirectoruserprogram.h"
#include "modulesettings.h"
#include "pathsegmentdescriptor.h"
#include "waypoint.h"
int8_t initialize_landing(); //<-- FIXME: Replace this by an include to the proper library function

// Private constants
enum airplane_configuration {
	CLEAN,
	FIRST_FLAPS,
	SECOND_FLAPS,
	FULL_FLAPS
};

enum landing_fsm {
	APPROACHING_IAP, // Initial Approach Point
	APPROACHING_LTP, // Last Turn Point
	APPROACHING_LAP, // Last Approach Point
	APPROACHING_RSP, // Runway Start Point
	APPROACHING_REP  // Runway End Point
};


enum landing_waypoints {
	WAYPOINT_IAP, // Initial Approach Point
	WAYPOINT_LTP, // Last Turn Point
	WAYPOINT_LAP, // Last Approach Point
	WAYPOINT_RSP, // Runway Start Point
	WAYPOINT_REP  // Runway End Point
};


#define MAX_QUEUE_SIZE 2
#define STACK_SIZE_BYTES 2000
#define TASK_PRIORITY (tskIDLE_PRIORITY+2)
#define MISSION_DIRECTOR_PERIOD_MS 1000

// Private variables
static xTaskHandle taskHandle;
static bool module_enabled = false;

// Private functions
static void MissionDirectorTask(void *parameters);
static bool is_success_possible(enum landing_fsm landing_fsm, enum airplane_configuration airplane_configuration);
static bool is_mission_completed();

/**
 * Start the module, called on startup
 */
static int32_t MissionDirectorStart(void)
{
	
	if (!module_enabled)
		return -1;

	// Start main task
	xTaskCreate(MissionDirectorTask, (signed char *)"MissionDirector", STACK_SIZE_BYTES/4, NULL, TASK_PRIORITY, &taskHandle);
	TaskMonitorAdd(TASKINFO_RUNNING_MISSIONDIRECTOR, taskHandle);
	return 0;
}


/**
 * Initialise the module, called on startup
 */
static int32_t MissionDirectorInitialize(void)
{
	ModuleSettingsInitialize();
	
#ifdef MODULE_MissionDirector_BUILTIN
	module_enabled = true;
#else
	uint8_t module_state[MODULESETTINGS_ADMINSTATE_NUMELEM];
	ModuleSettingsAdminStateGet(module_state);
	if (module_state[MODULESETTINGS_ADMINSTATE_MISSIONDIRECTOR] == MODULESETTINGS_ADMINSTATE_ENABLED) {
		module_enabled = true;
	} else {
		module_enabled = false;
	}
#endif
	
	if (!module_enabled) //If module not enabled...
		return -1;

	// Initialize UAVOs
	AirfieldSettingsInitialize();
	MissionDirectorSettingsInitialize();
	MissionDirectorStatusInitialize();
	MissionDirectorUserProgramInitialize();

	return 0;
}

MODULE_INITCALL(MissionDirectorInitialize, MissionDirectorStart);


static void MissionDirectorTask(void *parameters)
{
	// FIXME: This obviously shouldn't be here, since it's landing specific
	{
		// Don't run if the necessary UAVOs are not instantiated
		while (AirfieldSettingsHandle() == NULL ||
			WaypointHandle() == NULL) {
			// Delay 1 second
			vTaskDelay(TICKS2MS(1000));

			MissionDirectorStatusData missionDirectorStatus;
			MissionDirectorStatusGet(&missionDirectorStatus);
			missionDirectorStatus.MissionID = rand() %100;
			MissionDirectorStatusSet(&missionDirectorStatus);

		}

		int8_t ret = initialize_landing();
		if (ret != 0)
		{
			vTaskDelay(TICKS2MS(1000));

			MissionDirectorStatusData missionDirectorStatus;
			MissionDirectorStatusGet(&missionDirectorStatus);
			missionDirectorStatus.Latitude = rand() %100;
			missionDirectorStatus.MissionID = WaypointGetNumInstances();
			MissionDirectorStatusSet(&missionDirectorStatus);
		}
	}

	portTickType lastSysTime;
	bool success_is_possible = true;
	bool is_succeeded = false;
	uint8_t old_MissionID = 0;

	enum airplane_configuration airplane_configuration = CLEAN;
	enum landing_fsm landing_fsm = APPROACHING_IAP;

	// Main task loop
	lastSysTime = xTaskGetTickCount();
	while (1) {
		MissionDirectorStatusData missionDirectorStatus;
		MissionDirectorStatusGet(&missionDirectorStatus);
		vTaskDelayUntil(&lastSysTime, MS2TICKS(MISSION_DIRECTOR_PERIOD_MS));

		if (old_MissionID != missionDirectorStatus.MissionID) {
			old_MissionID = missionDirectorStatus.MissionID;
			success_is_possible = true;
			is_succeeded = false;
		}

		// Check for success.
		/*
		 * Right now, success is just if we have arrived at the end of our path plan
		 */
		is_succeeded = is_mission_completed();
		if (is_succeeded == true) {
			continue;
		}

		// Check if success is possible.
		/*
		 * Start with two simple cases, one for takeoff and one for landing.
		 */
		success_is_possible = is_success_possible(landing_fsm, airplane_configuration);

		// If success is impossible, go to plan B.
		if (success_is_possible == false) {
			continue;
		}
	}
}


//-----------------------------------------------------//
//-----------------------------------------------------//
//-----------------------------------------------------//
//-----------------------------------------------------//
//-----------------------------------------------------//
//-----------------------------------------------------//
//-----------------------------------------------------//


//-----------------------------------------------------//
//-----------------------------------------------------//
//-----------------------------------------------------//
//-----------------------------------------------------//
//-----------------------------------------------------//
//-----------------------------------------------------//
//-----------------------------------------------------//






float NED[3]; // UAV's NED coordinates, in [m]
float calibrated_airspeed; // UAV's calibrated airspeed. We use calibrated airspeed because we're interested in the stall speed

/* From http://www.pprune.org/tech-log/317172-vertical-speed-touchdown.html
 * "The A330 will print a Post Flight Report obviously at the end of the flight.
 *  One endeavours to avoid a Load 15 report but the following info comes from my OM Part A:
 *
 *  There are several categories of hard landing and this will dictate the level of post flight inspection required.
 *
 *  The criteria and parameters for aircraft type are as follows.
 *  1. A hard landing is a landing with an aircraft weight less than the Maximum Landing Weight (MLW) and:-
 *    * a vertical acceleration (Vert G) equal to or more than 2.6g and less than 2.86g at aircraft Centre of Gravity (CG) or,
 *    * a vertical speed (Vs) equal to or more than 10 ft/sec and less than 14 ft/sec.
 *  2. A severe hard landing is a landing with an aircraft weight less than the Maximum Landing Weight (MLW) and :-
 *    * a vertical acceleration (Vert G) equal to or more than 2.86g at aircraft Centre of Gravity (CG) or,
 *    * a vertical speed (Vs) equal to or more than 14 ft/sec."
 *
 * This leads to a design goal of 1m/s at touchdown, up to a 3 degree glide path (standard for airports) at
 * which the design speed allows for up to 60m/s (~116kts) touchdown speed without requiring a flare.
*/


#define MAXIMUM_VERTICAL_TOUCHDOWN_SPEED 3.0f // in [m/s]
#define DESIRED_VERTICAL_TOUCHDOWN_SPEED 1.0f // in [m/s]
#define MINIMUM_VERTICAL_TOUCHDOWN_SPEED 0.5f // in [m/s]

#define MINIMUM_GLIDEPATH_ANGLE_DEG ((2.0f) * DEG2RAD) // Never allow a glidepath angle below this threshold. PAPI are generally adjusted to show all red below 2 degrees.
#define MAXIMUM_GLIDEPATH_ANGLE_DEG ((20.0f) * DEG2RAD) // Never allow a glidepath angle above this threshold
#define MINIMUM_DESIRED_GLIDEPATH_ANGLE_DEG ((3.0f) * DEG2RAD) // Never allow a desired glidepath angle below this threshold. 3 degrees is standard approach angle at most airports
#define MAXIMUM_DESIRED_GLIDEPATH_ANGLE_DEG ((15.0f) * DEG2RAD) // Never allow a desired glidepath angle above this threshold.
#define MINIMUM_BEARING_ANGLE_DEG (4 * DEG2RAD) // We will allow +-2 deg lateral deviation from runway

#define stall_speed_clean 6
#define stall_speed_first_flaps 5
#define stall_speed_second_flaps 5
#define stall_speed_full_flaps 5
#define STALL_SPEED_MARGIN_RATIO (1 + 0.04f) // Stall margin ratio

#define MINIMUM_PATTERN_HEIGHT 30 //Minimum height in [m]. FIXME: THIS SHOULD NOT BE A MAGIC NUMBER. IT SHOULD BE A FUNCTION OF AIRPLANE SPEED AND GLIDE RATIO
#define MAXIMUM_PATTERN_HEIGHT 50 //Minimum height in [m]. FIXME: THIS SHOULD NOT BE A MAGIC NUMBER. IT SHOULD BE A FUNCTION OF AIRPLANE SPEED AND GLIDE RATIO
#define DESIRED_PATTERN_HEIGHT 70 //Minimum height in [m]. FIXME: THIS SHOULD NOT BE A MAGIC NUMBER. IT SHOULD BE A FUNCTION OF AIRPLANE SPEED AND GLIDE RATIO
#define CROSS_TRACK_PATTERN_ERROR 20; // Allowable error between IAP and LTP, in [m]. FIXME: THIS SHOULD NOT BE A MAGIC NUMBER, BUT INSTEAD BE A FUNCTION OF BANK ANGLE AND AIRSPEED. THE IDEA IS TO ALLOW THE AIRPLANE TO APPROACH THE IAP FROM THE COMPLETELY OPPOSITE DIRECTION TO THE LTP AND YET STILL BE ABLE TO DO A BANKING MANEUVER FROM THE IAP TO THE LTP

// All tuples in NED coordinates
float *IAP; // Initial Approach Point, in NED coordinates
float *LTP; // Last Turn Point, in NED coordinates
float *LAP; // Last Approach Point, in NED coordinates
float *RSP; // Runway Start Point, in NED coordinates
float *REP; // Runway End Point, in NED coordinates

float runway_heading_R;

float minimum_glidepath_angle_R; // Minimum glidepath angle in [rad]
float desired_glidepath_angle_R; // Desired glidepath angle in [rad]
float maximum_glidepath_angle_R; // Maximum glidepath angle in [rad]
float desired_flare_speed; // The desired speed at flare. This speed is maintained during most of the landing phase, especially on final approach

float minimum_pattern_altitude;
float maximum_pattern_altitude;
float desired_pattern_altitude;
float crosstrack_iap_to_ltp_error;

float crosstrack_ltp_to_lap_error;
float LT_center[3]; // Center of Last Turn, in NED coordinates
float arc_start_rad;
float LT_arc_length;
float LT_radius;
float start_x_track_error;
float end_x_track_error;
float end_slope;
float s_final;

float slope_1;
float IAP2LTP[2];

int8_t initialize_landing()
{
	AirfieldSettingsData airfieldSettings;
	AirfieldSettingsGet(&airfieldSettings);

	// Assign points;
	IAP = airfieldSettings.InitialApproachPoint;
	LTP = airfieldSettings.LastTurnPoint;
	LAP = airfieldSettings.LastApproachPoint;
	RSP = airfieldSettings.RunwayStartPoint;
	REP = airfieldSettings.RunwayEndPoint;

	// Create instances for all waypoints
	for (int i=WaypointGetNumInstances(); i<5; i++) {
		int32_t new_instance_id = WaypointCreateInstance();
		if (new_instance_id != i) {
			return -1;
		}
	}

	// Copy landing waypoints to waypoints UAVO
	for (int i=0; i<5; i++) {
		float *tmpPoint;
		uint32_t tmpWaypointCode = -1; // Negative numbers are universal error codes
		float tmpVelocity = 1.2f * stall_speed_clean; // Pattern airspeed is 20% faster than stall speed
		float tmpModeParameters = 0;
		uint8_t tmpMode = WAYPOINT_MODE_FLYVECTOR;

		switch (i) {
		case 0:
			tmpPoint = IAP;
			tmpWaypointCode = WAYPOINT_IAP;
			tmpVelocity = 1.4f * stall_speed_clean;
			break;
		case 1:
			tmpPoint = LTP;
			tmpWaypointCode = WAYPOINT_LTP;
			break;
		case 2:
			tmpPoint = LAP;
			tmpWaypointCode = WAYPOINT_LAP;
			tmpModeParameters = 50;
			tmpMode = WAYPOINT_MODE_FLYCIRCLELEFT;
			break;
		case 3:
			tmpPoint = RSP;
			tmpWaypointCode = WAYPOINT_RSP;
			break;
		case 4:
			tmpPoint = REP;
			tmpWaypointCode = WAYPOINT_REP;
			tmpVelocity = 0;
			break;
		}
		WaypointData tmpWaypoint;
		tmpWaypoint.Position[0] = tmpPoint[0];
		tmpWaypoint.Position[1] = tmpPoint[1];
		tmpWaypoint.Position[2] = tmpPoint[2];
		tmpWaypoint.WaypointCode = tmpWaypointCode;
		tmpWaypoint.Velocity = tmpVelocity;
		tmpWaypoint.ModeParameters = tmpModeParameters;
		tmpWaypoint.Mode = tmpMode;
		WaypointInstSet(i, &tmpWaypoint);
	}

//	from_lap_to_rsp;
	/* Final approach parameters, from LAP to RSP */
	// Get the desired approach glidepath angle
	desired_flare_speed = stall_speed_full_flaps * 1.3; // Standard industry practice is to approach at 130% of the stall speed
	desired_glidepath_angle_R = atan(DESIRED_VERTICAL_TOUCHDOWN_SPEED / desired_flare_speed);
	minimum_glidepath_angle_R = atan(MINIMUM_VERTICAL_TOUCHDOWN_SPEED / desired_flare_speed);
	maximum_glidepath_angle_R = atan(MAXIMUM_VERTICAL_TOUCHDOWN_SPEED / desired_flare_speed);

	// Saturate the desired angle
	desired_glidepath_angle_R = bound_min_max(desired_glidepath_angle_R, MINIMUM_DESIRED_GLIDEPATH_ANGLE_DEG, MAXIMUM_DESIRED_GLIDEPATH_ANGLE_DEG);
	minimum_glidepath_angle_R = bound_min_max(desired_glidepath_angle_R, MINIMUM_GLIDEPATH_ANGLE_DEG, MAXIMUM_GLIDEPATH_ANGLE_DEG);
	maximum_glidepath_angle_R = bound_min_max(desired_glidepath_angle_R, MINIMUM_GLIDEPATH_ANGLE_DEG, MAXIMUM_GLIDEPATH_ANGLE_DEG);

	// Create flight path corridor
//	from_iap_to_ltp;
	minimum_pattern_altitude = MINIMUM_PATTERN_HEIGHT + roundf(fminf(RSP[2], REP[2])); // Rounded in order to have human-friendly numbers. [cm] precision isn't important here
	maximum_pattern_altitude = MAXIMUM_PATTERN_HEIGHT + roundf(fminf(RSP[2], REP[2])); // Rounded in order to have human-friendly numbers. [cm] precision isn't important here
	desired_pattern_altitude = DESIRED_PATTERN_HEIGHT + roundf(fminf(RSP[2], REP[2])); // Rounded in order to have human-friendly numbers. [cm] precision isn't important here
	crosstrack_iap_to_ltp_error = CROSS_TRACK_PATTERN_ERROR;
	IAP2LTP[0] = LTP[0]-IAP[0];
	IAP2LTP[1] = LTP[1]-IAP[1];

//	from_ltp_to_lap;
	crosstrack_ltp_to_lap_error = CROSS_TRACK_PATTERN_ERROR;
	// Determine the center by finding the intersection of the two perpendicular lines from the LTP and LAP
	float LAP2RSP[2] = {RSP[0]-LAP[0], RSP[1]-LAP[1]};

	// line 1, find perpendicular slope:
	slope_1 = -IAP2LTP[0]/IAP2LTP[1];

	// line 2, find perpendicular slope:
	float slope_2 = -LAP2RSP[0]/LAP2RSP[1];

	float x_intersect = ((LAP[1]-LTP[0]) + slope_1*LTP[0] + slope_2*LAP[0]) / (slope_1-slope_2);
	float y_intersect = slope_1*(x_intersect-LTP[0]) + LTP[1];
	LT_center[0] = x_intersect;
	LT_center[1] = y_intersect;
	LT_center[2] = (LTP[2]+LAP[2])/2;
	LT_radius = sqrtf(powf(LAP2RSP[0]-LT_center[0], 2) + powf(LAP2RSP[1]-LT_center[1], 2));



	arc_start_rad = atan2f(LTP[1]-LT_center[1], LTP[0]-LT_center[0]);
	float arc_end_rad = atan2f(LAP[1]-LT_center[1], LAP[0]-LT_center[0]);
	LT_arc_length = arc_start_rad - arc_end_rad; // Arc length, can be negative
	start_x_track_error = crosstrack_iap_to_ltp_error;
	end_x_track_error = MIN(sqrtf(powf(LAP2RSP[0], 2) + powf(LAP2RSP[1], 2)) * tanf(MINIMUM_BEARING_ANGLE_DEG*DEG2RAD), start_x_track_error); // End threshold is either the initial threshold or the convergence cone width at the LAP

	// Define a cross-track error threshold which exponentially decreases from the permissible
	// x-track error at the LTP finishing at the x-track error at the LAP
	// We choose an "almost" period from a sine wave so that the first derivative of the
	// cross-track error along the entire trajectory is continuous, i.e. the trajectory is C1.

//	float start_slope = 0;
	end_slope = MINIMUM_BEARING_ANGLE_DEG;
	s_final = asinf(tanf(end_slope * RAD2DEG));




	// Get runway heading
	runway_heading_R = atan2(REP[1] - RSP[1], REP[0] - RSP[0]);

	return 0;
}

/**
 * @brief is_success_possible Initially, I'm just testing for airspeed and flight path that we want on landing
 * @return
 */
static bool is_success_possible(enum landing_fsm landing_fsm, enum airplane_configuration airplane_configuration)
{

	// 1) Are we within flight path cone?
	/*
	 * First we have to define the flight path corridor
	 */
	switch (landing_fsm) {
	case APPROACHING_IAP:
		// Nothing to be done when approaching the IAP. All altitudes and attitudes are good
		break;
	case APPROACHING_LTP:
	{
		//
		float a[2] = {LTP[0], LTP[1]};
		float p[2] = {NED[0], NED[1]};

		float p2a[2] = {a[1]-p[1], a[0]-p[0]};
		float bob[2];
		for (int i=0; i<2; i++) {
			bob[i] = p2a[i] - (IAP2LTP[0]*p2a[0] + IAP2LTP[1]*p2a[1]) * IAP2LTP[i];
		}

		float x_track_error2 = powf(bob[0], 2) + powf(bob[1], 2);

		if (x_track_error2 > crosstrack_ltp_to_lap_error*crosstrack_ltp_to_lap_error)
			return false;

		if (-NED[2] < minimum_pattern_altitude ||
				-NED[2] > maximum_pattern_altitude) {
			return false;
		}

		break;
	}
	case APPROACHING_LAP:
	{

		// Define the arc-length parameter `t`, which linearly grows from 0 to s_final. The position along the arc
		// is defined as the radial which the UAV is currently on.

		float arc_position = fabsf(atan2f(NED[1]-LT_center[1], NED[0]-LT_center[0]) - arc_start_rad);
		float t = (arc_position / LT_arc_length) * s_final;
		float x_track_threshold = start_x_track_error + (end_x_track_error - start_x_track_error) * (cosf(t)/cosf(s_final));

		float x_track_error = LT_radius - sqrtf(powf(NED[0]-LT_center[0], 2) + powf(NED[1]-LT_center[1], 2));

		if (fabsf(x_track_error) > x_track_threshold)
			return false;
/* Needs some more work
//		if (-NED[2] < minimum_pattern_altitude ||
//				-NED[2] > maximum_pattern_altitude) {
//			return false;
//		}
*/
		break;
	}
	case APPROACHING_RSP:
	{
		// Test if UAV is on approach path
		float runway_bearing_R = atan2f(NED[1] - RSP[1], NED[0] - RSP[0]);
		if (fabsf(circular_modulus_rad(runway_bearing_R-runway_heading_R)) > MINIMUM_BEARING_ANGLE_DEG) {
			// We're too far out of the runway convergence zone. Go round!
			return false;
		}

		// Test if UAV is on glide path
		float glidepath_intersection_angle = atan((NED[2] - RSP[2]) / sqrtf(powf(NED[0] - RSP[0], 2.0f) + powf(NED[1] - RSP[1], 2.0f)));
		if (glidepath_intersection_angle > maximum_glidepath_angle_R || glidepath_intersection_angle < minimum_glidepath_angle_R) {
			// This means we've busted our minimums or maximums. Go round!
			return false;
		}
	}
		break;
	case APPROACHING_REP:
		break;
	}

	// 2) Are we sufficiently faster than stall speed?
	switch (airplane_configuration) {
	case CLEAN:
		if (calibrated_airspeed < (STALL_SPEED_MARGIN_RATIO * stall_speed_clean)) {
			return false;
		}
		break;
	case FIRST_FLAPS:
		if (calibrated_airspeed < (STALL_SPEED_MARGIN_RATIO * stall_speed_first_flaps)) {
			return false;
		}
		break;
	case SECOND_FLAPS:
		if (calibrated_airspeed < (STALL_SPEED_MARGIN_RATIO * stall_speed_second_flaps)) {
			return false;
		}
		break;
	case FULL_FLAPS:
		if (calibrated_airspeed < (STALL_SPEED_MARGIN_RATIO * stall_speed_full_flaps)) {
			return false;
		}
		break;
	}


	// If we made it all the way to the end, then success is still possible.
	return true;
}


static bool is_mission_completed()
{
	return false;
}

/**
 * @}
 * @}
 */
