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

#include "missiondirectorsettings.h"
#include "missiondirectorstatus.h"
#include "missiondirectoruserprogram.h"
#include "modulesettings.h"


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
/*
	//Get the FFT window size
	uint16_t fft_window_size; // Make a local copy in order to check settings before allocating memory
	uint8_t num_upscale_bits;
	VibrationAnalysisSettingsFFTWindowSizeOptions fft_window_size_enum;
	VibrationAnalysisSettingsFFTWindowSizeGet(&fft_window_size_enum);
	switch (fft_window_size_enum) {
		case VIBRATIONANALYSISSETTINGS_FFTWINDOWSIZE_16:
			fft_window_size = 16;
			num_upscale_bits = 4;
			break;
		case VIBRATIONANALYSISSETTINGS_FFTWINDOWSIZE_64:
			fft_window_size = 64;
			num_upscale_bits = 6;
			break;
		case VIBRATIONANALYSISSETTINGS_FFTWINDOWSIZE_256:
			fft_window_size = 256;
			num_upscale_bits = 8;
			break;
		case VIBRATIONANALYSISSETTINGS_FFTWINDOWSIZE_1024:
			fft_window_size = 1024;
			num_upscale_bits = 10;
			break;
		default:
			//This represents a serious configuration error. Do not start module.
			module_enabled = false;
			return -1;
			break;
	}
	

	// Create instances for vibration analysis. Start from i=1 because the first instance is generated
	// by VibrationAnalysisOutputInitialize(). Generate three times the length because there are three
	// vectors. Generate half the length because the FFT output is symmetric about the mid-frequency, 
	// so there's no point in using memory additional memory.
	for (int i=1; i < (fft_window_size>>1); i++) {
		uint16_t ret = VibrationAnalysisOutputCreateInstance();
		if (ret == 0) {
			// This fails when it's a metaobject. Not a very helpful test.
			module_enabled = false;
			return -1;
		}
	}
	
	if (VibrationAnalysisOutputGetNumInstances() != (fft_window_size>>1)){
		// This is a more useful test for failure.
		module_enabled = false;
		return -1;
	}
	
	
	// Allocate and initialize the static data storage only if module is enabled
	vtd = (struct VibrationAnalysis_data *) pvPortMalloc(sizeof(struct VibrationAnalysis_data));
	if (vtd == NULL) {
		module_enabled = false;
		return -1;
	}
	
	// make sure that all struct values are zeroed...
	memset(vtd, 0, sizeof(struct VibrationAnalysis_data));
	//... except for Z axis static bias
	vtd->accels_static_bias_z=-GRAVITY; // [See note in definition of VibrationAnalysis_data structure]

	// Now place the fft window size and number of upscale bits variables into the buffer
	vtd->fft_window_size = fft_window_size;
	vtd->num_upscale_bits = num_upscale_bits;
	
	// Allocate ouput vector
	vtd->fft_output = (int16_t *) pvPortMalloc(fft_window_size*2*sizeof(typeof(*(vtd->fft_output))));
	if (vtd->fft_output == NULL) {
		module_enabled = false; //Check if allocation succeeded
		return -1;
	}
	
	//Create the buffers. They are in Q15 format.
	vtd->accel_buffer_complex_x_q15 = (int16_t *) pvPortMalloc(fft_window_size*2*sizeof(typeof(*vtd->accel_buffer_complex_x_q15)));
	if (vtd->accel_buffer_complex_x_q15 == NULL) {
		module_enabled = false; //Check if allocation succeeded
		return -1;
	}
	vtd->accel_buffer_complex_y_q15 = (int16_t *) pvPortMalloc(fft_window_size*2*sizeof(typeof(*vtd->accel_buffer_complex_y_q15)));
	if (vtd->accel_buffer_complex_y_q15 == NULL) {
		module_enabled = false; //Check if allocation succeeded
		return -1;
	}
	vtd->accel_buffer_complex_z_q15 = (int16_t *) pvPortMalloc(fft_window_size*2*sizeof(typeof(*vtd->accel_buffer_complex_z_q15)));
	if (vtd->accel_buffer_complex_z_q15 == NULL) {
		module_enabled = false; //Check if allocation succeeded
		return -1;
	}
*/
	
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
	MissionDirectorSettingsInitialize();
	MissionDirectorStatusInitialize();
	MissionDirectorUserProgramInitialize();

	return 0;
	
}

MODULE_INITCALL(MissionDirectorInitialize, MissionDirectorStart);


static void MissionDirectorTask(void *parameters)
{
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

#define MINIMUM_GLIDEPATH_ANGLE ((2.0f) * DEG2RAD) // Never allow a glidepath angle below this threshold. PAPI are generally adjusted to show all red below 2 degrees.
#define MAXIMUM_GLIDEPATH_ANGLE ((20.0f) * DEG2RAD) // Never allow a glidepath angle above this threshold
#define MINIMUM_DESIRED_GLIDEPATH_ANGLE ((3.0f) * DEG2RAD) // Never allow a desired glidepath angle below this threshold. 3 degrees is standard approach angle at most airports
#define MAXIMUM_DESIRED_GLIDEPATH_ANGLE ((15.0f) * DEG2RAD) // Never allow a desired glidepath angle above this threshold.
#define MINIMUM_BEARING_ANGLE (4 * DEG2RAD) // We will allow +-2 deg lateral deviation from runway

#define stall_speed_clean 6
#define stall_speed_first_flaps 5
#define stall_speed_second_flaps 5
#define stall_speed_full_flaps 5
#define STALL_SPEED_MARGIN_RATIO (1 + 0.04f) // Stall margin ratio

#define MINIMUM_PATTERN_HEIGHT 30 //Minimum height in [m]. FIXME: THIS SHOULD NOT BE A MAGIC NUMBER. IT SHOULD BE A FUNCTION OF AIRPLANE SPEED AND GLIDE RATIO
#define MAXIMUM_PATTERN_HEIGHT 50 //Minimum height in [m]. FIXME: THIS SHOULD NOT BE A MAGIC NUMBER. IT SHOULD BE A FUNCTION OF AIRPLANE SPEED AND GLIDE RATIO
#define DESIRED_PATTERN_HEIGHT 70 //Minimum height in [m]. FIXME: THIS SHOULD NOT BE A MAGIC NUMBER. IT SHOULD BE A FUNCTION OF AIRPLANE SPEED AND GLIDE RATIO
#define CROSS_TRACK_PATTERN_ERROR 20; // Allowable error between IAP and LTP, in [m]. FIXME: THIS SHOULD NOT BE A MAGIC NUMBER, BUT INSTEAD BE A FUNCTION OF BANK ANGLE AND AIRSPEED. THE IDEA IS TO ALLOW THE AIRPLANE TO APPROACH THE IAP FROM THE COMPLETELY OPPOSITE DIRECTION TO THE LTP AND YET STILL BE ABLE TO DO A BANKING MANEUVER FROM THE IAP TO THE LTP

float RSP[3]; // Runway Start Point, in NED coordinates
float REP[3]; // Runway End Point, in NED coordinates
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

void initialize_landing()
{
	/* Final approach parameters, from LAP to RSP */
	// Get the desired approach glidepath angle
	desired_flare_speed = stall_speed_full_flaps * 1.3; // Standard industry practice is to approach at 130% of the stall speed
	desired_glidepath_angle_R = atan(DESIRED_VERTICAL_TOUCHDOWN_SPEED / desired_flare_speed);
	minimum_glidepath_angle_R = atan(MINIMUM_VERTICAL_TOUCHDOWN_SPEED / desired_flare_speed);
	maximum_glidepath_angle_R = atan(MAXIMUM_VERTICAL_TOUCHDOWN_SPEED / desired_flare_speed);

	// Saturate the desired angle
	desired_glidepath_angle_R = bound_min_max(desired_glidepath_angle_R, MINIMUM_DESIRED_GLIDEPATH_ANGLE, MAXIMUM_DESIRED_GLIDEPATH_ANGLE);
	minimum_glidepath_angle_R = bound_min_max(desired_glidepath_angle_R, MINIMUM_GLIDEPATH_ANGLE, MAXIMUM_GLIDEPATH_ANGLE);
	maximum_glidepath_angle_R = bound_min_max(desired_glidepath_angle_R, MINIMUM_GLIDEPATH_ANGLE, MAXIMUM_GLIDEPATH_ANGLE);

	// Create flight path corridor
//	from_iap_to_ltp;
	minimum_pattern_altitude = MINIMUM_PATTERN_HEIGHT + roundf(fminf(RSP[2], REP[2])); // Rounded in order to have human-friendly numbers. [cm] precision isn't important here
	maximum_pattern_altitude = MAXIMUM_PATTERN_HEIGHT + roundf(fminf(RSP[2], REP[2])); // Rounded in order to have human-friendly numbers. [cm] precision isn't important here
	desired_pattern_altitude = DESIRED_PATTERN_HEIGHT + roundf(fminf(RSP[2], REP[2])); // Rounded in order to have human-friendly numbers. [cm] precision isn't important here
	crosstrack_iap_to_ltp_error = CROSS_TRACK_PATTERN_ERROR;

//	from_ltp_to_ltp;
	crosstrack_ltp_to_lap_error = CROSS_TRACK_PATTERN_ERROR;
//	from_lap_to_rsp;


	// Get runway heading
	runway_heading_R = atan2(REP[1] - RSP[1], REP[0] - RSP[0]);


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
		// Nothing to be done when approaching the IAP. All altitudes are good
		break;
	case APPROACHING_LTP:
		//
		break;
	case APPROACHING_LAP:
		break;
	case APPROACHING_RSP:
	{
		// Test if UAV is on approach path
		float runway_bearing_R = atan2f(NED[1] - RSP[1], NED[0] - RSP[0]);
		if (fabsf(circular_modulus_rad(runway_bearing_R-runway_heading_R)) > MINIMUM_BEARING_ANGLE) {
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
