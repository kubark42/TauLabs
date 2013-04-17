/**
 ******************************************************************************
 *
 * @file       fixedwingpathfollower.c
 * @author     Tau Labs, http://www.taulabs.org Copyright (C) 2013.
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


#include "openpilot.h"

#include "airspeedactual.h"
#include "attitudeactual.h"
#include "fixedwingairspeeds.h"
#include "fixedwingpathfollowersettings.h"
#include "flightstatus.h"
#include "manualcontrol.h"
#include "modulesettings.h"
#include "positionactual.h"
#include "velocityactual.h"
#include "homelocation.h"
#include "stabilizationdesired.h" // object that will be updated by the module
#include "systemsettings.h"

#include "follower_guidance.h"
#include "follwer_fsm.h"
#include "CoordinateConversions.h"

#include "pathfollowerstatus.h"
#include "pathplannersettings.h"
#include "pathsegmentdescriptor.h"


// Private constants
#define MAX_QUEUE_SIZE 4
#define STACK_SIZE_BYTES 750
#define TASK_PRIORITY (tskIDLE_PRIORITY+2)

// Private types

// Private variables
static bool module_enabled = false;
static xTaskHandle pathfollowerTaskHandle;

// Private functions
static void pathfollowerTask(void *parameters);

/**
 * Initialise the module, called on startup
 * \returns 0 on success or -1 if initialisation failed
 */
int32_t FixedWingPathFollowerStart()
{
	if (module_enabled) {
//		FlightStatusConnectCallback(SettingsUpdatedCb);
		FixedWingPathFollowerSettingsConnectCallback(GuidanceSettingsUpdatedCb);
		FixedWingAirspeedsConnectCallback(GuidanceSettingsUpdatedCb);

		// Start main task
		xTaskCreate(pathfollowerTask, (signed char *)"PathFollower", STACK_SIZE_BYTES/4, NULL, TASK_PRIORITY, &pathfollowerTaskHandle);
		TaskMonitorAdd(TASKINFO_RUNNING_PATHFOLLOWER, pathfollowerTaskHandle);
	}

	return 0;
}

/**
 * Initialise the module, called on startup
 * \returns 0 on success or -1 if initialisation failed
 */
int32_t FixedWingPathFollowerInitialize()
{
#ifdef MODULE_FixedWingPathFollower_BUILTIN
	module_enabled = true;
#else
	ModuleSettingsInitialize();
	uint8_t module_state[MODULESETTINGS_ADMINSTATE_NUMELEM];
	ModuleSettingsAdminStateGet(module_state);
	if (module_state[MODULESETTINGS_ADMINSTATE_FIXEDWINGPATHFOLLOWER] == MODULESETTINGS_ADMINSTATE_ENABLED) {
		module_enabled = true;
	} else {
		module_enabled = false;
	}
#endif

	SystemSettingsInitialize();
	uint8_t systemSettingsAirframeType;
	SystemSettingsAirframeTypeGet(&systemSettingsAirframeType);
	if ( (systemSettingsAirframeType != SYSTEMSETTINGS_AIRFRAMETYPE_FIXEDWING) &&
		(systemSettingsAirframeType != SYSTEMSETTINGS_AIRFRAMETYPE_FIXEDWINGELEVON) &&
		(systemSettingsAirframeType != SYSTEMSETTINGS_AIRFRAMETYPE_FIXEDWINGVTAIL) ) //TODO: Decide for custom mixers
	{
		module_enabled = false;
	}
	
	if (module_enabled) {
		// Initialize UAVOs
		FixedWingPathFollowerSettingsInitialize();
		FixedWingAirspeedsInitialize();
		PathSegmentDescriptorInitialize();
		AirspeedActualInitialize();

		// Allocate memory
		allocateGuidanceMemory();
		allocateCommutatorMemory();
	}

	return 0;
}
MODULE_INITCALL(FixedWingPathFollowerInitialize, FixedWingPathFollowerStart);

/**
 * Module thread, should not return.
 */

static void pathfollowerTask(void *parameters)
{
	portTickType lastUpdateTime;
	FixedWingPathFollowerSettingsData fixedwingpathfollowerSettings;

	// Force update of all the settings
	GuidanceSettingsUpdatedCb(NULL);
	CommutatorSettingsUpdatedCb(NULL);


	// Main task loop
	lastUpdateTime = xTaskGetTickCount();
	while (1) {
		FixedWingPathFollowerSettingsGet(&fixedwingpathfollowerSettings);  //IT WOULD BE NICE NOT TO DO THIS EVERY LOOP.
		
		// Wait.
		vTaskDelayUntil(&lastUpdateTime, fixedwingpathfollowerSettings.UpdatePeriod * portTICK_RATE_MS);
		
		// Check if path follower has reached swiching locus.

		// Compute path follower commands
		switch(0) {
			case 1:
				updateDesiredAttitude(&fixedwingpathfollowerSettings);
				break;
			default:
				// Be cleaner and reset integrals
				zeroGuidanceIntegral();
				break;
		}
	}
}
