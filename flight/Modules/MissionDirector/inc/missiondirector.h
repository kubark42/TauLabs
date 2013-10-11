/**
 ******************************************************************************
 * @addtogroup TauLabsModules Tau Labs Modules
 * @{
 * @addtogroup MissionDirectorModule Mission director module
 * @{
 *
 * @file       missiondirector.h
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

#ifndef MISSIONDIRECTOR_H
#define MISSIONDIRECTOR_H

#include "openpilot.h"

typedef struct {
	float gyro[3];
    float accel[3];
    float mag[3];
    float attitude[4];
    float pos[3];
    float vel[3];
    float airspeed[2];
	float baro[1];
//	    sensorUpdates updated;
} stateEstimation;

typedef struct missionProgramStruct {
    int32_t (*init)(struct missionProgramStruct *self);
	int32_t (*completion_test)(struct missionProgramStruct *self, stateEstimation *state);
	int32_t (*abort_test)(struct missionProgramStruct *self, stateEstimation *state);
    void *localdata;
} missionProgram;

//struct missionPipelineStruct;

typedef struct missionPipelineStruct {
     missionProgram *mission;
	 struct missionPipelineStruct *on_success;
	 struct missionPipelineStruct *on_abort;
} missionPipeline;

int32_t missionFixedwingLandingInitialize(missionProgram *handle);

/*
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

typedef struct {
	float gyro[3];
    float accel[3];
    float mag[3];
    float attitude[4];
    float pos[3];
    float vel[3];
    float airspeed[2];
	float baro[1];
//	    sensorUpdates updated;
} stateEstimation;

typedef struct missionProgramStruct {
    int32_t (*init)(struct missionProgramStruct *self);
	int32_t (*mission)(struct missionProgramStruct *self, stateEstimation *state);
    void *localdata;
} missionProgram;

//struct missionPipelineStruct;

typedef struct missionPipelineStruct {
     missionProgram *mission;
	 struct missionPipelineStruct *on_success;
	 struct missionPipelineStruct *on_failure;
} missionPipeline;


static missionProgram autonomous_landing;
static missionProgram go_round;
static missionProgram shut_down;

static missionPipeline shut_down_queue;
static missionPipeline go_round_queue;

static missionPipeline landing_queue = {
	.mission = &autonomous_landing,
	.on_success = &shut_down_queue,
	.on_failure = &go_round_queue
};

static missionPipeline shut_down_queue = {
	.mission = &shut_down,
	.on_success = NULL,
	.on_failure = NULL
};

static missionPipeline go_round_queue = {
	.mission = &go_round,
	.on_success = &landing_queue,
	.on_failure = NULL
};


//&(missionPipeline) {
//		.mission = &magFilter,
//        .on_success = NULL,
//		.on_failure = NULL,
//        }
//	}
//static const missionPipeline *auto_landing_mission = &(missionPipeline) {
//    .mission = NULL,
//    .on_success = NULL,
//	.on_failure = NULL,
//};


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
*/

/**
 * @}
 * @}
 */
#endif // MISSIONDIRECTOR_H
