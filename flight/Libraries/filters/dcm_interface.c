/**
 ******************************************************************************
 * @addtogroup TauLabsModules Tau Labs Modules
 * @{
 * @file       dcm_interface.c
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2013
 * @brief      Interface from the SE(3)+ infrastructure to DCM
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

#include "filter_interface.h"
#include "filter_infrastructure_se3p.h"

#include "stdint.h"
#include "stdbool.h"

static int32_t dcm_interface_init(uintptr_t *id);
static int32_t dcm_interface_reset(uintptr_t id);
static int32_t dcm_interface_update(uintptr_t id, float gyros[3], float accels[3], 
		float mag[3], float pos[3], float vel[3], float baro[1],
		float airspeed[1], float dt);
static int32_t dcm_interface_get_state(uintptr_t id, float pos[3], float vel[3],
		float attitude[4], float gyro_bias[3], float airspeed[1]);

struct filter_driver dcm_filter_driver = {
	.class = FILTER_CLASS_SE3P,

	// this will initialize the SE(3)+ infrastrcture too
	.init = dcm_interface_init,

	// connects the SE(3)+ queues
	.start = filter_infrastructure_se3p_start,
	.reset = dcm_interface_reset,
	.process = filter_infrastructure_se3p_process,
	.sub_driver = {
		.driver_se3p = {
			.update_filter = dcm_interface_update,
			.get_state = dcm_interface_get_state,
			.magic = FILTER_SE3P_MAGIC,
		}
	}
};


enum dcm_interface_magic {
	DCM_INTERFACE_MAGIC = 0x164BBE6C
};

struct dcm_interface_data {
	struct filter_infrastructure_se3_data *se3p_data;
	enum dcm_interface_magic magic;
};

static struct dcm_interface_data * dcm_interface_alloc()
{
	// TODO
	return NULL;
}

/**
 * Initialize this DCM filter and the SE(3)+ infrastructure
 * @param[out]  id   the handle for this filter instance
 * @return 0 if successful, -1 if not
 */
static int32_t dcm_interface_init(uintptr_t *id)
{
	// Allocate the data structure
	struct dcm_interface_data * dcm_interface_data = dcm_interface_alloc();
	if (dcm_interface_data == NULL)
		return -1;

	// Initialize the infrastructure
	if (filter_infrastructure_se3_init(&dcm_interface_data->se3p_data) != 0)
		return -2;
	
	// Return the handle
	(*id) = (uintptr_t) dcm_interface_data;

	return 0;
}


/********* formatting sensor data to the core math code goes below here *********/



/**
 * Reset the filter state to default
 * @param[in]  id        the filter handle to reset
 */
static int32_t dcm_interface_reset(uintptr_t id)
{
	//Change gyro calibration parameters...
	if ((xTaskGetTickCount() > 1000) && (xTaskGetTickCount() < 7000)) {
		//...during first 7 seconds or so...
		// For first 7 seconds use accels to get gyro bias
		glblAtt->accelKp = 1;
		glblAtt->accelKi = 0.9;
		glblAtt->yawBiasRate = 0.23;
		init = 0;

		//Force to use the CCC, because of the way it calibrates
		attitudeSettings.FilterChoice = ATTITUDESETTINGS_FILTERCHOICE_CCC;
	} else if (glblAtt->zero_during_arming && (flightStatus.Armed == FLIGHTSTATUS_ARMED_ARMING)) {
		//...during arming...
		glblAtt->accelKp = 1;
		glblAtt->accelKi = 0.9;
		glblAtt->yawBiasRate = 0.23;
		init = 0;

		//Force to use the CCC, because of the way it calibrates
		attitudeSettings.FilterChoice = ATTITUDESETTINGS_FILTERCHOICE_CCC;
	} else if (init == 0) {	//...once fully armed.
		// Reload settings (all the rates)
		AttitudeSettingsAccelKiGet(&glblAtt->accelKi);
		AttitudeSettingsAccelKpGet(&glblAtt->accelKp);
		AttitudeSettingsYawBiasRateGet(&glblAtt->yawBiasRate);

		attitudeSettings.FilterChoice = originalFilter;

		init = 1;
	}


	return 0;
}

/**
 * get_sensors Update the filter one time step
 * @param[in] id         the running filter handle
 * @param[in] gyros      new gyro data [deg/s] or NULL
 * @param[in] accels     new accel data [m/s^2] or NULL
 * @param[in] mag        new mag data [mGau] or NULL
 * @param[in] pos        new position measurement in NED [m] or NULL
 * @param[in] vel        new velocity meansurement in NED [m/s] or NULL
 * @param[in] baro       new baro data [m] or NULL
 * @param[in] airspeed   estimate of the airspeed
 * @param[in] dt         time step [s]
 * @returns 0 if sufficient data to run update
 */
static int32_t dcm_interface_update(uintptr_t id, float gyros[3], float accels[3], 
		float mag[3], float pos[3], float vel[3], float baro[1],
		float airspeed[1], float dt)
{
	if (vel != NULL) {
		if (1 /*gpsStatus == GPSPOSITION_STATUS_FIX3D*/) {
			//Estimate velocity in NED frame
			velocityActualData.North = (1 - alphaVelNorthEast) * velocityActualData.North +
				alphaVelNorthEast * gpsVelocityData->North;
			velocityActualData.East  = (1 - alphaVelNorthEast) * velocityActualData.East +
				alphaVelNorthEast * gpsVelocityData->East;
			velocityActualData.Down  = (1 - alphaVelDown) * velocityActualData.Down +
				alphaVelDown * gpsVelocityData->Down;

			//Estimate airspeed from GPS data
			//http://www.engineeringtoolbox.com/air-altitude-pressure-d_462.html
			float staticPressure =
				homeLocation.SeaLevelPressure * powf(1.0f - 2.2555e-5f *
					(homeLocation.Altitude - positionActualData. Down), 5.25588f);

			// Convert from millibar to Pa
			float staticAirDensity = staticPressure * 100 * 0.003483613507536f /
				(homeLocation.GroundTemperature + CELSIUS2KELVIN);

			gps_airspeed_update(&gpsVelocityData, staticAirDensity);
		}

	}


	return 0;
}

/**
 * get_state Retrieve the state from the SE(3)+ filter
 * any param can be null indicating it is not being fetched
 * @param[in]  id        the running filter handle
 * @param[out] pos       the updated position in NED [m]
 * @param[out] vel       the updated velocity in NED [m/s]
 * @param[out] attitude  the updated attitude quaternion
 * @param[out] gyro_bias the update gyro bias [deg/s]
 * @param[out] airspeed  estimate of the airspeed
 */
static int32_t dcm_interface_get_state(uintptr_t id, float pos[3], float vel[3],
		float attitude[4], float gyro_bias[3], float airspeed[1])
{
	return 0;
}

/**
 * @}
 */

