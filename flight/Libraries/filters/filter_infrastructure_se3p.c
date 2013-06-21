/**
 ******************************************************************************
 * @addtogroup TauLabsModules Tau Labs Modules
 * @{
 * @file       filter_infrastructure_se3p.c
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2013
 * @brief      Infrastructure for managing SE(3)+ filters
 *             because of the airspeed output this is slightly more than SE(3)
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

#include "filter_infrastructure_se3p.h"
#include "coordinate_conversions.h"
#include "physical_constants.h"

#include "accels.h"
#include "airspeedactual.h"
#include "attitudeactual.h"
#include "attitudesettings.h"
#include "baroairspeed.h"
#include "baroaltitude.h"
#include "flightstatus.h"
#include "gpsposition.h"
#include "gpsvelocity.h"
#include "gyros.h"
#include "gyrosbias.h"
#include "homelocation.h"
#include "sensorsettings.h"
#include "inssettings.h"
#include "insstate.h"
#include "magnetometer.h"
#include "nedposition.h"
#include "positionactual.h"
#include "stateestimation.h"
#include "velocityactual.h"

//! Maximum time to wait for data before setting an error
#define FAILSAFE_TIMEOUT_MS 10

//! Local pointer for the working data (should be moved into the instance)
static struct filter_infrastructure_se3p_data *se3p_data;

static int32_t getNED(GPSPositionData * gpsPosition, float * NED);

/**
 * Initialize SE(3)+ filter infrastructure
 * @param[out] data   the common part shared amongst SE(3)+ filters
 */
int32_t filter_infrastructure_se3p_init(struct filter_infrastructure_se3p_data **data)
{
	// Only create one instance of the common data.  This might not be what we want to
	// keep doing.  A easy (but more memory intense) way to run multiple filters would
	// be to make them all manage their own queues

	if (se3p_data == NULL) {
		se3p_data = (struct filter_infrastructure_se3p_data *) pvPortMalloc(sizeof(struct filter_infrastructure_se3p_data));
	}
	if (!se3p_data)
		return -1;

	(*data) = se3p_data;

	AttitudeActualInitialize();
	AttitudeSettingsInitialize();
	SensorSettingsInitialize();
	BaroAltitudeInitialize();
	BaroAirspedeInitialize();
	AirspeedActualInitialize();
	NEDPositionInitialize();
	PositionActualInitialize();
	VelocityActualInitialize();

	// Create the data queues
	se3p_data->gyroQueue = xQueueCreate(1, sizeof(UAVObjEvent));
	se3p_data->accelQueue = xQueueCreate(1, sizeof(UAVObjEvent));
	se3p_data->magQueue = xQueueCreate(2, sizeof(UAVObjEvent));
	se3p_data->baroQueue = xQueueCreate(1, sizeof(UAVObjEvent));
	se3p_data->airspeedQueue = xQueueCreate(1, sizeof(UAVObjEvent));
	se3p_data->gpsPosQueue = xQueueCreate(1, sizeof(UAVObjEvent));
	se3p_data->gpsVelQueue = xQueueCreate(1, sizeof(UAVObjEvent));

	return 0;
}

//! Connect the queues used for SE(3)+ filters
int32_t filter_infrastructure_se3p_start(uintptr_t id)
{
	if (GyrosHandle())
		GyrosConnectQueue(se3p_data->gyroQueue);
	if (AccelsHandle())
		AccelsConnectQueue(se3p_data->accelQueue);
	if (MagnetometerHandle())
		MagnetometerConnectQueue(se3p_data->magQueue);
	if (BaroAltitudeHandle())
		BaroAltitudeConnectQueue(se3p_data->baroQueue);
	if (BaroAirspeedHandle())
		BaroAirspeedConnectQueue(se3p_data->airspeedQueue);
	if (GPSPositionHandle())
		GPSPositionConnectQueue(se3p_data->gpsPosQueue);
	if (GPSVelocityHandle())
		GPSVelocityConnectQueue(se3p_data->gpsVelQueue);

	return 0;
}

/**
 * process_filter_generic Compute an update of an SE(3)+ filter
 * @param[in] driver The SE(3)+ filter driver
 * @param[in] dT the update time in seconds
 * @return 0 if succesfully updated or error code
 */
int32_t filter_infrastructure_se3p_process(struct filter_driver *upper_driver, uintptr_t id, float dt)
{
	// TODO: check error codes

	// Make sure we are safe to get the class specific driver
	if (!filter_interface_validate(upper_driver, id))
		return -1;
	struct filter_se3p *driver = &(upper_driver->sub_driver.driver_se3p);

	/* 1. fetch the data from queues and pass to filter                    */
	/* if we want to start running multiple instances of this filter class */
	/* simultaneously, then this step should be done once and then all     */
	/* filters should be processed with the same data                      */

	// Potential measurements
	float *gyros = NULL;
	float *accels = NULL;
	float *mag = NULL;
	float *pos = NULL;
	float *vel = NULL;
	float *baro = NULL;
	float *airspeed = NULL;

	// Check whether the measurements were updated and fetch if so
	UAVObjEvent ev;
	GyrosData gyrosData;
	AccelsData accelsData;
	MagnetometerData magData;
	BaroAltitudeData baroData;
	BaroAirspeedData airspeedData;
	GPSPositionData gpsPosition;
	GPSVelocityData gpsVelocity;
	float NED[3];

	if (xQueueReceive(se3p_data->gyroQueue, &ev, FAILSAFE_TIMEOUT_MS / portTICK_RATE_MS) == pdTRUE) {
		GyrosGet(&gyrosData);
		gyros = &gyrosData.x;
	}

	if (xQueueReceive(se3p_data->accelQueue, &ev, 1 / portTICK_RATE_MS) == pdTRUE) {
		AccelsGet(&accelsData);
		accels = &accelsData.x;
	}

	if (xQueueReceive(se3p_data->magQueue, &ev, 0 / portTICK_RATE_MS) == pdTRUE) {
		MagnetometerGet(&magData);
		mag = &magData.x;
	}

	if (xQueueReceive(se3p_data->baroQueue, &ev, 0 / portTICK_RATE_MS) == pdTRUE) {
		BaroAltitudeGet(&baroData);
		baro = &baroData.Altitude;
	}

	if (xQueueReceive(se3p_data->gpsPosQueue, &ev, 0 / portTICK_RATE_MS) == pdTRUE) {
		GPSPositionGet(&gpsPosition);
		getNED(&gpsPosition, NED);
		pos = NED;
	}

	if (xQueueReceive(se3p_data->gpsVelQueue, &ev, 0 / portTICK_RATE_MS) == pdTRUE) {
		GPSVelocityGet(&gpsVelocity);
		vel = &gpsVelocity.North;
	}

	if (xQueueReceive(se3p_data->airspeedQueue, &ev, 0 / portTICK_RATE_MS) == pdTRUE) {
		BaroAirspeedGet(&airspeedData);
		airspeed = &airspeedData.TrueAirspeed;
	}

	/* 2. compute update */
	driver->update_filter(id, gyros, accels, mag, pos, vel, baro, airspeed, dt);

	/* 3. get the state update from the filter */

	/* This section seems dangerous to me because it depends on the unique structure
		of the UAVOs. If any one of them is changed, even in datatype, then many filter
		functions will silently start returning bad results. Perhaps this is a spot where
		it would be appropriate to use the UAVO struct typedefs, even if we're not using
		the rest of the UAVO manager.*/
	float pos_state[3];
	float vel_state[3];
	float q_state[4];
	float gyro_bias_state[3];
	float airspeed_state[4];

	driver->get_state(id, pos_state, vel_state, q_state, gyro_bias_state, airspeed_state);

	// Store the data in UAVOs
	PositionActualData positionActual;
	positionActual.North = pos_state[0];
	positionActual.East  = pos_state[1];
	positionActual.Down  = pos_state[2];
	PositionActualSet(&positionActual);

	VelocityActualData velocityActual;
	velocityActual.North = vel_state[0];
	velocityActual.East  = vel_state[1];
	velocityActual.Down  = vel_state[2];
	VelocityActualSet(&velocityActual);

	AttitudeActualData attitudeActual;
	attitudeActual.q1 = q_state[0];
	attitudeActual.q2 = q_state[1];
	attitudeActual.q3 = q_state[2];
	attitudeActual.q4 = q_state[3];
	Quaternion2RPY(&attitudeActual.q1,&attitudeActual.Roll);
	AttitudeActualSet(&attitudeActual);

	AirspeedActualData = airspeedActual;
	airspeedActual.TrueAirspeed = airspeed_state[0];
	airspeedActual.CalibratedAirspeed = airspeed_state[1];
	airspeedActual.alpha = airspeed_state[2];
	airspeedActual.beta = airspeed_state[3];
	AirspeedActualSet(&airspeedActual);

	return 0;
}


/**
 * @brief Convert the GPS LLA position into NED coordinates
 * @note this method uses a taylor expansion around the home coordinates
 * to convert to NED which allows it to be done with all floating
 * calculations
 *
 * @TODO: refactor into coordinate convrsions
 *
 * @param[in] Current GPS coordinates
 * @param[out] NED frame coordinates
 * @returns 0 for success, -1 for failure
 */
static int32_t getNED(GPSPositionData * gpsPosition, float * NED)
{
	HomeLocationData homeLocation;
	HomeLocationGet(&homeLocation);

	float T[3];

	// Compute matrix to convert deltaLLA to NED
	float lat, alt;
	lat = homeLocation.Latitude / 10.0e6f * DEG2RAD;
	alt = homeLocation.Altitude;

	T[0] = alt+6.378137E6f;
	T[1] = cosf(lat)*(alt+6.378137E6f);
	T[2] = -1.0f;

	float dL[3] = {(gpsPosition->Latitude - homeLocation.Latitude) / 10.0e6f * DEG2RAD,
		(gpsPosition->Longitude - homeLocation.Longitude) / 10.0e6f * DEG2RAD,
		(gpsPosition->Altitude + gpsPosition->GeoidSeparation - homeLocation.Altitude)};

	NED[0] = T[0] * dL[0];
	NED[1] = T[1] * dL[1];
	NED[2] = T[2] * dL[2];

	return 0;
}

/**
 * @}
 */
