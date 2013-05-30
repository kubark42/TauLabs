/**
 ******************************************************************************
 * @addtogroup OpenPilotModules OpenPilot Modules
 * @{
 * @addtogroup GeoFenceModule Geo-fence Module
 * @brief Measures geo-fence position
 * Updates the ??? object
 * @{
 *
 * @file       geofence.c
 * @author     Tau Labs, http://taulabs.org Copyright (C) 2013.
 * @brief      Module to monitor position with respect to geo-fence and set alarms appropriately.
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
 * Output object: ???
 *
 * This module will monitor position with respect to geo-fence and set alarms appropriately.
 *
 * UAVObjects are automatically generated by the UAVObjectGenerator from
 * the object definition XML file.
 *
 * Modules have no API, all communication to other modules is done through UAVObjects.
 * However modules may use the API exposed by shared libraries.
 * See the OpenPilot wiki for more details.
 * http://www.openpilot.org/OpenPilot_Application_Architecture
 *
 */

#include "openpilot.h"
#include "coordinate_conversions.h"

#include "modulesettings.h"
#include "geofencevertices.h"
#include "geofencefaces.h"
#include "positionactual.h"
#include "velocityactual.h"

//
// Configuration
//
#define STACK_SIZE_BYTES   1500
#define SAMPLE_PERIOD_MS   2000
#define TASK_PRIORITY      (tskIDLE_PRIORITY + 1)

// Private functions
static void geofenceTask(void *parameters);
static bool test_line_triange_intersection(PositionActualData *positionActual, float lineCA[3], float lineBA[3], float vertexA[3], float t);
static void set_manual_control_error(SystemAlarmsGeoFenceOptions error_code);
static bool check_enabled();

// Private types

// Private variables
static bool geofence_enabled = false;
static xTaskHandle geofenceTaskHandle;

/**
 * Initialise the geofence module
 * \return -1 if initialisation failed
 * \return 0 on success
 */
int32_t GeoFenceStart(void)
{
	if (geofence_enabled) {
		// Start geofence task
		xTaskCreate(geofenceTask, (signed char *)"GeoFence", STACK_SIZE_BYTES/4, NULL, TASK_PRIORITY, &geofenceTaskHandle);
		TaskMonitorAdd(TASKINFO_RUNNING_GEOFENCE, geofenceTaskHandle);
		return 0;
	}
	return -1;
}

/**
 * Initialise the geofence module
 * \return -1 if initialisation failed
 * \return 0 on success
 */
int32_t GeoFenceInitialize(void)
{
	geofence_enabled = check_enabled();

	if (geofence_enabled) {
		GeoFenceFacesInitialize();
		GeoFenceVerticesInitialize();
		return 0;
	}
	
	return -1;
}

MODULE_INITCALL(GeoFenceInitialize, GeoFenceStart);

// ****************
/**
 * Main geo-fence task. It does not return.
 */
static void geofenceTask(void *parameters)
{
	while(1) {
		vTaskDelay(SAMPLE_PERIOD_MS);

		uint8_t sumCrossingsNow=0; //<-- This could just be a bool that is toggled each time there's a crossing
		uint8_t sumCrossingsSoon=0; //<-- This could just be a bool that is toggled each time there's a crossing
		
		uint16_t num_vertices=UAVObjGetNumInstances(GeoFenceVerticesHandle());
		uint16_t num_faces=UAVObjGetNumInstances(GeoFenceFacesHandle());

		if (num_vertices < 4) // The fewest number of vertices requiered to make a 3D volume is 4.
			continue;
		if (num_faces < 4) // The fewest number of faces requiered to make a 3D volume is 4.
			continue;

		VelocityActualData velocityActualData;
		PositionActualData positionActual_now;
		PositionActualData positionActual_soon;

		//Load UAVOs
		PositionActualGet(&positionActual_now);
		VelocityActualGet(&velocityActualData);

		//Predict UAVO future location
		float forwardTime=3; //Predict 3 seconds into the future// TODO: should perhaps not be hardcoded
		positionActual_soon.North=positionActual_now.North+velocityActualData.North*forwardTime;
		positionActual_soon.East=positionActual_now.East+velocityActualData.East*forwardTime;
		positionActual_soon.Down=positionActual_now.Down+velocityActualData.Down*forwardTime;
		
		//TODO: It's silly to recreate the normal vector and offset each loop. The equation for the plane should
		// be computed only when the vertices are changed. However, that is much less RAM efficient.
		for (uint16_t i=0; i<num_vertices; i++) {
			GeoFenceVerticesData geofenceVerticesData;
			GeoFenceFacesData geofenceFacesData;
			
			//Get the face of interest
			GeoFenceFacesInstGet(i, &geofenceFacesData);
			
			//Get the three face vertices. Order is important!
			GeoFenceVerticesInstGet(geofenceFacesData.Vertices[0], &geofenceVerticesData);
			float vertexA[3]={geofenceVerticesData.Vertex[0], geofenceVerticesData.Vertex[1], geofenceVerticesData.Vertex[2]};
			GeoFenceVerticesInstGet(geofenceFacesData.Vertices[1], &geofenceVerticesData);
			float vertexB[3]={geofenceVerticesData.Vertex[0], geofenceVerticesData.Vertex[1], geofenceVerticesData.Vertex[2]};
			GeoFenceVerticesInstGet(geofenceFacesData.Vertices[2], &geofenceVerticesData);
			float vertexC[3]={geofenceVerticesData.Vertex[0], geofenceVerticesData.Vertex[1], geofenceVerticesData.Vertex[2]};
			
			
			//From: http://adrianboeing.blogspot.com/2010/02/intersection-of-convex-hull-with-line.html
			float lineBA[3]={vertexB[0]-vertexA[0], vertexB[1]-vertexA[1], vertexB[2]-vertexA[2]};
			float lineCA[3]={vertexC[0]-vertexA[0], vertexC[1]-vertexA[1], vertexC[2]-vertexA[2]};
			float norm[3];
			CrossProduct(lineBA, lineCA, norm);
			
			float d=DotProduct(norm,vertexA);
			/* This is what the ray looks like, but we have optimized it out of the algorithm so it does not
				explicitly appear in the calculations:
					float ray={positionActual_soon[0]+1, positionActual_soon[1], positionActual_soon[2]};
			*/
			
			//Solve for line parameter t=d-n*x/(n*ray-x). However, take shortcut because we know that ray-x is [1;0;0]
			if (fabs(norm[0]<1e-4)) { //If the value of (n*ray-x) is too small, then the ray is parallel to the plane
				continue;
			}
			
			float NED_soon[3]={positionActual_soon.North, positionActual_soon.East, positionActual_soon.Down};
			float NED_now[3] ={positionActual_now.North,  positionActual_now.East,  positionActual_now.Down};
			float t_soon=(d-DotProduct(norm,NED_soon))/norm[0];
			float t_now= (d-DotProduct(norm,NED_now) )/norm[0];
			
			//Only use the positive side of the ray, as any faces behind it do not intersect with the ray
			if (t_soon<0 && t_now < 0){
				continue;
			}
			
			//Test if ray falls inside triangle. No need to independently test for both t_soon and t_now ray, as they are identical rays
			bool 	inside=test_line_triange_intersection(&positionActual_soon, lineCA, lineBA, vertexA, t_soon);
			
			if (inside && t_soon > 0) {
				sumCrossingsSoon++;
			}
			if (inside /*&& t_now > 0*/) { //No need to test t_now, as it's already implicit in the test several lines higher
				sumCrossingsNow++;
			}
		}
	
		//Test if we have crossed the geo-fence
		if (sumCrossingsSoon % 2) {	//If there are an odd number of faces crossed, then the UAV is and will be inside the polyhedron. 
			set_manual_control_error(SYSTEMALARMS_GEOFENCE_LEAVINGBOUNDARY);
		}
		else if (sumCrossingsNow % 2) {	//If there are an odd number of faces crossed, then the UAV is inside the polyhedron, but on its current course will soon leave the polygon. 
			set_manual_control_error(SYSTEMALARMS_GEOFENCE_LEFTBOUNDARY);
		}
		else{ //If there are an even number, then the UAV is outside the polyhedron.
			set_manual_control_error(SYSTEMALARMS_GEOFENCE_NONE);
		}
		
	}
	
}


/**
 * Test if a line intersects a triangle
 * From: http://www.blackpawn.com/texts/pointinpoly/default.html
 */

static bool test_line_triange_intersection(PositionActualData *positionActual, float lineCA[3], float lineBA[3], float vertexA[3], float t)
{
	float P[3]={positionActual->North+t, positionActual->East, positionActual->Down};
	
	float linePA[3]={P[0]-vertexA[0], P[1]-vertexA[1], P[2]-vertexA[2]};
	
	float dot00 = DotProduct(lineCA, lineCA);
	float dot01 = DotProduct(lineCA, lineBA);
	float dot02 = DotProduct(lineCA, linePA);
	float dot11 = DotProduct(lineBA, lineBA);
	float dot12 = DotProduct(lineBA, linePA);
	
	// Compute barycentric coordinates
	float invDenom = 1.0f / (dot00 * dot11 - dot01 * dot01);
	float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
	float v = (dot00 * dot12 - dot01 * dot02) * invDenom;
	
	//Test if point is inside triangle
	bool 	inside=(u >= 0) && (v >= 0) && (u + v < 1.0f);
	
	return inside;
}


static bool check_enabled()
{
	ModuleSettingsInitialize();
	uint8_t module_state[MODULESETTINGS_ADMINSTATE_NUMELEM];

	ModuleSettingsAdminStateGet(module_state);

	bool gps_module_enabled = false; 	//Geo-fence only works if we have GPS or groundtruth
	bool groundtruth_available = false; //Geo-fence only works if we have GPS or groundtruth
	bool pathfollower_module_enabled = false; // Geo-fence only works if we can autonomously steer the vehicle
	bool geofence_module_enabled = false;

#ifdef MODULE_GPS_BUILTIN
	gps_module_enabled = true;
#else
	if (module_state[MODULESETTINGS_ADMINSTATE_GPS] == MODULESETTINGS_ADMINSTATE_ENABLED)
		gps_module_enabled = true;
#endif

#ifdef MODULE_PATHFOLLOWER_BUILTIN
	pathfollower_module_enabled = true;
#else
if (module_state[MODULESETTINGS_ADMINSTATE_VTOLPATHFOLLOWER] == MODULESETTINGS_ADMINSTATE_ENABLED ||
		module_state[MODULESETTINGS_ADMINSTATE_FIXEDWINGPATHFOLLOWER] == MODULESETTINGS_ADMINSTATE_ENABLED ||
		module_state[MODULESETTINGS_ADMINSTATE_GROUNDPATHFOLLOWER] == MODULESETTINGS_ADMINSTATE_ENABLED) {
	pathfollower_module_enabled=true;
}
#endif

#ifdef MODULE_GEOFENCE_BUILTIN
	geofence_module_enabled = true;
#else
	if (module_state[MODULESETTINGS_ADMINSTATE_GEOFENCE] == MODULESETTINGS_ADMINSTATE_ENABLED) {
		geofence_module_enabled=true;
	}
#endif

	return (gps_module_enabled || groundtruth_available) && pathfollower_module_enabled && geofence_module_enabled;
}


/**
 * Set the error code and alarm state
 * @param[in] error code
 */
/**
 * @brief set_manual_control_error
 * @param error_code
 */
static void set_manual_control_error(SystemAlarmsGeoFenceOptions error_code)
{
	// Get the severity of the alarm given the error code
	SystemAlarmsAlarmOptions severity;
	switch (error_code) {
		case SYSTEMALARMS_GEOFENCE_NONE:
		severity = SYSTEMALARMS_ALARM_OK;
		break;
	case SYSTEMALARMS_GEOFENCE_LEAVINGBOUNDARY:
		severity = SYSTEMALARMS_ALARM_WARNING;
		break;
	case SYSTEMALARMS_GEOFENCE_LEFTBOUNDARY:
		severity = SYSTEMALARMS_ALARM_CRITICAL;
		break;
	default:
		severity = SYSTEMALARMS_ALARM_ERROR;
		error_code = SYSTEMALARMS_CONFIGERROR_UNDEFINED;
		break;
	}

	// Make sure not to set the error code if it didn't change
	SystemAlarmsGeoFenceOptions current_error_code;
	SystemAlarmsGeoFenceGet((uint8_t *) &current_error_code);
	if (current_error_code != error_code) {
		SystemAlarmsGeoFenceSet((uint8_t *) &error_code);
	}

	// AlarmSet checks only updates on toggle
	AlarmsSet(SYSTEMALARMS_ALARM_GEOFENCE, (uint8_t) severity);
}


/**
 * @}
 */
