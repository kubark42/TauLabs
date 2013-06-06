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
#include "physical_constants.h"

#include "modulesettings.h"
#include "geofencevertices.h"
#include "geofencefaces.h"
#include "geofencestatus.h"
#include "gpsposition.h"
#include "homelocation.h"
#include "positionactual.h"
#include "velocityactual.h"

//
// Configuration
//
#define STACK_SIZE_BYTES   1500
#define SAMPLE_PERIOD_MS   500
#define TASK_PRIORITY      (tskIDLE_PRIORITY + 1)

// Private functions
static void geofenceTask(void *parameters);
//static bool test_line_triange_intersection(PositionActualData *positionActual, float lineCA[3], float lineBA[3], float vertexA[3], float t);
static void set_geo_fence_error(SystemAlarmsGeoFenceOptions error_code);
static bool check_enabled();

//! Recompute the translation from LLA to NED
static void HomeLocationUpdatedCb(UAVObjEvent * objEv);

//! Convert LLH to NED
static int32_t LLH2NED(int32_t LL[2], float height_AGL, float *NED);

//! Calculate ray-triangle intersection
static bool intersect_triangle( const float V0[3], const float V1[3],const float V2[3], const float  O[3], const float  D[3], float *t);


// Private types

// Private variables
static bool geofence_enabled = false;
static xTaskHandle geofenceTaskHandle;
static HomeLocationData homeLocation;

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
		// Initialize UAVOs
		GPSPositionInitialize();
		HomeLocationInitialize();
		GeoFenceFacesInitialize();
		GeoFenceStatusInitialize();
		GeoFenceVerticesInitialize();

		HomeLocationConnectCallback(&HomeLocationUpdatedCb);

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
	GeoFenceStatusData geofenceStatusData;
	memset(&geofenceStatusData, 0, sizeof(geofenceStatusData));

	HomeLocationUpdatedCb((UAVObjEvent *)NULL);

	while(1) {
		vTaskDelay(SAMPLE_PERIOD_MS);

		uint8_t sum_crossings_buffer_zone = 0; //<-- This could just be a bool that is toggled each time there's a crossing
		uint8_t sum_crossings_safe_zone = 0; //<-- This could just be a bool that is toggled each time there's a crossing
		
		uint16_t num_vertices=UAVObjGetNumInstances(GeoFenceVerticesHandle());
		uint16_t num_faces=UAVObjGetNumInstances(GeoFenceFacesHandle());

		if (num_vertices < 4) {// The fewest number of vertices requiered to make a 3D volume is 4.
			set_geo_fence_error(SYSTEMALARMS_GEOFENCE_INSUFFICIENTVERTICES);
			continue;
		}
		if (num_faces < 4) {// The fewest number of faces requiered to make a 3D volume is 4.
				set_geo_fence_error(SYSTEMALARMS_GEOFENCE_INSUFFICIENTFACES);
			continue;
		}

		VelocityActualData velocityActualData;
		PositionActualData positionActual;
//		PositionActualData positionActual_soon;

		//Load UAVOs
		PositionActualGet(&positionActual);
		VelocityActualGet(&velocityActualData);

		//Predict UAVO future location
		float safety_buffer_time = 3; //Predict 3 seconds into the future// TODO: should perhaps not be hardcoded
//		positionActual_soon.North = positionActual.North + velocityActualData.North*safety_buffer_time;
//		positionActual_soon.East =  positionActual.East  + velocityActualData.East*safety_buffer_time;
//		positionActual_soon.Down =  positionActual.Down  + velocityActualData.Down*safety_buffer_time;
		
		//TODO: It's silly to recreate the normal vector and offset each loop. The equation for the plane should
		// be computed only when the vertices are changed. However, that is much less RAM efficient.

		for (uint16_t i=0; i<num_vertices; i++) {
			GeoFenceVerticesData geofenceVerticesData;
			GeoFenceFacesData geofenceFacesData;
			
			//Get the face of interest
			GeoFenceFacesInstGet(i, &geofenceFacesData);
			
			//Get the three face vertices and convert into NED. Vertex order is important!
			GeoFenceVerticesInstGet(geofenceFacesData.Vertices[0], &geofenceVerticesData);
			int32_t vertexA_LLH[2]={geofenceVerticesData.Latitude, geofenceVerticesData.Longitude};
			float vertexA[3];
			LLH2NED(vertexA_LLH, geofenceVerticesData.Height, vertexA);

			GeoFenceVerticesInstGet(geofenceFacesData.Vertices[1], &geofenceVerticesData);
			int32_t vertexB_LLH[2]={geofenceVerticesData.Latitude, geofenceVerticesData.Longitude};
			float vertexB[3];
			LLH2NED(vertexB_LLH, geofenceVerticesData.Height, vertexB);

			GeoFenceVerticesInstGet(geofenceFacesData.Vertices[2], &geofenceVerticesData);
			int32_t vertexC_LLH[2]={geofenceVerticesData.Latitude, geofenceVerticesData.Longitude};
			float vertexC[3];
			LLH2NED(vertexC_LLH, geofenceVerticesData.Height, vertexC);

			if (i == 0) {
				geofenceStatusData.Status[0] = vertexA[0];
				geofenceStatusData.Status[1] = vertexA[1];
				geofenceStatusData.Status[2] = vertexA[2];
				geofenceStatusData.Status[3] = vertexB[0];
				geofenceStatusData.Status[4] = vertexB[1];
				geofenceStatusData.Status[5] = vertexB[2];
				geofenceStatusData.Status[6] = vertexC[0];
				geofenceStatusData.Status[7] = vertexC[1];
				geofenceStatusData.Status[8] = vertexC[2];

				GeoFenceStatusSet(&geofenceStatusData);
			}
			if (i == 1) {
				geofenceStatusData.Status[9] = vertexC[0];
				geofenceStatusData.Status[10] = vertexC[1];
				geofenceStatusData.Status[11] = vertexC[2];
				GeoFenceStatusSet(&geofenceStatusData);
			}
			
//			//From: http://adrianboeing.blogspot.com/2010/02/intersection-of-convex-hull-with-line.html
//			float lineBA[3]={vertexB[0]-vertexA[0], vertexB[1]-vertexA[1], vertexB[2]-vertexA[2]};
//			float lineCA[3]={vertexC[0]-vertexA[0], vertexC[1]-vertexA[1], vertexC[2]-vertexA[2]};
//			float norm[3];
//			CrossProduct(lineBA, lineCA, norm);
			
//			float d=DotProduct(norm,vertexA);
//			/* This is what the ray looks like, but we have optimized it out of the algorithm so it does not
//				explicitly appear in the calculations:
//					float ray={positionActual_soon[0]+1, positionActual_soon[1], positionActual_soon[2]};
//			*/
			
//			//Solve for line parameter t=d-n*x/(n*ray-x). However, take shortcut because we know that ray-x is [1;0;0]
//			if (fabs(norm[0]<1e-4)) { //If the value of (n*ray-x) is too small, then the ray is parallel to the plane
//				continue;
//			}
			
//			float NED_soon[3]={positionActual_soon.North, positionActual_soon.East, positionActual_soon.Down};
			float NED_now[3] ={positionActual.North,  positionActual.East,  positionActual.Down};
//			float t_soon =(d-DotProduct(norm,NED_soon))/norm[0];
//			float t_now = (d-DotProduct(norm,NED_now) )/norm[0];

			float D[3] = {velocityActualData.North, velocityActualData.East, velocityActualData.Down};
			// Handle the case where the vehicle is stopped, and thus there is no directionality to the velocity vector
			if (D[0] == 0 && D[1] == 0 && D[2] == 0)
				D[0] = 1;
			float t_now = -1;

			// Test if ray falls inside triangle. No need to independently test for both t_soon and t_now ray, as they are identical rays
			bool inside = intersect_triangle(vertexA, vertexB, vertexC, NED_now, D, &t_now);

			geofenceStatusData.Status[12+2*i] = t_now;
			geofenceStatusData.Status[12+2*i+1] = inside == true;
			GeoFenceStatusSet(&geofenceStatusData);


			// Check ray results
			if (inside == false) // If no intersection, then continue
				continue;
			else if (t_now < 0) // If no positive intersection, then continue
				continue;
			else if (t_now < safety_buffer_time) { // The vehicle is in the safety buffer zone
				sum_crossings_buffer_zone++;
			}
			else { // The vehicle is safely inside the geo-fence
				sum_crossings_safe_zone++;
				sum_crossings_buffer_zone++;
			}

//			//Only use the positive side of the ray, as any faces behind it do not intersect with the ray
//			if (t_now < 0){
// //			if (t_soon<0 && t_now < 0){
//				continue;
//			}
			
// //			//Test if ray falls inside triangle. No need to independently test for both t_soon and t_now ray, as they are identical rays
// //			bool inside=test_line_triange_intersection(&positionActual_soon, lineCA, lineBA, vertexA, t_soon);
			
			//Only use the positive side of the ray, as any faces behind it do not intersect with the ray
//			if (t_soon > 0) {
//				sum_crossings_safe_zone++;
//			}
//			if (inside /*&& t_now > 0*/) { //No need to test t_now, as it's already implicit in the test several lines higher
//				sum_crossings_buffer_zone++;
//			}
		}

		geofenceStatusData.Status[21] = sum_crossings_safe_zone;
		geofenceStatusData.Status[22] = sum_crossings_buffer_zone;
	
		//Tests if we have crossed the geo-fence
		if (sum_crossings_safe_zone % 2) {	//If there are an odd number of faces crossed, then the UAV is and will be inside the polyhedron.
			set_geo_fence_error(SYSTEMALARMS_GEOFENCE_NONE);
		}
		else if (sum_crossings_buffer_zone % 2) {	//If sum_crossings_safe_zone is even but sum_crossings_buffer_zone is odd, then the UAV is inside the polyhedron but is leaving soon.
			set_geo_fence_error(SYSTEMALARMS_GEOFENCE_LEAVINGBOUNDARY);
		}
		else { //If sum_crossings_buffer_zone is even, then the UAV is outside the polyhedron.
			set_geo_fence_error(SYSTEMALARMS_GEOFENCE_LEFTBOUNDARY);
		}
		
	}
	
}

#define EPSILON .000001f

/**
 * @brief triangle_intersection "Fast Minimum Storage Ray Triange Intersection", Moller
 * and Trumbore, 1997.
 * @param[in] V0 VertexA
 * @param[in] V1 VertexB
 * @param[in] V2 VertexC
 * @param[in] O 3D Ray origin
 * @param[in] D 3D Ray direction
 * @param[out] t intersection line parameter
 * @return
 */
static bool intersect_triangle(const float V0[3],  // Triangle vertices
                               const float V1[3],
                               const float V2[3],
                               const float  O[3],  // Ray origin
                               const float  D[3],  // Ray direction
                                     float *t)     // output
{
	float edge1[3];
	float edge2[3];
	float P[3];
	float Q[3];
	float T[3];
	float det;
	float inv_det;
	float u;
	float v;

	//Find vectors for two edges sharing V0
	for (int i=0; i<3; i++) {
		edge1[i] = V1[i] - V0[i];
		edge2[i] = V2[i] - V0[i];
	}

	//Begin calculating determinant - also used to calculate u parameter
	CrossProduct(D, edge2, P); // P = D x e2

	//if determinant is near zero, ray lies in plane of triangle
	det = DotProduct(edge1, P);

	if(det > -EPSILON && det < EPSILON)
		return 0;

	//calculate distance from V0 to ray origin
	for (int i=0; i<3; i++)
		T[i] = O[i] - V0[i];

	//Calculate u parameter and test bound
	inv_det = 1.0f / det;
	u = DotProduct(T, P) * inv_det;
	//The intersection lies outside of the triangle
	if(u < 0.0f || u > 1.0f)
		return 0;

	//Prepare to test v parameter
	CrossProduct(T, edge1, Q); // Q = T x e1;

	//Calculate V parameter and test bound
	v = DotProduct(D, Q) * inv_det;
	//The intersection lies outside of the triangle
	if(v < 0.0f || u + v  > 1.0f)
		return 0;

	*t = DotProduct(edge2, Q) * inv_det;
	return 1;
}


///**
// * Test if a line intersects a triangle
// * From: http://www.blackpawn.com/texts/pointinpoly/default.html
// */

//static bool test_line_triange_intersection(PositionActualData *positionActual, float lineCA[3], float lineBA[3], float vertexA[3], float t)
//{
//	float P[3]={positionActual->North+t, positionActual->East, positionActual->Down};
	
//	float linePA[3]={P[0]-vertexA[0], P[1]-vertexA[1], P[2]-vertexA[2]};
	
//	float dot00 = DotProduct(lineCA, lineCA);
//	float dot01 = DotProduct(lineCA, lineBA);
//	float dot02 = DotProduct(lineCA, linePA);
//	float dot11 = DotProduct(lineBA, lineBA);
//	float dot12 = DotProduct(lineBA, linePA);
	
//	// Compute barycentric coordinates
//	float invDenom = 1.0f / (dot00 * dot11 - dot01 * dot01);
//	float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
//	float v = (dot00 * dot12 - dot01 * dot02) * invDenom;
	
//	//Test if point is inside triangle
//	bool 	inside=(u >= 0) && (v >= 0) && (u + v < 1.0f);
	
//	return inside;
//}


static bool check_enabled()
{
	ModuleSettingsInitialize();
	uint8_t module_state[MODULESETTINGS_ADMINSTATE_NUMELEM];

	ModuleSettingsAdminStateGet(module_state);

	bool homelocation_set = false;      //Geo-fence only works if the home location is set
	bool gps_module_enabled = false;    //Geo-fence only works if we have GPS or groundtruth
	bool groundtruth_available = false; //Geo-fence only works if we have GPS or groundtruth
	bool pathfollower_module_enabled = false; // Geo-fence only works if we can autonomously steer the vehicle
	bool geofence_module_enabled = false;

	HomeLocationGet(&homeLocation);
	if (homeLocation.Set == HOMELOCATION_SET_TRUE)
		homelocation_set = true;

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

	return (gps_module_enabled || groundtruth_available) && pathfollower_module_enabled && geofence_module_enabled && homelocation_set;
}


/**
 * Set the error code and alarm state
 * @param[in] error code
 */
/**
 * @brief set_geo_fence_error
 * @param error_code
 */
static void set_geo_fence_error(SystemAlarmsGeoFenceOptions error_code)
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
	case SYSTEMALARMS_GEOFENCE_INSUFFICIENTVERTICES:
		severity = SYSTEMALARMS_ALARM_ERROR;
		break;
	case SYSTEMALARMS_GEOFENCE_INSUFFICIENTFACES:
		severity = SYSTEMALARMS_ALARM_ERROR;
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
 * @brief Convert the Lat-Lon-Height position into NED coordinates
 * @note this method uses a taylor expansion around the home coordinates
 * to convert to NED which allows it to be done with all floating
 * calculations
 * @param[in] World frame coordinates, (Lat, Lon, Height above-ground-level)
 * @param[out] NED frame coordinates
 * @returns 0 for success, -1 for failure
 */
static float T[3];
static float geoidSeparation;
static int32_t LLH2NED(int32_t LL[2], float height_AGL, float *NED)
{
	float dL[3] = { (LL[0] - homeLocation.Latitude) / 10.0e6f * DEG2RAD,
		(LL[1] - homeLocation.Longitude) / 10.0e6f * DEG2RAD,
		height_AGL
	};

	NED[0] = T[0] * dL[0];
	NED[1] = T[1] * dL[1];
	NED[2] = T[2] * dL[2];

	return 0;
}


/**
 * @brief HomeLocationUpdatedCb Recompute the translation from LLH to NED
 * @param objEv
 */
static void HomeLocationUpdatedCb(UAVObjEvent *objEv)
{
	float lat, alt;

	HomeLocationGet(&homeLocation);

	// Compute vector for converting deltaLLA to NED
	lat = homeLocation.Latitude / 10.0e6f * DEG2RAD;
	alt = homeLocation.Altitude;

	T[0] = alt + 6.378137E6f;
	T[1] = cosf(lat) * (alt + 6.378137E6f);
	T[2] = -1.0f;

	GPSPositionGeoidSeparationGet(&geoidSeparation);

}


/**
 * @}
 */
