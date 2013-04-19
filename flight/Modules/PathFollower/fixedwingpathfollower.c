/**
 ******************************************************************************
 *
 * @file       fixedwingpathfollower.c
 * @author     Tau Labs, http://www.taulabs.org Copyright (C) 2013.
 * @brief
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

#include "physical_constants.h"
#include "paths.h"
#include "misc_math.h"

#include "attitudeactual.h"
#include "positionactual.h"
#include "velocityactual.h"
#include "manualcontrol.h"
#include "airspeedactual.h"
#include "homelocation.h"
#include "stabilizationdesired.h" // object that will be updated by the module
#include "systemsettings.h"
#include "fixedwingairspeeds.h"
#include "fixedwingpathfollowersettings.h"
#include "modulesettings.h"

#include "CoordinateConversions.h"
#include "fixedwingpathfollower.h"
#include "pathsegmentdescriptor.h"
#include "pathfollowerstatus.h"
#include "pathmanagerstatus.h"

// Private constants

// Private types
static struct Integral {
	float totalEnergyError;
	float calibratedAirspeedError;

	float lineError;
	float circleError;
} *integral;


struct ControllerOutput {
	float roll;
	float pitch;
	float yaw;
	float throttle;
};

// Private variables
static PathDesiredData pathDesired;
static FixedWingPathFollowerSettingsData fixedwingpathfollowerSettings;
static FixedWingAirspeedsData fixedWingAirspeeds;
static uint16_t activeSegment;
static float rho;
static xQueueHandle pathManagerStatusQueue;

// Private functions
static void SettingsUpdatedCb(UAVObjEvent * ev);
static void updateDestination();
static float followStraightLine(float r[3], float q[3], float p[3], float psi, float chi_inf, float k_path, float k_psi_int, float delT);
static float followOrbit(float c[3], float rho, bool direction, float p[3], float psi, float k_orbit, float k_psi_int, float delT);

void airspeedController(struct ControllerOutput *airspeedControl, float calibratedAirspeedError, float altitudeError, float dT);
void totalEnergyController(struct ControllerOutput *altitudeControl, float true_airspeed_desired,
						   float true_airspeed_actual, float altitude_desired_NED, float altitude_actual_NED, float dT);
void headingController(struct ControllerOutput *headingControl, float headingError_R);

float desiredTrackingHeading(PathSegmentDescriptorData *pathSegmentDescriptor, PositionActualData *positionActual, float headingActual_R, float trueAirspeedDesired, float dT);

void initializeFixedWingPathFollower()
{
	// Initialize UAVOs
	FixedWingPathFollowerSettingsInitialize();
	FixedWingAirspeedsInitialize();
	AirspeedActualInitialize();

	// Register callbacks
	FixedWingAirspeedsConnectCallback(SettingsUpdatedCb);
	FixedWingPathFollowerSettingsConnectCallback(SettingsUpdatedCb);

	// Register queues
	pathManagerStatusQueue = xQueueCreate(1, sizeof(UAVObjEvent));
	PathManagerStatusConnectQueue(pathManagerStatusQueue);

	// Allocate memory
	integral = (struct Integral *) pvPortMalloc(sizeof(struct Integral));
	memset(integral, 0, sizeof(struct Integral));

	// Load all settings
	SettingsUpdatedCb((UAVObjEvent *)NULL);
}

void zeroGuidanceIntegral(){
	integral->totalEnergyError = 0;
	integral->calibratedAirspeedError = 0;
	integral->lineError = 0;
	integral->circleError = 0;
}


/**
 * @brief
 */
int8_t updateFixedWingDesiredStabilization()
{
	// Check if the path manager has updated.
	UAVObjEvent ev;
	if (xQueueReceive(pathManagerStatusQueue, &ev, 0) == pdTRUE)
	{
		PathManagerStatusData pathManagerStatusData;
		PathManagerStatusGet(&pathManagerStatusData);

		if (activeSegment != pathManagerStatusData.ActiveSegment)
		{
			activeSegment = pathManagerStatusData.ActiveSegment;

			updateDestination();
		}
	}

	float dT = fixedwingpathfollowerSettings.UpdatePeriod / 1000.0f; //Convert from [ms] to [s]

	VelocityActualData velocityActual;
	StabilizationDesiredData stabilizationDesired;
	float trueAirspeed;
	float calibratedAirspeed;

	float trueAirspeedDesired;
	float calibratedAirspeedDesired;
	float calibratedAirspeedError;

	float altitudeError;
	float headingError_R;


	float altitudeDesired;
	float headingDesired_R;

	VelocityActualGet(&velocityActual);
	StabilizationDesiredGet(&stabilizationDesired);

	PositionActualData positionActual;
	PositionActualGet(&positionActual);

	PathSegmentDescriptorData pathSegmentDescriptor;
	PathSegmentDescriptorInstGet(activeSegment, &pathSegmentDescriptor);

	// Current airspeed
	AirspeedActualTrueAirspeedGet(&trueAirspeed);
	AirspeedActualCalibratedAirspeedGet(&calibratedAirspeed);

	// Current heading
	float headingActual_R = atan2f(velocityActual.East, velocityActual.North);

	/**
	 * Compute setpoints
	 */
	// Set desired calibrated airspeed, bounded by airframe limits
	calibratedAirspeedDesired = bound_min_max(pathDesired.EndingVelocity, fixedWingAirspeeds.StallSpeedDirty, fixedWingAirspeeds.AirSpeedMax);

	// Set the desired true airspeed
	trueAirspeedDesired = calibratedAirspeedDesired; // <--------- BOOOO. Need to use the correct definition of true airspeed, for the instantaneous altitude at which the plane is flying

	// Set the desired altitude
	altitudeDesired = pathDesired.End[2];

	// Set the desired heading.
	headingDesired_R = desiredTrackingHeading(&pathSegmentDescriptor, &positionActual, headingActual_R, trueAirspeedDesired, dT);

	/**
	 * Compute setpoint errors
	 */
	// Airspeed error
	calibratedAirspeedError = calibratedAirspeedDesired - calibratedAirspeed;

	// Altitude error
	altitudeError = altitudeDesired - positionActual.Down;

	// Heading error
	headingError_R = headingDesired_R - headingActual_R;

	//Wrap heading error around circle
	if (headingError_R < -PI)
		headingError_R+=2.0f*PI;
	if (headingError_R >  PI)
		headingError_R-=2.0f*PI;

	/**
	 * Compute controls
	 */
	// Compute airspeed control
	struct ControllerOutput airspeedControl;
	airspeedController(&airspeedControl, calibratedAirspeedError, altitudeError, dT);

	// Compute altitude control
	struct ControllerOutput totalEnergyControl;
	totalEnergyController(&totalEnergyControl, trueAirspeedDesired, trueAirspeed, pathDesired.End[2], positionActual.Down, dT);

	// Compute heading control
	struct ControllerOutput headingControl;
	headingController(&headingControl, headingError_R);


	// Sum all controllers
	stabilizationDesired.Throttle = bound_min_max(headingControl.throttle + airspeedControl.throttle + totalEnergyControl.throttle,
												  fixedwingpathfollowerSettings.ThrottleLimit[FIXEDWINGPATHFOLLOWERSETTINGS_THROTTLELIMIT_MIN],
												  fixedwingpathfollowerSettings.ThrottleLimit[FIXEDWINGPATHFOLLOWERSETTINGS_THROTTLELIMIT_MAX]);
	stabilizationDesired.Roll     = bound_min_max(headingControl.roll + airspeedControl.roll + totalEnergyControl.roll,
												  fixedwingpathfollowerSettings.RollLimit[FIXEDWINGPATHFOLLOWERSETTINGS_ROLLLIMIT_MIN],
												  fixedwingpathfollowerSettings.RollLimit[FIXEDWINGPATHFOLLOWERSETTINGS_ROLLLIMIT_MAX]);
	stabilizationDesired.Pitch    = bound_min_max(headingControl.pitch + airspeedControl.pitch + totalEnergyControl.pitch,
												  fixedwingpathfollowerSettings.PitchLimit[FIXEDWINGPATHFOLLOWERSETTINGS_PITCHLIMIT_MIN],
												  fixedwingpathfollowerSettings.PitchLimit[FIXEDWINGPATHFOLLOWERSETTINGS_PITCHLIMIT_MAX]);
	stabilizationDesired.Yaw      = headingControl.yaw + airspeedControl.yaw + altitudeControl.yaw; // Coordinated flight control only works when stabilizationDesired.Yaw == 0

	// Set stabilization modes
	stabilizationDesired.StabilizationMode[STABILIZATIONDESIRED_STABILIZATIONMODE_ROLL] = STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDE; //This needs to be EnhancedAttitude control
	stabilizationDesired.StabilizationMode[STABILIZATIONDESIRED_STABILIZATIONMODE_PITCH] = STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDE; //This needs to be EnhancedAttitude control
	stabilizationDesired.StabilizationMode[STABILIZATIONDESIRED_STABILIZATIONMODE_YAW] = STABILIZATIONDESIRED_STABILIZATIONMODE_COORDINATEDFLIGHT;

	StabilizationDesiredSet(&stabilizationDesired);

	return 0;
}

/**
 * Calculate command for following simple vector based line. Taken from R. Beard at BYU.
 */

float followStraightLine(float r[3], float q[3], float p[3], float psi, float chi_inf, float k_path, float k_psi_int, float delT){
	float chi_q=atan2f(q[1], q[0]);
	while (chi_q - psi < -PI) {
		chi_q+=2.0f*PI;
	}
	while (chi_q - psi > PI) {
		chi_q-=2.0f*PI;
	}

	float err_p=-sinf(chi_q)*(p[0]-r[0])+cosf(chi_q)*(p[1]-r[1]);
	integral->lineError+=delT*err_p;
	float psi_command = chi_q-chi_inf*2.0f/PI*atanf(k_path*err_p)-k_psi_int*integral->lineError;

	return psi_command;
}


/**
 * Calculate command for following simple vector based orbit. Taken from R. Beard at BYU.
 */
float followOrbit(float c[3], float rho, bool direction, float p[3], float psi, float k_orbit, float k_psi_int, float delT){
	float pncn=p[0]-c[0];
	float pece=p[1]-c[1];
	float d=sqrtf(pncn*pncn + pece*pece);

	float err_orbit=d-rho;
	integral->circleError+=err_orbit*delT;


	float phi=atan2f(pece, pncn);
	while(phi-psi < -PI){
		phi=phi+2.0f*PI;
	}
	while(phi-psi > PI){
		phi=phi-2.0f*PI;
	}


	float psi_command = direction==true?
		phi+(PI/2.0f + atanf(k_orbit*err_orbit) + k_psi_int*integral->circleError): // Turn clockwise
		phi-(PI/2.0f + atanf(k_orbit*err_orbit) + k_psi_int*integral->circleError); // Turn counter-clockwise

	return psi_command;
}

void updateDestination(){
	PathSegmentDescriptorData pathSegmentDescriptor;

	//VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
	// BLAH, BLAH, BLAH. THIS SHOULDN'T BE USING PATHDESIRED UAVO
	//----------------------------------------------
	PathSegmentDescriptorInstGet(activeSegment-1, &pathSegmentDescriptor);
	pathDesired.Start[0]=pathSegmentDescriptor.SwitchingLocus[0];
	pathDesired.Start[1]=pathSegmentDescriptor.SwitchingLocus[1];
	pathDesired.Start[2]=pathSegmentDescriptor.SwitchingLocus[2];

	PathSegmentDescriptorInstGet(activeSegment, &pathSegmentDescriptor);

	// For a straight line use the switching locus as the vector endpoint...
	if(pathSegmentDescriptor.PathCurvature == 0){
		pathDesired.End[0]=pathSegmentDescriptor.SwitchingLocus[0];
		pathDesired.End[1]=pathSegmentDescriptor.SwitchingLocus[1];
		pathDesired.End[2]=pathSegmentDescriptor.SwitchingLocus[2];
	}
	else{ // ...but for an arc, use the switching loci to calculate the arc center
		float oldPosition_NE[2] = {pathDesired.Start[0], pathDesired.Start[1]};
		float newPosition_NE[2] = {pathSegmentDescriptor.SwitchingLocus[0], pathSegmentDescriptor.SwitchingLocus[1]};
		float arcCenter_XY[2];
		bool ret;
		ret = arcCenterFromTwoPointsAndRadiusAndArcRank(oldPosition_NE, newPosition_NE, 1.0f/pathSegmentDescriptor.PathCurvature, arcCenter_XY, pathSegmentDescriptor.PathCurvature > 0, pathSegmentDescriptor.ArcRank == PATHSEGMENTDESCRIPTOR_ARCRANK_MINOR);

		if (ret == CENTER_FOUND){
			pathDesired.End[0]=arcCenter_XY[0];
			pathDesired.End[1]=arcCenter_XY[1];
			pathDesired.End[2]=pathSegmentDescriptor.SwitchingLocus[2];
		}
		else {
			// This is bad, but we have to handle it.
			// Probably the path manager should advance to the next waypoint, but for now we'll circle over the point
			pathDesired.End[0]=pathSegmentDescriptor.SwitchingLocus[0];
			pathDesired.End[1]=pathSegmentDescriptor.SwitchingLocus[1];
			pathDesired.End[2]=pathSegmentDescriptor.SwitchingLocus[2];

			// TODO: Set alarm warning
			AlarmsSet(SYSTEMALARMS_ALARM_PATHFOLLOWER, SYSTEMALARMS_ALARM_WARNING);
		}

#define MAX_ROLL_FOR_ARC 15.0f	 //Assume that we want a maximum 15 degree bank angle. This should yield a nice, non-agressive turn
#define MIN_RHO (powf(pathSegmentDescriptor.FinalVelocity,2)/(9.805f*tanf(fabs(MAX_ROLL_FOR_ARC*DEG2RAD))))
		//Calculate radius, rho, using r*omega=v and omega = g/V_g * tan(phi)
		rho = fabs(1.0f/pathSegmentDescriptor.PathCurvature) > MIN_RHO ? fabs(1.0f/pathSegmentDescriptor.PathCurvature) : MIN_RHO;
	}

	pathDesired.EndingVelocity=pathSegmentDescriptor.FinalVelocity;
	PathDesiredSet(&pathDesired);
	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	// BLAH, BLAH, BLAH. THIS SHOULDN'T BE USING PATHDESIRED
	//----------------------------------------------
}


void airspeedController(struct ControllerOutput *airspeedControl, float calibratedAirspeedError, float altitudeError, float dT)
{
	// This is the throttle value required for level flight at the given airspeed
	float feedForwardThrottle = fixedwingpathfollowerSettings.ThrottleLimit[FIXEDWINGPATHFOLLOWERSETTINGS_THROTTLELIMIT_NEUTRAL];


	/**
	 * Compute desired pitch command
	 */

#define AIRSPEED_KP      fixedwingpathfollowerSettings.AirspeedPI[FIXEDWINGPATHFOLLOWERSETTINGS_AIRSPEEDPI_KP]
#define AIRSPEED_KI      fixedwingpathfollowerSettings.AirspeedPI[FIXEDWINGPATHFOLLOWERSETTINGS_AIRSPEEDPI_KI]
#define AIRSPEED_ILIMIT	 fixedwingpathfollowerSettings.AirspeedPI[FIXEDWINGPATHFOLLOWERSETTINGS_AIRSPEEDPI_ILIMIT]

	if (AIRSPEED_KI > 0.0f){
		//Integrate with saturation
		integral->calibratedAirspeedError=bound_min_max(integral->calibratedAirspeedError + calibratedAirspeedError * dT,
									  -AIRSPEED_ILIMIT/AIRSPEED_KI,
									  AIRSPEED_ILIMIT/AIRSPEED_KI);
	}

	//Compute the cross feed from altitude to pitch, with saturation
#define PITCHCROSSFEED_KP fixedwingpathfollowerSettings.AltitudeErrorToPitchCrossFeed[FIXEDWINGPATHFOLLOWERSETTINGS_ALTITUDEERRORTOPITCHCROSSFEED_KP]
#define PITCHCROSSFEED_MIN	fixedwingpathfollowerSettings.AltitudeErrorToPitchCrossFeed[FIXEDWINGPATHFOLLOWERSETTINGS_ALTITUDEERRORTOPITCHCROSSFEED_KP]
#define PITCHCROSSFEED_MAX fixedwingpathfollowerSettings.AltitudeErrorToPitchCrossFeed[FIXEDWINGPATHFOLLOWERSETTINGS_ALTITUDEERRORTOPITCHCROSSFEED_KP]
	float altitudeErrorToPitchCommandComponent=bound_min_max(altitudeError* PITCHCROSSFEED_KP, -PITCHCROSSFEED_MIN, PITCHCROSSFEED_MAX);

	//Saturate pitch command
#define PITCHLIMIT_NEUTRAL  fixedwingpathfollowerSettings.PitchLimit[FIXEDWINGPATHFOLLOWERSETTINGS_PITCHLIMIT_NEUTRAL]

	// Assign airspeed controller outputs
	airspeedControl->throttle = feedForwardThrottle;
	airspeedControl->roll = 0;
	airspeedControl->pitch = -(calibratedAirspeedError*AIRSPEED_KP + integral->calibratedAirspeedError*AIRSPEED_KI) + altitudeErrorToPitchCommandComponent + PITCHLIMIT_NEUTRAL;
	airspeedControl->yaw = 0;
}


/**
  *
  */
void totalEnergyController(struct ControllerOutput *altitudeControl, float true_airspeed_desired, float true_airspeed_actual, float altitude_desired_NED, float altitude_actual_NED, float dT)
{
	//Proxy because instead of m*(1/2*v^2+g*h), it's v^2+2*gh. This saves processing power
	float totalEnergyProxySetpoint=powf(true_airspeed_desired, 2.0f) - 2.0f*9.8f*altitude_desired_NED;
	float totalEnergyProxyActual=powf(true_airspeed_actual, 2.0f) - 2.0f*9.8f*altitude_actual_NED;
	float errorTotalEnergy= totalEnergyProxySetpoint - totalEnergyProxyActual;

#define THROTTLE_KP fixedwingpathfollowerSettings.ThrottlePI[FIXEDWINGPATHFOLLOWERSETTINGS_THROTTLEPI_KP]
#define THROTTLE_KI fixedwingpathfollowerSettings.ThrottlePI[FIXEDWINGPATHFOLLOWERSETTINGS_THROTTLEPI_KI]
#define THROTTLE_ILIMIT fixedwingpathfollowerSettings.ThrottlePI[FIXEDWINGPATHFOLLOWERSETTINGS_THROTTLEPI_ILIMIT]

	//Integrate with bound. Make integral leaky for better performance. Approximately 30s time constant.
	if (THROTTLE_KI > 0.0f){
		integral->totalEnergyError=bound_min_max(integral->totalEnergyError+errorTotalEnergy*dT,
										 -THROTTLE_ILIMIT/THROTTLE_KI,
										 THROTTLE_ILIMIT/THROTTLE_KI)*(1.0f-1.0f/(1.0f+30.0f/dT));
	}

	// Assign altitude controller outputs
	altitudeControl->throttle = errorTotalEnergy*THROTTLE_KP + integral->totalEnergyError*THROTTLE_KI;
	altitudeControl->roll = 0;
	altitudeControl->pitch = 0;
	altitudeControl->yaw = 0;

}


/**
 * This simplified heading controller only computes a roll command
 */
float desiredTrackingHeading(PathSegmentDescriptorData *pathSegmentDescriptor, PositionActualData *positionActual, float headingActual_R, float trueAirspeedDesired, float dT)
{
	float p[3]={positionActual->North, positionActual->East, positionActual->Down};
	float *c = pathDesired.End;
	float *r = pathDesired.Start;
	float q[3] = {pathDesired.End[0]-pathDesired.Start[0], pathDesired.End[1]-pathDesired.Start[1], pathDesired.End[2]-pathDesired.Start[2]};

	float k_path  = fixedwingpathfollowerSettings.VectorFollowingGain/trueAirspeedDesired; //Divide gain by airspeed so that the turn rate is independent of airspeed
	float k_orbit = fixedwingpathfollowerSettings.OrbitFollowingGain/trueAirspeedDesired; //Divide gain by airspeed so that the turn rate is independent of airspeed
	float k_psi_int = fixedwingpathfollowerSettings.FollowerIntegralGain;
	//========================================
	//SHOULD NOT BE HARD CODED

	float chi_inf = PI/4.0f; //THIS NEEDS TO BE A FUNCTION OF HOW LONG OUR PATH IS.

	//Saturate chi_inf. I.e., never approach the path at a steeper angle than 45 degrees
	chi_inf = chi_inf > PI/4.0f ? PI/4.0f : chi_inf;
	//========================================

	float headingDesired_R;

	if (pathSegmentDescriptor->PathCurvature == 0) { // Straight line has no curvature
		headingDesired_R=followStraightLine(r, q, p, headingActual_R, chi_inf, k_path, k_psi_int, dT);
	}
	else {
		if(pathSegmentDescriptor->PathCurvature > 0) // Turn clockwise
			headingDesired_R=followOrbit(c, rho, true, p, headingActual_R, k_orbit, k_psi_int, dT);
		else // Turn counter-clockwise
			headingDesired_R=followOrbit(c, rho, false, p, headingActual_R, k_orbit, k_psi_int, dT);
	}

	return headingDesired_R;
}


/**
 * This simplified heading controller only computes a roll command
 */
void headingController(struct ControllerOutput *headingControl, float headingError_R)
{
	// Assign heading controller outputs
	headingControl->throttle = 0;
	headingControl->roll = (headingError_R * fixedwingpathfollowerSettings.HeadingPI[FIXEDWINGPATHFOLLOWERSETTINGS_HEADINGPI_KP]) * RAD2DEG;
	headingControl->pitch = 0;
	headingControl->yaw = 0;
}



void SettingsUpdatedCb(UAVObjEvent * ev)
{
	if (ev == NULL || ev->obj == FixedWingPathFollowerSettingsHandle())
		FixedWingPathFollowerSettingsGet(&fixedwingpathfollowerSettings);
	if (ev == NULL || ev->obj == FixedWingAirspeedsHandle())
		FixedWingAirspeedsGet(&fixedWingAirspeeds);
	if (ev == NULL || ev->obj == PathManagerStatusHandle())
	{

	}
}
