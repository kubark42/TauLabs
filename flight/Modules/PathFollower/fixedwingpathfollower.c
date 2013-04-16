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
#include "pathmanagerstatus.h"

// Private constants

// Private types
static struct Integral {
	float totalEnergyError;
	float airspeedError;

	float lineError;
	float circleError;
} *integral;

// Private variables
static PathDesiredData pathDesired;
static FixedWingPathFollowerSettingsData fixedwingpathfollowerSettings;
static FixedWingAirspeedsData fixedWingAirspeeds;
static uint16_t activeSegment;

// Private functions


void initializeFixedWingPathFollower()
{
	// Allocate memory
	integral = (struct Integral *) pvPortMalloc(sizeof(struct Integral));
	memset(integral, 0, sizeof(struct Integral));
}

void zeroGuidanceIntegral(){
	integral->totalEnergyError = 0;
	integral->airspeedError = 0;
	integral->lineError = 0;
	integral->circleError = 0;
}


/**
 * @brief
 */
int8_t updateFixedWingDesiredStabilization(FixedWingPathFollowerSettingsData *fixedwingpathfollowerSettings)
{
	float dT = fixedwingpathfollowerSettings->UpdatePeriod / 1000.0f; //Convert from [ms] to [s]

	VelocityActualData velocityActual;
	StabilizationDesiredData stabDesired;
	float trueAirspeed;

	float calibratedAirspeedActual;
	float airspeedDesired;
	float airspeedError;

	float pitchCommand;

	float powerCommand;
	float headingError_R;
	float rollCommand;

	VelocityActualGet(&velocityActual);
	StabilizationDesiredGet(&stabDesired);
	AirspeedActualTrueAirspeedGet(&trueAirspeed); //BOOOO!!! This not the way to get true airspeed. It needs to come from a UAVO that merges everything together.

	PositionActualData positionActual;
	PositionActualGet(&positionActual);

	PathSegmentDescriptorData pathSegmentDescriptor;
	PathSegmentDescriptorInstGet(activeSegment, &pathSegmentDescriptor);


	/**
	 * Compute speed error (required for throttle and pitch)
	 */

	// Current airspeed
	calibratedAirspeedActual = trueAirspeed; //BOOOOOOOOOO!!! Where's the conversion from TAS to CAS?

	// Current heading
	float headingActual_R = atan2f(velocityActual.East, velocityActual.North);

	// Desired airspeed
	airspeedDesired=pathDesired.EndingVelocity;


	// Airspeed error
	airspeedError = airspeedDesired - calibratedAirspeedActual;

	/**
	 * Compute desired throttle command
	 */

	//Proxy because instead of m*(1/2*v^2+g*h), it's v^2+2*gh. This saves processing power
	float totalEnergyProxySetpoint=powf(pathDesired.EndingVelocity,2.0f) - 2.0f*9.8f*pathDesired.End[2];
	float totalEnergyProxyActual=powf(trueAirspeed,2.0f) - 2.0f*9.8f*positionActual.Down;
	float errorTotalEnergy= totalEnergyProxySetpoint - totalEnergyProxyActual;

#define THROTTLE_KP fixedwingpathfollowerSettings->ThrottlePI[FIXEDWINGPATHFOLLOWERSETTINGS_THROTTLEPI_KP]
#define THROTTLE_KI fixedwingpathfollowerSettings->ThrottlePI[FIXEDWINGPATHFOLLOWERSETTINGS_THROTTLEPI_KI]
#define THROTTLE_ILIMIT fixedwingpathfollowerSettings->ThrottlePI[FIXEDWINGPATHFOLLOWERSETTINGS_THROTTLEPI_ILIMIT]

	//Integrate with bound. Make integral leaky for better performance. Approximately 30s time constant.
	if (THROTTLE_KI > 0.0f){
		integral->totalEnergyError=bound_min_max(integral->totalEnergyError+errorTotalEnergy*dT,
										 -THROTTLE_ILIMIT/THROTTLE_KI,
										 THROTTLE_ILIMIT/THROTTLE_KI)*(1.0f-1.0f/(1.0f+30.0f/dT));
	}

	powerCommand=errorTotalEnergy*THROTTLE_KP
	+ integral->totalEnergyError*THROTTLE_KI;

#define THROTTLELIMIT_NEUTRAL fixedwingpathfollowerSettings->ThrottleLimit[FIXEDWINGPATHFOLLOWERSETTINGS_THROTTLELIMIT_NEUTRAL]
#define THROTTLELIMIT_MIN     fixedwingpathfollowerSettings->ThrottleLimit[FIXEDWINGPATHFOLLOWERSETTINGS_THROTTLELIMIT_MIN]
#define THROTTLELIMIT_MAX     fixedwingpathfollowerSettings->ThrottleLimit[FIXEDWINGPATHFOLLOWERSETTINGS_THROTTLELIMIT_MAX]

	// set throttle
	stabDesired.Throttle = bound_min_max(powerCommand+THROTTLELIMIT_NEUTRAL,
								 THROTTLELIMIT_MIN,
								 THROTTLELIMIT_MAX);
	/**
	 * Compute desired pitch command
	 */

#define AIRSPEED_KP      fixedwingpathfollowerSettings->AirspeedPI[FIXEDWINGPATHFOLLOWERSETTINGS_AIRSPEEDPI_KP]
#define AIRSPEED_KI      fixedwingpathfollowerSettings->AirspeedPI[FIXEDWINGPATHFOLLOWERSETTINGS_AIRSPEEDPI_KI]
#define AIRSPEED_ILIMIT	 fixedwingpathfollowerSettings->AirspeedPI[FIXEDWINGPATHFOLLOWERSETTINGS_AIRSPEEDPI_ILIMIT]

	if (AIRSPEED_KI > 0.0f){
		//Integrate with saturation
		integral->airspeedError=bound_min_max(integral->airspeedError + airspeedError * dT,
									  -AIRSPEED_ILIMIT/AIRSPEED_KI,
									  AIRSPEED_ILIMIT/AIRSPEED_KI);
	}

	//Compute the cross feed from altitude to pitch, with saturation
#define PITCHCROSSFEED_KP fixedwingpathfollowerSettings->VerticalToPitchCrossFeed[FIXEDWINGPATHFOLLOWERSETTINGS_VERTICALTOPITCHCROSSFEED_KP]
#define PITCHCROSSFEED_MIN	fixedwingpathfollowerSettings->VerticalToPitchCrossFeed[FIXEDWINGPATHFOLLOWERSETTINGS_VERTICALTOPITCHCROSSFEED_MAX]
#define PITCHCROSSFEED_MAX fixedwingpathfollowerSettings->VerticalToPitchCrossFeed[FIXEDWINGPATHFOLLOWERSETTINGS_VERTICALTOPITCHCROSSFEED_MAX]
	float alitudeError=pathDesired.End[2]-positionActual.Down;
	float altitudeToPitchCommandComponent=bound_min_max( alitudeError* PITCHCROSSFEED_KP,
												 -PITCHCROSSFEED_MIN,
												 PITCHCROSSFEED_MAX);

	//Compute the pitch command as err*Kp + errInt*Ki + X_feed.
	pitchCommand= -(airspeedError*AIRSPEED_KP
					+ integral->airspeedError*AIRSPEED_KI)	+ altitudeToPitchCommandComponent;

	//Saturate pitch command
#define PITCHLIMIT_NEUTRAL  fixedwingpathfollowerSettings->PitchLimit[FIXEDWINGPATHFOLLOWERSETTINGS_PITCHLIMIT_NEUTRAL]
#define PITCHLIMIT_MIN      fixedwingpathfollowerSettings->PitchLimit[FIXEDWINGPATHFOLLOWERSETTINGS_PITCHLIMIT_MIN]
#define PITCHLIMIT_MAX      fixedwingpathfollowerSettings->PitchLimit[FIXEDWINGPATHFOLLOWERSETTINGS_PITCHLIMIT_MAX]

	stabDesired.Pitch = bound_min_max(PITCHLIMIT_NEUTRAL +
							  pitchCommand,
							  PITCHLIMIT_MIN,
							  PITCHLIMIT_MAX);

	/**
	 * Compute desired roll command
	 */

	float p[3]={positionActual.North, positionActual.East, positionActual.Down};
	float *c = pathDesired.End;
	float *r = pathDesired.Start;
	float q[3] = {pathDesired.End[0]-pathDesired.Start[0], pathDesired.End[1]-pathDesired.Start[1], pathDesired.End[2]-pathDesired.Start[2]};

	float k_path  = fixedwingpathfollowerSettings->VectorFollowingGain/pathDesired.EndingVelocity; //Divide gain by airspeed so that the turn rate is independent of airspeed
	float k_orbit = fixedwingpathfollowerSettings->OrbitFollowingGain/pathDesired.EndingVelocity; //Divide gain by airspeed so that the turn rate is independent of airspeed
	float k_psi_int = fixedwingpathfollowerSettings->FollowerIntegralGain;
	//========================================
	//SHOULD NOT BE HARD CODED

	float chi_inf=PI/4.0f; //THIS NEEDS TO BE A FUNCTION OF HOW LONG OUR PATH IS.

	//Saturate chi_inf. I.e., never approach the path at a steeper angle than 45 degrees
	chi_inf= chi_inf < PI/4.0f? PI/4.0f: chi_inf;
	//========================================

	float rho;
	float headingCommand_R;

#define ROLL_FOR_HOLDING_CIRCLE 15.0f	 //Assume that we want a 15 degree bank angle. This should yield a nice, non-agressive turn
	//Calculate radius, rho, using r*omega=v and omega = g/V_g * tan(phi)
	//THIS SHOULD ONLY BE CALCULATED ONCE, INSTEAD OF EVERY TIME
	rho=powf(pathDesired.EndingVelocity,2)/(9.805f*tanf(fabs(ROLL_FOR_HOLDING_CIRCLE*DEG2RAD)));


	switch (pathSegmentDescriptor.SegmentType){
		case PATHSEGMENTDESCRIPTOR_SEGMENTTYPE_CURVECOUNTERCLOCKWISE:
			headingCommand_R=followOrbit(c, rho, false, p, headingActual_R, k_orbit, k_psi_int, dT);
			break;
		case PATHSEGMENTDESCRIPTOR_SEGMENTTYPE_CURVECLOCKWISE:
			headingCommand_R=followOrbit(c, rho, true, p, headingActual_R, k_orbit, k_psi_int, dT);
			break;
		case PATHSEGMENTDESCRIPTOR_SEGMENTTYPE_LINE:
			headingCommand_R=followStraightLine(r, q, p, headingActual_R, chi_inf, k_path, k_psi_int, dT);
			break;
		default:
			// TODO: Throw a critical error
			return -1;
	}

	//Calculate heading error
	headingError_R = headingCommand_R-headingActual_R;

	//Wrap heading error around circle
	if (headingError_R < -PI) headingError_R+=2.0f*PI;
	if (headingError_R >  PI) headingError_R-=2.0f*PI;


	//GET RID OF THE RAD2DEG. IT CAN BE FACTORED INTO HeadingPI
#define ROLLLIMIT_NEUTRAL  fixedwingpathfollowerSettings->RollLimit[FIXEDWINGPATHFOLLOWERSETTINGS_ROLLLIMIT_NEUTRAL]
#define ROLLLIMIT_MIN      fixedwingpathfollowerSettings->RollLimit[FIXEDWINGPATHFOLLOWERSETTINGS_ROLLLIMIT_MIN]
#define ROLLLIMIT_MAX      fixedwingpathfollowerSettings->RollLimit[FIXEDWINGPATHFOLLOWERSETTINGS_ROLLLIMIT_MAX]
#define HEADINGPI_KP fixedwingpathfollowerSettings->HeadingPI[FIXEDWINGPATHFOLLOWERSETTINGS_HEADINGPI_KP]
	rollCommand = (headingError_R * HEADINGPI_KP)* RAD2DEG;

	//Turn heading

	stabDesired.Roll = bound_min_max( ROLLLIMIT_NEUTRAL +
							 rollCommand,
							 ROLLLIMIT_MIN,
							 ROLLLIMIT_MAX);

	/**
	 * Compute desired yaw command
	 */
	// TODO Once coordinated flight is merged in, YAW needs to switch to STABILIZATIONDESIRED_STABILIZATIONMODE_COORDINATEDFLIGHT
	stabDesired.Yaw = 0;

	stabDesired.StabilizationMode[STABILIZATIONDESIRED_STABILIZATIONMODE_ROLL] = STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDE;
	stabDesired.StabilizationMode[STABILIZATIONDESIRED_STABILIZATIONMODE_PITCH] = STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDE;
	stabDesired.StabilizationMode[STABILIZATIONDESIRED_STABILIZATIONMODE_YAW] = STABILIZATIONDESIRED_STABILIZATIONMODE_COORDINATEDFLIGHT;

	StabilizationDesiredSet(&stabDesired);

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


	float psi_command= direction==true?
		phi+(PI/2.0f + atanf(k_orbit*err_orbit) + k_psi_int*integral->circleError):
		phi-(PI/2.0f + atanf(k_orbit*err_orbit) + k_psi_int*integral->circleError);

	return psi_command;
}

void GuidanceSettingsUpdatedCb(UAVObjEvent * ev)
{
	if (ev == NULL || ev->obj == FixedWingPathFollowerSettingsHandle())
		FixedWingPathFollowerSettingsGet(&fixedwingpathfollowerSettings);
	if (ev == NULL || ev->obj == FixedWingAirspeedsHandle())
		FixedWingAirspeedsGet(&fixedWingAirspeeds);
	if (ev == NULL || ev->obj == PathManagerStatusHandle())
	{
		PathManagerStatusData pathManagerStatusData;
		PathManagerStatusGet(&pathManagerStatusData);
		activeSegment = pathManagerStatusData.ActiveSegment;
	}
}
