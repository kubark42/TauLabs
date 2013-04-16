/**
 ******************************************************************************
 * @addtogroup OpenPilotModules OpenPilot Modules
 * @{ 
 * @addtogroup PathFollower Module
 * @{ 
 *
 * @file       fixedwingpathfollower.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2011.
 * @brief      Acquires sensor data and fuses it into attitude estimate for CC
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
#ifndef FIXEDWINGPATHFOLLOWER_H
#define FIXEDWINGPATHFOLLOWER_H

#include "openpilot.h"

void initializeFixedWingPathFollower();
//uint8_t updateFixedWingDesiredStabilization(uint8_t flightMode, FixedWingPathFollowerSettingsCCData fixedwingpathfollowerSettingsCC);

int8_t updateFixedWingDesiredStabilization(FixedWingPathFollowerSettingsData *fixedwingpathfollowerSettings);
float followStraightLine(float r[3], float q[3], float p[3], float psi, float chi_inf, float k_path, float k_psi_int, float delT);
float followOrbit(float c[3], float rho, bool direction, float p[3], float psi, float k_orbit, float k_psi_int, float delT);
void PathFollowerUpdatedCb(UAVObjEvent * ev);
void zeroGuidanceIntegral();

#endif				// FIXEDWINGPATHFOLLOWER_H
