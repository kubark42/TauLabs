/**
 ******************************************************************************
 * @file       path_followers.h
 * @author     Tau Labs, http://www.taulabs.org, Copyright (C) 2013
 * @addtogroup Path Followers
 * @{
 * @brief Miscellaneous math support
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

#ifndef PATH_FOLLOWERS_H
#define PATH_FOLLOWERS_H

#include <pios.h>
#include "openpilot.h"

#include "positionactual.h"
#include "velocityactual.h"
#include "pathdesired.h"

typedef struct  {
	float total_energy_error;
	float calibrated_airspeed_error;

	float line_error;
	float circle_error;
} Integral;

//! Taken from "Small Unmanned Aircraft-- Theory and Practice"
float simple_line_follower(PositionActualData *positionActual, PathDesiredData *pathDesired, float chi_inf, float k_path, float k_psi_int, float delT, Integral *integral);
float simple_arc_follower(PositionActualData *positionActual, float c[2], float rho, float curvature, float k_orbit, float k_psi_int, float delT, Integral *integral);

//! Taken from "Fixed Wing UAV Path Following in Wind with Input Constraints"
float roll_limited_line_follower(PositionActualData *positionActual, VelocityActualData *velocityActual, PathDesiredData *pathDesired,
								  float true_airspeed, float true_airspeed_desired,
								  float headingActual_R, float gamma_max, float phi_max);
float roll_limited_arc_follower(PositionActualData *positionActual, VelocityActualData *velocityActual,
									  float arc_center_NED[2], float curvature, float arc_radius,
									  float true_airspeed, float true_airspeed_desired,
								      float headingActual_R, float gamma_max, float phi_max);



#endif /* PATH_FOLLOWERS_H */
