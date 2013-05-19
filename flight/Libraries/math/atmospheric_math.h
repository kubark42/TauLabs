/**
 ******************************************************************************
 * @file       atmospheric_math.h
 * @author     Tau Labs, http://www.taulabs.org, Copyright (C) 2013
 * @addtogroup Math Utilities
 * @{
 * @addtogroup MiscellaneousMath Math Various mathematical routines
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

#ifndef ATMOSPHERIC_MATH_H
#define ATMOSPHERIC_MATH_H

#include "openpilot.h"

typedef struct
{
    float air_density_at_surface;
    float air_temperature_at_surface;
    float sea_level_press;
    float temperature_lapse_rate;
    float univ_gas_constant;
    float dry_air_constant;
    float relative_humidity; //[%]
    float M; //Molar mass
} AirParameters;

float air_density_from_altitude(float altitude, AirParameters *air);
float air_pressure_from_altitude(float altitude, AirParameters *air);
float cas2tas(float CAS, float altitude, AirParameters *air);
float tas2cas(float TAS, float altitude, AirParameters *air);

AirParameters initialize_air_structure();

#endif /* ATMOSPHERIC_MATH_H */
