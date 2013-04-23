/**
 ******************************************************************************
 * @file       math_misc.h
 * @author     PhoenixPilot, http://github.com/PhoenixPilot, Copyright (C) 2012
 * @addtogroup OpenPilot Math Utilities
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

#ifndef MISC_MATH_H
#define MISC_MATH_H

#include <pios.h>

typedef enum CenterCalculationResult {CENTER_FOUND, COINCIDENT_POINTS, INSUFFICIENT_RADIUS} CenterCalculationResult;

//! This is but one definition of sign(.)
#define sign(x) (x < 0 ? -1 : 1)

//! Bound input value within range (plus or minus)
float bound_sym(float val, float range);

//! Bound input value between min and max
float bound_min_max(float val, float min, float max);

CenterCalculationResult arcCenterFromTwoPointsAndRadiusAndArcRank(float *start_point, float *end_point, float radius, float *center, bool clockwise, bool minor);

//! Measure angle between two points on a circle
float updateArcMeasure(float oldPosition_NE[2], float newPosition_NE[2], float arcCenter_NE[2]);

//! Measure angle between two 2d vectors
float angle_between_2d_vectors(float a[2], float b[2]);

#endif /* MISC_MATH_H */
