/**
 ******************************************************************************
 * @file       math_misc.c
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

#include "misc_math.h"

/**
 * Bound input value between min and max
 */
float bound_min_max(float val, float min, float max)
{
	if (val < min)
		return min;
	if (val > max)
		return max;
	return val;
}

/**
 * Bound input value within range (plus or minus)
 */
float bound_sym(float val, float range)
{
	if(val < -range) {
		val = -range;
	} else if(val > range) {
		val = range;
	}
	return val;
}


/**
 * @brief Compute the center of curvature of the arc, by calculating the intersection
 * of the two circles of radius R around the two points. Inspired by
 * http://www.mathworks.com/matlabcentral/newsreader/view_thread/255121
 * @param[in] start_point Starting point, in North-East coordinates
 * @param[in] end_point Ending point, in North-East coordinates
 * @param[in] radius Radius of the curve segment
 * @param[in] clockwise true if clockwise is the positive sense of the arc, false if otherwise
 * @param[in] minor true if minor arc, false if major arc
 * @param[out] center Center of circle formed by two points, in North-East coordinates
 */
void arcCenterFromTwoPointsAndRadiusAndArcRank(float *start_point,
	                   float *end_point,
	                   float radius,
	                   float *center,
					   bool clockwise,
					   bool minor
					   )
{
	float m_n, m_e, p_n, p_e, d;

	// Center between start and end
	m_n = (start_point[0] + end_point[0]) / 2;
	m_e = (start_point[1] + end_point[1]) / 2;

	// Normal vector to the line between start and end points
	if ((clockwise == true && minor == true) ||
			(clockwise == false && minor == false)) { //clockwise minor OR counterclockwise major
		p_n = -(end_point[1] - start_point[1]);
		p_e = (end_point[0] - start_point[0]);
	} else { //counterclockwise minor OR clockwise major
		p_n = (end_point[1] - start_point[1]);
		p_e = -(end_point[0] - start_point[0]);
	}

	// Work out how far to go along the perpendicular bisector
	d = sqrtf(radius * radius / (p_n * p_n + p_e * p_e) - 0.25f);

	float radius_sign = (radius > 0) ? 1 : -1;
	radius = fabs(radius);

	if (fabs(p_n) < 1e-3 && fabs(p_e) < 1e-3) {
		center[0] = m_n;
		center[1] = m_e;
	} else {
		center[0] = m_n + p_n * d * radius_sign;
		center[1] = m_e + p_e * d * radius_sign;
	}
}
