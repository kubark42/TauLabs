/**
 ******************************************************************************
 * @addtogroup TauLabsLibraries Tau Labs Libraries
 * @{
 * @file       controlcommand_agent.c
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2013
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

#include "openpilot.h"
#include "controlcommand_agent.h"
#include "flightstatus.h"


/**
 * Set the error code and alarm state
 * @param[in] error code
 */
bool do_I_have_the_ball(int flightMode, enum controlcommand_agent possessor)
{
	// Get the severity of the alarm given the error code
	switch(flightMode) {
	case FLIGHTSTATUS_FLIGHTMODE_MANUAL:
	case FLIGHTSTATUS_FLIGHTMODE_STABILIZED1:
	case FLIGHTSTATUS_FLIGHTMODE_STABILIZED2:
	case FLIGHTSTATUS_FLIGHTMODE_STABILIZED3:
	case FLIGHTSTATUS_FLIGHTMODE_AUTOTUNE:
	case FLIGHTSTATUS_FLIGHTMODE_ALTITUDEHOLD:
	case FLIGHTSTATUS_FLIGHTMODE_POSITIONHOLD:
	case FLIGHTSTATUS_FLIGHTMODE_RETURNTOHOME:
	case FLIGHTSTATUS_FLIGHTMODE_PATHPLANNER:
		if (possessor == AGENT_MANUALCONTROL)
			return true;
		else
			return false;
		break;
	default:
		return false;
		break;
	}
}

/**
 * @}
 */
