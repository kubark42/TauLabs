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


/**
 * Set the error code and alarm state
 * @param[in] error code
 */
bool do_I_have_the_ball(enum controlcommand_agent possessor)
{
	// Get the severity of the alarm given the error code
	switch (possessor) {
	case AGENT_MANUALCONTROL:
		return true;
		break;
	default:
		return false;
		break;
	}
}

/**
 * @}
 */
