/*
 * Copyright (C) 2015 Hann Woei Ho
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/computer_vision/opticflow/landing_SSL.c
 * @brief Optical-flow landing using flatness from optical flow/ Self-supervised model
 *
 * Control loops for optic flow based landing.
 */

// Own Header
#include "landing_SSL.h"
#include "state.h"
// Landing
//#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
//#include "firmwares/rotorcraft/guidance/guidance_v.h"
//#include "autopilot.h"
#include "subsystems/datalink/downlink.h"
//
// waypoint
#include "navigation.h"
#include "generated/flight_plan.h"
#define win_3D 300
struct EnuCoor_i waypoints_3D[win_3D];
struct EnuCoor_i waypoints_Distribution;
uint32_t buf_point_3D, stay_waypoint_3D, land_safe_count, max_safe, active_3D, land_distribution_count, land_distribution;
/**
 * Horizontal guidance mode enter resets the errors
 * and starts the controller.
 */

void guidance_v_module_enter(void)
{

}

/**
 * Read the RC commands
 */
void guidance_v_module_read_rc(void)
{

}

/**
 * Main guidance loop
 * @param[in] in_flight Whether we are in flight or not
 */

void guidance_v_module_run(bool_t in_flight)
{

}

/**
 * Update the controls based on a vision result
 * @param[in] *result The opticflow calculation result used for control
 */
void landing_SSL_init(void)
{
	//waypoints
	buf_point_3D = 0;
	stay_waypoint_3D = 0;
	land_safe_count = 0;
	max_safe = 0;
	active_3D = 0;
	land_distribution_count = 0;
	land_distribution = 0;
}

void landing_SSL_update(struct opticflow_result_t *result,  struct opticflow_state_t *opticflow_state)
{
	// *********************************************
	// move way point to the safe location
	// *********************************************

//	if(result->USE_SSL)
//	{
//		if((!stay_waypoint_3D) && ((abs(stateGetSpeedEnu_i()->x) < SPEED_BFP_OF_REAL(0.1)) && (abs(stateGetSpeedEnu_i()->y) < SPEED_BFP_OF_REAL(0.1))))
//		{
//			if(land_distribution)
//			{
//				land_distribution_count++;
//			}
//			else
//			{
//				land_distribution_count = 0;
//			}
//
//			if(land_distribution_count > 120) // stay there 2 seconds
//			{
//				waypoints_Distribution.x = stateGetPositionEnu_i()->x;
//				waypoints_Distribution.y = stateGetPositionEnu_i()->y;
//				waypoints_Distribution.z = stateGetPositionEnu_i()->z;
//				nav_move_waypoint_enu_i(WP_safe, &waypoints_Distribution);
//			}
//		}
//		else
//		{
//			land_distribution_count = 0;
//		}
//	}
//	else
//	{
//		if((!stay_waypoint_3D) && ((abs(stateGetSpeedEnu_i()->x) > SPEED_BFP_OF_REAL(0.3)) || (abs(stateGetSpeedEnu_i()->y) > SPEED_BFP_OF_REAL(0.3))))
//		{
//			active_3D = 1;
//			if(result->land_safe == 1)
//			{
//				waypoints_3D[buf_point_3D].x = stateGetPositionEnu_i()->x;
//				waypoints_3D[buf_point_3D].y = stateGetPositionEnu_i()->y;
//				waypoints_3D[buf_point_3D].z = stateGetPositionEnu_i()->z;
//				buf_point_3D = (buf_point_3D+1) %win_3D; // index starts from 0 to mov_block
//
//				land_safe_count ++;
//			}
//			else
//			{
//				if(land_safe_count > max_safe) //land with the largest possibility of safe region
//				{
//					max_safe = land_safe_count;
//					if (buf_point_3D > 2) nav_move_waypoint_enu_i(WP_safe, &waypoints_3D[buf_point_3D/2]); // save the waypoint having the minimum 3D value
//				}
//				land_safe_count = 0;
//				buf_point_3D = 0;
//			}
//		}
//		else
//		{
//			active_3D = 0;
//			if(land_safe_count > max_safe)
//			{
//				max_safe = land_safe_count;
//				if (buf_point_3D > 2) nav_move_waypoint_enu_i(WP_safe, &waypoints_3D[buf_point_3D/2]); // save the waypoint having the minimum 3D value
//			}
//			land_safe_count = 0;
//			buf_point_3D = 0;
//		}
//		result->land_safe_count = land_safe_count;
//		result->active_3D = active_3D;
//	}
}
