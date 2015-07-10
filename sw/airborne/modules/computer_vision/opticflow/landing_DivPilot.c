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
 * @file modules/computer_vision/opticflow/landing_DivPilot.c
 * @brief Optical-flow based control for Linux based systems
 *
 * Control loops for optic flow based landing.
 */

// Own Header
#include "landing_DivPilot.h"

// Stabilization
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "autopilot.h"
#include "subsystems/datalink/downlink.h"
#include "lib/vision/opticflow_fitting.h"

#define CMD_OF_SAT  1500 // 40 deg = 2859.1851

#define CMD_DIV 1500

#ifndef VISION_DIV_PGAIN
#define VISION_DIV_PGAIN 10
#endif
PRINT_CONFIG_VAR(VISION_DIV_PGAIN)

#ifndef VISION_DIV_IGAIN
#define VISION_DIV_IGAIN 0
#endif
PRINT_CONFIG_VAR(VISION_DIV_IGAIN)

#ifndef VISION_DESIRED_DIV
#define VISION_DESIRED_DIV 1
#endif
PRINT_CONFIG_VAR(VISION_DESIRED_DIV)

#ifndef VISION_CONTROLLER
#define VISION_CONTROLLER 1
#endif
PRINT_CONFIG_VAR(VISION_CONTROLLER)

/* Check the control gains */
#if (VISION_DIV_PGAIN < 0)      ||  \
  (VISION_DIV_IGAIN < 0)
#error "ALL control gains have to be positive!!!"
#endif

/* Initialize the default gains and settings */
struct DivPilot_landing_t DivPilot_landing = {
  .div_pgain = VISION_DIV_PGAIN,
  .div_igain = VISION_DIV_IGAIN,
  .desired_div = VISION_DESIRED_DIV,
  .controller = VISION_CONTROLLER
};

/**
 * Horizontal guidance mode enter resets the errors
 * and starts the controller.
 */

#define NO_BUF 10
float buf_div[NO_BUF], buf_thrust[NO_BUF];
uint8_t n_buf;

void guidance_v_module_enter(void)
{
  /* Reset the integrated errors */
  DivPilot_landing.err_div_int = 0;

  /* Set div thrust to 0 */
  DivPilot_landing.div_thrust_int = stabilization_cmd[COMMAND_THRUST];

  DivPilot_landing.div_thrust = DivPilot_landing.div_thrust_int;

  n_buf = 0;
  DivPilot_landing.div_cov = 0.0;
}

/**
 * Read the RC commands
 */
void guidance_v_module_read_rc(void)
{
  // TODO: change the desired vx/vy
}

/**
 * Main guidance loop
 * @param[in] in_flight Whether we are in flight or not
 */

void guidance_v_module_run(bool_t in_flight)
{
  stabilization_cmd[COMMAND_THRUST] = DivPilot_landing.div_thrust; //trim: guidance_v_delta_t
}

/**
 * Update the controls based on a vision result
 * @param[in] *result The opticflow calculation result used for control
 */


void landing_DivPilot_update(struct opticflow_result_t *result,  struct opticflow_state_t *opticflow_state)
{
  /* Check if we are in the correct AP_MODE before setting commands */
  if (autopilot_mode != AP_MODE_MODULE) {
    return;
  }

  // Force landing
//  if (result->Div_f<-5)
//  {
//	  DivPilot_landing.desired_div = 6.0;
//  }

  /* Calculate the error if we have enough flow */
  DivPilot_landing.err_div = 0;

  if (result->tracked_cnt > 3) {
//    err_div = (-DivPilot_landing.desired_div/3 - result->divergence);
	  DivPilot_landing.err_div = (-DivPilot_landing.desired_div/3 - result->Div_f);
  }

//  /* Calculate the integrated errors (TODO: bound??) */
  DivPilot_landing.err_div_int += DivPilot_landing.err_div;

  /* Calculate the commands */
  // PD
//  DivPilot_landing.div_thrust = (int32_t) (DivPilot_landing.div_thrust_int + (DivPilot_landing.div_pgain*err_div*10 +
//		  DivPilot_landing.div_igain*result->Div_d));
  // PI
//  DivPilot_landing.div_thrust = (int32_t) (DivPilot_landing.div_thrust_int + (DivPilot_landing.div_pgain*err_div*10 +
//		  DivPilot_landing.div_igain*DivPilot_landing.err_div_int));
  DivPilot_landing.div_thrust = (int32_t) (DivPilot_landing.div_thrust + DivPilot_landing.div_pgain*DivPilot_landing.err_div);
	// **********************************************************************************************************************
	// Oscillation Detection
	// **********************************************************************************************************************

	buf_div[n_buf] = result->Div_f;
	buf_thrust[n_buf] = (float) DivPilot_landing.div_thrust;
	DivPilot_landing.div_cov = CalcCov(buf_div,buf_thrust,n_buf+1);
	n_buf = (n_buf+1) %NO_BUF;

	if (DivPilot_landing.div_cov < -10 && DivPilot_landing.controller == 1)
	{
//		DivPilot_landing.div_pgain = DivPilot_landing.div_pgain/5;
		DivPilot_landing.desired_div = DivPilot_landing.desired_div*10;
		DivPilot_landing.controller = 0;
	}

//  Bound(DivPilot_landing.div_thrust,-100, 100);

//  DivPilot_landing.div_thrust = (int32_t) ((DivPilot_landing.div_pgain*err_div
//								  +DivPilot_landing.div_igain * DivPilot_landing.err_div_int))*CMD_DIV;

  /* Bound the roll and pitch commands */
  //Bound(DivPilot_landing.div_thrust,-CMD_DIV, CMD_DIV);
}
