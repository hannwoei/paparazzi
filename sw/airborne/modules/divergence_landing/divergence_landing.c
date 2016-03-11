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
 * @file modules/divergence_landing/divergence_landing.c
 * @Vertical landing using divergence
 *
 */

#include "modules/divergence_landing/divergence_landing.h"

#include "generated/airframe.h"
#include "paparazzi.h"
#include "subsystems/abi.h"
#include "firmwares/rotorcraft/stabilization.h"

#ifndef VISION_DIV_PGAIN
#define VISION_DIV_PGAIN 283
#endif
PRINT_CONFIG_VAR(VISION_DIV_PGAIN)

#ifndef VISION_DIV_IGAIN
#define VISION_DIV_IGAIN 13
#endif
PRINT_CONFIG_VAR(VISION_DIV_IGAIN)

#ifndef VISION_DIV_DGAIN
#define VISION_DIV_DGAIN 82
#endif
PRINT_CONFIG_VAR(VISION_DIV_DGAIN)

#ifndef VISION_NOMINAL_THROTTLE
#define VISION_NOMINAL_THROTTLE 0.655
#endif
PRINT_CONFIG_VAR(VISION_NOMINAL_THROTTLE)

#ifndef VISION_DESIRED_DIV
#define VISION_DESIRED_DIV 1
#endif
PRINT_CONFIG_VAR(VISION_DESIRED_DIV)

#ifndef VISION_CONTROLLER
#define VISION_CONTROLLER 1
#endif
PRINT_CONFIG_VAR(VISION_CONTROLLER)

#ifndef LANDING_AGL_ID
#define LANDING_AGL_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(LANDING_AGL_ID)

/* Check the control gains */
#if (VISION_DIV_PGAIN < 0)      ||  \
	(VISION_DIV_DGAIN < 0)      ||  \
    (VISION_DIV_IGAIN < 0)
#error "ALL control gains have to be positive!!!"
#endif

struct Div_landing_t Div_landing;

static abi_event agl_ev; ///< The altitude ABI event

/// Callback function of the ground altitude
static void landing_agl_cb(uint8_t sender_id __attribute__((unused)), float distance);


void divergence_landing_init(void);
void divergence_landing_run(bool_t in_flight);

void divergence_landing_init(void)
{
	/* Initialize the default gains and settings */
	Div_landing.div_pgain = VISION_DIV_PGAIN;
	Div_landing.div_igain = VISION_DIV_IGAIN;
	Div_landing.div_dgain = VISION_DIV_DGAIN;
	Div_landing.desired_div = VISION_DESIRED_DIV;
	Div_landing.nominal_throttle = VISION_NOMINAL_THROTTLE;
	Div_landing.controller = VISION_CONTROLLER;
	Div_landing.div_cov = 0.0;

	Div_landing.agl = 0.0f;
	Div_landing.z_sp = 1.0f;
	Div_landing.z_sum_err = 0.0f;

  // Subscribe to the altitude above ground level ABI messages
  AbiBindMsgAGL(LANDING_AGL_ID, &agl_ev, landing_agl_cb);
}


void divergence_landing_run(bool_t in_flight)
{
  if (!in_flight) {
    // Reset integrators
	Div_landing.z_sum_err = 0;
    stabilization_cmd[COMMAND_THRUST] = 0;
  } else {
    int32_t nominal_throttle = Div_landing.nominal_throttle * MAX_PPRZ;
    float err = Div_landing.z_sp - Div_landing.agl;
    int32_t thrust = nominal_throttle + Div_landing.div_pgain * err + Div_landing.div_igain * Div_landing.z_sum_err;
    Bound(thrust, 0, MAX_PPRZ);
    stabilization_cmd[COMMAND_THRUST] = thrust;
    Div_landing.z_sum_err += err;
  }
}

static void landing_agl_cb(uint8_t sender_id, float distance)
{
	Div_landing.agl = distance;
}

// vertical guidance from module
void guidance_v_module_init(void)
{
  divergence_landing_init();
}

void guidance_v_module_enter(void)
{
  // reset integrator
	Div_landing.z_sum_err = 0.0f;
}

void guidance_v_module_run(bool_t in_flight)
{
  divergence_landing_run(in_flight);
}
