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
 * @file modules/computer_vision/opticflow/landing_DivPilot.h
 * @brief Optical-flow based control for Linux based systems
 *
 * Control loops for optic flow based landing.
 */

#ifndef CV_LANDING_DIVPILOT_V_
#define CV_LANDING_DIVPILOT_V_

#include "std.h"
#include "lib/v4l/v4l2.h"
#include "inter_thread_data.h"
#include "math/pprz_algebra_int.h"

/* The opticflow stabilization */
struct DivPilot_landing_t {
  int32_t div_pgain;        ///< The divergence P gain on the err_div
  int32_t div_igain;        ///< The divergence I gain on the err_div_int
  float desired_div;        ///< The desired divergence
  int32_t controller;		///< The controller switch

  float err_div;
  float err_div_int;        ///< The integrated divergence error
  int32_t div_thrust;       ///< The commands that are send to thrust
  int32_t div_thrust_int;       ///< Hover thrust

  float div_cov;
};
extern struct DivPilot_landing_t DivPilot_landing;

// Implement own Horizontal loops
extern void guidance_v_module_enter(void);
extern void guidance_v_module_read_rc(void);
extern void guidance_v_module_run(bool_t in_flight);

// Update the stabiliztion commands based on a vision result
void landing_DivPilot_update(struct opticflow_result_t *result,  struct opticflow_state_t *opticflow_state);

#endif /* CV_LANDING_DIVPILOT_V_ */
