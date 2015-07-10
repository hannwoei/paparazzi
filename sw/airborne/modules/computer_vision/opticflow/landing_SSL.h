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
 * @file modules/computer_vision/opticflow/landing_SSL.h
 * @brief Optical-flow landing using flatness from optical flow/ Self-supervised model
 *
 * Control loops for optic flow based landing.
 */

#ifndef CV_LANDING_SSL_V_
#define CV_LANDING_SSL_V_

#include "std.h"
#include "lib/v4l/v4l2.h"
#include "inter_thread_data.h"
#include "math/pprz_algebra_int.h"

// Implement own Horizontal loops
extern void guidance_v_module_enter(void);
extern void guidance_v_module_read_rc(void);
extern void guidance_v_module_run(bool_t in_flight);

// Update the stabiliztion commands based on a vision result
extern void landing_SSL_init(void);
void landing_SSL_update(struct opticflow_result_t *result,  struct opticflow_state_t *opticflow_state);

extern unsigned int stay_waypoint_3D;
extern unsigned int land_distribution;

#endif /* CV_LANDING_SSL_V_ */
