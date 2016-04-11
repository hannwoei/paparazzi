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
 * @file modules/divergence_landing/divergence_landing.h
 * @Vertical landing using divergence
 */

#ifndef DIVERGENCE_LANDING_H_
#define DIVERGENCE_LANDING_H_

#include "std.h"

/* The divergence landing */
struct Div_landing_t {
  float div_pgain;        ///< The divergence P gain on the err_div
  float div_igain;        ///< The divergence I gain on the err_div_int
  float div_dgain;        ///< The divergence D gain on the err_div_int
  float nominal_throttle; ///< The nominal throttle
  float desired_div;      ///< The desired divergence
  int32_t controller;	  ///< The controller switch
  float div_cov;
  float div;              ///< The divergence
  float div_f;            ///< The filtered divergence (low-pass)
  float ground_div;       ///< The ground divergence
  float agl;
  float gps_z;
  float vel_z;
  float accel_z;
  float z_sp;
  float err;
  float z_sum_err;
  int32_t thrust;
  float fps;
};
extern struct Div_landing_t Div_landing;

// use the hover mode for horizontal control
#define GUIDANCE_H_MODE_MODULE_SETTING GUIDANCE_H_MODE_HOVER

// use guidance from module
#define GUIDANCE_V_MODE_MODULE_SETTING GUIDANCE_V_MODE_MODULE

// use vertical guidance loops from module
extern void guidance_v_module_init(void);
extern void guidance_v_module_enter(void);
extern void guidance_v_module_run(bool_t in_flight);

#endif /* DIVERGENCE_LANDING_H_ */
