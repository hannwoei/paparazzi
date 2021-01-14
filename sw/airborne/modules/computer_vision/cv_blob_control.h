/*
 * Copyright (C) C. De Wagter
 * 				 H. W. Ho
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/computer_vision/cv_blob_control.h"
 * @author H. W. Ho
 * Geo-reference computer vision detections
 */

#ifndef CV_BLOB_CONTROL_H
#define CV_BLOB_CONTROL_H

#include "std.h"
#include <stdint.h>
#include "math/pprz_algebra_int.h"

extern int32_t focus_length;

extern void blob_control_init(void);
extern void blob_control_run(void);

struct camera_frame_t {
  int32_t w;     ///< Frame width [px]
  int32_t h;     ///< Frame height [px]
  int32_t f;     ///< Camera Focal length in [px]
  int32_t px;    ///< Target pixel coordinate (left = 0)
  int32_t py;    ///< Target pixel coordinate (top = 0)
};

/* The blob stabilization */
struct blob_stab_t {
  int32_t phi_pgain;        ///< The roll P gain on the err_vx
  int32_t phi_igain;        ///< The roll I gain on the err_vx_int
  int32_t theta_pgain;      ///< The pitch P gain on the err_vy
  int32_t theta_igain;      ///< The pitch I gain on the err_vy_int
  float desired_x;         ///< The desired velocity in the x direction (cm/s)
  float desired_y;         ///< The desired velocity in the y direction (cm/s)

  float err_x_int;         ///< The integrated velocity error in x direction (m/s)
  float err_y_int;         ///< The integrated velocity error in y direction (m/s)
  struct Int32Eulers cmd;   ///< The commands that are send to the hover loop
};
extern struct blob_stab_t blob_stab;

void blob_control(struct camera_frame_t *tar);
//void blob_control_filter(bool kalman, int wp, int length);

// Implement own Horizontal loops
extern void guidance_h_module_init(void);
extern void guidance_h_module_enter(void);
extern void guidance_h_module_read_rc(void);
extern void guidance_h_module_run(bool in_flight);

#endif

