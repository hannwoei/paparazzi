/*
 * Copyright (C) 2015 The Paparazzi Community
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
 * @file modules/computer_vision/opticflow/inter_thread_data.h
 * @brief Inter-thread data structures.
 *
 * Data structures used to for inter-thread communication via Unix Domain sockets.
 */


#ifndef _INTER_THREAD_DATA_H
#define _INTER_THREAD_DATA_H

/* The result calculated from the opticflow */
struct opticflow_result_t {
  float fps;              ///< Frames per second of the optical flow calculation
  uint16_t corner_cnt;    ///< The amount of coners found by FAST9
  uint16_t tracked_cnt;   ///< The amount of tracked corners

  int16_t flow_x;         ///< Flow in x direction from the camera (in subpixels)
  int16_t flow_y;         ///< Flow in y direction from the camera (in subpixels)
  int16_t flow_der_x;     ///< The derotated flow calculation in the x direction (in subpixels)
  int16_t flow_der_y;     ///< The derotated flow calculation in the y direction (in subpixels)

  float vel_x;            ///< The velocity in the x direction
  float vel_y;            ///< The velocity in the y direction

  float divergence;       ///< Divergence
  float Div_f;			  ///< Filtered Divergence
  float Div_d;			  ///< Derivatives of Divergence
  float TTI;			  ///< Time-to-contact
  float flatness;		  ///< Flatness from OF
  float flatness_SSL;	  ///< flatness from SSL
  float zx;
  float zy;
  float d_heading;
  float d_pitch;
  float min_error;
  int16_t n_inlier;
  int16_t fit_uncertainty;

  uint8_t USE_VISION_METHOD;
  uint8_t land_safe;
  uint32_t land_safe_count;
  uint32_t active_3D;
#ifdef SUB_IMG
  uint8_t in_sub_min;
  float sub_min;
  float sub_flatness[9]; // set a maximum of 9 sub-images
#else
#ifdef DOWNLINK_DISTRIBUTIONS
//  float *texton;
  float texton[30]; // change the size according to number of words
#endif
#endif
};

/* The state of the drone when it took an image */
struct opticflow_state_t {
  float phi;      ///< roll [rad]
  float theta;    ///< pitch [rad]
  float psi;	  ///< yaw [rad]
  float agl;      ///< height above ground [m]
  float V_body_x; ///< body velocity x,y,z [m/s]
  float V_body_y;
  float V_body_z;
  float gps_x;    ///<  x_ENU from GPS [m]
  float gps_y;    ///<  y_ENU from GPS [m]
  float gps_z;    ///<  z_ENU from GPS [m]
};

#endif
