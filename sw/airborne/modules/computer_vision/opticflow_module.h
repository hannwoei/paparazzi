/*
 * Copyright (C) 2014 Hann Woei Ho
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
 * @file modules/computer_vision/opticflow_module.h
 * @brief optical-flow based hovering for Parrot AR.Drone 2.0
 *
 * Sensors from vertical camera and IMU of Parrot AR.Drone 2.0
 */

#ifndef OPTICFLOW_MODULE_H
#define OPTICFLOW_MODULE_H

// Include opticflow calculator and stabilization loops
#include "opticflow/opticflow_calculator.h"
//#include "opticflow/stabilization_opticflow.h"
#include "opticflow/landing_DivPilot.h"

// Needed for settings
extern struct opticflow_t opticflow;
// Main viewvideo structure
struct logvideo_t {
//  volatile bool_t is_streaming;   ///< When the device is streaming
//  struct v4l2_device *dev;        ///< The V4L2 device that is used for the video stream
//  uint8_t downsize_factor;        ///< Downsize factor during the stream
//  uint8_t quality_factor;         ///< Quality factor during the stream
//  uint8_t fps;                    ///< The amount of frames per second

  volatile bool_t take_shot;      ///< Wether to take an image
  uint16_t shot_number;           ///< The last shot number
};
extern struct logvideo_t logvideo;

struct logvideo_data_t{
	uint16_t corner_cnt;    ///< The amount of coners found by FAST9
	uint16_t tracked_cnt;	///< no. of tracked corners
	float FPS;				///< Frame rate
	float flatness;			///< flatness from OF
	float flatness_SSL;		///< flatness from SSL
	float div;              ///< Divergence from OF
	float phi;      		///< roll [rad]
	float theta;    		///< pitch [rad]
	float psi;	  			///< yaw [rad]
	float agl;      		///< height above ground [m]
	float V_body_x; 		///< body velocity x [m/s]
	float V_body_y;			///< body velocity y [m/s]
	float V_body_z;			///< body velocity z [m/s]
	float gps_x;    		///< x_ENU from GPS [m]
	float gps_y;    		///< y_ENU from GPS [m]
	float gps_z;    		///< z_ENU from GPS [m]
	float A_body_x; 		///< body acc x [m2/s]
	float A_body_y;			///< body acc y [m2/s]
	float A_body_z;			///< body acc z [m2/s]
};
extern struct logvideo_data_t logvideo_data;
// Module functions
extern void opticflow_module_init(void);
extern void opticflow_module_run(void);
extern void opticflow_module_start(void);
extern void opticflow_module_stop(void);
extern void log_video_start(bool_t take);

#endif /* OPTICFLOW_MODULE_H */
