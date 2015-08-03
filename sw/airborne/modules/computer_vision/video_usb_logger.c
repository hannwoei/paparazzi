/*
 * Copyright (C) 2015 The Paparazzi Community
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** @file modules/computer_vision/video_usb_logger.c
 */

#include "video_usb_logger.h"

#include <stdio.h>
#include "state.h"
#include"SSL_module.h"
//#include "viewvideo.h"

/** Set the default File logger path to the USB drive */
#ifndef VIDEO_USB_LOGGER_PATH
#define VIDEO_USB_LOGGER_PATH "/data/video/usb/"
#endif

/** The file pointer */
static FILE *video_usb_logger = NULL;

/** Start the file logger and open a new file */
void video_usb_logger_start(void)
{
//  uint32_t counter = 0;
//  char filename[512];
//
//  // Check for available files
//  sprintf(filename, "%s%05d.csv", VIDEO_USB_LOGGER_PATH, counter);
//  while ((video_usb_logger = fopen(filename, "r"))) {
//    fclose(video_usb_logger);
//
//    counter++;
//    sprintf(filename, "%s%05d.csv", VIDEO_USB_LOGGER_PATH, counter);
//  }
//
//  video_usb_logger = fopen(filename, "w");
//
//  if (video_usb_logger != NULL) {
//    fprintf(video_usb_logger, "counter,image,FPS,vx,vy,vz,sonar,flat,flat2,x,y,z,roll,pitch,yaw,corner,flow\n");
//  }
}

/** Stop the logger an nicely close the file */
void video_usb_logger_stop(void)
{
//  if (video_usb_logger != NULL) {
//    fclose(video_usb_logger);
//    video_usb_logger = NULL;
//  }
}

/** Log the values to a csv file */
void video_usb_logger_periodic(void)
{
//  if (video_usb_logger == NULL) {
//    return;
//  }
//  static uint32_t counter = 0;
////  struct NedCoor_i *ned = stateGetPositionNed_i();
////  struct Int32Eulers *euler = stateGetNedToBodyEulers_i();
////  static uint32_t sonar = 0;
//
//  // Take a new shot
  log_video_start(TRUE);
//
//  // Save to the file
//  fprintf(video_usb_logger, "%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d\n", counter,
//          logvideo.shot_number, logvideo_data.FPS, logvideo_data.V_body_x, logvideo_data.V_body_y,
//		  logvideo_data.V_body_z, logvideo_data.agl, logvideo_data.flatness,
//		  logvideo_data.flatness_SSL, logvideo_data.gps_x, logvideo_data.gps_y,
//		  logvideo_data.gps_z, logvideo_data.phi, logvideo_data.theta,
//		  logvideo_data.psi, logvideo_data.corner_cnt, logvideo_data.tracked_cnt);
//  counter++;
}
