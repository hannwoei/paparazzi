/*
* Copyright (C) 2015
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
* along with paparazzi; see the file COPYING.  If not, see
* <http://www.gnu.org/licenses/>.
*
*/

/**
 * @file modules/computer_vision/video_thread.c
 */

// Own header
#include "modules/computer_vision/video_thread.h"
#include "subsystems/abi.h"
#include "state.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <math.h>

// Video
#include "lib/v4l/v4l2.h"
#include "lib/vision/image.h"
#include "lib/vision/bayer.h"
#include "lib/encoding/jpeg.h"
#include "peripherals/video_device.h"

#include "mcu_periph/sys_time.h"

// include board for bottom_camera and front_camera on ARDrone2 and Bebop
#include BOARD_CONFIG

#if JPEG_WITH_EXIF_HEADER
#include "lib/exif/exif_module.h"
#endif

// Threaded computer vision
#include <pthread.h>
#include "rt_priority.h"

/// The camera video config (usually bottom_camera or front_camera)
#ifndef VIDEO_THREAD_CAMERA
#warning "Are you sure you don't want to use the bottom_camera or front_camera?"
// The video device buffers (the amount of V4L2 buffers)
#ifndef VIDEO_THREAD_DEVICE_BUFFERS
#define VIDEO_THREAD_DEVICE_BUFFERS 10
#endif
PRINT_CONFIG_VAR(VIDEO_THREAD_DEVICE_BUFFERS)
#ifndef VIDEO_THREAD_SUBDEV
#define VIDEO_THREAD_SUBDEV NULL
#endif
#ifndef VIDEO_THREAD_FILTERS
#define VIDEO_THREAD_FILTERS 0
#endif
#ifndef VIDEO_THREAD_FORMAT
#define VIDEO_THREAD_FORMAT V4L2_PIX_FMT_UYVY
#endif
struct video_config_t custom_camera = {
  .w = VIDEO_THREAD_VIDEO_WIDTH,
  .h = VIDEO_THREAD_VIDEO_HEIGHT,
  .dev_name = STRINGIFY(VIDEO_THREAD_DEVICE),
  .subdev_name = VIDEO_THREAD_SUBDEV,
  .buf_cnt = VIDEO_THREAD_DEVICE_BUFFERS,
  .filters = VIDEO_THREAD_FILTERS,
  .format = VIDEO_THREAD_FORMAT
};
#define VIDEO_THREAD_CAMERA custom_camera
#endif
PRINT_CONFIG_VAR(VIDEO_THREAD_CAMERA)


// Frames Per Seconds
#ifndef VIDEO_THREAD_FPS
#define VIDEO_THREAD_FPS 4
#endif
PRINT_CONFIG_VAR(VIDEO_THREAD_FPS)

// The place where the shots are saved (without slash on the end)
#ifndef VIDEO_THREAD_SHOT_PATH
#define VIDEO_THREAD_SHOT_PATH "/data/video/images"
#endif
PRINT_CONFIG_VAR(VIDEO_THREAD_SHOT_PATH)

/* Default sonar/agl to use in opticflow visual_estimator */
#ifndef OPTICFLOW_AGL_ID
#define OPTICFLOW_AGL_ID ABI_BROADCAST    ///< Default sonar/agl to use in opticflow visual_estimator
#endif
PRINT_CONFIG_VAR(OPTICFLOW_AGL_ID)

#ifndef OPTICFLOW_SENDER_ID
#define OPTICFLOW_SENDER_ID 1
#endif

/* The main opticflow variables */
struct opticflow_t opticflow_cv;                      ///< Opticflow calculations
static struct opticflow_result_t opticflow_result; ///< The opticflow result
static struct opticflow_state_t opticflow_state;   ///< State of the drone to communicate with the opticflow
static abi_event opticflow_agl_ev;                 ///< The altitude ABI event
static bool_t opticflow_got_result;                ///< When we have an optical flow calculation
static pthread_mutex_t opticflow_mutex;            ///< Mutex lock fo thread safety
struct FloatVect3 V_Ned, V_body;
struct FloatRMat Rmat_Ned2Body;
static uint16_t shot_number;

/* Static functions */
static void opticflow_agl_cb(uint8_t sender_id, float distance);  ///< Callback function of the ground altitude

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
/**
 * Send optical flow telemetry information
 * @param[in] *trans The transport structure to send the information over
 * @param[in] *dev The link to send the data over
 */
#ifdef SUB_IMG
static void SSL_SUB_telem_send(struct transport_tx *trans, struct link_device *dev)
{
  pthread_mutex_lock(&opticflow_mutex);
  pprz_msg_send_SSL_SUB(trans, dev, AC_ID,
                               &opticflow_result.fps,
							   &opticflow_result.sub_roughness[0],
							   &opticflow_result.sub_roughness[1],
							   &opticflow_result.sub_roughness[2],
							   &opticflow_result.sub_roughness[3],
							   &opticflow_result.sub_roughness[4],
							   &opticflow_result.sub_roughness[5],
							   &opticflow_result.sub_roughness[6],
							   &opticflow_result.sub_roughness[7],
							   &opticflow_result.sub_roughness[8],
							   &opticflow_result.sub_min,
							   &opticflow_result.in_sub_min,
                               &opticflow_state.V_body_x,
							   &opticflow_state.V_body_y,
                               &opticflow_state.V_body_z,
							   &opticflow_state.agl,
							   &opticflow_state.gps_x,
							   &opticflow_state.gps_y,
							   &opticflow_state.gps_z,
							   &opticflow_state.phi,
							   &opticflow_state.theta,
							   &opticflow_state.psi,
							   &opticflow_result.active_3D,
							   &opticflow_result.USE_VISION_METHOD,
							   &shot_number
  	  	  	  	  	  	  	   );
  pthread_mutex_unlock(&opticflow_mutex);
}
#else
static void opticflow_telem_send(struct transport_tx *trans, struct link_device *dev)
{
  pthread_mutex_lock(&opticflow_mutex);
  pprz_msg_send_OPTIC_FLOW_CV(trans, dev, AC_ID,
								&opticflow_result.fps,
								&opticflow_state.V_body_x,
								&opticflow_state.V_body_y,
								&opticflow_state.V_body_z,
								&opticflow_state.agl,
								&opticflow_result.surface_roughness,
								&opticflow_result.surface_roughness_SSL,
								&opticflow_state.gps_x,
								&opticflow_state.gps_y,
								&opticflow_state.gps_z,
								&opticflow_state.phi,
								&opticflow_state.theta,
								&opticflow_state.psi,
								&opticflow_result.corner_cnt,
								&opticflow_result.tracked_cnt,
								&opticflow_result.land_safe_count,
								&opticflow_result.active_3D,
								&opticflow_result.USE_VISION_METHOD,
								&shot_number,
                                &opticflow_result.flow_x,
                                &opticflow_result.flow_y,
								&opticflow_result.div_size,
                                &opticflow_result.divergence);
  pthread_mutex_unlock(&opticflow_mutex);
}
#endif
#ifdef DOWNLINK_DISTRIBUTIONS
static void SSL_telem_send(struct transport_tx *trans, struct link_device *dev)
{
  pthread_mutex_lock(&opticflow_mutex);
  pprz_msg_send_SSL_TEXTON(trans, dev, AC_ID,
                               &opticflow_result.flatness,
							   &opticflow_result.texton[0], &opticflow_result.texton[1], &opticflow_result.texton[2],
							   &opticflow_result.texton[3], &opticflow_result.texton[4], &opticflow_result.texton[5],
							   &opticflow_result.texton[6], &opticflow_result.texton[7], &opticflow_result.texton[8],
							   &opticflow_result.texton[9], &opticflow_result.texton[10], &opticflow_result.texton[11],
							   &opticflow_result.texton[12], &opticflow_result.texton[13], &opticflow_result.texton[14],
							   &opticflow_result.texton[15], &opticflow_result.texton[16], &opticflow_result.texton[17],
							   &opticflow_result.texton[18], &opticflow_result.texton[19], &opticflow_result.texton[20],
							   &opticflow_result.texton[21], &opticflow_result.texton[22], &opticflow_result.texton[23],
							   &opticflow_result.texton[24], &opticflow_result.texton[25], &opticflow_result.texton[26],
							   &opticflow_result.texton[27], &opticflow_result.texton[28], &opticflow_result.texton[29],
							   &shot_number
  	  	  	  	  	  	  	   );
  pthread_mutex_unlock(&opticflow_mutex);
}
#endif
#endif


// Main thread
static void *video_thread_function(void *data);
void video_thread_periodic(void) {
	  pthread_mutex_lock(&opticflow_mutex);
	  // Send Updated data to thread
	  // Send Updated data to thread
	  opticflow_state.phi = stateGetNedToBodyEulers_f()->phi;
	  opticflow_state.theta = stateGetNedToBodyEulers_f()->theta;
	  opticflow_state.psi = stateGetNedToBodyEulers_f()->psi;

	  // Compute body velocities from ENU
	  V_Ned.x = stateGetSpeedNed_f()->x;
	  V_Ned.y = stateGetSpeedNed_f()->y;
	  V_Ned.z = stateGetSpeedNed_f()->z;

	  struct FloatQuat* BodyQuaternions = stateGetNedToBodyQuat_f();
	  FLOAT_RMAT_OF_QUAT(Rmat_Ned2Body,*BodyQuaternions);
	  RMAT_VECT3_MUL(V_body, Rmat_Ned2Body, V_Ned);
	  opticflow_state.V_body_x = V_body.x;
	  opticflow_state.V_body_y = V_body.y;
	  opticflow_state.V_body_z = V_body.z;
	  opticflow_state.gps_x = stateGetPositionEnu_f()->x;
	  opticflow_state.gps_y = stateGetPositionEnu_f()->y;
	  opticflow_state.gps_z = stateGetPositionEnu_f()->z;

	  // Update the stabilization loops on the current calculation
	  if (opticflow_got_result) {
		landing_SSL_update(&opticflow_result, &opticflow_state);
	    opticflow_got_result = FALSE;
	  }
	  pthread_mutex_unlock(&opticflow_mutex);
}

// Initialize the video_thread structure with the defaults
struct video_thread_t video_thread = {
  .is_running = FALSE,
  .fps = VIDEO_THREAD_FPS,
  .take_shot = FALSE,
  .shot_number = 0
};

static void video_thread_save_shot(struct image_t *img, struct image_t *img_jpeg)
{

  // Search for a file where we can write to
  char save_name[128];
  for (; video_thread.shot_number < 99999; video_thread.shot_number++) {
    sprintf(save_name, "%s/img_%05d.jpg", STRINGIFY(VIDEO_THREAD_SHOT_PATH), video_thread.shot_number);
    // Check if file exists or not
    if (access(save_name, F_OK) == -1) {

      // Create a high quality image (99% JPEG encoded)
      jpeg_encode_image(img, img_jpeg, 99, TRUE);

#if JPEG_WITH_EXIF_HEADER
      write_exif_jpeg(save_name, img_jpeg->buf, img_jpeg->buf_size, img_jpeg->w, img_jpeg->h);
#else
      FILE *fp = fopen(save_name, "w");
      if (fp == NULL) {
        printf("[video_thread-thread] Could not write shot %s.\n", save_name);
      } else {
        // Save it to the file and close it
        fwrite(img_jpeg->buf, sizeof(uint8_t), img_jpeg->buf_size, fp);
        fclose(fp);
      }
#endif

      // We don't need to seek for a next index anymore
      break;
    }
  }
}


/**
 * Handles all the video streaming and saving of the image shots
 * This is a sepereate thread, so it needs to be thread safe!
 */
static void *video_thread_function(void *data)
{
  struct video_config_t *vid = (struct video_config_t *)&(VIDEO_THREAD_CAMERA);

  struct image_t img_jpeg;
  struct image_t img_color;

  // create the images
  if (vid->filters) {
    // fixme: don't hardcode size, works for bebop front camera for now
#define IMG_FLT_SIZE 272
    image_create(&img_color, IMG_FLT_SIZE, IMG_FLT_SIZE, IMAGE_YUV422);
    image_create(&img_jpeg, IMG_FLT_SIZE, IMG_FLT_SIZE, IMAGE_JPEG);
  }
  else {
    image_create(&img_jpeg, vid->w, vid->h, IMAGE_JPEG);
  }

  // Start the streaming of the V4L2 device
  if (!v4l2_start_capture(video_thread.dev)) {
    printf("[video_thread-thread] Could not start capture of %s.\n", video_thread.dev->name);
    return 0;
  }

  // be nice to the more important stuff
//  set_nice_level(10);

  // Start streaming
  video_thread.is_running = TRUE;
  while (video_thread.is_running) {
    // Wait for a new frame (blocking)
    struct image_t img;
    v4l2_image_get(video_thread.dev, &img);

    // pointer to the final image to pass for saving and further processing
    struct image_t *img_final = &img;

    // run selected filters
    if (vid->filters) {
      if (vid->filters & VIDEO_FILTER_DEBAYER) {
        BayerToYUV(&img, &img_color, 0, 0);
      }
      // use color image for further processing
      img_final = &img_color;
    }

    // Check if we need to take a shot
    if (video_thread.take_shot) {
      video_thread_save_shot(img_final, &img_jpeg);
      video_thread.take_shot = FALSE;
    }

    // Copy the state
    pthread_mutex_lock(&opticflow_mutex);
    struct opticflow_state_t temp_state;
    memcpy(&temp_state, &opticflow_state, sizeof(struct opticflow_state_t));
    pthread_mutex_unlock(&opticflow_mutex);

    // Do the optical flow calculation
    struct opticflow_result_t temp_result;
    opticflow_calc_frame(&opticflow_cv, &temp_state, &img, &temp_result);

    // Copy the result if finished
    pthread_mutex_lock(&opticflow_mutex);
    memcpy(&opticflow_result, &temp_result, sizeof(struct opticflow_result_t));
    shot_number = video_thread.shot_number;
    opticflow_got_result = TRUE;
    pthread_mutex_unlock(&opticflow_mutex);

    // Free the image
    v4l2_image_free(video_thread.dev, &img);
  }

  image_free(&img_jpeg);
  image_free(&img_color);

  return 0;
}

/**
 * Initialize the view video
 */
void video_thread_init(void)
{
  struct video_config_t *vid = (struct video_config_t *)&(VIDEO_THREAD_CAMERA);

  // Initialize the V4L2 subdevice if needed
  if (vid->subdev_name != NULL) {
    // FIXME! add subdev format to config, only needed on bebop front camera so far
    if (!v4l2_init_subdev(vid->subdev_name, 0, 0, V4L2_MBUS_FMT_SGBRG10_1X10, vid->w, vid->h)) {
      printf("[video_thread] Could not initialize the %s subdevice.\n", vid->subdev_name);
      return;
    }
  }

  // Initialize the V4L2 device
  video_thread.dev = v4l2_init(vid->dev_name, vid->w, vid->h, vid->buf_cnt, vid->format);
  if (video_thread.dev == NULL) {
    printf("[video_thread] Could not initialize the %s V4L2 device.\n", vid->dev_name);
    return;
  }

  landing_SSL_init();

  // Initialize cv
  // Subscribe to the altitude above ground level ABI messages
  AbiBindMsgAGL(OPTICFLOW_AGL_ID, &opticflow_agl_ev, opticflow_agl_cb);

  // Set the opticflow state to 0
  opticflow_state.phi = 0;
  opticflow_state.theta = 0;
  opticflow_state.psi = 0;
  opticflow_state.agl = 0;
  opticflow_state.V_body_x = 0.0;
  opticflow_state.V_body_y = 0.0;
  opticflow_state.V_body_z = 0.0;
  opticflow_state.gps_x = 0.0;
  opticflow_state.gps_y = 0.0;
  opticflow_state.gps_z = 0.0;

  shot_number = 0;

  // Initialize the opticflow calculation
  opticflow_calc_init(&opticflow_cv, vid->w, vid->h);
  opticflow_got_result = FALSE;

  // Create the shot directory
  char save_name[128];
  sprintf(save_name, "mkdir -p %s", STRINGIFY(VIDEO_THREAD_SHOT_PATH));
  if (system(save_name) != 0) {
    printf("[video_thread] Could not create shot directory %s.\n", STRINGIFY(VIDEO_THREAD_SHOT_PATH));
    return;
  }
#if PERIODIC_TELEMETRY
#ifdef SUB_IMG
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SSL_SUB, SSL_SUB_telem_send);
#else
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_OPTIC_FLOW_CV, opticflow_telem_send);
#endif
#ifdef DOWNLINK_DISTRIBUTIONS
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SSL_TEXTON, SSL_telem_send);
#endif
#endif
}

/**
 * Start with streaming
 */
void video_thread_start(void)
{
  // Check if we are already running
  if (video_thread.is_running) {
    return;
  }

  // Start the streaming thread
  pthread_t tid;
  if (pthread_create(&tid, NULL, video_thread_function, (void*)(&VIDEO_THREAD_CAMERA)) != 0) {
    printf("[vievideo] Could not create streaming thread.\n");
    return;
  }
}

/**
 * Stops the streaming
 * This could take some time, because the thread is stopped asynchronous.
 */
void video_thread_stop(void)
{
  // Check if not already stopped streaming
  if (!video_thread.is_running) {
    return;
  }

  // Stop the streaming thread
  video_thread.is_running = FALSE;

  // Stop the capturing
  if (!v4l2_stop_capture(video_thread.dev)) {
    printf("[video_thread] Could not stop capture of %s.\n", video_thread.dev->name);
    return;
  }

  // TODO: wait for the thread to finish to be able to start the thread again!
}

/**
 * Take a shot and save it
 * This will only work when the streaming is enabled
 */
void video_thread_take_shot(bool_t take)
{
  video_thread.take_shot = take;
}

/**
 * Get the altitude above ground of the drone
 * @param[in] sender_id The id that send the ABI message (unused)
 * @param[in] distance The distance above ground level in meters
 */
static void opticflow_agl_cb(uint8_t sender_id __attribute__((unused)), float distance)
{
  // Update the distance if we got a valid measurement
  if (distance > 0) {
    opticflow_state.agl = distance;
  }
}
