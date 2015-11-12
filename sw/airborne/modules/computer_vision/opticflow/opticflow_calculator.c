/*
 * Copyright (C) 2014 Hann Woei Ho
 *               2015 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file modules/computer_vision/opticflow/opticflow_calculator.c
 * @brief Estimate velocity from optic flow.
 *
 * Using images from a vertical camera and IMU sensor data.
 */

#include "std.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Own Header
#include "opticflow_calculator.h"

// Computer Vision
#include "lib/vision/image.h"
#include "lib/vision/lucas_kanade.h"
#include "lib/vision/fast_rosten.h"
#include "lib/vision/opticflow_fitting.h"
#include "size_divergence.h"

// Camera parameters (defaults are from an ARDrone 2)
#ifndef OPTICFLOW_FOV_W
#define OPTICFLOW_FOV_W 0.89360857702
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FOV_W)

#ifndef OPTICFLOW_FOV_H
#define OPTICFLOW_FOV_H 0.67020643276
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FOV_H)

#ifndef OPTICFLOW_FX
#define OPTICFLOW_FX 343.1211
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FX)

#ifndef OPTICFLOW_FY
#define OPTICFLOW_FY 348.5053
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FY)

/* Set the default values */
#ifndef OPTICFLOW_MAX_TRACK_CORNERS
#define OPTICFLOW_MAX_TRACK_CORNERS 25
#endif
PRINT_CONFIG_VAR(OPTICFLOW_MAX_TRACK_CORNERS)

#ifndef OPTICFLOW_WINDOW_SIZE
#define OPTICFLOW_WINDOW_SIZE 10
#endif
PRINT_CONFIG_VAR(OPTICFLOW_WINDOW_SIZE)

#ifndef OPTICFLOW_SUBPIXEL_FACTOR
#define OPTICFLOW_SUBPIXEL_FACTOR 10
#endif
PRINT_CONFIG_VAR(OPTICFLOW_SUBPIXEL_FACTOR)

#ifndef OPTICFLOW_MAX_ITERATIONS
#define OPTICFLOW_MAX_ITERATIONS 10
#endif
PRINT_CONFIG_VAR(OPTICFLOW_MAX_ITERATIONS)

#ifndef OPTICFLOW_THRESHOLD_VEC
#define OPTICFLOW_THRESHOLD_VEC 2
#endif
PRINT_CONFIG_VAR(OPTICFLOW_THRESHOLD_VEC)

#ifndef OPTICFLOW_FAST9_ADAPTIVE
#define OPTICFLOW_FAST9_ADAPTIVE 1
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FAST9_ADAPTIVE)

#ifndef OPTICFLOW_FAST9_THRESHOLD
#define OPTICFLOW_FAST9_THRESHOLD 20
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FAST9_THRESHOLD)

#ifndef OPTICFLOW_FAST9_MIN_DISTANCE
#define OPTICFLOW_FAST9_MIN_DISTANCE 10
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FAST9_MIN_DISTANCE)

#ifndef OPTICFLOW_SNAPSHOT
#define OPTICFLOW_SNAPSHOT 0
#endif
PRINT_CONFIG_VAR(OPTICFLOW_SNAPSHOT)

#ifndef OPTICFLOW_W_N
#define OPTICFLOW_W_N 0.8
#endif
PRINT_CONFIG_VAR(OPTICFLOW_SNAPSHOT)

/* Functions only used here */
static uint32_t timeval_diff(struct timeval *starttime, struct timeval *finishtime);
static int cmp_flow(const void *a, const void *b);

// flow fitting
float *pu, *pv, z_x, z_y, flatness, divergence, TTI, d_heading, d_pitch, min_error_u, min_error_v;
int n_inlier_minu, n_inlier_minv, FIT_UNCERTAINTY, USE_LINEAR_FIT, no_parameter;

// washout filter
float Div_dd, w_n, Div_d, t_step, Div_f;

// lowpass filter
float alpha, RC;

// size divergence
float size_divergence;
int n_samples;
float grd_divergence;
float size_buf[5];
unsigned int size_point = 0;

// snapshot
char filename[100];
int i_frame;
uint8_t snapshot;

/**
 * Initialize the opticflow calculator
 * @param[out] *opticflow The new optical flow calculator
 * @param[in] *w The image width
 * @param[in] *h The image height
 */
void opticflow_calc_init(struct opticflow_t *opticflow, uint16_t w, uint16_t h)
{
  /* Create the image buffers */
  image_create(&opticflow->img_gray, w, h, IMAGE_GRAYSCALE);
  image_create(&opticflow->prev_img_gray, w, h, IMAGE_GRAYSCALE);

  /* Set the previous values */
  opticflow->got_first_img = FALSE;
  opticflow->prev_phi = 0.0;
  opticflow->prev_theta = 0.0;

  /* Set the default values */
  opticflow->max_track_corners = OPTICFLOW_MAX_TRACK_CORNERS;
  opticflow->window_size = OPTICFLOW_WINDOW_SIZE;
  opticflow->subpixel_factor = OPTICFLOW_SUBPIXEL_FACTOR;
  opticflow->max_iterations = OPTICFLOW_MAX_ITERATIONS;
  opticflow->threshold_vec = OPTICFLOW_THRESHOLD_VEC;

  opticflow->fast9_adaptive = OPTICFLOW_FAST9_ADAPTIVE;
  opticflow->fast9_threshold = OPTICFLOW_FAST9_THRESHOLD;
  opticflow->fast9_min_distance = OPTICFLOW_FAST9_MIN_DISTANCE;
  opticflow->w_n = OPTICFLOW_W_N;

  // flow fitting
  USE_LINEAR_FIT = 1;
  if(USE_LINEAR_FIT == 1)
  {
	  no_parameter = 3;
  }
  else
  {
	  no_parameter = 5;
  }
  pu = (float *) calloc (no_parameter,sizeof(float));
  pv = (float *) calloc (no_parameter,sizeof(float));
  z_x = 0.0, z_y = 0.0, flatness = 0.0, divergence = 0.0, TTI = 0.0, d_heading = 0.0, d_pitch = 0.0, min_error_u = 0.0, min_error_v = 0.0;
  n_inlier_minu = 0, n_inlier_minv = 0, FIT_UNCERTAINTY = 0;

  // washout filter
  Div_dd = 0.0, Div_d = 0.0, t_step = 0.0, Div_f = 0.0;

  // lowpass filter
  alpha = 0.0, RC = 1.0/(2.0*2*3.14);

  // size divergence
  size_divergence = 0.0;
  n_samples = 100;

  // ground divergence
  grd_divergence = 0.0;

  // snapshot
  i_frame = 0;
  snapshot = OPTICFLOW_SNAPSHOT;

}

/**
 * Run the optical flow on a new image frame
 * @param[in] *opticflow The opticalflow structure that keeps track of previous images
 * @param[in] *state The state of the drone
 * @param[in] *img The image frame to calculate the optical flow from
 * @param[out] *result The optical flow result
 */
void opticflow_calc_frame(struct opticflow_t *opticflow, struct opticflow_state_t *state, struct image_t *img, struct opticflow_result_t *result)
{
  // Update FPS for information
  result->fps = 1 / (timeval_diff(&opticflow->prev_timestamp, &img->ts) / 1000.);
  memcpy(&opticflow->prev_timestamp, &img->ts, sizeof(struct timeval));

  // Convert image to grayscale
  image_to_grayscale(img, &opticflow->img_gray);

  // Copy to previous image if not set
  if (!opticflow->got_first_img) {
    image_copy(&opticflow->img_gray, &opticflow->prev_img_gray);
    opticflow->got_first_img = TRUE;
  }

  // *************************************************************************************
  // Corner detection
  // *************************************************************************************

  // FAST corner detection (TODO: non fixed threashold)
  struct point_t *corners = fast9_detect(img, opticflow->fast9_threshold, opticflow->fast9_min_distance,
                                         20, 20, &result->corner_cnt);

  // Adaptive threshold
  if (opticflow->fast9_adaptive) {

    // Decrease and increase the threshold based on previous values
    if (result->corner_cnt < 40 && opticflow->fast9_threshold > 5) {
      opticflow->fast9_threshold--;
    } else if (result->corner_cnt > 50 && opticflow->fast9_threshold < 60) {
      opticflow->fast9_threshold++;
    }
  }

#if OPTICFLOW_DEBUG && OPTICFLOW_SHOW_CORNERS
  image_show_points(img, corners, result->corner_cnt);
#endif

  // Check if we found some corners to track
  if (result->corner_cnt < 1) {
    free(corners);
    image_copy(&opticflow->img_gray, &opticflow->prev_img_gray);
    return;
  }

  // *************************************************************************************
  // Corner Tracking
  // *************************************************************************************

  // Execute a Lucas Kanade optical flow
  result->tracked_cnt = result->corner_cnt;
  struct flow_t *vectors = opticFlowLK(&opticflow->img_gray, &opticflow->prev_img_gray, corners, &result->tracked_cnt,
                                       opticflow->window_size / 2, opticflow->subpixel_factor, opticflow->max_iterations,
                                       opticflow->threshold_vec, opticflow->max_track_corners);

#if OPTICFLOW_DEBUG && OPTICFLOW_SHOW_FLOW
  image_show_flow(img, vectors, result->tracked_cnt, opticflow->subpixel_factor);
#endif

//  // Get the median flow
//  qsort(vectors, result->tracked_cnt, sizeof(struct flow_t), cmp_flow);
//  if (result->tracked_cnt == 0) {
//    // We got no flow
//    result->flow_x = 0;
//    result->flow_y = 0;
//  } else if (result->tracked_cnt > 3) {
//    // Take the average of the 3 median points
//    result->flow_x = vectors[result->tracked_cnt / 2 - 1].flow_x;
//    result->flow_y = vectors[result->tracked_cnt / 2 - 1].flow_y;
//    result->flow_x += vectors[result->tracked_cnt / 2].flow_x;
//    result->flow_y += vectors[result->tracked_cnt / 2].flow_y;
//    result->flow_x += vectors[result->tracked_cnt / 2 + 1].flow_x;
//    result->flow_y += vectors[result->tracked_cnt / 2 + 1].flow_y;
//    result->flow_x /= 3;
//    result->flow_y /= 3;
//  } else {
//    // Take the median point
//    result->flow_x = vectors[result->tracked_cnt / 2].flow_x;
//    result->flow_y = vectors[result->tracked_cnt / 2].flow_y;
//  }
//
//  // Flow Derotation
//  float diff_flow_x = (state->phi - opticflow->prev_phi) * img->w / OPTICFLOW_FOV_W;
//  float diff_flow_y = (state->theta - opticflow->prev_theta) * img->h / OPTICFLOW_FOV_H;
//  result->flow_der_x = result->flow_x - diff_flow_x * opticflow->subpixel_factor;
//  result->flow_der_y = result->flow_y - diff_flow_y * opticflow->subpixel_factor;
//  opticflow->prev_phi = state->phi;
//  opticflow->prev_theta = state->theta;
//
//  // Velocity calculation
//  result->vel_x = -result->flow_der_x * result->fps * state->agl/ opticflow->subpixel_factor * img->w / OPTICFLOW_FX;
//  result->vel_y =  result->flow_der_y * result->fps * state->agl/ opticflow->subpixel_factor * img->h / OPTICFLOW_FY;

  // *************************************************************************************
  // Flow Field Fitting
  // *************************************************************************************
//  for(int i=0; i<no_parameter; i++)
//  {
//	  pu[i] = 0.0;
//	  pv[i] = 0.0;
//  }
//
//  analyseTTI(pu, pv, &z_x, &z_y, &flatness, &divergence, &TTI, &d_heading, &d_pitch,
//  		&n_inlier_minu, &n_inlier_minv, &min_error_u, &min_error_v, &FIT_UNCERTAINTY,
//  		vectors, result->tracked_cnt, opticflow->subpixel_factor, result->fps, img->w, img->h, USE_LINEAR_FIT);

  // *************************************************************************************
  // Size Divergence
  // *************************************************************************************
  size_divergence = -get_size_divergence(vectors, result->tracked_cnt, n_samples)*65;

  // washout filter on divergence
  if (result->fps == 0)
  {
	  t_step = 0.0;
  }
  else
  {
	  t_step = 1.0/result->fps;
  }
//  TODO: Better outlier removal is needed.
//  Div_dd = ((size_divergence-opticflow->Div_f_prev)*w_n/2.0-opticflow->Div_d_prev)*2.0*w_n;
//  Div_d = Div_dd*t_step + opticflow->Div_d_prev;
//  Div_f = Div_d*t_step + opticflow->Div_f_prev;

  // low pass filter
//  alpha = t_step/(RC+t_step);
//  alpha = opticflow->w_n;
//  Div_f = opticflow->Div_f_prev + alpha*(size_divergence-opticflow->Div_f_prev);
//  Div_f = opticflow->Div_f_prev + alpha*(divergence-opticflow->Div_f_prev);

  // median filter
	size_buf[size_point] = size_divergence;
//	size_buf[size_point] = divergence;
	size_point = (size_point+1) %5;

	quick_sort(size_buf,5);
	Div_f  = size_buf[3];

  opticflow->Div_d_prev = Div_d;
  opticflow->Div_f_prev = Div_f;

  // *************************************************************************************
  // Ground Divergence
  // *************************************************************************************
  /* compute ground divergence */
  if(state->gps_z!=0)
  {
	  grd_divergence = -2.0*state->V_body_z/state->gps_z;
  }
  else
  {
	  grd_divergence = 0.0;
  }

  // *************************************************************************************
  // thread data
  // *************************************************************************************
  result->zx = z_x;
  result->zy = z_y;
  result->flatness = flatness;
  result->divergence = divergence;
  result->TTI = TTI;
  result->d_heading = d_heading;
  result->d_pitch = d_pitch;
  result->n_inlier = n_inlier_minu + n_inlier_minv;
  result->min_error = min_error_u + min_error_v;
  result->fit_uncertainty = FIT_UNCERTAINTY;
  result->Div_f = Div_f;
  result->Div_d = Div_d;
  result->div_size = size_divergence;
  result->Div_grd = grd_divergence;
	// **********************************************************************************************************************
	// Save an image
	// **********************************************************************************************************************
//	if(snapshot)
//	{
//		snapshot = 0;
//
//		sprintf(filename, "/data/video/image_%d.dat", i_frame);
//		saveSingleImageDataFile(img->buf, img->w, img->h, filename);
//
//	}
//	else
//	{
//
//	}

  // *************************************************************************************
  // Next Loop Preparation
  // *************************************************************************************
  free(corners);
  free(vectors);
  image_switch(&opticflow->img_gray, &opticflow->prev_img_gray);
}

/**
 * Calculate the difference from start till finish
 * @param[in] *starttime The start time to calculate the difference from
 * @param[in] *finishtime The finish time to calculate the difference from
 * @return The difference in milliseconds
 */
static uint32_t timeval_diff(struct timeval *starttime, struct timeval *finishtime)
{
  uint32_t msec;
  msec = (finishtime->tv_sec - starttime->tv_sec) * 1000;
  msec += (finishtime->tv_usec - starttime->tv_usec) / 1000;
  return msec;
}

/**
 * Compare two flow vectors based on flow distance
 * Used for sorting.
 * @param[in] *a The first flow vector (should be vect flow_t)
 * @param[in] *b The second flow vector (should be vect flow_t)
 * @return Negative if b has more flow than a, 0 if the same and positive if a has more flow than b
 */
static int cmp_flow(const void *a, const void *b)
{
  const struct flow_t *a_p = (const struct flow_t *)a;
  const struct flow_t *b_p = (const struct flow_t *)b;
  return (a_p->flow_x * a_p->flow_x + a_p->flow_y * a_p->flow_y) - (b_p->flow_x * b_p->flow_x + b_p->flow_y * b_p->flow_y);
}
