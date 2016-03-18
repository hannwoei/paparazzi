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
 * @file modules/computer_vision/cv.c
 *
 * Computer vision framework for onboard processing
 */

#include "cv.h"
#include <stdio.h>
#include <pthread.h>
#include "state.h"
#include "subsystems/abi.h"

/* Default sonar/agl to use in opticflow visual_estimator */
#ifndef OPTICFLOW_AGL_ID
#define OPTICFLOW_AGL_ID ABI_BROADCAST    ///< Default sonar/agl to use in opticflow visual_estimator
#endif
PRINT_CONFIG_VAR(OPTICFLOW_AGL_ID)

#ifndef OPTICFLOW_SENDER_ID
#define OPTICFLOW_SENDER_ID 1
#endif

#define MAX_CV_FUNC 10

int cv_func_cnt = 0;
cvFunction cv_func[MAX_CV_FUNC];

/* The main opticflow variables */
struct opticflow_t opticflow;
static struct opticflow_result_t opticflow_result; ///< The opticflow result
static struct opticflow_state_t opticflow_state;   ///< State of the drone to communicate with the opticflow
static abi_event opticflow_agl_ev;                 ///< The altitude ABI event
static bool_t opticflow_got_result;                ///< When we have an optical flow calculation
static pthread_mutex_t opticflow_mutex;            ///< Mutex lock fo thread safety

/* Static functions */
static void opticflow_agl_cb(uint8_t sender_id, float distance);  ///< Callback function of the ground altitude


#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
/**
 * Send optical flow telemetry information
 * @param[in] *trans The transport structure to send the information over
 * @param[in] *dev The link to send the data over
 */
static void opticflow_telem_send(struct transport_tx *trans, struct link_device *dev)
{
  pthread_mutex_lock(&opticflow_mutex);
  pprz_msg_send_OPTIC_FLOW_EST(trans, dev, AC_ID,
                               &opticflow_result.fps, &opticflow_result.corner_cnt,
                               &opticflow_result.tracked_cnt, &opticflow_result.flow_x,
                               &opticflow_result.flow_y, &opticflow_result.flow_der_x,
                               &opticflow_result.flow_der_y, &opticflow_result.vel_x,
                               &opticflow_result.vel_y, &opticflow_result.div_size,
                               &opticflow_result.surface_roughness, &opticflow_result.divergence);
  pthread_mutex_unlock(&opticflow_mutex);
}
#endif

void cv_add(cvFunction func)
{
  if (cv_func_cnt < (MAX_CV_FUNC - 1)) {
    cv_func[cv_func_cnt] = func;
    cv_func_cnt++;
  }
}

void cv_init(int img_W, int img_H)
{
	  // Subscribe to the altitude above ground level ABI messages
	  AbiBindMsgAGL(OPTICFLOW_AGL_ID, &opticflow_agl_ev, opticflow_agl_cb);

	  // Set the opticflow state to 0
	  opticflow_state.phi = 0;
	  opticflow_state.theta = 0;
	  opticflow_state.agl = 0;

	  // Initialize the opticflow calculation
	  opticflow_calc_init(&opticflow, img_W, img_H);
	  opticflow_got_result = FALSE;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_OPTIC_FLOW_EST, opticflow_telem_send);
#endif
}

void cv_run(struct image_t *img)
{
    // Copy the state
//    pthread_mutex_lock(&opticflow_mutex);
    struct opticflow_state_t temp_state;
    memcpy(&temp_state, &opticflow_state, sizeof(struct opticflow_state_t));
//    pthread_mutex_unlock(&opticflow_mutex);

    // Do the optical flow calculation
    struct opticflow_result_t temp_result;
    opticflow_calc_frame(&opticflow, &temp_state, img, &temp_result);

    // Copy the result if finished
//    pthread_mutex_lock(&opticflow_mutex);
    memcpy(&opticflow_result, &temp_result, sizeof(struct opticflow_result_t));
    opticflow_got_result = TRUE;
//    pthread_mutex_unlock(&opticflow_mutex);
}

void cv_periodic(void)
{
//  for (int i = 0; i < cv_func_cnt; i++) {
//    cv_func[i](img);
//  }
	  pthread_mutex_lock(&opticflow_mutex);
	  // Send Updated data to thread
	  opticflow_state.phi = stateGetNedToBodyEulers_f()->phi;
	  opticflow_state.theta = stateGetNedToBodyEulers_f()->theta;

	  // Update the control loops on the current calculation
	  if (opticflow_got_result) {
	    uint32_t now_ts = get_sys_time_usec();
	    uint8_t quality = opticflow_result.divergence; // FIXME, scale to some quality measure 0-255
	    AbiSendMsgOPTICAL_FLOW(OPTICFLOW_SENDER_ID, now_ts,
	                           opticflow_result.flow_x,
	                           opticflow_result.flow_y,
	                           opticflow_result.flow_der_x,
	                           opticflow_result.flow_der_x,
	                           quality,
	                           opticflow_state.agl);
	    //TODO Find an appropiate quality measure for the noise model in the state filter, for now it is tracked_cnt
	    if (opticflow_result.tracked_cnt > 0) {
	      AbiSendMsgVELOCITY_ESTIMATE(OPTICFLOW_SENDER_ID, now_ts,
	                                  opticflow_result.vel_x,
	                                  opticflow_result.vel_y,
	                                  0.0f,
	                                  opticflow_result.noise_measurement
	                                 );
	    }
	    opticflow_got_result = FALSE;
	  }
	  pthread_mutex_unlock(&opticflow_mutex);
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
