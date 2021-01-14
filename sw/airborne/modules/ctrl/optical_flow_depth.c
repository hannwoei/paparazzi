/*
 * Copyright (C) 2015 Guido de Croon.
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
 * @file modules/ctrl/optical_flow_landing.h
 * @brief This module implements optical flow landings in which the divergence is kept constant.
 * When using a fixed gain for control, the covariance between thrust and divergence is tracked,
 * so that the drone knows when it has arrived close to the landing surface. Then, a final landing
 * procedure is triggered. It can also be set to adaptive gain control, where the goal is to continuously
 * gauge the distance to the landing surface. In this mode, the drone will oscillate all the way down to
 * the surface.
 *
 * de Croon, G.C.H.E. (2016). Monocular distance estimation with optical flow maneuvers and efference copies:
 * a stability-based strategy. Bioinspiration & biomimetics, 11(1), 016004.
 * <http://iopscience.iop.org/article/10.1088/1748-3190/11/1/016004>
 *
 * Based on the above theory, we have also developed a new strategy for landing that consists of two phases:
 * (1) while hovering, the drone determines the optimal gain by increasing the gain until oscillation
 * (2) the drone starts landing while exponentially decreasing the gain over time
 *
 * This strategy leads to smooth, high-performance constant divergence landings, as explained in the article:
 * H.W. Ho, G.C.H.E. de Croon, E. van Kampen, Q.P. Chu, and M. Mulder (submitted)
 * Adaptive Control Strategy for Constant Optical Flow Divergence Landing,
 * <https://arxiv.org/abs/1609.06767>
 */

#include "optical_flow_depth.h"

#include "generated/airframe.h"
#include "paparazzi.h"
#include "subsystems/abi.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/guidance/guidance_v_adapt.h"

// used for automated landing:
#include "autopilot.h"
#include "subsystems/navigation/common_flight_plan.h"
#include "subsystems/datalink/telemetry.h"

// for measuring time
#include "mcu_periph/sys_time.h"

#include "math/pprz_stat.h"

/* Default sonar/agl to use */
#ifndef OFL_AGL_ID
#define OFL_AGL_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(OFL_AGL_ID)

/* Use optical flow estimates */
#ifndef OFL_OPTICAL_FLOW_ID
#define OFL_OPTICAL_FLOW_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(OFL_OPTICAL_FLOW_ID)

#ifndef OFL_OPTICAL_FLOW2_ID
#define OFL_OPTICAL_FLOW2_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(OFL_OPTICAL_FLOW2_ID)

// Other default values:
#ifndef OFL_PGAIN
#define OFL_PGAIN 0.3
#endif

#ifndef OFL_IGAIN
#define OFL_IGAIN 0.0
#endif

//#ifndef OFL_IGAIN
//#define OFL_IGAIN 0.0014
//#endif

#ifndef OFL_DGAIN
#define OFL_DGAIN 0.0
#endif

#ifndef OFL_PGAIN_HEIGHT
#define OFL_PGAIN_HEIGHT 0.03
#endif

#ifndef OFL_IGAIN_HEIGHT
#define OFL_IGAIN_HEIGHT 0.001
#endif

#ifndef OFL_VISION_METHOD
#define OFL_VISION_METHOD 1
#endif

#ifndef OFL_CONTROL_METHOD
#define OFL_CONTROL_METHOD 4
#endif

#ifndef OFL_COV_METHOD
#define OFL_COV_METHOD 0
#endif

#ifndef OFL_HEIGHT
#define OFL_HEIGHT 1.0
#endif

// number of time steps used for calculating the covariance (oscillations)
#ifndef OFL_COV_WINDOW_SIZE
#define OFL_COV_WINDOW_SIZE 30
#endif

#ifndef OFL_COV_LANDING_LIMIT
#define OFL_COV_LANDING_LIMIT 2.2
#endif

#ifndef OFL_COV_SETPOINT
#define OFL_COV_SETPOINT -0.0075
#endif

#ifndef OFL_LP_CONST
#define OFL_LP_CONST 0.02
#endif

#ifndef OFL_P_LAND_THRESHOLD
#define OFL_P_LAND_THRESHOLD 0.15
#endif

#ifndef OFL_ELC_OSCILLATE
#define OFL_ELC_OSCILLATE true
#endif

// Constants
// minimum value of the P-gain for divergence control
// adaptive control / exponential gain control will not be able to go lower
#define MINIMUM_GAIN 0.1

// for exponential gain landing, gain increase per second during the first (hover) phase:
#define INCREASE_GAIN_PER_SECOND 0.02

// variables retained between module calls
float divergence_vision;
float divergence_vision_dt;
float normalized_thrust;
float cov_div;
float pstate, pused;
float istate;
float dstate;
float vision_time,  prev_vision_time;
bool landing;
float previous_cov_err;
int32_t thrust_set;
float divergence_setpoint;
float err_height;
float dt_sum;

float divergence_vision2;
float flow_x2;
float flow_y2;

// for the exponentially decreasing gain strategy:
int32_t elc_phase;
uint32_t elc_time_start;
float elc_p_gain_start, elc_i_gain_start,  elc_d_gain_start;
int32_t count_covdiv;
float lp_cov_div;

static abi_event agl_ev; ///< The altitude ABI event
static abi_event optical_flow_ev;
static abi_event optical_flow2_ev;

// struct containing most relevant parameters
struct OpticalFlowLanding2 of_landing_ctrl2;

// sending the divergence message to the ground station:
static void send_divergence(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_DIVERGENCE(trans, dev, AC_ID,
                           &(of_landing_ctrl2.divergence), &dt_sum, &(of_landing_ctrl2.depth),
                           &thrust_set, &(of_landing_ctrl2.flow_x2), &(of_landing_ctrl2.flow_y2), &(of_landing_ctrl2.agl_lp), &(of_landing_ctrl2.alt), &(of_landing_ctrl2.vel),
						   &(of_landing_ctrl2.VISION_METHOD), &(of_landing_ctrl2.CONTROL_METHOD));
}

/// Function definitions
/// Callback function of the ground altitude
void vertical_ctrl_agl_cb(uint8_t sender_id, float distance);
// Callback function of the optical flow estimate:
void vertical_ctrl_optical_flow_cb(uint8_t sender_id, uint32_t stamp, int16_t flow_x,
                                   int16_t flow_y, int16_t flow_der_x, int16_t flow_der_y, float quality, float size_divergence, float fps);
void vertical_ctrl_optical_flow2_cb(uint8_t sender_id, uint32_t stamp, int16_t flow_x,
                                   int16_t flow_y, int16_t flow_der_x, int16_t flow_der_y, float quality, float size_divergence, float fps);

// common functions for different landing strategies:
static void set_cov_div(int32_t thrust);
static int32_t PID_divergence_control(float divergence_setpoint, float P, float I, float D, float dt);
static int32_t INDI_divergence_control(float divergence_setpoint, float dt);
static int32_t height_control(float setpoint, float dt);
static void update_errors(float error, float dt);
static uint32_t final_landing_procedure(void);

// resetting all variables to be called for instance when starting up / re-entering module
static void reset_all_vars(void);

float thrust_history[OFL_COV_WINDOW_SIZE];
float divergence_history[OFL_COV_WINDOW_SIZE];
float past_divergence_history[OFL_COV_WINDOW_SIZE];
uint32_t ind_hist;
uint8_t cov_array_filled;

void vertical_ctrl_module_init2(void);
void vertical_ctrl_module_run2(void);

/**
 * Initialize the optical flow landing module
 */
void vertical_ctrl_module_init2(void)
{
  // filling the of_landing_ctrl2 struct with default values:
  of_landing_ctrl2.agl = 0.0f;
  of_landing_ctrl2.alt = 0.0f;
  of_landing_ctrl2.agl_lp = 0.0f;
  of_landing_ctrl2.vel = 0.0f;
  of_landing_ctrl2.divergence_setpoint = -0.1f; // positive = down, negative = up
  of_landing_ctrl2.cov_set_point = OFL_COV_SETPOINT;
  of_landing_ctrl2.cov_limit = fabsf(OFL_COV_LANDING_LIMIT);
  of_landing_ctrl2.lp_const = OFL_LP_CONST;
  Bound(of_landing_ctrl2.lp_const, 0.001f, 1.f);
  of_landing_ctrl2.pgain = OFL_PGAIN;
  of_landing_ctrl2.igain = OFL_IGAIN;
  of_landing_ctrl2.dgain = OFL_DGAIN;
  of_landing_ctrl2.divergence = 0.;
  of_landing_ctrl2.previous_err = 0.;
  of_landing_ctrl2.sum_err = 0.0f;
  of_landing_ctrl2.d_err = 0.0f;
  of_landing_ctrl2.nominal_thrust = (float)guidance_v_nominal_throttle / MAX_PPRZ; // copy this value from guidance
  of_landing_ctrl2.VISION_METHOD = OFL_VISION_METHOD;
  of_landing_ctrl2.CONTROL_METHOD = OFL_CONTROL_METHOD;
  of_landing_ctrl2.COV_METHOD = OFL_COV_METHOD;
  of_landing_ctrl2.delay_steps = 15;
  of_landing_ctrl2.window_size = OFL_COV_WINDOW_SIZE;
  of_landing_ctrl2.pgain_adaptive = OFL_PGAIN;
  of_landing_ctrl2.igain_adaptive = OFL_IGAIN;
  of_landing_ctrl2.dgain_adaptive = OFL_DGAIN;
  of_landing_ctrl2.reduction_factor_elc =
    0.80f; // for exponential gain landing, after detecting oscillations, the gain is multiplied with this factor
  of_landing_ctrl2.lp_cov_div_factor =
    0.99f; // low pass filtering cov div so that the drone is really oscillating when triggering the descent
  of_landing_ctrl2.t_transition = 2.f;
  // if the gain reaches this value during an exponential landing, the drone makes the final landing.
  of_landing_ctrl2.p_land_threshold = OFL_P_LAND_THRESHOLD;
  of_landing_ctrl2.elc_oscillate = OFL_ELC_OSCILLATE;

  // INDI
  of_landing_ctrl2.yt_1 = 0.0;
  of_landing_ctrl2.yt_2 = 0.0;
  of_landing_ctrl2.ut_1 = 0.0;
  of_landing_ctrl2.ut_2 = 0.0;
  of_landing_ctrl2.ut_0 = 0.0;
  of_landing_ctrl2.ut_00 = 0.0;
  of_landing_ctrl2.delay_input = false;
  of_landing_ctrl2.P_rls11 = 10.0;
  of_landing_ctrl2.P_rls12 = 0.0;
  of_landing_ctrl2.P_rls21 = 0.0;
  of_landing_ctrl2.P_rls22 = 10.0;
  of_landing_ctrl2.gamma_RLS = 0.99; // update rate
  of_landing_ctrl2.F_t = 1.0;
  of_landing_ctrl2.G_t = 0.1;
  of_landing_ctrl2.G_t_prev = 0.0;
  of_landing_ctrl2.GainP = 30.0; //1.0
  of_landing_ctrl2.ut_Max = 0.1;
  of_landing_ctrl2.height_desired = OFL_HEIGHT;
  of_landing_ctrl2.pgain_height = OFL_PGAIN_HEIGHT;
  of_landing_ctrl2.igain_height = OFL_IGAIN_HEIGHT;

  of_landing_ctrl2.depth = 0.0;
  of_landing_ctrl2.flow_x2 = 0.0;
  of_landing_ctrl2.flow_y2 = 0.0;

  err_height = 0.0;
  dt_sum = 0.0;

  reset_all_vars();

  // Subscribe to the altitude above ground level ABI messages
  AbiBindMsgAGL(OFL_AGL_ID, &agl_ev, vertical_ctrl_agl_cb);
  // Subscribe to the optical flow estimator:
  // register telemetry:
  AbiBindMsgOPTICAL_FLOW(OFL_OPTICAL_FLOW_ID, &optical_flow_ev, vertical_ctrl_optical_flow_cb);

  AbiBindMsgOPTICAL_FLOW2(OFL_OPTICAL_FLOW2_ID, &optical_flow2_ev, vertical_ctrl_optical_flow2_cb);

  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DIVERGENCE, send_divergence);
}

/**
 * Reset all variables:
 */
static void reset_all_vars(void)
{
  of_landing_ctrl2.agl_lp = of_landing_ctrl2.agl;// = stateGetPositionEnu_f()->z;

  thrust_set = of_landing_ctrl2.nominal_thrust * MAX_PPRZ;

  cov_div = 0.;
  normalized_thrust = of_landing_ctrl2.nominal_thrust * 100;
  previous_cov_err = 0.;
  divergence_vision = 0.;
  divergence_vision_dt = 0.;
  divergence_setpoint = 0;

  divergence_vision2 = 0.;
  flow_x2 = 0.;
  flow_y2 = 0.;

  vision_time = get_sys_time_float();
  prev_vision_time = vision_time;

  ind_hist = 0;
  cov_array_filled = 0;
  uint32_t i;
  for (i = 0; i < OFL_COV_WINDOW_SIZE; i++) {
    thrust_history[i] = 0;
    divergence_history[i] = 0;
  }

  landing = false;

  elc_phase = 0;
  elc_time_start = 0;
  count_covdiv = 0;
  lp_cov_div = 0.0f;

  pstate = of_landing_ctrl2.pgain;
  pused = pstate;
  istate = of_landing_ctrl2.igain;
  dstate = of_landing_ctrl2.dgain;

  of_landing_ctrl2.divergence = 0.;
  of_landing_ctrl2.previous_err = 0.;
  of_landing_ctrl2.sum_err = 0.;
  of_landing_ctrl2.d_err = 0.;

  //INDI
  of_landing_ctrl2.yt_1 = 0.0;
  of_landing_ctrl2.yt_2 = 0.0;
  of_landing_ctrl2.ut_1 = 0.0;
  of_landing_ctrl2.ut_2 = 0.0;
  of_landing_ctrl2.ut_0 = 0.0;
  of_landing_ctrl2.ut_00 = 0.0;
  of_landing_ctrl2.P_rls11 = 10.0;
  of_landing_ctrl2.P_rls12 = 0.0;
  of_landing_ctrl2.P_rls21 = 0.0;
  of_landing_ctrl2.P_rls22 = 10.0;
  of_landing_ctrl2.gamma_RLS = 0.99;
  of_landing_ctrl2.F_t = 1.0; // Trial 5: F_t initial set to 0.2
  of_landing_ctrl2.G_t = 0.1;
  of_landing_ctrl2.G_t_prev = 0.0;
  of_landing_ctrl2.GainP = 30.0;
  of_landing_ctrl2.ut_Max = 0.1;

  of_landing_ctrl2.depth = 0.0;
  of_landing_ctrl2.flow_x2 = 0.0;
  of_landing_ctrl2.flow_y2 = 0.0;
}

/**
 * Run the optical flow landing module
 */
void vertical_ctrl_module_run2(void)
{
  float div_factor; // factor that maps divergence in pixels as received from vision to 1 / frame

  float dt = vision_time - prev_vision_time;

  dt_sum = dt;

  // check if new measurement received
  if (dt <= 1e-5f) {
    return;
  }

  Bound(of_landing_ctrl2.lp_const, 0.001f, 1.f);
  float lp_factor = dt / of_landing_ctrl2.lp_const;
  Bound(lp_factor, 0.f, 1.f);

  /***********
   * VISION
   ***********/
  if (of_landing_ctrl2.VISION_METHOD == 0) {
//    // SIMULATED DIVERGENCE:
//
//    // USE OPTITRACK HEIGHT
//    of_landing_ctrl2.agl = stateGetPositionEnu_f()->z;
//
//    if (fabsf(of_landing_ctrl2.agl - of_landing_ctrl2.agl_lp) > 1.0f) {
//      // ignore outliers:
//      of_landing_ctrl2.agl = of_landing_ctrl2.agl_lp;
//    }
//    // calculate the new low-pass height and the velocity
//    of_landing_ctrl2.agl_lp += (of_landing_ctrl2.agl - of_landing_ctrl2.agl_lp) * lp_factor;
//
//    // only calculate velocity and divergence if dt is large enough:
//    of_landing_ctrl2.vel = stateGetSpeedEnu_f()->z;
//
//    // calculate the fake divergence:
//    if (of_landing_ctrl2.agl_lp > 1e-5f) {
//      of_landing_ctrl2.divergence = of_landing_ctrl2.vel / of_landing_ctrl2.agl_lp;
//      // TODO: this time scaling should be done in optical flow module
//      divergence_vision_dt = (divergence_vision / dt);
//      if (fabsf(divergence_vision_dt) > 1e-5f) {
//        div_factor = of_landing_ctrl2.divergence / divergence_vision_dt;
//      }
//    }

	    // USE SONAR HEIGHT
	  of_landing_ctrl2.CONTROL_METHOD = 4;

	    if (fabsf(of_landing_ctrl2.agl - of_landing_ctrl2.agl_lp) > 1.0f) {
	      // ignore outliers:
	      of_landing_ctrl2.agl = of_landing_ctrl2.agl_lp;
	    }
	    // calculate the new low-pass height
//	    of_landing_ctrl2.agl_lp += (of_landing_ctrl2.agl - of_landing_ctrl2.agl_lp) * lp_factor;
	    of_landing_ctrl2.agl_lp = of_landing_ctrl2.agl;
//	    prev_vision_time = vision_time;

	    // USE REAL VISION OUTPUTS:
	    // TODO: this div_factor depends on the subpixel-factor (automatically adapt?)
	    // TODO: this factor is camera specific and should be implemented in the optical
	    // flow calculator module not here. Additionally, the time scaling should also
	    // be done in the calculator module
	    div_factor = -1.28f; // magic number comprising field of view etc.
	    float new_divergence = (divergence_vision * div_factor) / dt;

	    // deal with (unlikely) fast changes in divergence:
	    static const float max_div_dt = 0.20f;
	    if (fabsf(new_divergence - of_landing_ctrl2.divergence) > max_div_dt) {
	      if (new_divergence < of_landing_ctrl2.divergence) { new_divergence = of_landing_ctrl2.divergence - max_div_dt; }
	      else { new_divergence = of_landing_ctrl2.divergence + max_div_dt; }
	    }

	    // low-pass filter the divergence:
	    of_landing_ctrl2.divergence += (new_divergence - of_landing_ctrl2.divergence) * lp_factor;

	    //////////////////////////////////////////////////////////////////////////////////////////////////////////
	    // flow x and y
	    /////////////////////////////////////////////////////////////////////////////////////////////////////////
	    // flow in the x-axis from the frontal camera
//	    static const float max_flow_dt = 0.20f;
//
//	    float new_flow_x2 = (flow_x2) / dt;
//
//	    if (fabsf(new_flow_x2 - of_landing_ctrl2.flow_x2) > max_flow_dt) {
//	      if (new_flow_x2 < of_landing_ctrl2.flow_x2) { new_flow_x2 = of_landing_ctrl2.flow_x2 - max_flow_dt; }
//	      else { new_flow_x2 = of_landing_ctrl2.flow_x2 + max_flow_dt; }
//	    }
//
//	    // low-pass filter the flow x:
//	    of_landing_ctrl2.flow_x2 += (new_flow_x2 - of_landing_ctrl2.flow_x2) * lp_factor;
//
//	    // flow in the x-axis from the frontal camera
//	    float new_flow_y2 = (flow_y2) / dt;
//
//	    if (fabsf(new_flow_y2 - of_landing_ctrl2.flow_y2) > max_flow_dt) {
//	      if (new_flow_y2 < of_landing_ctrl2.flow_y2) { new_flow_y2 = of_landing_ctrl2.flow_y2 - max_flow_dt; }
//	      else { new_flow_y2 = of_landing_ctrl2.flow_y2 + max_flow_dt; }
//	    }
//
//	    // low-pass filter the flow y:
//	    of_landing_ctrl2.flow_y2 += (new_flow_y2 - of_landing_ctrl2.flow_y2) * lp_factor;

	    of_landing_ctrl2.flow_x2 = flow_x2/dt;
	    of_landing_ctrl2.flow_y2 = flow_y2/dt;


	    // depth estimation
	    if(fabs(of_landing_ctrl2.flow_x2)> 1e-5f) {
//	    	of_landing_ctrl2.depth = of_landing_ctrl2.divergence/of_landing_ctrl2.flow_x2*of_landing_ctrl2.agl_lp*256.0;
	    	of_landing_ctrl2.depth = of_landing_ctrl2.vel/of_landing_ctrl2.flow_x2*256.0;
	    }
	    else {
	    	of_landing_ctrl2.depth = 0.0;
	    }


	    prev_vision_time = vision_time;

  } else {

	// change to landing control
	of_landing_ctrl2.CONTROL_METHOD = 4;


    // USE REAL VISION OUTPUTS:
    // TODO: this div_factor depends on the subpixel-factor (automatically adapt?)
    // TODO: this factor is camera specific and should be implemented in the optical
    // flow calculator module not here. Additionally, the time scaling should also
    // be done in the calculator module
    div_factor = -1.28f; // magic number comprising field of view etc.
    float new_divergence = (divergence_vision * div_factor) / dt;

    // deal with (unlikely) fast changes in divergence:
    static const float max_div_dt = 0.20f;
    if (fabsf(new_divergence - of_landing_ctrl2.divergence) > max_div_dt) {
      if (new_divergence < of_landing_ctrl2.divergence) { new_divergence = of_landing_ctrl2.divergence - max_div_dt; }
      else { new_divergence = of_landing_ctrl2.divergence + max_div_dt; }
    }

    // low-pass filter the divergence:
    of_landing_ctrl2.divergence += (new_divergence - of_landing_ctrl2.divergence) * lp_factor;
    prev_vision_time = vision_time;
  }

  /***********
  * CONTROL
  ***********/
  // landing indicates whether the drone is already performing a final landing procedure (flare):
//  if (!landing) {
//    if (of_landing_ctrl2.CONTROL_METHOD == 0) {
//      // FIXED GAIN CONTROL, cov_limit for landing:
//
//      // use the divergence for control:
//      thrust_set = PID_divergence_control(of_landing_ctrl2.divergence_setpoint, of_landing_ctrl2.pgain, of_landing_ctrl2.igain,
//                                          of_landing_ctrl2.dgain, dt);
//
//      // trigger the landing if the cov div is too high:
////      if (fabsf(cov_div) > of_landing_ctrl2.cov_limit) {
////      // if (of_landing_ctrl2.agl < 0.2) {
////        thrust_set = final_landing_procedure();
////      }
//    } else if (of_landing_ctrl2.CONTROL_METHOD == 1) {
//      // ADAPTIVE GAIN CONTROL:
//      // TODO: i-gain and d-gain are currently not adapted
//
//      // adapt the gains according to the error in covariance:
//      float error_cov = of_landing_ctrl2.cov_set_point - cov_div;
//      // limit the error_cov, which could else become very large:
//      if (error_cov > fabsf(of_landing_ctrl2.cov_set_point)) { error_cov = fabsf(of_landing_ctrl2.cov_set_point); }
//      pstate -= (of_landing_ctrl2.igain_adaptive * pstate) * error_cov;
//      if (pstate < MINIMUM_GAIN) { pstate = MINIMUM_GAIN; }
//      pused = pstate - (of_landing_ctrl2.pgain_adaptive * pstate) * error_cov;
//      // make sure pused does not become too small, nor grows too fast:
//      if (pused < MINIMUM_GAIN) { pused = MINIMUM_GAIN; }
//      if (of_landing_ctrl2.COV_METHOD == 1 && error_cov > 0.001) {
//        pused = 0.5 * pused;
//      }
//
//      // use the divergence for control:
//      thrust_set = PID_divergence_control(of_landing_ctrl2.divergence_setpoint, pused, of_landing_ctrl2.igain,
//                                          of_landing_ctrl2.dgain, dt);
//
//      // when to make the final landing:
//      if (pstate < of_landing_ctrl2.p_land_threshold) {
//        thrust_set = final_landing_procedure();
//      }
//
//    } else if (of_landing_ctrl2.CONTROL_METHOD == 2) {
//      // EXPONENTIAL GAIN CONTROL:
//      static const float phase_0_set_point = 0.0f;
//      if (elc_phase == 0) {
//        // increase the gain till you start oscillating:
//
//        // if not yet oscillating, increase the gains:
//        if (of_landing_ctrl2.elc_oscillate && cov_div > of_landing_ctrl2.cov_set_point) {
//          pstate += dt * INCREASE_GAIN_PER_SECOND;
//          float gain_factor = pstate / pused;
//          istate *= gain_factor;
//          dstate *= gain_factor;
//          pused = pstate;
//        }
//
//        // use the divergence for control:
//        thrust_set = PID_divergence_control(phase_0_set_point, pused, istate, dstate, dt);
//
//        // low pass filter cov div and remove outliers:
//        if (fabsf(lp_cov_div - cov_div) < of_landing_ctrl2.cov_limit) {
//          lp_cov_div = of_landing_ctrl2.lp_cov_div_factor * lp_cov_div + (1 - of_landing_ctrl2.lp_cov_div_factor) * cov_div;
//        }
//        // if oscillating, maintain a counter to see if it endures:
//        if (lp_cov_div <= of_landing_ctrl2.cov_set_point) {
//          count_covdiv++;
//        } else {
//          count_covdiv = 0;
//          elc_time_start = get_sys_time_float();
//        }
//        // if the drone has been oscillating long enough, start landing:
//        if (!of_landing_ctrl2.elc_oscillate ||
//            (count_covdiv > 0 && (get_sys_time_float() - elc_time_start) >= of_landing_ctrl2.t_transition)) {
//          // next phase:
//          elc_phase = 1;
//          elc_time_start = get_sys_time_float();
//
//          // we don't want to oscillate, so reduce the gain:
//          elc_p_gain_start = of_landing_ctrl2.reduction_factor_elc * pstate;
//          elc_i_gain_start = of_landing_ctrl2.reduction_factor_elc * istate;
//          elc_d_gain_start = of_landing_ctrl2.reduction_factor_elc * dstate;
//          count_covdiv = 0;
//          of_landing_ctrl2.sum_err = 0.0f;
//        }
//      } else if (elc_phase == 1) {
//        // control divergence to 0 with the reduced gain:
//        pstate = elc_p_gain_start;
//        pused = pstate;
//        istate = elc_i_gain_start;
//        dstate = elc_d_gain_start;
//
//        float t_interval = get_sys_time_float() - elc_time_start;
//        // this should not happen, but just to be sure to prevent too high gain values:
//        if (t_interval < 0) { t_interval = 0.0f; }
//
//        // use the divergence for control:
//        thrust_set = PID_divergence_control(phase_0_set_point, pused, istate, dstate, dt);
//
//        // if we have been trying to hover stably again for 2 seconds and we move in the same way as the desired divergence, switch to landing:
//        if (t_interval >= 2.0f && of_landing_ctrl2.divergence * of_landing_ctrl2.divergence_setpoint >= 0.0f) {
//          // next phase:
//          elc_phase = 2;
//          elc_time_start = get_sys_time_float();
//          count_covdiv = 0;
//        }
//      } else if (elc_phase == 2) {
//        // land while exponentially decreasing the gain:
//        float t_interval = get_sys_time_float() - elc_time_start;
//
//        // this should not happen, but just to be sure to prevent too high gain values:
//        if (t_interval < 0) { t_interval = 0.0f; }
//
//        // determine the P-gain, exponentially decaying:
//        float gain_scaling = expf(of_landing_ctrl2.divergence_setpoint * t_interval);
//        pstate = elc_p_gain_start * gain_scaling;
//        istate = elc_i_gain_start * gain_scaling;
//        dstate = elc_d_gain_start * gain_scaling;
//        pused = pstate;
//
//        // 2 [1/s] ramp to setpoint
//        /*if (fabsf(of_landing_ctrl2.divergence_setpoint - divergence_setpoint) > 2.*dt){
//          divergence_setpoint += 2*dt * of_landing_ctrl2.divergence_setpoint / fabsf(of_landing_ctrl2.divergence_setpoint);
//        } else {
//          divergence_setpoint = of_landing_ctrl2.divergence_setpoint;
//        }*/
//
//        // use the divergence for control:
//        thrust_set = PID_divergence_control(of_landing_ctrl2.divergence_setpoint, pused, istate, dstate, dt);
//
//        // when to make the final landing:
//        if (pstate < of_landing_ctrl2.p_land_threshold) {
//          elc_phase = 3;
//        }
//      } else {
//        thrust_set = final_landing_procedure();
//      }
//    } else if (of_landing_ctrl2.CONTROL_METHOD == 3) {
//        // INDI CONTROL:
//    	thrust_set = INDI_divergence_control(of_landing_ctrl2.divergence_setpoint, dt);
//
//        // trigger the landing if the cov div is too high:
//        //if (fabsf(cov_div) > of_landing_ctrl2.cov_limit) {
//        if (of_landing_ctrl2.agl < 0.25) {
//          thrust_set = final_landing_procedure();
//        }
//    } else if (of_landing_ctrl2.CONTROL_METHOD == 4) {
//	  // Height CONTROL:
//
//    	thrust_set = height_control(of_landing_ctrl2.height_desired, dt);
//
//	  // trigger the landing if the cov div is too high:
//	  //if (fabsf(cov_div) > of_landing_ctrl2.cov_limit) {
////	  if (of_landing_ctrl2.agl < 0.2) {
////		thrust_set = final_landing_procedure();
////	  }
//	}
//
//    if (in_flight) {
//      Bound(thrust_set, 0.25 * of_landing_ctrl2.nominal_thrust * MAX_PPRZ, MAX_PPRZ);
//      stabilization_cmd[COMMAND_THRUST] = thrust_set;
//    }
//
//  }
}

/**
 * Execute a final landing procedure
 */
uint32_t final_landing_procedure()
{
  // land with 85% nominal thrust:
  uint32_t nominal_throttle = of_landing_ctrl2.nominal_thrust * MAX_PPRZ;
  uint32_t thrust = 0.85 * nominal_throttle;
  Bound(thrust, 0.6 * nominal_throttle, 0.9 * MAX_PPRZ);
  landing = true;

  return thrust;
}

/**
 * Set the covariance of the divergence and the thrust / past divergence
 * This funciton should only be called once per time step
 * @param[in] thrust: the current thrust value
 */
void set_cov_div(int32_t thrust)
{
  // histories and cov detection:
  divergence_history[ind_hist] = of_landing_ctrl2.divergence;

  normalized_thrust = (float)(thrust / (MAX_PPRZ / 100));
  thrust_history[ind_hist] = normalized_thrust;

  int ind_past = ind_hist - of_landing_ctrl2.delay_steps;
  while (ind_past < 0) { ind_past += of_landing_ctrl2.window_size; }
  past_divergence_history[ind_hist] = divergence_history[ind_past];

  // determine the covariance for landing detection:
  // only take covariance into account if there are enough samples in the histories:
  if (of_landing_ctrl2.COV_METHOD == 0 && cov_array_filled > 0) {
    // TODO: step in landing set point causes an incorrectly perceived covariance
    cov_div = covariance_f(thrust_history, divergence_history, of_landing_ctrl2.window_size);
  } else if (of_landing_ctrl2.COV_METHOD == 1 && cov_array_filled > 1) {
    // todo: delay steps should be invariant to the run frequency
    cov_div = covariance_f(past_divergence_history, divergence_history, of_landing_ctrl2.window_size);
  }

  if (cov_array_filled < 2 && ind_hist + 1 == of_landing_ctrl2.window_size) {
    cov_array_filled++;
  }
  ind_hist = (ind_hist + 1) % of_landing_ctrl2.window_size;
}

/**
 * Determine and set the thrust for constant divergence control
 * @param[out] thrust
 * @param[in] divergence_set_point: The desired divergence
 * @param[in] P: P-gain
 * @param[in] I: I-gain
 * @param[in] D: D-gain
 * @param[in] dt: time difference since last update
 */
int32_t PID_divergence_control(float setpoint, float P, float I, float D, float dt)
{
  // determine the error:
  float err = setpoint - of_landing_ctrl2.divergence;

  // update the controller errors:
  update_errors(err, dt);

  // PID control:
  int32_t thrust = (of_landing_ctrl2.nominal_thrust
                    + P * err
                    + I * of_landing_ctrl2.sum_err
                    + D * of_landing_ctrl2.d_err) * MAX_PPRZ;

  // bound thrust:
  Bound(thrust, 0.25 * of_landing_ctrl2.nominal_thrust * MAX_PPRZ, MAX_PPRZ);

  // update covariance
  // set_cov_div(thrust);

  return thrust;
}

/**
 * Determine and set the thrust for constant divergence control with INDI
 * @param[out] thrust
 * @param[in] divergence_set_point: The desired divergence
 * @param[in] dt: time difference since last update
 */
int32_t INDI_divergence_control(float setpoint, float dt)
{
  // determine the error:
//  float et = setpoint - of_landing_ctrl2.divergence;
  float et = of_landing_ctrl2.divergence - setpoint;

  // Incremental Model RLS
  float tar_M = of_landing_ctrl2.divergence - of_landing_ctrl2.yt_1;
  float input_M11 = of_landing_ctrl2.yt_1 - of_landing_ctrl2.yt_2;
  float input_M21 = of_landing_ctrl2.ut_1 - of_landing_ctrl2.ut_2;
  float Gain_denominator=of_landing_ctrl2.gamma_RLS+input_M11*of_landing_ctrl2.P_rls11*input_M11+input_M21*of_landing_ctrl2.P_rls21*input_M11+input_M11*of_landing_ctrl2.P_rls12*input_M21+input_M21*of_landing_ctrl2.P_rls22*input_M21;
  float Gain11 = 0.0;
  float Gain21 = 0.0;

  if (fabs(Gain_denominator) > 1e-5f) {
	  Gain11 = ( of_landing_ctrl2.P_rls11 * input_M11 + of_landing_ctrl2.P_rls12 * input_M21 ) / Gain_denominator;
	  Gain21 = ( of_landing_ctrl2.P_rls21 * input_M11 + of_landing_ctrl2.P_rls22 * input_M21 ) / Gain_denominator;
  }

  float error_istep = tar_M - ( input_M11 * of_landing_ctrl2.F_t +  input_M21 * of_landing_ctrl2.G_t );
  of_landing_ctrl2.F_t = of_landing_ctrl2.F_t + Gain11 * error_istep;
  of_landing_ctrl2.G_t = of_landing_ctrl2.G_t + Gain21 * error_istep;

  // Trial 0: original
  // Trial 1: use theoretical values
//  of_landing_ctrl2.F_t = -2.0*of_landing_ctrl2.divergence;
//  of_landing_ctrl2.G_t = dt/of_landing_ctrl2.agl;

  // Trial 2: bound G_t
  Bound(of_landing_ctrl2.G_t, 0.01, 3.0);

  // Trial 3: filter G_t
  // deal with (unlikely) fast changes in Gt:
//  static const float max_G_dt = 0.20f;
//  if (fabsf(of_landing_ctrl2.G_t - of_landing_ctrl2.G_t_prev) > max_G_dt) {
//    if (of_landing_ctrl2.G_t < of_landing_ctrl2.G_t_prev) { of_landing_ctrl2.G_t = of_landing_ctrl2.G_t_prev - max_G_dt; }
//    else { of_landing_ctrl2.G_t = of_landing_ctrl2.G_t_prev + max_G_dt; }
//  }
//
//  // low-pass filter the G_t:
//  float lp_factor = dt / of_landing_ctrl2.lp_const;
//  Bound(lp_factor, 0.f, 1.f);
//
//  of_landing_ctrl2.G_t += (of_landing_ctrl2.G_t - of_landing_ctrl2.G_t_prev) * lp_factor;
//
//  Bound(of_landing_ctrl2.G_t, 0.01, 3.0);
//
//  of_landing_ctrl2.G_t_prev = of_landing_ctrl2.G_t;


  float P_rls_new11 = of_landing_ctrl2.P_rls11 - ( Gain11 * input_M11 * of_landing_ctrl2.P_rls11 + Gain11 * input_M21 * of_landing_ctrl2.P_rls21 );
  float P_rls_new12 = of_landing_ctrl2.P_rls12 - ( Gain11 * input_M11 * of_landing_ctrl2.P_rls12 + Gain11 * input_M21 * of_landing_ctrl2.P_rls22 );
  float P_rls_new21 = of_landing_ctrl2.P_rls21 - ( Gain21 * input_M11 * of_landing_ctrl2.P_rls11 + Gain21 * input_M21 * of_landing_ctrl2.P_rls21 );
  float P_rls_new22 = of_landing_ctrl2.P_rls22 - ( Gain21 * input_M11 * of_landing_ctrl2.P_rls12 + Gain21 * input_M21 * of_landing_ctrl2.P_rls22 );

  if (( fabsf(P_rls_new11) + fabsf(P_rls_new12) + fabsf(P_rls_new21) + fabsf(P_rls_new22) ) < 1e+20f) {
	  of_landing_ctrl2.P_rls11 = P_rls_new11 / of_landing_ctrl2.gamma_RLS;
	  of_landing_ctrl2.P_rls12 = P_rls_new12 / of_landing_ctrl2.gamma_RLS;
	  of_landing_ctrl2.P_rls21 = P_rls_new21 / of_landing_ctrl2.gamma_RLS;
	  of_landing_ctrl2.P_rls22 = P_rls_new22 / of_landing_ctrl2.gamma_RLS;
  } else {
	  of_landing_ctrl2.P_rls11 = P_rls_new11;
	  of_landing_ctrl2.P_rls12 = P_rls_new12;
	  of_landing_ctrl2.P_rls21 = P_rls_new21;
	  of_landing_ctrl2.P_rls22 = P_rls_new22;
  }

  float virtual = -1.0*of_landing_ctrl2.GainP*et;
  float u_delta = 0.0;
//  float u_delta = 1.0/of_landing_ctrl2.G_t*(dt*virtual-of_landing_ctrl2.F_t*(of_landing_ctrl2.divergence-of_landing_ctrl2.yt_1)); // dt in sec

  if (fabs(of_landing_ctrl2.G_t) > 1e-5f) {
//  if (of_landing_ctrl2.G_t > 1e-5f) {
	  u_delta = 1.0/of_landing_ctrl2.G_t*(dt*virtual-of_landing_ctrl2.F_t*(of_landing_ctrl2.divergence-of_landing_ctrl2.yt_1)); // dt in sec
//	  u_delta = virtual;
  }
//  } else
//  {
//	  of_landing_ctrl2.ut_1 = 0.0;
//  }

  // Trial 4: bound u_delta
//  Bound(u_delta, -of_landing_ctrl2.ut_Max, of_landing_ctrl2.ut_Max);

  float ut = u_delta + of_landing_ctrl2.ut_1;

  // Limit UT
//  if (ut > of_landing_ctrl2.ut_Max){
//      ut = of_landing_ctrl2.ut_Max;
//  } elseif (ut < -of_landing_ctrl2.ut_Max){
//      ut = - of_landing_ctrl2.ut_Max;
//  }
  Bound(ut, -of_landing_ctrl2.ut_Max, of_landing_ctrl2.ut_Max);

  // save
	  // delay 1 step
//	  of_landing_ctrl2.ut_2 = of_landing_ctrl2.ut_1;
//	  of_landing_ctrl2.ut_1 = of_landing_ctrl2.ut_0;
//	  of_landing_ctrl2.ut_0 = ut;
//	  of_landing_ctrl2.yt_2 = of_landing_ctrl2.yt_1;
//	  of_landing_ctrl2.yt_1 = of_landing_ctrl2.divergence;

	  //delay 2 step
//	  of_landing_ctrl2.ut_2 = of_landing_ctrl2.ut_1;
//	  of_landing_ctrl2.ut_1 = of_landing_ctrl2.ut_0;
//	  of_landing_ctrl2.ut_0 = of_landing_ctrl2.ut_00;
//	  of_landing_ctrl2.ut_00 = ut;
//	  of_landing_ctrl2.yt_2 = of_landing_ctrl2.yt_1;
//	  of_landing_ctrl2.yt_1 = of_landing_ctrl2.divergence;

  	  // no delay
	  of_landing_ctrl2.ut_2 = of_landing_ctrl2.ut_1;
	  of_landing_ctrl2.ut_1 = ut;
	  of_landing_ctrl2.yt_2 = of_landing_ctrl2.yt_1;
	  of_landing_ctrl2.yt_1 = of_landing_ctrl2.divergence;

  int32_t thrust = (of_landing_ctrl2.nominal_thrust + of_landing_ctrl2.ut_1) * MAX_PPRZ;

  // PID control:
//  int32_t thrust = (of_landing_ctrl2.nominal_thrust
//                    + P * err
//                    + I * of_landing_ctrl2.sum_err
//                    + D * of_landing_ctrl2.d_err) * MAX_PPRZ;

  // bound thrust:
  Bound(thrust, 0.25 * of_landing_ctrl2.nominal_thrust * MAX_PPRZ, MAX_PPRZ);

  // update covariance
  // set_cov_div(thrust);

  return thrust;
}

/**
 * Determine and set the thrust for height control
 * @param[out] thrust
 * @param[in] height_set_point: The desired height
 * @param[in] dt: time difference since last update
 */
int32_t height_control(float setpoint, float dt)
{
	// determine the error:
	err_height = setpoint - of_landing_ctrl2.agl_lp;

	of_landing_ctrl2.sum_err += err_height*dt;

	float fb_cmd = (of_landing_ctrl2.pgain_height * err_height) + (of_landing_ctrl2.igain_height * of_landing_ctrl2.sum_err);

	int32_t thrust = (of_landing_ctrl2.nominal_thrust + fb_cmd) * MAX_PPRZ;


	// bound thrust:
	Bound(thrust, 0.25 * of_landing_ctrl2.nominal_thrust * MAX_PPRZ, MAX_PPRZ);

	return thrust;
}

/**
 * Updates the integral and differential errors for PID control and sets the previous error
 * @param[in] err: the error of the divergence and divergence setpoint
 * @param[in] dt:  time difference since last update
 */
void update_errors(float err, float dt)
{
  float lp_factor = dt / of_landing_ctrl2.lp_const;
  Bound(lp_factor, 0.f, 1.f);

  // maintain the controller errors:
  of_landing_ctrl2.sum_err += err;
  of_landing_ctrl2.d_err += (((err - of_landing_ctrl2.previous_err) / dt) - of_landing_ctrl2.d_err) * lp_factor;
  of_landing_ctrl2.previous_err = err;
}

// Reading from "sensors":
void vertical_ctrl_agl_cb(uint8_t sender_id UNUSED, float distance)
{
  of_landing_ctrl2.agl = distance;
  of_landing_ctrl2.vel = stateGetSpeedEnu_f()->z;
  of_landing_ctrl2.alt = stateGetPositionEnu_f()->z;
}

void vertical_ctrl_optical_flow_cb(uint8_t sender_id UNUSED, uint32_t stamp, int16_t flow_x UNUSED,
                                   int16_t flow_y UNUSED,
                                   int16_t flow_der_x UNUSED, int16_t flow_der_y UNUSED, float quality UNUSED, float size_divergence, float fps UNUSED)
{
  divergence_vision = size_divergence;
  //vision_time = ((float)stamp) / 1e6;
}

void vertical_ctrl_optical_flow2_cb(uint8_t sender_id UNUSED, uint32_t stamp, int16_t flow_x,
                                   int16_t flow_y,
                                   int16_t flow_der_x UNUSED, int16_t flow_der_y UNUSED, float quality UNUSED, float size_divergence, float fps UNUSED)
{
  divergence_vision2 = size_divergence;
  flow_x2 = ((float)flow_x)/10.0; //divided by subpixel (change it when subpixel_factor in opticflow_cal updated)
  flow_y2 = ((float)flow_y)/10.0;
  vision_time = ((float)stamp) / 1e6;
}

////////////////////////////////////////////////////////////////////
// Call our controller
void guidance_v_module_init(void)
{
  vertical_ctrl_module_init2();
}

/**
 * Entering the module (user switched to module)
 */
void guidance_v_module_enter(void)
{
  reset_all_vars();

  // adaptive estimation - assume hover condition when entering the module
  of_landing_ctrl2.nominal_thrust = (float) stabilization_cmd[COMMAND_THRUST] / MAX_PPRZ;
  thrust_set = of_landing_ctrl2.nominal_thrust * MAX_PPRZ;
}

void guidance_v_module_run(bool in_flight)
{
//  vertical_ctrl_module_run2(in_flight);
}