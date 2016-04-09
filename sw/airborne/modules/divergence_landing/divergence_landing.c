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
 * @file modules/divergence_landing/divergence_landing.c
 * @Vertical landing using divergence
 *
 */

#include "modules/divergence_landing/divergence_landing.h"

#include "generated/airframe.h"
#include "paparazzi.h"
#include "subsystems/abi.h"
#include "firmwares/rotorcraft/stabilization.h"

#ifndef VISION_DIV_PGAIN
#define VISION_DIV_PGAIN 283
#endif
PRINT_CONFIG_VAR(VISION_DIV_PGAIN)

#ifndef VISION_DIV_IGAIN
#define VISION_DIV_IGAIN 13
#endif
PRINT_CONFIG_VAR(VISION_DIV_IGAIN)

#ifndef VISION_DIV_DGAIN
#define VISION_DIV_DGAIN 82
#endif
PRINT_CONFIG_VAR(VISION_DIV_DGAIN)

#ifndef VISION_NOMINAL_THROTTLE
#define VISION_NOMINAL_THROTTLE 0.655
#endif
PRINT_CONFIG_VAR(VISION_NOMINAL_THROTTLE)

#ifndef VISION_DESIRED_DIV
#define VISION_DESIRED_DIV 1
#endif
PRINT_CONFIG_VAR(VISION_DESIRED_DIV)

#ifndef VISION_CONTROLLER
#define VISION_CONTROLLER 1
#endif
PRINT_CONFIG_VAR(VISION_CONTROLLER)

#ifndef LANDING_AGL_ID
#define LANDING_AGL_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(LANDING_AGL_ID)

/* Use optical flow estimates */
#ifndef VERTICAL_CTRL_MODULE_OPTICAL_FLOW_ID
#define VERTICAL_CTRL_MODULE_OPTICAL_FLOW_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(VERTICAL_CTRL_MODULE_OPTICAL_FLOW_ID)

/* Check the control gains */
/*
#if (VISION_DIV_PGAIN < 0)      ||  \
	(VISION_DIV_DGAIN < 0)      ||  \
    (VISION_DIV_IGAIN < 0)
#error "ALL control gains have to be positive!!!"
#endif
*/

struct Div_landing_t Div_landing;

static abi_event agl_ev; ///< The altitude ABI event
static abi_event optical_flow_ev;

/// Callback function of the ground altitude
static void landing_agl_cb(uint8_t sender_id __attribute__((unused)), float distance);
// Callback function of the optical flow estimate:
static void vertical_ctrl_optical_flow_cb(uint8_t sender_id __attribute__((unused)), uint32_t stamp, int16_t flow_x, int16_t flow_y, int16_t flow_der_x, int16_t flow_der_y, uint8_t quality, float size_divergence, float dist, float gps_z, float vel_z, float ground_divergence, float fps);
static void HeightEKT(float *Z, float *Vz, float *innov, float *P, float u, float div, float FPS);

int vision_message_nr;

// Height Estimation using EKF
float P_EKF[4], Z_EKF, Vz_EKF, innov_EKF;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
/**
 * Send optical flow telemetry information
 * @param[in] *trans The transport structure to send the information over
 * @param[in] *dev The link to send the data over
 */
static void div_ctrl_telem_send(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_DIV_CTRL(trans, dev, AC_ID,
		  &Div_landing.div_pgain, &Div_landing.div_igain, &Div_landing.div_dgain,
		  &Div_landing.desired_div, &Div_landing.nominal_throttle, &Div_landing.controller,
		  &Div_landing.agl, &Div_landing.gps_z, &Div_landing.vel_z, &Div_landing.z_sp, &Div_landing.err, &Div_landing.z_sum_err,
		  &Div_landing.div, &Div_landing.ground_div, &stabilization_cmd[COMMAND_THRUST], &Div_landing.thrust,
		  &Z_EKF, &Vz_EKF, &innov_EKF, &Div_landing.fps,
		  &P_EKF[0], &P_EKF[1], &P_EKF[2], &P_EKF[3]);
}
#endif

void divergence_landing_init(void);
void divergence_landing_run(bool_t in_flight);

void divergence_landing_init(void)
{
	/* Initialize the default gains and settings */
	Div_landing.div_pgain = VISION_DIV_PGAIN;
	Div_landing.div_igain = VISION_DIV_IGAIN;
	Div_landing.div_dgain = VISION_DIV_DGAIN;
	Div_landing.desired_div = VISION_DESIRED_DIV;
	Div_landing.nominal_throttle = VISION_NOMINAL_THROTTLE;
	Div_landing.controller = VISION_CONTROLLER;
	Div_landing.div_cov = 0.0f;
	Div_landing.div = 0.0f;

	Div_landing.agl = 0.0f;
	Div_landing.gps_z = 0.0f;
	Div_landing.z_sp = VISION_DESIRED_DIV;
	Div_landing.err = 0.0f;
	Div_landing.z_sum_err = 0.0f;
	Div_landing.thrust = 0;

	// Height estimation using EKF
	Z_EKF = 2.0;
	Vz_EKF = -0.01;
	innov_EKF = 0.0;
	P_EKF[0] = 1000; P_EKF[1] = 0; P_EKF[2] = 0; P_EKF[3] = 100;

  // Subscribe to the altitude above ground level ABI messages
  AbiBindMsgAGL(LANDING_AGL_ID, &agl_ev, landing_agl_cb);
  // Subscribe to the optical flow estimator:
  AbiBindMsgOPTICAL_FLOW(VERTICAL_CTRL_MODULE_OPTICAL_FLOW_ID, &optical_flow_ev, vertical_ctrl_optical_flow_cb);
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DIV_CTRL, div_ctrl_telem_send);
#endif
}


void divergence_landing_run(bool_t in_flight)
{
  if (!in_flight)
  {
    // Reset integrators
	Div_landing.z_sum_err = 0;
    stabilization_cmd[COMMAND_THRUST] = 0;
  }
  else
  {
    int32_t nominal_throttle = Div_landing.nominal_throttle * MAX_PPRZ;
//    Div_landing.err = Div_landing.desired_div - Div_landing.gps_z;
    Div_landing.err = -(Div_landing.desired_div - Div_landing.ground_div);
    Div_landing.thrust = nominal_throttle + (Div_landing.div_pgain * Div_landing.err) * MAX_PPRZ  + (Div_landing.div_igain * Div_landing.z_sum_err*0.001) * MAX_PPRZ;
    Bound(Div_landing.thrust, 0, MAX_PPRZ);
    stabilization_cmd[COMMAND_THRUST] = Div_landing.thrust;
    Div_landing.z_sum_err += Div_landing.err;

	// **********************************************************************************************************************
	// Height Estimation using EKF
	// **********************************************************************************************************************
	float u;
	u = (float) ((Div_landing.thrust - nominal_throttle)/MAX_PPRZ);
	HeightEKT(&Z_EKF, &Vz_EKF, &innov_EKF, P_EKF, u, Div_landing.ground_div, Div_landing.fps);
  }
}

static void landing_agl_cb(uint8_t sender_id, float distance)
{
	Div_landing.agl = distance;
}

static void vertical_ctrl_optical_flow_cb(uint8_t sender_id, uint32_t stamp, int16_t flow_x, int16_t flow_y, int16_t flow_der_x, int16_t flow_der_y, uint8_t quality, float size_divergence, float dist, float gps_z, float vel_z, float ground_divergence, float fps)
{
  Div_landing.div = size_divergence;
  Div_landing.gps_z = gps_z;
  Div_landing.ground_div = ground_divergence;
  Div_landing.fps = fps;
  Div_landing.vel_z = vel_z;
  vision_message_nr++;
  if(vision_message_nr > 10) vision_message_nr = 0;
  //printf("Received divergence: %f\n", divergence_vision);
}

// vertical guidance from module
void guidance_v_module_init(void)
{
  divergence_landing_init();
}

void guidance_v_module_enter(void)
{
  // reset integrator
	Div_landing.z_sum_err = 0.0f;
}

void guidance_v_module_run(bool_t in_flight)
{
  divergence_landing_run(in_flight);
}

static void HeightEKT(float *Z, float *Vz, float *innov, float *P, float u, float div, float FPS)
{
	float dt, dx1, dx2, xp1, xp2, zp, Pp[4], H[2], Ve, L[2];

	if (FPS == 0.0)
	{
		dt = 0.0;
	}
	else
	{
		dt = 1/FPS;
	}

	float phi[4] = {1,dt,0,1};
	float gamma[2] = {0,dt};
	float Q = 0.001; // for OT, both Q and R are 0.001
	float R = 0.001;

	// Prediction
	dx1 = *Vz;
	dx2 = u;
	xp1 = *Z + dx1*dt;
	xp2 = *Vz + dx2*dt;
	zp = 2.0*xp2/xp1;

	Pp[0] = Q*(gamma[0]*gamma[0]) + phi[0]*(P[0]*phi[0] + P[2]*phi[1]) + phi[1]*(P[1]*phi[0] + P[3]*phi[1]);
	Pp[1] = phi[2]*(P[0]*phi[0] + P[2]*phi[1]) + phi[3]*(P[1]*phi[0] + P[3]*phi[1]) + Q*gamma[0]*gamma[1];
	Pp[2] = phi[0]*(P[0]*phi[2] + P[2]*phi[3]) + phi[1]*(P[1]*phi[2] + P[3]*phi[3]) + Q*gamma[0]*gamma[1];
	Pp[3] = Q*(gamma[1]*gamma[1]) + phi[2]*(P[0]*phi[2] + P[2]*phi[3]) + phi[3]*(P[1]*phi[2] + P[3]*phi[3]);

	// Correction
	H[0] = -2*xp2/(xp1*xp1);
	H[1] = 2/xp1;

	Ve = R + H[0]*(H[0]*Pp[0] + H[1]*Pp[2]) + H[1]*(H[0]*Pp[1] + H[1]*Pp[3]);
	L[0] = (H[0]*Pp[0] + H[1]*Pp[1])/Ve;
	L[1] = (H[0]*Pp[2] + H[1]*Pp[3])/Ve;

	*innov = div-zp;

	*Z = xp1 + L[0]*(*innov);
	*Vz = xp2 + L[1]*(*innov);

	P[0] = (H[0]*L[0] - 1)*(Pp[0]*(H[0]*L[0] - 1) + H[1]*L[0]*Pp[2]) + L[0]*L[0]*R + H[1]*L[0]*(Pp[1]*(H[0]*L[0] - 1) + H[1]*L[0]*Pp[3]);
	P[1] = (H[1]*L[1] - 1)*(Pp[1]*(H[0]*L[0] - 1) + H[1]*L[0]*Pp[3]) + L[0]*L[1]*R + H[0]*L[1]*(Pp[0]*(H[0]*L[0] - 1) + H[1]*L[0]*Pp[2]);
	P[2] = (H[0]*L[0] - 1)*(Pp[2]*(H[1]*L[1] - 1) + H[0]*L[1]*Pp[0]) + L[0]*L[1]*R + H[1]*L[0]*(Pp[3]*(H[1]*L[1] - 1) + H[0]*L[1]*Pp[1]);
	P[3] = (H[1]*L[1] - 1)*(Pp[3]*(H[1]*L[1] - 1) + H[0]*L[1]*Pp[1]) + L[1]*L[1]*R + H[0]*L[1]*(Pp[2]*(H[1]*L[1] - 1) + H[0]*L[1]*Pp[0]);
}
