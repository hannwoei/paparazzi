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
#define VISION_DESIRED_DIV 0.3
#endif
PRINT_CONFIG_VAR(VISION_DESIRED_DIV)

#ifndef VISION_CONTROLLER
#define VISION_CONTROLLER 1
#endif
PRINT_CONFIG_VAR(VISION_CONTROLLER)

#ifndef VISION_LP_ALPHA
#define VISION_LP_ALPHA 0.6
#endif
PRINT_CONFIG_VAR(VISION_LP_ALPHA)

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
static void vertical_ctrl_optical_flow_cb(uint8_t sender_id __attribute__((unused)), uint32_t stamp, int16_t flow_x, int16_t flow_y, int16_t flow_der_x, int16_t flow_der_y, uint8_t quality, float size_divergence, float dist, float gps_z, float vel_z, float accel_z, float ground_divergence, float fps);
static void HeightEKT(float *Z, float *Vz, float *innov, float *P, float u, float div, float fps);

// Vision
float div_update, fb_cmd;
int message_count, previous_count, i_init;

// Height Estimation using EKF
float P_EKF[4], Z_EKF, Vz_EKF, innov_EKF, Z_est, V_est;
uint32_t prev_stamp, curr_stamp;

// Frame Rate
#include <sys/time.h>
float dt;
volatile long timestamp;
#define USEC_PER_SEC 1000000L

static long time_elapsed (struct timeval *t1, struct timeval *t2) {
	long sec, usec;
	sec = t2->tv_sec - t1->tv_sec;
	usec = t2->tv_usec - t1->tv_usec;
	if (usec < 0) {
		--sec;
		usec = usec + USEC_PER_SEC;
	}
	return sec*USEC_PER_SEC + usec;
}

struct timeval start_time;
struct timeval end_time;

static void start_timer(void) {
	gettimeofday (&start_time, NULL);
}
static long end_timer(void) {
	gettimeofday (&end_time, NULL);
	return time_elapsed(&start_time, &end_time);
}


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
		  &Div_landing.agl, &Div_landing.gps_z, &Div_landing.vel_z, &Div_landing.accel_z, &Div_landing.z_sp, &Div_landing.err, &Div_landing.z_sum_err,
		  &Div_landing.div, &Div_landing.div_f, &Div_landing.ground_div, &stabilization_cmd[COMMAND_THRUST], &Div_landing.thrust, &fb_cmd,
		  &Z_est, &V_est, &innov_EKF, &Div_landing.fps, &Div_landing.stamp,
		  &P_EKF[0], &P_EKF[1], &P_EKF[2], &P_EKF[3]);
}
#endif

void divergence_landing_init(void);
void divergence_landing_run(bool_t in_flight);

void divergence_landing_init(void)
{
	div_update = 0.0;
	fb_cmd = 0.0;
	message_count = 1;
	previous_count = 0;

	i_init = 0;

	// FPS
	dt = 0.0;
	timestamp=0;
//	start_timer();
	prev_stamp = 0;
	curr_stamp = 0;

	/* Initialize the default gains and settings */
	Div_landing.div_pgain = VISION_DIV_PGAIN;
	Div_landing.div_igain = VISION_DIV_IGAIN;
	Div_landing.div_dgain = VISION_DIV_DGAIN;
	Div_landing.desired_div = VISION_DESIRED_DIV;
	Div_landing.nominal_throttle = VISION_NOMINAL_THROTTLE;
	Div_landing.controller = VISION_CONTROLLER;
	Div_landing.alpha = VISION_LP_ALPHA;
	Div_landing.div_cov = 0.0f;
	Div_landing.div = 0.0f;
	Div_landing.div_f = VISION_DESIRED_DIV;
	Div_landing.ground_div = 0.0f;
	Div_landing.agl = 0.0f;
	Div_landing.gps_z = 0.0f;
	Div_landing.vel_z = 0.0f;
	Div_landing.accel_z = 0.0f;
	Div_landing.z_sp = VISION_DESIRED_DIV;
	Div_landing.err = 0.0f;
	Div_landing.z_sum_err = 0.0f;
	Div_landing.thrust = 0;
	Div_landing.fps = 0.0f;
	Div_landing.stamp = 0.0f;

	// Height estimation using EKF
	Z_EKF = 3.0;
	Vz_EKF = 0.01;
	innov_EKF = 0.0;
	P_EKF[0] = 10; P_EKF[1] = 0; P_EKF[2] = 0; P_EKF[3] = 10;
	Z_est = 0.0; V_est = 0.0;

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
    // FPS
//	timestamp = end_timer();
//	dt += (float)timestamp/1000000.0;
//	start_timer();
//	 ignore first dt
//	if(dt > 10.0f) {
//		printf("too long, dt = %f\n",dt);
//		dt = 0.0f;
//		return;
//	}


  if (!in_flight)
  {
    // Reset integrators
	Div_landing.z_sum_err = 0;
    stabilization_cmd[COMMAND_THRUST] = 0;
  }
  else
  {
	// **********************************************************************************************************************
	// Sinusoidal setpoints
	// **********************************************************************************************************************
	if(Div_landing.gps_z < 0.8)
	{
		Div_landing.desired_div = -VISION_DESIRED_DIV;
	}
	if(Div_landing.gps_z > 1.8)
	{
		Div_landing.desired_div = VISION_DESIRED_DIV;
	}


	if(message_count != previous_count)
	{
		// **********************************************************************************************************************
		// timestamp of messages
		// **********************************************************************************************************************
		Div_landing.stamp = (float) (curr_stamp-prev_stamp)/1000000.0;
		prev_stamp = curr_stamp;
//		printf("dt=%f, fps=%f, st=%f\n",dt,1/Div_landing.fps, Div_landing.stamp);

		if(i_init == 0)
		{
			Div_landing.div_f = VISION_DESIRED_DIV;
			Div_landing.stamp = 0.0;
			dt = 0.0;
			i_init ++;
			return;
		}
		else
		{
		// **********************************************************************************************************************
		// Vision Correction
		// **********************************************************************************************************************
			div_update = Div_landing.div*1.28;
			if(fabs(div_update - Div_landing.div_f) > 0.20) {
				if(div_update < Div_landing.div_f) div_update = Div_landing.div_f - 0.10f;
				else div_update = Div_landing.div_f + 0.10f;
			}
			Div_landing.div_f = Div_landing.div_f*Div_landing.alpha + div_update*(1.0f - Div_landing.alpha);
		}
		// **********************************************************************************************************************
		// Feedback Controller
		// **********************************************************************************************************************
	    int32_t nominal_throttle = Div_landing.nominal_throttle * MAX_PPRZ;
	    if(Div_landing.controller == 1)
	    {
	    	Div_landing.err = -(Div_landing.desired_div - Div_landing.div_f);
	    }
	    else if(Div_landing.controller == 2)
	    {
	    	Div_landing.err = -(Div_landing.desired_div - Div_landing.ground_div);
	    }
	    else
	    {
	    	Div_landing.err = Div_landing.desired_div - Div_landing.gps_z;
	    }

		// **********************************************************************************************************************
		// Adaptive gain
		// **********************************************************************************************************************
//	    Div_landing.div_pgain = Div_landing.div_pgain + innov_EKF;
//	    if(Div_landing.div_pgain < 0)
//	    {
//	    	Div_landing.div_pgain = 0.05;
//	    }
//	    if(Div_landing.div_pgain > 0.3)
//	    {
//	    	Div_landing.div_pgain = 0.3;
//	    }

	    fb_cmd = (Div_landing.div_pgain * Div_landing.err);//   + (Div_landing.div_igain * Div_landing.z_sum_err*0.001);
	    Div_landing.thrust = nominal_throttle + fb_cmd* MAX_PPRZ;
	    Bound(Div_landing.thrust, 0, MAX_PPRZ);
	    stabilization_cmd[COMMAND_THRUST] = Div_landing.thrust;
//	    Div_landing.z_sum_err += Div_landing.err;

		// **********************************************************************************************************************
		// Height Estimation using EKF
		// **********************************************************************************************************************
//		printf("u = %f, dt = %f\n",fb_cmd,dt);
		HeightEKT(&Z_EKF, &Vz_EKF, &innov_EKF, P_EKF, fb_cmd, -Div_landing.div_f, Div_landing.fps);
		Z_est = 15.0*Z_EKF;
		V_est = 15.0*Vz_EKF;

		previous_count = message_count;
		dt = 0.0;
	}
	else
	{
//		return;
	}
  }
}

static void landing_agl_cb(uint8_t sender_id, float distance)
{
	Div_landing.agl = distance;
}

static void vertical_ctrl_optical_flow_cb(uint8_t sender_id, uint32_t stamp, int16_t flow_x, int16_t flow_y, int16_t flow_der_x, int16_t flow_der_y, uint8_t quality, float size_divergence, float dist, float gps_z, float vel_z, float accel_z, float ground_divergence, float fps)
{
  Div_landing.div = size_divergence;
  Div_landing.gps_z = gps_z;
  Div_landing.ground_div = ground_divergence;
  Div_landing.fps = fps;
  Div_landing.vel_z = vel_z;
  Div_landing.accel_z = accel_z;
  curr_stamp = stamp;
  message_count++;
  if(message_count > 10) message_count = 0;
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

static void HeightEKT(float *Z, float *Vz, float *innov, float *P, float u, float div, float fps)
{
	float dt_ekf, dx1, dx2, xp1, xp2, zp, Pp[4], H[2], Ve, L[2];

	if (fps < 0.0001f)
	{
		dt_ekf = 0.0;
		return;
	}
	else
	{
		dt_ekf = 1.0/fps;
	}
//	dt_ekf = fps;

	float phi[4] = {1.0,dt_ekf,0.0,1.0};
	float gamma[2] = {dt_ekf*dt_ekf*0.5,dt_ekf};
	float Q = 0.01; // for OT, both Q and R are 0.001
	float R = 0.01;

	// Prediction
	dx1 = *Vz;
	dx2 = u;
	xp1 = *Z + dx1*dt_ekf;
	xp2 = *Vz + dx2*dt_ekf;
	zp = xp2/xp1;

//	printf("xp1=%f, dx2=%f, zp=%f ,", xp1, xp2, zp);

	Pp[0] = Q*(gamma[0]*gamma[0]) + phi[0]*(P[0]*phi[0] + P[2]*phi[1]) + phi[1]*(P[1]*phi[0] + P[3]*phi[1]);
	Pp[1] = phi[2]*(P[0]*phi[0] + P[2]*phi[1]) + phi[3]*(P[1]*phi[0] + P[3]*phi[1]) + Q*gamma[0]*gamma[1];
	Pp[2] = phi[0]*(P[0]*phi[2] + P[2]*phi[3]) + phi[1]*(P[1]*phi[2] + P[3]*phi[3]) + Q*gamma[0]*gamma[1];
	Pp[3] = Q*(gamma[1]*gamma[1]) + phi[2]*(P[0]*phi[2] + P[2]*phi[3]) + phi[3]*(P[1]*phi[2] + P[3]*phi[3]);

//	printf("pp1=%f, pp2=%f, pp3=%f, pp4=%f, ",Pp[0], Pp[1], Pp[2], Pp[3]);
	// Correction
	H[0] = -xp2/(xp1*xp1);
	H[1] = 1.0/xp1;
//	printf("H1=%f, H2=%f, ",H[0],H[1]);

	Ve = R + H[0]*(H[0]*Pp[0] + H[1]*Pp[2]) + H[1]*(H[0]*Pp[1] + H[1]*Pp[3]);
	L[0] = (H[0]*Pp[0] + H[1]*Pp[1])/Ve;
	L[1] = (H[0]*Pp[2] + H[1]*Pp[3])/Ve;
//	printf("Ve=%f, L1=%f, L2=%f, \n",Ve, L[0], L[1]);

	*innov = div-zp;

	*Z = xp1 + L[0]*(*innov);
	*Vz = xp2 + L[1]*(*innov);

	P[0] = (H[0]*L[0] - 1)*(Pp[0]*(H[0]*L[0] - 1) + H[1]*L[0]*Pp[2]) + L[0]*L[0]*R + H[1]*L[0]*(Pp[1]*(H[0]*L[0] - 1) + H[1]*L[0]*Pp[3]);
	P[1] = (H[1]*L[1] - 1)*(Pp[1]*(H[0]*L[0] - 1) + H[1]*L[0]*Pp[3]) + L[0]*L[1]*R + H[0]*L[1]*(Pp[0]*(H[0]*L[0] - 1) + H[1]*L[0]*Pp[2]);
	P[2] = (H[0]*L[0] - 1)*(Pp[2]*(H[1]*L[1] - 1) + H[0]*L[1]*Pp[0]) + L[0]*L[1]*R + H[1]*L[0]*(Pp[3]*(H[1]*L[1] - 1) + H[0]*L[1]*Pp[1]);
	P[3] = (H[1]*L[1] - 1)*(Pp[3]*(H[1]*L[1] - 1) + H[0]*L[1]*Pp[1]) + L[1]*L[1]*R + H[0]*L[1]*(Pp[2]*(H[1]*L[1] - 1) + H[0]*L[1]*Pp[0]);
}
