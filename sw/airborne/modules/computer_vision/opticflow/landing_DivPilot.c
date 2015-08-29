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
 * @file modules/computer_vision/opticflow/landing_DivPilot.c
 * @brief Optical-flow based control for Linux based systems
 *
 * Control loops for optic flow based landing.
 */

// Own Header
#include "landing_DivPilot.h"

// Stabilization
#include "firmwares/rotorcraft/stabilization.h"
//#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "autopilot.h"
#include "subsystems/datalink/downlink.h"
#include "lib/vision/opticflow_fitting.h"

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

// run_hover
#ifndef GUIDANCE_V_MIN_ERR_Z
#define GUIDANCE_V_MIN_ERR_Z POS_BFP_OF_REAL(-10.)
#endif

#ifndef GUIDANCE_V_MAX_ERR_Z
#define GUIDANCE_V_MAX_ERR_Z POS_BFP_OF_REAL(10.)
#endif

#ifndef GUIDANCE_V_MIN_ERR_ZD
#define GUIDANCE_V_MIN_ERR_ZD SPEED_BFP_OF_REAL(-10.)
#endif

#ifndef GUIDANCE_V_MAX_ERR_ZD
#define GUIDANCE_V_MAX_ERR_ZD SPEED_BFP_OF_REAL(10.)
#endif

#ifndef GUIDANCE_V_MAX_SUM_ERR
#define GUIDANCE_V_MAX_SUM_ERR 2000000
#endif

#define FF_CMD_FRAC 18

// run update
/* Saturations definition */
#ifndef GUIDANCE_V_REF_MIN_ZDD
#define GUIDANCE_V_REF_MIN_ZDD (-2.0*9.81)
#endif
#define GV_MIN_ZDD BFP_OF_REAL(GUIDANCE_V_REF_MIN_ZDD, GV_ZDD_REF_FRAC)

#ifndef GUIDANCE_V_REF_MAX_ZDD
#define GUIDANCE_V_REF_MAX_ZDD ( 0.8*9.81)
#endif
#define GV_MAX_ZDD BFP_OF_REAL(GUIDANCE_V_REF_MAX_ZDD, GV_ZDD_REF_FRAC)

/** maximum distance altitude setpoint is advanced in climb mode */
#ifndef GUIDANCE_V_REF_MAX_Z_DIFF
#define GUIDANCE_V_REF_MAX_Z_DIFF 2.0
#endif
#define GV_MAX_Z_DIFF BFP_OF_REAL(GUIDANCE_V_REF_MAX_Z_DIFF, GV_Z_REF_FRAC)

#ifndef GUIDANCE_V_REF_MIN_ZD
#define GUIDANCE_V_REF_MIN_ZD (-3.)
#endif

#ifndef GUIDANCE_V_REF_MAX_ZD
#define GUIDANCE_V_REF_MAX_ZD ( 3.)
#endif
#define GV_MIN_ZD  BFP_OF_REAL(GUIDANCE_V_REF_MIN_ZD , GV_ZD_REF_FRAC)
#define GV_MAX_ZD  BFP_OF_REAL(GUIDANCE_V_REF_MAX_ZD , GV_ZD_REF_FRAC)

/* first order time constant */
#define GV_REF_INV_THAU_FRAC 16
#define GV_REF_INV_THAU  BFP_OF_REAL((1./0.25), GV_REF_INV_THAU_FRAC)

#define GV_ZDD_REF_FRAC 8
#define GV_FREQ_FRAC 9

/** number of bits for the fractional part of #gv_zd_ref */
#define GV_ZD_REF_FRAC (GV_ZDD_REF_FRAC + GV_FREQ_FRAC)

/** number of bits for the fractional part of #gv_z_ref */
#define GV_Z_REF_FRAC (GV_ZD_REF_FRAC + GV_FREQ_FRAC)

static void gv_update_ref_from_div(int32_t zd_sp, int32_t z_pos);
static void run_landing_loop(int32_t pos, int32_t speed);
static int32_t get_vertical_thrust_coeff2(void);
static void HeightEKT(float *Z, float *Vz, float *innov, float *P, float u, float div, float FPS);

/* Check the control gains */
#if (VISION_DIV_PGAIN < 0)      ||  \
	(VISION_DIV_DGAIN < 0)      ||  \
    (VISION_DIV_IGAIN < 0)
#error "ALL control gains have to be positive!!!"
#endif

/* Initialize the default gains and settings */
struct DivPilot_landing_t DivPilot_landing = {
  .div_pgain = VISION_DIV_PGAIN,
  .div_igain = VISION_DIV_IGAIN,
  .div_dgain = VISION_DIV_DGAIN,
  .desired_div = VISION_DESIRED_DIV,
  .nominal_throttle = VISION_NOMINAL_THROTTLE,
  .controller = VISION_CONTROLLER,
  .div_cov = 0.0
};

/**
 * Horizontal guidance mode enter resets the errors
 * and starts the controller.
 */

#define NO_BUF 10
float buf_div[NO_BUF], buf_thrust[NO_BUF];
uint8_t n_buf;

float div_OT;
int64_t div_z_ref;
int32_t div_zd_ref;
int32_t div_zdd_ref;
int32_t divergence_v_z_ref, divergence_v_zd_ref, divergence_v_zdd_ref, divergence_v_z_sum_err,
divergence_v_ff_cmd, divergence_v_fb_cmd, divergence_v_delta_t, divergence_v_thrust_coeff;
int32_t curr_speed, curr_pos, div_sp;
int32_t err_z, err_zd;
int32_t kd_update, min_kd, max_kd, count_update;
int8_t gain_search;

// Height Estimation using EKF
float P_EKF[4], Z_EKF, Vz_EKF, innov_EKF;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_div_cmd(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_DIV_CMD(trans, dev, AC_ID,
		  	  	  	  	  &divergence_v_z_ref,
						  &divergence_v_zd_ref,
						  &divergence_v_zdd_ref,
                          &div_sp,
                          &curr_pos,
						  &(stateGetSpeedNed_i()->z),
                          &(stateGetAccelNed_i()->z),
						  &curr_speed,
                          &divergence_v_z_sum_err,
                          &divergence_v_ff_cmd,
                          &divergence_v_fb_cmd,
                          &divergence_v_delta_t,
						  &err_z,
						  &err_zd,
						  &div_OT,
						  &DivPilot_landing.div_pgain,
						  &DivPilot_landing.div_dgain,
						  &DivPilot_landing.div_igain,
						  &Z_EKF,
						  &Vz_EKF,
						  &innov_EKF,
						  &DivPilot_landing.div_cov);
}
#endif

void guidance_v_module_enter(void)
{
  /* ref for hover */
  div_z_ref = (((int64_t) curr_pos) << (GV_Z_REF_FRAC - INT32_POS_FRAC));
  div_zd_ref = 0;
  div_zdd_ref = 0;

  /* initialize parameters*/
  div_sp = SPEED_BFP_OF_REAL(DivPilot_landing.desired_div);
  divergence_v_z_sum_err = 0;
  div_OT = 0.0;
  n_buf = 0;
  kd_update = 80;
  min_kd = 20;
  max_kd = 300;
  gain_search = 1;
  count_update = 0;

  // Height estimation using EKF
  Z_EKF = 1.0;
  Vz_EKF = -0.01;
  innov_EKF = 0.0;
  P_EKF[0] = 1000; P_EKF[1] = 0; P_EKF[2] = 0; P_EKF[3] = 100;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, "DIV_CMD", send_div_cmd);
#endif
}

/**
 * Read the RC commands
 */
void guidance_v_module_read_rc(void)
{
  // TODO: change the desired vx/vy
}

/**
 * Main guidance loop
 * @param[in] in_flight Whether we are in flight or not
 */

void guidance_v_module_run(bool_t in_flight)
{
	divergence_v_thrust_coeff = get_vertical_thrust_coeff2();

	gv_update_ref_from_div(div_sp, curr_pos);
	run_landing_loop(curr_pos, curr_speed);

	stabilization_cmd[COMMAND_THRUST] = divergence_v_delta_t;
}

/**
 * Update the controls based on a vision result
 * @param[in] *result The opticflow calculation result used for control
 */


void landing_DivPilot_update(struct opticflow_result_t *result,  struct opticflow_state_t *opticflow_state)
{
	/* get states*/
//	curr_pos = stateGetPositionNed_i()->z;
	curr_pos = -POS_BFP_OF_REAL(opticflow_state->agl);

	/* Check if we are in the correct AP_MODE before setting commands */
	if (autopilot_mode != AP_MODE_MODULE) {
		return;
	}

	/* compute ground divergence */
	if(stateGetPositionNed_f()->z!=0)
	{
		div_OT = -2.0*stateGetSpeedNed_f()->z/stateGetPositionNed_f()->z;
	}
	else
	{
		div_OT = 0.0;
	}

	/* use normally landing when the height is below 10cm */
	if(curr_pos>-POS_BFP_OF_REAL(0.1))
	{
		DivPilot_landing.div_pgain = VISION_DIV_PGAIN;
		DivPilot_landing.div_dgain = VISION_DIV_DGAIN;
		DivPilot_landing.div_igain = 0; // avoid oscillation

		/* get speed */
		curr_speed = stateGetSpeedNed_i()->z;
	}
	else
	{
		/* disable position control during landing */
		if(DivPilot_landing.controller == 2)
		{
			DivPilot_landing.div_igain = 0;
			DivPilot_landing.div_pgain = 0;
			div_sp = SPEED_BFP_OF_REAL(DivPilot_landing.desired_div);
		}
		else
		{
			div_sp = 0;
		}
		DivPilot_landing.div_dgain = kd_update;

		/* get divergence */
		curr_speed = -(result->Div_f)*(1<<(INT32_SPEED_FRAC));
//		curr_speed = -(result->divergence)*(1<<(INT32_SPEED_FRAC));
//		curr_speed = div_OT*(1<<(INT32_SPEED_FRAC));
//		curr_speed = stateGetSpeedNed_i()->z;
	}

	/* sinuoidal signal for controlling climb rate */
//	if(curr_pos>-POS_BFP_OF_REAL(1))
//	{
//		div_sp = -SPEED_BFP_OF_REAL(0.7);
//	}
//	if(curr_pos<-POS_BFP_OF_REAL(4))
//	{
//		div_sp = SPEED_BFP_OF_REAL(0.7);
//	}

	/* detect oscillation*/
	buf_div[n_buf] = result->Div_f;
	buf_thrust[n_buf] = (float) divergence_v_delta_t;
	DivPilot_landing.div_cov = CalcCov(buf_div,buf_thrust,n_buf+1);
	n_buf = (n_buf+1) %NO_BUF;

	if(DivPilot_landing.controller != 0)
	{
		if (DivPilot_landing.div_cov < -25.0)
		{
			if(gain_search == 1)
			{
				kd_update = kd_update - 10;
				gain_search = 0;
				count_update = 0;
			}

			if(DivPilot_landing.controller == 2 && kd_update > min_kd)
			{
//				count_update += 1;
//				if(count_update > 20)
//				{
					kd_update = kd_update - 10;
//				}
			}
		}
		else
		{
			if(gain_search == 1)
			{
				count_update += 1;
				if(count_update > 60)
				{
					kd_update = kd_update + 10;
					count_update = 0;
				}
			}
		}
	}

	// **********************************************************************************************************************
	// Height Estimation using EKF
	// **********************************************************************************************************************
//	float u;
//	u = (float) divergence_v_fb_cmd/(1<<9);
//	HeightEKT(&Z_EKF, &Vz_EKF, &innov_EKF, P_EKF, u, result->Div_f, result->fps);
//	HeightEKT(&Z_EKF, &Vz_EKF, &innov_EKF, P_EKF, u, -div_OT, result->fps);
}

static void gv_update_ref_from_div(int32_t zd_sp, int32_t z_pos)
{
	div_z_ref  += div_zd_ref;
	div_zd_ref += div_zdd_ref;

	/* limit z_ref to GUIDANCE_V_REF_MAX_Z_DIFF from current z pos */
	int64_t cur_z = ((int64_t)z_pos) << (GV_Z_REF_FRAC - INT32_POS_FRAC);
	Bound(div_z_ref, cur_z - GV_MAX_Z_DIFF, cur_z + GV_MAX_Z_DIFF);

	int32_t zd_err = div_zd_ref - (zd_sp >> (INT32_SPEED_FRAC - GV_ZD_REF_FRAC));
	int32_t zd_err_zdd_res = zd_err >> (GV_ZD_REF_FRAC - GV_ZDD_REF_FRAC);
	div_zdd_ref = (-(int32_t)GV_REF_INV_THAU * zd_err_zdd_res) >> GV_REF_INV_THAU_FRAC;

	/* Saturate accelerations */
	Bound(div_zdd_ref, GV_MIN_ZDD, GV_MAX_ZDD);

	/* Saturate speed and adjust acceleration accordingly */
	if (div_zd_ref <= GV_MIN_ZD) {
		div_zd_ref = GV_MIN_ZD;
		if (div_zdd_ref < 0) {
			div_zdd_ref = 0;
		}
	} else if (div_zd_ref >= GV_MAX_ZD) {
		div_zd_ref = GV_MAX_ZD;
		if (div_zdd_ref > 0) {
			div_zdd_ref = 0;
		}
	}
}
#define MAX_PPRZ 9600

static void run_landing_loop(int32_t pos, int32_t speed)
{
	/* convert our reference to generic representation */
	int64_t tmp  = div_z_ref >> (GV_Z_REF_FRAC - INT32_POS_FRAC);
	divergence_v_z_ref = (int32_t)tmp;
	divergence_v_zd_ref = div_zd_ref << (INT32_SPEED_FRAC - GV_ZD_REF_FRAC);
	divergence_v_zdd_ref = div_zdd_ref << (INT32_ACCEL_FRAC - GV_ZDD_REF_FRAC);

	/* compute the error to our reference */
	err_z  = divergence_v_z_ref - pos;
	Bound(err_z, GUIDANCE_V_MIN_ERR_Z, GUIDANCE_V_MAX_ERR_Z);
	err_zd = divergence_v_zd_ref - speed;
	Bound(err_zd, GUIDANCE_V_MIN_ERR_ZD, GUIDANCE_V_MAX_ERR_ZD);

	divergence_v_z_sum_err += err_z;
	Bound(divergence_v_z_sum_err, -GUIDANCE_V_MAX_SUM_ERR, GUIDANCE_V_MAX_SUM_ERR);


	/* our nominal command : (g + zdd)*m   */
	int32_t inv_m;

	/* use the fixed nominal throttle */
	inv_m = BFP_OF_REAL(9.81 / (DivPilot_landing.nominal_throttle * MAX_PPRZ), FF_CMD_FRAC);

	const int32_t g_m_zdd = (int32_t)BFP_OF_REAL(9.81, FF_CMD_FRAC);
//	                          - (divergence_v_zdd_ref << (FF_CMD_FRAC - INT32_ACCEL_FRAC));

	divergence_v_ff_cmd = g_m_zdd / inv_m;
	/* feed forward command */
	divergence_v_ff_cmd = (divergence_v_ff_cmd << INT32_TRIG_FRAC) / divergence_v_thrust_coeff;

	/* bound the nominal command to 0.9*MAX_PPRZ */
	Bound(divergence_v_ff_cmd, 0, 8640);

	/* our error feed back command                   */
	/* z-axis pointing down -> positive error means we need less thrust */
	divergence_v_fb_cmd = ((-DivPilot_landing.div_pgain * err_z)  >> 7) + ((-DivPilot_landing.div_dgain * err_zd) >> 16) +
		  ((-DivPilot_landing.div_igain * divergence_v_z_sum_err) >> 16);

	divergence_v_delta_t = divergence_v_ff_cmd + divergence_v_fb_cmd;

	/* bound the result */
	Bound(divergence_v_delta_t, 0, MAX_PPRZ);
}

/// get the cosine of the angle between thrust vector and gravity vector
static int32_t get_vertical_thrust_coeff2(void)
{
	static const int32_t max_bank_coef = BFP_OF_REAL(RadOfDeg(30.), INT32_TRIG_FRAC);

	struct Int32RMat *att = stateGetNedToBodyRMat_i();
	/* thrust vector:
	*  int32_rmat_vmult(&thrust_vect, &att, &zaxis)
	* same as last colum of rmat with INT32_TRIG_FRAC
	* struct Int32Vect thrust_vect = {att.m[2], att.m[5], att.m[8]};
	*
	* Angle between two vectors v1 and v2:
	*  angle = acos(dot(v1, v2) / (norm(v1) * norm(v2)))
	* since here both are already of unit length:
	*  angle = acos(dot(v1, v2))
	* since we we want the cosine of the angle we simply need
	*  thrust_coeff = dot(v1, v2)
	* also can be simplified considering: v1 is zaxis with (0,0,1)
	*  dot(v1, v2) = v1.z * v2.z = v2.z
	*/
	int32_t coef = att->m[8];
	if (coef < max_bank_coef) {
	coef = max_bank_coef;
	}
	return coef;
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

//void Mat22Multi(float c[2][2], float a[2][2], float b[2][2])
//{
//	for (int i = 0; i < 2; i++)
//	{
//		for (int j = 0; j < 2; j++)
//		{
//			 float sum = 0.0;
//			 for (int k = 0; k < 2; k++)
//			 {
//				sum = sum + a[i][k] * b[k][j];
//			 }
//			 c[i][j] = sum;
//		}
//	}
//}
//
//void Mat22transp(float c[2][2], float a[2][2])
//{
//	for (int i = 0; i < 2; i++)
//	{
//	   for(int j = 0 ; j < 2 ; j++)
//	   {
//		   c[j][i] = a[i][j];
//	   }
//	}
//}

