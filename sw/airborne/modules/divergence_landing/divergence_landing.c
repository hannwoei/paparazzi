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
#define VISION_CONTROLLER 2
#endif
PRINT_CONFIG_VAR(VISION_CONTROLLER)

#ifndef VISION_LP_ALPHA
#define VISION_LP_ALPHA 0.6
#endif
PRINT_CONFIG_VAR(VISION_LP_ALPHA)

#ifndef VISION_T_INTERVAL
#define VISION_T_INTERVAL 2.0
#endif
PRINT_CONFIG_VAR(VISION_T_INTERVAL)

#ifndef COV_WIN_SIZE
#define COV_WIN_SIZE 30
#endif
PRINT_CONFIG_VAR(COV_WIN_SIZE)

#ifndef VISION_DELAY_STEP
#define VISION_DELAY_STEP 15
#endif
PRINT_CONFIG_VAR(VISION_DELAY_STEP)

#ifndef VISION_COV_THRESHOLD
#define VISION_COV_THRESHOLD 0.005
#endif
PRINT_CONFIG_VAR(VISION_COV_THRESHOLD)

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

// Vision
float div_update, fb_cmd, curr_height, t_interval;
int message_count, previous_count, i_init;
uint32_t prev_stamp, curr_stamp;

// Oscillation detection
float div_hist[COV_WIN_SIZE], prev_div_hist[COV_WIN_SIZE], pgain_init, igain_init, trim_init;
int64_t i_hist;
int8_t cov_trigger, trim_landing, landing_method, restart_init;

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
		  &Div_landing.div_pgain, &Div_landing.z_sum_err, &Div_landing.cov_div,
		  &Div_landing.desired_div, &Div_landing.nominal_throttle, &Div_landing.controller,
		  &Div_landing.agl, &Div_landing.gps_z, &Div_landing.vel_z, &Div_landing.accel_z,
		  &Div_landing.div_igain, &Div_landing.err_Z, &Div_landing.div_f, &Div_landing.ground_div,
		  &stabilization_cmd[COMMAND_THRUST], &Div_landing.thrust, &fb_cmd,
		  &Div_landing.fps, &Div_landing.stamp);
}
#endif

void divergence_landing_init(void);
void divergence_landing_run(bool_t in_flight);

void divergence_landing_init(void)
{
	curr_height = 0.0;
	div_update = 0.0;
	fb_cmd = 0.0;
	message_count = 1;
	previous_count = 0;

	i_init = 0;

//	start_timer();
	prev_stamp = 0;
	curr_stamp = 0;
	t_interval = 0.0;

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
	Div_landing.err_Z = 0.0f;
	Div_landing.err_Vz = 0.0f;
	Div_landing.z_sum_err = 0.0f;
	Div_landing.thrust = 0;
	Div_landing.fps = 0.0f;
	Div_landing.stamp = 0.0f;
	Div_landing.t_interval_sp = VISION_T_INTERVAL;
	Div_landing.delay_step = VISION_DELAY_STEP;
	Div_landing.cov_div = 0.0;
	Div_landing.cov_thres = VISION_COV_THRESHOLD;

	// Oscillation detection
	i_hist = 0;
	for(int8_t i = 0; i < COV_WIN_SIZE; i++) {
	  div_hist[i] = 0.0;
	  prev_div_hist[i] = 0.0;
	}
	cov_trigger = 0; trim_landing = 0; trim_init = 0.0;
	pgain_init = Div_landing.div_pgain;
	igain_init = Div_landing.div_igain;
	landing_method = 0; restart_init = 0;

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

	if(message_count != previous_count)
	{
		// **********************************************************************************************************************
		// timestamp of messages
		// **********************************************************************************************************************
		if(Div_landing.fps != 0.0)
		{
			Div_landing.stamp = 1.0/Div_landing.fps;
		}
		else
		{
			Div_landing.stamp = 0.0;
		}

		// skip the first timestamp
		if(i_init == 0)
		{
			curr_height = Div_landing.agl;
			Div_landing.stamp = 0.0;
			i_init ++;
			return;
		}

		t_interval += Div_landing.stamp;

		if(t_interval > 10.0 && Div_landing.controller != 1)
		{
			t_interval = 0.0;
		}

		// **********************************************************************************************************************
		// Vision Correction and filter
		// **********************************************************************************************************************
		div_update = Div_landing.div*1.28;
		if(fabs(div_update - Div_landing.div_f) > 0.20) {
			if(div_update < Div_landing.div_f) div_update = Div_landing.div_f - 0.10f;
			else div_update = Div_landing.div_f + 0.10f;
		}
		Div_landing.div_f = Div_landing.div_f*Div_landing.alpha + div_update*(1.0f - Div_landing.alpha);

		// **********************************************************************************************************************
		// Feedback Controller
		// **********************************************************************************************************************
		// trim thrust
	    int32_t nominal_throttle = Div_landing.nominal_throttle * MAX_PPRZ;

	    // 1. tracking a divergence setpoint
	    if(Div_landing.controller == 1)
	    {
	    	Div_landing.err_Z = -(Div_landing.desired_div - Div_landing.div_f);
	    }
	    // 2. hovering with divergence and small increment of the gains
	    else if(Div_landing.controller == 2)
	    {
	    	Div_landing.err_Z = Div_landing.div_f;
			Div_landing.div_pgain = Div_landing.div_pgain *1.001;
			Div_landing.div_igain = Div_landing.div_igain *1.001;
	    }
	    // 0. hovering with height control
	    else
	    {
	    	Div_landing.err_Z = curr_height - Div_landing.agl;
	    }

	    Div_landing.z_sum_err += Div_landing.err_Z*Div_landing.stamp;

	    fb_cmd = (Div_landing.div_pgain * Div_landing.err_Z) + (Div_landing.div_igain * Div_landing.z_sum_err);
	    Div_landing.thrust = nominal_throttle + fb_cmd* MAX_PPRZ;

		// **********************************************************************************************************************
		// Oscillation detection & re-configure the gains
		// **********************************************************************************************************************
		div_hist[i_hist%COV_WIN_SIZE] = Div_landing.div_f;
		int64_t i_prev = (i_hist%COV_WIN_SIZE) - Div_landing.delay_step;
		if(i_prev < 0) i_prev += COV_WIN_SIZE;
		float prev_div = div_hist[i_prev];
		prev_div_hist[i_hist%COV_WIN_SIZE] = prev_div;
		i_hist++;

		if(i_hist >= COV_WIN_SIZE*4) {
			Div_landing.cov_div = get_cov(prev_div_hist, div_hist, COV_WIN_SIZE);
		}
		// Go back to previous gain and start landing
		if(i_hist >= COV_WIN_SIZE && fabs(Div_landing.cov_div) > Div_landing.cov_thres && Div_landing.controller == 2)
		{
			Div_landing.controller = 1;
			landing_method = 2;
			Div_landing.div_pgain = Div_landing.div_pgain *(1-0.001);
			Div_landing.div_igain = Div_landing.div_igain *(1-0.001);
		}

		if(landing_method == 1)
		{
			// landing method 1: whole exponential decay
			if(i_hist >= COV_WIN_SIZE && fabs(Div_landing.cov_div) > Div_landing.cov_thres && cov_trigger == 0)
			{
				t_interval = 0.0;
				cov_trigger = 1;
				pgain_init = Div_landing.div_pgain;
				igain_init = Div_landing.div_igain;
			}

			if(cov_trigger == 1)
			{
				Div_landing.div_pgain = pgain_init*exp(-Div_landing.desired_div*t_interval);
				Div_landing.div_igain = igain_init*exp(-Div_landing.desired_div*t_interval);
				fb_cmd = (Div_landing.div_pgain * Div_landing.err_Z) + (Div_landing.div_igain * Div_landing.z_sum_err);
				Div_landing.thrust = nominal_throttle + fb_cmd* MAX_PPRZ;
			}
		}
		else if(landing_method == 2)
		{
			// landing method 2: decrease and restart when oscillate
			if(i_hist >= COV_WIN_SIZE && fabs(Div_landing.cov_div) > Div_landing.cov_thres && restart_init == 0)
			{
				t_interval = 0.0;
				pgain_init = Div_landing.div_pgain;
				igain_init = Div_landing.div_igain;
				restart_init = 1;
				cov_trigger = 1;
			}

			if(cov_trigger == 1)
			{
				Div_landing.div_pgain = pgain_init*exp(-Div_landing.desired_div*t_interval);
				Div_landing.div_igain = igain_init*exp(-Div_landing.desired_div*t_interval);
				fb_cmd = (Div_landing.div_pgain * Div_landing.err_Z) + (Div_landing.div_igain * Div_landing.z_sum_err);
				Div_landing.thrust = nominal_throttle + fb_cmd* MAX_PPRZ;

				if(fabs(Div_landing.cov_div) < Div_landing.cov_thres)
				{
					restart_init = 0;
				}
			}
		}
		else
		{
			// landing method 3: decrease when oscillate
		}

		Bound(Div_landing.thrust, 0, MAX_PPRZ);
	    stabilization_cmd[COMMAND_THRUST] = Div_landing.thrust;

		previous_count = message_count;
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
//  printf("t=%d\n",stamp);
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

float get_mean_array(float *a, int n_elements)
{
	// determine the mean for the vector:
	float mean = 0;
	for(unsigned int i = 0; i < n_elements; i++)
	{
		mean += a[i];
	}
	mean /= n_elements;

	return mean;
}

float get_cov(float* a, float* b, int n_elements)
{
	// determine means for each vector:
	float mean_a = get_mean_array(a, n_elements);
	float mean_b = get_mean_array(b, n_elements);
	float cov = 0;
	for(unsigned int i = 0; i < n_elements; i++)
	{
		cov += (a[i] - mean_a) * (b[i] - mean_b);
	}

	cov /= n_elements;

	return cov;
}
