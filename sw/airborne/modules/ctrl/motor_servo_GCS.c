/*
 * Copyright (C) 2019 HO HANN WOEI
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
 * @file modules/ctrl/motor_servo_GCS.h
 * @brief Control 2 motors and 2 servo from GCS
 *
 *
 */

#include "motor_servo_GCS.h"

#include "generated/airframe.h"
#include "paparazzi.h"
#include "subsystems/abi.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/guidance/guidance_v_adapt.h"

#include "autopilot.h"
#include "subsystems/navigation/common_flight_plan.h"
#include "subsystems/datalink/telemetry.h"
#include "subsystems/actuators.h"

#ifndef MS_THROTTLE
#define MS_THROTTLE 0
#endif

#ifndef MS_THROTTLE2
#define MS_THROTTLE2 0
#endif

#ifndef MS_MIN_THROTTLE
#define MS_MIN_THROTTLE 1
#endif

#ifndef MS_SERVO
#define MS_SERVO 0
#endif

#ifndef MS_SERVO2
#define MS_SERVO2 0
#endif

struct MotorServoGCS motor_servo_ctrl;

static void send_msthrottle(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_MSTHROTTLE(trans, dev, AC_ID,
                           &(motor_servo_ctrl.ms_throttle), &(motor_servo_ctrl.ms_throttle2), &(motor_servo_ctrl.ms_servo), &(motor_servo_ctrl.ms_servo2));
}

void ms_ctrl_module_init(void);
void ms_ctrl_module_run(void);


void ms_ctrl_module_init(void)
{
  motor_servo_ctrl.ms_throttle = MS_THROTTLE;
  motor_servo_ctrl.ms_throttle2 = MS_THROTTLE2;
  motor_servo_ctrl.ms_servo = MS_SERVO;
  motor_servo_ctrl.ms_servo2 = MS_SERVO2;
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_MSTHROTTLE, send_msthrottle);
}

void ms_ctrl_module_run(void)
{
	  Bound(motor_servo_ctrl.ms_throttle, MS_MIN_THROTTLE, MAX_PPRZ);
	  actuators_pprz[0] = motor_servo_ctrl.ms_throttle;

	  Bound(motor_servo_ctrl.ms_throttle2, MS_MIN_THROTTLE, MAX_PPRZ);
	  actuators_pprz[1] = motor_servo_ctrl.ms_throttle2;

      Bound(motor_servo_ctrl.ms_servo, MIN_PPRZ, MAX_PPRZ);
      actuators_pprz[2] = motor_servo_ctrl.ms_servo;

      Bound(motor_servo_ctrl.ms_servo2, MIN_PPRZ, MAX_PPRZ);
      actuators_pprz[3] = motor_servo_ctrl.ms_servo2;
}

void guidance_v_module_init(void)
{
  ms_ctrl_module_init();
}

void guidance_v_module_enter(void)
{

}

void guidance_v_module_run(bool in_flight)
{
  ms_ctrl_module_run();
}
