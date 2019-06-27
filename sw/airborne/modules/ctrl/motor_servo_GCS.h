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

#ifndef MOTOR_SERVO_GCS_H_
#define MOTOR_SERVO_GCS_H_

#include "std.h"

struct MotorServoGCS {
	uint16_t ms_throttle;
	uint16_t ms_throttle2;
	int16_t ms_servo;
	int16_t ms_servo2;
};

extern struct MotorServoGCS motor_servo_ctrl;

#define GUIDANCE_H_MODE_MODULE_SETTING GUIDANCE_H_MODE_ATTITUDE

#define GUIDANCE_V_MODE_MODULE_SETTING GUIDANCE_V_MODE_MODULE

extern void guidance_v_module_init(void);
extern void guidance_v_module_enter(void);
extern void guidance_v_module_run(bool in_flight);

#endif /* MOTOR_SERVO_GCS_H_ */
