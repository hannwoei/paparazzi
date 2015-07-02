/*
 * Copyright (C) 2014 Hann Woei Ho
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
 * @file modules/computer_vision/SSL_module.h
 * @brief optical-flow for self-supervised learning of obstacle appearance
 *
 * Sensors from vertical camera and IMU of Parrot AR.Drone 2.0
 */

#ifndef SSL_MODULE_H
#define SSL_MODULE_H

// Include SSL calculator and landing strategy
#include "opticflow/opticflow_SSL.h"
#include "opticflow/landing_SSL.h"

// Needed for settings
extern struct opticflow_t opticflow;

// Module functions
extern void SSL_module_init(void);
extern void SSL_module_run(void);
extern void SSL_module_start(void);
extern void SSL_module_stop(void);

#endif /* SSL_MODULE_H */
