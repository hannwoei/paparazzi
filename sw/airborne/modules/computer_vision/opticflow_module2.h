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
 * @file modules/computer_vision/opticflow_module2.h
 * @brief optical-flow calculation for Parrot Drones
 *
 */

#ifndef OPTICFLOW2_MODULE_H
#define OPTICFLOW2_MODULE_H

// Include opticflow calculator
#include "opticflow/opticflow_calculator2.h"

// Needed for settings
extern struct opticflow2_t opticflow2;

// Module functions
extern void opticflow2_module_init(void);
extern void opticflow2_module_run(void);
extern void opticflow2_module_start(void);
extern void opticflow2_module_stop(void);

#endif /* OPTICFLOW2_MODULE_H */
