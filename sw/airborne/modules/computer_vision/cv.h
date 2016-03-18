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
 * @file modules/computer_vision/cv.h
 *
 * Computer vision framework for onboard processing
 */


#ifndef CV_H_
#define CV_H_

#include "std.h"
#include "lib/vision/image.h"

// Include opticflow calculator
#include "opticflow/opticflow_calculator.h"

// Needed for settings
extern struct opticflow_t opticflow;

typedef bool_t (*cvFunction)(struct image_t *img);

extern void cv_add(cvFunction func);
extern void cv_init(int img_W, int img_H);
extern void cv_run(struct image_t *img);
extern void cv_periodic(void);

#endif /* CV_H_ */
