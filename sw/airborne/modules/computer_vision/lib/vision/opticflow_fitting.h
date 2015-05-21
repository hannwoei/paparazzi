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
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/*
 * @file paparazzi/sw/airborne/modules/computer_vision/lib/vision/opticflow_fitting.h
 * @brief optical-flow field fitting
 *
 * Sensors from vertical camera and IMU of Parrot AR.Drone 2.0
 */

#ifndef OPTIC
#define OPTIC
#include "lib/vision/image.h"

void MatVVMul(float* MVec, float** Mat, float* Vec, int MatW, int MatH);
void ScaleAdd(float* Mat3, float* Mat1, float Scale, float* Mat2, int MatW, int MatH);
int dsvd(float **a, int m, int n, float *w, float **v);
void svbksb(float **u, float *w, float **v, int m, int n, float *b, float *x);
void svdSolve(float *x_svd, float **u, int m, int n, float *b);
void fitLinearFlowField(float* pu, float* pv, float* min_error_u, float* min_error_v,
		int *n_inlier_minu, int *n_inlier_minv, int *rank_deficient,
		float *x, float *y, float *dx, float *dy, int count, int n_samples, int n_iterations, float error_threshold);
void fitQuadFlowField(float* pu, float* pv, float* min_error_u, float* min_error_v,
		int *n_inlier_minu, int *n_inlier_minv, int *rank_deficient,
		float *x, float *y, float *dx, float *dy, int count, int n_samples, int n_iterations, float error_threshold);
void extractInformationFromLinearFlowField(float *flatness, float *divergence, float *TTI, float *d_heading, float *d_pitch,
		float* pu, float* pv, float min_error_u, float min_error_v, int count, int imgWidth, int imgHeight);
void extractInformationFromQuadFlowField(float *z_x, float *z_y, float *flatness, float *divergence, float *TTI, float *d_heading, float *d_pitch,
		float* puq, float* pvq, float min_error_u, float min_error_v, int count, int imgWidth, int imgHeight);
void slopeEstimation(float *z_x, float *z_y, float *flatness, float *POE_x, float *POE_y, float d_heading, float d_pitch, float* pu, float* pv, float min_error_u, float min_error_v);
void analyseTTI(float *pu, float *pv, float *z_x, float *z_y, float *flatness, float *divergence,
		float *TTI, float *d_heading, float *d_pitch,
		int *n_inlier_minu, int *n_inlier_minv, float *min_error_u, float *min_error_v, int *FIT_UNCERTAINTY,
		struct flow_t *vectors, int count, int subpixel_factor, int imW, int imH, int USE_LINEAR_FIT);
int MatRank(float **mat, int row, int col);
void MatSwap(float **mat, int r1, int r2, int c);
#endif
