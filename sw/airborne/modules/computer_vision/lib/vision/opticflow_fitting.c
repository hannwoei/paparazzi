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

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "opticflow_fitting.h"
#define MIN(x,y) ( (x) < (y) ? (x) : (y) )
#define MAX(x,y) ((x)>(y)?(x):(y))
#define SIGN(a, b) ((b) >= 0.0 ? fabs(a) : -fabs(a))
#define NO_MEMORY -1
#define OK 0

// **********************************************************************************************************************
// Flow Field Fitting
// **********************************************************************************************************************

void MatVVMul(float* MVec, float** Mat, float* Vec, int MatW, int MatH)
{
  unsigned int i;
  unsigned int j;

  for(i = 0; i < MatH; i++)
  {
    for(j = 0; j < MatW; j++)
    {
    	MVec[i] += Mat[i][j] * Vec[j];
    }
  }
}

void ScaleAdd(float* Mat3, float* Mat1, float Scale, float* Mat2, int MatW, int MatH)
{
  unsigned int i;
  unsigned int j;
  unsigned int ii;

  for(i = 0; i < MatW; i++)
  {
    for(j = 0; j < MatH; j++)
    {
      ii = (j * MatW + i);
      Mat3[ii] = Scale*Mat1[ii] + Mat2[ii];
    }
  }
}
static float PYTHAG(float a, float b);
float PYTHAG(float a, float b)
{
    float at = fabs(a), bt = fabs(b), ct, result;

    if (at > bt)       { ct = bt / at; result = at * sqrt(1.0 + ct * ct); }
    else if (bt > 0.0) { ct = at / bt; result = bt * sqrt(1.0 + ct * ct); }
    else result = 0.0;
    return(result);
}

int dsvd(float **a, int m, int n, float *w, float **v)
{
    int flag, i, its, j, jj, k, l, nm;
    float c, f, h, s, x, y, z;
    float anorm = 0.0, g = 0.0, scale = 0.0;
    float *rv1;

    if (m < n)
    {
        fprintf(stderr, "#rows must be > #cols \n");
        return(0);
    }

    rv1 = (float *)malloc((unsigned int) n*sizeof(float));

/* Householder reduction to bidiagonal form */
    for (i = 0; i < n; i++)
    {
        /* left-hand reduction */
        l = i + 1;
        rv1[i] = scale * g;
        g = s = scale = 0.0;
        if (i < m)
        {
            for (k = i; k < m; k++)
                scale += fabs((float)a[k][i]);
            if (scale)
            {
                for (k = i; k < m; k++)
                {
                    a[k][i] = (float)((float)a[k][i]/scale);
                    s += ((float)a[k][i] * (float)a[k][i]);
                }
                f = (float)a[i][i];
                g = -SIGN(sqrt(s), f);
                h = f * g - s;
                a[i][i] = (float)(f - g);
                if (i != n - 1)
                {
                    for (j = l; j < n; j++)
                    {
                        for (s = 0.0, k = i; k < m; k++)
                            s += ((float)a[k][i] * (float)a[k][j]);
                        f = s / h;
                        for (k = i; k < m; k++)
                            a[k][j] += (float)(f * (float)a[k][i]);
                    }
                }
                for (k = i; k < m; k++)
                    a[k][i] = (float)((float)a[k][i]*scale);
            }
        }
        w[i] = (float)(scale * g);

        /* right-hand reduction */
        g = s = scale = 0.0;
        if (i < m && i != n - 1)
        {
            for (k = l; k < n; k++)
                scale += fabs((float)a[i][k]);
            if (scale)
            {
                for (k = l; k < n; k++)
                {
                    a[i][k] = (float)((float)a[i][k]/scale);
                    s += ((float)a[i][k] * (float)a[i][k]);
                }
                f = (float)a[i][l];
                g = -SIGN(sqrt(s), f);
                h = f * g - s;
                a[i][l] = (float)(f - g);
                for (k = l; k < n; k++)
                    rv1[k] = (float)a[i][k] / h;
                if (i != m - 1)
                {
                    for (j = l; j < m; j++)
                    {
                        for (s = 0.0, k = l; k < n; k++)
                            s += ((float)a[j][k] * (float)a[i][k]);
                        for (k = l; k < n; k++)
                            a[j][k] += (float)(s * rv1[k]);
                    }
                }
                for (k = l; k < n; k++)
                    a[i][k] = (float)((float)a[i][k]*scale);
            }
        }
        anorm = MAX(anorm, (fabs((float)w[i]) + fabs(rv1[i])));
    }

    /* accumulate the right-hand transformation */
    for (i = n - 1; i >= 0; i--)
    {
        if (i < n - 1)
        {
            if (g)
            {
                for (j = l; j < n; j++)
                    v[j][i] = (float)(((float)a[i][j] / (float)a[i][l]) / g);
                    /* float division to avoid underflow */
                for (j = l; j < n; j++)
                {
                    for (s = 0.0, k = l; k < n; k++)
                        s += ((float)a[i][k] * (float)v[k][j]);
                    for (k = l; k < n; k++)
                        v[k][j] += (float)(s * (float)v[k][i]);
                }
            }
            for (j = l; j < n; j++)
                v[i][j] = v[j][i] = 0.0;
        }
        v[i][i] = 1.0;
        g = rv1[i];
        l = i;
    }

    /* accumulate the left-hand transformation */
    for (i = n - 1; i >= 0; i--)
    {
        l = i + 1;
        g = (float)w[i];
        if (i < n - 1)
            for (j = l; j < n; j++)
                a[i][j] = 0.0;
        if (g)
        {
            g = 1.0 / g;
            if (i != n - 1)
            {
                for (j = l; j < n; j++)
                {
                    for (s = 0.0, k = l; k < m; k++)
                        s += ((float)a[k][i] * (float)a[k][j]);
                    f = (s / (float)a[i][i]) * g;
                    for (k = i; k < m; k++)
                        a[k][j] += (float)(f * (float)a[k][i]);
                }
            }
            for (j = i; j < m; j++)
                a[j][i] = (float)((float)a[j][i]*g);
        }
        else
        {
            for (j = i; j < m; j++)
                a[j][i] = 0.0;
        }
        ++a[i][i];
    }

    /* diagonalize the bidiagonal form */
    for (k = n - 1; k >= 0; k--)
    {                             /* loop over singular values */
        for (its = 0; its < 30; its++)
        {                         /* loop over allowed iterations */
            flag = 1;
            for (l = k; l >= 0; l--)
            {                     /* test for splitting */
                nm = l - 1;
                if (fabs(rv1[l]) + anorm == anorm)
                {
                    flag = 0;
                    break;
                }
                if (fabs((float)w[nm]) + anorm == anorm)
                    break;
            }
            if (flag)
            {
                c = 0.0;
                s = 1.0;
                for (i = l; i <= k; i++)
                {
                    f = s * rv1[i];
                    if (fabs(f) + anorm != anorm)
                    {
                        g = (float)w[i];
                        h = PYTHAG(f, g);
                        w[i] = (float)h;
                        h = 1.0 / h;
                        c = g * h;
                        s = (- f * h);
                        for (j = 0; j < m; j++)
                        {
                            y = (float)a[j][nm];
                            z = (float)a[j][i];
                            a[j][nm] = (float)(y * c + z * s);
                            a[j][i] = (float)(z * c - y * s);
                        }
                    }
                }
            }
            z = (float)w[k];
            if (l == k)
            {                  /* convergence */
                if (z < 0.0)
                {              /* make singular value nonnegative */
                    w[k] = (float)(-z);
                    for (j = 0; j < n; j++)
                        v[j][k] = (-v[j][k]);
                }
                break;
            }
            if (its >= 30) {
                free((void*) rv1);
                fprintf(stderr, "No convergence after 30,000! iterations \n");
                return(0);
            }

            /* shift from bottom 2 x 2 minor */
            x = (float)w[l];
            nm = k - 1;
            y = (float)w[nm];
            g = rv1[nm];
            h = rv1[k];
            f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
            g = PYTHAG(f, 1.0);
            f = ((x - z) * (x + z) + h * ((y / (f + SIGN(g, f))) - h)) / x;

            /* next QR transformation */
            c = s = 1.0;
            for (j = l; j <= nm; j++)
            {
                i = j + 1;
                g = rv1[i];
                y = (float)w[i];
                h = s * g;
                g = c * g;
                z = PYTHAG(f, h);
                rv1[j] = z;
                c = f / z;
                s = h / z;
                f = x * c + g * s;
                g = g * c - x * s;
                h = y * s;
                y = y * c;
                for (jj = 0; jj < n; jj++)
                {
                    x = (float)v[jj][j];
                    z = (float)v[jj][i];
                    v[jj][j] = (float)(x * c + z * s);
                    v[jj][i] = (float)(z * c - x * s);
                }
                z = PYTHAG(f, h);
                w[j] = (float)z;
                if (z)
                {
                    z = 1.0 / z;
                    c = f * z;
                    s = h * z;
                }
                f = (c * g) + (s * y);
                x = (c * y) - (s * g);
                for (jj = 0; jj < m; jj++)
                {
                    y = (float)a[jj][j];
                    z = (float)a[jj][i];
                    a[jj][j] = (float)(y * c + z * s);
                    a[jj][i] = (float)(z * c - y * s);
                }
            }
            rv1[l] = 0.0;
            rv1[k] = f;
            w[k] = (float)x;
        }
    }
    free((void*) rv1);
    return(1);
}

void svbksb(float **u, float *w, float **v, int m, int n, float *b, float *x)
{
	int jj, j, i;
	float s, *tmp;//, *vector(int nl, int nh);
	//void free_vector();
//	tmp = vector(1,n);

	tmp=(float *)malloc((unsigned int) ((n+1)*sizeof(float)));
	for(j=0; j<n; j++)
	{
		s = 0.0;
		if(w[j])
		{
			for(i=0; i<m; i++)
			{
				s += u[i][j]*b[i];
			}
			s /= w[j];
		}
		tmp[j] = s;
	}
	for(j=0; j<n; j++)
	{
		s = 0.0;
		for(jj=0; jj<n; jj++)
		{
			s += v[j][jj]*tmp[jj];
		}
		x[j] = s;
	}
	free(tmp);
}

void svdSolve(float *x_svd, float **u, int m, int n, float *b)
{
	// SVD
	int i, j;

	float *w, **v, **u_copy, *b_copy;
	w = (float *)malloc((unsigned int) n*sizeof(float));
	v = (float **)malloc((unsigned int) n*sizeof(float*));
	b_copy = (float *)malloc((unsigned int) m*sizeof(float));
	u_copy = (float **)malloc((unsigned int) m*sizeof(float*));
	for(i=0; i<n; i++) v[i] = (float *)malloc(n*sizeof(float));

	int ii, jj;
	for(ii=0; ii<m; ii++)
	{
		u_copy[ii] = (float *)malloc(n*sizeof(float));
		for(jj=0; jj<n; jj++)
		{
			u_copy[ii][jj] = u[ii][jj];
		}
		b_copy[ii] =  b[ii];
		//printf("%d,%f,%f,%f,%f\n",ii,u_copy[ii][0] ,u_copy[ii][1] ,u_copy[ii][2] ,b_copy[ii]);
	}
//printf("svdSolve stop 1\n");
	dsvd(u_copy, m, n, w, v);
	//printf("SVD_DONE = %d\n",SVD_DONE);

/*	for (i=0;i<m;i++)
	{
		for(j=0;j<n;j++)
		{
			printf("%f ",u_copy[i][j]);
		}
		printf("\n");
	}*/

	// LS Solution
	float wmax, wmin;
	wmax = 0.0;
	for(j=0; j<n; j++)
	{
		if(w[j] > wmax)
		{
			wmax = w[j];
		}
	}

	wmin = wmax*1.0e-6;

	for(j=0; j<n; j++)
	{
		if(w[j] < wmin)
		{
			w[j] = 0.0;
		}
	}
//printf("svdSolve stop 2\n");
	svbksb(u_copy, w, v, m, n, b_copy, x_svd);
	for(ii=0; ii<m; ii++) free(u_copy[ii]);
	for(ii=0; ii<n; ii++) free(v[ii]);
	free(w);
	free(v);
	free(u_copy);
	free(b_copy);
//printf("svdSolve stop 3\n");
}

void fitLinearFlowField(float* pu, float* pv, float* min_error_u, float* min_error_v,
		int *n_inlier_minu, int *n_inlier_minv, int *rank_deficient,
		float *x, float *y, float *dx, float *dy, int count, int n_samples, int n_iterations, float error_threshold)
{
//	printf("count=%d, n_sample=%d, n_iterations=%d, error_threshold=%f\n",count,n_samples,n_iterations,error_threshold);
//	for (int i=0; i<count;i++) {
//		printf("%d_%d, ",dx[i],dy[i]);
//	}
//	printf("\n");
		int *sample_indices;
		float **A, *bu, *bv, **AA, *bu_all, *bv_all;
		sample_indices =(int *) calloc(n_samples,sizeof(int));
		A = (float **) calloc(n_samples,sizeof(float*));// A1 is a N x 3 matrix with rows [x, y, 1]
		bu = (float *) calloc(n_samples,sizeof(float)); // bu is a N x 1 vector with elements dx (or dy)
		bv = (float *) calloc(n_samples,sizeof(float)); // bv is a N x 1 vector with elements dx (or dy)
		AA = (float **) calloc(count,sizeof(float*));   // AA contains all points with rows [x, y, 1]
		bu_all = (float *) calloc(count,sizeof(float)); // bu is a N x 1 vector with elements dx (or dy)
		bv_all = (float *) calloc(count,sizeof(float)); // bv is a N x 1 vector with elements dx (or dy)
		int si, add_si, p, i_rand, sam;
		for(sam = 0; sam < n_samples; sam++) A[sam] = (float *) calloc(3,sizeof(float));
		pu[0] = 0.0f; pu[1] = 0.0f; pu[2] = 0.0f;
		pv[0] = 0.0f; pv[1] = 0.0f; pv[2] = 0.0f;
		//		int n_inliers;
		float * PU, * errors_pu, * PV, * errors_pv;
		int * n_inliers_pu, * n_inliers_pv;
		PU = (float *) calloc(n_iterations*3,sizeof(float));
		PV = (float *) calloc(n_iterations*3,sizeof(float));
		errors_pu = (float *) calloc(n_iterations,sizeof(float));
		errors_pv = (float *) calloc(n_iterations,sizeof(float));
		n_inliers_pu = (int *) calloc(n_iterations,sizeof(int));
		n_inliers_pv = (int *) calloc(n_iterations,sizeof(int));

		float *bb, *C;
		bb = (float *) calloc(count,sizeof(float));
		C = (float *) calloc(count,sizeof(float));

		// initialize matrices and vectors for the full point set problem:
		// this is used for determining inliers
		for(sam = 0; sam < count; sam++)
		{
			AA[sam] = (float *) calloc(3,sizeof(float));
			AA[sam][0] = (float) x[sam];
			AA[sam][1] = (float) y[sam];
			AA[sam][2] = 1.0f;
			bu_all[sam] = (float) dx[sam];
			bv_all[sam] = (float) dy[sam];
		}

		int rank = MatRank(AA, count, 3);

		if(rank<3)
		{
//			printf("rank deficient\n");
			*rank_deficient = 1;
			return;
		}

		// perform RANSAC:
		int it, ii;
		for(it = 0; it < n_iterations; it++)
		{
			// select a random sample of n_sample points:
			memset(sample_indices, 0, n_samples*sizeof(int));
			i_rand = 0;

			while(i_rand < n_samples)
			{
				si = rand() % count;
				add_si = 1;
				for(ii = 0; ii < i_rand; ii++)
				{
					if(sample_indices[ii] == si) add_si = 0;
				}
				if(add_si)
				{
					sample_indices[i_rand] = si;
					i_rand ++;
				}
			}

			// Setup the system:
			for(sam = 0; sam < n_samples; sam++)
			{
				A[sam][0] = (float) x[sample_indices[sam]];
				A[sam][1] = (float) y[sample_indices[sam]];
				A[sam][2] = 1.0f;
				bu[sam] = (float) dx[sample_indices[sam]];
				bv[sam] = (float) dy[sample_indices[sam]];
				//printf("%d,%d,%d,%d,%d\n",A[sam][0],A[sam][1],A[sam][2],bu[sam],bv[sam]);
			}

			// Solve the small system:
/*            int i;
			for(i=0;i<3;i++)
			{
				printf("pu%d = %f, pv%d = %f\t",i,pu[i],i,pv[i]);
			}
			printf("\n");*/

			// for horizontal flow:
			svdSolve(pu, A, n_samples, 3, bu);
			PU[it*3] = pu[0];
			PU[it*3+1] = pu[1];
			PU[it*3+2] = pu[2];

			// for vertical flow:
			svdSolve(pv, A, n_samples, 3, bv);
			PV[it*3] = pv[0];
			PV[it*3+1] = pv[1];
			PV[it*3+2] = pv[2];

/*			int i;
			for(i=0;i<3;i++)
			{
				printf("pu%d = %f, pv%d = %f\t",i,pu[i],i,pv[i]);
			}
			printf("\n");*/

			// count inliers and determine their error:
			errors_pu[it] = 0;
			errors_pv[it] = 0;
			n_inliers_pu[it] = 0;
			n_inliers_pv[it] = 0;

			// for horizontal flow:

			MatVVMul(bb, AA, pu, 3, count);
			float scaleM;
			scaleM = -1.0;
			ScaleAdd(C, bb, scaleM, bu_all, 1, count);

			for(p = 0; p < count; p++)
			{
//				printf("h=%f ",C[p]);
				if(abs(C[p]) < error_threshold)
//				if(C[p] < error_threshold)
				{
					errors_pu[it] += abs(C[p]);
					n_inliers_pu[it]++;
				}
			}
			// for vertical flow:
			MatVVMul(bb, AA, pv, 3, count);
			ScaleAdd(C, bb, scaleM, bv_all, 1, count);

//			printf("\n");

			for(p = 0; p < count; p++)
			{
//				printf("v=%f ",C[p]);
				if(abs(C[p]) < error_threshold)
//				if(C[p] < error_threshold)
				{
					errors_pv[it] += abs(C[p]);
					n_inliers_pv[it]++;
				}
			}
//			printf("\n");
		}

		// select the parameters with lowest error/ max inlier:
		// for horizontal flow:
		int param;
		int USE_MIN_ERR = 0;

		int min_ind_u = 0;
		if(USE_MIN_ERR)
		{
			*min_error_u = (float)errors_pu[0];
		}
		else
		{
			*n_inlier_minu = n_inliers_pu[0];
		}

		for(it = 1; it < n_iterations; it++)
		{
			if(USE_MIN_ERR)
			{
				if(errors_pu[it] < *min_error_u)
				{
					*min_error_u = (float)errors_pu[it];
					min_ind_u = it;
				}
			}
			else
			{
				if(n_inliers_pu[it] > *n_inlier_minu)
				{
					*n_inlier_minu = n_inliers_pu[it];
					min_ind_u = it;
				}
			}
		}
		for(param = 0; param < 3; param++)
		{
			pu[param] = PU[min_ind_u*3+param];
		}

		if(USE_MIN_ERR)
		{
			*n_inlier_minu = n_inliers_pu[min_ind_u];
		}

		// for vertical flow:
		int min_ind_v = 0;
		if(USE_MIN_ERR)
		{
			*min_error_v = (float)errors_pv[0];
		}
		else
		{
			*n_inlier_minv = n_inliers_pv[0];
		}

		for(it = 0; it < n_iterations; it++)
		{
			if(USE_MIN_ERR)
			{
				if(errors_pv[it] < *min_error_v)
				{
					*min_error_v = (float)errors_pv[it];
					min_ind_v = it;
				}
			}
			else
			{
				if(n_inliers_pv[it] > *n_inlier_minv)
				{
					*n_inlier_minv = n_inliers_pv[it];
					min_ind_v = it;
				}
			}
		}
		for(param = 0; param < 3; param++)
		{
			pv[param] = PV[min_ind_v*3+param];
		}

		if(USE_MIN_ERR)
		{
			*n_inlier_minv = n_inliers_pv[min_ind_v];
		}

		// error has to be determined on the entire set:
		MatVVMul(bb, AA, pu, 3, count);
		float scaleM;
		scaleM = -1.0;
		ScaleAdd(C, bb, scaleM, bu_all, 1, count);

		*min_error_u = 0;
		for(p = 0; p < count; p++)
		{
			*min_error_u += abs(C[p]);
		}
		MatVVMul(bb, AA, pv, 3, count);
		ScaleAdd(C, bb, scaleM, bv_all, 1, count);

		*min_error_v = 0;
		for(p = 0; p < count; p++)
		{
			*min_error_v += abs(C[p]);
		}

		// delete allocated dynamic arrays

		for(sam = 0; sam < n_samples; sam++) free(A[sam]);
		for(sam = 0; sam < count; sam++) free(AA[sam]);
		free(A);
		free(PU);
		free(PV);
		free(n_inliers_pu);
		free(n_inliers_pv);
		free(errors_pu);
		free(errors_pv);
		free(bu);
		free(bv);
		free(AA);
		free(bu_all);
		free(bv_all);
		free(bb);
		free(C);
		free(sample_indices);
}

void fitQuadFlowField(float* pu, float* pv, float* min_error_u, float* min_error_v,
		int *n_inlier_minu, int *n_inlier_minv, int *rank_deficient,
		float *x, float *y, float *dx, float *dy, int count, int n_samples, int n_iterations, float error_threshold)
{
		int *sample_indices;
		float **Au, **Av, *bu, *bv, **AAu, **AAv, *bu_all, *bv_all;
		sample_indices =(int *) calloc(n_samples,sizeof(int));
		Au = (float **) calloc(n_samples,sizeof(float*));// A1 is a N x 5 matrix with rows [1, x, y, x^2, xy]
		Av = (float **) calloc(n_samples,sizeof(float*));// A2 is a N x 5 matrix with rows [1, x, y, y^2, xy]
		bu = (float *) calloc(n_samples,sizeof(float)); // bu is a N x 1 vector with elements dx (or dy)
		bv = (float *) calloc(n_samples,sizeof(float)); // bv is a N x 1 vector with elements dx (or dy)
		AAu = (float **) calloc(count,sizeof(float*));   // AA1 contains all points with rows [1, x, y, x^2, xy]
		AAv = (float **) calloc(count,sizeof(float*));   // AA2 contains all points with rows [1, x, y, y^2, xy]
		bu_all = (float *) calloc(count,sizeof(float)); // bu is a N x 1 vector with elements dx (or dy)
		bv_all = (float *) calloc(count,sizeof(float)); // bv is a N x 1 vector with elements dx (or dy)
		int si, add_si, p, i_rand, sam;
		for(sam = 0; sam < n_samples; sam++)
		{
			Au[sam] = (float *) calloc(5,sizeof(float));
			Av[sam] = (float *) calloc(5,sizeof(float));
		}
		pu[0] = 0.0f; pu[1] = 0.0f; pu[2] = 0.0f; pu[3] = 0.0f; pu[4] = 0.0f;
		pv[0] = 0.0f; pv[1] = 0.0f; pv[2] = 0.0f; pv[3] = 0.0f; pv[4] = 0.0f;
		//		int n_inliers;
		float * PU, * errors_pu, * PV, * errors_pv;
		int * n_inliers_pu, * n_inliers_pv;
		PU = (float *) calloc(n_iterations*5,sizeof(float));
		PV = (float *) calloc(n_iterations*5,sizeof(float));
		errors_pu = (float *) calloc(n_iterations,sizeof(float));
		errors_pv = (float *) calloc(n_iterations,sizeof(float));
		n_inliers_pu = (int *) calloc(n_iterations,sizeof(int));
		n_inliers_pv = (int *) calloc(n_iterations,sizeof(int));

		float *bb, *C;
		bb = (float *) calloc(count,sizeof(float));
		C = (float *) calloc(count,sizeof(float));

		// initialize matrices and vectors for the full point set problem:
		// this is used for determining inliers
		for(sam = 0; sam < count; sam++)
		{
			AAu[sam] = (float *) calloc(5,sizeof(float));
			AAv[sam] = (float *) calloc(5,sizeof(float));
			AAu[sam][0] = 1.0f;
			AAu[sam][1] = (float) x[sam];
			AAu[sam][2] = (float) y[sam];
			AAu[sam][3] = (float) (x[sam]*x[sam]);
			AAu[sam][4] = (float) (x[sam]*y[sam]);
			AAv[sam][0] = 1.0f;
			AAv[sam][1] = (float) x[sam];
			AAv[sam][2] = (float) y[sam];
			AAv[sam][3] = (float) (y[sam]*y[sam]);
			AAv[sam][4] = (float) (x[sam]*y[sam]);
			bu_all[sam] = (float) dx[sam];
			bv_all[sam] = (float) dy[sam];
		}

		int ranku = MatRank(AAu, count, 5);
		int rankv = MatRank(AAv, count, 5);
		if(ranku<5 || rankv<5)
		{
//			printf("rank deficient\n");
			*rank_deficient = 1;
			return;
		}

		// perform RANSAC:
		int it, ii;
		for(it = 0; it < n_iterations; it++)
		{
			// select a random sample of n_sample points:
			memset(sample_indices, 0, n_samples*sizeof(int));
			i_rand = 0;

			while(i_rand < n_samples)
			{
				si = rand() % count;
				add_si = 1;
				for(ii = 0; ii < i_rand; ii++)
				{
					if(sample_indices[ii] == si) add_si = 0;
				}
				if(add_si)
				{
					sample_indices[i_rand] = si;
					i_rand ++;
				}
			}

			// Setup the system:
			for(sam = 0; sam < n_samples; sam++)
			{
				Au[sam][0] = 1.0f;
				Au[sam][1] = (float) x[sample_indices[sam]];
				Au[sam][2] = (float) y[sample_indices[sam]];
				Au[sam][3] = (float) (x[sample_indices[sam]]*x[sample_indices[sam]]);
				Au[sam][4] = (float) (x[sample_indices[sam]]*y[sample_indices[sam]]);
				Av[sam][0] = 1.0f;
				Av[sam][1] = (float) x[sample_indices[sam]];
				Av[sam][2] = (float) y[sample_indices[sam]];
				Av[sam][3] = (float) (y[sample_indices[sam]]*y[sample_indices[sam]]);
				Av[sam][4] = (float) (x[sample_indices[sam]]*y[sample_indices[sam]]);
				bu[sam] = (float) dx[sample_indices[sam]];
				bv[sam] = (float) dy[sample_indices[sam]];
			}

			// for horizontal flow:
			svdSolve(pu, Au, n_samples, 5, bu);
			PU[it*5] = pu[0];
			PU[it*5+1] = pu[1];
			PU[it*5+2] = pu[2];
			PU[it*5+3] = pu[3];
			PU[it*5+4] = pu[4];

			// for vertical flow:
			svdSolve(pv, Av, n_samples, 5, bv);
			PV[it*5] = pv[0];
			PV[it*5+1] = pv[1];
			PV[it*5+2] = pv[2];
			PV[it*5+3] = pv[3];
			PV[it*5+4] = pv[4];

			// count inliers and determine their error:
			errors_pu[it] = 0;
			errors_pv[it] = 0;
			n_inliers_pu[it] = 0;
			n_inliers_pv[it] = 0;

			// for horizontal flow:
			MatVVMul(bb, AAu, pu, 5, count);
			float scaleM;
			scaleM = -1.0;
			ScaleAdd(C, bb, scaleM, bu_all, 1, count);

			for(p = 0; p < count; p++)
			{
				if(C[p] < error_threshold)
				{
					errors_pu[it] += abs(C[p]);
					n_inliers_pu[it]++;
				}
			}

			// for vertical flow:
			MatVVMul(bb, AAv, pv, 5, count);
			ScaleAdd(C, bb, scaleM, bv_all, 1, count);

			for(p = 0; p < count; p++)
			{
				if(C[p] < error_threshold)
				{
					errors_pv[it] += abs(C[p]);
					n_inliers_pv[it]++;
				}
			}
		}

		// select the parameters with lowest error/ max inlier:
		// for horizontal flow:
		int param;
		int USE_MIN_ERR = 0;

		int min_ind_u = 0;
		if(USE_MIN_ERR)
		{
			*min_error_u = (float)errors_pu[0];
		}
		else
		{
			*n_inlier_minu = n_inliers_pu[0];
		}

		for(it = 1; it < n_iterations; it++)
		{
			if(USE_MIN_ERR)
			{
				if(errors_pu[it] < *min_error_u)
				{
					*min_error_u = (float)errors_pu[it];
					min_ind_u = it;
				}
			}
			else
			{
				if(n_inliers_pu[it] > *n_inlier_minu)
				{
					*n_inlier_minu = n_inliers_pu[it];
					min_ind_u = it;
				}
			}
		}
		for(param = 0; param < 5; param++)
		{
			pu[param] = PU[min_ind_u*5+param];
		}

		if(USE_MIN_ERR)
		{
			*n_inlier_minu = n_inliers_pu[min_ind_u];
		}

		// for vertical flow:
		int min_ind_v = 0;
		if(USE_MIN_ERR)
		{
			*min_error_v = (float)errors_pv[0];
		}
		else
		{
			*n_inlier_minv = n_inliers_pv[0];
		}

		for(it = 0; it < n_iterations; it++)
		{
			if(USE_MIN_ERR)
			{
				if(errors_pv[it] < *min_error_v)
				{
					*min_error_v = (float)errors_pv[it];
					min_ind_v = it;
				}
			}
			else
			{
				if(n_inliers_pv[it] > *n_inlier_minv)
				{
					*n_inlier_minv = n_inliers_pv[it];
					min_ind_v = it;
				}
			}
		}
		for(param = 0; param < 5; param++)
		{
			pv[param] = PV[min_ind_v*5+param];
		}

		if(USE_MIN_ERR)
		{
			*n_inlier_minv = n_inliers_pv[min_ind_v];
		}

		// error has to be determined on the entire set:
		MatVVMul(bb, AAu, pu, 5, count);
		float scaleM;
		scaleM = -1.0;
		ScaleAdd(C, bb, scaleM, bu_all, 1, count);

		*min_error_u = 0;
		for(p = 0; p < count; p++)
		{
			*min_error_u += abs(C[p]);
		}
		MatVVMul(bb, AAv, pv, 5, count);
		ScaleAdd(C, bb, scaleM, bv_all, 1, count);

		*min_error_v = 0;
		for(p = 0; p < count; p++)
		{
			*min_error_v += abs(C[p]);
		}

		// delete allocated dynamic arrays

		for(sam = 0; sam < n_samples; sam++)
		{
			free(Au[sam]);
			free(Av[sam]);
		}
		for(sam = 0; sam < count; sam++)
		{
			free(AAu[sam]);
			free(AAv[sam]);
		}
		free(Au);
		free(Av);
		free(PU);
		free(PV);
		free(n_inliers_pu);
		free(n_inliers_pv);
		free(errors_pu);
		free(errors_pv);
		free(bu);
		free(bv);
		free(AAu);
		free(AAv);
		free(bu_all);
		free(bv_all);
		free(bb);
		free(C);
		free(sample_indices);
}

void quick_sort (float *a, int n)
{
    if (n < 2)
        return;
    float p = a[n / 2];
    float *l = a;
    float *r = a + n - 1;
    while (l <= r)
    {
        if (*l < p)
        {
            l++;
            continue;
        }
        if (*r > p)
        {
            r--;
            continue; // we need to check the condition (l <= r) every time we change the value of l or r
        }
        float t = *l;
        *l++ = *r;
        *r-- = t;
    }
    quick_sort(a, r - a + 1);
    quick_sort(l, a + n - l);
}

unsigned int mov_block = 5; //default: 15
float div_buf[5];
unsigned int div_point = 0;
float OFS_BUTTER_NUM_1 = 0.0004260;
float OFS_BUTTER_NUM_2 = 0.0008519;
float OFS_BUTTER_NUM_3 = 0.0004260;
float OFS_BUTTER_DEN_2 = -1.9408;
float OFS_BUTTER_DEN_3 = 0.9425;
float ofs_meas_dx_prev = 0.0;
float ofs_meas_dx_prev_prev = 0.0;
float ofs_filter_val_dx_prev = 0.0;
float ofs_filter_val_dx_prev_prev = 0.0;
float temp_divergence = 0.0;

void extractInformationFromLinearFlowField(float *flatness, float *divergence, float *TTI, float *d_heading, float *d_pitch,
		float* pu, float* pv, float min_error_u, float min_error_v, int count, float FPS, int imgWidth, int imgHeight)
{

		*flatness = (min_error_u + min_error_v) / (2 * count);

		// divergence:
		*divergence = -(pu[0] + pv[1])*FPS;

		// bound
//		if(*divergence>50.0) *divergence = 50.0;
//		if(*divergence<-50.0) *divergence = -50.0;

		// minimal measurable divergence:
//		float minimal_divergence = 2E-3;
//		if(abs(*divergence) > minimal_divergence)
//		{
//			*mean_tti = 2.0f / *divergence;
////			if(FPS > 1E-3) *mean_tti /= FPS;
////			else *mean_tti = ((2.0f / minimal_divergence) / FPS);
////			if(FPS > 1E-3) *mean_tti /= 60;
////			else *mean_tti = ((2.0f / minimal_divergence) / 60);
//			*median_tti = *mean_tti;
//		}
//		else
//		{
////			*mean_tti = ((2.0f / minimal_divergence) / FPS);
//			*mean_tti = ((2.0f / minimal_divergence) / 60);
//			*median_tti = *mean_tti;
//		}
		if(*divergence!=0)
		{
			*TTI = 2.0f / *divergence;
		}
		else
		{
			*TTI = -1.0;
		}

		// also adjust the divergence to the number of frames:
//		*divergence = *divergence * FPS;
//		*divergence = *divergence * 60;

		// translation orthogonal to the camera axis:
		// flow in the center of the image:
		*d_heading = (-(pu[2] + (imgWidth/2.0f) * pu[0] + (imgHeight/2.0f) * pu[1]));
		*d_pitch = (-(pv[2] + (imgWidth/2.0f) * pv[0] + (imgHeight/2.0f) * pv[1]));

		//apply a moving average
		int medianfilter = 1;
		int averagefilter = 0;
		int butterworthfilter = 0;
		float div_avg = 0.0f;

		if(averagefilter == 1)
		{
			if (*divergence < 3.0 && *divergence > -3.0) {
				div_buf[div_point] = *divergence;
				div_point = (div_point+1) %mov_block; // index starts from 0 to mov_block
			}

			int im;
			for (im=0;im<mov_block;im++) {
				div_avg+=div_buf[im];
			}
			*divergence = div_avg/ mov_block;
		}
		else if(medianfilter == 1)
		{
			//apply a median filter
//			if (*divergence < 3.0 && *divergence > -3.0) {
				div_buf[div_point] = *divergence;
				div_point = (div_point+1) %5;
//			}
			quick_sort(div_buf,5);
			*divergence  = div_buf[2];
		}
		else if(butterworthfilter == 1)
		{
			temp_divergence = *divergence;
			*divergence = OFS_BUTTER_NUM_1* (*divergence) + OFS_BUTTER_NUM_2*ofs_meas_dx_prev+ OFS_BUTTER_NUM_3*ofs_meas_dx_prev_prev- OFS_BUTTER_DEN_2*ofs_filter_val_dx_prev- OFS_BUTTER_DEN_3*ofs_filter_val_dx_prev_prev;
		    ofs_meas_dx_prev_prev = ofs_meas_dx_prev;
		    ofs_meas_dx_prev = temp_divergence;
		    ofs_filter_val_dx_prev_prev = ofs_filter_val_dx_prev;
		    ofs_filter_val_dx_prev = *divergence;
		}
		else
		{

		}
}

void extractInformationFromQuadFlowField(float *z_x, float *z_y, float *flatness, float *divergence, float *TTI, float *d_heading, float *d_pitch,
		float* puq, float* pvq, float min_error_u, float min_error_v, int count, float FPS, int imgWidth, int imgHeight)
{
	float threshold_flow_translational = 1.0;
	float threshold_rel_vert = 0.0;
	float center_x = (imgWidth/2.0f);
	float center_y = (imgHeight/2.0f);
	float rel_vert;

	// u = pu[0] + pu[1] * x + pu[2] * y + pu[3] * x^2 + pu[4] * xy
	// v = pv[0] + pv[1] * x + pv[2] * y + pv[3] * y^2 + pv[4] * xy

	*d_heading = -(puq[0] + center_x * puq[1] + center_y * puq[2] + center_x * center_x * puq[3] + center_x * center_y * puq[4]);
	*d_pitch = -(pvq[0] + center_x * pvq[1] + center_y * pvq[2] + center_y * center_y * pvq[3] + center_x * center_y * pvq[4]);

	*flatness = (min_error_u + min_error_v) / (2 * count);

	// Estimate z_x:
	int invalid_zx = 1;
    if(abs(*d_pitch) > threshold_flow_translational)
	{
        // estimate with the help of translational motion:
        // zx = vx / v0
        *z_x = (pvq[1] + center_y * pvq[4]) / (*d_pitch);
        invalid_zx = 0;
	}
    else
	{
        // Vy close to zero, so this equation:
        // rel_vert = dv/dy - Vy * Zyq
        // now becomes independent of Zyq.
        rel_vert = pvq[2] + 2 * pvq[3] * center_y + pvq[4] * center_x;

        if(abs(rel_vert) > threshold_rel_vert)
		{
            // there is considerable forward motion:
            *z_x = (-pvq[4] / rel_vert + -puq[3] / rel_vert) / 2.0f;
            invalid_zx = 0;
		}
        else
		{
            // there is little forward motion, but perhaps there is
            // horizontal motion:
            if(abs(*d_heading) > threshold_flow_translational)
			{
                *z_x = (puq[1] + 2 * puq[3] * center_x + puq[4] * center_y) / (*d_heading);
                invalid_zx = 0;
			}
            else
			{
				// not enough motion to estimate slope:
                *z_x = 0;
			}
		}
	}


    // estimate with the help of translational motion:
    int invalid_zy = 1;
    if(abs(*d_heading) > threshold_flow_translational)
	{
        *z_y = (puq[2] + puq[4] * center_x) / (*d_heading);
        invalid_zy = 0;
	}
    else
	{
        // *d_heading close to zero, so this equation:
        // rel_vert = du/dx + *d_heading * Zxq
        // now becomes independent of Zxq.
        rel_vert = puq[1] + 2 * puq[3] * center_x + puq[4] * center_y;

        if(abs(rel_vert) > threshold_rel_vert)
		{
            // there is considerable forward motion:
            *z_y = (puq[4] / rel_vert -pvq[3] / rel_vert) / 2.0f;
            invalid_zy = 0;
		}
        else
		{
            // there is little forward motion, but perhaps there is
            // vertical motion:
            if(abs(*d_pitch) > threshold_flow_translational)
			{
                *z_y = (pvq[2] + 2 * center_y * pvq[3] + pvq[4] * center_x) / (*d_pitch);
                invalid_zy = 0;
			}
            else
			{
                *z_y = 0;
			}
		}
	}

	float ux = puq[1] + 2 * puq[3] * (imgWidth/2.0) + puq[4] * (imgHeight/2.0);
	float vy = pvq[2] + 2 * pvq[3] * (imgHeight/2) + pvq[4] * (imgWidth/2);
	*divergence = -((ux)+(vy))/2.0*FPS;
	*TTI = 1.0/(*divergence);

    if(invalid_zx) *z_x = 0.0;
    if(invalid_zy) *z_y = 0.0;

//    // we now have no less than 6 equations to solve for the relative
//    // vertical velocity:
//    rel_vert = pvq[2] - (*d_pitch) * (*z_y);
//    rel_vert += -puq[1] + (*d_heading) *(*z_x);
//    rel_vert += puq[4] / (*z_y);
//    rel_vert += -pvq[3] / (*z_y);
//    rel_vert += -pvq[4] / (*z_x);
//    rel_vert += -puq[3] / (*z_x);
//    rel_vert /= 6;

//	// outputs:
//    *mean_tti = -1/rel_vert;
//    *divergence = 2.0f / *mean_tti;
//	v_prop_z = rel_vert;
}

void slopeEstimation(float *z_x, float *z_y, float *flatness, float *POE_x, float *POE_y, float d_heading, float d_pitch, float* pu, float* pv, float min_error_u, float min_error_v)
{
	float v_prop_x, v_prop_y, threshold_slope, eta;

	// extract proportional velocities / inclination from flow field:
	v_prop_x  = d_heading;
	v_prop_y = d_pitch;
	threshold_slope = 1.0;
	eta = 0.002;
	if(abs(pv[1]) < eta && abs(v_prop_y) < threshold_slope && abs(v_prop_x) >= 2* threshold_slope)
	{
		// there is not enough vertical motion, but also no forward motion:
		*z_x = pu[0] / v_prop_x;
	}
	else if(abs(v_prop_y) >= 2 * threshold_slope)
	{
		// there is sufficient vertical motion:
		*z_x = pv[0] / v_prop_y;
	}
	else
	{
		// there may be forward motion, then we can do a quadratic fit:
		*z_x = 0.0f;
	}

	*flatness = min_error_v + min_error_u;

	if(abs(pu[0]) < eta && abs(v_prop_x) < threshold_slope && abs(v_prop_y) >= 2*threshold_slope)
	{
		// there is little horizontal movement, but also no forward motion, and sufficient vertical motion:
		*z_y = pv[1] / v_prop_y;
	}
	else if(abs(v_prop_x) >= 2*threshold_slope)
	{
		// there is sufficient horizontal motion:
		*z_y = pu[1] / v_prop_x;
	}
	else
	{
		// there could be forward motion, then we can do a quadratic fit:
		*z_y = 0.0f;
	}

	// Focus of Expansion:
	// the flow planes intersect the flow=0 plane in a line
	// the FoE is the point where these 2 lines intersect (flow = (0,0))
	// x:
	float denominator = pv[0]*pu[1] - pu[0]*pv[1];
	if(abs(denominator) > 1E-5)
	{
		*POE_x = ((pu[2]*pv[1] - pv[2] * pu[1]) / denominator);
	}
	else *POE_x = 0.0f;
	// y:
	denominator = pu[1];
	if(abs(denominator) > 1E-5)
	{
		*POE_y = (-(pu[0] * *POE_x + pu[2]) / denominator);
	}
	else *POE_y = 0.0f;
}

void analyseTTI(float *pu, float *pv, float *z_x, float *z_y, float *flatness, float *divergence,
		float *TTI, float *d_heading, float *d_pitch,
		int *n_inlier_minu, int *n_inlier_minv, float *min_error_u, float *min_error_v, int *FIT_UNCERTAINTY,
		struct flow_t *vectors, int count, int subpixel_factor, float FPS, int imW, int imH, int USE_LINEAR_FIT)
{
		// new data
		float *x, *y, *dx, *dy;
		x = (float *) calloc (count,sizeof(float));
		y = (float *) calloc (count,sizeof(float));
		dx = (float *) calloc (count,sizeof(float));
		dy = (float *) calloc (count,sizeof(float));

		for (int i = 0; i < count; i++)
		{
			x[i] = (float) vectors[i].pos.x/ subpixel_factor;
			y[i] = (float) vectors[i].pos.y/ subpixel_factor;
			dx[i] = (float) vectors[i].flow_x/ subpixel_factor;
			dy[i] = (float) vectors[i].flow_y/ subpixel_factor;
		}

		// linear fit of the optic flow field
		float error_threshold = 0.5; // 10
		int n_iterations = 30; // 20
		int min_samples;

		// minimum = 3 for linear, and 5 for quadratic
		if(USE_LINEAR_FIT)
		{
			min_samples = 3;
		}
		else
		{
			min_samples = 5;
		}

		int n_samples = (count < min_samples) ? count : min_samples;
		*FIT_UNCERTAINTY = 0;

		if(n_samples < min_samples)
		{
			*z_x = 0.0;
			*z_y = 0.0;
			*flatness = 0.0;
			*divergence = 0.0;
			*TTI = 0.0;
			*d_heading = 0.0;
			*d_pitch = 0.0;
			*n_inlier_minu = 0;
			*n_inlier_minv = 0;
			*min_error_u = 0.0;
			*min_error_v = 0.0;
			*FIT_UNCERTAINTY = 1;
//			printf("Not enough features\n");
			return;
		}

		int rank_deficient=0;

		if(USE_LINEAR_FIT)
		{
			fitLinearFlowField(pu, pv, min_error_u, min_error_v, n_inlier_minu, n_inlier_minv, &rank_deficient,
					x, y, dx, dy, count, n_samples, n_iterations, error_threshold);

			if(rank_deficient)
			{
				*z_x = 0.0;
				*z_y = 0.0;
				*flatness = 0.0;
				*divergence = 0.0;
				*TTI = 0.0;
				*d_heading = 0.0;
				*d_pitch = 0.0;
				*n_inlier_minu = 0;
				*n_inlier_minv = 0;
				*min_error_u = 0.0;
				*min_error_v = 0.0;
				*FIT_UNCERTAINTY = 1;
//				printf("Rank Deficient\n");
			}
			else
			{
				extractInformationFromLinearFlowField(flatness, divergence, TTI, d_heading, d_pitch,
						pu, pv, *min_error_u, *min_error_v, count, FPS, imW, imH);
			}
		}
		else
		{
			fitQuadFlowField(pu, pv, min_error_u, min_error_v, n_inlier_minu, n_inlier_minv, &rank_deficient,
					x, y, dx, dy, count, n_samples, n_iterations, error_threshold);
							//			printf("pu = {%f %f %f %f %f}, pv ={ %f %f %f %f %f}\n",pu[0],pu[1],pu[2],pu[3],pu[4],pv[0],pv[1],pv[2],pv[3],pv[4]);

			if(rank_deficient)
			{
				*z_x = 0.0;
				*z_y = 0.0;
				*flatness = 0.0;
				*divergence = 0.0;
				*TTI = 0.0;
				*d_heading = 0.0;
				*d_pitch = 0.0;
				*n_inlier_minu = 0;
				*n_inlier_minv = 0;
				*min_error_u = 0.0;
				*min_error_v = 0.0;
				*FIT_UNCERTAINTY = 1;
//				printf("Rank Deficient\n");
			}
			else
			{
				extractInformationFromQuadFlowField(z_x, z_y, flatness, divergence, TTI,  d_heading, d_pitch, pu, pv, *min_error_u, *min_error_v, count, FPS, imW, imH);
			}
		}

		// delete data
		free(x);
		free(y);
		free(dx);
		free(dy);
}

void MatSwap(float **mat, int r1, int r2, int c)
{
	int i;
	float temp;
	for( i = 0; i < c; i++)
	{
		temp = mat[r1][i];
		mat[r1][i] = mat[r2][i];
		mat[r2][i] = temp;
	}
}

int MatRank(float **mat, int row, int col)
{
    int r, c, i;
    for(r = 0; r< col; r++)
    {
        if( mat[r][r] )  // Diagonal element is not zero
	{
        	for(c = 0; c < row; c++)
		{
			if(c != r)
			{
				/* Make all the elements above and below the current principal
			 	diagonal element zero */

				float ratio = mat[c][r]/ mat[r][r];
				for( i = 0; i < col; i++)
			    		mat[c][i] -= ratio * mat[r][i];
			}
		}
	}
                /* Principal Diagonal elment is zero */
        else
        {
		for(c =  r+1 ; c < row;  c++)
		{
			if (mat[c][r])
			{
				/*  Find non zero elements in the same column */
				MatSwap(mat,r,c,col);
				break ;
			}
		}
		if(c == row)
		{
			-- col;

			for(c = 0; c < row; c ++)
				mat[c][r] = mat[c][col];
		}
		--r;
        }
    }
    return col;
}

void saveSingleImageDataFile(struct image_t *input, int width, int height, char filename[100])
{

	uint8_t *frame_buf = (uint8_t *)input->buf;
	FILE *fp;

	fp=fopen(filename, "w");

	// convert to grayscale image (verified)
//	unsigned char *grayframe;
//	grayframe = (unsigned char*) calloc(width*height,sizeof(unsigned char));
//	CvtYUYV2Gray(grayframe, frame_buf, width, height);

	// convert to rgb
//	unsigned char *RGB;
//	RGB = (unsigned char *)calloc(width*height*3,sizeof(unsigned char));
//	unsigned char *grayframe;
//	grayframe = (unsigned char*) calloc(width*height,sizeof(unsigned char));
//	YUV422TORGB(frame_buf, RGB, grayframe, width, height);
//	uyvy_to_rgb24 (width, height, frame_buf, RGB);
	if(fp == NULL)
	{
		perror("Error while opening the file.\n");
	}
	else
	{
		for(int i = 0; i<height; i++)
		{
	//		for(int j = 0; j<width*3; j++) //for RGB
			for(int j = 0; j<width*2; j++) // for UYVY
	//		for(int j = 0; j<width; j++)   // for grayscale
			{
	//			fprintf(fp, "%u\n",RGB[i * width * 3 + j]); // for RGB
				fprintf(fp, "%u\n",frame_buf[i * width * 2 + j]); // for UYVY
	//			fprintf(fp, "%u\n",grayframe[i * width + j]); // use "mat2gray()" to convert it to grayscale image and then show it with "imshow()"
			}
		}
		fclose(fp);
	}
//	free(grayframe);
//	free(RGB);
}

/*
 * Compute mean of an array
 */
float CalcMean(float *arr, uint8_t size_arr)
{
	float mean_arr = 0.0;
	for(uint8_t i=0; i<size_arr; i++)
	{
		mean_arr += arr[i];
	}
	mean_arr /= size_arr;
	return mean_arr;
}

/*
 * Compute covariances of two arrays
 */
float CalcCov(float *arr1, float *arr2, uint8_t size_arr)
{
	float mean_arr1 = CalcMean(arr1,size_arr);
	float mean_arr2 = CalcMean(arr2,size_arr);
	float cov_arr = 0.0;
	for(uint8_t i=0; i<size_arr; i++)
	{
		cov_arr += (arr1[i]-mean_arr1)*(arr2[i]-mean_arr2);
	}
	cov_arr /= size_arr;
	return cov_arr;
}
