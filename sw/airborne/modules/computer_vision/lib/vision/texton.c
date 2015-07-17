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
 * @file paparazzi/sw/airborne/modules/computer_vision/lib/vision/texton.c
 * @brief texton distribution
 *
 * Sensors from vertical camera and IMU of Parrot AR.Drone 2.0
 */

#include <stdlib.h>
#include <stdio.h>
//#include <math.h>
//#include <string.h>
#include "texton.h"

/** Set the default Dictionary path to the USB drive */
#ifndef DICTIONARY_PATH
#define DICTIONARY_PATH /data/video/
#endif

/** The file pointer */
static FILE *dictionary_logger = NULL;
static FILE *model_logger = NULL;

// *********************************************
// Dictionary Training and Texton Extraction
// *********************************************
/** Input Parameters
 *  ================
 *  input: image
 *  dictionary_ready: 0. train a new dictionary 1. dictionary is ready 2. dictionary is loaded
 *  load_dictionary: 1. load a dictionary
 *  alpha: update rate
 **/
void SSL_Texton(float *flatness_SSL, float ****dictionary, float *word_distribution, float *LinearMap,
		struct image_t *input,
		uint8_t *dictionary_ready, uint8_t *load_dictionary, uint8_t *load_model, float alpha, uint8_t n_words, uint8_t patch_size, uint32_t n_samples,
		uint32_t *learned_samples, uint32_t n_samples_image, uint8_t *filled, uint8_t RANDOM_SAMPLES, uint32_t border_width,
		uint32_t border_height)
{
	uint8_t *frame = (uint8_t *)input->buf;

	if(*dictionary_ready == 0 && *load_dictionary == 0)
	{
		DictionaryTrainingYUV(dictionary, frame, n_words, patch_size, learned_samples, n_samples_image, alpha,
				input->w, input->h, filled);
		if(*learned_samples >= n_samples)
		{
//			printf("Done !!!\n");
			*dictionary_ready = 1;

			// lower learning rate
			alpha = 0.0;

			//save a dictionary
			uint8_t counter = 0;
			char filename[512];

			// Check for available files
			sprintf(filename, "%s/Dictionary_%05d.dat", STRINGIFY(DICTIONARY_PATH), counter);
			while ((dictionary_logger = fopen(filename, "r")))
			{
				fclose(dictionary_logger);

				counter++;
				sprintf(filename, "%s/Dictionary_%05d.dat", STRINGIFY(DICTIONARY_PATH), counter);
			}

			dictionary_logger = fopen(filename, "w");

			if(dictionary_logger == NULL)
			{
				perror("Error while opening the file.\n");
			}
			else
			{
				// delete dictionary
				for(uint8_t i = 0; i < n_words; i++)
				{
					for(uint8_t j = 0; j < patch_size;j++)
					{
						for(uint8_t k = 0; k < patch_size; k++)
						{
							fprintf(dictionary_logger, "%f\n",dictionary[i][j][k][0]);
							fprintf(dictionary_logger, "%f\n",dictionary[i][j][k][1]);
						}
					}
				}
				fclose(dictionary_logger);
			}
		}
	}

	if(*dictionary_ready == 1 || *load_dictionary == 1)
	{
//		printf("Loading\n");
		//load a dictionary
		uint8_t counter = 0;
		char filename[512];

		// Check for first 5 Dictionary files
		sprintf(filename, "%s/Dictionary_%05d.dat", STRINGIFY(DICTIONARY_PATH), counter);

		while (counter<5)
		{
			if((dictionary_logger = fopen(filename, "r")))
			{
				for(int i = 0; i < n_words; i++)
				{
					for(int j = 0; j < patch_size;j++)
					{
						for(int k = 0; k < patch_size; k++)
						{
							if(fscanf(dictionary_logger, "%f\n", &dictionary[i][j][k][0]) == EOF) break;
							if(fscanf(dictionary_logger, "%f\n", &dictionary[i][j][k][1]) == EOF) break;
						}
					}
				}
				fclose(dictionary_logger);
				*dictionary_ready = 2;
				break;
			}
			else
			{
				counter++;
				sprintf(filename, "%s/Dictionary_%05d.dat", STRINGIFY(DICTIONARY_PATH), counter);
			}
		}
	}

	if(*dictionary_ready == 2)
	{
//		printf("Extracting\n");
		DistributionExtraction(dictionary, frame, word_distribution, n_words, patch_size, n_samples_image, RANDOM_SAMPLES,
				input->w, input->h, border_width, border_height);

		if(*load_model == 1)
		{
			//load a model
			uint8_t counter = 0;
			char filename[512];

			// Check for first 5 Dictionary files
			sprintf(filename, "%s/LinearMap_%05d.dat", STRINGIFY(DICTIONARY_PATH), counter);

			while (counter<5)
			{
				if((model_logger = fopen(filename, "r")))
				{
					for(int i = 0; i < n_words+1; i++)
					{
						if(fscanf(model_logger, "%f\n", &LinearMap[i]) == EOF) break;
					}
					fclose(model_logger);
					break;
				}
				else
				{
					counter++;
					sprintf(filename, "%s/LinearMap_%05d.dat", STRINGIFY(DICTIONARY_PATH), counter);
				}
			}
			*load_model = 0;
		}

		/*
		 * Dictionary:
		 * Location:
		 */
//		*flatness_SSL = 2294.5
//				+ word_distribution[0]*(-2145.1) + word_distribution[1]*(583.7) + word_distribution[2]*(-2166.2) + word_distribution[3]*(1624.4) + word_distribution[4]*(1012)
//				+ word_distribution[5]*(-2860.2) + word_distribution[6]*(-790.6) + word_distribution[7]*(5165.1) + word_distribution[8]*(226.8) + word_distribution[9]*(-3481.1)
//				+ word_distribution[10]*(-1765.4) + word_distribution[11]*(-3412) + word_distribution[12]*(-1787.9) + word_distribution[13]*(-2698.2) + word_distribution[14]*(67.6)
//				+ word_distribution[15]*(-6578.4) + word_distribution[16]*(-2859.1) + word_distribution[17]*(-8525.5) + word_distribution[18]*(4740.1) + word_distribution[19]*(37066.1)
//				+ word_distribution[20]*(-2455.2) + word_distribution[21]*(-459) + word_distribution[22]*(8448.4) + word_distribution[23]*(-2449) + word_distribution[24]*(-1939.3)
//				+ word_distribution[25]*(-482.3) + word_distribution[26]*(-6793.7) + word_distribution[27]*(552.6) + word_distribution[28]*(-2314.5) + word_distribution[29]*(-1205.5);
//		*flatness_SSL = 91.4758175317669
//				+ word_distribution[0]*(129.357361232921) + word_distribution[1]*(-641.491873893695) + word_distribution[2]*(77.2734585281937) + word_distribution[3]*(-29.9900469831447) + word_distribution[4]*(789.515148155958)
//				+ word_distribution[5]*(1254.94150199362) + word_distribution[6]*(370.037883269326) + word_distribution[7]*(265.984201205625) + word_distribution[8]*(49.7583117986851) + word_distribution[9]*(822.776729536144)
//				+ word_distribution[10]*(1139.60762360203) + word_distribution[11]*(-1665.33975802238) + word_distribution[12]*(571.507584720294) + word_distribution[13]*(1563.54129647641) + word_distribution[14]*(-340.047442298264)
//				+ word_distribution[15]*(-566.618658650005) + word_distribution[16]*(516.447108259413) + word_distribution[17]*(-1531.81703260333) + word_distribution[18]*(-997.874692217590) + word_distribution[19]*(-822.953250053615)
//				+ word_distribution[20]*(-918.588189508986) + word_distribution[21]*(371.505728609769) + word_distribution[22]*(261.424387906646) + word_distribution[23]*(-63.3673731800899) + word_distribution[24]*(62.6078564312148)
//				+ word_distribution[25]*(-18.7211516319433) + word_distribution[26]*(162.941563750614) + word_distribution[27]*(-514.504213274695) + word_distribution[28]*(-156.997391098565) + word_distribution[29]*(55.5512625758230);

		*flatness_SSL = LinearMap[0];
		for(int i=0; i<n_words; i++)
		{
			*flatness_SSL += LinearMap[i+1]*word_distribution[i];
		}
	}
}

// **********************************************************************************************************************
// Dictionary Training for YUV images
// **********************************************************************************************************************

void DictionaryTrainingYUV(float ****color_words, uint8_t *frame, uint8_t n_words, uint8_t patch_size, uint32_t *learned_samples,
		uint32_t n_samples_image, float alpha, uint16_t Width, uint16_t Height, uint8_t *filled)
{
	int i, j, w, s, word, c; //loop variables
	int x,y;
	float error_word;
	int n_clusters = n_words;
	int clustered_ps = patch_size; // use even number fo YUV image

	uint8_t *buf;
//	buf = NULL;
	// ************************
	//       LEARNING
	// ************************
	//printf("Learning\n");
	if(!(*filled))
	{
		// **************
		// INITIALISATION
		// **************

		// we fix the values, so that the user cannot change them anymore with the control of the module.
//		printf("n_words = %d, patch_size = %d, n_samples = %d\n", n_words, patch_size, n_samples);

		// in the first image, we fill the neighbours/ words with the present patches
		for(w = 0; w < n_words; w++)
		{
			// select a coordinate
			x = rand() % (Width - clustered_ps);
			y = rand() % (Height - clustered_ps);

			// create a word
//			float *** c_word;
//			c_word = (float ***)calloc(clustered_ps,sizeof(float**));
//
//			for(i = 0; i < clustered_ps; i++)
//			{
//				c_word[i] = (float **)calloc(clustered_ps,sizeof(float *));
//
//				for(j = 0; j < clustered_ps;j++)
//				{
//					c_word[i][j] = (float *)calloc(3,sizeof(float));
//				}
//			}
//			int i_frame = 0;

			// take the sample
			for(i = 0; i < clustered_ps; i++)
			{
				buf = frame + (Width * 2 * (i+y)) + 2*x;
				for(j = 0; j < clustered_ps; j++)
				{
					// put it in a word
					// U/V component
//					c_word[i][j][1]
					color_words[w][i][j][0] = (float) *buf;
					// Y1/Y2 component
//					c_word[i][j][0]
					buf += 1;
					color_words[w][i][j][1] = (float) *buf;
					buf += 1;
//					printf("%f %f ",color_words[w][i][j][0],color_words[w][i][j][1]);
//					printf("%u %u ",frame[(Width * 2 * (i+y)) + 2*(x + j)],frame[(Width * 2 * (i+y)) + 2*(x + j ) + 1]);
//					printf("%u",frame[i_frame++]);
				}
			}
//			printf("\n");
		}
		*filled = 1;
	}
	else
	{
//		printf("Learning: samples = %d\n",*learned_samples);
		float *word_distances, ***p;
		word_distances = (float *)calloc(n_clusters,sizeof(float));
		p = (float ***)calloc(clustered_ps,sizeof(float**));

		for(i = 0; i < clustered_ps; i++)
		{
			p[i] = (float **)calloc(clustered_ps,sizeof(float*));
			for(j = 0; j < clustered_ps;j++)
			{
				p[i][j] = (float *)calloc(2,sizeof(float));
			}
		}

		for(s = 0; s < n_samples_image; s++)
		{
			// select a random sample from the image
			x = rand() % (Width - clustered_ps);
			y = rand() % (Height - clustered_ps);

			// reset word_distances
			for(word = 0; word < n_clusters; word++)
			{
				word_distances[word] = 0;
			}
			// extract sample
			for(i = 0; i < clustered_ps; i++)
			{
				buf = frame + (Width * 2 * (i+y)) + 2*x;
				for(j = 0; j < clustered_ps; j++)
				{
					// U/V componentt8
		        	p[i][j][0] = (float) *buf;
					// Y1/Y2 component
					buf += 1;
					p[i][j][1] = (float) *buf;
					buf += 1;
				}
			}

			// determine distances to the words:
			for(i = 0; i < clustered_ps; i++)
			{
				for(j = 0; j < clustered_ps; j++)
				{
					for(c = 0; c < 2; c++)
					{
						// determine the distance to words
						for(word = 0; word < n_clusters; word++)
						{
							word_distances[word] += (p[i][j][c] - color_words[word][i][j][c])
													* (p[i][j][c] - color_words[word][i][j][c]);
						}
					}
				}
			}
			// determine the nearest neighbour
			// search the closest centroid
			int assignment = 0;
			float min_dist = word_distances[0];
			for(word = 1; word < n_clusters; word++)
			{
				if(word_distances[word] < min_dist)
				{
					min_dist = word_distances[word];
					assignment = word;
				}
			}

			// move the neighbour closer to the input
			for(i = 0; i < clustered_ps; i++)
			{
				for(j = 0; j < clustered_ps; j++)
				{
					for(c = 0; c < 2; c++)
					{
						error_word = p[i][j][c] - color_words[assignment][i][j][c];
						color_words[assignment][i][j][c] += (alpha * error_word);
					}
				}
			}
			*learned_samples = *learned_samples + 1;
		}

		for(i = 0; i < clustered_ps; i++)
		{
			for(j = 0; j < clustered_ps; j++)
			{
                free(p[i][j]);
			}
			free(p[i]);
		}
		free(p);
		free(word_distances);
	}
	buf = NULL;
	free(buf);

//	printf("gathered samples = %d / %d.\n", learned_samples, N_SAMPLES);

}

// **********************************************************************************************************************
// Distribution Extraction for YUV images
// **********************************************************************************************************************

void DistributionExtraction(float ****color_words, uint8_t *frame, float* word_distribution, uint8_t n_words,
		uint8_t patch_size, uint32_t n_samples_image, uint8_t RANDOM_SAMPLES, uint16_t Width, uint16_t Height,
		uint32_t border_width, uint32_t border_height)
{
	int i, j, s, word, c; //loop variables
	int x, y;
	int n_clusters = n_words;
	int clustered_ps = patch_size; // use even number fo YUV image
	int n_extracted_words = 0;
	int FULL_SAMPLING;

	uint8_t *buf;

	if(RANDOM_SAMPLES == 1)
	{
		FULL_SAMPLING = 0;
	}
	else
	{
		FULL_SAMPLING = 1;
	}
	// ************************
	//       EXECUTION
	// ************************
//	printf("Execution\n");

	float *word_distances, ***p;
	word_distances = (float *)calloc(n_clusters,sizeof(float));
	p = (float ***)calloc(clustered_ps,sizeof(float**));

	for(i = 0; i < clustered_ps; i++)
	{
		p[i] = (float **)calloc(clustered_ps,sizeof(float*));
		for(j = 0; j < clustered_ps;j++)
		{
			p[i][j] = (float *)calloc(2,sizeof(float));
		}
	}

	int finished = 0;
	x = 0;
	y = 0;
	s = 0;
	while(!finished)
	{
		if(RANDOM_SAMPLES)
		{
			s++;
			x = border_width + rand() % (Width - clustered_ps - 2*border_width);
			y = border_height + rand() % (Height - clustered_ps - 2*border_height);
		}
		// FASTER: What if we determine the closest word while updating the distances at the last pixel?
		// reset word_distances
		for(word = 0; word < n_clusters; word++)
		{
			word_distances[word] = 0;
		}
		// extract sample
		for(i = 0; i < clustered_ps; i++)
		{
			buf = frame + (Width * 2 * (i+y)) + 2*x;
			for(j = 0; j < clustered_ps; j++)
			{
				// U/V component
	        	p[i][j][0] = (float) *buf;
				// Y1/Y2 component
				buf += 1;
				p[i][j][1] = (float) *buf;
				buf += 1;
			}
		}

		// The following comparison with all words in the dictionary can be made faster by:
		// a) not comparing all pixels, but stopping early
		// b) using a different distance measure such as L1
		// determine distances:
		for(i = 0; i < clustered_ps; i++)
		{
			for(j = 0; j < clustered_ps; j++)
			{
				for(c = 0; c < 2; c++)
				{
					// determine the distance to words
					for(word = 0; word < n_clusters; word++)
					{
						word_distances[word] += (p[i][j][c] - color_words[word][i][j][c])
												* (p[i][j][c] - color_words[word][i][j][c]);
					}
				}
			}
		}

		// determine the nearest neighbour
		// search the closest centroid
		int assignment = 0;
		float min_dist = word_distances[0];
		for(word = 1; word < n_clusters; word++)
		{
			if(word_distances[word] < min_dist)
			{
				min_dist = word_distances[word];
				assignment = word;
			}
		}

		// put the assignment in the histogram
		word_distribution[assignment]++;

		n_extracted_words++;

		if(RANDOM_SAMPLES)
		{
			if(s == n_samples_image)
			{
				finished = 1;
			}
		}
		else
		{
			if(!FULL_SAMPLING)
				y += clustered_ps;
			else
				y++;

			if(y > Height - clustered_ps)
			{
				if(!FULL_SAMPLING)
					x += clustered_ps;
				else
					x++;
				y = 0;
			}
			if(x > Width - clustered_ps)
			{
				finished = 1;
			}
		}
	} // sampling

	// Normalize distribution:
	// can be made faster by not determining max, min, and avg (only needed for visualization)
	float max_p = 0;
	float min_p = 1;
	float avg_p = 0;
	for(i = 0; i < n_clusters; i++)
	{
		word_distribution[i] = word_distribution[i] / (float) n_extracted_words;
		if(word_distribution[i] > max_p) max_p = word_distribution[i];
		if(word_distribution[i] < min_p) min_p = word_distribution[i];
		avg_p += word_distribution[i];
	}
	avg_p = avg_p /(float) n_clusters;

//	if(FULL_SAMPLING)
//	{
//		FILE *myfile;
//		// if file left empty by user, fill it.
//		char full_filename[255];
//		strcpy(full_filename,"./full_sampling.dat");
//
//		myfile = fopen(full_filename,"a");
//		if (myfile == 0)
//		{
//			printf("VisualWordsKohonen_C failed to open file '%s'\n",full_filename);
//		}
//
//		for(i = 0; i < n_clusters; i++)
//		{
//			if(i < n_clusters - 1)
//				fprintf(myfile, "%f ", word_distribution[i]);
//			else
//				fprintf(myfile, "%f\n", word_distribution[i]);
//		}
//
//		if(myfile != 0)
//		{
//			fclose(myfile);
//		}
//	}

	for(i = 0; i < clustered_ps; i++)
	{
		for(j = 0; j < clustered_ps; j++)
		{
			   free(p[i][j]);
		}
		free(p[i]);
	}
	free(p);

	buf = NULL;
	free(buf);

} // EXECUTION


void subimage_extraction(unsigned char *sub_frame,
		unsigned char *frame,
		int imgW, int imgH, int type, int n_reg, int in_reg_h, int in_reg_w)
{
	int n_reg_ax, subframe_h, subframe_w, start_reg_h, start_reg_w, end_reg_h, end_reg_w, m, n, i, j;
	n_reg_ax = (int) sqrt(n_reg);
	subframe_h = (int) (imgH/n_reg_ax);
	subframe_w = (int) (imgW/n_reg_ax);
	start_reg_h = in_reg_h*subframe_h;
	start_reg_w = in_reg_w*type*subframe_w;
	end_reg_h = (in_reg_h+1)*subframe_h-1;
	end_reg_w = (in_reg_w+1)*type*subframe_w-1;
	m = 0;
	n = 0;
//	printf("sub_img_size:(%d,%d)\n",subframe_w,subframe_h);
	for(i=start_reg_h; i<end_reg_h+1; i++)
	{
		for(j=start_reg_w; j<end_reg_w+1; j++)
		{
			sub_frame[m*type*subframe_w+n] = frame[type*i*imgW+j];
			n++;
		}
		m++;
		n = 0;
	}
}
