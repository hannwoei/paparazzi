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
 * @file paparazzi/sw/airborne/modules/computer_vision/lib/vision/texton.h
 * @brief texton distribution
 *
 * Sensors from vertical camera and IMU of Parrot AR.Drone 2.0
 */

#ifndef TEXTON_H
#define TEXTON_H
#include "lib/vision/image.h"
#ifdef SUB_IMG
void SSL_Texton(float *flatness_SSL, float ****dictionary, float *word_distribution, float *LinearMap,
		uint8_t *sub_frame, float *sub_flatness, uint8_t *in_sub_min, float *sub_min,
		struct image_t *input,
		uint8_t *dictionary_ready, uint8_t *load_dictionary, uint8_t *load_model, float alpha, uint8_t n_words, uint8_t patch_size, uint32_t n_samples,
		uint32_t *learned_samples, uint32_t n_samples_image, uint8_t *filled, uint8_t RANDOM_SAMPLES, uint32_t border_width,
		uint32_t border_height, uint16_t n_reg_ax, uint16_t type, uint16_t subframe_h, uint16_t subframe_w);
#else
void SSL_Texton(float *flatness_SSL, float ****dictionary, float *word_distribution, float *LinearMap,
		struct image_t *input,
		uint8_t *dictionary_ready, uint8_t *load_dictionary, uint8_t *load_model, float alpha, uint8_t n_words, uint8_t patch_size, uint32_t n_samples,
		uint32_t *learned_samples, uint32_t n_samples_image, uint8_t *filled, uint8_t RANDOM_SAMPLES, uint32_t border_width,
		uint32_t border_height);
#endif
void DictionaryTrainingYUV(float ****color_words, uint8_t *frame, uint8_t n_words, uint8_t patch_size, uint32_t *learned_samples,
		uint32_t n_samples_image, float alpha, uint16_t Width, uint16_t Height, uint8_t *filled);
void DistributionExtraction(float ****color_words, uint8_t *frame, float* word_distribution, uint8_t n_words,
		uint8_t patch_size, uint32_t n_samples_image, uint8_t RANDOM_SAMPLES, uint16_t Width, uint16_t Height,
		uint32_t border_width, uint32_t border_height);
void subimage_extraction(uint8_t *sub_frame,
		uint8_t *frame,
		uint16_t imgW, uint16_t imgH, uint16_t type, uint16_t subframe_h, uint16_t subframe_w, uint16_t in_reg_h, uint16_t in_reg_w);
#endif
