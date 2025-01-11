/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#ifndef SLIMENRF_VQF
#define SLIMENRF_VQF

#include "../../sensor.h"

void vqf_init(float g_time, float a_time, float m_time);
void vqf_load(const void *data);
void vqf_save(void *data);

void vqf_update_gyro(float *g, float time);
void vqf_update_accel(float *a, float time);
void vqf_update_mag(float *m, float time);
void vqf_update(float *g, float *a, float *m, float time);

void vqf_get_gyro_bias(float *g_off);
void vqf_set_gyro_bias(float *g_off);

void vqf_update_gyro_sanity(float *g, float *m);
int vqf_get_gyro_sanity(void);

void vqf_get_lin_a(float *lin_a);
void vqf_get_quat(float *q);

extern const sensor_fusion_t sensor_fusion_vqf;

#endif