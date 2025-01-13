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
#include "globals.h"
#include "util.h"

//#include

#include "motionsense.h"

LOG_MODULE_REGISTER(sensorfusion, LOG_LEVEL_INF);

void sensorfusion_init(float g_time, float a_time, float m_time)
{
}

void sensorfusion_load(const void *data)
{
}

void sensorfusion_save(void *data)
{
}

void sensorfusion_update_gyro(float *g, float time)
{
}

void sensorfusion_update_accel(float *a, float time)
{
}

void sensorfusion_update_mag(float *m, float time)
{
}

void sensorfusion_update(float *g, float *a, float *m, float time)
{
}

void sensorfusion_get_gyro_bias(float *g_off)
{
}

void sensorfusion_set_gyro_bias(float *g_off)
{
}

void sensorfusion_update_gyro_sanity(float *g, float *m)
{
}

int sensorfusion_get_gyro_sanity(void)
{
    return -1;
}

void sensorfusion_get_lin_a(float *lin_a)
{
}

void sensorfusion_get_quat(float *q)
{
}

const sensor_fusion_t sensor_fusion_motionsense = {
	*sensorfusion_init,
	*sensorfusion_load,
	*sensorfusion_save,

	*sensorfusion_update_gyro,
	*sensorfusion_update_accel,
	*sensorfusion_update_mag,
	*sensorfusion_update,

	*sensorfusion_get_gyro_bias,
	*sensorfusion_set_gyro_bias,

	*sensorfusion_update_gyro_sanity,
	*sensorfusion_get_gyro_sanity,

	*sensorfusion_get_lin_a,
	*sensorfusion_get_quat
};
