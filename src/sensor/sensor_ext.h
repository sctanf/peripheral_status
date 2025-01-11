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
#ifndef SLIMENRF_SENSOR_EXT
#define SLIMENRF_SENSOR_EXT

#include "sensor.h"

int mag_ext_setup(const sensor_imu_t *imu, const sensor_mag_t *mag, uint8_t addr);

int mag_ext_init(const struct i2c_dt_spec *dev_i2c, float time, float *actual_time);
void mag_ext_shutdown(const struct i2c_dt_spec *dev_i2c);

int mag_ext_update_odr(const struct i2c_dt_spec *dev_i2c, float time, float *actual_time);

void mag_ext_mag_oneshot(const struct i2c_dt_spec *dev_i2c);
void mag_ext_mag_read(const struct i2c_dt_spec *dev_i2c, float m[3]);
float mag_ext_temp_read(const struct i2c_dt_spec *dev_i2c, float bias[3]);

void mag_ext_mag_process(uint8_t *raw_m, float m[3]);

extern const sensor_mag_t sensor_mag_ext;

#endif
