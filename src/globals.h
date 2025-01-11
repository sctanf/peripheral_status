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
#ifndef SLIMENRF_GLOBALS
#define SLIMENRF_GLOBALS

#include <zephyr/logging/log.h>

#include "retained.h"

#define USER_SHUTDOWN_ENABLED CONFIG_USER_SHUTDOWN // Allow user to use reset or sw0 to shutdown
#define MAG_ENABLED CONFIG_SENSOR_USE_MAG // Use magnetometer if it is present
#define IGNORE_RESET CONFIG_IGNORE_RESET // If sw0 available, don't change any reset behavior
#define WOM_USE_DCDC CONFIG_WOM_USE_DCDC // Use DCDC instead of LDO for WOM if it is more efficient

#if defined(CONFIG_BOARD_SLIMEVRMINI_P1_UF2) || defined(CONFIG_BOARD_SLIMEVRMINI_P2_UF2)
#define SENSOR_GYROSCOPE_AXES_ALIGNMENT gy, -gz, -gx
#define SENSOR_ACCELEROMETER_AXES_ALIGNMENT ay, -az, -ax
#define SENSOR_MAGNETOMETER_AXES_ALIGNMENT -mx, mz, -my
#define SENSOR_QUATERNION_CORRECTION 0.7071f, 0.7071f, 0.0f, 0.0f
#endif

#ifndef SENSOR_GYROSCOPE_AXES_ALIGNMENT
#define SENSOR_GYROSCOPE_AXES_ALIGNMENT gx, -gz, gy // gyro axes alignment to sensor body
#endif
#ifndef SENSOR_ACCELEROMETER_AXES_ALIGNMENT
#define SENSOR_ACCELEROMETER_AXES_ALIGNMENT ax, -az, ay // accel axes alignment to sensor body
#endif
#ifndef SENSOR_MAGNETOMETER_AXES_ALIGNMENT
#define SENSOR_MAGNETOMETER_AXES_ALIGNMENT my, mz, -mx // mag axes alignment to sensor body
#endif
#ifndef SENSOR_QUATERNION_CORRECTION
#define SENSOR_QUATERNION_CORRECTION 0.5f, -0.5f, -0.5f, -0.5f // correction quat for sensor to mounting orientation
#endif

#endif