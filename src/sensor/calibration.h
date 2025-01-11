#ifndef SLIMENRF_SENSOR_CALIBRATION
#define SLIMENRF_SENSOR_CALIBRATION

#include "sensor.h"

#include <zephyr/drivers/i2c.h>

float *sensor_calibration_get_accelBias();
float (*sensor_calibration_get_accBAinv())[3];
float *sensor_calibration_get_gyroBias();
float *sensor_calibration_get_magBias();
float (*sensor_calibration_get_magBAinv())[3];
int sensor_calibration_get_mag_progress();

bool wait_for_motion(const sensor_imu_t *sensor_imu, const struct i2c_dt_spec *dev_i2c, bool motion, int samples);

void sensor_calibration_read(void);

void sensor_calibrate_imu(const sensor_imu_t *sensor_imu, const struct i2c_dt_spec *dev_i2c);
void sensor_calibrate_6_side(const sensor_imu_t *sensor_imu, const struct i2c_dt_spec *dev_i2c);
void sensor_sample_mag(const float a[3], const float m[3]);
void sensor_calibrate_mag(void);

int sensor_calibration_validate(void);
int sensor_calibration_validate_6_side(void);
int sensor_calibration_validate_mag(void);

void sensor_calibration_clear(void);
void sensor_calibration_clear_6_side(void);
void sensor_calibration_clear_mag(void);

void sensor_request_calibration(void);
void sensor_request_calibration_6_side(void);

int sensor_offsetBias(const sensor_imu_t *sensor_imu, const struct i2c_dt_spec *dev_i2c, float *dest1, float *dest2);
void sensor_6_sideBias(const sensor_imu_t *sensor_imu, const struct i2c_dt_spec *dev_i2c);

#endif