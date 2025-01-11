#include "../globals.h"
#include "../system/system.h"
#include "../util.h"

#include <math.h>

#include "magneto/magneto1_4.h"

#include "calibration.h"

static float accelBias[3] = {0}, gyroBias[3] = {0}, magBias[3] = {0}; // offset biases

static float magBAinv[4][3];
static float accBAinv[4][3];

static int mag_progress;
static int last_mag_progress;
static int64_t mag_progress_time;

static double ata[100]; // init calibration
static double norm_sum;
static double sample_count;

LOG_MODULE_REGISTER(calibration, LOG_LEVEL_INF);

float *sensor_calibration_get_accelBias()
{
    return accelBias;
}

float (*sensor_calibration_get_accBAinv())[3]
{
	return accBAinv;
}

float *sensor_calibration_get_gyroBias()
{
    return gyroBias;
}

float *sensor_calibration_get_magBias()
{
    return magBias;
}

float (*sensor_calibration_get_magBAinv())[3]
{
    return magBAinv;
}

int sensor_calibration_get_mag_progress()
{
    return mag_progress;
}

bool wait_for_motion(const sensor_imu_t *sensor_imu, const struct i2c_dt_spec *dev_i2c, bool motion, int samples)
{
	uint8_t counts = 0;
	float a[3], last_a[3];
	sensor_imu->accel_read(dev_i2c, last_a);
	for (int i = 0; i < samples + counts; i++)
	{
		LOG_INF("Accelerometer: %.5f %.5f %.5f", (double)a[0], (double)a[1], (double)a[2]);
		k_msleep(500);
		sensor_imu->accel_read(dev_i2c, a);
		if (v_epsilon(a, last_a, 0.1) != motion)
		{
			LOG_INF("No motion detected");
			counts++;
			if (counts == 2)
				return true;
		}
		else
		{
			counts = 0;
		}
		memcpy(last_a, a, sizeof(a));
	}
	LOG_INF("Motion detected");
	return false;
}

void sensor_calibration_read(void)
{
	memcpy(accelBias, retained.accelBias, sizeof(accelBias));
	memcpy(gyroBias, retained.gyroBias, sizeof(gyroBias));
	memcpy(magBias, retained.magBias, sizeof(magBias));
	memcpy(magBAinv, retained.magBAinv, sizeof(magBAinv));
	memcpy(accBAinv, retained.accBAinv, sizeof(accBAinv));
}

void sensor_calibrate_imu(const sensor_imu_t *sensor_imu, const struct i2c_dt_spec *dev_i2c)
{
//	float last_accelBias[3], last_gyroBias[3];
//	memcpy(last_accelBias, accelBias, sizeof(accelBias));
//	memcpy(last_gyroBias, gyroBias, sizeof(gyroBias));
	LOG_INF("Calibrating main accelerometer and gyroscope zero rate offset");
	LOG_INF("Rest the device on a stable surface");

	set_led(SYS_LED_PATTERN_LONG, SYS_LED_PRIORITY_SENSOR);
	if (!wait_for_motion(sensor_imu, dev_i2c, false, 6)) // Wait for accelerometer to settle, timeout 3s
	{
		set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
		return; // Timeout, calibration failed
	}

	set_led(SYS_LED_PATTERN_ON, SYS_LED_PRIORITY_SENSOR);
	k_msleep(500); // Delay before beginning acquisition

	LOG_INF("Reading data");
	sensor_calibration_clear();
	if (sensor_offsetBias(sensor_imu, dev_i2c, accelBias, gyroBias)) // This takes about 3s
	{
		LOG_INF("Motion detected");
		accelBias[0] = NAN; // invalidate calibration
	}
	else
	{
		sys_write(MAIN_ACCEL_BIAS_ID, &retained.accelBias, accelBias, sizeof(accelBias));
		sys_write(MAIN_GYRO_BIAS_ID, &retained.gyroBias, gyroBias, sizeof(gyroBias));
#if !CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
		LOG_INF("Accelerometer bias: %.5f %.5f %.5f", (double)accelBias[0], (double)accelBias[1], (double)accelBias[2]);
#endif
		LOG_INF("Gyroscope bias: %.5f %.5f %.5f", (double)gyroBias[0], (double)gyroBias[1], (double)gyroBias[2]);
	}
	if (sensor_calibration_validate())
	{
		set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
//		LOG_INF("Restoring previous calibration");
//		memcpy(accelBias, last_accelBias, sizeof(accelBias)); // restore last calibration
//		memcpy(gyroBias, last_gyroBias, sizeof(gyroBias)); // restore last calibration
//#if !CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
//		LOG_INF("Accelerometer bias: %.5f %.5f %.5f", (double)accelBias[0], (double)accelBias[1], (double)accelBias[2]);
//#endif
//		LOG_INF("Gyroscope bias: %.5f %.5f %.5f", (double)gyroBias[0], (double)gyroBias[1], (double)gyroBias[2]);
//		sensor_calibration_validate(); // additionally verify old calibration
		return;
	}

	LOG_INF("Finished calibration");
	sensor_fusion_invalidate();
	set_led(SYS_LED_PATTERN_ONESHOT_COMPLETE, SYS_LED_PRIORITY_SENSOR);
}

#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
void sensor_calibrate_6_side(const sensor_imu_t *sensor_imu, const struct i2c_dt_spec *dev_i2c)
{
//	float last_accBAinv[4][3];
//	memcpy(last_accBAinv, accBAinv, sizeof(accBAinv));
	LOG_INF("Calibrating main accelerometer 6-side offset");
	LOG_INF("Rest the device on a stable surface");

	sensor_calibration_clear_6_side();
	sensor_6_sideBias(sensor_imu, dev_i2c);
	sys_write(MAIN_ACC_6_BIAS_ID, &retained.accBAinv, accBAinv, sizeof(accBAinv));
	LOG_INF("Accelerometer matrix:");
	for (int i = 0; i < 3; i++)
		LOG_INF("%.5f %.5f %.5f %.5f", (double)accBAinv[0][i], (double)accBAinv[1][i], (double)accBAinv[2][i], (double)accBAinv[3][i]);
	if (sensor_calibration_validate_6_side())
	{
//		set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
//		LOG_INF("Restoring previous calibration");
//		memcpy(accBAinv, last_accBAinv, sizeof(accBAinv)); // restore last calibration
//		LOG_INF("Accelerometer matrix:");
//		for (int i = 0; i < 3; i++)
//			LOG_INF("%.5f %.5f %.5f %.5f", (double)accBAinv[0][i], (double)accBAinv[1][i], (double)accBAinv[2][i], (double)accBAinv[3][i]);
//		sensor_calibration_validate_6_side(); // additionally verify old calibration
		return;
	}

	LOG_INF("Finished calibration");
	set_led(SYS_LED_PATTERN_ONESHOT_COMPLETE, SYS_LED_PRIORITY_SENSOR);
}
#endif

void sensor_sample_mag(const float a[3], const float m[3])
{
	magneto_sample(m[0], m[1], m[2], ata, &norm_sum, &sample_count); // 400us
	int new_mag_progress = mag_progress;
	new_mag_progress |= (-1.2f < a[0] && a[0] < -0.8f ? 1 << 0 : 0) | (1.2f > a[0] && a[0] > 0.8f ? 1 << 1 : 0) | // dumb check if all accel axes were reached for calibration, assume the user is intentionally doing this
		(-1.2f < a[1] && a[1] < -0.8f ? 1 << 2 : 0) | (1.2f > a[1] && a[1] > 0.8f ? 1 << 3 : 0) |
		(-1.2f < a[2] && a[2] < -0.8f ? 1 << 4 : 0) | (1.2f > a[2] && a[2] > 0.8f ? 1 << 5 : 0);
	if (new_mag_progress > mag_progress && new_mag_progress == last_mag_progress)
	{
		if (k_uptime_get() > mag_progress_time)
		{
			mag_progress = new_mag_progress;
			//LOG_INF("Magnetometer calibration progress: %d", new_mag_progress);
			LOG_INF("Magnetometer calibration progress: %s %s %s %s %s %s" , (new_mag_progress & 0x01) ? "-X" : "--", (new_mag_progress & 0x02) ? "+X" : "--", (new_mag_progress & 0x04) ? "-Y" : "--", (new_mag_progress & 0x08) ? "+Y" : "--", (new_mag_progress & 0x10) ? "-Z" : "--", (new_mag_progress & 0x20) ? "+Z" : "--");
			set_led(SYS_LED_PATTERN_ONESHOT_PROGRESS, SYS_LED_PRIORITY_SENSOR);
		}
	}
	else
	{
		mag_progress_time = k_uptime_get() + 1000;
		last_mag_progress = new_mag_progress;
	}
	if (mag_progress == 0b111111)
		set_led(SYS_LED_PATTERN_FLASH, SYS_LED_PRIORITY_SENSOR); // Magnetometer calibration is ready to apply
}

void sensor_calibrate_mag(void)
{
	float last_magBAinv[4][3];
	memcpy(last_magBAinv, magBAinv, sizeof(magBAinv));
	LOG_INF("Calibrating magnetometer hard/soft iron offset");

	// max allocated 1072 bytes
	magneto_current_calibration(magBAinv, ata, norm_sum, sample_count); // 25ms
	//mag_progress |= 1 << 7;
	mag_progress = 0;
	// clear data
	memset(ata, 0, sizeof(ata));
	norm_sum = 0.0;
	sample_count = 0.0;
	sys_write(MAIN_MAG_BIAS_ID, &retained.magBAinv, magBAinv, sizeof(magBAinv));
	LOG_INF("Magnetometer matrix:");
	for (int i = 0; i < 3; i++)
		LOG_INF("%.5f %.5f %.5f %.5f", (double)magBAinv[0][i], (double)magBAinv[1][i],(double)magBAinv[2][i], (double)magBAinv[3][i]);
	if (sensor_calibration_validate_mag())
	{
		set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
		LOG_INF("Restoring previous calibration");
		memcpy(magBAinv, last_magBAinv, sizeof(magBAinv)); // restore last calibration
		LOG_INF("Magnetometer matrix:");
		for (int i = 0; i < 3; i++)
			LOG_INF("%.5f %.5f %.5f %.5f", (double)magBAinv[0][i], (double)magBAinv[1][i],(double)magBAinv[2][i], (double)magBAinv[3][i]);
		sensor_calibration_validate_mag(); // additionally verify old calibration
		return;
	}

	LOG_INF("Finished calibration");
	set_led(SYS_LED_PATTERN_ONESHOT_COMPLETE, SYS_LED_PRIORITY_SENSOR);
}

int sensor_calibration_validate(void)
{
	float zero[3] = {0};
	if (!v_epsilon(accelBias, zero, 0.5) || !v_epsilon(gyroBias, zero, 50.0)) // check accel is <0.5G and gyro <50dps
	{
		sensor_calibration_clear();
		LOG_WRN("Invalidated calibration");
		LOG_WRN("The IMU may be damaged or calibration was not completed properly");
		return -1;
	}
	return 0;
}

int sensor_calibration_validate_6_side(void)
{
	float zero[3] = {0};
	float diagonal[3];
	for (int i = 0; i < 3; i++)
		diagonal[i] = accBAinv[i + 1][i];
	float magnitude = v_avg(diagonal);
	float average[3] = {magnitude, magnitude, magnitude};
	if (!v_epsilon(accBAinv[0], zero, 0.5) || !v_epsilon(diagonal, average, magnitude * 0.1f)) // check accel is <0.5G and diagonals are within 10%
	{
		sensor_calibration_clear_6_side();
		LOG_WRN("Invalidated calibration");
		LOG_WRN("The IMU may be damaged or calibration was not completed properly");
		return -1;
	}
	return 0;
}

int sensor_calibration_validate_mag(void)
{
	float zero[3] = {0};
	float diagonal[3];
	for (int i = 0; i < 3; i++)
		diagonal[i] = magBAinv[i + 1][i];
	float magnitude = v_avg(diagonal);
	float average[3] = {magnitude, magnitude, magnitude};
	if (!v_epsilon(magBAinv[0], zero, 1) || !v_epsilon(diagonal, average, MAX(magnitude * 0.2f, 0.1f))) // check offset is <1 unit and diagonals are within 20%
	{
		sensor_calibration_clear_mag();
		LOG_WRN("Invalidated calibration");
		LOG_WRN("The magnetometer may be damaged or calibration was not completed properly");
		return -1;
	}
	return 0;
}

void sensor_calibration_clear(void)
{
	memset(accelBias, 0, sizeof(accelBias));
	memset(gyroBias, 0, sizeof(gyroBias));
	sys_write(MAIN_ACCEL_BIAS_ID, &retained.accelBias, accelBias, sizeof(accelBias));
	sys_write(MAIN_GYRO_BIAS_ID, &retained.gyroBias, gyroBias, sizeof(gyroBias));

	sensor_fusion_invalidate();
}

#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
void sensor_calibration_clear_6_side(void)
{
	memset(accBAinv, 0, sizeof(accBAinv));
	for (int i = 0; i < 3; i++) // set identity matrix
		accBAinv[i + 1][i] = 1;
	sys_write(MAIN_ACC_6_BIAS_ID, &retained.accBAinv, accBAinv, sizeof(accBAinv));
}
#endif

void sensor_calibration_clear_mag(void)
{
	memset(magBAinv, 0, sizeof(magBAinv)); // zeroed matrix will disable magnetometer in fusion
	sys_write(MAIN_MAG_BIAS_ID, &retained.magBAinv, magBAinv, sizeof(magBAinv));
}

void sensor_request_calibration(void)
{
	accelBias[0] = NAN;
	sys_write(MAIN_ACCEL_BIAS_ID, &retained.accelBias, accelBias, sizeof(accelBias));

	sensor_fusion_invalidate();
}

#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
void sensor_request_calibration_6_side(void)
{
	accBAinv[0][0] = NAN;
	sys_write(MAIN_ACC_6_BIAS_ID, &retained.accBAinv, accBAinv, sizeof(accBAinv));
}
#endif

// TODO: setup 6 sided calibration (bias and scale, and maybe gyro ZRO?), setup temp calibration (particulary for gyro ZRO)
int sensor_offsetBias(const sensor_imu_t *sensor_imu, const struct i2c_dt_spec *dev_i2c, float *dest1, float *dest2)
{
	float rawData[3], last_a[3];
	sensor_imu->accel_read(dev_i2c, last_a);
	for (int i = 0; i < 500; i++)
	{
		sensor_imu->accel_read(dev_i2c, rawData);
		if (!v_epsilon(rawData, last_a, 0.1))
			return -1;
#if !CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
		dest1[0] += rawData[0];
		dest1[1] += rawData[1];
		dest1[2] += rawData[2];
#endif
		sensor_imu->gyro_read(dev_i2c, rawData);
		dest2[0] += rawData[0];
		dest2[1] += rawData[1];
		dest2[2] += rawData[2];
		k_msleep(5);
	}
#if !CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
	dest1[0] /= 500.0f;
	dest1[1] /= 500.0f;
	dest1[2] /= 500.0f;
	if (dest1[0] > 0.9f)
		dest1[0] -= 1.0f; // Remove gravity from the x-axis accelerometer bias calculation
	else if (dest1[0] < -0.9f)
		dest1[0] += 1.0f; // Remove gravity from the x-axis accelerometer bias calculation
	else if (dest1[1] > 0.9f)
		dest1[1] -= 1.0f; // Remove gravity from the y-axis accelerometer bias calculation
	else if (dest1[1] < -0.9f)
		dest1[1] += 1.0f; // Remove gravity from the y-axis accelerometer bias calculation
	else if (dest1[2] > 0.9f)
		dest1[2] -= 1.0f; // Remove gravity from the z-axis accelerometer bias calculation
	else if (dest1[2] < -0.9f)
		dest1[2] += 1.0f; // Remove gravity from the z-axis accelerometer bias calculation
	else
		return -1;
#endif
	dest2[0] /= 500.0f;
	dest2[1] /= 500.0f;
	dest2[2] /= 500.0f;
	return 0;
}

#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
static int isAccRest(float *acc, float *pre_acc, float threshold, int *t, int restdelta)
{
	float delta_x = acc[0] - pre_acc[0];
	float delta_y = acc[1] - pre_acc[1];
	float delta_z = acc[2] - pre_acc[2];

	float norm_diff = sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);

	if (norm_diff <= threshold)
		*t += restdelta;
	else
		*t = 0;

	if (*t > 2000)
		return 1;
	return 0;
}

void sensor_6_sideBias(const sensor_imu_t *sensor_imu, const struct i2c_dt_spec *dev_i2c)
{
	// Acc 6 side calibrate
	float rawData[3];
	float pre_acc[3] = {0};

	const float THRESHOLD_ACC = 0.05f;
	int resttime = 0;

	mag_progress = 0; // reusing ata, so guarantee cleared mag progress
	memset(ata, 0, sizeof(ata));
	norm_sum = 0.0;
	sample_count = 0.0;
	int c = 0;
	printk("Starting accelerometer calibration.\n");
	while (1)
	{
		printk("Waiting for a resting state...\n");
		while (1)
		{
			sensor_imu->accel_read(dev_i2c, &rawData[0]);
			int rest = isAccRest(rawData, pre_acc, THRESHOLD_ACC, &resttime, 100);
			pre_acc[0] = rawData[0];
			pre_acc[1] = rawData[1];
			pre_acc[2] = rawData[2];

			if (rest == 1)
			{
				printk("Rest detected, starting recording. Please do not move. %d\n", c);
				k_msleep(1000);

				for (int i = 0; i < 100; i++)
				{
					sensor_imu->accel_read(dev_i2c, &rawData[0]);
					magneto_sample(rawData[0], rawData[1], rawData[2], ata, &norm_sum, &sample_count);
					if (i % 10 == 0)
						printk("#");
					k_msleep(10);
				}
				printk("Recorded values!\n");
				printk("%d side done \n", c);
				c++;
				k_msleep(1000);
				break;
			}
			k_msleep(100);
		}
		if(c >= 6) break;
		printk("Waiting for the next side... %d \n", c);
		while (1)
		{
			k_msleep(100);
			sensor_imu->accel_read(dev_i2c, &rawData[0]);
			int rest = isAccRest(rawData,pre_acc,THRESHOLD_ACC,&resttime, 100);
			pre_acc[0] = rawData[0];
			pre_acc[1] = rawData[1];
			pre_acc[2] = rawData[2];

			if (rest == 0)
			{
				resttime = 0;
				break;
			}

		}
		k_msleep(5);
	}
	

	printk("Calculating the data....\n");
	k_msleep(500);
	magneto_current_calibration(accBAinv, ata, norm_sum, sample_count);

	printk("Calibration is complete.\n");
}
#endif
