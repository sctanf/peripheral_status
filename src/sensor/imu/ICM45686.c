#include <math.h>

#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>
#include <hal/nrf_gpio.h>

#include "ICM45686.h"
#include "sensor/sensor_none.h"

#define PACKET_SIZE 20

static const float accel_sensitivity = 16.0f / 32768.0f; // Always 16G
static const float gyro_sensitivity = 2000.0f / 32768.0f; // Always 2000dps

static const float accel_sensitivity_32 = 32.0f / ((uint32_t)2<<30); // 32G forced
static const float gyro_sensitivity_32 = 4000.0f / ((uint32_t)2<<30); // 4000dps forced

static uint8_t last_accel_odr = 0xff;
static uint8_t last_gyro_odr = 0xff;
static const float clock_reference = 32000;
static float clock_scale = 1; // ODR is scaled by clock_rate/clock_reference

static bool fifo_primed = false;

LOG_MODULE_REGISTER(ICM45686, LOG_LEVEL_DBG);

int icm45_init(const struct i2c_dt_spec *dev_i2c, float clock_rate, float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time)
{
	// special handling of unknown fifo corruption
	fifo_primed = false;

	int err = 0;
	if (clock_rate > 0)
	{
		clock_scale = clock_rate / clock_reference;
		err |= i2c_reg_write_byte_dt(dev_i2c, ICM45686_IOC_PAD_SCENARIO_OVRD, 0x06); // override pin 9 to CLKIN
		err |= i2c_reg_update_byte_dt(dev_i2c, ICM45686_RTC_CONFIG, 0x20, 0x20); // enable external CLKIN
//		err |= i2c_reg_write_byte_dt(dev_i2c, ICM45686_RTC_CONFIG, 0x23); // enable external CLKIN (0x20, default register value is 0x03)
	}
	uint8_t ireg_buf[3];
	ireg_buf[0] = ICM45686_IPREG_TOP1; // address is a word, icm is big endian
	ireg_buf[1] = ICM45686_SREG_CTRL;
	ireg_buf[2] = 0x02; // set big endian
	err |= i2c_burst_write_dt(dev_i2c, ICM45686_IREG_ADDR_15_8, ireg_buf, 3); // write buffer
	last_accel_odr = 0xff; // reset last odr
	last_gyro_odr = 0xff; // reset last odr
	err |= icm45_update_odr(dev_i2c, accel_time, gyro_time, accel_actual_time, gyro_actual_time);
//	k_msleep(50); // 10ms Accel, 30ms Gyro startup
	k_msleep(1); // fuck i dont wanna wait that long
	err |= i2c_reg_write_byte_dt(dev_i2c, ICM45686_FIFO_CONFIG0, 0x40 | 0b000111); // set FIFO Stream mode, set FIFO depth to 2K bytes
	err |= i2c_reg_write_byte_dt(dev_i2c, ICM45686_FIFO_CONFIG3, 0x0F); // begin FIFO stream, hires, a+g
	if (err)
		LOG_ERR("I2C error");
	return (err < 0 ? err : 0);
}

void icm45_shutdown(const struct i2c_dt_spec *dev_i2c)
{
	last_accel_odr = 0xff; // reset last odr
	last_gyro_odr = 0xff; // reset last odr
	int err = i2c_reg_write_byte_dt(dev_i2c, ICM45686_REG_MISC2, 0x02); // Don't need to wait for ICM to finish reset
	if (err)
		LOG_ERR("I2C error");
}

int icm45_update_odr(const struct i2c_dt_spec *dev_i2c, float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time)
{
	int ODR;
	uint8_t ACCEL_UI_FS_SEL = ACCEL_UI_FS_SEL_16G;
	uint8_t GYRO_UI_FS_SEL = GYRO_UI_FS_SEL_2000DPS;
	uint8_t ACCEL_MODE;
	uint8_t GYRO_MODE;
	uint8_t ACCEL_ODR;
	uint8_t GYRO_ODR;

	// Calculate accel
	if (accel_time <= 0 || accel_time == INFINITY) // off, standby interpreted as off
	{
		ACCEL_MODE = ACCEL_MODE_OFF;
		ODR = 0;
	}
	else
	{
		ACCEL_MODE = ACCEL_MODE_LN;
		ODR = 1 / accel_time;
		ODR /= clock_scale; // scale clock
	}

	if (ACCEL_MODE != ACCEL_MODE_LN)
	{
		ACCEL_ODR = 0;
		accel_time = 0; // off
	}
	else if (ODR > 3200) // TODO: this is absolutely awful
	{
		ACCEL_ODR = ACCEL_ODR_6_4kHz;
		accel_time = 1.0 / 6400;
	}
	else if (ODR > 1600)
	{
		ACCEL_ODR = ACCEL_ODR_3_2kHz;
		accel_time = 1.0 / 3200;
	}
	else if (ODR > 800)
	{
		ACCEL_ODR = ACCEL_ODR_1_6kHz;
		accel_time = 1.0 / 1600;
	}
	else if (ODR > 400)
	{
		ACCEL_ODR = ACCEL_ODR_800Hz;
		accel_time = 1.0 / 800;
	}
	else if (ODR > 200)
	{
		ACCEL_ODR = ACCEL_ODR_400Hz;
		accel_time = 1.0 / 400;
	}
	else if (ODR > 100)
	{
		ACCEL_ODR = ACCEL_ODR_200Hz;
		accel_time = 1.0 / 200;
	}
	else if (ODR > 50)
	{
		ACCEL_ODR = ACCEL_ODR_100Hz;
		accel_time = 1.0 / 100;
	}
	else if (ODR > 25)
	{
		ACCEL_ODR = ACCEL_ODR_50Hz;
		accel_time = 1.0 / 50;
	}
	else if (ODR > 12.5)
	{
		ACCEL_ODR = ACCEL_ODR_25Hz;
		accel_time = 1.0 / 25;
	}
	else
	{
		ACCEL_ODR = ACCEL_ODR_12_5Hz;
		accel_time = 1.0 / 12.5;
	}
	accel_time /= clock_scale; // scale clock

	// Calculate gyro
	if (gyro_time <= 0) // off
	{
		GYRO_MODE = GYRO_MODE_OFF;
		ODR = 0;
	}
	else if (gyro_time == INFINITY) // standby
	{
		GYRO_MODE = GYRO_MODE_STANDBY;
		ODR = 0;
	}
	else
	{
		GYRO_MODE = GYRO_MODE_LN;
		ODR = 1 / gyro_time;
		ODR /= clock_scale; // scale clock
	}

	if (GYRO_MODE != GYRO_MODE_LN)
	{
		GYRO_ODR = 0;
		gyro_time = 0; // off
	}
	else if (ODR > 3200) // TODO: this is absolutely awful
	{
		GYRO_ODR = GYRO_ODR_6_4kHz;
		gyro_time = 1.0 / 6400;
	}
	else if (ODR > 1600)
	{
		GYRO_ODR = GYRO_ODR_3_2kHz;
		gyro_time = 1.0 / 3200;
	}
	else if (ODR > 800)
	{
		GYRO_ODR = GYRO_ODR_1_6kHz;
		gyro_time = 1.0 / 1600;
	}
	else if (ODR > 400)
	{
		GYRO_ODR = GYRO_ODR_800Hz;
		gyro_time = 1.0 / 800;
	}
	else if (ODR > 200)
	{
		GYRO_ODR = GYRO_ODR_400Hz;
		gyro_time = 1.0 / 400;
	}
	else if (ODR > 100)
	{
		GYRO_ODR = GYRO_ODR_200Hz;
		gyro_time = 1.0 / 200;
	}
	else if (ODR > 50)
	{
		GYRO_ODR = GYRO_ODR_100Hz;
		gyro_time = 1.0 / 100;
	}
	else if (ODR > 25)
	{
		GYRO_ODR = GYRO_ODR_50Hz;
		gyro_time = 1.0 / 50;
	}
	else if (ODR > 12.5)
	{
		GYRO_ODR = GYRO_ODR_25Hz;
		gyro_time = 1.0 / 25;
	}
	else
	{
		GYRO_ODR = GYRO_ODR_12_5Hz;
		gyro_time = 1.0 / 12.5;
	}
	gyro_time /= clock_scale; // scale clock

	if (last_accel_odr == ACCEL_ODR && last_gyro_odr == GYRO_ODR) // if both were already configured
		return 1;

	int err = 0;
	// only if the power mode has changed
	if (last_accel_odr == 0xff || last_gyro_odr == 0xff || (last_accel_odr == 0 ? 0 : 1) != (ACCEL_ODR == 0 ? 0 : 1) || (last_gyro_odr == 0 ? 0 : 1) != (GYRO_ODR == 0 ? 0 : 1))
	{ // TODO: can't tell difference between gyro off and gyro standby
		err |= i2c_reg_write_byte_dt(dev_i2c, ICM45686_PWR_MGMT0, GYRO_MODE << 2 | ACCEL_MODE); // set accel and gyro modes
		k_busy_wait(250); // wait >200us // TODO: is this needed?
	}
	last_accel_odr = ACCEL_ODR;
	last_gyro_odr = GYRO_ODR;

	err |= i2c_reg_write_byte_dt(dev_i2c, ICM45686_ACCEL_CONFIG0, ACCEL_UI_FS_SEL << 4 | ACCEL_ODR); // set accel ODR and FS
	err |= i2c_reg_write_byte_dt(dev_i2c, ICM45686_GYRO_CONFIG0, GYRO_UI_FS_SEL << 4 | GYRO_ODR); // set gyro ODR and FS
	if (err)
		LOG_ERR("I2C error");

	*accel_actual_time = accel_time;
	*gyro_actual_time = gyro_time;

	return 0;
}

static const uint8_t empty[PACKET_SIZE] = {[0 ... 7] = 0x7f};

uint16_t icm45_fifo_read(const struct i2c_dt_spec *dev_i2c, uint8_t *data, uint16_t len) // TODO: check if working
{
	int err = 0;
	uint16_t total = 0;
	uint16_t packets = UINT16_MAX;
	uint16_t empty_packets = 0;
	while (packets > 0 && len >= PACKET_SIZE)
	{
		memset(data, 0x7F, PACKET_SIZE); // Empty packet is 7F filled
		uint8_t rawCount[2];
		err |= i2c_burst_read_dt(dev_i2c, ICM45686_FIFO_COUNT_0, &rawCount[0], 2);
		packets = (uint16_t)(rawCount[0] << 8 | rawCount[1]); // Turn the 16 bits into a unsigned 16-bit value
		uint16_t limit = len / PACKET_SIZE;
		if (packets > limit)
			packets = limit;
		// specially handle packet error from unknown fifo corruption
		// TODO: this will discard some data in fifo, however the data is actually recoverable.
		for (int i = 0; i < packets; i++)
		{
			err |= i2c_burst_read_dt(dev_i2c, ICM45686_FIFO_DATA, &data[i * PACKET_SIZE], PACKET_SIZE); // must check each packet as its coming in
			// header should initially be 0x00, and 0x78 once data is being written correctly
			if (!fifo_primed && data[i * PACKET_SIZE] == 0x78) // wait for first valid packet
			{
				LOG_INF("FIFO ready");
				fifo_primed = true;
			}
			else if (data[i * PACKET_SIZE] != 0x78 && fifo_primed) // immediately reset fifo on invalid packet
			{
				if (!memcmp(&data[i * PACKET_SIZE], empty, PACKET_SIZE)) // skip if read empty packet
				{
					empty_packets++;
					continue;
				}
				LOG_ERR("FIFO error on packet %d/%d", i, packets);
				LOG_WRN("Discarded %d packets", packets - i);
				fifo_primed = false;
				err |= i2c_reg_write_byte_dt(dev_i2c, ICM45686_FIFO_CONFIG3, 0x00); // stop FIFO
				err |= i2c_reg_write_byte_dt(dev_i2c, ICM45686_FIFO_CONFIG0, 0x00); // reset FIFO config
				err |= i2c_reg_write_byte_dt(dev_i2c, ICM45686_FIFO_CONFIG0, 0x40 | 0b000111); // set FIFO Stream mode, set FIFO depth to 2K bytes
				err |= i2c_reg_write_byte_dt(dev_i2c, ICM45686_FIFO_CONFIG3, 0x0F); // begin FIFO stream, hires, a+g
			}
			else if (!fifo_primed)
			{
				LOG_INF("Current header: 0x%02X", data[i * PACKET_SIZE]);
			}
		}
		if (err)
			LOG_ERR("I2C error");
		data += packets * PACKET_SIZE;
		len -= packets * PACKET_SIZE;
		total += packets;
	}
	if (empty_packets)
		LOG_WRN("FIFO read %d/%d empty packet%s", empty_packets, total, empty_packets > 1 ? "s" : "");
	return total;
}

static const uint8_t invalid[6] = {0x80, 0x00, 0x80, 0x00, 0x80, 0x00};

int icm45_fifo_process(uint16_t index, uint8_t *data, float a[3], float g[3])
{
	index *= PACKET_SIZE;
	if (data[index] != 0x78) // ACCEL_EN, GYRO_EN, HIRES_EN, TMST_FIELD_EN
		return 1; // Skip invalid header
	// Empty packet is 7F filled
	// combine into 20 bit values in 32 bit int
	float a_raw[3] = {0};
	float g_raw[3] = {0};
	if (memcmp(&data[index + 1], invalid, sizeof(invalid))) // valid accel data
	{
		for (int i = 0; i < 3; i++) // accel x, y, z
			a_raw[i] = (int32_t)((((uint32_t)data[index + 1 + (i * 2)]) << 24) | (((uint32_t)data[index + 2 + (i * 2)]) << 16) | (((uint32_t)data[index + 17 + i] & 0xF0) << 8));
	}
	if (memcmp(&data[index + 7], invalid, sizeof(invalid))) // valid gyro data
	{
		for (int i = 0; i < 3; i++) // gyro x, y, z
			g_raw[i] = (int32_t)((((uint32_t)data[index + 7 + (i * 2)]) << 24) | (((uint32_t)data[index + 8 + (i * 2)]) << 16) | (((uint32_t)data[index + 17 + i] & 0x0F) << 12));
	}
	else if (!memcmp(&data[index + 1], invalid, sizeof(invalid))) // Skip invalid data
	{
		return 1;
	}
	for (int i = 0; i < 3; i++) // x, y, z
	{
		a_raw[i] *= accel_sensitivity_32;
		g_raw[i] *= gyro_sensitivity_32;
	}
	memcpy(a, a_raw, sizeof(a_raw));
	memcpy(g, g_raw, sizeof(g_raw));
	return 0;
}

void icm45_accel_read(const struct i2c_dt_spec *dev_i2c, float a[3])
{
	uint8_t rawAccel[6];
	int err = i2c_burst_read_dt(dev_i2c, ICM45686_ACCEL_DATA_X1_UI, &rawAccel[0], 6);
	if (err)
		LOG_ERR("I2C error");
	for (int i = 0; i < 3; i++) // x, y, z
	{
		a[i] = (int16_t)((((uint16_t)rawAccel[i * 2]) << 8) | rawAccel[1 + (i * 2)]);
		a[i] *= accel_sensitivity;
	}
}

void icm45_gyro_read(const struct i2c_dt_spec *dev_i2c, float g[3])
{
	uint8_t rawGyro[6];
	int err = i2c_burst_read_dt(dev_i2c, ICM45686_GYRO_DATA_X1_UI, &rawGyro[0], 6);
	if (err)
		LOG_ERR("I2C error");
	for (int i = 0; i < 3; i++) // x, y, z
	{
		g[i] = (int16_t)((((uint16_t)rawGyro[i * 2]) << 8) | rawGyro[1 + (i * 2)]);
		g[i] *= gyro_sensitivity;
	}
}

float icm45_temp_read(const struct i2c_dt_spec *dev_i2c)
{
	uint8_t rawTemp[2];
	int err = i2c_burst_read_dt(dev_i2c, ICM45686_TEMP_DATA1_UI, &rawTemp[0], 2);
	if (err)
		LOG_ERR("I2C error");
	// Temperature in Degrees Centigrade = (TEMP_DATA / 128) + 25
	float temp = (int16_t)((((uint16_t)rawTemp[0]) << 8) | rawTemp[1]);
	temp /= 128;
	temp += 25;
	return temp;
}

uint8_t icm45_setup_WOM(const struct i2c_dt_spec *dev_i2c) // TODO: check if working
{
	uint8_t interrupts;
	uint8_t ireg_buf[5];
	int err = i2c_reg_read_byte_dt(dev_i2c, ICM45686_INT1_STATUS0, &interrupts); // clear reset done int flag // TODO: is this needed
	err |= i2c_reg_write_byte_dt(dev_i2c, ICM45686_INT1_CONFIG0, 0x00); // disable default interrupt (RESET_DONE)
	err |= i2c_reg_write_byte_dt(dev_i2c, ICM45686_ACCEL_CONFIG0, ACCEL_UI_FS_SEL_8G << 4 | ACCEL_ODR_200Hz); // set accel ODR and FS
	err |= i2c_reg_write_byte_dt(dev_i2c, ICM45686_PWR_MGMT0, ACCEL_MODE_LP); // set accel and gyro modes
	ireg_buf[0] = ICM45686_IPREG_SYS2; // address is a word, icm is big endian
	ireg_buf[1] = ICM45686_IPREG_SYS2_REG_129;
	ireg_buf[2] = 0x00; // set ACCEL_LP_AVG_SEL to 1x
	err |= i2c_burst_write_dt(dev_i2c, ICM45686_IREG_ADDR_15_8, ireg_buf, 3); // write buffer
	// should already be defaulted to AULP
//	ireg_buf[0] = ICM45686_IPREG_TOP1;
//	ireg_buf[1] = ICM45686_SMC_CONTROL_0;
//	ireg_buf[2] = 0x60; // set ACCEL_LP_CLK_SEL to AULP
//	err |= i2c_burst_write_dt(dev_i2c, ICM45686_IREG_ADDR_15_8, ireg_buf, 3); // write buffer
	ireg_buf[0] = ICM45686_IPREG_TOP1;
	ireg_buf[1] = ICM45686_ACCEL_WOM_X_THR;
	ireg_buf[2] = 0x08; // set wake thresholds // 8 x 3.9 mg is ~31.25 mg
	ireg_buf[3] = 0x08; // set wake thresholds
	ireg_buf[4] = 0x08; // set wake thresholds
	err |= i2c_burst_write_dt(dev_i2c, ICM45686_IREG_ADDR_15_8, ireg_buf, 5); // write buffer
	err |= i2c_reg_write_byte_dt(dev_i2c, ICM45686_TMST_WOM_CONFIG, 0x14); // enable WOM, enable WOM interrupt
	err |= i2c_reg_write_byte_dt(dev_i2c, ICM45686_INT1_CONFIG1, 0x0E); // route WOM interrupt
	if (err)
		LOG_ERR("I2C error");
	return NRF_GPIO_PIN_PULLUP << 4 | NRF_GPIO_PIN_SENSE_LOW; // active low
}

int icm45_ext_passthrough(const struct i2c_dt_spec *dev_i2c, bool passthrough) // TODO: might need IOC_PAD_SCENARIO_AUX_OVRD instead
{
	int err = 0;
	if (passthrough)
		err |= i2c_reg_write_byte_dt(dev_i2c, ICM45686_IOC_PAD_SCENARIO_AUX_OVRD, 0x18); // AUX1_MODE_OVRD, AUX1 in I2CM Bypass, AUX1_ENABLE_OVRD, AUX1 enabled
	else
		err |= i2c_reg_write_byte_dt(dev_i2c, ICM45686_IOC_PAD_SCENARIO_AUX_OVRD, 0x00); // disable overrides
	if (err)
		LOG_ERR("I2C error");
	return 0;
}

const sensor_imu_t sensor_imu_icm45686 = {
	*icm45_init,
	*icm45_shutdown,

	*icm45_update_odr,

	*icm45_fifo_read,
	*icm45_fifo_process,
	*icm45_accel_read,
	*icm45_gyro_read,
	*icm45_temp_read,

	*icm45_setup_WOM,
	
	*imu_none_ext_setup,
	*imu_none_fifo_process_ext,
	*imu_none_ext_read,
	*icm45_ext_passthrough
};
