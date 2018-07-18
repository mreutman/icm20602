/***** Includes *****/

#include <stdlib.h>
#include "icm20602.h"
#include "icm20602_config.h"

/***** Defines *****/

#define REG_XG_OFFS_TC_H 0x04
#define REG_XG_OFFS_TC_L 0x05
#define REG_YG_OFFS_TC_H 0x07
#define REG_YG_OFFS_TC_L 0x08
#define REG_ZG_OFFS_TC_H 0x0A
#define REG_ZG_OFFS_TC_L 0x0B
#define REG_SELF_TEST_X_ACCEL 0x0D
#define REG_SELF_TEST_Y_ACCEL 0x0E
#define REG_SELF_TEST_Z_ACCEL 0x0F
#define REG_XG_OFFS_USRH 0x13
#define REG_XG_OFFS_USRL 0x14
#define REG_YG_OFFS_USRH 0x15
#define REG_YG_OFFS_USRL 0x16
#define REG_ZG_OFFS_USRH 0x17
#define REG_ZG_OFFS_USRL 0x18
#define REG_SMPLRT_DIV 0x19
#define REG_CONFIG 0x1A
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_CONFIG_2 0x1D
#define REG_LP_MODE_CFG 0x1E
#define REG_ACCEL_WOM_X_THR 0x20
#define REG_ACCEL_WOM_Y_THR 0x21
#define REG_ACCEL_WOM_Z_THR 0x22
#define REG_FIFO_EN 0x23
#define REG_FSYNC_INT 0x36
#define REG_INT_PIN_CFG 0x37
#define REG_INT_ENABLE 0x38
#define REG_FIFO_WM_INT_STATUS 0x39
#define REG_INT_STATUS 0x3A
#define REG_ACCEL_XOUT_H 0x3B
#define REG_ACCEL_XOUT_L 0x3C
#define REG_ACCEL_YOUT_H 0x3D
#define REG_ACCEL_YOUT_L 0x3E
#define REG_ACCEL_ZOUT_H 0x3F
#define REG_ACCEL_ZOUT_L 0x40
#define REG_TEMP_OUT_H 0x41
#define REG_TEMP_OUT_L 0x42
#define REG_GYRO_XOUT_H 0x43
#define REG_GYRO_XOUT_L 0x44
#define REG_GYRO_YOUT_H 0x45
#define REG_GYRO_YOUT_L 0x46
#define REG_GYRO_ZOUT_H 0x47
#define REG_GYRO_ZOUT_L 0x48
#define REG_SELF_TEST_X_GYRO 0x50
#define REG_SELF_TEST_Y_GYRO 0x51
#define REG_SELF_TEST_Z_GYRO 0x52
#define REG_FIFO_WM_TH1 0x60
#define REG_FIFO_WM_TH2 0x61
#define REG_SIGNAL_PATH_RESET 0x68
#define REG_ACCEL_INTEL_CTRL 0x69
#define REG_USER_CTRL 0x6A
#define REG_PWR_MGMT_1 0x6B
#define REG_PWR_MGMT_2 0x6C
#define REG_I2C_IF 0x70
#define REG_FIFO_COUNTH 0x72
#define REG_FIFO_COUNTL 0x73
#define REG_FIFO_R_W 0x74
#define REG_WHO_AM_I 0x75
#define REG_XA_OFFSET_H 0x77
#define REG_XA_OFFSET_L 0x78
#define REG_YA_OFFSET_H 0x7A
#define REG_YA_OFFSET_L 0x7B
#define REG_ZA_OFFSET_H 0x7D
#define REG_ZA_OFFSET_L 0x7E

#define REG_WHO_AM_I_CONST 0X12

/***** Macros *****/

#define ON_ERROR_GOTO(cond, symbol) \
	if (!(cond)) { goto symbol; }

// extra steps, but I'm paranoid about some compilers/systems not handling the
// conversion from uint8_t to int16_t correctly
#define UINT8_TO_INT16(dst, src_high, src_low) \
	do { \
		dst = (src_high); \
		dst <<= 8; \
		dst |= (src_low); \
	} while (0);

/***** Enums *****/

enum hal_io_type {
	HAL_INVALID = 0,
	HAL_I2C,
	HAL_SPI,
};

/***** Local Data *****/

static enum hal_io_type _hal_type = HAL_INVALID;
static icm20602_hal_wr _hal_wr = NULL;
static icm20602_hal_rd _hal_rd = NULL;

/***** Global Functions *****/

void
icm20602_hal_i2c_register(icm20602_hal_wr wr, icm20602_hal_rd rd)
{
	_hal_type = HAL_I2C;
	_hal_wr = wr;
	_hal_rd = rd;
}

void
icm20602_hal_spi_register(icm20602_hal_wr wr, icm20602_hal_rd rd)
{
	_hal_type = HAL_SPI;
	_hal_wr = wr;
	_hal_rd = rd;
}

bool
icm20602_init(struct icm20602_config * config)
{
	uint8_t tmp = 0;
	bool r = true;

	if (HAL_INVALID == _hal_type) {
		return false;
	}

	// General Procedure:
	//  1. reset chip
	//  2. set clock for PLL for optimum performance as documented in datasheet
	//  3. place accelerometer and gyroscope into standby
	//  4. disable fifo
	//  5. configure chip
	//  6. enable accelerometer and gyroscope

	MUTEX_LOCK();

	// full reset of chip
	tmp = 0x80;
	r = _hal_wr(REG_PWR_MGMT_1, &tmp, 1);
	ON_ERROR_GOTO(r, return_err);

	// TODO: better reset delay value
	SLEEP(1000);

	// set clock to internal PLL
	tmp = 0x01;
	r = _hal_wr(REG_PWR_MGMT_1, &tmp, 1);
	ON_ERROR_GOTO(r, return_err);

	// place accel and gyro on standby
	tmp = 0x3F;
	r = _hal_wr(REG_PWR_MGMT_2, &tmp, 1);
	ON_ERROR_GOTO(r, return_err);

	// disable fifo
	tmp = 0x00;
	r = _hal_wr(REG_USER_CTRL, &tmp, 1);
	ON_ERROR_GOTO(r, return_err);

	if (HAL_SPI == _hal_type) {
		tmp = 0x40;
		r = _hal_wr(REG_I2C_IF, &tmp, 1);
		ON_ERROR_GOTO(r, return_err);
	}

	if (config->use_gyro) {
		if (ICM20602_GYRO_DLPF_BYPASS_3281_HZ == config->gyro_dlpf) {
			// bypass dpf and set dps
			tmp = 0x00;
			r = _hal_wr(REG_CONFIG, &tmp, 1);
			ON_ERROR_GOTO(r, return_err);

			tmp = (config->gyro_dps << 3) | 0x02; // see table page 37 of datasheet
			r = _hal_wr(REG_GYRO_CONFIG, &tmp, 1);
			ON_ERROR_GOTO(r, return_err);
		}
		else if (ICM20602_GYRO_DLPF_BYPASS_8173_HZ == config->gyro_dlpf) {
			// bypass dpf and set dps
			tmp = 0x00;
			r = _hal_wr(REG_CONFIG, &tmp, 1);
			ON_ERROR_GOTO(r, return_err);

			tmp = (config->gyro_dps << 3) | 0x01; // see table page 37 of datasheet
			r = _hal_wr(REG_GYRO_CONFIG, &tmp, 1);
			ON_ERROR_GOTO(r, return_err);
		}
		else {
			// configure dpf and set dps
			tmp = config->gyro_dlpf;
			r = _hal_wr(REG_CONFIG, &tmp, 1);
			ON_ERROR_GOTO(r, return_err);

			tmp = config->gyro_dps << 3;
			r = _hal_wr(REG_GYRO_CONFIG, &tmp, 1);
			ON_ERROR_GOTO(r, return_err);
		}
	}

	if (config->use_accel) {
		if (ICM20602_ACCEL_DLPF_BYPASS_1046_HZ == config->accel_dlpf) {
			tmp = (1 << 3);
			r = _hal_wr(REG_ACCEL_CONFIG_2, &tmp, 1);
			ON_ERROR_GOTO(r, return_err);
		}
		else {
			tmp = config->accel_dlpf;
			r = _hal_wr(REG_ACCEL_CONFIG_2, &tmp, 1);
			ON_ERROR_GOTO(r, return_err);
		}

		tmp = (config->accel_g) << 2;
		r = _hal_wr(REG_ACCEL_CONFIG, &tmp, 1);
		ON_ERROR_GOTO(r, return_err);
	}

	// configure sample rate divider (TODO: is this gyro only?)
	// note: SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
	tmp = (0 != config->sample_rate_div) ? config->sample_rate_div - 1 : 1;
	r = _hal_wr(REG_SMPLRT_DIV, &tmp, 1);
	ON_ERROR_GOTO(r, return_err);

	tmp = 0;
	tmp |= (config->use_gyro) ? 0 : 0x07; // 0 - on, 1 - disabled
	tmp |= (config->use_accel) ? 0 : 0x38; // 0 - on, 1 - disabled
	r = _hal_wr(REG_PWR_MGMT_2, &tmp, 1);
	ON_ERROR_GOTO(r, return_err);

return_err:
	MUTEX_UNLOCK();
	return r;
}

bool
icm20602_read_gyro(int16_t * p_x, int16_t * p_y, int16_t * p_z)
{
	uint8_t buf[6] = {0};
	bool r = true;

	if (HAL_INVALID == _hal_type) {
		return false;
	}

	MUTEX_LOCK();

	r = _hal_rd(REG_GYRO_XOUT_H, buf, 6);
	ON_ERROR_GOTO(r, return_err);

	UINT8_TO_INT16(*p_x, buf[0], buf[1]);
	UINT8_TO_INT16(*p_y, buf[2], buf[3]);
	UINT8_TO_INT16(*p_z, buf[4], buf[5]);

return_err:
	MUTEX_UNLOCK();
	return r;
}

extern bool
icm20602_read_accel(int16_t * p_x, int16_t * p_y, int16_t * p_z)
{
	uint8_t buf[6] = {0};
	bool r = true;

	if (HAL_INVALID == _hal_type) {
		return false;
	}

	MUTEX_LOCK();

	r = _hal_rd(REG_ACCEL_XOUT_H, buf, 6);
	ON_ERROR_GOTO(r, return_err);

	UINT8_TO_INT16(*p_x, buf[0], buf[1]);
	UINT8_TO_INT16(*p_y, buf[2], buf[3]);
	UINT8_TO_INT16(*p_z, buf[4], buf[5]);

return_err:
	MUTEX_UNLOCK();
	return r;
}

extern bool
icm20602_read_data(int16_t * p_ax, int16_t * p_ay, int16_t * p_az,
	int16_t * p_gx, int16_t * p_gy, int16_t * p_gz)
{
	uint8_t buf[12] = {0};
	bool r = true;

	if (HAL_INVALID == _hal_type) {
		return false;
	}

	MUTEX_LOCK();

	r = _hal_rd(REG_ACCEL_XOUT_H, buf, 12);
	ON_ERROR_GOTO(r, return_err);

	UINT8_TO_INT16(*p_ax, buf[0], buf[1]);
	UINT8_TO_INT16(*p_ay, buf[2], buf[3]);
	UINT8_TO_INT16(*p_az, buf[4], buf[5]);
	UINT8_TO_INT16(*p_gx, buf[6], buf[7]);
	UINT8_TO_INT16(*p_gy, buf[8], buf[9]);
	UINT8_TO_INT16(*p_gz, buf[10], buf[11]);

return_err:
	MUTEX_UNLOCK();
	return r;
}
