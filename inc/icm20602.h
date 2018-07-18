#ifndef _ICM20602_H
#define _ICM20602_H

/***** Includes *****/

#include <stdint.h>
#include <stdbool.h>

/***** Macros *****/

/** This initializer will zero out the ICM20602 struct for initialization
  * purposes. This is advised as to avoid having uninitialized garbage values
  * left within the struct */
#define ICM20602_CONFIG_INIT() \
	{ \
		.use_gyro = false, \
		.gyro_dlpf = 0, \
		.gyro_dps = 0, \
		.use_accel = false, \
		.accel_dlpf = 0, \
		.accel_g = 0, \
		.sample_rate_div = 1, \
	}

/** This initializer will configure the ICM20602 with settings that should
  * yield some actual output on both the gyroscope and the accelerometer. It
  * is recommended to use this for testing purposes. */
#define ICM20602_CONFIG_DEFAULT() \
	{ \
		.use_gyro = true, \
		.gyro_dlpf = ICM20602_GYRO_DLPF_10_HZ, \
		.gyro_dps = ICM20602_GYRO_RANGE_2000_DPS, \
		.use_accel = true, \
		.accel_dlpf = ICM20602_ACCEL_DLPF_10_2_HZ, \
		.accel_g = ICM20602_ACCEL_RANGE_4G, \
		.sample_rate_div = 100, \
	}

/***** Typedefs *****/

typedef bool (*icm20602_hal_wr)(uint8_t, uint8_t *, uint32_t);
typedef bool (*icm20602_hal_rd)(uint8_t, uint8_t *, uint32_t);

/***** Enums *****/

/** Enumerated value corresponds with DLPF_CFG in the CONFIG register unless
  * BYPASS is specified in the name. If BYPASS is used, the DLPF is removed
  * from the signal path and FCHOICE_B is set in GYRO_CONFIG register. */
enum icm20602_gyro_dlpf {
	ICM20602_GYRO_DLPF_250_HZ = 0, // data clocked at 8kHz
	ICM20602_GYRO_DLPF_176_HZ = 1, // data clocked at 1kHz
	ICM20602_GYRO_DLPF_92_HZ = 2, // data clocked at 1kHz
	ICM20602_GYRO_DLPF_41_HZ = 3, // data clocked at 1kHz
	ICM20602_GYRO_DLPF_20_HZ = 4, // data clocked at 1kHz
	ICM20602_GYRO_DLPF_10_HZ = 5, // data clocked at 1kHz
	ICM20602_GYRO_DLPF_5_HZ = 6, // data clocked at 1kHz
	ICM20602_GYRO_DLPF_3281_HZ = 7, // data clocked at 8kHz
	ICM20602_GYRO_DLPF_BYPASS_3281_HZ, // no filter, data clocked at 32kHz
	ICM20602_GYRO_DLPF_BYPASS_8173_HZ, // no filter, data clocked at 32kHz
};

/** Enumerated value corresponds with FS_SEL in the GYRO_CONFIG register.
  * Values listed are the full +/- DPS range. */  
enum icm20602_gyro_dps {
	ICM20602_GYRO_RANGE_250_DPS = 0,
	ICM20602_GYRO_RANGE_500_DPS = 1,
	ICM20602_GYRO_RANGE_1000_DPS = 2,
	ICM20602_GYRO_RANGE_2000_DPS = 3,
};

/** Enumerated value corresponds with A_DLPF_CFG in the ACCEL_CONFIG2 register
  * unless BYPASS is specified in the name. If BYPASS is used, the DLPF is
  * removed from the signal path and ACCEL_FCHOICE_B is set in the
  * ACCEL_CONFIG2 register. */
enum icm20602_accel_dlpf {
	ICM20602_ACCEL_DLPF_218_1_HZ = 0, // data clocked at 1kHz
	ICM20602_ACCEL_DLPF_99_HZ = 2, // data clocked at 1kHz
	ICM20602_ACCEL_DLPF_44_8_HZ = 3, // data clocked at 1kHz
	ICM20602_ACCEL_DLPF_21_2_HZ = 4, // data clocked at 1kHz
	ICM20602_ACCEL_DLPF_10_2_HZ = 5, // data clocked at 1kHz
	ICM20602_ACCEL_DLPF_5_1_HZ = 6, // data clocked at 1kHz
	ICM20602_ACCEL_DLPF_420_HZ = 7, // data clocked at 1kHz
	ICM20602_ACCEL_DLPF_BYPASS_1046_HZ, // no filter, data clocked at 4kHz
};

/** Enumerated value corresponds with ACCEL_FS_SEL in the ACCEL_CONFIG
  * register. Values listed are the full +/- G range. */  
enum icm20602_accel_g {
	ICM20602_ACCEL_RANGE_2G = 0,
	ICM20602_ACCEL_RANGE_4G = 1,
	ICM20602_ACCEL_RANGE_8G = 2,
	ICM20602_ACCEL_RANGE_16G = 3,
};

/***** Structs *****/

struct icm20602_config {
	/// Set to "true" to configure the gyroscope.
	bool use_gyro;
	/// Select the digital low pass filter to use with the gyroscope.
	enum icm20602_gyro_dlpf gyro_dlpf;
	/// Select the gyroscope's degrees per second range.
	enum icm20602_gyro_dps gyro_dps;

	/// Set to "true" to configure the accelerometer.
	bool use_accel;
	/// Select the digital low pass filter to use with the accelerometer.
	enum icm20602_accel_dlpf accel_dlpf;
	/// Select the accelerometer's g-force range.
	enum icm20602_accel_g accel_g;

	/// Divides the data clock for both the accelerometer and gyroscope.
	uint8_t sample_rate_div;
};

/***** Global Functions *****/

/** \brief Sets HAL register IO functions for I2C communication.
  * \param wr function pointer to register write function
  * \param rd function pointer to register read function
  * \return void
  */
extern void
icm20602_hal_i2c_register(icm20602_hal_wr wr, icm20602_hal_rd rd);

/** \brief Sets HAL register IO functions for SPI communication.
  * \param wr function pointer to register write function
  * \param rd function pointer to register read function
  * \return void
  */
extern void
icm20602_hal_spi_register(icm20602_hal_wr wr, icm20602_hal_rd rd);

/** \brief Initializes the ICM20602 sensor.
  * \param config pointer to configuration struct
  * \return true on success, error on failure
  */
extern bool
icm20602_init(struct icm20602_config * config);

/** \brief Reads current value of gyroscope.
  * \param p_x destination for x value
  * \param p_y destination for y value
  * \param p_z destination for z value
  * \return true on success, error on failure
  */
extern bool
icm20602_read_gyro(int16_t * p_x, int16_t * p_y, int16_t * p_z);

/** \brief Reads current value of accelerometer.
  * \param p_x destination for x value
  * \param p_y destination for y value
  * \param p_z destination for z value
  * \return true on success, error on failure
  */
extern bool
icm20602_read_accel(int16_t * p_x, int16_t * p_y, int16_t * p_z);

/** \brief Reads current value of accelerometer and gyroscope.
  * \param p_ax destination for accelerometer x value
  * \param p_ay destination for accelerometer y value
  * \param p_az destination for accelerometer z value
  * \param p_gx destination for gyroscope x value
  * \param p_gy destination for gyroscope y value
  * \param p_gz destination for gyroscope z value
  * \return true on success, error on failure
  */
extern bool
icm20602_read_data(int16_t * p_ax, int16_t * p_ay, int16_t * p_az,
	int16_t * p_gx, int16_t * p_gy, int16_t * p_gz);

#endif
