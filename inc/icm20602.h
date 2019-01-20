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
    .hal_wr = NULL, \
    .hal_rd = NULL, \
    .hal_sleep = NULL, \
    .mutex_lock = NULL, \
    .mutex_unlock = NULL, \
    .use_accel = false, \
    .accel_fifo = false, \
    .accel_dlpf = 0, \
    .accel_g = 0, \
    .use_gyro = false, \
    .gyro_fifo = false, \
    .gyro_dlpf = 0, \
    .gyro_dps = 0, \
    .sample_rate_div = 1, \
    .i2c_disable = false, \
  }

/** This initializer will configure the ICM20602 with settings that should
  * yield some actual output on both the gyroscope and the accelerometer. All
  * that the developer should need to set manually are the "hal_wr", "hal_rd",
  * and "hal_sleep" function pointers. It is recommended to use this for
  * testing purposes. */
#define ICM20602_CONFIG_DEFAULT() \
  { \
    .hal_wr = NULL, \
    .hal_rd = NULL, \
    .hal_sleep = NULL, \
    .mutex_lock = NULL, \
    .mutex_unlock = NULL, \
    .use_accel = true, \
    .accel_fifo = false, \
    .accel_dlpf = ICM20602_ACCEL_DLPF_10_2_HZ, \
    .accel_g = ICM20602_ACCEL_RANGE_4G, \
    .use_gyro = true, \
    .gyro_fifo = false, \
    .gyro_dlpf = ICM20602_GYRO_DLPF_10_HZ, \
    .gyro_dps = ICM20602_GYRO_RANGE_2000_DPS, \
    .sample_rate_div = 100, \
    .i2c_disable = false, \
  }

/***** Typedefs *****/

typedef bool (*icm20602_hal_wr)(uint8_t, uint8_t *, uint32_t);
typedef bool (*icm20602_hal_rd)(uint8_t, uint8_t *, uint32_t);
typedef void (*icm20602_hal_sleep)(uint32_t ms);
typedef void (*icm20602_mutex_lock)(void);
typedef void (*icm20602_mutex_unlock)(void);

/***** Enums *****/

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

/***** Structs *****/

struct icm20602_config {
  /// Required function pointer for register write function.
  icm20602_hal_wr hal_wr;
  /// Required function pointer for register read function.
  icm20602_hal_rd hal_rd;
  /// Required function pointer for system sleep/delay.
  icm20602_hal_sleep hal_sleep;

  /// Optional function pointer to mutex lock if needed, NULL otherwise.
  icm20602_mutex_lock mutex_lock;
  /// Optional function pointer to mutex unlocking if needed, NULL otherwise.
  icm20602_mutex_lock mutex_unlock;

  /// Set to "true" to configure the accelerometer.
  bool use_accel;
  /// Enable or disable fifo for accelerometer.
  bool accel_fifo;
  /// Select the digital low pass filter to use with the accelerometer.
  enum icm20602_accel_dlpf accel_dlpf;
  /// Select the accelerometer's g-force range.
  enum icm20602_accel_g accel_g;

  /// Set to "true" to configure the gyroscope.
  bool use_gyro;
  /// Enable or disable fifo for gyroscope.
  bool gyro_fifo;
  /// Select the digital low pass filter to use with the gyroscope.
  enum icm20602_gyro_dlpf gyro_dlpf;
  /// Select the gyroscope's degrees per second range.
  enum icm20602_gyro_dps gyro_dps;

  /// Divides the data clock for both the accelerometer and gyroscope.
  uint8_t sample_rate_div;

  /// Disable hardware I2C communications to chip, recommeded if using SPI.
  bool i2c_disable;
};

/***** Global Functions *****/

/** \brief Initializes the ICM20602 sensor.
  * \param config pointer to configuration struct
  * \return true on success, error on failure
  */
extern bool
icm20602_init(struct icm20602_config * config);

/** \brief Reads current G-force values of accelerometer.
  * \param p_x destination for x G value
  * \param p_y destination for y G value
  * \param p_z destination for z G value
  * \return true on success, error on failure
  */
extern bool
icm20602_read_accel(float * p_x, float * p_y, float * p_z);

/** \brief Reads current degrees per second values of gyroscope.
  * \param p_x destination for x value
  * \param p_y destination for y value
  * \param p_z destination for z value
  * \return true on success, error on failure
  */
extern bool
icm20602_read_gyro(float * p_x, float * p_y, float * p_z);

/** \brief Reads current values of accelerometer and gyroscope.
  * \param p_ax destination for accelerometer x G value
  * \param p_ay destination for accelerometer y G value
  * \param p_az destination for accelerometer z G value
  * \param p_gx destination for gyroscope x DPS value
  * \param p_gy destination for gyroscope y DPS value
  * \param p_gz destination for gyroscope z DPS value
  * \param p_t destination for temperature degrees C value
  * \return true on success, error on failure
  */
extern bool
icm20602_read_data(float * p_ax, float * p_ay, float * p_az,
  float * p_gx, float * p_gy, float * p_gz, float * p_t);

/** \brief Reads current raw values of accelerometer.
  * \param p_x destination for x value
  * \param p_y destination for y value
  * \param p_z destination for z value
  * \return true on success, error on failure
  */
extern bool
icm20602_read_accel_raw(int16_t * p_x, int16_t * p_y, int16_t * p_z);

/** \brief Reads current raw values of gyroscope.
  * \param p_x destination for x value
  * \param p_y destination for y value
  * \param p_z destination for z value
  * \return true on success, error on failure
  */
extern bool
icm20602_read_gyro_raw(int16_t * p_x, int16_t * p_y, int16_t * p_z);

/** \brief Reads current raw values of accelerometer and gyroscope.
  * \param p_ax destination for accelerometer x value
  * \param p_ay destination for accelerometer y value
  * \param p_az destination for accelerometer z value
  * \param p_gx destination for gyroscope x value
  * \param p_gy destination for gyroscope y value
  * \param p_gz destination for gyroscope z value
  * \param p_t destination for temperature value
  * \return true on success, error on failure
  */
extern bool
icm20602_read_data_raw(int16_t * p_ax, int16_t * p_ay, int16_t * p_az,
  int16_t * p_gx, int16_t * p_gy, int16_t * p_gz, int16_t * p_t);

/** \brief Reads FIFO G-force values of accelerometer.
  * \param p_x destination for x G value
  * \param p_y destination for y G value
  * \param p_z destination for z G value
  * \return true on success, error on failure
  */
extern bool
icm20602_read_accel_fifo(float * x, float * y, float * z);

/** \brief Reads FIFO degrees per second values of gyroscope.
  * \param p_x destination for x value
  * \param p_y destination for y value
  * \param p_z destination for z value
  * \return true on success, error on failure
  */
extern bool
icm20602_read_gyro_fifo(float * x, float * y, float * z);

/** \brief Reads FIFO values of accelerometer and gyroscope. Note, both
  *        accelerometer and gyroscope fifos should be enabled if this
  *        function is to be used.
  * \param p_ax destination for accelerometer x G value
  * \param p_ay destination for accelerometer y G value
  * \param p_az destination for accelerometer z G value
  * \param p_gx destination for gyroscope x DPS value
  * \param p_gy destination for gyroscope y DPS value
  * \param p_gz destination for gyroscope z DPS value
  * \param p_t destination for temperature degrees C value
  * \return true on success, error on failure
  */
extern bool
icm20602_read_fifo_data(float * p_ax, float * p_ay, float * p_az,
  float * p_gx, float * p_gy, float * p_gz, float * p_t);

/** \brief Reads FIFO raw values of accelerometer.
  * \param p_x destination for x value
  * \param p_y destination for y value
  * \param p_z destination for z value
  * \return true on success, error on failure
  */
extern bool
icm20602_read_fifo_accel_raw(int16_t * p_x, int16_t * p_y, int16_t * p_z);

/** \brief Reads FIFO raw values of gyroscope.
  * \param p_x destination for x value
  * \param p_y destination for y value
  * \param p_z destination for z value
  * \return true on success, error on failure
  */
extern bool
icm20602_read_fifo_gyro_raw(int16_t * p_x, int16_t * p_y, int16_t * p_z);

/** \brief Reads FIFO raw values of accelerometer and gyroscope. Note, both
  *        accelerometer and gyroscope fifos should be enabled if this
  *        function is to be used.
  * \param p_ax destination for accelerometer x value
  * \param p_ay destination for accelerometer y value
  * \param p_az destination for accelerometer z value
  * \param p_gx destination for gyroscope x value
  * \param p_gy destination for gyroscope y value
  * \param p_gz destination for gyroscope z value
  * \param p_t destination for temperature value
  * \return true on success, error on failure
  */
extern bool
icm20602_read_fifo_data_raw(int16_t * p_ax, int16_t * p_ay, int16_t * p_az,
  int16_t * p_gx, int16_t * p_gy, int16_t * p_gz, int16_t * p_t);

#endif
