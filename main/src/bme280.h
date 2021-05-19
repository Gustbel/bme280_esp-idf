#ifndef BME280_H_
#define BME280_H_

#include "bme280_defs.h"

#define BME280_SENSOR_ADDR 0x76
#define I2C_MASTER_NUM 0

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

//
// Data types definitions -----------------------------------------------------
//
typedef struct bme280_dev BME280_DEVICE;
typedef struct bme280_data BME280_DATA;

//
// Functions prototypes -------------------------------------------------------
BME280_INTF_RET_TYPE esp32c3_read_bme280(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
BME280_INTF_RET_TYPE esp32c3_write_bme280(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
void esp32c3_delay_us_bme280(uint32_t period, void *intf_ptr);
bool init_bme280();
float get_bme280(int);


/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

/* Header includes */
#include "bme280_defs.h"

/**
 * \ingroup bme280
 * \defgroup bme280ApiInit Initialization
 * @brief Initialize the sensor and device structure
 */

/*!
 * \ingroup bme280ApiInit
 * \page bme280_api_bme280_init bme280_init
 * \code
 * int8_t bme280_init(struct bme280_dev *dev);
 * \endcode
 * @details This API reads the chip-id of the sensor which is the first step to
 * verify the sensor and also calibrates the sensor
 * As this API is the entry point, call this API before using other APIs.
 *
 * @param[in,out] dev : Structure instance of bme280_dev
 *
 * @return Result of API execution status.
 *
 * @retval   0 -> Success.
 * @retval > 0 -> Warning.
 * @retval < 0 -> Fail.
 *
 */
int8_t bme280_init(struct bme280_dev *dev);

/**
 * \ingroup bme280
 * \defgroup bme280ApiRegister Registers
 * @brief Generic API for accessing sensor registers
 */

/*!
 * \ingroup bme280ApiRegister
 * \page bme280_api_bme280_set_regs bme280_set_regs
 * \code
 * int8_t bme280_set_regs(const uint8_t reg_addr, const uint8_t *reg_data, uint8_t len, struct bme280_dev *dev);
 * \endcode
 * @details This API writes the given data to the register address of the sensor
 *
 * @param[in] reg_addr : Register addresses to where the data is to be written
 * @param[in] reg_data : Pointer to data buffer which is to be written
 *                       in the reg_addr of sensor.
 * @param[in] len      : No of bytes of data to write
 * @param[in,out] dev  : Structure instance of bme280_dev
 *
 * @return Result of API execution status.
 *
 * @retval   0 -> Success.
 * @retval > 0 -> Warning.
 * @retval < 0 -> Fail.
 *
 */
int8_t bme280_set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len, struct bme280_dev *dev);

/*!
 * \ingroup bme280ApiRegister
 * \page bme280_api_bme280_get_regs bme280_get_regs
 * \code
 * int8_t bme280_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint8_t len, struct bme280_dev *dev);
 * \endcode
 * @details This API reads the data from the given register address of sensor.
 *
 * @param[in] reg_addr  : Register address from where the data to be read
 * @param[out] reg_data : Pointer to data buffer to store the read data.
 * @param[in] len       : No of bytes of data to be read.
 * @param[in,out] dev   : Structure instance of bme280_dev.
 *
 * @return Result of API execution status.
 *
 * @retval   0 -> Success.
 * @retval > 0 -> Warning.
 * @retval < 0 -> Fail.
 *
 */
int8_t bme280_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, struct bme280_dev *dev);

/**
 * \ingroup bme280
 * \defgroup bme280ApiSensorSettings Sensor Settings
 * @brief Generic API for accessing sensor settings
 */

/*!
 * \ingroup bme280ApiSensorSettings
 * \page bme280_api_bme280_set_sensor_settings bme280_set_sensor_settings
 * \code
 * int8_t bme280_set_sensor_settings(uint8_t desired_settings, const struct bme280_dev *dev);
 * \endcode
 * @details This API sets the oversampling, filter and standby duration
 * (normal mode) settings in the sensor.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 * @param[in] desired_settings : Variable used to select the settings which
 * are to be set in the sensor.
 *
 * @note : Below are the macros to be used by the user for selecting the
 * desired settings. User can do OR operation of these macros for configuring
 * multiple settings.
 *
 * Macros         |   Functionality
 * -----------------------|----------------------------------------------
 * BME280_OSR_PRESS_SEL    |   To set pressure oversampling.
 * BME280_OSR_TEMP_SEL     |   To set temperature oversampling.
 * BME280_OSR_HUM_SEL    |   To set humidity oversampling.
 * BME280_FILTER_SEL     |   To set filter setting.
 * BME280_STANDBY_SEL  |   To set standby duration setting.
 *
 * @return Result of API execution status
 *
 * @retval   0 -> Success.
 * @retval > 0 -> Warning.
 * @retval < 0 -> Fail.
 *
 */
int8_t bme280_set_sensor_settings(uint8_t desired_settings, struct bme280_dev *dev);

/*!
 * \ingroup bme280ApiSensorSettings
 * \page bme280_api_bme280_get_sensor_settings bme280_get_sensor_settings
 * \code
 * int8_t bme280_get_sensor_settings(struct bme280_dev *dev);
 * \endcode
 * @details This API gets the oversampling, filter and standby duration
 * (normal mode) settings from the sensor.
 *
 * @param[in,out] dev : Structure instance of bme280_dev.
 *
 * @return Result of API execution status
 *
 * @retval   0 -> Success.
 * @retval > 0 -> Warning.
 * @retval < 0 -> Fail.
 *
 */
int8_t bme280_get_sensor_settings(struct bme280_dev *dev);

/**
 * \ingroup bme280
 * \defgroup bme280ApiSensorMode Sensor Mode
 * @brief Generic API for configuring sensor power mode
 */

/*!
 * \ingroup bme280ApiSensorMode
 * \page bme280_api_bme280_set_sensor_mode bme280_set_sensor_mode
 * \code
 * int8_t bme280_set_sensor_mode(uint8_t sensor_mode, const struct bme280_dev *dev);
 * \endcode
 * @details This API sets the power mode of the sensor.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 * @param[in] sensor_mode : Variable which contains the power mode to be set.
 *
 *    sensor_mode           |   Macros
 * ---------------------|-------------------
 *     0                | BME280_SLEEP_MODE
 *     1                | BME280_FORCED_MODE
 *     3                | BME280_NORMAL_MODE
 *
 * @return Result of API execution status
 *
 * @retval   0 -> Success.
 * @retval > 0 -> Warning.
 * @retval < 0 -> Fail.
 *
 */
int8_t bme280_set_sensor_mode(uint8_t sensor_mode, struct bme280_dev *dev);

/*!
 * \ingroup bme280ApiSensorMode
 * \page bme280_api_bme280_get_sensor_mode bme280_get_sensor_mode
 * \code
 * int8_t bme280_get_sensor_mode(uint8_t *sensor_mode, const struct bme280_dev *dev);
 * \endcode
 * @details This API gets the power mode of the sensor.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 * @param[out] sensor_mode : Pointer variable to store the power mode.
 *
 *   sensor_mode            |   Macros
 * ---------------------|-------------------
 *     0                | BME280_SLEEP_MODE
 *     1                | BME280_FORCED_MODE
 *     3                | BME280_NORMAL_MODE
 *
 * @return Result of API execution status
 *
 * @retval   0 -> Success.
 * @retval > 0 -> Warning.
 * @retval < 0 -> Fail.
 *
 */
int8_t bme280_get_sensor_mode(uint8_t *sensor_mode, struct bme280_dev *dev);

/**
 * \ingroup bme280
 * \defgroup bme280ApiSystem System
 * @brief API that performs system-level operations
 */

/*!
 * \ingroup bme280ApiSystem
 * \page bme280_api_bme280_soft_reset bme280_soft_reset
 * \code
 * int8_t bme280_soft_reset(struct bme280_dev *dev);
 * \endcode
 * @details This API soft-resets the sensor.
 *
 * @param[in,out] dev : Structure instance of bme280_dev.
 *
 * @return Result of API execution status.
 *
 * @retval   0 -> Success.
 * @retval > 0 -> Warning.
 * @retval < 0 -> Fail.
 *
 */
int8_t bme280_soft_reset(struct bme280_dev *dev);

/**
 * \ingroup bme280
 * \defgroup bme280ApiSensorData Sensor Data
 * @brief Data processing of sensor
 */

/*!
 * \ingroup bme280ApiSensorData
 * \page bme280_api_bme280_get_sensor_data bme280_get_sensor_data
 * \code
 * int8_t bme280_get_sensor_data(uint8_t sensor_comp, struct bme280_data *comp_data, struct bme280_dev *dev);
 * \endcode
 * @details This API reads the pressure, temperature and humidity data from the
 * sensor, compensates the data and store it in the bme280_data structure
 * instance passed by the user.
 *
 * @param[in] sensor_comp : Variable which selects which data to be read from
 * the sensor.
 *
 * sensor_comp |   Macros
 * ------------|-------------------
 *     1       | BME280_PRESS
 *     2       | BME280_TEMP
 *     4       | BME280_HUM
 *     7       | BME280_ALL
 *
 * @param[out] comp_data : Structure instance of bme280_data.
 * @param[in] dev : Structure instance of bme280_dev.
 *
 * @return Result of API execution status
 *
 * @retval   0 -> Success.
 * @retval > 0 -> Warning.
 * @retval < 0 -> Fail.
 *
 */
int8_t bme280_get_sensor_data(uint8_t sensor_comp, struct bme280_data *comp_data, struct bme280_dev *dev);

/*!
 * \ingroup bme280ApiSensorData
 * \page bme280_api_bme280_parse_sensor_data bme280_parse_sensor_data
 * \code
 * void bme280_parse_sensor_data(const uint8_t *reg_data, struct bme280_uncomp_data *uncomp_data);
 * \endcode
 *  @details This API is used to parse the pressure, temperature and
 *  humidity data and store it in the bme280_uncomp_data structure instance.
 *
 *  @param[in] reg_data     : Contains register data which needs to be parsed
 *  @param[out] uncomp_data : Contains the uncompensated pressure, temperature
 *  and humidity data.
 *
 */
void bme280_parse_sensor_data(const uint8_t *reg_data, struct bme280_uncomp_data *uncomp_data);

/*!
 * \ingroup bme280ApiSensorData
 * \page bme280_api_bme280_compensate_data bme280_compensate_data
 * \code
 * int8_t bme280_compensate_data(uint8_t sensor_comp,
 *                             const struct bme280_uncomp_data *uncomp_data,
 *                             struct bme280_data *comp_data,
 *                             struct bme280_calib_data *calib_data);
 * \endcode
 * @details This API is used to compensate the pressure and/or
 * temperature and/or humidity data according to the component selected by the
 * user.
 *
 * @param[in] sensor_comp : Used to select pressure and/or temperature and/or
 * humidity.
 * @param[in] uncomp_data : Contains the uncompensated pressure, temperature and
 * humidity data.
 * @param[out] comp_data : Contains the compensated pressure and/or temperature
 * and/or humidity data.
 * @param[in] calib_data : Pointer to the calibration data structure.
 *
 * @return Result of API execution status.
 *
 * @retval   0 -> Success.
 * @retval > 0 -> Warning.
 * @retval < 0 -> Fail.
 *
 */
int8_t bme280_compensate_data(uint8_t sensor_comp,
                              const struct bme280_uncomp_data *uncomp_data,
                              struct bme280_data *comp_data,
                              struct bme280_calib_data *calib_data);

/**
 * \ingroup bme280
 * \defgroup bme280ApiSensorDelay Sensor Delay
 * @brief Generic API for measuring sensor delay
 */

/*!
 * \ingroup bme280ApiSensorDelay
 * \page bme280_api_bme280_cal_meas_delay bme280_cal_meas_delay
 * \code
 * uint32_t bme280_cal_meas_delay(const struct bme280_settings *settings);
 * \endcode
 * @brief This API is used to calculate the maximum delay in milliseconds required for the
 * temperature/pressure/humidity(which ever are enabled) measurement to complete.
 * The delay depends upon the number of sensors enabled and their oversampling configuration.
 *
 * @param[in] settings : contains the oversampling configurations.
 *
 * @return delay required in milliseconds.
 *
 */
uint32_t bme280_cal_meas_delay(const struct bme280_settings *settings);

#ifdef __cplusplus
}
#endif /* End of CPP guard */
#endif /* BME280_H_ */
/** @}*/
