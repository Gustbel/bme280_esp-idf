#include "driver/i2c.h"
#include "driver/bme280.h"
#include "sdkconfig.h"

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)
#define BME280_SENSOR_ADDR 0x76
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */

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