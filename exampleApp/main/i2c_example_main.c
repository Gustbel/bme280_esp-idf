/* BME280 example application

This example shows the basic use (configuration, initialization, read) of the
BME280 API

*/
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "bme280_api/BME280_ESP32C3.h"

static const char *TAG = "exampleApp";

#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */

#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define MESS_STR_SIZE 22

SemaphoreHandle_t print_mux = NULL;
static BME280_DEVICE device;
static BME280_DATA mess_data;

//
// I2C master initialization
//
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    
    if (err != ESP_OK) 
    {
        return err;
    }

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

//
// Test task
//
static void i2c_test_task(void *arg)
{
    uint8_t ret;
    uint32_t task_idx = (uint32_t)arg;
    uint8_t settings_sel;
    int cnt = 0;
    char mess_str[MESS_STR_SIZE];

    // Recommended mode of operation: Indoor navigation. Here is the
    // setup for oversampling and filtering, according to the BOSH
    // docs about Indoor navigation use. For other configuration, please
    // refer to the docs.
    device.settings.osr_h = BME280_OVERSAMPLING_1X;
    device.settings.osr_p = BME280_OVERSAMPLING_16X;
    device.settings.osr_t = BME280_OVERSAMPLING_2X;
    device.settings.filter = BME280_FILTER_COEFF_16;
    settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;
    ret = bme280_set_sensor_settings(settings_sel, &device);

    // Here is the main loop. Periodically reads and print the parameters
    // measured and compensated from BME280.
    while (1) 
    {
        ESP_LOGI(TAG, "T: %d test #: %d", task_idx, cnt++);

        // Setup the sensor operation mode, wait for a 40 ms
        // and then, read the compensated data. The read is made by
        // passing the mess_data struct, which contains the
        // meas result.
        // After getting the values, get a semaphore in order to
        // print the information for monitoring
        ret = bme280_set_sensor_mode(BME280_FORCED_MODE, &device);
        device.delay_us(40000, device.intf_ptr);
        ret = bme280_get_sensor_data(BME280_ALL, &mess_data, &device);
        xSemaphoreTake(print_mux, portMAX_DELAY);

        // Print information retrieved. If the connection was successful, print
        // each value from the mess_data struct in a readable way; after this,
        // print a constant-wide formatted string, that could be used to send the data
        // through another communication port.
        // If the connection is NOK, print an error message.
        if (ret == BME280_OK) 
        {
            printf("********************************\n");
            printf("T: %d -  READING SENSOR( BME280 )\n", task_idx);
            printf("********************************\n\n");
            printf("Print direct values:\n");
            printf("Temperature: %.1f °C\n", mess_data.temperature);
            printf("Pressure: %.2f Pa\n", mess_data.pressure);
            printf("Humidity: %.2f %%\n\n", mess_data.humidity);
            printf("Formated string:\n");
            sprintf(mess_str, "%+03.1f%+06.1f%+03.1f", mess_data.temperature, mess_data.pressure, mess_data.humidity);
            printf("%s\n", mess_str);
        } 
        else 
        {
            ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
        }

        // Give the semaphore taken and wait for the next read.
        xSemaphoreGive(print_mux);
        device.delay_us(1000000, device.intf_ptr);
    }

    vSemaphoreDelete(print_mux);
    vTaskDelete(NULL);
}

//
// This is the entry point for the example application. This shows how to
// use the BME280 API functions and data structures.
//
void app_main(void)
{
    print_mux = xSemaphoreCreateMutex();

    // Before access to the BME280, we need to setup the device handler
    // by assign the platform specific functions which brings access
    // to the communication port. These functions are implemented in
    // bme280_api/BME280_ESP32C3.c file.
    device.read = esp32c3_read_bme280;
    device.write = esp32c3_write_bme280;
    device.delay_us = esp32c3_delay_us_bme280;
    device.intf = BME280_I2C_INTF;

    // The first step is to initialize the I2C peripheral as usual
    ESP_ERROR_CHECK(i2c_master_init());

    // After I2C initialization, BME280 initialization could be done.
    if (BME280_E_DEV_NOT_FOUND == bme280_init(&device))
    {
        ESP_LOGW(TAG, "BME280 sensor is not connected.");
        while(1);
    }

    // For this example there is only one task which setup the BME280, and then
    // reads the parameters periodically.
    xTaskCreate(i2c_test_task, "i2c_test_task_0", 1024 * 2, (void *)0, 10, NULL);
}
