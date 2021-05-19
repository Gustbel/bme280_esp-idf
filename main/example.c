#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_tls.h"

#include "i2c.h"
#include "src/bme280.h"
#include "src/bme280_defs.h"

void app_main(void) {

    printf("\n	-- BME280 lib test --\n");	

    // Init I2C
    init_i2c();

    // Init Sensor and set mode
    if (init_bme280()){
        printf("\nInitialized!!!\n");
    }
    else{
        printf("\nNO Initialized\n");
    }

    // Read data
    while(1){
        printf("Temperature: %.2f\n", get_bme280(0));
        vTaskDelay(15/portTICK_PERIOD_MS);
        printf("Humidity: %.2f\n", get_bme280(1));
        vTaskDelay(15/portTICK_PERIOD_MS);
        printf("Pressure: %.1f\n\n", get_bme280(2));
        vTaskDelay(15/portTICK_PERIOD_MS);
        vTaskDelay(1500/portTICK_PERIOD_MS);
    }
}
