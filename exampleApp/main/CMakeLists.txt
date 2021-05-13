idf_component_register(SRCS "i2c_example_main.c" "bme280_api/BME280_ESP32C3.c" "bme280_api/driver/bme280.c"
                    INCLUDE_DIRS "." "./bme280_api/" "./bme280_api/driver/")
