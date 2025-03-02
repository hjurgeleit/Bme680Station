/***************************************************************************************
 * @file        main.c
 * @brief       I2C communication with the BME680 sensor on the ESP32 using ESP-IDF.
 *              Initializes I2C, configures the BME680, and reads sensor data.
 * @author      Heiko Jurgeleit <jurgeleit.heiko@gmail.com>
 * @date        March 2, 2025
 * @version     1.0
 * @license     MIT License (see LICENSE file for details)
 *
 * @copyright   Copyright (c) 2025 Heiko Jurgeleit <jurgeleit.heiko@gmail.com>
 *
 * @details     Uses ESP-IDF v5.4 and bme68x library v4.4.8 to log temperature,
 *              pressure, humidity, and gas resistance data periodically.
 *
 * @hardware    ESP32 Dev Board, BME680 Sensor, I2C connections (configurable pins/freq).
 * @config      Requires I2C_MASTER_SCL, I2C_MASTER_SDA in sdkconfig.
 *
 * @see         BME680 Docs: https://www.bosch-sensortec.com/products/environmental-sensors/gas-sensors/bme680/
 *              Bosch BME68x SensorAPI: https://github.com/boschsensortec/BME68x_SensorAPI
 ***************************************************************************************/

#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_rom_sys.h"
#include "bme68x.h"

#define SCL_IO_NUM       CONFIG_I2C_MASTER_SCL
#define SDA_IO_NUM       CONFIG_I2C_MASTER_SDA
#define PORT_NUMBER      -1
#define SCL_SPEED_HZ     100000
#define SCL_WAIT_US      1000

static const char *TAG = "MAIN";

/**
 * @brief Reads data from the BME680 via I2C.
 * 
 * @param[in]  reg_addr  Register address in the sensor.
 * @param[out] reg_data  Buffer to store the read data.
 * @param[in]  length    Number of bytes to read.
 * @param[in]  intf_ptr  Pointer to the I2C device handle.
 * 
 * @return 0 on success, -1 on error.
 */
int8_t user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr) {
    i2c_master_dev_handle_t dev_handle = (i2c_master_dev_handle_t) intf_ptr;
    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg_addr, 1, reg_data, length, 1000 / portTICK_PERIOD_MS);

    if (ret != ESP_OK) {
        ESP_LOGE("I2C_READ", "Error reading from 0x%02X: %s", reg_addr, esp_err_to_name(ret));
    }

    return ret == ESP_OK ? 0 : -1;
}

/**
 * @brief Writes data to the BME680 via I2C.
 * 
 * @param[in] reg_addr   Register address in the sensor.
 * @param[in] reg_data   Data to write.
 * @param[in] length     Number of bytes to write.
 * @param[in] intf_ptr   Pointer to the I2C device handle.
 * 
 * @return 0 on success, -1 on error.
 */
int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr) {
    i2c_master_dev_handle_t dev_handle = (i2c_master_dev_handle_t) intf_ptr;
    uint8_t buffer[length + 1];
    buffer[0] = reg_addr;
    memcpy(&buffer[1], reg_data, length);
    esp_err_t ret = i2c_master_transmit(dev_handle, buffer, length + 1, 1000 / portTICK_PERIOD_MS);

    if (ret != ESP_OK) {
        ESP_LOGE("I2C_WRITE", "Fehler beim Schreiben auf 0x%02X: %s", reg_addr, esp_err_to_name(ret));
    }

    return ret == ESP_OK ? 0 : -1;
}

/**
 * @brief Delays execution for a specified period in microseconds.
 * 
 * @param[in] period     Delay duration in microseconds.
 * @param[in] intf_ptr   Pointer to the interface (not used).
 */
void user_delay_us(uint32_t period, void *intf_ptr) {
    esp_rom_delay_us(period);
}

/**
 * @brief Main entry point of the program.
 * 
 * Initializes the I2C interface, configures the BME680 sensor, and
 * periodically reads sensor data, which is logged to the console.
 */
void app_main(void)
{
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = PORT_NUMBER,
        .scl_io_num = SCL_IO_NUM,
        .sda_io_num = SDA_IO_NUM,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = 1,
            .allow_pd = 0
        }
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

    i2c_master_dev_handle_t dev_handle;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BME68X_I2C_ADDR_HIGH,
        .scl_speed_hz = SCL_SPEED_HZ,
        .scl_wait_us = SCL_WAIT_US,
        .flags = {
            .disable_ack_check = 0
        }
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    struct bme68x_dev bme680_device = {
        .intf = BME68X_I2C_INTF,
        .intf_ptr = dev_handle,
        .read = user_i2c_read,
        .write = user_i2c_write,
        .delay_us = user_delay_us
    };
    ESP_ERROR_CHECK(bme68x_init(&bme680_device));

    struct bme68x_conf bme680_conf = {
        .os_hum = BME68X_OS_4X,
        .os_temp = BME68X_OS_4X,
        .os_pres = BME68X_OS_4X,
        .filter = BME68X_FILTER_SIZE_3,
        .odr = BME68X_ODR_NONE
    };
    ESP_ERROR_CHECK(bme68x_set_conf(&bme680_conf, &bme680_device));

    struct bme68x_heatr_conf bme680_heatr_conf = {
        .enable = BME68X_ENABLE,
        .heatr_temp = BME68X_HIGH_TEMP,
        .heatr_dur = BME68X_HEATR_DUR1
    };
    ESP_ERROR_CHECK(bme68x_set_heatr_conf(BME68X_FORCED_MODE, &bme680_heatr_conf, &bme680_device));

    uint8_t chip_id = 0;
    bme680_device.read(0xD0, &chip_id, 1, dev_handle);
    ESP_LOGI(TAG, "Chip ID mit struct call: 0x%02X", chip_id);

    ESP_LOGI(TAG, "Starting endless messuring...");
    while (1) {
        struct bme68x_data data[3];
        uint8_t n_fields;
       //uint32_t del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &bme680_conf, &bme680_device) + bme680_heatr_conf.heatr_dur;
        
        ESP_ERROR_CHECK(bme68x_set_op_mode(BME68X_FORCED_MODE, &bme680_device));
        ESP_LOGI(TAG, "Waiting for the measurement to complete...");
        bme680_device.delay_us(BME68X_HEATR_DUR2_DELAY, dev_handle);
        
        int8_t rslt = bme68x_get_data(BME68X_FORCED_MODE, data, &n_fields, &bme680_device);
        if (rslt != BME68X_OK) {
            ESP_LOGE(TAG, "Error reading sensor data!");

            continue;
        }

        for (uint8_t i = 0; i < n_fields; i++) {
            ESP_LOGI(TAG, "Temp: %.2f °C", data[i].temperature);
            ESP_LOGI(TAG, "Air pressure: %.2f hPa", data[i].pressure / 100.0);
            //ESP_LOGI(TAG, "Humidity: %.2f %%", data[i].humidity / 1000.0);
            ESP_LOGI(TAG, "Humidity: %.2f %%", data[i].humidity); // looks like the value is not x1000
            ESP_LOGI(TAG, "Gas resistance: %.2f Ohm", (float)data[i].gas_resistance);
        }

        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
