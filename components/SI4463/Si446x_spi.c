/*
 * Project: Si4463 Radio Library for AVR and Arduino
 * Author: Zak Kemble, contact@zakkemble.co.uk
 * Copyright: (C) 2017 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 * Web: http://blog.zakkemble.co.uk/si4463-radio-library-avr-arduino/
 */

#include "Si446x_spi.h"
#include "radio_config.h"

static const char *TAG = "SI4463";

// SPI Handle
static spi_device_handle_t si4463_spi;

// Helper function to check CTS (Clear to Send)
static esp_err_t si4463_wait_cts()
{
    uint8_t cmd = 0x44;  // READ_CMD_BUFF Command
    uint8_t response;
    spi_transaction_t t = {};
    
    do {
        memset(&t, 0, sizeof(t));
        t.length = 8;
        t.tx_buffer = &cmd;
        t.rx_buffer = &response;

        esp_err_t ret = spi_device_transmit(si4463_spi, &t);
        if (ret != ESP_OK) return ret;

        vTaskDelay(pdMS_TO_TICKS(1));  // Small delay to allow processing
    } while (response != 0xFF);

    return ESP_OK;
}

// Function to send a command to Si4468
esp_err_t si4463_send_cmd(uint8_t *cmd, size_t cmd_len)
{
    spi_transaction_t t = {};
    memset(&t, 0, sizeof(t));
    t.length = cmd_len * 8;
    t.tx_buffer = cmd;

    esp_err_t ret = spi_device_transmit(si4463_spi, &t);
    if (ret != ESP_OK) return ret;

    return si4463_wait_cts();
}

// Function to initialize Si4468
esp_err_t si4463_init(spi_host_device_t host, gpio_num_t cs_pin)
{
    // SPI Device Configuration
    spi_device_interface_config_t devcfg = {};
    devcfg.command_bits = 0;
    devcfg.address_bits = 0;
    devcfg.dummy_bits = 0;
    devcfg.mode = 0;
    devcfg.clock_speed_hz = 10000000;  // 10 MHz
    devcfg.spics_io_num = cs_pin;
    devcfg.queue_size = 1;

    ESP_ERROR_CHECK(spi_bus_add_device(host, &devcfg, &si4463_spi));

    ESP_LOGI(TAG, "Si4463 SPI Initialized");

    // Reset sequence
    uint8_t power_up_cmd[] = {RF_POWER_UP};
    si4463_send_cmd(power_up_cmd, sizeof(power_up_cmd));
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "Si4463 Powered Up");

    return ESP_OK;
}
