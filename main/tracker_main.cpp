/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "driver/uart.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "TinyGPS++.h"
#include "si4468_defs.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"

// Logging TAG
static const char *TAG = "ESP32-GPS-SI4468";

// GPS UART Configuration
#define GPS_UART_NUM   UART_NUM_1
#define GPS_TX_PIN     4  // TX not needed (GPS is sending data)
#define GPS_RX_PIN     5  // ESP32 RX (Connect to GPS TX)
#define BUF_SIZE       1024

// Si4468 SPI Configuration
#define SI4468_MOSI    23
#define SI4468_MISO    19
#define SI4468_SCK     18
#define SI4468_CS      10

// TinyGPS++ Instance
TinyGPSPlus gps;

// SPI Device Handle
spi_device_handle_t si4468_handle;

// Function to Initialize GPS UART
void gps_uart_init() {
    uart_config_t uart_config = {};
    uart_config.baud_rate = 9600;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;

    uart_driver_install(GPS_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(GPS_UART_NUM, &uart_config);
    uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

// Function to Initialize SPI for Si4468
void spi_radio_bus_init() {
    spi_bus_config_t buscfg = {};  // Initialize to 0
    buscfg.mosi_io_num = SI4468_MOSI;
    buscfg.miso_io_num = SI4468_MISO;
    buscfg.sclk_io_num = SI4468_SCK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Si4468 SPI Initialized");
}

// Function to Send Data to Si4468
void send_si4468(const char *data) {
    spi_transaction_t trans = {};
    trans.length = strlen(data) * 8; // Length in bits
    trans.tx_buffer = data;

    esp_err_t ret = spi_device_transmit(si4468_handle, &trans);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Data Sent: %s", data);
    } else {
        ESP_LOGE(TAG, "SPI Transmission Failed");
    }
}

// Main Task for GPS Data Processing
void gps_task(void *pvParameters) {
    uint8_t data[BUF_SIZE];
    while (true) {
        int len = uart_read_bytes(GPS_UART_NUM, data, BUF_SIZE, 100 / portTICK_PERIOD_MS);
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                gps.encode(data[i]);
            }
        }

        // If new GPS location is available
        if (gps.location.isUpdated()) {
            char gps_buffer[64];
            snprintf(gps_buffer, sizeof(gps_buffer), "Lat: %.6f, Lon: %.6f",
                     gps.location.lat(), gps.location.lng());

            ESP_LOGI(TAG, "GPS: %s", gps_buffer);
            send_si4468(gps_buffer);
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for stability
    }
}

void chipIdEcho() {
    printf("Starting Chip Identification");

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n\n", esp_get_minimum_free_heap_size());
}

extern "C" void app_main(void)
{
    chipIdEcho();

    ESP_LOGI(TAG, "Initializing GPS and Si4468 Transceiver");

    gps_uart_init();
    spi_radio_bus_init();

    si4468_init(SPI2_HOST, SI4468_CS);
    si4468_set_frequency(144000000);  // Set 144 MHz
    si4468_set_power(127);  // 127 = ~20dBm

    xTaskCreate(gps_task, "gps_task", 4096, NULL, 5, NULL);
}
