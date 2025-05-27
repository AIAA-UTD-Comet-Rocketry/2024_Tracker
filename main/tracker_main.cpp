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
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "radio_hal.h"
#include "si446x_api_lib.h"
#include "radio_config.h"
#include "aprs.h"

// Logging TAG
static const char *TAG = "ESP32-GPS-SI4468";

// GPS UART Configuration
#define GPS_UART_NUM   UART_NUM_0
#define GPS_TX_PIN     21  // TX not needed (GPS is sending data)
#define GPS_RX_PIN     20  // ESP32 RX (Connect to GPS TX)
#define BUF_SIZE       1024

// Si4463 SPI Configuration
#define SI4463_MOSI    0
#define SI4463_MISO    1
#define SI4463_SCK     10

// TinyGPS++ instance
TinyGPSPlus gps;

// APRS codec instance
APRSPacket packet;

// Function to Initialize GPS UART
void gps_uart_init() {
    uart_config_t uart_config = {};
    uart_config.baud_rate = 38400;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;

    uart_driver_install(GPS_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(GPS_UART_NUM, &uart_config);
    uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}


void aprs_init() {
    packet.source = AX25Address::from_string("KK7SSP-11");
    packet.destination = AX25Address::from_string("APRS");
    packet.path = { AX25Address::from_string("WIDE1-1"), AX25Address::from_string("WIDE2-1") };
}


// Function to Initialize SPI for Si4463
void spi_radio_bus_init() {
    spi_bus_config_t buscfg = {};  // Initialize to 0
    buscfg.mosi_io_num = SI4463_MOSI;
    buscfg.miso_io_num = SI4463_MISO;
    buscfg.sclk_io_num = SI4463_SCK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Si4463 SPI Initialized");
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
            char gps_buffer[64] = {};

            // Debug string
            snprintf(gps_buffer, sizeof(gps_buffer), "Lat: %.6f, Lon: %.6f",
                     gps.location.lat(), gps.location.lng());
            ESP_LOGI(TAG, "GPS: %s", gps_buffer);

            // Radio string
            packet.payload = snprintf(gps_buffer, sizeof(gps_buffer),
                                      "=%.2fN/%.2fW-Team 317", 
                                      gps.location.lat(), gps.location.lng());

            std::vector<uint8_t> APRSencoded = packet.encode();

            si446x_fifo_info(SI446X_CMD_FIFO_INFO_ARG_FIFO_TX_BIT); // Clear fifo ???
            si446x_get_int_status(0u, 0u, 0u); // Clear pending interrupts ???
            si446x_write_tx_fifo(APRSencoded.size(), (uint8_t*)APRSencoded.data());
            si446x_start_tx(RADIO_CONFIGURATION_DATA_CHANNEL_NUMBER, 0x30, APRSencoded.size()); // 0x00 len uses PKT_FIELD_X_LENGTH for tx

            /*
             * Without FIFO stitching or interrupt based packet management (ISR/DMA) there
             * is a limit to SI4463 radio frames containing data payloads less than 64 bytes.
             * Reduce comment field lengths or split transmission into multiple packets.
             */
            if(APRSencoded.size() >= 64)
                ESP_LOGI(TAG, "Radio FIFO error, check main file for comments", gps_buffer);
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for stability
    }
}


// Radio task for testing transcievers
void radio_test(void *pvParameters) {
  while(1) {
    uint8_t testBuff[8] = {0x07, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
    si446x_fifo_info(SI446X_CMD_FIFO_INFO_ARG_FIFO_TX_BIT); // Clear fifo ???
    si446x_get_int_status(0u, 0u, 0u); // Clear pending interrupts ???
    si446x_write_tx_fifo(8, testBuff);
    si446x_start_tx(RADIO_CONFIGURATION_DATA_CHANNEL_NUMBER, 0x30, 8); // 0x00 len uses PKT_FIELD_X_LENGTH for tx

    vTaskDelay(pdMS_TO_TICKS(1000));
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

const uint8_t defaultCmds[] = RADIO_CONFIGURATION_DATA_ARRAY;
extern "C" void app_main(void)
{
    chipIdEcho();

    ESP_LOGI(TAG, "Initializing GPS and Si4463 Transceiver");

    gps_uart_init();
    spi_radio_bus_init();
    radio_hal_Init(); // Si4463 connection

    si446x_configuration_init(defaultCmds);
    si446x_get_int_status(0u, 0u, 0u);

    //xTaskCreate(radio_test, "radio_test", 2048, NULL, 5, NULL);
    xTaskCreate(gps_task, "gps_task", 4096, NULL, 5, NULL);
}
