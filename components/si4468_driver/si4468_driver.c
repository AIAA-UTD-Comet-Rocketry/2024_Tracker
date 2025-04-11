#include <stdio.h>
#include <inttypes.h>
#include <string.h>
//#include "driver/spi_master.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "si4468_defs.h"
#include "si4468_driver.h"
#include "driver/gpio.h"

static const char *TAG = "SI4468";

esp_err_t si4468_send_cmd(uint8_t *cmd, size_t cmd_len);

// SPI Handle
static spi_device_handle_t si4468_spi;

// Helper function to check CTS (Clear to Send)
static esp_err_t si4468_wait_cts()
{
    uint8_t cmd = 0x44;  // READ_CMD_BUFF Command
    uint8_t response;
    spi_transaction_t t = {};
    
    do {
        memset(&t, 0, sizeof(t));
        t.length = 8;
        t.tx_buffer = &cmd;
        t.rx_buffer = &response;

        esp_err_t ret = spi_device_transmit(si4468_spi, &t);
        if (ret != ESP_OK) return ret;

        vTaskDelay(pdMS_TO_TICKS(1));  // Small delay to allow processing
    } while (response != 0xFF);

    return ESP_OK;
}
esp_err_t si4468_write_tx_fifo(uint8_t *data, size_t len) {
    uint8_t cmd[65] = {0x66}; // WRITE_TX_FIFO (max 64 bytes payload)
    if (len > 64) return ESP_ERR_INVALID_SIZE;
    memcpy(cmd + 1, data, len);
    return si4468_send_cmd(cmd, len + 1);
}

esp_err_t si4468_read_rx_fifo(uint8_t *buf, size_t len) {
    uint8_t cmd = 0x77; // READ_RX_FIFO
    spi_transaction_t t = {};
    t.length = 8;
    t.tx_buffer = &cmd;
    t.rx_buffer = NULL;
    esp_err_t ret = spi_device_transmit(si4468_spi, &t);
    if (ret != ESP_OK) return ret;

    t.length = len * 8;
    t.tx_buffer = NULL;
    t.rx_buffer = buf;
    return spi_device_transmit(si4468_spi, &t);
}

// Function to send a command to Si4468
esp_err_t si4468_send_cmd(uint8_t *cmd, size_t cmd_len)
{
    spi_transaction_t t = {};
    memset(&t, 0, sizeof(t));
    t.length = cmd_len * 8;
    t.tx_buffer = cmd;

    esp_err_t ret = spi_device_transmit(si4468_spi, &t);
    if (ret != ESP_OK) return ret;

    return si4468_wait_cts();
}

// Function to initialize Si4468
esp_err_t si4468_init(spi_host_device_t host, gpio_num_t cs_pin)
{
    // SPI Device Configuration
    spi_device_interface_config_t devcfg = {};
    devcfg.command_bits = 0;
    devcfg.address_bits = 0;
    devcfg.dummy_bits = 0;
    devcfg.mode = 0;
    devcfg.clock_speed_hz = 4000000;  // 4 MHz
    devcfg.spics_io_num = cs_pin;
    devcfg.queue_size = 1;

    ESP_ERROR_CHECK(spi_bus_add_device(host, &devcfg, &si4468_spi));

    ESP_LOGI(TAG, "Si4468 SPI Initialized");

    // Reset sequence
    uint8_t power_up_cmd[] = {RF_POWER_UP};
    si4468_send_cmd(power_up_cmd, sizeof(power_up_cmd));
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "Si4468 Powered Up");

    return ESP_OK;
}

// Function to set RF frequency (144 MHz)
esp_err_t si4468_set_frequency(uint32_t freq_hz)
{
    esp_err_t err;

    //used param freq_hz
    //cmd = {0x11, 0x20, 0x01, 0x51, 0x0D}
    // 0x11 => set param
    // 0x20 => group
    // 0x01, 0x51 => change 1 param starting from 0x51
    // 0x0D => outdiv = /24 - Npresc = 2
    uint8_t band_cmd[] = {0x11, 0x20, 0x01, 0x51, 0x0D};

    err = si4468_send_cmd(band_cmd, sizeof(band_cmd));
    if (err != ESP_OK) {
        return err;
    }

    //used param freq_hz
    //cmd = {0x11, 0x40, 0x08, 0x00, 0x47, 0x0E, 0x00, 0x00, 0x14, 0x7B, 0x20, 0xFF}
    // 0x11 => set param
    // 0x40 => freq group
    // 0x08, 0x00 => change 8 params starting from begining of group
    // 0x47 => PLL div by int 71
    // 0x0E => PLL frac div [19:16]
    // 0x00 => PLL frac div [15:8]
    // 0x00 => PLL frac div [7:0]
    // 0x14 => Channel Size [15:8]
    // 0x7B => Channel Size [7:0]
    // 0x20 => Recommended Calibration Constant
    // 0xFF => Default Calibration Value
    // frequency formula: (int + frac / 2^19) * ((presc * 24Mhz)/odiv) = channel
    // real freq formula: (71 + 629146 / 2^19) * ((2 * 24Mhz)/24) = 145.500002 MHz
    // channel step size: (2^19 * odiv * target)/(presc * 24MHz) = Channel Size [15:0]
    // channel real size: (2^19 * 24 * 20kHz)/(2 * 24MHz) = 5243 = 0x147B (actual 20.0005 kHz)
    //
    // For us ***CENTER_FREQ = BASE + CHAN_NUM * CHAN_SIZE = 145.500 + N * 0.005 MHz***
    uint8_t freq_cmd[] = {RF_FREQ_CONTROL_INTE_8};

    err = si4468_send_cmd(freq_cmd, sizeof(freq_cmd));

    return err;
}

esp_err_t si4468_change_state(uint8_t next_state) {
    uint8_t cmd[] = {0x34, next_state};
    return si4468_send_cmd(cmd, sizeof(cmd));
}

// Function to set power output (20 dBm)
esp_err_t si4468_set_power(uint8_t power_level)
{
    //this is just default values for power
    //should make a compile option for freq calibration at lower power
    uint8_t pa_cmd[] = {0x11, 0x22, 0x02, 0x00, 0x08, 0x7F};
    return si4468_send_cmd(pa_cmd, sizeof(pa_cmd));
}
