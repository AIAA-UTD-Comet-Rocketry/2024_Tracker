
#include "Si446x_spi.h"
#include "radio_config.h"
#include "si446x_defs.h"

static const char *TAG = "SI4463";

// SPI Handle
static spi_device_handle_t si4463_spi;

// function to check CTS (Clear to Send)
esp_err_t si4463_wait_cts(void* out, uint8_t outLen)
{
    uint8_t cmd[2 + outLen]; // READ_CMD_BUFF Command
    memset(cmd, 0xFF, sizeof(cmd));
    cmd[0] = SI446X_CMD_ID_READ_CMD_BUFF;

    uint8_t response[2 + outLen];
    memset(response, 0, sizeof(response));
    spi_transaction_t t = {};
    
    do {
        memset(&t, 0, sizeof(t));
        t.length = 16 + outLen*8;
        t.tx_buffer = cmd;
        t.rx_buffer = response;

        esp_err_t ret = spi_device_transmit(si4463_spi, &t);
        if (ret != ESP_OK) return ret;

        vTaskDelay(pdMS_TO_TICKS(1));  // Small delay to allow processing
    } while (response[1] != 0xFF);

    if(out){ // Out buffer exists
        memcpy(out, response+2, outLen);
    }

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

    return ret;
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
    si4463_wait_cts(NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "Si4463 Powered Up");

    return ESP_OK;
}
