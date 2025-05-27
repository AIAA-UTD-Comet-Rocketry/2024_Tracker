/*
 * Project: Si4463 Radio Library for AVR and Arduino
 * Author: Zak Kemble, contact@zakkemble.co.uk
 * Copyright: (C) 2017 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 * Web: http://blog.zakkemble.co.uk/si4463-radio-library-avr-arduino/
 */

#ifndef SI446X_SPI_H_
#define SI446X_SPI_H_

#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "driver/spi_master.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

esp_err_t si4463_spi_init(spi_host_device_t host);
esp_err_t si4463_send_cmd(uint8_t *cmd, size_t cmd_len);
esp_err_t si4463_read_bytes(uint8_t *pData, size_t data_len);

#endif /* SI446X_SPI_H_ */
