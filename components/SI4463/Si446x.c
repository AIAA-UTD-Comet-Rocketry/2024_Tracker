#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <stdint.h>
#include "driver/spi_master.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"

#include "radio_config.h"
#include "si446x_defs.h"

#define rssi_dBm(val)			((val / 2) - 134)

#define	delay_ms(ms)			vTaskDelay(pdMS_TO_TICKS(ms))
#define delay_us(us)			ets_delay_us(us)

static const uint8_t config[] = RADIO_CONFIGURATION_DATA_ARRAY;
