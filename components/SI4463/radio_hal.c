/*!
 * File:
 *  radio_hal.c
 *
 * Description:
 *  This file contains RADIO HAL.
 *
 * Silicon Laboratories Confidential
 * Copyright 2011 Silicon Laboratories, Inc.
 */

                /* ======================================= *
                 *              I N C L U D E              *
                 * ======================================= */

#include "bsp.h"
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "driver/spi_master.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "Si446x_spi.h"


                /* ======================================= *
                 *          D E F I N I T I O N S          *
                 * ======================================= */

                /* ======================================= *
                 *     G L O B A L   V A R I A B L E S     *
                 * ======================================= */

                /* ======================================= *
                 *      L O C A L   F U N C T I O N S      *
                 * ======================================= */

                /* ======================================= *
                 *     P U B L I C   F U N C T I O N S     *
                 * ======================================= */

#define SI4463_CS   10
#define RADIO_SPI   SPI2_HOST

void radio_hal_Init(void){
  
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = 1<<SI4463_CS;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  gpio_config(&io_conf);

  gpio_set_level(SI4463_CS, 1);

  si4463_spi_init(RADIO_SPI);

#if 0
  gpio_config_t itr_conf = {};
  itr_conf.intr_type = GPIO_INTR_DISABLE;
  itr_conf.mode = GPIO_MODE_INPUT;
  itr_conf.pin_bit_mask = 1<<SI4463_IRQ;
  itr_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  itr_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  gpio_config(&io_conf);

  gpio_set_level(SI4463_CS, 1);
#endif
}

void radio_hal_AssertShutdown(void)
{
#if 0
  RF_PWRDN = 1;

#endif
}

void radio_hal_DeassertShutdown(void)
{
#if 0
  RF_PWRDN = 1;

#endif
}

void radio_hal_ClearNsel(void)
{
  gpio_set_level(SI4463_CS, 0);
}

void radio_hal_SetNsel(void)
{
  gpio_set_level(SI4463_CS, 1);
}

uint8_t radio_hal_NirqLevel(void)
{
  uint8_t retVal = 0;

#if 0
  retVal = gpio_get_level(SI4463_IRQ);
#endif

  return retVal;
}

void radio_hal_SpiWriteByte(uint8_t byteToWrite)
{
  uint8_t data[1] = {}; // Array to make sure array is in scope of SPI periph
  data[0] = byteToWrite;
  si4463_send_cmd(data, 1);
}

uint8_t radio_hal_SpiReadByte(void)
{
  uint8_t response[1];
  si4463_read_bytes(response, 1);
  return response[0];
}

void radio_hal_SpiWriteData(uint8_t byteCount, uint8_t* pData)
{
  si4463_send_cmd(pData, byteCount);
}

void radio_hal_SpiReadData(uint8_t byteCount, uint8_t* pData)
{
  si4463_read_bytes(pData, byteCount);
}

#ifdef RADIO_DRIVER_EXTENDED_SUPPORT
uint8_t radio_hal_Gpio0Level(void)
{
  uint8_t retVal = 0;

#if 0
  gpio_get_level(SI4463_GPIO0);
#endif

  return retVal;
}

uint8_t radio_hal_Gpio1Level(void)
{
  uint8_t retVal = 0;

#if 0
  gpio_get_level(SI4463_GPIO1);
#endif

  return retVal;
}

uint8_t radio_hal_Gpio2Level(void)
{
  uint8_t retVal = 0;

#if 0
  gpio_get_level(SI4463_GPIO2);
#endif

  return retVal;
}

uint8_t radio_hal_Gpio3Level(void)
{
  uint8_t retVal = 0;

#if 0
  gpio_get_level(SI4463_GPIO3);
#endif

  return retVal;
}

#endif
