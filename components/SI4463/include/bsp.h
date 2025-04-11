/*! @file bsp.h
 * @brief This file contains application specific definitions and includes.
 *
 * @b COPYRIGHT
 * @n Silicon Laboratories Confidential
 * @n Copyright 2012 Silicon Laboratories, Inc.
 * @n http://www.silabs.com
 */

#ifndef BSP_H
#define BSP_H

/*! Extended driver support 
 * Known issues: Some of the example projects 
 * might not build with some extended drivers 
 * due to data memory overflow */
#define RADIO_DRIVER_EXTENDED_SUPPORT

#undef  RADIO_DRIVER_FULL_SUPPORT
#undef  SPI_DRIVER_EXTENDED_SUPPORT
#undef  HMI_DRIVER_EXTENDED_SUPPORT
/*------------------------------------------------------------------------*/
/*            Application specific includes                               */
/*------------------------------------------------------------------------*/

#include "drivers\compiler_defs.h"
#include "platform_defs.h"
#include "hardware_defs.h"

#include "application\application_defs.h"

#include "drivers\spi.h"
#include "drivers\hmi.h"
#include "application\isr.h"
#include "radio_config.h"
#include "application\radio.h"
#include "application\sample_code_func.h"

#include "radio_hal.h"
#include "radio_comm.h"

#ifdef SILABS_RADIO_SI446X
#include "drivers\radio\Si446x\si446x_api_lib.h"
#include "drivers\radio\Si446x\si446x_defs.h"
#include "drivers\radio\Si446x\si446x_nirq.h"
//#include "drivers\radio\Si446x\si446x_patch.h"
#endif

#endif //BSP_H
