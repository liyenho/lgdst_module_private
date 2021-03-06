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

#define SILABS_RADIO_SI446X   //YH: added
/*------------------------------------------------------------------------*/
/*            Application specific global definitions                     */
/*------------------------------------------------------------------------*/
/*! Platform definition */
/* Note: Plaform is defined in Silabs IDE project file as
 * a command line flag for the compiler. */
//#define SILABS_PLATFORM_WMB930

/*! Extended driver support
 * Known issues: Some of the example projects
 * might not build with some extended drivers
 * due to data memory overflow */
#define RADIO_DRIVER_EXTENDED_SUPPORT

#define  RADIO_DRIVER_FULL_SUPPORT		// turn on by liyenho
#undef  SPI_DRIVER_EXTENDED_SUPPORT
#undef  HMI_DRIVER_EXTENDED_SUPPORT
#undef  TIMER_DRIVER_EXTENDED_SUPPORT
#undef  UART_DRIVER_EXTENDED_SUPPORT

/*------------------------------------------------------------------------*/
/*            Application specific includes                               */
/*------------------------------------------------------------------------*/

#include "compiler_defs.h"
#include "platform_defs_rx.h"
#include "hardware_defs_rx.h"

#include "application_defs.h"

//#include "drivers\cdd_common.h"
//#include "drivers\control_IO.h"
//#include "drivers\smbus.h"
#include "radio\core\spi.h"
//#include "drivers\hmi.h"
//#include "drivers\timer.h"
//#include "drivers\pca.h"
//#include "drivers\uart.h"
//#include "application\isr.h"
#include "radio.h"
//#include "application\sample_code_func.h"

#if ((defined SILABS_PLATFORM_LCDBB) || (defined SILABS_MCU_DC_EMIF_F930) || (defined SILABS_PLATFORM_WMB))
/* LCD driver includes */
#include "drivers\ascii5x7.h"  YH: should not be called
#include "drivers\dog_glcd.h"  YH: should not be called
#include "drivers\pictures.h"  YH: should not be called
#endif

#include "radio_hal.h"
#include "radio_comm.h"

#ifdef SILABS_RADIO_SI446X
#include "main.h" // all global definitions
#ifdef TEMPERATURE_MEASURE
	#include "radio_config-32mhz-915-cw.h"
#else	 // normal mode
	#include "radio_config.h"
#endif
#include "si446x_api_lib.h"
#include "si446x_defs.h"
#include "si446x_nirq.h"
//#include "drivers\radio\Si446x\si446x_patch.h"
#endif

#ifdef SILABS_RADIO_SI4455
#include "drivers\radio\si4455_api_lib.h"YH: should not be called
#include "drivers\radio\Si4455\si4455_defs.h"YH: should not be called
#include "drivers\radio\Si4455\si4455_nirq.h"YH: should not be called
#endif

#endif //BSP_H
