/**
 * \file
 *
 * Copyright (c) 2014-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#ifndef BOOTLOADER_H_INCLUDED
#define BOOTLOADER_H_INCLUDED

#include <asf.h>
#include <string.h>
#include "conf_bootloader.h"

/* Check configuration */

/* -- Information editor --- */
#ifdef DBG_USE_INFO_EDIT

#  ifndef DBG_USE_USART
#    error DBG_USE_USART must be defined to enable DBG console for InfoEdit
#  endif

#  ifndef DBG_INFO_EDIT_TRIGGER_PIN
#    error DBG_INFO_EDIT_TRIGGER_PIN must be defined for InfoEdit trigger
#  endif

#  ifdef TRIGGER_USE_BUTTONS
#    ifdef TRIGGER_LOAD_BUTTON
#      if TRIGGER_LOAD_BUTTON == DBG_INFO_EDIT_TRIGGER_PIN
#        error TRIGGER_LOAD_BUTTON and DBG_INFO_EDIT_TRIGGER_PIN must be different
#      endif
#    endif

#    ifdef TRIGGER_SWITCH_BUTTON
#      if TRIGGER_SWITCH_BUTTON == DBG_INFO_EDIT_TRIGGER_PIN
#        error TRIGGER_SWITCH_BUTTON and DBG_INFO_EDIT_TRIGGER_PIN must be different
#      endif
#    endif
#  endif
#endif

/* -- medias (currently SD supported) --- */
#if (!defined(MEDIA_USE_COMM) && \
	!defined(MEDIA_USE_ETH)   && \
	!defined(MEDIA_USE_SD)    && \
	!defined(MEDIA_USE_MSD)   && \
	!defined(MEDIA_USE_NAND))
#  error MEDIA_USE_SD must be defined
#endif

#ifdef MEDIA_USE_COMM
/* -- communication    */
#  ifndef COMM_USE_USART
#    error COMM_USE_USART and USART options must be defined
#  endif
#  ifndef COMM_USE_XON_XOFF
#    error COMM_USE_XON_XOFF must be defined
#  endif
#endif
/* -- trigger          */
/* -- memory           */
#ifndef MEM_USE_FLASH
#  error MEM_USE_FLASH must be defined
#endif
/* -- regions          */

/* include files for bootloader */
#include "debug.h"
#include "memories.h"

bool main_vender_specific();  // main entry for host comm on usb

//#define FWM_DNLD_DBG		// don't turn in on unless it's necessary, the last chunk of data would not match but it is ok...
#define USB_DEVICE_SPECIFIC_REQUEST()			main_vender_specific()
#define USB_FWM_BOOTUP_VAL		0xbe	// ;-)
#define USB_FWM_UPDATE_VAL		0xef
#define USB_STREAM_ON_VAL			0xe
  #define USB_QUERY_IDX							0xff	// used to query run time indicator
#define FW_UPGRADE_HDR_LEN		(8)	// two int, [0]: check word, [1]: image length
#define FW_DNLD_SIZE								7200 // NEVER be divisible into 64 or usb core shall malfunction (a big bug!!!)

void jump_to_app(void * code_addr);
/* Global tick in ms */
extern volatile uint32_t tick_ms;

/*! \brief Opens the communication port
 * This is called by CDC interface when USB Host enable it.
 *
 * \retval true if cdc startup is successfully done
 */
bool main_cdc_enable(uint8_t port);

/*! \brief Closes the communication port
 * This is called by CDC interface when USB Host disable it.
 */
void main_cdc_disable(uint8_t port);

/*! \brief Manages the leds behaviors
 * Called when a start of frame is received on USB line each 1ms.
 */
void main_sof_action(void);

/*! \brief Enters the application in low power mode
 * Callback called when USB host sets USB line in suspend state
 */
void main_suspend_action(void);

/*! \brief Turn on a led to notify active mode
 * Called when the USB line is resumed from the suspend state
 */
void main_resume_action(void);

/*! \brief Save new DTR state to change led behavior.
 * The DTR notify that the terminal have open or close the communication port.
 */
void main_cdc_set_dtr(uint8_t port, bool b_enable);

#ifdef USB_DEVICE_LPM_SUPPORT
/*! \brief Enters the application in low power mode
 * Callback called when USB host sets LPM suspend state
 */
void main_suspend_lpm_action(void);

/*! \brief Called by UDC when USB Host request to enable LPM remote wakeup
 */
void main_remotewakeup_lpm_enable(void);

/*! \brief Called by UDC when USB Host request to disable LPM remote wakeup
 */
void main_remotewakeup_lpm_disable(void);
#endif

#endif /* #ifndef BOOTLOADER_H_INCLUDED */

