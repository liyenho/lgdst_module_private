/**
 * \file
 *
 * \brief USART Serial Configuration
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

#ifndef CONF_USART_SERIAL_H_INCLUDED
#define CONF_USART_SERIAL_H_INCLUDED

/** UART Interface */
#define CONF_UART            CONSOLE_UART
/** Baudrate setting */
#define CONF_UART_BAUDRATE   (115200UL)
/** Character length setting */
#define CONF_UART_CHAR_LENGTH  US_MR_CHRL_8_BIT
/** Parity setting */
#define CONF_UART_PARITY     US_MR_PAR_NO
/** Stop bits setting */
#define CONF_UART_STOP_BITS    US_MR_NBSTOP_1_BIT

#define UART_TEST
#define MAX_USART_PKT_LEN		263 // downlink from drone, variable length
#define MAVLINK_HDR_LEN				6
#define RADIO_GRPPKT_LEN    	30	// uplink from controller, fixed length
				// two byte period per 115200 baud, 120 mhz core clock assumed
#define USART_WAIT_TIME				20833

typedef enum {
	SYNC_DOWN= 0,
	ACK_BACK =1,
	GET_LEN =2,
	QUEUE_UP= 3,
	IN_SYNC= 4,
} usart_state;

typedef enum {
	CMD_NONE= 0,
	INIT_VIDEO= 1,
	START_VIDEO= 2,
	SND_FRAME= 3,
	REC_FRAME= 4,
} usart_cmd;

typedef unsigned char  usart_packet_rx[RADIO_GRPPKT_LEN+MAVLINK_HDR_LEN];
typedef unsigned char  usart_packet_tx[MAX_USART_PKT_LEN-MAVLINK_HDR_LEN];

#define USART_PKT_QUEUE_LEN		10
#define USART_AT_CMD_LEN				3
#define USART_PKT_LEN_LEN				sizeof(short) //MAX_USART_PKT_LEN

typedef struct {
	enum usart_state state;
	enum usart_cmd cmd;
	usart_packet *buffer_queue_rx;
	usart_packet *queue_start_rx;
	usart_packet *queue_end_rx;
	usart_packet *queue_ptr_rd;
	usart_packet *buffer_queue_tx;
	usart_packet *queue_start_tx;
	usart_packet *queue_end_tx;
	usart_packet *queue_ptr_wr;
	uint8_t *mavlk_frm_ptr;
	uint32_t this_rec_tm;
	uint32_t last_rec_tm;
	uint32_t chr_cnt;
	uint8_t last_at_cmd[USART_AT_CMD_LEN];
	uint8_t frm_squ_rx;
	uint16_t mavlk_frm_sz;
	uint16_t mavlk_chksum;
} ctx_usart;

#endif/* CONF_USART_SERIAL_H_INCLUDED */
