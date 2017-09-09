/**
 * \file
 *
 * \brief UART functions
 *
 * Copyright (c) 2009-2015 Atmel Corporation. All rights reserved.
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

#ifndef _UART_H_
#define _UART_H_

#include <string.h>
#include "conf_usb.h"
#include "ctrl.h"
#if (RECEIVE_MAVLINK |SEND_MAVLINK)
  #include "MavLink.h"
#endif

#define MAX_USART_PKT_LEN		120 // downlink from drone, variable length
#define MAVLINK_HDR_LEN				6
#define MAVLINK_START_SIGN		0x55
#define START_SIGN_LEN				1  // one byte
#define CHKSUM_LEN							2  // two bytes
#define RADIO_GRPPKT_LEN    	30	// uplink from controller, fixed length
#define USART_ACK_LEN					3 // "ATA"

typedef enum {
	STA_NULL= 0,
	SYNC_UP= 1,
	ACK_BACK =2,
	GET_LEN =3,
	QUEUE_UP= 4,
} usart_state;

typedef enum {	// mainly for maintenance purpose, liyenho
	CMD_NONE= 0,
	INIT_VIDEO= 1,
	START_VIDEO= 2,
	SND_FRAME= 3,
	REC_FRAME= 4,
} usart_cmd;

typedef unsigned char  usart_packet_rx[RADIO_GRPPKT_LEN+MAVLINK_HDR_LEN+CHKSUM_LEN];
typedef unsigned char  usart_packet_tx[MAX_USART_PKT_LEN/*-MAVLINK_HDR_LEN*/];

#define USART_PKT_QUEUE_LEN		8 // be reasonable short to minimize refresh latency
typedef struct {
	 usart_state state_rx, state_tx;
	 usart_state state_next_rx, state_next_tx;
	 usart_cmd cmd_extr, cmd_tx;
	usart_packet_rx *queue_start_rx;
	usart_packet_rx *queue_end_rx;
	usart_packet_rx *queue_ptr_rd;
	usart_packet_tx *queue_start_tx;
	usart_packet_tx *queue_end_tx;
	usart_packet_tx *queue_ptr_wr;
	uint8_t *mavlk_frm_ptr_rx, *mavlk_frm_ptr_tx;
	uint32_t chr_cnt_rx, chr_cnt_tx;
	//uint16_t mavlk_frm_sz;
	uint16_t mavlk_chksum_rx,mavlk_chksum_tx;
	uint8_t frm_squ_rx;
} ctx_usart;

#if RECEIVE_MAVLINK
  #define GET_MSG_FRM_RADIO(incoming_data) \
				Queue_MavLink_Raw_Data(&incoming_MavLink_Data, UART_BUFFER_SIZE, incoming_data);
#else
  #define GET_MSG_FRM_RADIO(incoming_data) \
			for (int i =0; i< (UART_BUFFER_SIZE/RADIO_PKT_LEN);i++){ \
				wrptr_inc(&wrptr_rdo_rpacket, &rdptr_rdo_rpacket, RDO_TPACKET_FIFO_SIZE, 1); \
				memcpy(gs_rdo_rpacket + (wrptr_rdo_rpacket*RDO_ELEMENT_SIZE), incoming_data+(i*RADIO_PKT_LEN), RADIO_PKT_LEN); \
			} uint32_t wrptr_tmp=wrptr_rdo_rpacket; \
			wrptr_inc(&wrptr_tmp, &rdptr_rdo_rpacket, (uint32_t)RDO_RPACKET_FIFO_SIZE,1); \
			gp_rdo_rpacket_l = gs_rdo_rpacket + (RDO_ELEMENT_SIZE*wrptr_tmp);
#endif

extern uint32_t rxnorec_intv, hop_watchdog_intv;

#define ISR_USART_P(usart_base,\
																	overrun_cnt,\
																	buffer1,\
																	buffer2,\
																	nxt_loc,\
																	pdc_packet,\
																	pdc_packet2,\
																	p_pdc) \
	uint32_t sr = usart_get_status(usart_base); \
	/* disable TXRDY to avoid usb cdc rx get interferred, liyenho*/ \
	if ((sr & US_CSR_OVRE) == US_CSR_OVRE){ \
		overrun_cnt++; \
		} \
	if ((sr & US_CSR_ENDRX)== US_CSR_ENDRX){ \
		static bool buffer1_full = true; \
		uint8_t * buffer_to_read = buffer1_full?buffer1:buffer2; \
		if (buffer1_full){ \
			nxt_loc = &pdc_packet2; \
		}else{ \
			nxt_loc = &pdc_packet; \
		} \
		pdc_rx_init(p_pdc, NULL, nxt_loc);

#define ISR_USART_E(other_cnt) \
		buffer1_full = (!buffer1_full); \
							} \
	else{ \
		/*other interrupt*/ \
		other_cnt++; \
							} \
	if (sr & US_CSR_RXRDY) { \
		/*Data received*/ \
						}

#define ISR_USART_RADIO(usart_base,\
																	overrun_cnt,\
																	buffer1,\
																	buffer2,\
																	nxt_loc,\
																	pdc_packet,\
																	pdc_packet2,\
																	p_pdc,\
																	other_cnt) \
		ISR_USART_P(usart_base,\
											overrun_cnt,\
											buffer1,\
											buffer2,\
											nxt_loc,\
											pdc_packet,\
											pdc_packet2,\
											p_pdc) \
		radio_mon_rxcnt ++; \
		hop_watchdog_intv =0; \
		rxnorec_intv = 0; \
		GET_MSG_FRM_RADIO(buffer_to_read) \
		ISR_USART_E(other_cnt)

#ifdef CONF_BOARD_USB_RX
	#define ISR_USART_DRONE(usart_base,\
																		overrun_cnt,\
																		buffer1,\
																		buffer2,\
																		nxt_loc,\
																		pdc_packet,\
																		pdc_packet2,\
																		p_pdc,\
																		other_cnt) \
		ISR_USART_P(usart_base,\
											overrun_cnt,\
											buffer1,\
											buffer2,\
											nxt_loc,\
											pdc_packet,\
											pdc_packet2,\
											p_pdc) \
		Queue_MavLink_Raw_Data( &outgoing_MavLink_Data, UART_BUFFER_SIZE, buffer_to_read); \
		ISR_USART_E(other_cnt)
#endif

void Configure_UART_DMA(uint8_t port);

/*! \brief Called by CDC interface
 * Callback running when CDC device have received data
 */
void uart_rx_notify(uint8_t port);

/*! \brief Configures communication line
 *
 * \param cfg      line configuration
 */
void uart_config(uint8_t port, usb_cdc_line_coding_t * cfg);

/*! \brief Opens communication line
 */
void uart_open(uint8_t port);

/*! \brief Closes communication line
 */
void uart_close(uint8_t port);

void uart_send_message(char* msg);
#if (SEND_MAVLINK)
  void uart_send_Mavlink(MavLinkPacket pkt);
#endif
void uart_Send_Data(uint8_t *data, uint32_t bytes);
#ifdef CONF_BOARD_USB_RX
	void uart7_send_message(char* msg);
	#if (SEND_MAVLINK)
	  void uart7_send_Mavlink(MavLinkPacket pkt);
	#endif
	void uart7_Send_Data(uint8_t *data, uint32_t bytes);
#endif

#endif // _UART_H_
