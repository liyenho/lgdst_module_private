/**
 * \file
 *
 * \brief UART functions
 *
 * Copyright (c) 2012-2015 Atmel Corporation. All rights reserved.
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

#include <asf.h>
#include <assert.h>
#include "conf_example.h"
#include "uart.h"
#include "main.h"
#include "ui.h"
#if (SAMG55)
#include "flexcom.h"
#endif
#include "ctrl.h"  // for tm_delta() def
#include "delay.h"

#if SAM4L
#   define USART_PERIPH_CLK_ENABLE() sysclk_enable_peripheral_clock(USART_BASE)
#elif SAMG55
#   define USART_PERIPH_CLK_ENABLE() flexcom_enable(BOARD_FLEXCOM);      \
	                                                        flexcom_set_opmode(BOARD_FLEXCOM, FLEXCOM_USART);
#else
#   define USART_PERIPH_CLK_ENABLE() sysclk_enable_peripheral_clock(USART_ID)
#endif

extern volatile uint32_t *DWT_CYCCNT;
volatile static uint32_t prev_time_intr = 0,
												prev_time_snd = 0,
												tdelx_intr, tdelx;  // timing measure facility
extern volatile bool init_video_flag, start_video_flag;

static sam_usart_opt_t usart_options;
//volatile static uint8_t rec_intr_buffer[MAX_USART_PKT_LEN];
static volatile bool uart_port_start0 = false,
										uart_port_start = false/*true*/,
										first_ignored = true;  // timing measure facility

static volatile uint8_t dbg_chr_rx[1000] ;
static volatile uint32_t dbg_chr_cnt ;

volatile ctx_usart gl_usart_comm_ctx ; // global interrupt context for serial comm
			// point of view from control link
static usart_packet_rx usart_queue_recv[USART_PKT_QUEUE_LEN];
  const uint8_t mavlk_hdr_rx[MAVLINK_HDR_LEN] = {
	  																		0x55, // start sign
  																			RADIO_GRPPKT_LEN+MAVLINK_HDR_LEN,
  																			0x0, // sequence, to be filled in
  																			0x01,
  																			0x14,
  																			0x00, // msg def, to be filled in by Bill
  																		};
static usart_packet_tx usart_queue_sent[USART_PKT_QUEUE_LEN];

#ifdef UART_TEST
 #define BACK_TO_NONE_RX \
	gl_usart_comm_ctx.chr_cnt_rx = 3;/*wait for at cmd*/ \
	gl_usart_comm_ctx.mavlk_chksum_rx = X25_INIT_CRC; \
	gl_usart_comm_ctx.state_rx = STA_NULL; \
	gl_usart_comm_ctx.cmd_extr = CMD_NONE;
#else
 #define BACK_TO_NONE_RX \
	gl_usart_comm_ctx.chr_cnt_rx = START_SIGN_LEN; \
	gl_usart_comm_ctx.mavlk_chksum_rx = X25_INIT_CRC; \
	gl_usart_comm_ctx.state_rx = STA_NULL; \
	gl_usart_comm_ctx.cmd_extr = CMD_NONE;
#endif
#define BACK_TO_NONE_TX \
	gl_usart_comm_ctx.chr_cnt_tx = 0; \
	gl_usart_comm_ctx.mavlk_chksum_tx = X25_INIT_CRC; \
	gl_usart_comm_ctx.state_tx = STA_NULL; \
	gl_usart_comm_ctx.cmd_tx = CMD_NONE;

ISR(USART_HANDLER)  // we need this ISR to pump status data to usb cdc comm, liyenho
{
	uint32_t sr = usart_get_status(USART_BASE);
	uint32_t tdel, cur_time, prev_time = *DWT_CYCCNT;
	// disable TXRDY to avoid usb cdc rx get interferenced, liyenho
	if (sr & US_CSR_RXRDY) {
		// Data received
		//ui_com_tx_start();
		uint32_t value;
		bool b_error = usart_read(USART_BASE, &value) ||
			(sr & (US_CSR_FRAME | US_CSR_TIMEOUT | US_CSR_PARE));
		if (b_error) {
			usart_reset_rx(USART_BASE);
			usart_enable_rx(USART_BASE);
			//udi_cdc_signal_framing_error();
			//ui_com_error();
			BACK_TO_NONE_RX
			return;
		}
  if (1000 > dbg_chr_cnt)  // for debug
  dbg_chr_rx[dbg_chr_cnt++] = (char)value;
#if false  // not used for usb cdc app...
		// Transfer UART RX fifo to CDC TX
		if (!udi_cdc_is_tx_ready()) {
			// Fifo full
			udi_cdc_signal_overrun();
			ui_com_overflow();
		} else {
			udi_cdc_putc(value);
		}
		ui_com_tx_stop();
#else
		switch(gl_usart_comm_ctx.state_rx) {
			case STA_NULL:
						if (ACK_BACK == gl_usart_comm_ctx.state_next_tx) {
							if ('A' == (char)value) {
								gl_usart_comm_ctx.chr_cnt_rx = USART_ACK_LEN-1;
								gl_usart_comm_ctx.state_rx = QUEUE_UP;
							}
							else  {
								gl_usart_comm_ctx.queue_ptr_rd += 1;
								if (gl_usart_comm_ctx.queue_end_rx-1<gl_usart_comm_ctx.queue_ptr_rd)
									gl_usart_comm_ctx.queue_ptr_rd = gl_usart_comm_ctx.queue_end_rx-1;
								 gl_usart_comm_ctx.state_next_tx = STA_NULL;
								BACK_TO_NONE_RX // bad link
							}
							break;
						}
#ifdef UART_TEST
						else if ('A' == (char)value) {
												if (3 == gl_usart_comm_ctx.chr_cnt_rx--) {
													gl_usart_comm_ctx.state_rx = SYNC_UP;
												}
												else {
													BACK_TO_NONE_RX
												}
											}
#endif
						else if (MAVLINK_START_SIGN == (char)value) {
												gl_usart_comm_ctx.state_rx = SYNC_UP;
												gl_usart_comm_ctx.chr_cnt_rx = START_SIGN_LEN;
												gl_usart_comm_ctx.mavlk_frm_ptr_rx= /*rec_intr_buffer*/gl_usart_comm_ctx.queue_ptr_wr;
												*gl_usart_comm_ctx.mavlk_frm_ptr_rx++ = MAVLINK_START_SIGN;
											}
						break;
			case SYNC_UP:
#ifdef UART_TEST
						if ('T' == (char)value && 2 == gl_usart_comm_ctx.chr_cnt_rx)
							  { gl_usart_comm_ctx.chr_cnt_rx-- ;  }
						else if (('1' == (char)value || '2' == (char)value || '0' == (char)value) &&
										1 == gl_usart_comm_ctx.chr_cnt_rx ) {
								gl_usart_comm_ctx.chr_cnt_rx = 3;
								if ('0' == (char)value)
									uart_port_start0 = true;  // initial uart sub-system start
								else if ('1' == (char)value)
									init_video_flag = true;
								else if ('2' == (char)value)
									start_video_flag = true;
								BACK_TO_NONE_RX
								if (STA_NULL == gl_usart_comm_ctx.state_tx &&
										gl_usart_comm_ctx.state_next_tx == STA_NULL) {
									usart_enable_tx(USART_BASE);
									usart_enable_interrupt(USART_BASE, US_IER_TXRDY); // turn on Tx pipe, liyenho
									gl_usart_comm_ctx.state_next_rx = ACK_BACK;
									goto send_ack_1st_char;  // so we can trigger next US_CSR_TXRDY
								}
						} else
#endif
						if (!--gl_usart_comm_ctx.chr_cnt_rx) {
							if ((MAX_USART_PKT_LEN)>value &&
								(MAVLINK_HDR_LEN+CHKSUM_LEN)<=value ) {
								gl_usart_comm_ctx.chr_cnt_rx = (char)value-1;
								//gl_usart_comm_ctx.mavlk_frm_sz = (char)value;
								*gl_usart_comm_ctx.mavlk_frm_ptr_rx++= (char)value;
								crc_accumulate((uint8_t)value, &gl_usart_comm_ctx.mavlk_chksum_rx);
								gl_usart_comm_ctx.cmd_extr= REC_FRAME;
								gl_usart_comm_ctx.state_rx = QUEUE_UP;
							}
							else { // has to be greater than chksum length
								BACK_TO_NONE_RX
							}
						}
						break;
			case QUEUE_UP:
						if (gl_usart_comm_ctx.cmd_extr== REC_FRAME) {
							if (--gl_usart_comm_ctx.chr_cnt_rx) {
								*gl_usart_comm_ctx.mavlk_frm_ptr_rx++ = (char)value;
								if ( CHKSUM_LEN<=gl_usart_comm_ctx.chr_cnt_rx)
									crc_accumulate((uint8_t)value, &gl_usart_comm_ctx.mavlk_chksum_rx);
							}
							else { // check if a mavlink frame recv-ed
									*gl_usart_comm_ctx.mavlk_frm_ptr_rx = (char)value;
								uint16_t chksum;
								  chksum = *(gl_usart_comm_ctx.mavlk_frm_ptr_rx-1);
								  chksum |= ((uint16_t)*(gl_usart_comm_ctx.mavlk_frm_ptr_rx)<<8);
								if (chksum == gl_usart_comm_ctx.mavlk_chksum_rx) {
									// it is a valid packet
									if (gl_usart_comm_ctx.queue_end_tx-1 >gl_usart_comm_ctx.queue_ptr_wr) {
										//memcpy(gl_usart_comm_ctx.queue_ptr_wr,
										//					rec_intr_buffer/*+MAVLINK_HDR_LEN*/,
										//					gl_usart_comm_ctx.mavlk_frm_sz/*-MAVLINK_HDR_LEN*/);
										gl_usart_comm_ctx.queue_ptr_wr += 1;
									} // otherwise, host forwards too fast resulted buffer overflow
									//gl_usart_comm_ctx.mavlk_frm_sz =0;
									BACK_TO_NONE_RX
									if (STA_NULL == gl_usart_comm_ctx.state_tx &&
											gl_usart_comm_ctx.state_next_tx == STA_NULL) {
										usart_enable_tx(USART_BASE);
										usart_enable_interrupt(USART_BASE, US_IER_TXRDY); // turn on Tx pipe, liyenho
										gl_usart_comm_ctx.state_next_rx = ACK_BACK;
										goto send_ack_1st_char;  // so we can trigger next US_CSR_TXRDY
									}
								}
								else {
									gl_usart_comm_ctx.mavlk_frm_ptr_rx = 0;
									//gl_usart_comm_ctx.mavlk_frm_sz =0;
									BACK_TO_NONE_RX
								}
							}
						}
						else if (ACK_BACK == gl_usart_comm_ctx.state_next_tx) {
							/*if (!gl_usart_comm_ctx.chr_cnt_rx) {
								gl_usart_comm_ctx.state_next_tx = STA_NULL;
								BACK_TO_NONE_RX
							}
							else*/
								switch(gl_usart_comm_ctx.chr_cnt_rx) {
									case USART_ACK_LEN-1:
												if ('T' == (char)value)
													gl_usart_comm_ctx.chr_cnt_rx -= 1;
												else  {
													gl_usart_comm_ctx.queue_ptr_rd += 1;
													if (gl_usart_comm_ctx.queue_end_rx-1<gl_usart_comm_ctx.queue_ptr_rd)
														gl_usart_comm_ctx.queue_ptr_rd = gl_usart_comm_ctx.queue_end_rx-1;
													 gl_usart_comm_ctx.state_next_tx = STA_NULL;
													BACK_TO_NONE_RX // bad link
												}
												break;
									case USART_ACK_LEN-2:
												if ('K' == (char)value) {
													gl_usart_comm_ctx.chr_cnt_rx -= 1;
													// no more chr comes in...
													gl_usart_comm_ctx.state_next_tx = STA_NULL;
													BACK_TO_NONE_RX
												}
												else  {
													gl_usart_comm_ctx.queue_ptr_rd += 1;
													if (gl_usart_comm_ctx.queue_end_rx-1<gl_usart_comm_ctx.queue_ptr_rd)
														gl_usart_comm_ctx.queue_ptr_rd = gl_usart_comm_ctx.queue_end_rx-1;
													 gl_usart_comm_ctx.state_next_tx = STA_NULL;
													BACK_TO_NONE_RX // bad link
												}
												break;
									default: assert(0); // can't happen
								}
						}
						break;
			default : assert(0);
		}
#endif
		// return;
	}
	if ( sr & US_CSR_TXRDY) {
		// Data send
#if false  // not used for usb cdc app...
		if (udi_cdc_is_rx_ready()) {
			// Transmit next data
			ui_com_rx_start();
			int c = udi_cdc_getc();
			usart_write(USART_BASE, c);

		} else {
			// Fifo empty then Stop UART transmission
			usart_disable_tx(USART_BASE);
			usart_disable_interrupt(USART_BASE, US_IDR_TXRDY);
			ui_com_rx_stop();
		}
#else
		switch(gl_usart_comm_ctx.state_tx) {
			case STA_NULL:
						if (ACK_BACK == gl_usart_comm_ctx.state_next_rx) {
send_ack_1st_char:
							gl_usart_comm_ctx.chr_cnt_tx = USART_ACK_LEN-1;
							gl_usart_comm_ctx.state_tx = QUEUE_UP;
							usart_write(USART_BASE, 'A');
						}
						else if (gl_usart_comm_ctx.queue_start_rx<gl_usart_comm_ctx.queue_ptr_rd) {
							gl_usart_comm_ctx.mavlk_frm_ptr_tx= (uint8_t*)gl_usart_comm_ctx.queue_ptr_rd;
							*(gl_usart_comm_ctx.mavlk_frm_ptr_tx+1) = sizeof(usart_packet_rx)-START_SIGN_LEN; // assign payload length
							*(gl_usart_comm_ctx.mavlk_frm_ptr_tx+2) = gl_usart_comm_ctx.frm_squ_rx++; // assign sequence cnt
							gl_usart_comm_ctx.chr_cnt_tx=sizeof(usart_packet_rx)-1;
							gl_usart_comm_ctx.state_tx = QUEUE_UP;
							gl_usart_comm_ctx.cmd_tx= SND_FRAME;
							gl_usart_comm_ctx.queue_ptr_rd -= 1;
							usart_write(USART_BASE, *gl_usart_comm_ctx.mavlk_frm_ptr_tx++);
						}
						break;
			case QUEUE_UP:
						if (gl_usart_comm_ctx.cmd_tx== SND_FRAME) {
							if (gl_usart_comm_ctx.chr_cnt_tx--) {
								//usart_enable_interrupt(USART_BASE, US_IER_TXRDY); // for some reason we lost it in midway, re-enable intr
								if ( CHKSUM_LEN<=gl_usart_comm_ctx.chr_cnt_tx)
									crc_accumulate(*gl_usart_comm_ctx.mavlk_frm_ptr_tx,
																		&gl_usart_comm_ctx.mavlk_chksum_tx);
								else {
									if (gl_usart_comm_ctx.chr_cnt_tx) // low byte
								  		*(gl_usart_comm_ctx.mavlk_frm_ptr_tx) =
								  			(char)gl_usart_comm_ctx.mavlk_chksum_tx;
								  	else {// high byte
								  		*(gl_usart_comm_ctx.mavlk_frm_ptr_tx) =
								  			(char)(gl_usart_comm_ctx.mavlk_chksum_tx>>8);
								  			// to prevent ack byte from being returned too fast from the other end...
								  			usart_write(USART_BASE, *gl_usart_comm_ctx.mavlk_frm_ptr_tx++);
								  			goto last_byte_sent;
							  			}
								}
								usart_write(USART_BASE, *gl_usart_comm_ctx.mavlk_frm_ptr_tx++);
							}
							else { // a mavlink frame sent
last_byte_sent:
								usart_disable_tx(USART_BASE);
								usart_disable_interrupt(USART_BASE, US_IDR_TXRDY); // turn off Tx pipe, liyenho
								gl_usart_comm_ctx.state_next_tx = ACK_BACK;
								BACK_TO_NONE_TX
							}
						}
						else if (ACK_BACK == gl_usart_comm_ctx.state_next_rx) {
							if (!gl_usart_comm_ctx.chr_cnt_tx) {
								usart_disable_tx(USART_BASE);
								usart_disable_interrupt(USART_BASE, US_IDR_TXRDY); // turn off Tx pipe, liyenho
								gl_usart_comm_ctx.state_next_rx = STA_NULL;
								if (uart_port_start0)
									uart_port_start = true;
								BACK_TO_NONE_TX
							}
							else
								switch(gl_usart_comm_ctx.chr_cnt_tx--) {
									case USART_ACK_LEN-1:
												usart_write(USART_BASE, 'T');
												break;
									case USART_ACK_LEN-2:
												usart_write(USART_BASE, 'K');
												break;
									default: assert(0); // can't happen
								}
						}
						break;
			default : assert(0);
		}
#endif
		//return;
	}
	cur_time= *DWT_CYCCNT;
	tm_delta(prev_time, cur_time, tdel);
	if (tdelx < tdel)
		tdelx = tdel;
	if (prev_time_intr) {
		tm_delta(prev_time_intr, prev_time, tdel);
		// anything greater 5 sec is not of concern...
		if (5*120000000 > tdel && tdelx_intr < tdel)
			tdelx_intr = tdel;
	} // measure turnaround time
	prev_time_intr = prev_time;
}


void ctrl_buffer_send_ur(void* pctl) {
#ifdef DBG_UART_SND
	return ; // for debug uart recv on atmel
#endif
	if (!uart_port_start) return; // host isn't up yet, liyenho
	irqflags_t flags;
	if (gl_usart_comm_ctx.queue_end_rx-1>gl_usart_comm_ctx.queue_ptr_rd) {
		uint8_t *pr = (uint8_t*)(gl_usart_comm_ctx.queue_ptr_rd+1);
		memcpy(pr+MAVLINK_HDR_LEN,
							(uint8_t*)pctl, RADIO_GRPPKT_LEN);
#ifdef UART_TEST
		*(pr+MAVLINK_HDR_LEN-1) = 0x0; // data assumed
#else
		*(pr+MAVLINK_HDR_LEN-1) = /*Bill, can you implemented this per fs_sreamflow spec?t*/;
#endif
		flags = cpu_irq_save();
			gl_usart_comm_ctx.queue_ptr_rd += 1;
		cpu_irq_restore(flags);
	} // otherwise, host retreives too slow resulted buffer overflow
//	uint32_t tdel, cur_time = *DWT_CYCCNT;
	if (gl_usart_comm_ctx.state_tx == STA_NULL &&
			gl_usart_comm_ctx.state_next_tx == STA_NULL) {
		if (!(US_IMR_TXRDY & usart_get_interrupt_mask(USART_BASE))) {
			usart_enable_tx(USART_BASE);
			usart_enable_interrupt(USART_BASE, US_IER_TXRDY); // turn on Tx pipe, liyenho
		}
//		prev_time = cur_time;
	}
	/*else {
		if (!prev_time ) {
			assert(0); // shouldn't happen...
		}
		else {
			tm_delta(prev_time, cur_time, tdel);
			if (12000000 < tdel) { // 0.1 sec
		      // force giving up if takes too long
				gl_usart_comm_ctx.state_next_tx = STA_NULL;
				BACK_TO_NONE_TX
				prev_time = cur_time;
			}
		}
	}*/
}

void ctrl_buffer_recv_ur(void *pctl1) {
	irqflags_t flags;
	if (gl_usart_comm_ctx.queue_start_tx<gl_usart_comm_ctx.queue_ptr_wr) {
		flags = cpu_irq_save();
			uint8_t *pw = (uint8_t*)(gl_usart_comm_ctx.queue_ptr_wr), len ;
		cpu_irq_restore(flags);
		len = *(pw+1)-(MAVLINK_HDR_LEN+CHKSUM_LEN-1);
		memcpy((uint8_t*)pctl1,
							pw+MAVLINK_HDR_LEN, len);
		flags = cpu_irq_save();
			gl_usart_comm_ctx.queue_ptr_wr -= 1;
		cpu_irq_restore(flags);
	} // otherwise, host forwards too slow resulted buffer underflow
}

void uart_rx_notify(uint8_t port)
{
	UNUSED(port);
	// If UART is open
	if (usart_get_interrupt_mask(USART_BASE)
		& US_IMR_RXRDY) {
		// Enable UART TX interrupt to send a new value
		//usart_enable_tx(USART_BASE); //to avoid usb cdc rx get interferenced, liyenho
		// disable TXRDY to avoid usb cdc rx get interferenced, liyenho
		//usart_enable_interrupt(USART_BASE, US_IER_TXRDY);
	}
}


void uart_config(uint8_t port, usb_cdc_line_coding_t * cfg)
{
	uint32_t stopbits, parity, databits;
	uint32_t imr;
	UNUSED(port);

	switch (cfg->bCharFormat) {
	case CDC_STOP_BITS_2:
		stopbits = US_MR_NBSTOP_2_BIT;
		break;
	case CDC_STOP_BITS_1_5:
		stopbits = US_MR_NBSTOP_1_5_BIT;
		break;
	case CDC_STOP_BITS_1:
	default:
		// Default stop bit = 1 stop bit
		stopbits = US_MR_NBSTOP_1_BIT;
		break;
	}

	switch (cfg->bParityType) {
	case CDC_PAR_EVEN:
		parity = US_MR_PAR_EVEN;
		break;
	case CDC_PAR_ODD:
		parity = US_MR_PAR_ODD;
		break;
	case CDC_PAR_MARK:
		parity = US_MR_PAR_MARK;
		break;
	case CDC_PAR_SPACE:
		parity = US_MR_PAR_SPACE;
		break;
	default:
	case CDC_PAR_NONE:
		parity = US_MR_PAR_NO;
		break;
	}

	switch(cfg->bDataBits) {
	case 5: case 6: case 7:
		databits = cfg->bDataBits - 5;
		break;
	default:
	case 8:
		databits = US_MR_CHRL_8_BIT;
		break;
	}

	// Options for USART.
	usart_options.baudrate = LE32_TO_CPU(cfg->dwDTERate);
	usart_options.char_length = databits;
	usart_options.parity_type = parity;
	usart_options.stop_bits = stopbits;
																		// rx signal is connected back to tx pin, tx module is inactive but rx is active..., liyenho
	usart_options.channel_mode = US_MR_CHMODE_NORMAL/*US_MR_CHMODE_AUTOMATIC*/;
	imr = usart_get_interrupt_mask(USART_BASE);
	usart_disable_interrupt(USART_BASE, 0xFFFFFFFF);
	usart_init_rs232(USART_BASE, &usart_options,
			sysclk_get_peripheral_bus_hz(USART_BASE));
	// Restore both RX but disable TX to avoid usb cdc rx get interferenced, liyenho
	usart_enable_tx(USART_BASE);
	usart_enable_rx(USART_BASE);
	usart_enable_interrupt(USART_BASE, imr);
}

void uart_open(uint8_t port)
{
	UNUSED(port);

	for (int n=0; n<USART_PKT_QUEUE_LEN; n++) {
		memcpy((void*)(usart_queue_recv+n),
							mavlk_hdr_rx, MAVLINK_HDR_LEN);
	}
	gl_usart_comm_ctx.queue_end_rx = usart_queue_recv+USART_PKT_QUEUE_LEN;
	gl_usart_comm_ctx.queue_start_rx = usart_queue_recv;
	gl_usart_comm_ctx.queue_ptr_rd = usart_queue_recv;
	gl_usart_comm_ctx.frm_squ_rx = 0;
	gl_usart_comm_ctx.queue_end_tx = usart_queue_sent+USART_PKT_QUEUE_LEN;
	gl_usart_comm_ctx.queue_start_tx = usart_queue_sent;
	gl_usart_comm_ctx.queue_ptr_wr = usart_queue_sent;
	gl_usart_comm_ctx.state_rx = gl_usart_comm_ctx.state_next_rx = STA_NULL;
	gl_usart_comm_ctx.state_tx = gl_usart_comm_ctx.state_next_tx = STA_NULL;
	gl_usart_comm_ctx.cmd_extr = gl_usart_comm_ctx.cmd_tx = CMD_NONE;
#ifdef UART_TEST
	gl_usart_comm_ctx.chr_cnt_rx = 3; // wait for at cmd to init/start video sub-system
#else
	gl_usart_comm_ctx.chr_cnt_rx = START_SIGN_LEN;
#endif
	gl_usart_comm_ctx.chr_cnt_tx = 0;
	//gl_usart_comm_ctx.mavlk_frm_sz=(uint16_t)-1; // invalidated
	gl_usart_comm_ctx.mavlk_frm_ptr_rx = 0;
	gl_usart_comm_ctx.mavlk_chksum_rx =
	gl_usart_comm_ctx.mavlk_chksum_tx = X25_INIT_CRC;

	// IO is initialized in board init
	// Enable interrupt with priority higher than USB
	NVIC_SetPriority(USART_INT_IRQn, USART_INT_LEVEL);
	NVIC_EnableIRQ(USART_INT_IRQn);

	// Initialize it in RS232 mode.
	USART_PERIPH_CLK_ENABLE();
	if (usart_init_rs232(USART_BASE, &usart_options,
			sysclk_get_peripheral_bus_hz(USART_BASE))) {
		return;
	}
	// Enable USART
	USART_ENABLE();

	// Enable both RX but disable TX to avoid usb cdc rx get interferenced, liyenho
	usart_enable_tx(USART_BASE);
	usart_enable_rx(USART_BASE);
	// Enable interrupts, disable TXRDY to avoid usb cdc rx get interferenced, liyenho
	usart_enable_interrupt(USART_BASE, US_IER_RXRDY /*| US_IER_TXRDY*/);

}

void uart_close(uint8_t port)
{
	UNUSED(port);
	// Disable interrupts
	usart_disable_interrupt(USART_BASE, 0xFFFFFFFF);
	// Close RS232 communication
	USART_DISABLE();
}
