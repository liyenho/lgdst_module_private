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

#if SAM4L
#   define USART_PERIPH_CLK_ENABLE() sysclk_enable_peripheral_clock(USART_BASE)
#elif SAMG55
#   define USART_PERIPH_CLK_ENABLE() flexcom_enable(BOARD_FLEXCOM);      \
	                                                        flexcom_set_opmode(BOARD_FLEXCOM, FLEXCOM_USART);
#else
#   define USART_PERIPH_CLK_ENABLE() sysclk_enable_peripheral_clock(USART_ID)
#endif

extern volatile uint32_t *DWT_CYCCNT;

static sam_usart_opt_t usart_options;
volatile static uint8_t rec_intr_buffer[MAX_USART_PKT_LEN];

volatile ctx_usart gl_usart_comm_ctx ; // global interrupt context for serial comm
			// point of view from control link
static usart_packet_rx usart_queue_recv[USART_PKT_QUEUE_LEN];
  const uint8_t mavlk_hdr_rx[MAVLINK_HDR_LEN] = {
	  																		0x55, // start sign
  																			RADIO_GRPPKT_LEN+MAVLINK_HDR_LEN,
  																			0x0, // sequence, to be filled in
  																			0x01, // system id
  																			0x14, // component id
  																			0x00, // to be filled by Bill
  																		};
static usart_packet_tx usart_queue_sent[USART_PKT_QUEUE_LEN];

#define BACK_TO_NONE \
	gl_usart_comm_ctx.chr_cnt = USART_AT_CMD_LEN; \
	gl_usart_comm_ctx.state = STA_NULL; \
	gl_usart_comm_ctx.cmd = CMD_NONE;

ISR(USART_HANDLER)  // we need this ISR to pump status data to usb cdc comm, liyenho
{
	uint32_t sr = usart_get_status(USART_BASE);
	uint32_t tdel, atcmd=0, cur_time = *DWT_CYCCNT;
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
			gl_usart_comm_ctx.last_rec_tm = 0U;
			BACK_TO_NONE
			return;
		}
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
		switch(gl_usart_comm_ctx.state) {
			case STA_NULL:
						if ('A' == (char)value) {
												gl_usart_comm_ctx.this_rec_tm = cur_time;
												gl_usart_comm_ctx.last_at_cmd[gl_usart_comm_ctx.chr_cnt--]= (char)value;
												gl_usart_comm_ctx.state = SYNC_UP;
											}
							else goto again;
						break;
			case SYNC_UP:
						switch (gl_usart_comm_ctx.chr_cnt) {
							case USART_AT_CMD_LEN-1:
											if ('T' == (char)value) {
												gl_usart_comm_ctx.last_at_cmd[gl_usart_comm_ctx.chr_cnt--]= (char)value;
											}
											else goto again;
											break;
							case USART_AT_CMD_LEN-2:
											if ('0' <= (char)value
													&& '9' >= (char)value) {
												gl_usart_comm_ctx.last_at_cmd[gl_usart_comm_ctx.chr_cnt--]= (char)value;
											}
											else goto again;
											break;
							default : goto again;
						}
						tm_delta(gl_usart_comm_ctx.last_rec_tm,
											gl_usart_comm_ctx.this_rec_tm,
											tdel)
						if (USART_WAIT_TIME < tdel || !gl_usart_comm_ctx.last_rec_tm) {
							if (!gl_usart_comm_ctx.chr_cnt) {
								gl_usart_comm_ctx.chr_cnt =USART_AT_CMD_LEN-1;
								gl_usart_comm_ctx.state = ACK_BACK;
								usart_enable_interrupt(USART_BASE, US_IER_TXRDY); // turn on Tx pipe, liyenho
								while(usart_write(USART_BASE, 'A')) ;
							}}
						else {
again:				BACK_TO_NONE
						}
						break;
			case GET_LEN:
						switch (gl_usart_comm_ctx.chr_cnt) {
							case USART_PKT_LEN_LEN:
											gl_usart_comm_ctx.mavlk_frm_sz= 0xff&value;
											gl_usart_comm_ctx.chr_cnt -=1;
											break;
							case USART_PKT_LEN_LEN-1:
											gl_usart_comm_ctx.mavlk_frm_sz |= 0xff00&(value<<8);
											if (MAX_USART_PKT_LEN>=gl_usart_comm_ctx.mavlk_frm_sz &&
													MAVLINK_HDR_LEN < gl_usart_comm_ctx.mavlk_frm_sz) {
												// valid cmd sequence so far
												memcpy(&atcmd, gl_usart_comm_ctx.last_at_cmd, USART_AT_CMD_LEN);
												switch(atcmd) {
													case 0x335441:
														gl_usart_comm_ctx.cmd= REC_FRAME;
														gl_usart_comm_ctx.state = QUEUE_UP;
														gl_usart_comm_ctx.mavlk_frm_ptr= rec_intr_buffer;
														gl_usart_comm_ctx.chr_cnt=gl_usart_comm_ctx.mavlk_frm_sz;
														break;
													default : assert(0); // can't happen
												}
											}
											else {
												BACK_TO_NONE
											}
											break;
							default : assert(0); // can't happen
						}
						break;
			case QUEUE_UP:
						if (gl_usart_comm_ctx.cmd== REC_FRAME) {
							if (gl_usart_comm_ctx.chr_cnt--) {
								*gl_usart_comm_ctx.mavlk_frm_ptr++ = (char)value;
								if ( (2-1)<gl_usart_comm_ctx.chr_cnt && // excluded start sign
									gl_usart_comm_ctx.mavlk_frm_sz-1 != gl_usart_comm_ctx.chr_cnt )
									gl_usart_comm_ctx.mavlk_chksum += (short)value;
							}
							else { // check if a mavlink frame recv-ed
								uint16_t chksum;
								  chksum = *(gl_usart_comm_ctx.mavlk_frm_ptr-2);
								  chksum |= ((uint16_t)*(gl_usart_comm_ctx.mavlk_frm_ptr-1)<<8);
								if (chksum == gl_usart_comm_ctx.mavlk_chksum) {
									// it is a valid packet
									if (gl_usart_comm_ctx.queue_end_tx-1 >gl_usart_comm_ctx.queue_ptr_wr) {
										memcpy(gl_usart_comm_ctx.queue_ptr_wr,
															rec_intr_buffer/*+MAVLINK_HDR_LEN*/,
															gl_usart_comm_ctx.mavlk_frm_sz/*-MAVLINK_HDR_LEN*/);
										gl_usart_comm_ctx.queue_ptr_wr += 1;
									} // otherwise, host forwards too fast resulted buffer overflow
									gl_usart_comm_ctx.last_rec_tm = cur_time;
								}
								gl_usart_comm_ctx.mavlk_chksum = 0;
								gl_usart_comm_ctx.mavlk_frm_sz =0;
								BACK_TO_NONE
							}
						}
						break;
			default : assert(0);
		}
#endif
		return;
	}
	// disable TXRDY to avoid usb cdc rx get interferenced, liyenho
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
		switch(gl_usart_comm_ctx.state) {
			case ACK_BACK:
						switch (gl_usart_comm_ctx.chr_cnt) {
							case USART_AT_CMD_LEN-1:
											usart_write(USART_BASE, 'T');
											break;
							case USART_AT_CMD_LEN-2:
											usart_write(USART_BASE, 'A');
											break ;
							default : assert(0); // can't happen
						}
						gl_usart_comm_ctx.chr_cnt -= 1;
						if (!gl_usart_comm_ctx.chr_cnt) {
							memcpy(&atcmd, gl_usart_comm_ctx.last_at_cmd, USART_AT_CMD_LEN);
							switch(atcmd) {
								case 0x315441:
									gl_usart_comm_ctx.cmd = INIT_VIDEO;
									BACK_TO_NONE
									break;
								case 0x325441:
									gl_usart_comm_ctx.cmd = START_VIDEO;
									BACK_TO_NONE
									break;
								case 0x345441:
									// when ctrl link gets new radio packet, Bill shall copy into payload section of
									// gl_usart_comm_ctx.queue_ptr_rd, then bump up gl_usart_comm_ctx.queue_ptr_rd
									if (gl_usart_comm_ctx.queue_start_rx<gl_usart_comm_ctx.queue_ptr_rd) {
										gl_usart_comm_ctx.chr_cnt=sizeof(usart_packet_rx)-1;
										gl_usart_comm_ctx.state = QUEUE_UP;
										gl_usart_comm_ctx.queue_ptr_rd -= 1;
										usart_write(USART_BASE, *gl_usart_comm_ctx.mavlk_frm_ptr++);
									} // otherwise, host retreives too fast resulted buffer underflow
									else {
										BACK_TO_NONE
									}
									break;
								case 0x335441:
									gl_usart_comm_ctx.chr_cnt =USART_PKT_LEN_LEN;
									gl_usart_comm_ctx.state = GET_LEN;
									break;
								default:  // invalid or not implemented yet
									BACK_TO_NONE
									break;
							}
						}
						else if (1==gl_usart_comm_ctx.chr_cnt) {//one before last one
							memcpy(&atcmd, gl_usart_comm_ctx.last_at_cmd, USART_AT_CMD_LEN);
							if (0x345441 == atcmd &&
								gl_usart_comm_ctx.queue_start_rx<gl_usart_comm_ctx.queue_ptr_rd) {
								// do all these here in case it gets too tight at the last char of ATA ack
									gl_usart_comm_ctx.cmd= SND_FRAME;
									gl_usart_comm_ctx.mavlk_frm_ptr= (uint8_t*)gl_usart_comm_ctx.queue_ptr_rd;
									*(gl_usart_comm_ctx.mavlk_frm_ptr+1) = sizeof(usart_packet_rx); // assign payload length
									*(gl_usart_comm_ctx.mavlk_frm_ptr+2) = gl_usart_comm_ctx.frm_squ_rx++; // assign sequence cnt
							}
						}
						break;
			case QUEUE_UP:
						if (gl_usart_comm_ctx.cmd== SND_FRAME) {
							if (gl_usart_comm_ctx.chr_cnt--) {
								usart_write(USART_BASE, *gl_usart_comm_ctx.mavlk_frm_ptr++);
							}
							else { // a mavlink frame sent
								BACK_TO_NONE
								gl_usart_comm_ctx.last_rec_tm = cur_time;
							}
						}
						break;
			default : assert(0);
		}
#endif
	}
}


void ctrl_buffer_send_ur(void* pctl) {
	irqflags_t flags;
	if (gl_usart_comm_ctx.queue_end_rx-1>gl_usart_comm_ctx.queue_ptr_rd) {
		uint8_t *pr = (uint8_t*)(gl_usart_comm_ctx.queue_ptr_rd+1);
		memcpy(pr+MAVLINK_HDR_LEN,
							(uint8_t*)pctl, RADIO_GRPPKT_LEN);
#ifdef UART_TEST
		*(pr+MAVLINK_HDR_LEN-1) = 0x0; // data assumed
#else
		*(pr+MAVLINK_HDR_LEN-1) = /*Bill, can you implement this per fs_streamflow spec?*/;
#endif
		flags = cpu_irq_save();
			gl_usart_comm_ctx.queue_ptr_rd += 1;
		cpu_irq_restore(flags);
	} // otherwise, host retreives too slow resulted buffer overflow
}

void ctrl_buffer_recv_ur(void *pctl1) {
	irqflags_t flags;
	if (gl_usart_comm_ctx.queue_start_tx<gl_usart_comm_ctx.queue_ptr_wr) {
		flags = cpu_irq_save();
			uint8_t *pw = (uint8_t*)(gl_usart_comm_ctx.queue_ptr_wr),
								len= *(pw+1)-(MAVLINK_HDR_LEN-1);
		cpu_irq_restore(flags);
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
	gl_usart_comm_ctx.state = STA_NULL;
	gl_usart_comm_ctx.cmd = CMD_NONE;
	memset(gl_usart_comm_ctx.last_at_cmd, 0x0, USART_AT_CMD_LEN);
	gl_usart_comm_ctx.chr_cnt = USART_AT_CMD_LEN;
	gl_usart_comm_ctx.this_rec_tm = gl_usart_comm_ctx.last_rec_tm = 0; // invalidated
	gl_usart_comm_ctx.mavlk_frm_sz=(uint16_t)-1; // invalidated
	gl_usart_comm_ctx.mavlk_chksum = 0;

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
