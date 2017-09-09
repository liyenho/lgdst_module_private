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
#include "conf_example.h"
#include "uart.h"
#include "main.h"
#include "ui.h"
#if (SAMG55)
#include "flexcom.h"
#endif
#include "ctrl.h"  // for tm_delta() def
#include "delay.h"
#include "Control_Link/MavLink.h"
#include "Control_Link/Radio_Buffers.h"

#if SAM4L
#   define USART_PERIPH_CLK_ENABLE() sysclk_enable_peripheral_clock(USART_BASE)
#elif SAMG55
#   define USART_PERIPH_CLK_ENABLE() { flexcom_enable(BOARD_FLEXCOM);      \
	                                                        flexcom_set_opmode(BOARD_FLEXCOM, FLEXCOM_USART); }
 #ifdef CONF_BOARD_USB_RX
#   define USART7_PERIPH_CLK_ENABLE() { flexcom_enable(BOARD_FLEXCOM7);      \
	                                                        flexcom_set_opmode(BOARD_FLEXCOM7, FLEXCOM_USART); }
 #endif
#else
#   define USART_PERIPH_CLK_ENABLE() sysclk_enable_peripheral_clock(USART_ID)
#endif

extern volatile uint32_t *DWT_CYCCNT;
volatile static uint32_t prev_time_intr = 0,
												prev_time_snd = 0,
												tdelx_intr, tdelx;  // timing measure facility
//extern volatile bool init_video_flag, start_video_flag;

static sam_usart_opt_t usart_options;
//volatile static uint8_t rec_intr_buffer[MAX_USART_PKT_LEN];
static volatile bool uart_port_start0 = false,
										uart_port_start = false/*true*/,
										first_ignored = true;  // timing measure facility

//static volatile uint8_t dbg_chr_rx[1000] ;
static volatile uint32_t dbg_chr_cnt ;

uint32_t radio_mon_rxcnt = 0;
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


#define ALL_INTERRUPT_MASK  0xffffffff

/** PDC data packet. */
//pdc_packet_t g_st_packet;
 Pdc *g_p_pdc_UART, *g_p_pdc7_UART;
 static uint8_t uart_buffer1[1000];
 uint8_t uart_buffer2[1000];
#ifdef CONF_BOARD_USB_RX
 static uint8_t uart7_buffer1[1000];
 uint8_t uart7_buffer2[1000];
#endif
 pdc_packet_t g_p_uart_pdc_packet, g_p_uart7_pdc_packet;
 pdc_packet_t g_p_uart_pdc_packet2, g_p_uart7_pdc_packet2;
pdc_packet_t *next_location = &g_p_uart_pdc_packet;
pdc_packet_t *next7_location = &g_p_uart7_pdc_packet;

uint32_t other_irq_cnt=0, other7_irq_cnt=0;
uint32_t uart_overrun_cnt =0, uart7_overrun_cnt =0;

ISR(USART_HANDLER)  // we need this ISR to pump status data to usb cdc comm, liyenho
{
	ISR_USART_RADIO(USART_BASE,
														uart_overrun_cnt,
														uart_buffer1,
														uart_buffer2,
														next_location,
														g_p_uart_pdc_packet,
														g_p_uart_pdc_packet2,
														g_p_pdc_UART,
														other_irq_cnt)
#if defined(RADIO_CTRL_TXCONTINOUS)
	if ((sr & US_CSR_ENDTX)== US_CSR_ENDTX){
		if (!USE_915MHZ){
			uart_Send_Data(gp_rdo_tpacket_l, RADIO_LONG_PKT_LEN);
		extern unsigned char snd_asymm_cnt;
			snd_asymm_cnt = 0;
		}else{
			uart_Send_Data(gp_rdo_tpacket_l, RADIO_PKT_LEN);
		}
	}
#endif
}

#ifdef CONF_BOARD_USB_RX
	ISR(USART7_HANDLER)
	{
		ISR_USART_DRONE(USART7_BASE,
															uart7_overrun_cnt,
															uart7_buffer1,
															uart7_buffer2,
															next7_location,
															g_p_uart7_pdc_packet,
															g_p_uart7_pdc_packet2,
															g_p_pdc7_UART,
															other7_irq_cnt)
	}
#endif

#if false
void ctrl_buffer_send_ur(uint8_t port,void* pctl) {
#ifdef DBG_UART_SND
	return ; // for debug uart recv on atmel
#endif
	if (!uart_port_start) return; // host isn't up yet, liyenho
	irqflags_t flags;
	if (gl_usart_comm_ctx.queue_end_rx-1>gl_usart_comm_ctx.queue_ptr_rd) {
		uint8_t *pr = (uint8_t*)(gl_usart_comm_ctx.queue_ptr_rd+1);
		memcpy(pr+MAVLINK_HDR_LEN,
							(uint8_t*)pctl, RADIO_GRPPKT_LEN);
										/*Bill, can you implemented this per fs_streamflow spec?t*/
		*(pr+MAVLINK_HDR_LEN-1) = 0x0; // data assumed
		flags = cpu_irq_save();
			gl_usart_comm_ctx.queue_ptr_rd += 1;
		cpu_irq_restore(flags);
	} // otherwise, host retreives too slow resulted buffer overflow
//	uint32_t tdel, cur_time = *DWT_CYCCNT;
	if (gl_usart_comm_ctx.state_tx == STA_NULL &&
			gl_usart_comm_ctx.state_next_tx == STA_NULL) {
		if (!(US_IMR_TXRDY &
#ifdef CONF_BOARD_USB_RX
				usart_get_interrupt_mask((0!=port)?USART7_BASE:USART_BASE))) {
#else //CONF_BOARD_USB_TX
				usart_get_interrupt_mask(USART_BASE))) {
#endif
#ifdef CONF_BOARD_USB_RX
			usart_enable_tx((0!=port)?USART7_BASE:USART_BASE);
			usart_enable_interrupt((0!=port)?USART7_BASE:USART_BASE, US_IER_TXRDY); // turn on Tx pipe, liyenho
#else //CONF_BOARD_USB_TX
			usart_enable_tx(USART_BASE);
			usart_enable_interrupt(USART_BASE, US_IER_TXRDY); // turn on Tx pipe, liyenho
#endif
		}
//		prev_time = cur_time;
	}
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
#endif

void uart_rx_notify(uint8_t port)
{
	//UNUSED(port);
	// If UART is open
#ifdef CONF_BOARD_USB_RX
	if (usart_get_interrupt_mask((0!=port)?USART7_BASE:USART_BASE)
#else //CONF_BOARD_USB_TX
	if (usart_get_interrupt_mask(USART_BASE)
#endif
		& US_IMR_RXRDY) {
		// Enable UART TX interrupt to send a new value
		//usart_enable_tx(
		//	(0!=port)?USART7_BASE:USART_BASE); //to avoid usb cdc rx get interferenced, liyenho
		// disable TXRDY to avoid usb cdc rx get interferenced, liyenho
		//usart_enable_interrupt(
		//	(0!=port)?USART7_BASE:USART_BASE, US_IER_TXRDY);
	}
}

void Configure_UART_DMA(uint8_t port){
		sam_usart_opt_t usart_console_settings = {
			0,
			US_MR_CHRL_8_BIT,
			US_MR_PAR_NO,
			US_MR_NBSTOP_1_BIT,
			US_MR_CHMODE_NORMAL,
			/* This field is only used in IrDA mode. */
			0
		};

		usart_console_settings.baudrate = CONF_UART_BAUDRATE;

		#if (SAMG55)
		/* Enable the peripheral and set USART mode. */
		if (0==port) { // ctrl radio
			flexcom_enable(BOARD_FLEXCOM);
			flexcom_set_opmode(BOARD_FLEXCOM, FLEXCOM_USART);
		}
#ifdef CONF_BOARD_USB_RX
		else { // drone comm
			flexcom_enable(BOARD_FLEXCOM7);
			flexcom_set_opmode(BOARD_FLEXCOM7, FLEXCOM_USART);
		}
#endif
		#else
		/* Enable the peripheral clock in the PMC. */
		sysclk_enable_peripheral_clock(BOARD_ID_USART);
		#endif

		/* Configure USART in SYNC. master or slave mode. */
		/*if (ul_ismaster) {
			usart_init_sync_master(
				(0!=port)?USART7_BASE:USART_BASE,
				&usart_console_settings,
				sysclk_get_cpu_hz());
		} else {
			usart_init_sync_slave(
				(0!=port)?USART7_BASE:USART_BASE,
				&usart_console_settings);
		}*/
#if false	// unless stdio is necessary
  #ifdef CONF_BOARD_USB_RX
		stdio_serial_init((0!=port)?USART7_BASE:USART_BASE,&usart_console_settings);
	#else //CONF_BOARD_USB_TX
		stdio_serial_init(USART_BASE,&usart_console_settings);
	#endif
#endif
		/* Disable all the interrupts. */
#ifdef CONF_BOARD_USB_RX
		usart_disable_interrupt((0!=port)?USART7_BASE:USART_BASE,ALL_INTERRUPT_MASK);

		usart_init_rs232((0!=port)?USART7_BASE:USART_BASE,&usart_console_settings,
			sysclk_get_peripheral_bus_hz((0!=port)?USART7_BASE:USART_BASE));

		/* Enable TX & RX function. */
		usart_enable_tx((0!=port)?USART7_BASE:USART_BASE);
		usart_enable_rx((0!=port)?USART7_BASE:USART_BASE);

		/* Configure and enable interrupt of USART. */
		NVIC_SetPriority((0!=port)?USART7_INT_IRQn:USART_INT_IRQn,0) ;
		NVIC_EnableIRQ((0!=port)?USART7_INT_IRQn:USART_INT_IRQn);
#ifdef RADIO_CTRL_TXCONTINOUS
		usart_enable_interrupt(USART7_BASE,US_IER_RXRDY);
		usart_enable_interrupt(USART_BASE,US_IER_RXRDY | US_IER_TXRDY);
#else
		usart_enable_interrupt((0!=port)?USART7_BASE:USART_BASE,
			US_IER_RXRDY /*| US_IER_TXRDY*/);
#endif
#else  //CONF_BOARD_USB_TX
		usart_disable_interrupt(USART_BASE,ALL_INTERRUPT_MASK);

		usart_init_rs232(USART_BASE,&usart_console_settings,
			sysclk_get_peripheral_bus_hz(USART_BASE));

		/* Enable TX & RX function. */
		usart_enable_tx(USART_BASE);
		usart_enable_rx(USART_BASE);

		/* Configure and enable interrupt of USART. */
		NVIC_SetPriority(USART_INT_IRQn,0) ;
		NVIC_EnableIRQ(USART_INT_IRQn);

		usart_enable_interrupt(USART_BASE,
			US_IER_RXRDY /*| US_IER_TXRDY*/);
#endif
		/* Get board USART PDC base address and enable receiver and transmitter. */
#ifdef CONF_BOARD_USB_RX
		if (port /*drone comm*/) {
			g_p_pdc7_UART = usart_get_pdc_base(USART7_BASE);

			g_p_uart7_pdc_packet.ul_size= UART_BUFFER_SIZE;
			g_p_uart7_pdc_packet.ul_addr = uart_buffer1;

			g_p_uart7_pdc_packet2.ul_size= UART_BUFFER_SIZE;
			g_p_uart7_pdc_packet2.ul_addr = uart_buffer2;

			pdc_rx_init(g_p_pdc7_UART, &g_p_uart7_pdc_packet, &g_p_uart7_pdc_packet2);
																																				// tx complete is not processed, why turn it on? liyenho
			pdc_enable_transfer(g_p_pdc7_UART, PERIPH_PTCR_RXTEN /*| PERIPH_PTCR_TXTEN*/);
		}
		else
#endif
		{ // ctrl link
			g_p_pdc_UART = usart_get_pdc_base(USART_BASE);

			g_p_uart_pdc_packet.ul_size= UART_BUFFER_SIZE;
			g_p_uart_pdc_packet.ul_addr = uart_buffer1;

			g_p_uart_pdc_packet2.ul_size= UART_BUFFER_SIZE;
			g_p_uart_pdc_packet2.ul_addr = uart_buffer2;

			pdc_rx_init(g_p_pdc_UART, &g_p_uart_pdc_packet, &g_p_uart_pdc_packet2);
																																				// tx complete is not processed, why turn it on? liyenho
			pdc_enable_transfer(g_p_pdc_UART, PERIPH_PTCR_RXTEN /*| PERIPH_PTCR_TXTEN*/);
		}
}

void uart_config(uint8_t port, usb_cdc_line_coding_t * cfg)
{
	uint32_t stopbits, parity, databits;
	uint32_t imr;
	//UNUSED(port);

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
	usart_options.channel_mode = /*US_MR_CHMODE_NORMAL*/US_MR_CHMODE_AUTOMATIC;
#ifdef CONF_BOARD_USB_RX
	imr = usart_get_interrupt_mask((0!=port)?USART7_BASE:USART_BASE);
	usart_disable_interrupt((0!=port)?USART7_BASE:USART_BASE, 0xFFFFFFFF);
	usart_init_rs232((0!=port)?USART7_BASE:USART_BASE,
			&usart_options, sysclk_get_peripheral_bus_hz((0!=port)?USART7_BASE:USART_BASE));
	// Restore both RX but disable TX to avoid usb cdc rx get interferenced, liyenho
	//usart_enable_tx(USART_BASE);
	usart_enable_rx((0!=port)?USART7_BASE:USART_BASE);
	usart_enable_interrupt((0!=port)?USART7_BASE:USART_BASE, imr);
#else //CONF_BOARD_USB_TX
	imr = usart_get_interrupt_mask(USART_BASE);
	usart_disable_interrupt(USART_BASE, 0xFFFFFFFF);
	usart_init_rs232(USART_BASE,
			&usart_options, sysclk_get_peripheral_bus_hz(USART_BASE));
	// Restore both RX but disable TX to avoid usb cdc rx get interferenced, liyenho
	//usart_enable_tx(USART_BASE);
	usart_enable_rx(USART_BASE);
	usart_enable_interrupt(USART_BASE, imr);
#endif
}

void uart_open(uint8_t port)
{
	//UNUSED(port);

	// IO is initialized in board init
	// Enable interrupt with priority higher than USB
#ifdef CONF_BOARD_USB_RX
	NVIC_SetPriority((0!=port)?USART7_INT_IRQn:USART_INT_IRQn,
			(0!=port)?USART7_INT_LEVEL:USART_INT_LEVEL);
	NVIC_EnableIRQ((0!=port)?USART7_INT_IRQn:USART_INT_IRQn);
#else  //CONF_BOARD_USB_TX
	NVIC_SetPriority(USART_INT_IRQn,USART_INT_LEVEL);
	NVIC_EnableIRQ(USART_INT_IRQn);
#endif
	// Initialize it in RS232 mode.
#ifdef CONF_BOARD_USB_RX
	if (port) // drone comm link
		USART7_PERIPH_CLK_ENABLE() //;
	else  // ctrl radio link
#endif
		USART_PERIPH_CLK_ENABLE() //;
#ifdef CONF_BOARD_USB_RX
	if (usart_init_rs232((0!=port)?USART7_BASE:USART_BASE,
			&usart_options, sysclk_get_peripheral_bus_hz((0!=port)?USART7_BASE:USART_BASE))) {
		return;
	}
#else // CONF_BOARD_USB_TX
	if (usart_init_rs232(USART_BASE,&usart_options, sysclk_get_peripheral_bus_hz(USART_BASE))) {
		return;
	}
#endif
	// Enable USART
	USART_ENABLE();

	// Enable both RX but disable TX to avoid usb cdc rx get interferenced, liyenho
	//usart_enable_tx((0!=port)?USART7_BASE:USART_BASE);
#ifdef CONF_BOARD_USB_RX
	usart_enable_rx((0!=port)?USART7_BASE:USART_BASE);
	// Enable interrupts, disable TXRDY to avoid usb cdc rx get interferenced, liyenho
	usart_enable_interrupt((0!=port)?USART7_BASE:USART_BASE,
			US_IER_RXRDY /*| US_IER_TXRDY*/);
#else  //CONF_BOARD_USB_TX
	usart_enable_rx(USART_BASE);
	usart_enable_interrupt(USART_BASE,
			US_IER_RXRDY /*| US_IER_TXRDY*/);
#endif
}

void uart_close(uint8_t port)
{
	//UNUSED(port);
	// Disable interrupts
#ifdef CONF_BOARD_USB_RX
	usart_disable_interrupt((0!=port)?USART7_BASE:USART_BASE, 0xFFFFFFFF);
#else  //CONF_BOARD_USB_TX
	usart_disable_interrupt(USART_BASE, 0xFFFFFFFF);
#endif
	// Close RS232 communication
	USART_DISABLE();
}

static pdc_packet_t g_st_packet2;
static char outgoing_buff[1000] = "Buffer to send";
void uart_send_message(char* msg){
	memcpy(outgoing_buff, msg, strlen(msg));
	g_st_packet2.ul_addr = (uint32_t)outgoing_buff;
	g_st_packet2.ul_size = strlen(msg);
	pdc_tx_init(g_p_pdc_UART, &g_st_packet2, NULL);
}

void uart_send_Mavlink(MavLinkPacket pkt){
	memset(outgoing_buff, 0x0, sizeof(outgoing_buff));
	memcpy(outgoing_buff, &pkt, pkt.length+MAVLINK_HDR_LEN);
	memcpy(outgoing_buff+pkt.length+MAVLINK_HDR_LEN, pkt.checksum, MAVLINK_CHKSUM_LEN);
	g_st_packet2.ul_addr = (uint32_t)outgoing_buff;
	g_st_packet2.ul_size = MavLink_Total_Bytes_Used(pkt);
	pdc_tx_init(g_p_pdc_UART, &g_st_packet2, NULL);
}

void uart_Send_Data(uint8_t *data, uint32_t bytes){
	memcpy(outgoing_buff, data, bytes);
	g_st_packet2.ul_addr = (uint32_t)outgoing_buff;
	g_st_packet2.ul_size =bytes;
	pdc_tx_init(g_p_pdc_UART, &g_st_packet2, NULL);

}
#ifdef CONF_BOARD_USB_RX
	static pdc_packet_t g_st_packet7;
	static char outgoing_buff7[1000] = "Buffer to send";
	void uart7_send_message(char* msg){
		memcpy(outgoing_buff7, msg, strlen(msg));
		g_st_packet7.ul_addr = (uint32_t)outgoing_buff7;
		g_st_packet7.ul_size = strlen(msg);
		pdc_tx_init(g_p_pdc7_UART, &g_st_packet7, NULL);
	}

	void uart7_send_Mavlink(MavLinkPacket pkt){
		memset(outgoing_buff7, 0x0, sizeof(outgoing_buff7));
		memcpy(outgoing_buff7, &pkt, pkt.length+MAVLINK_HDR_LEN);
		memcpy(outgoing_buff7+pkt.length+MAVLINK_HDR_LEN, pkt.checksum, MAVLINK_CHKSUM_LEN);
		g_st_packet7.ul_addr = (uint32_t)outgoing_buff7;
		g_st_packet7.ul_size = MavLink_Total_Bytes_Used(pkt);
		pdc_tx_init(g_p_pdc7_UART, &g_st_packet7, NULL);
	}

	void uart7_Send_Data(uint8_t *data, uint32_t bytes){
		memcpy(outgoing_buff7, data, bytes);
		g_st_packet7.ul_addr = (uint32_t)outgoing_buff7;
		g_st_packet7.ul_size =bytes;
		pdc_tx_init(g_p_pdc7_UART, &g_st_packet7, NULL);

	}
#endif