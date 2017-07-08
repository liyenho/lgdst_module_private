/**
 * \file
 *
 * \brief Board configuration.
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
#define CONF_BOARD_EVM	// turn on rec on evm board, liyenho
//#define TEST_USB
//#define TEST_SPI
//#define CONF_BOARD_I2S_TX	// designate this board to be i2s transmitter, liyenho
//#define CONF_BOARD_I2S_RX	// designate this board to be i2s receiver, liyenho
#ifndef CONF_BOARD_H_INCLUDED
#define CONF_BOARD_H_INCLUDED

/** Enable Com Port. */
#define CONF_BOARD_UART_CONSOLE
#define CONF_BOARD_SPI
/** SPI slave select MACRO definition */
#define CONF_BOARD_SPI_NPCS0

/** SPI MACRO definition */
#ifdef CONF_BOARD_EVM  // kept original settings for rx
/** Spi Hw ID . */
  #define SPI_ID          ID_SPI5
/** SPI base address for SPI master mode*/
  #define SPI_MASTER_BASE      SPI5
/** SPI base address for SPI slave mode, (on different board) */
  #define SPI_SLAVE_BASE       SPI5
/** FLEXCOM base address for SPI mode*/
  #define BOARD_FLEXCOM_SPI    FLEXCOM5
#else  // alternate settings on tx
/** Spi Hw ID . */
  #define SPI_ID          ID_SPI1
/** SPI base address for SPI master mode*/
  #define SPI_MASTER_BASE      SPI1
/** SPI base address for SPI slave mode, (on different board) */
  #define SPI_SLAVE_BASE       SPI1
/** FLEXCOM base address for SPI mode*/
  #define BOARD_FLEXCOM_SPI    FLEXCOM1
#endif  //CONF_BOARD_EVM
/** SPI base address for SPI slave mode, (on different board) */
  #define SPI0_SLAVE_BASE       SPI0

/** Spi Hw ID . */
#define SPI0_ID          ID_SPI0
/** SPI base address for SPI master mode*/
#define SPI0_MASTER_BASE      SPI0
//#define SPI3_MASTER_BASE      SPI3
#define SPI7_MASTER_BASE      SPI7
/** FLEXCOM base address for SPI mode*/
#define BOARD_FLEXCOM_SPI0    FLEXCOM0
//#define BOARD_FLEXCOM_SPI3    FLEXCOM3
#define BOARD_FLEXCOM_SPI7    FLEXCOM7

/** Definition of TWI interrupt ID on board. */
#define BOARD_TWI_IRQn          TWI4_IRQn
#define BOARD_TWI_Handler    TWI4_Handler

/** Configure TWI4 pins */
#define CONF_BOARD_TWI4

/** Flexcom application to use */
#define BOARD_FLEXCOM_TWI          FLEXCOM4
#ifdef CONF_BOARD_EVM
  #define CONF_BOARD_I2S0	// enable I2S0, added by liyenho
#endif
/** I2S Hw ID . */
#define I2SC0_ID          ID_I2SC0
/** I2S base address for I2S tx/rx mode*/
#define I2SC0_BASE      I2SC0

#endif /* CONF_BOARD_H_INCLUDED */
