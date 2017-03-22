/**********************  DRIVER FOR SPI CONTROLLER ON ORION**********************

   Filename:     Serialize.h
   Description:  Header file of Serialize.c
   Version:      0.1


   THE FOLLOWING DRIVER HAS BEEN ADOPTED ON ATMEL SPI INTERFACE

********************************************************************************

   Version History.
   Ver.   Date      Comments

   0.2   Dec 2011  Alpha version

*******************************************************************************/

#ifndef _SERIALIZE_H_
#define _SERIALIZE_H_

#include "asf.h"		// declaration of all drivers stuffs
#include "conf_usb.h"	// main include to interface atmel spi apis

typedef unsigned char	uint8;
typedef signed char		sint8;
typedef unsigned int	uint16;
typedef int				sint16;
typedef unsigned long	uint32;
typedef long			sint32;

#define NULL_PTR 0x0   // a null pointer

#define REG_(x)			(*(volatile uint32*)(x))
#define RD_R(REG_)		(REG_)
#define WR_R(REG_, D)	(REG_ = D)


/* SPI RODAN registers */
#define SPIBASEADDR		SPI0   // ctrl/sts pipe which interfaces with fpga on tx board, liyenho
#define SPIFLEXCOM			BOARD_FLEXCOM_SPI0
#define SPIRDFIFO				/*REG_(SPIBASEADDR + 0x08) for any reason, this can't work?! */	/* SPI read FIFO register */ \
																(((Spi*)SPI0)->SPI_RDR)
#define SPIWRFIFO				/*REG_(SPIBASEADDR + 0x0C) for any reason, this can't work?! */	/* SPI write FIFO register */ \
																(((Spi*)SPI0)->SPI_TDR)
#define SPI_CHIP_SEL		0
#define ZERO_MASK       0x00000000
#define DUMMY_BYTE      0xff  // cannot be all 0s


/* Status register masks */
#define SPI_SR1_WIP				(1 << 0)
#define SPI_SR1_WEL				(1 << 1)
#define SPI_SR1_BP0				(1 << 2)
#define SPI_SR1_BP1				(1 << 3)
#define SPI_SR1_BP2				(1 << 4)
#define SPI_SR1_E_FAIL			(1 << 5)
#define SPI_SR1_P_FAIL			(1 << 6)
#define SPI_SR1_SRWD			(1 << 7)

#define SPI_SR1_FAIL_FLAGS		(SPI_SR1_E_FAIL | SPI_SR1_P_FAIL)

#define SDELAY_(x) \
   {int i = 0x0; while(i < x) {i++;}}

#define FLUSHRWFIFO 	/*delay_us(10)*/ delay_us(1)

#define CHECK_BSY   \
	while (!((Spi*)SPIBASEADDR)->SPI_SR & SPI_SR_TXEMPTY); \
	/*delay_us(10)*/ delay_us(1)

// SPI_SR_RDRF never been asserted?! liyenho
#define CHECK_RX_FIFO \
    (((Spi*)SPIBASEADDR)->SPI_SR & SPI_SR_RDRF)

#define SET_CS \
   pio_clear(PIOA, PIO_PA25);

#define CLEAR_CS \
   pio_set(PIOA, PIO_PA25);

/*Return Type*/

typedef enum
{
	RetSpiError,
	RetSpiSuccess
} SPI_STATUS;

typedef unsigned char Bool;

// Acceptable values for SPI master side configuration
typedef enum _SpiConfigOptions
{
	OpsNull,  			// do nothing
	OpsWakeUp,			// enable transfer
	OpsInitTransfer,
	OpsEndTransfer,

} SpiConfigOptions;


// char stream definition for
typedef struct _structCharStream
{
	uint8* pChar;                                // buffer address that holds the streams
	uint32 length;                               // length of the stream in bytes
} CharStream;

extern void spi_master_initialize(uint32_t ch, uint32_t base, uint32_t fxcom);
void download_fpga_fw(uint32_t *upgrade_fw_hdr);
SPI_STATUS Serialize_SPI(const CharStream* char_stream_send,
                         CharStream* char_stream_recv,
                         SpiConfigOptions optBefore, SpiConfigOptions optAfter) ;

SPI_STATUS Spi0PortTxInit(void);
void ConfigureSpi(SpiConfigOptions opt);
//

#endif //end of file


