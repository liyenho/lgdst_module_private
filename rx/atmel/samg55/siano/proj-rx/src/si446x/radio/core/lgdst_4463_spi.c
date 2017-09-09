#include <asf.h>
#include <compiler.h>  // for U8/16/32 definition, liyenho
#include "lgdst_4463_spi.h"
#include "ASF\sam\drivers\spi\spi.h"

U8 si4463_tx_8b(U8 mydata, bool bLast)
{
	U16 status;
	
	spi_set_bits_per_transfer(SI4463_SPI, 0, SPI_CSR_BITS_8_BIT);
	spi_put(SI4463_SPI, mydata);
	
	status = 0;
	while ( !(status & SPI_SR_TXEMPTY) ) {
		status = spi_read_status(SI4463_SPI);
	}
	
	status = spi_get(SI4463_SPI);
	return status & 0xFF;
}

U16 si4463_tx_16b(U16 mydata, bool bLast)
{
	U16 status;
	
	spi_set_bits_per_transfer(SI4463_SPI, 0, SPI_CSR_BITS_16_BIT);
	spi_put(SI4463_SPI, mydata); // Check Ready
	
	status = 0;
	while ( !(status & SPI_SR_TXEMPTY) ) {
		status = spi_read_status(SI4463_SPI);
	}
	
	status = spi_get(SI4463_SPI);
	return status & 0xFFFF;
}

U8 si4463_get_cts()
{
	U16 mydata;
	
	SBIT_W(RF4463_NSEL, 0);
	si4463_tx_16b(0x44FF, true);
	SBIT_W(RF4463_NSEL, 1);
	
	mydata = spi_get(SI4463_SPI);
	return mydata & 0xFF;
}

bool si4463_wrbytes(U8 *buf, U8 size)
{
	U16 tmp;
	for (uint8_t i=0; i < size; ) {
		if ((size-i) >= 3) {
			tmp = buf[i] << 8 | buf[i+1];
			si4463_tx_16b(tmp, false);
			i+= 2;
		}else if ((size-i) == 2) {
			tmp = buf[i] << 8 | buf[i+1];
			si4463_tx_16b(tmp, true);
			break;
		}
		else if ((size-i) == 1) {
			tmp = buf[i];
			si4463_tx_8b(tmp, true);
			break;
		}
	}
	return true;
}

bool si4463_rdbytes(U8 *buf, uint8_t size)
{
	U16 tmp = 0x44FF;
	for (U8 i=0; i < size; ) {
		
		if ((size-i) >= 3) {
			tmp = si4463_tx_16b(tmp, false);
			
			buf[i+1] = tmp & 0xFF;
			buf[i] = (tmp >> 8) & 0xFF;
			i+= 2;
			} else if ((size-i) == 2) {
			tmp = si4463_tx_16b(tmp, true);
			
			buf[i+1] = tmp & 0xFF;
			buf[i] = (tmp >> 8) & 0xFF;
			break;
		}
		else if ((size-i) == 1) {
			tmp = si4463_tx_8b(tmp, true);
			
			buf[i] = tmp & 0xFF;
			break;
		}
		tmp = 0xFFFF;
	}
}

void SBIT_W(U8 IO_name, U8 IO_value)
{
	if (IO_value != 1 && IO_value != 0)
		return;
		
	switch (IO_name) {
		case RF4463_PWRDN: 
			if (IO_value)
				pio_set(PIOA, PIO_PA15);
			else
				pio_clear(PIOA, PIO_PA15);
			break;
		case RF4463_NSEL:
			if (IO_value)
				pio_set(PIOA, PIO_PA30);
			else
				pio_clear(PIOA, PIO_PA30);
			break;
		default:
			;
	}
}

U8 SBIT_R(U8 IO_name)
{
	switch (IO_name) {
		case RF4463_PWRDN:
			return pio_get(PIOA, PIO_OUTPUT_1, PIO_PA15);
		
		case RF4463_NSEL:
			return pio_get(PIOA, PIO_OUTPUT_1, PIO_PA30);
		
		case RF4463_NIRQ:
			return pio_get(PIOB, PIO_INPUT, PIO_PB8);
		case RF4463_GPI00:
			return pio_get(PIOB, PIO_INPUT, PIO_PB13);
		case RF4463_GPI01:
			return pio_get(PIOA, PIO_INPUT, PIO_PA18);
			
		default:
		;
	}
}
