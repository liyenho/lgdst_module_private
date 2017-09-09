#include <asf.h>
#include <compiler.h>  // for U8/16/32 definition, liyenho
#include "lgdst_4463_spi.h"
#include "conf_board.h"
#include "si446x_nirq.h"


#include "usart.h"
#include "pio.h"
#include "pio_handler.h"
#include "sysclk.h"

#define SI4463_SPI_BAUD		9600U




uint8_t init_4463_comm(void){
	puts("Configuring Si4463 SPI comm\r\n");
	
	//settings for USART/SPI controller
	usart_spi_opt_t Si4463_usart_opt = {
		.baudrate = SI4463_SPI_BAUD,
		.char_length = US_MR_CHRL_8_BIT,
		.spi_mode = SPI_MODE_0,
		.channel_mode = US_MR_CHMODE_NORMAL,
	};


	/* Enable the peripheral clock in the PMC. */
	sysclk_enable_peripheral_clock(SI4463_USART_ID);



	pio_set_peripheral(PIOB, PIO_PERIPH_C, SI4463_SDO_PIN | SI4463_SDI_PIN| SI4463_RF_NSEL_PIN| SI4463_SCLK_PIN);

	//pio_set_peripheral(PIOC, PIO_PERIPH)
	uint32_t hz = sysclk_get_peripheral_hz();
	uint32_t error =0;
	error = usart_init_spi_master(SI4463_USART, &Si4463_usart_opt,  hz);
	//printf("Hz %d usart_init_spi_master return: %d \r\n", hz, error);
	if (error){
		printf("Error detected in Si4463 init_spi_master!\r\n");
		;
	}
	
	//enable both sending and receiving on comm
	usart_enable_tx(SI4463_USART);
	usart_enable_rx(SI4463_USART);


	//pio_configure(PIOB, PIO_INPUT, PIO_PB3, 0);    //RF_GPI00
	//pio_configure(PIOB, PIO_INPUT, PIO_PB2, 0);    //RF_GPI01
	pio_configure(PIOA, PIO_OUTPUT_1, SI4463_PWRDN_PIN, 0); //RF_PWRDN
	

	/* Initialize SI4463 Host_Int line */ // for time critical TDM process
	pio_set_input(PIOA, SI4463_NIRQ_PIN, PIO_PULLUP);
	pio_handler_set(PIOA, ID_PIOA, SI4463_NIRQ_PIN,
		PIO_IT_AIME /*| PIO_IT_RE_OR_HL*/ | PIO_IT_EDGE, si4463_radio_handler);
	pio_enable_interrupt(PIOA, SI4463_NIRQ_PIN);
	pio_handler_set_priority(PIOA, PIOA_IRQn, 1/*long latency event*/);

	return error;
}



U8 si4463_tx_8b(U8 mydata, bool bLast)
{
	printf("si4463_tx_8b %#x\r\n", mydata);
	//printf("si4463_tx_8b\r\n");
/*	U16 status;
	
	spi_set_bits_per_transfer(SI4463_SPI, 0, SPI_CSR_BITS_8_BIT);
	spi_put(SI4463_SPI, mydata);
	
	status = 0;
	while ( !(status & SPI_SR_TXEMPTY) ) {
		status = spi_read_status(SI4463_SPI);
	}
	
	status = spi_get(SI4463_SPI);
	return status & 0xFF;*/
	//printf("usart_write\r\n");
	usart_write(SI4463_USART, mydata);
	U16 status = 0;
	//printf("check status\r\n");
	while (!usart_is_tx_empty(SI4463_USART)){
		status = usart_get_status(SI4463_USART);
	}
//	printf("usart_read\r\n");
	uint32_t ret_val;
	status = usart_read(SI4463_USART, &ret_val);
	//printf("usart_read status %#x", ret_val);
	return (ret_val &0xFF);	
}

//USART SPI-mode cannot transmit 16 bytes at a time, only use tx_8b
//U16 si4463_tx_16b(U16 mydata, bool bLast)
//{
	//printf("si4463_tx_16b %#x\r\n", mydata);
	////printf("si4463_tx_16b\r\n");
	///*U16 status;
	//
	//spi_set_bits_per_transfer(SI4463_SPI, 0, SPI_CSR_BITS_16_BIT);
	//spi_put(SI4463_SPI, mydata); // Check Ready
	//
	//status = 0;
	//while ( !(status & SPI_SR_TXEMPTY) ) {
		//status = spi_read_status(SI4463_SPI);
	//}
	//
	//status = spi_get(SI4463_SPI);
	//return status & 0xFFFF;*/
	//
	//usart_write(SI4463_USART, mydata);
	//U32 status = 0;
	//while (!usart_is_tx_empty(SI4463_USART)){
		//status = usart_get_status(SI4463_USART);
	//}
	//uint32_t ret_val;
	//status = usart_read(SI4463_USART, &ret_val);
	//
	//return (ret_val &0xFFFF);
	//
//}

U8 si4463_get_cts()
{
	printf("Si4463 Get Cts\r\n");
	U32 mydata;
	
	SBIT_W(RF4463_NSEL, 0);
	const U16 get_cts_data =  0x44FF;
	si4463_wrbytes(&get_cts_data, 2);
	//si4463_tx_16b(0x44FF, true);
	SBIT_W(RF4463_NSEL, 1);
	
	usart_read(SI4463_USART, &mydata);

	return mydata & 0xFF;
}

bool si4463_wrbytes(U8 *buf, U8 size)
{
	
	#ifdef si4463_tx_16b
	U16 tmp;
	for (uint8_t i=0; i < size; ) {
		//printf("Sending Si4463 SPI data %#x\r\n", (uint16_t buf[i]);
		if ((size-i) >= 3) {
			tmp = buf[i] << 8 | buf[i+1];
			si4463_tx_16b(tmp, false);
			i+= 2;
		} else if ((size-i) == 2) {
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
	#else
	for (int i=0; i<size; i++){
		si4463_tx_8b(buf[i], false);
	}
	#endif
	return true;
}

bool si4463_rdbytes(U8 *buf, uint8_t size)
{
	printf("Reading Si4463 SPI\r\n");
	
	U16 tmp = 0x44FF;
	#ifdef si4463_tx_16b
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
	#else
	//tx_16b isn't defined, must do 8 bits at a time
	for (int i=0;i<size;i++){
		tmp = si4463_tx_8b(tmp, true);
		
		buf[i] = tmp & 0xFF;
	}
	
	#endif
	
	
	printf("Read Value: ");
	for(int j=0;j<size;j++){
		printf("%#x ", *(buf+j));
	}
	printf("\r\n");
}

void SBIT_W(U8 IO_name, U8 IO_value)
{
	if (IO_value != 1 && IO_value != 0)
	return;
	
	switch (IO_name) {
		case RF4463_PWRDN:
			if (IO_value)
				pio_set(PIOA, SI4463_PWRDN_PIN);
			else
				pio_clear(PIOA, SI4463_PWRDN_PIN);
			break;
		case RF4463_NSEL:
			if (IO_value)
				pio_set(PIOB, SI4463_RF_NSEL_PIN);
			else
				pio_clear(PIOB, SI4463_RF_NSEL_PIN);
			break;
		default:
		;
	}
}

U8 SBIT_R(U8 IO_name)
{
	switch (IO_name) {
		case RF4463_PWRDN:
			return pio_get(PIOA, PIO_OUTPUT_1, SI4463_PWRDN_PIN);
		
		case RF4463_NSEL:
			return pio_get(PIOA, PIO_OUTPUT_1, SI4463_RF_NSEL_PIN);
		
		case RF4463_NIRQ:
			return pio_get(PIOA, PIO_INPUT, SI4463_NIRQ_PIN);
			
		case RF4463_GPI00:
			return pio_get(PIOA, PIO_INPUT, SI4463_GPIO0_PIN);
			
		case RF4463_GPI01:
			return pio_get(PIOA, PIO_INPUT, SI4463_GPIO1_PIN);
		
		default:
		;
	}
}

