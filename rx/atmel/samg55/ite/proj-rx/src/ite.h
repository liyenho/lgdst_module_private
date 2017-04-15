#ifndef _SMS_H_
#define _SMS_H_ // detailed sms4470 header
#ifdef CONF_BOARD_USB_TX
#include "main.h"
#define LO_Frequency 							1583000
#define PID_VID 										0x00000001	/*0x100*/
#define TSLUT_BUFFER_SIZE		(188*10)
//#define MEASURE_TIMING // NEVER enable this option, it can't work!!! liyenho
#ifdef MEASURE_TIMING
	extern volatile uint32_t tick_30us ;
#endif
	extern void configure_rtt(unsigned int clkcnt);
	extern void spi_tx_transfer(void *p_tbuf, uint32_t tsize, void *p_rbuf,
		uint32_t rsize, uint32_t ch);
	extern void twi_sms4470_handler(const uint32_t id, const uint32_t index);
#endif //CONF_BOARD_USB_TX
#endif // _SMS_H_
