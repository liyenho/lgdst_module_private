#ifndef _ITE_H_
#define _ITE_H_ // detailed it9137 header
#ifdef CONF_BOARD_USB_TX
#include "main.h"
#define LO_Frequency 							/*1583000*/ 1686000
#define VIDEO_SETVCH_VAL								0x14
  #define VIDEO_SETVCH_IDX								0x2
#ifdef RECV_IT913X
	#define TS_FAIL_INT						PIO_PA24
	#define TS_CAPACITY						(4.5*1000000) // 4.5 mb/s assumed
																				// if errors account for 75% of ts traffic
	#define TP_ERR_RATE						0.75*TS_CAPACITY/(188*8)
#endif
#define PID_VID 										0x00000001	/*0x100*/
#define TSLUT_BUFFER_SIZE		(188*10)
#ifdef CTRL_RADIO_ENCAP
  #define PID_CTL										0x00000010	/*0x1000*/
  #define MDE_CTL										0x00300000 // adaptation field w payload, cc=0
  #define USR_CTL										0x00000082 // user private mode, no track of cc
#endif
//#define MEASURE_TIMING // NEVER enable this option, it can't work!!! liyenho
#ifdef MEASURE_TIMING
	extern volatile uint32_t tick_30us ;
#endif
	extern void configure_rtt(unsigned int clkcnt);
	extern void spi_tx_transfer(void *p_tbuf, uint32_t tsize, void *p_rbuf,
		uint32_t rsize, uint32_t ch);
	extern void twi_sms4470_handler(const uint32_t id, const uint32_t index);
#endif //CONF_BOARD_USB_TX
#endif // _ITE_H_
