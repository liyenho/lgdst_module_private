#ifndef _TAISYNC_H_
#define _TAISYNC_H_ // detailed taisync video header

#include "conf_usb.h"
//#include "main.h"

#ifdef CONF_BOARD_USB_RX
 //#define LO_Frequency 							/*1583000*/ 1686000
	#define VID_CH_BW							/*6000*/10000
	#define VID_CH_TTL							(VID_CH_BW+1000) // include guard band
	#define MAX_VID_CH_F					2478000
	#define MIN_VID_CH_F					2406000
	#define NUM_OF_VID_CH			((MAX_VID_CH_F-MIN_VID_CH_F)/VID_CH_TTL)
 #if NUM_OF_VID_CH < 3
	#error Number of video channels cannot be less than 3
 #endif
	#define VID_IF_CH_BASE				(MIN_VID_CH_F-LO_Frequency)
	#define VID_IF_CH_CEIL				VID_IF_CH_BASE+(NUM_OF_VID_CH-1)*VID_CH_TTL
	#define VID_CH_STR_THR			-61	// translated to approximately 1 miles attenuation

#elif defined(CONF_BOARD_USB_TX)
  #ifdef RECV_TAISYNC
	//#define TS_FAIL_INT						PIO_PA24
	#define TS_CAPACITY						(4.5*1000000) // 4.5 mb/s assumed
																				// if errors account for 75% of ts traffic
	#define TP_ERR_RATE						0.75*TS_CAPACITY/(188*8)
  #endif
#endif
#define Error_NO_ERROR					0
#define PID_VID 										0x00000001	/*0x100*/
#define TSLUT_BUFFER_SIZE		(188*10)
#ifdef CTRL_RADIO_ENCAP
  #define PID_CTL										0x00000010	/*0x1000*/
  #define MDE_CTL										0x00300000 // adaptation field w payload, cc=0
  #define USR_CTL										0x00000082 // user private mode, no track of cc
#endif
#define MAX_RDO_PWR						26
#define CMD_BUF_LEN						6		// 6 byte r/w command transaction

#define VAR_DECL_SPI \
					uint8_t pth[CMD_BUF_LEN], \
									prh[CMD_BUF_LEN]; \
					uint32_t wtmp;

typedef enum {
	FIVE_MHZ,
	TEN_MHZ,
	TWENTY_MHZ,
	FORTY_MHZ,
} VCH_BW;

typedef enum {
	BPSK_1B2,
	BPSK_3B4,
	BPSK_5B6,
	QPSK_1B2,
	QPSK_3B4,
	QPSK_5B6,
	QAM16_1B2,
	QAM16_3B4,
	QAM16_5B6,
	QAM64_1B2,
	QAM64_3B4,
	QAM64_5B6,
} VCH_MD;

#if defined(CONF_BOARD_USB_TX)
//#define MEASURE_TIMING // NEVER enable this option, it can't work!!! liyenho
#ifdef MEASURE_TIMING
	extern volatile uint32_t tick_30us ;
#endif
	extern void configure_rtt(unsigned int clkcnt);
	extern void twi_sms4470_handler(const uint32_t id, const uint32_t index);
#endif //CONF_BOARD_USB_TX

extern void spi_tx_transfer(void *p_tbuf, uint32_t tsize, void *p_rbuf,
		uint32_t rsize, uint32_t ch);
extern void spi_rx_transfer(void *p_tbuf, uint32_t tsize, void *p_rbuf,
		uint32_t rsize, uint32_t ch);
extern volatile uint8_t spi_tgt_done ; // from main.c, liyenho

 #define dev_spi_wr(adr,dly,wr) \
 	  wtmp = 0x0005 | (0xff00 & ((short)adr<<8)); \
 	  *(pth+0) = *(((uint8_t*)&wtmp)+0); \
 	  *(pth+1) = *(((uint8_t*)&wtmp)+1); \
 	spi_tgt_done = true; wtmp = (wr); \
		*(pth+2) = *(((uint8_t*)&wtmp)+0); \
		*(pth+3) = *(((uint8_t*)&wtmp)+1); \
		*(pth+4) = *(((uint8_t*)&wtmp)+2); \
		*(pth+5) = *(((uint8_t*)&wtmp)+3); \
 	  spi_tx_transfer(pth, 6, prh, 6, 0/*ctrl*/); \
 	  while (spi_tgt_done) ; delay_us(dly);

#include <assert.h>
 #define dev_spi_rd(adr,dly,rd,dcnt) \
  	  wtmp = 0x000a | (0xff00 & ((short)adr<<8)); \
 	  *(pth+0) = *(((uint8_t*)&wtmp)+0); \
 	  *(pth+1) = *(((uint8_t*)&wtmp)+1); \
 	spi_tgt_done = true; \
 	  spi_rx_transfer(pth, 6, prh, 6, 0/*ctrl*/); \
 	  while (spi_tgt_done) ; delay_us(dly); \
 	  switch(dcnt) { \
	 	  case 4 : \
			*(((uint8_t*)(rd))+0) = *(prh+2) ; \
			*(((uint8_t*)(rd))+1) = *(prh+3); \
			*(((uint8_t*)(rd))+2) = *(prh+4); \
			*(((uint8_t*)(rd))+3) = *(prh+5) ; \
			break ; \
	 	  case 2 : \
			*(((uint8_t*)(rd))+0) = *(prh+2) ; \
			*(((uint8_t*)(rd))+1) = *(prh+3); \
			break; \
	 	  case 1 : \
			*(((uint8_t*)(rd))+0) = *(prh+2) ; \
			break; \
	 	  default : assert(0); \
 	  }

void set_radio_frequency(uint32_t frq);
void set_radio_bandwidth(VCH_BW bw);
void enable_radio_antenna(bool en);
void get_radio_cmd_sts(uint8_t*);

	void set_radio_power(uint32_t pwr);
	void set_radio_modulation(VCH_MD mod);
	void get_radio_dnlk_buf(uint8_t*);
	void get_radio_dnlk_frms(uint32_t*);
	void get_radio_dnlk_lost_frms(uint32_t*);

	void get_radio_uplk_buf(uint16_t*);
	void get_radio_uplk_frms(uint32_t*);
	void get_radio_uplk_lost_frms(uint32_t*);
	void get_radio_snr(uint8_t*);
	void get_rdo_ldpc_failed(uint32_t*);
	void get_radio_rssi(uint8_t*);
	void get_radio_rx_vga(uint8_t*);
	void get_radio_bb_sts(uint8_t*);

#endif // _TAISYNC_H_
