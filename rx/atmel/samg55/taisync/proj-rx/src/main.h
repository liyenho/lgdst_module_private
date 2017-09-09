/**
 * \file
 *
 * \brief Main functions
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#ifndef _MAIN_H_
#define _MAIN_H_
#define __REL_DATE__	"June/15/2017"	// update only when released not every build
#include <compiler.h>  // for U8/16/32 definition, liyenho
#include "usb_protocol_cdc.h"

#define CTRL_RADIO_ENCAP  // ctrl radio data packet in video TS
#define  DEBUG_RADIOSTATUS //pass radiostatus from tx to rx through 188
                           //corrupts ctrl payload going out via USB
						   //works only with legacy typhoonH st16 test code
//#define DEBUG_BLOCK_TS_ISR //disable video isr, result in no spi activity to read ts spi
#define GRAND_BUFFER_BLKS    8
#define I2SC_BUFFER_SIZE		10*188
#define GRAND_BUFFER_SIZE	GRAND_BUFFER_BLKS*I2SC_BUFFER_SIZE

 // host usb/spi buffer for ctrl/sts, link up with fpga or 2072, liyenho
#define USB_DEVICE_SPECIFIC_REQUEST()			main_vender_specific()
#define USB_DEVICE_SYSTEM_RESTART()			main_loop_restart()  // restart yuneec design if usb restart, liyenho
 #define USB_SYSTEM_RESTART_VAL				0xa5
#define USB_INIT_VID_SUBSYS								0xa1		// ite video subsystem init
#define USB_START_VID_SUBSYS							0xa2		// ite video subsystem start
 #define USB_ATMEL_VER_VAL								0x20
#define USB_STREAM_OFF_VAL							0xa
#define USB_STREAM_ON_VAL							0xe
#define USB_ANT_SW_VAL										0xf
  #define USB_QUERY_IDX									0xff	// used to query run time indicator
  #define USB_STREAM_IDX								0x1	// data instead comm interface
  #define USB_STREAM_LEN								0
#define USB_HOST_MSG_TX_VAL						0x5
#define USB_HOST_MSG_RX_VAL						0x9
  #define HOST_BUFFER_SIZE							(256-1)
  #define USB_HOST_MSG_IDX						0x1	// data instead comm interface
  #define USB_HOST_MSG_LEN						sizeof(dev_access)
/* not useful at all, latency is not the point but end result does, it corrupts video no matter how short it is */
//#define TIME_ANT_SW		// enable latency measurement on video antenna switch
//#define VIDEO_DUAL_BUFFER	// enable video TS stream ruplication
#define RADIO_SPI_BR								1000000


#define RADIO_INFO_LEN      				4 // usb pipe information post header


#define RADIO_TAISYNC
#define RADIO_CTRL_AUTO 			// download si4463 w/o host config ctrl xfer, liyenho
#ifdef  RADIO_TAISYNC
 #undef RADIO_SPI_BR
 #define RADIO_SPI_BR											1000000
 #define RADIO_COMM_VAL								0x10  // using 5 bit out of 16 bit should be alright?
// it should be safe to use wIndex alternatively instead pointer to interface index
 #define RADIO_STARTUP_IDX						0x2
 #define RADIO_DATA_TX_IDX 						0x3
 #define RADIO_DATA_RX_IDX						0x4
 #define RADIO_STATS_IDX								0x5
 #define RADIO_STATS_LEN							sizeof(ctrl_radio_stats)

 // pairing operation IDs
 #define RADIO_PAIRID_IDX								0xa
 #define RADIO_PAIR_LOCKED_IDX			0xb
#define RADIO_PAIR_LOCKED_LEN			4
  enum pair_mode{
	  IDLE = 0,
	  PAIRING,
	  PAIRED,
  };


 //control link message type indicating FEC, first half
 #define MSG_TYPE_FEC_ONLY_HDR 0x5B
 //control link message type indicating FEC, second half
 #define MSG_TYPE_FEC_ONLY_FTR 0x5D

extern uint32_t pkt_start_tick;

#define FIXED_PAIR_ID						1
  #define tm_delta(prev, curr, del) \
		if (curr < prev)	\
			del=curr+(0xffffffff-prev);	\
		else	\
			del=curr -prev;
	#define minmax(mn, mx, v)	\
		if (0==mn)	\
			mn = (v);	\
		else if (mn>(v))	\
			mn = (v);	\
		else if (mx<(v))	\
			mx = (v);
#endif
#define SPI_CHIP_SEL 0	// device 0 on spi bus
#define RFFE_PARAMS
typedef enum {
	OFDM_BW = 0,
	OFDM_MCS = 1,
	OFDM_CARR = 2,
	RDO_PWR = 3, // max 26 dbm
	RDO_ANT = 4,	// on:1/off:0
	RDO_SNR = 5,
	LDPC_FAILED= 6,
	RDO_RSSI = 7,
	RDO_RX_VGA = 8,
	RX_BB_STS= 9,
	DNLK_BUF_STS= 10,
	UPLK_BUF_STS= 11,
	CMD_STS = 12,
	TOTAL_FRMS_DN= 13,
	LOST_FRMS_RX= 14,
	TOTAL_FRMS_UP= 15,
	LOST_FRMS_TX= 16,
	TS_VID_ACTIVE =17, // allow video to startup
} access_mode;

typedef struct {
	/*enum access_mode*/uint8_t access;
	uint8_t dcnt;	// data count
	uint16_t addr;	// first access address
	uint8_t data[1];
} dev_access;

 #define CONFIG_AD80402 // enable adi 80402 config
 #ifdef RFFE_PARAMS
 	#define CONFIG_AD80402 // forced to configure ad80402 prior to startup
 #endif
 #define RX_SPI_CHAINING       // enable spi chaining
 #define RECV_TAISYNC
 //#ifdef CTRL_DYNAMIC_MOD
#ifdef CONFIG_AD80402
	#define FPGA_ARTIX_TRIG    PIO_PA11
#endif

//#define FWM_DNLD_DBG		// don't turn it on unless it's necessary, the last chunk of data would not match but it is ok...
#define FW_DNLD_SIZE		/*8*1024*/ 7200 // NEVER be divisible into 64 or usb core shall malfunction (a big bug!!!)
#define FW_UPGRADE_HDR_LEN		(12)
#define USB_BOOT_APP_VAL						0xa4
#define USB_STM_UPGRADE_VAL 0x21	// stm32 upgrade cmd
#define USB_FPGA_UPGRADE_VAL	0x22  // fpga upgrade cmd
#define USB_ATMEL_UPGRADE_VAL	0x23	// atmel upgrade cmd

/*! \brief Opens the communication port
 * This is called by CDC interface when USB Host enable it.
 *
 * \retval true if cdc startup is successfully done
 */
bool main_cdc_enable(uint8_t port);

/*! \brief Closes the communication port
 * This is called by CDC interface when USB Host disable it.
 */
void main_cdc_disable(uint8_t port);

/*! \brief Manages the leds behaviors
 * Called when a start of frame is received on USB line each 1ms.
 */
void main_sof_action(void);

 #define NUM_OF_PAGES	400	// let's reserve 200 kbyte (half) storage for TS file
 #define NUM_OF_FPGA_REGS		128
 #define NUM_OF_ATMEL_REGS		64
 /* storage page start address. */
 #define PAGE_ADDRESS (IFLASH_ADDR + IFLASH_SIZE - IFLASH_PAGE_SIZE * NUM_OF_PAGES)
 #define SECTOR_SIZE_L	(16*IFLASH_LOCK_REGION_SIZE)
 #define SECTOR_RES 	(2/*two sectors used*/*SECTOR_SIZE_L - IFLASH_PAGE_SIZE * NUM_OF_PAGES)
 #define SECTOR_RES1 	(SECTOR_RES-SECTOR_SIZE_L)
#define MEM_ERASE_SIZE (IFLASH_PAGE_SIZE * 16)
/* memory aligned (8 pages to erase) */
#define mem_align(addr) \
	(((addr) % MEM_ERASE_SIZE) ? ((addr) - ((addr) % MEM_ERASE_SIZE)) : (addr))
																										//BOOT_SIZE shall be defined in toolchain or makefile! liyenho
#define APP_START     mem_align(IFLASH_ADDR + BOOT_SIZE)

  #define CONFIG_ON_FLASH
  //#define MEDIA_ON_FLASH
  #if defined(CONFIG_ON_FLASH) || defined(MEDIA_ON_FLASH)
    #define CHECKED_FLASH_WR(fadr, src, sz) \
							if (flash_write((fadr), (src), (sz), 0) != FLASH_RC_OK) \
								while (1) { \
									; /* Capture error */ \
								}
  #endif
extern uint32_t gs_uc_hrbuffer[(USB_HOST_MSG_LEN+HOST_BUFFER_SIZE+3)/sizeof(int)];

//bool main_start_usb_write(); // added by liyenho for sync up host rx
bool main_vender_specific(void);
void main_loop_restart(void) ; // restart main system process...

/*! \brief Enters the application in low power mode
 * Callback called when USB host sets USB line in suspend state
 */
void main_suspend_action(void);

/*! \brief Turn on a led to notify active mode
 * Called when the USB line is resumed from the suspend state
 */
void main_resume_action(void);

/*! \brief Save new DTR state to change led behavior.
 * The DTR notify that the terminal have open or close the communication port.
 */
void main_cdc_set_dtr(uint8_t port, bool b_enable);

#ifdef USB_DEVICE_LPM_SUPPORT
/*! \brief Enters the application in low power mode
 * Callback called when USB host sets LPM suspend state
 */
void main_suspend_lpm_action(void);

/*! \brief Called by UDC when USB Host request to enable LPM remote wakeup
 */
void main_remotewakeup_lpm_enable(void);

/*! \brief Called by UDC when USB Host request to disable LPM remote wakeup
 */
void main_remotewakeup_lpm_disable(void);
#endif

#endif // _MAIN_H_
