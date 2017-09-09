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
#define __REL_DATE__	"July/21/2017"	// update only when released not every build
#include <compiler.h>  // for U8/16/32 definition, liyenho
#include "usb_protocol_cdc.h"

#define USE_UART									0 /*disabled for now as we don't drone serial link*/
#define CTRL_RADIO_ENCAP  // ctrl radio data packet in video TS
//#define DEBUG_ANT_SWITCH	//landing gear based antaena select
#define  DEBUG_RADIOSTATUS //insert status data into 188 control payload
#define TP_SIZE					188
#define I2SC_BUFFER_SIZE		11*TP_SIZE
 // host usb/spi buffer for ctrl/sts, link up with fpga or 6612, liyenho
#define USB_DEVICE_SPECIFIC_REQUEST()			main_vender_specific()
#define USB_DEVICE_SYSTEM_RESTART()			main_loop_restart()  // restart yuneec design if usb restart, liyenho
 #define USB_SYSTEM_RESTART_VAL				0xa5
#define USB_INIT_VID_SUBSYS								0xa1		// ite video subsystem init
#define USB_START_VID_SUBSYS							0xa2		// ite video subsystem start
#define USB_VID_ANT_SWITCH								0xa3		// switch video antenna
 #define USB_ATMEL_VER_VAL								0x20
#define USB_STREAM_OFF_VAL							0xa
#define USB_STREAM_ON_VAL							0xe
  #define USB_STREAM_IDX								0x1	// data instead comm interface
  #define USB_QUERY_IDX									0xff	// used to query run time indicator
  #define USB_STREAM_LEN								0
#define USB_HOST_MSG_TX_VAL						0x5
#define USB_HOST_MSG_RX_VAL						0x9
  #define HOST_BUFFER_SIZE							(256-1)
  #define USB_HOST_MSG_IDX						0x1	// data instead comm interface
  #define USB_HOST_MSG_LEN						sizeof(dev_access)
  #define RADIO_SPI_BR											1000000
#define PID_VID 											0x00000147	/*0x100*/
//#define TIME_ANT_SW		// enable latency measurement on video antenna switch
//#define VIDEO_DUAL_BUFFER	// enable video TS stream ruplication
#define RADIO_TAISYNC
#define RADIO_CTRL_AUTO   //be sure to include #define MEDIA_ON_FLASH
                            //on usb_tx: must enable MEDIA_ON_FLASH and disable SRC_FRM_ENET
//#define RADIO_CTRL_TXCONTINOUS // must enable RADIO_CTRL_AUTO and
                               //  Independent of MEDIA_ON_FLASH  (updated)
                               // must disable CTRL_DYNAMIC_MOD
							   // must update the si4463/radio_config.h
//#define CTRL_DYNAMIC_MOD
#ifdef  RADIO_TAISYNC
 #undef RADIO_SPI_BR
 #define RADIO_SPI_BR											1000000
 #define RADIO_COMM_VAL								0x10  // using 5 bit out of 16 bit should be alright?
 #define RADIO_CAL_IDX										0x1 	//  factory calibration on crystal frequency
 #define RADIO_CAL_DONE_IDX 		 			0xc	// user query cmd index on cap value tuning process
 // it should be safe to use wIndex alternatively instead pointer to interface index
 #define RADIO_STARTUP_IDX						0x2
 #define RADIO_DATA_TX_IDX 						0x3
 #define RADIO_DATA_RX_IDX						0x4

 #define RADIO_STATS_IDX								0x5
 #define RADIO_STATS_LEN							sizeof(ctrl_radio_stats)

 // pairing operation IDs
 #define RADIO_PAIRID_IDX							0xa
 #define RADIO_PAIR_LOCKED_IDX 		                0xb
 #define RADIO_PAIR_LOCKED_LEN						4
  enum pair_mode {
	  IDLE = 0,
	  PAIRING,
	  PAIRED,
  } ;
//directional antenna selection
  #define DRONE_GPS_IDX								0x10
  #define DRONE_GPS_LEN								2*sizeof(float) //size in bytes
  #define DRONE_YAW_IDX								0x11
  #define DRONE_YAW_LEN								sizeof(float)//size in bytes
  #define CAMERA_YAW_IDX							0x12
  #define CAMERA_YAW_LEN							sizeof(float)//size in bytes
  #define RADIO_ANT_QUERY_IDX						0x13 //query which antenna is selected by yaw algo
#endif
/* Chip select. */
#define SPI_CHIP_SEL 0
#define RFFE_PARAMS				// shared between tx/rx platforms from host
typedef enum {
	OFDM_BW = 0,
	OFDM_MCS = 1,
	OFDM_CARR = 2,
	RDO_PWR = 3,	// max 26 dbm
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
} access_mode;

typedef struct {
	/*enum access_mode*/uint8_t access;
	uint8_t dcnt;	// data count
	uint16_t addr;	// first access address
	uint8_t data[1];
} dev_access;

 #define CONFIG_AD80402 // enable adi 80402 config
 #define SEND_TAISYNC
 #ifdef RFFE_PARAMS
 	#define CONFIG_AD80402 // forced to configure ad80402 prior to startup
 #endif

#ifdef CONFIG_AD80402
	#define FPGA_ARTIX_TRIG    PIO_PA11
#endif

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

//#define FWM_DNLD_DBG		// don't turn it on unless it's necessary, the last chunk of data would not match but it is ok...
#define FW_DNLD_SIZE			7200/*15000*/ // NEVER be divisible into 64 or usb core shall malfunction (a big bug!!!)
#define FW_UPGRADE_HDR_LEN		(12)
#define USB_BOOT_APP_VAL						0xa4
#define USB_STM_UPGRADE_VAL 0x21  // stm32 upgrade cmd
#define USB_FPGA_UPGRADE_VAL	0x22  // fpga upgrade cmd
#define USB_ATMEL_UPGRADE_VAL	0x23	// atmel upgrade cmd
/** parameters for video channel scan/select */
#define LO_Frequency 								/*1583000*/ 1686000
	#define VID_CH_BW							6000
	#define VID_CH_TTL							(VID_CH_BW+1000) // include guard band
	#define MAX_VID_CH_F					2478000
	#define MIN_VID_CH_F					2406000
	#define NUM_OF_VID_CH			((MAX_VID_CH_F-MIN_VID_CH_F)/VID_CH_TTL)
 #if NUM_OF_VID_CH < 3
	#error Number of video channels cannot be less than 3
 #endif
	#define VID_IF_CH_BASE				(MIN_VID_CH_F-LO_Frequency)

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
  //#define MEDIA_ON_FLASH  // use hosttx that loads flash (e.g. hosttx160720_ctrl1dir)
  #if defined(CONFIG_ON_FLASH) || defined(MEDIA_ON_FLASH)
	//#define NO_USB	// store TS packets on Sram if usb isn't available
    #include <string.h>  // declare memcpy()
    #define SECTOR_RES_C 	(SECTOR_SIZE_L - IFLASH_PAGE_SIZE * NUM_OF_PAGES)
    #define CHECKED_FLASH_WR(fadr, src, sz) \
							if (flash_write((fadr), (src), (sz), 0) != FLASH_RC_OK) \
								while (1) { \
									; /* Capture error */ \
								}
  #endif
 #if defined(MEDIA_ON_FLASH) && !defined(NO_USB)
  //#undef USB_DEVICE_SPECIFIC_REQUEST
  //#define USB_DEVICE_SPECIFIC_REQUEST()		main_usb_load_media()
  #define USB_LOAD_MEDIA					0xb
  #define USB_LOADM_IDX						0x1	// data instead comm interface
  #define USB_LOADM_LEN						sizeof(int)  // media file byte length
  bool main_usb_load_media(); // added by liyenho for media file download
  //#define DEBUG1
 #endif
extern uint32_t gs_uc_hrbuffer[(USB_HOST_MSG_LEN+HOST_BUFFER_SIZE+3)/sizeof(int)];

uint32_t radio_mon_rxcnt;
volatile bool ctrl_tdma_lock;


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
