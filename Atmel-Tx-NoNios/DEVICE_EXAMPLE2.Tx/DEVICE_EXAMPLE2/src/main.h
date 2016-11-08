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
#define __REL_DATE__	"Jun/06/2016"	// update only when released not every build
#include <compiler.h>  // for U8/16/32 definition, liyenho
#include "usb_protocol_cdc.h"
 // setup for potential nios replacement, liyenho
#include "config.h"
#include "parameters.h"
#include "platform.h"
#define I2SC_BUFFER_SIZE		10*188
 // host usb/spi buffer for ctrl/sts, link up with fpga or 6612, liyenho
#define USB_DEVICE_SPECIFIC_REQUEST()			main_vender_specific()
#define USB_DEVICE_SYSTEM_RESTART()			main_loop_restart()  // restart yuneec design if usb restart, liyenho
 #define USB_SYSTEM_RESTART_VAL				0xa5
 #define USB_ATMEL_VER_VAL								0x20
#define USB_STREAM_OFF_VAL							0xa
#define USB_STREAM_ON_VAL							0xe
  #define USB_STREAM_IDX								0x1	// data instead comm interface
  #define USB_QUERY_IDX									0xff	// used to query run time indicator
  #define USB_STREAM_LEN								0
#define USB_HOST_MSG_TX_VAL						0x5
#define USB_HOST_MSG_RX_VAL						0x9
  #define HOST_BUFFER_SIZE							(128-1)
  #define USB_HOST_MSG_IDX						0x1	// data instead comm interface
  #define USB_HOST_MSG_LEN						sizeof(dev_access)
  #define RADIO_SPI_BR											1000000
#define RADIO_SI4463
#define RADIO_CTRL_AUTO   //be sure to include #define MEDIA_ON_FLASH
                            //on usb_tx: must enable MEDIA_ON_FLASH and disable SRC_FRM_ENET
//#define RADIO_CTRL_TXCONTINOUS // must enable RADIO_CTRL_AUTO and
                               //  Independent of MEDIA_ON_FLASH  (updated)
                               // must disable CTRL_DYNAMIC_MOD
							   // must update the si4463/radio_config.h
//#define CTRL_DYNAMIC_MOD
#ifdef  RADIO_SI4463
 #undef RADIO_SPI_BR
 #define RADIO_SPI_BR											1000000
 #define RADIO_COMM_VAL								0x10  // using 5 bit out of 16 bit should be alright?
 #define RADIO_CAL_IDX										0x1 	//  factory calibration on crystal frequency
 #define RADIO_CAL_DONE_IDX 		 			0xc	// user query cmd index on cap value tuning process
 // it should be safe to use wIndex alternatively instead pointer to interface index
 #define RADIO_STARTUP_IDX						0x2
 #define RADIO_DATA_TX_IDX 						0x3
 #define RADIO_DATA_RX_IDX						0x4
  extern volatile uint8_t spi_radio_done;  // exposed to si4463 radio module
 #define RADIO_STATS_IDX								0x5
 #define RADIO_STATS_LEN							sizeof(ctrl_radio_stats)
 #define RADIO_MODEM_IDX							0x6
 #define RADIO_MODEM_LEN						sizeof(struct si446x_reply_GET_MODEM_STATUS_map)
 extern void si446x_get_modem_status( U8 MODEM_CLR_PEND );
 extern void si446x_get_property(U8 GROUP, U8 NUM_PROPS, U8 START_PROP);
 #define RADIO_CHSEL_IDX								0x7
 #define RADIO_CHSEL_LEN0							0xa  //{0x11, 0x40, 0x06, 0x00, 0x36, 0x0F, 0x80, 0x00, 0x5, 0x1f};
 #define RADIO_CHSEL_LEN1							0x7  //{0x11, 0x20, 0x03, 0x30, 0x67, 0xF8, 0x80};
 #define RADIO_CHSEL_LEN2							0x10  //{0x11, 0x21, 0x0C, 0x00, 0xFF, 0xC4, 0x30, 0x7F, 0xF5, 0xB5, 0xB8, 0xDE, 0x05, 0x17, 0x16, 0x0C};
 #define RADIO_CHSEL_LEN3							0x10  //{0x11, 0x21, 0x0C, 0x0C, 0x03, 0x00, 0x15, 0xFF, 0x00, 0x00, 0xFF, 0xC4, 0x30, 0x7F, 0xF5, 0xB5};
 #define RADIO_CHSEL_LEN4							0x10  //{0x11, 0x21, 0x0C, 0x18, 0xB8, 0xDE, 0x05, 0x17, 0x16, 0x0C, 0x03, 0x00, 0x15, 0xFF, 0x00, 0x00};
 //#define TEMPERATURE_MEASURE			// created single frequency tx @ base frequency defined with config file when defined, liyenho
 #define RADIO_TEMP_IDX									0x8
 #define RADIO_TEMP_LEN								sizeof(int16_t)
 #define RADIO_CTUNE_IDX								0x9
 #define RADIO_CTUNE_LEN								sizeof(uint8_t)
 #define RADIO_HOPLESS_IDX						0xd
 #define HOP_ID_LEN											10
 // pairing operation IDs
 #define RADIO_PAIRID_IDX							0xa
 #define RADIO_PAIR_LOCKED_IDX 		                0xb
 #define RADIO_PAIR_LOCKED_LEN		4
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
#define RFFE_PARAMS				// shared between tx/rx platforms from host
#ifdef RFFE_PARAMS
 typedef union {
	 struct {
		 uint16_t chan_idx;
		 uint16_t pwr_att;
	 } params_tx;
	 struct {
		 uint16_t chan_idx;
		 uint16_t if_carrier;  // in mhz
	 } params_rx;
 } rf_params;
 #define RF_TX_VAL								0x11
 #define RF_TX_FREQ_VAL				0x13
 #define RF_TX_ATTN_VAL				0x14
 #define RF_TX_NIOS_DONE			0x16
#endif
/*
  bit 15-12: SPI Command defined as following
	  4'b0000: Read from SPI 8-bit Data Register
      4'b0010: Read from SPI 8-bit Data Register and
               Assert a Read Command from the same Address
      4'b0011: Read from SPI 8-bit Data Register and
               Assert a Read Command from (Address Register + 1)
      4'b0100: Read from SPI 12-bit Address Register

      4'b1000: Write to SPI 8-bit Data Register
      4'b1010: Write to SPI 8-bit Data Register and
               Assert a Write Command
      4'b1011: Write to SPI 8-bit Data Register and
               Assert a Write Command and then
               Increase Address Register by 1
      4'b1100: Write to SPI 12-bit Address Register
      4'b1101: Write to SPI 12-bit Address Register and
               Prepare data in SPI 8-bit Data Register
    bit 11-0: Data/Address Field (AD)
*/
enum access_mode {
	/*! \read data register current content */
	READ_CUR = 0,
	/*! \read addressed content from data register */
	READ_BY_ADDR,
	/*! \modify data register content */
	WRITE_CUR,
	/*! \modify addressed data content */
	WRITE_BY_ADDR,
	/*! \burst read at the same fifo address */
	BURST_FIFO_READ,
	/*! \burst write at the same fifo address */
	BURST_FIFO_WRITE,
	/*! \burst read at contiguous memory address */
	BURST_MEM_READ,
	/*! \burst write at contiguous memory address */
	BURST_MEM_WRITE,
	/*! \dvbt/2 signal detected from sms4470 @ rec */
	SIGNAL_DETECTED,
	/*! \dvbt/2 signal status from sms4470 @ rec */
	SIGNAL_STATUS,
	/*! \transmission status from sms4470 @ rec */
	TRANS_STATUS,
	/*! \reciever status from sms4470 @ rec */
	RECV_STATUS,
	/*! \locked status from sms4470 @ rec */
	LOCKED_STATUS,
};

typedef struct {
	/*enum access_mode*/uint16_t access;
	uint16_t dcnt;	// data count
	uint16_t addr;	// first access address, 12 bit
	uint16_t toflash; // request to burn to flash or not
	uint16_t data[1]; // only valid on lower half
} dev_access;

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

//#define FWM_DNLD_DBG		// don't turn in on unless it's necessary, the last chunk of data would not match but it is ok...
#define FW_DNLD_SIZE			/*7200*/15000 // NEVER be divisible into 64 or usb core shall malfunction (a big bug!!!)
#define FW_UPGRADE_HDR_LEN		(12)
#define USB_FPGA_UPGRADE_VAL		0x22  // fpga upgrade cmd
#define USB_ATMEL_UPGRADE_VAL	0x23	// atmel upgrade cmd

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
							ul_rc = flash_write((fadr), (src), (sz), 0); \
							if (ul_rc != FLASH_RC_OK) \
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
 #ifdef RADIO_SI4463
   #define SI4463_HOST_INT			PIO_PA29
   #define SI4463_PWRDN			PIO_PA30
   #define HEALTH_LED			PIO_PB13
 #else
   #define HEALTH_LED			PIO_PA30
 #endif
//bool main_start_usb_write(); // added by liyenho for sync up host rx
bool main_vender_specific();
void main_loop_restart() ; // restart main system process...

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
