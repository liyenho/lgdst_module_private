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

#define USE_UART									/*0*/ 1
#define CTRL_RADIO_ENCAP  // ctrl radio data packet in video TS
//#define DEBUG_ANT_SWITCH	//landing gear based antaena select
#define  DEBUG_RADIOSTATUS //insert status data into 188 control payload
#define TP_SIZE					188
#define I2SC_BUFFER_SIZE		12*TP_SIZE  // to accommodate Biil's potential two blocks mavlink packet
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
 #define RADIO_HOPLESS_IDX							0xd

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
extern bool TX_Active;
#define RFFE_PARAMS				// shared between tx/rx platforms from host
#ifdef RFFE_PARAMS
 #define CONFIG_RF2072 // forced to configure rf2072 prior to startup
	#define CPLD_2072_TRIG    PIO_PA25
	#define FLUSH_2072_SPI \
						while (spi_tgt_done) ; /*flush any pending spi xfer*/ \
							spi_tgt_done = true;
	#define ACCESS_PROLOG_2072 \
							pio_configure(PIOB, PIO_OUTPUT_1, PIO_PB0, 0); \
						/*delay_cycles(0);*/ \
							pio_clear(PIOA, CPLD_2072_TRIG); \
						/*delay_cycles(0);*/ \
						  pio_clear(PIOB, PIO_PB0); \
						/*delay_us(1);*/ /*gap active cs from 1st clk*/ \
						  pio_set(PIOB, PIO_PB0); \
						/*delay_cycles(0);*/ \
						  pio_clear(PIOB, PIO_PB0); \
						  pio_set_peripheral(PIOB, PIO_PERIPH_A, PIO_PB0);
	#define WRITE_2072_SPI(adr, hi, lo) \
							*pth = 0x7f & adr; /*write access*/ \
							spi_tx_transfer(pth, 1, &tmpw, 1, 0/*ctrl/sts*/); \
							while (spi_tgt_done) ; spi_tgt_done = true; \
							*pth = (uint16_t)hi; /*high byte*/ \
							spi_tx_transfer(pth, 1, &tmpw, 1, 0/*ctrl/sts*/); \
							while (spi_tgt_done) ; spi_tgt_done = true; \
							*pth = (uint16_t)lo; /*low byte*/ \
							spi_tx_transfer(pth, 1, &tmpw, 1, 0/*ctrl/sts*/); \
							while (spi_tgt_done) ; \
						  /*delay_us(1);*/ \
						  pio_set(PIOA, CPLD_2072_TRIG);
	#define READ_MID_PROC_2072 \
							pio_configure(PIOB, PIO_OUTPUT_1, PIO_PB0, 0); \
						/*delay_cycles(0);*/ \
						  pio_clear(PIOB, PIO_PB0); \
						  pio_set_peripheral(PIOB, PIO_PERIPH_A, PIO_PB0); \
						  /*delay_cycles(0);*/ \
 						/*spi_set_clock_phase*/(SPI0, SPI_CHIP_SEL, 0/*captured @ falling, transit @ rising*/); \
 						spi_set_clock_polarity(SPI0, SPI_CHIP_SEL, 1/*clk idle state is high*/);
						/*delay_cycles(0);*/
	#define READ_2072_SPI(adr, hi, lo) \
							*pth = 0x80| (0x7f&adr); /*read access*/ \
							spi_tx_transfer(pth, 1, &tmpw, 1, 0); \
							while (spi_tgt_done); spi_tgt_done = true; \
							READ_MID_PROC_2072 \
							spi_rx_transfer(pth, 1, &tmpw, 1, 0/*ctrl/sts*/); /*high byte*/ \
							while (spi_tgt_done) ; spi_tgt_done = true; \
							hi = 0xff & tmpw; \
							spi_rx_transfer(pth, 1, &tmpw, 1, 0/*ctrl/sts*/); /*low byte*/ \
							while (spi_tgt_done) ; \
							lo = (0xff & tmpw);
	#define READ_END_REV_2072 \
						/*spi_set_clock_phase*/(SPI0, SPI_CHIP_SEL, 1/*captured @ rising, transit @ falling*/); \
						 pio_set(PIOA, CPLD_2072_TRIG); \
 						spi_set_clock_polarity(SPI0, SPI_CHIP_SEL, 0/*clk idle state is high*/);
#endif
typedef enum {
	RF2072_RESET=  0, // lastly added for rf2072 reset
	IT951X_READ = 1,
	IT951X_WRITE= 2,
	RF2072_READ=  3,
	RF2072_WRITE= 4,
	IT951X_DOWNLOAD = 5,
	TS_VID_ACTIVE = 6,
} access_mode;

typedef struct {
	int8_t error;  // error status returned
	/*enum access_mode*/uint8_t access;
	uint8_t dcnt;	// data count
	uint16_t addr;	// first access address
	int8_t reserved[3];	// padded to align on 32 bit boundary
	uint8_t data[1];
} dev_access;

typedef struct {
	uint32_t page_size;
	uint32_t ICB_start;
	uint32_t ICB_size;
	uint32_t UFM_start;
	uint32_t UFM_size;
	uint32_t CFM0_start;
	uint32_t CFM0_size;
} cpld_flash_map;

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

#define Cmd_buildCommand(command, processor, chip)  (command + (uint16_t) (processor << 12) + (uint16_t) (chip << 12))
#define Command_SCATTER_WRITE           0x0029

#define FW_DNLD_SIZE		8*1024
  #define ITE_BUFFER_SIZE					    255
  #define USB_ITE_MSG_IDX						/*0x0*/0x1 // comm intf included
  #define USB_ITE_FW_VAL							0x6
  #define ITE_FW_HDR_LEN						(4) // total firmware byte length

//#define FWM_DNLD_DBG		// don't turn in on unless it's necessary, the last chunk of data would not match but it is ok...
#define FW_DNLD_SIZE			7200/*15000*/ // NEVER be divisible into 64 or usb core shall malfunction (a big bug!!!)
#define FW_UPGRADE_HDR_LEN		(12)
#define USB_BOOT_APP_VAL						0xa4
#define USB_CPLD_UPGRADE_VAL	0x21  // cpld upgrade cmd
#define USB_ATMEL_UPGRADE_VAL	0x23	// atmel upgrade cmd
/** ite 951x to host irq assignment */
#define LO_Frequency 								/*1583000*/ 1693000
	#define VID_CH_BW							6000
	#define VID_CH_TTL							(VID_CH_BW+1000) // include guard band
	#define MAX_VID_CH_F					2478000
	#define MIN_VID_CH_F					2406000
	#define NUM_OF_VID_CH			((MAX_VID_CH_F-MIN_VID_CH_F)/VID_CH_TTL)
 #if NUM_OF_VID_CH < 3
	#error Number of video channels cannot be less than 3
 #endif
	#define VID_IF_CH_BASE				(MIN_VID_CH_F-LO_Frequency)

  /** TWI Bus Clock 100kHz */
  #define TWI_CLK     /*200000*/ 100000
  #define TWI_ERR_PROT_RD		/*5*/0 // extra byte read to protect against i2c traffic errors, too risky...
#define ITE_HOST_INT			PIO_PA26 // not used at all...
typedef struct {
	uint32_t it951x_access;
	bool i2c_done_wr;
	uint32_t it951x_err_wr;
	bool i2c_done_rd;
	uint32_t it951x_err_rd;
	bool i2c_overrun;
} context_it951x;	// it951x access on usb/i2c bus

#define _TWI_READ_(adr,buf,len) \
	packet_rx.chip = adr; \
	packet_rx.buffer = (uint8_t *)buf; \
	packet_rx.length = len; \
	twi_sms4470_handler(ID_PIOA, ITE_HOST_INT);
 #define BOARD_ID_TWI_SMS4470         ID_TWI4
 #define BOARD_BASE_TWI_SMS4470       TWI4
#define TWI_WRITE(adr,buf,size) \
	packet_tx.chip = adr; \
	packet_tx.buffer = (uint8_t *)buf; \
	packet_tx.length = size; \
	ctx_951x.i2c_done_wr = false; \
	if ((ctx_951x.it951x_err_wr= \
		twi_master_write(BOARD_BASE_TWI_SMS4470, &packet_tx)) != TWI_SUCCESS) { \
		/*puts("-E-\tTWI master write packet failed.\r");*/ \
		while (1/*don't lock atmel up*/) { \
			/*Configure the options of TWI driver*/ \
			twi_options_t opt; \
			opt.master_clk = sysclk_get_cpu_hz(); \
			opt.speed      = TWI_CLK; \
           twi_master_init(BOARD_BASE_TWI_SMS4470, &opt); \
			break; \
		} \
	} \
	ctx_951x.i2c_done_wr = true;
#define TWI_READ \
	err = twi_master_read(BOARD_BASE_TWI_SMS4470, &packet_rx); \
	if ( err != TWI_SUCCESS ) { \
		/*if(err != TWI_ERROR_TIMEOUT)*/ \
			/*puts("-E-\tTWI master read packet failed.\r");*/ \
			while (0/*don't lock atmel up*/) { \
				; /* Capture error */ \
			} \
         if (err == TWI_ERROR_TIMEOUT) { \
	      	len_prev = packet_rx.length; \
            memcpy(tmp_buff, \
            	packet_rx.buffer, \
               len_prev); \
         } \
         else  { /*nack received*/ \
				/*Configure the options of TWI driver*/ \
				twi_options_t opt; \
				opt.master_clk = sysclk_get_cpu_hz(); \
				opt.speed      = TWI_CLK; \
            twi_master_init(BOARD_BASE_TWI_SMS4470, &opt); \
          } \
	} \
	if (err_prev == TWI_ERROR_TIMEOUT) { \
		/*concatenate two transfer buffers*/ \
		memmove((uint8_t*)packet_rx.buffer+len_prev, \
								(uint8_t*)packet_rx.buffer, \
		                	len0 - len_prev); \
		memcpy(packet_rx.buffer, \
		             tmp_buff, \
		             len_prev); \
	} \
	ctx_951x.it951x_err_rd = err_prev = err; // keep previous err code

typedef struct { // atmel debugger context under mavlink arch
	bool primed/*, active*/;
	bool active;
	uint16_t str_len;
	char *str_buffer;
} ctx_debug_atmel;

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
 #ifdef RADIO_SI4463
   #define SI4463_HOST_INT			PIO_PA29
   #define SI4463_PWRDN			PIO_PA30
 #endif
#define RED_LED					PIO_PB13
#define GREEN_LED				PIO_PB14
#define BLUE_LED				PIO_PB15
extern uint32_t gs_uc_hrbuffer[(USB_HOST_MSG_LEN+HOST_BUFFER_SIZE+3)/sizeof(int)];

uint32_t radio_mon_rxcnt;
volatile bool ctrl_tdma_lock;

#if USE_UART
 #define DBG_BUF_SIZE	/*240*/ 160
  extern void atmel_debugger_dump();
  extern ctx_debug_atmel logger;

#define TRANSFER_LOG \
	if (DBG_BUF_SIZE<=logger.str_len+dbg_len) { \
		int clen = DBG_BUF_SIZE-logger.str_len; \
		memcpy(logger.str_buffer+logger.str_len, dbg_msg, clen); \
		memcpy(dbg_msg, dbg_msg+clen, dbg_len-=clen); \
		logger.str_len = DBG_BUF_SIZE; \
		atmel_debugger_dump(); \
	} \
	else { \
		memcpy(logger.str_buffer+logger.str_len, dbg_msg, dbg_len); \
		logger.str_len += dbg_len; \
		dbg_len = 0; \
	} \
	pdbg = dbg_msg+dbg_len;

#define LOG_BIN_DATA \
	assert(bufferLength<=255); \
	int rem = (1+2)*bufferLength; \
	dbg_len += rem+1; \
	char *pdat = buffer; \
	do { \
		sprintf(pdbg, "%02x ", *pdat); \
		pdbg += 2+1; \
		pdat += 1; \
	} while(0<(rem-=(2+1))); \
	*pdbg = 0; /* null terminate*/ \
	for (i=0; i<(logger.str_len+dbg_len+DBG_BUF_SIZE/2)/DBG_BUF_SIZE; i++) { \
		TRANSFER_LOG \
	}
#endif

extern void usb_write_buf1(void *pb, int size, bool log);
extern int init_video_subsystem(void);
extern int start_video_subsystem(void);

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
