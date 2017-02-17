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

#define GRAND_BUFFER_BLKS    8
#define I2SC_BUFFER_SIZE		10*188
#define GRAND_BUFFER_SIZE	GRAND_BUFFER_BLKS*I2SC_BUFFER_SIZE

 // host usb/spi buffer for ctrl/sts, link up with fpga or 2072, liyenho
#define USB_DEVICE_SPECIFIC_REQUEST()			main_vender_specific()
#define USB_DEVICE_SYSTEM_RESTART()			main_loop_restart()  // restart yuneec design if usb restart, liyenho
 #define USB_SYSTEM_RESTART_VAL				0xa5
 #define USB_ATMEL_VER_VAL								0x20
#define USB_STREAM_OFF_VAL							0xa
#define USB_STREAM_ON_VAL							0xe
  #define USB_QUERY_IDX									0xff	// used to query run time indicator
  #define USB_STREAM_IDX								0x1	// data instead comm interface
  #define USB_STREAM_LEN								0
#define USB_HOST_MSG_TX_VAL						0x5
#define USB_HOST_MSG_RX_VAL						0x9
  #define HOST_BUFFER_SIZE							(256-1)
  #define USB_HOST_MSG_IDX						0x1	// data instead comm interface
  #define USB_HOST_MSG_LEN						sizeof(dev_access)
#define RADIO_SPI_BR								1000000
#define ASYMM_RATIO								/*1*/ 2
#define TDMA_PERIOD    							/*3600000*/ /*3240000*/ 2880000 // see TX ctrl.h for details
#define  TDMA_BOUND								 /*5400000*/ /*4860000*/ 4320000 // 100 ms = 12000000
#define RADIO_PKT_LEN							16 // ctl/sts radio payload byte length
#define RADIO_INFO_LEN      				4 // usb pipe information post header
#define RADIO_GRPPKT_LEN                    30
#define RDO_ELEMENT_SIZE   			(RADIO_PKT_LEN/sizeof(uint32_t))	// RADIO_PKT_LEN must divide into sizeof(uint32_t), liyenho
#define RDO_TPACKET_FIFO_SIZE               8
#define RDO_RPACKET_FIFO_SIZE               8
#define RADIO_SI4463
#define RADIO_CTRL_AUTO 			// downlaod si4463 w/o host config ctrl xfer, liyenho
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
 //#define TEMPERATURE_MEASURE			// created single frequency tx @ base frequency //defined with config file when defined, liyenho
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
  enum pair_mode{
	  IDLE = 0,
	  PAIRING,
	  PAIRED,
  };
 #define BASE_GPS_IDX								0xe
 #define BASE_GPS_LEN								2*sizeof(float)

 /* used to be channel selection table for control radio band plan, now it can serve as lookup table for frequency hopp */
 extern const uint8_t RF_FREQ_CONTROL_INTE_HOP0_6[10] ; // 902.285 mhz
 extern const uint8_t RF_MODEM_AFC_LIMITER_HOP0_3[7] ;
 extern const uint8_t RF_FREQ_CONTROL_INTE_HOP13_6[10] ; // 925.035 mhz
 extern const uint8_t RF_MODEM_AFC_LIMITER_HOP13_3[7];
/* tabulate avaialble ctrl rf bands (hopping channels) */
#define HOPPING_TABLE_SIZE					50
#define WRAP_OFFSET(x) (HOPPING_TABLE_SIZE<=(x))?((x)-HOPPING_TABLE_SIZE):(x)
#define HOP_2CH_ENABLE                      0/*1*/   //debug testing
#define HOP_2CH_OFFSET0                     0
#define HOP_2CH_OFFSET1                     1/*24*/
 extern const uint8_t *chtbl_ctrl_rdo[HOPPING_TABLE_SIZE*2] ;
 #define RF_FREQ_CONTROL_INTE_LEN	sizeof(RF_FREQ_CONTROL_INTE_HOP0_6)
 #define RF_MODEM_AFC_LIMITER_LEN	sizeof(RF_MODEM_AFC_LIMITER_HOP0_3)
 #define CHTBL_SIZE											sizeof(chtbl_ctrl_rdo)/(2*sizeof(*chtbl_ctrl_rdo))
 #define FORWARD_HOP									((CHTBL_SIZE+1) / 10)
 #define BACKWARD_HOP								(FORWARD_HOP-1)
 #define HOP_CHN_SPACE								500000	/*hop channel spacing*/
 #define FREQ_SHIFT_STEP							50000		/*frequency shift step*/
#ifdef SI4463_CRYSTAL_32MHZ
 #define FREQ_CTRL_FACTOR						((4*524288.0)/(2*32*1000000.0))
#else  // 30 mhz crystal old design
 #define FREQ_CTRL_FACTOR						((4*524288.0)/(2*30*1000000.0))
#endif
 #define NUM_FREQ_SHIFT							(HOP_CHN_SPACE / FREQ_SHIFT_STEP)
 #define FREQ_FRAC_POS								5 // zero based
 #define FREQ_INTR_POS									4	// zero based
  #define CAP_VAL_POS										0x4
  #define CAP_TUNE_THR									7	// above halfway of 20 ms/pk in 200 ms
  #define CALIB_DWELL_INTV						24000000	// eval each cap value in 200 ms
  #define RECALIB_INTV_F								240000000L // recheck/recalibrate every 2 sec
  #define TEMP_DEL_THR									3		// adjust cap value if temp delta is greater than 3 degree
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
	typedef struct si4463_capv_tune {
		bool calib_req_h;
		bool calib_gated;
		uint32_t calib_det_rx; // to tell if any rx activity
		uint8_t cap_curr;
		uint8_t lower;
		uint8_t upper;
		uint8_t median;
		uint32_t tm_started; // sampled calib eval start time
		uint32_t tm_curr;
		uint32_t tm_ended; // sampled calib eval end time
	} capv_tune_t;
#endif
#define RFFE_PARAMS
typedef enum {
	RF2072_RESET=  0, // lastly added for rf2072 reset
	IT913X_READ = 1,
	IT913X_WRITE= 2,
	RF2072_READ=  3,
	RF2072_WRITE= 4,
	IT913X_DOWNLOAD = 5,
	TS_VID_ACTIVE = 6,
} access_mode;

typedef struct {
	/*enum access_mode*/uint8_t access;
	uint8_t dcnt;	// data count
	uint16_t addr;	// first access address
	uint8_t data[1];
} dev_access;

 #define CONFIG_RF2072 // enable rfmd 2072 config
 #ifdef RFFE_PARAMS
 	#define CONFIG_RF2072 // forced to configure rf2072 prior to startup
 #endif
 #define RX_SPI_CHAINING       // enable spi chaining
 #define RECV_IT913X
 //#ifdef CTRL_DYNAMIC_MOD
#ifdef CONFIG_RF2072
	#define CPLD_2072_TRIG    PIO_PA25
	#define ACCESS_PROLOG_2072 \
						  pio_set(PIOA, PIO_PA23); \
						/*delay_cycles(0);*/ \
							pio_clear(PIOA, CPLD_2072_TRIG); \
						/*delay_cycles(0);*/ \
						  pio_clear(PIOA, PIO_PA23); \
						/*delay_us(1);*/ /*gap active cs from 1st clk*/ \
						  pio_set(PIOA, PIO_PA23); \
						/*delay_cycles(0);*/ \
						  pio_clear(PIOA, PIO_PA23);
	#define READ_MID_PROC_2072 \
						  pio_set(PIOA, PIO_PA23); \
						/*delay_cycles(0);*/ \
						  pio_clear(PIOA, PIO_PA23); \
 						 spi_set_clock_phase(SPI0, SPI_CHIP_SEL, 0/*captured @ falling, transit @ rising*/); \
						/*delay_cycles(0);*/
	#define READ_END_REV_2072 \
						 spi_set_clock_phase(SPI0, SPI_CHIP_SEL, 1/*captured @ rising, transit @ falling*/); \
						 pio_set(PIOA, CPLD_2072_TRIG);
#endif
 #ifdef RADIO_SI4463
 #define SI4463_HOST_INT			PIO_PB8
 #define SI4463_PWRDN			PIO_PA15
 #endif
#define	DVBT	0
#define	DVBT2	1
 #define MODE DVBT  // start with dvbt mode first, liyenho

#define Cmd_buildCommand(command, processor, chip)  (command + (uint16_t) (processor << 12) + (uint16_t) (chip << 12))
#define Command_SCATTER_WRITE           0x0029
typedef enum {
    Processor_LINK = 0,
    Processor_OFDM = 8
} Processor;

#define FW_DNLD_SIZE		8*1024
  #define ITE_BUFFER_SIZE					    255
  #define USB_ITE_MSG_IDX						/*0x0*/0x1 // comm intf included
  #define USB_ITE_FW_VAL							0x6
  #define ITE_FW_HDR_LEN						(4) // total firmware byte length

typedef struct {
	uint32_t page_size;
	uint32_t ICB_start;
	uint32_t ICB_size;
	uint32_t UFM_start;
	uint32_t UFM_size;
	uint32_t CFM0_start;
	uint32_t CFM0_size;
} cpld_flash_map;

#define FW_UPGRADE_HDR_LEN		(12)
#define USB_BOOT_APP_VAL						0xa4
#define USB_CPLD_UPGRADE_VAL	0x21  // cpld upgrade cmd
#define USB_ATMEL_UPGRADE_VAL	0x23	// atmel upgrade cmd

/** ite 913x to host irq assignment */
#define ITE_HOST_INT			PIO_PA26 // not used at all...
#define _TWI_READ_(adr,buf,len) \
	packet_rx.chip = adr; \
	packet_rx.buffer = (uint8_t *)buf; \
	packet_rx.length = len; \
	i2c_read_done = false; \
	twi_sms4470_handler(ID_PIOA, ITE_HOST_INT);
 #define BOARD_ID_TWI_SMS4470         ID_TWI4
 #define BOARD_BASE_TWI_SMS4470       TWI4
#define TWI_WRITE(adr,buf,size) \
	packet_tx.chip = adr; \
	packet_tx.buffer = (uint8_t *)buf; \
	packet_tx.length = size; \
	if (twi_master_write(BOARD_BASE_TWI_SMS4470, &packet_tx) != TWI_SUCCESS) { \
		/*puts("-E-\tTWI master write packet failed.\r");*/ \
		while (1) { \
			; /* Capture error */ \
		} \
	}
#define TWI_READ \
	err = twi_master_read(BOARD_BASE_TWI_SMS4470, &packet_rx); \
	if ( err != TWI_SUCCESS ) { \
		/*if(err != TWI_ERROR_TIMEOUT)*/ \
			/*puts("-E-\tTWI master read packet failed.\r");*/ \
			while (1) { \
				; /* Capture error */ \
			} \
	}
// redefine twi_packet type for callback feature
typedef void (*cb_task_t) ();
typedef struct twi_packet_cb {
	//! TWI address/commands to issue to the other chip (node).
	uint8_t addr[3];
	//! Length of the TWI data address segment (1-3 bytes).
	uint32_t addr_length;
	//! Where to find the data to be transferred.
	void *buffer;
	//! How many bytes do we want to transfer.
	uint32_t length;
	//! TWI chip address to communicate with.
	uint8_t chip;
	//! iteration count for callback invocation, liyenho
	uint32_t iters_cb ;
	//! added callback ext for multitask cap, liyenho
	cb_task_t callback;
} twi_packet_cb_t;
extern uint32_t twi_master_cb_read(Twi *p_twi, twi_packet_cb_t *p_packet) ;
// support callback feature on i2c access
#define TWI_READ_CB \
	err = twi_master_cb_read(BOARD_BASE_TWI_SMS4470, &packet_rx); \
	if ( err != TWI_SUCCESS ) { \
		/*if(err != TWI_ERROR_TIMEOUT )*/ \
			/*puts("-E-\tTWI master read packet failed.\r");*/ \
			while (1) { \
				; /* Capture error */ \
			} \
	}

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
