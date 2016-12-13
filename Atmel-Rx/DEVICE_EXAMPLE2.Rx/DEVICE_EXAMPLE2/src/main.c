/* Digibest Reference Code: Changes:  */
// 1. DigiBestSms4470Core.c: change #define MAX_CHUNK_SIZE        (8*1024)
// 2. SMS4470_A2_DVBT_MRC_Firmware_(2.0.0.47).h: UINT8 SianoSMS4470_DVBT_A2_FirmwareImage[16]=...
//    disable UINT8 SianoSMS4470_DVBT_A2_FirmwareImaget[SMS4470_DVBT_FIRMWARE_CODE_SIZE]=... (add extra "t")
// 3. DigiBestPlatformPorting.h: extern unsigned int* fw_sms_rbuffer;
// 4. SMS4470_single_mode_download_firmware_data: add many special lines.
/**
 * \file
 *
 * \brief CDC Application Main functions
 *
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <string.h>
#include <asf.h>
#include "conf_usb.h"
#include "conf_example.h"
#include "ui.h"
#include "uart.h"
#include "delay.h"
#include "DigiBestDefine.h"
#include "DigiBestFrontend.h"
#define INCLUDEINMAIN  // for Kevin's ctrl design
#include "si4463/bsp.h"
#include "si4463/lgdst_4463_spi.h"
#if (SAMG55)
#include "flexcom.h"
#endif

static volatile bool system_main_restart = false;  // system restart flag, liyenho
static volatile uint8_t system_upgrade = 0;  // system upgrade flag, liyenho
/*static*/ char __version_atmel__[3];  // stored as mon/day/year
#ifdef RADIO_SI4463
  #include "si4463/si446x_nirq.h"
  #include "si4463/radio.h"
#endif
#include "GPS.h"
#include "sms.h"
 extern SMS4470_DEVICE_GROUP_OPERATION_INFORMATION dvbt; // extern from sms.c
  #define SPI0_Handler     FLEXCOM0_Handler
  #define SPI_Handler     FLEXCOM5_Handler
  #define SPI0_IRQn        FLEXCOM0_IRQn
  #define SPI_IRQn        FLEXCOM5_IRQn
// extra SPI port for additional data chip on Tx side, not configured yet, liyenho
#define SPI2_Handler     FLEXCOM2_Handler
#define SPI2_IRQn        FLEXCOM2_IRQn
// extra SPI port for additional data chip on Rx side, not configured yet, liyenho
#define SPI7_Handler     FLEXCOM7_Handler
#define SPI7_IRQn        FLEXCOM7_IRQn
volatile uint8_t main_loop_on = false; // run time indicator
volatile uint8_t usb_data_done = false;  // workaround flooding i2s intr issue, liyenho
volatile uint8_t usb_host_msg = false, spi_tgt_done=false; // triggered by usb rx isr for host (fpga/sms) comm, liyenho
volatile uint8_t spidma_active = false;
volatile uint8_t spibuff_wrptr_filled, spibuff_rdptr;
volatile uint8_t spibuff_wrptr_currentlyfilling;
volatile uint8_t spibuff_wrptr_yettobefilled;
#ifdef RADIO_SI4463
  volatile uint8_t spi_radio_done = false;
  volatile uint8_t spi_dma_mode = false;
#endif
volatile bool set_rf_params= false;
#ifdef CONFIG_ADI_6612
  volatile uint32_t num_6612_regs = 0;
#endif
volatile uint32_t fc_siano_tuned = 482000000;
static volatile bool main_b_cdc_enable = false;
  /** TWI Bus Clock 400kHz */
  // slow down to see if it improve data integrity, it does, liyenho
  #define TWI_CLK     /*400000*/ 200000
  /** The address for TWI SMS4470 */
  #define SMS4470_ADDRESS        0x68 /*0xd0*/
  /* still need to confirm on these..., done */
  #define SMS_REG_ADDR         0
  #define SMS_REG_ADDR_LENGTH  /*2*/ 0
/* Chip select. */
#define SPI_CHIP_SEL 0
/* Clock polarity. */
#define SPI_CLK_POLARITY /*0*/ 1
/* Clock phase. */
#define SPI_CLK_PHASE 0
/* Delay before SPCK. */
#define SPI_DLYBS /*0x40*/ 0x10
/* Delay between consecutive transfers. */
#define SPI_DLYBCT /*0x10*/ 0x0
 // 10 TS packet per ping/pong buffer
//#define I2SC_BUFFER_SIZE		10*188
/* UART baudrate. */
#define UART_BAUDRATE      115200
/* SPI clock setting (Hz). */
  static uint32_t gs_ul_spi_clock[2+1] = {  // shall be higher than usb data rate
  				  	2000000, 8000000, RADIO_SPI_BR}; // (ctrl/sts, video, radio) spi bit rate
volatile uint32_t statusflags = 0;
extern volatile bool udi_cdc_data_running; // from udi_cdc.c, liyenho
#ifdef  RADIO_SI4463
  volatile bool si4463_radio_started = false; // radio startup flag...
  volatile bool ctrl_tdma_enable = false,
  								ctrl_tdma_enable_shdw;  // shadow of ctrl_tdma_enable
  volatile capv_tune_t si4463_factory_tune;
  volatile bool ctrl_tdma_lock = false;  //always, since RX is master
  //volatile bool ctrl_pkt_rd_available = false;
	static int timedelta(bool reset, unsigned int bignum, unsigned int smallnum);
	static bool fhop_dir, timedelta_reset ; //to handle system restart
	/*static*/ uint8_t hop_id[HOP_ID_LEN]; // 10 byte hop id from host
	static int fhop_idx= 0, fhop_offset = HOP_2CH_ENABLE?WRAP_OFFSET(HOP_2CH_OFFSET0):0; //to produce frequency hopping sequence
	static int fhop_base=0, fhopless= 0/*can be 1, 2 or 3*/;
  /*static*/ enum pair_mode hop_state;
  	volatile unsigned char ynsdbyte ; //Take care in Rate Control Section!!! liyenho
#endif
volatile RECEPTION_STATISTICS_ST recptStatG; //legacy: for liyen use
volatile RECEPTION_STATISTICS_ST recptStatG_m;
volatile RECEPTION_STATISTICS_ST recptStatG_s;
volatile Short_Statistics_ST shortStatG;
unsigned int log2a[100];
unsigned char log2p =0;
volatile bool stream_flag = true; // default to TS stream automatic on
#ifdef CONFIG_ON_FLASH
	uint32_t ul_page_addr_ctune =IFLASH_ADDR + IFLASH_SIZE - NUM_OF_ATMEL_REGS, // 1st atmel reg on flash
						// temperature @ current tuning @ 2nd atmel reg on flash
					ul_page_addr_mtemp =IFLASH_ADDR + IFLASH_SIZE - NUM_OF_ATMEL_REGS + 1,
				ul_page_addr_bootapp =IFLASH_ADDR + IFLASH_SIZE - NUM_OF_ATMEL_REGS + 2;
uint8_t backup[NUM_OF_FPGA_REGS+NUM_OF_ATMEL_REGS+4+1],
				bootapp = (uint8_t)-1;
#endif
 #ifdef MEDIA_ON_FLASH
  #ifdef NO_USB
  	#undef PAGE_ADDRESS
  	#include "ts-data.h"
  	static int ts_inc = 10;	// from 1 (1 out of 10 is real packet) to 10 (all are real packets), liyenho
volatile int media_file_len = TS_PACKETS*188;
volatile bool usb_load_media = true;
  #else
volatile int media_file_len = 0; // fill in from callback, liyenho
  		static uint32_t page_addr_mlen = IFLASH_ADDR + IFLASH_SIZE -
											(NUM_OF_FPGA_REGS+ NUM_OF_ATMEL_REGS) -4/*sizeof(int)*/;
volatile bool usb_load_media = false ; // called back from udc.c, liyenho
  #endif
 #endif
extern udd_ctrl_request_t udd_g_ctrlreq; // from udp_device.c, liyenho
volatile uint32_t g_ul_10ms_ticks=0, g_ul_wait_10ms=0;
/*static*/ bool systick_enabled = false;  // default to disable sys timer
volatile static uint32_t g_ul_led_ticks=0, g_ul_wait_100ms=10, g_ul_wait_1s=100;
static bool health_led_onoff = false;
volatile bool usb_write_start = false ; // called back from udc.c, liyenho
#ifdef RECV_SMS4470
	uint32_t gs_uc_rbuffer[GRAND_BUFFER_SIZE/sizeof(int)];
 #ifndef RX_SPI_CHAINING
	typedef struct state_write_usb {
		uint32_t usb_failcnt;
		bool skip_rd ;
		bool skip_wr ;
		uint8_t *buf_start;
		uint8_t *buf_last;
		uint8_t *buf_end;
		uint8_t *spi_buf_rd;
		uint8_t *usb_buf_wr;
	} state_write_usb_t;
	// added for spi/usb callback monitor, liyenho
	state_write_usb_t sms4470_usb_ctx ;
  	bool i2c_read_cb_on = false; // adopt cb ext, liyenho
 #endif
#else // non-real time mode
	uint32_t gs_uc_rbuffer[2*I2SC_BUFFER_SIZE/sizeof(int)];
#endif
static uint32_t gs_uc_tbuffer[2*I2SC_BUFFER_SIZE/sizeof(int)];
volatile static uint32_t usbfrm = 0, prev_usbfrm= 0;
 //algorithm from digibest sdk, using bulk transfer pipe, liyenho
volatile uint32_t upgrade_fw_hdr[FW_UPGRADE_HDR_LEN/sizeof(int)]={-1} ;
#if defined(FWM_DNLD_DBG)
  volatile bool usb_host_active = false;
  extern volatile bool usb_tgt_active ;
#endif
 // host to fpga/6612 ctrl/sts buffer
static uint32_t gs_uc_htbuffer[(USB_HOST_MSG_LEN+HOST_BUFFER_SIZE+3)/sizeof(int)];
static uint32_t gs_uc_hrbuffer[(USB_HOST_MSG_LEN+HOST_BUFFER_SIZE+3)/sizeof(int)];
  volatile bool i2c_read_done = false;
  // sms4470 operation state
  /*static*/ Sms4470_State sms_state = 0;
  // sms4470 i2c packet context
  /*static*/ twi_packet_t packet_tx;
#ifdef RX_SPI_CHAINING
  twi_packet_t packet_rx;
#else //!RX_SPI_CHAINING
  twi_packet_cb_t packet_rx; // with cb ext, liyenho
#endif
  // sms4470 i2c device information
  static smsi2c_device smsdev;
   // host/sms ctrl/sts buffer
   volatile uint32_t sms4470_fw_hdr[3+1]={-1} ; //algorithm from digibest sdk, liyenho
   /*static*/ uint32_t gs_sms_tbuffer[(USB_SMS_MSG_LEN+SMS_BUFFER_SIZE+3)/sizeof(int)];
   /*static*/ uint32_t gs_sms_rbuffer[(USB_SMS_MSG_LEN+SMS_BUFFER_SIZE+3)/sizeof(int)];
 #if true  // add two big memory chunks for sms fw download
   /*static*/ uint32_t fw_sms_tbuffer[FW_DNLD_SIZE/sizeof(int)];
   /*static*/ uint32_t fw_sms_rbuffer[FW_DNLD_SIZE/sizeof(int)];
 #endif
/* Pointer to UART PDC register base */
Pdc *g_p_spim_pdc [1+2]/*fpga/sms, video, maybe more later, liyenho*/,*g_p_spis_pdc;
Pdc *g_p_i2st_pdc, *g_p_i2sr_pdc;
struct i2s_dev_inst dev_inst_i2s;
#ifdef RADIO_SI4463
 static uint8_t tune_cap_str[] = {RF_GLOBAL_XO_TUNE_2}; // used by cap val tuning process internally

volatile uint32_t gs_rdo_tpacket[RDO_TPACKET_FIFO_SIZE*RDO_ELEMENT_SIZE] = {0xffffffff};
volatile uint32_t tpacket_idle[ASYMM_RATIO* RDO_ELEMENT_SIZE];
unsigned char tpacket_grp[RADIO_GRPPKT_LEN];
unsigned int *gp_rdo_tpacket_l = ((unsigned int*)gs_rdo_tpacket);
unsigned int *gp_rdo_tpacket = (unsigned int*)gs_rdo_tpacket;
unsigned char gs_rdo_tpacket_ovflw=0;
volatile uint32_t wrptr_rdo_tpacket=RDO_TPACKET_FIFO_SIZE-1,   //wrptr to valid data
                  rdptr_rdo_tpacket=RDO_TPACKET_FIFO_SIZE-1;   //rdptr to last consummed data, this
				                                                   //rdptr=wrptr, data consummed, rdptr take priority
unsigned char snd_asymm_cnt=0;
unsigned int  snd_asymm_rdptr=0;

volatile uint32_t gs_rdo_rpacket[RDO_RPACKET_FIFO_SIZE*RDO_ELEMENT_SIZE] = {0xffffffff};
unsigned int *gp_rdo_rpacket = (unsigned int*)gs_rdo_rpacket;
unsigned int *gp_rdo_rpacket_l = ((unsigned int*)gs_rdo_rpacket);
volatile uint32_t rpacket_idle[RDO_ELEMENT_SIZE];
uint32_t rpacket_ov[RDO_ELEMENT_SIZE];
unsigned char rpacket_grp[RADIO_GRPPKT_LEN+RADIO_INFO_LEN]; // fixed
unsigned char rpacket_grp_partial[RADIO_GRPPKT_LEN];
unsigned char gs_rdo_rpacket_ovflw=0;
volatile uint32_t wrptr_rdo_rpacket=RDO_RPACKET_FIFO_SIZE-1,   //wrptr to valid data
                  rdptr_rdo_rpacket=RDO_RPACKET_FIFO_SIZE-1;  //rdptr to last consummed data
                                                                        //rdptr=wrptr, data consummed, rdptr take priority

	unsigned int  radio_mon_txfailcnt=0;
	unsigned int  radio_mon_rxcnt = 0;
	unsigned int  radio_mon_txidlecnt = 0;
	unsigned int  radio_mon_txcnt = 0;
	unsigned char radio_mon_rcvjitter=0;

/*******************************************************************/
//const unsigned int initPeriod = 3000; // startup period in msec, m.a. was in fast tracking pace
#define initCnt		30
const unsigned int initWgt = 8,
									initScl = 3,
									Ewis = 3,
									Ewim = 8-3;
//const unsigned int intePeriod = 30000; // intermediate period in msec, m.a. was in normal tracking pace
#define inteCnt		200
const unsigned int inteWgt = 16,
									inteScl = 4,
									Ewms = 5,
									Ewmm = 16-5;
const unsigned int normWgt = 16,
									normScl = 4,
									Ewns = 3,
									Ewnm = 16-3;
const unsigned int lgThr = 3, shThr= 1;
const uint32_t error_weight_cks6b[] __attribute__((aligned(8)))= {
	0, 1, 1, 2, 1, 2, 2, 3,
	1, 2, 2, 3, 2, 3, 3, 4,
	1, 2, 2, 3, 2, 3, 3, 4,
	2, 3, 3, 4, 3, 4, 4, 5,
	1, 2, 2, 3, 2, 3, 3, 4,
	2, 3, 3, 4, 3, 4, 4, 5,
	2, 3, 3, 4, 3, 4, 4, 5,
	3, 4, 4, 5, 4, 5, 5, 6,
} ;
#define CTRL_MON_PERIOD	300  // in msec
#define CTRL_FAIL_PERIOD	(5* CTRL_MON_PERIOD) // can't be too short in order to prevent spi comm lockup
#define CTRL0_MSK			(-1+(1<<CTRL_BITS))
#define CTRL0_IMSK			(0xff & ~CTRL0_MSK)
#define CTRL_MSK				(CTRL0_MSK<<CTRL_BITS)
#define CHKSM_MSK		(0xff ^ CTRL0_MSK)
// 4463 stats mon obj
volatile ctrl_radio_stats  r4463_sts;
/*******************************************************************/
#endif //RADIO_SI4463

ULONG TwoDiversityGroupFrontendID;
ULONG FrequencyLoop;
TUNING_PARAMETER TuningParameter;
BOOL TwoDiversityGroupFrontendChannelLockStatus;
STATISTICS_INFORMATION TwoDiversityGroupFrontendChannelStatistics;
unsigned char trig500ms=0;

/* monitoring / tracking */
//production support
volatile uint8_t mon_ts47bad_cnt;
volatile uint8_t mon_spidmachainfail_cnt;

//engineering debug support
volatile uint32_t *DWT_CONTROL = (uint32_t *)0xE0001000;
volatile uint32_t *DWT_CYCCNT = (uint32_t *)0xE0001004;
volatile uint32_t *DEMCR = (uint32_t *)0xE000EDFC;
unsigned char dbg_spififo_lvl_max=0;
volatile uint32_t ctrl_sndflag = 100;

// function prototyping
uint32_t wrptr_inc(uint32_t *wrptr,  uint32_t *rdptr, uint32_t fifodepth, int step);
uint32_t rdptr_inc(uint32_t *wrptr,  uint32_t *rdptr, uint32_t fifodepth, int step);
uint32_t fifolvlcalc(uint32_t wrptr, uint32_t rdptr, uint32_t fifodepth);

/**
 *  \brief Configure the Console UART.
 */
static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
#ifdef CONF_UART_CHAR_LENGTH
		.charlength = CONF_UART_CHAR_LENGTH,
#endif
		.paritytype = CONF_UART_PARITY,
#ifdef CONF_UART_STOP_BITS
		.stopbits = CONF_UART_STOP_BITS,
#endif
	};

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}

/**
 * \brief Set SPI slave transfer.
 *
 * \param p_buf Pointer to buffer to transfer.
 * \param size Size of the buffer.
 */
void spi_rx_transfer(void *p_tbuf, uint32_t tsize, void *p_rbuf,
		uint32_t rsize, uint32_t ch)
{
	uint32_t base, spi_ier;
	pdc_packet_t pdc_spi_packet;

	pdc_spi_packet.ul_addr = (uint32_t)p_rbuf;
	pdc_spi_packet.ul_size = rsize;
	pdc_rx_init(g_p_spim_pdc [ch], &pdc_spi_packet, NULL);

	pdc_spi_packet.ul_addr = (uint32_t)p_tbuf;
	pdc_spi_packet.ul_size = tsize;
	pdc_tx_init(g_p_spim_pdc [ch], &pdc_spi_packet, NULL);

	/* Enable the RX and TX PDC transfer requests */
	pdc_enable_transfer(g_p_spim_pdc [ch], PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);
	/* Transfer done handler is in ISR */
	spi_ier = SPI_IER_RXBUFF;
	base = (1==ch)? SPI_MASTER_BASE: SPI0_MASTER_BASE;
 #ifdef  RADIO_SI4463
 if (2==ch) base = SPI7_MASTER_BASE;
 #endif
	spi_enable_interrupt(base, spi_ier) ;
}
#ifdef RECV_SMS4470
	static void spi_rx_transfer_ts(void *p_rbuf, uint32_t rsize, void *p_rbuf_n, uint32_t rsize_n, uint32_t ch)
	{
		uint32_t base, spi_ier;
		pdc_packet_t pdc_spi_packet, pdc_spi_packet_n;

		pdc_spi_packet.ul_addr = (uint32_t)p_rbuf;
		pdc_spi_packet.ul_size = rsize;
		pdc_spi_packet_n.ul_addr = (uint32_t)p_rbuf_n;
		pdc_spi_packet_n.ul_size = rsize_n;

		pdc_rx_init(g_p_spim_pdc [ch], &pdc_spi_packet, &pdc_spi_packet_n);

		/* Enable the RX and TX PDC transfer requests */
		pdc_enable_transfer(g_p_spim_pdc [ch], PERIPH_PTCR_RXTEN);
		/* Transfer done handler is in ISR */
		spi_ier = SPI_IER_RXBUFF;
		spi_enable_interrupt(SPI_MASTER_BASE, spi_ier) ;
	}
#endif
void SPI0_Handler(void) // 6612 ctrl spi
 {
	uint32_t status;
	spi_disable_interrupt(SPI0_MASTER_BASE, SPI_IER_TXBUFE) ;
	spi_disable_interrupt(SPI0_MASTER_BASE, SPI_IER_RXBUFF) ;

	spi_tgt_done = false;  // handshake with main loop
	status = spi_read_status(SPI0_MASTER_BASE) ;

	//if(status & SPI_SR_NSSR) {
		if ( status & SPI_SR_TXBUFE ) {
//			printf("transfer done\n");
		}
	//}
 }
 #ifdef RADIO_SI4463
 void SPI7_Handler(void) // si4463 ctrl/sts/data spi
 {
	 uint32_t status;
	 spi_disable_interrupt(SPI7_MASTER_BASE, SPI_IER_TXBUFE) ;
	 spi_disable_interrupt(SPI7_MASTER_BASE, SPI_IER_RXBUFF) ;

	 spi_radio_done = false;  // handshake with main loop
	 status = spi_read_status(SPI7_MASTER_BASE) ;

	 //if(status & SPI_SR_NSSR) {
	 if ( status & SPI_SR_TXBUFE ) {
		 //			printf("transfer done\n");
	 }
	 //}
 }
 #endif
void SPI_Handler(void)  // video spi pipe
{
	uint32_t status;
	spi_disable_interrupt(SPI_MASTER_BASE, SPI_IER_RXBUFF) ;
	spidma_active = false;  // handshake with mainloop
	mon_spidmachainfail_cnt++;
	status = spi_read_status(SPI_MASTER_BASE) ;
	//if(status & SPI_SR_NSSR) {
		if ( status & SPI_SR_TXBUFE ) {
//			printf("transfer done\n");
		}
	//}
#ifndef RX_SPI_CHAINING
	if (i2c_read_cb_on) {
		sms4470_usb_ctx.usb_failcnt += 1;
		sms4470_usb_ctx.spi_buf_rd -= I2SC_BUFFER_SIZE; // equivalent to drop a block to align
		if (sms4470_usb_ctx.buf_start>sms4470_usb_ctx.spi_buf_rd)
			sms4470_usb_ctx.spi_buf_rd = sms4470_usb_ctx.buf_end;
		sms4470_usb_ctx.skip_rd = true;
		pio_clear(PIOB, PIO_PB9); // disable TS gate
	}
#endif
}
void twi_sms4470_handler(const uint32_t id, const uint32_t index);
/* SMS4470 intr handler for i2c intf */
  void twi_sms4470_handler(const uint32_t id, const uint32_t index)
	{
		if ((id == ID_PIOA) && (index == SMS_HOST_INT)){
			uint32_t err; // error code requried
#ifndef RX_SPI_CHAINING
			if (i2c_read_cb_on) {
				TWI_READ_CB
			} else
#endif
			{	TWI_READ	}
			i2c_read_done = true;
		}
	}
/**
 * \brief Initialize TWI as master.
 */
static void twi_master_initialize(uint32_t speed)
{
	/* Enable the peripheral and set TWI mode. */
	flexcom_enable(BOARD_FLEXCOM_TWI);
	flexcom_set_opmode(BOARD_FLEXCOM_TWI, FLEXCOM_TWI);
	/* Configure the options of TWI driver */
	twi_options_t opt;

	opt.master_clk = sysclk_get_cpu_hz();
	opt.speed      = /*TWI_CLK*/speed;

	/* Configure the data packet to be transmitted */
	packet_tx.chip        = SMS4470_ADDRESS;
	packet_tx.addr[0]     = SMS_REG_ADDR >> 8;
	packet_tx.addr[1]     = SMS_REG_ADDR;
	packet_tx.addr_length = SMS_REG_ADDR_LENGTH;
	packet_tx.buffer      = (uint8_t *) gs_sms_tbuffer;
	packet_tx.length      = sizeof(sms_access);

	/* Configure the data packet to be received */
	packet_rx.chip        = packet_tx.chip;
	packet_rx.addr[0]     = packet_tx.addr[0];
	packet_rx.addr[1]     = packet_tx.addr[1];
	packet_rx.addr_length = packet_tx.addr_length;
	packet_rx.buffer      = (uint8_t *) fw_sms_rbuffer;
	packet_rx.length      = sizeof(fw_sms_rbuffer);
	/***************************************/
#ifndef RX_SPI_CHAINING
	packet_rx.iters_cb = 0.0006/*0.6 ms callback intv*/*speed/10/*10 bit per byte xfer*/;
	packet_rx.callback = usb_write_buf_cb;  // usb write cb which also monitor spi siano xfer stats
#endif
	if (twi_master_init(BOARD_BASE_TWI_SMS4470, &opt) != TWI_SUCCESS) {
		//puts("-E-\tTWI master initialization failed.\r");
#if SAM3XA
		LED_On(LED0_GPIO);
		LED_On(LED1_GPIO);
#endif
		while (1) {
			/* Capture error */
		}
	}
}
/**
 *  \brief Handler for System Tick interrupt.
 *
 *  Process System Tick Event
 *  increments the timestamp counter.
 */
#ifdef  RADIO_SI4463
/*****************************************************************************
 *  si4464 definitions and Global Variables
 *****************************************************************************/
  /*const*/ COMPILER_ALIGNED(8) tRadioConfiguration RadioConfiguration = RADIO_CONFIGURATION_DATA;
  /*const*/ tRadioConfiguration *pRadioConfiguration = &RadioConfiguration;
  SEGMENT_VARIABLE(bMain_IT_Status, U8, SEG_XDATA);
 #if true
  static int hop_chn_sel(int offset) {
	  if (fhopless) {
		  switch(fhopless) {
			  case 1/*low*/: return 0;
			  case 2/*mid*/: return (CHTBL_SIZE)/2-1;
			  case 3/*high*/: return (CHTBL_SIZE)-1;
			  default: /* Capture error */
					while (1) {
						;
					}
		  }
	  }
#if HOP_2CH_ENABLE
	 //debug testing
	 int curr_oft = WRAP_OFFSET(HOP_2CH_OFFSET0+fhop_base);
		  if(offset==curr_oft )
		    offset=WRAP_OFFSET(HOP_2CH_OFFSET1+fhop_base);
		  else /*(offset==(HOP_2CH_OFFSET1+fhop_base)||offset==(HOP_2CH_OFFSET0))*/
		  	offset = WRAP_OFFSET(HOP_2CH_OFFSET0+fhop_base);
#else
	  // simple random walk (forwar-backward) approach
	  /*if (true == fhop_dir) {
		offset = FORWARD_HOP + offset;
		offset = (HOPPING_TABLE_SIZE>offset) ? offset : offset-HOPPING_TABLE_SIZE;
	  	fhop_dir = false;
	  }
	  	else {
		offset = offset - BACKWARD_HOP;
		offset = (0 <= offset) ? offset : HOPPING_TABLE_SIZE + offset;
	  	fhop_dir = true;
	  	}*/
  extern const char hopping_patterns[] ;
	  	offset = hopping_patterns[fhop_idx++] ;
	  	fhop_idx = (CHTBL_SIZE>fhop_idx)? fhop_idx: 0;
#endif
	  	return offset;
  }
 #endif
#endif
void SysTick_Handler(void)
{
	static uint32_t ul_2ms_ticks = 0;
	ul_2ms_ticks ++;
#ifdef  RADIO_SI4463
	if(ctrl_tdma_enable) {
	  // local declarations ------------------------------
	  static unsigned int tdma_sndthr=0;
	  unsigned int lc;
	  int cdelta;

	  //clock adjustment ---------------------------------
	  lc = *DWT_CYCCNT;  //not working when in "no-debug" mode
	  cdelta = timedelta(timedelta_reset, tdma_sndthr, lc);
	  timedelta_reset = false ;
	  if((cdelta>TDMA_BOUND)||(cdelta<-TDMA_BOUND)){
		//restart
		tdma_sndthr=lc + TDMA_PERIOD/2;
		cdelta = TDMA_PERIOD/2;
	  }

	  //snd event identification -------------------------
	  if(cdelta <0) {
		/*******************************************/
		  ynsdbyte = 0; //YH: copy in ynsd special data, Take care in Rate Control Section!!! liyenho
		/*******************************************/
	  if(fifolvlcalc(wrptr_rdo_tpacket, rdptr_rdo_tpacket, RDO_TPACKET_FIFO_SIZE)<ASYMM_RATIO)
      { //idle packet case
	      uint32_t i, n, *pkt = tpacket_idle;
	      for (n=0; n<ASYMM_RATIO; n++) {  // fill in both pkts a time, liyenho
			  radio_mon_txidlecnt++;
			  pkt[0+n*RDO_ELEMENT_SIZE]=0x0000e5e5 |((radio_mon_txidlecnt<<16)&0x00ff0000)
			                               |((radio_mon_rxcnt<<24)&0xff000000)
										   ;  //no new tx payload (byte7=0xE5)
	          for(i=1;i<RDO_ELEMENT_SIZE;i++)
	          	pkt[i+n*RDO_ELEMENT_SIZE]=0xe5e5e5e5;  //no new tx payload (byte7=0xE5)

			  *((uint8_t*)&pkt[(n+1)*RDO_ELEMENT_SIZE]-1) = ynsdbyte & 0x7f; //set payload invalid
	      }

		gp_rdo_tpacket_l = tpacket_idle;
		snd_asymm_cnt = ASYMM_RATIO-1;
		snd_asymm_rdptr=0;
	  } //idle packet case
	  else {
		  // valid data case - setup for radio send
		  rdptr_inc(&wrptr_rdo_tpacket, &rdptr_rdo_tpacket, RDO_TPACKET_FIFO_SIZE, 1);
		  gp_rdo_tpacket_l = gs_rdo_tpacket + (RDO_ELEMENT_SIZE*rdptr_rdo_tpacket);
		  snd_asymm_cnt = ASYMM_RATIO-1;
		  snd_asymm_rdptr=rdptr_rdo_tpacket;
	      rdptr_inc(&wrptr_rdo_tpacket, &rdptr_rdo_tpacket, RDO_TPACKET_FIFO_SIZE, 1);
	  }
#ifndef TEMPERATURE_MEASURE
		{ // frequency hopping in the action, rec freq is superseded by this frequency too, liyenho
#if HOP_2CH_ENABLE
			if (0 != ctrl_band_select(RF_FREQ_CONTROL_INTE_LEN, chtbl_ctrl_rdo[fhop_offset*2])) {
#else // accommodate further frequency shift algorithm too, liyenho
			uint32_t frac=0, fshf = fhop_base * FREQ_SHIFT_STEP;
			uint8_t intr, *f=&frac, fctrl_str[RF_FREQ_CONTROL_INTE_LEN];
			 memcpy(fctrl_str, chtbl_ctrl_rdo[fhop_offset*2], sizeof(fctrl_str));
			 intr = *(fctrl_str + FREQ_INTR_POS);
			 *(f+2) = *(fctrl_str + FREQ_FRAC_POS);
			 *(f+1) = *(fctrl_str + FREQ_FRAC_POS+1);
			 *(f) = *(fctrl_str + FREQ_FRAC_POS+2);
			 frac = frac + (float)fshf * FREQ_CTRL_FACTOR;
			 if (0x100000 <= frac) {
				 frac -= 0x100000;
				 intr += 0x1;
			 }
			 // update frequency control string with new settings
			 *(fctrl_str + FREQ_FRAC_POS) = *(f+2);
			 *(fctrl_str + FREQ_FRAC_POS+1) = *(f+1);
			 *(fctrl_str + FREQ_FRAC_POS+2) = *(f);
			 *(fctrl_str + FREQ_INTR_POS) = intr ;
			if (0 != ctrl_band_select(RF_FREQ_CONTROL_INTE_LEN, fctrl_str)) {
#endif
				while (1) {
					; // Capture error
				}
			} // update integral part of frequency control
			if (0 != ctrl_band_select(RF_MODEM_AFC_LIMITER_LEN, chtbl_ctrl_rdo[fhop_offset*2+1])) {
				while (1) {
					; // Capture error
				}
			} // update AFC tracking limiter range
			fhop_offset = hop_chn_sel(fhop_offset);
		}
#endif  //!TEMPERATURE_MEASURE
		if(hop_state!= IDLE)
	    vRadio_StartTx_Variable_Packet(pRadioConfiguration->Radio_ChannelNumber, gp_rdo_tpacket_l,
	    	 RADIO_PKT_LEN);

	    radio_mon_txcnt++;
		tdma_sndthr = tdma_sndthr + TDMA_PERIOD;
	  }
	} //if(ctrl_tdma_enable)
#endif
#ifndef DIGIBEST_DOWNLOAD
	g_ul_10ms_ticks = ul_2ms_ticks / 5;
	if (systick_enabled && g_ul_wait_10ms<=g_ul_10ms_ticks) {
		systick_enabled = false;
		g_ul_10ms_ticks = 0;
		// pending task must finish in 10 msec? not sure if NVIC can handle
		twi_sms4470_handler(ID_PIOA, SMS_HOST_INT);
	}
#endif
}
/**
 * \brief Initialize SPI as slave.
 */
static void spi_slave_initialize(void) // only for video if run on evm
{
	/* Get pointer to SPI slave PDC register base */
	g_p_spis_pdc = spi_get_pdc_base(SPI_SLAVE_BASE);

//	puts("-I- Initialize SPI as slave \r");
#if (SAMG55)
	/* Enable the peripheral and set SPI mode. */
	flexcom_enable(BOARD_FLEXCOM_SPI);
	flexcom_set_opmode(BOARD_FLEXCOM_SPI, FLEXCOM_SPI);
#else
	/* Configure an SPI peripheral. */
	pmc_enable_periph_clk(SPI_ID);
#endif
	spi_disable(SPI_SLAVE_BASE);
	spi_reset(SPI_SLAVE_BASE);
#ifdef TEST_SPI
	spi_set_lastxfer(SPI_SLAVE_BASE);
	spi_set_master_mode(SPI_SLAVE_BASE);
#else
	spi_set_slave_mode(SPI_SLAVE_BASE);
#endif
	spi_disable_mode_fault_detect(SPI_SLAVE_BASE);
	spi_set_peripheral_chip_select_value(SPI_SLAVE_BASE, SPI_CHIP_SEL);
	spi_set_clock_polarity(SPI_SLAVE_BASE, SPI_CHIP_SEL, SPI_CLK_POLARITY);
	spi_set_clock_phase(SPI_SLAVE_BASE, SPI_CHIP_SEL, SPI_CLK_PHASE);
	spi_set_bits_per_transfer(SPI_SLAVE_BASE, SPI_CHIP_SEL,
#if true //ndef RECV_SMS4470 // try it out see if we can avoid byte drop, liyenho
			SPI_CSR_BITS_16_BIT);
#else //RECV_SMS4470
			SPI_CSR_BITS_8_BIT); // only used for sms4470 video pipe
#endif
#ifdef TEST_SPI
	spi_set_baudrate_div(SPI_SLAVE_BASE, SPI_CHIP_SEL,
			(sysclk_get_cpu_hz() / gs_ul_spi_clock[1]));
	spi_set_transfer_delay(SPI_SLAVE_BASE, SPI_CHIP_SEL, SPI_DLYBS,
			SPI_DLYBCT);
#endif
#ifdef TEST_SPI
	spi_enable_loopback(SPI_SLAVE_BASE);
#endif
	pdc_disable_transfer(g_p_spis_pdc, PERIPH_PTCR_RXTDIS |
			PERIPH_PTCR_TXTDIS);
}
/**
 * \brief Initialize SPI as master.
 */
/*static*/ void spi_master_initialize(uint32_t ch, uint32_t base, uint32_t fxcom)
{
//	puts("-I- Initialize SPI as master\r");
	/* Get pointer to SPI master PDC register base */
	g_p_spim_pdc [ch] = spi_get_pdc_base(base);

#if (SAMG55)
	/* Enable the peripheral and set SPI mode. */
	flexcom_enable(fxcom);
	flexcom_set_opmode(fxcom, FLEXCOM_SPI);
#else
	/* Configure an SPI peripheral. */
	pmc_enable_periph_clk(SPI_ID);
#endif
	spi_disable(base);
	spi_reset(base);
	spi_set_lastxfer(base);
	spi_set_master_mode(base);
	spi_disable_mode_fault_detect(base);
	spi_set_peripheral_chip_select_value(base, SPI_CHIP_SEL);
 #ifdef RADIO_SI4463
 if (2/*data from si4463*/ == ch) {
	 spi_set_clock_polarity(base, SPI_CHIP_SEL, 0/*clk idle state is low*/);
 	spi_set_clock_phase(base, SPI_CHIP_SEL, 1/*captured @ rising, transit @ falling*/); }
 else // ctrl/sts/video
 #endif
	{ spi_set_clock_polarity(base, SPI_CHIP_SEL, SPI_CLK_POLARITY);
	spi_set_clock_phase(base, SPI_CHIP_SEL, SPI_CLK_PHASE); }
 #ifdef RADIO_SI4463
 if (2/*data from si4463*/ == ch)
	spi_set_bits_per_transfer(base, SPI_CHIP_SEL,
 		SPI_CSR_BITS_8_BIT);  // 8 bit spi xfer, liyenho
 else // ctrl/sts/video
 #endif
	spi_set_bits_per_transfer(base, SPI_CHIP_SEL,
			(ch)?SPI_CSR_BITS_16_BIT:SPI_CSR_BITS_8_BIT);  // either 8 or 16 bit spi xfer, liyenho
	spi_set_baudrate_div(base, SPI_CHIP_SEL,
			(sysclk_get_cpu_hz() / gs_ul_spi_clock [ch]));
#ifdef RADIO_SI4463
	if (2/*data from si4463*/ == ch)
		spi_set_transfer_delay(base, SPI_CHIP_SEL, 0x10/*delay between bytes*/,
			0x10/*delay between spi xfer*/);
	else // ctrl/sts and video
#endif
	spi_set_transfer_delay(base, SPI_CHIP_SEL, SPI_DLYBS,
			SPI_DLYBCT);
	spi_enable(base);
#ifdef TEST_SPI
	spi_enable_loopback(base);
#endif
	pdc_disable_transfer(g_p_spim_pdc [ch], PERIPH_PTCR_RXTDIS |
			PERIPH_PTCR_TXTDIS);
}
#ifdef I2C_FAKE_POLL
/**
 * \brief Initialize I2S tx site.
 */
static void i2s_tx_initialize(void)
{
	uint32_t i;
	struct i2s_config config= {0};

//	puts("-I- Initialize I2S as tx\r");
	config.master_mode = true;  // assumed clk source from atmel, liyenho
	config.master_clock_enable = true;
	//config.tx_dma = I2S_ONE_DMA_CHANNEL_FOR_ONE_CHANNEL;
	config.tx_channels = I2S_CHANNEL_MONO; // can't be stereo because 188 can't divide into 8
#ifndef I2C_FAKE_POLL
	config.data_format = I2S_DATE_32BIT;
	config.fs_ratio = I2S_FS_RATE_512;
	config.loopback = false;

	i2s_init(&dev_inst_i2s, I2SC0_BASE, &config, 0); // we use I2SC0 instance, liyenho
	//i2s_enable(&dev_inst_i2s);
	/* Get pointer to I2SC0 PDC register base */
	g_p_i2st_pdc = i2s_get_pdc_base(&dev_inst_i2s);
	i2s_set_callback(&dev_inst_i2s, I2S_INTERRUPT_TXBUFE, &callback_i2sc0_tx, 2/*0*/) ;

	i2s_enable_transmission(&dev_inst_i2s);
	i2s_enable_clocks(&dev_inst_i2s);
	pdc_disable_transfer(g_p_i2st_pdc, PERIPH_PTCR_RXTDIS |
			PERIPH_PTCR_TXTDIS);

#if false  // trigger upon data availability
	i2sc_tx_transfer(gs_uc_tbuffer, I2SC_BUFFER_SIZE);
#endif
#else
	config.data_format = 4; // fake i2c clk gen
	config.fs_ratio = 0; // fake i2c clk gen
	config.loopback = false;
	// expected 234 khz i2c fake clk
	i2s_init(&dev_inst_i2s, I2SC0_BASE, &config, 15); // we use I2SC0 instance, liyenho
	i2s_enable(&dev_inst_i2s);
#endif
}
#endif
/**
 * \brief Perform SPI master transfer.
 *
 * \param pbuf Pointer to buffer to transfer.
 * \param size Size of the buffer.
 */
void spi_tx_transfer(void *p_tbuf, uint32_t tsize, void *p_rbuf,
		uint32_t rsize, uint32_t ch);
/*static*/ void spi_tx_transfer(void *p_tbuf, uint32_t tsize, void *p_rbuf,
		uint32_t rsize, uint32_t ch)
{
	pdc_packet_t pdc_spi_packet;

	pdc_spi_packet.ul_addr = (uint32_t)p_rbuf;
	pdc_spi_packet.ul_size = rsize;
	pdc_rx_init(g_p_spim_pdc [ch], &pdc_spi_packet, NULL);

	pdc_spi_packet.ul_addr = (uint32_t)p_tbuf;
	pdc_spi_packet.ul_size = tsize;
	pdc_tx_init(g_p_spim_pdc [ch], &pdc_spi_packet, NULL);

	/* Enable the RX and TX PDC transfer requests */
	pdc_enable_transfer(g_p_spim_pdc [ch], PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);

	/* Transfer done handler is in ISR */
#ifdef TEST_SPI
	uint32_t base, spi_ier = SPI_IER_RXBUFF;
#else
	uint32_t base, spi_ier = (1==ch)?SPI_IER_TXBUFE/*only ok for video*/:SPI_IER_RXBUFF;
#endif
	base = (1==ch)? SPI_MASTER_BASE: SPI0_MASTER_BASE;
 #ifdef  RADIO_SI4463
 if (2==ch) base = SPI7_MASTER_BASE;
 #endif
	spi_enable_interrupt(base, spi_ier) ;
}

static inline void usb_read_buf(void *pb)
{
	int read=0, size = I2SC_BUFFER_SIZE;
	do {
		iram_size_t b = size-udi_cdc_read_buf(pb, size);
		pb += b;
		size -= b;
		read += b;
	} while (I2SC_BUFFER_SIZE != read && !system_main_restart);
}
	static inline void usb_write_buf1(void *pb, int size0)
	{
		int written=0, size = size0;
		do {
			iram_size_t b = size-udi_cdc_write_buf(pb, size);
			pb += b;
			size -= b;
			written += b;
		} while (size0 != written);
	}
#ifdef SMS_DVBT2_DOWNLOAD
void usb_read_buf1(void *pb, int size0);
/*static inline*/ void usb_read_buf1(void *pb, int size0)
{
	int read=0, size = size0;
	do {
		iram_size_t b = size-udi_cdc_read_buf(pb, size);
		pb += b;
		size -= b;
		read += b;
	} while (size0 != read);
}

fordigibest_usb_read_buf1(void* *pbuff, int size0)
{
	*pbuff = fw_sms_rbuffer;
	usb_read_buf1(fw_sms_rbuffer, size0);
}
#endif
static inline void usb_write_buf(void *pb)
{
	int written=0, size = I2SC_BUFFER_SIZE;
	do {
		iram_size_t b = size-udi_cdc_write_buf(pb, size);
		pb += b;
		size -= b;
		written += b;
	} while (I2SC_BUFFER_SIZE != written && !system_main_restart);
}
#if defined(RECV_SMS4470) && !defined(RX_SPI_CHAINING)
  /*static*/ void usb_write_buf_cb() {
	  volatile uint32_t cycle_now, cycle_delta;
	  volatile static uint32_t cycle_now1, cycle_max=0, cycle_min=0x7fffffff;
	  //volatile bool hl_curr;
	  cycle_now = *DWT_CYCCNT;
	  cycle_delta_0 = cycle_now - cycle_cnt;
  //cycle_max = (cycle_max<cycle_delta_0)?cycle_delta_0:cycle_max;
  //cycle_min = (cycle_min>cycle_delta_0)?cycle_delta_0:cycle_min;
		cycle_cnt = *DWT_CYCCNT;  // update global cycle counter
	  if (!i2c_read_cb_on  ) {
	  	 return; /* do not proceed if in restart mode */}
	  	// check on spi siano xfer stats
		if (0==g_p_spis_pdc->PERIPH_RNCR) {
			/*if (cycle_now1) {
				cycle_delta = *DWT_CYCCNT - cycle_now1;
  	cycle_max = (cycle_max<cycle_delta)?cycle_delta:cycle_max;
  	cycle_min = (cycle_min>cycle_delta)?cycle_delta:cycle_min;
			}
			cycle_now1 = *DWT_CYCCNT;*/
			g_p_spis_pdc->PERIPH_RNPR = sms4470_usb_ctx.spi_buf_rd;
	 	 	g_p_spis_pdc->PERIPH_RNCR = I2SC_BUFFER_SIZE/2;
			if (sms4470_usb_ctx.spi_buf_rd == sms4470_usb_ctx.usb_buf_wr) {
				sms4470_usb_ctx.usb_failcnt += 1;
				sms4470_usb_ctx.spi_buf_rd -= I2SC_BUFFER_SIZE; // equivalent to drop a block to align
				sms4470_usb_ctx.skip_rd = true;
				pio_clear(PIOB, PIO_PB9); // disable TS gate
			}
			sms4470_usb_ctx.spi_buf_rd += I2SC_BUFFER_SIZE;
			if (sms4470_usb_ctx.buf_end<=sms4470_usb_ctx.spi_buf_rd)
				sms4470_usb_ctx.spi_buf_rd = sms4470_usb_ctx.buf_start;
			sms4470_usb_ctx.skip_wr = false;
	  }
	  uint32_t udi_cdc_lvl = udi_cdc_multi_get_free_tx_buffer(0);
	/*if (I2SC_BUFFER_SIZE<=udi_cdc_lvl && cycle_now1) {
		cycle_delta = *DWT_CYCCNT - cycle_now1;
  	  cycle_max = (cycle_max<cycle_delta)?cycle_delta:cycle_max;
  	  cycle_min = (cycle_min>cycle_delta)?cycle_delta:cycle_min;
	}
	cycle_now1 = *DWT_CYCCNT;*/
	  if (!sms4470_usb_ctx.skip_wr && I2SC_BUFFER_SIZE<=udi_cdc_lvl) {
//cycle_now = *DWT_CYCCNT;
			usb_write_buf(sms4470_usb_ctx.usb_buf_wr);
//cycle_delta = *DWT_CYCCNT - cycle_now;
			sms4470_usb_ctx.usb_buf_wr += I2SC_BUFFER_SIZE;
			if (sms4470_usb_ctx.buf_end<=sms4470_usb_ctx.usb_buf_wr)
				sms4470_usb_ctx.usb_buf_wr = sms4470_usb_ctx.buf_start;
			if (sms4470_usb_ctx.skip_rd) {
				if (sms4470_usb_ctx.usb_buf_wr < sms4470_usb_ctx.spi_buf_rd) {
					// write ptr wrapped around
					if (sms4470_usb_ctx.spi_buf_rd <= (sms4470_usb_ctx.usb_buf_wr+GRAND_BUFFER_SIZE/2)) {
						pio_set(PIOB, PIO_PB9); // enable TS gate
						sms4470_usb_ctx.skip_rd = false;
					}
				}
				else {
					if (GRAND_BUFFER_SIZE/2 <= (sms4470_usb_ctx.usb_buf_wr-sms4470_usb_ctx.spi_buf_rd)) {
						pio_set(PIOB, PIO_PB9); // enable TS gate
						sms4470_usb_ctx.skip_rd = false;
					}
				}
			}
			if (sms4470_usb_ctx.buf_start == sms4470_usb_ctx.spi_buf_rd) {
				if ((sms4470_usb_ctx.buf_last-I2SC_BUFFER_SIZE) == sms4470_usb_ctx.usb_buf_wr) {
					sms4470_usb_ctx.skip_wr = true;
				}
			} else {
				if (((sms4470_usb_ctx.buf_start+I2SC_BUFFER_SIZE) == sms4470_usb_ctx.spi_buf_rd) &&
					(sms4470_usb_ctx.buf_last == sms4470_usb_ctx.usb_buf_wr)) {
					sms4470_usb_ctx.skip_wr = true;
				}
				else if ((sms4470_usb_ctx.spi_buf_rd-2*I2SC_BUFFER_SIZE) == sms4470_usb_ctx.usb_buf_wr) {
					sms4470_usb_ctx.skip_wr = true;
				}
			}
	  }
	  cycle_now = *DWT_CYCCNT;
	  cycle_delta_1 = cycle_now - cycle_cnt;
  }
#endif
 #if defined(MEDIA_ON_FLASH) && !defined(NO_USB)
  static void host_usb_mda_flash_cb() {
		if (Is_udd_in_sent(0) ||
			udd_g_ctrlreq.payload_size != udd_g_ctrlreq.req.wLength)
			return; // invalid call
		memcpy(&media_file_len,
						udd_g_ctrlreq.payload,
						USB_LOADM_LEN);
		if ((media_file_len % I2SC_BUFFER_SIZE) ||
			NUM_OF_PAGES * IFLASH_PAGE_SIZE<media_file_len)
			return ; /* error in file len */
		uint32_t ul_rc;
		usb_load_media = true; // enable download process
  }
 #endif
/*! \brief host ctrl/sts callback for sms4470 interface
 */
#ifdef SMS_DVBT2_DOWNLOAD
  static void siano_tuning(uint32_t fc);
#endif
 void erase_last_sector() {
		memcpy(backup, (void*)(IFLASH_ADDR + IFLASH_SIZE-sizeof(backup)), sizeof(backup));
		uint32_t er_adr =IFLASH_ADDR + IFLASH_SIZE-SECTOR_SIZE_L;
		flash_erase_sector(er_adr);
	}
  static void host_usb_cb() {
		if (Is_udd_in_sent(0) ||
			udd_g_ctrlreq.payload_size >/*!=*/ udd_g_ctrlreq.req.wLength)
			return; // invalid call
		if (USB_BOOT_APP_VAL == udd_g_ctrlreq.req.wValue) {
			erase_last_sector() ;
			CHECKED_FLASH_WR(
				IFLASH_ADDR + IFLASH_SIZE-sizeof(backup),
				backup, NUM_OF_FPGA_REGS +1 +4 +2)
			CHECKED_FLASH_WR(ul_page_addr_bootapp, &bootapp, 1)
			CHECKED_FLASH_WR(
				IFLASH_ADDR + IFLASH_SIZE-NUM_OF_ATMEL_REGS +3,
				backup +NUM_OF_FPGA_REGS +1 +4 +3,
				sizeof(backup)-NUM_OF_FPGA_REGS-1 -4 -3)
			return ;
		}
		if ((USB_CPLD_UPGRADE_VAL == udd_g_ctrlreq.req.wValue ||
				USB_ATMEL_UPGRADE_VAL == udd_g_ctrlreq.req.wValue) &&
				FW_UPGRADE_HDR_LEN >=udd_g_ctrlreq.payload_size) {
			memcpy(upgrade_fw_hdr, // set fpga fw /img hdr, liyenho
								udd_g_ctrlreq.payload,
								FW_UPGRADE_HDR_LEN);
			system_upgrade = (USB_CPLD_UPGRADE_VAL == udd_g_ctrlreq.req.wValue)?1/*cpld*/:2/*atmel*/;
			main_loop_on = false ; // clear up the 'upgrade flag' ;-)
			return;
		}
#ifdef CONFIG_ADI_6612
		else if ((RF_RX_FREQ_VAL == udd_g_ctrlreq.req.wValue || RF_RX_FREQ_VAL_F == udd_g_ctrlreq.req.wValue) &&
						sizeof(uint32_t) ==udd_g_ctrlreq.payload_size)
		{
			memcpy(&num_6612_regs, // set # of 6612 regs to be configured, liyenho
								udd_g_ctrlreq.payload,
								sizeof(uint32_t));
			if (RF_RX_FREQ_VAL_F == udd_g_ctrlreq.req.wValue) {
				if (!num_6612_regs) {
					// termination process, revert back to normal ops
					if (ctrl_tdma_enable_shdw) {
						ctrl_tdma_enable_shdw = false;
						ctrl_tdma_enable = true;
					}
				}
				else if (ctrl_tdma_enable) {
					ctrl_tdma_enable_shdw = true;
					ctrl_tdma_enable = false;
				}
			}
			return;
		}
	 	else if (RF_RX_VAL == udd_g_ctrlreq.req.wValue) {
		 	set_rf_params = true;  // adi 6612 is configured
			 return; // crucial...
	 	}
#endif
#ifdef SMS_DVBT2_DOWNLOAD
		else if ((USB_SMS_FW_VAL == udd_g_ctrlreq.req.wValue ||
						 USB_SMS_DATA_VAL == udd_g_ctrlreq.req.wValue) &&
						SMS_FW_HDR_LEN >=udd_g_ctrlreq.payload_size) {
			memcpy(sms4470_fw_hdr, // set sms4470 fw hdr, liyenho
								udd_g_ctrlreq.payload,
								SMS_FW_HDR_LEN);
			return;
		}
#endif
#ifdef SIANO_RETUNE
		else if ((USB_RX_TUNE_VAL == udd_g_ctrlreq.req.wValue) &&
						sizeof(uint32_t) ==udd_g_ctrlreq.payload_size) {
			memcpy(&fc_siano_tuned, // set tuning frequency from user, liyenho
								udd_g_ctrlreq.payload,
								sizeof(uint32_t));
			pio_clear(PIOB, PIO_PB9); // disable TS gate
#ifdef RX_SPI_CHAINING
			spidma_active = FALSE;
#endif
			siano_tuning(fc_siano_tuned);
#ifndef RX_SPI_CHAINING
			sms4470_usb_ctx.spi_buf_rd -= I2SC_BUFFER_SIZE; // equivalent to drop a block to align
			if (sms4470_usb_ctx.buf_start>sms4470_usb_ctx.spi_buf_rd)
				sms4470_usb_ctx.spi_buf_rd = sms4470_usb_ctx.buf_end;
			pio_set(PIOB, PIO_PB9); // enable TS gate
#endif
			return;
		}
#endif
		dev_access *ps = gs_uc_htbuffer,*ps1 = gs_uc_hrbuffer;
		// prepare confirm msg by echo whatever received
		memcpy(gs_uc_hrbuffer, udd_g_ctrlreq.payload, USB_HOST_MSG_LEN-sizeof(ps->data[0]));
#ifdef SMS_DVBT2_DOWNLOAD
		if (SIGNAL_DETECTED == ps->access) {
			if (0!=dvbt.SMS4470DetectNoSignalFlag)
				 ps1->data[0] = (uint16_t)true; // not present
			else
				 ps1->data[0] = (uint16_t)false; // present
			return;
		}
		if (SIGNAL_STATUS == ps->access) {
			Short_Statistics_ST *psts= &ps1->data[0];
			memcpy(psts, &/*dvbt.DVBTSignalStatistic*/shortStatG, sizeof(*psts));
			return;
		}
		if (TRANS_STATUS == ps->access) {
			TRANSMISSION_STATISTICS_ST *psts= &ps1->data[0];
			memcpy(psts, &dvbt.DVBTTransmissionStatistic, sizeof(*psts));
			return;
		}
		if (RECV_STATUS == ps->access) {
			RECEPTION_STATISTICS_ST *psts= &ps1->data[0];
			memcpy(psts, &/*dvbt.DVBTReceptionStatistic*/recptStatG_s, sizeof(*psts));
			return;
		}
		if (LOCKED_STATUS == ps->access) {
			ps1->data[0] = (uint16_t)sms_dvbt_lock;
			return;
		}
#endif
		usb_host_msg = true; // enable mainloop process
  }
#ifdef  RADIO_SI4463
extern uint8_t get_si446x_temp();
extern void recalibrate_capval (void* ul_page_addr_mtemp, uint8_t median);
static void si4463_radio_cb() {
	if (udd_g_ctrlreq.payload_size < udd_g_ctrlreq.req.wLength)
	return; // invalid call
	else if (RADIO_STARTUP_IDX == udd_g_ctrlreq.req.wIndex) {
		spi_dma_mode = false;
		/* init radio stats obj */
		{
			for (int j=0; j<CTRL_CTX_LEN; j++) {
				r4463_sts.ctrl_bits_ctx[j] = LONG_RNG;
			}
			r4463_sts.bw_ctrl_bits = LONG_RNG;
			r4463_sts.errPerAcc =0;
			r4463_sts.loop_cnt= 0;
		}
		vRadio_Init();
	#ifdef CONFIG_ON_FLASH
		uint8_t median = *(uint8_t*)ul_page_addr_ctune;
		if (0xff != median)  // adjust cap bank value per stored const
			recalibrate_capval((void*)ul_page_addr_mtemp, median);
	#endif
		{
			  for(int i=0;i<2*RDO_ELEMENT_SIZE;i++)
			  gs_rdo_tpacket[i]=0xc5c5c5c5;
		}
#ifndef RX_SPI_CHAINING
		pio_set(PIOB, PIO_PB9); // enable TS gate
#endif
		si4463_radio_started = true;
		ctrl_tdma_enable = true;
	}
	else if (RADIO_CHSEL_IDX == udd_g_ctrlreq.req.wIndex) {
		if (0 != ctrl_band_select(udd_g_ctrlreq.req.wLength, (U8*)gs_uc_hrbuffer))
			return ;  // cmd error or time out...
	}
	else if (RADIO_CTUNE_IDX == udd_g_ctrlreq.req.wIndex) {
		uint8_t ctune = *(U8*)gs_uc_hrbuffer;
		si446x_set_property1( 0x0, 0x1, 0x0, ctune ); //si446x_set_property shall crash...
	}
	else if (RADIO_DATA_TX_IDX == udd_g_ctrlreq.req.wIndex) {
		//a lot of this is duplicated in vender_specific. ToDo: prune
		unsigned int wrptr_tmp1 = wrptr_rdo_tpacket;
		unsigned int wrptr_tmp2;
		unsigned int ovflag1,ovflag2;
		ovflag1 = wrptr_inc(&wrptr_tmp1,&rdptr_rdo_tpacket,RDO_TPACKET_FIFO_SIZE,1);
		wrptr_tmp2=wrptr_tmp1;
		ovflag2 = wrptr_inc(&wrptr_tmp2,&rdptr_rdo_tpacket,RDO_TPACKET_FIFO_SIZE,1);
		if(ovflag1 || ovflag2){
		;
		}
		else{
			//transfer to radio buffer needs to happen in callback, i.e. after USB transfer is complete
			//otherwise source buffer will not contain newest data
			gp_rdo_tpacket = gs_rdo_tpacket + (RDO_ELEMENT_SIZE*wrptr_tmp1);
			memcpy(gp_rdo_tpacket,tpacket_grp,/*RADIO_PKT_LEN*/RADIO_GRPPKT_LEN/2); // fixed
			((uint8_t *)gp_rdo_tpacket)[RADIO_GRPPKT_LEN/2/*stepped on Rate Control Status byte*/]=0x80; //set grp header flag
			gp_rdo_tpacket = gs_rdo_tpacket + (RDO_ELEMENT_SIZE*wrptr_tmp2);
			memcpy(gp_rdo_tpacket,tpacket_grp+(RADIO_GRPPKT_LEN/2),/*RADIO_PKT_LEN*/RADIO_GRPPKT_LEN/2);// fixed
			((uint8_t *)gp_rdo_tpacket)[RADIO_GRPPKT_LEN/2/*stepped on Rate Control Status byte*/]=0x00; //set grp payload flag
			wrptr_rdo_tpacket = wrptr_tmp2;
		}
	}
	else if (RADIO_PAIRID_IDX == udd_g_ctrlreq.req.wIndex) {
		uint8_t* pv = (U8*)gs_uc_hrbuffer;
		uint8_t idleflag=1, pairingidflag=1, valb;
		for (int j=0; j<HOP_ID_LEN; j++) {
		  valb= *(pv+j);
		  if(valb!=0) idleflag=0;
		  if((j<HOP_ID_LEN-1)&(valb!=0)) pairingidflag=0;
		  if((j==HOP_ID_LEN-1)&(valb!=1)) pairingidflag=0;
			*(hop_id+j) = *(pv+j);
		}
		if(idleflag) hop_state = IDLE;
		else if(pairingidflag) hop_state = PAIRING ;
		else hop_state = PAIRED ;

		if(hop_state== IDLE){
			ctrl_tdma_enable = false;
			ctrl_tdma_lock = false;
		}
		else if (hop_state == PAIRED) {
				int id=0, bit = 0;
				id |= *(hop_id+0) & (1<<bit); bit+=1 ;
				id |= *(hop_id+2) & (1<<bit); bit+=1 ;
				id |= *(hop_id+4) & (1<<bit); bit+=1 ;
				id |= *(hop_id+5) & (1<<bit); bit+=1 ;
				id |= *(hop_id+7) & (1<<bit); bit+=1 ;
				id |= *(hop_id+9) & (1<<bit); bit+=1 ;
				// calculated hop id from PAIR ID mode
#if false
				fhop_offset = (CHTBL_SIZE>id) ? id : id-CHTBL_SIZE ;
				fhop_base=0; //TBD for 50ch hop case
#else // accommodate further frequency shift algorithm too, liyenho
				int fshf=0;
				bit = 0;
				fshf |= *(hop_id+1) & (1<<bit); bit+=1 ;
				fshf |= *(hop_id+3) & (1<<bit); bit+=1 ;
				fshf |= *(hop_id+6) & (1<<bit); bit+=1 ;
				fshf |= *(hop_id+8) & (1<<bit); bit+=1 ;
				fhop_offset = (CHTBL_SIZE>id) ? id : id-CHTBL_SIZE ;
				fhop_base = (NUM_FREQ_SHIFT>fshf) ? fshf : fshf-NUM_FREQ_SHIFT ;
#endif
				if(HOP_2CH_ENABLE) {
				  int i;
				  for(i=0;i<10;i++)
				    fhop_base= fhop_base + (*(hop_id+i));
				  fhop_base =( 1+(  (fhop_base & 0x0f)+((fhop_base>>4)&0x0f)&0x0f  ) )*2;
				            //add all bytes, then, add the two 4bitNibbles ---> range (1-16)*2
				  fhop_offset = WRAP_OFFSET(fhop_base+HOP_2CH_OFFSET0);
				}
				ctrl_tdma_enable = true;
				ctrl_tdma_lock = true; // tdd activity assumed to be active
		}
		else //pairing
		{
			fhop_base = 0;
			fhop_offset = HOP_2CH_ENABLE?WRAP_OFFSET(HOP_2CH_OFFSET0):0;
			ctrl_tdma_enable = true;
		}
		// configure the channel for listening
		//   NO NEED: since RX is master, and will set the channel when prior to sending (see systick)
	}
	else if (RADIO_HOPLESS_IDX == udd_g_ctrlreq.req.wIndex) {
			memcpy(&fhopless, // set hopless section from user
								udd_g_ctrlreq.payload,
								sizeof(fhopless));
	}
	else if (BASE_GPS_IDX == udd_g_ctrlreq.req.wIndex){
		//update base GPS struct
		BaseLocation.latitude = Degrees_To_Radians(*(float *)gs_uc_hrbuffer);
		BaseLocation.longitude = Degrees_To_Radians(*((float *)gs_uc_hrbuffer+1));
	}
}
#endif
#ifdef SMS_DVBT2_DOWNLOAD
 static void download_sms_fw(uint32_t sync, uint8_t *chk )
 {
	while (sync != *sms4470_fw_hdr/*sms fw sync*/);
	// download sms4470 in chunk, algorithm from digibest sdk
	sms_state = STATE_FW_DOWNLOAD;
	uint32_t fw_addr = sms4470_fw_hdr[2],
						fw_len0, fw_len = sms4470_fw_hdr[1],
						_hdr[3], len, len1, len2, ll, ll1, ofst, crc, i;
	uint8_t *pb = fw_sms_rbuffer,
					 *pe = pb+FW_DNLD_SIZE;
	uint8_t DownloadCommand[8] = {0x9a,0x02,0x96,0x0b,0x08,0x00,0x01,0x00 };
#ifdef FWM_DNLD_DBG
	uint32_t fw_dbg_buffer[(sizeof(DownloadCommand)+sizeof(_hdr)+FW_DNLD_SIZE)/sizeof(int)];
	volatile bool erase = true;
	uint32_t ul_page_addr=PAGE_ADDRESS ;
	volatile uint32_t lm, le, ul_rc, er_adr =ul_page_addr-SECTOR_RES, rem = fw_len;
	const uint32_t erlen = SECTOR_SIZE_L; // last two sector size must be 128 kbytes
	lm = SECTOR_RES+FW_DNLD_SIZE;
	le = erlen;
#endif
	usb_read_buf1(fw_sms_rbuffer, FW_DNLD_SIZE);
	 do { // bypass annoying flush data prior to xfer from bulk ep, liyenho
	 	// download is the bottom up process...
		if (chk[0] == pb[0] &&
			chk[1] == pb[1] &&
			chk[2] == pb[2] &&
			chk[3] == pb[3])
			  break;
	} while (pe>++pb);
	if (pb==pe) {
		/*puts("-E-\tfailed to locate 1st chunk of sms fw.\r");*/
		while (1) {
			; /* Capture error */
		}
	}
	len = FW_DNLD_SIZE - ((uint32_t)pb - (uint32_t)fw_sms_rbuffer);
	len2 = (len1=FW_DNLD_SIZE) - (ll=len);
	memcpy(fw_sms_tbuffer, pb, len); // has to use the other buffer
	ofst = fw_addr + fw_len;
	delay_ms(100); // match delay used @ host side
	pb = (uint8_t*)fw_sms_tbuffer + len;
	 pe = (uint8_t*)fw_sms_rbuffer + len2;
	fw_len0 = fw_len - len + len2;
	while(1) {
		len = (FW_DNLD_SIZE<=fw_len0)?FW_DNLD_SIZE:fw_len0;
		if (0 != fw_len0)
			usb_read_buf1(fw_sms_rbuffer, len);
		fw_len0 -= len;
		if (FW_DNLD_SIZE != len) {
	#include <assert.h>
			if (0 != len) {
				if (len2>len) {
					assert(len1+len2 == len);
					memcpy(pb, fw_sms_rbuffer, len);
				}
				else {
					memcpy(pb, fw_sms_rbuffer, len2);
					ll1 = len - len2;
				}
		   }
			else  {
				assert(len1+len2 == ll1);
				memcpy(fw_sms_tbuffer, pe, ll1);
		   }
	  	}
		else
	  		memcpy(pb, fw_sms_rbuffer, len2);
		TWI_WRITE(DownloadCommand,sizeof(DownloadCommand))
		_hdr [1] = len1;
		_hdr[2] = (ofst-=len1);
		crc = 0;
		for (i=0; i<8; i++)
			crc ^= ((uint8_t*)_hdr)[4+i];
		for (i=0; i<len1; i++)
			crc ^= ((uint8_t*)fw_sms_tbuffer)[i];
		if (fw_addr == ofst)
			_hdr[0] = crc;  // actual crc at last chunk
		else // mess crc up if it's not last chunk
			_hdr[0] = crc^0x55;
		delay_ms(30);
		TWI_WRITE(_hdr,sizeof(_hdr))
		delay_ms(20);
		TWI_WRITE(fw_sms_tbuffer,len1)
#ifdef FWM_DNLD_DBG
	uint8_t *pdb = (uint8_t*)fw_dbg_buffer;
		memcpy(pdb, DownloadCommand, sizeof(DownloadCommand));
			pdb += sizeof(DownloadCommand);
		memcpy(pdb, _hdr, sizeof(_hdr));
			pdb += sizeof(_hdr);
		memcpy(pdb, fw_sms_tbuffer, len1);
		if (erase) {
			ul_rc = flash_erase_sector(er_adr);
			if (ul_rc != FLASH_RC_OK) {
				//printf("- Pages erase error %lu\n\r", (UL)ul_rc);
				return; // error when erase pages
			}
			erase = false;
		}
		ul_rc = flash_write(ul_page_addr, fw_dbg_buffer, sizeof(DownloadCommand)+sizeof(_hdr)+len1, 0);
		if (ul_rc != FLASH_RC_OK) {
			//printf("- Pages write error %lu\n\r", (UL)ul_rc);
			return; // error when write pages
		}
		ul_page_addr += (sizeof(DownloadCommand)+sizeof(_hdr)+len1);
		// determine whether erasure is necessary or not
		lm += (sizeof(DownloadCommand)+sizeof(_hdr)+len1);
		if (le < lm) {
			erase = true;
			le += erlen;
			er_adr += erlen;
		}
#endif
		delay_ms(30);
		if (!(fw_len -= len1)) break;
		len1 = (FW_DNLD_SIZE<=fw_len)?FW_DNLD_SIZE:fw_len;
		if (FW_DNLD_SIZE == len) {
			memcpy(fw_sms_tbuffer, pe, ll);
		}
	}
 #ifdef FWM_DNLD_DBG
  ul_page_addr=PAGE_ADDRESS ;
  i = 0;
  	do {
  		if (FW_DNLD_SIZE<=rem) {
  			len = sizeof(DownloadCommand)+sizeof(_hdr)+FW_DNLD_SIZE;
  			rem -= FW_DNLD_SIZE;
		}
  		else {
  			len = sizeof(DownloadCommand)+sizeof(_hdr)+rem;
  			rem = 0;
  		}
  		i = i+1;
	  	memcpy(fw_dbg_buffer, ul_page_addr, len);
		  delay_us(3000); // see if this stablize? it does!
		usb_write_buf1(fw_dbg_buffer, len);
  		ul_page_addr += len;
  	} while (0<rem);
 #endif
	delay_ms(400);
 }
/* download firmware data */
static void download_sms_data(uint32_t sync, uint8_t *chk ) // rely on read_sms_response can't fail, liyenho
{
	/* request reload DVBT firmware  */
	sms_state = 	STATE_REQ_FW_RELOAD;
	SmsMsgData_ST *msg =(SmsMsgData_ST*)gs_sms_tbuffer;
	SMS_INIT_MSG(&msg->xMsgHeader, MSG_SW_RELOAD_START_REQ,
		     sizeof(SmsMsgData_ST));
	msg->msgData[0] = SMSHOSTLIB_DNLD_ALL_SLAVES_INCHAIN_ASYNC_NO_PRE_TASKS_SHUTDOWN;
	TWI_WRITE(gs_sms_tbuffer,sizeof(SmsMsgData_ST))
	_WAIT_(1); // scheduled 10 msec
	read_sms_response(MSG_SW_RELOAD_START_RES, STATE_FW_RELOAD_RES);
	SmsDataDownload_ST *msg3 =(SmsDataDownload_ST*)gs_uc_tbuffer;
	sms_state = 	STATE_DATA_DOWNLOAD;
	while (sync != *sms4470_fw_hdr/*sms fw sync*/);
	uint32_t fw_addr = sms4470_fw_hdr[2]/*0xe00000*/,
						fw_len = sms4470_fw_hdr[1],
						ll, len, len1, size = 0;
	uint8_t *pb = fw_sms_rbuffer,
					 *pe = pb+DATA_DNLD_SIZE;
	usb_read_buf1(fw_sms_rbuffer, DATA_DNLD_SIZE);
	 do { // bypass annoying flush data prior to xfer from bulk ep, liyenho
	 	// download is the bottom up process...
		if (chk[0] == pb[0] &&
			chk[1] == pb[1] &&
			chk[2] == pb[2] &&
			chk[3] == pb[3])
			  break;
	} while (pe>++pb);
	if (pb==pe) {
		/*puts("-E-\tfailed to locate 1st chunk of sms fw.\r");*/
		while (1) {
			; /* Capture error */
		}
	}
	len = DATA_DNLD_SIZE - ((uint32_t)pb - (uint32_t)fw_sms_rbuffer);
	pb = (uint8_t*)fw_sms_rbuffer + (DATA_DNLD_SIZE- len);
	len1 = fw_len - DATA_DNLD_SIZE;
	while (1) {
		memcpy(msg3->Payload, pb, len);
		SMS_INIT_MSG(&msg3->xMsgHeader, MSG_SMS_DATA_DOWNLOAD_REQ,
		     sizeof(SmsMsgData_ST)+len);
		msg3->MemAddr = fw_addr+size;
		TWI_WRITE(gs_uc_tbuffer,msg3->xMsgHeader.msgLength)
		_WAIT_(10); // scheduled 100 msec
		read_sms_response(MSG_SMS_DATA_DOWNLOAD_RES, STATE_DATA_DOWNLOAD);
		if (0 !=len1) {
			if (len+DATA_DNLD_SIZE<=len1) {
				ll = DATA_DNLD_SIZE;
				len1 = len1- len;
			}
			else {
				if (DATA_DNLD_SIZE<=len1) {
					ll = DATA_DNLD_SIZE;
					len1 -= len;
				}
				else {
					ll = len1;
					len1 = 0;
				}
			}
			usb_read_buf1(fw_sms_rbuffer, ll);
			pb = fw_sms_rbuffer;
		}
		size += len;
		len = ( ( fw_len - size ) >= DATA_DNLD_SIZE )?DATA_DNLD_SIZE : fw_len- size;
		if (0==len) break;
	#include <assert.h>
		assert(ll == len);
 	}
 	sms_state = STATE_REQ_DATA_VALIDITY;
 	SmsMsgData3Args_ST *msg1 =(SmsMsgData3Args_ST*)gs_sms_tbuffer;
	SMS_INIT_MSG(&msg1->xMsgHeader, MSG_SMS_DATA_VALIDITY_REQ,
	     sizeof(SmsMsgData3Args_ST));
	msg1->msgData[0] = fw_addr;
	msg1->msgData[1] = fw_len;
	msg1->msgData[2] = 0;
	TWI_WRITE(gs_sms_tbuffer,msg1->xMsgHeader.msgLength)
	_WAIT_(1); // scheduled 10 msec
	read_sms_response(MSG_SMS_DATA_VALIDITY_RES, STATE_DATA_VALIDITY_RES);
}
#endif
#ifdef SMS_DVBT2_DOWNLOAD
  static void siano_tuning(uint32_t fc)
	{
		#if (MODE==DVBT2)
			SMS4470_send_device_init_lh(DVBT2_DEMODULATOR_TUNING); // init with DVBT2 standard
			delay_ms(500);
			SMS4470_RemovePidFilter_lh(0x2000);
			SMS4470_tune_lh(DVBT2_DEMODULATOR_TUNING,
													8/*8 mhz bandwidth*/,
													fc/*482000000*//*474000000*//*482 mhz fc*/);
			SMS4470_AddPidFilter_lh(0x2000);
		#elif (MODE==DVBT)
			SMS4470_send_device_init_lh(DVBT_DEMODULATOR_TUNING); // init with DVBT standard
			delay_ms(500);
			SMS4470_RemovePidFilter_lh(0x2000);
			SMS4470_tune_lh(DVBT_DEMODULATOR_TUNING,
													/*8*//*8 mhz bandwidth*/6/*6 mhz bandwidth*/,
													/*711750000*/fc/*482000000*//*474000000*//*482 mhz fc*/);
			SMS4470_AddPidFilter_lh(0x2000); // experiment to workaround unlocked issue, liyenho
		#endif
	}
#endif
#ifdef CONFIG_ADI_6612
static void regs_access()
{
	uint16_t tmp, tmpw, *pth = &tmp;
#if true // modified access scheme that can be translated by cpld, liyenho
		while (!usb_host_msg) {
			if (set_rf_params)
				return ; // it's done already
		} usb_host_msg = false;
		while (spi_tgt_done) ; spi_tgt_done = true;
		pio_set(PIOA, CPLD_6612_TRIG);
		delay_us(100); // gapped between two consecutive xfer
#endif
		dev_access *pr =(dev_access*)gs_uc_hrbuffer, *pt = (dev_access*)gs_uc_htbuffer;
		switch (pt->access) {
			case WRITE_BY_ADDR:
				pio_clear(PIOA, CPLD_6612_TRIG);
				*pth = 0xfe & (pt->addr<<1); // write access
				spi_tx_transfer(pth, 1, &tmpw, 1, 0/*ctrl/sts*/);
				while (spi_tgt_done) ; spi_tgt_done = true;
				*pth = (0x0ff & (pt->data[0]>>8)); // high byte
				spi_tx_transfer(pth, 1, &tmpw, 1, 0/*ctrl/sts*/);
				while (spi_tgt_done) ; spi_tgt_done = true;
				*pth = (0x0ff & pt->data[0]); // low byte
				spi_tx_transfer(pth, 1, &tmpw, 1, 0/*ctrl/sts*/);
				break;
			case READ_BY_ADDR: // does this require one more read as does in Tx?
				pio_clear(PIOA, CPLD_6612_TRIG);
				*pth = 0x1|(pt->addr<<1); // read access
				spi_tx_transfer(pth, 1, &tmpw, 1, 0);
				while (spi_tgt_done);  spi_tgt_done = true;
				spi_tx_transfer(pth, 1, &tmpw, 1, 0/*ctrl/sts*/); // high byte
				while (spi_tgt_done) ; spi_tgt_done = true;
				pr->data[0] = 0xff00 & (tmpw<<8);
				spi_tx_transfer(pth, 1, &tmpw, 1, 0/*ctrl/sts*/); // low byte
				while (spi_tgt_done) ;
				pr->data[0] |= (0xff & tmpw);
				break;
			default: //puts("Unexpected access mode! stop the write");
				spi_tgt_done= false; break;
		}
		while (spi_tgt_done) ; delay_us(1);
#if true // modified access scheme that can be translated by cpld, liyenho
		pio_set(PIOA, CPLD_6612_TRIG);
#endif
}
#endif
	void upgrade_sys_fw(uint8_t system_upgrade) {
			main_loop_on = true;	// to start up upgrade proc on host...
			if (1 == system_upgrade) {
				cpld_flash_map fdo ;
				// altera cpld upgrade
/*****************************************************/
				fdo.page_size =  16 * 1024 / 8;	// page size of max 10 CFM0 in byte
				// the rest fields are not used at all
/*****************************************************/
				pio_set(PIOA, PIO_PA0);
		extern void download_cpld_fw(cpld_flash_map* fdo, uint32_t *upgrade_fw_hdr );
				download_cpld_fw(&fdo, upgrade_fw_hdr );
				pio_clear(PIOA, PIO_PA0);
/*****************************************************/
				delay_ms(100);
				wdt_init(WDT, WDT_MR_WDRSTEN, 256/*1 sec*/, 0xfff) ;
				while (1); // wait for processor reset
			}
			else if (2 == system_upgrade) {
				// atmel upgrade
				wdt_init(WDT, WDT_MR_WDRPROC, 256/*1 sec*/, 0xfff) ;
				while (1); // wait for processor reset
			}
	}
static void init_4463()
{
	pio_set_peripheral(PIOA, PIO_PERIPH_B, PIO_PA27 | PIO_PA28 | PIO_PA29);
	spi_master_initialize(2, SPI7_MASTER_BASE, BOARD_FLEXCOM_SPI7);
	spi_set_clock_polarity(SPI7_MASTER_BASE, SPI_CHIP_SEL, 0 /*SPI_CLK_POLARITY*/);
	spi_set_clock_phase(SPI7_MASTER_BASE, SPI_CHIP_SEL, 1 /*SPI_CLK_PHASE*/);

	pio_configure(PIOB, PIO_INPUT, PIO_PB8, 0);    //RF_NIRQ
	//pio_configure(PIOB, PIO_INPUT, PIO_PB13, 0);    //RF_GPI00
	//pio_configure(PIOA, PIO_INPUT, PIO_PA18, 0);    //RF_GPI01
	pio_configure(PIOA, PIO_OUTPUT_1, PIO_PA15, 0); //RF_PWRDN
	pio_configure(PIOA, PIO_OUTPUT_1, PIO_PA30, 0); //RF_NSEL

	/* Initialize SI4463 Host_Int line */ // for time critical TDM process
	pmc_enable_periph_clk(ID_PIOB);
	pio_set_input(PIOB, SI4463_HOST_INT, PIO_PULLUP);
	pio_handler_set(PIOB, ID_PIOB, SI4463_HOST_INT,
	PIO_IT_AIME /*| PIO_IT_RE_OR_HL*/ | PIO_IT_EDGE, si4463_radio_handler);
	pio_enable_interrupt(PIOB, SI4463_HOST_INT);
	pio_handler_set_priority(PIOB, PIOB_IRQn, 1/*long latency event*/);

	radio_mon_rxcnt = 0;
	delay_ms(50); // Extra waiting for SPI7 Master Ready
}
#ifdef RX_SPI_CHAINING
  static void start_siano_spi(bool restart)
	{
	  int lvl1, lvl2, lvl3, lvlr;
	  irqflags_t flags;
		flags = cpu_irq_save();
		  lvl1 = spibuff_wrptr_filled;
		  lvl2 = spibuff_wrptr_currentlyfilling;
		  lvl3 = spibuff_wrptr_yettobefilled;
		  lvlr = spibuff_rdptr;
		cpu_irq_restore(flags);
		int del = abs(lvl1 - lvlr);
		flags = cpu_irq_save();
		if (restart) { // back off don't reset
			if ((del < 2) || (del == GRAND_BUFFER_BLKS-1))
				goto reset;  // re-initialize don't back off
			spibuff_wrptr_filled = lvl1 - 1;
			 if (GRAND_BUFFER_BLKS < spibuff_wrptr_filled)
				spibuff_wrptr_filled = GRAND_BUFFER_BLKS-1;
			spibuff_wrptr_currentlyfilling = spibuff_wrptr_filled+1;
			if (GRAND_BUFFER_BLKS <= spibuff_wrptr_currentlyfilling)
				spibuff_wrptr_currentlyfilling = GRAND_BUFFER_BLKS-1;
			spibuff_wrptr_yettobefilled = spibuff_wrptr_currentlyfilling+1;
			if (GRAND_BUFFER_BLKS <= spibuff_wrptr_yettobefilled)
				spibuff_wrptr_yettobefilled = GRAND_BUFFER_BLKS-1;
		} else { // initialize
reset: spibuff_wrptr_filled = 0; // initialize circular DMA buffer pointers
			spibuff_wrptr_currentlyfilling = 1;
			spibuff_wrptr_yettobefilled = 2;
			spibuff_rdptr = 0;
		}
		dbg_spififo_lvl_max = 0;
		cpu_irq_restore(flags);
		//	do not use spi_rx_transfer(*)!!! or upon 1st get in rtt handler, it began all the wrong doings! liyenho
		spi_rx_transfer_ts((uint8_t*)gs_uc_rbuffer+spibuff_wrptr_filled*I2SC_BUFFER_SIZE, I2SC_BUFFER_SIZE/2,
			(uint8_t*)gs_uc_rbuffer+spibuff_wrptr_currentlyfilling*I2SC_BUFFER_SIZE,
			I2SC_BUFFER_SIZE/2, 1);
		delay_ms(10);  //YH: must delay to keep dma alignment
		pio_set(PIOB, PIO_PB9); // enable TS gate
		spidma_active = true;
	}
#endif
//////////////////////////////////////////////////////////////////////////////
/*! \brief Main function. Execution starts here.*/
//////////////////////////////////////////////////////////////////////////////
int main(void)
{
	volatile uint8_t *pusbe=(uint8_t*)gs_uc_tbuffer+sizeof(gs_uc_tbuffer),
										*pusbs=(uint8_t*)gs_uc_tbuffer+0,
#ifdef CONF_BOARD_USB_TX
										*pusb=gs_uc_rbuffer;
#elif defined(CONF_BOARD_USB_RX)
										*pusb=gs_uc_tbuffer;
#endif
	uint32_t i, n, tdel, tcurr, first_in = true;
#if defined(MEDIA_ON_FLASH) || defined(CONFIG_ON_FLASH) || defined(FWM_DNLD_DBG)
	uint32_t page_addr_bypass = IFLASH_ADDR + IFLASH_SIZE -  // to store flash media download flag, liyenho
												(NUM_OF_FPGA_REGS+ NUM_OF_ATMEL_REGS) -4/*sizeof(int)*/-1 ;
	volatile uint32_t le, lm, ul_rc;
	volatile uint32_t ul_page_end, ul_page_addr=PAGE_ADDRESS ;
	const uint32_t erlen = SECTOR_SIZE_L; // last two sector size must be 128 kbytes
	static uint32_t ul_page_addr_c= IFLASH_ADDR + IFLASH_SIZE - /*is still some memory corrupted?*/
														(NUM_OF_FPGA_REGS+ NUM_OF_ATMEL_REGS);
#elif defined(FWM_DNLD_DBG)
	volatile uint32_t ul_page_end, ul_page_addr=PAGE_ADDRESS ;
#endif

   //Digibest Based variable declarations
	BOOL Result;
	BOOL lockStatus;

	//Chaining Related Variables
	unsigned  char spibuffnext=0;
	unsigned char dbg_spififo_lvl;

	//control path related
	U8 sndflag;
	U8 radioini_failcnt=0;
	uint16_t ledoncnt;
	U8 ctrl_ledstate=0;
    int ctrlrxloopcnt;
//----------------------------------------------------------------------------

	irq_initialize_vectors();
	cpu_irq_enable();
#if 0
	// Initialize the sleep manager
	sleepmgr_init();
#endif
#if !SAM0
	sysclk_init();
	/* Ensure all priority bits are assigned as preemption priority bits. */
	//NVIC_SetPriorityGrouping(0);
	board_init();
#else
	system_init();
#endif
	ui_init();
	delay_ms(10); //debugging break
	ui_powerdown();
#if 0 //ndef CONF_BOARD_EVM
	/* Initialize the console UART. */
	configure_console(); // will be used but inited along with cdc comm module, liyenho
#endif
#if (defined(MEDIA_ON_FLASH) && !defined(NO_USB)) || defined(CONFIG_ON_FLASH) || defined(FWM_DNLD_DBG)
	/* Initialize flash: 6 wait states for flash writing. */
	ul_rc = flash_init(FLASH_ACCESS_MODE_128, 6);
	if (ul_rc != FLASH_RC_OK) {
		//printf("-F- Initialization error %lu\n\r", (UL)ul_rc);
		return 0;
	}
	//printf("-I- Unlocking page: 0x%08x\r\n", ul_page_addr);
	ul_rc = flash_unlock(ul_page_addr,
			ul_page_addr + NUM_OF_PAGES*IFLASH_PAGE_SIZE - 1, 0, 0);
	if (ul_rc != FLASH_RC_OK) {
		//printf("-F- Unlock error %lu\n\r", (UL)ul_rc);
		return 0;
	}
#endif
	/* Configure SPI interrupts for slave? only. */
#if 1 //def CONF_BOARD_EVM, for video now
	NVIC_DisableIRQ(SPI_IRQn);  // spi5 peripheral instance = 21, liyenho
	NVIC_ClearPendingIRQ(SPI_IRQn);
	NVIC_SetPriority(SPI_IRQn, 1);
	NVIC_EnableIRQ(SPI_IRQn);
#endif
	NVIC_DisableIRQ(SPI0_IRQn);  // spi3/7 peripheral instance = 19/7, liyenho
	NVIC_ClearPendingIRQ(SPI0_IRQn);
	NVIC_SetPriority(SPI0_IRQn, 1);
	NVIC_EnableIRQ(SPI0_IRQn);
 #ifdef RADIO_SI4463
	 NVIC_DisableIRQ(PIOB_IRQn);  // piob radio instance = 12, liyenho
	 NVIC_ClearPendingIRQ(PIOB_IRQn);
	 NVIC_SetPriority(PIOB_IRQn, 1);
	 NVIC_EnableIRQ(PIOB_IRQn);
	 NVIC_DisableIRQ(SPI7_IRQn);  // spi7 peripheral instance = 7, liyenho
	 NVIC_ClearPendingIRQ(SPI7_IRQn);
	 NVIC_SetPriority(SPI7_IRQn, 1);
	 NVIC_EnableIRQ(SPI7_IRQn);
 #endif
	if (SysTick_Config(sysclk_get_cpu_hz() / 500)) { // 2 msec tick
		//puts("-E- Systick configuration error\r");
		while (1) {
			/* Capture error */
		}
	}
 #ifdef RECV_SMS4470
 	pio_set_output(PIOB, PIO_PB9, LOW, DISABLE, ENABLE); //stop TS gate
 	delay_ms(10); // flush all data from Pipe
 #endif
	twi_master_initialize(TWI_CLK); // communicate with sms4470, liyenho
#if false // not available in actual design, liyenho
	/* Initialize SMS4470 Host_Int line */
	pmc_enable_periph_clk(ID_PIOA);
	pio_set_input(PIOA, SMS_HOST_INT, PIO_PULLUP);
	 pio_handler_set(PIOA, ID_PIOA, SMS_HOST_INT,
	 			PIO_IT_AIME | PIO_IT_RE_OR_HL | PIO_IT_EDGE, twi_sms4470_handler);
	 pio_enable_interrupt(PIOA, SMS_HOST_INT);
	pio_handler_set_priority(PIOA, PIOA_IRQn, 1/*long latency event*/);
#endif
	spi_master_initialize(0, SPI0_MASTER_BASE, BOARD_FLEXCOM_SPI0);// 6612 ctrl pipe
	spi_slave_initialize();  // shall be applied on sms video pipe
	g_p_spim_pdc[1] = g_p_spis_pdc;
 #ifdef I2C_FAKE_POLL
	//i2s_tx_initialize(); // fake i2c clk gen
	pio_set_output(PIOA, PIO_PA4, LOW, DISABLE, ENABLE); // disable trigger
 #endif
#ifdef CONFIG_ON_FLASH
		if (1!=*(uint8_t*)ul_page_addr_bootapp) {
			erase_last_sector() ;
			CHECKED_FLASH_WR(
				IFLASH_ADDR + IFLASH_SIZE-sizeof(backup),
				backup, NUM_OF_FPGA_REGS +1 +4 +2)
			bootapp = 1; // boot right into app
			CHECKED_FLASH_WR(ul_page_addr_bootapp, &bootapp, 1)
			CHECKED_FLASH_WR(
				IFLASH_ADDR + IFLASH_SIZE-NUM_OF_ATMEL_REGS +3,
				backup +NUM_OF_FPGA_REGS +1 +4 +3,
				sizeof(backup)-NUM_OF_FPGA_REGS-1 -4 -3)
		}
#endif
	// Start USB stack to authorize VBus monitoring
	udc_start();
system_restart:  // system restart entry, liyenho
	pio_set_output(PIOA, PIO_PA0, LOW, DISABLE, ENABLE); // flag of cpld remote upgrade, liyenho
	system_main_restart = false;
	set_rf_params = false ;
#ifdef  RADIO_SI4463
	si4463_radio_started = false;
	ctrl_tdma_enable = false;
	timedelta_reset = true;
	// start up with constant offset, we'll modify to adopt pairing reset later, liyenho
	fhop_base=0;
	fhop_offset = HOP_2CH_ENABLE?WRAP_OFFSET(HOP_2CH_OFFSET0):fhop_base;
	fhop_dir = true;  // hop forward when startup
	fhop_idx= 0 ; // used for pattern access
	hop_state = IDLE;
#endif
#ifdef RECV_SMS4470
 #ifndef RX_SPI_CHAINING
	i2c_read_cb_on = false ;
	first_in = true; // be sure to reset 'first time flag' too
 #endif
	pio_clear(PIOB, PIO_PB9); // disable TS gate
	spi_disable(g_p_spis_pdc); // I think that is only spi need to be reset? liyenho
	Frontend_Uninitialization(); // unset siano setting to force reinitialization
#endif
#ifndef MEDIA_ON_FLASH
 	while (!udi_cdc_data_running) ; // wait for cdc data intf ready, liyenho
#endif
  // enable cpu counter
  *DEMCR = *DEMCR | 0x01000000;
  *DWT_CYCCNT = 0; // reset the counter
  *DWT_CONTROL = *DWT_CONTROL | 1 ; // enable the counter
#ifdef MEDIA_ON_FLASH
 #ifndef NO_USB
		int w = *(uint32_t*)(~0x3&page_addr_bypass);
		int s = (page_addr_bypass & 0x3) * 8;
		if (true!= (0xff & (w >>=  s))) {
			while (!udi_cdc_data_running) ; // wait for cdc data intf ready, liyenho
		}
 #endif
#endif
 #ifdef CONFIG_ADI_6612
		pio_set_output(PIOA, CPLD_6612_TRIG, HIGH, DISABLE, ENABLE); // extra trigger line for 6612 access with cpld, liyenho
 		while (!num_6612_regs && !system_upgrade) ; // wait for host to init config data len
		if (system_upgrade)
			upgrade_sys_fw(system_upgrade);
		// all automated/taken care by host, liyenho
		do {
			regs_access();
	 	} while(!set_rf_params && !system_upgrade);
		if (system_upgrade)
			upgrade_sys_fw(system_upgrade);
		set_rf_params = false;
    	num_6612_regs = 0;
 #endif
#ifdef RECV_SMS4470
		usb_data_done = true;
	  (void) REG_SPI5_SR; // read to clear OVRES bit
	  // chain at least two rx xfer for ts stream
  #ifndef RX_SPI_CHAINING // but in fact this is linked dma setup
	  // chain at least two rx xfer for ts stream
		spi_rx_transfer_ts(gs_uc_rbuffer, GRAND_BUFFER_SIZE/2/2,
				(uint8_t*)gs_uc_rbuffer+GRAND_BUFFER_SIZE/2, I2SC_BUFFER_SIZE/2,
				1/*video*/);
  #endif
/*
		MATRIX->MATRIX_SCFG[3] = // make peripheral bridge the default master to be pdc
			(MATRIX->MATRIX_SCFG[3] & ~(MATRIX_SCFG_FIXED_DEFMSTR_Msk | MATRIX_SCFG_DEFMSTR_TYPE_Msk)) |
			(MATRIX_SCFG_FIXED_DEFMSTR(0x2) | MATRIX_SCFG_DEFMSTR_TYPE_FIXED);
		// lower down master, system bus and crccu, on peripheral bridge access priority,
			// the process access already has lowest priority, so master pdc own highest priority
		MATRIX->MATRIX_PRAS3 =
			(MATRIX->MATRIX_PRAS3 & ~(MATRIX_PRAS3_M1PR_Msk | MATRIX_PRAS3_M3PR_Msk))
			| MATRIX_PRAS3_M1PR(0x2) | MATRIX_PRAS3_M3PR(0x2);
*/
		// setup comparator to track 0x47 in ts stream
//		REG_SPI5_CMPR = (((uint32_t)0x47)<<16) | ((uint32_t)0x47) ;
//		REG_SPI5_MR = (REG_SPI5_MR & ~SPI_MR_CMPMODE) |SPI_MR_CMPMODE_START_CONDITION;
#endif
#ifdef RECV_SMS4470
		// restart the comparison trigger
//		REG_SPI5_CR = REG_SPI5_CR | SPI_CR_REQCLR;
#endif
//////////////////////////////////////////////////////////////////////////////
#ifdef DIGIBEST_DOWNLOAD
//Kevin
	if (1) {
		uint8_t *align_pb = fw_sms_rbuffer;
		bool hit_ab = FALSE, hit_00 = FALSE, hit_05 = FALSE;
		for (int kk = 0; kk < 500;) {
			usb_read_buf1(align_pb, 1);

			if (align_pb[0] == 0xAB) {
				hit_ab = TRUE; hit_00 = FALSE; hit_05 = FALSE;
				} else if (align_pb[0] == 0x00 && hit_ab) {
				hit_00 = TRUE; hit_05 = FALSE;
				} else if (align_pb[0] == 0x05 && hit_ab && hit_00) {
				hit_05 = TRUE;
				} else {
				hit_ab = FALSE; hit_00 = FALSE; hit_05 = FALSE;
			}

			if (hit_ab && hit_00 && hit_05)
			break;
		}
	}
//Yendo Digibest testing
  Result = Frontend_Initialization();
  if(Result != TRUE)
    while(1){}
  Result = Frontend_Open(TWO_DIVERSITY_GROUP,DVBT_WORKING_MODE,&TwoDiversityGroupFrontendID);
  //Result = Frontend_Open(TWO_DIVERSITY_GROUP,DVBT2_WORKING_MODE,&TwoDiversityGroupFrontendID);
  if(Result != TRUE)
	while(1){}
 	 TuningParameter.TuningFrequency = 482000000;   //482001000, 5000,10000,50000,100000, 481995000-BAD
	                                     //473985000 gives 0x074 Carrier offset (digibest brd)
													//474010000 gives -8254
													//474020000 gives -6232
													// every 10000hz gives 2000 carrierOffset increase
													//474045000 gives -298 (rx v1 brd)
 	 TuningParameter.BandwidthType = BANDWIDTH_6_MHZ;
 	 //TuningParameter.BandwidthType = BANDWIDTH_8_MHZ;
 	 TuningParameter.DemodulatorTuningFeature = DVBT_DEMODULATOR_TUNING;
 	 //TuningParameter.DemodulatorTuningFeature = DVBT2_DEMODULATOR_TUNING;

 	 Result = Frontend_Tuning(TwoDiversityGroupFrontendID,&TuningParameter);

  if(Result == FALSE)
    while(1){}
#endif
//////////////////////////////////////////////////////////////////////////////
#ifndef DIGIBEST_DOWNLOAD  //Liyen Siano Management ---------------------------------------------
										#if defined(SMS_DVBT2_DOWNLOAD)
											dvbt.SMS4470DetectNoSignalFlag = (bool)-1; // invalidate
											sms_state = STATE_INITIALIZE;
											SMS4470_get_version_lh();
											sms_access *phdr = (sms_access*)fw_sms_rbuffer;
											 uint16_t chip_model = *((U16*)(phdr+1));
											 uint8_t chip_metal = ((U8*)(phdr+1))[3];
											 //printf("sms device model = 0x%x, sms device step = %d\n",
	 											//chip_model, chip_metal); // does this make sense at all? must watch during debug... liyenho
											 uint8_t chk[4] ;  // to fill in the check bytes for align usb bulk xfer, liyenho
										#if (MODE==DVBT2)
											 /* download DVBT2 firmware */
	 											chk[0] = 0x00;
	 											chk[1] = 0x0c;
	 											chk[2] = 0xf0;
	 											chk[3] = 0x82;
											 download_sms_fw(/*0x6e*/0xfd, chk );
										#elif (MODE==DVBT)
											 /* download DVBT firmware data */
	 											chk[0] = 0x28;
	 											chk[1] = 0x25;
	 											chk[2] = 0x64;
	 											chk[3] = 0x29;
											download_sms_fw(/*0x93*/0x35, chk );
										#endif
											memset(sms4470_fw_hdr, -1, sizeof(int)); // perpare to wait for another download
											delay_ms(500);
											SMS4470_get_version_lh();
											memcpy(&smsdev,
												(uint8_t*)fw_sms_rbuffer+sizeof(sms_access),
												sizeof(smsdev));
										/*
											printf("ChipModel : 0x%x\n",pVer->xVersion.ChipModel);
											printf("FirmwareId : %d\n",pVer->xVersion.FirmwareId);
											printf("SupportedProtocols : 0x%x\n",pVer->xVersion.SupportedProtocols);
											printf("FwVer(Major) : %d\n",pVer->xVersion.FwVer.Major);
											printf("FwVer(Minor) : %d\n",pVer->xVersion.FwVer.Minor);
											printf("RomVer(Major) : %d\n",pVer->xVersion.RomVer.Major);
											printf("RomVer(Minor) : %d\n",pVer->xVersion.RomVer.Minor);
											printf("TextLabel : %s\n",pVer->xVersion.TextLabel);
										*/
											SMS4470_set_polling_mode_lh();
											//MSG_SMS_I2C_SHORT_STAT_IND came right after this...
											SMS4470_enable_ts_interface_lh();
											siano_tuning(fc_siano_tuned);
											// sms4470 now should start to pump TS packet stream...
											dvbt.Sms4470CurrentWorkingMode = DVBT_WORKING_MODE;
											dvbt.SMS4470DetectNoSignalFlag = false; // may be present later, it's not necessary there
											sms_state = STATE_NORMAL_OPS;
										#endif
									  while (!usb_write_start && !system_upgrade) ; // wait for host rec to start, liyenho
										if (system_upgrade)
											upgrade_sys_fw(system_upgrade);
#endif //Liyen Siano Management---------------------------------------------------
	init_4463();  // this init function has to be after siano came up!? liyenho
    pio_set_output(PIOA, PIO_PA17, HIGH, DISABLE, ENABLE);  //setup ctrl led monitoring control
	pio_set(PIOA, PIO_PA17);  //turns off wifiled
#ifdef RADIO_CTRL_AUTO
		/* init radio stats obj */
		{
			for (int j=0; j<CTRL_CTX_LEN; j++) {
				r4463_sts.ctrl_bits_ctx[j] = LONG_RNG;
			}
			r4463_sts.bw_ctrl_bits = LONG_RNG;
			r4463_sts.errPerAcc =0;
			r4463_sts.loop_cnt= 0;
		}
		vRadio_Init();
	#ifdef CONFIG_ON_FLASH
		uint8_t median = *(uint8_t*)ul_page_addr_ctune;
		if (0xff != median) // adjust cap bank value per stored const
			recalibrate_capval((void*)ul_page_addr_mtemp, median);
	#endif
		{
			  for(int i=0;i<2*RDO_ELEMENT_SIZE;i++)
			  gs_rdo_tpacket[i]=0xc5c5c5c5;
		}
		si4463_radio_started = true;
		ctrl_tdma_enable = true;
#endif
	main_loop_on = true;  // enter run time stage, liyenho

	// The main loop manages
#if defined(RECV_SMS4470)
  #if defined(RX_SPI_CHAINING)
		mon_spidmachainfail_cnt  = 0;
		mon_ts47bad_cnt = 0;
	// from spi enabled to start dma can't be longer than 488 us, bad design, liyenho
		configure_rtt(16); // arm 500us isr for SPI/USB data pipe processing
  #else //!defined(RX_SPI_CHAINING)
		sms4470_usb_ctx.usb_failcnt = 0;
		sms4470_usb_ctx.skip_rd = false;
		sms4470_usb_ctx.skip_wr = true;
		sms4470_usb_ctx.buf_start = (uint8_t*)gs_uc_rbuffer;
		sms4470_usb_ctx.buf_last = (uint8_t*)gs_uc_rbuffer+GRAND_BUFFER_SIZE-I2SC_BUFFER_SIZE;
		sms4470_usb_ctx.buf_end = (uint8_t*)gs_uc_rbuffer+GRAND_BUFFER_SIZE;
		sms4470_usb_ctx.spi_buf_rd = (uint8_t*)gs_uc_rbuffer+GRAND_BUFFER_SIZE/2+I2SC_BUFFER_SIZE;
		sms4470_usb_ctx.usb_buf_wr = (uint8_t*)gs_uc_rbuffer;
  #endif
	// now allow video spi to be active, liyenho
	spi_enable(SPI_SLAVE_BASE);
#ifndef RX_SPI_CHAINING
	pio_set(PIOB, PIO_PB9); // enable TS gate
#else
	// pull over to here instead init inside main loop to get rid of time constraint, liyenho
	start_siano_spi(false);
#endif
#endif

	while (true) {
		if (system_main_restart ) goto system_restart;
		if (system_upgrade)
			upgrade_sys_fw(system_upgrade);
		if (!stream_flag) goto _reg_acs; // stop TS stream if flag isn't true, liyenho

#ifdef RX_SPI_CHAINING
    //VIDEO SPI DMA Management ----------------------------------------------------
	//YH:   1. spi_rx_transfer() starts SPI DMA transfer
	//      2. set spidma_active = TRUE.  (Misleading, it is saying, spi DMA is running)
	//      3  while SPI dma is running, start USB transfer usb_write_buf()
	//      4. when done, goto while(usb_data_done && siano_wake_up) and wait ...
	//      5. spi_handler() - triggers at dma end, set usb_data_done=FALSE
	//      6. release from while() wait, and move to arm the spi DMA again (spi_rx_transfer())
	if(spidma_active != TRUE)
		start_siano_spi(true);

	  Frontend_GetLockStatus(TwoDiversityGroupFrontendID,&lockStatus);

	  //Ctrl led control -------------------------------------------------
	  if(ctrl_sndflag >0 )
	    ctrl_sndflag--;
	  if(ctrl_sndflag == (100-1) ) {
		if(ctrl_ledstate ==0)
	    {
			pio_set(PIOA, PIO_PA17);
			ctrl_ledstate=1;
			}
		else
		{
			pio_clear(PIOA, PIO_PA17);
			ctrl_ledstate=0;
			} //force blinking if ctrl too fast
	  }

	  if(ctrl_sndflag ==1)  {
	    pio_clear(PIOA, PIO_PA17);
		ctrl_ledstate = 0;  }

	  // ctrl RX Processing -----------------------------------------------
	  if(si4463_radio_started)
	  {
  #ifdef TEMPERATURE_MEASURE
			static int once = false;
			if (!once) {
				vRadio_StartTx(pRadioConfiguration->Radio_ChannelNumber); // continued transmit & stop listening, liyenho
				once = 1;
			}
  #endif
		 if (si4463_factory_tune.calib_gated) {
			tcurr = *DWT_CYCCNT;
			tm_delta(si4463_factory_tune.tm_curr, tcurr, tdel)
			if (CALIB_DWELL_INTV<=tdel) {
				if (0x7f==si4463_factory_tune.cap_curr) {
					irqflags_t flags;
					flags = cpu_irq_save();
					  si4463_factory_tune.calib_gated = false;
					cpu_irq_restore(flags);
					si4463_factory_tune.tm_ended = tcurr;
					si4463_factory_tune.median =
						(si4463_factory_tune.lower+
						si4463_factory_tune.upper+1) / 2;
	#ifdef CONFIG_ON_FLASH
					erase_last_sector();
					CHECKED_FLASH_WR(
						IFLASH_ADDR + IFLASH_SIZE-sizeof(backup),
						backup, NUM_OF_FPGA_REGS +1 +4)
					CHECKED_FLASH_WR(ul_page_addr_ctune,
																			&si4463_factory_tune.median,
																			1/*1 byte flag*/)
					uint8_t ctemp = get_si446x_temp();
					CHECKED_FLASH_WR(ul_page_addr_mtemp,
																			&ctemp, 1/*1 byte flag*/)
					CHECKED_FLASH_WR(
						IFLASH_ADDR + IFLASH_SIZE-NUM_OF_ATMEL_REGS +2,
						backup +NUM_OF_FPGA_REGS +1 +4 +2,
						sizeof(backup)-NUM_OF_FPGA_REGS-1 -4 -2)
	#endif
					tune_cap_str[CAP_VAL_POS] = si4463_factory_tune.median;
					if (radio_comm_SendCmdGetResp(sizeof(tune_cap_str), tune_cap_str, 0, 0) != 0xFF) {
						while (1) {
							; // Capture error
						}
					}
					si4463_factory_tune.tm_curr = *DWT_CYCCNT; // record startup time for recurrent adjustment
					ctrl_tdma_enable = true;	// turn flag back on
					goto tune_done;
				}
				si4463_factory_tune.tm_curr = tcurr;
				if (si4463_factory_tune.calib_det_rx) {
					if (CAP_TUNE_THR<si4463_factory_tune.calib_det_rx) {
					  minmax(si4463_factory_tune.lower,
										si4463_factory_tune.upper,
										si4463_factory_tune.cap_curr)
					}
					si4463_factory_tune.calib_det_rx = 0; // invalidated
				}
				si4463_factory_tune.cap_curr += 1;
				tune_cap_str[CAP_VAL_POS] = si4463_factory_tune.cap_curr;
				if (radio_comm_SendCmdGetResp(sizeof(tune_cap_str), tune_cap_str, 0, 0) != 0xFF) {
					while (1) {
						; // Capture error
					}
				}
			}
		 }
		 else if (si4463_factory_tune.calib_req_h) {
			 const uint8_t freq_reset_str[] = {RF_FREQ_CONTROL_INTE_8}; // tune frequency on 915 mhz
			if (radio_comm_SendCmdGetResp(sizeof(freq_reset_str), freq_reset_str, 0, 0) != 0xFF) {
				while (1) {
					; // Capture error
				}
			}
			const uint8_t afc_gear_str[] = {0x11, 0x20, 0x03, 0x30, 0x25, 0x16, 0xC0}; // use narrower tracking range on AFC limiter
			if (radio_comm_SendCmdGetResp(sizeof(afc_gear_str), afc_gear_str, 0, 0) != 0xFF) {
				while (1) {
					; // Capture error
				}
			}
			si4463_factory_tune.tm_started = *DWT_CYCCNT;
			si4463_factory_tune.tm_curr = si4463_factory_tune.tm_started;
			si4463_factory_tune.calib_req_h = false;
			si4463_factory_tune.calib_det_rx = 0; // invalidated
			si4463_factory_tune.cap_curr = 0x0; // start from lowest possible cap value
			tune_cap_str[CAP_VAL_POS] = si4463_factory_tune.cap_curr;
			if (radio_comm_SendCmdGetResp(sizeof(tune_cap_str), tune_cap_str, 0, 0) != 0xFF) {
				while (1) {
					; // Capture error
				}
			}
		  	vRadio_StartRX(pRadioConfiguration->Radio_ChannelNumber,
		  		RADIO_PKT_LEN);  // enter listening mode
			si4463_factory_tune.calib_gated = true;  // let si4463_radio_handler() begin to receive
		 }
tune_done:
	  if(bMain_IT_Status == SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_PACKET_SENT_PEND_BIT)
	  {
		  // A TX was just finished  Need to start RX

		  #ifdef CTRL_DYNAMIC_MOD
		  static BW_CTRL prev_ctrl_bits = (BW_CTRL) -1;
			/********************************************************/
			if (prev_ctrl_bits != r4463_sts.bw_ctrl_bits && NEUTRAL != r4463_sts.bw_ctrl_bits) {
				if (0 != range_mode_configure(SHORT_RNG!=r4463_sts.bw_ctrl_bits)) {
					//puts("error from range_mode_configure()");
					/*return*/ ; // time out, don't proceed
				}
			}
			/********************************************************/
			prev_ctrl_bits = r4463_sts.bw_ctrl_bits;
		  #endif  //CTRL_DYNAMIC_MOD
	  }
	  if((bMain_IT_Status == SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_PACKET_RX_PEND_BIT))
	  {
		  // An RX data was received, and an interrupt was triggered.
			/************************************************************/
			#ifdef CTRL_DYNAMIC_MOD
			{	static unsigned int recv_cnt = 0;
				BW_CTRL ctrl_bits_tmp;
				uint8_t j, err, cks=0, *pb = (uint8_t*)gp_rdo_rpacket_l;
				for (j=0;j<pRadioConfiguration->Radio_PacketLength-1;j++)
				{	cks ^= *pb++;  } // generate 2's mod checksum
				cks &= CTRL0_IMSK;  // took high 6 bits
				cks ^= CTRL_MSK & (*pb<<CTRL_BITS); // then bw ctrl bits @ end
				err = (CHKSM_MSK & *pb) ^ cks;
				uint32_t ew;
				ew = error_weight_cks6b[err>>CTRL_BITS];
				//lapse = recv_cnt * CTRL_RX_LOOP_LATENCY;
				// update error accum per time lapse
				if (initCnt > recv_cnt) {
					r4463_sts.errPerAcc = r4463_sts.errPerAcc * Ewim + ew * Ewis;
					r4463_sts.errPerAcc >>= initScl;
				}
				else if (initCnt <= recv_cnt && inteCnt > recv_cnt) {
					r4463_sts.errPerAcc = r4463_sts.errPerAcc * Ewmm + ew * Ewms;
					r4463_sts.errPerAcc >>= inteScl;
				}
				else {
					r4463_sts.errPerAcc = r4463_sts.errPerAcc * Ewnm + ew * Ewns;
					r4463_sts.errPerAcc >>= normScl;
				}
				// compute range estimation
				if (shThr >= r4463_sts.errPerAcc) {
					if (SHORT_RNG == (CTRL0_MSK & *pb)) {
						if (shThr >= ew)
							ctrl_bits_tmp = SHORT_RNG;
						else
							ctrl_bits_tmp = NEUTRAL;
					}
					else
						ctrl_bits_tmp = NEUTRAL;
				}
				else if (lgThr < r4463_sts.errPerAcc) {
					ctrl_bits_tmp = LONG_RNG;
				}
				else {
					if (LONG_RNG == (CTRL0_MSK & *pb))
						ctrl_bits_tmp = LONG_RNG;
					else
						ctrl_bits_tmp = NEUTRAL;
				}
				// perform hangover procedure
				switch(ctrl_bits_tmp) {
					case SHORT_RNG :
						for (j=0; j<CTRL_CTX_LEN; j++) {
							if (SHORT_RNG != r4463_sts.ctrl_bits_ctx[j])
								break;
						}
						if (CTRL_CTX_LEN == j)
							r4463_sts.bw_ctrl_bits = SHORT_RNG;
						else  { // in hysteresis region
							for (j=0; j<CTRL_CTX_LEN; j++) {
								if (NEUTRAL != r4463_sts.ctrl_bits_ctx[j])
									break;
							}
							if (CTRL_CTX_LEN == j)
								r4463_sts.bw_ctrl_bits = SHORT_RNG;
							else  // cond above shall break continuous neutral case...
								r4463_sts.bw_ctrl_bits = NEUTRAL;
						}
						break;
					case LONG_RNG :
						for (j=0; j<CTRL_CTX_LEN; j++) {
							if (LONG_RNG == r4463_sts.ctrl_bits_ctx[j])
								break; // found a LG req in ctx
						}
						if (CTRL_CTX_LEN != j)
							r4463_sts.bw_ctrl_bits = LONG_RNG;
						else // close in hysteresis region
							r4463_sts.bw_ctrl_bits = NEUTRAL;
						break;
					default : /*NEUTRAL*/
						r4463_sts.bw_ctrl_bits = NEUTRAL;
							if (NEUTRAL == r4463_sts.ctrl_bits_ctx[0] &&
								NEUTRAL == r4463_sts.ctrl_bits_ctx[1]) {
								// fall back to long range mode if two neutral seen in a row
								r4463_sts.bw_ctrl_bits = LONG_RNG;
								break;
							}
						break;
				}
				// bump up recv_cnt
				recv_cnt = recv_cnt + 1;
			}
			r4463_sts.loop_cnt = 0;  //reset
			#endif //CTRL_DYNAMIC_MOD
	  } else { // not receiving anything yet
		  #ifdef CTRL_DYNAMIC_MOD
		  unsigned int lapse = r4463_sts.loop_cnt *CTRL_RX_LOOP_LATENCY;
			if (LONG_RNG!=r4463_sts.bw_ctrl_bits && CTRL_MON_PERIOD <lapse) {
				r4463_sts.bw_ctrl_bits = NEUTRAL;
				if (CTRL_FAIL_PERIOD <lapse) {
					r4463_sts.bw_ctrl_bits = LONG_RNG;
					if (0 != range_mode_configure(LONG_RNG)) {
						// puts("error on range_mode_configure()");
						//return ; // time out, don't proceed
					}
					r4463_sts.loop_cnt = 0;
				}
			}
			#endif //#ifdef CTRL_DYNAMIC_MOD
		}
		#ifdef CTRL_DYNAMIC_MOD
		// update ctrl bits history
		for (int j=CTRL_CTX_LEN-1; j>0; j--)
			r4463_sts.ctrl_bits_ctx[j] = r4463_sts.ctrl_bits_ctx[j-1];
		*r4463_sts.ctrl_bits_ctx = r4463_sts.bw_ctrl_bits;
		r4463_sts.loop_cnt = r4463_sts.loop_cnt + 1;
		/************************************************************/
		#endif
		}//if(si4463_radio_started)
		//spidma fallout processing ---------------------------------------
		if(spidma_active != TRUE)
		{
            //spidmachainfail_cnt++; //this is now updated in spi_handler, more accurate
		}
#endif //RX_SPI_CHAINING

#ifndef RX_SPI_CHAINING
	if (true == first_in)	{
		volatile uint32_t cycle_now, cycle_delta;
		// enable the use DWT
		*DEMCR = *DEMCR | 0x01000000;
		// Reset cycle counter
		*DWT_CYCCNT = 0;
		// enable cycle counter
		*DWT_CONTROL = *DWT_CONTROL | 1 ;
		cycle_cnt = *DWT_CYCCNT;
		twi_master_initialize(50000); // slow down i2c bus for more flexible timing, liyenho
 //cycle_now = *DWT_CYCCNT;
 //cycle_delta = cycle_now - cycle_cnt;
		i2c_read_cb_on = true;  // turn on i2c cb ext, liyenho
		first_in = false ;
	}
	SMS4470_check_signal(&sms_dvbt_lock); // apply callback ext, liyenho
		usbfrm = usbfrm + 1;
#endif //SPI_CHAINING
_reg_acs:
  		if (usb_host_msg && !system_main_restart) {	// host ctrl/sts link with fpga/sms process, liyenho
			usb_host_msg = false;
//#define TEST_FLASH
 #ifdef TEST_FLASH
	uint32_t shf, wtmp;
 #endif
#if defined(CONFIG_ADI_6612)
			dev_access *pr=(dev_access*)gs_uc_hrbuffer, *pt = (dev_access*)gs_uc_htbuffer;
			uint16_t tmp, tmpw, *pth = &tmp;
  #ifdef I2C_FAKE_POLL
  		while(pio_get(PIOA, PIO_TYPE_PIO_OUTPUT_0, PIO_PA4));
  #endif
			while (spi_tgt_done) ; // flush any pending spi xfer
				spi_tgt_done = true;
			switch (pt->access) {
				case READ_BY_ADDR:
							pio_clear(PIOA, CPLD_6612_TRIG);
							*pth = 0x1|(pt->addr<<1); // read access
							spi_tx_transfer(pth, 1, &tmpw, 1, 0);
							while (spi_tgt_done);  spi_tgt_done = true;
							spi_rx_transfer(pth, 1, &tmpw, 1, 0/*ctrl/sts*/); // high byte
							while (spi_tgt_done) ; spi_tgt_done = true;
							pr->data[0] = 0xff00 & (tmpw<<8);
							spi_rx_transfer(pth, 1, &tmpw, 1, 0/*ctrl/sts*/); // low byte
							while (spi_tgt_done) ;
							pr->data[0] |= (0xff & tmpw);
#ifdef TEST_FLASH
	wtmp = *(uint32_t*)(~0x3&(ul_page_addr_c+pt->addr));
	shf = ((ul_page_addr_c+pt->addr) & 0x3) * 8;
  *(uint8_t*)pr->data = 0xff & (wtmp >> shf);
#endif
							break;
				case WRITE_BY_ADDR:
							// setup spi to write addressed data
							pio_clear(PIOA, CPLD_6612_TRIG);
							*pth = 0xfe & (pt->addr<<1); // write access
							spi_tx_transfer(pth, 1, &tmpw, 1, 0/*ctrl/sts*/);
							while (spi_tgt_done) ; spi_tgt_done = true;
							*pth = (0x0ff & (pt->data[0]>>8)); // high byte
							spi_tx_transfer(pth, 1, &tmpw, 1, 0/*ctrl/sts*/);
							while (spi_tgt_done) ; spi_tgt_done = true;
							*pth = (0x0ff & pt->data[0]); // low byte
							spi_tx_transfer(pth, 1, &tmpw, 1, 0/*ctrl/sts*/);
				#ifdef CONFIG_ON_FLASH
					if (pt->toflash) {
						CHECKED_FLASH_WR(ul_page_addr_c+pt->addr, &pt->data[0], 1/*1 byte register*/)
					}
				#endif
							break;
				default: /* host msg in error */
							spi_tgt_done = false;
							break;
			}
			while (spi_tgt_done) ; delay_us(1);
			pio_set(PIOA, CPLD_6612_TRIG);
#endif //CONFIG_ADI_6612
  		}
#if 0
		sleepmgr_enter_sleep();
#endif
	}
}

void main_suspend_action(void)
{
	ui_powerdown();
}

void main_resume_action(void)
{
	ui_wakeup();
}

  static bool main_start_usb_write()
  {
		if (USB_DEVICE_START_RX_VAL != udd_g_ctrlreq.req.wValue)
			return false ;
		if (USB_DEVICE_START_RX_IDX != udd_g_ctrlreq.req.wIndex)
			return false ;
		if (USB_DEVICE_START_RX_LEN != udd_g_ctrlreq.req.wLength)
			return false ;
		return (usb_write_start = true);
  }
volatile bool main_usb_host_msg() // Attach this api to usb rx isr service, liyenho
{
	if (USB_HOST_MSG_TX_VAL != udd_g_ctrlreq.req.wValue &&
		 USB_CPLD_UPGRADE_VAL != udd_g_ctrlreq.req.wValue &&
		 USB_ATMEL_UPGRADE_VAL != udd_g_ctrlreq.req.wValue
#ifdef CONFIG_ADI_6612
		&& RF_RX_FREQ_VAL != udd_g_ctrlreq.req.wValue && RF_RX_FREQ_VAL_F != udd_g_ctrlreq.req.wValue
		&& RF_RX_VAL != udd_g_ctrlreq.req.wValue
#endif
#ifdef SMS_DVBT2_DOWNLOAD
		&& USB_SMS_FW_VAL != udd_g_ctrlreq.req.wValue
		&& USB_SMS_DATA_VAL != udd_g_ctrlreq.req.wValue
#endif
#ifdef SIANO_RETUNE
		&& USB_RX_TUNE_VAL != udd_g_ctrlreq.req.wValue
#endif
		)
		return false ;
	if (USB_HOST_MSG_IDX != udd_g_ctrlreq.req.wIndex)
		return false ;
	if (USB_HOST_MSG_LEN > udd_g_ctrlreq.req.wLength
#ifdef CONFIG_ADI_6612
		&& sizeof(short) != udd_g_ctrlreq.req.wLength
		&& sizeof(uint32_t) != udd_g_ctrlreq.req.wLength
#endif
#ifdef SMS_DVBT2_DOWNLOAD
		&& SMS_FW_HDR_LEN != udd_g_ctrlreq.req.wLength
#endif
		)
		return false ;
	udd_set_setup_payload( gs_uc_htbuffer, udd_g_ctrlreq.req.wLength);
	udd_g_ctrlreq.callback = host_usb_cb;
	return true;
}

volatile bool main_usb_host_reply()
{
	int cnf_echo = 0;
	if (USB_HOST_MSG_RX_VAL != udd_g_ctrlreq.req.wValue)
		return false ;
	if (USB_HOST_MSG_IDX != udd_g_ctrlreq.req.wIndex)
		return false ;
	// must have already echoed from original host comm
	if (spi_tgt_done) {
		udd_set_setup_payload( NULL, 0);
		return false; // let host retry
	}
		dev_access *ps = gs_uc_hrbuffer;
	if (USB_HOST_MSG_LEN-sizeof(ps->data[0]) == udd_g_ctrlreq.req.wLength)
		// (0==ps->dcnt) meant confirmation of write cmd, liyenho
		cnf_echo = 1;
		// must setup packet size
		if (!cnf_echo) {
			udd_set_setup_payload( gs_uc_hrbuffer,
				USB_HOST_MSG_LEN+(ps->dcnt-1)*sizeof(uint16_t));
		} else { //confirmation by echo, 1 data entry is included
			udd_set_setup_payload( gs_uc_hrbuffer,
				USB_HOST_MSG_LEN-sizeof(ps->data[0]));
		}
	/* no need to setup callback in READ case */
	return (true != spi_tgt_done);
}

 #if defined(MEDIA_ON_FLASH) && !defined(NO_USB)
bool main_usb_load_media() {
	if (USB_LOAD_MEDIA != udd_g_ctrlreq.req.wValue)
		return false ;
	if (USB_LOADM_IDX != udd_g_ctrlreq.req.wIndex)
		return false ;
	if (USB_LOADM_LEN > udd_g_ctrlreq.req.wLength)
		return false ;
	// must setup max packet size! liyenho
	udd_set_setup_payload( gs_uc_tbuffer, USB_LOADM_LEN);
	udd_g_ctrlreq.callback = host_usb_mda_flash_cb;
 	return true;
}
 #endif

volatile bool main_vender_specific() {
 #ifdef CONFIG_ON_FLASH
	if (USB_BOOT_APP_VAL == udd_g_ctrlreq.req.wValue) {
		udd_set_setup_payload( &bootapp, sizeof(bootapp));
		udd_g_ctrlreq.callback = host_usb_cb;
	}
 #endif
	if (USB_SYSTEM_RESTART_VAL == udd_g_ctrlreq.req.wValue) {
		main_loop_restart(); return true;
	}
	if (USB_HOST_MSG_TX_VAL == udd_g_ctrlreq.req.wValue ||
		USB_CPLD_UPGRADE_VAL == udd_g_ctrlreq.req.wValue ||
		USB_ATMEL_UPGRADE_VAL == udd_g_ctrlreq.req.wValue
#ifdef CONFIG_ADI_6612
		|| RF_RX_FREQ_VAL == udd_g_ctrlreq.req.wValue || RF_RX_FREQ_VAL_F == udd_g_ctrlreq.req.wValue
		|| RF_RX_VAL == udd_g_ctrlreq.req.wValue
#endif
#ifdef SMS_DVBT2_DOWNLOAD
		|| USB_SMS_FW_VAL == udd_g_ctrlreq.req.wValue
		|| USB_SMS_DATA_VAL == udd_g_ctrlreq.req.wValue
#endif
#ifdef SIANO_RETUNE
		|| USB_RX_TUNE_VAL == udd_g_ctrlreq.req.wValue
#endif
		)
		return main_usb_host_msg();
 #if defined(MEDIA_ON_FLASH) && !defined(NO_USB)
	if (USB_LOAD_MEDIA == udd_g_ctrlreq.req.wValue)
		return main_usb_load_media();
 #endif
	else if (USB_HOST_MSG_RX_VAL == udd_g_ctrlreq.req.wValue)
		return main_usb_host_reply();
 #ifdef CONF_BOARD_USB_TX
	else if (USB_DEVICE_START_RX_VAL == udd_g_ctrlreq.req.wValue)
		return main_start_usb_write();
	else if (USB_DEVICE_BYPASS_USB_VAL == udd_g_ctrlreq.req.wValue)
		usb_data_done = false; // set usb done flag on behalf of spi, to facilitate lgdst acs
 #endif
	 else if (USB_STREAM_OFF_VAL == udd_g_ctrlreq.req.wValue) {
		if (USB_STREAM_IDX != udd_g_ctrlreq.req.wIndex)
			return (bool)-1 ;
		else stream_flag = false;
	 }
	 else if (USB_STREAM_ON_VAL == udd_g_ctrlreq.req.wValue) {
		 if (USB_STREAM_IDX == udd_g_ctrlreq.req.wIndex)
		 	stream_flag = true;
		 if (USB_QUERY_IDX == udd_g_ctrlreq.req.wIndex)
		 	udd_set_setup_payload( &main_loop_on, sizeof(main_loop_on));
		 return (bool)-1 ;
	 }
#ifdef  RADIO_SI4463
	else if (RADIO_COMM_VAL == udd_g_ctrlreq.req.wValue) {
		// it should be safe to use wIndex alternatively instead pointer to interface index
		if (RADIO_STARTUP_IDX == udd_g_ctrlreq.req.wIndex) {
			// re-initialized by host dynamically
			udd_set_setup_payload(pRadioConfiguration, sizeof(tRadioConfiguration));
			pio_clear(PIOB, PIO_PB9); // disable TS gate
#ifdef RX_SPI_CHAINING
			spidma_active = FALSE;
#else //!RX_SPI_CHAINING
			sms4470_usb_ctx.spi_buf_rd -= I2SC_BUFFER_SIZE; // equivalent to drop a block to align
			if (sms4470_usb_ctx.buf_start>sms4470_usb_ctx.spi_buf_rd)
				sms4470_usb_ctx.spi_buf_rd = sms4470_usb_ctx.buf_end;
#endif
			udd_g_ctrlreq.callback = si4463_radio_cb; // radio callback
		}
		if (RADIO_CAL_IDX == udd_g_ctrlreq.req.wIndex) {
			memset(&si4463_factory_tune, 0x0, sizeof(si4463_factory_tune));
			ctrl_tdma_enable = false; // turn off the si4463 tdd flag first
			si4463_factory_tune.calib_req_h = true; // turn on factory cap tuning request
		}
		if (RADIO_CAL_DONE_IDX == udd_g_ctrlreq.req.wIndex) {
			static uint16_t tune_done[2] ; /*sts flag and final cbv*/
			*(tune_done) = 0 != si4463_factory_tune.median;
			*(tune_done+1) = si4463_factory_tune.median;
			udd_set_setup_payload(&tune_done, sizeof(tune_done));
		}
		if (RADIO_HOPLESS_IDX == udd_g_ctrlreq.req.wIndex) {
			udd_set_setup_payload( (uint8_t *)gs_uc_hrbuffer, udd_g_ctrlreq.req.wLength);
			udd_g_ctrlreq.callback = si4463_radio_cb; // radio callback
		}
		else if (RADIO_DATA_TX_IDX == udd_g_ctrlreq.req.wIndex) {
		   unsigned int wrptr_tmp1 = wrptr_rdo_tpacket;
		   unsigned int wrptr_tmp2;
		   unsigned int ovflag1,ovflag2;
		   ovflag1 = wrptr_inc(&wrptr_tmp1,&rdptr_rdo_tpacket,RDO_TPACKET_FIFO_SIZE,1);
		   wrptr_tmp2=wrptr_tmp1;
		   ovflag2 = wrptr_inc(&wrptr_tmp2,&rdptr_rdo_tpacket,RDO_TPACKET_FIFO_SIZE,1);
		   if(ovflag1 || ovflag2){
			   //overflow condition
			   gs_rdo_tpacket_ovflw++;
		   }
		   else{  // hardcoded again;-( I had to accommodate with such fixed packet length assumption vs user packet length, liyenho
			   udd_set_setup_payload( tpacket_grp, RADIO_GRPPKT_LEN); //stores payload in holding buffer
		   }
		   udd_g_ctrlreq.callback = si4463_radio_cb; // radio callback
		}
		else if (RADIO_DATA_RX_IDX == udd_g_ctrlreq.req.wIndex) {
		  // byte[0]: no-packet: 0xee,  idle-packet: 0xe5, partial-packet:0xe2 tail-payload-only:0x73
		  // byte[1]: no-packet: 0xee,  idle_packet: 0xe5, partial-packet:0xe2 tail-payload-only:0x73
		  // byte[2]: no-packet: 0xee,  idle-packet: sender side send pkt count
		  // byte[3]: no-packet: 0xee,  idle-packet: sender side recv pkt count
		  // byte[4]: idle-packet: 0xe5
		  // byte[5]: idle-packet: 0xe5
		  // byte[lastPayload]:
		  // byte[U0]: user status field
		  //          bit0: usr payload valid flag
		  //          bit1: rf link lock
		  // byte[U1]: {rxlvl4bits, txlvl4bits}
		  // byte[U2]: Ctrl-rcv jitter delta (0.1ms)
		  // byte[U3]: not defined
		  unsigned char tdma_lock_char; //covert bool to uchar
		  uint8_t fifolvlr, fifolvlt, payldvalid_flag,filler_flag=0, usrvalid_flag;

      if(ctrl_tdma_lock) tdma_lock_char=2; else tdma_lock_char=0;
      fifolvlr=((uint8_t)fifolvlcalc(wrptr_rdo_rpacket,rdptr_rdo_rpacket, RDO_RPACKET_FIFO_SIZE));
		  fifolvlt=((uint8_t)fifolvlcalc(wrptr_rdo_tpacket,rdptr_rdo_tpacket, RDO_TPACKET_FIFO_SIZE));
		  if(rdptr_rdo_rpacket == wrptr_rdo_rpacket){
			  //identify no data to send
			  usrvalid_flag=0;
			  memset(rpacket_grp, 0xee, RADIO_GRPPKT_LEN);
		  }
		  else
      { //process next fifo element
        unsigned int rdptr_race; //tmp to prevent race condition
        static bool gotpayloadhead=false;
        rdptr_race = rdptr_rdo_rpacket;
        rdptr_inc(&wrptr_rdo_rpacket, &rdptr_race, RDO_RPACKET_FIFO_SIZE,1);
		    gp_rdo_rpacket = gs_rdo_rpacket + (RDO_ELEMENT_SIZE*rdptr_race);
		    payldvalid_flag=(((uint8_t *)gp_rdo_rpacket)[RADIO_PKT_LEN-1]>>7)&0x01;
		    if( (((uint8_t *)gp_rdo_rpacket)[0] == 0xe5)&&
		      (((uint8_t *)gp_rdo_rpacket)[1] == 0xe5)&&
		      (((uint8_t *)gp_rdo_rpacket)[4] == 0xe5))
		      filler_flag=1;
		    if(ctrl_tdma_lock) tdma_lock_char=2; else tdma_lock_char=0;

        if((payldvalid_flag==0)&&(filler_flag==1))
		    {
			    //got filler idle rf packet
			    usrvalid_flag=0;
			    memcpy(rpacket_grp, gp_rdo_rpacket, RADIO_GRPPKT_LEN/2);
			    for(int i=0;i<RADIO_GRPPKT_LEN/2;i++)
			    ((uint8_t *)rpacket_grp)[RADIO_GRPPKT_LEN/2+i]=0xe5; //force fill 2nd half
		    }
		    else if(payldvalid_flag==1)
		    {
			    //got header payload
			    memset(rpacket_grp, 0xe2, RADIO_GRPPKT_LEN);
			    usrvalid_flag = 0;
			    memcpy(rpacket_grp_partial, gp_rdo_rpacket, RADIO_GRPPKT_LEN/2);
          gotpayloadhead=true;
		    }
		    else
		    {
			    //got tail end payload
          if(gotpayloadhead==false){
            memset(rpacket_grp,0x73,RADIO_GRPPKT_LEN/2);
            usrvalid_flag=0;}
          else {
            memcpy(rpacket_grp,rpacket_grp_partial,RADIO_GRPPKT_LEN/2);
            usrvalid_flag=1;}
          gotpayloadhead=false;

			    memcpy(rpacket_grp+RADIO_GRPPKT_LEN/2,gp_rdo_rpacket,RADIO_GRPPKT_LEN/2);
		    }
        rdptr_rdo_rpacket = rdptr_race;
      }
		  ((uint8_t *)rpacket_grp)[RADIO_GRPPKT_LEN]= usrvalid_flag | tdma_lock_char;
		  ((uint8_t *)rpacket_grp)[RADIO_GRPPKT_LEN+1]= (fifolvlr<<4) + fifolvlt;
		  ((uint8_t *)rpacket_grp)[RADIO_GRPPKT_LEN+2]= radio_mon_rcvjitter;
		  ((uint8_t *)rpacket_grp)[RADIO_GRPPKT_LEN+3]= ((uint8_t *)gp_rdo_rpacket)[RADIO_PKT_LEN-1];
		  udd_set_setup_payload( (uint8_t *)rpacket_grp, RADIO_GRPPKT_LEN+RADIO_INFO_LEN);
		}
		else if (RADIO_STATS_IDX == udd_g_ctrlreq.req.wIndex) {
			udd_set_setup_payload( (uint8_t *)&r4463_sts, RADIO_STATS_LEN);
		}
		else if (RADIO_MODEM_IDX == udd_g_ctrlreq.req.wIndex) {
			si446x_get_modem_status( 0xff );  // do not clear up pending status
			memcpy(gs_uc_hrbuffer, &Si446xCmd.GET_MODEM_STATUS, RADIO_MODEM_LEN);
			si446x_get_property(0x20, 1, 0x4e); // read current RSSI comp bias
			*(uint8_t*)gs_uc_hrbuffer = Si446xCmd.GET_PROPERTY.DATA[0];
			// be sure to overwrite the 1st entry in modem states buffer to carry RSSI comp bias value
			udd_set_setup_payload( (uint8_t *)gs_uc_hrbuffer, RADIO_MODEM_LEN);
		}
		else if (RADIO_CHSEL_IDX == udd_g_ctrlreq.req.wIndex) {
			if (RADIO_CHSEL_LEN0 != udd_g_ctrlreq.req.wLength &&
				RADIO_CHSEL_LEN1 != udd_g_ctrlreq.req.wLength
	#if false // channel filter coefficients were removed, see radio.c for details
				&& RADIO_CHSEL_LEN2 != udd_g_ctrlreq.req.wLength
				&& RADIO_CHSEL_LEN3 != udd_g_ctrlreq.req.wLength
				&& RADIO_CHSEL_LEN4 != udd_g_ctrlreq.req.wLength
	#endif
			)
				return (bool) -1; // error in msg len
			udd_set_setup_payload( (uint8_t *)gs_uc_hrbuffer, udd_g_ctrlreq.req.wLength);
			udd_g_ctrlreq.callback = si4463_radio_cb; // radio callback
		}
		else if (RADIO_TEMP_IDX == udd_g_ctrlreq.req.wIndex) {
		    extern volatile uint16_t temp1_intm;	// from si446x_nirq.c
		    *(int16_t*)gs_uc_hrbuffer = temp1_intm;
		    udd_set_setup_payload( (uint8_t *)gs_uc_hrbuffer, RADIO_TEMP_LEN);
		}
		else if (RADIO_CTUNE_IDX == udd_g_ctrlreq.req.wIndex) {
			udd_set_setup_payload( gs_uc_hrbuffer, RADIO_CTUNE_LEN);
			udd_g_ctrlreq.callback = si4463_radio_cb; // radio callback
		}
		else if (RADIO_PAIRID_IDX == udd_g_ctrlreq.req.wIndex) {
			if (HOP_ID_LEN != udd_g_ctrlreq.req.wLength)
				return -1;
			udd_set_setup_payload( gs_uc_hrbuffer, HOP_ID_LEN);
			udd_g_ctrlreq.callback = si4463_radio_cb; // radio callback
		}
		else if (RADIO_PAIR_LOCKED_IDX == udd_g_ctrlreq.req.wIndex) {
			*(uint32_t*)gs_uc_hrbuffer = ctrl_tdma_lock?1:0;
			udd_set_setup_payload( gs_uc_hrbuffer, RADIO_PAIR_LOCKED_LEN);
		}
		else if (BASE_GPS_IDX == udd_g_ctrlreq.req.wIndex){
			//update base GPS location
			udd_set_setup_payload((float *)gs_uc_hrbuffer, BASE_GPS_LEN);
			udd_g_ctrlreq.callback = si4463_radio_cb; // radio callback
		}
	}
#endif
	else if (USB_ATMEL_VER_VAL == udd_g_ctrlreq.req.wValue) {
		char month[3+1], day[2+1], year[4+1];
		{
			month[0] = __DATE__[0];
			 month[1] = __DATE__[1];
			  month[2] = __DATE__[2];
			   month[3] = 0x0;
			day[0] = __DATE__[4];
			 day[1] = __DATE__[5];
			  day[2] = 0x0;
			year[0] = __DATE__[7];
			 year[1] = __DATE__[8];
			  year[2] = __DATE__[9];
			   year[3] = __DATE__[10];
			    year[4] = 0x0;
		}
		udd_set_setup_payload( __version_atmel__, sizeof(__version_atmel__));
		if (!strcmp(month,"Jan") )
			 __version_atmel__[0] = 1;
		else if (!strcmp(month,"Feb") )
			__version_atmel__[0] = 2;
		else if (!strcmp(month,"Mar") )
			__version_atmel__[0] = 3;
		else if (!strcmp(month,"Apr") )
			__version_atmel__[0] = 4;
		else if (!strcmp(month,"May") )
			__version_atmel__[0] = 5;
		else if (!strcmp(month,"Jun") )
			__version_atmel__[0] = 6;
		else if (!strcmp(month,"Jul") )
			__version_atmel__[0] = 7;
		else if (!strcmp(month,"Aug") )
			__version_atmel__[0] = 8;
		else if (!strcmp(month,"Sep") )
			__version_atmel__[0] = 9;
		else if (!strcmp(month,"Oct") )
			__version_atmel__[0] = 10;
		else if (!strcmp(month,"Nov") )
			__version_atmel__[0] = 11;
		else if (!strcmp(month,"Dec") )
			__version_atmel__[0] = 12;
		__version_atmel__[1] = (uint8_t)atoi(day);
		__version_atmel__[2] = (uint8_t)(atoi(year)-2000);
	}
 	return (bool) -1; /* error in msg */
}

void main_sof_action(void)
{
	if (!main_b_cdc_enable)
		return;
	ui_process(udd_get_frame_number());
}

#ifdef USB_DEVICE_LPM_SUPPORT
void main_suspend_lpm_action(void)
{
	ui_powerdown();
}

void main_remotewakeup_lpm_disable(void)
{
	ui_wakeup_disable();
}

void main_remotewakeup_lpm_enable(void)
{
	ui_wakeup_enable();
}
#endif

bool main_cdc_enable(uint8_t port)
{
	main_b_cdc_enable = true;
	// Open communication
	//uart_open(port);
	return true;
}

void main_cdc_disable(uint8_t port)
{
	main_b_cdc_enable = false;
	// Close communication
	//uart_close(port);
}

void main_cdc_set_dtr(uint8_t port, bool b_enable)
{
	if (b_enable) {
		// Host terminal has open COM
		ui_com_open(port);
	}else{
		// Host terminal has close COM
		ui_com_close(port);
	}
}

void main_loop_restart() {
	if (main_loop_on) {
		system_main_restart = true ;  // setup restart flag
		main_loop_on = false;
		spidma_active = false; // enable reset TS gate
	}
}

static int timedelta(bool reset, unsigned int bignum, unsigned int smallnum)
{ // what the hack on the old codes doing??? liyenho
	static int64_t bprev, sprev;
	int64_t bl, sl, delta;
	if (reset) {
		bprev = sprev = 0L; // chances to collide with 0 are less than twice being by lightning in a day
	}
#define EXTEND64(o, i, i0) o = (i < i0) ? 0x100000000+i : i;
	EXTEND64(bl, bignum, bprev)
	EXTEND64(sl, smallnum, sprev)
	delta = bl - sl;	// delta shall be of 32 bit range, liyenho
#undef EXTEND64
	bprev = bignum;
	sprev = smallnum;
	return (int)delta ;
}

uint32_t fifolvlcalc(uint32_t wrptr, uint32_t rdptr, uint32_t fifodepth)
{
	if(wrptr==rdptr)
	return(0);
	if(wrptr> rdptr )
	return(wrptr - rdptr);
	return(wrptr + (fifodepth-rdptr));
}

uint32_t wrptr_inc(uint32_t *wrptr,  uint32_t *rdptr, uint32_t fifodepth, int step)
{
	uint32_t wrptr_tmp;  //get local snapshot due to race concerns
	uint32_t rdptr_tmp;  //get local snapshot due to race concerns
	int i;
	wrptr_tmp = *wrptr;
	rdptr_tmp = *rdptr;
	for(i=0;i<step;i++) {
		if(wrptr_tmp >= (fifodepth-1))  wrptr_tmp=0;
		else                        wrptr_tmp++;
		if(wrptr_tmp== rdptr_tmp){
			//overflow condition
			return 1; //overflow
		}
	}

	*wrptr = wrptr_tmp;
	return 0;  //no overflow
}

uint32_t rdptr_inc(uint32_t *wrptr,  uint32_t *rdptr, uint32_t fifodepth, int step)
{
	uint32_t rdptr_tmp, wrptr_tmp;
	int i;
	rdptr_tmp = *rdptr;  //get local snapshot due to race concerns
	wrptr_tmp = *wrptr;  //get local snapshot due to race concerns
	for(i=0;i<step;i++) {
		if(wrptr_tmp== rdptr_tmp){
			//overflow condition
			return 1; //underflow
		}
		if(rdptr_tmp >= (fifodepth-1))  rdptr_tmp=0;
		else                         rdptr_tmp++;
	}
	*rdptr = rdptr_tmp;
	return 0;  //good data
}

/*
void assert_triggered(const char *file, uint32_t line)
{
	volatile uint32_t block_var = 0, line_in;
	const char *file_in;

	// These assignments are made to prevent the compiler optimizing the values away.
	file_in = file;
	line_in = line;
	(void) file_in;
	(void) line_in;

	taskENTER_CRITICAL();
	{
		while (block_var == 0) {
			// Set block_var to a non-zero value in the debugger to step out of this function.
		}
	}
	taskEXIT_CRITICAL();
}
*/
/**
 * \mainpage ASF USB Device CDC
 *
 * \section intro Introduction
 * This example shows how to implement a USB Device CDC
 * on Atmel MCU with USB module.
 * The application note AVR4907 provides more information
 * about this implementation.
 *
 * \section desc Description of the Communication Device Class (CDC)
 * The Communication Device Class (CDC) is a general-purpose way to enable all
 * types of communications on the Universal Serial Bus (USB).
 * This class makes it possible to connect communication devices such as
 * digital telephones or analog modems, as well as networking devices
 * like ADSL or Cable modems.
 * While a CDC device enables the implementation of quite complex devices,
 * it can also be used as a very simple method for communication on the USB.
 * For example, a CDC device can appear as a virtual COM port, which greatly
 * simplifies application development on the host side.
 *
 * \section startup Startup
 * The example is a bridge between a USART from the main MCU
 * and the USB CDC interface.
 *
 * In this example, we will use a PC as a USB host:
 * it connects to the USB and to the USART board connector.
 * - Connect the USART peripheral to the USART interface of the board.
 * - Connect the application to a USB host (e.g. a PC)
 *   with a mini-B (embedded side) to A (PC host side) cable.
 * The application will behave as a virtual COM (see Windows Device Manager).
 * - Open a HyperTerminal on both COM ports (RS232 and Virtual COM)
 * - Select the same configuration for both COM ports up to 115200 baud.
 * - Type a character in one HyperTerminal and it will echo in the other.
 *
 * \note
 * On the first connection of the board on the PC,
 * the operating system will detect a new peripheral:
 * - This will open a new hardware installation window.
 * - Choose "No, not this time" to connect to Windows Update for this installation
 * - click "Next"
 * - When requested by Windows for a driver INF file, select the
 *   atmel_devices_cdc.inf file in the directory indicated in the Atmel Studio
 *   "Solution Explorer" window.
 * - click "Next"
 *
 * \copydoc UI
 *
 * \section example About example
 *
 * The example uses the following module groups:
 * - Basic modules:
 *   Startup, board, clock, interrupt, power management
 * - USB Device stack and CDC modules:
 *   <br>services/usb/
 *   <br>services/usb/udc/
 *   <br>services/usb/class/cdc/
 * - Specific implementation:
 *    - main.c,
 *      <br>initializes clock
 *      <br>initializes interrupt
 *      <br>manages UI
 *      <br>
 *    - uart_xmega.c,
 *      <br>implementation of RS232 bridge for XMEGA parts
 *    - uart_uc3.c,
 *      <br>implementation of RS232 bridge for UC3 parts
 *    - uart_sam.c,
 *      <br>implementation of RS232 bridge for SAM parts
 *    - specific implementation for each target "./examples/product_board/":
 *       - conf_foo.h   configuration of each module
 *       - ui.c        implement of user's interface (leds,buttons...)
 */
