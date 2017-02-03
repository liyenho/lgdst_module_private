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

  #define SPI0_Handler     FLEXCOM0_Handler
  #define SPI_Handler     FLEXCOM5_Handler
  #define SPI0_IRQn        FLEXCOM0_IRQn
  #define SPI_IRQn        FLEXCOM5_IRQn

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
static volatile bool main_b_cdc_enable = false;
  /** TWI Bus Clock 400kHz */
  // slow down to see if it improve data integrity, it does, liyenho
  #define TWI_CLK     /*200000*/ 100000
  /** The address for TWI IT913X */
  #define IT913X_ADDRESS        0x3A

  /* still need to confirm on these..., done */
  #define ITE_REG_ADDR         0
  #define ITE_REG_ADDR_LENGTH  /*2*/ 0
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

/* UART baudrate. */
#define UART_BAUDRATE      115200
/* SPI clock setting (Hz). */
  static uint32_t gs_ul_spi_clock[2+1] = {  // shall be higher than usb data rate
  				  	2000000, 8000000, RADIO_SPI_BR}; // (ctrl/sts, video, radio) spi bit rate

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

volatile bool stream_flag = true; // default to TS stream automatic on
#ifdef CONFIG_ON_FLASH
	uint32_t ul_page_addr_ctune =IFLASH_ADDR + IFLASH_SIZE - NUM_OF_ATMEL_REGS, // 1st atmel reg on flash
						// temperature @ current tuning @ 2nd atmel reg on flash
					ul_page_addr_mtemp =IFLASH_ADDR + IFLASH_SIZE - NUM_OF_ATMEL_REGS + 1;
#endif
extern udd_ctrl_request_t udd_g_ctrlreq; // from udp_device.c, liyenho
volatile uint32_t g_ul_10ms_ticks=0, g_ul_wait_10ms=0;
/*static*/ bool systick_enabled = false;  // default to disable sys timer
volatile static uint32_t g_ul_led_ticks=0, g_ul_wait_100ms=10, g_ul_wait_1s=100;
static bool health_led_onoff = false;
volatile bool usb_write_start = false ; // called back from udc.c, liyenho
#ifdef RECV_IT913X
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
volatile uint32_t upgrade_fw_hdr[FW_UPGRADE_HDR_LEN/sizeof(int)]={-1} ;
#if defined(FWM_DNLD_DBG)
  volatile bool usb_host_active = false;
  extern volatile bool usb_tgt_active ;
#endif
 // host to fpga/2072 ctrl/sts buffer
static uint32_t gs_uc_htbuffer[(USB_HOST_MSG_LEN+HOST_BUFFER_SIZE+3)/sizeof(int)];
static uint32_t gs_uc_hrbuffer[(USB_HOST_MSG_LEN+HOST_BUFFER_SIZE+3)/sizeof(int)];
  volatile bool i2c_read_done = false;
  /*static*/ twi_packet_t packet_tx;
#ifdef RX_SPI_CHAINING
  twi_packet_t packet_rx;
#else //!RX_SPI_CHAINING
  twi_packet_cb_t packet_rx; // with cb ext, liyenho
#endif
   // host/sms ctrl/sts buffer
   volatile uint32_t it913x_fw_hdr[1]={-1} ; //byte lenght of firmware
   /*static*/ uint32_t gs_ite_tbuffer[(ITE_BUFFER_SIZE+3)/sizeof(int)];
   /*static*/ uint32_t gs_ite_rbuffer[(ITE_BUFFER_SIZE+3)/sizeof(int)];
 #if true  // add two big memory chunks for ite fw download
   /*static*/ uint32_t fw_ite_tbuffer[FW_DNLD_SIZE/sizeof(int)];
   /*static*/ uint32_t fw_ite_rbuffer[FW_DNLD_SIZE/sizeof(int)];
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
#ifdef RECV_IT913X
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
void SPI0_Handler(void) // 2072 ctrl spi
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
		if ((id == ID_PIOA) && (index == ITE_HOST_INT)){
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
	packet_tx.chip        = IT913X_ADDRESS;
	packet_tx.addr[0]     = ITE_REG_ADDR >> 8;
	packet_tx.addr[1]     = ITE_REG_ADDR;
	packet_tx.addr_length = ITE_REG_ADDR_LENGTH;
	packet_tx.buffer      = (uint8_t *) gs_ite_tbuffer;
	packet_tx.length      = 255 ; // max user data length

	/* Configure the data packet to be received */
	packet_rx.chip        = packet_tx.chip;
	packet_rx.addr[0]     = packet_tx.addr[0];
	packet_rx.addr[1]     = packet_tx.addr[1];
	packet_rx.addr_length = packet_tx.addr_length;
	packet_rx.buffer      = (uint8_t *) fw_ite_rbuffer;
	packet_rx.length      = sizeof(fw_ite_rbuffer);
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
#if true //ndef RECV_IT913X // try it out see if we can avoid byte drop, liyenho
			SPI_CSR_BITS_16_BIT);
#else //RECV_IT913X
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
	 spi_set_clock_polarity(base, SPI_CHIP_SEL, 0/*clk idle state is low*/);
 	spi_set_clock_phase(base, SPI_CHIP_SEL, 1/*captured @ rising, transit @ falling*/);
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
		spi_set_transfer_delay(base, SPI_CHIP_SEL, 0x10/*delay between bytes*/,
			0x10/*delay between spi xfer*/);
	spi_enable(base);
#ifdef TEST_SPI
	spi_enable_loopback(base);
#endif
	pdc_disable_transfer(g_p_spim_pdc [ch], PERIPH_PTCR_RXTDIS |
			PERIPH_PTCR_TXTDIS);
}

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
#if defined(RECV_IT913X) && !defined(RX_SPI_CHAINING)
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
/*! \brief host ctrl/sts callback for sys upgrade interface
 */
  static void host_usb_cb() {
		if (Is_udd_in_sent(0) ||
			udd_g_ctrlreq.payload_size >/*!=*/ udd_g_ctrlreq.req.wLength)
			return; // invalid call
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
		else if ((USB_ITE_FW_VAL == udd_g_ctrlreq.req.wValue) &&
						ITE_FW_HDR_LEN >=udd_g_ctrlreq.payload_size) {
			memcpy(it913x_fw_hdr, // set ite913x fw hdr, liyenho
								udd_g_ctrlreq.payload,
								ITE_FW_HDR_LEN);
			return;
		}
		dev_access *ps = gs_uc_htbuffer,*ps1 = gs_uc_hrbuffer;
		// prepare confirm msg by echo whatever received
		memcpy(gs_uc_hrbuffer, udd_g_ctrlreq.payload, USB_HOST_MSG_LEN-sizeof(ps->data[0]));

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
		// the content was transfered to gp_rdo_tpacket by main_vender_specific();
		// The radio send is driven by radio receive, so no actions needed here
		unsigned int wrptr_tmp1 = wrptr_rdo_tpacket;
		unsigned int wrptr_tmp2;
		unsigned int ovflag1,ovflag2;
		ovflag1 = wrptr_inc(&wrptr_tmp1,&rdptr_rdo_tpacket,RDO_TPACKET_FIFO_SIZE,1);
		wrptr_tmp2=wrptr_tmp1;
		ovflag2 = wrptr_inc(&wrptr_tmp2,&rdptr_rdo_tpacket,RDO_TPACKET_FIFO_SIZE,1);
		if(ovflag1 || ovflag2){
		;}
		else{
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
uint32_t Cmd_addChecksum (
    uint32_t*          bufferLength,
    uint8_t*           buffer
) {
    uint32_t error  = 0;
    uint32_t loop   = (*bufferLength - 1) / 2;
    uint32_t remain = (*bufferLength - 1) % 2;
    uint32_t i;
    uint16_t  checksum = 0;

    for (i = 0; i < loop; i++)
        checksum = checksum + (uint16_t) (buffer[2 * i + 1] << 8) + (uint16_t) (buffer[2 * i + 2]);
    if (remain)
        checksum = checksum + (uint16_t) (buffer[*bufferLength - 1] << 8);

    checksum = ~checksum;
    buffer[*bufferLength]     = (uint8_t) ((checksum & 0xFF00) >> 8);
    buffer[*bufferLength + 1] = (uint8_t) (checksum & 0x00FF);
    buffer[0]                 = (uint8_t) (*bufferLength + 1);  // because buffer[0] indicates count which does NOT include itself, liyenho
    *bufferLength            += 2;

    return (error);
}

 static void download_ite_fw(uint32_t fw_dnld_size, uint8_t address, uint8_t *chk )
 {
	uint32_t fw_len0, fw_len = it913x_fw_hdr[0],
						len, len1, len2, ll, ll1, crc, i,
						fw_dnld_size1;
	while (0 > (int)fw_len) {
		fw_len = it913x_fw_hdr[0];
	}
	uint8_t *fw_ite_tbuffer1 = (uint8_t*)fw_ite_tbuffer+6/*per ITE algorithm*/,
					 *pb = fw_ite_rbuffer,
					 *pe = pb+fw_dnld_size,
					 Cmd_sequence = 0/*it should be alright...*/;
	uint16_t command = Cmd_buildCommand ( \
															Command_SCATTER_WRITE, \
															Processor_LINK, \
															0);
#ifdef FWM_DNLD_DBG
	uint32_t fw_dbg_buffer[(FW_DNLD_SIZE)/sizeof(int)];
	volatile bool erase = true;
	uint32_t ul_page_addr=PAGE_ADDRESS ;
	volatile uint32_t lm, le, ul_rc, er_adr =ul_page_addr-SECTOR_RES, rem = fw_len;
	const uint32_t erlen = SECTOR_SIZE_L; // last two sector size must be 128 kbytes
	lm = SECTOR_RES+fw_dnld_size;
	le = erlen;
#endif
	usb_read_buf1(fw_ite_rbuffer, fw_dnld_size);
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
	len = fw_dnld_size - ((uint32_t)pb - (uint32_t)fw_ite_rbuffer);
	len2 = (len1=fw_dnld_size) - (ll=len);
	memcpy(fw_ite_tbuffer1, pb, len); // has to use the other buffer
	delay_ms(100); // match delay used @ host side
	pb = (uint8_t*)fw_ite_tbuffer1 + len;
	 pe = (uint8_t*)fw_ite_rbuffer + len2;
	fw_len0 = fw_len - len + len2;
	while(1) {
		len = (fw_dnld_size<=fw_len0)?fw_dnld_size:fw_len0;
		if (0 != fw_len0)
			usb_read_buf1(fw_ite_rbuffer, len);
		fw_len0 -= len;
		if (fw_dnld_size != len) {
	#include <assert.h>
			if (0 != len) {
				if (len2>len) {
					assert(len1+len2 == len);
					memcpy(pb, fw_ite_rbuffer, len);
				}
				else {
					memcpy(pb, fw_ite_rbuffer, len2);
					ll1 = len - len2;
				}
		   }
			else  {
				assert(len1+len2 == ll1);
				memcpy(fw_ite_tbuffer1, pe, ll1);
		   }
	  	}
		else
	  		memcpy(pb, fw_ite_rbuffer, len2);

        fw_ite_tbuffer[1] = (uint8_t) (command >> 8);
        fw_ite_tbuffer[2] = (uint8_t) command;
        fw_ite_tbuffer[3] = (uint8_t) Cmd_sequence++;
        memcpy(fw_ite_tbuffer+4, fw_ite_tbuffer1, fw_dnld_size);
        fw_dnld_size1 = len1 + 4;
		Cmd_addChecksum (&fw_dnld_size1, fw_ite_tbuffer);

		TWI_WRITE(address, fw_ite_tbuffer, fw_dnld_size1)
#ifdef FWM_DNLD_DBG
	uint8_t *pdb = (uint8_t*)fw_dbg_buffer;
		memcpy(pdb, fw_ite_tbuffer, fw_dnld_size1);
		if (erase) {
			ul_rc = flash_erase_sector(er_adr);
			if (ul_rc != FLASH_RC_OK) {
				//printf("- Pages erase error %lu\n\r", (UL)ul_rc);
				return; // error when erase pages
			}
			erase = false;
		}
		ul_rc = flash_write(ul_page_addr, fw_dbg_buffer, fw_dnld_size1, 0);
		if (ul_rc != FLASH_RC_OK) {
			//printf("- Pages write error %lu\n\r", (UL)ul_rc);
			return; // error when write pages
		}
		ul_page_addr += (fw_dnld_size1);
		// determine whether erasure is necessary or not
		lm += (fw_dnld_size1);
		if (le < lm) {
			erase = true;
			le += erlen;
			er_adr += erlen;
		}
#endif
		delay_ms(30);
		if (!(fw_len -= len1)) break;
		len1 = (fw_dnld_size<=fw_len)?fw_dnld_size:fw_len;
		if (fw_dnld_size == len) {
			memcpy(fw_ite_tbuffer1, pe, ll);
		}
	}
 #ifdef FWM_DNLD_DBG
  ul_page_addr=PAGE_ADDRESS ;
  i = 0;
  	do {
  		if (fw_dnld_size<=rem) {
  			len = fw_dnld_size;
  			rem -= fw_dnld_size;
		}
  		else {
  			len = rem;
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
/* Jump to CM vector table */
#if defined   (__CC_ARM)     /* Keil µVision 4 */
static __asm__ void jump_to_app(void *code_addr)
{
	mov r1, r0
	ldr r0, [r1, # 4]
	ldr sp, [r1]
	blx r0
}

#elif defined (__ICCARM__)   /* IAR Ewarm 5.41+ */
static void jump_to_app(void *code_addr)
{
	UNUSED(code_addr);
	__asm(
			"mov     r1, r0        \n"
			"ldr     r0, [r1, #4]  \n"
			"ldr     sp, [r1]      \n"
			"blx     r0"
			);
}

#elif defined (__GNUC__)     /* GCC CS3 2009q3-68 */
static void jump_to_app(void *code_addr)
{
	__asm__(
			"mov   r1, r0        \n"
			"ldr   r0, [r1, #4]  \n"
			"ldr   sp, [r1]      \n"
			"blx   r0"
			);
}

#else /* General C, no stack reset */
static void jump_to_app(void *code_addr)
{
	void (*pFct)(void) = NULL;
	/* Point on __main address located in the second word in vector table */
	pFct = (void (*)(void))(*(uint32_t *)((uint32_t)code_addr + 4));
	pFct();
}
#endif
/**
 * Execute the application binary
 *
 * \param addr Application start address.
 */
static void _app_exec(void *addr)
{
	uint32_t i;
	__disable_irq();
	/* Disable SysTick */
	SysTick->CTRL = 0;
	/* Disable IRQs & clear pending IRQs */
	for (i = 0; i < 8; i++) {
		NVIC->ICER[i] = 0xFFFFFFFF;
		NVIC->ICPR[i] = 0xFFFFFFFF;
	}

	/* Switch clock to slow RC */
	osc_enable(OSC_SLCK_32K_RC);
	osc_wait_ready(OSC_SLCK_32K_RC);
	pmc_switch_mck_to_sclk(SYSCLK_PRES_1);
	/* Switch clock to fast RC */
#if SAMG55
	osc_enable(OSC_MAINCK_24M_RC);
	osc_wait_ready(OSC_MAINCK_24M_RC);
#else
	osc_enable(OSC_MAINCK_12M_RC);
	osc_wait_ready(OSC_MAINCK_12M_RC);
#endif
	pmc_switch_mck_to_mainck(SYSCLK_PRES_1);

	/* Modify vector table location */
	__DSB();
	__ISB();
	SCB->VTOR = ((uint32_t)addr & SCB_VTOR_TBLOFF_Msk); // 7 bit alignment and rest are EA
	__DSB();
	__ISB();
	__enable_irq();
	/* Jump to application */
	jump_to_app(addr);
	/* Never be here */
	return;
}
	void upgrade_sys_fw(uint8_t system_upgrade) {
			main_loop_on = true;	// to start up upgrade proc on host...
			if (1 == system_upgrade) {
				cpld_flash_map fdo ;
				// altera cpld upgrade
/*****************************************************/
				fdo.page_size =  16 * 1024 / 8;	// page size of max 10 CFM0 in byte
				// the rest fields are not used at all
/*****************************************************/
//				pio_set(PIOA, PIO_PA0);
		extern void download_cpld_fw(cpld_flash_map* fdo, uint32_t *upgrade_fw_hdr );
				download_cpld_fw(&fdo, upgrade_fw_hdr );
//				pio_clear(PIOA, PIO_PA0);
				delay_ms(100);
/*****************************************************/
				/* stop usb device operation */
				udc_stop();
				/* run application */
				_app_exec(APP_START);
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
  static void start_it913x_spi(bool restart)
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

	//control path related
	U8 ctrl_ledstate=0;
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
 #ifdef RECV_IT913X
	pio_set_output(PIOA, PIO_PA2, LOW, DISABLE, ENABLE);
	delay_ms(10);  // reset IT931x
	pio_set(PIOA, PIO_PA2);
 	pio_set_output(PIOB, PIO_PB9, LOW, DISABLE, ENABLE); //stop TS gate
 	delay_ms(10); // flush all data from Pipe
 #endif
	twi_master_initialize(TWI_CLK); // communicate with it913x, liyenho
	spi_master_initialize(0, SPI0_MASTER_BASE, BOARD_FLEXCOM_SPI0);// 2072 ctrl pipe
	spi_slave_initialize();  // shall be applied on sms video pipe
	g_p_spim_pdc[1] = g_p_spis_pdc;
	// Start USB stack to authorize VBus monitoring
	udc_start();
system_restart:  // system restart entry, liyenho
	//pio_set_output(PIOA, PIO_PA0, LOW, DISABLE, ENABLE); // flag of cpld remote upgrade, liyenho
	system_main_restart = false;
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
#ifdef RECV_IT913X
 #ifndef RX_SPI_CHAINING
	i2c_read_cb_on = false ;
	first_in = true; // be sure to reset 'first time flag' too
 #endif
	pio_clear(PIOB, PIO_PB9); // disable TS gate
	spi_disable(g_p_spis_pdc); // I think that is only spi need to be reset? liyenho
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
 #ifdef CONFIG_RF2072
 		pio_set_output(PIOA, PIO_PA17, HIGH, DISABLE, ENABLE); // set enbl to low after inverted
		pio_set_output(PIOA, PIO_PA16, HIGH, DISABLE, ENABLE); // rf2072 out of reset
		pio_set_output(PIOA, PIO_PA23, LOW, DISABLE, ENABLE); // flag to signal cpld to insert 1 sdio clk pulse, rf2072 access
		pio_set_output(PIOA, CPLD_2072_TRIG, HIGH, DISABLE, ENABLE); // extra trigger line for 2072 access with cpld, liyenho
		if (system_upgrade)
			upgrade_sys_fw(system_upgrade);
 #endif
#ifdef RECV_IT913X
		usb_data_done = true;
#endif
//////////////////////////////////////////////////////////////////////////////
	init_4463();  // this init function has to be after siano came up!? liyenho
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
	while (true) {
		if (system_main_restart ) goto system_restart;
		if (system_upgrade)
			upgrade_sys_fw(system_upgrade);
		if (!stream_flag) goto _reg_acs; // stop TS stream if flag isn't true, liyenho

#ifdef RX_SPI_CHAINING
	  //Ctrl led control -------------------------------------------------
	  if(ctrl_sndflag >0 )
	    ctrl_sndflag--;
	  if(ctrl_sndflag == (100-1) ) {
		if(ctrl_ledstate ==0)
	    {
			ctrl_ledstate=1;
			}
		else
		{
			ctrl_ledstate=0;
			} //force blinking if ctrl too fast
	  }

	  if(ctrl_sndflag ==1)  {
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
					uint32_t er_adr =ul_page_addr-SECTOR_RES1;
					ul_rc = flash_erase_sector(er_adr);
					if (ul_rc != FLASH_RC_OK) {
						//printf("- Pages erase error %lu\n\r", (UL)ul_rc);
						return; /* error when erase pages */
					}
					CHECKED_FLASH_WR(ul_page_addr_ctune,
																			&si4463_factory_tune.median,
																			1/*1 byte flag*/)
					uint8_t ctemp = get_si446x_temp();
					CHECKED_FLASH_WR(ul_page_addr_mtemp,
																			&ctemp, 1/*1 byte flag*/)
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
#endif //RX_SPI_CHAINING
_reg_acs:
  		if (usb_host_msg && !system_main_restart) {	// host ctrl/sts link with fpga/sms process, liyenho
			usb_host_msg = false;
//#define TEST_FLASH
 #ifdef TEST_FLASH
	uint32_t shf, wtmp;
 #endif
#if defined(CONFIG_RF2072)
			dev_access *pr=(dev_access*)gs_uc_hrbuffer, *pt = (dev_access*)gs_uc_htbuffer;
			uint16_t tmp, tmpw, *pth = &tmp;
			uint8_t *pdbg = pt->data;  // to watch data content
			#include <assert.h>
			switch (pt->access) {
				case RF2072_RESET:
						pio_clear (PIOA, PIO_PA16);
						delay_ms(1);
						pio_set (PIOA, PIO_PA16);
						delay_us(1);
						pio_clear (PIOA, PIO_PA17); // keep enbl low
						while (spi_tgt_done) ; // flush any pending spi xfer
							spi_tgt_done = true;
							ACCESS_PROLOG_2072
							// setup spi to write addressed data
							*pth = 0x7f & 0x9; // set relock in PLL_CTRL
							spi_tx_transfer(pth, 1, &tmpw, 1, 0/*ctrl/sts*/);
							while (spi_tgt_done) ; spi_tgt_done = true;
							*pth = (uint16_t)0x0; // high byte
							spi_tx_transfer(pth, 1, &tmpw, 1, 0/*ctrl/sts*/);
							while (spi_tgt_done) ; spi_tgt_done = true;
							*pth = (uint16_t)0x8; // low byte
							spi_tx_transfer(pth, 1, &tmpw, 1, 0/*ctrl/sts*/);
							while (spi_tgt_done) ;
						 delay_us(1);
						 pio_set(PIOA, CPLD_2072_TRIG);
						break;
				case RF2072_READ:
						assert(!(pt->dcnt & 1));
						while (spi_tgt_done) ; // flush any pending spi xfer
							spi_tgt_done = true;
							ACCESS_PROLOG_2072
							*pth = 0x80| (0x7f&pt->addr); // read access
							spi_tx_transfer(pth, 1, &tmpw, 1, 0);
							while (spi_tgt_done);
							spi_tgt_done = true;
							READ_MID_PROC_2072
							spi_rx_transfer(pth, 1, &tmpw, 1, 0/*ctrl/sts*/); // high byte
							while (spi_tgt_done) ; spi_tgt_done = true;
							pr->data[1] = 0xff & tmpw;
							spi_rx_transfer(pth, 1, &tmpw, 1, 0/*ctrl/sts*/); // low byte
							while (spi_tgt_done) ;
							pr->data[0] = (0xff & tmpw);
#ifdef TEST_FLASH
	wtmp = *(uint32_t*)(~0x3&(ul_page_addr_c+pt->addr));
	shf = ((ul_page_addr_c+pt->addr) & 0x3) * 8;
  *(uint8_t*)pr->data = 0xff & (wtmp >> shf);
#endif
							READ_END_REV_2072
  volatile uint16_t readv = *(uint16_t*)pr->data; // for debug
							break;
				case RF2072_WRITE:
						assert(!(pt->dcnt & 1));
						while (spi_tgt_done) ; // flush any pending spi xfer
							spi_tgt_done = true;
							ACCESS_PROLOG_2072
							// setup spi to write addressed data
							*pth = 0x7f & pt->addr; // write access
							spi_tx_transfer(pth, 1, &tmpw, 1, 0/*ctrl/sts*/);
							while (spi_tgt_done) ;
							spi_tgt_done = true;
							*pth = (uint16_t)pt->data[1]; // high byte
							spi_tx_transfer(pth, 1, &tmpw, 1, 0/*ctrl/sts*/);
							while (spi_tgt_done) ; spi_tgt_done = true;
							*pth = (uint16_t)pt->data[0]; // low byte
							spi_tx_transfer(pth, 1, &tmpw, 1, 0/*ctrl/sts*/);
							while (spi_tgt_done) ;
						  delay_us(1);
							pio_set(PIOA, CPLD_2072_TRIG);
							break;
				case IT913X_READ:
							_TWI_READ_(pr->addr,pr->data,pr->dcnt)
							break;
				case IT913X_WRITE:
							TWI_WRITE(pt->addr,pt->data,pt->dcnt)
							break;
				case IT913X_DOWNLOAD:
							download_ite_fw(pt->dcnt, pt->addr, pt->data );
							break;
				case TS_VID_ACTIVE:
							#if defined(RECV_IT913X)
								// now allow video spi to be active, liyenho
								spi_enable(SPI_SLAVE_BASE);
							#ifndef RX_SPI_CHAINING
								pio_set(PIOB, PIO_PB9); // enable TS gate
							#else
								// pull over to here instead init inside main loop to get rid of time constraint, liyenho
								start_it913x_spi(false);
							#endif
							#endif
							break;
				default: /* host msg in error */
							continue;
			}
#endif //CONFIG_RF2072
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

volatile bool main_usb_host_msg() // Attach this api to usb rx isr service, liyenho
{
	if (USB_HOST_MSG_TX_VAL != udd_g_ctrlreq.req.wValue &&
		 USB_CPLD_UPGRADE_VAL != udd_g_ctrlreq.req.wValue &&
		 USB_ATMEL_UPGRADE_VAL != udd_g_ctrlreq.req.wValue
		 && USB_ITE_FW_VAL != udd_g_ctrlreq.req.wValue
		)
		return false ;
	if (USB_HOST_MSG_IDX != udd_g_ctrlreq.req.wIndex)
		return false ;
	if (USB_HOST_MSG_LEN > udd_g_ctrlreq.req.wLength
		&& sizeof(short) != udd_g_ctrlreq.req.wLength
		&& sizeof(uint32_t) != udd_g_ctrlreq.req.wLength
		&& ITE_FW_HDR_LEN != udd_g_ctrlreq.req.wLength
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

volatile bool main_vender_specific() {
	if (USB_SYSTEM_RESTART_VAL == udd_g_ctrlreq.req.wValue) {
		main_loop_restart(); return true;
	}
	if (USB_HOST_MSG_TX_VAL == udd_g_ctrlreq.req.wValue ||
		USB_CPLD_UPGRADE_VAL == udd_g_ctrlreq.req.wValue ||
		USB_ATMEL_UPGRADE_VAL == udd_g_ctrlreq.req.wValue
		|| USB_ITE_FW_VAL == udd_g_ctrlreq.req.wValue
		)
		return main_usb_host_msg();
	else if (USB_HOST_MSG_RX_VAL == udd_g_ctrlreq.req.wValue)
		return main_usb_host_reply();
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
			month[0] = __REL_DATE__[0];
			 month[1] = __REL_DATE__[1];
			  month[2] = __REL_DATE__[2];
			   month[3] = 0x0;
			day[0] = __REL_DATE__[4];
			 day[1] = __REL_DATE__[5];
			  day[2] = 0x0;
			year[0] = __REL_DATE__[7];
			 year[1] = __REL_DATE__[8];
			  year[2] = __REL_DATE__[9];
			   year[3] = __REL_DATE__[10];
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


