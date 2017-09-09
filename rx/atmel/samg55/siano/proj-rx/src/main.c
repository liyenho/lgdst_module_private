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
#include "bsp.h"
#include "lgdst_4463_spi.h"
#if (SAMG55)
#include "flexcom.h"
#endif

static volatile bool system_main_restart = false;  // system restart flag, liyenho
static volatile uint8_t system_upgrade = 0;  // system upgrade flag, liyenho
/*static*/ char __version_atmel__[3];  // stored as mon/day/year
#ifdef RADIO_SI4463
  #include "si446x_nirq.h"
  #include "radio.h"
#endif

#include "ReedSolomon.h"
#include "ctrl.h"
#include "USB_Commands.h"
#include "GPS.h"
#include "Radio_Buffers.h"
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
  volatile bool ctrl_tdma_lock = false;  //always, since RX is master
  //volatile bool ctrl_pkt_rd_available = false;
	static int timedelta(bool reset, unsigned int bignum, unsigned int smallnum);
	static bool fhop_dir, timedelta_reset ; //to handle system restart
	/*static*/ uint8_t hop_id[HOP_ID_LEN]; // 10 byte hop id from host

  /*static*/ enum pair_mode hop_state;
  	volatile unsigned char ynsdbyte ; //Take care in Rate Control Section!!! liyenho
#endif
volatile RECEPTION_STATISTICS_ST recptStatG; //legacy: for liyen use
volatile RECEPTION_STATISTICS_ST recptStatG_m;
volatile RECEPTION_STATISTICS_ST recptStatG_s;
volatile Short_Statistics_ST shortStatG;
unsigned int log2a[100];
unsigned char log2p =0;
volatile bool ctrl_tdma_rxactive = false;
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
/*static*/ uint32_t gs_uc_tbuffer[2*I2SC_BUFFER_SIZE/sizeof(int)];
volatile static uint32_t usbfrm = 0, prev_usbfrm= 0;
 //algorithm from digibest sdk, using bulk transfer pipe, liyenho
volatile uint32_t upgrade_fw_hdr[FW_UPGRADE_HDR_LEN/sizeof(int)]={-1} ;
#if defined(FWM_DNLD_DBG)
  volatile bool usb_host_active = false;
  extern volatile bool usb_tgt_active ;
#endif
 // host to fpga/6612 ctrl/sts buffer
static uint32_t gs_uc_htbuffer[(USB_HOST_MSG_LEN+HOST_BUFFER_SIZE+3)/sizeof(int)];
uint32_t gs_uc_hrbuffer[(USB_HOST_MSG_LEN+HOST_BUFFER_SIZE+3)/sizeof(int)],
					gs_uc_hrbuffer1[(USB_HOST_MSG_LEN+HOST_BUFFER_SIZE+3)/sizeof(int)];
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

unsigned char tpacket_grp[RADIO_GRPPKT_LEN];
unsigned char gs_rdo_tpacket_ovflw=0;
unsigned char snd_asymm_cnt=0;

volatile uint32_t rpacket_idle[RDO_ELEMENT_SIZE];
uint32_t rpacket_ov[RDO_ELEMENT_SIZE];
unsigned char rpacket_grp[RADIO_GRPPKT_LEN+RADIO_INFO_LEN]; // fixed
unsigned char gs_rdo_rpacket_ovflw=0;

unsigned int  radio_mon_txfailcnt=0;
unsigned int  radio_mon_rxcnt = 0;
unsigned int  radio_mon_txcnt = 0;

bool TX_Active = false;
  // 4463 stats mon obj
  extern volatile ctrl_radio_stats  r4463_sts;
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
#ifndef RX_SPI_CHAINING
  void usb_write_buf_cb();
#endif
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
  /*const*/ COMPILER_ALIGNED(8) tRadioConfiguration RadioConfiguration_915 = RADIO_CONFIGURATION_DATA_915;
  /*const*/ COMPILER_ALIGNED(8) tRadioConfiguration RadioConfiguration_869 = RADIO_CONFIGURATION_DATA_869;
  /*const*/ tRadioConfiguration *pRadioConfiguration = &RadioConfiguration_915;
  SEGMENT_VARIABLE(bMain_IT_Status, U8, SEG_XDATA);

#endif

void SysTick_Handler(void)
{
	static uint32_t ul_2ms_ticks = 0;
	ul_2ms_ticks ++;
#ifdef  RADIO_SI4463
	if(si4463_radio_started){//ctrl_tdma_enable) {
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
		  if(fifolvlcalc(wrptr_rdo_tpacket, rdptr_rdo_tpacket, RDO_TPACKET_FIFO_SIZE)<1)
		  { //idle packet case
			  //add idle packet to the output queue
			  Queue_Control_Idle_Packet();
		  }

		  //setup for radio send
		  // rdptr_inc(&wrptr_rdo_tpacket, &rdptr_rdo_tpacket, RDO_TPACKET_FIFO_SIZE, 1);
		  gp_rdo_tpacket_l = gs_rdo_tpacket + (RDO_ELEMENT_SIZE*rdptr_rdo_tpacket);
		  snd_asymm_cnt = ASYMM_RATIO-1;

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
		if(true){//hop_state!= IDLE){

			if (!USE_915MHZ){
				vRadio_StartTx(control_channel, gp_rdo_tpacket_l,RADIO_LONG_PKT_LEN);
				snd_asymm_cnt = 0;
			}else{
				vRadio_StartTx(control_channel,gp_rdo_tpacket_l,RADIO_PKT_LEN);
			}
		}
	    radio_mon_txcnt++;
		tdma_sndthr = tdma_sndthr + TDMA_PERIOD;
	  }
	} //if(ctrl_tdma_enable)
#endif
#ifndef DIGIBEST_DOWNLOAD
	//g_ul_10ms_ticks = ul_2ms_ticks / 5;
	//if (systick_enabled && g_ul_wait_10ms<=g_ul_10ms_ticks) {
		//systick_enabled = false;
		//g_ul_10ms_ticks = 0;
		//// pending task must finish in 10 msec? not sure if NVIC can handle
		//twi_sms4470_handler(ID_PIOA, SMS_HOST_INT);
	//}
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
void usb_write_buf1(void *pb, int size0);
	/*static inline*/ void usb_write_buf1(void *pb, int size0)
	{
		int written=0, size = size0;
		do {
			iram_size_t b = size-udi_cdc_write_buf(pb, size);
			pb += b;
			size -= b;
			written += b;
		} while (size0 != written && !system_main_restart);
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
	} while (size0 != read && !system_main_restart);
}

fordigibest_usb_read_buf1(void* *pbuff, int size0)
{
	*pbuff = fw_sms_rbuffer;
	usb_read_buf1(fw_sms_rbuffer, size0);
}
#endif

#if defined(RECV_SMS4470) && !defined(RX_SPI_CHAINING)
  volatile uint32_t cycle_cnt = 0; // global cpu cycle counter
  volatile uint32_t cycle_delta_0,  cycle_delta_1;
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
			usb_write_buf1(sms4470_usb_ctx.usb_buf_wr,I2SC_BUFFER_SIZE);
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
  extern void download_sms_fw(uint32_t sync, uint8_t *chk );
  extern void download_sms_data(uint32_t sync, uint8_t *chk );
  extern void siano_tuning(uint32_t fc);
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
		dev_access *ps = gs_uc_htbuffer,*ps1 = gs_uc_hrbuffer1;
		// prepare confirm msg by echo whatever received
		memcpy(gs_uc_hrbuffer1, udd_g_ctrlreq.payload, USB_HOST_MSG_LEN-sizeof(ps->data[0]));
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
		dev_access *pr =(dev_access*)gs_uc_hrbuffer1, *pt = (dev_access*)gs_uc_htbuffer;
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
extern void recalibrate_capval (void* ul_page_addr_mtemp, uint8_t median);
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
  if(Result != TRUE)
	while(1){}
 	 TuningParameter.TuningFrequency = 482000000;   //482001000, 5000,10000,50000,100000, 481995000-BAD
	                                     //473985000 gives 0x074 Carrier offset (digibest brd)
													//474010000 gives -8254
													//474020000 gives -6232
													// every 10000hz gives 2000 carrierOffset increase
													//474045000 gives -298 (rx v1 brd)
 	 TuningParameter.BandwidthType = BANDWIDTH_6_MHZ;
 	 TuningParameter.DemodulatorTuningFeature = DVBT_DEMODULATOR_TUNING;

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

	main_loop_on = true;  // enter run time stage, liyenho

	// The main loop manages
#if defined(RECV_SMS4470)
  #if defined(RX_SPI_CHAINING)
		mon_spidmachainfail_cnt  = 0;
		mon_ts47bad_cnt = 0;
	// from spi enabled to start dma can't be longer than 488 us, bad design, liyenho
//		configure_rtt(16); // arm 500us isr for SPI/USB data pipe processing
		configure_rtt(32); // try out 1 ms thought, it works perfectly...
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
#if FIXED_PAIR_ID
	uint8_t hop_id[HOP_ID_LEN] = {1};
	Set_Pair_ID(hop_id);
#endif //FIXED_PAIR_ID

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
  		cap_bank_calibrate();
#ifdef CTRL_DYNAMIC_MOD
		si446x_get_int_status(0xff, 0xff, 0xff);
		if (Si446xCmd.GET_INT_STATUS.PH_PEND & SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_PACKET_SENT_PEND_BIT)
			bMain_IT_Status = SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_PACKET_SENT_PEND_BIT;
		else if(Si446xCmd.GET_INT_STATUS.PH_PEND & SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_PACKET_RX_PEND_BIT)
			bMain_IT_Status = SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_PACKET_RX_PEND_BIT;
  		process_range_mode(bMain_IT_Status);
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
	SMS4470_check_signal_lh(&sms_dvbt_lock); // apply callback ext, liyenho
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
			dev_access *pr=(dev_access*)gs_uc_hrbuffer1, *pt = (dev_access*)gs_uc_htbuffer;
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
		dev_access *ps = gs_uc_hrbuffer1;
	if (USB_HOST_MSG_LEN-sizeof(ps->data[0]) == udd_g_ctrlreq.req.wLength)
		// (0==ps->dcnt) meant confirmation of write cmd, liyenho
		cnf_echo = 1;
		// must setup packet size
		if (!cnf_echo) {
			udd_set_setup_payload( gs_uc_hrbuffer1,
				USB_HOST_MSG_LEN+(ps->dcnt-1)*sizeof(uint16_t));
		} else { //confirmation by echo, 1 data entry is included
			udd_set_setup_payload( gs_uc_hrbuffer1,
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
		if (RADIO_STARTUP_IDX == udd_g_ctrlreq.req.wIndex) {
			pio_clear(PIOB, PIO_PB9); // disable TS gate
#ifdef RX_SPI_CHAINING
			spidma_active = FALSE;
#else //!RX_SPI_CHAINING
			sms4470_usb_ctx.spi_buf_rd -= I2SC_BUFFER_SIZE; // equivalent to drop a block to align
			if (sms4470_usb_ctx.buf_start>sms4470_usb_ctx.spi_buf_rd)
				sms4470_usb_ctx.spi_buf_rd = sms4470_usb_ctx.buf_end;
#endif
		}
		usb_ctrl_cmd_portal(&udd_g_ctrlreq);
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
