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
#include "bsp.h"
#include "lgdst_4463_spi.h"
#if (SAMG55)
#include "flexcom.h"
#endif
#if defined(RECV_IT913X)
  #include "ite.h"
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
volatile uint8_t dbg_ctrlvidbuff[10]; //0x92: new data 0x93: stale data
//           ctrlcnt, inc on every st16->drone control data
//           tdma_lock(bit0), antenna(bit2,3)
//           video188ctrlTransferCnt: inc on every 188 ctrl pkt /4
//           gotPayloadSize, valid
#ifdef RADIO_SI4463
  volatile uint8_t spi_radio_done = false;
  volatile uint8_t spi_dma_mode = false;
#endif
static volatile bool main_b_cdc_enable = false;
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
  volatile bool ctrl_tdma_lock = false;  //always, since RX is master
  //volatile bool ctrl_pkt_rd_available = false;
	static int timedelta0(bool *reset, unsigned int bignum, unsigned int smallnum);
	int timedelta(unsigned int bignum, unsigned int smallnum); // no static var inside
	static bool fhop_dir, timedelta_reset ; //to handle system restart
	/*static*/ uint8_t hop_id[HOP_ID_LEN]; // 10 byte hop id from host

  /*static*/ enum pair_mode hop_state;
  	volatile unsigned char ynsdbyte ; //Take care in Rate Control Section!!! liyenho
	bool control_channel_scan_complete = false;
#endif
volatile bool ctrl_tdma_rxactive = false;
volatile bool stream_flag = true; // default to TS stream automatic on
volatile uint8_t id_byte=ID_BYTE_DEFAULT; //to be used in ctrl.c file
#ifdef VIDEO_DUAL_BUFFER
	extern volatile int32_t stream;
#endif
#ifdef CONFIG_ON_FLASH
	uint32_t ul_page_addr_ctune =IFLASH_ADDR + IFLASH_SIZE - NUM_OF_ATMEL_REGS, // 1st atmel reg on flash
						// temperature @ current tuning @ 2nd atmel reg on flash
					ul_page_addr_mtemp =IFLASH_ADDR + IFLASH_SIZE - NUM_OF_ATMEL_REGS + 1,
					ul_page_addr_bootapp =IFLASH_ADDR + IFLASH_SIZE - NUM_OF_ATMEL_REGS + 2;
uint8_t backup[NUM_OF_FPGA_REGS+NUM_OF_ATMEL_REGS+4+1],
				bootapp = (uint8_t)-1;
#endif
extern udd_ctrl_request_t udd_g_ctrlreq; // from udp_device.c, liyenho
volatile uint32_t g_ul_10ms_ticks=0, g_ul_wait_10ms=0;
/*static*/ bool systick_enabled = false;  // default to disable sys timer
volatile static uint32_t g_ul_led_ticks=0, g_ul_wait_100ms=10, g_ul_wait_1s=100;
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
  bool first_in = true ;
	const uint32_t vch_tbl[] = {
		2392, 2406, 2413, 2420,
		2427, 2434, 2441, 2448,
		2455, 2462, 2469
	};  // kept for mem alloc and reference
  	static uint8_t vch= 0; //dynamic video chan select
 extern volatile bool reset_ts_count_error;
 extern volatile bool gl_vid_ant_sw;
#else // non-real time mode
	uint32_t gs_uc_rbuffer[2*I2SC_BUFFER_SIZE/sizeof(int)];
#endif
static uint32_t gs_uc_tbuffer[2*I2SC_BUFFER_SIZE/sizeof(int)];
#ifdef CTRL_RADIO_ENCAP
extern uint8_t  pb_rdo_ctrl[] ,
								*pb_rdo_ctrl_e;
#endif
volatile static uint32_t usbfrm = 0, prev_usbfrm= 0;
volatile uint32_t upgrade_fw_hdr[FW_UPGRADE_HDR_LEN/sizeof(int)]={-1} ;
#if defined(FWM_DNLD_DBG)
  volatile bool usb_host_active = false;
  extern volatile bool usb_tgt_active ;
#endif
 // host to fpga/2072 ctrl/sts buffer
static uint32_t gs_uc_htbuffer[(USB_HOST_MSG_LEN+HOST_BUFFER_SIZE+3)/sizeof(int)],
						gs_uc_htbuffer_tm[(USB_HOST_MSG_LEN+HOST_BUFFER_SIZE+3)/sizeof(int)];
uint32_t gs_uc_hrbuffer[(USB_HOST_MSG_LEN+HOST_BUFFER_SIZE+3)/sizeof(int)],
					gs_uc_hrbuffer1[(USB_HOST_MSG_LEN+HOST_BUFFER_SIZE+3)/sizeof(int)];
  //volatile bool i2c_read_done = true;
  volatile context_it913x ctx_913x={0};
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
   extern uint32_t fw_ite_rbuffer[FW_DNLD_SIZE/sizeof(int)];
 #endif
/* Pointer to UART PDC register base */
Pdc *g_p_spim_pdc [1+2]/*fpga/sms, video, maybe more later, liyenho*/,*g_p_spis_pdc;
Pdc *g_p_i2st_pdc, *g_p_i2sr_pdc;
struct i2s_dev_inst dev_inst_i2s;
#ifdef RADIO_SI4463
 static uint8_t tune_cap_str[] = {RF_GLOBAL_XO_TUNE_2}; // used by cap val tuning process internally

unsigned char gs_rdo_tpacket_ovflw=0;
volatile uint32_t rpacket_idle[RDO_ELEMENT_SIZE];
unsigned char gs_rdo_rpacket_ovflw=0;

unsigned int  radio_mon_txfailcnt=0;
unsigned int  radio_mon_rxcnt = 0;


bool TX_Active = false;
uint32_t pkt_start_tick=0;
  // 4463 stats mon obj
  extern volatile ctrl_radio_stats  r4463_sts;
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

static volatile int PB12_cnt =0;
static volatile uint32_t tm_btn_dwn_prev= 0;
void PB12_pin_edge_handler(const uint32_t id, const uint32_t index)
{
	if ((id == ID_PIOB) && (index == PIO_PB12)){
		PB12_cnt++;
		uint32_t tm_btn_dwn= *DWT_CYCCNT;
		if (!tm_btn_dwn_prev) {
			goto assign_prev;  // arm the handler, nothing else to do
		}
		else {
			int tdel= timedelta(tm_btn_dwn,
													tm_btn_dwn_prev);
			if (120000000 < tdel)
				gl_vid_ant_sw = 2; // switch to next antenna
			else  // pressed more than once within a sec...
				gl_vid_ant_sw = 1; // switch to original antenna
		}
assign_prev:
		tm_btn_dwn_prev = tm_btn_dwn;
	}
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
			ctx_913x.i2c_done_rd = false;
			uint32_t err, // error code requried
								len0 = packet_rx.length;
			static uint8_t tmp_buff[HOST_BUFFER_SIZE];
			static uint32_t err_prev, len_prev;
#ifndef RX_SPI_CHAINING
			if (i2c_read_cb_on) {
				TWI_READ_CB
			} else
#endif
			{	TWI_READ	}
			ctx_913x.i2c_done_rd = true;
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
	if(ctrl_tdma_enable) {
	  // local declarations ------------------------------
	  static unsigned int tdma_sndthr=0;
	  unsigned int lc;
	  int cdelta;

	  //clock adjustment ---------------------------------
	  lc = *DWT_CYCCNT;  //not working when in "no-debug" mode
	  cdelta = timedelta0(&timedelta_reset, tdma_sndthr, lc);
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
		//make sure there are enough messages to send
		#if SEND_MAVLINK
		if (RADIO_PKT_LEN>outgoing_messages.byte_cnt) // look for the current byte cnt instead pkt cnt
		{
			Queue_Idle_Mavlink();
		}
		#else																		// bad codings again, 3 is not necessary, instead 1 can be applied in 2 ms intr period, liyenho
		while (fifolvlcalc(wrptr_rdo_tpacket, rdptr_rdo_tpacket, RDO_TPACKET_FIFO_SIZE)<1){
			Queue_Control_Idle_Packet();
		}
		gp_rdo_tpacket_l = gs_rdo_tpacket + (RDO_ELEMENT_SIZE*rdptr_rdo_tpacket);
		#endif

		//setup for radio send
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
		tdma_sndthr = tdma_sndthr + TDMA_PERIOD;
		if(hop_state!= IDLE){
			pkt_start_tick = *DWT_CYCCNT;
			Control_Send_Event();
		}
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
 void usb_write_buf1(void *pb, int size0);
	/*static inline*/ void usb_write_buf1(void *pb, int size0)
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
/*! \brief host ctrl/sts callback for sys upgrade interface
 */
 void erase_last_sector() {
		memcpy(backup, (void*)(IFLASH_ADDR + IFLASH_SIZE-sizeof(backup)), sizeof(backup));
		uint32_t er_adr =IFLASH_ADDR + IFLASH_SIZE-SECTOR_SIZE_L;
		flash_erase_sector(er_adr);
	}
  static void host_usb_cb() {
		if (Is_udd_in_sent(0) ||
			udd_g_ctrlreq.payload_size >/*!=*/ udd_g_ctrlreq.req.wLength)
			return; // invalid call
		if (VIDEO_SETVCH_VAL == udd_g_ctrlreq.req.wValue) {
			if (sizeof(vch_tbl)>vch) {
				pio_clear(PIOB, PIO_PB9); // disable TS gate
				spi_disable(g_p_spis_pdc);
			}
		}
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
		else if ((USB_ITE_FW_VAL == udd_g_ctrlreq.req.wValue) &&
						ITE_FW_HDR_LEN >=udd_g_ctrlreq.payload_size) {
			memcpy(it913x_fw_hdr, // set ite913x fw hdr, liyenho
								udd_g_ctrlreq.payload,
								ITE_FW_HDR_LEN);
			return;
		}
		dev_access *ps = gs_uc_htbuffer,*ps1 = gs_uc_hrbuffer1;
		 /////////////////
		//host/client handshake time sensitive processes (YH)
		if (0!=ctx_913x.it913x_access &&
			(!ctx_913x.i2c_done_rd || !ctx_913x.i2c_done_wr))
			return ; // no further action taken
       if (ps->access == IT913X_READ) {
	      ctx_913x.it913x_access = IT913X_READ;
	      ctx_913x.i2c_done_rd = false;  //YH: not-ready response to usb request
	      ps->dcnt += TWI_ERR_PROT_RD;
		}
		else if (ps->access == IT913X_WRITE ) {
	      ctx_913x.it913x_access = IT913X_WRITE;
	      ctx_913x.i2c_done_wr = false;  //YH: not-ready response to usb request
		}
		// prepare confirm msg by echo whatever received
		memcpy(gs_uc_hrbuffer1, udd_g_ctrlreq.payload, USB_HOST_MSG_LEN-sizeof(ps->data[0]));
		usb_host_msg = true; // enable mainloop process
  }
extern void recalibrate_capval (void* ul_page_addr_mtemp, uint8_t median);
extern void download_ite_fw(uint32_t fw_dnld_size, uint8_t address, uint8_t *chk );
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
static void init_4463(void)
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
	uint32_t i, n, tdel, tcurr, last_three_sec;
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
	configure_console(); // will be used but inited along with cdc comm module
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
	NVIC_DisableIRQ(SPI_IRQn);  // spi5 peripheral instance = 21,
	NVIC_ClearPendingIRQ(SPI_IRQn);
	NVIC_SetPriority(SPI_IRQn, 1);
	NVIC_EnableIRQ(SPI_IRQn);
#endif
	NVIC_DisableIRQ(SPI0_IRQn);  // spi3/7 peripheral instance = 19/7,
	NVIC_ClearPendingIRQ(SPI0_IRQn);
	NVIC_SetPriority(SPI0_IRQn, 1);
	NVIC_EnableIRQ(SPI0_IRQn);
 #ifdef RADIO_SI4463
	 NVIC_DisableIRQ(PIOB_IRQn);  // piob radio instance = 12,
	 NVIC_ClearPendingIRQ(PIOB_IRQn);
	 NVIC_SetPriority(PIOB_IRQn, 1);
	 NVIC_EnableIRQ(PIOB_IRQn);
	 NVIC_DisableIRQ(SPI7_IRQn);  // spi7 peripheral instance = 7,
	 NVIC_ClearPendingIRQ(SPI7_IRQn);
	 NVIC_SetPriority(SPI7_IRQn, 1);
	 NVIC_EnableIRQ(SPI7_IRQn);
 #endif
 	if (SysTick_Config(sysclk_get_cpu_hz() / 250)) { // 4 msec tick
		//puts("-E- Systick configuration error\r");
		while (1) {
			// Capture error
		}
	}
	twi_master_initialize(TWI_CLK); // communicate with it913x, liyenho
	spi_master_initialize(0, SPI0_MASTER_BASE, BOARD_FLEXCOM_SPI0);// 2072 ctrl pipe
	spi_slave_initialize();  // shall be applied on sms video pipe
	g_p_spim_pdc[1] = g_p_spis_pdc;
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

#if CW_MODE//mode for generation unmoulated carrier wave
	init_4463();
	vRadio_Init();
	vRadio_StartTx_No_Data(pRadioConfiguration->Radio_ChannelNumber);
	while(1){
		//infinite loop
	}
#endif

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
	pio_set_output(PIOA, PIO_PA20, LOW, DISABLE, ENABLE); // 12 mhz crystal
	pio_set_output(PIOA, PIO_PA2, LOW, DISABLE, ENABLE);
	delay_ms(/*100*/200);  // reset IT931x
	pio_set(PIOA, PIO_PA2);
	delay_ms(/*100*/300);  // pull IT913x out of reset
 	pio_set_output(PIOB, PIO_PB9, LOW, DISABLE, ENABLE); //stop TS gate
 	delay_ms(10); // flush all data from Pipe
 #ifndef RX_SPI_CHAINING
	i2c_read_cb_on = false ;
 #endif
	first_in = true; // be sure to reset 'first time flag' too
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

//disable ERASE function on PB12
*(uint32_t *)(0x400E0314) |= (1u<<12) ;

// manual Tx video antenna diversity switch
//set PB12 as input
pio_set_input(PIOB, PIO_PB12, PIO_DEBOUNCE);
//enable pull down resistor on pin
pio_pull_down(PIOB, PIO_PB12, 1);
//debounce switch, 1Hz(?) filter
pio_set_debounce_filter(PIOB, PIO_PB12, 32768/2);

//pio_handler_set(PIOB, ID_PIOB, PIO_PB12, PIO_IT_AIME| PIO_IT_RISE_EDGE, PB12_pin_edge_handler);
//pio_enable_interrupt(PIOB, PIO_PB12);
//pio_handler_set_priority(PIOB, PIOB_IRQn, 1/*long latency event*/);


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
	rdptr_rdo_tpacket=RDO_TPACKET_FIFO_SIZE-1;
	wrptr_rdo_tpacket=RDO_TPACKET_FIFO_SIZE-1;
	rdptr_rdo_rpacket=RDO_RPACKET_FIFO_SIZE-1;
	wrptr_rdo_rpacket=RDO_RPACKET_FIFO_SIZE-1;
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
#if defined(RECV_IT913X)
		reset_ts_count_error = true;
  #if defined(RX_SPI_CHAINING)
		mon_spidmachainfail_cnt  = 0;
		mon_ts47bad_cnt = 0;
	// from spi enabled to start dma can't be longer than 488 us, bad design, liyenho
    //		configure_rtt(16); // arm 500us isr for SPI/USB data pipe processing
	#ifndef DEBUG_BLOCK_TS_ISR
	configure_rtt(32); // try out 1 mc though?
	#endif
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
#endif

#if FIXED_PAIR_ID
	uint8_t hop_id[HOP_ID_LEN] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 1};
	Set_Pair_ID(hop_id);
#endif //FIXED_PAIR_ID

	while (true) {
		if (system_main_restart ) goto system_restart;
		if (system_upgrade)
			upgrade_sys_fw(system_upgrade);
		  // ctrl RX Processing -----------------------------------------------
		  if(si4463_radio_started)
		  {
	  #ifdef TEMPERATURE_MEASURE
				static int once = false;
				if (!once) {
					vRadio_StartTx_No_Data(pRadioConfiguration->Radio_ChannelNumber); // continued transmit & stop listening, liyenho
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
		if (!stream_flag) goto _reg_acs; // stop TS stream if flag isn't true, liyenho

#ifdef RX_SPI_CHAINING
	if(spidma_active != TRUE) {
#ifdef VIDEO_DUAL_BUFFER
		stream = -1;  // invalidated
#endif
		start_it913x_spi(true);
	} else {
		if (first_in) {
			last_three_sec = *DWT_CYCCNT;
			//first_in = false;
		}
		else if(reset_ts_count_error) {
			uint32_t cur_time;
			 cur_time = *DWT_CYCCNT;
			int dur;
			 dur= timedelta(cur_time, last_three_sec);
			if ((3*120000000)<dur) {
				// assume video dem/dec get stable after 3 sec, liyenho
				reset_ts_count_error = false;
			}
		}
	}
#endif //RX_SPI_CHAINING
#ifdef TIME_ANT_SW
	extern uint32_t startup_video_tm,
										last_done_spi;
	extern bool startup_meas;
	int64_t hold_off_tm ;
		if (startup_video_tm && last_done_spi<startup_video_tm) {
			hold_off_tm = 0x100000000LL+(int64_t)last_done_spi;
		} else
		hold_off_tm = (int64_t)last_done_spi;
		if (startup_video_tm &&
			60000000LL/*30 sec*/<hold_off_tm-(int64_t)startup_video_tm) {
			startup_meas = true;
		}
#endif
_reg_acs:
  		if (usb_host_msg && !system_main_restart) {	// host ctrl/sts link with fpga/sms process, liyenho
			usb_host_msg = false;
//#define TEST_FLASH
 #ifdef TEST_FLASH
	uint32_t shf, wtmp;
 #endif
#if defined(CONFIG_RF2072)
			dev_access *pr=(dev_access*)gs_uc_hrbuffer1, *pt = (dev_access*)gs_uc_htbuffer;
			uint16_t tmp, tmpw, *pth = &tmp;
			uint8_t *pdbg = pr->data;  // to watch data content
		#include <assert.h>
			switch (pt->access) {
				case RF2072_RESET:
						pio_clear (PIOA, PIO_PA16);
						delay_ms(200);
						pio_set (PIOA, PIO_PA16);
						delay_us(1);
						pio_clear (PIOA, PIO_PA17); // keep enbl low
#if false  // do not set relock bit prior to program 2072...
							FLUSH_2072_SPI
							ACCESS_PROLOG_2072
							WRITE_2072_SPI(0x9, 0x0, 0x8)
#endif
						break;
				case RF2072_READ:
						assert(!(pt->dcnt & 1));
						 FLUSH_2072_SPI
							ACCESS_PROLOG_2072
							READ_2072_SPI(pt->addr,pr->data[1],pr->data[0])
#ifdef TEST_FLASH
	wtmp = *(uint32_t*)(~0x3&(ul_page_addr_c+pt->addr));
	shf = ((ul_page_addr_c+pt->addr) & 0x3) * 8;
  *(uint8_t*)pr->data = 0xff & (wtmp >> shf);
#endif
							READ_END_REV_2072
							break;
				case RF2072_WRITE:
						assert(!(pt->dcnt & 1));
						 FLUSH_2072_SPI
							ACCESS_PROLOG_2072
							WRITE_2072_SPI(pt->addr,pt->data[1],pt->data[0])
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
#ifdef VIDEO_DUAL_BUFFER
								stream = -1;  // invalidated
#endif
								// now allow video spi to be active, liyenho
								spi_enable(SPI_SLAVE_BASE);
							#ifndef RX_SPI_CHAINING
								pio_set(PIOB, PIO_PB9); // enable TS gate
							#else
					static bool spi_vid_restart = false;
								// pull over to here instead init inside main loop to get rid of time constraint, liyenho
								start_it913x_spi(spi_vid_restart);
								spi_vid_restart = true;
							#endif
							 #ifdef TIME_ANT_SW
							 	startup_video_tm = *DWT_CYCCNT;
							 #endif
								first_in = false;  // now it is safe to time vid ant sw
							#endif
							break;
				default: /* host msg in error */
							break;
			}
#endif //CONFIG_RF2072
  		}
#if 0
		sleepmgr_enter_sleep();
#endif
		if (gl_vid_ant_sw){
			Queue_Msg_Vid_Ant_Switch();
		}
		uint32_t channel_change_tick = 0;
		if ((hop_state == PAIRED) && !control_channel_scan_complete && !USE_915MHZ){
			//for 869 MHz, if pairing is complete, scan to select best channel
			control_channel_scan_complete = Channel_Scan();
			if (control_channel_scan_complete){}
		}
																																				// be sure video service has begun, liyenho
		if (control_channel_scan_complete && !control_channge_change_ack && false==first_in){
			//send request to change message until acknowledgment received
			Queue_Channel_Change_Message();
		}


		//disable for testing purposes
		//check to see if FEC should be requested
		//if ((FRR_Copy[0] < FEC_ON_RSSI_THRESHOLD) && !Requested_FEC_On){
			//Requested_FEC_On = Queue_FEC_Request_On_Packet();
		//}else if ((FRR_Copy[0] > FEC_OFF_RSSI_THRESHOLD) && Requested_FEC_On){
			//if (Queue_FEC_Request_Off_Packet()){
				//Requested_FEC_On = false;
			//}
		//}
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

volatile bool main_usb_host_msg(void) // Attach this api to usb rx isr service, liyenho
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
	if (USB_HOST_MSG_TX_VAL == udd_g_ctrlreq.req.wValue &&
			0!=ctx_913x.it913x_access &&	(!ctx_913x.i2c_done_rd || !ctx_913x.i2c_done_wr))
		{
			udd_set_setup_payload( gs_uc_htbuffer_tm, udd_g_ctrlreq.req.wLength);
			ctx_913x.i2c_overrun = true; // can't tell which type of dev_access yet....
			return true; // don't interfere with cur task
		}
	else
		udd_set_setup_payload( gs_uc_htbuffer, udd_g_ctrlreq.req.wLength);
	udd_g_ctrlreq.callback = host_usb_cb;
	return true;
}

volatile bool main_usb_host_reply(void)
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
		dev_access *ps = (ctx_913x.i2c_overrun)?gs_uc_htbuffer_tm:gs_uc_hrbuffer1;
	if (USB_HOST_MSG_LEN-sizeof(ps->data[0]) == udd_g_ctrlreq.req.wLength)
		// (0==ps->dcnt) meant confirmation of write cmd
		cnf_echo = 1;
		// must setup packet size
		if (!cnf_echo) {
			udd_set_setup_payload( gs_uc_hrbuffer1,USB_HOST_MSG_LEN+(ps->dcnt-1)*sizeof(uint8_t));
			if(ps->access == IT913X_READ) {
				if (!ctx_913x.i2c_overrun && IT913X_READ==ctx_913x.it913x_access) {
				  int tmp_cnt = ps->dcnt - TWI_ERR_PROT_RD;
					udd_set_setup_payload( gs_uc_hrbuffer1,
																		USB_HOST_MSG_LEN+(tmp_cnt-1)*sizeof(uint8_t));
				}
				if(ctx_913x.i2c_done_rd == false) {
					((dev_access*)gs_uc_hrbuffer1)->error= -128;
				}
				else if (ctx_913x.it913x_err_rd !=TWI_SUCCESS) {
					((dev_access*)gs_uc_hrbuffer1)->error= -127;
					ctx_913x.it913x_access = 0; // invalidate for next access
				}
	        else if(ctx_913x.it913x_access !=IT913X_READ) {
	          //error: status request, when read request not yet initiated.
	          ((dev_access*)gs_uc_hrbuffer1)->error=-126;
	        }
				else {
					((dev_access*)gs_uc_hrbuffer1)->error= 0;
   	       ctx_913x.it913x_access = 0; // invalidate for next access
       	  }
			}
		}
		else //cnf_echo == 1
		{
			udd_set_setup_payload( gs_uc_hrbuffer1,
				USB_HOST_MSG_LEN-sizeof(ps->data[0]));
			if(ps->access == IT913X_WRITE) {
				if(ctx_913x.i2c_done_wr == false) {
					((dev_access*)gs_uc_hrbuffer1)->error= -128;
				}
				else if (ctx_913x.it913x_err_wr !=TWI_SUCCESS) {
					((dev_access*)gs_uc_hrbuffer1)->error= -127;
					ctx_913x.it913x_access = 0; // invalidate for next access
				}
				else if(ctx_913x.it913x_access !=IT913X_WRITE) {
					//error: status request, when write request not yet initiated.
					((dev_access*)gs_uc_hrbuffer1)->error=-126;
				}
				else {
					((dev_access*)gs_uc_hrbuffer1)->error= 0;
					ctx_913x.it913x_access = 0; // invalidate for next access
				}
			}
		}
	ctx_913x.i2c_overrun = false;
	/* no need to setup callback in READ case */
	return (true != spi_tgt_done);
}

// to enable embedded ITE asic  video subsystem ********************
extern int init_video_subsystem(void);
extern int start_video_subsystem(void);
/**********************************************************/
volatile bool main_vender_specific(void) {
 #ifdef CONFIG_ON_FLASH
	if (USB_BOOT_APP_VAL == udd_g_ctrlreq.req.wValue) {
		udd_set_setup_payload( &bootapp, sizeof(bootapp));
		udd_g_ctrlreq.callback = host_usb_cb;
	}
 #endif
	if (USB_SYSTEM_RESTART_VAL == udd_g_ctrlreq.req.wValue) {
		main_loop_restart(); return true;
	}
/**********************************************************/
	if (USB_INIT_VID_SUBSYS == udd_g_ctrlreq.req.wValue) {
		if (!init_video_subsystem()) return true;
		else return false ;
	}
	if (USB_START_VID_SUBSYS == udd_g_ctrlreq.req.wValue) {
		if (!start_video_subsystem()) {
			 first_in = false;  // now it is safe to time vid ant sw
			return true;
		}
		else return false ;
	}
/**********************************************************/
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
	 else if (USB_ANT_SW_VAL == udd_g_ctrlreq.req.wValue) {
			gl_vid_ant_sw = true; // try to switch antenna
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
	else if (VIDEO_SETVCH_VAL == udd_g_ctrlreq.req.wValue) {
		udd_set_setup_payload( &vch, sizeof(vch));
		udd_g_ctrlreq.callback = host_usb_cb;
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

static int timedelta0(bool *reset, unsigned int bignum, unsigned int smallnum) {
	static int64_t bprev, sprev;
	int64_t bl, sl, delta;
	if (reset) {
		bprev = sprev = 0L; // chances to collide with 0 are less than twice being by lightning in a day
		*reset = false;
	}
#define EXTEND64(o, i, i0) o = (i < i0) ? 0x100000000+i : i;
	EXTEND64(bl, bignum, bprev)
	EXTEND64(sl, smallnum, sprev)
	delta = bl - sl;	// delta shall be of 32 bit range,
#undef EXTEND64
	bprev = bignum;
	sprev = smallnum;
	return (int)delta ;
}

int timedelta(unsigned int bignum, unsigned int smallnum) {
	int64_t bl=bignum, sl=smallnum, delta;
	delta = bl - sl;	// delta shall be of 32 bit range,
	return (int)delta ;
}
