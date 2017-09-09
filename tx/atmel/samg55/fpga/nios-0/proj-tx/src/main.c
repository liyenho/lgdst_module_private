/**
 * \file
 *
 * \brief CDC Application Main functions
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
#if (SAMG55)
#include "flexcom.h"
#endif
#include "Serialize.h"  // for fpga upgrade api
static volatile bool system_main_restart = false;  // system restart flag, liyenho
static volatile uint8_t system_upgrade = 0;  // system upgrade flag, liyenho
/*static*/ volatile char __version_atmel__[3];  // stored as mon/day/year
#define INCLUDEINMAIN  // for Kevin's ctrl design
#include "bsp.h"
#include "lgdst_4463_spi.h"
#ifdef RADIO_SI4463 // not used with Kevin's design
 #include "si446x_nirq.h"
 #include "radio.h"
#endif
#include "ctrl.h"
#include "Radio_Buffers.h"
#include "USB_Commands.h"
#include "ReedSolomon.h"
#include "Antenna_Diversity.h"
  #define SPI0_Handler     FLEXCOM0_Handler // spi0 -> spi1 on tx
  #define SPI_Handler     FLEXCOM1_Handler  // spi5 -> spi1 on tx
  #define SPI0_IRQn        FLEXCOM0_IRQn
  #define SPI_IRQn        FLEXCOM1_IRQn
// extra SPI port for additional data chip on Tx side, not configured yet, liyenho
#define SPI2_Handler     FLEXCOM2_Handler
#define SPI2_IRQn        FLEXCOM2_IRQn
// extra SPI port for additional data chip on Rx side, not configured yet, liyenho
#define SPI7_Handler     FLEXCOM7_Handler
#define SPI7_IRQn        FLEXCOM7_IRQn
volatile uint8_t main_loop_on = false;  // run time indicator
volatile uint8_t usb_data_done = false;  // workaround flooding i2s intr issue, liyenho
volatile uint8_t usb_host_msg = false, spi_tgt_done=false; // triggered by usb rx isr for host (fpga/sms) comm, liyenho
//engineering debug support
volatile uint32_t *DWT_CONTROL = (uint32_t *)0xE0001000;
volatile uint32_t *DWT_CYCCNT = (uint32_t *)0xE0001004;
volatile uint32_t *DEMCR = (uint32_t *)0xE000EDFC;

#ifdef RADIO_SI4463
  volatile uint8_t spi_radio_done = false;
  volatile uint8_t spi_dma_mode = false;
#endif
static volatile bool main_b_cdc_enable = false;
/* Chip select. */
#define SPI_CHIP_SEL 0
/* Clock polarity. */
#define SPI_CLK_POLARITY /*0*/ 1
/* Clock phase. */
#define SPI_CLK_PHASE 0
/* Delay before SPCK. */
#define SPI_DLYBS 0x40 /*0x10*/ // accommodate low cost fpga with sluggish perf, liyenho
/* Delay between consecutive transfers. */
#define SPI_DLYBCT 0x10 /*0x0*/ // accommodate low cost fpga with sluggish perf, liyenho
 // 10 TS packet per ping/pong buffer
//#define I2SC_BUFFER_SIZE		10*188
/* UART baudrate. */
#define UART_BAUDRATE      115200
/* SPI clock setting (Hz). */
  /*static*/ uint32_t gs_ul_spi_clock[2+1+1] = {  // shall be higher than usb data rate
  				  	2000000, 8000000, RADIO_SPI_BR, ADI936x_SPI_BR}; // (fpga/sms, video, radio, dvbt-rf) spi bit rate
extern volatile bool udi_cdc_data_running; // from udi_cdc.c, liyenho
#ifdef  RADIO_SI4463
  volatile bool si4463_radio_started = false; // radio startup flag...
  volatile bool ctrl_tdma_enable = false;
/*******************************************************************/
  volatile bool ctrl_tdma_lock = false;
  volatile bool ctrl_tdma_rxactive = false;  //block rtt_handle startTx() call
	static int timedelta(bool reset, unsigned int bignum, unsigned int smallnum);
	/*static*/ bool timedelta_reset, timedelta_reset_rx; //to handle system restart
	volatile bool fhop_in_search= false, fhop_flag, fhop_dir;
  unsigned int tdma_sndthr=0;


  /*static*/ enum pair_mode hop_state;
  	volatile unsigned char ynsdbyte ; //Take care in Rate Control Section!!! liyenho
#endif
volatile bool stream_flag = true; // default to TS stream automatic on
#ifdef CONFIG_ON_FLASH
uint32_t ul_page_addr_ctune =IFLASH_ADDR + IFLASH_SIZE - NUM_OF_ATMEL_REGS, // 1st atmel reg on flash
					// temperature @ current tuning @ 2nd atmel reg on flash
				ul_page_addr_mtemp =IFLASH_ADDR + IFLASH_SIZE - NUM_OF_ATMEL_REGS + 1,
				ul_page_addr_bootapp =IFLASH_ADDR + IFLASH_SIZE - NUM_OF_ATMEL_REGS + 2,
				ul_page_addr_fpgadef =IFLASH_ADDR + IFLASH_SIZE - NUM_OF_ATMEL_REGS + 3;
uint8_t backup[NUM_OF_FPGA_REGS+NUM_OF_ATMEL_REGS+4+1],
				bootapp = (uint8_t)/*-1*/0,
				fpgadef = (uint8_t)-1;
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
volatile static uint32_t g_ul_led_ticks=0, g_ul_wait_100ms=50, g_ul_wait_1s=100;
static bool health_led_onoff = false;
volatile bool usb_write_start = false ; // called back from udc.c, liyenho
	uint32_t gs_uc_rbuffer[2*I2SC_BUFFER_SIZE/sizeof(int)];
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
uint32_t gs_uc_hrbuffer[(USB_HOST_MSG_LEN+HOST_BUFFER_SIZE+3)/sizeof(int)],
					gs_uc_hrbuffer1[(USB_HOST_MSG_LEN+HOST_BUFFER_SIZE+3)/sizeof(int)];
/* Pointer to UART PDC register base */
Pdc *g_p_spim_pdc [1+2+1]/*fpga/sms, video, radio ctrl/sts, dvbt-rf (ad9364), liyenho*/,*g_p_spis_pdc;
Pdc *g_p_i2st_pdc, *g_p_i2sr_pdc;
struct i2s_dev_inst dev_inst_i2s;
#ifdef RADIO_SI4463
volatile uint32_t tpacket_idle[RDO_ELEMENT_SIZE];
unsigned char gs_rdo_tpacket_ovflw=0;
volatile uint32_t rpacket_idle[ASYMM_RATIO* RDO_ELEMENT_SIZE];
uint32_t rpacket_ov[RDO_ELEMENT_SIZE]; //gs_rdo_rpacket fifo overflow holder
unsigned char rpacket_grp[RADIO_GRPPKT_LEN+RADIO_INFO_LEN];  // fixed
unsigned char gs_rdo_rpacket_ovflw=0;

uint32_t radio_mon_txfailcnt=0;
uint32_t radio_mon_rxcnt = 0;
uint32_t radio_mon_rxerr = 0;
uint32_t  radio_mon_txcnt = 0;
uint32_t rxnorec_intv=0;  // not a count but interval

unsigned char snd_asymm_cnt=0;
static uint32_t tick_curr, tick_prev;

bool TX_Active = false;
  // 4463 stats mon obj
  extern volatile ctrl_radio_stats  r4463_sts;
#endif //RADIO_SI4463

#ifdef RFFE_PARAMS
 #ifndef MEDIA_ON_FLASH
	static rf_params TX_Rf_Params;
 #else // pre-define rf ch/att
	static rf_params TX_Rf_Params =
		{ .params_tx.chan_idx = 0/*ch*/,
			.params_tx.pwr_att = 10000/*att*/};
 #endif
	volatile uint32_t nios_done = 0;
	volatile bool set_rf_params= false;
#endif
/**
 *  \brief Configure the Console UART.
 */

// function prototyping
int hop_chn_sel(int offset) ;

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
		uint32_t rsize, uint32_t ch);
/*static*/ void spi_rx_transfer(void *p_tbuf, uint32_t tsize, void *p_rbuf,
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
 	if (2==ch) base = SPI2_MASTER_BASE;
 #endif
	spi_enable_interrupt(base, spi_ier) ;
}

 void SPI0_Handler(void) // fpga ctrl spi
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
	 void SPI2_Handler(void) // si4463 ctrl/sts/data spi
	 {
		uint32_t status;
		spi_disable_interrupt(SPI2_MASTER_BASE, SPI_IER_TXBUFE) ;
		spi_disable_interrupt(SPI2_MASTER_BASE, SPI_IER_RXBUFF) ;

		spi_radio_done = false;  // handshake with main loop
		status = spi_read_status(SPI2_MASTER_BASE) ;

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
	spi_disable_interrupt(SPI_MASTER_BASE, SPI_IER_TXBUFE) ;
 #ifdef TEST_SPI
	spi_disable_interrupt(SPI_MASTER_BASE, SPI_IER_RXBUFF) ;
 #endif
	usb_data_done = false;  // handshake with mainloop
	status = spi_read_status(SPI_MASTER_BASE) ;

	//if(status & SPI_SR_NSSR) {
		if ( status & SPI_SR_TXBUFE ) {
//			printf("transfer done\n");
		}
	//}
}
/**
 *  \brief Handler for System Tick interrupt.
 *
 *  Process System Tick Event
 *  increments the timestamp counter.
 */

/**
 * \brief Initialize SPI as slave.
 */
/**
 * \brief Initialize SPI as master.
 */
#ifdef SPI0_FOR_SPI5
 #define ioport_set_pin_peripheral_mode(pin, mode) \
	 do {\
		 ioport_set_pin_mode(pin, mode);\
		 ioport_disable_pin(pin);\
	 } while (0)
/*static*/ void spi0_set_peripheral() { // all related pins are assigned back to peripheral mode
	pio_clear(PIOA, PIO_PA24);  // revert spi0 for regular usage
	ioport_set_pin_peripheral_mode(SPI0_MISO_GPIO, SPI0_MISO_FLAGS);
	ioport_set_pin_peripheral_mode(SPI0_MOSI_GPIO, SPI0_MOSI_FLAGS);
	ioport_set_pin_peripheral_mode(SPI0_SPCK_GPIO, SPI0_SPCK_FLAGS);
	ioport_set_pin_peripheral_mode(SPI0_NPCS0_GPIO, SPI0_NPCS0_FLAGS);
	ioport_set_pin_peripheral_mode(SPI0_NPCS1_GPIO, SPI0_NPCS1_FLAGS);
}
#endif
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
	else // ctrl/sts/video/dvbt-rf
 #endif
 #ifdef NOIS_EXTRACTED
	if (3/*ctrl to ad9364*/ == ch) {
	  spi_set_clock_polarity(base, SPI_CHIP_SEL, 0/*clk idle state is low*/);
	  spi_set_clock_phase(base, SPI_CHIP_SEL, 0/*transit @ rising, captured @ falling*/); }
	else // ctrl/sts/video
 #endif
	{  spi_set_clock_polarity(base, SPI_CHIP_SEL, SPI_CLK_POLARITY);
	spi_set_clock_phase(base, SPI_CHIP_SEL, SPI_CLK_PHASE); }
 #ifdef RADIO_SI4463
	if (2/*data from si4463*/ == ch)
	  spi_set_bits_per_transfer(base, SPI_CHIP_SEL,
			SPI_CSR_BITS_8_BIT);  // 8 bit spi xfer, liyenho
	else // ctrl/sts/video/dvbt-rf
 #endif
 #ifdef NOIS_EXTRACTED
	if (3/*ctrl to ad9364*/ == ch)
	  spi_set_bits_per_transfer(base, SPI_CHIP_SEL,
			SPI_CSR_BITS_8_BIT);  // 8 bit spi xfer, liyenho
	else // ctrl/sts/video/
 #endif
	  spi_set_bits_per_transfer(base, SPI_CHIP_SEL,
			SPI_CSR_BITS_16_BIT);  // either 8 or 16 bit spi xfer, liyenho
	spi_set_baudrate_div(base, SPI_CHIP_SEL,
			(sysclk_get_cpu_hz() / gs_ul_spi_clock [ch]));
#ifdef RADIO_SI4463
	if (2/*data from si4463*/ == ch)
	  spi_set_transfer_delay(base, SPI_CHIP_SEL, 0x10/*delay between bytes*/,
			0x10/*delay between spi xfer*/);
	else // ctrl/sts and video
#endif
#ifdef NOIS_EXTRACTED
	if (3/*ctrl to ad9364*/ == ch)
	  spi_set_transfer_delay(base, SPI_CHIP_SEL,
	  		0/*delay between bytes, 2 clk cycles*/,
			2*sysclk_get_cpu_hz() / (gs_ul_spi_clock [ch]*32)/*delay between spi xfer, 3 clk cycles*/);
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
 	if (2==ch) base = SPI2_MASTER_BASE;
 #endif
	spi_enable_interrupt(base, spi_ier) ;
}

//#ifdef SMS_DVBT2_DOWNLOAD
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
//#endif

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
		if (USB_FPGA_DEF_VAL == udd_g_ctrlreq.req.wValue) {
			erase_last_sector() ;
			CHECKED_FLASH_WR(
				IFLASH_ADDR + IFLASH_SIZE-sizeof(backup),
				backup, NUM_OF_FPGA_REGS +1 +4 +3)
			CHECKED_FLASH_WR(ul_page_addr_fpgadef, &fpgadef, 1)
			CHECKED_FLASH_WR(
				IFLASH_ADDR + IFLASH_SIZE-NUM_OF_ATMEL_REGS +4,
				backup +NUM_OF_FPGA_REGS +1 +4 +4,
				sizeof(backup)-NUM_OF_FPGA_REGS-1 -4 -4)
			return ;
		}
		if ((USB_FPGA_UPGRADE_VAL == udd_g_ctrlreq.req.wValue ||
						 USB_ATMEL_UPGRADE_VAL == udd_g_ctrlreq.req.wValue) &&
						FW_UPGRADE_HDR_LEN >=udd_g_ctrlreq.payload_size) {
			memcpy(upgrade_fw_hdr, // set fpga fw /img hdr, liyenho
								udd_g_ctrlreq.payload,
								FW_UPGRADE_HDR_LEN);
			system_upgrade = (USB_FPGA_UPGRADE_VAL == udd_g_ctrlreq.req.wValue)?1/*fpga*/:2/*atmel*/;
			main_loop_on = false ; // clear up the 'upgrade flag' ;-)
			return;
		}
		dev_access *ps = gs_uc_htbuffer,*ps1 = gs_uc_hrbuffer1;
		// prepare confirm msg by echo whatever received
		memcpy(gs_uc_hrbuffer1, udd_g_ctrlreq.payload, USB_HOST_MSG_LEN-sizeof(ps->data[0]));
		usb_host_msg = true; // enable mainloop process
  }

#ifdef RADIO_SI4463
/*****************************************************************************
 *  si4464 definitions and Global Variables
 *****************************************************************************/
/*const*/ COMPILER_ALIGNED(8) tRadioConfiguration RadioConfiguration_915 = RADIO_CONFIGURATION_DATA_915;
/*const*/ COMPILER_ALIGNED(8) tRadioConfiguration RadioConfiguration_869 = RADIO_CONFIGURATION_DATA_869;
/*const*/ tRadioConfiguration *pRadioConfiguration = &RadioConfiguration_915;
	SEGMENT_VARIABLE(bMain_IT_Status, U8, SEG_XDATA);

extern uint8_t get_si446x_temp();
extern void recalibrate_capval (void* ul_page_addr_mtemp, uint8_t median);
/*****************************************************************************/
#endif
void SysTick_Handler(void)
{
  //2ms interrupt routine
  g_ul_led_ticks ++;
 #ifdef RADIO_SI4463 // RADIO_SI4463 inside SysTick_Handler

  if (si4463_radio_started) {
  // ctrl tdma management ------------------------------------------------------------
  	if(ctrl_tdma_enable) {
	  	// local declarations ------------------------------
		unsigned int lc;
	  	int cdelta,rxdelta,tdmadelta;

	  	//clock adjustment ---------------------------------
	  	lc = *DWT_CYCCNT;  //not working when in "no-debug" mode
	  	cdelta = timedelta(timedelta_reset, tdma_sndthr, lc);
	  	timedelta_reset = false;
	  	if((cdelta>TDMA_BOUND)||(cdelta<-TDMA_BOUND)||
	  		((false==ctrl_tdma_lock) && (false == fhop_flag))){
		  	//restart									// allow frequ hop take action during search
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
		  	rdptr_inc(&wrptr_rdo_tpacket, &rdptr_rdo_tpacket, RDO_TPACKET_FIFO_SIZE, 1);
		  gp_rdo_tpacket_l = gs_rdo_tpacket + (RDO_ELEMENT_SIZE*rdptr_rdo_tpacket);
		  	snd_asymm_cnt = ASYMM_RATIO-1;


	  if((ctrl_tdma_rxactive==false)&&
	     (hop_state != IDLE)) {//bypass send if main loop receive active (flywheel collision happenned)
 		  	  vRadio_StartTx(pRadioConfiguration->Radio_ChannelNumber, gp_rdo_tpacket_l,
	  		RADIO_PKT_LEN);
			radio_mon_txcnt++;
	  }else
	    radio_mon_txfailcnt++;
	  	// frequency hopping in the action, rec freq is superseded by this frequency too, liyenho
	    ctrl_hop_global_update(false);

	  tdma_sndthr = tdma_sndthr + TDMA_PERIOD;
		fhop_flag = false ;
	  	}
  	}
		if (SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_PACKET_SENT_PEND_BIT == bMain_IT_Status) {
			//just sent a packet, //8555226=71.3ms
      #ifdef RADIO_CTRL_TXCONTINOUS
			flags = cpu_irq_save();
		      bMain_IT_Status = 0;  // reset nirq flag
			cpu_irq_restore(flags);
        vRadio_StartTx(pRadioConfiguration->Radio_ChannelNumber, gp_rdo_tpacket_l,
	   		   RADIO_PKT_LEN); //pRadioConfiguration->Radio_PacketLength);
      #endif
		}
  	//periodic data send
	#ifdef RADIO_CTRL_TXCONTINOUS
	  ctrl_tdma_enable =false;
	#endif

  } //si4463_radio_started

#endif // RADIO_SI4463 inside SysTick_Handler
blink_led:
	if (prev_usbfrm != usbfrm && g_ul_wait_100ms<=g_ul_led_ticks) {
		  if (health_led_onoff) // on
 #ifndef RADIO_SI4463
			 pio_set(PIOA, HEALTH_LED);
 #else // data radio present
			 pio_set(PIOB, HEALTH_LED);
 #endif
		  else // off
 #ifndef RADIO_SI4463
			 pio_clear(PIOA, HEALTH_LED);
 #else // data radio present
			 pio_clear(PIOB, HEALTH_LED);
 #endif
		  health_led_onoff ^= 1;
		  g_ul_led_ticks = 0;
	}
	else if (g_ul_wait_1s<=g_ul_led_ticks) {
		  if (health_led_onoff) // on
 #ifndef RADIO_SI4463
			 pio_set(PIOA, HEALTH_LED);
 #else // data radio present
 			 pio_set(PIOB, HEALTH_LED);
 #endif
		  else // off
 #ifndef RADIO_SI4463
			 pio_clear(PIOA, HEALTH_LED);
 #else // data radio present
			 pio_clear(PIOB, HEALTH_LED);
 #endif
		  health_led_onoff ^= 1;
		g_ul_led_ticks = 0;
	}
	prev_usbfrm = usbfrm;
}
#ifdef RFFE_PARAMS
static void set_frequency(uint32_t reg78, uint32_t reg79) {
	uint32_t tmp, wtmp, *pth = &tmp;
	dev_spi_wr(78, wtmp, 1, reg78)
	dev_spi_wr(79, wtmp, 1, reg79)
}
static void frequency_select() {
		uint32_t tmp, wtmp, *pth = &tmp;
		switch(TX_Rf_Params.params_tx.chan_idx) {
			case 1 : //freqency setting: 2403.25mhz: reg (78,79) (9613 = 0x25,0x8D)
							set_frequency(0x25, 0x8D); break;
			case 2 : //freqency setting: 2407.25mhz: reg (78,79) (9629 = 0x25,0x9D)
							set_frequency(0x25, 0x9D); break;
			case 3 : //freqency setting: 2411.25mhz: reg (78,79) (9645 = 0x25,0xAD)
							set_frequency(0x25, 0xAD); break;
			case 4 : //freqency setting: 2415.25mhz: reg (78,79) (9661 = 0x25,0xBD)
							set_frequency(0x25, 0xBD); break;
			case 5 : //freqency setting: 2419.25mhz: reg (78,79) (9677 = 0x25, 0xCD)
							set_frequency(0x25, 0xCD); break;
			case 6 : //freqency setting: 2423.25mhz: reg (78,79) (9693 = 0x25, 0xDD)
							set_frequency(0x25, 0xDD); break;
			case 7 : //freqency setting: 2427.25mhz: reg (78,79) (9709 = 0x25, 0xED)
							set_frequency(0x25, 0xED); break;
			case 8 : //freqency setting: 2431.25mhz: reg (78,79) (9725 = 0x25, 0xFD)
							set_frequency(0x25, 0xFD); break;
			case 9 : //freqency setting: 2435.25mhz: reg (78,79) (9741 = 0x26,0x0D)
							set_frequency(0x26, 0x0D); break;
			case 10 : //freqency setting: 2439.25mhz: reg (78,79) (9757 = 0x26,0x1D)
							set_frequency(0x26,0x1D); break;
			case 11 : //freqency setting: 2443.25mhz: reg (78,79) (9773 = 0x26,0x2D)
							set_frequency(0x26,0x2D); break;
			case 12 : //freqency setting: 2447.25mhz: reg (78,79) (9789 = 0x26,0x3D)
							set_frequency(0x26,0x3D); break;
			case 13 : //freqency setting: 2451.25mhz: reg (78,79) (9805 = 0x26,0x4D)
							set_frequency(0x26,0x4D); break;
			case 14 : //freqency setting: 2455.25mhz: reg (78,79) (9821 = 0x26,0x5D)
							set_frequency(0x26,0x5D); break;
			case 15 : //freqency setting: 2459.25mhz: reg (78,79) (9837 = 0x26,0x6D)
							set_frequency(0x26,0x6D); break;
			case 16 : //freqency setting: 2463.25mhz: reg (78,79) (9853 = 0x26,0x7D)
							set_frequency(0x26,0x7D); break;
			case 17 : //freqency setting: 2467.25mhz: reg (78,79) (9869 = 0x26,0x8D)
							set_frequency(0x26,0x8D); break;
			case 18 : //freqency setting: 2471.25mhz: reg (78,79) (9885 = 0x26,0x9D)
							set_frequency(0x26,0x9D); break;
			case 19 : //freqency setting: 2475.25mhz: reg (78,79) (9901 = 0x26,0xAD)
							set_frequency(0x26,0xAD); break;
			case 20 : //freqency setting: 2479.25mhz: reg (78,79) (9917 = 0x26,0xBD)
							set_frequency(0x26,0xBD); break;
			default: //freqency setting: 2392mhz: reg (78,79) = 0x09,0x58 (9568 = 0x25,0x60)
							set_frequency(0x25,0x60); break;
		}

		delay_s(1);
		dev_spi_wr(65, wtmp, 1, 0x40) // set check-in bit
		delay_s(1);

#ifdef NOIS_EXTRACTED
  #ifdef SPI0_FOR_SPI5
		spi0_set_peripheral();
		spi_master_initialize(0, SPI0_MASTER_BASE, BOARD_FLEXCOM_SPI0);// fpga ctrl pipe
		spi_configure_cs_behavior(SPI0_MASTER_BASE, 0, SPI_CS_RISE_NO_TX);
  #endif
		rf_dvbt_control();
  #ifdef SPI0_FOR_SPI5
		spi0_set_peripheral();
		spi_master_initialize(0, SPI0_MASTER_BASE, BOARD_FLEXCOM_SPI0);// fpga ctrl pipe
		spi_configure_cs_behavior(SPI0_MASTER_BASE, 0, SPI_CS_RISE_NO_TX);
  #endif
#endif
}
static void attenuation_select() {
		uint32_t tmp, wtmp, *pth = &tmp;
		  dev_spi_wr(72, wtmp, 1, (TX_Rf_Params.params_tx.pwr_att>>8))
		  dev_spi_wr(73, wtmp, 1, (TX_Rf_Params.params_tx.pwr_att))
		delay_s(1);
		  dev_spi_wr(65, wtmp, 1, 0x40) // set check-in bit
		delay_s(1);

#ifdef NOIS_EXTRACTED
  #ifdef SPI0_FOR_SPI5
		spi0_set_peripheral();
		spi_master_initialize(0, SPI0_MASTER_BASE, BOARD_FLEXCOM_SPI0);// fpga ctrl pipe
		spi_configure_cs_behavior(SPI0_MASTER_BASE, 0, SPI_CS_RISE_NO_TX);
  #endif
		rf_dvbt_control();
  #ifdef SPI0_FOR_SPI5
		spi0_set_peripheral();
		spi_master_initialize(0, SPI0_MASTER_BASE, BOARD_FLEXCOM_SPI0);// fpga ctrl pipe
		spi_configure_cs_behavior(SPI0_MASTER_BASE, 0, SPI_CS_RISE_NO_TX);
  #endif
#endif
}
 static void rf_params_cb() {
	memcpy(&TX_Rf_Params.params_tx,
						udd_g_ctrlreq.payload,
						sizeof(TX_Rf_Params.params_tx));
	if (set_rf_params) {
	 	if (RF_TX_FREQ_VAL == udd_g_ctrlreq.req.wValue) {
		 	frequency_select();
	 	}
	 	else if (RF_TX_ATTN_VAL == udd_g_ctrlreq.req.wValue) {
		 	attenuation_select();
	 	}
		return;
	}
	set_rf_params = true;
 }
#endif
void init_4463()
 {
	 if (RF4463_TX_MODE)
	 {
	 	pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PA5 | PIO_PA6);
	 	pio_set_peripheral(PIOA, PIO_PERIPH_B, PIO_PA15);
	 	spi_master_initialize(2, SI4463_SPI, BOARD_FLEXCOM_SPI2);
	 	spi_set_clock_polarity(SI4463_SPI, SPI_CHIP_SEL, 0 /*SPI_CLK_POLARITY*/);
	 	spi_set_clock_phase(SI4463_SPI, SPI_CHIP_SEL, 1 /*SPI_CLK_PHASE*/);
	 	spi_configure_cs_behavior(SI4463_SPI, 0, SPI_CS_KEEP_LOW);

	 	pio_configure(PIOA, PIO_INPUT, PIO_PA29, 0);    //RF_NIRQ
		//pio_configure(PIOB, PIO_INPUT, PIO_PB3, 0);    //RF_GPI00
		//pio_configure(PIOB, PIO_INPUT, PIO_PB2, 0);    //RF_GPI01
	 	pio_configure(PIOA, PIO_OUTPUT_1, PIO_PA30, 0); //RF_PWRDN
	 	pio_configure(PIOA, PIO_OUTPUT_1, PIO_PA16, 0); //RF_NSEL

		/* Initialize SI4463 Host_Int line */ // for time critical TDM process
	 pio_set_input(PIOA, SI4463_HOST_INT, PIO_PULLUP);
	  pio_handler_set(PIOA, ID_PIOA, SI4463_HOST_INT,
	 			PIO_IT_AIME /*| PIO_IT_RE_OR_HL*/ | PIO_IT_EDGE, si4463_radio_handler);
	  pio_enable_interrupt(PIOA, SI4463_HOST_INT);
	 pio_handler_set_priority(PIOA, PIOA_IRQn, 1/*long latency event*/);
	 } else
	 {
		pio_set_peripheral(PIOA, PIO_PERIPH_B, PIO_PA27 | PIO_PA28 | PIO_PA29);
		spi_master_initialize(2, SI4463_SPI, BOARD_FLEXCOM_SPI7);
		spi_set_clock_polarity(SI4463_SPI, SPI_CHIP_SEL, 1 /*SPI_CLK_POLARITY*/);
		spi_set_clock_phase(SI4463_SPI, SPI_CHIP_SEL, 0 /*SPI_CLK_PHASE*/);
		spi_configure_cs_behavior(SI4463_SPI, 0, SPI_CS_KEEP_LOW);

		pio_configure(PIOB, PIO_INPUT, PIO_PB8, 0);    //RF_NIRQ
	 	//pio_configure(PIOB, PIO_INPUT, PIO_PB13, 0);    //RF_GPI00
	 	//pio_configure(PIOA, PIO_INPUT, PIO_PA18, 0);    //RF_GPI01
		pio_configure(PIOA, PIO_OUTPUT_1, PIO_PA15, 0); //RF_PWRDN
		pio_configure(PIOA, PIO_OUTPUT_1, PIO_PA30, 0); //RF_NSEL
	 }

	 radio_mon_rxcnt =0;
	 delay_ms(50); // Extra waiting for SPI7 Master Ready

 	 return;
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
				uint32_t tmp, wtmp, *pth = &tmp;
			if (1 == system_upgrade) {
				// altera upgrade
/*****************************************************/
				//enable reconfig
				dev_spi_wr(12, wtmp, 1, 0x1) // reg12 <- 0x1
				delay_s(2);
/*****************************************************/
				pio_set(PIOA, PIO_PA0);
				uint32_t fw_addr = upgrade_fw_hdr[2];
				download_fpga_fw( upgrade_fw_hdr );
				pio_clear(PIOA, PIO_PA0);
/*****************************************************/
				//disable reconfig
				delay_s(1);
				dev_spi_wr(12, wtmp, 1, 0x0) // reg12 <- 0x0
/*****************************************************/
				delay_ms(/*100*/5000);
/*****************************************************/
			dev_spi_wr(98, wtmp, 1, 0x1) //reg98 <- 0x1
/*****************************************************/
				/* stop usb device operation */
				udc_stop();
				/* run application */
				_app_exec(APP_START);
			}
			else if (2 == system_upgrade) {
				// atmel upgrade
//				download_atmel_fw( upgrade_fw_hdr );
				wdt_init(WDT, WDT_MR_WDRPROC, 256/*1 sec*/, 0xfff) ;
				while (1); // wait for processor reset
			}
			else if (3 == system_upgrade) {
				// fpga switch to app image
/*****************************************************/
				//enable reconfig
				dev_spi_wr(12, wtmp, 1, 0x1) // reg12 <- 0x1
/*****************************************************/
				// stop usb device operation
				udc_stop();
				// run application
				_app_exec(APP_START);
/*****************************************************/
			}
	}
/*! \brief Main function. Execution starts here.
 */
int main(void)
{
	volatile uint8_t *pusbe=(uint8_t*)gs_uc_tbuffer+sizeof(gs_uc_tbuffer),
										*pusbs=(uint8_t*)gs_uc_tbuffer+0,
#ifdef CONF_BOARD_USB_TX
										*pusb=gs_uc_rbuffer;
#elif defined(CONF_BOARD_USB_RX)
										*pusb=gs_uc_tbuffer;
#endif
	uint32_t tdel, tcurr, n;
#if defined(MEDIA_ON_FLASH) || defined(CONFIG_ON_FLASH) || defined(FWM_DNLD_DBG)
	uint32_t page_addr_bypass = IFLASH_ADDR + IFLASH_SIZE -  // to store flash media download flag, liyenho
												(NUM_OF_FPGA_REGS+ NUM_OF_ATMEL_REGS) -4/*sizeof(int)*/-1 ;
	volatile uint32_t le, lm, ul_rc;
	volatile uint32_t ul_page_end, ul_page_addr=PAGE_ADDRESS ;
	const uint32_t erlen = SECTOR_SIZE_L; // last two sector size must be 128 kbytes
		// reset page address to top region for config registry, liyenho
	uint32_t ul_page_addr_c= IFLASH_ADDR + IFLASH_SIZE -
														(NUM_OF_FPGA_REGS+ NUM_OF_ATMEL_REGS);
	//control path related
	U8 sndflag;
	U8 radioini_failcnt=0;
	uint16_t ledoncnt;

#elif defined(FWM_DNLD_DBG)
	volatile uint32_t ul_page_end, ul_page_addr=PAGE_ADDRESS ;
#endif
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
	delay_ms(10); //atmelStudio fail debug breakpoint
	ui_powerdown();
#if 0 /*defined(RADIO_SI4463)*/
	/* Initialize the console UART. */
	configure_console(); // used for si446x radio dev, liyenho
#endif
	// Start USB stack to authorize VBus monitoring
	udc_start();
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
#if true  //for video now
	NVIC_DisableIRQ(SPI_IRQn);  // spi5 peripheral instance = 21, liyenho
	NVIC_ClearPendingIRQ(SPI_IRQn);
	NVIC_SetPriority(SPI_IRQn, 0);
	NVIC_EnableIRQ(SPI_IRQn);
#endif
	NVIC_DisableIRQ(SPI0_IRQn);  // spi0 peripheral instance = 8, liyenho
	NVIC_ClearPendingIRQ(SPI0_IRQn);
	NVIC_SetPriority(SPI0_IRQn, 0);
	NVIC_EnableIRQ(SPI0_IRQn);
 #ifdef RADIO_SI4463
	NVIC_DisableIRQ(PIOA_IRQn);  // pioa radio instance = 11, liyenho
	NVIC_ClearPendingIRQ(PIOA_IRQn);
	NVIC_SetPriority(PIOA_IRQn, 0);
	NVIC_EnableIRQ(PIOA_IRQn);
	NVIC_DisableIRQ(SPI2_IRQn);  // spi2 peripheral instance = 8, liyenho
	NVIC_ClearPendingIRQ(SPI2_IRQn);
	NVIC_SetPriority(SPI2_IRQn, 0);
	NVIC_EnableIRQ(SPI2_IRQn);
 #endif
 	if (SysTick_Config(sysclk_get_cpu_hz() / 500)) { // 2 msec tick
		//puts("-E- Systick configuration error\r");
		while (1) {
			/* Capture error */
		}
	}
	pmc_enable_periph_clk(ID_PIOA);
  pio_set_output(PIOA, PIO_PA0, LOW, DISABLE, ENABLE);  // turn fpga into config mode, liyenho
	//pio_set_output(PIOA, PIO_PA19, HIGH, DISABLE, ENABLE); // hold fpga reset (no reset), liyenho
 #ifndef RADIO_SI4463
	pio_set_output(PIOA, HEALTH_LED, HIGH, DISABLE, ENABLE);
 #else //RADIO_SI4463
	pio_set_output(PIOB, HEALTH_LED, HIGH, DISABLE, ENABLE);
 #endif
 #ifdef SPI0_FOR_SPI5
	pio_set_output(PIOA, PIO_PA24, LOW, DISABLE, ENABLE);  // setup spi0 regular usage
 #endif
	spi_master_initialize(0, SPI0_MASTER_BASE, BOARD_FLEXCOM_SPI0);// fpga ctrl pipe
  #ifdef RADIO_SI4463
	spi_master_initialize(2, SPI2_MASTER_BASE, BOARD_FLEXCOM_SPI2);// radio data pipe
  #endif
	spi_master_initialize(1, SPI_MASTER_BASE, BOARD_FLEXCOM_SPI);// video pipe @ tx end
#ifdef CONFIG_ON_FLASH
		if (1!=*(uint8_t*)ul_page_addr_bootapp || 0!=*(uint8_t*)ul_page_addr_fpgadef) {
			erase_last_sector() ;
			CHECKED_FLASH_WR(
				IFLASH_ADDR + IFLASH_SIZE-sizeof(backup),
				backup, NUM_OF_FPGA_REGS +1 +4 +2)
			//bootapp = 0; // tx board behaved as usual, so allow bootloader switch on udc
			bootapp = 1; // boot right into app
			CHECKED_FLASH_WR(ul_page_addr_bootapp, &bootapp, 1)
			fpgadef = 0; // switch fpga to app image
			CHECKED_FLASH_WR(ul_page_addr_fpgadef, &fpgadef, 1)
			CHECKED_FLASH_WR(
				IFLASH_ADDR + IFLASH_SIZE-NUM_OF_ATMEL_REGS +4,
				backup +NUM_OF_FPGA_REGS +1 +4 +4,
				sizeof(backup)-NUM_OF_FPGA_REGS-1 -4 -4)
		}
#endif
system_restart:  // system restart entry, liyenho
	system_main_restart = false;
	set_rf_params = false ;
#ifdef  RADIO_SI4463
  si4463_radio_started = false;
  ctrl_tdma_enable = false;
	ctrl_tdma_lock = false;
	timedelta_reset = timedelta_reset_rx = true;
	fhop_in_search = true;
	fhop_flag = false ;
	// start up with constant offset, we'll modify to adopt pairing reset later, liyenho
	fhop_base = 0;
    fhop_offset = HOP_2CH_ENABLE?WRAP_OFFSET(HOP_2CH_OFFSET0):fhop_base;
	fhop_dir = true;  // hop forward when startup
	fhop_idx= 0 ; // used for pattern access
  #ifdef RADIO_CTRL_AUTO
	 #ifdef TEMPERATURE_MEASURE
		hop_state = IDLE;
	 #else
		hop_state = PAIRING;
	 #endif
  #else
	hop_state = IDLE;
  #endif
#endif
#ifndef MEDIA_ON_FLASH
 	while (!udi_cdc_data_running) ; // wait for cdc data intf ready, liyenho
#endif

#ifdef MEDIA_ON_FLASH
 #ifndef NO_USB
		int w = *(uint32_t*)(~0x3&page_addr_bypass);
		int s = (page_addr_bypass & 0x3) * 8;
		if (true!= (0xff & (w >>=  s))) {
			while (!udi_cdc_data_running) ; // wait for cdc data intf ready, liyenho
		}
 #endif
#endif
 // enable cpu counter
 *DEMCR = *DEMCR | 0x01000000;
 *DWT_CYCCNT = 0; // reset the counter
 *DWT_CONTROL = *DWT_CONTROL | 1 ; // enable the counter
 #ifdef MEDIA_ON_FLASH
		w = *(uint32_t*)(~0x3&page_addr_bypass);
		s = (page_addr_bypass & 0x3) * 8;
		if (true == (0xff & (w >>=  s))) {
			goto bypass;
		}
  #if !defined(NO_USB)
	while (!usb_load_media && !system_upgrade) ; // wait for host media download signal
		if (system_upgrade)
			upgrade_sys_fw(system_upgrade);
	volatile bool erase = true;
	volatile uint32_t er_adr =ul_page_addr-SECTOR_RES, rem = media_file_len;
#ifdef DEBUG1
	volatile int ii, bcnt = 0;
#endif
	lm = SECTOR_RES+I2SC_BUFFER_SIZE;
	le = erlen;
	do {
		usb_read_buf1(gs_uc_tbuffer,I2SC_BUFFER_SIZE);
		if (!system_main_restart) while (1);	// it can't happen...
		if (erase) {
			static bool once = false;
			if (!once) {
				erase_last_sector() ;
				once = true;
			}
			// flash_erase_page() doesn't work at all! liyenho
			ul_rc = flash_erase_sector(er_adr);
			if (ul_rc != FLASH_RC_OK) {
				//printf("- Pages erase error %lu\n\r", (UL)ul_rc);
				return; /* error when erase pages */
			}
			erase = false;
		}
		ul_rc = flash_write(ul_page_addr, gs_uc_tbuffer, I2SC_BUFFER_SIZE, 0);
		if (ul_rc != FLASH_RC_OK) {
			//printf("- Pages write error %lu\n\r", (UL)ul_rc);
			return; /* error when write pages */
		}
		rem -= I2SC_BUFFER_SIZE;
		ul_page_addr += I2SC_BUFFER_SIZE;
		// determine whether erasure is necessary or not
		lm += I2SC_BUFFER_SIZE;
		if (le < lm) {
			erase = true;
			le += erlen;
			er_adr += erlen;
		}
 #ifdef DEBUG1
		bcnt += 1;  // track how many blocks in I2SC_BUFFER_SIZE
 #endif
	} while (0<rem);
  #ifdef DEBUG1
  ul_page_addr=PAGE_ADDRESS ;
  	do {
	  	memcpy(gs_uc_rbuffer, ul_page_addr, I2SC_BUFFER_SIZE);

		  delay_us(3000); // see if this stablize? it does!
  		usb_write_buf1(gs_uc_rbuffer,I2SC_BUFFER_SIZE);
  		ul_page_addr += I2SC_BUFFER_SIZE;
  	} while (0<--bcnt);
  #endif
   // write this media length onto flash
	CHECKED_FLASH_WR(page_addr_mlen, &media_file_len, 4)
  // mark up bypass flag so next power up won't need media download again...
  w = true;
  CHECKED_FLASH_WR(page_addr_bypass, &w, 1/*1 byte flag*/)
	CHECKED_FLASH_WR(
		IFLASH_ADDR + IFLASH_SIZE-NUM_OF_ATMEL_REGS-NUM_OF_FPGA_REGS,
		backup +1 +4 ,
		sizeof(backup)-1 -4)
bypass:
  ul_page_addr=PAGE_ADDRESS ;
		uint32_t wtmp = *(uint32_t*)(~0x3&page_addr_mlen);
		uint32_t shf = (page_addr_mlen & 0x3) * 8;
		media_file_len = (wtmp >> shf);
		if (shf) {
			wtmp = *(uint32_t*)(~0x3&(page_addr_mlen+4));
			media_file_len |= ((1<<(shf+8))-1) & (wtmp << (32-shf));
		}
  ul_page_end=ul_page_addr + media_file_len;
  #else /*NO_USB*/
  	pusbs = ul_page_addr;
  	pusbe = pusbs+ sizeof(PAGE_ADDRESS);
  #endif
 #endif
 #ifdef CONFIG_ON_FLASH
	{ //volatile uint32_t er_adr_c =ul_page_addr_c-SECTOR_RES_C;
	uint32_t shf, tmp, wtmp, *pth = &tmp;
#if false // don't erase ever...
		ul_rc = flash_erase_sector(er_adr_c);
		if (ul_rc != FLASH_RC_OK) {
			//printf("- Pages erase error %lu\n\r", (UL)ul_rc);
			return; /* error when erase pages */
		}
#endif
  		// load config registry upon power up
#ifdef NOIS_EXTRACTED
  		pio_set_output(PIOA, PIO_PA20, HIGH, DISABLE, ENABLE);  // for ad9364 reset
	  rf_dvbt_startup();  // setup ad9364/config mem defaults
#endif
		spi_tgt_done = true;
  #if false // not now, first let's config only a few of fpga registers
  		dev_spi_wadr(0x0, wtmp, 1) // adr<- 0x0
		for (n=0; n<128/*all r/w registers*/ ;n++) {
			wtmp = *(uint32_t*)(~0x3&(ul_page_addr_c+n));
			shf = ((ul_page_addr_c+n) & 0x3) * 8;
		  wtmp >>=  shf;
			dev_spi_wcnt(0xb000, wtmp, 1, wtmp)
		}
  #else	// tentative config 4 fpga registers by constants
        delay_ms(5000);  //delay to see effect  (3s is too short, nios not up yet)
#ifndef RFFE_PARAMS
		//enable TX power output
		dev_spi_wr(68, wtmp, 1, 0x80) // reg68<- 0x80
		//trigger NIOS to update parameters
#ifndef NOIS_EXTRACTED
		dev_spi_wr(65, wtmp, 1, 0x40) // set check-in bit
#else
  #ifdef SPI0_FOR_SPI5
		spi0_set_peripheral();
		spi_master_initialize(0, SPI0_MASTER_BASE, BOARD_FLEXCOM_SPI0);// fpga ctrl pipe
		spi_configure_cs_behavior(SPI0_MASTER_BASE, 0, SPI_CS_RISE_NO_TX);
  #endif
		rf_dvbt_control();
  #ifdef SPI0_FOR_SPI5
		spi0_set_peripheral();
		spi_master_initialize(0, SPI0_MASTER_BASE, BOARD_FLEXCOM_SPI0);// fpga ctrl pipe
		spi_configure_cs_behavior(SPI0_MASTER_BASE, 0, SPI_CS_RISE_NO_TX);
  #endif
#endif
		spi_tgt_done = true;
				//power setting: 10000mdb = reg(72,73)= 0x27,0x10
				dev_spi_wr(72, wtmp, 1, 0x27) // reg72<- 0x27
				dev_spi_wr(73, wtmp, 1, 0x10) // reg73<- 0x10
#else //RFFE_PARAMS
#ifndef MEDIA_ON_FLASH
		while (!set_rf_params && !system_upgrade) ;
		if (system_upgrade)
			upgrade_sys_fw(system_upgrade);
#endif
		dev_access *pr, *pt = (dev_access*)gs_uc_htbuffer;
		  pr = gs_uc_hrbuffer1;
#ifndef NOIS_EXTRACTED
	for (int itr=0; itr<10000; itr++) {
			spi_tgt_done = true;
		  // setup spi to read addressed data
		  dev_spi_rd(0x100, wtmp, 1, &nios_done)
		if (nios_done == 0x81)
			break; // nios init is done
	}
		if (nios_done != 0x81) {
			/*puts("NIOS did not come up!");*/ while(1); // raise exception
		}
#endif
		frequency_select();
		attenuation_select();
		//enable TX power output: reg (68)
		//delay is here to fix problems where reg_word1 would read back a shifted value
		//delay value of 200 us experimentally determined (100 was fine, 2x to be safe)
		dev_spi_wr(68, wtmp, 200, 0x80)
		delay_ms(1);
		//trigger NIOS to update parameters
#ifndef NOIS_EXTRACTED
		dev_spi_wr(65, wtmp, 1, 0x40)
#else
  #ifdef SPI0_FOR_SPI5
		spi0_set_peripheral();
		spi_master_initialize(0, SPI0_MASTER_BASE, BOARD_FLEXCOM_SPI0);// fpga ctrl pipe
		spi_configure_cs_behavior(SPI0_MASTER_BASE, 0, SPI_CS_RISE_NO_TX);
  #endif
		rf_dvbt_control();
  #ifdef SPI0_FOR_SPI5
		spi0_set_peripheral();
		spi_master_initialize(0, SPI0_MASTER_BASE, BOARD_FLEXCOM_SPI0);// fpga ctrl pipe
		spi_configure_cs_behavior(SPI0_MASTER_BASE, 0, SPI_CS_RISE_NO_TX);
  #endif
#endif
#endif
  #endif
		spi_tgt_done = false; }
 #endif
	init_4463();  // be sure to place this function before next line! liyenho
	rdptr_rdo_tpacket=0;
	wrptr_rdo_tpacket=0;
	rdptr_rdo_rpacket=0;
	wrptr_rdo_rpacket=0;
	//initialize to slot 1 to prevent skipping errors
	//otherwise pointer goes slot 0, 2, 3
	gp_rdo_rpacket_l = gs_rdo_rpacket + (RDO_ELEMENT_SIZE*1);

#ifdef RADIO_CTRL_AUTO
		vRadio_Init();
	#ifdef CONFIG_ON_FLASH
		uint8_t median = *(uint8_t*)ul_page_addr_ctune;
		if (0xff != median)  // adjust cap bank value per stored const
			recalibrate_capval((void*)ul_page_addr_mtemp, median);
	#endif
    #ifdef RADIO_CTRL_TXCONTINOUS
	    {
	    for(int i=0;i<2*RDO_ELEMENT_SIZE;i++)
	    gs_rdo_tpacket[i]=0xc5c5c5c5;
		}
        vRadio_StartTx_Variable_Packet(pRadioConfiguration->Radio_ChannelNumber, gp_rdo_tpacket_l,
			   RADIO_PKT_LEN); //pRadioConfiguration->Radio_PacketLength);
    #elif !defined(TEMPERATURE_MEASURE)
    	ctrl_hop_global_update(true);
    #endif
		si4463_radio_started = true;
		ctrl_tdma_enable = true;
#endif //RADIO_CTRL_AUTO
 	main_loop_on = true;  // enter run time stage, liyenho
	if (system_upgrade)
		upgrade_sys_fw(system_upgrade);
 	nios_done = false; // reset to protect restart process
#ifdef RADIO_SI4463
	r4463_sts.tick_prev = tick_prev = *DWT_CYCCNT;
#endif
	// The main loop manages only the power mode
	// because the USB management is done by interrupt

	#if FIXED_PAIR_ID
	uint8_t hop_id[HOP_ID_LEN] = {1};
	Set_Pair_ID(hop_id);
	#endif //FIXED_PAIR_ID

	while (true) {
		if (system_main_restart ) goto system_restart;
	if (system_upgrade)
		upgrade_sys_fw(system_upgrade);
		if (!stream_flag) goto _reg_acs; // stop TS stream if flag isn't true, liyenho
#ifdef RADIO_SI4463
  #ifdef TEMPERATURE_MEASURE
    static int once = false;
		if (!once) {
			vRadio_StartTx(pRadioConfiguration->Radio_ChannelNumber); // continued transmit & stop listening, liyenho
			once = 1;
		}
  #endif
  	cap_bank_calibrate();
    //tdd lock time out routine ----------------------------------------
	tick_curr = *DWT_CYCCNT;
	if (tick_curr < tick_prev) {
	 	rxnorec_intv+=tick_curr+(0xffffffff-tick_prev);
	}
	else // not wrapped yet
	 	rxnorec_intv+=tick_curr -tick_prev;
	tick_prev = tick_curr;
	if(rxnorec_intv> TDMA_UNLOCK_DELAY_MS) { //no rx unlock routine
	  	rxnorec_intv=0;
		if (fhopless)
		  switch(fhopless) {
			  case 1/*low*/: fhop_offset = 0;
			  case 2/*mid*/: fhop_offset = (CHTBL_SIZE)/2-1;
			  case 3/*high*/: fhop_offset = (CHTBL_SIZE)-1;
		  }
		ctrl_tdma_lock = false;
		fhop_in_search = true;
		fhop_flag = false ;
	}
 #ifdef CTRL_DYNAMIC_MOD
	process_range_mode(tick_curr, tick_prev);
 #endif
#endif
		// start usb rx line
		pusb = (1 & usbfrm) ?((uint8_t*)gs_uc_tbuffer)+I2SC_BUFFER_SIZE : gs_uc_tbuffer;
  #ifdef MEDIA_ON_FLASH
   #ifdef NO_USB
   	memcpy(pusb, pusbs, ts_inc*188);
   	memset(pusb+ts_inc*188, 0x0, (10-ts_inc)*188);
   	pusbs += ts_inc*188;
   	if (pusbe<=pusbs)
   		pusbs= ul_page_addr;
   #else
  		memcpy(pusb, ul_page_addr, I2SC_BUFFER_SIZE);
  		ul_page_addr += I2SC_BUFFER_SIZE;
  		if (ul_page_end <=ul_page_addr)
  			ul_page_addr = PAGE_ADDRESS;
  	#endif
  		delay_us(109000); // approximate 136 kb/s to simulate usb read
  #else
  		usb_read_buf1(pusb,I2SC_BUFFER_SIZE);
		if (!system_main_restart)
			goto _reg_acs ;
  #endif
		while (usb_data_done) ; // usb pipe overflow, liyenho
  #if defined(TEST_USB) || defined(TEST_SPI)
	 volatile uint8_t *pusb1;
	   pusb1 = (1 & usbfrm) ?((uint8_t*)gs_uc_rbuffer)+I2SC_BUFFER_SIZE : gs_uc_rbuffer;
  #endif
  #ifndef TEST_USB
  	#ifndef TEST_SPI
		spi_tx_transfer(pusb, I2SC_BUFFER_SIZE/2,
			gs_uc_rbuffer/*don't care*/, I2SC_BUFFER_SIZE/2, 1/*video*/);
   #else
		spi_tx_transfer(pusb, I2SC_BUFFER_SIZE/2,
			pusb1, I2SC_BUFFER_SIZE/2, 1/*video*/);
   #endif
  #endif
		usb_data_done = true;
  #if defined(TEST_USB) || defined(TEST_SPI)
#ifdef TEST_SPI
		while (usb_data_done) ;
#else
  		delay_us(800); // a bit delay to lower bit rate
  		memcpy(pusb1,pusb,I2SC_BUFFER_SIZE);
	   usb_data_done = false;
#endif
   #ifndef MEDIA_ON_FLASH
  		usb_write_buf1(pusb1,I2SC_BUFFER_SIZE);
   #endif
  #endif
		usbfrm = usbfrm + 1;
_reg_acs:
  		if (usb_host_msg && !system_main_restart) {	// host ctrl/sts link with fpga/sms process, liyenho
			usb_host_msg = false;
//#define TEST_FLASH
 #ifdef TEST_FLASH
	uint32_t shf, wtmp;
 #endif
		//now keep regs acs on for both tx/rx, liyenho
			dev_access *pr, *pt = (dev_access*)gs_uc_htbuffer;
			uint16_t tmp, tmpw, *prh, *pth = &tmp;
			while (spi_tgt_done) ; // flush any pending spi xfer
				spi_tgt_done = true;
			switch (pt->access) {
				case READ_CUR:
							pr = gs_uc_hrbuffer1;
							// setup spi to read curr data
							dev_spi_rcnt(0x00ff, 1, pr->data)
							break;
				case READ_BY_ADDR:
							pr = gs_uc_hrbuffer1;
							// setup spi to read addressed data
							dev_spi_rd(pt->addr, tmpw , 1,pr->data)
#ifdef TEST_FLASH
	wtmp = *(uint32_t*)(~0x3&(ul_page_addr_c+pt->addr));
	shf = ((ul_page_addr_c+pt->addr) & 0x3) * 8;
  *(uint8_t*)pr->data = 0xff & (wtmp >> shf);
#endif
							break;
				case WRITE_CUR:
							// setup spi to write curr data
							dev_spi_wcnt(0x8000, tmpw,1,pt->data[0])
							break;
				case WRITE_BY_ADDR:
							// setup spi to write addressed data, mask 0xb000 instead 0xa000
							dev_spi_wr(pt->addr, tmpw, 1, pt->data[0])
				#ifdef CONFIG_ON_FLASH
					if (pt->toflash) {
						CHECKED_FLASH_WR(ul_page_addr_c+pt->addr, &pt->data[0], 1/*1 byte register*/)
					}
				#endif
							break;
				case BURST_FIFO_READ:
							pr = gs_uc_hrbuffer1; prh = pr->data;
							// setup spi to address then read fifo
							dev_spi_radr(pt->addr, tmpw, 1)
							for (n=0; (!system_main_restart && n<pt->dcnt) ;n++) {
								dev_spi_rcnt(0x20ff, 1, prh++)
//#define TEST_READ
#ifdef TEST_READ
  while (spi_tgt_done) ;
  *(prh-1) = 90+n;
#endif
							}
							break;
				case BURST_FIFO_WRITE:
							// setup spi to write addressed fifo
							dev_spi_wadr(pt->addr, tmpw ,1)
							for (n=0; (!system_main_restart && n<pt->dcnt) ;n++) {
								dev_spi_wcnt(0xa000, tmpw, 1, pt->data[n])
							}
							break;
				case BURST_MEM_READ:
							pr = gs_uc_hrbuffer1; prh = pr->data;
							// setup spi to address then read mem
							dev_spi_radr(pt->addr, tmpw, 1)
							for (n=0; (!system_main_restart && n<pt->dcnt) ;n++) {
								dev_spi_rcnt(0x30ff, 1, prh++)
#ifdef TEST_FLASH
	wtmp = *(uint32_t*)(~0x3&(ul_page_addr_c+pt->addr+n));
	shf = ((ul_page_addr_c+pt->addr+n) & 0x3) * 8;
	pr->data[n] = 0xff & (wtmp >> shf);
#endif
							}
							break;
				case BURST_MEM_WRITE:
							// setup spi to write addressed mem
							dev_spi_wadr(pt->addr, tmpw ,1)
							for (n=0; (!system_main_restart && n<pt->dcnt) ;n++) {
								dev_spi_wcnt(0xb000, tmpw, 1, pt->data[n])
					#ifdef CONFIG_ON_FLASH
						if (pt->toflash) {
							CHECKED_FLASH_WR(ul_page_addr_c+pt->addr+n, &pt->data[n], 1/*1 byte register*/)
						}
					#endif
							}
							break;
				default: /* host msg in error */
							continue;
			} while (spi_tgt_done) ;
  		}
#if 0
		sleepmgr_enter_sleep();
#endif
	}
}

void main_suspend_action(void)
{
	return ;
	ui_powerdown();
}

void main_resume_action(void)
{
	return ;
	ui_wakeup();
}

volatile bool main_usb_host_msg() // Attach this api to usb rx isr service, liyenho
{
	if (USB_HOST_MSG_TX_VAL != udd_g_ctrlreq.req.wValue) {
		if (USB_FPGA_UPGRADE_VAL != udd_g_ctrlreq.req.wValue &&
				USB_ATMEL_UPGRADE_VAL != udd_g_ctrlreq.req.wValue)
		return false ;
		else goto payload_next;
	}
	if (USB_HOST_MSG_IDX != udd_g_ctrlreq.req.wIndex)
		return false ;
	if (USB_HOST_MSG_LEN > udd_g_ctrlreq.req.wLength	)
		return false ;
payload_next:
	// must setup max packet size! liyenho
	udd_set_setup_payload( gs_uc_htbuffer,
		USB_HOST_MSG_LEN+HOST_BUFFER_SIZE*sizeof(uint16_t));
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
		volatile dev_access *ps = gs_uc_hrbuffer1;
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
	if (USB_FPGA_DEF_VAL == udd_g_ctrlreq.req.wValue) {
		udd_set_setup_payload( &fpgadef, sizeof(fpgadef));
		udd_g_ctrlreq.callback = host_usb_cb;
	}
 #endif
	if (USB_SYSTEM_RESTART_VAL == udd_g_ctrlreq.req.wValue) {
		main_loop_restart(); return true;
	}
	if (USB_HOST_MSG_TX_VAL == udd_g_ctrlreq.req.wValue ||
		USB_FPGA_UPGRADE_VAL == udd_g_ctrlreq.req.wValue ||
		USB_ATMEL_UPGRADE_VAL == udd_g_ctrlreq.req.wValue)
		return main_usb_host_msg();
	if (USB_FPGA_NEW_VAL == udd_g_ctrlreq.req.wValue) {
		system_upgrade = 3;  // switch to fpga app image
	}
 #if defined(MEDIA_ON_FLASH) && !defined(NO_USB)
	if (USB_LOAD_MEDIA == udd_g_ctrlreq.req.wValue)
		return main_usb_load_media();
 #endif
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
	 } // si4462 radio will operate on both rx/tx ends
#ifdef  RADIO_SI4463
	 else if (RADIO_COMM_VAL == udd_g_ctrlreq.req.wValue) {
		 usb_ctrl_cmd_portal(&udd_g_ctrlreq);
	 }
#endif
#ifdef RFFE_PARAMS
	 else if (RF_TX_VAL == udd_g_ctrlreq.req.wValue ||
	 				RF_TX_FREQ_VAL == udd_g_ctrlreq.req.wValue ||
	 				RF_TX_ATTN_VAL == udd_g_ctrlreq.req.wValue) {
		 if (sizeof(TX_Rf_Params.params_tx) != udd_g_ctrlreq.req.wLength) {
			 return (bool) -1;
		 }
		udd_set_setup_payload( gs_uc_htbuffer, sizeof(TX_Rf_Params.params_tx));
		udd_g_ctrlreq.callback = rf_params_cb; // RF setup callback
	 }
	 else if (RF_TX_NIOS_DONE == udd_g_ctrlreq.req.wValue) {
	 	udd_set_setup_payload( &nios_done, sizeof(uint32_t));
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
#ifdef FWM_DNLD_DBG
	else if (0xd7 == udd_g_ctrlreq.req.wValue) {
		usb_host_active = true;
		usb_tgt_active = false;
	}
	else if (0x7d == udd_g_ctrlreq.req.wValue) {
		udd_set_setup_payload( &usb_tgt_active, sizeof(usb_tgt_active));
	}
#endif
 	return (bool) -1; /* error in msg */
}

void main_sof_action(void)
{
	return ;
	if (!main_b_cdc_enable)
		return;
	ui_process(udd_get_frame_number());
}

#ifdef USB_DEVICE_LPM_SUPPORT
void main_suspend_lpm_action(void)
{
	return ;
	ui_powerdown();
}

void main_remotewakeup_lpm_disable(void)
{
	return ;
	ui_wakeup_disable();
}

void main_remotewakeup_lpm_enable(void)
{
	return ;
	ui_wakeup_enable();
}
#endif

bool main_cdc_enable(uint8_t port)
{
	main_b_cdc_enable = true;
	// Open communication
	//uart_open(port); // don't use usart in usb context
	return true;
}

void main_cdc_disable(uint8_t port)
{
	main_b_cdc_enable = false;
	// Close communication
	//uart_close(port); // don't use usart in usb context
}

void main_cdc_set_dtr(uint8_t port, bool b_enable)
{
	return ;
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
	}
}

static int timedelta(bool reset, unsigned int bignum, unsigned int smallnum)
{ // what the hack on old codes doing??? liyenho
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
