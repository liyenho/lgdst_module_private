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
static volatile bool system_main_restart = false;  // system restart flag, liyenho
static volatile uint8_t system_upgrade = 0;  // system upgrade flag, liyenho
/*static*/ volatile char __version_atmel__[3];  // stored as mon/day/year
#define INCLUDEINMAIN  // for Kevin's ctrl design
#include "bsp.h"
#include "lgdst_4463_spi.h"
#ifdef RADIO_SI4463 // not used with Kevin's design
 #include "si446x_nirq.h"
 #include "radio.h"
 #include "ctrl.h"
#endif
#include "Directional_Antenna_Selection.h"
#include "USB_Commands/USB_Commands.h"
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
  /** The address for TWI IT951X */
  #define IT951X_ADDRESS        0x3A  // not sure if this is the one but it is ok to guess

  /* still need to confirm on these..., done */
  #define ITE_REG_ADDR         0
  #define ITE_REG_ADDR_LENGTH  /*2*/ 0
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
  				  	2000000, 8000000, RADIO_SPI_BR}; // (fpga/sms, video, radio) spi bit rate
extern volatile bool udi_cdc_data_running; // from udi_cdc.c, liyenho
#ifdef  RADIO_SI4463
  volatile bool si4463_radio_started = false; // radio startup flag...
  volatile bool ctrl_tdma_enable = false;
  volatile capv_tune_t si4463_factory_tune;
/*******************************************************************/
  volatile bool ctrl_tdma_lock = false;
  volatile bool ctrl_tdma_rxactive = false;  //block rtt_handle startTx() call
	static int timedelta(bool reset, unsigned int bignum, unsigned int smallnum);
	/*static*/ bool timedelta_reset, timedelta_reset_rx; //to handle system restart
	volatile bool fhop_in_search= false, fhop_flag, fhop_dir;
  unsigned int tdma_sndthr=0;
	/*static*/ uint8_t hop_id[HOP_ID_LEN]={0,0,0,0,0,0,0,0,0,0}; // 10 byte hop id from host
	int fhop_idx= 0, fhop_offset = HOP_2CH_ENABLE?(HOP_2CH_OFFSET0):0; //to produce frequency hopping sequence
	int fhop_base = 0, fhopless= 0/*can be 1, 2 or 3*/;
  /*static*/ enum pair_mode hop_state;
  	volatile unsigned char ynsdbyte ; //Take care in Rate Control Section!!! liyenho
#endif
volatile bool stream_flag = false, // wait for host to signal TS stream on
							vid_ant_switch = false;
#ifdef CONFIG_ON_FLASH
uint32_t ul_page_addr_ctune =IFLASH_ADDR + IFLASH_SIZE - NUM_OF_ATMEL_REGS, // 1st atmel reg on flash
					// temperature @ current tuning @ 2nd atmel reg on flash
				ul_page_addr_mtemp =IFLASH_ADDR + IFLASH_SIZE - NUM_OF_ATMEL_REGS + 1,
				ul_page_addr_bootapp =IFLASH_ADDR + IFLASH_SIZE - NUM_OF_ATMEL_REGS + 2;
uint8_t backup[NUM_OF_FPGA_REGS+NUM_OF_ATMEL_REGS+4+1],
				bootapp = (uint8_t)/*-1*/0;
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
#ifndef CTRL_RADIO_ENCAP
	static uint32_t gs_uc_tbuffer[2*(I2SC_BUFFER_SIZE)/sizeof(int)];
#else
	/* adapt to requirement of control radio data encapsulation */
static uint32_t gs_uc_tbuffer[2*(TP_SIZE+I2SC_BUFFER_SIZE)/sizeof(int)];
extern uint8_t fill_radio_pkt(uint8_t *pusb);
#endif
volatile static uint32_t usbfrm = 0, prev_usbfrm= 0;
//algorithm from digibest sdk, using bulk transfer pipe, liyenho
volatile uint32_t upgrade_fw_hdr[FW_UPGRADE_HDR_LEN/sizeof(int)]={-1} ;
#if defined(FWM_DNLD_DBG)
  volatile bool usb_host_active = false;
  extern volatile bool usb_tgt_active ;
#endif
 // host to fpga/6612 ctrl/sts buffer
/*static*/ uint32_t gs_uc_htbuffer[(USB_HOST_MSG_LEN+HOST_BUFFER_SIZE+3)/sizeof(int)],
										gs_uc_htbuffer_tm[(USB_HOST_MSG_LEN+HOST_BUFFER_SIZE+3)/sizeof(int)];
/*static*/ uint32_t gs_uc_hrbuffer[(USB_HOST_MSG_LEN+HOST_BUFFER_SIZE+3)/sizeof(int)];
  //volatile bool i2c_read_done = false;
  volatile context_it951x ctx_951x={0};
  /*static*/ twi_packet_t packet_tx;
  twi_packet_t packet_rx;
   // host/sms ctrl/sts buffer
   volatile uint32_t it951x_fw_hdr[1]={-1} ; //byte lenght of firmware
   /*static*/ uint32_t gs_ite_tbuffer[(ITE_BUFFER_SIZE+3)/sizeof(int)];
   /*static*/ uint32_t gs_ite_rbuffer[(ITE_BUFFER_SIZE+3)/sizeof(int)];
 #if true  // add two big memory chunks for ite fw download
   /*static*/ uint32_t fw_ite_tbuffer[FW_DNLD_SIZE/sizeof(int)];
   /*static*/ uint32_t fw_ite_rbuffer[FW_DNLD_SIZE/sizeof(int)];
 #endif
/* Pointer to UART PDC register base */
Pdc *g_p_spim_pdc [1+2]/*fpga/sms, video, radio ctrl/sts, liyenho*/,*g_p_spis_pdc;
Pdc *g_p_i2st_pdc, *g_p_i2sr_pdc;
struct i2s_dev_inst dev_inst_i2s;
#ifdef RADIO_SI4463
volatile uint32_t gs_rdo_tpacket[RDO_TPACKET_FIFO_SIZE*RDO_ELEMENT_SIZE] = {0xffffffff};
volatile uint32_t tpacket_idle[RDO_ELEMENT_SIZE];
unsigned char tpacket_grp[RADIO_GRPPKT_LEN];
unsigned int *gp_rdo_tpacket_l = ((unsigned int*)gs_rdo_tpacket);
unsigned int *gp_rdo_tpacket = (unsigned int*)gs_rdo_tpacket;
unsigned char gs_rdo_tpacket_ovflw=0;
volatile uint32_t wrptr_rdo_tpacket=RDO_TPACKET_FIFO_SIZE-1,       //wrptr to valid data
                  rdptr_rdo_tpacket=RDO_TPACKET_FIFO_SIZE-1;       //rdptr to last consummed data
                                                                   //rdptr=wrptr, data consummed, rdptr take priority

volatile uint32_t gs_rdo_rpacket[RDO_RPACKET_FIFO_SIZE*RDO_ELEMENT_SIZE] = {0xffffffff};
unsigned int *gp_rdo_rpacket = (unsigned int*)gs_rdo_rpacket;
unsigned int *gp_rdo_rpacket_l = ((unsigned int*)gs_rdo_rpacket);
volatile uint32_t rpacket_idle[ASYMM_RATIO* RDO_ELEMENT_SIZE];
uint32_t rpacket_ov[RDO_ELEMENT_SIZE]; //gs_rdo_rpacket fifo overflow holder
unsigned char rpacket_grp[RADIO_GRPPKT_LEN+RADIO_INFO_LEN];  // fixed
unsigned char rpacket_grp_partial[RADIO_GRPPKT_LEN];
unsigned char gs_rdo_rpacket_ovflw=0;
volatile uint32_t wrptr_rdo_rpacket=RDO_RPACKET_FIFO_SIZE-1,       //wrptr to valid data
                  rdptr_rdo_rpacket=RDO_RPACKET_FIFO_SIZE-1;       //rdptr to last consummed data

	unsigned int  radio_mon_txfailcnt=0;
	unsigned int  radio_mon_rxcnt = 0;
  unsigned int  radio_mon_rxerr = 0;
	unsigned int  radio_mon_txidlecnt = 0;
	unsigned int  radio_mon_txcnt = 0;
	unsigned int  rxnorec_intv=0;  // not a count but interval

	static uint32_t tick_curr, tick_prev;
	extern volatile ctrl_radio_stats  r4463_sts;
#endif //RADIO_SI4463
/**
 *  \brief Configure the Console UART.
 */

// function prototyping
uint32_t wrptr_inc(uint32_t *wrptr,  uint32_t *rdptr, uint32_t fifodepth, int step);
uint32_t rdptr_inc(uint32_t *wrptr,  uint32_t *rdptr, uint32_t fifodepth, int step);
uint32_t fifolvlcalc(uint32_t wrptr, uint32_t rdptr, uint32_t fifodepth);
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
 	delay_cycles(0.25*120/8*8); // assumed 120 mhz atmel clk, 8 mhz spi clk, and 8 bit data
	 pio_set (PIOA, PIO_PA28); // cs disabled @ spi1
	usb_data_done = false;  // handshake with mainloop
	status = spi_read_status(SPI_MASTER_BASE) ;

	//if(status & SPI_SR_NSSR) {
		if ( status & SPI_SR_TXBUFE ) {
//			printf("transfer done\n");
		}
	//}
}
void twi_sms4470_handler(const uint32_t id, const uint32_t index);
/* SMS4470 intr handler for i2c intf */
  void twi_sms4470_handler(const uint32_t id, const uint32_t index)
	{
		if ((id == ID_PIOA) && (index == ITE_HOST_INT)){
			ctx_951x.i2c_done_rd = false;
			uint32_t err, // error code requried
								len0 = packet_rx.length;
			static uint8_t tmp_buff[HOST_BUFFER_SIZE];
			static uint32_t err_prev, len_prev;
			{	TWI_READ	}
			ctx_951x.i2c_done_rd = true;
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
	packet_tx.chip        = IT951X_ADDRESS;
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
 #if true
 	if (1 == ch)
	 	spi_set_clock_phase(base, SPI_CHIP_SEL, 0/*captured @ falling, transit @ rising*/);
	  spi_set_bits_per_transfer(base, SPI_CHIP_SEL,
			SPI_CSR_BITS_8_BIT);  // 8 bit spi xfer, liyenho
 #endif
	spi_set_baudrate_div(base, SPI_CHIP_SEL,
			(sysclk_get_cpu_hz() / gs_ul_spi_clock [ch]));
	if (2 == ch) // delay for ctrl spi link
	  spi_set_transfer_delay(base, SPI_CHIP_SEL, 0x10/*delay between bytes*/,
			0x10/*delay between spi xfer*/);
	else
	  spi_set_transfer_delay(base, SPI_CHIP_SEL, SPI_DLYBS/*delay between bytes*/,
			SPI_DLYBCT/*delay between spi xfer*/);
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

	if (1 == ch) {// cs enabled @ spi1
 		delay_cycles(0.25*120/8*8); // assumed 120 mhz atmel clk, 8 mhz spi clk, and 8 bit data
		pio_clear(PIOA, PIO_PA28);  // must be called prior to pdc_xx_init, liyenho
	}
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

static inline bool usb_read_buf(void *pb)
{
	int itr=0, read=0, size=I2SC_BUFFER_SIZE;
	static uint8_t rd_int_buff[I2SC_BUFFER_SIZE];
	static int once=1, left=0;
	uint8_t *pbr= rd_int_buff, *pbi;
	if (0 == once) {
		memcpy(pb,
			rd_int_buff+I2SC_BUFFER_SIZE-left,
			left);
	}
	do {
		iram_size_t b = size-udi_cdc_read_buf(pbr, size);
		if (0 == b) {
			itr += 1;
			if (10000 == itr)
				return false;
		} else itr = 0;
		pbr += b;
		size -= b;
		read += b;
	} while (I2SC_BUFFER_SIZE != read && !system_main_restart);
	if (1 == once) {
		size = I2SC_BUFFER_SIZE;
		pbr = rd_int_buff;
		pbi = &read;
		do {
			*(pbi+2) = *(pbr+2);
			*(pbi+1) = *(pbr+1);
			*(pbi+0) = *pbr++;
			size -= 1;
		} while (PID_VID != (0x00ff1fff & read)
							&& 0 < size
							&& !system_main_restart);
		if (0 == size)
			return false; // video packet not found yet
		left = size+1;
		once = 0;
	}
	else if (I2SC_BUFFER_SIZE>left) {
		memcpy(
			pb+left,
			rd_int_buff,
			I2SC_BUFFER_SIZE-left);
	}
	return true;
}
/*{
	int itr=0, read=0, size = I2SC_BUFFER_SIZE;
	do {
		iram_size_t b = size-udi_cdc_read_buf(pb, size);
		if (0 == b) {
			itr += 1;
			if (10000 == itr)
			return false;
		} else itr = 0;
		pb += b;
		size -= b;
		read += b;
	} while (I2SC_BUFFER_SIZE != read && !system_main_restart);
	return true;
}*/
//#ifdef SMS_DVBT2_DOWNLOAD
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
		else if ((USB_ITE_FW_VAL == udd_g_ctrlreq.req.wValue) &&
						ITE_FW_HDR_LEN >=udd_g_ctrlreq.payload_size) {
			memcpy(it951x_fw_hdr, // set ite913x fw hdr, liyenho
								udd_g_ctrlreq.payload,
								ITE_FW_HDR_LEN);
			return;
		}
		dev_access *ps = gs_uc_htbuffer,*ps1 = gs_uc_hrbuffer;
		 /////////////////
		//host/client handshake time sensitive processes (YH)
		if (0!=ctx_951x.it951x_access &&
			(!ctx_951x.i2c_done_rd || !ctx_951x.i2c_done_wr))
			return ; // no further action taken
       if (ps->access == IT951X_READ) {
	      ctx_951x.it951x_access = IT951X_READ;
	      ctx_951x.i2c_done_rd = false;  //YH: not-ready response to usb request
		}
		else if (ps->access == IT951X_WRITE ) {
	      ctx_951x.it951x_access = IT951X_WRITE;
	      ctx_951x.i2c_done_wr = false;  //YH: not-ready response to usb request
		}
		// prepare confirm msg by echo whatever received
		memcpy(gs_uc_hrbuffer, udd_g_ctrlreq.payload, USB_HOST_MSG_LEN-sizeof(ps->data[0]));
		usb_host_msg = true; // enable mainloop process
  }

#ifdef RADIO_SI4463
/*****************************************************************************
 *  si4464 definitions and Global Variables
 *****************************************************************************/
/*const*/ COMPILER_ALIGNED(8) tRadioConfiguration RadioConfiguration = RADIO_CONFIGURATION_DATA;
/*const*/ tRadioConfiguration *pRadioConfiguration = &RadioConfiguration;
	SEGMENT_VARIABLE(bMain_IT_Status, U8, SEG_XDATA);

	 /*static*/ void ctrl_hop_global_update(bool listen) {
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
		if (listen) {
			vRadio_StartRX(pRadioConfiguration->Radio_ChannelNumber,
				RADIO_PKT_LEN);
		}
	}
 #if true
int hop_chn_sel(int offset) {
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
	  if(wrptr_rdo_tpacket == rdptr_rdo_tpacket) { //idle packet case
		  radio_mon_txidlecnt++;
		  tpacket_idle[0]=0x0000e5e5 |((radio_mon_txidlecnt<<16)&0x00ff0000)
		  |((radio_mon_rxcnt<<24)&0xff000000);  //no new tx payload (byte7=0xE5)
      tpacket_idle[1]=0xe50000e5 |((radio_mon_rxerr<<8)&0x0000ff00)
      |((radio_mon_rxerr<<16)&0x00ff0000);  //no new tx payload (byte7=0xE5)
		  for(int i=2;i<RDO_ELEMENT_SIZE;i++) tpacket_idle[i]=0xe5e5e5e5;  //no new tx payload (byte7=0xE5)
		  ((uint8_t *)tpacket_idle)[RADIO_PKT_LEN-1] = ynsdbyte & 0x7f; //set payload invalid
		  gp_rdo_tpacket_l = tpacket_idle;
	  }
	  else {
		  int rdptr_tmp;
		  if(rdptr_rdo_tpacket >= (RDO_TPACKET_FIFO_SIZE-1))  rdptr_tmp=0;
		  else                                                rdptr_tmp =  rdptr_rdo_tpacket+1;
		  rdptr_rdo_tpacket = rdptr_tmp;
		  gp_rdo_tpacket_l = gs_rdo_tpacket + (RDO_ELEMENT_SIZE*rdptr_rdo_tpacket);
		  //((uint8_t *)gp_rdo_tpacket_l)[RADIO_PKT_LEN-1]= ynsdbyte | 0x80; //grp header builtin
	  }

	  if((ctrl_tdma_rxactive==false)&&
	     (hop_state != IDLE)) {//bypass send if main loop receive active (flywheel collision happenned)
 		  	  vRadio_StartTx_Variable_Packet(pRadioConfiguration->Radio_ChannelNumber, gp_rdo_tpacket_l,
	  		RADIO_PKT_LEN);
	  }else
	    radio_mon_txfailcnt++;
	  	// frequency hopping in the action, rec freq is superseded by this frequency too, liyenho
	    ctrl_hop_global_update(false);
	  radio_mon_txcnt++;
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
        vRadio_StartTx_Variable_Packet(pRadioConfiguration->Radio_ChannelNumber, gp_rdo_tpacket_l,
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
	uint32_t fw_len0, fw_len = it951x_fw_hdr[0],
						len, len1, len2, ll, ll1, crc, i,
						fw_dnld_size1;
	while (0 > (int)fw_len) {
		fw_len = it951x_fw_hdr[0];
	}
	uint8_t *fw_ite_tbuffer1 = (uint8_t*)fw_ite_tbuffer+6/*per ITE algorithm*/,
					 *pb = fw_ite_rbuffer,
					 *pe = pb+fw_dnld_size,
					 Cmd_sequence = 0/*it should be alright...*/;
#include "type.h"
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
			uint32_t tmp, wtmp, *pth = &tmp;
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
/*****************************************************/
				/* stop usb device operation */
				udc_stop();
				/* run application */
				_app_exec(APP_START);
			}
			if (2 == system_upgrade) {
				// atmel upgrade
//				download_atmel_fw( upgrade_fw_hdr );
				wdt_init(WDT, WDT_MR_WDRPROC, 256/*1 sec*/, 0xfff) ;
				while (1); // wait for processor reset
			}
	}
// to enable embedded ITE asic  video subsystem ***********
extern int init_video_subsystem();
 volatile bool init_video_flag = false;
extern int start_video_subsystem();
 volatile bool start_video_flag = false;
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
	uint32_t tdel, tcurr, n, i;
	uint32_t startup_video_tm,
						last_done_spi;
	int32_t /*tm_const_spi1 = //time spent per TS block goes thru spi xfer
							120*1000000*(int64_t)I2SC_BUFFER_SIZE*8/(int64_t)gs_ul_spi_clock[1],*/
					// due to largest interleaver inside DVB-T taking 11 packets latency
					tm_const_tsb =  //time spent per (12.5>11) TS packet goes thru video pipe @ (4<4.524) mb/s
							120*1000000*(int64_t)188*(12.5)*8/(int64_t)(4*1000000);
  	bool vid_ant_switch1 = false;
#ifdef TIME_ANT_SW
	unsigned int tick_prev_antv, tick_curr_antv;
	int tick_del_antv = 0;
#endif
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
	//ui_init();
	delay_ms(10); //atmelStudio fail debug breakpoint
	//ui_powerdown();
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
	NVIC_SetPriority(SPI_IRQn, /*0*/2);
	NVIC_EnableIRQ(SPI_IRQn);
#endif
	NVIC_DisableIRQ(SPI0_IRQn);  // spi0 peripheral instance = 8, liyenho
	NVIC_ClearPendingIRQ(SPI0_IRQn);
	NVIC_SetPriority(SPI0_IRQn, /*0*/1);
	NVIC_EnableIRQ(SPI0_IRQn);
 #ifdef RADIO_SI4463
	NVIC_DisableIRQ(PIOA_IRQn);  // pioa radio instance = 11, liyenho
	NVIC_ClearPendingIRQ(PIOA_IRQn);
	NVIC_SetPriority(PIOA_IRQn, /*0*/1);
	NVIC_EnableIRQ(PIOA_IRQn);
	NVIC_DisableIRQ(SPI2_IRQn);  // spi2 peripheral instance = 8, liyenho
	NVIC_ClearPendingIRQ(SPI2_IRQn);
	NVIC_SetPriority(SPI2_IRQn, /*0*/1);
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
	twi_master_initialize(TWI_CLK); // communicate with it951x, liyenho
	spi_master_initialize(0, SPI0_MASTER_BASE, BOARD_FLEXCOM_SPI0);// 2072 ctrl pipe
  #ifdef RADIO_SI4463
	spi_master_initialize(2, SPI2_MASTER_BASE, BOARD_FLEXCOM_SPI2);// radio data pipe
  #endif
	spi_master_initialize(1, SPI_MASTER_BASE, BOARD_FLEXCOM_SPI);// video pipe @ tx end
	  pio_configure(PIOA, PIO_OUTPUT_1, PIO_PA28, 0); // manually controlled cs @ spi1
#ifdef CONFIG_ON_FLASH
		if (1!=*(uint8_t*)ul_page_addr_bootapp) {
			erase_last_sector() ;
			CHECKED_FLASH_WR(
				IFLASH_ADDR + IFLASH_SIZE-sizeof(backup),
				backup, NUM_OF_FPGA_REGS +1 +4 +2)
			//bootapp = 0; // tx board behaved as usual, so allow bootloader switch on udc
			bootapp = 1; // boot right into app
			CHECKED_FLASH_WR(ul_page_addr_bootapp, &bootapp, 1)
			CHECKED_FLASH_WR(
				IFLASH_ADDR + IFLASH_SIZE-NUM_OF_ATMEL_REGS +4,
				backup +NUM_OF_FPGA_REGS +1 +4 +4,
				sizeof(backup)-NUM_OF_FPGA_REGS-1 -4 -4)
		}
#endif
system_restart:  // system restart entry, liyenho
	system_main_restart = false;
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
 #ifdef CONFIG_RF2072
 //Force PA switch (PA23, PA24) Picks the inner rf port, can't afford to wait for host, in case it's not present
   pio_set_output(PIOA, PIO_PA23, HIGH, DISABLE, ENABLE);
	pio_set_output(PIOA, PIO_PA24, LOW, DISABLE, ENABLE);
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
		if (!usb_read_buf(gs_uc_tbuffer))
			while (1);	// it can't happen...
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
		if (system_upgrade)
			upgrade_sys_fw(system_upgrade);
 	}
 #endif
 #ifdef CONFIG_RF2072
  	pio_set_output(PIOA, PIO_PA26, HIGH, DISABLE, ENABLE); // rf2072 out of reset
  	pio_set_output(PIOA, CPLD_2072_TRIG, HIGH, DISABLE, ENABLE); // extra trigger line for 2072 access with cpld
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
extern void recalibrate_capval (void* ul_page_addr_mtemp, uint8_t median);
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
#if /*defined(RADIO_SI4463)*/ false
	/* Initialize the console UART. */
		configure_console(); // used for generic system messages logging, liyenho
	COMPILER_WORD_ALIGNED
	static usb_cdc_line_coding_t uart_coding; // used for si446x radio dev, liyenho
	  uart_coding.dwDTERate = CPU_TO_LE32(UDI_CDC_DEFAULT_RATE);
	  uart_coding.bCharFormat = UDI_CDC_DEFAULT_STOPBITS;
	  uart_coding.bParityType = UDI_CDC_DEFAULT_PARITY;
	  uart_coding.bDataBits = UDI_CDC_DEFAULT_DATABITS;
	// re-config/open for si4463 ctrl with uart port, liyenho
		uart_config(0, &uart_coding);
		uart_open(0);
 #if 0 //def UART_TEST
		int c = getchar();
		puts("hello");
 #endif
#endif
#ifdef RADIO_SI4463
	r4463_sts.tick_prev = tick_prev = *DWT_CYCCNT;
#endif
#ifdef TIME_ANT_SW
 #if false  // because of stupid usb hub in front of atmel, it never worked as rx does...
  extern void configure_rtt(unsigned int clkcnt);
	configure_rtt(0/*16*/); // arm 2 sec isr toggle video ant
 #else  // workaround the stupid usb hub
 	tick_prev_antv = *DWT_CYCCNT;
 #endif
#endif
	// The main loop manages only the power mode
	// because the USB management is done by interrupt
	while (true) {
		if (system_main_restart ) goto system_restart;
	if (system_upgrade)
		upgrade_sys_fw(system_upgrade);
		if (init_video_flag) {
			init_video_subsystem();
			init_video_flag = false;
		}
		else if (start_video_flag) {
			start_video_subsystem();
			start_video_flag = false;
		}
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
 	// Bill shall utilize these funtions to pull or push actual
 	// control packets from inbound and outbound while
 	// outbound traffic can be variable length, with
 	// maximum length of (120 - 6) bytes, how to fit these
 	// into fixed length control packets is not in the scope
 	// of these functions
 #ifdef UART_TEST
 	ctrl_buffer_send_ur(rpacket_grp); // send constant 30 bytes to host
  	ctrl_buffer_recv_ur(tpacket_grp);  // receive variable bytes from host
 #endif
#endif
		if (!stream_flag) goto _reg_acs; // stop TS stream if flag isn't true, liyenho
#ifdef TIME_ANT_SW
		else /*if (stream_flag)*/ {
			tick_curr_antv = *DWT_CYCCNT;
			tick_del_antv= timedelta(timedelta_reset,
																	tick_curr_antv,
																	tick_prev_antv);
			// toggle every three seconds
			if (3*120000000<tick_del_antv)	{
				vid_ant_switch = true; // activate vid ant sw
				tick_prev_antv = tick_curr_antv;
			}
		}
#endif
		// start usb rx line
#ifndef CTRL_RADIO_ENCAP
		pusb = (1 & usbfrm) ?((uint8_t*)gs_uc_tbuffer)+I2SC_BUFFER_SIZE : gs_uc_tbuffer;
#else
												/* adapt to requirement of control radio data encapsulation */
		pusb = (1 & usbfrm) ?((uint8_t*)gs_uc_tbuffer)+I2SC_BUFFER_SIZE+TP_SIZE : gs_uc_tbuffer;
#endif
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
		if (!usb_read_buf(pusb))
			goto _reg_acs ;
  #endif
  #if defined(TEST_USB) || defined(TEST_SPI)
	 volatile uint8_t *pusb1;
	   pusb1 = (1 & usbfrm) ?((uint8_t*)gs_uc_rbuffer)+I2SC_BUFFER_SIZE : gs_uc_rbuffer;
  #endif
  #ifndef TEST_USB
  	#ifndef TEST_SPI
		 if (vid_ant_switch1) {
			 int delta;
				bool state= pio_get(PIOA, PIO_OUTPUT_1, PIO_PA24);
			 /*do not disrupt the service by connecting both antenna*/
			 pio_set(PIOA, PIO_PA23);
			 pio_set(PIOA, PIO_PA24);
			 // survey time interval for requested period of time
			 last_done_spi = *DWT_CYCCNT;
			 delta=timedelta(timedelta_reset,
			 									last_done_spi,
			 									startup_video_tm);
			 while (tm_const_tsb>delta) {
			 	/*wait until it is sure a block of TS has been delivered thru air*/
			 	delay_ms(1);
			 	last_done_spi = *DWT_CYCCNT;
			 	delta=timedelta(timedelta_reset,
			 									last_done_spi,
			 									startup_video_tm);
		 	 }
			// switch between two antenna only takes 0.96 usec,
			// but end result is amazingly profound, it always
			// corrupts video on the other end, sooner or later
			 {
				if (state) {
					pio_clear(PIOA, PIO_PA24);
				}
				else {
	   			pio_clear(PIOA, PIO_PA23);
				}
				vid_ant_switch = false;
			}
			vid_ant_switch1 = false;
		 }
#ifdef VIDEO_DUAL_BUFFER
		static uint32_t pre_tbuffer[I2SC_BUFFER_SIZE/4];
		const uint32_t *pte =pre_tbuffer+sizeof(pre_tbuffer)/4;
		static uint32_t *ptw=pre_tbuffer,
										*ptr = pre_tbuffer;
		static bool first_blk_vid = true;
		if (first_blk_vid) {
			uint32_t *ptb = pre_tbuffer ;
	  		usb_data_done = true;
	  		for (n=0; n<I2SC_BUFFER_SIZE/188; n++) {
				spi_tx_transfer(pusb, 188,
					gs_uc_rbuffer/*don't care*/, 188, 1/*video*/);
				memcpy(ptb, pusb, 188);
				ptb += 188/4;
				while (usb_data_done) ; // usb pipe overflow, liyenho
				usb_data_done = true;
				pusb += 188;  // accommodate cpld to generate ts_syn/ts_valid, liyenho
			}
			//ptw = pre_tbuffer;  // wrap write ptr at end, it has been done...
			first_blk_vid = false;
			goto next;
		}
#endif
  		usb_data_done = true;
  		for (n=0; n<I2SC_BUFFER_SIZE/188; n++) {
			spi_tx_transfer(pusb, 188,
				gs_uc_rbuffer/*don't care*/, 188, 1/*video*/);
			while (usb_data_done) ; // usb pipe overflow, liyenho
			usb_data_done = true;
#ifdef VIDEO_DUAL_BUFFER
	/*apply write/read pointer update scheme in case we later use different size of TS pre-buffer other than 10*188*/
  #define _PTR_UPD_(p,i,e,b) \
  						p += i; \
  						if (e<=p) \
  							p = b;
			spi_tx_transfer(ptr, 188,
				gs_uc_rbuffer/*don't care*/, 188, 1/*video*/);
			_PTR_UPD_(ptr, 188/4, pte, pre_tbuffer);
			while (usb_data_done) ; // usb pipe overflow, liyenho
			memcpy(ptw, pusb, 188);
			_PTR_UPD_(ptw, 188/4, pte, pre_tbuffer);
	#undef _PTR_UPD_
			usb_data_done = true;
#endif
			pusb += 188;  // accommodate cpld to generate ts_syn/ts_valid, liyenho
		}
		startup_video_tm = *DWT_CYCCNT;
		if (vid_ant_switch)
			vid_ant_switch1 = true;
#ifdef CTRL_RADIO_ENCAP
			if (ctrl_tdma_enable) {
				uint8_t bs = fill_radio_pkt(pusb);
				if (bs) {// ctrl data filled
					*(pusb+6) = bs;
					const int hdr_sz=/*sizeof(ts_rdo_hdr)*/7;
					memset(pusb+hdr_sz+bs,
									0xff /*stuff bytes*/,
									188-hdr_sz-bs);
					spi_tx_transfer(pusb, 188,
						gs_uc_rbuffer/*don't care*/, 188, 1/*video*/);
					while (usb_data_done) ;
				}
			}
#endif
   #else
		spi_tx_transfer(pusb, I2SC_BUFFER_SIZE,
			pusb1, I2SC_BUFFER_SIZE, 1/*video*/);
		while (usb_data_done) ; // usb pipe overflow, liyenho
		usb_data_done = true;
   #endif
  #endif
  #if defined(TEST_USB) || defined(TEST_SPI)
#ifdef TEST_SPI
		while (usb_data_done) ;
#else
  		delay_us(800); // a bit delay to lower bit rate
  		memcpy(pusb1,pusb,I2SC_BUFFER_SIZE);
	   usb_data_done = false;
#endif
   #ifndef MEDIA_ON_FLASH
  		usb_write_buf(pusb1,I2SC_BUFFER_SIZE);
   #endif
  #endif
next:
		usbfrm = usbfrm + 1;
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
			uint8_t *pdbg = pr->data;  // to watch data content
			#include <assert.h>
			switch (pt->access) {
				case RF2072_RESET:
						pio_clear (PIOA, PIO_PA26);
						delay_ms(200);
						pio_set (PIOA, PIO_PA26);
						//delay_us(1);
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
				case IT951X_READ:
							_TWI_READ_(pr->addr,pr->data,pr->dcnt)
							break;
				case IT951X_WRITE:
							TWI_WRITE(pt->addr,pt->data,pt->dcnt)
							break;
				case IT951X_DOWNLOAD:
							download_ite_fw(pt->dcnt, pt->addr, pt->data );
							break;
				case TS_VID_ACTIVE:
								// now allow video spi to be active, liyenho
								stream_flag = true;
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
	return ;
	//ui_powerdown();
}

void main_resume_action(void)
{
	return ;
	//ui_wakeup();
}

volatile bool main_usb_host_msg() // Attach this api to usb rx isr service, liyenho
{
	if (USB_HOST_MSG_TX_VAL != udd_g_ctrlreq.req.wValue) {
		if (USB_ATMEL_UPGRADE_VAL != udd_g_ctrlreq.req.wValue
		&& USB_ITE_FW_VAL != udd_g_ctrlreq.req.wValue
		)
		return false ;
		else goto payload_next;
	}
	if (USB_HOST_MSG_IDX != udd_g_ctrlreq.req.wIndex)
		return false ;
	if (USB_HOST_MSG_LEN > udd_g_ctrlreq.req.wLength	)
		return false ;
payload_next:
	if (USB_ITE_FW_VAL == udd_g_ctrlreq.req.wValue
		&& ITE_FW_HDR_LEN != udd_g_ctrlreq.req.wLength)
		return false ;
	if (USB_HOST_MSG_TX_VAL == udd_g_ctrlreq.req.wValue &&
			0!=ctx_951x.it951x_access &&	(!ctx_951x.i2c_done_rd || !ctx_951x.i2c_done_wr))
		{
			udd_set_setup_payload( gs_uc_htbuffer_tm, udd_g_ctrlreq.req.wLength);
			return; // don't interfere with cur task
		}
	else
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
		volatile dev_access *ps = gs_uc_hrbuffer;
	if (USB_HOST_MSG_LEN-sizeof(ps->data[0]) == udd_g_ctrlreq.req.wLength)
		// (0==ps->dcnt) meant confirmation of write cmd, liyenho
		cnf_echo = 1;
		// must setup packet size
		if (!cnf_echo) {
			udd_set_setup_payload( gs_uc_hrbuffer,
				USB_HOST_MSG_LEN+(ps->dcnt-1)*sizeof(uint8_t));
			if(ps->access == IT951X_READ) {
				if(ctx_951x.i2c_done_rd == false) {
					((dev_access*)gs_uc_hrbuffer)->access= (uint8_t)-128;
				}
				else if (ctx_951x.it951x_err_rd !=TWI_SUCCESS) {
					((dev_access*)gs_uc_hrbuffer)->access= (uint8_t)-127;
				}
	        else if(ctx_951x.it951x_access !=IT951X_READ) {
	          //error: status request, when read request not yet initiated.
	          ((dev_access*)gs_uc_hrbuffer)->access=(uint8_t)-126;
	        }
				else {
   	       ctx_951x.it951x_access = 0; // invalidate for next access
       	  }
			}
		} else { //confirmation by echo, 1 data entry is included
			udd_set_setup_payload( gs_uc_hrbuffer,
				USB_HOST_MSG_LEN-sizeof(ps->data[0]));
			if(ps->access == IT951X_WRITE) {
				if(ctx_951x.i2c_done_wr == false) {
					((dev_access*)gs_uc_hrbuffer)->access= (uint8_t)-128;
				}
				else if (ctx_951x.it951x_err_wr !=TWI_SUCCESS) {
					((dev_access*)gs_uc_hrbuffer)->access= (uint8_t)-127;
				}
				else if(ctx_951x.it951x_access !=IT951X_WRITE) {
					//error: status request, when write request not yet initiated.
					((dev_access*)gs_uc_hrbuffer)->access=(uint8_t)-126;
				}
				else {
					ctx_951x.it951x_access = 0; // invalidate for next access
				}
			}
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

/**********************************************************/
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
/**********************************************************/
	if (USB_INIT_VID_SUBSYS == udd_g_ctrlreq.req.wValue) {
		if (!init_video_subsystem()) return true;
		else return false ;
	}
	if (USB_START_VID_SUBSYS == udd_g_ctrlreq.req.wValue) {
		if (!start_video_subsystem()) return true;
		else return false ;
	}
	if (USB_VID_ANT_SWITCH == udd_g_ctrlreq.req.wValue) {
		vid_ant_switch = true; // host request to switch video antenna
	}
/**********************************************************/
	if (USB_HOST_MSG_TX_VAL == udd_g_ctrlreq.req.wValue ||
		USB_ATMEL_UPGRADE_VAL == udd_g_ctrlreq.req.wValue
		|| USB_ITE_FW_VAL == udd_g_ctrlreq.req.wValue
		)
		return main_usb_host_msg();
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
	//ui_process(udd_get_frame_number());
}

#ifdef USB_DEVICE_LPM_SUPPORT
void main_suspend_lpm_action(void)
{
	return ;
	//ui_powerdown();
}

void main_remotewakeup_lpm_disable(void)
{
	return ;
	//ui_wakeup_disable();
}

void main_remotewakeup_lpm_enable(void)
{
	return ;
	//ui_wakeup_enable();
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
