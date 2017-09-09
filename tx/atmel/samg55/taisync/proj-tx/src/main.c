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
#ifdef SEND_TAISYNC
 #include "taisync.h"
#endif
/*static*/ volatile bool system_main_restart = false;  // system restart flag, liyenho
static volatile uint8_t system_upgrade = 0;  // system upgrade flag, liyenho
/*static*/ volatile char __version_atmel__[3];  // stored as mon/day/year

#include "ctrl.h"
#include "Radio_Buffers.h"
#include "USB_Commands.h"
#include "ReedSolomon.h"
#include "Antenna_Diversity.h"

volatile uint8_t main_loop_on = false;  // run time indicator
volatile uint8_t usb_data_done = false;  // workaround flooding i2s intr issue, liyenho
volatile uint8_t usb_host_msg = false, spi_tgt_done=false; // triggered by usb rx isr for host (fpga/sms) comm, liyenho
//engineering debug support
volatile uint32_t *DWT_CONTROL = (uint32_t *)0xE0001000;
volatile uint32_t *DWT_CYCCNT = (uint32_t *)0xE0001004;
volatile uint32_t *DEMCR = (uint32_t *)0xE000EDFC;

static volatile bool main_b_cdc_enable = false;

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
  static uint32_t gs_ul_spi_clock[2/*+1*/] = {  // shall be higher than usb data rate
  				  	2000000, 8000000/*, RADIO_SPI_BR*/}; // (ctrl/sts, video) spi bit rate, radio is on uart

extern volatile bool udi_cdc_data_running; // from udi_cdc.c, liyenho
#ifdef  RADIO_TAISYNC
  volatile bool ctrl_radio_started = false; // radio startup flag...
  volatile bool ctrl_tdma_enable = false;
/*******************************************************************/
  volatile bool ctrl_tdma_lock = false;
   volatile uint8_t dbg_antpos;

	static int timedelta(bool reset, unsigned int bignum, unsigned int smallnum);
	/*static*/ bool timedelta_reset, timedelta_reset_rx; //to handle system restart
	volatile bool fhop_in_search= false, fhop_dir;
  unsigned int tdma_sndthr=0;
  /*static*/ enum pair_mode hop_state;
#endif
volatile bool stream_flag = false; // wait for host to signal TS stream on

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
volatile static uint32_t g_ul_led_ticks=0;
uint32_t g_ul_wait_100ms=50, g_ul_wait_250ms=125, g_ul_wait_500ms=250, g_ul_wait_1s=500;
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
static uint32_t gs_uc_htbuffer[(USB_HOST_MSG_LEN+HOST_BUFFER_SIZE+3)/sizeof(int)];
uint32_t gs_uc_hrbuffer[(USB_HOST_MSG_LEN+HOST_BUFFER_SIZE+3)/sizeof(int)];

   // host/sms ctrl/sts buffer
   volatile uint32_t stm32_fw_hdr[1]={-1} ; //byte lenght of firmware
 #if true  // add two big memory chunks for ite fw download
   /*static*/ uint32_t fw_stm_tbuffer[FW_DNLD_SIZE/sizeof(int)];
   /*static*/ uint32_t fw_stm_rbuffer[FW_DNLD_SIZE/sizeof(int)];
 #endif
/* Pointer to UART PDC register base */
Pdc *g_p_spim_pdc [/*1+*/2]/*fpga/sts, video, liyenho*/,*g_p_spis_pdc;
Pdc *g_p_i2st_pdc, *g_p_i2sr_pdc;

#ifdef RADIO_TAISYNC
volatile uint32_t tpacket_idle[RDO_ELEMENT_SIZE];
unsigned char gs_rdo_tpacket_ovflw=0;
volatile uint32_t rpacket_idle[ASYMM_RATIO* RDO_ELEMENT_SIZE];
uint32_t rpacket_ov[RDO_ELEMENT_SIZE]; //gs_rdo_rpacket fifo overflow holder
unsigned char gs_rdo_rpacket_ovflw=0;

uint32_t radio_mon_txfailcnt=0;
uint32_t radio_mon_rxerr = 0;
uint32_t radio_mon_txcnt = 0;
uint32_t rxnorec_intv=0;  // not a count but interval
uint32_t no_recive_cnt =0;
uint32_t no_receive_reset_cnt =0;
uint32_t hop_watchdog_reset_cnt= 0;
uint32_t hop_watchdog_intv= 0;

unsigned char snd_asymm_cnt=0;
static uint32_t tick_curr, tick_prev;

  // 4463 stats mon obj
  extern volatile ctrl_radio_stats  ctrl_sts;
#endif //RADIO_TAISYNC

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

	if (1 == ch) {// cs enabled @ spi5
 		delay_cycles(0.25*120/8*8); // assumed 120 mhz atmel clk, 8 mhz spi clk, and 8 bit data
		pio_clear(PIOA, PIO_PA11);  // must be called prior to pdc_xx_init, liyenho
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
	spi_enable_interrupt(base, spi_ier) ;
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
void SPI_Handler(void)  // video spi pipe
{
	uint32_t status;
	spi_disable_interrupt(SPI_MASTER_BASE, SPI_IER_TXBUFE) ;
 #ifdef TEST_SPI
	spi_disable_interrupt(SPI_MASTER_BASE, SPI_IER_RXBUFF) ;
 #endif
 	delay_cycles(0.25*120/8*8); // assumed 120 mhz atmel clk, 8 mhz spi clk, and 8 bit data
	 pio_set (PIOA, PIO_PA11); // cs disabled @ spi5
	usb_data_done = false;  // handshake with mainloop
	status = spi_read_status(SPI_MASTER_BASE) ;

	//if(status & SPI_SR_NSSR) {
		if ( status & SPI_SR_TXBUFE ) {
//			printf("transfer done\n");
		}
	//}
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
	spi_set_clock_polarity(base, SPI_CHIP_SEL, 1/*clk idle state is high*/);
	spi_set_clock_phase(base, SPI_CHIP_SEL, 0/*captured @ rising, transit @ falling*/);
 #if true
 	if (1 == ch)
	 	spi_set_clock_phase(base, SPI_CHIP_SEL, 0/*captured @ rising, transit @ falling*/);
	  spi_set_bits_per_transfer(base, SPI_CHIP_SEL,
			SPI_CSR_BITS_8_BIT);  // 8 bit spi xfer, liyenho
 #endif
	spi_set_baudrate_div(base, SPI_CHIP_SEL,
			(sysclk_get_cpu_hz() / gs_ul_spi_clock [ch]));
	  spi_set_transfer_delay(base, SPI_CHIP_SEL, SPI_DLYBS/*delay between bytes*/,
			SPI_DLYBCT/*delay between spi xfer*/);
	spi_enable(base);
#ifdef TEST_SPI
	spi_enable_loopback(base);
#endif
	pdc_disable_transfer(g_p_spim_pdc [ch], PERIPH_PTCR_RXTDIS |
			PERIPH_PTCR_TXTDIS);
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

 #if defined(MEDIA_ON_FLASH) && !defined(NO_USB)
  static void host_usb_mda_flash_cb(void) {
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
 void erase_last_sector(void) {
		memcpy(backup, (void*)(IFLASH_ADDR + IFLASH_SIZE-sizeof(backup)), sizeof(backup));
		uint32_t er_adr =IFLASH_ADDR + IFLASH_SIZE-SECTOR_SIZE_L;
		flash_erase_sector(er_adr);
	}
  static void host_usb_cb(void) {
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
		if ((USB_STM_UPGRADE_VAL <= udd_g_ctrlreq.req.wValue &&
				USB_ATMEL_UPGRADE_VAL >= udd_g_ctrlreq.req.wValue) &&
				FW_UPGRADE_HDR_LEN >=udd_g_ctrlreq.payload_size) {
			memcpy(upgrade_fw_hdr, // set fpga fw /img hdr, liyenho
								udd_g_ctrlreq.payload,
								FW_UPGRADE_HDR_LEN);
			system_upgrade = \
				(USB_STM_UPGRADE_VAL == udd_g_ctrlreq.req.wValue)?1/*stm*/:
					(USB_FPGA_UPGRADE_VAL == udd_g_ctrlreq.req.wValue)?2/*fpga*/:3/*atmel*/;
			main_loop_on = false ; // clear up the 'upgrade flag' ;-)
			return;
		}
		dev_access *ps = gs_uc_htbuffer,*ps1 = gs_uc_hrbuffer;
		// prepare confirm msg by echo whatever received
		memcpy(gs_uc_hrbuffer, udd_g_ctrlreq.payload, USB_HOST_MSG_LEN-sizeof(ps->data[0]));
		usb_host_msg = true; // enable mainloop process
  }

void SysTick_Handler(void)
{
	//2ms interrupt routine
#ifdef RADIO_TAISYNC // RADIO_TAISYNC inside SysTick_Handler

	if (ctrl_radio_started) {
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
				(false==ctrl_tdma_lock )){
				//restart									// allow frequ hop take action during search
				tdma_sndthr=lc + (TDMA_PERIOD)/2;
				cdelta = (TDMA_PERIOD)/2;
			}
			//snd event identification -------------------------
			if(cdelta <0) {
				/*******************************************/
				/*******************************************/
				if(fifolvlcalc(wrptr_rdo_tpacket, rdptr_rdo_tpacket, RDO_TPACKET_FIFO_SIZE)<1)
				{ //idle packet case
					//add idle packet to the output queue
					Queue_Control_Idle_Packet();
				}

				//setup for radio send
				// 	rdptr_inc(&wrptr_rdo_tpacket, &rdptr_rdo_tpacket, RDO_TPACKET_FIFO_SIZE, 1);
				gp_rdo_tpacket_l = gs_rdo_tpacket + (RDO_ELEMENT_SIZE*rdptr_rdo_tpacket);
				snd_asymm_cnt = ASYMM_RATIO-1;

				if(hop_state != IDLE) { //bypass if main loop receive active (flywheel collision happened)
					if (!USE_915MHZ){
						uart_Send_Data(gp_rdo_tpacket_l, RADIO_LONG_PKT_LEN);
						snd_asymm_cnt = 0;
					}else{
						uart_Send_Data(gp_rdo_tpacket_l, RADIO_PKT_LEN);
					}
					// frequency hopping in the action, rec freq is superseded by this frequency too, liyenho
						ctrl_hop_global_update(true);
				}

				tdma_sndthr = tdma_sndthr + TDMA_PERIOD;
			}
		}
		//periodic data send
		#ifdef RADIO_CTRL_TXCONTINOUS
		ctrl_tdma_enable =false;
		#endif

	} //ctrl_radio_started

#endif // RADIO_TAISYNC inside SysTick_Handler
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
extern download_sys_fw(bool sys_hw_mask, uint32_t *upgrade_fw_hdr );
	void upgrade_sys_fw(uint8_t system_upgrade) {
			main_loop_on = true;	// to start up upgrade proc on host...
				uint32_t tmp, wtmp, *pth = &tmp;
			if (1 == system_upgrade) {
				// xilinx artix upgrade
				download_sys_fw(false/*fpga dnld*/, upgrade_fw_hdr );
				delay_ms(100);
/*****************************************************/
				/* stop usb device operation */
				udc_stop();
				/* run application */
				_app_exec(APP_START);
			}
			if (2 == system_upgrade) {
				// stm32 firmware upgrade
				download_sys_fw(true/*stm32 dnld*/, upgrade_fw_hdr );
				delay_ms(100);
/*****************************************************/
				/* stop usb device operation */
				udc_stop();
				/* run application */
				_app_exec(APP_START);
			}
			else if (3 == system_upgrade) {
				// atmel upgrade
				wdt_init(WDT, WDT_MR_WDRPROC, 256/*1 sec*/, 0xfff) ;
				while (1); // wait for processor reset
			}
	}

// to enable embedded TAISYNC video subsystem ***********
extern int init_video_subsystem(void);
 volatile bool init_video_flag = false;
extern int start_video_subsystem(void);
 volatile bool start_video_flag = false;
/*! \brief Main function. Execution starts here.
 */
int main(void)
{
	//for debugging, disable write buffer to find cause of hard fault
	//uint32_t *ACTLR = (uint32_t *)0xE000E008;
	//*ACTLR |= 2; //setting the DISDEFWBUF bit (bit 1)

	volatile uint8_t *pusbe=(uint8_t*)gs_uc_tbuffer+sizeof(gs_uc_tbuffer),
										*pusbs=(uint8_t*)gs_uc_tbuffer+0,
#ifdef CONF_BOARD_USB_TX
										*pusb=gs_uc_rbuffer;
#elif defined(CONF_BOARD_USB_RX)
										*pusb=gs_uc_tbuffer;
#endif
	uint32_t n, i;

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

#if CW_MODE//mode for generation unmoulated carrier wave
	#error Unmodulated carrier waveform is NOT supported
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
#if true // for video ts stream
	NVIC_DisableIRQ(SPI_IRQn);  // spi0 peripheral instance = 8, liyenho
	NVIC_ClearPendingIRQ(SPI_IRQn);
	NVIC_SetPriority(SPI_IRQn, 1);
	NVIC_EnableIRQ(SPI_IRQn);
#endif
	NVIC_DisableIRQ(SPI0_IRQn);  // spi5 peripheral instance = 21, liyenho
	NVIC_ClearPendingIRQ(SPI0_IRQn);
	NVIC_SetPriority(SPI0_IRQn, 1);
	NVIC_EnableIRQ(SPI0_IRQn);
 	if (SysTick_Config(sysclk_get_cpu_hz() / 500)) { // 2 msec tick
		//puts("-E- Systick configuration error\r");
		while (1) {
			// Capture error
		}
	}

	//pmc_enable_periph_clk(ID_PIOA);
	spi_master_initialize(0, SPI0_MASTER_BASE, BOARD_FLEXCOM_SPI0);// 2072 ctrl pipe
	spi_master_initialize(1, SPI_MASTER_BASE, BOARD_FLEXCOM_SPI);// video pipe @ tx end
	  pio_configure(PIOA, PIO_OUTPUT_1, PIO_PA11, 0); // manually controlled cs @ spi5
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

	// Start USB stack to authorize VBus monitoring
	udc_start();
system_restart:  // system restart entry, liyenho
	system_main_restart = false;
#ifdef  RADIO_TAISYNC
  ctrl_radio_started = false;
  ctrl_tdma_enable = false;
	ctrl_tdma_lock = false;
	timedelta_reset = timedelta_reset_rx = true;
	fhop_in_search = true;
	// start up with constant offset, we'll modify to adopt pairing reset later, liyenho
	fhop_base = 0;
    fhop_offset = HOP_2CH_ENABLE?WRAP_OFFSET(HOP_2CH_OFFSET0):fhop_base;
	fhop_dir = true;  // hop forward when startup
	fhop_idx= 0 ; // used for pattern access
  #ifdef RADIO_CTRL_AUTO
	hop_state = PAIRING;
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
		usb_read_buf1(gs_uc_tbuffer, 1880);
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
  		usb_write_buf(gs_uc_rbuffer,I2SC_BUFFER_SIZE);
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

#ifdef RADIO_CTRL_AUTO
		/* init radio stats obj */
		{
			for (int j=0; j<CTRL_CTX_LEN; j++) {
				ctrl_sts.ctrl_bits_ctx[j] = LONG_RNG;
			}
			ctrl_sts.bw_ctrl_bits = LONG_RNG;
			ctrl_sts.errPerAcc =0;
		}
		vRadio_Init();
    #ifdef RADIO_CTRL_TXCONTINOUS
    	memset(gs_rdo_tpacket, 0xc5,  2*4*RDO_ELEMENT_SIZE) ;
		if (!USE_915MHZ){
			uart_Send_Data(gs_rdo_tpacket, RADIO_LONG_PKT_LEN);
		extern unsigned char snd_asymm_cnt;
			snd_asymm_cnt = 0;
		}else{
			uart_Send_Data(gs_rdo_tpacket, RADIO_PKT_LEN);
		}
    #else
    	ctrl_hop_global_update(true);
    #endif
		ctrl_radio_started = true;
		ctrl_tdma_enable = true;
#endif //RADIO_CTRL_AUTO
 	main_loop_on = true;  // enter run time stage, liyenho

	if (system_upgrade)
		upgrade_sys_fw(system_upgrade);
#if USE_UART
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
#endif
#ifdef RADIO_TAISYNC
	ctrl_sts.tick_prev = tick_prev = *DWT_CYCCNT;
#endif
	#if FIXED_PAIR_ID
		uint8_t temp_hop_id[HOP_ID_LEN] = {1};
		Set_Pair_ID(temp_hop_id);
	#endif //FIXED_PAIR_ID
	static bool video_started = false;
	#if USE_UART

	if (!video_started){
		init_video_flag = true;
	}
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
			start_video_flag = true;
		}
		else if (start_video_flag) {
			start_video_subsystem();
			start_video_flag = false;
			video_started = true;
		}
#ifdef RADIO_TAISYNC

    //tdd lock time out routine ----------------------------------------
	tick_curr = *DWT_CYCCNT;
	if (tick_curr < tick_prev) {
	 	rxnorec_intv+=tick_curr+(UINT32_MAX-tick_prev);
		hop_watchdog_intv+=tick_curr+(UINT32_MAX-tick_prev);
		 idle_msg_queue_intv +=tick_curr+(UINT32_MAX-tick_prev);
	}
	else {
	 	rxnorec_intv += tick_curr -tick_prev;
		hop_watchdog_intv += tick_curr -tick_prev;
		idle_msg_queue_intv +=tick_curr+(UINT32_MAX-tick_prev);
	}

	tick_prev = tick_curr;
	if(rxnorec_intv> TDMA_UNLOCK_DELAY_MS) { //no rx unlock routine
	  	rxnorec_intv=0;
		ctrl_tdma_lock = false;
		fhop_in_search = true;
	}
	if (ctrl_tdma_lock && (hop_watchdog_intv>TDMA_UNLOCK_DELAY_MS)){ //hopping not completely synced
		//hop forward in attempt to re-establish lock
		ctrl_hop_global_update(false);
		ctrl_hop_global_update(true);

		ctrl_tdma_lock = false;
		fhop_in_search = true;
		hop_watchdog_reset_cnt++;
	}
	//static bool control_channel_scan_complete = false;
	 //if (!control_channel_scan_complete){
		 //control_channel_scan_complete = Channel_Scan();
	 //}

#ifdef CTRL_RADIO_ENCAP
	if (idle_msg_queue_intv> MilliSec_To_Tick(1000)){
		//attempt to send idle message every 1 sec
		//if Si4463 is used to send radio packets, this is handled automatically
		Queue_Control_Idle_Packet();
		idle_msg_queue_intv = 0;
	}
#endif  //CTRL_RADIO_ENCAP
 #ifdef CTRL_DYNAMIC_MOD
	process_range_mode(tick_curr, tick_prev);
 #endif
#if USE_UART
	Process_MavLink_Raw_Data();
 #endif
 #if RECEIVE_MAVLINK
	Process_MavLink_Raw_Radio_Data();

	//if a MavLink packet is waiting, send to host
	MavLinkPacket pkt;
	if (Get_MavLink(&incoming_messages,&pkt)){
		//ToDo: add support for handling other types of messages
		//e.g. messages that contain instructions for Atmel
		uart_send_Mavlink(pkt);
		memset(&pkt, 0x0, sizeof(pkt));
	}
 #endif
#endif
		if (!stream_flag) goto _reg_acs; // stop TS stream if flag isn't true, liyenho
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
#ifdef CTRL_RADIO_ENCAP
		uint8_t blocks_used = Insert_Control_In_TSStream(pusb);
		if (blocks_used){
			//only do transfer if there is data to send
			spi_tx_transfer(pusb, 188*blocks_used, gs_uc_rbuffer/*don't care*/, 188*blocks_used, 1/*video*/);
					while (usb_data_done) ;
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
#if defined(SEND_TAISYNC)
			dev_access *pr=(dev_access*)gs_uc_hrbuffer, *pt = (dev_access*)gs_uc_htbuffer;
			uint8_t tmpb, *pdbg = pr->data;  // to watch data content
			uint16_t tmphw;
			uint32_t tmpw;
			#include <assert.h>
  #define SANITY_CHECK(ubnd,lbnd,var,type) \
						var = *(type*)(pt->data); \
						if ((type)ubnd<var ||(type)lbnd>var) \
							assert(0); /* continue;*/
			switch (pt->access) {
				case OFDM_BW :
								 SANITY_CHECK(FORTY_MHZ, FIVE_MHZ,
								 											tmpb, uint8_t)
								set_radio_bandwidth((VCH_BW)tmpb);
								break;
				case OFDM_CARR:
//								 SANITY_CHECK(MAX_VID_CH_F, MIN_VID_CH_F,
//								 											tmphw, uint16_t)
								tmphw = *(uint16_t*)pt->data; // allow 2392 mhz for now...
								set_radio_frequency((uint32_t)tmphw);
								break;
				case RDO_ANT :
								SANITY_CHECK(true, false, tmpb, bool)
								enable_radio_antenna((bool)tmpb);
								break;
				case CMD_STS :
								get_radio_cmd_sts(&tmpb);
								*(uint8_t*)pr->data = tmpb;
								break;
				case OFDM_MCS:
								SANITY_CHECK(QAM64_5B6, BPSK_1B2,
																			tmpb, VCH_MD)
								set_radio_modulation((VCH_MD)tmpb);
								break;
				case RDO_PWR :
								SANITY_CHECK(0, MAX_RDO_PWR,
																			tmpb, uint8_t)
								set_radio_power((uint32_t)tmpb);
								break;
				case UPLK_BUF_STS :
								get_radio_uplk_buf(&tmphw);
								*(uint16_t*)pr->data = tmphw;
								break;
				case TOTAL_FRMS_UP :
								get_radio_uplk_frms(&tmpw);
								*(uint32_t*)pr->data = tmpw;
								break;
				case LOST_FRMS_TX :
								get_radio_uplk_lost_frms(&tmpw);
								*(uint32_t*)pr->data = tmpw;
								break;
				case RDO_SNR :
								get_radio_snr(&tmpb);
								*(uint8_t*)pr->data = tmpb;
								break;
				case LDPC_FAILED :
								get_rdo_ldpc_failed(&tmpw);
								*(uint32_t*)pr->data = tmpw;
								break;
				case RDO_RSSI :
								get_radio_rssi(&tmpb);
								*(uint8_t*)pr->data = tmpb;
								break;
				case RDO_RX_VGA :
								get_radio_rx_vga(&tmpb);
								*(uint8_t*)pr->data = tmpb;
								break;
				case RX_BB_STS :
								get_radio_bb_sts(&tmpb);
								*(uint8_t*)pr->data = tmpb;
								break;
				case DNLK_BUF_STS :
								get_radio_dnlk_buf(&tmpb);
								*(uint8_t*)pr->data = tmpb;
								break;
				case TOTAL_FRMS_DN :
								get_radio_dnlk_frms(&tmpw);
								*(uint32_t*)pr->data = tmpw;
								break;
				case LOST_FRMS_RX :
								get_radio_dnlk_lost_frms(&tmpw);
								*(uint32_t*)pr->data = tmpw;
								break;
				default: /* host msg in error */
							continue;
			}
#endif //SEND_TAISYNC
  		}
#if 0
		sleepmgr_enter_sleep();
#endif
		if(ctrl_tdma_lock){
			//disable for testing purposes
			//check to see if FEC should be requested
			//if ((FRR_Copy[0] < FEC_ON_RSSI_THRESHOLD) && !Requested_FEC_On){
				//Requested_FEC_On = Queue_FEC_Request_On_Packet;
				//}else if ((FRR_Copy[0] > FEC_OFF_RSSI_THRESHOLD) && Requested_FEC_On){
				//if (Queue_FEC_Request_Off_Packet()){
					//Requested_FEC_On = false;
				//}
			//}
		}
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

volatile bool main_usb_host_msg(void) // Attach this api to usb rx isr service, liyenho
{
	if (USB_HOST_MSG_TX_VAL != udd_g_ctrlreq.req.wValue &&
		 USB_STM_UPGRADE_VAL != udd_g_ctrlreq.req.wValue &&
		 USB_FPGA_UPGRADE_VAL != udd_g_ctrlreq.req.wValue &&
		 USB_ATMEL_UPGRADE_VAL != udd_g_ctrlreq.req.wValue
		)
		return false ;
	if (USB_HOST_MSG_IDX != udd_g_ctrlreq.req.wIndex)
		return false ;
	if (USB_HOST_MSG_LEN > udd_g_ctrlreq.req.wLength
		&& sizeof(short) != udd_g_ctrlreq.req.wLength
		&& sizeof(uint32_t) != udd_g_ctrlreq.req.wLength
		)
		return false ;
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
		volatile dev_access *ps = gs_uc_hrbuffer;
	if (USB_HOST_MSG_LEN-sizeof(ps->data[0]) == udd_g_ctrlreq.req.wLength)
		// (0==ps->dcnt) meant confirmation of write cmd, liyenho
		cnf_echo = 1;
		// must setup packet size
		if (!cnf_echo) {
			udd_set_setup_payload( gs_uc_hrbuffer,
				USB_HOST_MSG_LEN+(ps->dcnt-1)*sizeof(uint8_t));
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
/**********************************************************/
	if (USB_HOST_MSG_TX_VAL == udd_g_ctrlreq.req.wValue ||
		USB_STM_UPGRADE_VAL == udd_g_ctrlreq.req.wValue ||
		USB_FPGA_UPGRADE_VAL == udd_g_ctrlreq.req.wValue ||
		USB_ATMEL_UPGRADE_VAL == udd_g_ctrlreq.req.wValue)
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
#ifdef  RADIO_TAISYNC
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

