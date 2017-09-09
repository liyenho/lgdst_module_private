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
#include "taisync.h"
#include "uart.h"
#include "delay.h"
#if (SAMG55)
#include "flexcom.h"
#endif

static volatile bool system_main_restart = false;  // system restart flag, liyenho
static volatile uint8_t system_upgrade = 0;  // system upgrade flag, liyenho
/*static*/ char __version_atmel__[3];  // stored as mon/day/year

#include "ReedSolomon.h"
#include "ctrl.h"
#include "USB_Commands.h"
#include "GPS.h"
#include "Radio_Buffers.h"

volatile uint8_t main_loop_on = false; // run time indicator
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
static volatile bool main_b_cdc_enable = false;

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
  static uint32_t gs_ul_spi_clock[2/*+1*/] = {  // shall be higher than usb data rate
  				  	2000000, 8000000/*, RADIO_SPI_BR*/}; // (ctrl/sts, video) spi bit rate, radio is on uart

extern volatile bool udi_cdc_data_running; // from udi_cdc.c, liyenho
#ifdef  RADIO_TAISYNC
  volatile bool ctrl_radio_started = false; // radio startup flag...
  volatile bool ctrl_tdma_enable = false,
  								ctrl_tdma_enable_shdw;  // shadow of ctrl_tdma_enable
  volatile bool ctrl_tdma_lock = false;  //always, since RX is master
  //volatile bool ctrl_pkt_rd_available = false;
	/*static*/ int timedelta(bool reset, unsigned int bignum, unsigned int smallnum);
	static bool fhop_dir, timedelta_reset ; //to handle system restart
	/*static*/ uint8_t hop_id[HOP_ID_LEN]; // 10 byte hop id from host

  /*static*/ enum pair_mode hop_state;
	bool control_channel_scan_complete = false;
#endif
volatile bool stream_flag = true; // default to TS stream automatic on
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
static bool health_led_onoff = false;
volatile bool usb_write_start = false ; // called back from udc.c, liyenho
#ifdef RECV_TAISYNC
	uint32_t gs_uc_rbuffer[GRAND_BUFFER_SIZE/sizeof(int)];
  bool first_in = true ;
 #ifdef WITH_ANT_SWITCH
  extern volatile bool reset_ts_count_error;
  extern volatile bool gl_vid_ant_sw;
 #endif
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
struct i2s_dev_inst dev_inst_i2s;
#ifdef RADIO_TAISYNC
unsigned char gs_rdo_tpacket_ovflw=0;
unsigned char snd_asymm_cnt=0;

volatile uint32_t rpacket_idle[RDO_ELEMENT_SIZE];
uint32_t rpacket_ov[RDO_ELEMENT_SIZE];

unsigned int radio_mon_txcnt = 0;
uint32_t pkt_start_tick=0;
  // 4463 stats mon obj
  extern volatile ctrl_radio_stats  ctrl_sts;
#endif //RADIO_TAISYNC

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
	spi_enable_interrupt(base, spi_ier) ;
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
	spi_enable_interrupt(base, spi_ier) ;
}
#ifdef RECV_TAISYNC
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
	spi_disable_interrupt(SPI_MASTER_BASE, SPI_IER_RXBUFF) ;
	spidma_active = false;  // handshake with mainloop
	mon_spidmachainfail_cnt++;
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
uint8_t bMain_IT_Status;  // kept as global var from radio isr module, liyenho

void SysTick_Handler(void)
{
	static uint32_t ul_2ms_ticks = 0;
	ul_2ms_ticks ++;
#ifdef  RADIO_TAISYNC
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
		//make sure there are enough messages to send
		while (fifolvlcalc(wrptr_rdo_tpacket, rdptr_rdo_tpacket, RDO_TPACKET_FIFO_SIZE)<3){
			Queue_Control_Idle_Packet();
		}

		//setup for radio send
		//rdptr_inc(&wrptr_rdo_tpacket, &rdptr_rdo_tpacket, RDO_TPACKET_FIFO_SIZE, 1);
		gp_rdo_tpacket_l = gs_rdo_tpacket + (RDO_ELEMENT_SIZE*rdptr_rdo_tpacket);
		snd_asymm_cnt = ASYMM_RATIO-1;

		if(hop_state!= IDLE){
			pkt_start_tick = *DWT_CYCCNT;
			if (!USE_915MHZ){
				uart_Send_Data(gp_rdo_tpacket_l, RADIO_LONG_PKT_LEN);
				snd_asymm_cnt = 0;
			}else{
				uart_Send_Data(gp_rdo_tpacket_l, RADIO_PKT_LEN);
			}
		}
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
#if true //ndef RECV_TAISYNC // try it out see if we can avoid byte drop, liyenho
			SPI_CSR_BITS_16_BIT);
#else //RECV_TAISYNC
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
/*static*/ void spi_master_initialize(uint32_t base, uint32_t fxcom)
{
//	puts("-I- Initialize SPI as master\r");
	/* Get pointer to SPI master PDC register base */
	g_p_spim_pdc [0] = spi_get_pdc_base(base);

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
	spi_set_bits_per_transfer(base, SPI_CHIP_SEL, SPI_CSR_BITS_8_BIT);  // 8 bit spi xfer, liyenho
	spi_set_baudrate_div(base, SPI_CHIP_SEL,
			(sysclk_get_cpu_hz() / gs_ul_spi_clock [0]));
		spi_set_transfer_delay(base, SPI_CHIP_SEL, 0x10/*delay between bytes*/,
			0x10/*delay between spi xfer*/);
	spi_enable(base);
#ifdef TEST_SPI
	spi_enable_loopback(base);
#endif
	pdc_disable_transfer(g_p_spim_pdc [0], PERIPH_PTCR_RXTDIS |
			PERIPH_PTCR_TXTDIS);
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
				// stm32 firmware upgrade
				download_sys_fw(true/*stm32 dnld*/, upgrade_fw_hdr );
				delay_ms(100);
/*****************************************************/
				/* stop usb device operation */
				udc_stop();
				/* run application */
				_app_exec(APP_START);
			}
			else if (2 == system_upgrade) {
				// xilinx artix upgrade
				download_sys_fw(false/*fpga dnld*/, upgrade_fw_hdr );
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

#ifdef CTRL_RADIO_ENCAP
void radio_pkt_filled(uint8_t num_bytes) {
	// To be filled in by Bill
	static uint8_t *pbr1= pb_rdo_ctrl;


	uint32_t write_err_cnt =0;
	int i =0;

	for (i =0; i<(num_bytes/RADIO_PKT_LEN); i++){
		//do 1 by 1 instead of in bulk in case bulk fails, this way at least some of writes are ok
		memcpy(gs_rdo_rpacket +(wrptr_rdo_rpacket*RDO_ELEMENT_SIZE), pbr1+(i*RADIO_PKT_LEN), RADIO_PKT_LEN);
		write_err_cnt += wrptr_inc(&wrptr_rdo_rpacket, &rdptr_rdo_rpacket, RDO_RPACKET_FIFO_SIZE,1);
	}
	pbr1 += i*RADIO_PKT_LEN;

	#ifdef DEBUG_RADIOSTATUS

	dbg_ctrlvidbuff[0]=pbr1[0]; pbr1++; //0x92
	dbg_ctrlvidbuff[1]=pbr1[0]; pbr1++;  //ctrl st16->drone cnt
	dbg_ctrlvidbuff[2]=pbr1[0]; pbr1++;  //tdma_lock
	dbg_ctrlvidbuff[3]=pbr1[0]; pbr1++;  //video ctrl 188 transfer cnt
	dbg_ctrlvidbuff[4]=num_bytes;  //gotPayloadSize
	dbg_ctrlvidbuff[5]=1;    //validFlag

	#endif


	  if ((pb_rdo_ctrl_e-188)<pbr1){
	  	 pbr1 = pb_rdo_ctrl;
	  }
	return;
}
#endif
#ifdef RX_SPI_CHAINING
  static void start_video_spi(bool restart)
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
//----------------------------------------------------------------------------

	irq_initialize_vectors();
	cpu_irq_enable();
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

	spi_master_initialize(SPI0_MASTER_BASE, BOARD_FLEXCOM_SPI0);// fpga/stm ctrl/sts pipe
	spi_slave_initialize();  // shall be applied on fpga video pipe
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
	#error Unmodulated carrier waveform is NOT supported
#endif

	// Start USB stack to authorize VBus monitoring
	udc_start();
system_restart:  // system restart entry, liyenho
	//pio_set_output(PIOA, PIO_PA0, LOW, DISABLE, ENABLE); // flag of cpld remote upgrade, liyenho
	system_main_restart = false;
#ifdef  RADIO_TAISYNC
	ctrl_radio_started = false;
	ctrl_tdma_enable = false;
	timedelta_reset = true;
	// start up with constant offset, we'll modify to adopt pairing reset later, liyenho
	fhop_base=0;
	fhop_offset = HOP_2CH_ENABLE?WRAP_OFFSET(HOP_2CH_OFFSET0):fhop_base;
	fhop_dir = true;  // hop forward when startup
	fhop_idx= 0 ; // used for pattern access
	hop_state = IDLE;
#endif
#ifdef RECV_TAISYNC
 	pio_set_output(PIOB, PIO_PB9, LOW, DISABLE, ENABLE); //stop TS gate
 	delay_ms(10); // flush all data from Pipe
	first_in = true; // be sure to reset 'first time flag' too
	pio_clear(PIOB, PIO_PB9); // disable TS gate
	spi_disable(g_p_spis_pdc); // I think that is the only spi to be reset? liyenho
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

		if (system_upgrade)
			upgrade_sys_fw(system_upgrade);

#if defined(CONF_BOARD_USB_RX) && defined(USE_UART)
	COMPILER_WORD_ALIGNED
	Configure_UART_DMA(1);
	static usb_cdc_line_coding_t uart_coding; // used for si446x radio dev, liyenho
	  uart_coding.dwDTERate = CPU_TO_LE32(UDI_CDC_DEFAULT_RATE);
	  uart_coding.bCharFormat = UDI_CDC_DEFAULT_STOPBITS;
	  uart_coding.bParityType = UDI_CDC_DEFAULT_PARITY;
	  uart_coding.bDataBits = UDI_CDC_DEFAULT_DATABITS;
	// re-config/open for drone comm with uart port, liyenho
		uart_config(1, &uart_coding);
		uart_open(1);
#endif
//////////////////////////////////////////////////////////////////////////////
#ifdef RADIO_CTRL_AUTO
		/* init radio stats obj */
		{
			for (int j=0; j<CTRL_CTX_LEN; j++) {
				ctrl_sts.ctrl_bits_ctx[j] = LONG_RNG;
			}
			ctrl_sts.bw_ctrl_bits = LONG_RNG;
			ctrl_sts.errPerAcc =0;
			ctrl_sts.loop_cnt= 0;
		}
		vRadio_Init();

		ctrl_radio_started = true;
		ctrl_tdma_enable = true;
#endif
	main_loop_on = true;  // enter run time stage, liyenho

	// The main loop manages
#if defined(RECV_TAISYNC)
 #ifdef WITH_ANT_SWITCH
		reset_ts_count_error = true;
 #endif
  #if defined(RX_SPI_CHAINING)
		mon_spidmachainfail_cnt  = 0;
		mon_ts47bad_cnt = 0;
	// from spi enabled to start dma can't be longer than 488 us, bad design, liyenho
	#ifndef DEBUG_BLOCK_TS_ISR
	configure_rtt(32); // arm 500us isr for TS, try out 1 mc though?
	#endif
  #endif //!defined(RX_SPI_CHAINING)
	// now allow video spi to be active, liyenho
	spi_enable(SPI_SLAVE_BASE);
#endif

#if FIXED_PAIR_ID
	uint8_t hop_id[HOP_ID_LEN] = {1};
	Set_Pair_ID(hop_id);
#endif //FIXED_PAIR_ID

	while (true) {
		if (system_main_restart ) goto system_restart;
		if (system_upgrade)
			upgrade_sys_fw(system_upgrade);
		  // ctrl RX Processing -----------------------------------------------
		if(ctrl_radio_started)
		{
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
	#ifdef CTRL_DYNAMIC_MOD
			si446x_get_int_status(0xff, 0xff, 0xff); // leave intr status bits unchanged
			if (Si446xCmd.GET_INT_STATUS.PH_PEND & SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_PACKET_SENT_PEND_BIT)
				bMain_IT_Status = SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_PACKET_SENT_PEND_BIT;
			if(Si446xCmd.GET_INT_STATUS.PH_PEND & SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_PACKET_RX_PEND_BIT)
				bMain_IT_Status = SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_PACKET_RX_PEND_BIT;
	  		process_range_mode(bMain_IT_Status);
	#endif
		} //if(ctrl_radio_started)
		if (!stream_flag) goto _reg_acs; // stop TS stream if flag isn't true, liyenho

#ifdef RX_SPI_CHAINING
	if(spidma_active != true) {
#ifdef VIDEO_DUAL_BUFFER
		stream = -1;  // invalidated
#endif
		start_video_spi(true);
	} else {
#ifdef WITH_ANT_SWITCH
		if (first_in) {
			last_three_sec = *DWT_CYCCNT;
			//first_in = false;
		}
		else if(reset_ts_count_error) {
			uint32_t cur_time;
			 cur_time = *DWT_CYCCNT;
			int dur;
			 dur= timedelta(0, cur_time, last_three_sec);
			if ((3*120000000)<dur) {
				// assume video dem/dec get stable after 3 sec, liyenho
				reset_ts_count_error = false;
			}
		}
#endif
	; }
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
#if defined(RECV_TAISYNC)
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
				case TS_VID_ACTIVE:
							#if defined(RECV_TAISYNC)
#ifdef VIDEO_DUAL_BUFFER
								stream = -1;  // invalidated
#endif
								// now allow video spi to be active, liyenho
								spi_enable(SPI_SLAVE_BASE);
							#ifndef RX_SPI_CHAINING
								pio_set(PIOB, PIO_PB9); // enable TS gate
							#else
								// pull over to here instead init inside main loop to get rid of time constraint, liyenho
								start_video_spi(false);
							#endif
								first_in = false;  // now it is safe to time vid ant sw
							#endif
							break;
				default: /* host msg in error */
							continue;
			}
#endif //RECV_TAISYNC
  		}
#ifdef WITH_ANT_SWITCH
		if (gl_vid_ant_sw){
			Queue_Msg_Vid_Ant_Switch();
		}
#endif
#if false  // unavailble on taisync platform
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
#endif
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

volatile bool main_usb_host_msg() // Attach this api to usb rx isr service, liyenho
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
			udd_set_setup_payload( gs_uc_hrbuffer,USB_HOST_MSG_LEN+(ps->dcnt-1)*sizeof(uint8_t));
		}
		else //cnf_echo == 1
		{
			udd_set_setup_payload( gs_uc_hrbuffer,
				USB_HOST_MSG_LEN-sizeof(ps->data[0]));
		}
	/* no need to setup callback in READ case */
	return (true != spi_tgt_done);
}

// to enable embedded video subsystem ********************
extern int init_video_subsystem();
extern int start_video_subsystem();
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
		if (!start_video_subsystem()) {
			 first_in = false;  // now it is safe to time vid ant sw
			return true;
		}
		else return false ;
	}
/**********************************************************/
	if (USB_HOST_MSG_TX_VAL == udd_g_ctrlreq.req.wValue ||
		USB_STM_UPGRADE_VAL == udd_g_ctrlreq.req.wValue ||
		USB_FPGA_UPGRADE_VAL == udd_g_ctrlreq.req.wValue ||
		USB_ATMEL_UPGRADE_VAL == udd_g_ctrlreq.req.wValue)
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
#ifdef WITH_ANT_SWITCH
	 else if (USB_ANT_SW_VAL == udd_g_ctrlreq.req.wValue) {
			gl_vid_ant_sw = true; // try to switch antenna
	 }
#endif
#ifdef  RADIO_TAISYNC
	else if (RADIO_COMM_VAL == udd_g_ctrlreq.req.wValue) {
		if (RADIO_STARTUP_IDX == udd_g_ctrlreq.req.wIndex) {
			pio_clear(PIOB, PIO_PB9); // disable TS gate
#ifdef RX_SPI_CHAINING
			spidma_active = false;
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

/*static*/ int timedelta(bool reset, unsigned int bignum, unsigned int smallnum)
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

