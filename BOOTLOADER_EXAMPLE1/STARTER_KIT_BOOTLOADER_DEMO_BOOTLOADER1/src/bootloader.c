/**
 * \file
 *
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include "bootloader.h"
#include "conf_usb.h"
#include "assert.h"
/**
 * \mainpage Bootloader of Starter Kit Bootloader Demo
 *
 * \section Introduction (only worked on usb fwm media now)
 *
 */

/* Dump written data after load */
#define DUMP_ENABLE   0

/** Cortex-M Code region start */
#define CM_CODE_START      0x00000080
/** Cortex-M Code region end */
#define CM_CODE_END        0x1FFFFF80
/** Cortex-M SRAM region start */
#define CM_SRAM_START      0x20000000
/** Cortex-M SRAM region end */
#define CM_SRAM_END        0x3FFFFF80

extern volatile bool udi_cdc_data_running; // from udi_cdc.c, liyenho
const uint32_t ul_page_addr_bootapp = FLAG_BOOTAPP_ADDR;
const uint32_t ul_page_addr_fpgadef = FLAG_FPGADEF_ADDR;
static volatile bool main_b_cdc_enable = false;
static volatile bool system_upgrade = false;  // system upgrade flag, liyenho
static volatile uint8_t main_loop_on = false;  // run time indicator
// boot current atmel fwm on flash when this is true, load new atmel fwm from host when this is false
static volatile uint8_t usb_boot_app = (uint8_t)-1;
  static uint32_t fw_sys_tbuffer[FW_DNLD_SIZE/sizeof(int)];
  static uint32_t fw_sys_rbuffer[FW_DNLD_SIZE/sizeof(int)];
  static uint32_t fw_dbg_buffer[FW_DNLD_SIZE/sizeof(int)];
  static uint32_t flash_erasure_buff[MEM_ERASE_SIZE/sizeof(int)];
  static uint8_t *pba = (uint8_t*)flash_erasure_buff;
//algorithm from digibest sdk, using bulk transfer pipe, liyenho
static volatile uint32_t upgrade_fw_hdr[FW_UPGRADE_HDR_LEN/sizeof(int)]={-1} ;
#if defined(FWM_DNLD_DBG)
	volatile bool usb_tgt_active = false;
  volatile bool usb_host_active = false;
#endif
  inline void usb_read_buf1(void *pb, int size0);
  inline void usb_write_buf1(void *pb, int size0);

static inline void usb_read_buf1(void *pb, int size0)
{
	int read=0, size = size0;
	do {
		iram_size_t b = size-udi_cdc_read_buf(pb, size);
		pb += b;
		size -= b;
		read += b;
	} while (size0 != read);
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

/**
 * Get boot region information
 * \return 0 or 1.
 */
static uint32_t _app_boot_get(void)
{
	uint32_t gpmvm_status = flash_is_gpnvm_set(2);
	if (gpmvm_status == FLASH_RC_YES) {
		return 1;
	} else {
		return 0;
	}
}

/** Turn LED on
 * \param led_pin The pin index that connected to LED
 */
#define _app_led_on(led_pin) \
	ioport_set_pin_level(led_pin, DBG_LED_PIN_ON_LEVEL)

/** Turn LED off
 * \param led_pin The pin index that connected to LED
 */
#define _app_led_off(led_pin) \
	ioport_set_pin_level(led_pin, !DBG_LED_PIN_ON_LEVEL)

/** Toggle LED
 * \param led_pin The pin index that connected to LED
 */
//#define _app_led_toggle(led_pin) \
//	ioport_toggle_pin_level(led_pin)

/**
 * Blink LED
 * \param delay_ms LED toggle delay time in ms
 * \param count Blink times
 */
static void _app_led_blink(uint32_t delay_ms, uint32_t count)
{
	uint32_t i;
	for (i = 0; i < count; i++) {
		_app_led_on(DBG_LED_PIN);
		delay_ms(delay_ms);
		_app_led_off(DBG_LED_PIN);
		delay_ms(delay_ms);
	}
}

/**
 * Indicate error
 * - ON for 1s
 * - Blink very fast 1s
 * - OFF
 */
static void _app_led_error(void)
{
	for (int n=0; n<3; n++) {
		_app_led_on(DBG_LED_PIN);
		delay_ms(1000);
		_app_led_off(DBG_LED_PIN);
		delay_ms(1000);
	}
}

/**
 * Receive new firmware
 *
 * \return received firmware size
 */
static uint32_t _app_load(uint32_t app_addr)
{
	while (!system_upgrade) ; // wait for host to turn on download ops
#ifdef FWM_DNLD_DBG
  	irqflags_t flags;
#endif
	static bool first_access = true;
	uint8_t chk[4] = {
	 		0xff & (*upgrade_fw_hdr>>0),
	 		0xff & (*upgrade_fw_hdr>>8),
	 		 0xff & (*upgrade_fw_hdr>>16),
	 		 0xff & (*upgrade_fw_hdr>>24) };
	uint32_t fw_addr = app_addr /*constant, not from host*/,
						fw_len0, fw_len = upgrade_fw_hdr[1],
						len, len1, len2, ll, ll1, crc, crc1, i;
	uint8_t *pb = fw_sys_rbuffer,
					 *pe = pb+FW_DNLD_SIZE;
	volatile bool erase = true;
	uint32_t ul_page_addr = fw_addr, bytes0 = 0 ;
	uint32_t lm, le, er_adr =ul_page_addr-(ul_page_addr & (MEM_ERASE_SIZE-1));
	uint32_t erlen = MEM_ERASE_SIZE ;
	lm = (ul_page_addr & (MEM_ERASE_SIZE-1))+FW_DNLD_SIZE;
	le = erlen;
#ifdef FWM_DNLD_DBG
	usb_tgt_active = true;
	while (!usb_host_active) ;
	flags = cpu_irq_save();
	  usb_host_active = false;
	cpu_irq_restore(flags);
#endif
	usb_read_buf1(fw_sys_rbuffer, FW_DNLD_SIZE);
	 do { // bypass annoying flush data prior to xfer from bulk ep, liyenho
		if (chk[0] == pb[0] &&
			chk[1] == pb[1] &&
			chk[2] == pb[2] &&
			chk[3] == pb[3])
			  break;
	} while (pe>++pb);
	if (pb==pe) {
		/*puts("-E-\tfailed to locate 1st chunk of sms fw.\r");*/
		return 0;
	}
/*********** prepare intrenal flash on app area by unlock ****************/
#ifdef DBG_USE_LED
		_app_led_blink(50, 4);
		_app_led_on(DBG_LED_PIN);
#endif
 	dbg_print("bl: Unlock download buffer area ...\r\n");
	memory_unlock(fw_addr, (void *)((uint32_t)fw_addr + APP_CODE_SIZE - 1));
	dbg_print("bl: Unlock download buffer area done\r\n");
/*****************************************************************/
	len = FW_DNLD_SIZE - ((uint32_t)pb - (uint32_t)fw_sys_rbuffer);
	len2 = (len1=FW_DNLD_SIZE) - (ll=len);
	memcpy(fw_sys_tbuffer, pb, len); // has to use the other buffer
	pb = (uint8_t*)fw_sys_tbuffer + len;
	 pe = (uint8_t*)fw_sys_rbuffer + len2;
	fw_len0 = fw_len - len + len2;
	// buffer last chunk existing data on flash prior to erase, only done once
	memcpy(pba, er_adr, fw_addr & (MEM_ERASE_SIZE-1));
	while(1) {
		len = (FW_DNLD_SIZE<=fw_len0)?FW_DNLD_SIZE:fw_len0;
		if (0 != fw_len0) {
#ifdef FWM_DNLD_DBG
	usb_tgt_active = true;
	while (!usb_host_active) ;
	flags = cpu_irq_save();
	  usb_host_active = false;
	cpu_irq_restore(flags);
#endif
			usb_read_buf1(fw_sys_rbuffer, len); }
		fw_len0 -= len;
		if (FW_DNLD_SIZE != len) {
			/* NEVER PLACE BREAKPOINT INSIDE THIS SECTION!!!
			  or the host/atmel sync mechanism shall fail... */
			if (0 != len) {
				if (len2>len) {
					assert(len1+len2 == len);
					memcpy(pb, fw_sys_rbuffer, len);
				}
				else {
					memcpy(pb, fw_sys_rbuffer, len2);
					ll1 = len - len2;
				}
		   }
			else  {
				assert(len1+len2 == ll1);
				memcpy(fw_sys_tbuffer, pe, ll1);
		   }
	  	}
		else
	  		memcpy(pb, fw_sys_rbuffer, len2);
#if true
		crc = 0;
		for (i=0; i<len1; i++)
			crc ^= ((uint8_t*)fw_sys_tbuffer)[i];
#endif
		if (erase) {
			i = mem_flash_erase(er_adr, MEM_ERASE_SIZE);
			if (MEM_ERASE_SIZE != i) {
				/*puts("-E-\tfailed on DataErase().\r");*/
				return 0;
			}
			if (first_access && (fw_addr & (MEM_ERASE_SIZE-1))) {
				// put back the existing data on flash
				int j, pg = 0;
				for (j=0; j<((fw_addr & (MEM_ERASE_SIZE-1))/MEM_BLOCK_SIZE); j++) {
					if (!mem_flash_page_write(er_adr+pg, pba+ pg)) {
						/*puts("-E-\tfailed on DataWrite().\r");*/
						return 0;
					}
					pg += MEM_BLOCK_SIZE;
				}
				first_access = false;
			}
			erase = false;
		}
	uint64_t block[MEM_BLOCK_SIZE/sizeof(uint64_t)];
	int bytes1=0, bb=0;
		// handle first chunk which may not align to page boundary
		if ((MEM_BLOCK_SIZE-1) & bytes0) {
			// raed from flash by previous write
			bytes1 = (MEM_BLOCK_SIZE-1) & bytes0;
			memcpy(block, ul_page_addr+bytes0-bytes1, bytes1);
			// read from fwm load buffer
			memcpy((uint8_t*)block+bytes1, fw_sys_tbuffer, MEM_BLOCK_SIZE-bytes1);
			if (!mem_flash_page_write(ul_page_addr+bytes0-bytes1, block)) {
				/*puts("-E-\tfailed on DataWrite().\r");*/
				return 0;
			}
#ifdef DBG_USE_LED
			/* turn on the led */
			_app_led_on(DBG_LED_PIN);
#endif
			bytes0 += MEM_BLOCK_SIZE-bytes1;
			bytes1 = MEM_BLOCK_SIZE-bytes1;
		}
		do {
			if (!mem_flash_page_write(ul_page_addr+bytes0, (uint8_t*)fw_sys_tbuffer+bytes1)) {
				/*puts("-E-\tfailed on DataWrite().\r");*/
				return 0;
			}
#ifdef DBG_USE_LED
			/* Toggle the led when app load. */
			if (1 & bb)
				_app_led_on(DBG_LED_PIN);
			else
				_app_led_off(DBG_LED_PIN);
#endif
			bytes0 += MEM_BLOCK_SIZE;
			bytes1 += MEM_BLOCK_SIZE;
			bb += 1;
		} while((len1-MEM_BLOCK_SIZE)>=bytes1);
		// handle last chunk which may not align to page boundary
		if (true) {
			memcpy(block, (uint8_t*)fw_sys_tbuffer+bytes1, len1-bytes1);
			// set the rest of buffer to 0xff (empty)
			memset((uint8_t*)block+len1-bytes1, 0xff, MEM_BLOCK_SIZE-len1+bytes1);
			if (!mem_flash_page_write(ul_page_addr+bytes0, block)) {
				/*puts("-E-\tfailed on DataWrite().\r");*/
				return 0;
			}
	#ifdef DBG_USE_LED
				/* Toggle the led when app load. */
			if (1 & bb)
				_app_led_on(DBG_LED_PIN);
			else
				_app_led_off(DBG_LED_PIN);
	#endif
			bytes0 += len1-bytes1;
		}
#if true
		memcpy(fw_dbg_buffer, ul_page_addr+bytes0-len1, len1);
		crc1 = 0;
		for (i=0; i<len1; i++)
			crc1 ^= ((uint8_t*)fw_dbg_buffer)[i];
		if (crc != crc1){
		/*puts("-E-\tfailed on checksum verification.\r");*/
		return 0;
	}
#endif
 #ifdef FWM_DNLD_DBG
 	usb_tgt_active = true;
	while (!usb_host_active) ;
	flags = cpu_irq_save();
	  usb_host_active = false;
	cpu_irq_restore(flags);
		usb_write_buf1(fw_dbg_buffer, len1);
 #endif
		// determine whether erasure is necessary or not
		lm += (len1);
		if (le < lm) {
			erase = true;
			le += erlen;
			er_adr += erlen;
		}
		if (!(fw_len -= len1)) break;
		len1 = (FW_DNLD_SIZE<=fw_len)?FW_DNLD_SIZE:fw_len;
		if (FW_DNLD_SIZE == len) {
			memcpy(fw_sys_tbuffer, pe, ll);
		}
	}
#ifdef DBG_USE_LED
		_app_led_off(DBG_LED_PIN);
#endif
	delay_ms(100);
 #ifdef FWM_DNLD_DBG
		usb_write_buf1(fw_dbg_buffer, 160/*flush dbg dump data on host*/);
 #endif
/************** finish intrenal flash on app area by lock ****************/
	dbg_print("bl: Lock download buffer area ...\r\n");
	memory_lock(fw_addr, (void *)((uint32_t)fw_addr + APP_CODE_SIZE - 1));
	dbg_print("bl: Lock download buffer area done\r\n");
/*****************************************************************/
 	return bytes0;
}

/**
 * Jump to CM vector table
 *
 * \param code_addr Application start address (vector table address)
 */
#if defined   (__CC_ARM)     /* Keil µVision 4 */
__asm__ void jump_to_app(void *code_addr)
{
	mov r1, r0
	ldr r0, [r1, # 4]
	ldr sp, [r1]
	blx r0
}

#elif defined (__ICCARM__)   /* IAR Ewarm 5.41+ */
void jump_to_app(void *code_addr)
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
void jump_to_app(void *code_addr)
{
	__asm__(
			"mov   r1, r0        \n"
			"ldr   r0, [r1, #4]  \n"
			"ldr   sp, [r1]      \n"
			"blx   r0"
			);
}

#else /* General C, no stack reset */
void jump_to_app(void *code_addr)
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
 * \return If success, no return;
 *         1 - address alignment error;
 *         2 - address not executable.
 */
static uint8_t _app_exec(void *addr)
{
	uint32_t i;
	/* Check parameters */
	if ((uint32_t)addr & 0x7F) {
		dbg_print("bl: application start is not properly aligned...\r\n\n");
		return 1;
	}

	if ((uint32_t)addr > CM_SRAM_END) {
		dbg_print("bl: application end went beyond memory space...\r\n\n");
		return 2;
	}

#ifdef DBG_USE_USART
	delay_ms(50); /* Wait USART lines idle */

	dbg_cleanup();

	delay_ms(50);
#endif

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
	return 0;
}

#ifdef DBG_USE_INFO_EDIT


#define ESC 0x1B
#define CR  0x0D
#define LF  0x0A
#define BS  0x08
#define DBGIN_ECHO_ON  (1u << 0) /**< Echo inputs */
#define DBGIN_NUM_ONLY (1u << 1) /**< Accept numbers only */
#define DBGIN_WANT_ESC (1u << 2) /**< Accept ESC to cancel input */
#define DBGIN_WANT_CR  (1u << 3) /**< Accept enter to terminate input */
/** console input buffer */
uint8_t input_buf[16];

/**
 * Wait for input
 * \param keys pointer to buffer to store inputs
 * \param nb_keys max number of keys
 * \param ctrl_flags flags to control inputs (DBGIN_ECHO_ON, DBGIN_NUM_ONLY,
 *                   DBGIN_WANT_ESC and DBGIN_WANT_CR can be used)
 * \return -1 if ESC, else number of input chars
 */
static int _app_dbg_input(uint8_t *keys, int nb_keys, uint32_t ctrl_flags)
{
	uint8_t key;
	int i;
	for (i = 0; i < nb_keys;) {
		if (!dbg_rx_ready()) {
			continue;
		}

		key = dbg_getchar();
		if (key == CR) {
			if (DBGIN_WANT_CR & ctrl_flags) {
				keys[i++] = 0;
				break;
			}

			/* Ignore */
			continue;
		} else if (key == ESC) {
			if (DBGIN_WANT_ESC & ctrl_flags) {
				keys[0] = 0;
				return -1;
			}
		} else {
			if (DBGIN_NUM_ONLY & ctrl_flags) {
				if (key < '0' || key > '9') {
					/* Not accepted */
					continue;
				}
			}

			if (key <= ' ' || key >= 'z') {
				/* Not accepted */
				continue;
			}
		}

		if (ctrl_flags & DBGIN_ECHO_ON) {
			dbg_putchar(key);
		}

		keys[i++] = key;
	}
	return i;
}

#endif

  #define SPI0_Handler     FLEXCOM0_Handler // spi0 -> spi1 on tx
  #define SPI0_IRQn        FLEXCOM0_IRQn
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

/* Pointer to PDC register base */
static Pdc *g_p_spim_pdc [1+2] ;/*fpga/sms, video, radio ctrl/sts, liyenho*/
/* SPI clock setting (Hz). */
  static uint32_t gs_ul_spi_clock[2+1] = {  // shall be higher than usb data rate
  				  	2000000, 8000000, 1000000}; // (fpga/sms, video, radio) spi bit rate
volatile uint8_t spi_tgt_done=false; // triggered by spi isr for (fpga/cpld) comm

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
/**
 * \brief Initialize SPI0 as master.
 */
static void spi0_master_initialize()
{
//	puts("-I- Initialize SPI as master\r");
	/* Get pointer to SPI master PDC register base */
	g_p_spim_pdc [0] = spi_get_pdc_base(SPI0_MASTER_BASE);

#if (SAMG55)
	/* Enable the peripheral and set SPI mode. */
	flexcom_enable(BOARD_FLEXCOM_SPI0);
	flexcom_set_opmode(BOARD_FLEXCOM_SPI0, FLEXCOM_SPI);
#else
	/* Configure an SPI peripheral. */
	pmc_enable_periph_clk(SPI_ID);
#endif
	spi_disable(SPI0_MASTER_BASE);
	spi_reset(SPI0_MASTER_BASE);
	spi_set_lastxfer(SPI0_MASTER_BASE);
	spi_set_master_mode(SPI0_MASTER_BASE);
	spi_disable_mode_fault_detect(SPI0_MASTER_BASE);
	spi_set_peripheral_chip_select_value(SPI0_MASTER_BASE, SPI_CHIP_SEL);
	spi_set_clock_polarity(SPI0_MASTER_BASE, SPI_CHIP_SEL, SPI_CLK_POLARITY);
	spi_set_clock_phase(SPI0_MASTER_BASE, SPI_CHIP_SEL, SPI_CLK_PHASE);
	 spi_set_bits_per_transfer(SPI0_MASTER_BASE, SPI_CHIP_SEL,
			SPI_CSR_BITS_16_BIT);  // 16 bit spi xfer
	spi_set_baudrate_div(SPI0_MASTER_BASE, SPI_CHIP_SEL,
			(sysclk_get_cpu_hz() / gs_ul_spi_clock [0]));
	spi_set_transfer_delay(SPI0_MASTER_BASE, SPI_CHIP_SEL, SPI_DLYBS,
			SPI_DLYBCT);
	spi_enable(SPI0_MASTER_BASE);
	pdc_disable_transfer(g_p_spim_pdc [0], PERIPH_PTCR_RXTDIS |
			PERIPH_PTCR_TXTDIS);
}
/**
 * \brief Perform SPI master transfer.
 *
 * \param pbuf Pointer to buffer to transfer.
 * \param size Size of the buffer.
 */
static void spi0_tx_transfer(void *p_tbuf, uint32_t tsize, void *p_rbuf, uint32_t rsize)
{
	pdc_packet_t pdc_spi_packet;

	pdc_spi_packet.ul_addr = (uint32_t)p_rbuf;
	pdc_spi_packet.ul_size = rsize;
	pdc_rx_init(g_p_spim_pdc [0], &pdc_spi_packet, NULL);

	pdc_spi_packet.ul_addr = (uint32_t)p_tbuf;
	pdc_spi_packet.ul_size = tsize;
	pdc_tx_init(g_p_spim_pdc [0], &pdc_spi_packet, NULL);

	/* Enable the RX and TX PDC transfer requests */
	pdc_enable_transfer(g_p_spim_pdc [0], PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);

	/* Transfer done handler is in ISR */
	uint32_t spi_ier = SPI_IER_RXBUFF;
	spi_enable_interrupt(SPI0_MASTER_BASE, spi_ier) ;
}
/**
 * \brief Set SPI slave transfer.
 *
 * \param p_buf Pointer to buffer to transfer.
 * \param size Size of the buffer.
 */
static void spi0_rx_transfer(void *p_tbuf, uint32_t tsize, void *p_rbuf, uint32_t rsize)
{
	uint32_t spi_ier;
	pdc_packet_t pdc_spi_packet;

	pdc_spi_packet.ul_addr = (uint32_t)p_rbuf;
	pdc_spi_packet.ul_size = rsize;
	pdc_rx_init(g_p_spim_pdc [0], &pdc_spi_packet, NULL);

	pdc_spi_packet.ul_addr = (uint32_t)p_tbuf;
	pdc_spi_packet.ul_size = tsize;
	pdc_tx_init(g_p_spim_pdc [0], &pdc_spi_packet, NULL);

	/* Enable the RX and TX PDC transfer requests */
	pdc_enable_transfer(g_p_spim_pdc [0], PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);
	/* Transfer done handler is in ISR */
	spi_ier = SPI_IER_RXBUFF;
	spi_enable_interrupt(SPI0_MASTER_BASE, spi_ier) ;
}

/**
 * Bootloader main entry
 */
int main(void)
{
	uint8_t flag_boot_app = *(uint8_t*)ul_page_addr_bootapp;
	uint8_t flag_fpga_def = *(uint8_t*)ul_page_addr_fpgadef;
	uint8_t boot_region = 0; /* Real boot region at this time */
#ifdef DBG_USE_USART
	uint8_t load_region = 0; /* Real region to put loaded data */
#endif
	void *app_addr = NULL;
	uint32_t app_size = 0;
	bool app_valid = false;
	/* these may be required for usb ops? */
	irq_initialize_vectors();
	cpu_irq_enable();

	wdt_disable(WDT);
	sysclk_init();
	board_init();

	/* First turn on the led to indicate the bootloader run. */
	ioport_set_pin_dir(DBG_LED_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(DBG_LED_PIN, DBG_LED_PIN_ON_LEVEL);

	NVIC_DisableIRQ(SPI0_IRQn);  // spi0 peripheral instance = 8, liyenho
	NVIC_ClearPendingIRQ(SPI0_IRQn);
	NVIC_SetPriority(SPI0_IRQn, 0);
	NVIC_EnableIRQ(SPI0_IRQn);

	spi0_master_initialize();// fpga/cpld ctrl pipe
#ifdef FPGA_IMAGE_APP
	if (!flag_fpga_def || 0xff ==flag_fpga_def) {
		uint32_t tmp, wtmp, *pth = &tmp;
		delay_s(3);  // allow fpga to complete initialization
		// fpga switch to app image
/*****************************************************/
		//enable reconfig
		*pth = 0xc000 | (0xfff & 12); // reg 12d
		spi_tgt_done = true;
		spi0_tx_transfer(pth, 2/2, &wtmp, 2/2);
		while (spi_tgt_done) ; spi_tgt_done = true;
		wtmp = (0x01); *pth = 0xb000 | (0x0ff & wtmp);  // with 0x1
		spi0_tx_transfer(pth, 2/2, &wtmp, 2/2);
		while (spi_tgt_done) ; spi_tgt_done = true;
/*****************************************************/
	}
#endif

	if (!flag_boot_app || (0xff==flag_boot_app))
	// Start USB stack to authorize VBus monitoring
		udc_start(); // turn on usb comm only when flash flag is empty or reset
	else  // otherwise jump to main app directly
		usb_boot_app = true;

	dbg_init();
	dbg_print("\r\n\n----------------------\r\n");
#ifdef CONF_BOARD_USB_RX
	dbg_print("%s Bootloader\r\n", "YUNEEC_LGDST_TX");
#elif defined(CONF_BOARD_USB_TX)
	dbg_print("%s Bootloader\r\n", "YUNEEC_LGDST_RX");
#endif
	dbg_print("Boot region: %x, size %dK\r\n", BOOT0_START, BOOT_SIZE /
			1024);
	dbg_print("App  region: %x, size %dK\r\n", APP0_START,
			(int)APP_SIZE / 1024);
	dbg_print(" - Code %dK + Info %dK\r\n", (int)APP_CODE_SIZE,
			(int)INFO_SIZE);
	dbg_print("----------------------\r\n");

	/* bootloader initialize */
	dbg_print("bl: init ...\r\n");
	memory_init();
	dbg_print("bl: init done\r\n");

	boot_region = _app_boot_get();
	dbg_print("bl: current boot region %d\r\n", (int)boot_region);

#ifdef MEM_LOCK_BOOT_REGION
	dbg_print("bl: lock boot region ...\r\n");
	memory_lock((void *)BOOT0_START, (void *)BOOT0_END);
	dbg_print("bl: lock boot region done\r\n");
#endif
	if (1 != flag_boot_app)
 	while (!udi_cdc_data_running) ; // wait for cdc data intf ready, liyenho

	app_addr = (void *)(APP_START(boot_region));

	dbg_print("bl: boot\t       %c\t       %c\r\n", boot_region ? ' ' : 'Y',
			boot_region ? 'Y' : ' ');
	dbg_print("bl: App @ %d, %x - data %x ...\r\n", (int)boot_region,
			(unsigned)app_addr, (unsigned)*(uint32_t *)app_addr);
	dbg_print("bl: App valid: %c\r\n", app_valid ? 'Y' : 'N');

	while ((uint8_t)-1 == usb_boot_app) {;}

	/* bootloader trigger check (from host Not saved on flash) */
	dbg_print("bl: trigger mode %s\r\n", (usb_boot_app)?"BOOTUP":"UPDATE" );
	if (true == usb_boot_app) {
		goto main_run_app_check;
	}

	/* Now any other trigger load file to update application directly */
#ifdef DBG_USE_LED
	_app_led_blink(100, 1);
#endif

main_load_app:
	/* load new firmware */
#ifdef DBG_USE_USART
	load_region = boot_region;
	dbg_print("bl: download @ %d ...\r\n", load_region);
#endif
	main_loop_on = true;
	app_size = _app_load((uint32_t)app_addr);
	if (app_size != upgrade_fw_hdr[1]) {
		_app_led_error();
		dbg_print("bl: download fail, retry\r\n");
		main_loop_on = false; // reset to restart
		goto main_load_app;
	} else {
		dbg_print("bl: download done, size %d\r\n", (int)app_size);
	}

main_run_app_check:
	if (!flag_boot_app || (0xff==flag_boot_app))
	/* stop usb device operation */
		udc_stop();  // turn on usb comm only when flash flag is empty or reset

	/* Is application valid? not available for verification... */
	dbg_print("bl: application validity is not available, run\r\n");

	/* Turn off the led before jump to the app. */
	_app_led_off(DBG_LED_PIN);

	/* cleanup */
	dbg_print("bl: cleanup ...\r\n");
	memory_cleanup();
	dbg_print("bl: cleanup done\r\n");

	/* load application */
	dbg_print("bl: load application ...\r\n\n");

	/* run application */
	_app_exec(app_addr);

	return (int)app_addr;
}

static void host_usb_cb() {
	system_upgrade = true;  // host turn on atmel upgrade operation
}

volatile bool main_vender_specific() {
	if (USB_FWM_BOOTUP_VAL == udd_g_ctrlreq.req.wValue) {
		usb_boot_app = true;
		return true;
	}
	else if (USB_FWM_UPDATE_VAL == udd_g_ctrlreq.req.wValue) {
		udd_set_setup_payload( upgrade_fw_hdr, sizeof(upgrade_fw_hdr));
		udd_g_ctrlreq.callback = host_usb_cb;
		usb_boot_app = false;
		return true;
	}
	 else if (USB_STREAM_ON_VAL == udd_g_ctrlreq.req.wValue) {
		 if (USB_QUERY_IDX == udd_g_ctrlreq.req.wIndex)
		 	udd_set_setup_payload( &main_loop_on, sizeof(main_loop_on));
		 return (bool)-1 ;
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

void main_suspend_action(void)
{
	return ;
}

void main_resume_action(void)
{
	return ;
}

void main_sof_action(void)
{
	return ;
}

#ifdef USB_DEVICE_LPM_SUPPORT
void main_suspend_lpm_action(void)
{
	return ;
}

void main_remotewakeup_lpm_disable(void)
{
	return ;
}

void main_remotewakeup_lpm_enable(void)
{
	return ;
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
}
