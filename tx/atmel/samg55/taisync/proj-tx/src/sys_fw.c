#include <string.h>
#include <assert.h>
#include "asf.h"		// declaration of all drivers stuffs
#include "conf_usb.h"	// main include to interface atmel spi apis
#include "delay.h"

typedef struct {
	uint8_t cmd;
	uint8_t rw ;  // 0: write, 1: read
	uint16_t padd; // 32 bit alignment on stack
	uint32_t i_data;
	uint32_t o_data;
} spi_cmd_data;

typedef enum
{
	STM32_ERASE	= 0x1,
	FPGA_ERASE	= 0x2,
	STM32_PROGRAM	= 0x4,
	FPGA_PROGRAM = 0x8,
} access_type;

#define SPIBASEADDR		/*SPI0*/SPI5   // ctrl/sts pipe which interfaces with fpga on tx board, liyenho
#define SPIFLEXCOM			BOARD_FLEXCOM_SPI0
#define SPI_CHIP_SEL			0
#define SPIRDFIFO				(((Spi*)/*SPI0*/SPI5)->SPI_RDR)
#define SPIWRFIFO				(((Spi*)/*SPI0*/SPI5)->SPI_TDR)

#define CMD_RD						0xa
#define CMD_WR						0x5
#define ADR_MD						0x88
#define ADR_STS						0x8a
#define ADR_WR						0x89

const static uint32_t page_size = 4096;  // assumed 4096 byte access page, liyenho

 extern void spi_master_initialize(uint32_t base, uint32_t fxcom);
 extern void usb_read_buf1(void *pb, int size0);

 static uint32_t fw_sys_tbuffer[FW_DNLD_SIZE/sizeof(int)];
 static uint32_t fw_sys_rbuffer[FW_DNLD_SIZE/sizeof(int)];
#if /*true*/false
 static uint32_t fw_dbg_buffer[FW_DNLD_SIZE/sizeof(int)];
#endif
 #if defined(FWM_DNLD_DBG)
	volatile bool usb_tgt_active = false;
  extern volatile bool usb_host_active;
#endif

/*******************************************************************************
     SPI 0 Rx controller init
Function:     Spi0PortRxInit()
Arguments:    There is no argument for this function
Description:  This function has to be called at the beginnning to configure the
			  port.

/******************************************************************************/
void Spi0PortRxInit(void)
{
	 	pio_set_peripheral(PIOA, PIO_PERIPH_A, /*PIO_PA9 | PIO_PA10*/PIO_PA12 | PIO_PA13);
	 	pio_set_peripheral(/*PIOB*/PIOA, PIO_PERIPH_A, /*PIO_PB0*/PIO_PA14 );
	 	spi_master_initialize(SPIBASEADDR, SPIFLEXCOM);
	 	spi_set_clock_polarity(SPIBASEADDR, SPI_CHIP_SEL, 0 /*SPI_CLK_POLARITY*/);
	 	spi_set_clock_phase(SPIBASEADDR, SPI_CHIP_SEL, 1 /*SPI_CLK_PHASE*/);
		spi_configure_cs_behavior(SPIBASEADDR, 0, SPI_CS_KEEP_LOW);
		pio_configure(PIOA, PIO_OUTPUT_1, FPGA_ARTIX_TRIG, 0);  // manual CS control

	delay_ms(50); // Extra waiting for SPI0 Master Ready
}

#define CHECK_BSY   \
	while (!((Spi*)SPIBASEADDR)->SPI_SR & SPI_SR_TXEMPTY) { \
	delay_us(/*10*/2); } delay_us(/*10*/2)
#define SET_CS \
	pio_clear(PIOA, FPGA_ARTIX_TRIG);
#define CLEAR_CS \
	pio_set(PIOA, FPGA_ARTIX_TRIG);
#define FLUSHRWFIFO 	delay_us(/*10*/5)
#define DUMMY_BYTE      0xff  // cannot be all 0s
#define REG_(x)			(*(volatile uint32*)(x))
#define RD_R(REG_)		(REG_)
#define WR_R(REG_, D)	(REG_ = D)

static void Serialize_SPI(spi_cmd_data *p) {
	uint16_t tx_len, rx_len;
	uint8_t *pb ;

	CHECK_BSY;
	SET_CS;
	// send command byte
	WR_R(SPIWRFIFO,  SPI_TDR_TD(0xff & *pb));
		CHECK_BSY;

	// send reg addr byte
	pb = (uint8_t*)&p->padd;
	WR_R(SPIWRFIFO,  SPI_TDR_TD(0xff & *pb));
		CHECK_BSY;

	if (p->rw) {
		rx_len = sizeof(uint32_t);
		pb = ((uint8_t*)&p->i_data)+3;
		tx_len = 0;
	} else {
		rx_len = 0;
		tx_len = sizeof(uint32_t);
		pb = ((uint8_t*)&p->o_data)+3;
	}

	while (tx_len-- > 0)
	{
		WR_R(SPIWRFIFO,  SPI_TDR_TD(*pb--));
		CHECK_BSY;

		RD_R(SPIRDFIFO & SPI_RDR_RD_Msk);
	}
	while (rx_len-- > 0)
	{
		WR_R(SPIWRFIFO,  SPI_TDR_TD(DUMMY_BYTE));
		CHECK_BSY;

		*pb-- = RD_R(SPIRDFIFO & SPI_RDR_RD_Msk);
	}

	CLEAR_CS;
	FLUSHRWFIFO;
}

static void Flash_Erase(access_type type) {
	spi_cmd_data msg;
	msg.cmd = CMD_WR;
	msg.rw = 0; // write
	msg.padd = ADR_MD;
	msg.o_data = type;
	Serialize_SPI(&msg);
	// poll for readiness
	msg.cmd = CMD_RD;
	msg.rw = 1; // read
	do { Serialize_SPI(&msg);
		} while(type & msg.i_data);
}

static void Flash_Program(access_type type, bool poll) {
	spi_cmd_data msg;
	msg.padd = ADR_MD;
	if (false==poll) {
		msg.cmd = CMD_WR;
		msg.rw = 0; // write
		msg.o_data = type;
		Serialize_SPI(&msg);
		return;
	}
	else { // poll for completeness
		msg.cmd = CMD_RD;
		msg.rw = 1; // read
		do { Serialize_SPI(&msg);
			} while(type & msg.i_data);
	}
}

static void Flash_Write(uint32_t len, uint32_t *code) {
	spi_cmd_data msg;
	msg.cmd = CMD_RD;
	// poll for readiness
	msg.rw = 1; // read
	msg.padd = ADR_STS;
	do { Serialize_SPI(&msg);
		} while(0x1 & msg.i_data);
	// begin download
	msg.cmd = CMD_WR;
	msg.padd = ADR_WR;
	for (int l=0; l<len; l++) {
		msg.rw = 0; // write
		msg.o_data = *(code+l);
		Serialize_SPI(&msg);
	}
}

 #ifdef FWM_DNLD_DBG
extern void usb_write_buf1(void *pb, int size);
 #endif
/* system/hw image upgrade procedures */
void download_sys_fw(bool sys_hw_mask, uint32_t *upgrade_fw_hdr );
 void download_sys_fw(bool sys_hw_mask, uint32_t *upgrade_fw_hdr )
 {
	// init spi0 channel using manual mode for cpld upgrade
	Spi0PortRxInit() ;
	// CPLD upgrade procedures thru SPI port on Rx atmel
	spi_cmd_data msg;
#ifdef FWM_DNLD_DBG
  	irqflags_t flags;
#endif
	//static uint32_t page = 0;
	uint8_t chk[4] = {
	 		0xff & (*upgrade_fw_hdr>>0),
	 		0xff & (*upgrade_fw_hdr>>8),
	 		 0xff & (*upgrade_fw_hdr>>16),
	 		 0xff & (*upgrade_fw_hdr>>24) };
	uint32_t fw_addr = upgrade_fw_hdr[2],
						fw_len0, fw_len = upgrade_fw_hdr[1],
						len, len1, len2, ll, ll1, crc, crc1, i;
	uint8_t *pb = fw_sys_rbuffer,
					 *pe = pb+FW_DNLD_SIZE;
	uint32_t ul_page_addr = fw_addr ;
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
		while (1) {
			; /* Capture error */
		}
	}
	len = FW_DNLD_SIZE - ((uint32_t)pb - (uint32_t)fw_sys_rbuffer);
	len2 = (len1=FW_DNLD_SIZE) - (ll=len);
	memcpy(fw_sys_tbuffer, pb, len); // has to use the other buffer
	pb = (uint8_t*)fw_sys_tbuffer + len;
	 pe = (uint8_t*)fw_sys_rbuffer + len2;
	fw_len0 = fw_len - len + len2;
	if (sys_hw_mask) {
		// erase current firmware on flash
		Flash_Erase(STM32_ERASE);
		// trigger firmware download
		Flash_Program(STM32_PROGRAM, false);
	}
	else { // download fpga
		Flash_Erase(FPGA_ERASE);
		// trigger firmware download
		Flash_Program(FPGA_PROGRAM, false);
	}
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
#if /*true*/false
		crc = 0;
		for (i=0; i<len1; i++)
			crc ^= ((uint8_t*)fw_sys_tbuffer)[i];
		crc1 = 0;
#endif
		uint32_t ii, addr = 0, *pbuf = fw_sys_tbuffer;
		uint32_t size = ((len1>page_size)?page_size:len1)/sizeof(int);
		if ((ii=(0x3&len1))) {
			switch(ii) {
				case 1: ii =0xffffff00;
								break;
				case 2: ii =0xffff0000;
								break;
				case 3: ii =0xff000000;
								break;
			} // this can ONLY happen at the end of download, liyenho
			*(pbuf+size-1) |= ii; // padd 0xff at the end
		}
#define max(a,b) ((a)>(b))?(a):(b)
		for (i=0; i<max(1,len1/page_size); i++) {
			Flash_Write(size, pbuf);
#if /*true*/false
		//	Flash_Read(size, fw_dbg_buffer);
		for (i=0; i<size*sizeof(int); i++)
			crc1 ^= ((uint8_t*)fw_dbg_buffer)[i];
#endif
			addr = addr + size;
			pbuf = pbuf + size;
			//page = page + 1;
		}
#undef max(a,b)
#if /*true*/false
		if (crc != crc1){
		/*puts("-E-\tfailed on checksum verification.\r");*/
		while (1) {
			; /* Capture error */
		}
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
		ul_page_addr += (len1);
		if (!(fw_len -= len1)) break;
		len1 = (FW_DNLD_SIZE<=fw_len)?FW_DNLD_SIZE:fw_len;
		if (FW_DNLD_SIZE == len) {
			memcpy(fw_sys_tbuffer, pe, ll);
		}
	}
	delay_ms(100);
 #ifdef FWM_DNLD_DBG
		usb_write_buf1(fw_dbg_buffer, 160/*flush dbg dump data on host*/);
 #endif
 	if (sys_hw_mask) // stm32
		Flash_Program(STM32_PROGRAM, true); // wrap up stm32 download
	else  // fpga
		Flash_Program(FPGA_PROGRAM, true); // wrap up fpga download
 }

