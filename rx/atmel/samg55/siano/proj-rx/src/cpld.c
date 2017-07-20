
#include <string.h>
#include <assert.h>
#include "asf.h"		// declaration of all drivers stuffs
#include "conf_usb.h"	// main include to interface atmel spi apis
#include "delay.h"

typedef enum
{
	RetSpiError,
	RetSpiSuccess
} SPI_STATUS;

typedef struct {
	uint8_t cmd;
	uint8_t rw ;  // 0: write, 1: read
	uint16_t padd; // 32 bit alignment on stack
	uint32_t i_data;
	uint32_t o_data;
} spi_cmd_data;

#define SPIBASEADDR		SPI0   // ctrl/sts pipe which interfaces with fpga on tx board, liyenho
#define SPIFLEXCOM			BOARD_FLEXCOM_SPI0
#define SPI_CHIP_SEL		0
#define SPIRDFIFO				(((Spi*)SPI0)->SPI_RDR)
#define SPIWRFIFO				(((Spi*)SPI0)->SPI_TDR)

extern volatile uint8_t spi_tgt_done;
 extern void spi_master_initialize(uint32_t ch, uint32_t base, uint32_t fxcom);
 extern void spi_tx_transfer(void *p_tbuf, uint32_t tsize, void *p_rbuf,
		uint32_t rsize, uint32_t ch) ;
 extern void spi_rx_transfer(void *p_tbuf, uint32_t tsize, void *p_rbuf,
		uint32_t rsize, uint32_t ch);
 extern void usb_read_buf1(void *pb, int size0);

 static uint32_t fw_sys_tbuffer[FW_DNLD_SIZE/sizeof(int)];
 static uint32_t fw_sys_rbuffer[FW_DNLD_SIZE/sizeof(int)];
 static uint32_t fw_dbg_buffer[FW_DNLD_SIZE/sizeof(int)];
 #if defined(FWM_DNLD_DBG)
	volatile bool usb_tgt_active = false;
  extern volatile bool usb_host_active;
#endif

/*******************************************************************************
     SPI 0 Rx controller init
Function:     Spi0PortRxInit()
Arguments:    There is no argument for this function
Return Values:Return RetSpiError if the SPI configuration is not correct.
Description:  This function has to be called at the beginnning to configure the
			  port.

/******************************************************************************/
SPI_STATUS Spi0PortRxInit(void)
{
	 	pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PA9 | PIO_PA10);
	 	pio_set_peripheral(PIOB, PIO_PERIPH_A, PIO_PB0 );
	 	spi_master_initialize(0, SPIBASEADDR, SPIFLEXCOM);
	 	spi_set_clock_polarity(SPIBASEADDR, SPI_CHIP_SEL, 0 /*SPI_CLK_POLARITY*/);
	 	spi_set_clock_phase(SPIBASEADDR, SPI_CHIP_SEL, 1 /*SPI_CLK_PHASE*/);
		spi_configure_cs_behavior(SPIBASEADDR, 0, SPI_CS_KEEP_LOW);
		pio_configure(PIOA, PIO_OUTPUT_1, CPLD_6612_TRIG, 0);  // manual CS control

	delay_ms(50); // Extra waiting for SPI0 Master Ready
	return RetSpiSuccess;
}

#define CPLD_VER		0x80023456		// hardcoded until it is updated again...

#define CHECK_BSY   \
	while (!((Spi*)SPIBASEADDR)->SPI_SR & SPI_SR_TXEMPTY); \
	delay_us(/*10*/2)
#define SET_CS \
	pio_clear(PIOA, CPLD_6612_TRIG);
#define CLEAR_CS \
	pio_set(PIOA, CPLD_6612_TRIG);
#define FLUSHRWFIFO 	delay_us(/*10*/5)
#define DUMMY_BYTE      0xffff  // cannot be all 0s
#define REG_(x)			(*(volatile uint32*)(x))
#define RD_R(REG_)		(REG_)
#define WR_R(REG_, D)	(REG_ = D)

void Serialize_SPI(spi_cmd_data *p) {
	uint16_t tx_len, rx_len, tmp, *pw;
	uint8_t *pb = &p->cmd;

	CHECK_BSY;
	SET_CS;
	// send command byte
	WR_R(SPIWRFIFO,  SPI_TDR_TD(0xff & *pb));
		CHECK_BSY;

	if (p->rw) {
#if true
		rx_len = sizeof(uint32_t);
		pb = ((uint8_t*)&p->i_data)+3;
#else
		rx_len = sizeof(uint32_t)/sizeof(uint16_t);
		pw = ((uint16_t*)&p->i_data)+1;
#endif
		tx_len = 0;
	} else {
		rx_len = 0;
#if true
		tx_len = sizeof(uint32_t);
		pb = ((uint8_t*)&p->o_data)+3;
#else
		tx_len = sizeof(uint32_t)/sizeof(uint16_t);
		pw = ((uint16_t*)&p->o_data)+1;
#endif
	}
	while (tx_len-- > 0)
	{
#if true
		WR_R(SPIWRFIFO,  SPI_TDR_TD(*pb--));
		CHECK_BSY;
		RD_R(SPIRDFIFO & SPI_RDR_RD_Msk);
#else
		spi_tgt_done = true;
		spi_tx_transfer(pw--, 2, &tmp, 2, 0);
		while (spi_tgt_done) ;
#endif
	}
#if true
	tmp = DUMMY_BYTE;
#endif
	while (rx_len-- > 0)
	{
#if true
		WR_R(SPIWRFIFO,  SPI_TDR_TD(DUMMY_BYTE));
		CHECK_BSY;
		*pb-- = RD_R(SPIRDFIFO & SPI_RDR_RD_Msk);
#else
		spi_tgt_done = true;
		spi_rx_transfer(&tmp, 2, pw--, 2, 0);
		while (spi_tgt_done) ;
#endif
	}

	CLEAR_CS;
	FLUSHRWFIFO;
}

void Page_Erase(uint32_t page) {
	spi_cmd_data msg;
	msg.cmd = 0x3;
	msg.rw = 0; // write mode
	msg.o_data = (0xf<<28)|(0<<27)|(0xf<<23) | (0x5<<20) | 0xfffff; // erase pages on CFM0
	Serialize_SPI(&msg);
}

void Set_Address(uint32_t addr) {
	spi_cmd_data msg;
	msg.cmd = 0x4;
	msg.rw = 0; // write mode
	msg.o_data = addr;
	Serialize_SPI(&msg);
}

void Flash_Write(uint32_t len, uint32_t *code) {
	spi_cmd_data msg;
	msg.cmd = 0x6;
	msg.rw = 0; // write mode
	for (int l=0; l<len; l++) {
		msg.o_data = *(code+l);
		Serialize_SPI(&msg);
	}
}

void Flash_Read(uint32_t len, uint32_t *data) {
	spi_cmd_data msg;
	msg.cmd = 0x7;
	msg.rw = 1; // read mode
	for (int l=0; l<len; l++) {
		Serialize_SPI(&msg);
		*(data+l) = msg.i_data;
	}
}

 #ifdef FWM_DNLD_DBG
   extern void usb_write_buf1(void *pb, int size);
 #endif
/* CPLD image upgrade procedures */
 void download_cpld_fw(cpld_flash_map* fdo, uint32_t *upgrade_fw_hdr )
 {
	// init spi0 channel using manual mode for cpld upgrade
	Spi0PortRxInit() ;
	// CPLD upgrade procedures thru SPI port on Rx atmel
	spi_cmd_data msg;
	/* get CPLD version */
		msg.cmd = 0x1;
		msg.rw = 1; // read mode
		Serialize_SPI(&msg);
	if (CPLD_VER != msg.i_data)
		while (1); //printf("cpld version=0x%08x, read back=0x%08x\n",CPLD_VER,msg.i_data);
#ifdef FWM_DNLD_DBG
  	irqflags_t flags;
#endif
	static uint32_t page = 0;
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
	assert(!(fw_addr & (fdo->page_size-1)));
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
		crc1 = 0;
#endif
		uint32_t addr = 0, *pbuf = fw_sys_tbuffer;
		uint32_t size = ((len1>fdo->page_size)?fdo->page_size:len1)/sizeof(int);
		for (i=0; i<len1/fdo->page_size; i++) {
			Page_Erase(page);
			Set_Address(/*ul_page_addr+*/addr);
	Flash_Read(size, fw_dbg_buffer);  // for debug, liyenho
			Flash_Write(size, pbuf);
			Flash_Read(size, fw_dbg_buffer);
#if true
		for (i=0; i<size*sizeof(int); i++)
			crc1 ^= ((uint8_t*)fw_dbg_buffer)[i];
#endif
			addr = addr + size;
			pbuf = pbuf + size;
			page = page + 1;
		}
#if true
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

 }

