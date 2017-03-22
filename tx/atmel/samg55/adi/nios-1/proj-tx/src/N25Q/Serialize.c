/**********************  DRIVER FOR SPI CONTROLLER ON ORION**********************

   Filename:    Serialize.c
   Description:  Support to . This files is aimed at giving a basic example of
   SPI Controller to simulate the SPI serial interface.

   Version:    0.2
   Date:       Decemb. 2011
   Authors:    Micron S.r.l. Arzano (Napoli)


   THE FOLLOWING DRIVER HAS BEEN TESTED ON ORION WITH FPGA RELEASE 1.4 NON_DDR.

   Since there is no dedicated QUAD SPI controller on PXA processor, the peripheral is
   synthesized on FPGA and it is interfaced with CPU through memory controller. It is
   implemented on chip select-5 region of PXA processor and communicates with device using
   32bits WRFIFO and RDFIFO. This controller is aligned with  Micron SPI Flash Memory.
   That's mean that in extended, dual and quad mode works only with command of these memory.

   These are list of address for different SPI controller registers:

   			Chip Base  is mapped at 0x16000000

   Register         |     Address          |    Read/Write
                    |                      |
   RXFIFO           | (Chip Base + 0x0)    |      Read
   WRFIFO           | (Chip Base + 0x4)    |      Write
   Control Register | (Chip Base + 0x8)    |      R/W
   Status Register  | (Chip Base + 0xC)    |      Read



********************************************************************************

*******************************************************************************/
#include "assert.h"
#include "N25Q.h"
#include "Serialize.h"
#include "delay.h"  // for delay_xx()
  static uint32_t fw_sys_tbuffer[FW_DNLD_SIZE/sizeof(int)];
  static uint32_t fw_sys_rbuffer[FW_DNLD_SIZE/sizeof(int)];
  static uint32_t fw_dbg_buffer[FW_DNLD_SIZE/sizeof(int)];
 #ifndef MEDIA_ON_FLASH
 	#define flash_erasure_buff_size	65536
 #else  // due to its huge size, it can't be on while storing media on flash
 	#define flash_erasure_buff_size	0
 #endif
  static uint32_t flash_erasure_buff[flash_erasure_buff_size/sizeof(int)];  // see if this can be created?
  static uint8_t *pba = (uint8_t*)flash_erasure_buff;
//#define ENABLE_PRINT_DEBUG
#if defined(FWM_DNLD_DBG)
	volatile bool usb_tgt_active = false;
  extern volatile bool usb_host_active;
#endif
	/*static*/ bool first_flash_access = true;
  extern /*inline*/ void usb_read_buf1(void *pb, int size0);
  extern /*inline*/ void usb_write_buf1(void *pb, int size0);

/* FPGA image upgrade procedures taken from Siano download in Rx design */
  void download_fpga_fw(uint32_t *upgrade_fw_hdr )
 {
#ifdef FWM_DNLD_DBG
  	irqflags_t flags;
#endif
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
	volatile bool erase = true;
	static FLASH_DEVICE_OBJECT fdo; // don't want to see stray garbage data fields
	ReturnType ret= Driver_Init(&fdo);
	assert(Flash_Success == ret);
	assert(!(fw_addr & (fdo.Desc.FlashPageSize-1)));
	uint32_t ul_page_addr = fw_addr ;
	uint32_t lm, le, er_adr =ul_page_addr-(ul_page_addr & (fdo.Desc.FlashSectorSize-1));
	uint32_t erlen = fdo.Desc.FlashSectorSize, dnshf = fdo.Desc.FlashSectorSize_bit;
	lm = (ul_page_addr & (fdo.Desc.FlashSectorSize-1))+FW_DNLD_SIZE;
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
	// buffer last chunk existing data on flash prior to erase, only done once
	ParameterType para;
		para.Read.udAddr = er_adr;
		para.Read.pArray = pba;
		para.Read.udNrOfElementsToRead = fw_addr & (fdo.Desc.FlashSectorSize-1);
		if (Flash_Success != fdo.GenOp.DataRead(Read, &para)){
			/*puts("-E-\tfailed on DataRead().\r");*/
			while (1) {
				; /* Capture error */
			}
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
#if true
		crc = 0;
		for (i=0; i<len1; i++)
			crc ^= ((uint8_t*)fw_sys_tbuffer)[i];
#endif
		if (erase) {
			if (Flash_Success != fdo.GenOp.SectorErase(er_adr >> dnshf)){
				/*puts("-E-\tfailed on SectorErase().\r");*/
				while (1) {
					; /* Capture error */
				}
			} // NEVER place break point inside erase{.} when debug!!!!!!! liyenho
			if (first_flash_access && (fw_addr & (fdo.Desc.FlashSectorSize-1))) {
				// put back the existing data on flash
				para.PageProgram.udAddr = er_adr;
				para.PageProgram.pArray = pba;
				para.PageProgram.udNrOfElementsInArray = fw_addr & (fdo.Desc.FlashSectorSize-1);
				if (Flash_Success != fdo.GenOp.DataProgram(PageProgram, &para)){
					/*puts("-E-\tfailed on DataProgram().\r");*/
					while (1) {
						; /* Capture error */
					}
				}
				first_flash_access = false;
			}
			erase = false;
		}
		para.PageProgram.udAddr = ul_page_addr;
		para.PageProgram.pArray = fw_sys_tbuffer;
		para.PageProgram.udNrOfElementsInArray = len1;
		if (Flash_Success != fdo.GenOp.DataProgram(PageProgram, &para)){
			/*puts("-E-\tfailed on DataProgram().\r");*/
			while (1) {
				; /* Capture error */
			}
		}
		para.Read.udAddr = ul_page_addr;
		para.Read.pArray = fw_dbg_buffer;
		para.Read.udNrOfElementsToRead = len1;
		if (Flash_Success != fdo.GenOp.DataRead(Read, &para)){
			/*puts("-E-\tfailed on DataRead().\r");*/
			while (1) {
				; /* Capture error */
			}
		}
#if true
		crc1 = 0;
		for (i=0; i<len1; i++)
			crc1 ^= ((uint8_t*)fw_dbg_buffer)[i];
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
	delay_ms(100);
 #ifdef FWM_DNLD_DBG
		usb_write_buf1(fw_dbg_buffer, 160/*flush dbg dump data on host*/);
 #endif

 }

/*******************************************************************************
     SPI 0 Tx controller init
Function:     Spi0PortTxInit()
Arguments:    There is no argument for this function
Return Values:Return RetSpiError if the SPI configuration is not correct.
Description:  This function has to be called at the beginnning to configure the
			  port.

/******************************************************************************/
SPI_STATUS Spi0PortTxInit(void)
{
	 	pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PA9 | PIO_PA10);
	 	pio_set_peripheral(PIOB, PIO_PERIPH_A, PIO_PB0 );
	 	spi_master_initialize(0, SPIBASEADDR, SPIFLEXCOM);
	 	spi_set_clock_polarity(SPIBASEADDR, SPI_CHIP_SEL, 0 /*SPI_CLK_POLARITY*/);
	 	spi_set_clock_phase(SPIBASEADDR, SPI_CHIP_SEL, 1 /*SPI_CLK_PHASE*/);
	 	spi_set_bits_per_transfer(SPIBASEADDR, 0, SPI_CSR_BITS_8_BIT); // 8 bit transfer
		spi_configure_cs_behavior(SPIBASEADDR, 0, SPI_CS_KEEP_LOW);
		pio_configure(PIOA, PIO_OUTPUT_1, PIO_PA25, 0);  // manual CS control

	delay_ms(50); // Extra waiting for SPI0 Master Ready
	return RetSpiSuccess;

}

/*******************************************************************************
Function:     ConfigureSpi(SpiConfigOptions opt)
Arguments:    opt configuration options, all acceptable values are enumerated in
              SpiMasterConfigOptions, which is a typedefed enum.
Return Values:There is no return value for this function.
Description:  This function can be used to properly configure the SPI master
              before and after the transfer/receive operation
Pseudo Code:
   Step 1  : perform or skip select/deselect slave
   Step 2  : perform or skip enable/disable transfer
   Step 3  : perform or skip enable/disable receive
*******************************************************************************/

void ConfigureSpi(SpiConfigOptions opt)
{
	switch (opt)
	{
	case OpsWakeUp:
		CHECK_BSY;
		SET_CS;
		break;
	case OpsInitTransfer:
		FLUSHRWFIFO;
		break;
	case OpsEndTransfer:
		CLEAR_CS;
		FLUSHRWFIFO;
		break;
	default:
		break;
	}
}



/*******************************************************************************
Function:     Serialize(const CharStream* char_stream_send,
					CharStream* char_stream_recv,
					SpiMasterConfigOptions optBefore,
					SpiMasterConfigOptions optAfter
				)
Arguments:    char_stream_send, the char stream to be sent from the SPI master to
              the Flash memory, usually contains instruction, address, and data to be
              programmed.
              char_stream_recv, the char stream to be received from the Flash memory
              to the SPI master, usually contains data to be read from the memory.
              optBefore, configurations of the SPI master before any transfer/receive
              optAfter, configurations of the SPI after any transfer/receive
Return Values:TRUE
Description:  This function can be used to encapsulate a complete transfer/receive
              operation
Pseudo Code:
   Step 1  : perform pre-transfer configuration
   Step 2  : perform transfer/ receive
   Step 3  : perform post-transfer configuration
*******************************************************************************/
SPI_STATUS Serialize_SPI(const CharStream* char_stream_send,
                         CharStream* char_stream_recv,
                         SpiConfigOptions optBefore,
                         SpiConfigOptions optAfter
                        )
{

	uint8 *char_send, *char_recv;
	uint16 rx_len = 0, tx_len = 0;

#ifdef ENABLE_PRINT_DEBUG
	int i;
	printf("\nSEND: ");
	for(i=0; i<char_stream_send->length; i++)
		printf(" 0x%x ", char_stream_send->pChar[i]);
	printf("\n");
#endif

	tx_len = char_stream_send->length;
	char_send = char_stream_send->pChar;

	if (NULL_PTR != char_stream_recv)
	{
		rx_len = char_stream_recv->length;
		char_recv = char_stream_recv->pChar;
	}



	ConfigureSpi(optBefore);

	while (tx_len-- > 0)
	{
		WR_R(SPIWRFIFO,  SPI_TDR_TD(*(char_send++)));
		CHECK_BSY;

		RD_R(SPIRDFIFO & SPI_RDR_RD_Msk);
	}

	while (rx_len-- > 0)
	{
		WR_R(SPIWRFIFO,  SPI_TDR_TD(DUMMY_BYTE));
		CHECK_BSY;

//		if (CHECK_RX_FIFO)
			*char_recv++ = RD_R(SPIRDFIFO & SPI_RDR_RD_Msk);
//		else
//			rx_len++;
	}

#ifdef ENABLE_PRINT_DEBUG
	printf("\nRECV: ");
	for(i=0; i<char_stream_recv->length; i++)
		printf(" 0x%x ", char_stream_recv->pChar[i]);
	printf("\n");
#endif
	ConfigureSpi(optAfter);


	return RetSpiSuccess;
}
