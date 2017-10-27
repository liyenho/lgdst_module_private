#include <asf.h>
//#include <stdio.h>
#include <string.h>
//#include <unistd.h>
#include "rf2072_set.h"
#include "it9510.h"
#include "platform_it9517.h"

#include <math.h>
#include <delay.h>
#include <assert.h>
#include "main.h"
#include "ctrl.h"

extern volatile uint32_t *DWT_CYCCNT;
extern volatile uint8_t spi_tgt_done;
extern volatile bool ctrl_tdma_lock;
extern twi_packet_t packet_tx;
extern volatile uint32_t it951x_fw_hdr[1];
extern volatile context_it951x ctx_951x;
extern volatile uint8_t main_loop_on;
extern volatile uint8_t vch;
extern IT9510INFO eagle;

const uint32_t vch_tbl[] = {
	2392, 2406, 2413, 2420,
	2427, 2434, 2441, 2448,
	2455, 2462, 2469
};

static int pwr_attn=10000;
static int chsel_2072 = 0;
static uint32_t fw_ite_tbuffer[FW_DNLD_SIZE/sizeof(int)];
uint32_t fw_ite_rbuffer[FW_DNLD_SIZE/sizeof(int)];

#if USE_UART
 #define HALF10_SEC	6000000  // assumed 120 mhz atmel core clk
 COMPILER_ALIGNED(8) \
  	static char dbg_msg_buff[DBG_BUF_SIZE] = {0};
	// debugger info dump under mavlink arch
	ctx_debug_atmel logger = {0};
#endif

//function prototyping
extern void spi_tx_transfer(void *p_tbuf, uint32_t tsize, void *p_rbuf,
		uint32_t rsize, uint32_t ch);
extern void spi_rx_transfer(void *p_tbuf, uint32_t tsize, void *p_rbuf,
		uint32_t rsize, uint32_t ch);
extern void usb_read_buf1(void *pb, int size);
extern int timedelta(bool reset, unsigned int bignum, unsigned int smallnum);

	/*static inline*/ void usb_write_buf1(void *pb, int size0, bool log)
	{
		uint32_t tm_entry, tm_curr;
		int written=0, size = size0;
		if (log) // in atmel logging mode
			tm_entry= *DWT_CYCCNT;
		do {
			iram_size_t b = size-udi_cdc_write_buf(pb, size);
			pb += b;
			size -= b;
			written += b;
			if (log) {
				if (0 != b)
					tm_entry= *DWT_CYCCNT;
				else {
					tm_curr= *DWT_CYCCNT;
			#if USE_UART
					int tdel =timedelta(false, tm_curr, tm_entry);
					if (HALF10_SEC <=tdel) {
						logger.active = false;  // not active
						return ;
					}
			#endif
				}
			}
		} while (size0 != written);
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

 void download_ite_fw(uint32_t fw_dnld_size, uint8_t address, uint8_t *chk )
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
		usb_write_buf1(fw_dbg_buffer, len, false);
  		ul_page_addr += len;
  	} while (0<rem);
 #endif
	delay_ms(400);
 }

 void rffe_write_regs(dev_cfg* pregs) {
	uint16_t tmp, tmpw, *pth = &tmp;
#if USE_UART
	char dbg_msg[255], dbg_len, *pdbg ;
	sprintf(dbg_msg, "enter %s\n", __func__);
	dbg_len = strlen(dbg_msg);
	TRANSFER_LOG
#endif
	//RF2072_WRITE:
			while (spi_tgt_done) ; // flush any pending spi xfer
			spi_tgt_done = true;
			ACCESS_PROLOG_2072
			// setup spi to write addressed data
			*pth = 0x7f & pregs->addr; // write access
			spi_tx_transfer(pth, 1, &tmpw, 1, 0/*ctrl/sts*/);
			while (spi_tgt_done) ;
			spi_tgt_done = true;
			*pth = 0xff&(pregs->data>>8); // high byte
			spi_tx_transfer(pth, 1, &tmpw, 1, 0/*ctrl/sts*/);
			while (spi_tgt_done) ; spi_tgt_done = true;
			*pth = 0xff&pregs->data; // low byte
			spi_tx_transfer(pth, 1, &tmpw, 1, 0/*ctrl/sts*/);
			while (spi_tgt_done) ;
		  //delay_us(1);
			pio_set(PIOA, CPLD_2072_TRIG);
#if USE_UART
	sprintf(pdbg, "exit %s, %x\n", __func__, (int)pregs->data);
	dbg_len += strlen(pdbg);
	TRANSFER_LOG
#endif
}

uint16_t rffe_read_regs(dev_cfg* pregs) {
	uint8_t msg[80];
#if USE_UART
	char dbg_msg[80], dbg_len, *pdbg ;
	sprintf(dbg_msg, "enter %s\n", __func__);
	dbg_len = strlen(dbg_msg);
	TRANSFER_LOG
#endif
	dev_access *pr= (dev_access*)msg;
	uint16_t tmp, tmpw, *pth = &tmp;
	//RF2072_READ:
		while (spi_tgt_done) ; // flush any pending spi xfer
		spi_tgt_done = true;
		ACCESS_PROLOG_2072
		*pth = 0x80| (0x7f&pregs->addr); // read access
		spi_tx_transfer(pth, 1, &tmpw, 1, 0);
		while (spi_tgt_done);
		spi_tgt_done = true;
		READ_MID_PROC_2072
		spi_rx_transfer(pth, 1, &tmpw, 1, 0/*ctrl/sts*/); // high byte
		while (spi_tgt_done) ; spi_tgt_done = true;
		pr->data[1] = 0xff & tmpw;
		spi_rx_transfer(pth, 1, &tmpw, 1, 0/*ctrl/sts*/); // low byte
		while (spi_tgt_done) ;
		pr->data[0] = (0xff & tmpw);
		READ_END_REV_2072
#if USE_UART
	sprintf(pdbg, "exit %s, %x\n", __func__, (int)pr->data);
	dbg_len += strlen(pdbg);
	TRANSFER_LOG
#endif
		return *(uint16_t*)pr->data;
}

int set_frequency_rf2072(uint32_t f_lo_KHz)
{
	dev_cfg regs;
	uint16_t writeValue, P2_FREQ1,P2_FREQ2,P2_FREQ3;

	int i;
	uint32_t error;

	uint32_t dw_temp, dw_temp1000, dw_temp2, dw_ndiv1000,dw_fvco_KHz;
	uint8_t n_lo2, lodiv2,fbkdiv2, numlsb2,p2presc2,p2lodiv2;
	uint16_t n2 ,nummsb2 ;
#if USE_UART
	char dbg_msg[80], dbg_len, *pdbg ;
	sprintf(dbg_msg, "enter %s\n", __func__);
	dbg_len = strlen(dbg_msg);
	TRANSFER_LOG
#endif

	dw_temp = 5400000;
	dw_temp2 = dw_temp/(f_lo_KHz);
	dw_temp = log2(dw_temp2);
	n_lo2 = (uint8_t)dw_temp;
	p2lodiv2 = n_lo2;
	//lodiv =pow (2.0, n_lo);// 2^n_lo;
	lodiv2 = 1;
	for(i=0;i<n_lo2;i++)
		lodiv2 = lodiv2 * 2;


	dw_fvco_KHz = lodiv2 * f_lo_KHz;

	if(dw_fvco_KHz>3200000){
		fbkdiv2 = 4;

		p2presc2 = 2;

		//pllcpl to 3 to do ???
	}else{
		fbkdiv2 = 2;

		p2presc2 = 1;

	}
	dw_ndiv1000 = (dw_fvco_KHz*10)/fbkdiv2/26;

	n2 =  (uint16_t) (dw_fvco_KHz/fbkdiv2/26000);
	dw_temp1000 = (65536*(dw_ndiv1000-n2*10000));
	nummsb2 = (uint16_t)((65536*(dw_ndiv1000-n2*10000))/10000);
	numlsb2 = (uint8_t)((256*(dw_temp1000-nummsb2*10000))/10000);
	//-------------------------------------------


	P2_FREQ1 = n2<<7 | p2lodiv2<<4 | p2presc2<<2 | 0x02;
	P2_FREQ2 = nummsb2;
	P2_FREQ3 = numlsb2<<8;

	//printf("P2_FREQ1=%x,P2_FREQ2=%x,P2_FREQ3=%x\n",P2_FREQ1,P2_FREQ2,P2_FREQ3);

	regs.addr = 0x08;
	regs.data = (0xFC06 & 0x7FFE) | 0x8000;
	rffe_write_regs(&regs);

	delay_ms(10); 	// validate echo after 0.01 sec
	//printf("xxxxxx 0x%04x @ 0x%x written xxxxxx\n",regs.data,regs.addr);

	regs.addr = 0x0F;
	regs.data = P2_FREQ1;
	rffe_write_regs(&regs);

	delay_ms(10); 	// validate echo after 0.01 sec
	//printf("xxxxxx 0x%04x @ 0x%x written xxxxxx\n",regs.data,regs.addr);

	regs.addr = 0x10;
	regs.data = P2_FREQ2;
	rffe_write_regs(&regs);

	delay_ms(10); 	// validate echo after 0.01 sec
	//printf("xxxxxx 0x%04x @ 0x%x written xxxxxx\n",regs.data,regs.addr);

	//libusb_control_transfer(devh,CTRL_OUT, USB_RQ,USB_HOST_MSG_TX_VAL,USB_HOST_MSG_IDX,(unsigned char*)acs, sizeof(*acs)+(acs->dcnt-1), 0);
	regs.addr = 0x11;
	regs.data = P2_FREQ3;
	rffe_write_regs(&regs);

	delay_ms(10); 	// validate echo after 0.01 sec
	//printf("xxxxxx 0x%04x @ 0x%x written xxxxxx\n",regs.data,regs.addr);

#if USE_UART
	sprintf(pdbg, "exit %s, %x\n", __func__, 0);
	dbg_len += strlen(pdbg);
	TRANSFER_LOG
#endif
	return 0;
}

int init_rf2072(void)
{
#if USE_UART
	char dbg_msg[80], dbg_len, *pdbg ;
	sprintf(dbg_msg, "enter %s\n", __func__);
	dbg_len = strlen(dbg_msg);
	TRANSFER_LOG
#endif
	//RF2072_RESET:
		pio_clear (PIOA, PIO_PA26);
		delay_ms(200);
		pio_set (PIOA, PIO_PA26);
	delay_ms(100);

	dev_cfg regs,*pregs=GET_ARRAY(chsel_2072);
		for (int32_t i=0; i<ARRAY_SIZE(chsel_2072); i++,pregs++)
	//RF2072_WRITE:
			rffe_write_regs(pregs);

	set_frequency_rf2072(LO_Frequency);
	delay_ms(100);

	regs.addr = 0x09;
	regs.data =((0x8224&0xFFF7) | 0x0008);
	rffe_write_regs(&regs);

	delay_ms(10);
	//printf("xxxxxx 0x%04x @ 0x%x written xxxxxx\n",regs.data,regs.addr);

	regs.addr = 0x1D;
	regs.data = 0x1001;
	rffe_write_regs(&regs);

	delay_ms(10);
	//printf("setup device control = 0x%04x\n",*(uint16_t*)acs->data);

	regs.addr = 0x1F;
	int locked = 0!=(0x8000&rffe_read_regs(&regs));
#if USE_UART
	sprintf(pdbg, "exit %s, %x\n", __func__, locked);
	dbg_len += strlen(pdbg);
	TRANSFER_LOG
#endif
	return locked;
	//printf("tx rffe is running...\n");
}

#if USE_UART
void atmel_debugger_dump() {
	if (!logger.active)
		goto skip ;  // not inited yet...
	else
		while(1) {
			if (DBG_BUF_SIZE==logger.str_len)
				break; // work around flash corruption?
		}
	//assert(DBG_BUF_SIZE==logger.str_len);
	// transfer include all null terminator chars
	usb_write_buf1(dbg_msg_buff, DBG_BUF_SIZE, true);
skip:
	// reset dump context
	logger.str_len = 0 ;
}
#endif

#define SCHEME_RETRY(it9517_func_call, err_val) \
	err_cnt = 0; \
	do { \
		error=it9517_func_call ; \
		if (5<=err_cnt++) \
			{error=err_val; goto exit;} \
		if (Error_I2C_READ_FAILED == error || \
			 Error_I2C_WRITE_FAILED == error) { /*i2c nack breaker from Shaq*/ \
				uint8_t value = 0x01; \
				IT9510_writeRegisters (&eagle, Processor_LINK, 0x4900, 1, &value); \
		} \
	} while(error );

int init_video_subsystem(void)
{
	uint16_t bandwidth = 6000;
	uint16_t sawBandwidth = 8000;
	uint32_t error = ModulatorError_NO_ERROR,
						err_cnt ; // to retry from error, liyenho
#if USE_UART
	// initialize atmel debug facility, mainly for monitoring ite intf
	if (logger.primed)
		logger.active = true;
	logger.str_len = 0/*DBG_BUF_SIZE*/;
	logger.str_buffer = dbg_msg_buff;
#endif

	init_rf2072();
	//	goto exit; //liyen says checking response of init_rf2072 not necessary
	SCHEME_RETRY(it9517_initialize (Bus_I2C,SERIAL_TS_INPUT), 1)
	//error= it9517_loadIQ_calibration_table (const char*file_name);
	//if(error)goto exit;
	SCHEME_RETRY(it9517_reset_pidfilter(), 2)
	SCHEME_RETRY(it9517_control_pidfilter(0,0), 3)
//	puts ("video subsystem initialized...");
	return 0;
 exit:
	//printf("error=%x,%d\n",error,__LINE__);
	while (1) {;} // sticky error exception
}

int start_video_subsystem(void)
{
	static bool boot_state = false;
	uint32_t error = ModulatorError_NO_ERROR,
						err_cnt ;	 // to retry from error, liyenho
	ChannelModulation      channel_Modulation;
	if (boot_state)
		SCHEME_RETRY(it9517_enable_transmission_mode(0), 4)
	//	channel_Modulation.frequency=/*802000*/713000;
	//	channel_Modulation.bandwidth=6000;
	channel_Modulation.constellation=Constellation_QPSK;
	channel_Modulation.highCodeRate=CodeRate_1_OVER_2;
	channel_Modulation.interval=Interval_1_OVER_32;
	channel_Modulation.transmissionMode=TransmissionMode_2K;
	SCHEME_RETRY(it9517_set_channel_modulation( channel_Modulation,2), 1)
#if /*true*/ false   // dynamic video channel selection
	err_cnt = 0;
	do {
		if (boot_state && it9517_video_channel_select()) {
			if (2<=err_cnt++)
				{ error=5; goto exit ; }}
		else boot_state = true; // bypass vchan select ops during first boot
	} while(error );
#else
	if (sizeof(vch_tbl)>vch)
		SCHEME_RETRY(it9517_acquire_channel(vch_tbl[vch]*1000-LO_Frequency,6000), 2)
	//error=it9517_get_output_gain();
	//if(error)goto exit;
	//error=it9517_get_output_gain_range(/*809000*/720000,6000);
	//if(error)goto exit;
	SCHEME_RETRY(it9517_adjust_output_gain(0), 3)
	//	error = it9517_reset_pidfilter();
	//	if(error)goto exit;
	//	error= it9517_control_pidfilter(0,1);
	//	if(error)goto exit;
	//error=it9517_add_pidfilter(0, 0x100);
	//if(error)goto exit;
	//	error=it9517_pcr_restamp(PcrModeDisable,1);
	//	if(error)goto exit;
	SCHEME_RETRY(it9517_enable_transmission_mode(1), 4)
	boot_state = true;
#endif
	main_loop_on = true;  // enter run time stage
	return 0;
 exit:
	//printf("error=%x,%d\n",error,__LINE__);
	while (1) {;} // sticky error exception
}

#ifdef TIME_ANT_SW
void configure_rtt(unsigned int clkcnt);
extern volatile bool stream_flag;
extern volatile int vid_ant_switch;
/**********************************************************************************************************
 * \brief Interrupt handler for the RTT.
 *
 */
void RTT_Handler(void)
{
	// rtt clkcnt=16: period range =  d419 to 1d059 (452us - 1.4ms)
	uint32_t ul_status;
    /* Get RTT status */
	ul_status = rtt_get_status(RTT);

	/* Time has changed, refresh display */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {
		if (stream_flag)
			vid_ant_switch = true; // activate vid ant sw
		return;
	}//if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC)
	/* Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
          //Error condition, should not happen
		return;
	}
}
/**********************************************************************************************************
 * \brief RTT configuration function.
 *
 * Configure the RTT to generate a one second tick, which triggers the RTTINC
 * interrupt.
 */
void configure_rtt(unsigned int clkcnt )
{
	//tick period = clkcnt*30.5us
	//e.g.: 16 = 500us
	uint32_t ul_previous_time;

	/* Configure RTT for a 1 second tick interrupt */
#if SAM4N || SAM4S || SAM4E || SAM4C || SAM4CP || SAM4CM || SAMV71 || SAMV70 || SAME70 || SAMS70
	rtt_sel_source(RTT, false);
#endif

	rtt_init(RTT, clkcnt); // fired every sec (0 gives about 2 sec)

	ul_previous_time = rtt_read_timer_value(RTT);
	while (ul_previous_time == rtt_read_timer_value(RTT));

	rtt_write_alarm_time(RTT, clkcnt ); // fired every sec (0 gives about 2sec)

	/* Enable RTT interrupt */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 15);  // RTT take lower priority than usb?
	NVIC_EnableIRQ(RTT_IRQn);
	rtt_enable_interrupt(RTT, RTT_MR_RTTINCIEN);
}
#endif