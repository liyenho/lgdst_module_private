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
#include "taisync.h"

#include <math.h>
//#define DEBUG_VIDEOPIPE
 extern int timedelta(bool reset, unsigned int bignum, unsigned int smallnum);
#ifdef TIME_ANT_SW
  volatile uint32_t startup_video_tm= 0, // to measure hold off time
  										last_done_spi= 0;
  volatile int32_t intv_max= -1,
  									intv_min= 120000000;
  volatile bool startup_meas =false;
#endif
#ifdef VIDEO_DUAL_BUFFER
	volatile int32_t cc, st_pos, stream = -1; // invalidated
	volatile static uint8_t lkup_video_buffer[TSLUT_BUFFER_SIZE+I2SC_BUFFER_SIZE],
													new_video_buffer[I2SC_BUFFER_SIZE],
													*pbn /*new ts buf ptr*/;
	const uint32_t *pblw = (uint32_t*)(lkup_video_buffer+TSLUT_BUFFER_SIZE),
								 	*pblr = (uint32_t*)lkup_video_buffer;
#endif
#ifdef CTRL_RADIO_ENCAP
	uint8_t pb_rdo_ctrl[I2SC_BUFFER_SIZE],
					*pb_rdo_ctrl_e= pb_rdo_ctrl+sizeof(pb_rdo_ctrl);
	static uint8_t *pbr= pb_rdo_ctrl;
	extern void radio_pkt_filled(int bsz);
#endif
#if /*defined(SMS_DVBT2_DOWNLOAD) ||*/ defined(RECV_TAISYNC)
 extern uint32_t g_ul_wait_10ms;
 extern twi_packet_t packet_tx, packet_rx;
 extern volatile uint32_t sms4470_fw_hdr;
 extern volatile uint32_t *DWT_CYCCNT;
 #ifndef RX_SPI_CHAINING
  extern bool i2c_read_cb_on;
 #endif
 extern Pdc *g_p_spim_pdc[];
 extern uint8_t spibuff_wrptr_filled;
 extern uint8_t spibuff_wrptr_currentlyfilling;
 extern uint8_t spibuff_wrptr_yettobefilled;
 extern uint8_t spibuff_rdptr;
 extern uint8_t spidma_active;
 extern uint32_t gs_uc_rbuffer[GRAND_BUFFER_SIZE/sizeof(int)];
 extern unsigned char dbg_spififo_lvl_max;

 extern unsigned int log2a[100];
 extern unsigned char log2p;

unsigned int dbg_usbtransfercnt=0;
unsigned int dbg_usbtransferfail=0;
#ifndef RECV_TAISYNC
  volatile static uint32_t ts_count_error=0;
  volatile static uint32_t last_half_sec= 0;
  //bypass transient stage during video startup
  volatile bool reset_ts_count_error=true;
  volatile bool gl_vid_ant_sw = false; // antenna switch request
#endif
extern unsigned char trig500ms;
extern uint8_t mon_ts47bad_cnt;

extern void usb_write_buf1(void *pb, int size);

/**********************************************************************************************************
 * \brief Interrupt handler for the RTT.
 *
 */
void RTT_Handler(void)
{
	// rtt clkcnt=16: period range =  d419 to 1d059 (452us - 1.4ms)
	uint32_t ul_status;
	unsigned int spi_pdcrxcnt_next;
    pdc_packet_t dma_chain;
	unsigned int udi_cdc_lvl;
	unsigned int cc_curr;
	static unsigned char ts47badcnt_pre=0;
#if defined(DEBUG_VIDEOPIPE) || defined(TIME_ANT_SW)
	static unsigned int cc_prev=0;
	unsigned int cc_next;
#endif
	int i, ii;
	                                                            #ifdef DEBUG_VIDEOPIPE
																					//monitoring/debugging variables
																					unsigned int cpucyclecntlocal;
																					static unsigned int rttcnt=0;
																					static unsigned int cpumin=0xffffffff, cpumax=0;
																					static unsigned int dbg_spidmaov=0;
																					static unsigned int dbg_spidmacnt=0;
																					unsigned char dbg_spififo_lvl,spibuff_rdptr_l;
																					static unsigned int dbg_ccerr=0;
																					#endif
    /* Get RTT status */
	ul_status = rtt_get_status(RTT);

	/* Time has changed, refresh display */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {
																				#ifdef DEBUG_VIDEOPIPE
																					  // isr stability benchmarking -----------------------
																					  cpucyclecntlocal = *DWT_CYCCNT;
																					 *DWT_CYCCNT = 0;
																					 if(rttcnt>10) {
																					   if(cpumin> cpucyclecntlocal) cpumin= cpucyclecntlocal;
																					   if(cpumax< cpucyclecntlocal) cpumax = cpucyclecntlocal;
																					 }
																					rttcnt++;
																					if(rttcnt>1000) {
																						trig500ms=1;
																						rttcnt=0;
																					}
																				#endif

  if (spidma_active)
  {
	// SPI Chain Management ------------------------------------------------------
	spi_pdcrxcnt_next = pdc_read_rx_next_counter(g_p_spim_pdc[1]);
	if(spi_pdcrxcnt_next ==0){ // spidma buffer ptr updata
#if defined(TIME_ANT_SW) && !defined(DEBUG_VIDEOPIPE)
		static uint32_t intv_stats_idx= 0,
 										intv_stats[100];
		uint32_t cur_time;
		int n, dur;
		if (startup_meas/*hold off not measure immediately*/) {
			cur_time = *DWT_CYCCNT;
			dur= timedelta(0, cur_time, last_done_spi);
			if (1200000<dur && 19200000>dur) { // cpu clk @ 120 mhz assumed
				// max possible latency due to ant sw to be 150 msec assumed
				if (intv_stats_idx<sizeof(intv_stats)/sizeof(intv_stats[0])) {
					intv_stats[intv_stats_idx] = dur;
				}
				intv_stats_idx = intv_stats_idx+ 1;
			}
		}
		last_done_spi = *DWT_CYCCNT;
		if (sizeof(intv_stats)/sizeof(intv_stats[0])==intv_stats_idx) {
			// once stats buffer is filled, we may stop measurement run
			for (n=0; n<intv_stats_idx; n++) {
				if (intv_stats[n] > intv_max)
					intv_max = intv_stats[n];
				if (intv_stats[n] < intv_min)
					intv_min = intv_stats[n];
			}
			n = -1; // place brkpt here to watch intv stats, liyenho
		}
#endif
		uint8_t spibuff_wrptr_filled0 = spibuff_wrptr_filled; // to keep old cnt after inc, liyenho
	  spibuff_wrptr_filled = spibuff_wrptr_currentlyfilling;
	  spibuff_wrptr_currentlyfilling = spibuff_wrptr_yettobefilled;
	  spibuff_wrptr_yettobefilled++;  //unsigned char, of the Currently on-going writing buffer
	  if(spibuff_wrptr_yettobefilled>=GRAND_BUFFER_BLKS)
	    spibuff_wrptr_yettobefilled = 0;				// wrong! changed spibuff_wrptr_yettobefilled to spibuff_wrptr_currentlyfilling, liyenho
	  dma_chain.ul_addr = gs_uc_rbuffer+((unsigned int)spibuff_wrptr_currentlyfilling* (I2SC_BUFFER_SIZE/sizeof(int)) );
	  dma_chain.ul_size = I2SC_BUFFER_SIZE/2;  //16 bit word count
	  pdc_rx_init(g_p_spim_pdc[1],NULL,&dma_chain);

	  //0x47 mis-alignment checker
	  cc_curr = *(gs_uc_rbuffer  // wrong! changed spibuff_wrptr_filled to spibuff_wrptr_filled0, liyenho
	  	           + (spibuff_wrptr_filled0*(I2SC_BUFFER_SIZE/sizeof(int)) )
	  		  );
	  if((cc_curr & 0x0000ff00) != 0x00004700)
	    ts47badcnt_pre++;
	  else
	    ts47badcnt_pre=0;
	 // horrible design??? shall all mis-alignment trigger reset after all??? liyenho
	  if(ts47badcnt_pre /*== 0xff*/) {
		 pio_clear(PIOB, PIO_PB9); // disable TS gate right away
	    spidma_active = 0;
		ts47badcnt_pre = 0;
		mon_ts47bad_cnt++;
	  }
															  #ifdef DEBUG_VIDEOPIPE
															  //cc error checking
															  for(i=0;i<10;i++)
															  {
															    cc_curr = *(gs_uc_rbuffer	// wrong! changed spibuff_wrptr_filled to spibuff_wrptr_filled0, liyenho
															  	           + (spibuff_wrptr_filled0*(I2SC_BUFFER_SIZE/sizeof(int)) )
															  			   + (i*188/4)
															  		  );
															  if((cc_curr & 0xff00001f) == 0x00000001) //video pid (0x100)
															  {
																cc_next = cc_prev+0x00010000;
																if((cc_curr&0x000f0000) != (cc_next&0x000f0000)){
															      dbg_ccerr++;
																  if(dbg_spidmacnt < 100)
																    dbg_ccerr=0;
																}
															    cc_prev = cc_curr;
															  }
															 }
															 #endif
	}
  }

  // USB transfer -----------------------------------------------------------------------
  if(spidma_active)
  {
	if(spibuff_wrptr_filled != spibuff_rdptr)
	{
		udi_cdc_lvl = udi_cdc_multi_get_free_tx_buffer(0);
		if(udi_cdc_lvl > I2SC_BUFFER_SIZE)
		{
#ifdef VIDEO_DUAL_BUFFER
			uint32_t *pb, ofst=0, tcnt, cc1, cc2;
			pb=gs_uc_rbuffer+((unsigned int)spibuff_rdptr*(I2SC_BUFFER_SIZE/4));
			if (-1 == stream) {
				cc = *(pb+0/4) & 0xff00001f ;
				cc1 = *(pb+188/4) & 0xff00001f ;
				cc2 = *(pb+376/4) & 0xff00001f ;
				// only process packets on video pid...
				if (PID_VID ==cc && PID_VID ==cc1 && PID_VID ==cc2) {
					cc = *(pb+0/4) & 0x000f0000 ;
					cc1 = *(pb+188/4) & 0x000f0000 ;
					cc2 = *(pb+376/4) & 0x000f0000 ;
					uint32_t tmp2, tmp1, tmp = 0x000f0000&(cc+0x00010000);
					if (tmp == cc2) {
						tmp1 = 0x000f0000&(cc-0x000a0000);
						tmp2 = 0x000f0000&(cc+0x000b0000);
						if (tmp1 == cc1 || tmp2 == cc1) {
							// tmp1 == cc1, next stream is to output
							// tmp2 == cc1, this stream is to output
							stream = (tmp1 == cc1)? 188/4 : 0/4;
							uint32_t *pb1 = pb+stream;
							memcpy(new_video_buffer, pb1, 188);
							cc = *(pb1) & 0x000f0000 ;
							pbn = new_video_buffer+188;
							ofst = 188*2/4;
							st_pos = 1;
							goto found;
						}
					}
				}
				goto next;
			}
found: {
  			uint32_t *pbl1=pblr+stream, // ptr to ts lookup section
  								*pbl0=pblw+(stream+ofst),
  								pid ;
			bool taken = false;
			memcpy(pblw, pb, I2SC_BUFFER_SIZE);
			// fill up new_video_buffer & process thru
			tcnt = I2SC_BUFFER_SIZE-(ofst<<2);
			do {
				cc1 = *(pbl0) & 0x000f0000 ;
				pid = *(pbl0) & 0xff00001f;
				cc2 = 0x000f0000&(cc+0x00010000);
				if ((PID_VID ==pid) && (cc1 != cc2)) { // perhaps can try xxx/(188), to seek thru each packet
					for (i=0; i<TSLUT_BUFFER_SIZE/ /*(188*2)*/188; i++) {
						uint32_t cc11 = *(pbl1) & 0x000f0000,
											pid1 = *(pbl1) & 0xff00001f;
						if ((PID_VID ==pid1) && (cc11 == cc2)) {
							memcpy(pbn, pbl1, 188);
							pbn += 188;
							if (I2SC_BUFFER_SIZE/188 == ++st_pos) {
								usb_write_buf1(new_video_buffer, I2SC_BUFFER_SIZE);
								pbn = new_video_buffer;
								st_pos = 0;
							}
							cc = cc2; // update for next
							taken = true;
							break;
						}
						pbl1 += 188/**2*/ /4;
					}
				}
				if (!taken || TSLUT_BUFFER_SIZE/ /*(188*2)*/188 == i) {
					memcpy(pbn, pbl0, 188);
					pbn += 188;
					if (I2SC_BUFFER_SIZE/188 == ++st_pos) {
						usb_write_buf1(new_video_buffer, I2SC_BUFFER_SIZE);
						pbn = new_video_buffer;
						st_pos = 0;
					}
					cc = cc1; // update for next
					pbl0 += 188*2/4;
					tcnt = tcnt-188*2;
				}
				pbl1=pblr+stream ;  // reset lkup ptr
				taken = false;
			} while (0 != tcnt);
		}
			// update lookup buffer
  			memmove(lkup_video_buffer,
  									lkup_video_buffer+I2SC_BUFFER_SIZE,
  									TSLUT_BUFFER_SIZE);
next:
#else
		  usb_write_buf1(
		  		gs_uc_rbuffer+((unsigned int)spibuff_rdptr*(I2SC_BUFFER_SIZE/4)),
		  		I2SC_BUFFER_SIZE );  //burst out a usb transfer
#ifdef CTRL_RADIO_ENCAP
		uint32_t *pbt, pid, mde, usr, bsz;
		  pbt=gs_uc_rbuffer+((unsigned int)spibuff_rdptr*(I2SC_BUFFER_SIZE/4));
		uint8_t *pbi= ((uint8_t*)pbt)+/*sizeof(ts_rdo_hdr)*/7;
		for (i=0; i<I2SC_BUFFER_SIZE/188; i++) {
			pid = *(pbt+0) & 0xff00001f;
			mde = *(pbt+0) & 0x00ff0000;
			usr = *(pbt+1) & 0x000000ff;
			if (PID_CTL ==pid && MDE_CTL ==mde && USR_CTL ==usr) {
				// radio control packet found
				bsz = (*(pbt+1) & 0xff000000)>>24;
				if (1>bsz) goto usr_next;  // invalid packet or user's joke
				uint16_t word, *pw= (uint16_t*)(pbi+1);
				*pbr++ = *(pbi-1); // odd position
				for (ii=0; ii<(bsz-1)/2; ii++) {
					word = *pw++;
					*pbr++ = 0xff & (word>>8);
					*pbr++ = 0xff & word;
				}
				if (!(1&bsz))
					*pbr++ = 0xff & (*pw>>8);
				radio_pkt_filled(bsz);
				if ((pb_rdo_ctrl_e-188)<pbr)
					pbr =pb_rdo_ctrl ;
			}
usr_next:
			pbt += 188/4;
			pbi += 188;
		}
#endif
#endif
		spibuff_rdptr++;
		if(spibuff_rdptr>= GRAND_BUFFER_BLKS)
			spibuff_rdptr = 0;
		dbg_usbtransfercnt++;
		}
		else
		dbg_usbtransferfail++;

		//if(dbg_usbtransfercnt>10000)
			//dbg_usbtransfercnt=0;  //break point tap
	}
#ifdef RECV_TAISYNC
		if (reset_ts_count_error)
			last_half_sec = *DWT_CYCCNT;
		else if (last_half_sec) {
			uint32_t cur_time;
			 cur_time = *DWT_CYCCNT;
			int dur;
			 dur= timedelta(0, cur_time, last_half_sec);
			 // 120 mhz core clock assumed, tried to align
			 // rtt fire period into closest time on half sec,
			 // it will be 480 ms if it is time exact...
			 if (/*(115200000/2)*/(5*115200000) < dur) { // let it be 5 sec instead half sec, to gain stability
				 if ((5*TP_ERR_RATE)< ts_count_error) {
					 // try to switch antenna
#if false  // user button triggered for now...
					gl_vid_ant_sw = true;
#endif
				 }
			 	 last_half_sec = cur_time;
				 ts_count_error = 0;
			 }
		}
#endif
  }
	                                                           #ifdef DEBUG_VIDEOPIPE
																				  //monitoring and debuging
																				  spibuff_rdptr_l = spibuff_rdptr;  //prevent race conditions
																				  if (REG_SPI5_SR & SPI_SR_OVRES)
																					  dbg_spidmaov++;
																				  dbg_spififo_lvl = (spibuff_wrptr_filled>= spibuff_rdptr)?(spibuff_wrptr_filled-spibuff_rdptr):
																											   ((spibuff_wrptr_filled+GRAND_BUFFER_BLKS)-spibuff_rdptr);
																				  if(dbg_spififo_lvl> dbg_spififo_lvl_max)
																					  dbg_spififo_lvl_max = dbg_spififo_lvl;

																				  if(dbg_spidmacnt >= 10000)
																						dbg_spidmacnt = 0;     //breakpoint tag points

																				  if(dbg_spidmacnt < 100) {    //clear out startu transient instability logs
																					  dbg_spififo_lvl_max = 0;
																					  cpumax = 0;
																					  cpumin=0xffffffff;
																					  dbg_usbtransferfail=0;
																					  dbg_spidmaov = 0;
																				  }
																				  #endif
  return;
 }//if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC)
	/* Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
          //Error condition, should not happen
		return;
	}
}

/**********************************************************************************************************/
#ifndef RECV_TAISYNC
 #define TWI_Handler     FLEXCOM4_Handler
 #define TWI_IRQn        FLEXCOM4_IRQn
 volatile static int sms_poll_count = 0, // i2c message length
									sms_poll_done = 0;
 volatile int sms_poll_error = 0;

void TWI_Handler(void)  // sms4470 i2c polling handler
{
	volatile static int index= sizeof(sms_access);
		uint32_t status = BOARD_BASE_TWI_SMS4470->TWI_SR;
		if (status & TWI_SR_NACK) {
			sms_poll_done = TWI_RECEIVE_NACK;
			return ; // i2c read failed
		}
		/* Last byte ? */
		if (sms_poll_count-index == 1 ) {
			BOARD_BASE_TWI_SMS4470->TWI_CR = TWI_CR_STOP;
		}
		// fetch the byte read
		*((uint8_t*)fw_sms_rbuffer+index++) =
			BOARD_BASE_TWI_SMS4470->TWI_RHR;
		if (	sms_poll_count== index) {
			twi_disable_interrupt(BOARD_BASE_TWI_SMS4470, TWI_IDR_RXRDY|TWI_IDR_NACK);
			index = sizeof(sms_access); // reset buffer index
			sms_poll_done = (int)true;
		}
}
void ts_fail_handler(const uint32_t id, const uint32_t index)
{
	if ((id == ID_PIOA) && (index == TS_FAIL_INT)){
		// bump up ts fail cnt each time we get interrupt
		if (reset_ts_count_error) {
			ts_count_error = 1;
		} else
		ts_count_error = ts_count_error+ 1;
	}
}
#endif
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

	rtt_init(RTT, clkcnt); // fired every sec (0 gives about 2sec)

	ul_previous_time = rtt_read_timer_value(RTT);
	while (ul_previous_time == rtt_read_timer_value(RTT));

	rtt_write_alarm_time(RTT, clkcnt ); // fired every sec (0 gives about 2sec)
#ifndef RECV_TAISYNC
	pio_handler_set(PIOA, ID_PIOA, TS_FAIL_INT,
		PIO_IT_AIME | PIO_IT_RE_OR_HL| PIO_IT_EDGE, ts_fail_handler); // add ts_fail counter
	pio_enable_interrupt(PIOA, TS_FAIL_INT);
	pio_handler_set_priority(PIOA, PIOA_IRQn, 1/*long latency event*/);
#endif
	/* Enable RTT interrupt */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 0);  // RTT take highest priority as Siano spi/usb xfer handler
	NVIC_EnableIRQ(RTT_IRQn);
	rtt_enable_interrupt(RTT, RTT_MR_RTTINCIEN);
}

#endif // SMS_DVBT2_DOWNLOAD

extern void spi_rx_transfer(void *p_tbuf,
		uint32_t tsize, void *p_rbuf, uint32_t rsize, uint32_t ch);

void set_radio_frequency(uint32_t frq) { // in mhz
	VAR_DECL_SPI
	dev_spi_wr(0x01, 1, frq) // rx freq
}

void set_radio_bandwidth(VCH_BW bw) { // enum
	VAR_DECL_SPI
	dev_spi_wr(0x03, 1, (uint32_t)bw)
}

	void set_radio_power(uint32_t pwr) { // in dbm
		if (MAX_RDO_PWR < pwr)
			return; // can't support yet
		VAR_DECL_SPI
		dev_spi_wr(0x02, 1, pwr)
	}

	void set_radio_modulation(VCH_MD mod) { // enum
		VAR_DECL_SPI
		dev_spi_wr(0x04, 1, (uint32_t)mod)
	}

	void get_radio_uplk_buf(uint16_t* phw) {
		VAR_DECL_SPI
		dev_spi_rd(0x0d, 1, phw, 2)
	}

	void get_radio_uplk_frms(uint32_t *pw) {
		VAR_DECL_SPI
		dev_spi_rd(0x72, 1, pw, 4)
	}

	void get_radio_uplk_lost_frms(uint32_t *pw) {
		VAR_DECL_SPI
		dev_spi_rd(0x73, 1, pw, 4)
	}

	void get_radio_snr(uint8_t* pb) {
		VAR_DECL_SPI
		dev_spi_rd(0x07, 1, pb, 1)
	}

	void get_rdo_ldpc_failed(uint32_t *pw) {
		VAR_DECL_SPI
		dev_spi_rd(0x08, 1, pw, 4)
	}

	void get_radio_rssi(uint8_t *pb) {
		VAR_DECL_SPI
		dev_spi_rd(0x09, 1, pb, 1)
	}

	void get_radio_rx_vga(uint8_t *pb) {
		VAR_DECL_SPI
		dev_spi_rd(0x0a, 1, pb, 1)
	}

	void get_radio_bb_sts(uint8_t *pb) {
		VAR_DECL_SPI
		dev_spi_rd(0x0b, 1, pb, 1)
	}

	void get_radio_dnlk_buf(uint8_t *pb) {
		VAR_DECL_SPI
		dev_spi_rd(0x0c, 1, pb, 1)
	}

	void get_radio_dnlk_frms(uint32_t *pw) {
		VAR_DECL_SPI
		dev_spi_rd(0x70, 1, pw, 4)
	}

	void get_radio_dnlk_lost_frms(uint32_t *pw) {
		VAR_DECL_SPI
		dev_spi_rd(0x71, 1, pw, 4)
	}

void enable_radio_antenna(bool en) { // 0: disable, 1: enable
	VAR_DECL_SPI
	dev_spi_wr(0x05, 1, (uint32_t)en)
}

void get_radio_cmd_sts(uint8_t* pb) {
	VAR_DECL_SPI
	dev_spi_rd(0x60, 1, pb, 1)
}

int  init_video_subsystem()
{
	uint32_t error=Error_NO_ERROR;
	set_radio_frequency(/*809000*/720);
	set_radio_bandwidth(TEN_MHZ);
	//set_radio_modulation(QPSK_1B2); // do I need this on rx side? liyenho
	return error;
_exit:
	while (1) {;} // sticky error exception
}

int  start_video_subsystem()
{
	uint32_t error=Error_NO_ERROR;
	enable_radio_antenna(true);
	return error;
_exit:
	while (1) {;} // sticky error exception

}
