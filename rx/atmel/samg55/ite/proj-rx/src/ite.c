#include <string.h>
#include <asf.h>
#include "conf_usb.h"
#include "conf_example.h"
#include "ui.h"
#include "uart.h"
#include "delay.h"
#include "ctrl.h"
#if (SAMG55)
#include "flexcom.h"
#endif
#include "ite.h"

#include <math.h>
#include "..\video\inc\rf2072_set.h"
#include "..\video\inc\platform_it9137.h"
#include "..\video\inc\type.h"
//#define DEBUG_VIDEOPIPE
 extern volatile int8_t id_byte;
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
#if defined(SMS_DVBT2_DOWNLOAD) || defined(RECV_IT913X)
 extern uint32_t g_ul_wait_10ms;
 extern twi_packet_t packet_tx, packet_rx;
 extern bool systick_enabled;
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
#ifdef RECV_IT913X
  volatile static uint32_t ts_count_error=0;
  volatile static uint32_t last_half_sec= 0;
  //bypass transient stage during video startup
  volatile bool reset_ts_count_error=true;
  volatile bool gl_vid_ant_sw = false; // antenna switch request
#endif
extern unsigned char trig500ms;
extern uint8_t mon_ts47bad_cnt;

extern void usb_read_buf1(void *pb, int size);
static inline void usb_write_buf(void *pb)
{
	int written=0, size = I2SC_BUFFER_SIZE;
	do {
		iram_size_t b = size-udi_cdc_write_buf(pb, size);
		pb += b;
		size -= b;
		written += b;
	} while (I2SC_BUFFER_SIZE != written);
}

 uint32_t fw_ite_rbuffer[FW_DNLD_SIZE/sizeof(int)];
 static uint32_t fw_ite_tbuffer[FW_DNLD_SIZE/sizeof(int)];
 extern volatile uint32_t it913x_fw_hdr[1];
 extern volatile context_it913x ctx_913x;

extern uint32_t Cmd_addChecksum (
uint32_t*          bufferLength,
uint8_t*           buffer
) ;
 void download_ite_fw(uint32_t fw_dnld_size, uint8_t address, uint8_t *chk )
 {
	uint32_t fw_len0, fw_len = it913x_fw_hdr[0],
						len, len1, len2, ll, ll1, crc, i,
						fw_dnld_size1;
	while (0 > (int)fw_len) {
		fw_len = it913x_fw_hdr[0];
	}
	uint8_t *fw_ite_tbuffer1 = (uint8_t*)fw_ite_tbuffer+6/*per ITE algorithm*/,
					 *pb = fw_ite_rbuffer,
					 *pe = pb+fw_dnld_size,
					 Cmd_sequence = 0/*it should be alright...*/;
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
	uint32_t* tsheader;
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
			dur= timedelta(cur_time, last_done_spi);
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

 //video id filtering adjustment through id_byte
 for(i=0;i<10;i++)
 {
	 tsheader =(uint32_t *) (gs_uc_rbuffer	// wrong! changed spibuff_wrptr_filled to spibuff_wrptr_filled0, liyenho
	 + (spibuff_wrptr_filled0*(I2SC_BUFFER_SIZE/sizeof(int)) )
	 + (i*188/4)
	 );
	 if(( (*tsheader) & 0xff00001f) != 0xff00001f)
	 {
		 uint8_t* pid_lo = ((uint8_t *)tsheader);
		 pid_lo[3] = pid_lo[3]-id_byte;
	 }
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
			uint8_t tei ;
			pb=gs_uc_rbuffer+((unsigned int)spibuff_rdptr*(I2SC_BUFFER_SIZE/4));
			if (-1 == stream) {
				cc = *(pb+0/4) & 0xff00001f ;
				cc1 = *(pb+188/4) & 0xff00001f ;
				cc2 = *(pb+376/4) & 0xff00001f ;
				// only process packets on video pid...
				if (PID_VID ==cc && PID_VID ==cc1 && PID_VID ==cc2) {
					uint8_t tei1, tei2;
					tei = *(pb+0/4) & 0x00000080 ;
					tei1 = *(pb+188/4) & 0x00000080 ;
					tei2 = *(pb+376/4) & 0x00000080 ;
					if (tei || tei1 || tei2)
						goto next;  // don't pick up erratic packets
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
						tei = *(pbl1) & 0x00000080; // don't pick up erratic packet
						if ((PID_VID ==pid1) && (cc11 == cc2) && !tei) {
							memcpy(pbn, pbl1, 188);
							pbn += 188;
							if (I2SC_BUFFER_SIZE/188 == ++st_pos) {
								usb_write_buf(new_video_buffer);
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
						usb_write_buf(new_video_buffer);
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
		  usb_write_buf(gs_uc_rbuffer+((unsigned int)spibuff_rdptr*(I2SC_BUFFER_SIZE/4)) );  //burst out a usb transfer
#ifdef CTRL_RADIO_ENCAP
		uint32_t *pbt, pid, mde, usr, bsz;
		  pbt=gs_uc_rbuffer+((unsigned int)spibuff_rdptr*(I2SC_BUFFER_SIZE/4));
		uint8_t *pbi= ((uint8_t*)pbt)+/*sizeof(ts_rdo_hdr)*/7;
		for (i=0; i<I2SC_BUFFER_SIZE/188; i++) {
			if (*(pbt+0) & 0x00000080) // TS error indicator
				goto usr_next; // abandon this packet
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
#ifdef RECV_IT913X
		if (reset_ts_count_error)
			last_half_sec = *DWT_CYCCNT;
		else if (last_half_sec) {
			uint32_t cur_time;
			 cur_time = *DWT_CYCCNT;
			int dur;
			 dur= timedelta(cur_time, last_half_sec);
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
#ifndef RECV_IT913X
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
#else
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

	pio_handler_set(PIOA, ID_PIOA, TS_FAIL_INT,
		PIO_IT_AIME | PIO_IT_RE_OR_HL| PIO_IT_EDGE, ts_fail_handler); // add ts_fail counter
	pio_enable_interrupt(PIOA, TS_FAIL_INT);
	pio_handler_set_priority(PIOA, PIOA_IRQn, 1/*long latency event*/);

	/* Enable RTT interrupt */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 0);  // RTT take highest priority as Siano spi/usb xfer handler
	NVIC_EnableIRQ(RTT_IRQn);
	rtt_enable_interrupt(RTT, RTT_MR_RTTINCIEN);
}

#endif // SMS_DVBT2_DOWNLOAD

extern volatile uint8_t spi_tgt_done;
extern void spi_tx_transfer(void *p_tbuf, uint32_t tsize, void *p_rbuf,
		uint32_t rsize, uint32_t ch);
extern void spi_rx_transfer(void *p_tbuf, uint32_t tsize, void *p_rbuf,
		uint32_t rsize, uint32_t ch);

void rffe_write_regs(dev_cfg* pregs) {
	uint16_t tmp, tmpw, *pth = &tmp;
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
}

uint16_t rffe_read_regs(dev_cfg* pregs) {
	uint8_t msg[80];
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

	delay_ms(200); 	// validate echo after 0.2 sec
	//printf("xxxxxx 0x%04x @ 0x%x written xxxxxx\n",regs.data,regs.addr);

	regs.addr = 0x0F;
	regs.data = P2_FREQ1;
	rffe_write_regs(&regs);

	delay_ms(200); 	// validate echo after 0.2 sec
	//printf("xxxxxx 0x%04x @ 0x%x written xxxxxx\n",regs.data,regs.addr);

	regs.addr = 0x10;
	regs.data = P2_FREQ2;
	rffe_write_regs(&regs);

	delay_ms(200); 	// validate echo after 0.2 sec
	//printf("xxxxxx 0x%04x @ 0x%x written xxxxxx\n",regs.data,regs.addr);

	//libusb_control_transfer(devh,CTRL_OUT, USB_RQ,USB_HOST_MSG_TX_VAL,USB_HOST_MSG_IDX,(unsigned char*)acs, sizeof(*acs)+(acs->dcnt-1), 0);
	regs.addr = 0x11;
	regs.data = P2_FREQ3;
	rffe_write_regs(&regs);

	delay_ms(200); 	// validate echo after 0.2 sec
	//printf("xxxxxx 0x%04x @ 0x%x written xxxxxx\n",regs.data,regs.addr);

	return 0;
}

int init_rf2072(void)
{
	//RF2072_RESET:
		pio_clear (PIOA, PIO_PA16);
		delay_ms(200);
		pio_set (PIOA, PIO_PA16);
		delay_us(1);
		pio_clear (PIOA, PIO_PA17); // keep enbl low
	delay_ms(100);
	int chsel_2072=0;
	dev_cfg regs,*pregs=GET_ARRAY(chsel_2072);
		for (int32_t i=0; i<ARRAY_SIZE(chsel_2072); i++,pregs++)
	//RF2072_WRITE:
			rffe_write_regs(pregs);

	set_frequency_rf2072(LO_Frequency);
	delay_ms(200+500);

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
	return 0!=(0x8000&rffe_read_regs(&regs));

//	 printf("rx rffe is running...\n");
}

int  init_video_subsystem(void)
{
	Pid pid;
	pid.value=0x100;
	uint32_t error=Error_NO_ERROR;

	init_rf2072();

	error=it9137_init();
	if(error)goto _exit;

	//error=it9137_get_firmwareversion();
	//if(error)goto _exit;
//     error= it9137_scan_channel(0,747000,832000, 6000);
  //  if(error)goto _exit;
	error=it9137_acquire_channel(0,/*809000*/720000,6000);
	if(error)goto _exit;
	//error=it9137_get_if_agc(0);
//	if(error)goto _exit;
//	error=it9137_get_rf_agc_gain(0);
//	if(error)goto _exit;
#if 0
	error=it9137_control_pid_filter(0,1);
	if(error)goto _exit;
	error=it9137_add_pid_filter(0,0,pid);
	if(error)goto _exit;
	error= it9137_control_power_saving(0,1);
	if(error)goto _exit;
#endif
	return 0;
_exit:
	while (1) {;} // sticky error exception
}

int  start_video_subsystem(void)
{
	uint32_t i=0, error=Error_NO_ERROR;
	while(i<1){
	error=it9137_check_tpslocked(0);
	if(error)goto _exit;
	error=it9137_check_mpeg2locked(0);
	if(error)goto _exit;
	//do not call this api interface ,some error inside
	//error=it9137_get_channel_modulation(0);
	//if(error)goto _exit;
	error=it9137_get_signal_quality(0);
	if(error)goto _exit;
	error=it9137_get_signal_quality_indication(0);
	if(error)goto _exit;
	error=it9137_get_signal_strength(0);
	if(error)goto _exit;
	error=it9137_get_signal_strength_indication(0);
	if(error)goto _exit;
	error=it9137_get_statistic(0);
	if(error)goto _exit;
	error=it9137_get_signal_strength_dbm(0);
	if(error)goto _exit;
	error=it9137_get_postviterbi_bit_error_rate(0);
	if(error)goto _exit;
	error=it9137_get_snr(0);
	if(error)goto _exit;
	//error= it9137_reset();
	//if(error)goto _exit;
	//error=it9137_reboot();
	//if(error)goto _exit;
	i++;
	}
	return 0;
_exit:
	while (1) {;} // sticky error exception

}
