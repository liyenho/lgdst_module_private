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
#include "ite.h"

//#define DEBUG_VIDEOPIPE

#if defined(SMS_DVBT2_DOWNLOAD) || defined(RECV_IT913X)
 extern uint32_t g_ul_wait_10ms;
 extern twi_packet_t packet_tx, packet_rx;
 extern bool systick_enabled;
 extern volatile bool i2c_read_done ;
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
volatile uint32_t gl_TS_count_error=0;
extern unsigned char trig500ms;
extern uint8_t mon_ts47bad_cnt;

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

	                                                            #ifdef DEBUG_VIDEOPIPE
																					//monitoring/debugging variables
																					unsigned int cpucyclecntlocal;
																					static unsigned int rttcnt=0;
																					static unsigned int cpumin=0xffffffff, cpumax=0;
																					static unsigned int dbg_spidmaov=0;
																					static unsigned int dbg_spidmacnt=0;
																					unsigned char dbg_spififo_lvl,spibuff_rdptr_l;
																					static unsigned int cc_prev=0;
																					static unsigned int dbg_ccerr=0;
																					unsigned int cc_next;
																					int i;
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
															  if((cc_curr & 0xff000000) == 0x40000000) //video pid
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
		  usb_write_buf(gs_uc_rbuffer+((unsigned int)spibuff_rdptr*(I2SC_BUFFER_SIZE/4)) );  //burst out a usb transfer
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

	/* Enable RTT interrupt */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 0);  // RTT take highest priority as Siano spi/usb xfer handler
	NVIC_EnableIRQ(RTT_IRQn);
	rtt_enable_interrupt(RTT, RTT_MR_RTTINCIEN);
}

#endif // SMS_DVBT2_DOWNLOAD

