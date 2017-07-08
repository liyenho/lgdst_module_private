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
#include "sms.h"
#include "DigiBestFrontend.h"
//#include "si4463/bsp.h"
//#include "si4463/lgdst_4463_spi.h"

//#define DEBUG_VIDEOPIPE
#ifdef _TST_RDO_CTL_ENCAP_
	uint8_t pb_rdo_ctrl[I2SC_BUFFER_SIZE],
					*pb_rdo_ctrl_e= pb_rdo_ctrl+sizeof(pb_rdo_ctrl);
	static uint8_t *pbr= pb_rdo_ctrl;
	extern void radio_pkt_filled(int bsz);
#endif
#if defined(SMS_DVBT2_DOWNLOAD) || defined(RECV_SMS4470)
 extern uint32_t gs_sms_tbuffer[(USB_SMS_MSG_LEN+SMS_BUFFER_SIZE+3)/sizeof(int)];
 extern uint32_t fw_sms_tbuffer[FW_DNLD_SIZE/sizeof(int)], fw_sms_rbuffer[FW_DNLD_SIZE/sizeof(int)];
 extern uint32_t g_ul_wait_10ms;
 extern twi_packet_t packet_tx, packet_rx;
 extern Sms4470_State sms_state;
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
 //extern tRadioConfiguration *pRadioConfiguration;  //YH: cause failure

 SMS4470_DEVICE_GROUP_OPERATION_INFORMATION dvbt;  // dvbt/2 ctrl ctx, liyenho

volatile bool sms_dvbt_lock= false;
 extern volatile RECEPTION_STATISTICS_ST recptStatG;
 extern volatile RECEPTION_STATISTICS_ST recptStatG_m;
 extern volatile RECEPTION_STATISTICS_ST recptStatG_s;
 extern volatile Short_Statistics_ST shortStatG;
 extern unsigned int log2a[100];
 extern unsigned char log2p;

unsigned int dbg_usbtransfercnt=0;
unsigned int dbg_usbtransferfail=0;
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
	int i, ii;
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
#ifdef _TST_RDO_CTL_ENCAP_
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
			pbt += 188/4;
			pbi += 188;
		}
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

//**********************************************************************************************************
/*! \brief host read sms device version response
 */
 /*static*/ bool read_sms_response(
   MsgTypes_ET res,   //expected mesage type
   Sms4470_State ste
   )
 {
	int loop = 0, limit = (MSG_SW_SWITCH_STANDARD_RES==res)?1000:200;
    sms_access *phdr= (sms_access*)fw_sms_rbuffer;
	do {
	  //Evaluate packet length ----------------------------------------------------
		while (!i2c_read_done) ;
		 // re-use sms dnld rx buffer, should be ok, liyenho
		int len = phdr->msgLength;
		if (len > sizeof(*phdr) && MSG_SMS_I2C_SHORT_STAT_IND!=phdr->msgType) {
			if (MSG_SMS_DVBT2_TRANSMISSION_IND<phdr->msgType
					|| sizeof(fw_sms_rbuffer) < len) {
				/*puts("-E-\tSMS response is too much.\r");*/
				/*while(1)*/ {
					return false; /* Capture error */
				}
		  }
	  // Read rest of the package ----------------------------------------------------
			len -=  sizeof(*phdr);
		  // re-use video rx buffer, should be ok, liyenho
		  packet_rx.buffer  = (uint8_t *)fw_sms_rbuffer+sizeof(*phdr);
		  packet_rx.length = len;
		  uint32_t err; // error code requried
			//YH: delay for siano to prepare next packet
#ifdef RX_SPI_CHAINING
			delay_ms(10);
		  TWI_READ
#else // !RX_SPI_CHAINING
			if (!i2c_read_cb_on)
				delay_ms(10);
			else {
			 int itr = 10/0.6;
			 do {
				delay_us(600);  // invoke callback every 0.6 msec
				 usb_write_buf_cb();
			 	} while (--itr>0);
			 }
			 if (!i2c_read_cb_on) {
			  	TWI_READ
		  	}
			 else {
			 	TWI_READ_CB
		 	}
#endif
		}
      //Unexpected message type processing -------------------------------------------
		if (res != phdr->msgType) {
			//did not get what was expected (which is res)
			if (phdr->msgType == MSG_SMS_HO_PER_SLICES_IND) {
				SMS4470_HandlePerSlicesIndication_lh( \
					(uint8_t *)fw_sms_rbuffer+sizeof(*phdr), phdr->msgLength-sizeof(*phdr));
			}
			else if (phdr->msgType == MSG_SMS_TRANSMISSION_IND) {
				SMS4470_TransStatsIndication_lh((uint8_t*)fw_sms_rbuffer+sizeof(*phdr));
			}
			else if (phdr->msgType == MSG_SMS_NO_SIGNAL_IND) {
				SMS4470_NoSignalDetected();
			}
			/*else if (phdr->msgType == MSG_SMS_I2C_SHORT_STAT_IND) {//testing
				memcpy(&dvbt.DVBTSignalStatistic,
					(uint8_t *)fw_sms_rbuffer+ 2*sizeof(*phdr),
					sizeof(Short_Statistics_ST));
			}*/
			if (MSG_SMS_DVBT2_TRANSMISSION_IND/*very last msg by far*/<res)
			{ return true; /*purpose to wait for ind events*/ }
			if (STATE_GET_STATISTICS_EX_RES!=ste /*DVBT2*/ &&
					STATE_GET_I2C_STATUS_IND != ste /*DVBT*/) {
				/*if (MSG_SMS_RF_TUNE_RES==res || MSG_SMS_ADD_PID_FILTER_RES==res) {
					_WAIT_(100);} // try again after 1 sec
				else*/ {
					_WAIT_(1);} // try again after 10 msec
			}
			else { // invoked from polling (rtt) isr, don't trigger another (systick) isr
#ifdef RX_SPI_CHAINING
				delay_ms(10);
#else // !RX_SPI_CHAINING
				if (!i2c_read_cb_on)
					delay_ms(10);
				else {
				 int itr = 10/0.6;
				 do {
					delay_us(600);  // invoke callback every 0.6 msec
					 usb_write_buf_cb();
				 } while (--itr>0);
				}
#endif
				_TWI_READ_
			}
			continue;
		}  //end res != phdr->msgType
      // Successful Termination Wraput ---------------------------------------------
		// must restore the default values, liyenho
		packet_rx.buffer      = (uint8_t *) fw_sms_rbuffer;
		packet_rx.length      = /*sizeof(fw_sms_rbuffer)*/FW_DNLD_SIZE;
		break;
	} while(limit>++loop);
	if (limit > loop) {
		sms_state = ste;
		return true;
	}
	else if (MSG_SW_RELOAD_START_RES != res){
	  //puts("-E-\tSMS packet response unexpected.\r");
		while(1){ /* got bad response. */ }
	}
 }

 //**********************************************************************************************************
void SMS4470_get_version_lh() {
	// query sms4470 device info
	sms_access *phdr= (sms_access*)gs_sms_tbuffer;
	SMS_INIT_MSG(phdr, MSG_SMS_GET_VERSION_EX_REQ,
		     sizeof(sms_access));
	TWI_WRITE(gs_sms_tbuffer,sizeof(sms_access))
	sms_state = STATE_REQ_VERSION_EX;
	_WAIT_(5); // scheduled 50 msec
	// read sms4470 device info
	read_sms_response(MSG_SMS_GET_VERSION_EX_RES, STATE_VERSION_EX_RES);
}

void fordigibest_twi_wr(unsigned char* buffer, int length)
{
	uint32_t *pb = (sizeof(gs_sms_tbuffer)<length)?fw_sms_tbuffer :gs_sms_tbuffer;
	memcpy((void*)pb , buffer, length);
	TWI_WRITE(pb, length)
}
void fordigibest_twi_rd(unsigned char* buffer, int length)
{
	uint32_t err; // error code requried
	packet_rx.buffer  = (uint8_t *)fw_sms_rbuffer;
	packet_rx.length = length;

	err = twi_master_read(BOARD_BASE_TWI_SMS4470, &packet_rx);
	if ( err != TWI_SUCCESS ) {
		/*if(err != TWI_ERROR_TIMEOUT || sizeof(sms_access) != packet_rx.length)*/
			/*puts("-E-\tTWI master read packet failed.\r");*/
			while (1) {
				; /* Capture error */
			}

	}
	memcpy(buffer,fw_sms_rbuffer,length);
}

void fordigibest_reception_statistics(void* pRecptStat_src)
{
	RECEPTION_STATISTICS_ST * pRecptStatp = (RECEPTION_STATISTICS_ST *)pRecptStat_src;
	if(pRecptStatp->ModemState == 0x19)
	memcpy(&recptStatG_s, (void*)pRecptStat_src, sizeof(RECEPTION_STATISTICS_ST));
	else
	memcpy(&recptStatG_m, (void*)pRecptStat_src, sizeof(RECEPTION_STATISTICS_ST));

	log2a[log2p] = pRecptStatp->InBandPwr;
	log2p = ((log2p+1)%100);
	log2a[log2p] = pRecptStatp->ModemState;
	log2p = ((log2p+1)%100);
}

void fordigibest_short_statistics(void* pShortStat_src)
{
	memcpy(&shortStatG, pShortStat_src, sizeof(Short_Statistics_ST));
}

 //**********************************************************************************************************
void SMS4470_send_device_init_lh(DEMODULATOR_TUNING_FEATURE feature)
{
	// initialize sms4470 device
	SmsMsgData_ST *msg = (SmsMsgData_ST*)gs_sms_tbuffer;
	SMS_INIT_MSG(&msg->xMsgHeader, MSG_SMS_INIT_DEVICE_REQ,
		     sizeof(SmsMsgData_ST));
	if (DVBT_DEMODULATOR_TUNING == feature)
		msg->msgData[0]           = SMSHOSTLIB_DEVMD_DVBT;
	else if (DVBT2_DEMODULATOR_TUNING == feature)
	   msg->msgData[0]           = SMSHOSTLIB_DEVMD_DVBT2;
	TWI_WRITE(gs_sms_tbuffer,sizeof(SmsMsgData_ST))
	sms_state = STATE_DEV_INIT_RES;
	_WAIT_(1); // scheduled 10 msec
	// read init device response
	read_sms_response(MSG_SMS_INIT_DEVICE_RES, STATE_DEV_INIT_RES);
}

//**********************************************************************************************************
void SMS4470_set_polling_mode_lh() {
	// setup polling mode ops
	SmsMsgData3Args_ST *msg1 =(SmsMsgData3Args_ST*)gs_sms_tbuffer;
	sms_state = STATE_REQ_POLL_MODE;
	SMS_INIT_MSG(&msg1->xMsgHeader, MSG_SMS_SPI_INT_LINE_SET_REQ,
		     sizeof(SmsMsgData3Args_ST));
	msg1->msgData[0] = I2C_SEC_CTR; // Controller
	msg1->msgData[1] = SMS_GPIO_NONE; // GpioNum
	msg1->msgData[2] = SPI_PULSE_WIDTH; // PulseWidth
	TWI_WRITE(gs_sms_tbuffer,sizeof(SmsMsgData3Args_ST))
	_WAIT_(1); // scheduled 10 msec
	read_sms_response(MSG_SMS_SPI_INT_LINE_SET_RES, STATE_POLL_MODE_RES);
}

//**********************************************************************************************************
void SMS4470_enable_ts_interface_lh() {
	// enable TS interface
	SmsTsEnable_ST *msg2 =(SmsTsEnable_ST*)gs_sms_tbuffer;
	sms_state = STATE_REQ_ENABLE_TS_INTF;
	SMS_INIT_MSG(&msg2->xMsgHeader, MSG_SMS_ENBALE_TS_INTERFACE_REQ,
		     sizeof(SmsTsEnable_ST));
	msg2->TsClock                 = 12000000/*48000000*/;   // 12000000; // 24000000; // 36000000; // 48000000; // 96000000; // 192000000;
	msg2->eTsiMode                = TSI_SERIAL_SECONDARY;    // TSI_SERIAL_MAIN; // TSI_SERIAL_SECONDARY;
	msg2->eTsiSignals             = TSI_SIGNALS_ACTIVE_HIGH; // TSI_SIGNALS_ACTIVE_LOW; // TSI_SIGNALS_ACTIVE_HIGH;
	msg2->nTsiPcktDelay           = 4 /*30*/ /*0*/;  /*because of our TS bufer size?*/     // 4; // 0; // 4;
	msg2->eTsClockPolarity        = TSI_SIG_OUT_RISE_EDGE;   // TSI_SIG_OUT_FALL_EDGE; // TSI_SIG_OUT_RISE_EDGE;
	msg2->TsBitOrder              = TSI_BIT0_IS_MSB;
	msg2->EnableControlOverTs     = 0;                       // 0; // 1;
	msg2->TsiEncapsulationFormat  = TSI_TRANSPARENT;         // TSI_ENCAPSULATED; // TSI_TRANSPARENT;
	msg2->TsiPaddingPackets       = 22/*0*/;                      // 0; //21;//22;
	msg2->eTsiElectrical          = TSI_ELEC_NORMAL;         // TSI_ELEC_LOW; // TSI_ELEC_NORMAL; // TSI_ELEC_HIGH;
	msg2->IoVoltage               = IOC_VOLTAGE_3_3;         // IOC_VOLTAGE_0; // IOC_VOLTAGE_0; // IOC_VOLTAGE_1_8; // IOC_VOLTAGE_3_3;
	msg2->eTsiErrActive           = TSI_ERR_ACTIVE;
	msg2->eTsiClockKeepGo         = TSI_CLK_KEEP_GO_NO_PKT;
	TWI_WRITE(gs_sms_tbuffer,sizeof(SmsTsEnable_ST))
	_WAIT_(1); // scheduled 10 msec
	read_sms_response(MSG_SMS_ENBALE_TS_INTERFACE_RES, STATE_ENABLE_TS_INTF_RES);
}

//**********************************************************************************************************
void SMS4470_RemovePidFilter_lh(uint32_t pid)
{
	// remove pid filter
	SmsMsgData_ST *msg = (SmsMsgData_ST*)gs_sms_tbuffer;
	SMS_INIT_MSG(&msg->xMsgHeader, MSG_SMS_REMOVE_PID_FILTER_REQ,
		     sizeof(SmsMsgData_ST));
	msg->msgData[0]           = pid;
	TWI_WRITE(gs_sms_tbuffer,sizeof(SmsMsgData_ST))
	sms_state = STATE_REQ_REMOVE_PID_FILTER;
	_WAIT_(1); // scheduled 10 msec
	// read pid filter remove response
	read_sms_response(MSG_SMS_REMOVE_PID_FILTER_RES, STATE_REMOVE_PID_FILTER_RES);
}

//**********************************************************************************************************
void SMS4470_tune_lh(DEMODULATOR_TUNING_FEATURE feature,
													uint32_t bandwidth, uint32_t frequency)
{
	// setup tuning params
	SmsMsgData4Args_ST *msg = (SmsMsgData4Args_ST*)gs_sms_tbuffer;
	SMS_INIT_MSG(&msg->xMsgHeader, MSG_SMS_RF_TUNE_REQ,
		     sizeof(SmsMsgData3Args_ST));
	msg->msgData[0]           = frequency;
	if (DVBT2_DEMODULATOR_TUNING == feature)
		switch(bandwidth) {
			case 8: msg->msgData[1] = SIANO_BANDWIDTH_T2_8_MHZ;
								break;
			case 7: msg->msgData[1] = SIANO_BANDWIDTH_T2_7_MHZ;
								break;
			case 6: msg->msgData[1] = SIANO_BANDWIDTH_T2_6_MHZ;
								break;
			case 5: msg->msgData[1] = SIANO_BANDWIDTH_T2_5_MHZ;
								break;
			default : msg->msgData[1] = SIANO_BANDWIDTH_T2_8_MHZ;
								break;
		}
	else if (DVBT_DEMODULATOR_TUNING == feature)
		switch(bandwidth) {
			case 8: msg->msgData[1] = SIANO_BANDWIDTH_8M;
								break;
			case 7: msg->msgData[1] = SIANO_BANDWIDTH_7M;
								break;
			case 6: msg->msgData[1] = SIANO_BANDWIDTH_6M;
								break;
			case 5: msg->msgData[1] = SIANO_BANDWIDTH_5M;
								break;
			default : msg->msgData[1] = SIANO_BANDWIDTH_8M;
								break;
		}
    msg->msgData[2]           = 12000000;
    //msg->msgData[3]        = 1; // not used
	TWI_WRITE(gs_sms_tbuffer,sizeof(SmsMsgData3Args_ST))
	sms_state = STATE_REQ_SMS_RF_TUNE;
	_WAIT_(/*1*/100); // scheduled 1 sec
	// read tuning setup response
	read_sms_response(MSG_SMS_RF_TUNE_RES, STATE_SMS_RF_TUNE_RES);
}

//**********************************************************************************************************
void SMS4470_AddPidFilter_lh(uint32_t pid)
{
	// add pid filter
	SmsMsgData_ST *msg = (SmsMsgData_ST*)gs_sms_tbuffer;
	SMS_INIT_MSG(&msg->xMsgHeader, MSG_SMS_ADD_PID_FILTER_REQ,
		     sizeof(SmsMsgData_ST));
	msg->msgData[0]           = pid;
	TWI_WRITE(gs_sms_tbuffer,sizeof(SmsMsgData_ST))
	sms_state = STATE_REQ_ADD_PID_FILTER;
	_WAIT_(/*1*/100); // scheduled 1 sec
	// read pid filter add response
	read_sms_response(MSG_SMS_ADD_PID_FILTER_RES, STATE_ADD_PID_FILTER_RES);
}

//**********************************************************************************************************
void SMS4470_OpenPlp_lh(uint32_t plpId)
{
	// open plp pipe
	SmsMsgData_ST *msg = (SmsMsgData_ST*)gs_sms_tbuffer;
	SMS_INIT_MSG(&msg->xMsgHeader, MSG_SMS_DVBT2_OPEN_PLP_REQ,
		     sizeof(SmsMsgData_ST));
	msg->msgData[0] = plpId;
	TWI_WRITE(gs_sms_tbuffer,sizeof(SmsMsgData_ST))
	sms_state = STATE_REQ_DVBT2_OPEN_PLP;
	_WAIT_(1); // scheduled 10 msec
	// read open plp pipe response
	read_sms_response(MSG_SMS_DVBT2_OPEN_PLP_RES, STATE_DVBT2_OPEN_PLP_RES);
}

//**********************************************************************************************************
bool SMS4470_check_signal_lh(bool * pLockStatus)
{
	SmsMsgData_ST *msg = (SmsMsgData_ST*)gs_sms_tbuffer;
	if (pLockStatus)
   	*pLockStatus = false;
	else return false;
	if (DVBT2_WORKING_MODE == dvbt.Sms4470CurrentWorkingMode) {
		SMS_INIT_MSG(&msg->xMsgHeader, MSG_SMS_GET_STATISTICS_EX_REQ,
		     sizeof(SmsMsgData_ST));
		msg->msgData[0] = 0 /*?not used?*/;
		TWI_WRITE(gs_sms_tbuffer,sizeof(SmsMsgData_ST)); //YH: digibest sizeof(SmsMsgData_ST). same as liyen code
		sms_state = STATE_REQ_GET_STATISTICS_EX;
		_WAIT_(1); // scheduled 10 msec
		// read get status ex response
		if (!read_sms_response(MSG_SMS_GET_STATISTICS_EX_RES, STATE_GET_STATISTICS_EX_RES))
			return false;  // didn't get response
		msg = (SmsMsgData_ST*)(uint8_t*)fw_sms_rbuffer;
       if(msg->msgData[0] == 0)
	   {
			SMSHOSTLIB_STATISTICS_DVBT2_ST* pStat;
          pStat = (SMSHOSTLIB_STATISTICS_DVBT2_ST*)&msg->msgData[1];
          if(pStat->ReceptionData.IsModemLocked == 0x01)
		  {
			 *pLockStatus = true;
				dvbt.SMS4470DetectNoSignalFlag = false;
		  }
	   }
	   else
	   {	return false;
          /*//SMS4470DebugPrintf("(SMS4470_check_signal) msg->msgData[0] != 0\n");
			while (1) {
				; // Capture error
			}*/
	   }
	}
	else if (DVBT_WORKING_MODE == dvbt.Sms4470CurrentWorkingMode) {
		// read i2c short status ind
		_TWI_READ_	// Read 32 bytes
		if (!read_sms_response(MSG_SMS_I2C_SHORT_STAT_IND, STATE_GET_I2C_STATUS_IND))
			return false;  // didn't get response
       msg = (SmsMsgData_ST*)(uint8_t*)fw_sms_rbuffer+ sizeof(sms_access);
       Short_Statistics_ST* pStat = (Short_Statistics_ST*)&msg->msgData[0];
       // signal status bookkeeping, added by liyenho
       memcpy(&dvbt.DVBTSignalStatistic, pStat, sizeof(Short_Statistics_ST));
       if(pStat->IsDemodLocked == 0x01)
	   {
		  if(dvbt.DVBTReceptionStatistic.IsDemodLocked)
		  {
			 if(dvbt.DVBTTransmissionStatistic.IsDemodLocked)
			 {
		        *pLockStatus = true;
					dvbt.SMS4470DetectNoSignalFlag = false;
			 }
		  }
	   }
	}
   return true;
}
//**********************************************************************************************************
void SMS4470_check_signal_dvbt_sliced(bool * pLockStatus)
	{
		static bool sms_poll_first_time = true;
		sms_access *phdr= (sms_access*)fw_sms_rbuffer;
		if (!sms_poll_first_time) {
			if (!sms_poll_done) {
				sms_poll_error = TWI_ERROR_TIMEOUT;
				return;
			}
			else if (true != sms_poll_done) {
				sms_poll_error = sms_poll_done;
				sms_poll_done = (int)false;
				return; // place brk point to exam
			}
			while (!(BOARD_BASE_TWI_SMS4470->TWI_SR & TWI_SR_TXCOMP)) {
			}	BOARD_BASE_TWI_SMS4470->TWI_SR; // clear i2c status reg
			// parse message previously polled
			if (true) {
				if (phdr->msgType == MSG_SMS_HO_PER_SLICES_IND) {
					SMS4470_HandlePerSlicesIndication_lh( \
						(uint8_t *)fw_sms_rbuffer+sizeof(*phdr), phdr->msgLength-sizeof(*phdr));
				}
				else if (phdr->msgType == MSG_SMS_TRANSMISSION_IND) {
					SMS4470_TransStatsIndication_lh((uint8_t*)fw_sms_rbuffer+sizeof(*phdr));
				}
				else if (phdr->msgType == MSG_SMS_NO_SIGNAL_IND) {
					SMS4470_NoSignalDetected();
				}
				else if (phdr->msgType == MSG_SMS_I2C_SHORT_STAT_IND) {
					memcpy(&dvbt.DVBTSignalStatistic,
										(uint8_t *)fw_sms_rbuffer+ 2*sizeof(*phdr),
										sizeof(Short_Statistics_ST));
					SmsMsgData_ST *msg = (SmsMsgData_ST*)(uint8_t*)fw_sms_rbuffer+ sizeof(sms_access);
       			Short_Statistics_ST* pStat = (Short_Statistics_ST*)&msg->msgData[0];
			       if(pStat->IsDemodLocked == 0x01)
				   {
					  if(dvbt.DVBTReceptionStatistic.IsDemodLocked)
					  {
						 if(dvbt.DVBTTransmissionStatistic.IsDemodLocked)
						 {
					        *pLockStatus = true;
								dvbt.SMS4470DetectNoSignalFlag = false;
						 }
					  }
				   }
				}
			}
		}
		else {
			/* Enable TWI interrupt */
			NVIC_DisableIRQ(TWI_IRQn);
			NVIC_ClearPendingIRQ(TWI_IRQn);
			NVIC_SetPriority(TWI_IRQn, 0);
			NVIC_EnableIRQ(TWI_IRQn);
			sms_poll_first_time = false;
		}
		_TWI_READ_	// trigger twi read action, took 320 us to complete @ 200 khz bus clk
		sms_poll_count = phdr->msgLength /*- sizeof(*phdr)*/; // hdr len included
		if (sms_poll_count< sizeof(*phdr))
			return; // too short
			if (MSG_SMS_DVBT2_TRANSMISSION_IND<phdr->msgType
					|| /*sizeof(fw_sms_rbuffer)*/FW_DNLD_SIZE < sms_poll_count) {
				/*puts("-E-\tSMS response is too much.\r");*/
				/*while (1)*/ {
					return ; /* Capture error */
				}
			}
		if (8>=sms_poll_count || MSG_SMS_I2C_SHORT_STAT_IND==phdr->msgType) {
			sms_poll_done = (int)true;
			return ;
		}
		/* Set read mode, slave address and 3 internal address byte lengths */
		BOARD_BASE_TWI_SMS4470->TWI_MMR = 0;
		BOARD_BASE_TWI_SMS4470->TWI_MMR = TWI_MMR_MREAD | TWI_MMR_DADR(packet_rx.chip) |
				((packet_rx.addr_length << TWI_MMR_IADRSZ_Pos) &
				TWI_MMR_IADRSZ_Msk);

		/* Set internal address for remote chip */
		BOARD_BASE_TWI_SMS4470->TWI_IADR = 0;
		BOARD_BASE_TWI_SMS4470->TWI_IADR = twi_mk_addr(packet_rx.addr, packet_rx.addr_length);

		twi_enable_interrupt(BOARD_BASE_TWI_SMS4470, TWI_IER_RXRDY|TWI_IER_NACK);
		sms_poll_done = (int)false;  // reset i2c fetch flag

		/* Send a START condition */
		if (sms_poll_count- sizeof(*phdr) == 1) {
			BOARD_BASE_TWI_SMS4470->TWI_CR = TWI_CR_START | TWI_CR_STOP;
		} else {
			BOARD_BASE_TWI_SMS4470->TWI_CR = TWI_CR_START;
		}
	}

//**********************************************************************************************************
#define CORRECT_STAT_RSSI(_stat) (_stat).RSSI *= -1
#define CORRECT_STAT_MRC_RSSI(_stat) (_stat).MRC_RSSI *= -1

void SMS4470_HandlePerSlicesIndication_lh(uint32_t* pMsgData, uint32_t len)
{
	RECEPTION_STATISTICS_ST* pReceptionStatistic= &dvbt.DVBTReceptionStatistic;
	uint32_t RecQualSum = 0;
	uint32_t Snr;
	uint32_t InBandPower;
	uint32_t TsPackets;
	uint32_t EtsPackets;
	uint32_t Constellation;
	uint32_t HpCode;

	Snr             = pMsgData[1];
	InBandPower     = (int32_t)pMsgData[2];
	TsPackets       = pMsgData[3];
	EtsPackets      = pMsgData[4];
	Constellation   = pMsgData[5];
	HpCode          = pMsgData[6];

	pReceptionStatistic->IsRfLocked			= pMsgData[16];
	if (pReceptionStatistic->IsRfLocked)
		dvbt.SMS4470DetectNoSignalFlag = false;
	pReceptionStatistic->IsDemodLocked		= pMsgData[17];
	pReceptionStatistic->ModemState			= pMsgData[12];
	pReceptionStatistic->SNR			    = pMsgData[1];
	pReceptionStatistic->BER				= pMsgData[13];
	pReceptionStatistic->RSSI				= pMsgData[14];
	CORRECT_STAT_RSSI(*pReceptionStatistic);

	pReceptionStatistic->InBandPwr			= (int32_t)pMsgData[2];
	pReceptionStatistic->CarrierOffset		= (int32_t)pMsgData[15];
	pReceptionStatistic->TotalTSPackets		= pMsgData[3];
	pReceptionStatistic->ErrorTSPackets		= pMsgData[4];

	//TSPER
	if ((TsPackets + EtsPackets) > 0)
	{
		pReceptionStatistic->TS_PER = (EtsPackets * 100) / (TsPackets + EtsPackets);
	}
	else
	{
		pReceptionStatistic->TS_PER = 0;
	}

	pReceptionStatistic->BERBitCount			= pMsgData[18];
	pReceptionStatistic->BERErrorCount			= pMsgData[19];

	pReceptionStatistic->MRC_SNR				= pMsgData[20];
	pReceptionStatistic->MRC_InBandPwr			= pMsgData[21];
	pReceptionStatistic->MRC_RSSI				= pMsgData[22];
	CORRECT_STAT_MRC_RSSI(*pReceptionStatistic);

	pReceptionStatistic->RefDevPPM				= pMsgData[23];
	pReceptionStatistic->FreqDevHz				= pMsgData[24];
}

//**********************************************************************************************************
void SMS4470_TransStatsIndication_lh(uint8_t *pData)
{
		/*packet_rx.buffer      = (void*)&dvbt.DVBTTransmissionStatistic;
		packet_rx.length = sizeof(dvbt.DVBTTransmissionStatistic);
		TWI_READ*/
	 memcpy(&dvbt.DVBTTransmissionStatistic,pData,sizeof(TRANSMISSION_STATISTICS_ST));
		// definitely remember to restore default values
		/*packet_rx.buffer      = (uint8_t *) gs_sms_rbuffer;
		packet_rx.length      = sizeof(sms_access);*/
	if (dvbt.DVBTTransmissionStatistic.IsDemodLocked)
		dvbt.SMS4470DetectNoSignalFlag = false;
}

//**********************************************************************************************************
void SMS4470_NoSignalDetected()
{
	dvbt.SMS4470DetectNoSignalFlag = true;
}

//**********************************************************************************************************
bool SMS4470_CheckDVBT2PhysicalLayerPipeInformation_lh(bool* pDoesPhysicalLayerPipeInformationExistFlag)
{
		SmsMsgData_ST *msg = (SmsMsgData_ST*)gs_sms_tbuffer;
		SMS_INIT_MSG(&msg->xMsgHeader, MSG_SMS_GET_STATISTICS_EX_REQ,
		     sizeof(SmsMsgData_ST));
		msg->msgData[0] = 0 /*?not used?*/;
		TWI_WRITE(gs_sms_tbuffer,sizeof(SmsMsgData_ST))
		sms_state = STATE_REQ_GET_STATISTICS_EX;
		_WAIT_(1); // scheduled 10 msec
		// read get status ex response
		if (!read_sms_response(MSG_SMS_GET_STATISTICS_EX_RES, STATE_GET_STATISTICS_EX_RES))
			return false;  // didn't get response
		msg = (SmsMsgData_ST*)fw_sms_rbuffer;
       if(msg->msgData[0] == 0)
	   {
			SMSHOSTLIB_STATISTICS_DVBT2_ST* pStat;
          pStat = (SMSHOSTLIB_STATISTICS_DVBT2_ST*)&msg->msgData[1];
	      if(pStat->ReceptionData.IsModemLocked == 0x01)
		  {
				uint32_t i;
				bool CheckFlag;
	         if(pStat->ReceptionData.numOfPlps != 0)
			 {
			    CheckFlag = true;
                for(i=0;i<pStat->ReceptionData.numOfPlps;i++)
				{
			        if(pStat->PlpData[i].plpStatistics.plpId == -1)
					{
                       CheckFlag = false;
				       break;
					}
				}
                if(CheckFlag)
				{
				   *pDoesPhysicalLayerPipeInformationExistFlag = true;

                   if(dvbt.SMS4470DVBT2LastPlpIdSetup != 0xFF)
				   {
                      if(pStat->activePlps[0].plpId != 0xFFFFFFFF)
					  {
					     if(pStat->activePlps[0].plpId != dvbt.SMS4470DVBT2LastPlpIdSetup)
						 {
                            //SMS4470InfoPrintf("(SMS4470_CheckDVBT2PhysicalLayerPipeInformation) Working around for unmatch PLP ID\n");
	                        SMS4470_OpenPlp_lh(dvbt.SMS4470DVBT2LastPlpIdSetup);
						 }
					  }
				   }
				}
			 }
		  } return true;
	   }
	   return false;
	   /*else
	   {
          //SMS4470DebugPrintf("(SMS4470_CheckDVBT2PhysicalLayerPipeInformation) msg->msgData[0] != 0\n");
			while (1) {
				; //Capture error
			}
	   }*/
}

//*******************************************************************************************************
void SMS4470ReadResponseWithDelay_lh(uint32_t Milliseconds)
{
	uint32_t           Index;
    uint32_t          i;
	uint32_t          ResponseResult;

    //SMS4470FunctionNamePrintf("(SMS4470ReadResponseWithDelay)\n");

	if(Milliseconds < 10)
	   Milliseconds = 10;

    for(i=0 ; i<(Milliseconds/10) ; i++) {
		_WAIT_(1); // scheduled 10 msec
		read_sms_response(MSG_LAST_MSG_TYPE - 2, STATE_WAIT_FOR_IND);
	}
}

//**********************************************************************************************************
bool Sms4470CoreAPI_CheckLockStatusForChannelSearch_lh(uint8_t MaxCheckCounter,bool* pLockStatus)
{
	uint32_t Loop;
	uint32_t CheckPlpInfoLoop;
	bool LockStatus = false;
	bool DVBT2PhysicalLayerPipeInformationExistFlag = false;
	bool Result = false;

    //SMS4470FunctionNamePrintf("(Sms4470CoreAPI_CheckLockStatusForChannelSearch)\n");

	   if(pLockStatus)
	   {
		  *pLockStatus = false;

	             while(1)
			     {
					   Result = true;

					   if(dvbt.Sms4470CurrentWorkingMode == DVBT2_WORKING_MODE)
					   {
		                  dvbt.SMS4470DetectNoSignalFlag = false;

                          if(MaxCheckCounter > 1)
					      {
                             for(Loop=0;Loop<(MaxCheckCounter - 1);Loop++)
						     {
							     SMS4470_check_signal_lh(&LockStatus);
                                    if(LockStatus)
									{
					                   for(CheckPlpInfoLoop=0;CheckPlpInfoLoop<6;CheckPlpInfoLoop++)
									   {
					                       if(dvbt.SMS4470DetectNoSignalFlag)
					                          break;

					                       SMS4470ReadResponseWithDelay_lh(500);

					                       if(dvbt.SMS4470DetectNoSignalFlag)
					                          break;

				                           if(SMS4470_CheckDVBT2PhysicalLayerPipeInformation_lh(&DVBT2PhysicalLayerPipeInformationExistFlag) == true)
										   {
											  if(DVBT2PhysicalLayerPipeInformationExistFlag)
											  {
												 *pLockStatus = true;

							                     break;
											  }
										   }
										   else
										   {
                                              Result = false;

											  break;
										   }
									   }
				                       break;
									}

					                if(dvbt.SMS4470DetectNoSignalFlag)
					                   break;

					                SMS4470ReadResponseWithDelay_lh(500);

					                if(dvbt.SMS4470DetectNoSignalFlag)
					                   break;
								 {
									Result = false;

									break;
								 }
						     }
					      }
			              if(DVBT2PhysicalLayerPipeInformationExistFlag == false && dvbt.SMS4470DetectNoSignalFlag == false)
					      {
							 if(SMS4470_check_signal_lh(&LockStatus) == true)
							 {
                                if(LockStatus)
								{
					               for(CheckPlpInfoLoop=0;CheckPlpInfoLoop<6;CheckPlpInfoLoop++)
								   {
					                   if(dvbt.SMS4470DetectNoSignalFlag)
					                      break;

					                   SMS4470ReadResponseWithDelay_lh(500);

					                   if(dvbt.SMS4470DetectNoSignalFlag)
					                      break;

				                       if(SMS4470_CheckDVBT2PhysicalLayerPipeInformation_lh(&DVBT2PhysicalLayerPipeInformationExistFlag) == true)
									   {
										  if(DVBT2PhysicalLayerPipeInformationExistFlag)
										  {
											 *pLockStatus = true;

							                 break;
										  }
									   }
									   else
									   {
                                          Result = false;

										  break;
									   }
								   }
								}
							 }
							 else
								Result = false;
					      }
					   }
			           else
					   {
		                  dvbt.SMS4470DetectNoSignalFlag = false;

                          if(MaxCheckCounter > 1)
					      {
                             for(Loop=0;Loop<(MaxCheckCounter - 1);Loop++)
						     {
							     if(SMS4470_check_signal_lh(&LockStatus) == true)
								 {
                                    if(LockStatus)
									{
									   *pLockStatus = true;

							           break;
									}
								 }
								 else
								 {
                                    Result = false;

									break;
								 }

					             if(dvbt.SMS4470DetectNoSignalFlag)
					                break;

					             SMS4470ReadResponseWithDelay_lh(500);

					             if(dvbt.SMS4470DetectNoSignalFlag)
					                break;
							 }
					      }
			              if(LockStatus == false && dvbt.SMS4470DetectNoSignalFlag == false)
					      {
	                         Result = SMS4470_check_signal_lh(&LockStatus);
							 if(Result)
							 {
                                if(LockStatus)
								   *pLockStatus = true;
							 }
					      }
			           }
			           break;
			     }
	   }

    return Result;
}

//**********************************************************************************************************
bool SMS4470_GetReceptionStatistics_lh(PSTATISTICS_INFORMATION pStatistics)
{
    uint32_t          i;
	uint32_t          ResponseResult;
    uint32_t          MaxPlpData;
    uint32_t          MaxActivePlp;
	SmsMsgData_ST   SmsMsg = {{0}};
    SmsMsgData_ST*  pRespondseMsg;
	bool            PlpValidCheckFlag;
	bool            ActivePlpMatchCheckFlag;
	uint16_t          TempMsgLength;

    //SMS4470FunctionNamePrintf("(SMS4470_GetReceptionStatistics)\n");

    pStatistics->IsSignalLocked = false;
	pStatistics->StatisticAvailableFlag = false;

	if(dvbt.Sms4470CurrentWorkingMode == DVBT2_WORKING_MODE)
	{
	   // for DVB-T2
		SmsMsgData_ST *msg = (SmsMsgData_ST*)gs_sms_tbuffer;
		SMS_INIT_MSG(&msg->xMsgHeader, MSG_SMS_GET_STATISTICS_EX_REQ,
		     	sizeof(SmsMsgData_ST));
		msg->msgData[0]           = 0;
		TWI_WRITE(gs_sms_tbuffer,sizeof(SmsMsgData_ST))

		_WAIT_(1); // scheduled 10 msec
		// read sms response
		if(!read_sms_response(MSG_SMS_GET_STATISTICS_EX_RES, STATE_GET_STATISTICS_EX_RES))
			return false;  // didn't get response
       SMSHOSTLIB_STATISTICS_DVBT2_ST* pStat;

		msg = (SmsMsgData_ST*)fw_sms_rbuffer;
       if(msg->msgData[0] == 0)
	   {
          pStat = (SMSHOSTLIB_STATISTICS_DVBT2_ST*)&msg->msgData[1];

	      if(pStat->ReceptionData.IsModemLocked == 0x01)
		  {
             pStatistics->IsSignalLocked = true;
	         *(sms_status_regs+MODEM_LOCKED) = pStat->ReceptionData.IsModemLocked;
	         *(sms_status_regs+FREQUENCY) = pStat->ReceptionData.txStatistics.Frequency;
	         *(sms_status_regs+BANDWIDTH) = pStat->ReceptionData.txStatistics.Bandwidth;
	         *(sms_status_regs+CARRIEER_OFFSET) = pStat->ReceptionData.carrierOffset;
	         *(sms_status_regs+INBAND_POWER) = pStat->ReceptionData.inbandPower;
	         *(sms_status_regs+EXT_LNA) = pStat->ReceptionData.extLna;
	         *(sms_status_regs+TOTAL_FRAMES) = pStat->ReceptionData.totalFrames;
	         *(sms_status_regs+SNR) = pStat->ReceptionData.SNR;
	         *(sms_status_regs+RSSI) = pStat->ReceptionData.RSSI;
	         *(sms_status_regs+FER) = pStat->ReceptionData.FER;
	         *(sms_status_regs+CELL_ID) = pStat->ReceptionData.CellId;
	         *(sms_status_regs+NET_ID) = pStat->ReceptionData.netId;
	         *(sms_status_regs+RECEPTION_QUALITY) = pStat->ReceptionData.receptionQuality;
	         *(sms_status_regs+BWT_EXT) = pStat->ReceptionData.bwt_ext;
	         *(sms_status_regs+FFT_MODE) = pStat->ReceptionData.fftMode;
	         *(sms_status_regs+GUARD_INTERVAL) = pStat->ReceptionData.guardInterval;
	         *(sms_status_regs+PILOT_PATTERN) = pStat->ReceptionData.pilotPattern;
	         *(sms_status_regs+BIT_RATE) = pStat->ReceptionData.bitRate;
	         *(sms_status_regs+EXTENDED) = pStat->ReceptionData.extended;
	         *(sms_status_regs+TONE_RESERVATION) = pStat->ReceptionData.toneReservation;
	         *(sms_status_regs+L1_POST_SIZE) = pStat->ReceptionData.l1PostSize;
	         *(sms_status_regs+NUM_OF_AUXS) = pStat->ReceptionData.numOfAuxs;
	         *(sms_status_regs+NUM_OF_PLPS) = pStat->ReceptionData.numOfPlps;
	         *(sms_status_regs+LITE_MODE) = pStat->ReceptionData.liteMode;
	         *(sms_status_regs+MRC_SNR) = pStat->ReceptionData.MRC_SNR;
	         *(sms_status_regs+SNR_FULL_RES) = pStat->ReceptionData.SNRFullRes;
	         *(sms_status_regs+MRC_INBAND_PWR) = pStat->ReceptionData.MRC_InBandPwr;
				*(sms_status_regs+MRC_RSSI) = pStat->ReceptionData.MRC_Rssi;
				*(sms_status_regs+CMN_PLP_NOT_SUPPORTED) = pStat->ReceptionData.commonPlpNotSupported;
				*(sms_status_regs+L1_MODULATION) = pStat->ReceptionData.l1modulation;
				*(sms_status_regs+NUM_DATA_SYMB) = pStat->ReceptionData.numdatasymbols;

	         if(pStat->ReceptionData.numOfPlps != 0)
			 {
			    PlpValidCheckFlag = true;
                for(i=0;i<pStat->ReceptionData.numOfPlps;i++)
				{
			        if(pStat->PlpData[i].plpStatistics.plpId == -1)
					{
                       PlpValidCheckFlag = false;
				       break;
					}
				}
                if(PlpValidCheckFlag)
				{
					uint32_t *regs_plp = sms_status_regs+NUM_BASE_REGS;
	               //SMS4470StatisticPrintf("********************************* PLP ID ***************************************\n");
                   if(pStat->ReceptionData.numOfPlps > DVBT2_MAX_PLPS_LITE)
			          MaxPlpData = DVBT2_MAX_PLPS_LITE;
			       else
				      MaxPlpData = pStat->ReceptionData.numOfPlps;
                   for(i=0;i<MaxPlpData;i++)
				   {
                       *(regs_plp + i*NUM_PLP_REGS +PLP_ID) = pStat->PlpData[i].plpStatistics.plpId;
                       *(regs_plp + i*NUM_PLP_REGS + PLP_TYPE) = pStat->PlpData[i].plpStatistics.plpType;
                       *(regs_plp + i*NUM_PLP_REGS + PLP_PAYLOAD_TYPE) = pStat->PlpData[i].plpStatistics.plpPayloadType;
                       *(regs_plp + i*NUM_PLP_REGS + FF_FLAG) = pStat->PlpData[i].plpStatistics.ffFlag;
                       *(regs_plp + i*NUM_PLP_REGS + FIRST_RF_IDX) = pStat->PlpData[i].plpStatistics.firstRfIdx;
                       *(regs_plp + i*NUM_PLP_REGS + FIRST_FRAME_IDX) = pStat->PlpData[i].plpStatistics.firstFrameIdx;
                       *(regs_plp + i*NUM_PLP_REGS + PLP_GROUP_ID) = pStat->PlpData[i].plpStatistics.plpGroupId;
                       *(regs_plp + i*NUM_PLP_REGS + PLP_COD) = pStat->PlpData[i].plpStatistics.plpCod;
                       *(regs_plp + i*NUM_PLP_REGS + PLP_MOD) = pStat->PlpData[i].plpStatistics.plpMod;
                       *(regs_plp + i*NUM_PLP_REGS + PLP_ROTATION) = pStat->PlpData[i].plpStatistics.plpRotation;
                       *(regs_plp + i*NUM_PLP_REGS + PLP_FEC_TYPE) = pStat->PlpData[i].plpStatistics.plpFecType;
                       *(regs_plp + i*NUM_PLP_REGS + PLP_NUM_BLK_MAX) = pStat->PlpData[i].plpStatistics.plpNumBlocksMax;
                       *(regs_plp + i*NUM_PLP_REGS + FRAME_INTERVAL) = pStat->PlpData[i].plpStatistics.frameInterval;
                       *(regs_plp + i*NUM_PLP_REGS + TIME_IL_LENGTH)= pStat->PlpData[i].plpStatistics.timeIlLength;
					   	  *(regs_plp + i*NUM_PLP_REGS + TIME_IL_TYPE) = pStat->PlpData[i].plpStatistics.timeIlType;
					   	  *(regs_plp + i*NUM_PLP_REGS + INBAND_A_FLAG) = pStat->PlpData[i].plpStatistics.inbandA_Flag;
					   	  *(regs_plp + i*NUM_PLP_REGS + INBAND_B_FLAG) = pStat->PlpData[i].plpStatistics.inbandB_Flag;
					   	  *(regs_plp + i*NUM_PLP_REGS + PLP_MODE) = pStat->PlpData[i].plpStatistics.plpMode;
					   	  *(regs_plp + i*NUM_PLP_REGS + STATIC_FLAG) = pStat->PlpData[i].plpStatistics.staticFlag;
					   	  *(regs_plp + i*NUM_PLP_REGS + STATIC_PADDING_FLAG) = pStat->PlpData[i].plpStatistics.staticPaddingFlag;
				   }

	               //SMS4470StatisticPrintf("************************* Active PLP Information *******************************\n");
                   if(pStat->ReceptionData.numOfPlps > DVBT2_ACTIVE_PLPS_LITE)
			          MaxActivePlp = DVBT2_ACTIVE_PLPS_LITE;
			       else
				      MaxActivePlp = pStat->ReceptionData.numOfPlps;
				   ActivePlpMatchCheckFlag = true;
					uint32_t *regs_active_plp = sms_status_regs+NUM_BASE_REGS + DVBT2_MAX_PLPS_LITE* NUM_PLP_REGS;
			       for(i=0;i<MaxActivePlp;i++)
				   {
                       if(i == 0)
					   {
                          if(dvbt.SMS4470DVBT2LastPlpIdSetup != 0xFF)
						  {
                             if(pStat->activePlps[0].plpId != 0xFFFFFFFF)
							 {
					            if(pStat->activePlps[0].plpId != dvbt.SMS4470DVBT2LastPlpIdSetup)
								{

                                   //SMS4470InfoPrintf("(SMS4470_GetReceptionStatistics) Working around for unmatch PLP ID\n");

	                               SMS4470_OpenPlp_lh(dvbt.SMS4470DVBT2LastPlpIdSetup);

								   ActivePlpMatchCheckFlag = false;
								}
							 }
						  }
					   }

                       *(regs_active_plp + i*NUM_ACTIVE_PLP_REGS +ACT_PLP_ID) = pStat->activePlps[i].plpId;
                       *(regs_active_plp + i*NUM_ACTIVE_PLP_REGS + ACT_PLP_TYPE) = pStat->activePlps[i].plpType;
                       *(regs_active_plp + i*NUM_ACTIVE_PLP_REGS + PLP_EFF_MODE) = pStat->activePlps[i].plpEfficiencyMode;
                       *(regs_active_plp + i*NUM_ACTIVE_PLP_REGS + DNP) = pStat->activePlps[i].dnp;
                       *(regs_active_plp + i*NUM_ACTIVE_PLP_REGS + ISSYI) = pStat->activePlps[i].issyi;
                       *(regs_active_plp + i*NUM_ACTIVE_PLP_REGS + CRC_ERRORS) = pStat->activePlps[i].crcErrors;
                       *(regs_active_plp + i*NUM_ACTIVE_PLP_REGS + NUM_LDPC_ITERS) = pStat->activePlps[i].numOfLdpcIters;
                       *(regs_active_plp + i*NUM_ACTIVE_PLP_REGS + TOT_NUM_BB_FRAMS_RECV) = pStat->activePlps[i].totalNumBBFramesReceived;
                       *(regs_active_plp + i*NUM_ACTIVE_PLP_REGS + TOT_NUM_TS_PKT_RECV) = pStat->activePlps[i].totalNumTsPktsReceived;
                       *(regs_active_plp + i*NUM_ACTIVE_PLP_REGS + TOT_NUM_ERR_TS_PKT_RECV) = pStat->activePlps[i].totalNumErrTsPktsReceived;
                       *(regs_active_plp + i*NUM_ACTIVE_PLP_REGS + NUM_OF_OVERFLOW) = pStat->activePlps[i].numOfOverflow;
                       *(regs_active_plp + i*NUM_ACTIVE_PLP_REGS + NUM_OF_UNDERFLOW) = pStat->activePlps[i].numOfUnderflow;
                       *(regs_active_plp + i*NUM_ACTIVE_PLP_REGS + DEJITTER_BUFFER_SIZE) = pStat->activePlps[i].dejitterBufferSize;
                       *(regs_active_plp + i*NUM_ACTIVE_PLP_REGS + TOT_NUM_PKT_INSERTED)= pStat->activePlps[i].totalNumOfPktsInserted;
					   	  *(regs_active_plp + i*NUM_ACTIVE_PLP_REGS + TOT_NUM_TS_PKT_FORWD) = pStat->activePlps[i].totalNumTsPktsForwarded;
					   	  *(regs_active_plp + i*NUM_ACTIVE_PLP_REGS + TOT_POST_LDPC_ERR) = pStat->activePlps[i].totalPostLdpcErr;
					   	  *(regs_active_plp + i*NUM_ACTIVE_PLP_REGS + NUM_TS_PKT_RECV_AFTER_RESET) = pStat->activePlps[i].numTsPktsReceivedAfterReset;
					   	  *(regs_active_plp + i*NUM_ACTIVE_PLP_REGS + NUM_ERR_TS_PKT_RECV_AFTER_RESET) = pStat->activePlps[i].numErrTsPktsReceivedAfterReset;
			 	   }
                   if(ActivePlpMatchCheckFlag)
				   {
                      pStatistics->StatisticAvailableFlag = true;
                      pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.SNR = pStat->ReceptionData.MRC_SNR;
                      pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.InBandPower = pStat->ReceptionData.MRC_InBandPwr;
                      pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.ErrorTSPackets = pStat->activePlps[0].totalNumErrTsPktsReceived;
                      pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.TotalTSPackets = pStat->activePlps[0].totalNumTsPktsReceived;
                      pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.CellID = pStat->ReceptionData.CellId;
                      pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.NetworkID = pStat->ReceptionData.netId;
					  if(pStat->ReceptionData.liteMode)
					     pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.T2LiteActive = true;
					  else
						 pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.T2LiteActive = false;
					  // fix-me, the fftMode value maybe need to double check !
                      if((pStat->ReceptionData.fftMode & 0x0E) == 0)
                         pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.TransmissionMode = FFT_MODE_2K;
					  else
                      if((pStat->ReceptionData.fftMode & 0x0E) == 2)
                         pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.TransmissionMode = FFT_MODE_8K;
					  else
                      if((pStat->ReceptionData.fftMode & 0x0E) == 4)
                         pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.TransmissionMode = FFT_MODE_4K;
					  else
                      if((pStat->ReceptionData.fftMode & 0x0E) == 6)
					  {
						 if(pStat->ReceptionData.liteMode)
                            pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.TransmissionMode = FFT_MODE_16K;
						 else
                            pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.TransmissionMode = FFT_MODE_1K;
					  }
					  else
                      if((pStat->ReceptionData.fftMode & 0x0E) == 8)
                         pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.TransmissionMode = FFT_MODE_16K;
					  else
                      if((pStat->ReceptionData.fftMode & 0x0E) == 10)
					  {
						 if(pStat->ReceptionData.liteMode)
						    pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.TransmissionMode = FFT_MODE_RESERVED;
						 else
                            pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.TransmissionMode = FFT_MODE_32K;
					  }
					  else
                      if((pStat->ReceptionData.fftMode & 0x0E) == 12)
                         pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.TransmissionMode = FFT_MODE_8K;
					  else
                      if((pStat->ReceptionData.fftMode & 0x0E) == 14)
					  {
						 if(pStat->ReceptionData.liteMode)
						    pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.TransmissionMode = FFT_MODE_RESERVED;
						 else
                            pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.TransmissionMode = FFT_MODE_32K;
					  }
					  else
					     pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.TransmissionMode = UNKNOWN_FFT_MODE;
					  if(pStat->ReceptionData.guardInterval == 0)
						 pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.GuardInterval = GUARD_INTERVAL_1_32;
					  else
					  if(pStat->ReceptionData.guardInterval == 1)
						 pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.GuardInterval = GUARD_INTERVAL_1_16;
					  else
					  if(pStat->ReceptionData.guardInterval == 2)
						 pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.GuardInterval = GUARD_INTERVAL_1_8;
					  else
					  if(pStat->ReceptionData.guardInterval == 3)
						 pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.GuardInterval = GUARD_INTERVAL_1_4;
					  else
					  if(pStat->ReceptionData.guardInterval == 4)
						 pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.GuardInterval = GUARD_INTERVAL_1_128;
					  else
					  if(pStat->ReceptionData.guardInterval == 5)
						 pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.GuardInterval = GUARD_INTERVAL_19_128;
					  else
					  if(pStat->ReceptionData.guardInterval == 6)
						 pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.GuardInterval = GUARD_INTERVAL_19_256;
					  else
						 pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.GuardInterval = UNKNOWN_GUARD_INTERVAL;
                      if(pStat->ReceptionData.pilotPattern == 1)
                         pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PilotPattern = PILOT_PATTERN_1;
					  else
                      if(pStat->ReceptionData.pilotPattern == 2)
                         pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PilotPattern = PILOT_PATTERN_2;
					  else
                      if(pStat->ReceptionData.pilotPattern == 3)
                         pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PilotPattern = PILOT_PATTERN_3;
					  else
                      if(pStat->ReceptionData.pilotPattern == 4)
                         pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PilotPattern = PILOT_PATTERN_4;
					  else
                      if(pStat->ReceptionData.pilotPattern == 5)
                         pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PilotPattern = PILOT_PATTERN_5;
					  else
                      if(pStat->ReceptionData.pilotPattern == 6)
                         pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PilotPattern = PILOT_PATTERN_6;
					  else
                      if(pStat->ReceptionData.pilotPattern == 7)
                         pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PilotPattern = PILOT_PATTERN_7;
					  else
                      if(pStat->ReceptionData.pilotPattern == 8)
                         pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PilotPattern = PILOT_PATTERN_8;
					  else
                         pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PilotPattern = UNKNOWN_PILOT_PATTERN;
					  pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformationAvailableFlag = true;
                      pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.NumberOfPlps = MaxPlpData;
					  for(i=0;i<MaxPlpData;i++)
					  {
                          pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].PlpID = pStat->PlpData[i].plpStatistics.plpId;
                          pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].PlpGroupID = pStat->PlpData[i].plpStatistics.plpGroupId;
						  if(pStat->PlpData[i].plpStatistics.plpType == 0)
							 pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].PlpType = COMMON_PLP;
						  else
						  if(pStat->PlpData[i].plpStatistics.plpType == 1)
							 pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].PlpType = DATA_TYPE1_PLP;
						  else
						  if(pStat->PlpData[i].plpStatistics.plpType == 2)
							 pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].PlpType = DATA_TYPE2_PLP;
						  else
							 pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].PlpType = UNKNOWN_PLP_TYPE;
                          if(pStat->PlpData[i].plpStatistics.plpMode == 0)
                             pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].PlpMode = PLP_NOT_SPECIFIED_MODE;
                          else
                          if(pStat->PlpData[i].plpStatistics.plpMode == 1)
                             pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].PlpMode = PLP_NORMAL_EFFICIENCY_MODE;
                          else
                          if(pStat->PlpData[i].plpStatistics.plpMode == 2)
                             pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].PlpMode = PLP_HIGH_EFFICIENCY_MODE;
                          else
                          if(pStat->PlpData[i].plpStatistics.plpMode == 3)
                             pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].PlpMode = PLP_RESERVED_MODE;
                          else
							 pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].PlpMode = UNKNOWN_PLP_MODE;
                          if(pStat->PlpData[i].plpStatistics.plpFecType == 0)
							 pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].FecFrameType = FEC_FRAME_16K_LDPC;
						  else
                          if(pStat->PlpData[i].plpStatistics.plpFecType == 1)
							 pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].FecFrameType = FEC_FRAME_64K_LDPC;
						  else
							 pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].FecFrameType = UNKNOWN_FEC_FRAME;

						  if(pStat->PlpData[i].plpStatistics.plpCod == 0)
                             pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].CodeRate = CODE_RATE_1_2;
						  else
						  if(pStat->PlpData[i].plpStatistics.plpCod == 1)
                             pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].CodeRate = CODE_RATE_3_5;
						  else
						  if(pStat->PlpData[i].plpStatistics.plpCod == 2)
                             pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].CodeRate = CODE_RATE_2_3;
						  else
						  if(pStat->PlpData[i].plpStatistics.plpCod == 3)
                             pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].CodeRate = CODE_RATE_3_4;
						  else
						  if(pStat->PlpData[i].plpStatistics.plpCod == 4)
                             pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].CodeRate = CODE_RATE_4_5;
						  else
						  if(pStat->PlpData[i].plpStatistics.plpCod == 5)
                             pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].CodeRate = CODE_RATE_5_6;
						  else
						  if(pStat->PlpData[i].plpStatistics.plpCod == 6)
                             pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].CodeRate = CODE_RATE_1_3;
						  else
						  if(pStat->PlpData[i].plpStatistics.plpCod == 7)
                             pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].CodeRate = CODE_RATE_2_5;
						  else
                             pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].CodeRate = UNKNOWN_CODE_RATE;
                          if(pStat->PlpData[i].plpStatistics.plpMod == 0)
                             pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].Constellation = CONSTELLATION_QPSK;
						  else
                          if(pStat->PlpData[i].plpStatistics.plpMod == 1)
                             pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].Constellation = CONSTELLATION_16_QAM;
						  else
                          if(pStat->PlpData[i].plpStatistics.plpMod == 2)
                             pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].Constellation = CONSTELLATION_64_QAM;
						  else
                          if(pStat->PlpData[i].plpStatistics.plpMod == 3)
                             pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].Constellation = CONSTELLATION_256_QAM;
						  else
                             pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].Constellation = UNKNOWN_CONSTELLATION;
                          if(pStat->PlpData[i].plpStatistics.plpRotation)
							 pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].PlpRotationEnable = true;
						  else
							 pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].PlpRotationEnable = false;
                          pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].PlpNumberBlocksMax = pStat->PlpData[i].plpStatistics.plpNumBlocksMax;
                          pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].FrameInterval = pStat->PlpData[i].plpStatistics.frameInterval;
						  pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].TimeIlLength = pStat->PlpData[i].plpStatistics.timeIlLength;
						  pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].TimeIlType = pStat->PlpData[i].plpStatistics.timeIlType;
					  }
				   }

				}
			 }
		  }
	      else
		  {
             //SMS4470StatisticPrintf("********************************************************************************\n");
             //SMS4470StatisticPrintf("*************************     SMS4470 No Signal !     **************************\n");
             //SMS4470StatisticPrintf("********************************************************************************\n");
		  }
       }
	   else
	   {
          //SMS4470StatisticPrintf("(SMS4470_check_signal_lh) pRespondseMsg->msgData[0] != 0\n");
			while (1) {
				; /* Capture error */
			}
	   }
	}
	else
	if(dvbt.Sms4470CurrentWorkingMode == DVBT_WORKING_MODE)
	{
	   // for DVB-T
		_TWI_READ_	// trigger twi read action
		if (!read_sms_response(MSG_SMS_I2C_SHORT_STAT_IND, STATE_GET_I2C_STATUS_IND))
			return false;  // didn't get response
		// read i2c short status ind
       SmsMsgData_ST *msg = (SmsMsgData_ST*)((uint8_t*)fw_sms_rbuffer+sizeof(sms_access));
       Short_Statistics_ST* pStat = (Short_Statistics_ST*)&msg->msgData[0];

       if(pStat->IsDemodLocked == 0x01)
	   {
		  pStatistics->IsSignalLocked = true;

          //SMS4470StatisticPrintf("********************************************************************************\n");
          //SMS4470StatisticPrintf("***********************     SMS4470 Signal Locked !     ************************\n");
          //SMS4470StatisticPrintf("********************************************************************************\n");
          //SMS4470StatisticPrintf("************************* Statistics Data Information **************************\n");

	         *(sms_status_regs+MODEM_LOCKED_DVBT) = pStat->IsDemodLocked;
	         *(sms_status_regs+INBAND_POWER_DVBT) = pStat->InBandPwr;
	         *(sms_status_regs+BER_DVBT) = pStat->BER;
	         *(sms_status_regs+SNR_DVBT) = pStat->SNR;
            *(sms_status_regs+TOT_TS_PKT_DVBT) = pStat->TotalTSPackets;
            *(sms_status_regs+ERR_TS_PKT_DVBT) = pStat->ErrorTSPackets;

		  if(dvbt.DVBTReceptionStatistic.IsDemodLocked)
		  {	// all DVBTReceptionStatistic info items in usb host, liyenho
			  	 //dvbt.DVBTReceptionStatistic.IsRfLocked);
             //dvbt.DVBTReceptionStatistic.IsDemodLocked) ;

             //dvbt.DVBTReceptionStatistic.ModemState);
             //dvbt.DVBTReceptionStatistic.SNR);
             //dvbt.DVBTReceptionStatistic.BER);
             //dvbt.DVBTReceptionStatistic.BERErrorCount);
             //dvbt.DVBTReceptionStatistic.BERBitCount);
             //dvbt.DVBTReceptionStatistic.TS_PER);
			 //dvbt.DVBTReceptionStatistic.MFER);
             //dvbt.DVBTReceptionStatistic.RSSI);
             //dvbt.DVBTReceptionStatistic.InBandPwr);
             //dvbt.DVBTReceptionStatistic.CarrierOffset);
             //dvbt.DVBTReceptionStatistic.ErrorTSPackets);
             //dvbt.DVBTReceptionStatistic.TotalTSPackets);

             //dvbt.DVBTReceptionStatistic.RefDevPPM);
             //dvbt.DVBTReceptionStatistic.FreqDevHz);

             //dvbt.DVBTReceptionStatistic.MRC_SNR);
             //dvbt.DVBTReceptionStatistic.MRC_RSSI);
             //dvbt.DVBTReceptionStatistic.MRC_InBandPwr);

		     if(dvbt.DVBTTransmissionStatistic.IsDemodLocked)
			 {	// all DVBTReceptionStatistic info items in usb host, liyenho
                //dvbt.DVBTTransmissionStatistic.Frequency);
                //dvbt.DVBTTransmissionStatistic.Bandwidth);
                //dvbt.DVBTTransmissionStatistic.TransmissionMode);
                //dvbt.DVBTTransmissionStatistic.GuardInterval);
                //dvbt.DVBTTransmissionStatistic.CodeRate);
                //dvbt.DVBTTransmissionStatistic.LPCodeRate);
                //dvbt.DVBTTransmissionStatistic.Hierarchy);
                //dvbt.DVBTTransmissionStatistic.Constellation);
	            // DVB-H TPS parameters
                //dvbt.DVBTTransmissionStatistic.CellId);
                //dvbt.DVBTTransmissionStatistic.DvbhSrvIndHP);
                //dvbt.DVBTTransmissionStatistic.DvbhSrvIndLP);
                //dvbt.DVBTTransmissionStatistic.IsDemodLocked);

                pStatistics->StatisticAvailableFlag = true;
                pStatistics->DTV_STATISTIC.DVBTStatisticInfo.SNR = dvbt.DVBTReceptionStatistic.SNR;
                pStatistics->DTV_STATISTIC.DVBTStatisticInfo.InBandPower = dvbt.DVBTReceptionStatistic.InBandPwr;
                pStatistics->DTV_STATISTIC.DVBTStatisticInfo.ErrorTSPackets = dvbt.DVBTReceptionStatistic.ErrorTSPackets;
                pStatistics->DTV_STATISTIC.DVBTStatisticInfo.TotalTSPackets = dvbt.DVBTReceptionStatistic.TotalTSPackets;
			    if(dvbt.DVBTTransmissionStatistic.TransmissionMode == 0)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.TransmissionMode = FFT_MODE_2K;
			    else
			    if(dvbt.DVBTTransmissionStatistic.TransmissionMode == 1)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.TransmissionMode = FFT_MODE_8K;
			    else
			    if(dvbt.DVBTTransmissionStatistic.TransmissionMode == 2)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.TransmissionMode = FFT_MODE_4K;
			    else
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.TransmissionMode = UNKNOWN_FFT_MODE;
			    if(dvbt.DVBTTransmissionStatistic.GuardInterval == 3)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.GuardInterval = GUARD_INTERVAL_1_4;
			    else
			    if(dvbt.DVBTTransmissionStatistic.GuardInterval == 2)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.GuardInterval = GUARD_INTERVAL_1_8;
			    else
			    if(dvbt.DVBTTransmissionStatistic.GuardInterval == 1)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.GuardInterval = GUARD_INTERVAL_1_16;
			    else
			    if(dvbt.DVBTTransmissionStatistic.GuardInterval == 0)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.GuardInterval = GUARD_INTERVAL_1_32;
			    else
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.GuardInterval = UNKNOWN_GUARD_INTERVAL;
			    if(dvbt.DVBTTransmissionStatistic.CodeRate == 0)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.CodeRate = CODE_RATE_1_2;
			    else
			    if(dvbt.DVBTTransmissionStatistic.CodeRate == 1)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.CodeRate = CODE_RATE_2_3;
			    else
			    if(dvbt.DVBTTransmissionStatistic.CodeRate == 2)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.CodeRate = CODE_RATE_3_4;
			    else
			    if(dvbt.DVBTTransmissionStatistic.CodeRate == 3)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.CodeRate = CODE_RATE_5_6;
			    else
			    if(dvbt.DVBTTransmissionStatistic.CodeRate == 4)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.CodeRate = CODE_RATE_7_8;
			    else
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.CodeRate = UNKNOWN_CODE_RATE;
			    if(dvbt.DVBTTransmissionStatistic.LPCodeRate == 0)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.LPCodeRate = CODE_RATE_1_2;
			    else
			    if(dvbt.DVBTTransmissionStatistic.LPCodeRate == 1)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.LPCodeRate = CODE_RATE_2_3;
			    else
			    if(dvbt.DVBTTransmissionStatistic.LPCodeRate == 2)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.LPCodeRate = CODE_RATE_3_4;
			    else
			    if(dvbt.DVBTTransmissionStatistic.LPCodeRate == 3)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.LPCodeRate = CODE_RATE_5_6;
			    else
			    if(dvbt.DVBTTransmissionStatistic.LPCodeRate == 4)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.LPCodeRate = CODE_RATE_7_8;
			    else
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.LPCodeRate = UNKNOWN_CODE_RATE;
			    if(dvbt.DVBTTransmissionStatistic.Constellation == 0)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.Constellation = CONSTELLATION_QPSK;
			    else
			    if(dvbt.DVBTTransmissionStatistic.Constellation == 1)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.Constellation = CONSTELLATION_16_QAM;
			    else
			    if(dvbt.DVBTTransmissionStatistic.Constellation == 2)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.Constellation = CONSTELLATION_64_QAM;
			    else
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.Constellation = UNKNOWN_CONSTELLATION;
			    if(dvbt.DVBTTransmissionStatistic.Hierarchy == 0)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.Hierarchical = HIERARCHY_NONE;
			    else
			    if(dvbt.DVBTTransmissionStatistic.Hierarchy == 1)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.Hierarchical = HIERARCHY_ALPHA_1;
			    else
			    if(dvbt.DVBTTransmissionStatistic.Hierarchy == 2)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.Hierarchical = HIERARCHY_ALPHA_2;
			    else
			    if(dvbt.DVBTTransmissionStatistic.Hierarchy == 2)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.Hierarchical = HIERARCHY_ALPHA_4;
			    else
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.Hierarchical = UNKNOWN_HIERARCHY;
                pStatistics->DTV_STATISTIC.DVBTStatisticInfo.DVBH_FrameErrorRate = dvbt.DVBTReceptionStatistic.MFER;
                pStatistics->DTV_STATISTIC.DVBTStatisticInfo.DVBHCellID = dvbt.DVBTTransmissionStatistic.CellId;
	            if((dvbt.DVBTTransmissionStatistic.DvbhSrvIndHP & 0x02) == 0x02)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.DVBH_TimeSlicing_HP = true;
				else
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.DVBH_TimeSlicing_HP = false;
	            if((dvbt.DVBTTransmissionStatistic.DvbhSrvIndHP & 0x01) == 0x01)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.DVBH_MPE_FEC_HP = true;
				else
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.DVBH_MPE_FEC_HP = false;
	            if((dvbt.DVBTTransmissionStatistic.DvbhSrvIndLP & 0x02) == 0x02)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.DVBH_TimeSlicing_LP = true;
				else
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.DVBH_TimeSlicing_LP = false;
	            if((dvbt.DVBTTransmissionStatistic.DvbhSrvIndLP & 0x01) == 0x01)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.DVBH_MPE_FEC_LP = true;
				else
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.DVBH_MPE_FEC_LP = false;
		     }
		  }
	   }
	   else
	   {
          //SMS4470StatisticPrintf("********************************************************************************\n");
          //SMS4470StatisticPrintf("*************************     SMS4470 No Signal !     **************************\n");
          //SMS4470StatisticPrintf("********************************************************************************\n");
	   }
    }

	return true;
}
#endif // SMS_DVBT2_DOWNLOAD

