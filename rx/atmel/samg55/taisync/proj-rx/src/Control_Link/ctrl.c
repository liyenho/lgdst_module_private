/*
 * ctrl.c
 *
 *
 */


#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "uart.h"
#include "ctrl.h"
#include "Radio_Buffers.h"
#include "ReedSolomon.h"
#include "GPS.h"

#include "asf.h" // to avoid problems of usart_serial_options_t
#include "main.h"
#include "taisync.h"
#include "conf_uart_serial.h"

uint8_t channel_scan_selection = 0;
bool Send_with_FEC = false; // whether control link messages are sent with FEC
enum FEC_Options FEC_Option = AUTO; //controls whether host or Atmel owns Send_With_FEC

bool Requested_FEC_On = false;

bool send_long_packet = true;

unsigned int radio_mon_txidlecnt = 0;
bool control_channge_change_ack = false;
// ctrl range stats mon obj
volatile ctrl_radio_stats  ctrl_sts;

const char hopping_patterns[CHTBL_SIZE] = {
	0, 5, 1, 6, 2, 7, 3, 8, 4, 9,
	10, 15, 11, 16, 12, 17, 13, 18, 14, 19,
	20, 25, 21, 26, 22, 27, 23, 28, 24, 29,
	30, 35, 31, 36, 32, 37, 33, 38, 34, 39,
	40, 45, 41, 46, 42, 47, 43, 48, 44, 49,
} ;

/*******************************************************************/
//const unsigned int initPeriod = 3000; // startup period in msec, m.a. was in fast tracking pace
#define initCnt		30
const unsigned int initWgt = 8,
									initScl = 3,
									Ewis = 3,
									Ewim = 8-3;
//const unsigned int intePeriod = 30000; // intermediate period in msec, m.a. was in normal tracking pace
#define inteCnt		200
const unsigned int inteWgt = 16,
									inteScl = 4,
									Ewms = 5,
									Ewmm = 16-5;
const unsigned int normWgt = 16,
									normScl = 4,
									Ewns = 3,
									Ewnm = 16-3;
const unsigned int lgThr = 3, shThr= 1;
const uint32_t error_weight_cks6b[] __attribute__((aligned(8)))= {
	0, 1, 1, 2, 1, 2, 2, 3,
	1, 2, 2, 3, 2, 3, 3, 4,
	1, 2, 2, 3, 2, 3, 3, 4,
	2, 3, 3, 4, 3, 4, 4, 5,
	1, 2, 2, 3, 2, 3, 3, 4,
	2, 3, 3, 4, 3, 4, 4, 5,
	2, 3, 3, 4, 3, 4, 4, 5,
	3, 4, 4, 5, 4, 5, 5, 6,
} ;
#define CTRL_MON_PERIOD	300  // in msec
#define CTRL_FAIL_PERIOD	(5* CTRL_MON_PERIOD) // can't be too short in order to prevent spi comm lockup
#define CTRL0_MSK			(-1+(1<<CTRL_BITS))
#define CTRL0_IMSK			(0xff & ~CTRL0_MSK)
#define CTRL_MSK				(CTRL0_MSK<<CTRL_BITS)
#define CHKSM_MSK		(0xff ^ CTRL0_MSK)

#define CTRL_RX_LOOP_LATENCY		20 // msec, to be measured??? liyenho
// ctrl range stats mon obj
volatile ctrl_radio_stats  ctrl_sts;
/*******************************************************************/

/**
 *  \brief Configure the Console UART.
 */
static void configure_console(void)
{
	Configure_UART_DMA(0);
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
#ifdef CONF_UART_CHAR_LENGTH
		.charlength = CONF_UART_CHAR_LENGTH,
#endif
		.paritytype = CONF_UART_PARITY,
#ifdef CONF_UART_STOP_BITS
		.stopbits = CONF_UART_STOP_BITS,
#endif
	};

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}

 /*!
 *  Radio Initialization.
 *
 *  @author Sz. Papp
 *
 *  @note
 *
 */
uint8_t vRadio_Init(void)
{
	/* Initialize the console UART. */
	configure_console(); // will be used but inited along with cdc comm module, liyenho
	COMPILER_WORD_ALIGNED
	static usb_cdc_line_coding_t uart_coding; // used for si446x radio dev, liyenho
	  uart_coding.dwDTERate = CPU_TO_LE32(UDI_CDC_DEFAULT_RATE);
	  uart_coding.bCharFormat = UDI_CDC_DEFAULT_STOPBITS;
	  uart_coding.bParityType = UDI_CDC_DEFAULT_PARITY;
	  uart_coding.bDataBits = UDI_CDC_DEFAULT_DATABITS;
	// re-config/open for si4463 ctrl with uart port, liyenho
		uart_config(0, &uart_coding);
		uart_open(0);
	return 0;
}

uint8_t control_channel = 0;
uint8_t hop_id[HOP_ID_LEN]={0,0,0,0,0,0,0,0,0,0}; // 10 byte hop id from host
int fhop_offset = HOP_2CH_ENABLE?(HOP_2CH_OFFSET0):0;
int fhop_base = 0;
int fhopless= 0/*can be 1, 2 or 3*/;
int fhop_idx= 0; //to produce frequency hopping sequence

void Determine_FEC_Status(void){
	if (FEC_Option != AUTO){
		//host has control over FEC status, don't do anything here
		return;
	}
	//not implemented yet

}
//global variable to control using 915 vs 868 MHz
bool USE_915MHZ = true;//false;


uint32_t wait_cycles = 0;
//adds 1 message to the outgoing message buffer
//if FEC is enabled, will split into 2 packets
//returns true if the message was added
//returns false if the buffer is full
bool Queue_Control_Packet(uint8_t * pending_msg){

	if (Send_with_FEC){

		uint8_t pkt1_data[16] = {0};
		uint8_t pkt2_data[16] = {0};
		//put in header information
		pkt1_data[0] = (MSG_TYPE_HDR_HAS_FEC | MSG_TYPE_HOST_GENERATED_A);
		pkt2_data[0] = (MSG_TYPE_HDR_HAS_FEC | MSG_TYPE_HOST_GENERATED_B);
		memcpy(pkt1_data+1, pending_msg, RADIO_GRPPKT_LEN/2);
		memcpy(pkt2_data+1, pending_msg+(RADIO_GRPPKT_LEN/2), RADIO_GRPPKT_LEN/2);

		uint8_t msg_a[RADIO_PKT_LEN] = {0};
		uint8_t msg_b[RADIO_PKT_LEN] = {0};
		Encode_Control_Packet(pkt1_data, msg_a );
		Encode_Control_Packet(pkt2_data, msg_b );

		//send messages to be queued
		bool success = false;
		success = Queue_Message(msg_a);
		if (success){
			success &= Queue_Message(msg_b);
		}
		return success;

	}else{
		//add header info to message and send to be queued
		uint8_t radio_msg[RADIO_PKT_LEN] ={0};
		radio_msg[0] = (MSG_TYPE_HDR_NO_FEC | MSG_TYPE_HOST_GENERATED);
		memcpy(radio_msg+1, pending_msg, RADIO_GRPPKT_LEN);
		return Queue_Message(radio_msg);
	}
}

#define RADIO_IDLE_CHAR		0xe5
uint8_t idle_pkt[RADIO_PKT_LEN] = {RADIO_IDLE_CHAR, RADIO_IDLE_CHAR, RADIO_IDLE_CHAR, RADIO_IDLE_CHAR, RADIO_IDLE_CHAR,\
					RADIO_IDLE_CHAR, RADIO_IDLE_CHAR, RADIO_IDLE_CHAR, RADIO_IDLE_CHAR, RADIO_IDLE_CHAR,\
					RADIO_IDLE_CHAR, RADIO_IDLE_CHAR, RADIO_IDLE_CHAR, RADIO_IDLE_CHAR, RADIO_IDLE_CHAR,\
					RADIO_IDLE_CHAR, RADIO_IDLE_CHAR, RADIO_IDLE_CHAR, RADIO_IDLE_CHAR, RADIO_IDLE_CHAR,\
					RADIO_IDLE_CHAR, RADIO_IDLE_CHAR, RADIO_IDLE_CHAR, RADIO_IDLE_CHAR, RADIO_IDLE_CHAR,\
					RADIO_IDLE_CHAR, RADIO_IDLE_CHAR, RADIO_IDLE_CHAR, RADIO_IDLE_CHAR, RADIO_IDLE_CHAR,\
					RADIO_IDLE_CHAR, RADIO_IDLE_CHAR};

//place idle packet in the queue
bool Queue_Control_Idle_Packet(){
	//put in header information
	idle_pkt[0] = (Send_with_FEC?MSG_TYPE_HDR_HAS_FEC:MSG_TYPE_HDR_NO_FEC)|\
		MSG_TYPE_IDLE;

	*(uint16_t*)(idle_pkt+3) = radio_mon_txidlecnt;
	uint8_t *ptr = idle_pkt;
	if (Send_with_FEC){
		uint8_t temp[RADIO_PKT_LEN] = {0};
		//only sending 16 bytes of idle message, that is ok
		Encode_Control_Packet(idle_pkt,  temp);

		ptr = temp;
	}

	bool success = Queue_Message(ptr);
	if(success){
		radio_mon_txidlecnt++;
	}
	return success;
}

bool Queue_Base_GPS_Packet(){
	uint8_t msg[RADIO_PKT_LEN] = {0};
	//assign FEC status and Base GPS Message header
	msg[0] = (Send_with_FEC?MSG_TYPE_HDR_HAS_FEC:MSG_TYPE_HDR_NO_FEC)|MSG_TYPE_BASE_GPS_INFO;

	*(uint8_t *)gp_rdo_tpacket = (MSG_TYPE_HDR_NO_FEC | MSG_TYPE_BASE_GPS_INFO);
	Encode_Base_GPS_into_Control_Message(msg);
	return Queue_Message(msg);
}

bool Queue_FEC_Request_On_Packet(){
	uint8_t request_msg[RADIO_GRPPKT_LEN] = {0};

	request_msg[0]= (Send_with_FEC?MSG_TYPE_HDR_HAS_FEC: MSG_TYPE_HDR_NO_FEC)|\
		MSG_TYPE_REQUEST_FEC_ON;

	return Queue_Message(request_msg);
}

bool Queue_FEC_Request_Off_Packet(){
	uint8_t request_msg[RADIO_GRPPKT_LEN] = {0};

	request_msg[0]= (Send_with_FEC?MSG_TYPE_HDR_HAS_FEC: MSG_TYPE_HDR_NO_FEC)|\
		MSG_TYPE_REQUEST_FEC_OFF;

	return Queue_Message(request_msg);
}


//returns how many packet slots are free in the outgoing buffer
uint32_t Control_Outbound_Queue_Available_Slots(){
	return fifolvlcalc(wrptr_rdo_tpacket,rdptr_rdo_tpacket,RDO_TPACKET_FIFO_SIZE);
}

#ifdef WITH_ANT_SWITCH
 extern volatile bool gl_vid_ant_sw;
bool Queue_Msg_Vid_Ant_Switch(void){
	uint8_t msg[RADIO_GRPPKT_LEN] = {0};
	msg[0] = (Send_with_FEC?MSG_TYPE_HDR_HAS_FEC:MSG_TYPE_HDR_NO_FEC)|MSG_TYPE_SWITCH_VID_ANT;

	bool success = Queue_Message(msg);
	gl_vid_ant_sw = !success; //if message was stored, turn off flag
	return success;
}
#endif

#define CHANNEL_CHANGE_REPEAT_CNT	4
bool Queue_Channel_Change_Message(void){
	uint8_t msg[RADIO_GRPPKT_LEN] = {0};

	for (int i =0; i< CHANNEL_CHANGE_REPEAT_CNT; i++){
		msg[2*i] = MSG_TYPE_SET_CHANNEL;
		msg[1+2*i] = channel_scan_selection;
	}
	return Queue_Message(msg);
}

void Set_Pair_ID(uint8_t *ID){
	#if FIXED_PAIR_ID
	uint8_t fixed_id[HOP_ID_LEN] = {1};
	ID = fixed_id;
	#endif //FIXED_PAIR_ID


	uint8_t idleflag=1, pairingidflag=1, valb;
	for (int j=0; j<HOP_ID_LEN; j++) {
		valb= *(ID+j);
		if(valb!=0) idleflag=0;
		if((j<HOP_ID_LEN-1)&(valb!=0)) pairingidflag=0;
		if((j==HOP_ID_LEN-1)&(valb!=1)) pairingidflag=0;
		*(hop_id+j) = *(ID+j);
	}
	if(idleflag) hop_state = IDLE;
	else if(pairingidflag) hop_state = PAIRING ;
	else hop_state = PAIRED ;

	if(hop_state == idleflag) {
		ctrl_tdma_enable = false;
		ctrl_tdma_lock = false;
	}
	else if (hop_state == PAIRED) {
		int id=0, bit = 0;
		id |= *(hop_id+0) & (1<<bit); bit+=1 ;
		id |= *(hop_id+2) & (1<<bit); bit+=1 ;
		id |= *(hop_id+4) & (1<<bit); bit+=1 ;
		id |= *(hop_id+5) & (1<<bit); bit+=1 ;
		id |= *(hop_id+7) & (1<<bit); bit+=1 ;
		id |= *(hop_id+9) & (1<<bit); bit+=1 ;

		// calculated hop id from PAIR ID mode
	#if false
		fhop_offset = (CHTBL_SIZE>id) ? id : id-CHTBL_SIZE ;
		fhop_base=0;  //TBD for 50ch hop case
	#else // accommodate further frequency shift algorithm too, liyenho
		int fshf=0;
		bit = 0;
		fshf |= *(hop_id+1) & (1<<bit); bit+=1 ;
		fshf |= *(hop_id+3) & (1<<bit); bit+=1 ;
		fshf |= *(hop_id+6) & (1<<bit); bit+=1 ;
		fshf |= *(hop_id+8) & (1<<bit); bit+=1 ;
		fhop_offset = (CHTBL_SIZE>id) ? id : id-CHTBL_SIZE ;
		fhop_base = (NUM_FREQ_SHIFT>fshf) ? fshf : fshf-NUM_FREQ_SHIFT ;
	#endif
		if(HOP_2CH_ENABLE) {
			int i;
			for(i=0;i<10;i++)
			fhop_base= fhop_base + (*(hop_id+i));
			fhop_base = (1+(  (fhop_base & 0x0f)+((fhop_base>>4)&0x0f)&0x0f  ))*2;
			//add all bytes, then, add the two 4bitNibbles ---> range (1 - 16)*2
			fhop_offset = WRAP_OFFSET(fhop_base+HOP_2CH_OFFSET0);
		}
		ctrl_tdma_enable = true;
	}
	else  //pairing
	{
		fhop_base = 0;
		fhop_offset = HOP_2CH_ENABLE?WRAP_OFFSET(HOP_2CH_OFFSET0):0;
		ctrl_tdma_enable = true;
	}
	// frequency hopping in the action, initialization prior to hop operation because Rx side is currently the slave

	//ToDo: Update Sync Words using hop id information
	//Set_Sync_Words(byte1,byte2);

	ctrl_hop_global_update(true); // how should we select

}

void ctrl_hop_global_update(bool listen) {
#if false // unavailable on taisync platform
	#if HOP_2CH_ENABLE
	if (0 != ctrl_band_select(RF_FREQ_CONTROL_INTE_LEN, chtbl_ctrl_rdo[fhop_offset*2])) {
	#else // accommodate further frequency shift algorithm too, liyenho
	uint32_t frac=0, fshf = fhop_base * FREQ_SHIFT_STEP;
	uint8_t intr, *f=&frac, fctrl_str[RF_FREQ_CONTROL_INTE_LEN];
	memcpy(fctrl_str, chtbl_ctrl_rdo[fhop_offset*2], sizeof(fctrl_str));
	intr = *(fctrl_str + FREQ_INTR_POS);
	*(f+2) = *(fctrl_str + FREQ_FRAC_POS);
	*(f+1) = *(fctrl_str + FREQ_FRAC_POS+1);
	*(f) = *(fctrl_str + FREQ_FRAC_POS+2);
	frac = frac + (float)fshf * FREQ_CTRL_FACTOR;
	if (0x100000 <= frac) {
		frac -= 0x100000;
		intr += 0x1;
	}
	// update frequency control string with new settings
	*(fctrl_str + FREQ_FRAC_POS) = *(f+2);
	*(fctrl_str + FREQ_FRAC_POS+1) = *(f+1);
	*(fctrl_str + FREQ_FRAC_POS+2) = *(f);
	*(fctrl_str + FREQ_INTR_POS) = intr ;
	if (0 != ctrl_band_select(RF_FREQ_CONTROL_INTE_LEN, fctrl_str)) {
	#endif
		while (1) {
			; // Capture error
		}
	} // update integral part of frequency control
	if (0 != ctrl_band_select(RF_MODEM_AFC_LIMITER_LEN, chtbl_ctrl_rdo[fhop_offset*2+1])) {
		while (1) {
			; // Capture error
		}
	} // update AFC tracking limiter range
#endif
	fhop_offset = hop_chn_sel(fhop_offset);
#if false  // no need on taisync platform, liyenho
	if (listen) {
		if (!USE_915MHZ){
			vRadio_StartRX(control_channel,	RADIO_LONG_PKT_LEN);
		}else{
			vRadio_StartRX(control_channel, RADIO_PKT_LEN);
		}
	}
#endif
}

int hop_chn_sel(int offset) {
	  if (fhopless) {
		  switch(fhopless) {
			  case 1/*low*/: return 0;
			  case 2/*mid*/: return (CHTBL_SIZE)/2-1;
			  case 3/*high*/: return (CHTBL_SIZE)-1;
			  default: /* Capture error */
					while (1) {
						;
					}
		  }
	  }
#if HOP_2CH_ENABLE
		  //debug testing
	 int curr_oft = WRAP_OFFSET(HOP_2CH_OFFSET0+fhop_base);
		  if(offset==curr_oft )
		  	offset=WRAP_OFFSET(HOP_2CH_OFFSET1+fhop_base);
		  else /*(offset==(HOP_2CH_OFFSET1+fhop_base)||offset==(HOP_2CH_OFFSET0))*/
		  	offset = WRAP_OFFSET(HOP_2CH_OFFSET0+fhop_base);
#else
	  // simple random walk (forwar-backward) approach
	  /*if (true == fhop_dir) {
		offset = FORWARD_HOP + offset;
		offset = (HOPPING_TABLE_SIZE>offset) ? offset : offset-HOPPING_TABLE_SIZE;
	  	fhop_dir = false;
	  }
	  	else {
		offset = offset - BACKWARD_HOP;
		offset = (0 <= offset) ? offset : HOPPING_TABLE_SIZE + offset;
	  	fhop_dir = true;
	  	}*/
	  	offset = hopping_patterns[fhop_idx++] ;
	  	fhop_idx = (CHTBL_SIZE>fhop_idx)? fhop_idx: 0;
#endif
	  	return offset;
}

#ifdef CTRL_DYNAMIC_MOD
  void process_range_mode(U8 bMain_IT_Status) {
	  if(bMain_IT_Status == SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_PACKET_SENT_PEND_BIT)
	  {
		  // A TX was just finished  Need to start RX
		  static BW_CTRL prev_ctrl_bits = (BW_CTRL) -1;
			/********************************************************/
			if (prev_ctrl_bits != ctrl_sts.bw_ctrl_bits && NEUTRAL != ctrl_sts.bw_ctrl_bits) {
				if (0 != range_mode_configure(SHORT_RNG!=ctrl_sts.bw_ctrl_bits)) {
					//puts("error from range_mode_configure()");
					/*return*/ ; // time out, don't proceed
				}
			}
			/********************************************************/
			prev_ctrl_bits = ctrl_sts.bw_ctrl_bits;
	  }
	  if((bMain_IT_Status == SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_PACKET_RX_PEND_BIT))
	  {
		  // An RX data was received, and an interrupt was triggered.
			/************************************************************/
			{	static unsigned int recv_cnt = 0;
				BW_CTRL ctrl_bits_tmp;
				uint8_t j, err, cks=0, *pb = (uint8_t*)gp_rdo_rpacket_l;
				for (j=0;j<pRadioConfiguration->Radio_PacketLength-1;j++)
				{	cks ^= *pb++;  } // generate 2's mod checksum
				cks &= CTRL0_IMSK;  // took high 6 bits
				cks ^= CTRL_MSK & (*pb<<CTRL_BITS); // then bw ctrl bits @ end
				err = (CHKSM_MSK & *pb) ^ cks;
				uint32_t ew;
				ew = error_weight_cks6b[err>>CTRL_BITS];
				//lapse = recv_cnt * CTRL_RX_LOOP_LATENCY;
				// update error accum per time lapse
				if (initCnt > recv_cnt) {
					ctrl_sts.errPerAcc = ctrl_sts.errPerAcc * Ewim + ew * Ewis;
					ctrl_sts.errPerAcc >>= initScl;
				}
				else if (initCnt <= recv_cnt && inteCnt > recv_cnt) {
					ctrl_sts.errPerAcc = ctrl_sts.errPerAcc * Ewmm + ew * Ewms;
					ctrl_sts.errPerAcc >>= inteScl;
				}
				else {
					ctrl_sts.errPerAcc = ctrl_sts.errPerAcc * Ewnm + ew * Ewns;
					ctrl_sts.errPerAcc >>= normScl;
				}
				// compute range estimation
				if (shThr >= ctrl_sts.errPerAcc) {
					if (SHORT_RNG == (CTRL0_MSK & *pb)) {
						if (shThr >= ew)
							ctrl_bits_tmp = SHORT_RNG;
						else
							ctrl_bits_tmp = NEUTRAL;
					}
					else
						ctrl_bits_tmp = NEUTRAL;
				}
				else if (lgThr < ctrl_sts.errPerAcc) {
					ctrl_bits_tmp = LONG_RNG;
				}
				else {
					if (LONG_RNG == (CTRL0_MSK & *pb))
						ctrl_bits_tmp = LONG_RNG;
					else
						ctrl_bits_tmp = NEUTRAL;
				}
				// perform hangover procedure
				switch(ctrl_bits_tmp) {
					case SHORT_RNG :
						for (j=0; j<CTRL_CTX_LEN; j++) {
							if (SHORT_RNG != ctrl_sts.ctrl_bits_ctx[j])
								break;
						}
						if (CTRL_CTX_LEN == j)
							ctrl_sts.bw_ctrl_bits = SHORT_RNG;
						else  { // in hysteresis region
							for (j=0; j<CTRL_CTX_LEN; j++) {
								if (NEUTRAL != ctrl_sts.ctrl_bits_ctx[j])
									break;
							}
							if (CTRL_CTX_LEN == j)
								ctrl_sts.bw_ctrl_bits = SHORT_RNG;
							else  // cond above shall break continuous neutral case...
								ctrl_sts.bw_ctrl_bits = NEUTRAL;
						}
						break;
					case LONG_RNG :
						for (j=0; j<CTRL_CTX_LEN; j++) {
							if (LONG_RNG == ctrl_sts.ctrl_bits_ctx[j])
								break; // found a LG req in ctx
						}
						if (CTRL_CTX_LEN != j)
							ctrl_sts.bw_ctrl_bits = LONG_RNG;
						else // close in hysteresis region
							ctrl_sts.bw_ctrl_bits = NEUTRAL;
						break;
					default : /*NEUTRAL*/
						ctrl_sts.bw_ctrl_bits = NEUTRAL;
							if (NEUTRAL == ctrl_sts.ctrl_bits_ctx[0] &&
								NEUTRAL == ctrl_sts.ctrl_bits_ctx[1]) {
								// fall back to long range mode if two neutral seen in a row
								ctrl_sts.bw_ctrl_bits = LONG_RNG;
								break;
							}
						break;
				}
				// bump up recv_cnt
				recv_cnt = recv_cnt + 1;
			}
			ctrl_sts.loop_cnt = 0;  //reset
	  } else { // not receiving anything yet
		  unsigned int lapse = ctrl_sts.loop_cnt *CTRL_RX_LOOP_LATENCY;
			if (LONG_RNG!=ctrl_sts.bw_ctrl_bits && CTRL_MON_PERIOD <lapse) {
				ctrl_sts.bw_ctrl_bits = NEUTRAL;
				if (CTRL_FAIL_PERIOD <lapse) {
					ctrl_sts.bw_ctrl_bits = LONG_RNG;
					if (0 != range_mode_configure(LONG_RNG)) {
						// puts("error on range_mode_configure()");
						//return ; // time out, don't proceed
					}
					ctrl_sts.loop_cnt = 0;
				}
			}
		}
		// update ctrl bits history
		for (int j=CTRL_CTX_LEN-1; j>0; j--)
			ctrl_sts.ctrl_bits_ctx[j] = ctrl_sts.ctrl_bits_ctx[j-1];
		*ctrl_sts.ctrl_bits_ctx = ctrl_sts.bw_ctrl_bits;
		ctrl_sts.loop_cnt = ctrl_sts.loop_cnt + 1;
		/************************************************************/
}
#endif //CTRL_DYNAMIC_MOD
