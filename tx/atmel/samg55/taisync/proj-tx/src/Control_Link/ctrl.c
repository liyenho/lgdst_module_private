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
#include "main.h"
#include "taisync.h"

#include "asf.h" // to avoid problems of usart_serial_options_t
#include "conf_uart_serial.h"

bool Send_with_FEC = false; // whether control link messages are sent with FEC
enum FEC_Options FEC_Option = AUTO; //controls whether host or Atmel owns Send_With_FEC

bool Requested_FEC_On = false;
bool send_long_packet = true;

unsigned int radio_mon_txidlecnt = 0;

//used to every so often send idle message
uint32_t idle_msg_queue_intv = 0;

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

	//temp for testing
	//put RSSI in message to monitor during flight
	*(uint32_t *)(idle_pkt+20) = radio_mon_rxcnt;
	uint32_t ldpc_err_taisync;
		get_rdo_ldpc_failed(&ldpc_err_taisync) ;
	*(uint32_t *)(idle_pkt+24) = ldpc_err_taisync; /*crc_err_cnt; unavailable on taisync platform*/

	idle_pkt[29] = ctrl_tdma_lock;
	uint8_t rssi_taisync;
		get_radio_rssi(&rssi_taisync) ;
	idle_pkt[30] = rssi_taisync; /*FRR_Copy[0]; latched rssi:[0], cur_state:[1]*/

	bool success = Queue_Message(ptr);
	if(success){
		radio_mon_txidlecnt++;
	}
	return success;
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


//copies a radio packet from the queue to location specified by *ptr, if a message is available
uint8_t Get_Control_Packet(uint8_t *ptr){

	if (fifolvlcalc(wrptr_rdo_tpacket, rdptr_rdo_tpacket, RDO_TPACKET_FIFO_SIZE)>1)
	{
		//copy packet from storage buffer to ptr
		memcpy(ptr, gs_rdo_tpacket+(rdptr_rdo_tpacket*RDO_ELEMENT_SIZE), RADIO_PKT_LEN);
		rdptr_inc(&wrptr_rdo_tpacket, &rdptr_rdo_tpacket, RDO_TPACKET_FIFO_SIZE,1);
		return 1;
	}

	return 0;
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

	ctrl_hop_global_update(true);

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

#ifdef CTRL_RADIO_ENCAP
// To be utilized by Bill
const unsigned char ts_rdo_hdr[] = { // ctrl radio pid : 0x1000
	0x47,0x10,0x00,0x30,0x02,0x82,
0x00/*size to be filled*/ };


#endif


#ifdef CTRL_RADIO_ENCAP

#if SEND_MAVLINK

#define MSG_TYPE_MAV_ONE_BLOCK		0xD0
#define MSG_TYPE_MAV_TWO_BLOCKS_A	0xDA
#define MSG_TYPE_MAV_TWO_BLOCKS_B	0xDB
//returns number of TS blocks used
uint8_t Insert_Control_In_TSStream(uint8_t *ptr){
	uint8_t packets_used =0;
	static int TS_hdr_sz = sizeof(ts_rdo_hdr);

	static uint32_t frame_size = 188;
	uint8_t bytes_avail = frame_size - TS_hdr_sz-1;
	MavLinkPacket pkt={0};

	if (!Get_MavLink(&outgoing_messages, &pkt))
	{
		//No MavLink packets are queued to send, don't fill in anything
		packets_used = 0;
	}else{
		//A MavLink packet is available to send
		memcpy(ptr, ts_rdo_hdr, TS_hdr_sz);

		uint8_t *MavStart = ptr+TS_hdr_sz+1;
		uint32_t packet_size = MavLink_Total_Bytes_Used(pkt);

		if (packet_size<=bytes_avail){
			//MavLink packet will fit into 1 TS block
			//copy header plus payload
			*(MavStart-1) = MSG_TYPE_MAV_ONE_BLOCK;
			memcpy(MavStart, &(pkt.header), packet_size-MAVLINK_CHKSUM_LEN);
			//copy checksum separately since there may be a gap between end of payload and checksum
			memcpy(MavStart+packet_size-MAVLINK_CHKSUM_LEN, &(pkt.checksum[0]), MAVLINK_CHKSUM_LEN);

			*(ptr+6) = packet_size+1;
			memset(ptr+TS_hdr_sz+packet_size+1, 0xff,	frame_size-TS_hdr_sz-packet_size-1);
			packets_used = 1;

		}else{
			//need to split across two TS blocks

			//identify as first half of group
			*(MavStart-1) = MSG_TYPE_MAV_TWO_BLOCKS_A;
			//get start of second block
			uint8_t *ptr2 = ptr+frame_size;
			//fill in second frame header
			memcpy(ptr2, ts_rdo_hdr, TS_hdr_sz);
			//identify as second half of group
			*(ptr2+TS_hdr_sz) = MSG_TYPE_MAV_TWO_BLOCKS_B;


			//copy beginning of Mav packet into first frame
			uint8_t bytes_in_first = frame_size-TS_hdr_sz-1;
			memcpy(MavStart, &(pkt.header),bytes_in_first );

			//copy end of Mav packet into second frame

			uint8_t bytes_in_second = packet_size-bytes_in_first;
			memcpy(ptr2+TS_hdr_sz+1, &(pkt.header)+bytes_in_first, bytes_in_second-MAVLINK_CHKSUM_LEN);
			memcpy(ptr2+TS_hdr_sz+1+bytes_in_second-MAVLINK_CHKSUM_LEN, pkt.checksum, MAVLINK_CHKSUM_LEN);

			*(ptr+6) = bytes_in_first+1; //plus 1 is because of Mav msg type byte
			*(ptr2+6) = bytes_in_second+1;

			//fill in dummy data here!
			memset(ptr2+TS_hdr_sz+1+bytes_in_second, 0xff,	frame_size-TS_hdr_sz-1-bytes_in_second);

			packets_used = 2;
		  }
	  }
	return packets_used;
}

#else
extern volatile uint8_t dbg_ctrlcnt;
extern volatile uint8_t dbg_antpos;
uint8_t Insert_Control_In_TSStream(uint8_t *ptr){

	static int hdr_sz = sizeof(ts_rdo_hdr);
	uint8_t * pusb = ptr;
	memcpy(pusb, ts_rdo_hdr, hdr_sz);
	pusb += hdr_sz;
	static uint32_t frame_size = 188;
	uint32_t free_space = frame_size- hdr_sz;
	//uint32_t num_packets = free_space/RADIO_PKT_LEN;
	uint32_t max_packets = 1; //rate limit for now
	uint32_t pkts_added = 0;
	uint32_t payloadAcc;
	uint32_t i;

	for (i =0; i<max_packets; i++){
		pkts_added += Get_Control_Packet(pusb+(i*RADIO_PKT_LEN));
	}
	if (pkts_added == 0){
		//if no messages available, just return
		return 0;
	}

    payloadAcc = pkts_added*RADIO_PKT_LEN;



	uint8_t * pusb2 = pusb + pkts_added*RADIO_PKT_LEN ;

	#ifdef DEBUG_RADIOSTATUS
		static int tstcntyh=0;
		uint8_t tstcntyh_byte;
		tstcntyh++;
		tstcntyh_byte = (char)(tstcntyh>>4);
		*pusb2++ = 0x92;
		*pusb2++  = dbg_ctrlcnt;
		*pusb2++  = (ctrl_tdma_lock | dbg_antpos); //byte[0]=lock, byte[3-2]=antpos
		*pusb2++  = tstcntyh_byte;
		payloadAcc = payloadAcc + 4;
	#endif //DEBUG_RADIOSTATUS

	*(ptr+6) = payloadAcc;

	//fill in dummy data here!
	memset(ptr+hdr_sz+payloadAcc, 0xff, 188-hdr_sz-payloadAcc);

	//1 block filled
	return 1;
}


#endif //end SEND_MAVLINK
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

#endif
