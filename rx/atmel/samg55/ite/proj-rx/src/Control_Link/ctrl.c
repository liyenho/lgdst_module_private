/*
 * ctrl.c
 *
 *
 */


#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include "ctrl.h"
#include "Radio_Buffers.h"
#include "ReedSolomon.h"
#include "GPS.h"
#include "bsp.h"
#include "radio.h"


bool Send_with_FEC = FEC_ON; // whether control link messages are sent with FEC
enum FEC_Options FEC_Option = AUTO; //controls whether host or Atmel owns Send_With_FEC

bool Requested_FEC_On = false;

bool send_long_packet = true;

unsigned int radio_mon_txidlecnt = 0;
bool control_channge_change_ack = false;

volatile static uint32_t remaining_blocks = 0;
uint8_t control_channel = 0;
uint8_t hop_id[HOP_ID_LEN]={0,0,0,0,0,0,0,0,0,0}; // 10 byte hop id from host
int fhop_offset = HOP_2CH_ENABLE?(HOP_2CH_OFFSET0):0;
int fhop_base = 0;
int fhopless= 0/*can be 1, 2 or 3*/;
int fhop_idx= 0; //to produce frequency hopping sequence
extern volatile  uint8_t id_byte;
unsigned int  radio_mon_txcnt = 0;
unsigned char snd_asymm_cnt=0;

void Determine_FEC_Status(void){
	if (FEC_Option != AUTO){
		//host has control over FEC status, don't do anything here
		return;
	}
	//not implemented yet

}
//global variable to control using 915 vs 868 MHz
bool USE_915MHZ = true /*false*/;

//adds 1 message to the outgoing message buffer
//returns true if the message was added
//returns false if the buffer is full
bool Queue_Control_Packet(uint8_t * pending_msg){
		//add header info to message and send to be queued
		uint8_t lo, radio_msg[RADIO_PKT_LEN];
		radio_msg[0] = (MSG_TYPE_HOST_GENERATED);
		memcpy(radio_msg+1, pending_msg, RADIO_GRPPKT_LEN);
		 //padded last byte instead fill in all 0s, liyenho
		lo = RADIO_GRPPKT_LEN+1;
		while (lo <RADIO_PKT_LEN)
			radio_msg[lo++] = 0;
		return Queue_Message(radio_msg);
}


bool Queue_MavLink_from_USB(uint8_t *pkt_start){
	return Queue_MavLink(&outgoing_messages, pkt_start);
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
	idle_pkt[0] = MSG_TYPE_IDLE;

	*(uint16_t*)(idle_pkt+3) = radio_mon_txidlecnt;
	uint8_t *ptr = idle_pkt;
	bool success = Queue_Message(ptr);
	if(success){
		radio_mon_txidlecnt++;
	}
	return success;
}

bool Queue_Base_GPS_Packet(){
	uint8_t msg[RADIO_PKT_LEN] ;
	//assign FEC status and Base GPS Message header
	msg[0] = MSG_TYPE_BASE_GPS_INFO;

	Encode_Base_GPS_into_Control_Message(msg);
	return Queue_Message(msg);
}

bool Queue_FEC_Request_On_Packet(){
	uint8_t request_msg[RADIO_GRPPKT_LEN] ;

	request_msg[0]= MSG_TYPE_REQUEST_FEC_ON;

	return Queue_Message(request_msg);
}

bool Queue_FEC_Request_Off_Packet(){
	uint8_t request_msg[RADIO_GRPPKT_LEN] ;

	request_msg[0]= MSG_TYPE_REQUEST_FEC_OFF;

	return Queue_Message(request_msg);
}


//returns how many packet slots are free in the outgoing buffer
uint32_t Control_Outbound_Queue_Available_Slots(){
	return fifolvlcalc(wrptr_rdo_tpacket,rdptr_rdo_tpacket,RDO_TPACKET_FIFO_SIZE);
}


 extern volatile bool gl_vid_ant_sw;
bool Queue_Msg_Vid_Ant_Switch(void){
	uint8_t msg[RADIO_GRPPKT_LEN] ;
	msg[0] = MSG_TYPE_SWITCH_VID_ANT;

	bool success = Queue_Message(msg);
	gl_vid_ant_sw = !success; //if message was stored, turn off flag
	return success;
}

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


	uint8_t idleflag=1, pairingidflag=1, valb,i;
	int fshf=0;

	for (int j=0; j<HOP_ID_LEN; j++) {
		valb= *(ID+j);
		if(valb!=0) idleflag=0;
		if((j<HOP_ID_LEN-1)&(valb!=0)) pairingidflag=0;
		if((j==HOP_ID_LEN-1)&(valb!=1)) pairingidflag=0;
		*(hop_id+j) = *(ID+j);
	}

	//always compute new id_byte and id freq-shift (fshf)
	id_byte=ID_BYTE_SEED;
	for(i=0;i<HOP_ID_LEN;i++) {
		id_byte += hop_id[i];
		id_byte = id_byte%(CHTBL_SIZE);
		fshf += hop_id[i];
		fshf = fshf%(NUM_FREQ_SHIFT);
	}

	if(idleflag) hop_state = IDLE;
	else if(pairingidflag) hop_state = PAIRING ;
	else hop_state = PAIRED ;

	if(hop_state == idleflag) {
		ctrl_tdma_enable = false;
		ctrl_tdma_lock = false;
	}
	else if (hop_state == PAIRED) {

		// calculated hop id from PAIR ID mode
	#if false
		fhop_offset = (CHTBL_SIZE>id_byte) ? id_byte : id_byte-CHTBL_SIZE ;
		fhop_base=0;  //TBD for 50ch hop case
	#else // accommodate further frequency shift algorithm too, liyenho
		fhop_offset = (CHTBL_SIZE>id_byte) ? id_byte : id_byte-CHTBL_SIZE ;
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
  extern const char hopping_patterns[] ;
	  	offset = hopping_patterns[fhop_idx++] ;
	  	fhop_idx = (CHTBL_SIZE>fhop_idx)? fhop_idx: 0;
#endif
	  	return offset;
}

#ifdef CTRL_RADIO_ENCAP
	extern uint8_t  pb_rdo_ctrl[] ,*pb_rdo_ctrl_e;
#endif

#ifdef CTRL_RADIO_ENCAP
#define MSG_TYPE_MAV_ONE_BLOCK		0xD0
#define MSG_TYPE_MAV_TWO_BLOCKS_A	0xDA
#define MSG_TYPE_MAV_TWO_BLOCKS_B	0xDB

	#if RECEIVE_MAVLINK
	void radio_pkt_filled(uint8_t num_bytes) {
		static bool first_half_recv = false;
		static int bytes_read=0;

		static uint8_t *pbr1= pb_rdo_ctrl;

		uint8_t msg_type = *pbr1;

		if (MSG_TYPE_MAV_ONE_BLOCK==msg_type ){
			Queue_MavLink(&incoming_messages, pbr1+1);

			MavLink_Total_Bytes_Used((MavLinkPacket*)(pbr1+1));
		}else{
			static uint8_t mav_pkt_buff[MavLinkPacketSize]= {0};

			if (MSG_TYPE_MAV_TWO_BLOCKS_A == msg_type){

				memcpy(mav_pkt_buff, pbr1+1, num_bytes-1);
				first_half_recv = true;
				bytes_read = num_bytes-1;

			}else if (MSG_TYPE_MAV_TWO_BLOCKS_B == msg_type){
				if (first_half_recv){
					memcpy(mav_pkt_buff+bytes_read, pbr1+1, num_bytes-1);
					Queue_MavLink(&incoming_messages, mav_pkt_buff);
				}else{
					//error, first half not received, missing data
					bytes_read=0;
				}

			}else{
				//unrecognized
				//reset!
			}
		}

		pbr1+=num_bytes;
		if ((pb_rdo_ctrl_e-188)<pbr1){
			pbr1 = pb_rdo_ctrl;
		}
	}
	#else
	extern volatile uint8_t dbg_ctrlvidbuff[10];
	void radio_pkt_filled(uint8_t num_bytes) {

		static uint8_t *pbr1= pb_rdo_ctrl;

		uint32_t write_err_cnt =0;
		int i =0;

		for (i =0; i<(num_bytes/RADIO_PKT_LEN); i++){
			//do 1 by 1 instead of in bulk in case bulk fails, this way at least some of writes are ok
			memcpy(gs_rdo_rpacket +(wrptr_rdo_rpacket*RDO_ELEMENT_SIZE), pbr1+(i*RADIO_PKT_LEN), RADIO_PKT_LEN);
			write_err_cnt += wrptr_inc(&wrptr_rdo_rpacket, &rdptr_rdo_rpacket, RDO_RPACKET_FIFO_SIZE,1);
		}
		pbr1 += i*RADIO_PKT_LEN;

		#ifdef DEBUG_RADIOSTATUS

		dbg_ctrlvidbuff[0]=pbr1[0]; pbr1++; //0x92
		dbg_ctrlvidbuff[1]=pbr1[0]; pbr1++;  //ctrl st16->drone cnt
		dbg_ctrlvidbuff[2]=pbr1[0]; pbr1++;  //tdma_lock
		dbg_ctrlvidbuff[3]=pbr1[0]; pbr1++;  //video ctrl 188 transfer cnt
		dbg_ctrlvidbuff[4]=num_bytes;  //gotPayloadSize
		dbg_ctrlvidbuff[5]=1;    //validFlag

		#endif

		if ((pb_rdo_ctrl_e-188)<pbr1){
			pbr1 = pb_rdo_ctrl;
		}
		return;
	}
	#endif //end RECEIVE_MAVLINK
#endif

void Control_Send_Event(void){
	#if  SEND_MAVLINK
		static uint8_t to_send[640] ; // liyenho
		const block_size = RADIO_PKT_LEN; // based on 4463 pkt size
		static uint32_t blocks_used =0;
		static uint32_t bytes_prev= 0; // keep track on buffer ptr
		if (remaining_blocks == 0){
			if (0 != bytes_prev) {
				int rem = bytes_prev -blocks_used *block_size;
				assert (0<=rem);
				memcpy(to_send, to_send+bytes_prev -rem, rem);
				bytes_prev = rem;  // move remaining data forward, liyenho
			}
			uint32_t bytes_used;
			do {
				Get_MavLink(&outgoing_messages, to_send+bytes_prev);
				bytes_used= MavLink_Total_Bytes_Used((MavLinkPacket*)(to_send+bytes_prev));
			} while (RADIO_PKT_LEN>(bytes_prev += bytes_used));
			remaining_blocks = (bytes_prev)>>5; // RADIO_PKT_LEN==32
			blocks_used=0;
		}
		vRadio_StartTx(control_channel, to_send+block_size*blocks_used, block_size);
		remaining_blocks--;
		blocks_used++;
	#else
		uint8_t *gp_rdo= (uint8_t*)gp_rdo_tpacket_l;
		if (Send_with_FEC){
			static uint8_t msg_fec[RADIO_PKT_LEN] ;
			// added FEC hdr flag with outgoing pkt
			*gp_rdo = (0xf&*gp_rdo)|MSG_TYPE_HDR_HAS_FEC;
			Encode_Control_Packet((uint8_t*)gp_rdo_tpacket_l, msg_fec );
			gp_rdo = msg_fec;
		}
		gp_rdo[RADIO_PKT_LEN-1] = id_byte; //YH: tagging control messages with id
		if (USE_915MHZ){
			vRadio_StartTx(control_channel, gp_rdo, RADIO_PKT_LEN);
		}else{
			vRadio_StartTx(control_channel, gp_rdo, RADIO_LONG_PKT_LEN);
			snd_asymm_cnt = 0;
		}
	#endif
	radio_mon_txcnt++;
}


uint8_t idle_mav_cnt =0;
bool Queue_Idle_Mavlink(void){
	uint8_t mav_pkt_buff[MAVLINK_HDR_LEN+RADIO_GRPPKT_LEN+MAVLINK_CHKSUM_LEN];
	MavLinkPacket *idle_pkt =(MavLinkPacket*)mav_pkt_buff; //for efficiency

	idle_pkt->header = MAVLINK_START_SIGN;
	idle_pkt->length = RADIO_GRPPKT_LEN;
#ifdef MAVLINK_V2
	memset(idle_pkt->comp_flags,
				0x0, sizeof(idle_pkt->comp_flags));
#endif
	idle_pkt->sequence = idle_mav_cnt;
	idle_pkt->system_ID = 0xAB; //made up values for testing
	idle_pkt->component_ID = 0xCD; //made up values for testing
#ifdef MAVLINK_V1
	idle_pkt->message_ID = 0x1 ^ MAVLINK_ID_DATA;
#elif defined(MAVLINK_V2)
	memset(idle_pkt->message_ID,
				0x0, sizeof(idle_pkt->message_ID));
	idle_pkt->message_ID[0] = 0x1 ^ MAVLINK_ID_DATA;
#endif
	idle_mav_cnt++;

	memset(idle_pkt->data, RADIO_IDLE_CHAR, idle_pkt->length);

	uint16_t chksm = Compute_Mavlink_Checksum(idle_pkt);
	uint8_t lo=0, *pcs = (uint8_t*)&chksm,
					*pkt1 = ((uint8_t*)mav_pkt_buff+MAVLINK_HDR_LEN+RADIO_GRPPKT_LEN);
	do {	// in place of memcpy for efficiency,
		*((uint8_t*)pkt1+lo) = *pcs++;
	} while(++lo < MAVLINK_CHKSUM_LEN);

	return Queue_MavLink(&outgoing_messages, mav_pkt_buff);
}
