/*
 * ctrl.c
 *
 *
 */


#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "ctrl.h"
#include "Radio_Buffers.h"
#include "ReedSolomon.h"
#include "main.h"
#include "bsp.h"
#include "radio.h"
#include "uart.h"


bool Send_with_FEC = false; // whether control link messages are sent with FEC
enum FEC_Options FEC_Option = AUTO; //controls whether host or Atmel owns Send_With_FEC

bool Requested_FEC_On = false;
bool send_long_packet = true;

unsigned int radio_mon_txidlecnt = 0;

//used to every so often send idle message
uint32_t idle_msg_queue_intv = 0;

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
	*(uint32_t *)(idle_pkt+24) = crc_err_cnt;

	idle_pkt[29] = ctrl_tdma_lock;
	idle_pkt[30] = FRR_Copy[0];

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

bool Queue_Channel_Change_ACK(void){
	uint8_t msg[RADIO_GRPPKT_LEN] = {0};

	msg[0] = MSG_TYPE_SET_CHANNEL;
	msg[1] = MSG_TYPE_SET_CHANNEL;
	msg[2] = MSG_TYPE_SET_CHANNEL;

	return Queue_Message(msg);
}

#define CHANNEL_CHANGE_REPEAT_CNT	4
void Handle_Channel_Change_Request(uint8_t *msg_start){
	//checks message to make sure channel change request is valid
	uint8_t header_ok_cnt = 0;
	uint8_t new_channel = 0;
	uint8_t channel_ok_cnt =0;

	if (USE_915MHZ){
		//only applies to 869 MHz. 915 uses hopping
		return;
	}

	new_channel = *(msg_start+1);
	for (int i =0; i<CHANNEL_CHANGE_REPEAT_CNT; i++){
		if (MSG_TYPE_SET_CHANNEL == (*(msg_start+i*2) & 0x0F)){
			//every other byte should be a copy of the message header
			header_ok_cnt++;
		}
		if (new_channel == *(msg_start+1+(2*i))){
			//every other byte should be a copy of the channel
			channel_ok_cnt++;
		}

	}
	//require all bytes to be correct to accept change
	if (header_ok_cnt == CHANNEL_CHANGE_REPEAT_CNT){
		if (channel_ok_cnt == CHANNEL_CHANGE_REPEAT_CNT){
			control_channel = new_channel;
			Queue_Channel_Change_ACK();
			Queue_Channel_Change_ACK();
			Queue_Channel_Change_ACK();
			Queue_Channel_Change_ACK();
		}
	}

	return;
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

#if SEND_MAVLINK  // added,
uint8_t idle_mav_cnt =0;
bool Queue_Idle_Mavlink(void){
	uint8_t mav_pkt_buff[MAVLINK_HDR_LEN+RADIO_GRPPKT_LEN+MAVLINK_CHKSUM_LEN];
	MavLinkPacket *idle_pkt =(MavLinkPacket*)mav_pkt_buff; //for efficiency

	idle_pkt->header = MAVLINK_START_SIGN;
	idle_pkt->length = RADIO_GRPPKT_LEN;
	idle_pkt->sequence = idle_mav_cnt;
	idle_mav_cnt++;

	memset(idle_pkt->data, RADIO_IDLE_CHAR, idle_pkt->length);

	*(uint16_t*) (mav_pkt_buff+MAVLINK_HDR_LEN+RADIO_GRPPKT_LEN) = Compute_Mavlink_Checksum(idle_pkt);
	return Queue_MavLink(&outgoing_messages, mav_pkt_buff);
}
#endif

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

	const uint32_t frame_size = 188;
	uint8_t bytes_avail = frame_size - TS_hdr_sz-1,
					pkt[MAX_MAVLINK_SIZE];

	if (!Get_MavLink(&outgoing_messages, pkt))
	{
		//No MavLink packets are queued to send, don't fill in anything
		packets_used = 0;
	}else{
		//A MavLink packet is available to send
		memcpy(ptr, ts_rdo_hdr, TS_hdr_sz);

		uint8_t *MavStart = ptr+TS_hdr_sz+1;
		uint32_t packet_size = MavLink_Total_Bytes_Used((MavLinkPacket*)pkt);

		if (packet_size<=bytes_avail){
			//MavLink packet will fit into 1 TS block
			//copy header plus payload
			*(MavStart-1) = MSG_TYPE_MAV_ONE_BLOCK;
			memcpy(MavStart, pkt, packet_size);

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
			memcpy(MavStart, pkt, bytes_in_first );

			//copy end of Mav packet into second frame

			uint8_t bytes_in_second = packet_size-bytes_in_first;
			memcpy(ptr2+TS_hdr_sz+1, pkt+bytes_in_first, bytes_in_second);

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
	const uint32_t frame_size = 188;
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

#endif
