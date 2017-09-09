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


bool Send_with_FEC = false; // whether control link messages are sent with FEC
enum FEC_Options FEC_Option = AUTO; //controls whether host or Atmel owns Send_With_FEC

bool Requested_FEC_On = false;

unsigned int  radio_mon_txidlecnt = 0;



void Determine_FEC_Status(){
	if (FEC_Option != AUTO){
		//host has control over FEC status, don't do anything here
		return;
	}
	//not implemented yet
	
}


//void Ctrl_Set_FEC(uint8_t FEC_Options){
	//switch(host_setting){
		//case OFF:
			//send_with_FEC = false;
			//break;
		//case ON:
			//send_with_FEC = true;
			//break;
		//case AUTO:
			//break;
		//default:
			////unrecognized code!!! Don't do anything
			//return;
	//}
//}

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
//place idle packet in the queue

//TODO: FIX THIS
bool Queue_Control_Idle_Packet(){
	uint8_t idle_msg[RADIO_PKT_LEN];
	//put in header information
	idle_msg[0] = (Send_with_FEC?MSG_TYPE_HDR_HAS_FEC:MSG_TYPE_HDR_NO_FEC)|\
		MSG_TYPE_IDLE;	
	memset(idle_msg+1, RADIO_IDLE_CHAR, RADIO_GRPPKT_LEN);
	*(uint16_t*)(idle_msg+3) = radio_mon_txidlecnt;
	
	if (Send_with_FEC){
		uint8_t temp[RADIO_PKT_LEN] = {0};
		Encode_Control_Packet(idle_msg,  temp);
		//copy actual message to send
		//this will overwrite the second half of the idle message
		//that is ok, there is nothing important there
		memcpy(idle_msg,temp,RADIO_PKT_LEN);
	}
	
	bool success = Queue_Message(idle_msg);
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


//returns how many packet slots are free in the outgoing buffer
uint32_t Control_Outbound_Queue_Available_Slots(){
	return fifolvlcalc(wrptr_rdo_tpacket,rdptr_rdo_tpacket,RDO_TPACKET_FIFO_SIZE);
}
