/*
* Radio_Buffers.c
*
*/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "Radio_Buffers.h"
#include "ctrl.h"
#include "MavLink.h"


//array for storing received radio data
volatile uint32_t gs_rdo_rpacket[RDO_RPACKET_FIFO_SIZE*RDO_ELEMENT_SIZE] = {0xffffffff};
//array for storing outgoing radio data
volatile uint32_t gs_rdo_tpacket[RDO_TPACKET_FIFO_SIZE*RDO_ELEMENT_SIZE] = {0xffffffff};


unsigned int *gp_rdo_tpacket_l = ((unsigned int*)gs_rdo_tpacket);
unsigned int *gp_rdo_tpacket = (unsigned int*)gs_rdo_tpacket;

unsigned int *gp_rdo_rpacket = (unsigned int*)gs_rdo_rpacket;
unsigned int *gp_rdo_rpacket_l = ((unsigned int*)gs_rdo_rpacket);

volatile uint32_t wrptr_rdo_tpacket=RDO_TPACKET_FIFO_SIZE-1;   //wrptr to valid data
volatile uint32_t rdptr_rdo_tpacket=RDO_TPACKET_FIFO_SIZE-1;   //rdptr to last consummed data, this
	//rdptr=wrptr, data consumed, rdptr take priority
volatile uint32_t wrptr_rdo_rpacket=RDO_RPACKET_FIFO_SIZE-1;   //wrptr to valid data
volatile uint32_t rdptr_rdo_rpacket=RDO_RPACKET_FIFO_SIZE-1;  //rdptr to last consummed data
	//rdptr=wrptr, data consumed, rdptr take priority

extern volatile int8_t id_byte;

uint32_t fifolvlcalc(uint32_t wrptr, uint32_t rdptr, uint32_t fifodepth)
{
	if(wrptr==rdptr)
	return(0);
	if(wrptr> rdptr )
	return(wrptr - rdptr);
	return(wrptr + (fifodepth-rdptr));
}

uint32_t wrptr_inc(uint32_t *wrptr,  uint32_t *rdptr, uint32_t fifodepth, int step)
{
	uint32_t wrptr_tmp;  //get local snapshot due to race concerns
	uint32_t rdptr_tmp;  //get local snapshot due to race concerns
	int i;
	wrptr_tmp = *wrptr;
	rdptr_tmp = *rdptr;
	for(i=0;i<step;i++) {
		if(wrptr_tmp >= (fifodepth-1))
		wrptr_tmp=0;
		else
		wrptr_tmp++;
		if(wrptr_tmp== rdptr_tmp){
			//overflow condition
			return 1; //overflow
		}
	}

	*wrptr = wrptr_tmp;
	return 0;  //no overflow
}

uint32_t rdptr_inc(uint32_t *wrptr,  uint32_t *rdptr, uint32_t fifodepth, int step)
{
	uint32_t rdptr_tmp,wrptr_tmp;
	int i;
	rdptr_tmp = *rdptr;  //get local snapshot due to race concerns
	wrptr_tmp = *wrptr;  //get local snapshot due to race concerns
	for(i=0;i<step;i++) {
		if(wrptr_tmp== rdptr_tmp){
			//overflow condition
			return 1; //underflow
		}
		if(rdptr_tmp >= (fifodepth-1))
		rdptr_tmp=0;
		else
		rdptr_tmp++;
	}
	*rdptr = rdptr_tmp;
	return 0;  //good data
}


uint32_t TX_overflow = 0;
static bool lock_obj= false;

static int tries=0;

#define  QM_common_content \
	uint32_t overflow=0; \
	if (lock_obj){ \
		tries++; \
		if (tries>10){ \
			lock_obj = false; \
		} \
		return false; \
	} \
	tries =0; \
	lock_obj = true;

//adds 1 packet to the outgoing queue
bool Queue_Message(uint8_t *msg){
	QM_common_content
	uint32_t wrtptr_tmp = wrptr_rdo_tpacket;
	//check if there is  space for the new message
	overflow = wrptr_inc(&wrtptr_tmp,&rdptr_rdo_tpacket,RDO_TPACKET_FIFO_SIZE, 1);
	if (overflow){
		//not enough space available
		TX_overflow++;
		lock_obj = false;
		return false;
	}
	gp_rdo_tpacket = gs_rdo_tpacket + (RDO_ELEMENT_SIZE*wrtptr_tmp);
	//copy to output buffer
	memcpy(gp_rdo_tpacket, msg, RADIO_PKT_LEN);
	wrptr_rdo_tpacket = wrtptr_tmp;
	lock_obj = false;
	return true;
}

volatile MavLink_FIFO_Buffer outgoing_messages = {0};
volatile MavLink_FIFO_Buffer incoming_messages = {0};


//pkt is the MavLink Packet that is attempting to be queued
bool Queue_MavLink(MavLink_FIFO_Buffer *fifo, uint8_t *pkt){

	//prevent race conditions caused by interrupts etc
	if (fifo->lock_obj){
		fifo->lock_cnt++;
		//prevent indefinite deadlock
		if (fifo->lock_cnt >10){
			fifo->lock_cnt = 0;
			fifo->lock_obj = false;
		}
		return false;
	}
	uint32_t wrtptr_tmp = fifo->write_pointer;
	uint32_t overflow = wrptr_inc(&wrtptr_tmp, &(fifo->read_pointer), MavLinkBufferSize, 1);
	if (overflow){
		//buffer is full
		fifo->overflow_cnt++;
		fifo->lock_obj = false;
		return false;
	}
	//ok to add to buffer
	memcpy((MavLinkPacket*)(fifo->buffer)+wrtptr_tmp, pkt, // reduce memcpy overhead
		MAVLINK_HDR_LEN+((MavLinkPacket*)pkt)->length+MAVLINK_CHKSUM_LEN);
	fifo->write_pointer = wrtptr_tmp;
	fifo->lock_obj = false;
	return true;
}

//attempts to retrieve a MavLink packet
//if successful, packet is copied to the location of the pkt pointer
bool Get_MavLink(MavLink_FIFO_Buffer *fifo, uint8_t *pkt, bool id_chk){
	if (fifo->lock_obj){
		fifo->lock_cnt++;
		//prevent indefinite deadlock
		if (fifo->lock_cnt >10){
			fifo->lock_cnt = 0;
			fifo->lock_obj = false;
		}
		return false;
	}

	if (fifolvlcalc(fifo->write_pointer,fifo->read_pointer, MavLinkBufferSize)< 1){

		fifo->lock_obj = false;
		return false;
	}
	//packet is available, copy to destination
	rdptr_inc(&(fifo->write_pointer), &(fifo->read_pointer), MavLinkBufferSize, 1);
	MavLinkPacket* pkt0 = (MavLinkPacket*)(fifo->buffer)+fifo->read_pointer;
	if (id_chk && id_byte != pkt0->system_ID)
		return false;	// filter out strayed pkt from other units
	memcpy(pkt, pkt0, // reduce memcpy overhead
		MAVLINK_HDR_LEN+pkt0->length+MAVLINK_CHKSUM_LEN);
	fifo->lock_obj = false;
	return true;
}

bool Queue_MavLink_Raw_Data(MavLink_Bytestream *stream, uint32_t num_bytes, uint8_t * bytes){

	uint32_t bytes_read ;
	// abandon naive approach for efficiency, no need of irq blocking to protect against 'read_pointer'
	// all the read ops were done in sync manners, liyenho
	bool overflow = false;
	if (stream->write_pointer>=stream->read_pointer) {
		bytes_read = MAVLINK_BYTESTREAM_DEPTH+stream->read_pointer-stream->write_pointer-1;
		if (bytes_read>num_bytes) {
			bytes_read =num_bytes ;
		}
		else {
			overflow = true;
		}
		if (MAVLINK_BYTESTREAM_DEPTH<=bytes_read+stream->write_pointer) {
			int tmp_len = MAVLINK_BYTESTREAM_DEPTH-stream->write_pointer;
			memcpy(stream->data+stream->write_pointer, bytes, tmp_len);
			memcpy(stream->data, bytes+ tmp_len, bytes_read- tmp_len);
		}
		else {
			memcpy(stream->data+stream->write_pointer,
								bytes, bytes_read);
		}
	}
	else { // race from behind
		bytes_read = stream->read_pointer-stream->write_pointer-1;
		if (bytes_read>num_bytes) {
			bytes_read =num_bytes ;
		}
		else {
			overflow = true;
		}
		memcpy(stream->data+stream->write_pointer,
							bytes, bytes_read) ;
	}
	wrptr_inc(&(stream->write_pointer), &(stream->read_pointer),
							sizeof(stream->data)/sizeof(stream->data[0]), bytes_read);
	return !overflow;
};

//stream contains the raw bytes to process
//pkt_fifo is the fifo buffer to store the complete packets
//this is used for data coming in through serial port
void Process_MavLink_Raw_Data(void){

	MavLink_Bytestream *stream = &outgoing_MavLink_Data;
	MavLink_FIFO_Buffer *pkt_fifo = &outgoing_messages;

	if((stream->read_pointer == stream->write_pointer) || // also check availability on fifo obj,
		(pkt_fifo->read_pointer>	pkt_fifo->write_pointer &&2>(pkt_fifo->read_pointer-pkt_fifo->write_pointer))){
		//no data to process
		return;
	}
static uint32_t sts_next = 0;
	Build_MavLink_from_Byte_Stream(&sts_next, pkt_fifo, stream);
}


volatile MavLink_Bytestream incoming_MavLink_Data = {0};
volatile MavLink_Bytestream outgoing_MavLink_Data = {0};

//this is used for data received over wireless link
void Process_MavLink_Raw_Radio_Data(void){
	MavLink_Bytestream *stream = &incoming_MavLink_Data;
	MavLink_FIFO_Buffer *pkt_fifo = &incoming_messages;

	if((stream->read_pointer == stream->write_pointer) || // also check availability on fifo obj,
		(pkt_fifo->read_pointer>	pkt_fifo->write_pointer &&2>(pkt_fifo->read_pointer-pkt_fifo->write_pointer))){
		//no data to process
		return;
	}
static uint32_t sts_next = 0;
	Build_MavLink_from_Byte_Stream(&sts_next, pkt_fifo, stream);
}

bool Build_MavLink_from_Byte_Stream(uint32_t *sts_next, MavLink_FIFO_Buffer *fifo, MavLink_Bytestream *stream) {
#define SEARCH_NEXT(sn) \
		rdptr_str= (MAVLINK_BYTESTREAM_DEPTH-1) & (1+rdptr_str); \
		if (wrptr_str == rdptr_str) { \
			stream->read_pointer = rdptr_str; \
			*sts_next = sn; \
			return false;  /*search exhausted*/ \
		} \
		next_byte = *(stream->data+rdptr_str);
#define SEARCH_COMPLETE(sn) \
		if (sn) { \
			stream->read_pointer= rdptr_str; \
			*sts_next = sn; \
			return false; \
		} \
		else /*search restart*/\
			goto sts_next_0;

	int32_t wrptr_fifo=fifo->write_pointer,
					rdptr_fifo=fifo->read_pointer,
					tlen0, tlen1, tlen2;
	uint8_t *pkt, tmp_l, next_byte;
	if (rdptr_fifo>wrptr_fifo && 2> (rdptr_fifo-wrptr_fifo))
		return false;  // no available pkt buffer
	int32_t wrptr_str, rdptr_str=stream->read_pointer;
	 wrptr_str = stream->write_pointer;
	// get a pkt buffer from fifo
	pkt = fifo->buffer+wrptr_fifo *MavLinkPacketSize;
	// load a byte from bit stream
	next_byte = *(stream->data+rdptr_str);
	// check for next state
	switch(*sts_next) {
		case 1: goto sts_next_1;
		default: break;
	}
	// search start sign only when previous search ended on mav pkt boundary
	while (1) {
sts_next_0:
		if (MAVLINK_START_SIGN == next_byte) {
			SEARCH_NEXT(1)
sts_next_1:
			tlen1 = MAVLINK_HDR_LEN+MAVLINK_CHKSUM_LEN+next_byte/*pkt len*/-1/*-1*/;
			tlen0 = (MAVLINK_BYTESTREAM_DEPTH-1) & (wrptr_str-rdptr_str);
			if (tlen1<tlen0) {
				tlen2 = (MAVLINK_BYTESTREAM_DEPTH-1)&(rdptr_str+tlen1);
				tmp_l = *(stream->data+tlen2);
				if (MAVLINK_START_SIGN != tmp_l) {
					SEARCH_COMPLETE(0)
				}
				else {
					pkt[0] = MAVLINK_START_SIGN;
					pkt[1]= next_byte;
					break;	// found a mav pkt
				}
			}
			else {
				SEARCH_COMPLETE(1)
			}
		}
		SEARCH_NEXT(0)
	}
	if (MAVLINK_BYTESTREAM_DEPTH<rdptr_str+tlen1) {
		tlen2 = MAVLINK_BYTESTREAM_DEPTH-rdptr_str-1;
		memcpy(pkt+2, stream->data+rdptr_str+1, tlen2);
		memcpy(pkt+tlen2+2, stream->data, tlen1-tlen2-1);
	}
	else {
		memcpy(pkt+2, stream->data+rdptr_str+1, tlen1-1);
	}
	stream->read_pointer = (MAVLINK_BYTESTREAM_DEPTH-1)& (rdptr_str+tlen1);
	fifo->write_pointer = (MavLinkBufferSize-1) & (wrptr_fifo+1);
	*sts_next = 0; // restart state machine
	return true;

#undef  SEARCH_NEXT
#undef SEARCH_COMPLETE
}
