/*
* Radio_Buffers.c
*
*/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include "compiler_defs.h"
#include "Radio_Buffers.h"
#include "ctrl.h"
#include "MavLink.h"
#include "radio.h"
#include "bsp.h"
#include "si446x_api_lib.h"

//array for storing received radio data
volatile uint32_t gs_rdo_rpacket[RDO_RPACKET_FIFO_SIZE*RDO_ELEMENT_SIZE] = {0xffffffff};
//array for storing outgoing radio data
volatile uint32_t gs_rdo_tpacket[RDO_TPACKET_FIFO_SIZE*RDO_ELEMENT_SIZE] = {0xffffffff};

uint32_t radio_tx_pending[5*RDO_ELEMENT_SIZE] = {0};

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

//adds 1 packet to the outgoing queue
bool Queue_Message(uint8_t *msg){
	uint32_t wrtptr_tmp = wrptr_rdo_tpacket;
	uint32_t overflow=0;
	if (lock_obj){
		tries++;
		if (tries>10){
			lock_obj = false;
		}
		//a different call is already adding a message
		return false;
	}
	tries =0;
	lock_obj = true;
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
  int byte_cnt ; // calculated # bytes in a mav pkt
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
		byte_cnt = MAVLINK_HDR_LEN+((MavLinkPacket*)pkt)->length+MAVLINK_CHKSUM_LEN);
	fifo->write_pointer = wrtptr_tmp;
	fifo->lock_obj = false;
	fifo->byte_cnt += byte_cnt ;
	return true;
}

//attempts to retrieve a MavLink packet
//if successful, packet is copied to the location of the pkt pointer
bool Get_MavLink(MavLink_FIFO_Buffer *fifo, uint8_t *pkt){
  int byte_cnt ; // calculated # bytes in a mav pkt
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
 	// it is ok, just sent out actual pkt content
 	MavLinkPacket* pkt0 = (MavLinkPacket*)(fifo->buffer)+fifo->read_pointer;
	memcpy(pkt, pkt0, // reduce memcpy overhead
		byte_cnt = MAVLINK_HDR_LEN+pkt0->length+MAVLINK_CHKSUM_LEN);
	fifo->lock_obj = false;
	fifo->byte_cnt -= byte_cnt; // track current byte cnt in fifo obj
	assert(0<=fifo->byte_cnt);
	return true;
}
