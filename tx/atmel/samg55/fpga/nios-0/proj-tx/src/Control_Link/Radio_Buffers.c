/*
* Radio_Buffers.c
*
*/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "Radio_Buffers.h"
#include "ctrl.h"


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

bool Read_Message(){
	
}



uint32_t TX_overflow = 0; //for debug purposes
static bool lock_obj= false;

//adds 1 packet to the outgoing queue
bool Queue_Message(uint8_t *msg){
	unsigned int wrtptr_tmp = wrptr_rdo_tpacket;
	unsigned int overflow=0;
	if (lock_obj){
		//a different call is already adding a message
		return false;
	}
	
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
