/*
 * Radio_Buffers.h
*
 */ 


#ifndef RADIO_BUFFERS_H_
#define RADIO_BUFFERS_H_

#include "ctrl.h"
#include "MavLink.h"
//number of radio packets that can be held in memory
#define RDO_TPACKET_FIFO_SIZE               25
#define RDO_RPACKET_FIFO_SIZE               25

//outgoing buffer
extern volatile uint32_t gs_rdo_tpacket[RDO_TPACKET_FIFO_SIZE*RDO_ELEMENT_SIZE];
extern unsigned int *gp_rdo_tpacket;
extern unsigned int *gp_rdo_tpacket_l;
extern volatile uint32_t rdptr_rdo_tpacket;
extern volatile uint32_t wrptr_rdo_tpacket;
//received buffer
extern volatile uint32_t gs_rdo_rpacket[RDO_RPACKET_FIFO_SIZE*RDO_ELEMENT_SIZE];
extern unsigned int *gp_rdo_rpacket;
extern unsigned int *gp_rdo_rpacket_l;
extern volatile uint32_t rdptr_rdo_rpacket;
extern volatile uint32_t wrptr_rdo_rpacket;

extern uint8_t dummy_data[3*RADIO_LONG_PKT_LEN];

#define MavLinkBufferSize			10

typedef struct{
	
	uint32_t read_pointer;
	uint32_t write_pointer;
	MavLinkPacket buffer[MavLinkBufferSize];
	bool lock_obj;
	uint32_t lock_cnt;
	uint32_t overflow_cnt;
	
}MavLink_FIFO_Buffer;

MavLink_FIFO_Buffer outgoing_messages;
MavLink_FIFO_Buffer incoming_messages;

//function declarations
uint32_t fifolvlcalc(uint32_t wrptr, uint32_t rdptr, uint32_t fifodepth);
uint32_t wrptr_inc(uint32_t *wrptr,  uint32_t *rdptr, uint32_t fifodepth, int step);
uint32_t rdptr_inc(uint32_t *wrptr,  uint32_t *rdptr, uint32_t fifodepth, int step);
bool Queue_Message(uint8_t *msg);
bool QueueMavLink_Send(MavLinkPacket Mav);
bool QueueMavLink_Recv(MavLinkPacket Mav);
bool Handle_TX_FIFO_Low(void);
bool Queue_MavLink(MavLink_FIFO_Buffer *fifo, MavLinkPacket *pkt);
bool Get_MavLink(MavLink_FIFO_Buffer *fifo, MavLinkPacket *pkt);

#endif /* RADIO_BUFFERS_H_ */