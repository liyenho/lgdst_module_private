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

#define MavLinkBufferSize			8 //number of MavLink structs to store, for efficiency
#define MavLinkPacketSize			sizeof(MavLinkPacket)

typedef struct{

	uint32_t read_pointer;
	uint32_t write_pointer;
	uint8_t buffer[ MavLinkBufferSize * MavLinkPacketSize];
	bool lock_obj;
	uint32_t lock_cnt;
	uint32_t overflow_cnt;

}MavLink_FIFO_Buffer;


volatile MavLink_FIFO_Buffer outgoing_messages;
volatile MavLink_FIFO_Buffer incoming_messages;

#define UART_BUFFER_SIZE 128  // make it 2's power for efficiency,
#define MAVLINK_BYTESTREAM_DEPTH		16*UART_BUFFER_SIZE  // make it 2's power for efficiency,

typedef struct{

	uint32_t read_pointer;
	uint32_t write_pointer;
	uint8_t data[MAVLINK_BYTESTREAM_DEPTH];

}MavLink_Bytestream;

volatile MavLink_Bytestream incoming_MavLink_Data;
volatile MavLink_Bytestream outgoing_MavLink_Data;



//function declarations
uint32_t fifolvlcalc(uint32_t wrptr, uint32_t rdptr, uint32_t fifodepth);
uint32_t wrptr_inc(uint32_t *wrptr,  uint32_t *rdptr, uint32_t fifodepth, int step);
uint32_t rdptr_inc(uint32_t *wrptr,  uint32_t *rdptr, uint32_t fifodepth, int step);
bool Queue_Message(uint8_t *msg);
bool Queue_MavLink(MavLink_FIFO_Buffer *fifo, uint8_t *pkt);
bool Get_MavLink(MavLink_FIFO_Buffer *fifo, uint8_t *pkt);
bool Queue_MavLink_Raw_Data(MavLink_Bytestream *stream, uint32_t num_bytes, uint8_t * bytes);
void Process_MavLink_Raw_Data(void);
void Process_MavLink_Raw_Radio_Data(void);
bool Build_MavLink_from_Byte_Stream(uint32_t*, MavLink_FIFO_Buffer*, MavLink_Bytestream*);
#endif /* RADIO_BUFFERS_H_ */