/*
 * Radio_Buffers.h
*
 */ 


#ifndef RADIO_BUFFERS_H_
#define RADIO_BUFFERS_H_

#include "ctrl.h"
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


//function declarations
uint32_t fifolvlcalc(uint32_t wrptr, uint32_t rdptr, uint32_t fifodepth);
uint32_t wrptr_inc(uint32_t *wrptr,  uint32_t *rdptr, uint32_t fifodepth, int step);
uint32_t rdptr_inc(uint32_t *wrptr,  uint32_t *rdptr, uint32_t fifodepth, int step);
bool Queue_Message(uint8_t *msg);

#endif /* RADIO_BUFFERS_H_ */