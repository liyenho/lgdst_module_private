/*
 * USB_Commands.h
 *
 */


#ifndef USB_COMMANDS_H_
#define USB_COMMANDS_H_


 #define BASE_GPS_IDX						0xe
 #define BASE_GPS_LEN						2*sizeof(float)

#define SET_FEC_IDX							0x10
#define SET_FEC_LEN							sizeof(uint8_t)


//Get_Property messages
#define RADIO_GET_PROPERTY_IDX				0x30
#define RADIO_GET_PROPERTY_HOST_LEN			3		//length of host query
#define RADIO_GET_PROPERTY_REPLY_IDX		0x31
#define RADIO_GET_PROPERTY_ATMEL_LEN		16		//length of atmel response
//maximum number of properties that can be retrieved with a single call is 16
#define MAX_4463_PROPS 16
//Get RSSI Reading
#define RADIO_GET_RSSI_IDX					0x32
//set radio power level
#define RADIO_SET_PWR_LVL_IDX				0x33
#define RADIO_SET_PWR_LVL_LEN				1




uint8_t Si4463_Properties[MAX_4463_PROPS];



//Function Prototypes
void Si4463_GetProperty();

void USB_Read_Base_GPS();

void USB_Send_Si4463_Props();

void USB_Host_Set_FEC();

void USB_Send_Latched_RSSI();

void USB_Set_Radio_Power();

  #include <string.h>
  #include "asf.h"
  #include <udd.h>
  #include "main.h"
  #include "Radio_Buffers.h"

  extern void si4463_radio_cb();
  extern uint32_t fifolvlcalc(uint32_t wrptr, uint32_t rdptr, uint32_t fifodepth);
  extern uint32_t wrptr_inc(uint32_t *wrptr,  uint32_t *rdptr, uint32_t fifodepth, int step);
  extern uint32_t rdptr_inc(uint32_t *wrptr,  uint32_t *rdptr, uint32_t fifodepth, int step);
  extern volatile bool ctrl_tdma_lock, ctrl_tdma_enable;
  extern unsigned char tpacket_grp[RADIO_GRPPKT_LEN],
  												rpacket_grp[RADIO_GRPPKT_LEN+RADIO_INFO_LEN];
  extern unsigned char gs_rdo_tpacket_ovflw;
  extern volatile uint32_t wrptr_rdo_tpacket, wrptr_rdo_rpacket,
  									rdptr_rdo_tpacket, rdptr_rdo_rpacket;
  extern unsigned int *gp_rdo_rpacket;
  extern volatile uint32_t gs_rdo_rpacket[RDO_RPACKET_FIFO_SIZE*RDO_ELEMENT_SIZE];

  // usb control command portal
  void usb_ctrl_cmd_portal(udd_ctrl_request_t *udd_g_ctrlreq);

#endif /* USB_COMMANDS_H_ */