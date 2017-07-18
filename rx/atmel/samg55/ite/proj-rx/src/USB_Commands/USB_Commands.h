/*
 * USB_Commands.h
 *
 */


#ifndef USB_COMMANDS_H_
#define USB_COMMANDS_H_

  #include <udd.h>

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


//Get_Property messages
#define RADIO_GET_PROPERTY_IDX				0x30
#define RADIO_GET_PROPERTY_HOST_LEN			3		//length of host query
#define RADIO_GET_PROPERTY_REPLY_IDX		0x31
#define RADIO_GET_PROPERTY_ATMEL_LEN		16		//length of atmel response

//maximum number of properties that can be retrieved with a single call is 16
#define MAX_4463_PROPS 16



uint8_t Si4463_Properties[MAX_4463_PROPS];



//Function Prototypes
void Si4463_GetProperty();
void USB_Send_Si4463_Props();





#endif /* USB_COMMANDS_H_ */