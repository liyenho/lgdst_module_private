/*
 * USB_Commands.h
 *
 * Created: 2/2/2017 6:25:53 PM
 *  Author: Drone-0
 */


#ifndef USB_COMMANDS_H_
#define USB_COMMANDS_H_

  #include <udd.h>
  
  extern void si4463_radio_cb();
  extern uint32_t fifolvlcalc(uint32_t wrptr, uint32_t rdptr, uint32_t fifodepth);
  extern uint32_t wrptr_inc(uint32_t *wrptr,  uint32_t *rdptr, uint32_t fifodepth, int step);
  extern uint32_t rdptr_inc(uint32_t *wrptr,  uint32_t *rdptr, uint32_t fifodepth, int step);
  extern volatile bool ctrl_tdma_lock, ctrl_tdma_enable;
  extern unsigned char gs_rdo_tpacket_ovflw;
  extern volatile uint32_t wrptr_rdo_tpacket, wrptr_rdo_rpacket,
  									rdptr_rdo_tpacket, rdptr_rdo_rpacket;
  extern unsigned int *gp_rdo_rpacket;
  extern uint32_t gs_uc_hrbuffer[(USB_HOST_MSG_LEN+HOST_BUFFER_SIZE+3)/sizeof(int)];

  // usb control command portal
  void usb_ctrl_cmd_portal(udd_ctrl_request_t *udd_g_ctrlreq);

//directional antenna selection
#define DRONE_GPS_IDX								0x10
#define DRONE_GPS_LEN								2*sizeof(float) //size in bytes
#define DRONE_YAW_IDX								0x11
#define DRONE_YAW_LEN								sizeof(float)//size in bytes
#define CAMERA_YAW_IDX							0x12
#define CAMERA_YAW_LEN							sizeof(float)//size in bytes
#define RADIO_ANT_QUERY_IDX						0x13 //query which antenna is selected by yaw algo

//Get_Property messages
#define RADIO_GET_PROPERTY_IDX				0x30
#define RADIO_GET_PROPERTY_HOST_LEN			3		//length of host query
#define RADIO_GET_PROPERTY_REPLY_IDX		0x31
#define RADIO_GET_PROPERTY_ATMEL_LEN		16		//length of atmel response

//maximum number of properties that can be retrieved with a single call is 16
#define MAX_4463_PROPS 16

void Si4463_GetProperty(uint8_t group, uint8_t num_props, uint8_t start_prop);




uint8_t Si4463_Properties[MAX_4463_PROPS];


#endif /* USB_COMMANDS_H_ */