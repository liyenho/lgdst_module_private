/*
 * USB_Commands.h
 *
 */


#ifndef USB_COMMANDS_H_
#define USB_COMMANDS_H_

  #include "udd.h"
  #include "Radio_Buffers.h"

  extern void si4463_radio_cb(void);
  extern volatile bool ctrl_tdma_lock;
  extern unsigned char tpacket_grp[RADIO_GRPPKT_LEN],
  												rpacket_grp[RADIO_GRPPKT_LEN+RADIO_INFO_LEN];
  extern unsigned char gs_rdo_tpacket_ovflw;
  extern volatile int vid_ant_switch ;

#ifdef DEBUG_RADIOSTATUS
	extern volatile uint8_t dbg_ctrlvidbuff[10];
#endif
  // usb control command portal
  void usb_ctrl_cmd_portal(udd_ctrl_request_t *udd_g_ctrlreq);

//directional antenna selection
#define DRONE_GPS_IDX								0x10
#define DRONE_GPS_LEN								2*sizeof(float) //size in bytes
#define DRONE_YAW_IDX								0x11
#define DRONE_YAW_LEN								sizeof(float)//size in bytes
#define CAMERA_YAW_IDX								0x12
#define CAMERA_YAW_LEN								sizeof(float)//size in bytes
#define RADIO_ANT_QUERY_IDX							0x13 //query which antenna is selected by yaw algo

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

#define RADIO_SET_CHANNEL_IDX				0x35
#define RADIO_SET_CHANNEL_LEN				1

uint8_t Si4463_Properties[MAX_4463_PROPS];



//Function Prototypes
void Si4463_GetProperty(void);
void USB_Read_Camera_Yaw(void);
void USB_Read_Drone_Yaw(void);
void USB_Read_Drone_GPS(void);
void USB_Send_Si4463_Props(void);
void USB_Send_Active_Ant(void);
void USB_Send_Latched_RSSI(void);
void USB_Set_Radio_Power(void);
void USB_Set_Radio_Channel(void);



#endif /* USB_COMMANDS_H_ */