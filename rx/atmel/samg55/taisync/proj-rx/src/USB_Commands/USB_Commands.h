/*
 * USB_Commands.h
 *
 */


#ifndef USB_COMMANDS_H_
#define USB_COMMANDS_H_

  #include "udd.h"
  #include "ctrl.h"
  #include "Radio_Buffers.h"

  extern volatile bool ctrl_tdma_lock;
  extern uint8_t channel_scan_selection;

#if RECEIVE_MAVLINK
	extern unsigned char rpacket_grp[MAVLINK_USB_TRANSFER_LEN]; //make big just because
#else
	extern unsigned char rpacket_grp[RADIO_GRPPKT_LEN+RADIO_INFO_LEN];
#endif

#if SEND_MAVLINK
	extern unsigned char tpacket_grp[MAVLINK_USB_TRANSFER_LEN];
#else
	extern unsigned char tpacket_grp[RADIO_GRPPKT_LEN];
#endif

#ifdef DEBUG_RADIOSTATUS
	extern volatile uint8_t dbg_ctrlvidbuff[10];
#endif
  // usb control command portal
  void usb_ctrl_cmd_portal(udd_ctrl_request_t *udd_g_ctrlreq);

 #define BASE_GPS_IDX						0xe
 #define BASE_GPS_LEN						2*sizeof(float)

#define SET_FEC_IDX							0x10
#define SET_FEC_LEN							sizeof(uint8_t)

//Get_Property messages
#define RADIO_GET_PROPERTY_HOST_LEN			3		//length of host query
#define RADIO_GET_PROPERTY_ATMEL_LEN		16		//length of atmel response

//Get RSSI Reading
#define RADIO_GET_RSSI_IDX					0x32
//set radio power level
#define RADIO_SET_PWR_LVL_IDX				0x33
#define RADIO_SET_PWR_LVL_LEN				1



//Function Prototypes
void USB_Read_Base_GPS(void);

void USB_Host_Set_FEC(void);

void USB_Send_Latched_RSSI(void);

void USB_Set_Radio_Power(void);

void Transfer_Control_Data_Out(void);

#endif /* USB_COMMANDS_H_ */