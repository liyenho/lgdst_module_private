/*
 * ctrl.h
 *
 */


#ifndef CTRL_H_
#define CTRL_H_

#define ASYMM_RATIO								/*1*/ 2

#define SEND_MAVLINK					1 /*0*/		//data to send to Drone is in MavLink form
#define  RECEIVE_MAVLINK				1 /*0*/	//data received from Drone is in MavLink form

#if RECEIVE_MAVLINK
	#define RADIO_PKT_LEN							40 //split MavLink into 40 byte chunks
#else
	#define RADIO_PKT_LEN							32 // ctl/sts radio payload byte length
#endif
#define RADIO_GRPPKT_LEN						30
#define RDO_ELEMENT_SIZE   			(RADIO_PKT_LEN/sizeof(uint32_t))	// RADIO_PKT_LEN must divide into sizeof(uint32_t), liyenho

#define RADIO_LONG_PKT_LEN				(32*3)//(RADIO_PKT_LEN*1) //long transmissions for 868 MHz

#define TICKS_IN_MS			120000 //120000=1ms
#define BITRATE					40000

#define MilliSec_To_Tick(X) ((X)*TICKS_IN_MS)

#define REAL_PKT_LEN				(RADIO_PKT_LEN+13+2+2+4) //payload + preamble + sync word + length field
#define EURO_TDMA_PERIOD			MilliSec_To_Tick(100)//MilliSec_To_Tick(40+3*(1000*(13+2+32*3+2)*8/10000))

#define US_TDMA_PERIOD    				MilliSec_To_Tick(15+3*(1000*REAL_PKT_LEN*8/40000))		// see TX ctrl.h for details
#define  TDMA_BOUND								 /*5400000*/ /*4860000*/ (12*4320000) // 100 ms = 12000000

#define TDMA_PERIOD			(USE_915MHZ?US_TDMA_PERIOD:EURO_TDMA_PERIOD)  //macro to select between 915MHz and 869 MHz

 #define HOP_ID_LEN											10
//upper nibble of 1st byte encodes whether the message has FEC or not
#define MSG_TYPE_HDR_HAS_FEC			(0xF<<4)
#define MSG_TYPE_HDR_NO_FEC				(0x0<<4)
//lower nibble of 1st byte determines message type
#define MSG_TYPE_BASE_GPS_INFO			0x01	//contains base station gps
#define MSG_TYPE_SWITCH_VID_ANT			0x02	//used to switch Tx video antenna
#define MSG_TYPE_REQUEST_FEC_ON			0x03
#define MSG_TYPE_REQUEST_FEC_OFF		0x04

#define MSG_TYPE_HOST_GENERATED			0x06	//message passed in by host - just pass through
#define MSG_TYPE_HOST_GENERATED_A		0x07
#define MSG_TYPE_HOST_GENERATED_B		0x08
#define MSG_TYPE_IDLE					0x09
#define MSG_TYPE_SET_CHANNEL			0x0A	//used to set the radio channel

#define FEC_ON_RSSI_THRESHOLD		0x10  //RSSI reading at which to request FEC On
#define FEC_OFF_RSSI_THRESHOLD		(FEC_ON_RSSI_THRESHOLD+10)   //RSSI reading at which to request FEC Off
//approx 5 db above ON threshold

#define MAVLINK_USB_TRANSFER_LEN		263//(30+MAVLINK_HDR_LEN+MAVLINK_CHKSUM_LEN)

//keep track of has a request has been sent to other side to turn FEC on
extern bool Requested_FEC_On;

enum FEC_Options {
	OFF = 0x00, //Host has asserted control
	ON = 0x0F,	//Host has asserted control
	AUTO = 0xFF	//Atmel has control
} ;
extern enum FEC_Options FEC_Option;
extern bool Send_with_FEC; //turn on to send messages with FEC
extern bool send_long_packet;
bool USE_915MHZ;
uint8_t control_channel;
bool control_channge_change_ack;


extern volatile bool ctrl_tdma_enable;
extern enum pair_mode hop_state;
int fhop_offset, fhop_base;
uint8_t hop_id[HOP_ID_LEN];
int fhopless;
int fhop_idx;
unsigned char snd_asymm_cnt;
unsigned int  radio_mon_txcnt;


uint32_t Control_Outbound_Queue_Available_Slots(void);
bool Queue_Control_Idle_Packet(void);
bool Queue_Control_Packet(uint8_t * pending_msg);
bool Queue_FEC_Request_On_Packet(void);
bool Queue_FEC_Request_Off_Packet(void);
bool Queue_Msg_Vid_Ant_Switch(void);
bool Queue_Base_GPS_Packet(void);
bool Queue_Channel_Change_Message(void);
void Set_Pair_ID(uint8_t *ID);
int hop_chn_sel(int offset);
void Control_Send_Event(void);
bool Queue_MavLink_from_USB(uint8_t *pkt_start);
bool Queue_Idle_Mavlink(void);
#endif /* CTRL_H_ */