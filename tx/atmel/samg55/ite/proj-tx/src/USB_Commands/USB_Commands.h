/*
 * USB_Commands.h
 *
 * Created: 2/2/2017 6:25:53 PM
 *  Author: Drone-0
 */ 


#ifndef USB_COMMANDS_H_
#define USB_COMMANDS_H_




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