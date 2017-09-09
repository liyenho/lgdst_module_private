/*
 * GPS.c
 *
 * Created: 10/31/2016 12:36:37 PM
 *  Author: Drone-0
 */ 

#include <math.h>
#include <stdint.h>
#include <string.h>
#include "GPS.h"


float Degrees_To_Radians(float degrees){
	return degrees*M_PI/180.0;
}

//copies GPS information into message, starting at the second byte
//first byte is assumed to be header information
void Encode_Base_GPS_into_Control_Message(uint8_t *msg_start){
	memcpy(msg_start+1, &Base_Location.latitude, sizeof(float));
	memcpy(msg_start+1+sizeof(float), &Base_Location.longitude, sizeof(float));
}