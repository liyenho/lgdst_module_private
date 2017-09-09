/*
 * GPS.h
 *
 * Created: 10/28/2016 5:20:41 PM
 *  Author: Drone-0
 */ 


#ifndef GPS_H_
#define GPS_H_


struct GPSInfo{
	float latitude;
	float longitude;
};

struct GPSInfo Base_Location;

float Degrees_To_Radians(float degrees);
void Encode_Base_GPS_into_Control_Message(uint8_t *msg_start);


#endif /* GPS_H_ */