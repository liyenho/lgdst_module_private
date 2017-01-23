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

struct GPSInfo BaseLocation;

float Degrees_To_Radians(float degrees);



#endif /* GPS_H_ */