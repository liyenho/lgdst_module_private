/*
 * Directional_Antenna_Selection.h
 *
 * Created: 10/25/2016 5:44:06 PM
 *  Author: Drone-0
 */ 


#ifndef DIRECTIONAL_ANTENNA_SELECTION_H_
#define DIRECTIONAL_ANTENNA_SELECTION_H_

struct GPSInfo{
	float latitude;
	float longitude;
	};

//contains latitude and longitude of Base, in radians
struct GPSInfo Base_Location;
//contains latitude and longitude of Drone, in radians
struct GPSInfo Drone_Location;

	
enum Antenna{
	LEFT = 0,
	RIGHT = 1
};


//yaw of drone wrt North
float Drone_Yaw;
//yaw of camera wrt drone
float Camera_Yaw;

enum Antenna Active_Antenna;

/*
/Function Prototypes
*/

float Calculate_Bearing(void);

int Select_Antenna_To_Broadcast(void);

float Degrees_To_Radians(float degrees);


#endif /* DIRECTIONAL_ANTENNA_SELECTION_H_ */