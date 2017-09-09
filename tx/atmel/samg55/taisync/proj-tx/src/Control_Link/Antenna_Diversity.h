/*
 * Antenna_Diversity.h
 *
 */ 


#ifndef ANTENNA_DIVERSITY_H_
#define ANTENNA_DIVERSITY_H_

struct GPSInfo{
	float latitude;
	float longitude;
	};

//contains latitude and longitude of Base, in radians
struct GPSInfo Base_Location;
//contains latitude and longitude of Drone, in radians
struct GPSInfo Drone_Location;

	
typedef enum Antenna{
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

void Set_Antenna(enum Antenna ant);

float Calculate_Bearing(void);

int Select_Antenna_To_Broadcast(void);

float Degrees_To_Radians(float degrees);

void Extract_Base_GPS_from_Control_Message(uint8_t *msg_start);
void Encode_Base_GPS_into_Control_Message(uint8_t *msg_start);


#endif /* ANTENNA_DIVERSITY_H_ */