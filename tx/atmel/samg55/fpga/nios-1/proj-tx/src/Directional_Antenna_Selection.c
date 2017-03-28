/*! @file YawAntennaSelection.c
 * @brief This file contains functions to select an antenna
 *
 * @b
 * @n
 * @n
 * @n
 */

/*		Assumed antenna positions
				mid-line
					* 0 deg
					*
			 _______*_______
			/	  front		\
			|				|
	Ant 1	|				| Ant 2
	~~~~~~~~|				|~~~~~~~~
	-90 deg	|				| +90 deg
			|	   rear		|
			\_______________/

*/

#include <math.h>

#include "Directional_Antenna_Selection.h"

//90 degrees in rad
#define	QUARTER_TURN						(M_PI/2.0)
#define LEFT_ANT_POS		   			   -QUARTER_TURN
#define RIGHT_ANT_POS						QUARTER_TURN


//rotation of the drone wrt North, in radians
//clockwise is positive to match compass headings
float Drone_Yaw = 0;
//rotation of the camera wrt the drone, in radians
//clockwise is positive to match compass headings
float Camera_Yaw=0;


enum Antenna Active_Antenna = LEFT;

//Calculates the forward azimuth from the drone to the base location (i.e. heading)
//Angle is in the earth frame (0 degrees is North, clockwise is positive)
float Calculate_Bearing(){
	float y = sin(Base_Location.longitude - Drone_Location.longitude)	\
			*cos(Base_Location.latitude);
	float x = cos(Drone_Location.latitude)*sin(Base_Location.latitude) - \
			sin(Drone_Location.latitude)*cos(Base_Location.latitude)*\
			cos(Base_Location.longitude-Drone_Location.longitude);

	return	atan2(y,x);
}

int Select_Antenna_To_Broadcast(){

	Base_Location.latitude = Degrees_To_Radians(32.833867);
	Base_Location.longitude = Degrees_To_Radians(-117.146074);
	//heading to base in camera frame
	static float angle_camera_base = 0;
	angle_camera_base = Calculate_Bearing() - Drone_Yaw - Camera_Yaw;


	//if angle is negative, add 2*PI until it is positive
	while(0>angle_camera_base){
		angle_camera_base+= 2.0*M_PI;
	}
	//if angle is greater than 2*PI, reduce
	while((2*M_PI)<angle_camera_base){
		angle_camera_base-= 2.0*M_PI;
	}

	if ( ((RIGHT_ANT_POS-QUARTER_TURN) < angle_camera_base) && \
				((RIGHT_ANT_POS+QUARTER_TURN) > angle_camera_base) ){
		//base station is on the right side of the camera's mid-line
		Active_Antenna = RIGHT;
	}else{
		Active_Antenna=LEFT;
	}

}

float Degrees_To_Radians(float degrees){
	return degrees*M_PI/180.0;
}

