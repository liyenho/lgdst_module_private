/*! @file Antenna_Diversity.c
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
#include <stdint.h>
#include <string.h>
#include "compiler.h"

#include "Antenna_Diversity.h"
#include "radio_config.h"


//90 degrees in rad
#define	QUARTER_TURN						(M_PI/2.0)
#define LEFT_ANT_POS		   			   -QUARTER_TURN
#define RIGHT_ANT_POS						QUARTER_TURN

extern void si446x_gpio_pin_cfg(U8 GPIO0, U8 GPIO1, U8 GPIO2, U8 GPIO3, U8 NIRQ, U8 SDO, U8 GEN_CONFIG);

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

#define GPIO_CONFIG_PROP_GRP  
int Select_Antenna_To_Broadcast(){

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
	
	Set_Antenna(Active_Antenna);
	
	return 0;
}

//tell Si4463 which antenna to use
void Set_Antenna(enum Antenna ant){
	return; //disabled for now, Si4463 is controlling antenna selection
	//uint8_t GPIO3,GPIO4;
	//GPIO3 = 0x20 + (LEFT == ant);
	//GPIO4 = 0x20 + (RIGHT == ant);
	//si446x_gpio_pin_cfg(Si4463_GPIO0, Si4463_GPIO1, GPIO3, GPIO4, 0, 0, 0);
}



float Degrees_To_Radians(float degrees){
	return degrees*M_PI/180.0;
}

void Encode_Base_GPS_into_Control_Message(uint8_t *msg_start){
	memcpy(msg_start+1, &Base_Location.latitude, sizeof(float));
	memcpy(msg_start+1+sizeof(float), &Base_Location.longitude, sizeof(float));
}

void Extract_Base_GPS_from_Control_Message(uint8_t *msg_start){
	Base_Location.latitude = *(float *)(msg_start+1);
	Base_Location.longitude = *((float*)(msg_start+1)+1);
	Select_Antenna_To_Broadcast(); //calculate which antenna should be used
}

