/*
 * USB_Commands.c
 *
 * Created: 2/3/2017 1:57:55 PM
 *  Author: Drone-0
 */ 


#include <stdint.h>
#include <string.h>
#include <compiler.h>
#include "si446x_cmd.h"
#include "compiler_defs.h"
#include "si446x_api_lib.h"
#include "USB_Commands.h"



 extern void si446x_get_property(uint8_t group, uint8_t num_props, uint8_t start_prop);



void Si4463_GetProperty(uint8_t group, uint8_t num_props, uint8_t start_prop){
	//wipe memory
	memset(Si4463_Properties, 0x00, sizeof(Si4463_Properties));
	
	si446x_get_property(group, num_props, start_prop);
	memcpy(Si4463_Properties, &Si446xCmd.GET_PROPERTY.DATA, MAX_4463_PROPS);
	
	return;
}