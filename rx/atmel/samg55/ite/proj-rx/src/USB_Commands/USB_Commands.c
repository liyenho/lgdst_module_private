/*
 * USB_Commands.c
 * 
 */ 


#include <stdint.h>
#include <string.h>
#include <compiler.h>
#include "udd.h"
#include "si446x_cmd.h"
#include "compiler_defs.h"
#include "si446x_api_lib.h"
#include "USB_Commands.h"
#include "main.h"



 extern void si446x_get_property(uint8_t group, uint8_t num_props, uint8_t start_prop);


//Request property info from Si4463
void Si4463_GetProperty(){
	uint8_t group, num_props,start_prop = 0;
	group = *(uint8_t*)(gs_uc_hrbuffer);
	num_props = *((uint8_t*)(gs_uc_hrbuffer)+1);
	start_prop = *((uint8_t*)(gs_uc_hrbuffer)+2);
	//wipe memory
	memset(Si4463_Properties, 0x00, sizeof(Si4463_Properties));
	
	si446x_get_property(group, num_props, start_prop);
	memcpy(Si4463_Properties, &Si446xCmd.GET_PROPERTY.DATA, MAX_4463_PROPS);
	return;
}



void USB_Send_Si4463_Props(){
	memcpy(gs_uc_hrbuffer, Si4463_Properties, RADIO_GET_PROPERTY_ATMEL_LEN);
	//send data back to host
	udd_set_setup_payload(gs_uc_hrbuffer, RADIO_GET_PROPERTY_ATMEL_LEN);
}

