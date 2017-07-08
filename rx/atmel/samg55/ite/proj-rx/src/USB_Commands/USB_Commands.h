/*
 * USB_Commands.h
 *
 */ 


#ifndef USB_COMMANDS_H_
#define USB_COMMANDS_H_





//Get_Property messages
#define RADIO_GET_PROPERTY_IDX				0x30
#define RADIO_GET_PROPERTY_HOST_LEN			3		//length of host query
#define RADIO_GET_PROPERTY_REPLY_IDX		0x31
#define RADIO_GET_PROPERTY_ATMEL_LEN		16		//length of atmel response

//maximum number of properties that can be retrieved with a single call is 16
#define MAX_4463_PROPS 16



uint8_t Si4463_Properties[MAX_4463_PROPS];



//Function Prototypes
void Si4463_GetProperty();
void USB_Send_Si4463_Props();





#endif /* USB_COMMANDS_H_ */