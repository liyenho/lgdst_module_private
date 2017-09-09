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
#include "Antenna_Diversity.h"
#include "radio.h"
#include "ctrl.h"

#include "ReedSolomon.h"

unsigned char rpacket_grp_partial[RADIO_GRPPKT_LEN];
unsigned char radio_mon_rcvjitter=0;
tRadioConfiguration *pRadioConfiguration_temp;

extern unsigned char tpacket_grp[RADIO_GRPPKT_LEN],
										rpacket_grp[RADIO_GRPPKT_LEN+RADIO_INFO_LEN];
extern volatile uint32_t gs_rdo_rpacket[RDO_RPACKET_FIFO_SIZE*RDO_ELEMENT_SIZE];
extern volatile capv_tune_t si4463_factory_tune;
extern volatile ctrl_radio_stats  r4463_sts;

extern void si446x_get_property(uint8_t group, uint8_t num_props, uint8_t start_prop);
extern void si446x_frr_a_read(U8 respByteCount );

void usb_ctrl_cmd_portal(udd_ctrl_request_t *udd_g_ctrlreq) {
	 // it should be safe to use wIndex alternatively instead pointer to interface index
		 if (RADIO_STARTUP_IDX == udd_g_ctrlreq->req.wIndex) {
			 // re-initialized by host dynamically
			#ifdef RADIO_CTRL_AUTO //disable host loading of ctrl configuration
			  udd_set_setup_payload(pRadioConfiguration_temp, sizeof(tRadioConfiguration));
			#else
			  udd_set_setup_payload(pRadioConfiguration, sizeof(tRadioConfiguration));
			  udd_g_ctrlreq->callback = si4463_radio_cb; // radio callback
			#endif
		 }
		if (RADIO_CAL_IDX == udd_g_ctrlreq->req.wIndex) {
			memset(&si4463_factory_tune, 0x0, sizeof(si4463_factory_tune));
			ctrl_tdma_enable = false; // turn off the si4463 tdd flag first
			si4463_factory_tune.calib_req_h = true; // turn on factory cap tuning request
		}
		if (RADIO_CAL_DONE_IDX == udd_g_ctrlreq->req.wIndex) {
			static uint16_t tune_done[2] ; /*sts flag and final cbv*/
			*(tune_done) = 0 != si4463_factory_tune.median;
			*(tune_done+1) = si4463_factory_tune.median;
			udd_set_setup_payload(&tune_done, sizeof(tune_done));
		}
		if (RADIO_HOPLESS_IDX == udd_g_ctrlreq->req.wIndex) {
			udd_set_setup_payload( (uint8_t *)gs_uc_hrbuffer, udd_g_ctrlreq->req.wLength);
			udd_g_ctrlreq->callback = si4463_radio_cb; // radio callback
		}
		 else if (RADIO_DATA_TX_IDX == udd_g_ctrlreq->req.wIndex) {
		   unsigned int wrptr_tmp1 = wrptr_rdo_tpacket;
		   unsigned int wrptr_tmp2;
		   unsigned int ovflag1,ovflag2;
		   ovflag1 = wrptr_inc(&wrptr_tmp1,&rdptr_rdo_tpacket,RDO_TPACKET_FIFO_SIZE, 1);
		   wrptr_tmp2=wrptr_tmp1;
		   ovflag2 = wrptr_inc(&wrptr_tmp2,&rdptr_rdo_tpacket,RDO_TPACKET_FIFO_SIZE, 1);
		   if(ovflag1 || ovflag2){
			   //overflow condition
			   gs_rdo_tpacket_ovflw++;
		   }
		   else{  // hardcoded again;-( I had to accommodate with such fixed packet length assumption vs user packet length, liyenho
			   udd_set_setup_payload( tpacket_grp, RADIO_GRPPKT_LEN); //stores payload in holding buffer
		   }
			udd_g_ctrlreq->callback = si4463_radio_cb; // radio callback
		 }
		 else if (RADIO_DATA_RX_IDX == udd_g_ctrlreq->req.wIndex) {
		  // byte[0]: no-packet: 0xee,  idle-packet: 0xe5, partial-packet:0xe2 tail-payload-only:0x73
		  // byte[1]: no-packet: 0xee,  idle_packet: 0xe5, partial-packet:0xe2 tail-payload-only:0x73
		  // byte[2]: no-packet: 0xee,  idle-packet: sender side send pkt count
		  // byte[3]: no-packet: 0xee,  idle-packet: sender side recv pkt count
		  // byte[4]: idle-packet: 0xe5
		  // byte[5]: idle-packet: 0xe5
		  // byte[lastPayload]:
		  // byte[U0]: user status field
		  //          bit0: usr payload valid flag
		  //          bit1: rf link lock
		  // byte[U1]: {rxlvl4bits, txlvl4bits}
		  // byte[U2]: Ctrl-rcv jitter delta (0.1ms)
		  // byte[U3]: not defined
		  unsigned char tdma_lock_char; //covert bool to uchar
		  uint8_t fifolvlr, fifolvlt, payldvalid_flag,filler_flag=0, usrvalid_flag;

		  if(ctrl_tdma_lock) tdma_lock_char=2; else tdma_lock_char=0;
		  fifolvlr=((uint8_t)fifolvlcalc(wrptr_rdo_rpacket,rdptr_rdo_rpacket, RDO_RPACKET_FIFO_SIZE));
		  fifolvlt=((uint8_t)fifolvlcalc(wrptr_rdo_tpacket,rdptr_rdo_tpacket, RDO_TPACKET_FIFO_SIZE));
		  if(rdptr_rdo_rpacket == wrptr_rdo_rpacket){
  			 //identify no data to send
  			 usrvalid_flag=0;
  			 memset(rpacket_grp, 0xee, RADIO_GRPPKT_LEN);
		  }
		  else{ //process next fifo element

			unsigned int rdptr_race; //tmp to prevent race condition
  			static bool gotpayloadhead=false;
			static uint8_t partial_packet[RADIO_PKT_LEN/2];
  			rdptr_race = rdptr_rdo_rpacket;
  			rdptr_inc(&wrptr_rdo_rpacket, &rdptr_race, RDO_RPACKET_FIFO_SIZE,1);
  			gp_rdo_rpacket = gs_rdo_rpacket + (RDO_ELEMENT_SIZE*rdptr_race);
  			payldvalid_flag=(((uint8_t *)gp_rdo_rpacket)[RADIO_PKT_LEN-1]>>7)&0x01;
  			if(ctrl_tdma_lock) tdma_lock_char=2; else tdma_lock_char=0;

			static bool first_half_received = false;
			uint8_t *msg_header = (uint8_t *)gp_rdo_rpacket;


			if (MSG_TYPE_HDR_HAS_FEC == (*msg_header & 0xF0)){
				//message has FEC - decode
				uint8_t decoded_pkt[16] ={0};
				Decode_Control_Packet(gp_rdo_rpacket,decoded_pkt);
				memcpy(gp_rdo_rpacket, decoded_pkt,16);
				memset(gp_rdo_rpacket+16,0x0, 16);
			}


				if (MSG_TYPE_HOST_GENERATED_A == (*msg_header & 0x0F)){
					//first half of a host message
					memcpy(partial_packet, msg_header+1, 15);
					first_half_received = true;
					memset(rpacket_grp, 0xee, RADIO_GRPPKT_LEN);
				usrvalid_flag = 0;
				}else if (MSG_TYPE_HOST_GENERATED_B == (*msg_header & 0x0F)){
					//second half of a host message received
					if (first_half_received){
						//copy first half of message to output buffer
						memcpy(rpacket_grp,partial_packet,15);
						//copy second half of message to output buffer
						memcpy(rpacket_grp+15,msg_header+1,15);
						//for debug info
						usrvalid_flag = 5;
						//reset first half of message flag
						first_half_received = false;
				}else{
						//received second half, but first half was dropped
					//memset(rpacket_grp, 0xd2, RADIO_GRPPKT_LEN);
					//usrvalid_flag = 0x15;
						memset(rpacket_grp, 0xee, RADIO_GRPPKT_LEN);
						usrvalid_flag = 0;
					first_half_received = false;
					}
			}else if (MSG_TYPE_HOST_GENERATED == (*msg_header&0x0F)){
				//entire host message, copy to output buffer, omitting header byte
				memcpy(rpacket_grp, msg_header+1,RADIO_GRPPKT_LEN);
				usrvalid_flag = 9;
				first_half_received = false;
			}
			else if (MSG_TYPE_BASE_GPS_INFO == (*msg_header & 0x0F)){
				//message contains base station GPS info
				//can be received either with or without FEC
				Extract_Base_GPS_from_Control_Message(msg_header);
				//memcpy(rpacket_grp, msg_header,RADIO_GRPPKT_LEN); //for debug
				memset(rpacket_grp, 0xee, RADIO_GRPPKT_LEN);
				usrvalid_flag = 0x00;
			}
			else if (MSG_TYPE_REQUEST_FEC_ON == (*msg_header & 0x0F)){
				Send_with_FEC = true;
				//transfer dummy data to host
				memset(rpacket_grp, 0xee, RADIO_GRPPKT_LEN);
				usrvalid_flag = 0;
				first_half_received = false;

			}
			else if (MSG_TYPE_REQUEST_FEC_OFF == (*msg_header & 0x0F)){
				Send_with_FEC = false;
				//transfer dummy data to host
				memset(rpacket_grp, 0xee, RADIO_GRPPKT_LEN);
				usrvalid_flag = 0;
				first_half_received = false;
			}
			else if (MSG_TYPE_IDLE == (*msg_header & 0x0F)){
				if (MSG_TYPE_HDR_HAS_FEC == (*msg_header & 0xF0)){
					memcpy(rpacket_grp, msg_header+1, 15);
					//idle message with FEC only has 15 bytes of data, fill in second half manually
					memset(rpacket_grp+15, 0xe5, 15);
				}else{
					memcpy(rpacket_grp, msg_header+1, RADIO_GRPPKT_LEN);
			}
				usrvalid_flag = 0;
			}
			else{
				//shouldn't get here right now, unrecognized code
				//memset(rpacket_grp, 0x6B, RADIO_GRPPKT_LEN);
				memcpy(rpacket_grp, msg_header+1,RADIO_GRPPKT_LEN);
				usrvalid_flag = 13;
				first_half_received = false;
			}

  			rdptr_rdo_rpacket = rdptr_race;
		  }

		  ((uint8_t *)rpacket_grp)[RADIO_GRPPKT_LEN]= usrvalid_flag | tdma_lock_char;
		  ((uint8_t *)rpacket_grp)[RADIO_GRPPKT_LEN+1]= (fifolvlr<<4) + fifolvlt;
		  ((uint8_t *)rpacket_grp)[RADIO_GRPPKT_LEN+2]= radio_mon_rcvjitter;
		  ((uint8_t *)rpacket_grp)[RADIO_GRPPKT_LEN+3]= FRR_Copy[0];
			//((uint8_t *)rpacket_grp)[RADIO_GRPPKT_LEN+3]= ((uint8_t *)gp_rdo_rpacket)[RADIO_PKT_LEN-1];
		  udd_set_setup_payload( (uint8_t *)rpacket_grp, RADIO_GRPPKT_LEN+RADIO_INFO_LEN);

		 }
		else if (RADIO_STATS_IDX == udd_g_ctrlreq->req.wIndex) {
			udd_set_setup_payload( (uint8_t *)&r4463_sts, RADIO_STATS_LEN);
		}
		else if (RADIO_MODEM_IDX == udd_g_ctrlreq->req.wIndex) {
			si446x_get_modem_status( 0xff );  // do not clear up pending status
			memcpy(gs_uc_hrbuffer, &Si446xCmd.GET_MODEM_STATUS, RADIO_MODEM_LEN);
			si446x_get_property(0x20, 1, 0x4e); // read current RSSI comp bias
			*(uint8_t*)gs_uc_hrbuffer = Si446xCmd.GET_PROPERTY.DATA[0];
			// be sure to overwrite the 1st entry in modem states buffer to carry RSSI comp bias value
			udd_set_setup_payload( (uint8_t *)gs_uc_hrbuffer, RADIO_MODEM_LEN);
		}
		else if (RADIO_CHSEL_IDX == udd_g_ctrlreq->req.wIndex) {
			if (RADIO_CHSEL_LEN0 != udd_g_ctrlreq->req.wLength &&
				RADIO_CHSEL_LEN1 != udd_g_ctrlreq->req.wLength
#if false // channel filter coefficients were removed, see radio.c for details
				&& RADIO_CHSEL_LEN2 != udd_g_ctrlreq->req.wLength
				&& RADIO_CHSEL_LEN3 != udd_g_ctrlreq->req.wLength
				&& RADIO_CHSEL_LEN4 != udd_g_ctrlreq->req.wLength
#endif
			)
				return (bool) -1; // error in msg len
			udd_set_setup_payload( (uint8_t *)gs_uc_hrbuffer, udd_g_ctrlreq->req.wLength);
			udd_g_ctrlreq->callback = si4463_radio_cb; // radio callback
		}
		else if (RADIO_TEMP_IDX == udd_g_ctrlreq->req.wIndex) {
		    extern volatile uint16_t temp1_intm;	// from si446x_nirq.c
		    *(int16_t*)gs_uc_hrbuffer = temp1_intm;
		    udd_set_setup_payload( (uint8_t *)gs_uc_hrbuffer, RADIO_TEMP_LEN);
		}
		else if (RADIO_CTUNE_IDX == udd_g_ctrlreq->req.wIndex) {
			udd_set_setup_payload( gs_uc_hrbuffer, RADIO_CTUNE_LEN);
			udd_g_ctrlreq->callback = si4463_radio_cb; // radio callback
		}
		else if (RADIO_PAIRID_IDX == udd_g_ctrlreq->req.wIndex) {
			if (HOP_ID_LEN != udd_g_ctrlreq->req.wLength)
				return -1;
			udd_set_setup_payload( gs_uc_hrbuffer, HOP_ID_LEN);
			udd_g_ctrlreq->callback = si4463_radio_cb; // radio callback
		}
		else if (RADIO_PAIR_LOCKED_IDX == udd_g_ctrlreq->req.wIndex) {
			*(uint32_t*)gs_uc_hrbuffer = ctrl_tdma_lock?1:0;
			udd_set_setup_payload( gs_uc_hrbuffer, RADIO_PAIR_LOCKED_LEN);
		}
		else if (DRONE_GPS_IDX == udd_g_ctrlreq->req.wIndex){
			//update drone GPS location
			udd_set_setup_payload((float *)gs_uc_hrbuffer, DRONE_GPS_LEN);
			udd_g_ctrlreq->callback = USB_Read_Drone_GPS; // radio callback
		}
		else if (DRONE_YAW_IDX== udd_g_ctrlreq->req.wIndex){
			//update drone yaw location
			udd_set_setup_payload((float *)gs_uc_hrbuffer, DRONE_YAW_LEN);
			udd_g_ctrlreq->callback = USB_Read_Drone_Yaw; // radio callback
		}
		else if (CAMERA_YAW_IDX == udd_g_ctrlreq->req.wIndex){
			//update camera yaw location
			udd_set_setup_payload((float *)gs_uc_hrbuffer, CAMERA_YAW_LEN);
			udd_g_ctrlreq->callback = USB_Read_Camera_Yaw; // radio callback
		}
		else if (RADIO_ANT_QUERY_IDX == udd_g_ctrlreq->req.wIndex){
			USB_Send_Active_Ant();
		}
		else if (RADIO_GET_PROPERTY_IDX == udd_g_ctrlreq->req.wIndex){
			//get property
			udd_set_setup_payload((uint8_t *)gs_uc_hrbuffer, RADIO_GET_PROPERTY_HOST_LEN);
			udd_g_ctrlreq->callback = Si4463_GetProperty; // radio callback
		}
		else if (RADIO_GET_PROPERTY_REPLY_IDX == udd_g_ctrlreq->req.wIndex){
			USB_Send_Si4463_Props();
		}
		else if (RADIO_GET_RSSI_IDX == udd_g_ctrlreq->req.wIndex){
			USB_Send_Latched_RSSI();
		}
		else if (RADIO_SET_PWR_LVL_IDX == udd_g_ctrlreq->req.wIndex){
			udd_set_setup_payload((uint8_t *)gs_uc_hrbuffer, RADIO_GET_PROPERTY_HOST_LEN);
			udd_g_ctrlreq->callback = USB_Set_Radio_Power;
		}
}

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


void USB_Read_Camera_Yaw(){
	Camera_Yaw = Degrees_To_Radians(*((float *)gs_uc_hrbuffer));
	//variable related to antenna selection may have changed, so update selection
	Select_Antenna_To_Broadcast();
	return;
}

void USB_Read_Drone_Yaw(){
	//update drone yaw
	Drone_Yaw = Degrees_To_Radians(*((float *)gs_uc_hrbuffer));
	//variable related to antenna selection may have changed, so update selection
	Select_Antenna_To_Broadcast();
	return;
}

void USB_Read_Drone_GPS(){
	//copy latitude and longitude from buffer to GPS struct
	Drone_Location.latitude = Degrees_To_Radians(*(float *)(gs_uc_hrbuffer));
	Drone_Location.longitude = Degrees_To_Radians(*((float *)gs_uc_hrbuffer+1));
	//variable related to antenna selection may have changed, so update selection
	Select_Antenna_To_Broadcast();
	return;
}

void USB_Send_Si4463_Props(){
	memcpy(gs_uc_hrbuffer, Si4463_Properties, RADIO_GET_PROPERTY_ATMEL_LEN);
	//send data back to host
	udd_set_setup_payload(gs_uc_hrbuffer, RADIO_GET_PROPERTY_ATMEL_LEN);
}

void USB_Send_Active_Ant(){
	//user asked which antenna is selected
	static uint16_t antenna = 0;
	antenna = Active_Antenna;
	udd_set_setup_payload(&antenna, sizeof(antenna));
}

void USB_Send_Latched_RSSI(){
	//si446x_frr_a_read(4);
	memcpy(gs_uc_hrbuffer, FRR_Copy, 4);
	udd_set_setup_payload(gs_uc_hrbuffer, 4);
}

void USB_Set_Radio_Power(){
	uint8_t power_level = *(uint8_t *)gs_uc_hrbuffer;
	const uint8_t group = 0x22;
	const uint8_t num_props = 1;
	const uint8_t start_prop = 1;

	si446x_set_property1(group, num_props, start_prop, power_level);
}
