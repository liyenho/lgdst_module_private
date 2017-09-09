/*
 * USB_Commands.c
 *
 */


#include <stdint.h>
#include <string.h>
#include <compiler.h>
#include "udd.h"
#include "USB_Commands.h"
#include "main.h"
#include "ctrl.h"
#include "Antenna_Diversity.h"
#include "taisync.h"

#include "ReedSolomon.h"

#if RECEIVE_MAVLINK
	unsigned char rpacket_grp[MAVLINK_USB_TRANSFER_LEN]; //make big just because
#else
	unsigned char rpacket_grp[RADIO_GRPPKT_LEN+RADIO_INFO_LEN];
#endif

#if SEND_MAVLINK
	unsigned char tpacket_grp[MAVLINK_USB_TRANSFER_LEN];
#else
	unsigned char tpacket_grp[RADIO_GRPPKT_LEN];
#endif
 extern volatile uint32_t gs_rdo_rpacket[RDO_RPACKET_FIFO_SIZE*RDO_ELEMENT_SIZE];

extern volatile ctrl_radio_stats ctrl_sts;
extern volatile bool ctrl_radio_started, ctrl_tdma_enable;
extern uint32_t hop_watchdog_reset_cnt;

unsigned char radio_mon_rcvjitter=0;
 volatile uint8_t dbg_ctrlcnt;

#ifdef  RADIO_TAISYNC
void ctrl_radio_cb(void) {
	 if (udd_g_ctrlreq.payload_size < udd_g_ctrlreq.req.wLength)
			return; // invalid call
	 else if (RADIO_STARTUP_IDX == udd_g_ctrlreq.req.wIndex) {
		/* init radio stats obj */
			for (int j=0; j<CTRL_CTX_LEN; j++) {
				ctrl_sts.ctrl_bits_ctx[j] = LONG_RNG;
			}
			ctrl_sts.bw_ctrl_bits = LONG_RNG;
			ctrl_sts.errPerAcc =0;
		 vRadio_Init();
		 // frequency hopping in the action, initialization prior to hop operation because Rx side is currently the slave
		 ctrl_hop_global_update(true);
		ctrl_radio_started = true;
		ctrl_tdma_enable = true;
	}
	else if (RADIO_DATA_TX_IDX == udd_g_ctrlreq.req.wIndex) {
		Queue_Control_Packet(tpacket_grp);
	}
	else if (RADIO_PAIRID_IDX == udd_g_ctrlreq.req.wIndex) {
		uint8_t* pv = (U8*)gs_uc_hrbuffer;
		Set_Pair_ID(pv);
	}
}
#endif
void usb_ctrl_cmd_portal(udd_ctrl_request_t *udd_g_ctrlreq) {
		 // it should be safe to use wIndex alternatively instead pointer to interface index
		 if (RADIO_STARTUP_IDX == udd_g_ctrlreq->req.wIndex) {
			  udd_g_ctrlreq->callback = ctrl_radio_cb; // radio callback
		 }
		else if (RADIO_DATA_TX_IDX == udd_g_ctrlreq->req.wIndex) {
			   udd_set_setup_payload( tpacket_grp, RADIO_GRPPKT_LEN); //stores payload in holding buffer
		   udd_g_ctrlreq->callback = ctrl_radio_cb; // radio callback
		}
		else if (RADIO_DATA_RX_IDX == udd_g_ctrlreq->req.wIndex) {
				Transfer_Control_Data_Out();
		 }
		else if (RADIO_STATS_IDX == udd_g_ctrlreq->req.wIndex) {
			udd_set_setup_payload( (uint8_t *)&ctrl_sts, RADIO_STATS_LEN);
		}
		else if (RADIO_PAIRID_IDX == udd_g_ctrlreq->req.wIndex) {
			if (HOP_ID_LEN != udd_g_ctrlreq->req.wLength)
				return -1;
			udd_set_setup_payload( gs_uc_hrbuffer, HOP_ID_LEN);
			udd_g_ctrlreq->callback = ctrl_radio_cb; // radio callback
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
		else if (RADIO_GET_RSSI_IDX == udd_g_ctrlreq->req.wIndex){
			USB_Send_Latched_RSSI();
		}
		else if (RADIO_SET_PWR_LVL_IDX == udd_g_ctrlreq->req.wIndex){
			udd_set_setup_payload((uint8_t *)gs_uc_hrbuffer, RADIO_GET_PROPERTY_HOST_LEN);
			udd_g_ctrlreq->callback = USB_Set_Radio_Power;
		}
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

void USB_Send_Latched_RSSI(void){
	uint8_t rssi;
	get_radio_rssi(&rssi);
	memcpy(gs_uc_hrbuffer, &rssi, 1);
}

void USB_Set_Radio_Power(void){
	uint8_t pwr = *(uint8_t *)gs_uc_hrbuffer;
	if (MAX_RDO_PWR<pwr)
		return;  // invalid param
	set_radio_power((uint32_t)pwr);
}

#if RECEIVE_MAVLINK
//ToDo: This needs to change if MavLink packets transfered out can be variable length!!!
void Transfer_Control_Data_Out(void){
	MavLinkPacket pkt = {0};
	if (Get_MavLink(&incoming_messages, &pkt)){
		memcpy(rpacket_grp, &pkt, sizeof(pkt));
	}else{
		//no MavLink packet available, use filler
		memset(rpacket_grp, 0xee, MAVLINK_USB_TRANSFER_LEN);
	}

	udd_set_setup_payload( (uint8_t *)rpacket_grp, MAVLINK_USB_TRANSFER_LEN);
	return;
}

#else
void Transfer_Control_Data_Out(void){
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
		  unsigned char tdma_lock_char, rssi_taisync; //covert bool to uchar
			uint8_t fifolvlr, fifolvlt, payldvalid_flag, usrvalid_flag=0;

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
				memcpy(rpacket_grp, msg_header,RADIO_GRPPKT_LEN);
				usrvalid_flag = 12;
				first_half_received = false;
			}

			rdptr_rdo_rpacket = rdptr_race;
	}
							                                //msgType byte from RX board
	#ifdef DEBUG_ANT_SWITCH
	///////////////////////////////////////////////////////////////////////////////////////////////////////
	// video antenna selection manual mode

	if((usrvalid_flag&0x01)==0x01)
	if((rpacket_grp[16]==0xff)||(rpacket_grp[16]==0xB5)) //0xB5 is ubuntu test ctrl message
	{
		//default antenna position
		pio_set(PIOA, PIO_PA23);
		pio_clear(PIOA, PIO_PA24);
		dbg_antpos = 4;
	}
	else
	{
		//alternative video antenna position
		pio_clear(PIOA, PIO_PA23);
		pio_set(PIOA, PIO_PA24);
		dbg_antpos = 8;
	}
	rpacket_grp[16]=0xff;
	rpacket_grp[17]=0xff;
	rpacket_grp[18]=0xff;
	///////////////////////////////////////////////////////////////////////////////////////////////////////
	#endif
	///////////////////////////////////////////////////////////////////////////////////////////////////////
	// control data counter monitoring  YH:test22
	if((usrvalid_flag & 0x01)==0x01)
	dbg_ctrlcnt++;
	///////////////////////////////////////////////////////////////////////////////////////////////////////


		  ((uint8_t *)rpacket_grp)[RADIO_GRPPKT_LEN]= usrvalid_flag | tdma_lock_char;
		  ((uint8_t *)rpacket_grp)[RADIO_GRPPKT_LEN+1]= hop_watchdog_reset_cnt;
		  ((uint8_t *)rpacket_grp)[RADIO_GRPPKT_LEN+2]= 0; //radio_mon_rcvjitter; not used...
			get_radio_rssi(&rssi_taisync) ;
		  ((uint8_t *)rpacket_grp)[RADIO_GRPPKT_LEN+3]= rssi_taisync; /*FRR_Copy[0]; latched rssi:[0], cur_state:[1]*/

		  udd_set_setup_payload( (uint8_t *)rpacket_grp, RADIO_GRPPKT_LEN+RADIO_INFO_LEN);
}
#endif  //RECEIVE_MAVLINK
