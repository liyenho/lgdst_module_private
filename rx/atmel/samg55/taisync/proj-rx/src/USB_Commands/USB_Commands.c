/*
 * USB_Commands.c
 *
 */


#include <stdint.h>
#include <string.h>
#include <compiler.h>
#include "USB_Commands.h"
#include "main.h"
#include "ctrl.h"
#include "GPS.h"
#include "taisync.h"

#include "ReedSolomon.h"
#include "taisync.h"

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

extern volatile ctrl_radio_stats  ctrl_sts;
extern volatile bool ctrl_radio_started, ctrl_tdma_enable;

#ifdef  RADIO_TAISYNC
/*static*/ void ctrl_radio_cb() {
	if (udd_g_ctrlreq.payload_size < udd_g_ctrlreq.req.wLength)
	return; // invalid call
	else if (RADIO_STARTUP_IDX == udd_g_ctrlreq.req.wIndex) {
		/* init radio stats obj */
		{
			for (int j=0; j<CTRL_CTX_LEN; j++) {
				ctrl_sts.ctrl_bits_ctx[j] = LONG_RNG;
			}
			ctrl_sts.bw_ctrl_bits = LONG_RNG;
			ctrl_sts.errPerAcc =0;
			ctrl_sts.loop_cnt= 0;
		}
		vRadio_Init();
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
			// re-initialized by host dynamically
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
		else if (BASE_GPS_IDX == udd_g_ctrlreq->req.wIndex){
			//update base GPS location
			udd_set_setup_payload((float *)gs_uc_hrbuffer, BASE_GPS_LEN);
			udd_g_ctrlreq->callback = USB_Read_Base_GPS; // radio callback
		}
		else if (SET_FEC_IDX == udd_g_ctrlreq->req.wIndex){
			udd_set_setup_payload((uint8_t *)gs_uc_hrbuffer, SET_FEC_LEN);
			udd_g_ctrlreq->callback = USB_Host_Set_FEC;
		}
		else if (RADIO_GET_RSSI_IDX == udd_g_ctrlreq->req.wIndex){
			USB_Send_Latched_RSSI();
		}
		else if (RADIO_SET_PWR_LVL_IDX == udd_g_ctrlreq->req.wIndex){
			udd_set_setup_payload((uint8_t *)gs_uc_hrbuffer, RADIO_GET_PROPERTY_HOST_LEN);
			udd_g_ctrlreq->callback = USB_Set_Radio_Power;
		}
}

void USB_Read_Base_GPS(void){
		//update base GPS struct
		Base_Location.latitude = Degrees_To_Radians(*(float *)gs_uc_hrbuffer);
		Base_Location.longitude = Degrees_To_Radians(*((float *)gs_uc_hrbuffer+1));
		//Send updated GPS coords to drone
		Queue_Base_GPS_Packet();

}

void USB_Host_Set_FEC(void){
	uint8_t host_setting = *(uint8_t *)gs_uc_hrbuffer;
	switch(host_setting){
		case OFF:
			Send_with_FEC = false;
			break;
		case ON:
			Send_with_FEC = true;
			break;
		case AUTO:
			break;
		default:
		//unrecognized code!!! Don't do anything
			return;
	}
	//update the FEC_Option variable
	FEC_Option = host_setting;
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
Transfer_Control_Data_Out(void){
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
		unsigned char tdma_lock_char, rssi_taisync; //convert bool to uchar
		uint8_t fifolvlr, fifolvlt, payldvalid_flag, usrvalid_flag=0;
		uint32_t ldpc_err_taisync;

		if(ctrl_tdma_lock) tdma_lock_char=2; else tdma_lock_char=0;
		fifolvlr=((uint8_t)fifolvlcalc(wrptr_rdo_rpacket,rdptr_rdo_rpacket, RDO_RPACKET_FIFO_SIZE));
		fifolvlt=((uint8_t)fifolvlcalc(wrptr_rdo_tpacket,rdptr_rdo_tpacket, RDO_TPACKET_FIFO_SIZE));
		if(rdptr_rdo_rpacket == wrptr_rdo_rpacket){
			//identify no data to send
			usrvalid_flag=0;
			memset(rpacket_grp, 0xee, RADIO_GRPPKT_LEN);
		}
		else
		{ //process next fifo element
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
			else if (MSG_TYPE_REQUEST_FEC_ON == (*msg_header & 0x0F)){
				Send_with_FEC = true;
				//transfer dummy data to host
				//	memset(rpacket_grp, 0xee, RADIO_GRPPKT_LEN);
				memcpy(rpacket_grp, msg_header+1,RADIO_GRPPKT_LEN); //for debug
				usrvalid_flag = 1;
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
				#ifdef DEBUG_RADIOSTATUS
				rpacket_grp[21]=dbg_ctrlvidbuff[0];
				rpacket_grp[22]=dbg_ctrlvidbuff[1];
				rpacket_grp[23]=dbg_ctrlvidbuff[2];
				rpacket_grp[24]=dbg_ctrlvidbuff[3];
				rpacket_grp[25]=dbg_ctrlvidbuff[4];
				rpacket_grp[26]=dbg_ctrlvidbuff[5];
				dbg_ctrlvidbuff[0]=0x93; //data already read flag = 0x93. data new = 0x92
				#endif
			}
			else if (MSG_TYPE_SET_CHANNEL == (*msg_header & 0x0F)){
				if (MSG_TYPE_SET_CHANNEL == (*(msg_header+1) & 0x0F)){
					if (MSG_TYPE_SET_CHANNEL == (*(msg_header+2) & 0x0F)){
						if (!USE_915MHZ){
							control_channge_change_ack = true;
							//switch channel
							control_channel = channel_scan_selection;
						}
					}
				}
				memset(rpacket_grp, msg_header+1, RADIO_GRPPKT_LEN);
				usrvalid_flag = 0x1C;
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

		((uint8_t *)rpacket_grp)[RADIO_GRPPKT_LEN]= (usrvalid_flag) | tdma_lock_char;
			get_rdo_ldpc_failed(&ldpc_err_taisync) ;
		((uint8_t *)rpacket_grp)[RADIO_GRPPKT_LEN+1]= ldpc_err_taisync; /*crc_err_cnt; unavailable on taisync platform*/
		((uint8_t *)rpacket_grp)[RADIO_GRPPKT_LEN+2]= 0;
			get_radio_rssi(&rssi_taisync) ;
		((uint8_t *)rpacket_grp)[RADIO_GRPPKT_LEN+3]= rssi_taisync; /*FRR_Copy[0]; latched rssi:[0], cur_state:[1]*/
		udd_set_setup_payload( (uint8_t *)rpacket_grp, RADIO_GRPPKT_LEN+RADIO_INFO_LEN);
}
#endif  //RECEIVE_MAVLINK
