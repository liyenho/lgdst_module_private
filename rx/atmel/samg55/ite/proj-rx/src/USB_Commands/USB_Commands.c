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
#include "ctrl.h"
#include "GPS.h"
#include "radio.h"

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

extern volatile capv_tune_t si4463_factory_tune;
extern volatile ctrl_radio_stats r4463_sts;

#ifdef  RADIO_SI4463
#ifdef CONFIG_ON_FLASH
 extern	uint32_t ul_page_addr_ctune, // 1st atmel reg on flash
									// temperature @ current tuning @ 2nd atmel reg on flash
									ul_page_addr_mtemp,
									ul_page_addr_bootapp;
#endif
 extern volatile uint8_t spi_dma_mode;
extern void recalibrate_capval (void* ul_page_addr_mtemp, uint8_t median);
/*static*/ void si4463_radio_cb() {
	if (udd_g_ctrlreq.payload_size < udd_g_ctrlreq.req.wLength)
	return; // invalid call
	else if (RADIO_STARTUP_IDX == udd_g_ctrlreq.req.wIndex) {
		spi_dma_mode = false;
		/* init radio stats obj */
		{
			for (int j=0; j<CTRL_CTX_LEN; j++) {
				r4463_sts.ctrl_bits_ctx[j] = LONG_RNG;
			}
			r4463_sts.bw_ctrl_bits = LONG_RNG;
			r4463_sts.errPerAcc =0;
			r4463_sts.loop_cnt= 0;
		}
		vRadio_Init();
	#ifdef CONFIG_ON_FLASH
		uint8_t median = *(uint8_t*)ul_page_addr_ctune;
		if (0xff != median)  // adjust cap bank value per stored const
			recalibrate_capval((void*)ul_page_addr_mtemp, median);
	#endif
		{
			  for(int i=0;i<2*RDO_ELEMENT_SIZE;i++)
			  gs_rdo_tpacket[i]=0xc5c5c5c5;
		}
#ifndef RX_SPI_CHAINING
		pio_set(PIOB, PIO_PB9); // enable TS gate
#endif
		si4463_radio_started = true;
		ctrl_tdma_enable = true;
	}
	else if (RADIO_CHSEL_IDX == udd_g_ctrlreq.req.wIndex) {
		if (0 != ctrl_band_select(udd_g_ctrlreq.req.wLength, (U8*)gs_uc_hrbuffer))
			return ;  // cmd error or time out...
	}
	else if (RADIO_CTUNE_IDX == udd_g_ctrlreq.req.wIndex) {
		uint8_t ctune = *(U8*)gs_uc_hrbuffer;
		si446x_set_property1( 0x0, 0x1, 0x0, ctune ); //si446x_set_property shall crash...
	}
	else if (RADIO_DATA_TX_IDX == udd_g_ctrlreq.req.wIndex) {
		#if SEND_MAVLINK
			Queue_MavLink_from_USB(tpacket_grp);
		#else
			Queue_Control_Packet(tpacket_grp);
		#endif
	}
	else if (RADIO_PAIRID_IDX == udd_g_ctrlreq.req.wIndex) {
		uint8_t* pv = (U8*)gs_uc_hrbuffer;
		Set_Pair_ID(pv);
	}
	else if (RADIO_HOPLESS_IDX == udd_g_ctrlreq.req.wIndex) {
			memcpy(&fhopless, // set hopless section from user
								udd_g_ctrlreq.payload,
								sizeof(fhopless));
	}
}
#endif

unsigned char rpacket_grp_partial[RADIO_GRPPKT_LEN];
unsigned char radio_mon_rcvjitter=0;

void usb_ctrl_cmd_portal(udd_ctrl_request_t *udd_g_ctrlreq) {
		// it should be safe to use wIndex alternatively instead pointer to interface index
		if (RADIO_STARTUP_IDX == udd_g_ctrlreq->req.wIndex) {
			// re-initialized by host dynamically
			udd_set_setup_payload(pRadioConfiguration, sizeof(tRadioConfiguration));
			udd_g_ctrlreq->callback = si4463_radio_cb; // radio callback
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
			#if SEND_MAVLINK
				udd_set_setup_payload(tpacket_grp, MAVLINK_USB_TRANSFER_LEN); //stores payload in holding buffer
			#else
				udd_set_setup_payload(tpacket_grp, RADIO_GRPPKT_LEN); //stores payload in holding buffer
			#endif
		   udd_g_ctrlreq->callback = si4463_radio_cb; // radio callback
		}
		else if (RADIO_DATA_RX_IDX == udd_g_ctrlreq->req.wIndex) {
			Transfer_Control_Data_Out();
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
		else if (BASE_GPS_IDX == udd_g_ctrlreq->req.wIndex){
			//update base GPS location
			udd_set_setup_payload((float *)gs_uc_hrbuffer, BASE_GPS_LEN);
			udd_g_ctrlreq->callback = USB_Read_Base_GPS; // radio callback
		}
		else if (SET_FEC_IDX == udd_g_ctrlreq->req.wIndex){
			udd_set_setup_payload((uint8_t *)gs_uc_hrbuffer, SET_FEC_LEN);
			udd_g_ctrlreq->callback = USB_Host_Set_FEC;
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
		else if (RADIO_SET_REGIME_915_IDX == udd_g_ctrlreq->req.wIndex){
			USB_Set_Radio_Regime_915();
		}
		else if (RADIO_SET_REGIME_869_IDX == udd_g_ctrlreq->req.wIndex){
			USB_Set_Radio_Regime_869();
		}
		else if (RADIO_SET_CHANNEL_IDX == udd_g_ctrlreq->req.wIndex){
			udd_set_setup_payload((uint8_t *)gs_uc_hrbuffer, RADIO_SET_CHANNEL_LEN);
			udd_g_ctrlreq->callback = USB_Set_Radio_Channel;
		}
	}


//Request property info from Si4463
void Si4463_GetProperty(void){
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

void USB_Read_Base_GPS(void){
		//update base GPS struct
		Base_Location.latitude = Degrees_To_Radians(*(float *)gs_uc_hrbuffer);
		Base_Location.longitude = Degrees_To_Radians(*((float *)gs_uc_hrbuffer+1));
		//Send updated GPS coords to drone
		Queue_Base_GPS_Packet();

}

void USB_Send_Si4463_Props(void){
	memcpy(gs_uc_hrbuffer, Si4463_Properties, RADIO_GET_PROPERTY_ATMEL_LEN);
	//send data back to host
	udd_set_setup_payload(gs_uc_hrbuffer, RADIO_GET_PROPERTY_ATMEL_LEN);
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
	//si446x_frr_a_read(4);
	memcpy(gs_uc_hrbuffer, FRR_Copy, 4);
	udd_set_setup_payload(gs_uc_hrbuffer, 4);
}

void USB_Set_Radio_Power(void){
	uint8_t power_level = *(uint8_t *)gs_uc_hrbuffer;
	const uint8_t group = 0x22;
	const uint8_t num_props = 1;
	const uint8_t start_prop = 1;

	si446x_set_property1(group, num_props, start_prop, power_level);
}

void USB_Set_Radio_Regime_915(void){
	USE_915MHZ = true;
	vRadio_Init();
}


void USB_Set_Radio_Regime_869(void){
	USE_915MHZ = false;
	vRadio_Init();
}

void USB_Set_Radio_Channel(void){
	control_channel = (uint8_t)gs_uc_hrbuffer[0];
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
		unsigned char tdma_lock_char; //covert bool to uchar
		uint8_t fifolvlr, fifolvlt, payldvalid_flag, usrvalid_flag=0;

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
		((uint8_t *)rpacket_grp)[RADIO_GRPPKT_LEN+1]= crc_err_cnt;
		((uint8_t *)rpacket_grp)[RADIO_GRPPKT_LEN+2]= 0;
		((uint8_t *)rpacket_grp)[RADIO_GRPPKT_LEN+3]= FRR_Copy[0];
		udd_set_setup_payload( (uint8_t *)rpacket_grp, RADIO_GRPPKT_LEN+RADIO_INFO_LEN);
}
#endif  //RECEIVE_MAVLINK