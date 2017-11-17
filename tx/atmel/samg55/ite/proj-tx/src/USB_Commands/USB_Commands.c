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
#include "Antenna_Diversity.h"
#include "radio.h"

extern void si446x_get_property(uint8_t group, uint8_t num_props, uint8_t start_prop);
extern void si446x_frr_a_read(U8 respByteCount );

#include "ReedSolomon.h"

 extern unsigned char tpacket_grp[RADIO_GRPPKT_LEN],
									rpacket_grp[RADIO_GRPPKT_LEN+RADIO_INFO_LEN];
 extern volatile uint32_t gs_rdo_rpacket[RDO_RPACKET_FIFO_SIZE*RDO_ELEMENT_SIZE];

extern volatile capv_tune_t si4463_factory_tune;
extern volatile ctrl_radio_stats r4463_sts;

#ifdef RADIO_SI4463
#ifdef CONFIG_ON_FLASH
 extern	uint32_t ul_page_addr_ctune,
								// temperature @ current tuning @ 2nd atmel reg on flash
								ul_page_addr_mtemp ,
								ul_page_addr_bootapp;
#endif
 extern volatile uint8_t spi_dma_mode;
 extern volatile bool si4463_radio_started;
extern void recalibrate_capval (void* ul_page_addr_mtemp, uint8_t median);
/*****************************************************************************/
void si4463_radio_cb(void) {
	 if (udd_g_ctrlreq.payload_size < udd_g_ctrlreq.req.wLength)
			return; // invalid call
	 else if (RADIO_STARTUP_IDX == udd_g_ctrlreq.req.wIndex) {
		spi_dma_mode = false;
		/* init radio stats obj */
			for (int j=0; j<CTRL_CTX_LEN; j++) {
				r4463_sts.ctrl_bits_ctx[j] = LONG_RNG;
			}
			r4463_sts.bw_ctrl_bits = LONG_RNG;
			r4463_sts.errPerAcc =0;
		vRadio_Init();
		#ifdef CONFIG_ON_FLASH
			uint8_t median = *(uint8_t*)ul_page_addr_ctune;
			if (0xff != median)  // adjust cap bank value per stored const
				recalibrate_capval((void*)ul_page_addr_mtemp, median);
		#endif
		 // frequency hopping in the action, initialization prior to hop operation because Rx side is currently the slave
		  ctrl_hop_global_update(true);
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
		Queue_Control_Packet(tpacket_grp);
	}
	else if (RADIO_PAIRID_IDX == udd_g_ctrlreq.req.wIndex) {
		uint8_t* pv = (U8*)gs_uc_hrbuffer;
		Set_Pair_ID(pv);
	}
	else if (RADIO_HOPLESS_IDX == udd_g_ctrlreq.req.wIndex) {
			memcpy(&fhopless, // set hopless section from user
								udd_g_ctrlreq.payload,
								sizeof(fhopless));
			fhop_offset = hop_chn_sel(fhop_offset);
	}

}
#endif

unsigned char radio_mon_rcvjitter=0;
tRadioConfiguration *pRadioConfiguration_temp;
 volatile uint8_t dbg_ctrlcnt;

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
		    else{  // hardcoded again;-( I had to accommodate with such fixed packet length assumption vs user packet length
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

			uint8_t *msg_header = (uint8_t *)gp_rdo_rpacket;

			if (Send_with_FEC){
				//message has FEC - decode, it is safe to reuse in buff as out buff
				Decode_Control_Packet(gp_rdo_rpacket,gp_rdo_rpacket);
			}


			if (MSG_TYPE_HOST_GENERATED == (*msg_header&0x0F)){
				//entire host message, copy to output buffer, omitting header byte
				memcpy(rpacket_grp, msg_header+1,RADIO_GRPPKT_LEN);
				usrvalid_flag = 9;
			}
			else if (MSG_TYPE_BASE_GPS_INFO == (*msg_header & 0x0F)){
				//message contains base station GPS info
				//can be received either with or without FEC
				Extract_Base_GPS_from_Control_Message(msg_header);
				memset(rpacket_grp, 0xee, RADIO_GRPPKT_LEN);
				usrvalid_flag = 0x00;
			}
			else if (MSG_TYPE_REQUEST_FEC_ON == (*msg_header & 0x0F)){
				Send_with_FEC = true;
				//transfer dummy data to host
				memset(rpacket_grp, 0xee, RADIO_GRPPKT_LEN);
				usrvalid_flag = 0;

			}
			else if (MSG_TYPE_REQUEST_FEC_OFF == (*msg_header & 0x0F)){
				Send_with_FEC = false;
				//transfer dummy data to host
				memset(rpacket_grp, 0xee, RADIO_GRPPKT_LEN);
				usrvalid_flag = 0;
			}
			else if (MSG_TYPE_IDLE == (*msg_header & 0x0F)){
					memcpy(rpacket_grp, msg_header+1, RADIO_GRPPKT_LEN);
				usrvalid_flag = 0;
			}
			else if(MSG_TYPE_SWITCH_VID_ANT == (*msg_header & 0x0F)){
				//Rx board sent message to switch the video antenna
				vid_ant_switch = true;
				memset(rpacket_grp, 0xee, RADIO_GRPPKT_LEN);
				usrvalid_flag = 0x00;
			}
			else if (MSG_TYPE_SET_CHANNEL == (*msg_header & 0x0F)){
				//message from RX board to change the radio channel
				//used with 869 only
				Handle_Channel_Change_Request(msg_header);
				memcpy(rpacket_grp, msg_header+1, RADIO_GRPPKT_LEN);
				usrvalid_flag = 0x1C;
			}
			else{
				//shouldn't get here right now, unrecognized code
				memset(rpacket_grp, 0x0, RADIO_GRPPKT_LEN);
				usrvalid_flag = 0;
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
		  ((uint8_t *)rpacket_grp)[RADIO_GRPPKT_LEN+2]= radio_mon_rcvjitter;
		  ((uint8_t *)rpacket_grp)[RADIO_GRPPKT_LEN+3]= FRR_Copy[0];

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
		else if (RADIO_SET_CHANNEL_IDX == udd_g_ctrlreq->req.wIndex){
			udd_set_setup_payload((uint8_t *)gs_uc_hrbuffer, RADIO_SET_CHANNEL_LEN);
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

void USB_Set_Radio_Channel(void){
	control_channel = (uint8_t)gs_uc_hrbuffer[0];
}

