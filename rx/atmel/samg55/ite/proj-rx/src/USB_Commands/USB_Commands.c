/*
 * USB_Commands.c
 *
 */


#include <stdint.h>
#include <string.h>
#include <compiler.h>
#include "si446x_cmd.h"
#include "compiler_defs.h"
#include "si446x_api_lib.h"
#include "USB_Commands.h"
#include "main.h"
#include "radio.h"

extern volatile capv_tune_t si4463_factory_tune;
extern volatile ctrl_radio_stats r4463_sts;

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
		   unsigned int wrptr_tmp1 = wrptr_rdo_tpacket;
		   unsigned int wrptr_tmp2;
		   unsigned int ovflag1,ovflag2;
		   ovflag1 = wrptr_inc(&wrptr_tmp1,&rdptr_rdo_tpacket,RDO_TPACKET_FIFO_SIZE,1);
		   wrptr_tmp2=wrptr_tmp1;
		   ovflag2 = wrptr_inc(&wrptr_tmp2,&rdptr_rdo_tpacket,RDO_TPACKET_FIFO_SIZE,1);
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
		  else
      { //process next fifo element
        unsigned int rdptr_race; //tmp to prevent race condition
        static bool gotpayloadhead=false;
        rdptr_race = rdptr_rdo_rpacket;
        rdptr_inc(&wrptr_rdo_rpacket, &rdptr_race, RDO_RPACKET_FIFO_SIZE,1);
		    gp_rdo_rpacket = gs_rdo_rpacket + (RDO_ELEMENT_SIZE*rdptr_race);
		    payldvalid_flag=(((uint8_t *)gp_rdo_rpacket)[RADIO_PKT_LEN-1]>>7)&0x01;
		    if( (((uint8_t *)gp_rdo_rpacket)[0] == 0xe5)&&
		      (((uint8_t *)gp_rdo_rpacket)[1] == 0xe5)&&
		      (((uint8_t *)gp_rdo_rpacket)[4] == 0xe5))
		      filler_flag=1;
		    if(ctrl_tdma_lock) tdma_lock_char=2; else tdma_lock_char=0;

        if((payldvalid_flag==0)&&(filler_flag==1))
		    {
			    //got filler idle rf packet
			    usrvalid_flag=0;
			    memcpy(rpacket_grp, gp_rdo_rpacket, RADIO_GRPPKT_LEN/2);
			    for(int i=0;i<RADIO_GRPPKT_LEN/2;i++)
			    ((uint8_t *)rpacket_grp)[RADIO_GRPPKT_LEN/2+i]=0xe5; //force fill 2nd half
		    }
		    else if(payldvalid_flag==1)
		    {
			    //got header payload
			    memset(rpacket_grp, 0xe2, RADIO_GRPPKT_LEN);
			    usrvalid_flag = 0;
			    memcpy(rpacket_grp_partial, gp_rdo_rpacket, RADIO_GRPPKT_LEN/2);
          gotpayloadhead=true;
		    }
		    else
		    {
			    //got tail end payload
          if(gotpayloadhead==false){
            memset(rpacket_grp,0x73,RADIO_GRPPKT_LEN/2);
            usrvalid_flag=0;}
          else {
            memcpy(rpacket_grp,rpacket_grp_partial,RADIO_GRPPKT_LEN/2);
            usrvalid_flag=1;}
          gotpayloadhead=false;

			    memcpy(rpacket_grp+RADIO_GRPPKT_LEN/2,gp_rdo_rpacket,RADIO_GRPPKT_LEN/2);
		    }
        rdptr_rdo_rpacket = rdptr_race;
      }
		  ((uint8_t *)rpacket_grp)[RADIO_GRPPKT_LEN]= usrvalid_flag | tdma_lock_char;
		  ((uint8_t *)rpacket_grp)[RADIO_GRPPKT_LEN+1]= (fifolvlr<<4) + fifolvlt;
		  ((uint8_t *)rpacket_grp)[RADIO_GRPPKT_LEN+2]= radio_mon_rcvjitter;
		  ((uint8_t *)rpacket_grp)[RADIO_GRPPKT_LEN+3]= ((uint8_t *)gp_rdo_rpacket)[RADIO_PKT_LEN-1];
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
		else if (BASE_GPS_IDX == udd_g_ctrlreq->req.wIndex){
			//update base GPS location
			udd_set_setup_payload((float *)gs_uc_hrbuffer, BASE_GPS_LEN);
			udd_g_ctrlreq->callback = si4463_radio_cb; // radio callback
		}
		else if (RADIO_GET_PROPERTY_IDX == udd_g_ctrlreq->req.wIndex){
			//get property
			udd_set_setup_payload((uint8_t *)gs_uc_hrbuffer, RADIO_GET_PROPERTY_HOST_LEN);
			udd_g_ctrlreq->callback = Si4463_GetProperty; // radio callback
		}
		else if (RADIO_GET_PROPERTY_REPLY_IDX == udd_g_ctrlreq->req.wIndex){
			USB_Send_Si4463_Props();
		}
}

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

