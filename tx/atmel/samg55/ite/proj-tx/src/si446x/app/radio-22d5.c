/*! @file radio.c
 * @brief This file contains functions to interface with the radio chip.
 *
 * @b COPYRIGHT
 * @n Silicon Laboratories Confidential
 * @n Copyright 2012 Silicon Laboratories, Inc.
 * @n http://www.silabs.com
 */
#include <string.h>
#include "asf.h"
#include "delay.h"
#define INCLUDEINRADIO
#include "bsp.h"
#include "lgdst_4463_spi.h"
#include "ctrl.h"
#include "Radio_Buffers.h"
#include "ReedSolomon.h" // to init GF/Poly tables
extern bool ctrl_tdma_rxactive;

/*****************************************************************************
 *  Local Macros & Definitions
 *****************************************************************************/

const uint8_t cfg_lg_data_rate[] = {RF_MODEM_DATA_RATE_LG_3};
const uint8_t cfg_sh_data_rate[] = {RF_MODEM_DATA_RATE_SH_3};
const uint8_t cfg_lg_freq_dev[] = {RF_MODEM_FREQ_DEV_LG_3};
const uint8_t cfg_sh_freq_dev[] = {RF_MODEM_FREQ_DEV_SH_3};
const uint8_t cfg_lg_ramp_dly[] = {RF_MODEM_TX_RAMP_DELAY_LG_8};
const uint8_t cfg_sh_ramp_dly[] = {RF_MODEM_TX_RAMP_DELAY_SH_8};
const uint8_t cfg_lg_bcr_osr_1[] = {RF_MODEM_BCR_OSR_1_LG_9};
const uint8_t cfg_sh_bcr_osr_1[] = {RF_MODEM_BCR_OSR_1_SH_9};
const uint8_t cfg_lg_afc_gear[] = {RF_MODEM_AFC_GEAR_LG_7};
const uint8_t cfg_sh_afc_gear[] = {RF_MODEM_AFC_GEAR_SH_7};
const uint8_t cfg_lg_agc_ctrl[] = {RF_MODEM_AGC_CONTROL_LG_1};
const uint8_t cfg_sh_agc_ctrl[] = {RF_MODEM_AGC_CONTROL_SH_1};
const uint8_t cfg_lg_agc_win_size[] = {RF_MODEM_AGC_WINDOW_SIZE_LG_9};
const uint8_t cfg_sh_agc_win_size[] = {RF_MODEM_AGC_WINDOW_SIZE_SH_9};
const uint8_t cfg_lg_raw_ctrl[] = {RF_MODEM_RAW_CNT_LG_3};
const uint8_t cfg_sh_raw_ctrl[] = {RF_MODEM_RAW_CNT_SH_3};
const uint8_t cfg_lg_chflt_rx1_coef13[] = {RF_MODEM_CHFLT_RX1_CHFLT_COE13_LG_12};
const uint8_t cfg_sh_chflt_rx1_coef13[] = {RF_MODEM_CHFLT_RX1_CHFLT_COE13_SH_12};
const uint8_t cfg_lg_chflt_rx1_coef1[] = {RF_MODEM_CHFLT_RX1_CHFLT_COE1_LG_12};
const uint8_t cfg_sh_chflt_rx1_coef1[] = {RF_MODEM_CHFLT_RX1_CHFLT_COE1_SH_12};
const uint8_t cfg_lg_chflt_rx2_coef7[] = {RF_MODEM_CHFLT_RX2_CHFLT_COE7_LG_12};
const uint8_t cfg_sh_chflt_rx2_coef7[] = {RF_MODEM_CHFLT_RX2_CHFLT_COE7_SH_12};

const uint8_t rssi_comp_offset[] = {RSSI_COMP_OFFSET_1};
//extern uint32_t wrptr_rdo_rpacket;
//extern uint32_t rdptr_rdo_rpacket;
extern volatile uint32_t *DWT_CYCCNT;

uint8_t FRR_Copy[4] = {0};
uint32_t pkt_rcv_timestamp = 0;
uint8_t incoming_data[MAX_PACKET_LEN];
uint32_t crc_err_cnt = 0;

uint32_t hop_watchdog_intv = 0;  //used to track how long since both (all) messages on a hop received
uint32_t hop_watchdog_reset_cnt =0;

SEGMENT_VARIABLE(bMain_IT_Status_m, U8, SEG_XDATA); // backup flag
/*******************************************************************/
//const unsigned int initPeriod = 3000; // startup period in msec, m.a. was in fast tracking pace
#define initCnt		30
const unsigned int initWgt = 8,
									initScl = 3,
									Ewis = 3,
									Ewim = 8-3;
//const unsigned int intePeriod = 30000; // intermediate period in msec, m.a. was in normal tracking pace
#define inteCnt		200
const unsigned int inteWgt = 16,
									inteScl = 4,
									Ewms = 5,
									Ewmm = 16-5;
const unsigned int normWgt = 16,
									normScl = 4,
									Ewns = 3,
									Ewnm = 16-3;
const unsigned int lgThr = 3, shThr= 1;
const uint32_t error_weight_cks6b[] __attribute__((aligned(8)))= {
	0, 1, 1, 2, 1, 2, 2, 3,
	1, 2, 2, 3, 2, 3, 3, 4,
	1, 2, 2, 3, 2, 3, 3, 4,
	2, 3, 3, 4, 3, 4, 4, 5,
	1, 2, 2, 3, 2, 3, 3, 4,
	2, 3, 3, 4, 3, 4, 4, 5,
	2, 3, 3, 4, 3, 4, 4, 5,
	3, 4, 4, 5, 4, 5, 5, 6,
} ;
#define CTRL_MON_PERIOD	250  // in msec
#define CTRL_FAIL_PERIOD	(6* CTRL_MON_PERIOD) // can't be too short in order to prevent spi comm lockup
#define CTRL0_MSK			(-1+(1<<CTRL_BITS))
#define CTRL0_IMSK			(0xff & ~CTRL0_MSK)
#define CTRL_MSK				(CTRL0_MSK<<CTRL_BITS)
#define CHKSM_MSK		(0xff ^ CTRL0_MSK)
// 4463 stats mon obj
volatile ctrl_radio_stats  r4463_sts;
volatile capv_tune_t si4463_factory_tune;
static BW_CTRL cmd_ctrl_bits = LONG_RNG;
/*******************************************************************/
extern unsigned int *gp_rdo_tpacket_l;
extern uint32_t ul_page_addr_ctune, ul_page_addr_mtemp;
extern volatile bool ctrl_tdma_enable, ctrl_tdma_lock;
extern volatile uint32_t *DWT_CYCCNT;
extern volatile bool fhop_in_search, fhop_flag;
extern int fhop_base, fhopless/*can be 1, 2 or 3*/;
extern int fhop_offset ;
extern uint8_t backup[NUM_OF_FPGA_REGS+NUM_OF_ATMEL_REGS+4+1];
extern volatile  uint8_t id_byte;

/*****************************************************************************
 *  Function Prototyping
 *****************************************************************************/
uint32_t wrptr_inc(uint32_t *wrptr,  uint32_t *rdptr, uint32_t fifodepth, int step);
uint32_t rdptr_inc(uint32_t *wrptr,  uint32_t *rdptr, uint32_t fifodepth, int step);

/*****************************************************************************
 *  Local Function Declarations
 *****************************************************************************/
void vRadio_PowerUp(void);
/*!
 *  Power up the Radio.
 *
 *  @note
 *
 */
void vRadio_PowerUp(void)
{
  //SEGMENT_VARIABLE(wDelay,  U16, SEG_XDATA) = 0u;

  /* Hardware reset the chip */
  si446x_reset();

  /* Wait until reset timeout or Reset IT signal */
  //for (; wDelay < pRadioConfiguration->Radio_Delay_Cnt_After_Reset; wDelay++);
  delay_ms(10);
}

/*!
 *  Radio Initialization.
 *
 *  @author Sz. Papp
 *
 *  @note
 *
 */
U8 vRadio_Init(void)
{
  U16 wDelay;
  U8 vradiofailcnt=0;

  /* Power Up the radio chip */
  vRadio_PowerUp();

  /* Load radio configuration */
  if(USE_915MHZ){
	  pRadioConfiguration = &RadioConfiguration_915;
  }	else{
	  pRadioConfiguration = &RadioConfiguration_869;
  }
  while (SI446X_SUCCESS != si446x_configuration_init(pRadioConfiguration->Radio_ConfigurationArray))
  {
    vradiofailcnt++;
	delay_ms(10);
    /* Power Up the radio chip */
    vRadio_PowerUp();
  }

  // Read ITs, clear pending ones
  si446x_get_int_status(0u, 0u, 0u);

  // hardwired calibration on RSSI compensation
  if (radio_comm_SendCmdGetResp(sizeof(rssi_comp_offset), rssi_comp_offset, 0, 0) != 0xFF) {
		vradiofailcnt ++;
  }
  // init RS GF/Poly tables at local
  uint8_t t_in[RADIO_PKT_LEN],
  					t_out[RADIO_PKT_LEN];
	Encode_Control_Packet(t_in,  t_out);
  return(vradiofailcnt);
}


/*!
 *  Check if Packet sent IT flag or Packet Received IT is pending.
 *
 *  @return   SI4455_CMD_GET_INT_STATUS_REP_PACKET_SENT_PEND_BIT / SI4455_CMD_GET_INT_STATUS_REP_PACKET_RX_PEND_BIT
 *
 *  @note
 *
 */
static uint32_t underflowcnt=0;
static uint32_t tx_complete_cnt=0;

static uint8_t *fifo_write_ptr;
static uint8_t fifo_write_offset;

uint32_t bytes_read=0;
uint32_t bytes_sent =0;
static bool sending_long_packet = false;

U8 bRadio_Check_Tx_RX(void)
{
	if ( SBIT_R(RF4463_NIRQ) == FALSE)//RF_NIRQ == FALSE)
	{

		ctrl_tdma_rxactive=true;
		/* Read ITs, clear pending ones */
		si446x_get_int_status(0u, 0u, 0u);
		si446x_request_device_state();

		if (Si446xCmd.GET_INT_STATUS.CHIP_PEND & SI446X_CMD_GET_CHIP_STATUS_REP_CHIP_PEND_CMD_ERROR_PEND_BIT)
		{
			/* State change to */
			si446x_change_state(SI446X_CMD_CHANGE_STATE_ARG_NEXT_STATE1_NEW_STATE_ENUM_SLEEP);

			/* Reset FIFO */
			si446x_fifo_info(SI446X_CMD_FIFO_INFO_ARG_FIFO_RX_BIT|SI446X_CMD_FIFO_INFO_ARG_FIFO_TX_BIT);

			/* State change to */
			si446x_change_state(SI446X_CMD_CHANGE_STATE_ARG_NEXT_STATE1_NEW_STATE_ENUM_RX);

			return 0xFA;
		}

		if(Si446xCmd.GET_INT_STATUS.PH_PEND & SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_PACKET_SENT_PEND_BIT)
		{
			ctrl_tdma_rxactive=false;
			TX_Active = false;
			tx_complete_cnt++;

			return SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_PACKET_SENT_PEND_BIT;
		}

		if (Si446xCmd.GET_INT_STATUS.PH_STATUS & SI446X_CMD_GET_INT_STATUS_REP_PH_STATUS_TX_FIFO_ALMOST_EMPTY_BIT){
			//Tx FIFO is almost empty. refill!
			ctrl_tdma_rxactive=false;
			if (sending_long_packet){
				si446x_write_tx_fifo(PKT_TX_THRESHOLD, fifo_write_ptr+fifo_write_offset);
				fifo_write_offset+=PKT_TX_THRESHOLD;

				bytes_sent+= PKT_TX_THRESHOLD;
				if ((bytes_sent%RADIO_PKT_LEN)==0){
					rdptr_inc(&wrptr_rdo_tpacket, &rdptr_rdo_tpacket, RDO_TPACKET_FIFO_SIZE, 1);
					bytes_sent = 0;
				}

				return SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_TX_FIFO_ALMOST_EMPTY_PEND_BIT;
			}
		}

		if (Si446xCmd.GET_INT_STATUS.CHIP_PEND & SI446X_CMD_GET_CHIP_STATUS_REP_CHIP_PEND_FIFO_UNDERFLOW_OVERFLOW_ERROR_PEND_BIT){
			underflowcnt++;
			//si446x_change_state(SI446X_CMD_CHANGE_STATE_ARG_NEXT_STATE1_NEW_STATE_ENUM_SLEEP);
			//si446x_fifo_info(0x00);
			si446x_fifo_info(0x01); //need to reset FIFO
			si446x_get_int_status(0u, 0u, 0u); //clear any pending interrupts
			TX_Active = false;
			//si446x_change_state(SI446X_CMD_CHANGE_STATE_ARG_NEXT_STATE1_NEW_STATE_ENUM_RX);

			return 0xFC;

		}

		if (Si446xCmd.GET_INT_STATUS.PH_PEND & SI446X_CMD_GET_INT_STATUS_REP_PH_STATUS_CRC_ERROR_BIT)
		{
			//assumes CRC check only comes at end of packet
			//even with error, record time for TDMA sync purposes
			pkt_rcv_timestamp = *DWT_CYCCNT;

			/* Reset FIFO */
			si446x_fifo_info(SI446X_CMD_FIFO_INFO_ARG_FIFO_RX_BIT);

			crc_err_cnt++;
			//reset for next packet1
			bytes_read = 0;


			return SI446X_CMD_GET_INT_STATUS_REP_PH_STATUS_CRC_ERROR_BIT;
		}

		if(Si446xCmd.GET_INT_STATUS.PH_PEND & SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_PACKET_RX_PEND_BIT)
		{
			/* Packet RX */
			pkt_rcv_timestamp = *DWT_CYCCNT;

			/* Get payload length */
			si446x_fifo_info(0x00);
			si446x_read_rx_fifo(Si446xCmd.FIFO_INFO.RX_FIFO_COUNT, incoming_data+bytes_read);

			bytes_read+=Si446xCmd.FIFO_INFO.RX_FIFO_COUNT;
			//save RSSI for later
			si446x_frr_a_read(4);
			memcpy(FRR_Copy, &Si446xCmd.FRR_A_READ.FRR_A_VALUE, 4);

			ctrl_tdma_rxactive=false;

			#if RECEIVE_MAVLINK
				Queue_MavLink_Raw_Data(&incoming_MavLink_Data, bytes_read, incoming_data);
			#else

			if(incoming_data[RADIO_PKT_LEN-1] != id_byte)
			{
				//YH: bypass this read due to bad id info
				return SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_PACKET_RX_PEND_BIT;
			}
			//copy data from temp buffer
			for (int i =0; i< (bytes_read/RADIO_PKT_LEN);i++){
				wrptr_inc(&wrptr_rdo_rpacket, &rdptr_rdo_rpacket, RDO_TPACKET_FIFO_SIZE, 1);
				memcpy(gs_rdo_rpacket + (wrptr_rdo_rpacket*RDO_ELEMENT_SIZE), incoming_data+(i*RADIO_PKT_LEN),RADIO_PKT_LEN);
			}
			uint32_t wrptr_tmp=wrptr_rdo_rpacket;
			wrptr_inc(&wrptr_tmp, &rdptr_rdo_rpacket, (uint32_t)RDO_RPACKET_FIFO_SIZE,1);//to be written position
			gp_rdo_rpacket_l = gs_rdo_rpacket + (RDO_ELEMENT_SIZE*wrptr_tmp);

			#endif
			//reset the count in preparation for the next packet
			bytes_read=0;

			return SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_PACKET_RX_PEND_BIT;
		}

		if(Si446xCmd.GET_INT_STATUS.PH_PEND & SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_RX_FIFO_ALMOST_FULL_PEND_BIT)
		{
			static unsigned int *write_location;
			/*Get payload length*/
			si446x_fifo_info(0x00);
			si446x_read_rx_fifo(Si446xCmd.FIFO_INFO.RX_FIFO_COUNT,  incoming_data+bytes_read);

			bytes_read+=Si446xCmd.FIFO_INFO.RX_FIFO_COUNT;

			return   SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_RX_FIFO_ALMOST_FULL_PEND_BIT;
		}


		ctrl_tdma_rxactive=false;

	}

	return 0;
}




/*!
 *  Set Radio to RX mode. .
 *
 *  @param channel Freq. Channel,  packetLength : 0 Packet handler fields are used , nonzero: only Field1 is used
 *
 *  @note
 *
 */
void vRadio_StartRX(U8 channel, U8 packetLength )
{
	bytes_read=0;
  // Read ITs, clear pending ones
  si446x_get_int_status(0u, 0u, 0u);

   // Reset the Rx Fifo
   si446x_fifo_info(SI446X_CMD_FIFO_INFO_ARG_FIFO_RX_BIT);

  /* Start Receiving packet, channel 0, START immediately, Packet length used or not according to packetLength */
  si446x_start_rx(channel, 0u, packetLength,
                  SI446X_CMD_START_RX_ARG_NEXT_STATE1_RXTIMEOUT_STATE_ENUM_NOCHANGE,
                  SI446X_CMD_START_RX_ARG_NEXT_STATE2_RXVALID_STATE_ENUM_READY,
                  SI446X_CMD_START_RX_ARG_NEXT_STATE3_RXINVALID_STATE_ENUM_RX );


}

//use to receive a packet with variable lengths
void vRadio_StartRX_variableLength(U8 channel)
{
	vRadio_StartRX(channel, 0);
}

/*!
 *  Set Radio to TX mode, variable packet length.
 *
 *  @param channel Freq. Channel, Packet to be sent length of of the packet sent to TXFIFO
 *
 *  @note
 *
 */
static int TX_Active_lock_cnt =0;
void vRadio_StartTx(U8 channel, U8 *pioRadioPacket, U16  length)
{
	if (TX_Active){
		TX_Active_lock_cnt++;
		if(TX_Active_lock_cnt>100){
			TX_Active_lock_cnt = 0; //reset to prevent lock up
			TX_Active = false;
		}
		//already transmitting, abort
		return;
	}
	TX_Active = true;
	TX_Active_lock_cnt =0;
	fifo_write_ptr = pioRadioPacket;
	fifo_write_offset=0;

  /* Leave RX state */
  si446x_change_state(SI446X_CMD_CHANGE_STATE_ARG_NEXT_STATE1_NEW_STATE_ENUM_READY);

  /* Read ITs, clear pending ones */
  si446x_get_int_status(0u, 0u, 0u);

  /* Reset the Tx Fifo */
  si446x_fifo_info(SI446X_CMD_FIFO_INFO_ARG_FIFO_TX_BIT);

	/* Fill the TX fifo with data */
	if (length>RADIO_TX_FIFO_SIZE){
		//message is too big to fit into FIFO, send first portion
		//rely on FIFO almost empty interrupt to transfer rest of message
		sending_long_packet = true;
		si446x_write_tx_fifo(RADIO_TX_FIFO_SIZE, fifo_write_ptr+fifo_write_offset);
		fifo_write_offset+=RADIO_TX_FIFO_SIZE;
		rdptr_inc(&wrptr_rdo_tpacket, &rdptr_rdo_tpacket, RDO_TPACKET_FIFO_SIZE, length/RADIO_PKT_LEN);
	}else{
		sending_long_packet = false;
		si446x_write_tx_fifo(length, pioRadioPacket);
		rdptr_inc(&wrptr_rdo_tpacket, &rdptr_rdo_tpacket, RDO_TPACKET_FIFO_SIZE, 1);
	}


  /* Start sending packet, channel 0, START immediately */
	// condition, 0x80, instruct si4463 switch to rx state once finish tx
   si446x_start_tx(channel, 0x80, length);

}

static uint8_t pending_transmit[1000] = {0};

//use to send a packet that will be structured:
// Field1 (length,2 bytes, little endian)|Field 2 (data, var length)
void vRadio_StartTx_Variable_Length(U8 channel, U8 *pioRadioPacket, U16  length){
	//build packet to send
	//first 2 bytes indicate length of payload, little endian
	memcpy(pending_transmit, &length, 2);
	//copy payload data
	memcpy(pending_transmit+2, pioRadioPacket, length);
	//send!
	vRadio_StartTx(channel, pending_transmit, length+2);
}

void vRadio_StartTx_No_Data(U8 channel)
{

	/* Read ITs, clear pending ones */
	si446x_get_int_status(0u, 0u, 0u);

	/* Start sending packet, channel 0, START immediately, no valid user data and never switch to listen */
	si446x_start_tx(channel, 0u, 0u);

}
/*!
 *  Set Radio to TX range mode, currently support long and short range in variant data rate and frequency deviation
 *
 *  @param range mode. 0: short, 1: long, range configuration
 *
 *  @note
 *
 */
U8 range_mode_configure(U8 range) {
	if (1/*long*/ == range) {
	    if (radio_comm_SendCmdGetResp(sizeof(cfg_lg_data_rate), cfg_lg_data_rate, 0, 0) != 0xFF)
	    {
	      /* Timeout occured */
	      return SI446X_CTS_TIMEOUT;
	    }
	    if (radio_comm_SendCmdGetResp(sizeof(cfg_lg_freq_dev), cfg_lg_freq_dev, 0, 0) != 0xFF)
	    {
	      /* Timeout occured */
	      return SI446X_CTS_TIMEOUT;
	    }
	    if (radio_comm_SendCmdGetResp(sizeof(cfg_lg_ramp_dly), cfg_lg_ramp_dly, 0, 0) != 0xFF)
	    {
	      /* Timeout occured */
	      return SI446X_CTS_TIMEOUT;
	    }
	    if (radio_comm_SendCmdGetResp(sizeof(cfg_lg_bcr_osr_1), cfg_lg_bcr_osr_1, 0, 0) != 0xFF)
	    {
	      /* Timeout occured */
	      return SI446X_CTS_TIMEOUT;
	    }
	    if (radio_comm_SendCmdGetResp(sizeof(cfg_lg_afc_gear), cfg_lg_afc_gear, 0, 0) != 0xFF)
	    {
	      /* Timeout occured */
	      return SI446X_CTS_TIMEOUT;
	    }
	    if (radio_comm_SendCmdGetResp(sizeof(cfg_lg_agc_ctrl), cfg_lg_agc_ctrl, 0, 0) != 0xFF)
	    {
	      /* Timeout occured */
	      return SI446X_CTS_TIMEOUT;
	    }
	    if (radio_comm_SendCmdGetResp(sizeof(cfg_lg_agc_win_size), cfg_lg_agc_win_size, 0, 0) != 0xFF)
	    {
	      /* Timeout occured */
	      return SI446X_CTS_TIMEOUT;
	    }
	    if (radio_comm_SendCmdGetResp(sizeof(cfg_lg_raw_ctrl), cfg_lg_raw_ctrl, 0, 0) != 0xFF)
	    {
	      /* Timeout occured */
	      return SI446X_CTS_TIMEOUT;
	    }
	    if (radio_comm_SendCmdGetResp(sizeof(cfg_lg_chflt_rx1_coef13), cfg_lg_chflt_rx1_coef13, 0, 0) != 0xFF)
	    {
	      /* Timeout occured */
	      return SI446X_CTS_TIMEOUT;
	    }
	    if (radio_comm_SendCmdGetResp(sizeof(cfg_lg_chflt_rx1_coef1), cfg_lg_chflt_rx1_coef1, 0, 0) != 0xFF)
	    {
	      /* Timeout occured */
	      return SI446X_CTS_TIMEOUT;
	    }
	    if (radio_comm_SendCmdGetResp(sizeof(cfg_lg_chflt_rx2_coef7), cfg_lg_chflt_rx2_coef7, 0, 0) != 0xFF)
	    {
	      /* Timeout occured */
	      return SI446X_CTS_TIMEOUT;
	    }
	}
	else /* short*/{
	    if (radio_comm_SendCmdGetResp(sizeof(cfg_sh_data_rate), cfg_sh_data_rate, 0, 0) != 0xFF)
	    {
	      /* Timeout occured */
	      return SI446X_CTS_TIMEOUT;
	    }
	    if (radio_comm_SendCmdGetResp(sizeof(cfg_sh_freq_dev), cfg_sh_freq_dev, 0, 0) != 0xFF)
	    {
	      /* Timeout occured */
	      return SI446X_CTS_TIMEOUT;
	    }
	    if (radio_comm_SendCmdGetResp(sizeof(cfg_sh_ramp_dly), cfg_sh_ramp_dly, 0, 0) != 0xFF)
	    {
	      /* Timeout occured */
	      return SI446X_CTS_TIMEOUT;
	    }
	    if (radio_comm_SendCmdGetResp(sizeof(cfg_sh_bcr_osr_1), cfg_sh_bcr_osr_1, 0, 0) != 0xFF)
	    {
	      /* Timeout occured */
	      return SI446X_CTS_TIMEOUT;
	    }
	    if (radio_comm_SendCmdGetResp(sizeof(cfg_sh_afc_gear), cfg_sh_afc_gear, 0, 0) != 0xFF)
	    {
	      /* Timeout occured */
	      return SI446X_CTS_TIMEOUT;
	    }
	    if (radio_comm_SendCmdGetResp(sizeof(cfg_sh_agc_ctrl), cfg_sh_agc_ctrl, 0, 0) != 0xFF)
	    {
	      /* Timeout occured */
	      return SI446X_CTS_TIMEOUT;
	    }
	    if (radio_comm_SendCmdGetResp(sizeof(cfg_sh_agc_win_size), cfg_sh_agc_win_size, 0, 0) != 0xFF)
	    {
	      /* Timeout occured */
	      return SI446X_CTS_TIMEOUT;
	    }
	    if (radio_comm_SendCmdGetResp(sizeof(cfg_sh_raw_ctrl), cfg_sh_raw_ctrl, 0, 0) != 0xFF)
	    {
	      /* Timeout occured */
	      return SI446X_CTS_TIMEOUT;
	    }
	    if (radio_comm_SendCmdGetResp(sizeof(cfg_sh_chflt_rx1_coef13), cfg_sh_chflt_rx1_coef13, 0, 0) != 0xFF)
	    {
	      /* Timeout occured */
	      return SI446X_CTS_TIMEOUT;
	    }
	    if (radio_comm_SendCmdGetResp(sizeof(cfg_sh_chflt_rx1_coef1), cfg_sh_chflt_rx1_coef1, 0, 0) != 0xFF)
	    {
	      /* Timeout occured */
	      return SI446X_CTS_TIMEOUT;
	    }
	    if (radio_comm_SendCmdGetResp(sizeof(cfg_sh_chflt_rx2_coef7), cfg_sh_chflt_rx2_coef7, 0, 0) != 0xFF)
	    {
	      /* Timeout occured */
	      return SI446X_CTS_TIMEOUT;
	    }
	}
	return SI446X_SUCCESS;
}


void Set_Sync_Words(uint8_t byte1, uint8_t byte2){

	si446x_set_property1(0x11,0x02, 0x01, byte1);
	si446x_set_property1(0x11,0x02, 0x02, byte2);

	return;
}


uint8_t channel_power[10] = {0};
#define NUM_SAMPLES	4


//Channel_Scan returns true when all channels have been measured, Bad design not yet used, liyenho

uint8_t selected_channel = 0;
 extern volatile bool ctrl_tdma_enable;
bool Channel_Scan(void){
	static int channel = 0, idx=0;
	static uint32_t running_total = 0;

	ctrl_tdma_enable = false; //block transmitting while meausring power

	static bool measuring = false;

	if (!measuring){
		idx = 0;
		vRadio_StartRX(channel, 0xFF);
		//ToDo: what if RX stops?
		measuring = true;
		return false;
	}

	if (measuring){
		if (idx<NUM_SAMPLES){
			si446x_get_modem_status(0);
			running_total +=Si446xCmd.GET_MODEM_STATUS.CURR_RSSI;
			idx++;
		}else{
			measuring = false; //measurements are finished on this channel
			idx = 0;
			channel_power[channel] = running_total/NUM_SAMPLES;
			running_total =0;
			channel++;
		}
	}

	uint8_t RSSI_MIN = 0xFF;
	uint8_t best_chnl = 0;
	if (channel == 10){
		//when channel gets to 10, all channels have been measured
		for (int i =1; i<10; i++){
			//skip channel 0, that is base channel for pairing etc
			if (channel_power[i]<RSSI_MIN){
				RSSI_MIN = channel_power[i];
				selected_channel = i;
			}
		}
		ctrl_tdma_enable = true;
		return true;
	}
	return false;
}

/*!
 *  Set Radio channel selection,10 bands per 2 mhz on each to cover 20 mhz radio frequency spectrum
 *
 *  @param len : number of bytes of ch sel param are passed
 *  @param ch_param : a series of bytes to form chan sele command
 *
 *  @note
 *
 */
#include "main.h"  // for the length of specific property set
 /* used to be channel selection table for control radio band plan, now it can serve as lookup table for frequency hopp */
#if false
const uint8_t RF_FREQ_CONTROL_INTE_BAND0_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0B, 0x24, 0x7A, 0x08, 0xF6}; // 902.285 mhz
 const uint8_t RF_MODEM_AFC_LIMITER_BAND0_3[] = {0x11, 0x20, 0x03, 0x30, 0x25, 0x16, 0xC0};
const uint8_t RF_FREQ_CONTROL_INTE_BAND13_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0E, 0x84, 0x7A, 0x08, 0xF6}; // 925.035 mhz
 const uint8_t RF_MODEM_AFC_LIMITER_BAND13_3[] = {0x11, 0x20, 0x03, 0x30, 0x25, 0x16, 0xC0};
#else  // building frequency hopping table, 500 khz channel spacing? based upon config file of 30 kb/s, 15 khz and 250 khz channel space
const uint8_t RF_FREQ_CONTROL_INTE_HOP0_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0B, 0x60, 0x00, 0x40, 0x00}; // 902.75 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP0_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP0_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};  // testing with dynamic new rate, liyenho
const uint8_t RF_FREQ_CONTROL_INTE_HOP1_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0B, 0xA0, 0x00, 0x40, 0x00}; // 903.25 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP1_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP1_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP2_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0B, 0xE0, 0x00, 0x40, 0x00}; // 903.75 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP2_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP2_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP3_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0C, 0x20, 0x00, 0x40, 0x00}; // 904.25 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP3_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP3_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP4_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0C, 0x60, 0x00, 0x40, 0x00}; // 904.75 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP4_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP4_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP5_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0C, 0xA0, 0x00, 0x40, 0x00}; // 905.25 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP5_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP5_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP6_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0C, 0xE0, 0x00, 0x40, 0x00}; // 905.75 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP6_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP6_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP7_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0D, 0x20, 0x00, 0x40, 0x00}; // 906.25 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP7_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP7_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP8_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0D, 0x60, 0x00, 0x40, 0x00}; // 906.75 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP8_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP8_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP9_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0D, 0xA0, 0x00, 0x40, 0x00}; // 907.25 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP9_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP9_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP10_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0D, 0xE0, 0x00, 0x40, 0x00}; // 907.75 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP10_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP10_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP11_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0E, 0x20, 0x00, 0x40, 0x00}; // 908.25 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP11_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP11_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP12_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0E, 0x60, 0x00, 0x40, 0x00}; // 908.75 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP12_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP12_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP13_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0E, 0xA0, 0x00, 0x40, 0x00}; // 909.25 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP13_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP13_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP14_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0E, 0xE0, 0x00, 0x40, 0x00}; // 909.75 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP14_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP14_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP15_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0F, 0x20, 0x00, 0x40, 0x00}; // 910.25 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP15_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP15_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP16_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0F, 0x60, 0x00, 0x40, 0x00}; // 910.75 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP16_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP16_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP17_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0F, 0xA0, 0x00, 0x40, 0x00}; // 911.25 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP17_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP17_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP18_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0F, 0xE0, 0x00, 0x40, 0x00}; // 911.75 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP18_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP18_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP19_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x08, 0x20, 0x00, 0x40, 0x00}; // 912.25 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP19_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP19_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP20_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x08, 0x60, 0x00, 0x40, 0x00}; // 912.75 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP20_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP20_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP21_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x08, 0xA0, 0x00, 0x40, 0x00}; // 913.25 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP21_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP21_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP22_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x08, 0xE0, 0x00, 0x40, 0x00}; // 913.75 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP22_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP22_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP23_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x09, 0x20, 0x00, 0x40, 0x00}; // 914.25 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP23_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP23_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP24_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x09, 0x60, 0x00, 0x40, 0x00}; // 914.75 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP24_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP24_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP25_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x09, 0xA0, 0x00, 0x40, 0x00}; // 915.25 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP25_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP25_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP26_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x09, 0xE0, 0x00, 0x40, 0x00}; // 915.75 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP26_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP26_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP27_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0A, 0x20, 0x00, 0x40, 0x00}; // 916.25 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP27_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP27_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP28_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0A, 0x60, 0x00, 0x40, 0x00}; // 916.75 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP28_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP28_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP29_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0A, 0xA0, 0x00, 0x40, 0x00}; // 917.25 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP29_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP29_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP30_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0A, 0xE0, 0x00, 0x40, 0x00}; // 917.75 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP30_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP30_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP31_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0B, 0x20, 0x00, 0x40, 0x00}; // 918.25 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP31_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP31_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP32_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0B, 0x60, 0x00, 0x40, 0x00}; // 918.75 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP32_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP32_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP33_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0B, 0xA0, 0x00, 0x40, 0x00}; // 919.25 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP33_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP33_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP34_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0B, 0xE0, 0x00, 0x40, 0x00}; // 919.75 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP34_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP34_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP35_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0C, 0x20, 0x00, 0x40, 0x00}; // 920.25 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP35_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP35_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP36_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0C, 0x60, 0x00, 0x40, 0x00}; // 920.75 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP36_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP36_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP37_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0C, 0xA0, 0x00, 0x40, 0x00}; // 921.25 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP37_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP37_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP38_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0C, 0xE0, 0x00, 0x40, 0x00}; // 921.75 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP38_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP38_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP39_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0D, 0x20, 0x00, 0x40, 0x00}; // 922.25 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP39_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP39_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP40_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0D, 0x60, 0x00, 0x40, 0x00}; // 922.75 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP40_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP40_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP41_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0D, 0xA0, 0x00, 0x40, 0x00}; // 923.25 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP41_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP41_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP42_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0D, 0xE0, 0x00, 0x40, 0x00}; // 923.75 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP42_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP42_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP43_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0E, 0x20, 0x00, 0x40, 0x00}; // 924.25 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP43_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP43_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP44_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0E, 0x60, 0x00, 0x40, 0x00}; // 924.75 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP44_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP44_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP45_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0E, 0xA0, 0x00, 0x40, 0x00}; // 925.25 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP45_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP45_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP46_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0E, 0xE0, 0x00, 0x40, 0x00}; // 925.75 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP46_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP46_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP47_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0F, 0x20, 0x00, 0x40, 0x00}; // 926.25 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP47_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP47_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP48_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0F, 0x60, 0x00, 0x40, 0x00}; // 926.75 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP48_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP48_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP49_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0F, 0xA0, 0x00, 0x40, 0x00}; // 927.25 mhz
  //const uint8_t RF_MODEM_AFC_LIMITER_HOP49_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
  const uint8_t RF_MODEM_AFC_LIMITER_HOP49_3[] = {0x11, 0x20, 0x03, 0x30, 0x1D, 0x59, 0xE0};
#endif
/* tabulate avaialble ctrl rf bands (hopping channels) */
#if false
const uint8_t *chtbl_ctrl_rdo[/*14*2*/] = {
	RF_FREQ_CONTROL_INTE_BAND0_6,
	 RF_MODEM_AFC_LIMITER_BAND0_3,
	RF_FREQ_CONTROL_INTE_BAND13_6,
	 RF_MODEM_AFC_LIMITER_BAND13_3,
} ;
#else  // building frequency hopping table, 500 khz channel spacing
const uint8_t *chtbl_ctrl_rdo[/*50*2*/] = {
	RF_FREQ_CONTROL_INTE_HOP0_6,
	 RF_MODEM_AFC_LIMITER_HOP0_3,
	RF_FREQ_CONTROL_INTE_HOP1_6,
	 RF_MODEM_AFC_LIMITER_HOP1_3,
	RF_FREQ_CONTROL_INTE_HOP2_6,
	 RF_MODEM_AFC_LIMITER_HOP2_3,
	RF_FREQ_CONTROL_INTE_HOP3_6,
	 RF_MODEM_AFC_LIMITER_HOP3_3,
	RF_FREQ_CONTROL_INTE_HOP4_6,
	 RF_MODEM_AFC_LIMITER_HOP4_3,
	RF_FREQ_CONTROL_INTE_HOP5_6,
	 RF_MODEM_AFC_LIMITER_HOP5_3,
	RF_FREQ_CONTROL_INTE_HOP6_6,
	 RF_MODEM_AFC_LIMITER_HOP6_3,
	RF_FREQ_CONTROL_INTE_HOP7_6,
	 RF_MODEM_AFC_LIMITER_HOP7_3,
	RF_FREQ_CONTROL_INTE_HOP8_6,
	 RF_MODEM_AFC_LIMITER_HOP8_3,
	RF_FREQ_CONTROL_INTE_HOP9_6,
	 RF_MODEM_AFC_LIMITER_HOP9_3,
	RF_FREQ_CONTROL_INTE_HOP10_6,
	 RF_MODEM_AFC_LIMITER_HOP10_3,
	RF_FREQ_CONTROL_INTE_HOP11_6,
	 RF_MODEM_AFC_LIMITER_HOP11_3,
	RF_FREQ_CONTROL_INTE_HOP12_6,
	 RF_MODEM_AFC_LIMITER_HOP12_3,
	RF_FREQ_CONTROL_INTE_HOP13_6,
	 RF_MODEM_AFC_LIMITER_HOP13_3,
	RF_FREQ_CONTROL_INTE_HOP14_6,
	 RF_MODEM_AFC_LIMITER_HOP14_3,
	RF_FREQ_CONTROL_INTE_HOP15_6,
	 RF_MODEM_AFC_LIMITER_HOP15_3,
	RF_FREQ_CONTROL_INTE_HOP16_6,
	 RF_MODEM_AFC_LIMITER_HOP16_3,
	RF_FREQ_CONTROL_INTE_HOP17_6,
	 RF_MODEM_AFC_LIMITER_HOP17_3,
	RF_FREQ_CONTROL_INTE_HOP18_6,
	 RF_MODEM_AFC_LIMITER_HOP18_3,
	RF_FREQ_CONTROL_INTE_HOP19_6,
	 RF_MODEM_AFC_LIMITER_HOP19_3,
	RF_FREQ_CONTROL_INTE_HOP20_6,
	 RF_MODEM_AFC_LIMITER_HOP20_3,
	RF_FREQ_CONTROL_INTE_HOP21_6,
	 RF_MODEM_AFC_LIMITER_HOP21_3,
	RF_FREQ_CONTROL_INTE_HOP22_6,
	 RF_MODEM_AFC_LIMITER_HOP22_3,
	RF_FREQ_CONTROL_INTE_HOP23_6,
	 RF_MODEM_AFC_LIMITER_HOP23_3,
	RF_FREQ_CONTROL_INTE_HOP24_6,
	 RF_MODEM_AFC_LIMITER_HOP24_3,
	RF_FREQ_CONTROL_INTE_HOP25_6,
	 RF_MODEM_AFC_LIMITER_HOP25_3,
	RF_FREQ_CONTROL_INTE_HOP26_6,
	 RF_MODEM_AFC_LIMITER_HOP26_3,
	RF_FREQ_CONTROL_INTE_HOP27_6,
	 RF_MODEM_AFC_LIMITER_HOP27_3,
	RF_FREQ_CONTROL_INTE_HOP28_6,
	 RF_MODEM_AFC_LIMITER_HOP28_3,
	RF_FREQ_CONTROL_INTE_HOP29_6,
	 RF_MODEM_AFC_LIMITER_HOP29_3,
	RF_FREQ_CONTROL_INTE_HOP30_6,
	 RF_MODEM_AFC_LIMITER_HOP30_3,
	RF_FREQ_CONTROL_INTE_HOP31_6,
	 RF_MODEM_AFC_LIMITER_HOP31_3,
	RF_FREQ_CONTROL_INTE_HOP32_6,
	 RF_MODEM_AFC_LIMITER_HOP32_3,
	RF_FREQ_CONTROL_INTE_HOP33_6,
	 RF_MODEM_AFC_LIMITER_HOP33_3,
	RF_FREQ_CONTROL_INTE_HOP34_6,
	 RF_MODEM_AFC_LIMITER_HOP34_3,
	RF_FREQ_CONTROL_INTE_HOP35_6,
	 RF_MODEM_AFC_LIMITER_HOP35_3,
	RF_FREQ_CONTROL_INTE_HOP36_6,
	 RF_MODEM_AFC_LIMITER_HOP36_3,
	RF_FREQ_CONTROL_INTE_HOP37_6,
	 RF_MODEM_AFC_LIMITER_HOP37_3,
	RF_FREQ_CONTROL_INTE_HOP38_6,
	 RF_MODEM_AFC_LIMITER_HOP38_3,
	RF_FREQ_CONTROL_INTE_HOP39_6,
	 RF_MODEM_AFC_LIMITER_HOP39_3,
	RF_FREQ_CONTROL_INTE_HOP40_6,
	 RF_MODEM_AFC_LIMITER_HOP40_3,
	RF_FREQ_CONTROL_INTE_HOP41_6,
	 RF_MODEM_AFC_LIMITER_HOP41_3,
	RF_FREQ_CONTROL_INTE_HOP42_6,
	 RF_MODEM_AFC_LIMITER_HOP42_3,
	RF_FREQ_CONTROL_INTE_HOP43_6,
	 RF_MODEM_AFC_LIMITER_HOP43_3,
	RF_FREQ_CONTROL_INTE_HOP44_6,
	 RF_MODEM_AFC_LIMITER_HOP44_3,
	RF_FREQ_CONTROL_INTE_HOP45_6,
	 RF_MODEM_AFC_LIMITER_HOP45_3,
	RF_FREQ_CONTROL_INTE_HOP46_6,
	 RF_MODEM_AFC_LIMITER_HOP46_3,
	RF_FREQ_CONTROL_INTE_HOP47_6,
	 RF_MODEM_AFC_LIMITER_HOP47_3,
	RF_FREQ_CONTROL_INTE_HOP48_6,
	 RF_MODEM_AFC_LIMITER_HOP48_3,
	RF_FREQ_CONTROL_INTE_HOP49_6,
	 RF_MODEM_AFC_LIMITER_HOP49_3,
} ;
#endif
U8 ctrl_band_select(U8 len, U8 *ch_param) {
	if (!USE_915MHZ){
		return 0;
	}
	if (RADIO_CHSEL_LEN0 == len && (0x11 != ch_param[0] || 0x40 != ch_param[1] || 0x0 != ch_param[3]))
	{
		/* unexpected command params */
		return SI446X_COMMAND_ERROR;
	}
	else if (RADIO_CHSEL_LEN1 == len && (0x11 != ch_param[0] || 0x20 != ch_param[1] || 0x30 != ch_param[3]))
	{
		/* unexpected command params */
		return SI446X_COMMAND_ERROR;
	}
#if false // channel filters define signal bandwidth so only pertain to data rate
	if (RADIO_CHSEL_LEN2 == len || RADIO_CHSEL_LEN3 == len || RADIO_CHSEL_LEN4 == len) {
		if ((0x11 != ch_param[0] || 0x21 != ch_param[1] || 0x00 != ch_param[3]) &&
			(0x11 != ch_param[0] || 0x21 != ch_param[1] || 0x0c != ch_param[3]) &&
			(0x11 != ch_param[0] || 0x21 != ch_param[1] || 0x18 != ch_param[3]))
		{
			/* unexpected command params */
			return SI446X_COMMAND_ERROR;
		}
	}
#endif

	if (radio_comm_SendCmdGetResp(len, ch_param, 0, 0) != 0xFF)
	{
		// Timeout occured
	   return SI446X_CTS_TIMEOUT;
	}

	return SI446X_SUCCESS;
}

const char temp_cap_table[/*56*/] = {
	5, 4, 4, 4, 4, 4, 4,
	3, 3, 3, 3, 3, 3, 2,
	2, 2, 2, 2, 2, 1, 1,
	1, 1, 1, 1, 0, 0, 0,
	0, 0, -1, -1, -1, -1, -1,
	-1, -2, -2, -2, -2, -2, -2,
	-3, -3, -3, -3, -3, -3, -4,
	-4, -4, -4, -4, -5, -5, -5,
} ;

const char hopping_patterns[CHTBL_SIZE] = {
	0, 5, 1, 6, 2, 7, 3, 8, 4, 9,
	10, 15, 11, 16, 12, 17, 13, 18, 14, 19,
	20, 25, 21, 26, 22, 27, 23, 28, 24, 29,
	30, 35, 31, 36, 32, 37, 33, 38, 34, 39,
	40, 45, 41, 46, 42, 47, 43, 48, 44, 49,
} ;


void ctrl_hop_global_update(bool listen) {
	#if HOP_2CH_ENABLE
	if (0 != ctrl_band_select(RF_FREQ_CONTROL_INTE_LEN, chtbl_ctrl_rdo[fhop_offset*2])) {
	#else // accommodate further frequency shift algorithm too, liyenho
	uint32_t frac=0, fshf = fhop_base * FREQ_SHIFT_STEP;
	uint8_t intr, *f=&frac, fctrl_str[RF_FREQ_CONTROL_INTE_LEN];
	memcpy(fctrl_str, chtbl_ctrl_rdo[fhop_offset*2], sizeof(fctrl_str));
	intr = *(fctrl_str + FREQ_INTR_POS);
	*(f+2) = *(fctrl_str + FREQ_FRAC_POS);
	*(f+1) = *(fctrl_str + FREQ_FRAC_POS+1);
	*(f) = *(fctrl_str + FREQ_FRAC_POS+2);
	frac = frac + (float)fshf * FREQ_CTRL_FACTOR;
	if (0x100000 <= frac) {
		frac -= 0x100000;
		intr += 0x1;
	}
	// update frequency control string with new settings
	*(fctrl_str + FREQ_FRAC_POS) = *(f+2);
	*(fctrl_str + FREQ_FRAC_POS+1) = *(f+1);
	*(fctrl_str + FREQ_FRAC_POS+2) = *(f);
	*(fctrl_str + FREQ_INTR_POS) = intr ;
	if (0 != ctrl_band_select(RF_FREQ_CONTROL_INTE_LEN, fctrl_str)) {
	#endif
		while (1) {
			; // Capture error
		}
	} // update integral part of frequency control
	if (0 != ctrl_band_select(RF_MODEM_AFC_LIMITER_LEN, chtbl_ctrl_rdo[fhop_offset*2+1])) {
		while (1) {
			; // Capture error
		}
	} // update AFC tracking limiter range
	fhop_offset = hop_chn_sel(fhop_offset);
	if (listen) {
		if (!USE_915MHZ){
			vRadio_StartRX(control_channel,	RADIO_LONG_PKT_LEN);
		}else{
			vRadio_StartRX(control_channel, RADIO_PKT_LEN);
		}
	}
}

int hop_chn_sel(int offset) {
	  if (fhopless) {
		  switch(fhopless) {
			  case 1/*low*/: return 0;
			  case 2/*mid*/: return (CHTBL_SIZE)/2-1;
			  case 3/*high*/: return (CHTBL_SIZE)-1;
			  default: /* Capture error */
					while (1) {
						;
					}
		  }
	  }
#if HOP_2CH_ENABLE
		  //debug testing
	 int curr_oft = WRAP_OFFSET(HOP_2CH_OFFSET0+fhop_base);
		  if(offset==curr_oft )
		  	offset=WRAP_OFFSET(HOP_2CH_OFFSET1+fhop_base);
		  else /*(offset==(HOP_2CH_OFFSET1+fhop_base)||offset==(HOP_2CH_OFFSET0))*/
		  	offset = WRAP_OFFSET(HOP_2CH_OFFSET0+fhop_base);
#else
  extern const char hopping_patterns[] ;
	  	offset = hopping_patterns[fhop_idx++] ;
	  	fhop_idx = (CHTBL_SIZE>fhop_idx)? fhop_idx: 0;
#endif
	  	return offset;
}

extern void erase_last_sector(void);
extern uint8_t get_si446x_temp(void);

void cap_bank_calibrate(void) {
	static uint8_t tune_cap_str[] = {RF_GLOBAL_XO_TUNE_2};
	uint32_t tdel, tcurr;
		 if (si4463_factory_tune.calib_gated) {
			tcurr = *DWT_CYCCNT;
			tm_delta(si4463_factory_tune.tm_curr, tcurr, tdel)
			if (CALIB_DWELL_INTV<=tdel) {
				if (0x7f==si4463_factory_tune.cap_curr) {
					irqflags_t flags;
					flags = cpu_irq_save();
					  si4463_factory_tune.calib_gated = false;
					cpu_irq_restore(flags);
					si4463_factory_tune.tm_ended = tcurr;
					si4463_factory_tune.median =
						(si4463_factory_tune.lower+
						si4463_factory_tune.upper+1) / 2;
	#ifdef CONFIG_ON_FLASH
					erase_last_sector();
					CHECKED_FLASH_WR(
						IFLASH_ADDR + IFLASH_SIZE-sizeof(backup),
						backup, NUM_OF_FPGA_REGS +1 +4)
					CHECKED_FLASH_WR(ul_page_addr_ctune,
																			&si4463_factory_tune.median,
																			1/*1 byte flag*/)
					uint8_t ctemp = get_si446x_temp();
					CHECKED_FLASH_WR(ul_page_addr_mtemp,
																			&ctemp, 1/*1 byte flag*/)
					CHECKED_FLASH_WR(
						IFLASH_ADDR + IFLASH_SIZE-NUM_OF_ATMEL_REGS +2,
						backup +NUM_OF_FPGA_REGS +1 +4 +2,
						sizeof(backup)-NUM_OF_FPGA_REGS-1 -4 -2)
	#endif
					tune_cap_str[CAP_VAL_POS] = si4463_factory_tune.median;
					if (radio_comm_SendCmdGetResp(sizeof(tune_cap_str), tune_cap_str, 0, 0) != 0xFF) {
						while (1) {
							; // Capture error
						}
					}
			  #ifdef RADIO_CTRL_AUTO
			  		ctrl_tdma_lock = false;
					fhop_in_search = true;
					fhop_flag = false ;
					fhop_base = 0;
					fhop_offset = HOP_2CH_ENABLE?WRAP_OFFSET(HOP_2CH_OFFSET0):0;
					if (fhopless)
					  switch(fhopless) {
						  case 0/*low*/: fhop_offset = 0;
						  case 2/*mid*/: fhop_offset = (CHTBL_SIZE)/2-1;
						  case 3/*high*/: fhop_offset = (CHTBL_SIZE)-1;
					  }
					ctrl_hop_global_update(true);
				#endif
					si4463_factory_tune.tm_curr = *DWT_CYCCNT; // record startup time for recurrent adjustment
					ctrl_tdma_enable = true;	// turn flag back on
					/*goto tune_done;*/ return;
				}
				si4463_factory_tune.tm_curr = tcurr;
				if (si4463_factory_tune.calib_det_rx) {
					if (CAP_TUNE_THR<si4463_factory_tune.calib_det_rx) {
						minmax(si4463_factory_tune.lower,
										si4463_factory_tune.upper,
										si4463_factory_tune.cap_curr)
					}
					si4463_factory_tune.calib_det_rx = 0; // invalidated
				}
				si4463_factory_tune.cap_curr += 1;
				tune_cap_str[CAP_VAL_POS] = si4463_factory_tune.cap_curr;
				if (radio_comm_SendCmdGetResp(sizeof(tune_cap_str), tune_cap_str, 0, 0) != 0xFF) {
					while (1) {
						; // Capture error
					}
				}
			}
		 }
		 else if (si4463_factory_tune.calib_req_h) {
			 const uint8_t freq_reset_str[] = {RF_FREQ_CONTROL_INTE_8_915}; // tune frequency on 915 mhz
			if (radio_comm_SendCmdGetResp(sizeof(freq_reset_str), freq_reset_str, 0, 0) != 0xFF) {
				while (1) {
					; // Capture error
				}
			}
			const uint8_t afc_gear_str[] = {0x11, 0x20, 0x03, 0x30, 0x25, 0x16, 0xC0}; // use narrower tracking range on AFC limiter
			if (radio_comm_SendCmdGetResp(sizeof(afc_gear_str), afc_gear_str, 0, 0) != 0xFF) {
				while (1) {
					; // Capture error
				}
			}
			si4463_factory_tune.tm_started = *DWT_CYCCNT;
			si4463_factory_tune.tm_curr = si4463_factory_tune.tm_started;
			si4463_factory_tune.calib_req_h = false;
			si4463_factory_tune.calib_det_rx = 0; // invalidated
			si4463_factory_tune.cap_curr = 0x0; // start from lowest possible cap value
			tune_cap_str[CAP_VAL_POS] = si4463_factory_tune.cap_curr;
			if (radio_comm_SendCmdGetResp(sizeof(tune_cap_str), tune_cap_str, 0, 0) != 0xFF) {
				while (1) {
					; // Capture error
				}
			}
		  	vRadio_StartRX(pRadioConfiguration->Radio_ChannelNumber,
		  		pRadioConfiguration->Radio_PacketLength);  // enter listening mode
			si4463_factory_tune.calib_gated = true;  // let si4463_radio_handler() begin to receive
		 }
//tune_done:
}
#ifdef CTRL_DYNAMIC_MOD
  void process_range_mode(uint32_t tick_curr,uint32_t tick_prev) {
		if (SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_PACKET_RX_PEND_BIT==bMain_IT_Status_m) {
	  		irqflags_t flags;
			flags = cpu_irq_save();
		      bMain_IT_Status_m = 0;  // reset nirq flag
			cpu_irq_restore(flags);
			//just got an RX packet from radio, bRadio_Check_Tx_RX() already read the fifo
			static BW_CTRL prev_ctrl_bits = (BW_CTRL) -1;
			/************************************************************/
			{	static unsigned int recv_cnt = 0;
				BW_CTRL ctrl_bits_tmp;
				uint8_t j, err, cks=0, *pb = (uint8_t*)gp_rdo_rpacket_l;
				for (j=0;j<pRadioConfiguration->Radio_PacketLength-1;j++)
				{	cks ^= *pb++;  } // generate 2's mod checksum
				cmd_ctrl_bits = CTRL0_MSK & *pb; // fetch range cmd from rx side
				cks &= CTRL0_IMSK;  // took high 6 bits
				cks ^= CTRL_MSK & (*pb<<CTRL_BITS); // then bw ctrl bits @ end
				err = (CHKSM_MSK & *pb) ^ cks;
				uint32_t ew;
				ew = error_weight_cks6b[err>>CTRL_BITS];
				//lapse = recv_cnt * LOOP_LATENCY;
				// update error accum per time lapse
				if (initCnt > recv_cnt) {
					r4463_sts.errPerAcc = r4463_sts.errPerAcc * Ewim + ew * Ewis;
					r4463_sts.errPerAcc >>= initScl;
				}
				else if (initCnt <= recv_cnt && inteCnt > recv_cnt) {
					r4463_sts.errPerAcc = r4463_sts.errPerAcc * Ewmm + ew * Ewms;
					r4463_sts.errPerAcc >>= inteScl;
				}
				else {
					r4463_sts.errPerAcc = r4463_sts.errPerAcc * Ewnm + ew * Ewns;
					r4463_sts.errPerAcc >>= normScl;
				}
				// compute range estimation
				if (shThr >= r4463_sts.errPerAcc) {
						if (shThr >= ew)
							ctrl_bits_tmp = SHORT_RNG;
						else
							ctrl_bits_tmp = NEUTRAL;
					}
				else if (lgThr < r4463_sts.errPerAcc) {
					ctrl_bits_tmp = LONG_RNG;
				}
				else {
					if (lgThr < ew)
						ctrl_bits_tmp = LONG_RNG;
					else
						ctrl_bits_tmp = NEUTRAL;
				}
				// perform hangover procedure
				switch(ctrl_bits_tmp) {
					case SHORT_RNG :
						for (j=0; j<CTRL_CTX_LEN; j++) {
							if (SHORT_RNG != r4463_sts.ctrl_bits_ctx[j])
								break;
						}
						if (CTRL_CTX_LEN == j)
							r4463_sts.bw_ctrl_bits = SHORT_RNG;
						else  { // in hysteresis region
							for (j=0; j<CTRL_CTX_LEN; j++) {
								if (NEUTRAL != r4463_sts.ctrl_bits_ctx[j])
									break;
							}
							if (CTRL_CTX_LEN == j)
								r4463_sts.bw_ctrl_bits = SHORT_RNG;
							else  // cond above shall break continuous neutral case...
							r4463_sts.bw_ctrl_bits = NEUTRAL;
						}
						break;
					case LONG_RNG :
						for (j=0; j<CTRL_CTX_LEN; j++) {
							if (LONG_RNG == r4463_sts.ctrl_bits_ctx[j])
								break; // found a LG req in ctx
						}
						if (CTRL_CTX_LEN != j)
							r4463_sts.bw_ctrl_bits = LONG_RNG;
						else // close in hysteresis region
							r4463_sts.bw_ctrl_bits = NEUTRAL;
						break;
					default : /*NEUTRAL*/
						r4463_sts.bw_ctrl_bits = NEUTRAL;
							if (NEUTRAL == r4463_sts.ctrl_bits_ctx[0] &&
								NEUTRAL == r4463_sts.ctrl_bits_ctx[1]) {
								// fall back to long range mode if two neutral seen in a row
								r4463_sts.bw_ctrl_bits = LONG_RNG;
								break;
							}
						break;
				}
				// bump up recv_cnt
				recv_cnt = recv_cnt + 1;
			}
			/********************************************************/
			uint8_t cks = 0, *pb = (uint8_t*)gp_rdo_tpacket_l;
			for (int n=0;n<pRadioConfiguration->Radio_PacketLength-1;n++)
				cks ^= *pb++;  // generate 2's mod checksum
			cks &= CTRL0_IMSK;  // took high 6 bits
			/********************************************************/
			cks ^= r4463_sts.bw_ctrl_bits<<CTRL_BITS; // included bw ctrl bits
			*pb = (cks | r4463_sts.bw_ctrl_bits);  // insert ctrl byte @ end
			/********************************************************/
			if (prev_ctrl_bits != cmd_ctrl_bits && NEUTRAL != cmd_ctrl_bits) {
					// normal ops mode
					if (0 != range_mode_configure(SHORT_RNG!=cmd_ctrl_bits)) {
						// puts("error from range_mode_configure()");
						/*goto blink_led;*/ return; // time out, don't proceed
					}
			}
			/********************************************************/
			prev_ctrl_bits = cmd_ctrl_bits;
		//slot based ctrl
  	} else { // not receiving anything yet
		  unsigned int lapse;
			tick_curr = *DWT_CYCCNT;
			if (tick_curr < r4463_sts.tick_prev) {
			 	lapse+=tick_curr+(0xffffffff-r4463_sts.tick_prev);
			}
			else // not wrapped yet
			 	lapse+=tick_curr -r4463_sts.tick_prev;
			r4463_sts.tick_prev = tick_curr;
			if (LONG_RNG!=r4463_sts.bw_ctrl_bits && CTRL_MON_PERIOD <lapse) {
				r4463_sts.bw_ctrl_bits = NEUTRAL;
				// no need to force cmd_ctrl_bits updated yet for next run
				if (CTRL_FAIL_PERIOD <lapse) {
					r4463_sts.bw_ctrl_bits = LONG_RNG;
					cmd_ctrl_bits = 	LONG_RNG;  // forced into long range mode
					if (0 != range_mode_configure(LONG_RNG)) {
						// puts("error from range_mode_configure()");
						/*goto blink_led;*/ return; // time out, don't proceed
					}
				}
			}
		}// not receiving anything yet
		// update ctrl bits history
		for (int n=CTRL_CTX_LEN-1; n>0; n--)
			r4463_sts.ctrl_bits_ctx[n] = r4463_sts.ctrl_bits_ctx[n-1];
		*r4463_sts.ctrl_bits_ctx = r4463_sts.bw_ctrl_bits;
}
#endif //CTRL_DYNAMIC_MOD
