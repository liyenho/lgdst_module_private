/*! @file radio.c
 * @brief This file contains functions to interface with the radio chip.
 *
 * @b COPYRIGHT
 * @n Silicon Laboratories Confidential
 * @n Copyright 2012 Silicon Laboratories, Inc.
 * @n http://www.silabs.com
 */
#include "asf.h"
#include "delay.h"
#define INCLUDEINRADIO
#include "bsp.h"
#include "lgdst_4463_spi.h"

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
#define CTRL_MON_PERIOD	300  // in msec
#define CTRL_FAIL_PERIOD	(5* CTRL_MON_PERIOD) // can't be too short in order to prevent spi comm lockup
#define CTRL0_MSK			(-1+(1<<CTRL_BITS))
#define CTRL0_IMSK			(0xff & ~CTRL0_MSK)
#define CTRL_MSK				(CTRL0_MSK<<CTRL_BITS)
#define CHKSM_MSK		(0xff ^ CTRL0_MSK)

#define CTRL_RX_LOOP_LATENCY		20 // msec, to be measured??? liyenho
// 4463 stats mon obj
volatile ctrl_radio_stats  r4463_sts;
/*******************************************************************/

extern uint32_t wrptr_rdo_rpacket;
extern uint32_t rdptr_rdo_rpacket;
extern uint32_t ul_page_addr_ctune, ul_page_addr_mtemp;
extern volatile capv_tune_t si4463_factory_tune;
extern volatile bool ctrl_tdma_enable;
extern volatile uint32_t *DWT_CYCCNT;
extern uint8_t backup[NUM_OF_FPGA_REGS+NUM_OF_ATMEL_REGS+4+1];

extern uint8_t get_si446x_temp();
extern void erase_last_sector();

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
  U8 failcnt=0;

  /* Power Up the radio chip */
  vRadio_PowerUp();

  /* Load radio configuration */
  while (SI446X_SUCCESS != si446x_configuration_init(pRadioConfiguration->Radio_ConfigurationArray))
  {

	delay_ms(10);
    /* Power Up the radio chip */
    vRadio_PowerUp();
	failcnt++;
  }

  // Read ITs, clear pending ones
  si446x_get_int_status(0u, 0u, 0u);

  // hardwired calibration on RSSI compensation
  if (radio_comm_SendCmdGetResp(sizeof(rssi_comp_offset), rssi_comp_offset, 0, 0) != 0xFF) {
		failcnt ++;
  }
  return(failcnt);
}

/*!
 *  Check if Packet sent IT flag or Packet Received IT is pending.
 *
 *  @return   SI4455_CMD_GET_INT_STATUS_REP_PACKET_SENT_PEND_BIT / SI4455_CMD_GET_INT_STATUS_REP_PACKET_RX_PEND_BIT
 *
 *  @note
 *
 */
U8 bRadio_Check_Tx_RX(void)
{
  if (SBIT_R(RF4463_NIRQ) == FALSE)
  {
      /* Read ITs, clear pending ones */
      si446x_get_int_status(0u, 0u, 0u);

	  if (Si446xCmd.GET_INT_STATUS.CHIP_PEND & SI446X_CMD_GET_CHIP_STATUS_REP_CHIP_PEND_CMD_ERROR_PEND_BIT)
      {
      	/* State change to */
      	si446x_change_state(SI446X_CMD_CHANGE_STATE_ARG_NEXT_STATE1_NEW_STATE_ENUM_SLEEP);

	  	/* Reset FIFO */
      	si446x_fifo_info(SI446X_CMD_FIFO_INFO_ARG_FIFO_RX_BIT);

	  	/* State change to */
        si446x_change_state(SI446X_CMD_CHANGE_STATE_ARG_NEXT_STATE1_NEW_STATE_ENUM_RX);
      }

      if(Si446xCmd.GET_INT_STATUS.PH_PEND & SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_PACKET_SENT_PEND_BIT)
      {
#if false
			// it's ok not to take any action, liyenho
        ; //return SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_PACKET_SENT_PEND_BIT;
#else  // for debug now
			return SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_PACKET_SENT_PEND_BIT;
#endif
      }

      if(Si446xCmd.GET_INT_STATUS.PH_PEND & SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_PACKET_RX_PEND_BIT)
      {
        /* Packet RX */
        /* Get payload length */
        si446x_fifo_info(0x00);
        si446x_read_rx_fifo(RADIO_PKT_LEN, &gp_rdo_rpacket_l[0]);
        if(Si446xCmd.FIFO_INFO.RX_FIFO_COUNT > RADIO_PKT_LEN)
        { //flush extra fifo. Must do this to prevent gp_rdo_rpacket buffer overwrite
          /* Reset FIFO */
          si446x_fifo_info(SI446X_CMD_FIFO_INFO_ARG_FIFO_RX_BIT);
        }
        return SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_PACKET_RX_PEND_BIT;
      }

	  if (Si446xCmd.GET_INT_STATUS.PH_PEND & SI446X_CMD_GET_INT_STATUS_REP_PH_STATUS_CRC_ERROR_BIT)
      {
      	/* Reset FIFO */
      	si446x_fifo_info(SI446X_CMD_FIFO_INFO_ARG_FIFO_RX_BIT);
      }

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
void vRadio_StartRX(U8 channel, U8 packetLenght )
{
  // Read ITs, clear pending ones
  si446x_get_int_status(0u, 0u, 0u);

   // Reset the Rx Fifo
   si446x_fifo_info(SI446X_CMD_FIFO_INFO_ARG_FIFO_RX_BIT);

  /* Start Receiving packet, channel 0, START immediately, Packet length used or not according to packetLength */
  si446x_start_rx(channel, 0u, packetLenght,
                  SI446X_CMD_START_RX_ARG_NEXT_STATE1_RXTIMEOUT_STATE_ENUM_NOCHANGE,
                  SI446X_CMD_START_RX_ARG_NEXT_STATE2_RXVALID_STATE_ENUM_READY,
                  SI446X_CMD_START_RX_ARG_NEXT_STATE3_RXINVALID_STATE_ENUM_RX );
}

/*!
 *  Set Radio to TX mode, variable packet length.
 *
 *  @param channel Freq. Channel, Packet to be sent length of of the packet sent to TXFIFO
 *
 *  @note
 *
 */
void vRadio_StartTx_Variable_Packet(U8 channel, U8 *pioRadioPacket, U8 length)
{
  /* Leave RX state */
  si446x_change_state(SI446X_CMD_CHANGE_STATE_ARG_NEXT_STATE1_NEW_STATE_ENUM_READY);

  /* Read ITs, clear pending ones */
  si446x_get_int_status(0u, 0u, 0u);

  /* Reset the Tx Fifo */
  si446x_fifo_info(SI446X_CMD_FIFO_INFO_ARG_FIFO_TX_BIT);

  /* Fill the TX fifo with datas */
  si446x_write_tx_fifo(length, pioRadioPacket);

  /* Start sending packet, channel 0, START immediately */
  // condition, 0x80, instruct si4463 switch to rx state once finish tx
   si446x_start_tx(channel, 0x80, length);

}

void vRadio_StartTx(U8 channel)
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
#else  // building frequency hopping table, 500 khz channel spacing
const uint8_t RF_FREQ_CONTROL_INTE_HOP0_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0B, 0x60, 0x00, 0x40, 0x00}; // 902.75 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP0_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP1_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0B, 0xA0, 0x00, 0x40, 0x00}; // 903.25 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP1_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP2_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0B, 0xE0, 0x00, 0x40, 0x00}; // 903.75 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP2_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP3_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0C, 0x20, 0x00, 0x40, 0x00}; // 904.25 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP3_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP4_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0C, 0x60, 0x00, 0x40, 0x00}; // 904.75 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP4_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP5_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0C, 0xA0, 0x00, 0x40, 0x00}; // 905.25 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP5_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP6_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0C, 0xE0, 0x00, 0x40, 0x00}; // 905.75 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP6_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP7_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0D, 0x20, 0x00, 0x40, 0x00}; // 906.25 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP7_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP8_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0D, 0x60, 0x00, 0x40, 0x00}; // 906.75 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP8_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP9_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0D, 0xA0, 0x00, 0x40, 0x00}; // 907.25 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP9_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP10_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0D, 0xE0, 0x00, 0x40, 0x00}; // 907.75 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP10_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP11_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0E, 0x20, 0x00, 0x40, 0x00}; // 908.25 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP11_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP12_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0E, 0x60, 0x00, 0x40, 0x00}; // 908.75 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP12_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP13_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0E, 0xA0, 0x00, 0x40, 0x00}; // 909.25 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP13_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP14_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0E, 0xE0, 0x00, 0x40, 0x00}; // 909.75 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP14_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP15_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0F, 0x20, 0x00, 0x40, 0x00}; // 910.25 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP15_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP16_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0F, 0x60, 0x00, 0x40, 0x00}; // 910.75 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP16_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP17_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0F, 0xA0, 0x00, 0x40, 0x00}; // 911.25 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP17_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP18_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0F, 0xE0, 0x00, 0x40, 0x00}; // 911.75 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP18_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP19_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x08, 0x20, 0x00, 0x40, 0x00}; // 912.25 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP19_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP20_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x08, 0x60, 0x00, 0x40, 0x00}; // 912.75 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP20_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP21_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x08, 0xA0, 0x00, 0x40, 0x00}; // 913.25 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP21_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP22_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x08, 0xE0, 0x00, 0x40, 0x00}; // 913.75 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP22_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP23_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x09, 0x20, 0x00, 0x40, 0x00}; // 914.25 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP23_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP24_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x09, 0x60, 0x00, 0x40, 0x00}; // 914.75 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP24_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP25_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x09, 0xA0, 0x00, 0x40, 0x00}; // 915.25 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP25_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP26_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x09, 0xE0, 0x00, 0x40, 0x00}; // 915.75 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP26_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP27_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0A, 0x20, 0x00, 0x40, 0x00}; // 916.25 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP27_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP28_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0A, 0x60, 0x00, 0x40, 0x00}; // 916.75 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP28_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP29_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0A, 0xA0, 0x00, 0x40, 0x00}; // 917.25 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP29_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP30_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0A, 0xE0, 0x00, 0x40, 0x00}; // 917.75 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP30_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP31_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0B, 0x20, 0x00, 0x40, 0x00}; // 918.25 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP31_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP32_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0B, 0x60, 0x00, 0x40, 0x00}; // 918.75 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP32_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP33_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0B, 0xA0, 0x00, 0x40, 0x00}; // 919.25 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP33_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP34_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0B, 0xE0, 0x00, 0x40, 0x00}; // 919.75 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP34_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP35_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0C, 0x20, 0x00, 0x40, 0x00}; // 920.25 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP35_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP36_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0C, 0x60, 0x00, 0x40, 0x00}; // 920.75 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP36_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP37_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0C, 0xA0, 0x00, 0x40, 0x00}; // 921.25 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP37_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP38_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0C, 0xE0, 0x00, 0x40, 0x00}; // 921.75 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP38_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP39_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0D, 0x20, 0x00, 0x40, 0x00}; // 922.25 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP39_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP40_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0D, 0x60, 0x00, 0x40, 0x00}; // 922.75 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP40_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP41_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0D, 0xA0, 0x00, 0x40, 0x00}; // 923.25 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP41_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP42_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0D, 0xE0, 0x00, 0x40, 0x00}; // 923.75 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP42_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP43_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0E, 0x20, 0x00, 0x40, 0x00}; // 924.25 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP43_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP44_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0E, 0x60, 0x00, 0x40, 0x00}; // 924.75 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP44_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP45_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0E, 0xA0, 0x00, 0x40, 0x00}; // 925.25 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP45_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP46_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0E, 0xE0, 0x00, 0x40, 0x00}; // 925.75 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP46_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP47_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0F, 0x20, 0x00, 0x40, 0x00}; // 926.25 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP47_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP48_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0F, 0x60, 0x00, 0x40, 0x00}; // 926.75 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP48_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
const uint8_t RF_FREQ_CONTROL_INTE_HOP49_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0F, 0xA0, 0x00, 0x40, 0x00}; // 927.25 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_HOP49_3[] = {0x11, 0x20, 0x03, 0x30, 0x02, 0xDD, 0xE0};
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

/*const*/ char temp_cap_table[/*56*/] = {
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

void cap_bank_calibrate() {
 static uint8_t tune_cap_str[] = {RF_GLOBAL_XO_TUNE_2};
 uint32_t i, n, tdel, tcurr;
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
			 const uint8_t freq_reset_str[] = {RF_FREQ_CONTROL_INTE_8}; // tune frequency on 915 mhz
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
		  		RADIO_PKT_LEN);  // enter listening mode
			si4463_factory_tune.calib_gated = true;  // let si4463_radio_handler() begin to receive
		 }
//tune_done:
}
#ifdef CTRL_DYNAMIC_MOD
  void process_range_mode(U8 bMain_IT_Status) {
	  if(bMain_IT_Status == SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_PACKET_SENT_PEND_BIT)
	  {
		  // A TX was just finished  Need to start RX
		  static BW_CTRL prev_ctrl_bits = (BW_CTRL) -1;
			/********************************************************/
			if (prev_ctrl_bits != r4463_sts.bw_ctrl_bits && NEUTRAL != r4463_sts.bw_ctrl_bits) {
				if (0 != range_mode_configure(SHORT_RNG!=r4463_sts.bw_ctrl_bits)) {
					//puts("error from range_mode_configure()");
					/*return*/ ; // time out, don't proceed
				}
			}
			/********************************************************/
			prev_ctrl_bits = r4463_sts.bw_ctrl_bits;
	  }
	  if((bMain_IT_Status == SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_PACKET_RX_PEND_BIT))
	  {
		  // An RX data was received, and an interrupt was triggered.
			/************************************************************/
			{	static unsigned int recv_cnt = 0;
				BW_CTRL ctrl_bits_tmp;
				uint8_t j, err, cks=0, *pb = (uint8_t*)gp_rdo_rpacket_l;
				for (j=0;j<pRadioConfiguration->Radio_PacketLength-1;j++)
				{	cks ^= *pb++;  } // generate 2's mod checksum
				cks &= CTRL0_IMSK;  // took high 6 bits
				cks ^= CTRL_MSK & (*pb<<CTRL_BITS); // then bw ctrl bits @ end
				err = (CHKSM_MSK & *pb) ^ cks;
				uint32_t ew;
				ew = error_weight_cks6b[err>>CTRL_BITS];
				//lapse = recv_cnt * CTRL_RX_LOOP_LATENCY;
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
					if (SHORT_RNG == (CTRL0_MSK & *pb)) {
						if (shThr >= ew)
							ctrl_bits_tmp = SHORT_RNG;
						else
							ctrl_bits_tmp = NEUTRAL;
					}
					else
						ctrl_bits_tmp = NEUTRAL;
				}
				else if (lgThr < r4463_sts.errPerAcc) {
					ctrl_bits_tmp = LONG_RNG;
				}
				else {
					if (LONG_RNG == (CTRL0_MSK & *pb))
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
			r4463_sts.loop_cnt = 0;  //reset
	  } else { // not receiving anything yet
		  unsigned int lapse = r4463_sts.loop_cnt *CTRL_RX_LOOP_LATENCY;
			if (LONG_RNG!=r4463_sts.bw_ctrl_bits && CTRL_MON_PERIOD <lapse) {
				r4463_sts.bw_ctrl_bits = NEUTRAL;
				if (CTRL_FAIL_PERIOD <lapse) {
					r4463_sts.bw_ctrl_bits = LONG_RNG;
					if (0 != range_mode_configure(LONG_RNG)) {
						// puts("error on range_mode_configure()");
						//return ; // time out, don't proceed
					}
					r4463_sts.loop_cnt = 0;
				}
			}
		}
		// update ctrl bits history
		for (int j=CTRL_CTX_LEN-1; j>0; j--)
			r4463_sts.ctrl_bits_ctx[j] = r4463_sts.ctrl_bits_ctx[j-1];
		*r4463_sts.ctrl_bits_ctx = r4463_sts.bw_ctrl_bits;
		r4463_sts.loop_cnt = r4463_sts.loop_cnt + 1;
		/************************************************************/
}
#endif //CTRL_DYNAMIC_MOD
