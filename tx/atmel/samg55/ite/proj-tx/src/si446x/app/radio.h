/*! @file radio.h
 * @brief This file is contains the public radio interface functions.
 *
 * @b COPYRIGHT
 * @n Silicon Laboratories Confidential
 * @n Copyright 2012 Silicon Laboratories, Inc.
 * @n http://www.silabs.com
 */

#ifndef RADIO_H_
#define RADIO_H_

/*****************************************************************************
 *  Global Macros & Definitions
 *****************************************************************************/
/*! Maximal packet length definition (FIFO size) */
#define RADIO_TX_FIFO_SIZE     64u
/*! Maximal configuration parameters length definition */
#define RADIO_MAX_CONFIG_LENGTH     512u
extern unsigned int *gp_rdo_rpacket_l; // radio rx buffer
#define TCTBL_SIZE			56
extern const char temp_cap_table[TCTBL_SIZE] ; // lookup table for temp/cap compensation
#define TCTBL_ACCESS(c,t)  c=(TCTBL_SIZE<=t)? temp_cap_table[TCTBL_SIZE-1]: temp_cap_table[t];
/*****************************************************************************
 *  Global Typedefs & Enums
 *****************************************************************************/
typedef struct
{
#if false
    U8   *Radio_ConfigurationArray;
#endif
    U8   Radio_ChannelNumber;
    U8   Radio_PacketLength;
    U8   Radio_State_After_Power_Up;
    U8	alignment_not_used;  // don't remove this line, liyenho
    U32  Radio_Delay_Cnt_After_Reset;
#if true
    U8   Radio_ConfigurationArray[RADIO_MAX_CONFIG_LENGTH];
#else
	U8   Radio_CustomPayload[RADIO_MAX_PACKET_LENGTH];
#endif
} tRadioConfiguration;

extern tRadioConfiguration RadioConfiguration_915;
extern tRadioConfiguration RadioConfiguration_869;

/*****************************************************************************
 *  Global Variable Declarations
 *****************************************************************************/
#ifndef INCLUDEINMAIN
extern const SEGMENT_VARIABLE_SEGMENT_POINTER(pRadioConfiguration, tRadioConfiguration, SEG_CODE, SEG_CODE);
extern SEGMENT_VARIABLE(customRadioPacket[RADIO_TX_FIFO_SIZE], U8, SEG_XDATA);

// !Si446x configuration array
extern const SEGMENT_VARIABLE(Radio_Configuration_Data_Array[], U8, SEG_CODE);
#endif

// !Si446x status monitor parameters
#define CTRL_BITS				2
#define CTRL_CTX_LEN	(5-1)
typedef enum {
	NEUTRAL = 0,
	LONG_RNG = 1,
	SHORT_RNG = 2,
	RESERVED = 3,
} BW_CTRL;
typedef struct {
	unsigned int bw_ctrl_bits;
	unsigned int ctrl_bits_ctx[CTRL_CTX_LEN];
	unsigned int errPerAcc; // checksum error accum
	unsigned int tick_prev;
} ctrl_radio_stats;

// hardwired calibration on RSSI compensation
#define RSSI_COMP_OFFSET_1	0x11, 0x20, 0x01, 0x4e, 0x19

#define RF_MODEM_DATA_RATE_LG_3		0x11, 0x20, 0x03, 0x03, 0x03, 0x0d, 0x40
#define RF_MODEM_DATA_RATE_SH_3		0x11, 0x20, 0x03, 0x03, 0x06, 0x1A, 0x80
#ifdef SI4463_CRYSTAL_32MHZ  // defined in the build tool
  #define RF_MODEM_FREQ_DEV_LG_3			 0x11, 0x20, 0x03, 0x0a, 0x00, 0x01, 0x48
  #define RF_MODEM_FREQ_DEV_SH_3			 0x11, 0x20, 0x03, 0x0a, 0x00, 0x01, 0x48
	#define RF_MODEM_TX_RAMP_DELAY_LG_8 0x11, 0x20, 0x08, 0x18, 0x01, 0x80, 0x08, 0x03, 0xC0, 0x00, 0x20, 0x20
	#define RF_MODEM_TX_RAMP_DELAY_SH_8 0x11, 0x20, 0x08, 0x18, 0x01, 0x80, 0x08, 0x03, 0xC0, 0x00, 0x20, 0x20
	#define RF_MODEM_BCR_OSR_1_LG_9 0x11, 0x20, 0x09, 0x22, 0x03, 0x20, 0x00, 0xA3, 0xD7, 0x00, 0x52, 0x02, 0xC2
	#define RF_MODEM_BCR_OSR_1_SH_9 0x11, 0x20, 0x09, 0x22, 0x01, 0x90, 0x01, 0x47, 0xAE, 0x00, 0xA4, 0x02, 0xC2
	#define RF_MODEM_AFC_GEAR_LG_7 0x11, 0x20, 0x07, 0x2C, 0x04, 0x36, 0xC0, 0x07, 0x25, 0x16, 0xC0
	#define RF_MODEM_AFC_GEAR_SH_7 0x11, 0x20, 0x07, 0x2C, 0x04, 0x36, 0xC0, 0x0E, 0x14, 0x9A, 0xC0
	#define RF_MODEM_AGC_CONTROL_LG_1 0x11, 0x20, 0x01, 0x35, 0xE2
	#define RF_MODEM_AGC_CONTROL_SH_1 0x11, 0x20, 0x01, 0x35, 0xE2
	#define RF_MODEM_AGC_WINDOW_SIZE_LG_9 0x11, 0x20, 0x09, 0x38, 0x11, 0xAF, 0xAF, 0x00, 0x1A, 0xFF, 0xFF, 0x00, 0x2B
	#define RF_MODEM_AGC_WINDOW_SIZE_SH_9 0x11, 0x20, 0x09, 0x38, 0x11, 0x58, 0x58, 0x00, 0x1A, 0x80, 0x00, 0x00, 0x2A
	#define RF_MODEM_RAW_CNT_LG_3 0x11, 0x20, 0x03, 0x45, 0x83, 0x00, 0x51
	#define RF_MODEM_RAW_CNT_SH_3 0x11, 0x20, 0x03, 0x45, 0x83, 0x00, 0x51
	#define RF_MODEM_CHFLT_RX1_CHFLT_COE13_LG_12 0x11, 0x21, 0x0C, 0x00, 0xCC, 0xA1, 0x30, 0xA0, 0x21, 0xD1, 0xB9, 0xC9, 0xEA, 0x05, 0x12, 0x11
	#define RF_MODEM_CHFLT_RX1_CHFLT_COE13_SH_12 0x11, 0x21, 0x0C, 0x00, 0xFF, 0xC4, 0x30, 0x7F, 0xF5, 0xB5, 0xB8, 0xDE, 0x05, 0x17, 0x16, 0x0C
	#define RF_MODEM_CHFLT_RX1_CHFLT_COE1_LG_12 0x11, 0x21, 0x0C, 0x0C, 0x0A, 0x04, 0x15, 0xFC, 0x03, 0x00, 0xA2, 0xA0, 0x97, 0x8A, 0x79, 0x66
	#define RF_MODEM_CHFLT_RX1_CHFLT_COE1_SH_12 0x11, 0x21, 0x0C, 0x0C, 0x03, 0x00, 0x15, 0xFF, 0x00, 0x00, 0xB4, 0xB0, 0xA5, 0x93, 0x7D, 0x65
	#define RF_MODEM_CHFLT_RX2_CHFLT_COE7_LG_12 0x11, 0x21, 0x0C, 0x18, 0x52, 0x3F, 0x2E, 0x1F, 0x14, 0x0B, 0x06, 0x02, 0x00, 0x00, 0x00, 0x00
	#define RF_MODEM_CHFLT_RX2_CHFLT_COE7_SH_12 0x11, 0x21, 0x0C, 0x18, 0x4D, 0x37, 0x25, 0x16, 0x02, 0x05, 0x02, 0xFF, 0x00, 0x00, 0x00, 0x0C
#else  // default using 30 mhz crystal
  #define RF_MODEM_FREQ_DEV_LG_3			 0x11, 0x20, 0x03, 0x0a, 0x00, 0x02, 0xbb
  #define RF_MODEM_FREQ_DEV_SH_3			 0x11, 0x20, 0x03, 0x0a, 0x00, 0x01, 0x5d
#endif

uint32_t pkt_rcv_timestamp;
uint32_t crc_err_cnt;
uint32_t hop_watchdog_intv;
uint32_t hop_watchdog_reset_cnt;
/*****************************************************************************
 *  Global Function Declarations
 *****************************************************************************/
U8  vRadio_Init(void);
U8    bRadio_Check_Tx_RX(void);
void  vRadio_StartRX(U8,U8);
void vRadio_StartRX_variableLength(U8 channel);
U8    bRadio_Check_Ezconfig(U16);
void  vRadio_StartTx(U8, U8*, U16);
void vRadio_StartTx_No_Data(U8 channel);
void vRadio_StartTx_Variable_Length(U8 channel, U8 *pioRadioPacket, U16  length);
void Set_Sync_Words(uint8_t byte1, uint8_t byte2);
bool Channel_Scan(void);
/*****************************************************************************/
U8 range_mode_configure(U8 range); // added for dynamic range configuration
U8 ctrl_band_select(U8 len, U8 *ch_param); // ctrl radio band selection
void ctrl_hop_global_update(bool listen);
int hop_chn_sel(int offset);
extern uint8_t FRR_Copy[4];

void cap_bank_calibrate(void);
//#define CTRL_DYNAMIC_MOD
 #ifdef CTRL_DYNAMIC_MOD
  void process_range_mode(uint32_t tick_curr,uint32_t tick_prev);
 #endif
#endif /* RADIO_H_ */
