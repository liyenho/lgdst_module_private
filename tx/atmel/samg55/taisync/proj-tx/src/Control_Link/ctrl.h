/*
 * ctrl.h
 *
 */

#ifndef CTRL_H_
#define CTRL_H_


#define RECEIVE_MAVLINK					1/*0*/ //received radio data is MavLink form
#define SEND_MAVLINK					1/*0*/

#define RADIO_LONG_PKT_LEN				96//long transmissions for 868 MHz


#define MilliSecToTick			120000 //120000=1ms
#define MilliSec_To_Tick(X) ((uint32_t)(X)*MilliSecToTick)

#define REAL_PKT_LEN			(RADIO_PKT_LEN+13+2+2+4) //payload + preamble + sync word + length field
#define EURO_TDMA_PERIOD		MilliSec_To_Tick(40+3*(1000*REAL_PKT_LEN*8/10000))


#define ASYMM_RATIO				/*1*/ 2
#if RECEIVE_MAVLINK
#define RADIO_PKT_LEN		40 // ctl/sts radio payload byte length +1
#else
#define RADIO_PKT_LEN		32 // ctl/sts radio payload byte length +1
#endif
#define RADIO_GRPPKT_LEN    30
#define RADIO_INFO_LEN      4 // usb pipe information post header
#define RDO_ELEMENT_SIZE   (RADIO_PKT_LEN/sizeof(uint32_t))  // RADIO_PKT_LEN must divide into sizeof(uint32_t), liyenho
#define US_TDMA_PERIOD     MilliSec_To_Tick(15+3*(1000*REAL_PKT_LEN*8/40000))
								// 30kbps 2snd1rx case--------------------------------------------
								//   actual transfer period = 26B*8*3/30000 = 20.8ms
								//   TX delivery time = 26*8*2/30000 = 13.86ms
								//   TDMA_PERIOD= 25ms = 3000000
								//   RX interrupt to TX start = TDMA_RX_TO_TX= 1368000
								//   Empirically Measured: snd26Bpkt time: 8.2ms, TDMA_TX_PERIOD = 984000
								//   TDMA_PERIOD_TOL = 96000 = +/-0.8ms (FYI:saw only 0.2ms jitter)
								//   TDMA_RX_TO_TX 120000=1ms. immediately start to transmit.
#define TDMA_TX_PERIOD     ((TDMA_PERIOD)*2/3)
#define  TDMA_BOUND	 /*5400000*/ /*4860000*/ (12*4320000) // 5400000 = 45 ms
#define TDMA_PERIOD_TOL /*96000*/ MilliSec_To_Tick(5) //(12*144000) /*192000*/ // jitter tolerance 480000=4ms 600000=5ms
#define TDMA_PERIOD_MAX ((TDMA_PERIOD)+TDMA_PERIOD_TOL)
#define TDMA_PERIOD_MIN ((TDMA_PERIOD)-TDMA_PERIOD_TOL)
#define TDMA_RX_TO_TX   (1.5*1.5*120000) /**180000**/ /*360000*/ // RX interrupt to TX startTX()
                                // 1200000
                                 // results show that this gap delay is accurate enough (no tuning required)
                                // 600000=5ms
								// testing: TX_interrupt to RX_interrupt got 36ms (TDMA_PERIOD=80ms)
                                //   (See TX, main.c, s[tsp]=3;)
								// 1200000=10ms
								// testing: TXinterpt to RXinterpt = 40ms (TDMA_PERIOD=80ms)
#define TDMA_UNLOCK_DELAY_MS	MilliSec_To_Tick((5*1000)) //no receive data duration before tdma unlocking

#define TDMA_PERIOD			(USE_915MHZ?US_TDMA_PERIOD:EURO_TDMA_PERIOD)  //macro to select between 915MHz and 869 MHz




//upper nibble of 1st byte encodes whether the message has FEC or not
#define MSG_TYPE_HDR_HAS_FEC			(0xF<<4)
#define MSG_TYPE_HDR_NO_FEC				(0x0<<4)
//lower nibble of 1st byte determines message type
#define MSG_TYPE_BASE_GPS_INFO			0x01	//contains base station gps
#define MSG_TYPE_REQUEST_FEC_ON			0x03
#define MSG_TYPE_REQUEST_FEC_OFF		0x04

#define MSG_TYPE_HOST_GENERATED			0x06	//message passed in by host - just pass through
#define MSG_TYPE_HOST_GENERATED_A		0x07
#define MSG_TYPE_HOST_GENERATED_B		0x08
#define MSG_TYPE_IDLE					0x09

#define MAVLINK_USB_TRANSFER_LEN		263 //(30+MAVLINK_HDR_LEN+MAVLINK_CHKSUM_LEN)

// these params shall be obsolete, liyenho
 #include <compiler.h>
/* tabulate available ctrl rf bands (hopping channels) */
#define HOPPING_TABLE_SIZE					50
#define WRAP_OFFSET(x) (HOPPING_TABLE_SIZE<=(x))?((x)-HOPPING_TABLE_SIZE):(x)
#define HOP_2CH_ENABLE                      0/*1*/   //debug testing
#define HOP_2CH_OFFSET0                     0
#define HOP_2CH_OFFSET1                     1/*24*/
#define FIXED_PAIR_ID						1
#define HOP_ID_LEN									10
 #define CHTBL_SIZE											HOPPING_TABLE_SIZE /*sizeof(chtbl_ctrl_rdo)/(2*sizeof(*chtbl_ctrl_rdo))*/
 #define FORWARD_HOP									((CHTBL_SIZE+1) / 10)
 #define BACKWARD_HOP								(FORWARD_HOP-1)
 #define HOP_CHN_SPACE								500000	/*hop channel spacing*/
 #define FREQ_SHIFT_STEP							50000		/*frequency shift step*/
 #define NUM_FREQ_SHIFT							(HOP_CHN_SPACE / FREQ_SHIFT_STEP)
 #define FREQ_FRAC_POS								5 // zero based
 #define FREQ_INTR_POS									4	// zero based
  #define tm_delta(prev, curr, del) \
		if (curr < prev)	\
			del=curr+(0xffffffff-prev);	\
		else	\
			del=curr -prev;
	#define minmax(mn, mx, v)	\
		if (0==mn)	\
			mn = (v);	\
		else if (mn>(v))	\
			mn = (v);	\
		else if (mx<(v))	\
			mx = (v);

#define FEC_ON_RSSI_THRESHOLD		0x30  //RSSI reading at which to request FEC On
#define FEC_OFF_RSSI_THRESHOLD		(FEC_ON_RSSI_THRESHOLD+10)   //RSSI reading at which to request FEC Off
																	//approx 5 db above ON threshold
//keep track of has a request has been sent to other side to turn FEC on
extern bool Requested_FEC_On;

enum FEC_Options {
	OFF = 0x00, //Host has asserted control
	ON = 0x0F,	//Host has asserted control
	AUTO = 0xFF	//Atmel has control
} ;
extern enum FEC_Options FEC_Option;
extern bool Send_with_FEC; //turn on to send messages with FEC
extern bool send_long_packet;
bool USE_915MHZ;
uint8_t control_channel;
uint32_t idle_msg_queue_intv;

// !ctrl range status monitor parameters
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

extern volatile bool ctrl_tdma_enable;
extern enum pair_mode hop_state;
int fhop_offset, fhop_base;
uint8_t hop_id[HOP_ID_LEN];
int fhopless;
int fhop_idx;

uint8_t vRadio_Init(void) ;
uint32_t Control_Outbound_Queue_Available_Slots(void);
bool Queue_Control_Idle_Packet(void);
bool Queue_Control_Packet(uint8_t * pending_msg);
bool Queue_FEC_Request_On_Packet(void);
bool Queue_FEC_Request_Off_Packet(void);
//void Ctrl_Set_FEC(uint8_t FEC_Options);
uint8_t Get_Control_Packet(uint8_t *ptr);
void Handle_Channel_Change_Request(uint8_t *msg_start);
void Set_Pair_ID(uint8_t *ID);
void ctrl_hop_global_update(bool listen);
int hop_chn_sel(int offset);
uint8_t Insert_Control_In_TSStream(uint8_t *ptr);
#endif /* CTRL_H_ */