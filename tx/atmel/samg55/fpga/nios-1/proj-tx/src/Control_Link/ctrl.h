/*
 * ctrl.h
 *
 */ 

#ifndef CTRL_H_
#define CTRL_H_

#define ASYMM_RATIO				/*1*/ 2
#define RADIO_PKT_LEN		32 // ctl/sts radio payload byte length +1
#define RADIO_GRPPKT_LEN    30
#define RADIO_INFO_LEN      4 // usb pipe information post header
#define RDO_ELEMENT_SIZE   (RADIO_PKT_LEN/sizeof(uint32_t))  // RADIO_PKT_LEN must divide into sizeof(uint32_t), liyenho
#define TDMA_PERIOD     /*3600000*/ /*3240000*/(2*2880000) // 2880000: 24ms
								// 30kbps 2snd1rx case--------------------------------------------
								//   actual transfer period = 26B*8*3/30000 = 20.8ms
								//   TX delivery time = 26*8*2/30000 = 13.86ms
								//   TDMA_PERIOD= 25ms = 3000000
								//   RX interrupt to TX start = TDMA_RX_TO_TX= 1368000
								//   Empirically Measured: snd26Bpkt time: 8.2ms, TDMA_TX_PERIOD = 984000
								//   TDMA_PERIOD_TOL = 96000 = +/-0.8ms (FYI:saw only 0.2ms jitter)
								//   TDMA_RX_TO_TX 120000=1ms. immediately start to transmit.
#define TDMA_TX_PERIOD      (2*984000)
#define  TDMA_BOUND	 /*5400000*/ /*4860000*/ (2*4320000) // 5400000 = 45 ms
#define TDMA_PERIOD_TOL /*96000*/ (2*144000) /*192000*/ // jitter tolerance 480000=4ms 600000=5ms
#define TDMA_PERIOD_MAX (TDMA_PERIOD+TDMA_PERIOD_TOL)
#define TDMA_PERIOD_MIN (TDMA_PERIOD-TDMA_PERIOD_TOL)
#define TDMA_RX_TO_TX   (1.5*120000) /**180000**/ /*360000*/ // RX interrupt to TX startTX()
                                // 1200000
                                 // results show that this gap delay is accurate enough (no tuning required)
                                // 600000=5ms
								// testing: TX_interrupt to RX_interrupt got 36ms (TDMA_PERIOD=80ms)
                                //   (See TX, main.c, s[tsp]=3;)
								// 1200000=10ms
								// testing: TXinterpt to RXinterpt = 40ms (TDMA_PERIOD=80ms)
#define TDMA_UNLOCK_DELAY_MS (5*120000000) //no receive data duration before tdma unlocking
#define RDO_TPACKET_FIFO_SIZE       8
#define RDO_RPACKET_FIFO_SIZE       8

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



 #include <compiler.h>
 /* used to be channel selection table for control radio band plan, now it can serve as lookup table for frequency hopp */
extern const uint8_t RF_FREQ_CONTROL_INTE_HOP0_6[10]; // 902.285 MHz
 extern const uint8_t RF_MODEM_AFC_LIMITER_HOP0_3[7];
extern const uint8_t RF_FREQ_CONTROL_INTE_HOP13_6[10]; // 925.035 MHz
 extern const uint8_t RF_MODEM_AFC_LIMITER_HOP13_3[7];
/* tabulate available ctrl rf bands (hopping channels) */
#define HOPPING_TABLE_SIZE					50
#define WRAP_OFFSET(x) (HOPPING_TABLE_SIZE<=(x))?((x)-HOPPING_TABLE_SIZE):(x)
#define HOP_2CH_ENABLE                      0/*1*/   //debug testing
#define HOP_2CH_OFFSET0                     0
#define HOP_2CH_OFFSET1                     1/*24*/
 extern const uint8_t *chtbl_ctrl_rdo[HOPPING_TABLE_SIZE*2] ;
#define RF_FREQ_CONTROL_INTE_LEN	sizeof(RF_FREQ_CONTROL_INTE_HOP0_6)
#define RF_MODEM_AFC_LIMITER_LEN	sizeof(RF_MODEM_AFC_LIMITER_HOP0_3)
 #define CHTBL_SIZE											sizeof(chtbl_ctrl_rdo)/(2*sizeof(*chtbl_ctrl_rdo))
 #define FORWARD_HOP									((CHTBL_SIZE+1) / 10)
 #define BACKWARD_HOP								(FORWARD_HOP-1)
 #define HOP_CHN_SPACE								500000	/*hop channel spacing*/
 #define FREQ_SHIFT_STEP							50000		/*frequency shift step*/
#ifdef SI4463_CRYSTAL_32MHZ
 #define FREQ_CTRL_FACTOR						((4*524288.0)/(2*32*1000000.0))
#else  // 30 MHz crystal old design
 #define FREQ_CTRL_FACTOR						((4*524288.0)/(2*30*1000000.0))
#endif
 #define NUM_FREQ_SHIFT							(HOP_CHN_SPACE / FREQ_SHIFT_STEP)
 #define FREQ_FRAC_POS								5 // zero based
 #define FREQ_INTR_POS									4	// zero based
  #define CAP_VAL_POS										0x4
  #define CAP_TUNE_THR									7	// above halfway of 20 ms/pk in 200 ms
  #define CALIB_DWELL_INTV						24000000	// eval each cap value in 200 ms
  #define RECALIB_INTV_F								240000000L // recheck/recalibrate every 2 sec
  #define TEMP_DEL_THR									3		// adjust cap value if temp delta is greater than 3 degree
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
	typedef struct si4463_capv_tune {
		bool calib_req_h;
		bool calib_gated;
		uint32_t calib_det_rx; // to tell if any rx activity
		uint8_t cap_curr;
		uint8_t lower;
		uint8_t upper;
		uint8_t median;
		uint32_t tm_started; // sampled calib eval start time
		uint32_t tm_curr;
		uint32_t tm_ended; // sampled calib eval end time
	} capv_tune_t;

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

bool Queue_Control_Packet(uint8_t * pending_msg);
uint32_t fifo_lvl_calc(uint32_t wrptr, uint32_t rdptr, uint32_t fifodepth);
uint32_t Control_Outbound_Queue_Available_Slots(void);
bool Queue_Control_Idle_Packet(void);
bool Queue_Control_Packet(uint8_t * pending_msg);
bool Queue_FEC_Request_On_Packet(void);
bool Queue_FEC_Request_Off_Packet(void);
//void Ctrl_Set_FEC(uint8_t FEC_Options);


#endif /* CTRL_H_ */