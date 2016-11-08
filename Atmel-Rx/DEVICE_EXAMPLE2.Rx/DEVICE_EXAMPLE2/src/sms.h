#ifndef _SMS_H_
#define _SMS_H_ // detailed sms4470 header
#ifdef CONF_BOARD_USB_TX
#include "main.h"
#include "DigiBestFrontend.h"
/*
typedef enum
{
      DVBT_WORKING_MODE,
      DVBT2_WORKING_MODE,
      DVBT_DVBT2_DUAL_WORKING_MODE,
      ISDBT_WORKING_MODE,
      UNKNOWN_WORKING_MODE
}FRONTEND_WORKING_MODE;
*/

/*
typedef enum
{
      ISDBT_13SEGMENT_DEMODULATOR_TUNING,
	  ISDBT_1SEGMENT_DEMODULATOR_TUNING,
	  DVBT_DEMODULATOR_TUNING,
	  DVBH_DEMODULATOR_TUNING,
	  DVBT2_DEMODULATOR_TUNING,
	  DVBT2_LITE_DEMODULATOR_TUNING,
	  UNKNOWN_DEMODULATOR_TUNING
}DEMODULATOR_TUNING_FEATURE;
*/

//#ifdef FRONTEND_ENABLE_REGULAR_TIMER_CALLBACK
/*
typedef enum
{
      FFT_MODE_1K,
      FFT_MODE_2K,
      FFT_MODE_4K,
      FFT_MODE_8K,
      FFT_MODE_16K,
      FFT_MODE_16K_EXT,
      FFT_MODE_32K,
      FFT_MODE_32K_EXT,
	  FFT_MODE_RESERVED,
      UNKNOWN_FFT_MODE
}DTV_MODULATION_TRANSMISSION_MODE;
*/
/*
typedef enum
{
      GUARD_INTERVAL_1_4,
	  GUARD_INTERVAL_1_8,
	  GUARD_INTERVAL_1_16,
	  GUARD_INTERVAL_1_32,
	  GUARD_INTERVAL_1_128,
	  GUARD_INTERVAL_19_128,
	  GUARD_INTERVAL_19_256,
	  UNKNOWN_GUARD_INTERVAL
}DTV_MODULATION_GUARD_INTERVAL;
*/
/*
typedef enum
{
      CODE_RATE_1_2,
	  CODE_RATE_1_3,
	  CODE_RATE_2_3,
	  CODE_RATE_3_4,
	  CODE_RATE_2_5,
	  CODE_RATE_3_5,
	  CODE_RATE_4_5,
	  CODE_RATE_5_6,
	  CODE_RATE_7_8,
	  UNKNOWN_CODE_RATE
}DTV_MODULATION_CODE_RATE;
*/
/*
typedef enum
{
      CONSTELLATION_QPSK,
	  CONSTELLATION_16_QAM,
	  CONSTELLATION_64_QAM,
	  CONSTELLATION_256_QAM,
      UNKNOWN_CONSTELLATION
}DTV_MODULATION_CONSTELLATION;
*/
/*
typedef enum
{
	  HIERARCHY_NONE,
	  HIERARCHY_ALPHA_1,
	  HIERARCHY_ALPHA_2,
	  HIERARCHY_ALPHA_4,
	  UNKNOWN_HIERARCHY
}DTV_MODULATION_HIERARCHY;
*/

/*
typedef struct
{
      long SNR;
      long InBandPower;
      unsigned long ErrorTSPackets;
      unsigned long TotalTSPackets;
      DTV_MODULATION_TRANSMISSION_MODE TransmissionMode;
      DTV_MODULATION_GUARD_INTERVAL GuardInterval;
      DTV_MODULATION_CODE_RATE CodeRate;
      DTV_MODULATION_CODE_RATE LPCodeRate;
      DTV_MODULATION_CONSTELLATION Constellation;
      DTV_MODULATION_HIERARCHY Hierarchical;
      unsigned long DVBH_FrameErrorRate;
      unsigned long DVBHCellID;
      bool DVBH_TimeSlicing_HP;
      bool DVBH_MPE_FEC_HP;
      bool DVBH_TimeSlicing_LP;
      bool DVBH_MPE_FEC_LP;
}DVBT_STATISTIC_INFORMATION,*PDVBT_STATISTIC_INFORMATION;
*/
/*
typedef enum
{
      PILOT_PATTERN_1,
      PILOT_PATTERN_2,
      PILOT_PATTERN_3,
      PILOT_PATTERN_4,
      PILOT_PATTERN_5,
      PILOT_PATTERN_6,
      PILOT_PATTERN_7,
      PILOT_PATTERN_8,
      UNKNOWN_PILOT_PATTERN
}DVBT2_PILOT_PATTERN;
*/
/*
typedef enum
{
	  COMMON_PLP,
	  DATA_TYPE1_PLP,
	  DATA_TYPE2_PLP,
      UNKNOWN_PLP_TYPE
}DVBT2_PLP_TYPE;
*/
/*
typedef enum
{
	  PLP_NOT_SPECIFIED_MODE,
	  PLP_NORMAL_EFFICIENCY_MODE,
	  PLP_HIGH_EFFICIENCY_MODE,
	  PLP_RESERVED_MODE,
	  UNKNOWN_PLP_MODE
}DVBT2_PLP_MODE;
*/
/*
typedef enum
{
	  FEC_FRAME_16K_LDPC,
	  FEC_FRAME_64K_LDPC,
      UNKNOWN_FEC_FRAME
}DVBT2_FEC_FRAME_TYPE;
*/
/*
typedef struct
{
      unsigned long PlpID;
      unsigned long PlpGroupID;
      DVBT2_PLP_TYPE PlpType;
      DVBT2_PLP_MODE PlpMode;
      DVBT2_FEC_FRAME_TYPE FecFrameType;
      DTV_MODULATION_CODE_RATE CodeRate;
      DTV_MODULATION_CONSTELLATION Constellation;
      bool PlpRotationEnable;
      unsigned long PlpNumberBlocksMax;
      unsigned long FrameInterval;
      unsigned long TimeIlLength;
      unsigned long TimeIlType;
}DVBT2_PLP_STATISTIC,*PDVBT2_PLP_STATISTIC;
*/
/*
#define FRONTEND_DVBT2_MAX_PLPS                  8
typedef struct
{
      unsigned char NumberOfPlps;
      DVBT2_PLP_STATISTIC PlpStatistic[FRONTEND_DVBT2_MAX_PLPS];
}DVBT2_PLP_INFORMATION,*PDVBT2_PLP_INFORMATION;
*/
/*
typedef struct
{
      long SNR;
      long InBandPower;
      unsigned long ErrorTSPackets;
      unsigned long TotalTSPackets;
      unsigned long CellID;
      unsigned long NetworkID;
      bool T2LiteActive;
      DTV_MODULATION_TRANSMISSION_MODE TransmissionMode;
      DTV_MODULATION_GUARD_INTERVAL GuardInterval;
      DVBT2_PILOT_PATTERN PilotPattern;
	  bool PlpInformationAvailableFlag;
	  DVBT2_PLP_INFORMATION PlpInformation;
}DVBT2_STATISTIC_INFORMATION,*PDVBT2_STATISTIC_INFORMATION;
*/
//#endif //FRONTEND_ENABLE_REGULAR_TIMER_CALLBACK
/*
typedef struct
{
      bool IsSignalLocked;
      bool StatisticAvailableFlag;
      union
      {
	      DVBT_STATISTIC_INFORMATION DVBTStatisticInfo;
	      DVBT2_STATISTIC_INFORMATION DVBT2StatisticInfo;
	      //ISDBT_STATISTIC_INFORMATION ISDBTStatisticInfo;
      }DTV_STATISTIC;
}STATISTICS_INFORMATION,*PSTATISTICS_INFORMATION;
*/

typedef struct RECEPTION_STATISTICS_S
{
	uint32_t IsRfLocked;				//!< 0 - not locked, 1 - locked
	uint32_t IsDemodLocked;			//!< 0 - not locked, 1 - locked
	uint32_t IsExternalLNAOn;			//!< 0 - external LNA off, 1 - external LNA on

	uint32_t ModemState;				//!< from SMSHOSTLIB_DVB_MODEM_STATE_ET
	int32_t  SNR;						//!< dB
	uint32_t BER;						//!< Post Viterbi BER [1E-5]
	uint32_t BERErrorCount;			//!< Number of erroneous SYNC bits.
	uint32_t BERBitCount;				//!< Total number of SYNC bits.
	uint32_t TS_PER;					//!< Transport stream PER, 0xFFFFFFFF indicate N/A
	uint32_t MFER;					//!< DVB-H frame error rate in percentage, 0xFFFFFFFF indicate N/A, valid only for DVB-H
	int32_t  RSSI;					//!< dBm
	int32_t  InBandPwr;				//!< In band power in dBM
	int32_t  CarrierOffset;			//!< Carrier Offset in bin/1024
	uint32_t ErrorTSPackets;			//!< Number of erroneous transport-stream packets
	uint32_t TotalTSPackets;			//!< Total number of transport-stream packets

	int32_t  RefDevPPM;
	int32_t  FreqDevHz;

	int32_t  MRC_SNR;					//!< dB //in non MRC application: maximum dvbt cnt
	int32_t  MRC_RSSI;				//!< dBm
	int32_t  MRC_InBandPwr;			//!< In band power in dBM, in Non MRC application: dvbt buffer max count. Should be less than 345*188

	uint32_t ErrorTSPacketsAfterReset;			//!< Number of erroneous transport-stream packets from the last reset
	uint32_t TotalTSPacketsAfterReset;			//!< Total number of transport-stream packets from the last reset
}RECEPTION_STATISTICS_ST;

// Statistics information returned as response for SmsLiteMsGetStatistics_Req for DVB applications, SMS1100 and up
typedef struct SMSHOSTLIB_STATISTICS_DVBT_S
{
	// Reception
	RECEPTION_STATISTICS_ST ReceptionData;
	// Transmission parameters
	TRANSMISSION_STATISTICS_ST TransmissionData;
	uint32_t ReceptionQuality;
} SMSHOSTLIB_STATISTICS_DVBT_ST;

typedef struct TRANSMISSION_STATISTICS_DVBT2_S
{
	uint32_t Frequency;				//!< Frequency in Hz
	uint32_t Bandwidth;				//!< Bandwidth in MHz
	uint32_t res[3];
}TRANSMISSION_STATISTICS_DVBT2_ST;

typedef struct RECEPTION_STATISTICS_DVBT2_S
{
		uint32_t	IsModemLocked;				//!< 0 - not locked, 1 - locked
		TRANSMISSION_STATISTICS_DVBT2_ST  txStatistics;
		int32_t carrierOffset;
		int32_t inbandPower;
		uint32_t extLna;
		uint32_t totalFrames;
		int32_t  SNR;						//!< dB
       int32_t  RSSI;					//!< dBm
		uint32_t FER;
		uint32_t CellId;
		uint32_t netId;
		uint32_t receptionQuality;
		uint32_t bwt_ext;
		uint32_t fftMode;
		uint32_t guardInterval;
		uint32_t pilotPattern;
		uint32_t bitRate;
		uint32_t extended;
		uint32_t toneReservation;
		uint32_t l1PostSize;
		uint32_t numOfAuxs;
		uint32_t numOfPlps;
		uint32_t liteMode;

		int32_t  MRC_SNR;					// !< dB
		uint32_t SNRFullRes;				// !< dB x 65536
		int32_t  MRC_InBandPwr;			// !< In band power in dBM
		int32_t  MRC_Rssi;

		int8_t  commonPlpNotSupported;
		int8_t  l1modulation;
		uint16_t numdatasymbols;
		uint32_t res[2];
}RECEPTION_STATISTICS_DVBT2_ST;

typedef struct DVBT2_GENERAL_INFO_S
{
	uint32_t smoothing;
	uint32_t res[3];
}DVBT2_GENERAL_INFO_ST;

typedef struct DVBT2_PLP_STATISTICS_DATA_S
{
		uint32_t plpId;
		uint32_t plpType;
		uint32_t plpPayloadType;
		uint32_t ffFlag;
		uint32_t firstRfIdx;
		uint32_t firstFrameIdx;
		uint32_t plpGroupId;
		uint32_t plpCod ;
		uint32_t plpMod ;
		uint32_t plpRotation;
		uint32_t plpFecType;
		uint32_t plpNumBlocksMax;
		uint32_t frameInterval;
		uint32_t timeIlLength;
		uint32_t timeIlType;
		uint32_t inbandA_Flag;
		uint32_t inbandB_Flag;
		uint32_t plpMode;
		uint32_t staticFlag;
		uint32_t staticPaddingFlag;
		uint32_t res[3];
} DVBT2_PLP_STATISTICS_DATA_ST;

typedef struct PLP_DATA_S
{
	DVBT2_PLP_STATISTICS_DATA_ST plpStatistics;
}PLP_DATA_ST;

typedef struct
{
		uint32_t plpId;
		uint32_t plpType;
		uint32_t plpEfficiencyMode;
		uint32_t dnp;
		uint32_t issyi;
		uint32_t crcErrors;
		uint32_t numOfLdpcIters;
		uint32_t totalNumBBFramesReceived;  // Total number of BB frames received.
		uint32_t totalNumErrBBFramesReceived; // Total number of error BB frames received.
		uint32_t totalNumTsPktsReceived;  // Total number of TS packets received.
		uint32_t totalNumTsPktsTransmitted;  // Total number of TS packets transmitted to the TSI.
		uint32_t totalNumErrTsPktsReceived;  // Total number of error TS packets received.
		uint32_t numOfOverflow;
		uint32_t numOfUnderflow;
		uint32_t dejitterBufferSize;
		uint32_t totalNumOfPktsInserted;
		uint32_t totalNumTsPktsForwarded;
		uint32_t totalPostLdpcErr;
		uint32_t numTsPktsReceivedAfterReset;			//!< Total number of transport-stream packets from the last reset
		uint32_t numErrTsPktsReceivedAfterReset;		//!< Number of erroneous transport-stream packets from the last reset
		uint32_t res[1];
}  ACTIVE_PLP_STATISTICS_ST;

#define	DVBT2_MAX_PLPS_LITE												(8)
#define	DVBT2_ACTIVE_PLPS_LITE										(2)
// Statistics information returned as response for SmsLiteMsGetStatistics_Req for DVB applications, SMS1100 and up
typedef struct SMSHOSTLIB_STATISTICS_DVBT2_S
{
	// Reception
	RECEPTION_STATISTICS_DVBT2_ST ReceptionData;
	// Transmission parameters
	TRANSMISSION_STATISTICS_DVBT2_ST TransmissionData;
	DVBT2_GENERAL_INFO_ST   generalInfo;                            // 20140426
	// Burst parameters, valid only for DVBT2
	PLP_DATA_ST PlpData[DVBT2_MAX_PLPS_LITE];
	ACTIVE_PLP_STATISTICS_ST activePlps[DVBT2_ACTIVE_PLPS_LITE];
} SMSHOSTLIB_STATISTICS_DVBT2_ST;

typedef struct
{
	  FRONTEND_WORKING_MODE            Sms4470CurrentWorkingMode;
#ifdef FRONTEND_ENABLE_REGULAR_TIMER_CALLBACK
      bool                             SMS4470CurrentStatisticInformationAvailableFlag;
	  STATISTICS_INFORMATION           SMS4470CurrentStatisticInformation;
#endif
	  bool                             SMS4470DetectNoSignalFlag;
	  RECEPTION_STATISTICS_ST          DVBTReceptionStatistic;
	  TRANSMISSION_STATISTICS_ST       DVBTTransmissionStatistic;
      uint8_t                            SMS4470DVBT2LastPlpIdSetup;
      Short_Statistics_ST		DVBTSignalStatistic;  // added by liyenho
}SMS4470_DEVICE_GROUP_OPERATION_INFORMATION;

#define SIANO_BANDWIDTH_8M          (0)
#define SIANO_BANDWIDTH_7M          (1)
#define SIANO_BANDWIDTH_6M          (2)
#define SIANO_BANDWIDTH_5M          (3)
#define SIANO_BANDWIDTH_1_SEG       (4)
#define SIANO_BANDWIDTH_3_SEG       (5)
#define SIANO_BANDWIDTH_2M          (6)
#define SIANO_BANDWIDTH_FM_RADIO    (7)
#define SIANO_BANDWIDTH_13_SEG      (8)
#define SIANO_BANDWIDTH_13SEG_7MHZ  (11)
#define SIANO_BANDWIDTH_13SEG_8MHZ  (12)
#define SIANO_BANDWIDTH_1SEG_7MHZ   (13)
#define SIANO_BANDWIDTH_1SEG_8MHZ   (14)
#define SIANO_BANDWIDTH_1_5_M       (15)
#define SIANO_BANDWIDTH_T2_8_MHZ	(16)
#define SIANO_BANDWIDTH_T2_7_MHZ	(17)
#define SIANO_BANDWIDTH_T2_6_MHZ	(18)
#define SIANO_BANDWIDTH_T2_5_MHZ	(19)
#define SIANO_BANDWIDTH_UNKNOW      (0xffff)

typedef enum {
	MODEM_LOCKED = 0,
	FREQUENCY,
	BANDWIDTH,
	CARRIEER_OFFSET,
	INBAND_POWER,
	EXT_LNA,
	TOTAL_FRAMES,
	SNR,
	RSSI,
	FER,
	CELL_ID,
	NET_ID,
	RECEPTION_QUALITY,
	BWT_EXT,
	FFT_MODE,
	GUARD_INTERVAL,
	PILOT_PATTERN,
	BIT_RATE,
	EXTENDED,
	TONE_RESERVATION,
	L1_POST_SIZE,
	NUM_OF_AUXS,
	NUM_OF_PLPS,
	LITE_MODE,
	MRC_SNR,
	SNR_FULL_RES,
	MRC_INBAND_PWR,
	MRC_RSSI,
	CMN_PLP_NOT_SUPPORTED,
	L1_MODULATION,
	NUM_DATA_SYMB,
	NUM_BASE_REGS,
} BASE_SMS_REG_TYPE;

typedef enum {
	PLP_ID,
	PLP_TYPE,
	PLP_PAYLOAD_TYPE,
	FF_FLAG,
	FIRST_RF_IDX,
	FIRST_FRAME_IDX,
	PLP_GROUP_ID,
	PLP_COD,
	PLP_MOD,
	PLP_ROTATION,
	PLP_FEC_TYPE,
	PLP_NUM_BLK_MAX,
	FRAME_INTERVAL,
	TIME_IL_LENGTH,
	TIME_IL_TYPE,
	INBAND_A_FLAG,
	INBAND_B_FLAG,
	PLP_MODE,
	STATIC_FLAG,
	STATIC_PADDING_FLAG,
	NUM_PLP_REGS,
} PLP_SMS_REG_TYPE;

typedef enum {
	ACT_PLP_ID,
	ACT_PLP_TYPE,
	PLP_EFF_MODE,
	DNP,
	ISSYI,
	CRC_ERRORS,
	NUM_LDPC_ITERS,
	TOT_NUM_BB_FRAMS_RECV,
	TOT_NUM_TS_PKT_RECV,
	TOT_NUM_ERR_TS_PKT_RECV,
	NUM_OF_OVERFLOW,
	NUM_OF_UNDERFLOW,
	DEJITTER_BUFFER_SIZE,
	TOT_NUM_PKT_INSERTED,
	TOT_NUM_TS_PKT_FORWD,
	TOT_POST_LDPC_ERR,
	NUM_TS_PKT_RECV_AFTER_RESET,
	NUM_ERR_TS_PKT_RECV_AFTER_RESET,
	NUM_ACTIVE_PLP_REGS,
} ACTIVE_PLP_REG_TYPE;

#define DVBT_REGS_BASE 	(NUM_BASE_REGS+ DVBT2_MAX_PLPS_LITE*NUM_PLP_REGS+DVBT2_ACTIVE_PLPS_LITE*NUM_ACTIVE_PLP_REGS)

typedef enum {
	MODEM_LOCKED_DVBT = DVBT_REGS_BASE,
	INBAND_POWER_DVBT,
	BER_DVBT,
	SNR_DVBT,
	TOT_TS_PKT_DVBT,
	ERR_TS_PKT_DVBT,
	LAST_SMS_REG,
} DVBT_REG_TYPE;

#define NUM_OF_REGS		(int)LAST_SMS_REG
//#define MEASURE_TIMING // NEVER enable this option, it can't work!!! liyenho
#ifdef MEASURE_TIMING
	extern volatile uint32_t tick_30us ;
#endif
#if defined(SMS_DVBT2_DOWNLOAD) || defined(RECV_SMS4470) || defined(I2C_FAKE_POLL)
  volatile uint32_t WORKING_MODE; // address to access first sms register
  volatile uint32_t sms_status_regs [NUM_OF_REGS];
  extern volatile bool sms_dvbt_lock; // global recv locked satus
	extern void configure_rtt(unsigned int clkcnt);
	extern void spi_tx_transfer(void *p_tbuf, uint32_t tsize, void *p_rbuf,
		uint32_t rsize, uint32_t ch);
	extern void twi_sms4470_handler(const uint32_t id, const uint32_t index);
	extern bool read_sms_response(MsgTypes_ET res, Sms4470_State ste);
  extern void SMS4470_HandlePerSlicesIndication_lh(uint32_t* pMsgData,uint32_t len);
  extern void SMS4470_TransStatsIndication_lh(uint8_t *pData);
  extern void SMS4470_NoSignalDetected();
  extern void SMS4470_get_version_lh();
	extern void SMS4470_send_device_init_lh(DEMODULATOR_TUNING_FEATURE feature);
	extern void SMS4470_set_polling_mode_lh();
	extern void SMS4470_enable_ts_interface_lh();
	extern void SMS4470_RemovePidFilter_lh(uint32_t pid);
	extern void SMS4470_tune_lh(DEMODULATOR_TUNING_FEATURE feature,
																		uint32_t bandwidth, uint32_t frequency);
	extern void SMS4470_AddPidFilter_lh(uint32_t pid);
	extern void SMS4470_OpenPlp_lh(uint32_t plpId);
	extern bool SMS4470_check_signal_lh(bool * pLockStatus);
	extern bool SMS4470_CheckDVBT2PhysicalLayerPipeInformation_lh(bool* pDoesPhysicalLayerPipeInformationExistFlag);
	extern void SMS4470ReadResponseWithDelay_lh(uint32_t Milliseconds);
	extern bool Sms4470CoreAPI_CheckLockStatusForChannelSearch_lh(uint8_t MaxCheckCounter,bool* pLockStatus);
	extern bool SMS4470_GetReceptionStatistics_lh(PSTATISTICS_INFORMATION pStatistics);
#endif
#ifdef I2C_FAKE_POLL
 typedef struct {
	uint16_t mlen;	// message length
	uint16_t lsfr;	// lfsr pattern
 } i2c_poll_size;
  extern i2c_poll_size LSFR_tbl[10];
#endif
#endif //CONF_BOARD_USB_TX
#endif // _SMS_H_
