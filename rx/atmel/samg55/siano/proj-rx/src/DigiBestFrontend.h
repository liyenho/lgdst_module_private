#include "DigiBestTypedef.h"

#ifndef __DIGIBESTFRONTEND_H
#define __DIGIBESTFRONTEND_H

//*********************************************************************************
//*********************************************************************************
//*********************************************************************************
#define FRONTEND_UNUSED_ID                       0x80000000
#define FRONTEND_DVBT2_MAX_PLPS                  8
//*********************************************************************************
//*********************************************************************************
//*********************************************************************************
typedef enum
{
      TWO_DIVERSITY_GROUP,
	  FOUR_DIVERSITY_GROUP,
	  UNKNOWN_FRONTEND_GROUP

}FRONTEND_GROUP_TYPE;

typedef enum
{
      DVBT_WORKING_MODE,
      DVBT2_WORKING_MODE,
      DVBT_DVBT2_DUAL_WORKING_MODE,
      ISDBT_WORKING_MODE,
      UNKNOWN_WORKING_MODE

}FRONTEND_WORKING_MODE;

typedef enum
{ 
      BANDWIDTH_1_7_MHZ,
	  BANDWIDTH_5_MHZ,
	  BANDWIDTH_6_MHZ,
	  BANDWIDTH_7_MHZ,
	  BANDWIDTH_8_MHZ,
	  UNKNOWN_BANDWIDTH

}BANDWIDTH_TYPE;

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

typedef enum
{
      CONSTELLATION_QPSK,
	  CONSTELLATION_16_QAM,
	  CONSTELLATION_64_QAM,
	  CONSTELLATION_256_QAM,
      UNKNOWN_CONSTELLATION

}DTV_MODULATION_CONSTELLATION;

typedef enum
{
	  HIERARCHY_NONE,
	  HIERARCHY_ALPHA_1,
	  HIERARCHY_ALPHA_2,
	  HIERARCHY_ALPHA_4,
	  UNKNOWN_HIERARCHY

}DTV_MODULATION_HIERARCHY;

typedef enum
{
      TRANSMISSION_MODE_1,
      TRANSMISSION_MODE_2,
      TRANSMISSION_MODE_3,
	  UNKNOWN_TRANSMISSION_MODE

}ISDBT_TRANSMISSION_MODE;

typedef enum
{
      TIME_INTERLEAVER_0,
	  TIME_INTERLEAVER_1,
	  TIME_INTERLEAVER_2,
	  TIME_INTERLEAVER_4,
	  UNKNOWN_TIME_INTERLEAVER

}ISDBT_TIME_INTERLEAVER;

typedef enum
{
	  COMMON_PLP,
	  DATA_TYPE1_PLP,  
	  DATA_TYPE2_PLP,  
      UNKNOWN_PLP_TYPE

}DVBT2_PLP_TYPE;

typedef enum
{
	  PLP_NOT_SPECIFIED_MODE,
	  PLP_NORMAL_EFFICIENCY_MODE,
	  PLP_HIGH_EFFICIENCY_MODE,
	  PLP_RESERVED_MODE,
	  UNKNOWN_PLP_MODE

}DVBT2_PLP_MODE;

typedef enum
{
	  FEC_FRAME_16K_LDPC,
	  FEC_FRAME_64K_LDPC,
      UNKNOWN_FEC_FRAME

}DVBT2_FEC_FRAME_TYPE;

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

//*********************************************************************************
//*********************************************************************************
//*********************************************************************************

typedef struct
{
      ULONG TuningFrequency;  
      BANDWIDTH_TYPE BandwidthType;                 
      DEMODULATOR_TUNING_FEATURE DemodulatorTuningFeature;   

}TUNING_PARAMETER,*PTUNING_PARAMETER;

typedef struct
{
      UBYTE NumberOfSegments;  
      ULONG ErrorTSPackets; 
      ULONG TotalTSPackets;
      DTV_MODULATION_CODE_RATE CodeRate;
      DTV_MODULATION_CONSTELLATION Constellation;
      ISDBT_TIME_INTERLEAVER TimeInterleaver;

}ISDBT_LAYER_STATISTIC,*PISDBT_LAYER_STATISTIC;

typedef struct
{
      ULONG PlpID;
      ULONG PlpGroupID;
      DVBT2_PLP_TYPE PlpType;
      DVBT2_PLP_MODE PlpMode;
      DVBT2_FEC_FRAME_TYPE FecFrameType;
      DTV_MODULATION_CODE_RATE CodeRate;
      DTV_MODULATION_CONSTELLATION Constellation;
      BOOL PlpRotationEnable;
      ULONG PlpNumberBlocksMax;	
      ULONG FrameInterval;		
      ULONG TimeIlLength;	
      ULONG TimeIlType;	

}DVBT2_PLP_STATISTIC,*PDVBT2_PLP_STATISTIC;

typedef struct
{
      UBYTE NumberOfPlps;
      DVBT2_PLP_STATISTIC PlpStatistic[FRONTEND_DVBT2_MAX_PLPS];

}DVBT2_PLP_INFORMATION,*PDVBT2_PLP_INFORMATION;

typedef struct
{
      LONG SNR; 
      LONG InBandPower; 
      ULONG ErrorTSPackets;
      ULONG TotalTSPackets; 
      DTV_MODULATION_TRANSMISSION_MODE TransmissionMode;
      DTV_MODULATION_GUARD_INTERVAL GuardInterval;
      DTV_MODULATION_CODE_RATE CodeRate;
      DTV_MODULATION_CODE_RATE LPCodeRate;
      DTV_MODULATION_CONSTELLATION Constellation;
      DTV_MODULATION_HIERARCHY Hierarchical;
      ULONG DVBH_FrameErrorRate;
      ULONG DVBHCellID;
      BOOL DVBH_TimeSlicing_HP;
      BOOL DVBH_MPE_FEC_HP;
      BOOL DVBH_TimeSlicing_LP;
      BOOL DVBH_MPE_FEC_LP;

}DVBT_STATISTIC_INFORMATION,*PDVBT_STATISTIC_INFORMATION;

typedef struct
{
      LONG SNR;   
      LONG InBandPower;  
      ULONG ErrorTSPackets; 
      ULONG TotalTSPackets;
      ULONG CellID;
      ULONG NetworkID;
      BOOL T2LiteActive;
      DTV_MODULATION_TRANSMISSION_MODE TransmissionMode;
      DTV_MODULATION_GUARD_INTERVAL GuardInterval;
      DVBT2_PILOT_PATTERN PilotPattern;
	  BOOL PlpInformationAvailableFlag;
	  DVBT2_PLP_INFORMATION PlpInformation;

}DVBT2_STATISTIC_INFORMATION,*PDVBT2_STATISTIC_INFORMATION;

typedef struct
{
      LONG SNR;                      
      LONG InBandPower;                                 
      BOOL PartialReception;
      BOOL EWSAlertActive;
      ISDBT_TRANSMISSION_MODE TransmissionMode;           
      DTV_MODULATION_GUARD_INTERVAL GuardInterval;
      ISDBT_LAYER_STATISTIC LayerStatistic[3];

}ISDBT_STATISTIC_INFORMATION,*PISDBT_STATISTIC_INFORMATION;

typedef struct
{
      BOOL IsSignalLocked;
      BOOL StatisticAvailableFlag; 

      union 
      {
	      DVBT_STATISTIC_INFORMATION DVBTStatisticInfo;
	      DVBT2_STATISTIC_INFORMATION DVBT2StatisticInfo;
	      ISDBT_STATISTIC_INFORMATION ISDBTStatisticInfo;

      }DTV_STATISTIC;

}STATISTICS_INFORMATION,*PSTATISTICS_INFORMATION;

//*********************************************************************************
//*********************************************************************************
//*********************************************************************************

#ifdef __cplusplus
extern "C" {
#endif

extern BOOL Frontend_Initialization();
extern VOID Frontend_Uninitialization();
extern BOOL Frontend_Open(FRONTEND_GROUP_TYPE Type,FRONTEND_WORKING_MODE Mode,PULONG pFrontendID);
extern BOOL Frontend_Close(ULONG FrontendID);
extern BOOL Frontend_SoftwareReset(ULONG FrontendID);
extern BOOL Frontend_SetWorkingMode(ULONG FrontendID,FRONTEND_WORKING_MODE Mode);
extern BOOL Frontend_GetWorkingMode(ULONG FrontendID,FRONTEND_WORKING_MODE* pMode);
extern BOOL Frontend_Tuning(ULONG FrontendID,PTUNING_PARAMETER pTuningParameter);
extern BOOL Frontend_GetLockStatusForChannelSearch(ULONG FrontendID,UBYTE MaxCheckCounter,BOOL* pLockStatus);
extern BOOL Frontend_GetLockStatus(ULONG FrontendID,BOOL* pLockStatus);
extern BOOL Frontend_GetStatistics(ULONG FrontendID,PSTATISTICS_INFORMATION pStatistics);
extern BOOL Frontend_SetupChannelDVBT2PlpID(ULONG FrontendID,UBYTE PlpID);
#ifdef FRONTEND_ENABLE_REGULAR_TIMER_CALLBACK
extern BOOL Frontend_TimerCallback(ULONG FrontendID);
#endif

#ifdef __cplusplus
}
#endif

#endif




