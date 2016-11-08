#include "DigiBestTypedef.h"
#include "DigiBestFrontend.h"
#include "DigiBestSms4470Core.h"
#include "DigiBestDefine.h"
#include "DigiBestPlatformPorting.h"
#include "DigiBestOSPorting.h"
#include <string.h>

#if 1
#include "SMS4470_A2_DVBT_MRC_Firmware_(2.0.0.47).h"
//#include "SMS4470_A2_DVBT2_MRC_Firmware_(2.0.1.14).h"
#else
#include "SMS4470_A2_DVBT_MRC_Firmware_(2.0.0.30).h"
#include "SMS4470_A2_DVBT2_MRC_Firmware_(2.0.0.89b).h"
#endif

#define SMS4470DebugPrintf                         // printf
#define SMS4470InfoPrintf                          // printf
#define SMS4470StatisticPrintf                     // printf
#define SMS4470ErrorPrintf                         // printf
#define SMS4470FunctionNamePrintf                  // printf

#ifdef SMS4470_SERIAL_TRANSPORT_STREAM_OUTPUT
#define DEMODULATOR_DEVICE_ADDRESS			       0xD0
#else
#define DEMODULATOR_DEVICE_ADDRESS			       0xEE
#endif

#define SMS4470_DUAL_MODE_SWITCH_STANDARD_REBOOT_UP_DELAY   1000  // 1000

typedef struct
{
	  FRONTEND_WORKING_MODE            Sms4470CurrentWorkingMode;

      ULONG                            Sms4470CurrentRF;
      BANDWIDTH_TYPE                   Sms4470CurrentBandWidth;
      ULONG                            Sms4470PreviousRF;
      BANDWIDTH_TYPE                   Sms4470PreviousBandWidth;
	  DEMODULATOR_TUNING_FEATURE       Sms4470PreviousDemodulatorTuningFeature;

      BOOL                             Sms4470DeviceAlreadyDownloadFirmwareFlag;
      UINT8                            Sms4470MessageTxBuffer[SIANO_IIC_TX_BUFFER_SIZE];
      UINT8					           Sms4470MessageRxBuffer[SIANO_IIC_RX_BUFFER_SIZE];
      
      SMS4470_DVBT_DVBT2_DUAL_WORKING_MODE_DEMODULATION_MODE        SMS4470CurrentDemodulationModeForDvbtDvbt2DualWorkingMode;

#ifdef FRONTEND_ENABLE_REGULAR_TIMER_CALLBACK

      BOOL                             SMS4470CurrentStatisticInformationAvailableFlag;
	  STATISTICS_INFORMATION           SMS4470CurrentStatisticInformation;

#endif

	  BOOL                             SMS4470DetectNoSignalFlag;
	  RECEPTION_STATISTICS_ST          SMS4470CurrentRespondentDVBTReceptionStatistic;
	  TRANSMISSION_STATISTICS_ST       SMS4470CurrentRespondentDVBTTransmissionStatistic;

      UBYTE                            SMS4470DVBT2LastPlpIdSetup;

}SMS4470_DEVICE_GROUP_OPERATION_INFORMATION,*PSMS4470_DEVICE_GROUP_OPERATION_INFORMATION;

SMS4470_DEVICE_GROUP_OPERATION_INFORMATION Sms4470DeviceGroupInformation[MAX_FRONTEND_RECEIVER_GROUP_NUMBER];

BOOL                                       Sms4470InitializationFlag = FALSE;

//********************************************************************************************
//********************************************************************************************
//********************************************************************************************
BOOL Sms4470CoreAPI_Initialization()
{
	ULONG Loop;
	BOOL Result = FALSE;

    SMS4470FunctionNamePrintf("(Sms4470CoreAPI_Initialization)\n");

	if(Sms4470InitializationFlag == FALSE)
	{
       for(Loop=0;Loop<MAX_FRONTEND_RECEIVER_GROUP_NUMBER;Loop++)
	   {
		   Sms4470DeviceGroupInformation[Loop].Sms4470CurrentWorkingMode = UNKNOWN_WORKING_MODE;
           Sms4470DeviceGroupInformation[Loop].Sms4470CurrentRF = 0;
		   Sms4470DeviceGroupInformation[Loop].Sms4470CurrentBandWidth = UNKNOWN_BANDWIDTH;
		   Sms4470DeviceGroupInformation[Loop].Sms4470PreviousRF = 0xFFFFFFFF;
		   Sms4470DeviceGroupInformation[Loop].Sms4470PreviousBandWidth = UNKNOWN_BANDWIDTH;
		   Sms4470DeviceGroupInformation[Loop].Sms4470PreviousDemodulatorTuningFeature = UNKNOWN_DEMODULATOR_TUNING;
		   Sms4470DeviceGroupInformation[Loop].Sms4470DeviceAlreadyDownloadFirmwareFlag = FALSE;
           memset(Sms4470DeviceGroupInformation[Loop].Sms4470MessageTxBuffer,0,SIANO_IIC_TX_BUFFER_SIZE);
           memset(Sms4470DeviceGroupInformation[Loop].Sms4470MessageRxBuffer,0,SIANO_IIC_RX_BUFFER_SIZE);
		   Sms4470DeviceGroupInformation[Loop].SMS4470CurrentDemodulationModeForDvbtDvbt2DualWorkingMode = UNKNOWN_SMS4470_DEMODULATION_MODE;
#ifdef FRONTEND_ENABLE_REGULAR_TIMER_CALLBACK
		   Sms4470DeviceGroupInformation[Loop].SMS4470CurrentStatisticInformationAvailableFlag = FALSE;
           memset(&Sms4470DeviceGroupInformation[Loop].SMS4470CurrentStatisticInformation,0,sizeof(STATISTICS_INFORMATION));
#endif
		   Sms4470DeviceGroupInformation[Loop].SMS4470DetectNoSignalFlag = FALSE;
		   memset(&Sms4470DeviceGroupInformation[Loop].SMS4470CurrentRespondentDVBTReceptionStatistic,0,sizeof(RECEPTION_STATISTICS_ST));
		   memset(&Sms4470DeviceGroupInformation[Loop].SMS4470CurrentRespondentDVBTTransmissionStatistic,0,sizeof(TRANSMISSION_STATISTICS_ST));
		   Sms4470DeviceGroupInformation[Loop].SMS4470DVBT2LastPlpIdSetup = 0xFF;
	   }

	   Sms4470InitializationFlag = TRUE;

	   Result = TRUE;
	}

    return Result;
}
VOID Sms4470CoreAPI_Uninitialization()
{
    SMS4470FunctionNamePrintf("(Sms4470CoreAPI_Uninitialization)\n");

	if(Sms4470InitializationFlag)
	   Sms4470InitializationFlag = FALSE;
}
BOOL Sms4470CoreAPI_HardwareReset(ULONG FrontendGroupID)
{
	ULONG Index;
	BOOL Result = FALSE;

    SMS4470FunctionNamePrintf("(Sms4470CoreAPI_HardwareReset)\n");

	if(Sms4470InitializationFlag)
	{
	   if(PlatformPorting_Frontend_HardwareReset(FrontendGroupID) == TRUE)
	   {
		  if(FrontendGroupID == FRONTEND_TWO_DIVERSITY_GROUP_ID)
		     Index = 0;
		  else
		     Index = 1;

		  Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode = UNKNOWN_WORKING_MODE;
          Sms4470DeviceGroupInformation[Index].Sms4470CurrentRF = 0;
		  Sms4470DeviceGroupInformation[Index].Sms4470CurrentBandWidth = UNKNOWN_BANDWIDTH;
		  Sms4470DeviceGroupInformation[Index].Sms4470PreviousRF = 0xFFFFFFFF;
		  Sms4470DeviceGroupInformation[Index].Sms4470PreviousBandWidth = UNKNOWN_BANDWIDTH;
		  Sms4470DeviceGroupInformation[Index].Sms4470PreviousDemodulatorTuningFeature = UNKNOWN_DEMODULATOR_TUNING;
		  Sms4470DeviceGroupInformation[Index].Sms4470DeviceAlreadyDownloadFirmwareFlag = FALSE;
          Sms4470DeviceGroupInformation[Index].SMS4470CurrentDemodulationModeForDvbtDvbt2DualWorkingMode = UNKNOWN_SMS4470_DEMODULATION_MODE;
#ifdef FRONTEND_ENABLE_REGULAR_TIMER_CALLBACK
		  Sms4470DeviceGroupInformation[Index].SMS4470CurrentStatisticInformationAvailableFlag = FALSE;
#endif
		  Sms4470DeviceGroupInformation[Index].SMS4470DVBT2LastPlpIdSetup = 0xFF;

		  Result = TRUE;
	   }
	}

	return Result;
}
BOOL Sms4470CoreAPI_SetWorkingMode(ULONG FrontendGroupID,FRONTEND_WORKING_MODE Mode)
{
	ULONG Index;
	BOOL Result = FALSE;

    SMS4470FunctionNamePrintf("(Sms4470CoreAPI_SetWorkingMode)\n");

	if(Sms4470InitializationFlag)
	{
	   if(FrontendGroupID == FRONTEND_TWO_DIVERSITY_GROUP_ID || FrontendGroupID == FRONTEND_FOUR_DIVERSITY_GROUP_ID)
	   {
		  if(FrontendGroupID == FRONTEND_TWO_DIVERSITY_GROUP_ID)
		     Index = 0;
		  else
		     Index = 1;

		  if(Sms4470DeviceGroupInformation[Index].Sms4470DeviceAlreadyDownloadFirmwareFlag == FALSE)
		  {
             if(Mode == DVBT_DVBT2_DUAL_WORKING_MODE)
			 {
// Step 1
                SMS4470InfoPrintf("(Sms4470CoreAPI_SetWorkingMode) setp 1: get version beofore loading FW\n");

	            if(SMS4470_get_version(FrontendGroupID) != TRUE)
	               return FALSE;
	
// Step 2
	            SMS4470InfoPrintf("(Sms4470CoreAPI_SetWorkingMode) step 2: load DVB-T2 fw\n");

	            //if(SMS4470_dual_mode_download_dvbt2_firmware_data(FrontendGroupID,SianoSMS4470_DVBT2_A2_FirmwareImaget) != TRUE)
	            //   return FALSE;

	            SMS4470SysDelay(500);	
		
// Step 3
                SMS4470InfoPrintf("(Sms4470CoreAPI_SetWorkingMode) setp 3 : get version after loading FW\n");

	            if(SMS4470_get_version(FrontendGroupID) != TRUE)
	               return FALSE;

	            SMS4470SysDelay(250);

// Step 4
                SMS4470InfoPrintf("(Sms4470CoreAPI_SetWorkingMode) setp 4 : send device init command\n");

                if(SMS4470_send_device_init(FrontendGroupID,DVBT2_DEMODULATOR_TUNING) != TRUE)
	               return FALSE;

// Step 55
                SMS4470InfoPrintf("(Sms4470CoreAPI_SetWorkingMode) setp 5 : set polling mode\n");

	            if(SMS4470_set_polling_mode(FrontendGroupID) != TRUE)
	               return FALSE;

// Step 6
                SMS4470InfoPrintf("(Sms4470CoreAPI_SetWorkingMode) setp 6 : send ts interface enable command\n");

 	            if(SMS4470_enable_ts_interface(FrontendGroupID) != TRUE)
	               return FALSE;
	
// Step 7
                SMS4470InfoPrintf("(Sms4470CoreAPI_SetWorkingMode) setp 7 : load DVB-T fw\n");

	     //       if(SMS4470_dual_mode_download_dvbt_firmware_data(FrontendGroupID,SianoSMS4470_DVBT_A2_FirmwareImaget) != TRUE)
	     //          return FALSE;

#ifdef SMS4470_DUAL_MODE_ENABLE_WORKING_AROUND
// Step 8
                SMS4470InfoPrintf("(Sms4470CoreAPI_SetWorkingMode) setp 8 : working around for the failure of first time DVB-T channel scan after system booting up.\n");
       
                if(SMS4470_dual_mode_SwitchStandard(FrontendGroupID) != TRUE)
	               return FALSE;

	            SMS4470SysDelay(SMS4470_DUAL_MODE_SWITCH_STANDARD_REBOOT_UP_DELAY);	// need to wait enough time to make sure MRC devices are boot up

	            if(SMS4470_get_version(FrontendGroupID) != TRUE)
	               return FALSE;

                if(SMS4470_send_device_init(FrontendGroupID,DVBT_DEMODULATOR_TUNING) != TRUE)
	               return FALSE;

				Sms4470DeviceGroupInformation[Index].Sms4470PreviousDemodulatorTuningFeature = DVBT_DEMODULATOR_TUNING;
                Sms4470DeviceGroupInformation[Index].SMS4470CurrentDemodulationModeForDvbtDvbt2DualWorkingMode = SMS4470_DVBT_DEMODULATION_MODE;
#else
				Sms4470DeviceGroupInformation[Index].Sms4470PreviousDemodulatorTuningFeature = DVBT2_DEMODULATOR_TUNING;
                Sms4470DeviceGroupInformation[Index].SMS4470CurrentDemodulationModeForDvbtDvbt2DualWorkingMode = SMS4470_DVBT2_DEMODULATION_MODE;
#endif				
			 }
		     else
			 {
// Step 1
                SMS4470InfoPrintf("(Sms4470CoreAPI_SetWorkingMode) setp 1: get version beofore loading FW\n");

	            if(SMS4470_get_version(FrontendGroupID) != TRUE)
	               return FALSE;
	
// Step 2
	            SMS4470InfoPrintf("(Sms4470CoreAPI_SetWorkingMode) step 2: load fw\n");

                if(Mode == DVBT_WORKING_MODE)
				{
                   if(SMS4470_single_mode_download_firmware_data(FrontendGroupID,SianoSMS4470_DVBT_A2_FirmwareImage) != TRUE)
	                  return FALSE;
				}
			    else
                if(Mode == DVBT2_WORKING_MODE)
				{
                   //if(SMS4470_single_mode_download_firmware_data(FrontendGroupID,SianoSMS4470_DVBT2_A2_FirmwareImage) != TRUE)
	               //   return FALSE;
				}
			    else
				{
					//YH: no ISDBT
                   //if(SMS4470_single_mode_download_firmware_data(FrontendGroupID,SianoSMS4470_ISDBT_A2_FirmwareImage) != TRUE)
	               //   return FALSE;
				}

	            SMS4470SysDelay(500);
	
// Step 3
                SMS4470InfoPrintf("(Sms4470CoreAPI_SetWorkingMode) setp 3 : get version after loading FW\n");

	            if(SMS4470_get_version(FrontendGroupID) != TRUE)
	               return FALSE;

// Step 4
                SMS4470InfoPrintf("(Sms4470CoreAPI_SetWorkingMode) setp 4 : set polling mode\n");

	            if(SMS4470_set_polling_mode(FrontendGroupID) != TRUE)
	               return FALSE;

// Step 5
                SMS4470InfoPrintf("(Sms4470CoreAPI_SetWorkingMode) setp 5 : send ts interface enable command\n");

 	            if(SMS4470_enable_ts_interface(FrontendGroupID) != TRUE)
	               return FALSE;

// Step 6

                SMS4470InfoPrintf("(Sms4470CoreAPI_SetWorkingMode) setp 6 : send device init command\n");

	            if(Mode == DVBT_WORKING_MODE)
				{
                   if(SMS4470_send_device_init(FrontendGroupID,DVBT_DEMODULATOR_TUNING) != TRUE)
	                  return FALSE;

				   Sms4470DeviceGroupInformation[Index].Sms4470PreviousDemodulatorTuningFeature = DVBT_DEMODULATOR_TUNING;
				}
				else
	            if(Mode == DVBT2_WORKING_MODE)
				{
                   if(SMS4470_send_device_init(FrontendGroupID,DVBT2_DEMODULATOR_TUNING) != TRUE)
	                  return FALSE;

				   Sms4470DeviceGroupInformation[Index].Sms4470PreviousDemodulatorTuningFeature = DVBT2_DEMODULATOR_TUNING;
				}
				else
				{
                   if(SMS4470_send_device_init(FrontendGroupID,ISDBT_13SEGMENT_DEMODULATOR_TUNING) != TRUE)
	                  return FALSE;

				   Sms4470DeviceGroupInformation[Index].Sms4470PreviousDemodulatorTuningFeature = ISDBT_13SEGMENT_DEMODULATOR_TUNING;
				}

	            SMS4470SysDelay(500);
			 }

		     Sms4470DeviceGroupInformation[Index].Sms4470DeviceAlreadyDownloadFirmwareFlag = TRUE;
             Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode = Mode;

		     Result = TRUE;
		  }
	   }
	}

	return Result;
}
BOOL Sms4470CoreAPI_SoftwareReset(ULONG FrontendGroupID)
{
	ULONG Index;
	BOOL Result = FALSE;

    SMS4470FunctionNamePrintf("(Sms4470CoreAPI_SoftwareReset)\n");

	if(Sms4470InitializationFlag)
	{
	   if(FrontendGroupID == FRONTEND_TWO_DIVERSITY_GROUP_ID)
		  Index = 0;
	   else
		  Index = 1;

	   if(Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode != UNKNOWN_WORKING_MODE)
	   {
          if(Sms4470DeviceGroupInformation[Index].Sms4470DeviceAlreadyDownloadFirmwareFlag)
		  {
	         if(SMS4470_send_device_init(FrontendGroupID,Sms4470DeviceGroupInformation[Index].Sms4470PreviousDemodulatorTuningFeature) == TRUE)
			 {
                Sms4470DeviceGroupInformation[Index].Sms4470CurrentRF = 0;
		        Sms4470DeviceGroupInformation[Index].Sms4470CurrentBandWidth = UNKNOWN_BANDWIDTH;
		        Sms4470DeviceGroupInformation[Index].Sms4470PreviousRF = 0xFFFFFFFF;
				Sms4470DeviceGroupInformation[Index].Sms4470PreviousBandWidth = UNKNOWN_BANDWIDTH;
#ifdef FRONTEND_ENABLE_REGULAR_TIMER_CALLBACK
		        Sms4470DeviceGroupInformation[Index].SMS4470CurrentStatisticInformationAvailableFlag = FALSE;
#endif
		        Sms4470DeviceGroupInformation[Index].SMS4470DVBT2LastPlpIdSetup = 0xFF;

		        Result = TRUE;
			 }
		  }
	   }
	}

	return Result;
}
BOOL Sms4470CoreAPI_SetFrequencyAndBandwidth(ULONG FrontendGroupID,ULONG RF,BANDWIDTH_TYPE BandWidth)
{
	ULONG Index;
	BOOL Result = FALSE;

    SMS4470FunctionNamePrintf("(Sms4470CoreAPI_SetFrequencyAndBandwidth)\n");
	
	if(Sms4470InitializationFlag)
	{
	   if(FrontendGroupID == FRONTEND_TWO_DIVERSITY_GROUP_ID)
		  Index = 0;
	   else
		  Index = 1;

	   if(Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode != UNKNOWN_WORKING_MODE)
	   {
          if(Sms4470DeviceGroupInformation[Index].Sms4470DeviceAlreadyDownloadFirmwareFlag)
		  {
	         if(Sms4470DeviceGroupInformation[Index].Sms4470PreviousRF != RF)
                Sms4470DeviceGroupInformation[Index].Sms4470CurrentRF = RF;
	   
             if(Sms4470DeviceGroupInformation[Index].Sms4470PreviousBandWidth != BandWidth)
		        Sms4470DeviceGroupInformation[Index].Sms4470CurrentBandWidth = BandWidth;

	         Result = TRUE;
		  }
	   }
	}

    return Result;
}
BOOL Sms4470CoreAPI_SetDemodulatorTuningFeature(ULONG FrontendGroupID,DEMODULATOR_TUNING_FEATURE DemodulatorTuningFeature)
{
	ULONG Index;
	BOOL Result = FALSE;

    SMS4470FunctionNamePrintf("(Sms4470CoreAPI_SetDemodulatorTuningFeature)\n");
	
	if(Sms4470InitializationFlag)
	{
	   if(FrontendGroupID == FRONTEND_TWO_DIVERSITY_GROUP_ID)
		  Index = 0;
	   else
		  Index = 1;

	   if(Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode != UNKNOWN_WORKING_MODE)
	   {
          if(Sms4470DeviceGroupInformation[Index].Sms4470DeviceAlreadyDownloadFirmwareFlag)
		  {
             if(Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == DVBT_DVBT2_DUAL_WORKING_MODE)
			 {
                if(DemodulatorTuningFeature == DVBT_DEMODULATOR_TUNING || DemodulatorTuningFeature == DVBH_DEMODULATOR_TUNING)
	            {
		           if(Sms4470DeviceGroupInformation[Index].SMS4470CurrentDemodulationModeForDvbtDvbt2DualWorkingMode != SMS4470_DVBT_DEMODULATION_MODE)
		           {
	                  if(SMS4470_dual_mode_SwitchStandard(FrontendGroupID) != TRUE)
						 return FALSE;

					  SMS4470SysDelay(SMS4470_DUAL_MODE_SWITCH_STANDARD_REBOOT_UP_DELAY);	// need to wait enough time to make sure MRC devices are boot up

	                  if(SMS4470_get_version(FrontendGroupID) != TRUE)
	                     return FALSE;

				      SMS4470SysDelay(250);

					  Sms4470DeviceGroupInformation[Index].Sms4470PreviousDemodulatorTuningFeature = UNKNOWN_DEMODULATOR_TUNING;

	                  if(SMS4470_send_device_init(FrontendGroupID,DemodulatorTuningFeature) != TRUE)
				         return FALSE;

	                  if(SMS4470_set_polling_mode(FrontendGroupID) != TRUE)
				         return FALSE;

 	                  if(SMS4470_enable_ts_interface(FrontendGroupID) != TRUE)
				         return FALSE;

		              Sms4470DeviceGroupInformation[Index].Sms4470PreviousRF = 0xFFFFFFFF;
					  Sms4470DeviceGroupInformation[Index].Sms4470PreviousBandWidth = UNKNOWN_BANDWIDTH;
#ifdef FRONTEND_ENABLE_REGULAR_TIMER_CALLBACK
		              Sms4470DeviceGroupInformation[Index].SMS4470CurrentStatisticInformationAvailableFlag = FALSE;
#endif
		              Sms4470DeviceGroupInformation[Index].SMS4470DVBT2LastPlpIdSetup = 0xFF;

			          if(SMS4470_RemovePidFilter(FrontendGroupID,0x2000) != TRUE)
				         return FALSE;

	                  if(SMS4470_tune(FrontendGroupID,Sms4470DeviceGroupInformation[Index].Sms4470CurrentRF,Sms4470DeviceGroupInformation[Index].Sms4470CurrentBandWidth,DemodulatorTuningFeature) != TRUE)
				         return FALSE;

			          if(SMS4470_AddPidFilter(FrontendGroupID,0x2000) != TRUE)
				         return FALSE;

					  Sms4470DeviceGroupInformation[Index].Sms4470PreviousDemodulatorTuningFeature = DemodulatorTuningFeature;
                      Sms4470DeviceGroupInformation[Index].Sms4470PreviousRF = Sms4470DeviceGroupInformation[Index].Sms4470CurrentRF;
                      Sms4470DeviceGroupInformation[Index].Sms4470PreviousBandWidth = Sms4470DeviceGroupInformation[Index].Sms4470CurrentBandWidth;
					  Sms4470DeviceGroupInformation[Index].SMS4470CurrentDemodulationModeForDvbtDvbt2DualWorkingMode = SMS4470_DVBT_DEMODULATION_MODE;
		           }
		           else
		           {
                      if(Sms4470DeviceGroupInformation[Index].Sms4470PreviousDemodulatorTuningFeature != DemodulatorTuningFeature)
					  {
	                     if(SMS4470_send_device_init(FrontendGroupID,DemodulatorTuningFeature) != TRUE)
				            return FALSE;
					  }

			          if(SMS4470_RemovePidFilter(FrontendGroupID,0x2000) != TRUE)
				         return FALSE;

	                  if(SMS4470_tune(FrontendGroupID,Sms4470DeviceGroupInformation[Index].Sms4470CurrentRF,Sms4470DeviceGroupInformation[Index].Sms4470CurrentBandWidth,DemodulatorTuningFeature) != TRUE)
				         return FALSE;

			          if(SMS4470_AddPidFilter(FrontendGroupID,0x2000) != TRUE)
				         return FALSE;

					  Sms4470DeviceGroupInformation[Index].Sms4470PreviousDemodulatorTuningFeature = DemodulatorTuningFeature;
                      Sms4470DeviceGroupInformation[Index].Sms4470PreviousRF = Sms4470DeviceGroupInformation[Index].Sms4470CurrentRF;
                      Sms4470DeviceGroupInformation[Index].Sms4470PreviousBandWidth = Sms4470DeviceGroupInformation[Index].Sms4470CurrentBandWidth;
		           }
	            }
                else
	            {
		           if(Sms4470DeviceGroupInformation[Index].SMS4470CurrentDemodulationModeForDvbtDvbt2DualWorkingMode != SMS4470_DVBT2_DEMODULATION_MODE)
		           {
	                  if(SMS4470_dual_mode_SwitchStandard(FrontendGroupID) != TRUE)
				         return FALSE;

				      SMS4470SysDelay(SMS4470_DUAL_MODE_SWITCH_STANDARD_REBOOT_UP_DELAY);	// need to wait enough time to make sure MRC devices are boot up

	                  if(SMS4470_get_version(FrontendGroupID) != TRUE)
				         return FALSE;

				      SMS4470SysDelay(250);

					  Sms4470DeviceGroupInformation[Index].Sms4470PreviousDemodulatorTuningFeature = UNKNOWN_DEMODULATOR_TUNING;

	                  if(SMS4470_send_device_init(FrontendGroupID,DemodulatorTuningFeature) != TRUE)
				         return FALSE;

	                  if(SMS4470_set_polling_mode(FrontendGroupID) != TRUE)
				         return FALSE;

 	                  if(SMS4470_enable_ts_interface(FrontendGroupID) != TRUE)
				         return FALSE;

		              Sms4470DeviceGroupInformation[Index].Sms4470PreviousRF = 0xFFFFFFFF;
					  Sms4470DeviceGroupInformation[Index].Sms4470PreviousBandWidth = UNKNOWN_BANDWIDTH;
#ifdef FRONTEND_ENABLE_REGULAR_TIMER_CALLBACK
		              Sms4470DeviceGroupInformation[Index].SMS4470CurrentStatisticInformationAvailableFlag = FALSE;
#endif
		              Sms4470DeviceGroupInformation[Index].SMS4470DVBT2LastPlpIdSetup = 0xFF;
				
				      if(SMS4470_RemovePidFilter(FrontendGroupID,0x2000) != TRUE)
				         return FALSE;

	                  if(SMS4470_tune(FrontendGroupID,Sms4470DeviceGroupInformation[Index].Sms4470CurrentRF,Sms4470DeviceGroupInformation[Index].Sms4470CurrentBandWidth,DemodulatorTuningFeature) != TRUE)
				         return FALSE;

				      if(SMS4470_AddPidFilter(FrontendGroupID,0x2000) != TRUE)
				         return FALSE;

					  Sms4470DeviceGroupInformation[Index].Sms4470PreviousDemodulatorTuningFeature = DemodulatorTuningFeature;
                      Sms4470DeviceGroupInformation[Index].Sms4470PreviousRF = Sms4470DeviceGroupInformation[Index].Sms4470CurrentRF;
                      Sms4470DeviceGroupInformation[Index].Sms4470PreviousBandWidth = Sms4470DeviceGroupInformation[Index].Sms4470CurrentBandWidth;
					  Sms4470DeviceGroupInformation[Index].SMS4470CurrentDemodulationModeForDvbtDvbt2DualWorkingMode = SMS4470_DVBT2_DEMODULATION_MODE;
		           }
		           else
		           {
                      if(Sms4470DeviceGroupInformation[Index].Sms4470PreviousDemodulatorTuningFeature != DemodulatorTuningFeature)
					  {
	                     if(SMS4470_send_device_init(FrontendGroupID,DemodulatorTuningFeature) != TRUE)
				            return FALSE;
					  }

				      if(SMS4470_RemovePidFilter(FrontendGroupID,0x2000) != TRUE)
				         return FALSE;

	                  if(SMS4470_tune(FrontendGroupID,Sms4470DeviceGroupInformation[Index].Sms4470CurrentRF,Sms4470DeviceGroupInformation[Index].Sms4470CurrentBandWidth,DemodulatorTuningFeature) != TRUE)
				         return FALSE;

				      if(SMS4470_AddPidFilter(FrontendGroupID,0x2000) != TRUE)
				         return FALSE;

					  Sms4470DeviceGroupInformation[Index].Sms4470PreviousDemodulatorTuningFeature = DemodulatorTuningFeature;
                      Sms4470DeviceGroupInformation[Index].Sms4470PreviousRF = Sms4470DeviceGroupInformation[Index].Sms4470CurrentRF;
                      Sms4470DeviceGroupInformation[Index].Sms4470PreviousBandWidth = Sms4470DeviceGroupInformation[Index].Sms4470CurrentBandWidth;
		           }
	            }

				Result = TRUE;
			 }
			 else
			 if(Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == DVBT2_WORKING_MODE &&
				(DemodulatorTuningFeature == DVBT2_DEMODULATOR_TUNING || DemodulatorTuningFeature == DVBT2_LITE_DEMODULATOR_TUNING))
			 {
                if(Sms4470DeviceGroupInformation[Index].Sms4470PreviousDemodulatorTuningFeature != DemodulatorTuningFeature)
				{
	               if(SMS4470_send_device_init(FrontendGroupID,DemodulatorTuningFeature) != TRUE)
				      return FALSE;
				}

				if(SMS4470_RemovePidFilter(FrontendGroupID,0x2000) != TRUE)
				   return FALSE;

	            if(SMS4470_tune(FrontendGroupID,Sms4470DeviceGroupInformation[Index].Sms4470CurrentRF,Sms4470DeviceGroupInformation[Index].Sms4470CurrentBandWidth,DemodulatorTuningFeature) != TRUE)
				   return FALSE;

				if(SMS4470_AddPidFilter(FrontendGroupID,0x2000) != TRUE)
				   return FALSE;

				Sms4470DeviceGroupInformation[Index].Sms4470PreviousDemodulatorTuningFeature = DemodulatorTuningFeature;
                Sms4470DeviceGroupInformation[Index].Sms4470PreviousRF = Sms4470DeviceGroupInformation[Index].Sms4470CurrentRF;
                Sms4470DeviceGroupInformation[Index].Sms4470PreviousBandWidth = Sms4470DeviceGroupInformation[Index].Sms4470CurrentBandWidth;

				Result = TRUE;
			 }
			 else
			 if(Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == DVBT_WORKING_MODE &&
				(DemodulatorTuningFeature == DVBT_DEMODULATOR_TUNING || DemodulatorTuningFeature == DVBH_DEMODULATOR_TUNING))
			 {
                if(Sms4470DeviceGroupInformation[Index].Sms4470PreviousDemodulatorTuningFeature != DemodulatorTuningFeature)
				{
	               if(SMS4470_send_device_init(FrontendGroupID,DemodulatorTuningFeature) != TRUE)
				      return FALSE;
				}

				if(SMS4470_RemovePidFilter(FrontendGroupID,0x2000) != TRUE)
				   return FALSE;

	            if(SMS4470_tune(FrontendGroupID,Sms4470DeviceGroupInformation[Index].Sms4470CurrentRF,Sms4470DeviceGroupInformation[Index].Sms4470CurrentBandWidth,DemodulatorTuningFeature) != TRUE)
				   return FALSE;

				if(SMS4470_AddPidFilter(FrontendGroupID,0x2000) != TRUE)
				   return FALSE;

				Sms4470DeviceGroupInformation[Index].Sms4470PreviousDemodulatorTuningFeature = DemodulatorTuningFeature;
                Sms4470DeviceGroupInformation[Index].Sms4470PreviousRF = Sms4470DeviceGroupInformation[Index].Sms4470CurrentRF;
                Sms4470DeviceGroupInformation[Index].Sms4470PreviousBandWidth = Sms4470DeviceGroupInformation[Index].Sms4470CurrentBandWidth;

				Result = TRUE;
			 }
			 else
			 if(Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == ISDBT_WORKING_MODE &&
				(DemodulatorTuningFeature == ISDBT_13SEGMENT_DEMODULATOR_TUNING || DemodulatorTuningFeature == ISDBT_1SEGMENT_DEMODULATOR_TUNING))
			 {
                if(Sms4470DeviceGroupInformation[Index].Sms4470PreviousDemodulatorTuningFeature != DemodulatorTuningFeature)
				{
	               if(SMS4470_send_device_init(FrontendGroupID,DemodulatorTuningFeature) != TRUE)
				      return FALSE;
				}

				if(SMS4470_RemovePidFilter(FrontendGroupID,0x2000) != TRUE)
				   return FALSE;

	            if(SMS4470_tune(FrontendGroupID,Sms4470DeviceGroupInformation[Index].Sms4470CurrentRF,Sms4470DeviceGroupInformation[Index].Sms4470CurrentBandWidth,DemodulatorTuningFeature) != TRUE)
				   return FALSE;

				if(SMS4470_AddPidFilter(FrontendGroupID,0x2000) != TRUE)
				   return FALSE;

				Sms4470DeviceGroupInformation[Index].Sms4470PreviousDemodulatorTuningFeature = DemodulatorTuningFeature;
                Sms4470DeviceGroupInformation[Index].Sms4470PreviousRF = Sms4470DeviceGroupInformation[Index].Sms4470CurrentRF;
                Sms4470DeviceGroupInformation[Index].Sms4470PreviousBandWidth = Sms4470DeviceGroupInformation[Index].Sms4470CurrentBandWidth;

				Result = TRUE;
			 }
			 else
			 {
                SMS4470ErrorPrintf("(Sms4470CoreAPI_SetDemodulatorTuningFeature) xxxxxxxxx Sms4470CurrentWorkingMode and DemodulatorTuningFeature are not matched !\n");
			 }
		  }
	   }
	}
	
	return Result;
}
BOOL Sms4470CoreAPI_CheckLockStatusForChannelSearch(ULONG FrontendGroupID,UBYTE MaxCheckCounter,BOOL* pLockStatus)
{
	ULONG Index;
	ULONG Loop;
	ULONG CheckPlpInfoLoop;
	BOOL LockStatus = FALSE;
	BOOL DVBT2PhysicalLayerPipeInformationExistFlag = FALSE;
	BOOL Result = FALSE;

    SMS4470FunctionNamePrintf("(Sms4470CoreAPI_CheckLockStatusForChannelSearch)\n");

	if(Sms4470InitializationFlag)
	{
	   if(pLockStatus)
	   {
		  *pLockStatus = FALSE;

	      if(FrontendGroupID == FRONTEND_TWO_DIVERSITY_GROUP_ID)
		     Index = 0;
	      else
		     Index = 1;

	      if(Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode != UNKNOWN_WORKING_MODE)
	      {
             if(Sms4470DeviceGroupInformation[Index].Sms4470DeviceAlreadyDownloadFirmwareFlag)
		     {
	             while(1)
			     {
					   Result = TRUE;

					   if((Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == DVBT_DVBT2_DUAL_WORKING_MODE && Sms4470DeviceGroupInformation[Index].SMS4470CurrentDemodulationModeForDvbtDvbt2DualWorkingMode == SMS4470_DVBT2_DEMODULATION_MODE) ||
						  Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == DVBT2_WORKING_MODE)
					   {
		                  Sms4470DeviceGroupInformation[Index].SMS4470DetectNoSignalFlag = FALSE;

                          if(MaxCheckCounter > 1)
					      {
                             for(Loop=0;Loop<(MaxCheckCounter - 1);Loop++)
						     {                       
							     if(SMS4470_check_signal(FrontendGroupID,&LockStatus) == TRUE)
								 {
                                    if(LockStatus)
									{
					                   for(CheckPlpInfoLoop=0;CheckPlpInfoLoop<6;CheckPlpInfoLoop++)
									   {
					                       if(Sms4470DeviceGroupInformation[Index].SMS4470DetectNoSignalFlag)
					                          break;
	
					                       SMS4470ReadResponseWithDelay(FrontendGroupID,500);

					                       if(Sms4470DeviceGroupInformation[Index].SMS4470DetectNoSignalFlag)
					                          break;

				                           if(SMS4470_CheckDVBT2PhysicalLayerPipeInformation(FrontendGroupID,&DVBT2PhysicalLayerPipeInformationExistFlag) == TRUE)
										   {
											  if(DVBT2PhysicalLayerPipeInformationExistFlag)
											  {
												 *pLockStatus = TRUE;

							                     break;
											  }
										   }
										   else
										   {
                                              Result = FALSE;
											  
											  break;
										   }
									   }
				                       break;
									}

					                if(Sms4470DeviceGroupInformation[Index].SMS4470DetectNoSignalFlag)
					                   break;
	
					                SMS4470ReadResponseWithDelay(FrontendGroupID,500);

					                if(Sms4470DeviceGroupInformation[Index].SMS4470DetectNoSignalFlag)
					                   break;
								 }
								 else
								 {
									Result = FALSE;

									break;
								 }
						     }
					      }
			              if(DVBT2PhysicalLayerPipeInformationExistFlag == FALSE && Sms4470DeviceGroupInformation[Index].SMS4470DetectNoSignalFlag == FALSE)
					      {
							 if(SMS4470_check_signal(FrontendGroupID,&LockStatus) == TRUE)
							 {
                                if(LockStatus)
								{
					               for(CheckPlpInfoLoop=0;CheckPlpInfoLoop<6;CheckPlpInfoLoop++)
								   {
					                   if(Sms4470DeviceGroupInformation[Index].SMS4470DetectNoSignalFlag)
					                      break;
	
					                   SMS4470ReadResponseWithDelay(FrontendGroupID,500);

					                   if(Sms4470DeviceGroupInformation[Index].SMS4470DetectNoSignalFlag)
					                      break;

				                       if(SMS4470_CheckDVBT2PhysicalLayerPipeInformation(FrontendGroupID,&DVBT2PhysicalLayerPipeInformationExistFlag) == TRUE)
									   {
										  if(DVBT2PhysicalLayerPipeInformationExistFlag)
										  {
											 *pLockStatus = TRUE;

							                 break;
										  }
									   }
									   else
									   {
                                          Result = FALSE;
											  
										  break;
									   }
								   }
								}
							 }
							 else
								Result = FALSE;
					      }
					   }
			           else
					   {
		                  Sms4470DeviceGroupInformation[Index].SMS4470DetectNoSignalFlag = FALSE;

                          if(MaxCheckCounter > 1)
					      {
                             for(Loop=0;Loop<(MaxCheckCounter - 1);Loop++)
						     {                       
							     if(SMS4470_check_signal(FrontendGroupID,&LockStatus) == TRUE)
								 {
                                    if(LockStatus)
									{
									   *pLockStatus = TRUE;

							           break;
									}
								 }
								 else
								 {
                                    Result = FALSE;
											  
									break;
								 }

					             if(Sms4470DeviceGroupInformation[Index].SMS4470DetectNoSignalFlag)
					                break;
	
					             SMS4470ReadResponseWithDelay(FrontendGroupID,500);

					             if(Sms4470DeviceGroupInformation[Index].SMS4470DetectNoSignalFlag)
					                break;
							 }
					      }
			              if(LockStatus == FALSE && Sms4470DeviceGroupInformation[Index].SMS4470DetectNoSignalFlag == FALSE)
					      {
	                         Result = SMS4470_check_signal(FrontendGroupID,&LockStatus);
							 if(Result)
							 {
                                if(LockStatus)
								   *pLockStatus = TRUE;
							 }
					      }
			           }
			           break;
			     }
		     }
	      }
	   }
	}

    return Result;
}
BOOL Sms4470CoreAPI_CheckLockStatus(ULONG FrontendGroupID,BOOL* pLockStatus)
{
	ULONG Index;
	BOOL LockStatus;
	BOOL DVBT2PhysicalLayerPipeInformationExistFlag;
	BOOL Result = FALSE;

    SMS4470FunctionNamePrintf("(Sms4470CoreAPI_CheckLockStatus)\n");

	if(Sms4470InitializationFlag)
	{
	   if(pLockStatus)
	   {
		  *pLockStatus = FALSE;

	      if(FrontendGroupID == FRONTEND_TWO_DIVERSITY_GROUP_ID)
		     Index = 0;
	      else
		     Index = 1;

	      if(Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode != UNKNOWN_WORKING_MODE)
		  {
             if(Sms4470DeviceGroupInformation[Index].Sms4470DeviceAlreadyDownloadFirmwareFlag)
			 {
	            if(SMS4470_check_signal(FrontendGroupID,&LockStatus) == TRUE)
				{
	               if(LockStatus)
				   {
					  if((Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == DVBT_DVBT2_DUAL_WORKING_MODE && Sms4470DeviceGroupInformation[Index].SMS4470CurrentDemodulationModeForDvbtDvbt2DualWorkingMode == SMS4470_DVBT2_DEMODULATION_MODE) ||
						 Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == DVBT2_WORKING_MODE)
					  {
						 if(SMS4470_CheckDVBT2PhysicalLayerPipeInformation(FrontendGroupID,&DVBT2PhysicalLayerPipeInformationExistFlag) == TRUE)
						 {
                            *pLockStatus = DVBT2PhysicalLayerPipeInformationExistFlag;

							Result = TRUE;
						 }
					  }
					  else
					  {
						 *pLockStatus = TRUE;

                         Result = TRUE;
					  }
				   }
				   else
                      Result = TRUE;
				}
			 }
		  }
	   }
	}

    return Result;
}
BOOL Sms4470CoreAPI_GetStatistics(ULONG FrontendGroupID,PSTATISTICS_INFORMATION pStatistics)
{
	ULONG Index;
	BOOL Result = FALSE;

    SMS4470FunctionNamePrintf("(DigiBestAPI_FrontendPorting_DigitalTerrestrialDVBT2DemodulatorSearchPhysicalLayerPipeInformation)\n");

	if(Sms4470InitializationFlag)
	{
	   if(pStatistics)
	   {
	      if(FrontendGroupID == FRONTEND_TWO_DIVERSITY_GROUP_ID)
		     Index = 0;
	      else
		     Index = 1;

	      if(Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode != UNKNOWN_WORKING_MODE)
		  {
             if(Sms4470DeviceGroupInformation[Index].Sms4470DeviceAlreadyDownloadFirmwareFlag)
			 {
#ifdef FRONTEND_ENABLE_REGULAR_TIMER_CALLBACK

				if(Sms4470DeviceGroupInformation[Index].SMS4470CurrentStatisticInformationAvailableFlag)
				   memcpy(pStatistics,&Sms4470DeviceGroupInformation[Index].SMS4470CurrentStatisticInformation,sizeof(STATISTICS_INFORMATION));
                else
				{
				   pStatistics->IsSignalLocked = FALSE;
				   pStatistics->StatisticAvailableFlag = FALSE;
				}

				Result = TRUE;

#else

	            Result = SMS4470_GetReceptionStatistics(FrontendGroupID,pStatistics);

#endif
			 }
		  }
	   }
	}

    return Result;
}
BOOL Sms4470CoreAPI_SetupChannelDVBT2PlpID(ULONG FrontendGroupID,UBYTE PlpID)
{
	ULONG Index;
	BOOL Result = FALSE;

    SMS4470FunctionNamePrintf("(Sms4470CoreAPI_SetupChannelDVBT2PlpID)\n");

	if(Sms4470InitializationFlag)
	{
	   if(FrontendGroupID == FRONTEND_TWO_DIVERSITY_GROUP_ID)
		  Index = 0;
	   else
		  Index = 1;

	   if(Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode != UNKNOWN_WORKING_MODE)
	   {
          if(Sms4470DeviceGroupInformation[Index].Sms4470DeviceAlreadyDownloadFirmwareFlag)
		  {
			 if((Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == DVBT_DVBT2_DUAL_WORKING_MODE && Sms4470DeviceGroupInformation[Index].SMS4470CurrentDemodulationModeForDvbtDvbt2DualWorkingMode == SMS4470_DVBT2_DEMODULATION_MODE) ||
				Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == DVBT2_WORKING_MODE)
			 {
                if(Sms4470DeviceGroupInformation[Index].SMS4470DVBT2LastPlpIdSetup != 0xFF)
				{
				   if(SMS4470_ClosePlp(FrontendGroupID,(UINT32)Sms4470DeviceGroupInformation[Index].SMS4470DVBT2LastPlpIdSetup) == TRUE)
				   {					
					  if(PlpID != 0xFF)
					  {
                         if(SMS4470_OpenPlp(FrontendGroupID,(UINT32)PlpID) == TRUE)
						 {
                            Sms4470DeviceGroupInformation[Index].SMS4470DVBT2LastPlpIdSetup = PlpID;

							Result = TRUE;
						 }
						 else
							Sms4470DeviceGroupInformation[Index].SMS4470DVBT2LastPlpIdSetup = 0xFF;
					  }
					  else
					  {
						 Sms4470DeviceGroupInformation[Index].SMS4470DVBT2LastPlpIdSetup = 0xFF;

						 Result = TRUE;
					  }
				   }
				}
				else
				{
				   if(PlpID != 0xFF)
				   {
                      if(SMS4470_OpenPlp(FrontendGroupID,(UINT32)PlpID) == TRUE)
					  {
                         Sms4470DeviceGroupInformation[Index].SMS4470DVBT2LastPlpIdSetup = PlpID;

					     Result = TRUE;
					  }
				   }
				   else
					  Result = TRUE;
				}
			 }
		  }
	   }
	}

    return Result;
}
#ifdef FRONTEND_ENABLE_REGULAR_TIMER_CALLBACK
BOOL Sms4470CoreAPI_TimerCallback(ULONG FrontendGroupID)
{
	ULONG Index;
	BOOL Result = FALSE;

    SMS4470FunctionNamePrintf("(Sms4470CoreAPI_TimerCallback)\n");

	if(Sms4470InitializationFlag)
	{
	   if(FrontendGroupID == FRONTEND_TWO_DIVERSITY_GROUP_ID)
		  Index = 0;
	   else
		  Index = 1;

	   if(Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode != UNKNOWN_WORKING_MODE)
	   {
          if(Sms4470DeviceGroupInformation[Index].Sms4470DeviceAlreadyDownloadFirmwareFlag)
		  {
	         Result = SMS4470_GetReceptionStatistics(FrontendGroupID,&Sms4470DeviceGroupInformation[Index].SMS4470CurrentStatisticInformation);

             Sms4470DeviceGroupInformation[Index].SMS4470CurrentStatisticInformationAvailableFlag = Result;
		  }
	   }
	}

    return Result;
}
#endif

//********************************************************************************************
//********************************************************************************************
//********************************************************************************************
//********************************************************************************************
//********************************************************************************************
//********************************************************************************************
BOOL SMS4470_tune(ULONG FrontendGroupID,ULONG Frequency,BANDWIDTH_TYPE BandWidth,DEMODULATOR_TUNING_FEATURE DemodulatorTuningFeature)
{
	ULONG         Index;
	SmsMsgData4Args_ST SmsMsg = {{0}};
    UINT32        i;
	UINT32        ResponseResult;
	UINT16        TempMsgLength;

	SMS4470FunctionNamePrintf("(SMS4470_tune)\n");
	SMS4470DebugPrintf("(SMS4470_tune) Start\n");
	SMS4470DebugPrintf("(SMS4470_tune) Frequency : %d\n",Frequency);
	SMS4470DebugPrintf("(SMS4470_tune) BandWidth : 0x%x\n",BandWidth);

	if(FrontendGroupID == FRONTEND_TWO_DIVERSITY_GROUP_ID)
	   Index = 0;
	else
	   Index = 1;

	SMS4470DebugPrintf("(SMS4470_tune) Sms4470DeviceGroupInformation[%d].SMS4470CurrentDemodulationModeForDvbtDvbt2DualWorkingMode : 0x%x\n",Index,Sms4470DeviceGroupInformation[Index].SMS4470CurrentDemodulationModeForDvbtDvbt2DualWorkingMode);

	if(Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == ISDBT_WORKING_MODE)
       SmsMsg.xMsgHeader.msgType   = MSG_SMS_ISDBT_TUNE_REQ;
	else
       SmsMsg.xMsgHeader.msgType   = MSG_SMS_RF_TUNE_REQ;
    SmsMsg.xMsgHeader.msgSrcId  = SMS_HOST_LIB; // DVBT_BDA_CONTROL_MSG_ID; // 0;
    SmsMsg.xMsgHeader.msgDstId  = HIF_TASK;
	if(Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == ISDBT_WORKING_MODE)
       SmsMsg.xMsgHeader.msgLength = (UINT16)sizeof(SmsMsg);
	else
       SmsMsg.xMsgHeader.msgLength = (UINT16)sizeof(SmsMsgData3Args_ST);
    SmsMsg.xMsgHeader.msgFlags  = MSG_HDR_FLAG_STATIC_MSG;//0;
    SmsMsg.msgData[0]           = Frequency;
    SmsMsg.msgData[2]           = 12000000;
	if(Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == ISDBT_WORKING_MODE)
	{
	   if(DemodulatorTuningFeature == ISDBT_13SEGMENT_DEMODULATOR_TUNING)
          SmsMsg.msgData[3]        = 13;
	   else
          SmsMsg.msgData[3]        = 1;
	}

    switch (BandWidth) 
	{
	     case BANDWIDTH_8_MHZ:
	          if((Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == DVBT_DVBT2_DUAL_WORKING_MODE && Sms4470DeviceGroupInformation[Index].SMS4470CurrentDemodulationModeForDvbtDvbt2DualWorkingMode == SMS4470_DVBT2_DEMODULATION_MODE) ||
	             Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == DVBT2_WORKING_MODE)
		         SmsMsg.msgData[1] = SIANO_BANDWIDTH_8M; // SIANO_BANDWIDTH_8M; // SIANO_BANDWIDTH_T2_8_MHZ;
              else
			  {
				 if(Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == ISDBT_WORKING_MODE)
				 {
				    if(DemodulatorTuningFeature == ISDBT_13SEGMENT_DEMODULATOR_TUNING)
					   SmsMsg.msgData[1] = SIANO_BANDWIDTH_13SEG_8MHZ;
				    else
					   SmsMsg.msgData[1] = SIANO_BANDWIDTH_1SEG_8MHZ;
				 }
				 else
			        SmsMsg.msgData[1] = SIANO_BANDWIDTH_8M;
			  }
		      break;
	     case BANDWIDTH_7_MHZ:
	          if((Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == DVBT_DVBT2_DUAL_WORKING_MODE && Sms4470DeviceGroupInformation[Index].SMS4470CurrentDemodulationModeForDvbtDvbt2DualWorkingMode == SMS4470_DVBT2_DEMODULATION_MODE) ||
	             Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == DVBT2_WORKING_MODE)
		         SmsMsg.msgData[1] = SIANO_BANDWIDTH_7M; // SIANO_BANDWIDTH_7M; // SIANO_BANDWIDTH_T2_7_MHZ;
              else
			  {
				 if(Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == ISDBT_WORKING_MODE)
				 {
				    if(DemodulatorTuningFeature == ISDBT_13SEGMENT_DEMODULATOR_TUNING)
					   SmsMsg.msgData[1] = SIANO_BANDWIDTH_13SEG_7MHZ;
				    else
					   SmsMsg.msgData[1] = SIANO_BANDWIDTH_1SEG_7MHZ;
				 }
				 else
			        SmsMsg.msgData[1] = SIANO_BANDWIDTH_7M;
			  }
		      break;
	     case BANDWIDTH_6_MHZ:
	          if((Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == DVBT_DVBT2_DUAL_WORKING_MODE && Sms4470DeviceGroupInformation[Index].SMS4470CurrentDemodulationModeForDvbtDvbt2DualWorkingMode == SMS4470_DVBT2_DEMODULATION_MODE) ||
	             Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == DVBT2_WORKING_MODE)
		         SmsMsg.msgData[1] = SIANO_BANDWIDTH_6M; // SIANO_BANDWIDTH_6M; // SIANO_BANDWIDTH_T2_6_MHZ;
              else
			  {
				 if(Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == ISDBT_WORKING_MODE)
				 {
				    if(DemodulatorTuningFeature == ISDBT_13SEGMENT_DEMODULATOR_TUNING)
					   SmsMsg.msgData[1] = SIANO_BANDWIDTH_13_SEG;
				    else
				  	   SmsMsg.msgData[1] = SIANO_BANDWIDTH_1_SEG;
				 }
				 else
			        SmsMsg.msgData[1] = SIANO_BANDWIDTH_6M;
			  }
		      break;
		 case BANDWIDTH_5_MHZ:
	          if((Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == DVBT_DVBT2_DUAL_WORKING_MODE && Sms4470DeviceGroupInformation[Index].SMS4470CurrentDemodulationModeForDvbtDvbt2DualWorkingMode == SMS4470_DVBT2_DEMODULATION_MODE) ||
	             Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == DVBT2_WORKING_MODE)
			     SmsMsg.msgData[1] = SIANO_BANDWIDTH_5M; // SIANO_BANDWIDTH_5M; // SIANO_BANDWIDTH_T2_5_MHZ;
              else
			  {
				 if(Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == ISDBT_WORKING_MODE)
				 {
				    if(DemodulatorTuningFeature == ISDBT_13SEGMENT_DEMODULATOR_TUNING)
					   SmsMsg.msgData[1] = SIANO_BANDWIDTH_13_SEG;
				    else
					   SmsMsg.msgData[1] = SIANO_BANDWIDTH_1_SEG;
				 }
				 else
			        SmsMsg.msgData[1] = SIANO_BANDWIDTH_5M;
			  }
		      break;
	     default:
	          if((Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == DVBT_DVBT2_DUAL_WORKING_MODE && Sms4470DeviceGroupInformation[Index].SMS4470CurrentDemodulationModeForDvbtDvbt2DualWorkingMode == SMS4470_DVBT2_DEMODULATION_MODE) ||
	             Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == DVBT2_WORKING_MODE)
		         SmsMsg.msgData[1] = SIANO_BANDWIDTH_8M; // SIANO_BANDWIDTH_8M; // SIANO_BANDWIDTH_T2_8_MHZ;
              else
			  {
				 if(Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == ISDBT_WORKING_MODE)
				 {
				    if(DemodulatorTuningFeature == ISDBT_13SEGMENT_DEMODULATOR_TUNING)
					   SmsMsg.msgData[1] = SIANO_BANDWIDTH_13_SEG;
				    else
					   SmsMsg.msgData[1] = SIANO_BANDWIDTH_1_SEG;
				 }
				 else
			        SmsMsg.msgData[1] = SIANO_BANDWIDTH_8M;
			  }
		      break;
	}
   
SMS4470DebugPrintf("(SMS4470_tune) Send command (MSG_SMS_RF_TUNE_REQ) to SMS4470\n");

    TempMsgLength = SmsMsg.xMsgHeader.msgLength;
#ifdef SMS_BIGENDIAN
    SmsMsg.xMsgHeader.msgType    = SMS_ENDIAN_SWAP16(SmsMsg.xMsgHeader.msgType);
    SmsMsg.xMsgHeader.msgLength  = SMS_ENDIAN_SWAP16(SmsMsg.xMsgHeader.msgLength);
    SmsMsg.xMsgHeader.msgFlags   = SMS_ENDIAN_SWAP16(SmsMsg.xMsgHeader.msgFlags);
    SmsMsg.msgData[0]            = SMS_ENDIAN_SWAP32(SmsMsg.msgData[0]);
    SmsMsg.msgData[1]            = SMS_ENDIAN_SWAP32(SmsMsg.msgData[1]);
    SmsMsg.msgData[2]            = SMS_ENDIAN_SWAP32(SmsMsg.msgData[2]);
    SmsMsg.msgData[3]            = SMS_ENDIAN_SWAP32(SmsMsg.msgData[3]);
#endif

//    SMS4470SysDelay(5);
	SMS4470I2CWrite(FrontendGroupID,(UINT8*)&SmsMsg,TempMsgLength);
	
SMS4470DebugPrintf("(SMS4470_tune) Get respence from SMS4470\n");

    for(i=0; i<CommonCommandWaitResponseLoop; i++)
    {
        SMS4470SysDelay(10);
        if(Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == ISDBT_WORKING_MODE)
           ResponseResult = SMS4470_read_response(FrontendGroupID,MSG_SMS_ISDBT_TUNE_RES, Sms4470DeviceGroupInformation[Index].Sms4470MessageRxBuffer, SIANO_IIC_RX_BUFFER_SIZE);
		else
           ResponseResult = SMS4470_read_response(FrontendGroupID,MSG_SMS_RF_TUNE_RES, Sms4470DeviceGroupInformation[Index].Sms4470MessageRxBuffer, SIANO_IIC_RX_BUFFER_SIZE);
        if(ResponseResult == 0xffff)
		{
		   break;
		}
		else
		if(ResponseResult == 0xf000)
		{
           i = CommonCommandWaitResponseLoop;
		   break;
		}
    }
    
    if(i == CommonCommandWaitResponseLoop)
	{
	   SMS4470ErrorPrintf("(SMS4470_tune) xxxxxxxxx Get response failed !\n");	
	}
 
	SMS4470DebugPrintf("(SMS4470_tune) End\n");	
    
    return TRUE;
}
//**********************************************************************************************************
//**********************************************************************************************************
//**********************************************************************************************************
BOOL SMS4470_get_version(ULONG FrontendGroupID)
{
	ULONG                   Index;
	UINT32                  i;
	UINT32                  ResponseResult;
	SmsMsgData_ST           SmsMsg = {{0}};
	SmsVersionRes_ST*       pVer;
	int ret;
	UINT16                  TempMsgLength;

    SMS4470FunctionNamePrintf("(SMS4470_get_version)\n");

	if(FrontendGroupID == FRONTEND_TWO_DIVERSITY_GROUP_ID)
	   Index = 0;
	else
	   Index = 1;


    SmsMsg.xMsgHeader.msgType   = MSG_SMS_GET_VERSION_EX_REQ;
	SmsMsg.xMsgHeader.msgSrcId  = SMS_HOST_LIB;
	SmsMsg.xMsgHeader.msgDstId  = HIF_TASK;
	SmsMsg.xMsgHeader.msgLength = sizeof(SmsMsgHdr_ST);
	SmsMsg.xMsgHeader.msgFlags  = MSG_HDR_FLAG_STATIC_MSG;

   
SMS4470DebugPrintf("(SMS4470_get_version) Send command (MSG_SMS_GET_VERSION_EX_REQ) to SMS4470\n");

    TempMsgLength = SmsMsg.xMsgHeader.msgLength;
#ifdef SMS_BIGENDIAN
    SmsMsg.xMsgHeader.msgType    = SMS_ENDIAN_SWAP16(SmsMsg.xMsgHeader.msgType);
    SmsMsg.xMsgHeader.msgLength  = SMS_ENDIAN_SWAP16(SmsMsg.xMsgHeader.msgLength);
    SmsMsg.xMsgHeader.msgFlags   = SMS_ENDIAN_SWAP16(SmsMsg.xMsgHeader.msgFlags);
#endif

//    SMS4470SysDelay(5);
	SMS4470I2CWrite(FrontendGroupID,(UINT8*)&SmsMsg,TempMsgLength);
	
   
SMS4470DebugPrintf("(SMS4470_get_version) Get respence from SMS4470\n");

//	memset(Sms4470DeviceGroupInformation[Index].Sms4470MessageRxBuffer,0xaa,SIANO_IIC_RX_BUFFER_SIZE);

	for(i=0; i<CommonCommandWaitResponseLoop; i++)
	{
        SMS4470SysDelay(10);
		ResponseResult = SMS4470_read_response(FrontendGroupID,MSG_SMS_GET_VERSION_EX_RES,Sms4470DeviceGroupInformation[Index].Sms4470MessageRxBuffer,SIANO_IIC_RX_BUFFER_SIZE);
		if(ResponseResult == 0xffff)
		{
		   break;
		}
		else
		if(ResponseResult == 0xf000)
		{
           i = CommonCommandWaitResponseLoop;
		   break;
		}
	}
	if(i == CommonCommandWaitResponseLoop)
	{
       SMS4470ErrorPrintf("(SMS4470_get_version) xxxxxxxxx Get response failed !\n");
	   OSPorting_Frontend_failflag(1,1);
	   return FALSE;
	}

	pVer = (SmsVersionRes_ST*)Sms4470DeviceGroupInformation[Index].Sms4470MessageRxBuffer;

	SMS4470InfoPrintf("**************************************************************\n");
	SMS4470InfoPrintf("(SMS4470_get_version) ChipModel : 0x%x\n",pVer->xVersion.ChipModel);
	SMS4470InfoPrintf("(SMS4470_get_version) FirmwareId : %d\n",pVer->xVersion.FirmwareId);
	SMS4470InfoPrintf("(SMS4470_get_version) SupportedProtocols : 0x%x\n",pVer->xVersion.SupportedProtocols);
	SMS4470InfoPrintf("(SMS4470_get_version) FwVer(Major) : %d\n",pVer->xVersion.FwVer.Major);
	SMS4470InfoPrintf("(SMS4470_get_version) FwVer(Minor) : %d\n",pVer->xVersion.FwVer.Minor);
	SMS4470InfoPrintf("(SMS4470_get_version) RomVer(Major) : %d\n",pVer->xVersion.RomVer.Major);
	SMS4470InfoPrintf("(SMS4470_get_version) RomVer(Minor) : %d\n",pVer->xVersion.RomVer.Minor);
	SMS4470InfoPrintf("(SMS4470_get_version) TextLabel : %s\n",pVer->xVersion.TextLabel);

	return TRUE;
}
//**********************************************************************************************************
//**********************************************************************************************************
//**********************************************************************************************************

#define MAX_CHUNK_SIZE        (8*1024) // (8*1024) // (10*1024) // (64*1024) // (128*1024)

BOOL SMS4470_dual_mode_download_dvbt2_firmware_data(ULONG FrontendGroupID,UINT8* pFwImage)
{
	UINT8* fw_data = (UINT8*)pFwImage + 12;
	UINT32* fw_hdr = (UINT32*)pFwImage;
	UINT32 actual_crc, dummy_crc,i;
	UINT32 chunk_size;
	UINT32 dummy_hdr[3];
	UINT32 fw_addr, fw_len, dnl_offset;
	UINT8  DownloadCommand[8] = {0x9a,0x02,0x96,0x0b,0x08,0x00,0x01,0x00 };

	
	fw_addr = fw_hdr[2];
#ifdef SMS_BIGENDIAN
	fw_addr = SMS_ENDIAN_SWAP32(fw_addr);
#endif
	fw_len = fw_hdr[1];
#ifdef SMS_BIGENDIAN
	fw_len = SMS_ENDIAN_SWAP32(fw_len);
#endif

	// The full CRC is for debug and printing, the function doesn't use this
	actual_crc=0;
	for (i = 0; i < fw_len+8 ; i++)
	{
		actual_crc ^= ((UINT8*)pFwImage)[4+i];
	}

	dnl_offset = fw_addr + fw_len;

	while (fw_len)
	{
  	    SMS4470I2CWrite(FrontendGroupID,DownloadCommand, 8);
	    SMS4470SysDelay(30);

		if(fw_len > MAX_CHUNK_SIZE)
		   chunk_size = MAX_CHUNK_SIZE;
		else
		   chunk_size = fw_len;

		dnl_offset -= chunk_size;
		fw_len -= chunk_size;
		dummy_hdr[1] = chunk_size;
#ifdef SMS_BIGENDIAN
	    dummy_hdr[1] = SMS_ENDIAN_SWAP32(dummy_hdr[1]);
#endif
		dummy_hdr[2] = dnl_offset;
#ifdef SMS_BIGENDIAN
	    dummy_hdr[2] = SMS_ENDIAN_SWAP32(dummy_hdr[2]);
#endif

		dummy_crc=0;
		for (i = 0; i < 8 ; i++)
		{
			dummy_crc ^= ((UINT8*)dummy_hdr)[4+i];
		}
		for (i = 0; i < chunk_size ; i++)
		{
			dummy_crc ^= ((UINT8*)(fw_data+fw_len))[i];
		}
		if (dnl_offset == fw_addr)
		{ // Only for the last chunk send the correct CRC
			dummy_hdr[0] = dummy_crc;
#ifdef SMS_BIGENDIAN
	        dummy_hdr[0] = SMS_ENDIAN_SWAP32(dummy_hdr[0]);
#endif
		}
		else
		{// for all but last chunk, make sure crc is wrong
			dummy_hdr[0] = dummy_crc^0x55;
#ifdef SMS_BIGENDIAN
	        dummy_hdr[0] = SMS_ENDIAN_SWAP32(dummy_hdr[0]);
#endif
		}
		//send header of current chunk
		SMS4470I2CWrite(FrontendGroupID,(UINT8*)dummy_hdr, 12);
		SMS4470SysDelay(20);
		//send the data of current chunk
		SMS4470I2CWrite_FW(FrontendGroupID,(UINT8*)(fw_data+fw_len),chunk_size);
		SMS4470SysDelay(30);
	}
	
	SMS4470SysDelay(400);

	return TRUE;
}
BOOL SMS4470_dual_mode_download_dvbt_firmware_data(ULONG FrontendGroupID,UINT8* pFwImage)
{
	ULONG Index;
	UINT32 fwLen, fwAddr, size, sizeToSend;
	UINT8* fwBuf;
	UINT32 i, *ptr, packetNumber = 0, burstSize = 0;
	UINT32 ResponseResult;
	SmsMsgData_ST* pStaticMsg;
	SmsDataDownload_ST *pSmsMsg = NULL;
	SmsMsgData3Args_ST* pMsg;
	UINT16 TempMsgLength;

    SMS4470FunctionNamePrintf("(SMS4470_dual_mode_download_dvbt_firmware_data)\n");

	if(FrontendGroupID == FRONTEND_TWO_DIVERSITY_GROUP_ID)
	   Index = 0;
	else
	   Index = 1;

	pStaticMsg = (SmsMsgData_ST*)Sms4470DeviceGroupInformation[Index].Sms4470MessageTxBuffer;

	fwAddr = 0xe00000;
	fwLen = ((UINT32*)pFwImage)[1];
#ifdef SMS_BIGENDIAN
	fwLen = SMS_ENDIAN_SWAP32(fwLen);
#endif

    SMS4470DebugPrintf("(SMS4470_dual_mode_download_dvbt_firmware_data) fwLen : 0x%x\n",fwLen);

	//For alignment reasons, and since checksum is using 32 bit steps
	//the buffer is a bit larger than the FW size, and end of buffer is padded with 0
	fwBuf = &pFwImage[12];

	//Start the actual reload process

	pStaticMsg->xMsgHeader.msgSrcId = SMS_HOST_LIB;
	pStaticMsg->xMsgHeader.msgDstId = HIF_TASK;
	pStaticMsg->xMsgHeader.msgFlags = MSG_HDR_FLAG_STATIC_MSG;
	pStaticMsg->xMsgHeader.msgType  = MSG_SW_RELOAD_START_REQ;
	pStaticMsg->xMsgHeader.msgLength = (UINT16)(sizeof(SmsMsgHdr_ST) + sizeof(UINT32));
	pStaticMsg->msgData[0] = SMSHOSTLIB_DNLD_ALL_SLAVES_INCHAIN_ASYNC_NO_PRE_TASKS_SHUTDOWN;
   
SMS4470DebugPrintf("(SMS4470_dual_mode_download_dvbt_firmware_data) Send command (MSG_SW_RELOAD_START_REQ) to SMS4470\n");

    TempMsgLength = pStaticMsg->xMsgHeader.msgLength;
#ifdef SMS_BIGENDIAN
    pStaticMsg->xMsgHeader.msgType    = SMS_ENDIAN_SWAP16(pStaticMsg->xMsgHeader.msgType);
    pStaticMsg->xMsgHeader.msgLength  = SMS_ENDIAN_SWAP16(pStaticMsg->xMsgHeader.msgLength);
    pStaticMsg->xMsgHeader.msgFlags   = SMS_ENDIAN_SWAP16(pStaticMsg->xMsgHeader.msgFlags);
    pStaticMsg->msgData[0]            = SMS_ENDIAN_SWAP32(pStaticMsg->msgData[0]);
#endif

//    SMS4470SysDelay(5);
    SMS4470I2CWrite(FrontendGroupID,(UINT8*)pStaticMsg, TempMsgLength);

    for(i=0; i<SpecialCommandWaitResponseLoop; i++)
    {
        SMS4470SysDelay(10);
		ResponseResult = SMS4470_read_response(FrontendGroupID,MSG_SW_RELOAD_START_RES, Sms4470DeviceGroupInformation[Index].Sms4470MessageRxBuffer, SIANO_IIC_RX_BUFFER_SIZE);
        if(ResponseResult == 0xffff)
		{
           break;
		}
		else
		if(ResponseResult == 0xf000)
		{
           i = SpecialCommandWaitResponseLoop;
		   break;
		}
    }
    
    if(i == SpecialCommandWaitResponseLoop)
	{
	   SMS4470ErrorPrintf("(SMS4470_dual_mode_download_dvbt_firmware_data) xxxxxxxxx Get response failed(do not care) !\n");	
	}
 
	//////////////////////////////////////////////////////////////////////////
	
	pSmsMsg = (SmsDataDownload_ST*)Sms4470DeviceGroupInformation[Index].Sms4470MessageTxBuffer;
	pSmsMsg->xMsgHeader.msgSrcId = SMS_HOST_LIB;
	pSmsMsg->xMsgHeader.msgDstId = HIF_TASK;
	pSmsMsg->xMsgHeader.msgFlags = MSG_HDR_FLAG_STATIC_MSG;
	pSmsMsg->xMsgHeader.msgType  = MSG_SMS_DATA_DOWNLOAD_REQ;

#ifdef SMS_BIGENDIAN
    pSmsMsg->xMsgHeader.msgType    = SMS_ENDIAN_SWAP16(pSmsMsg->xMsgHeader.msgType);
    pSmsMsg->xMsgHeader.msgFlags   = SMS_ENDIAN_SWAP16(pSmsMsg->xMsgHeader.msgFlags);
#endif

	size=0;
	while (size < fwLen)
	{
		if ( ( fwLen - size ) >= 240 )
			sizeToSend = 240; 
		else
			sizeToSend = fwLen - size;

		pSmsMsg->xMsgHeader.msgLength = (UINT16)(sizeof(SmsMsgData_ST) + sizeToSend);
		memcpy(pSmsMsg->Payload, &fwBuf[size], sizeToSend);
		pSmsMsg->MemAddr = fwAddr+size;              

        TempMsgLength = pSmsMsg->xMsgHeader.msgLength;
#ifdef SMS_BIGENDIAN
        pSmsMsg->xMsgHeader.msgLength  = SMS_ENDIAN_SWAP16(pSmsMsg->xMsgHeader.msgLength);
        pSmsMsg->MemAddr               = SMS_ENDIAN_SWAP32(pSmsMsg->MemAddr);
#endif

//        SMS4470SysDelay(5);
        SMS4470I2CWrite(FrontendGroupID,(UINT8*)pSmsMsg, TempMsgLength);

        for(i=0; i<SpecialCommandWaitResponseLoop; i++)
		{
            SMS4470SysDelay(10);
			ResponseResult = SMS4470_read_response(FrontendGroupID,MSG_SMS_DATA_DOWNLOAD_RES, Sms4470DeviceGroupInformation[Index].Sms4470MessageRxBuffer, SIANO_IIC_RX_BUFFER_SIZE);
            if(ResponseResult == 0xffff)
			{
              break;
			}
		    else
		    if(ResponseResult == 0xf000)
			{
               i = SpecialCommandWaitResponseLoop;
		       break;
			}
		}
    
        if(i == SpecialCommandWaitResponseLoop)
		{ 
	       SMS4470ErrorPrintf("(SMS4470_dual_mode_download_dvbt_firmware_data) xxxxxxxxx Get response failed !\n");	

           return FALSE;
		}

		packetNumber++;
		size+=sizeToSend;
	}

	pMsg = (SmsMsgData3Args_ST*)Sms4470DeviceGroupInformation[Index].Sms4470MessageTxBuffer;
	pMsg->xMsgHeader.msgSrcId = SMS_HOST_LIB;
	pMsg->xMsgHeader.msgDstId = HIF_TASK;
	pMsg->xMsgHeader.msgFlags = MSG_HDR_FLAG_STATIC_MSG;
	pMsg->xMsgHeader.msgType  = MSG_SMS_DATA_VALIDITY_REQ;
	pMsg->xMsgHeader.msgLength = (UINT16)(sizeof(SmsMsgHdr_ST) + 3 * sizeof(UINT32));
	pMsg->msgData[0] = 0xe00000;
	pMsg->msgData[1] = fwLen;
	pMsg->msgData[2] = 0;

    TempMsgLength = pMsg->xMsgHeader.msgLength;
#ifdef SMS_BIGENDIAN
    pMsg->xMsgHeader.msgType    = SMS_ENDIAN_SWAP16(pMsg->xMsgHeader.msgType);
    pMsg->xMsgHeader.msgLength  = SMS_ENDIAN_SWAP16(pMsg->xMsgHeader.msgLength);
    pMsg->xMsgHeader.msgFlags   = SMS_ENDIAN_SWAP16(pMsg->xMsgHeader.msgFlags);
    pMsg->msgData[0]            = SMS_ENDIAN_SWAP32(pMsg->msgData[0]);
    pMsg->msgData[1]            = SMS_ENDIAN_SWAP32(pMsg->msgData[1]);
    pMsg->msgData[2]            = SMS_ENDIAN_SWAP32(pMsg->msgData[2]);
#endif

//    SMS4470SysDelay(5);
    SMS4470I2CWrite(FrontendGroupID,(UINT8*)pMsg, TempMsgLength);

    for(i=0; i<SpecialCommandWaitResponseLoop; i++)
    {
        SMS4470SysDelay(10);
		ResponseResult = SMS4470_read_response(FrontendGroupID,MSG_SMS_DATA_VALIDITY_RES, Sms4470DeviceGroupInformation[Index].Sms4470MessageRxBuffer, SIANO_IIC_RX_BUFFER_SIZE);
        if(ResponseResult == 0xffff)
		{
           break;
		}
		else
		if(ResponseResult == 0xf000)
		{
           i = SpecialCommandWaitResponseLoop;
		   break;
		}
    }
    
    if(i == SpecialCommandWaitResponseLoop)
	{
	   SMS4470ErrorPrintf("(SMS4470_dual_mode_download_dvbt_firmware_data) xxxxxxxxx Get response failed !\n");	

       return FALSE;
	}

	return TRUE;
}
BOOL SMS4470_single_mode_download_firmware_data(ULONG FrontendGroupID,UINT8* pFwImage)
{
	UINT8* fw_data = (UINT8*)pFwImage + 12;
	UINT32* fw_hdr = (UINT32*)pFwImage;
	UINT32 actual_crc, dummy_crc,i;
	UINT32 chunk_size;
	UINT32 dummy_hdr[3];
	UINT32 fw_addr, fw_len, dnl_offset;
	UINT8  DownloadCommand[8] = {0x9a,0x02,0x96,0x0b,0x08,0x00,0x01,0x00 };
	UINT   fw_len_orig;

	
	fw_addr = fw_hdr[2];
#ifdef SMS_BIGENDIAN
	fw_addr = SMS_ENDIAN_SWAP32(fw_addr);
#endif
	fw_len = fw_hdr[1];
#ifdef SMS_BIGENDIAN
	fw_len = SMS_ENDIAN_SWAP32(fw_len);
#endif

	// The full CRC is for debug and printing, the function doesn't use this
	actual_crc=0;
	for (i = 0; i < fw_len+8 ; i++)
	{
		actual_crc ^= ((UINT8*)pFwImage)[4+i];
	}

	dnl_offset = fw_addr + fw_len;

	while (fw_len)
	{
  	    SMS4470I2CWrite(FrontendGroupID,DownloadCommand, 8);
	    SMS4470SysDelay(30);

		if(fw_len > MAX_CHUNK_SIZE)
		   chunk_size = MAX_CHUNK_SIZE;
		else
		   chunk_size = fw_len;

		dnl_offset -= chunk_size;
		fw_len -= chunk_size;
		dummy_hdr[1] = chunk_size;
		
#ifdef SMS_BIGENDIAN
	    dummy_hdr[1] = SMS_ENDIAN_SWAP32(dummy_hdr[1]);
#endif
		dummy_hdr[2] = dnl_offset;
#ifdef SMS_BIGENDIAN
	    dummy_hdr[2] = SMS_ENDIAN_SWAP32(dummy_hdr[2]);
#endif

        //Yuneec special code inseration BEGIN --------------
		fw_len_orig = fw_len;
		fw_len = 0;
		PlatformPorting_Frontend_Fw_Download_Usb_Read(&fw_data, chunk_size);
		//Yuneec special code insertion END -----------------
		
		dummy_crc=0;
		for (i = 0; i < 8 ; i++)
		{
			dummy_crc ^= ((UINT8*)dummy_hdr)[4+i];
		}
		for (i = 0; i < chunk_size ; i++)
		{
			dummy_crc ^= ((UINT8*)(fw_data+fw_len))[i];
		}
		if (dnl_offset == fw_addr)
		{ // Only for the last chunk send the correct CRC
			dummy_hdr[0] = dummy_crc;
#ifdef SMS_BIGENDIAN
	        dummy_hdr[0] = SMS_ENDIAN_SWAP32(dummy_hdr[0]);
#endif
		}
		else
		{// for all but last chunk, make sure crc is wrong
			dummy_hdr[0] = dummy_crc^0x55;
#ifdef SMS_BIGENDIAN
	        dummy_hdr[0] = SMS_ENDIAN_SWAP32(dummy_hdr[0]);
#endif
		}
		//send header of current chunk
		SMS4470I2CWrite(FrontendGroupID,(UINT8*)dummy_hdr, 12);
		SMS4470SysDelay(20);
		//send the data of current chunk
		SMS4470I2CWrite_FW(FrontendGroupID,(UINT8*)(fw_data+fw_len),chunk_size);
		SMS4470SysDelay(30);
	//Yuneec special code inseration BEGIN --------------
	fw_len = fw_len_orig;
	//Yuneec special code insertion END -----------------
	}

	        
	SMS4470SysDelay(400);

	return TRUE;
}
//**********************************************************************************************************
//**********************************************************************************************************
//**********************************************************************************************************
BOOL SMS4470_wait_device_redetected(ULONG FrontendGroupID)
{
	ULONG              Index;
    UINT32             i;
	UINT32             ResponseResult;

	if(FrontendGroupID == FRONTEND_TWO_DIVERSITY_GROUP_ID)
	   Index = 0;
	else
	   Index = 1;
    
    for(i=0 ; i<CriticalCommandWaitResponseLoop ; i++)
	{
        SMS4470SysDelay(10);
		ResponseResult = SMS4470_read_response(FrontendGroupID,MSG_SMS_DEVICE_DETECTED_IND, Sms4470DeviceGroupInformation[Index].Sms4470MessageRxBuffer, SIANO_IIC_RX_BUFFER_SIZE);
        if(ResponseResult == 0xffff) 
		{
           break;
		}
		else
		if(ResponseResult == 0xf000)
		{
           i = CriticalCommandWaitResponseLoop;
		   break;
		}
	}
    
    if(i == CriticalCommandWaitResponseLoop)
	{
	   SMS4470ErrorPrintf("(SMS4470_wait_device_redetected) xxxxxxxxx Get response failed !\n");

       return FALSE;
	}

	return TRUE;
}
BOOL SMS4470_set_polling_mode(ULONG FrontendGroupID)
{
	ULONG                   Index;
	sms_intr_line_t         gpioMsg;
	UINT32                  i;
	UINT32                  ResponseResult;
	UINT16                  TempMsgLength;
    
    SMS4470FunctionNamePrintf("(SMS4470_set_polling_mode)\n");

	if(FrontendGroupID == FRONTEND_TWO_DIVERSITY_GROUP_ID)
	   Index = 0;
	else
	   Index = 1;

	gpioMsg.xMsgHeader.msgType   = MSG_SMS_SPI_INT_LINE_SET_REQ;
	gpioMsg.xMsgHeader.msgSrcId  = SMS_HOST_LIB; // 0;
	gpioMsg.xMsgHeader.msgDstId  = HIF_TASK;
	gpioMsg.xMsgHeader.msgLength = sizeof(gpioMsg);
	gpioMsg.xMsgHeader.msgFlags  = MSG_HDR_FLAG_STATIC_MSG;
	gpioMsg.Controler			 = I2C_SEC_CTR;                //1;  // 0;             // ?? // it seems that it must be 0 , otherwise the signal reception will get worse !
	gpioMsg.GpioNum			     = SMS_GPIO_NONE;	           // 32
	gpioMsg.PulseWidth 		     = SPI_PULSE_WIDTH;            //20; // 0;             // ?? // it seems that it must be 0 , otherwise the signal reception will get worse !
		
   
SMS4470DebugPrintf("(SMS4470_set_polling_mode) Send command (MSG_SMS_SPI_INT_LINE_SET_REQ) to SMS4470\n");

    TempMsgLength = gpioMsg.xMsgHeader.msgLength;
#ifdef SMS_BIGENDIAN
    gpioMsg.xMsgHeader.msgType    = SMS_ENDIAN_SWAP16(gpioMsg.xMsgHeader.msgType);
    gpioMsg.xMsgHeader.msgLength  = SMS_ENDIAN_SWAP16(gpioMsg.xMsgHeader.msgLength);
    gpioMsg.xMsgHeader.msgFlags   = SMS_ENDIAN_SWAP16(gpioMsg.xMsgHeader.msgFlags);
    gpioMsg.Controler             = SMS_ENDIAN_SWAP32(gpioMsg.Controler);
    gpioMsg.GpioNum               = SMS_ENDIAN_SWAP32(gpioMsg.GpioNum);
    gpioMsg.PulseWidth            = SMS_ENDIAN_SWAP32(gpioMsg.PulseWidth);
#endif

//    SMS4470SysDelay(5);
	SMS4470I2CWrite(FrontendGroupID,(UINT8*)&gpioMsg, TempMsgLength);
	
   
SMS4470DebugPrintf("(SMS4470_set_polling_mode) Get respence from SMS4470\n");

	for(i=0; i<CommonCommandWaitResponseLoop; i++)
	{
        SMS4470SysDelay(10);
		ResponseResult = SMS4470_read_response(FrontendGroupID,MSG_SMS_SPI_INT_LINE_SET_RES, Sms4470DeviceGroupInformation[Index].Sms4470MessageRxBuffer, SIANO_IIC_RX_BUFFER_SIZE);
		if(ResponseResult == 0xffff)
		{
		   break;
		}
		else
		if(ResponseResult == 0xf000)
		{
           i = CommonCommandWaitResponseLoop;
		   OSPorting_Frontend_failflag(4, 4);
		   break;
		}
	}

	if(i == CommonCommandWaitResponseLoop)
	{
	   SMS4470ErrorPrintf("(SMS4470_set_polling_mode) xxxxxxxxx Get response failed !\n");
	   OSPorting_Frontend_failflag(8, 8);
	}

	return TRUE;
}
//**********************************************************************************************************
//**********************************************************************************************************
//**********************************************************************************************************
BOOL SMS4470_enable_ts_interface(ULONG FrontendGroupID)
{
	ULONG                  Index;
	SmsTsEnable_ST         tsEnableMsg;
	UINT32                 i;
	UINT32                 ResponseResult;
	UINT16                 TempMsgLength;

    SMS4470FunctionNamePrintf("(SMS4470_enable_ts_interface)\n");

	if(FrontendGroupID == FRONTEND_TWO_DIVERSITY_GROUP_ID)
	   Index = 0;
	else
	   Index = 1;

	tsEnableMsg.xMsgHeader.msgType      = MSG_SMS_ENBALE_TS_INTERFACE_REQ;
	tsEnableMsg.xMsgHeader.msgSrcId     = SMS_HOST_LIB; // 0;
	tsEnableMsg.xMsgHeader.msgDstId     = HIF_TASK;
	tsEnableMsg.xMsgHeader.msgLength    = sizeof(tsEnableMsg);
	tsEnableMsg.xMsgHeader.msgFlags     = MSG_HDR_FLAG_STATIC_MSG;
#ifdef SMS4470_SERIAL_TRANSPORT_STREAM_OUTPUT
	tsEnableMsg.TsClock                 = 12000000;                // 12000000; // 24000000; // 36000000; // 48000000; // 96000000; // 192000000;
	tsEnableMsg.eTsiMode                = TSI_SERIAL_SECONDARY;    // TSI_SERIAL_MAIN; // TSI_SERIAL_SECONDARY;
#else
	tsEnableMsg.TsClock				    = 12000000;				   // 0 - TS Clock Speed in Hz
	tsEnableMsg.eTsiMode				= TSI_PARALLEL_MAIN;	   // 1 - TS Mode of operation Serial (on SDIO or HIF Pins), or Parallel
#endif
	tsEnableMsg.eTsiSignals             = TSI_SIGNALS_ACTIVE_HIGH; // TSI_SIGNALS_ACTIVE_LOW; // TSI_SIGNALS_ACTIVE_HIGH;            
	tsEnableMsg.nTsiPcktDelay           = 4;                       // 4; // 0; // 4;          
	tsEnableMsg.eTsClockPolarity        = TSI_SIG_OUT_RISE_EDGE;   // TSI_SIG_OUT_FALL_EDGE; // TSI_SIG_OUT_RISE_EDGE;       
	tsEnableMsg.TsBitOrder              = TSI_BIT0_IS_MSB;             
	tsEnableMsg.EnableControlOverTs     = 0;                       // 0; // 1;    
	tsEnableMsg.TsiEncapsulationFormat  = TSI_TRANSPARENT;         // TSI_ENCAPSULATED; // TSI_TRANSPARENT; 
	tsEnableMsg.TsiPaddingPackets       = 22;                      // 0; //21;//22;
	tsEnableMsg.eTsiElectrical          = TSI_ELEC_NORMAL;         // TSI_ELEC_LOW; // TSI_ELEC_NORMAL; // TSI_ELEC_HIGH; 
	tsEnableMsg.IoVoltage               = IOC_VOLTAGE_3_3;         // IOC_VOLTAGE_0; // IOC_VOLTAGE_0; // IOC_VOLTAGE_1_8; // IOC_VOLTAGE_3_3;
	tsEnableMsg.eTsiErrActive           = TSI_ERR_ACTIVE;
	tsEnableMsg.eTsiClockKeepGo         = TSI_CLK_KEEP_GO_NO_PKT;
	
SMS4470DebugPrintf("(SMS4470_enable_ts_interface) Send command (MSG_SMS_ENBALE_TS_INTERFACE_REQ) to SMS4470\n");


    TempMsgLength = tsEnableMsg.xMsgHeader.msgLength;
#ifdef SMS_BIGENDIAN
    tsEnableMsg.xMsgHeader.msgType     = SMS_ENDIAN_SWAP16(tsEnableMsg.xMsgHeader.msgType);
    tsEnableMsg.xMsgHeader.msgLength   = SMS_ENDIAN_SWAP16(tsEnableMsg.xMsgHeader.msgLength);
    tsEnableMsg.xMsgHeader.msgFlags    = SMS_ENDIAN_SWAP16(tsEnableMsg.xMsgHeader.msgFlags);
    tsEnableMsg.TsClock                = SMS_ENDIAN_SWAP32(tsEnableMsg.TsClock);
    tsEnableMsg.eTsiMode               = SMS_ENDIAN_SWAP32(tsEnableMsg.eTsiMode);
    tsEnableMsg.eTsiSignals            = SMS_ENDIAN_SWAP32(tsEnableMsg.eTsiSignals);
    tsEnableMsg.nTsiPcktDelay          = SMS_ENDIAN_SWAP32(tsEnableMsg.nTsiPcktDelay);
    tsEnableMsg.eTsClockPolarity       = SMS_ENDIAN_SWAP32(tsEnableMsg.eTsClockPolarity);
    tsEnableMsg.TsBitOrder             = SMS_ENDIAN_SWAP32(tsEnableMsg.TsBitOrder);
    tsEnableMsg.EnableControlOverTs    = SMS_ENDIAN_SWAP32(tsEnableMsg.EnableControlOverTs);
    tsEnableMsg.TsiEncapsulationFormat = SMS_ENDIAN_SWAP32(tsEnableMsg.TsiEncapsulationFormat);
    tsEnableMsg.TsiPaddingPackets      = SMS_ENDIAN_SWAP32(tsEnableMsg.TsiPaddingPackets);
    tsEnableMsg.eTsiElectrical         = SMS_ENDIAN_SWAP32(tsEnableMsg.eTsiElectrical);
    tsEnableMsg.IoVoltage              = SMS_ENDIAN_SWAP32(tsEnableMsg.IoVoltage);
    tsEnableMsg.eTsiErrActive          = SMS_ENDIAN_SWAP32(tsEnableMsg.eTsiErrActive);
    tsEnableMsg.eTsiClockKeepGo        = SMS_ENDIAN_SWAP32(tsEnableMsg.eTsiClockKeepGo);
#endif

//    SMS4470SysDelay(5);
	SMS4470I2CWrite(FrontendGroupID,(UINT8*)&tsEnableMsg,TempMsgLength);
	
   
SMS4470DebugPrintf("(SMS4470_enable_ts_interface) Get respence from SMS4470\n");

	for(i=0; i<CommonCommandWaitResponseLoop; i++)
	{
        SMS4470SysDelay(10);
		ResponseResult = SMS4470_read_response(FrontendGroupID,MSG_SMS_ENBALE_TS_INTERFACE_RES, Sms4470DeviceGroupInformation[Index].Sms4470MessageRxBuffer, SIANO_IIC_RX_BUFFER_SIZE);
		if(ResponseResult == 0xffff)
		{
		   break;
		}
		else
		if(ResponseResult == 0xf000)
		{
           i = CommonCommandWaitResponseLoop;
		   break;
		}
	}

	if(i == CommonCommandWaitResponseLoop)
	{
	   SMS4470ErrorPrintf("(SMS4470_enable_ts_interface) xxxxxxxxx Get response failed !\n");
	}

	return TRUE;
}
//**********************************************************************************************************
//**********************************************************************************************************
//**********************************************************************************************************
BOOL SMS4470_send_device_init(ULONG FrontendGroupID,DEMODULATOR_TUNING_FEATURE DemodulatorTuningFeature)
{
	ULONG                   Index;
    SmsMsgData_ST           SmsMsg = {{0}};
	UINT32                  i;
	UINT32                  ResponseResult;
	UINT16                  TempMsgLength;
    
    SMS4470FunctionNamePrintf("(SMS4470_send_device_init)\n");

	if(FrontendGroupID == FRONTEND_TWO_DIVERSITY_GROUP_ID)
	   Index = 0;
	else
	   Index = 1;

	SmsMsg.xMsgHeader.msgType   = MSG_SMS_INIT_DEVICE_REQ;
	SmsMsg.xMsgHeader.msgSrcId  = SMS_HOST_LIB; // 0;
	SmsMsg.xMsgHeader.msgDstId  = HIF_TASK;
	SmsMsg.xMsgHeader.msgLength = (UINT16)sizeof(SmsMsg);
	SmsMsg.xMsgHeader.msgFlags  = MSG_HDR_FLAG_STATIC_MSG;
    if(DemodulatorTuningFeature == DVBT_DEMODULATOR_TUNING)
	   SmsMsg.msgData[0]           = SMSHOSTLIB_DEVMD_DVBT;
    else
    if(DemodulatorTuningFeature == DVBH_DEMODULATOR_TUNING)
	   SmsMsg.msgData[0]           = SMSHOSTLIB_DEVMD_DVBH;  
    else
    if(DemodulatorTuningFeature == DVBT2_DEMODULATOR_TUNING)
	   SmsMsg.msgData[0]           = SMSHOSTLIB_DEVMD_DVBT2;  
    else
    if(DemodulatorTuningFeature == DVBT2_LITE_DEMODULATOR_TUNING)
	   SmsMsg.msgData[0]           = SMSHOSTLIB_DEVMD_DVBT2;  
    else
    if(DemodulatorTuningFeature == ISDBT_13SEGMENT_DEMODULATOR_TUNING)
	   SmsMsg.msgData[0]           = SMSHOSTLIB_DEVMD_ISDBT;  
	else
	   SmsMsg.msgData[0]           = SMSHOSTLIB_DEVMD_ISDBT;  
   
SMS4470DebugPrintf("(SMS4470_send_device_init) Send command (MSG_SMS_INIT_DEVICE_REQ) to SMS4470\n");


    TempMsgLength = SmsMsg.xMsgHeader.msgLength;
#ifdef SMS_BIGENDIAN
    SmsMsg.xMsgHeader.msgType     = SMS_ENDIAN_SWAP16(SmsMsg.xMsgHeader.msgType);
    SmsMsg.xMsgHeader.msgLength   = SMS_ENDIAN_SWAP16(SmsMsg.xMsgHeader.msgLength);
    SmsMsg.xMsgHeader.msgFlags    = SMS_ENDIAN_SWAP16(SmsMsg.xMsgHeader.msgFlags);
    SmsMsg.msgData[0]             = SMS_ENDIAN_SWAP32(SmsMsg.msgData[0]);
#endif

//    SMS4470SysDelay(5);
	SMS4470I2CWrite(FrontendGroupID,(UINT8*)&SmsMsg,TempMsgLength);
	
	
SMS4470DebugPrintf("(SMS4470_send_device_init) Get respence from SMS4470\n");

	for(i=0; i<SpecialCommandWaitResponseLoop; i++)
	{
        SMS4470SysDelay(10);
		ResponseResult = SMS4470_read_response(FrontendGroupID,MSG_SMS_INIT_DEVICE_RES, Sms4470DeviceGroupInformation[Index].Sms4470MessageRxBuffer, SIANO_IIC_RX_BUFFER_SIZE);
		if(ResponseResult == 0xffff)
		{
           break;
		}
		else
		if(ResponseResult == 0xf000)
		{
           i = SpecialCommandWaitResponseLoop;
		   break;
		}
	}
	
	if(i == SpecialCommandWaitResponseLoop)
	{
	   SMS4470ErrorPrintf("(SMS4470_send_device_init) xxxxxxxxx Get response failed !\n");
	}

	return TRUE;
}
//**********************************************************************************************************
//**********************************************************************************************************
//**********************************************************************************************************
UINT32 SMS4470_read_response(ULONG FrontendGroupID,UINT32 response, UINT8* iicRxBuf,UINT16 MaxiicRxBufSize)
{
	ULONG               Index;
    UINT32              i;
    SmsMsgData_ST*      pMsg;
    UINT32              bytesToRead = 0;
	BOOL                Result;

//    SMS4470DebugPrintf("(SMS4470_read_response)\n");    
    
	if(FrontendGroupID == FRONTEND_TWO_DIVERSITY_GROUP_ID)
	   Index = 0;
	else
	   Index = 1;

	i = 0;

    if(iicRxBuf == NULL)
	{        
       return 0;
	}

    memset(iicRxBuf, 0xaa, MaxiicRxBufSize);

    // Note: need to modify I2C read fun 
    // read message header
	Result = SMS4470I2CRead(FrontendGroupID,sizeof(ShortStatMsg_ST),iicRxBuf);
	if(Result != TRUE)
	{        
	   return 0xf000;      
	}

	pMsg = (SmsMsgData_ST*)iicRxBuf;

#ifdef SMS_BIGENDIAN
    pMsg->xMsgHeader.msgType = SMS_ENDIAN_SWAP16(pMsg->xMsgHeader.msgType);
    pMsg->xMsgHeader.msgLength = SMS_ENDIAN_SWAP16(pMsg->xMsgHeader.msgLength);
    pMsg->xMsgHeader.msgFlags = SMS_ENDIAN_SWAP16(pMsg->xMsgHeader.msgFlags);
#endif


    //* check if there is a payload to the message
	if((pMsg->xMsgHeader.msgLength > 8) && (pMsg->xMsgHeader.msgType != MSG_SMS_I2C_SHORT_STAT_IND))
	{
		bytesToRead = pMsg->xMsgHeader.msgLength > MaxiicRxBufSize ? MaxiicRxBufSize : pMsg->xMsgHeader.msgLength;

		//* since host is much faster then device, need to wait for device to prepare next buffer for read
		SMS4470SysDelay(10);  //YH: ctrl polling blocking delay

		//* lock Bus
		Result = SMS4470I2CRead(FrontendGroupID,bytesToRead - 8,(UINT8*)&(iicRxBuf[8]));
		if(Result != TRUE)
		{    
			return 0xf000;      
		}
		//* unlock bus
	}
    
	if(pMsg->xMsgHeader.msgType == response)
	{
#ifdef SMS_BIGENDIAN
		if(pMsg->xMsgHeader.msgType == MSG_SMS_GET_VERSION_EX_RES)
		{
		   SMSHOSTLIB_VERSION_ST* pVersion; 

		   pVersion = (SMSHOSTLIB_VERSION_ST*)&iicRxBuf[8];

           pVersion->ChipModel    = SMS_ENDIAN_SWAP16(pVersion->ChipModel);
           pVersion->PkgVer       = SMS_ENDIAN_SWAP32(pVersion->PkgVer);
           pVersion->Reserved[0]  = SMS_ENDIAN_SWAP32(pVersion->Reserved[0]);
           pVersion->Reserved[1]  = SMS_ENDIAN_SWAP32(pVersion->Reserved[1]);
           pVersion->Reserved[2]  = SMS_ENDIAN_SWAP32(pVersion->Reserved[2]);
           pVersion->Reserved[3]  = SMS_ENDIAN_SWAP32(pVersion->Reserved[3]);
           pVersion->Reserved[4]  = SMS_ENDIAN_SWAP32(pVersion->Reserved[4]);
           pVersion->Reserved[5]  = SMS_ENDIAN_SWAP32(pVersion->Reserved[5]);
           pVersion->Reserved[6]  = SMS_ENDIAN_SWAP32(pVersion->Reserved[6]);
           pVersion->Reserved[7]  = SMS_ENDIAN_SWAP32(pVersion->Reserved[7]);
           pVersion->Reserved[8]  = SMS_ENDIAN_SWAP32(pVersion->Reserved[8]);
		}
		else
		if(pMsg->xMsgHeader.msgType == MSG_SMS_GET_STATISTICS_EX_RES)
		{
	       if((Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == DVBT_DVBT2_DUAL_WORKING_MODE && Sms4470DeviceGroupInformation[Index].SMS4470CurrentDemodulationModeForDvbtDvbt2DualWorkingMode == SMS4470_DVBT2_DEMODULATION_MODE) ||
	          Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == DVBT2_WORKING_MODE)
		   {
			  UINT32* pData;
			  SMSHOSTLIB_STATISTICS_DVBT2_ST* pStat;
			  ULONG Loop;

			  pData = (UINT32*)&iicRxBuf[8];
              if(pData[0] == 0)
			  {          
                 pStat = (SMSHOSTLIB_STATISTICS_DVBT2_ST*)&pData[1];

		         pStat->ReceptionData.IsModemLocked = SMS_ENDIAN_SWAP32(pStat->ReceptionData.IsModemLocked);
		         pStat->ReceptionData.txStatistics.Frequency = SMS_ENDIAN_SWAP32(pStat->ReceptionData.txStatistics.Frequency);
		         pStat->ReceptionData.txStatistics.Bandwidth = SMS_ENDIAN_SWAP32(pStat->ReceptionData.txStatistics.Bandwidth);
		         pStat->ReceptionData.txStatistics.res[0] = SMS_ENDIAN_SWAP32(pStat->ReceptionData.txStatistics.res[0]);
		         pStat->ReceptionData.txStatistics.res[1] = SMS_ENDIAN_SWAP32(pStat->ReceptionData.txStatistics.res[1]);
		         pStat->ReceptionData.txStatistics.res[2] = SMS_ENDIAN_SWAP32(pStat->ReceptionData.txStatistics.res[2]);
		         pStat->ReceptionData.carrierOffset = SMS_ENDIAN_SWAP32(pStat->ReceptionData.carrierOffset);
		         pStat->ReceptionData.inbandPower = SMS_ENDIAN_SWAP32(pStat->ReceptionData.inbandPower); 
		         pStat->ReceptionData.extLna = SMS_ENDIAN_SWAP32(pStat->ReceptionData.extLna);      
		         pStat->ReceptionData.totalFrames = SMS_ENDIAN_SWAP32(pStat->ReceptionData.totalFrames);          
		         pStat->ReceptionData.SNR = SMS_ENDIAN_SWAP32(pStat->ReceptionData.SNR);     
		         pStat->ReceptionData.RSSI = SMS_ENDIAN_SWAP32(pStat->ReceptionData.RSSI);         
		         pStat->ReceptionData.FER = SMS_ENDIAN_SWAP32(pStat->ReceptionData.FER);	
		         pStat->ReceptionData.CellId = SMS_ENDIAN_SWAP32(pStat->ReceptionData.CellId);	
		         pStat->ReceptionData.netId = SMS_ENDIAN_SWAP32(pStat->ReceptionData.netId);	
		         pStat->ReceptionData.receptionQuality = SMS_ENDIAN_SWAP32(pStat->ReceptionData.receptionQuality);        
		         pStat->ReceptionData.bwt_ext = SMS_ENDIAN_SWAP32(pStat->ReceptionData.bwt_ext);
		         pStat->ReceptionData.fftMode = SMS_ENDIAN_SWAP32(pStat->ReceptionData.fftMode);       
		         pStat->ReceptionData.guardInterval = SMS_ENDIAN_SWAP32(pStat->ReceptionData.guardInterval);       
		         pStat->ReceptionData.pilotPattern = SMS_ENDIAN_SWAP32(pStat->ReceptionData.pilotPattern);       
		         pStat->ReceptionData.bitRate = SMS_ENDIAN_SWAP32(pStat->ReceptionData.bitRate);   
		         pStat->ReceptionData.extended = SMS_ENDIAN_SWAP32(pStat->ReceptionData.extended);
		         pStat->ReceptionData.toneReservation = SMS_ENDIAN_SWAP32(pStat->ReceptionData.toneReservation);
		         pStat->ReceptionData.l1PostSize = SMS_ENDIAN_SWAP32(pStat->ReceptionData.l1PostSize);
		         pStat->ReceptionData.numOfAuxs = SMS_ENDIAN_SWAP32(pStat->ReceptionData.numOfAuxs);
		         pStat->ReceptionData.numOfPlps = SMS_ENDIAN_SWAP32(pStat->ReceptionData.numOfPlps);
		         pStat->ReceptionData.liteMode = SMS_ENDIAN_SWAP32(pStat->ReceptionData.liteMode);
		         pStat->ReceptionData.MRC_SNR = SMS_ENDIAN_SWAP32(pStat->ReceptionData.MRC_SNR);
		         pStat->ReceptionData.SNRFullRes = SMS_ENDIAN_SWAP32(pStat->ReceptionData.SNRFullRes);	
		         pStat->ReceptionData.MRC_InBandPwr = SMS_ENDIAN_SWAP32(pStat->ReceptionData.MRC_InBandPwr);
		         pStat->ReceptionData.MRC_Rssi = SMS_ENDIAN_SWAP32(pStat->ReceptionData.MRC_Rssi);
		         pStat->ReceptionData.numdatasymbols = SMS_ENDIAN_SWAP16(pStat->ReceptionData.numdatasymbols);
		         pStat->ReceptionData.res[0] = SMS_ENDIAN_SWAP32(pStat->ReceptionData.res[0]);
		         pStat->ReceptionData.res[1] = SMS_ENDIAN_SWAP32(pStat->ReceptionData.res[1]);

		         pStat->TransmissionData.Frequency = SMS_ENDIAN_SWAP32(pStat->TransmissionData.Frequency);
		         pStat->TransmissionData.Bandwidth = SMS_ENDIAN_SWAP32(pStat->TransmissionData.Bandwidth);
		         pStat->TransmissionData.res[0] = SMS_ENDIAN_SWAP32(pStat->TransmissionData.res[0]);
		         pStat->TransmissionData.res[1] = SMS_ENDIAN_SWAP32(pStat->TransmissionData.res[1]);
		         pStat->TransmissionData.res[2] = SMS_ENDIAN_SWAP32(pStat->TransmissionData.res[2]);

				 pStat->generalInfo.smoothing = SMS_ENDIAN_SWAP32(pStat->generalInfo.smoothing);
		         pStat->generalInfo.res[0] = SMS_ENDIAN_SWAP32(pStat->generalInfo.res[0]);
		         pStat->generalInfo.res[1] = SMS_ENDIAN_SWAP32(pStat->generalInfo.res[1]);
		         pStat->generalInfo.res[2] = SMS_ENDIAN_SWAP32(pStat->generalInfo.res[2]);

				 for(Loop=0;Loop<DVBT2_MAX_PLPS_LITE;Loop++)
				 {
                     pStat->PlpData[Loop].plpStatistics.plpId = SMS_ENDIAN_SWAP32(pStat->PlpData[Loop].plpStatistics.plpId);	
                     pStat->PlpData[Loop].plpStatistics.plpType = SMS_ENDIAN_SWAP32(pStat->PlpData[Loop].plpStatistics.plpType);	
                     pStat->PlpData[Loop].plpStatistics.plpPayloadType = SMS_ENDIAN_SWAP32(pStat->PlpData[Loop].plpStatistics.plpPayloadType);	
                     pStat->PlpData[Loop].plpStatistics.ffFlag = SMS_ENDIAN_SWAP32(pStat->PlpData[Loop].plpStatistics.ffFlag);		
                     pStat->PlpData[Loop].plpStatistics.firstRfIdx = SMS_ENDIAN_SWAP32(pStat->PlpData[Loop].plpStatistics.firstRfIdx);	
                     pStat->PlpData[Loop].plpStatistics.firstFrameIdx = SMS_ENDIAN_SWAP32(pStat->PlpData[Loop].plpStatistics.firstFrameIdx);  
                     pStat->PlpData[Loop].plpStatistics.plpGroupId = SMS_ENDIAN_SWAP32(pStat->PlpData[Loop].plpStatistics.plpGroupId);		
                     pStat->PlpData[Loop].plpStatistics.plpCod = SMS_ENDIAN_SWAP32(pStat->PlpData[Loop].plpStatistics.plpCod);	
                     pStat->PlpData[Loop].plpStatistics.plpMod = SMS_ENDIAN_SWAP32(pStat->PlpData[Loop].plpStatistics.plpMod);
                     pStat->PlpData[Loop].plpStatistics.plpRotation = SMS_ENDIAN_SWAP32(pStat->PlpData[Loop].plpStatistics.plpRotation);		
                     pStat->PlpData[Loop].plpStatistics.plpFecType = SMS_ENDIAN_SWAP32(pStat->PlpData[Loop].plpStatistics.plpFecType);	
                     pStat->PlpData[Loop].plpStatistics.plpNumBlocksMax = SMS_ENDIAN_SWAP32(pStat->PlpData[Loop].plpStatistics.plpNumBlocksMax);	
                     pStat->PlpData[Loop].plpStatistics.frameInterval = SMS_ENDIAN_SWAP32(pStat->PlpData[Loop].plpStatistics.frameInterval);		
                     pStat->PlpData[Loop].plpStatistics.timeIlLength = SMS_ENDIAN_SWAP32(pStat->PlpData[Loop].plpStatistics.timeIlLength);	
                     pStat->PlpData[Loop].plpStatistics.timeIlType = SMS_ENDIAN_SWAP32(pStat->PlpData[Loop].plpStatistics.timeIlType);	
                     pStat->PlpData[Loop].plpStatistics.inbandA_Flag = SMS_ENDIAN_SWAP32(pStat->PlpData[Loop].plpStatistics.inbandA_Flag);		
                     pStat->PlpData[Loop].plpStatistics.inbandB_Flag = SMS_ENDIAN_SWAP32(pStat->PlpData[Loop].plpStatistics.inbandB_Flag);	
                     pStat->PlpData[Loop].plpStatistics.plpMode = SMS_ENDIAN_SWAP32(pStat->PlpData[Loop].plpStatistics.plpMode);	
                     pStat->PlpData[Loop].plpStatistics.staticFlag = SMS_ENDIAN_SWAP32(pStat->PlpData[Loop].plpStatistics.staticFlag);	
                     pStat->PlpData[Loop].plpStatistics.staticPaddingFlag = SMS_ENDIAN_SWAP32(pStat->PlpData[Loop].plpStatistics.staticPaddingFlag);	
                     pStat->PlpData[Loop].plpStatistics.res[0] = SMS_ENDIAN_SWAP32(pStat->PlpData[Loop].plpStatistics.res[0]);
                     pStat->PlpData[Loop].plpStatistics.res[1] = SMS_ENDIAN_SWAP32(pStat->PlpData[Loop].plpStatistics.res[1]);
                     pStat->PlpData[Loop].plpStatistics.res[2] = SMS_ENDIAN_SWAP32(pStat->PlpData[Loop].plpStatistics.res[2]);
				 }

				 for(Loop=0;Loop<DVBT2_ACTIVE_PLPS_LITE;Loop++)
				 {
                     pStat->activePlps[Loop].plpId = SMS_ENDIAN_SWAP32(pStat->activePlps[Loop].plpId);
                     pStat->activePlps[Loop].plpType = SMS_ENDIAN_SWAP32(pStat->activePlps[Loop].plpType);
                     pStat->activePlps[Loop].plpEfficiencyMode = SMS_ENDIAN_SWAP32(pStat->activePlps[Loop].plpEfficiencyMode);
                     pStat->activePlps[Loop].dnp = SMS_ENDIAN_SWAP32(pStat->activePlps[Loop].dnp);
                     pStat->activePlps[Loop].issyi = SMS_ENDIAN_SWAP32(pStat->activePlps[Loop].issyi);
                     pStat->activePlps[Loop].crcErrors = SMS_ENDIAN_SWAP32(pStat->activePlps[Loop].crcErrors);
                     pStat->activePlps[Loop].numOfLdpcIters = SMS_ENDIAN_SWAP32(pStat->activePlps[Loop].numOfLdpcIters);
                     pStat->activePlps[Loop].totalNumBBFramesReceived = SMS_ENDIAN_SWAP32(pStat->activePlps[Loop].totalNumBBFramesReceived); 
                     pStat->activePlps[Loop].totalNumErrBBFramesReceived = SMS_ENDIAN_SWAP32(pStat->activePlps[Loop].totalNumErrBBFramesReceived); 
                     pStat->activePlps[Loop].totalNumTsPktsReceived = SMS_ENDIAN_SWAP32(pStat->activePlps[Loop].totalNumTsPktsReceived);  
                     pStat->activePlps[Loop].totalNumTsPktsTransmitted = SMS_ENDIAN_SWAP32(pStat->activePlps[Loop].totalNumTsPktsTransmitted); 
                     pStat->activePlps[Loop].totalNumErrTsPktsReceived = SMS_ENDIAN_SWAP32(pStat->activePlps[Loop].totalNumErrTsPktsReceived);  
                     pStat->activePlps[Loop].numOfOverflow = SMS_ENDIAN_SWAP32(pStat->activePlps[Loop].numOfOverflow); 
                     pStat->activePlps[Loop].numOfUnderflow = SMS_ENDIAN_SWAP32(pStat->activePlps[Loop].numOfUnderflow);
                     pStat->activePlps[Loop].dejitterBufferSize = SMS_ENDIAN_SWAP32(pStat->activePlps[Loop].dejitterBufferSize);
                     pStat->activePlps[Loop].totalNumOfPktsInserted = SMS_ENDIAN_SWAP32(pStat->activePlps[Loop].totalNumOfPktsInserted);
                     pStat->activePlps[Loop].totalNumTsPktsForwarded = SMS_ENDIAN_SWAP32(pStat->activePlps[Loop].totalNumTsPktsForwarded);
                     pStat->activePlps[Loop].totalPostLdpcErr = SMS_ENDIAN_SWAP32(pStat->activePlps[Loop].totalPostLdpcErr);
                     pStat->activePlps[Loop].numTsPktsReceivedAfterReset = SMS_ENDIAN_SWAP32(pStat->activePlps[Loop].numTsPktsReceivedAfterReset);		
                     pStat->activePlps[Loop].numErrTsPktsReceivedAfterReset = SMS_ENDIAN_SWAP32(pStat->activePlps[Loop].numErrTsPktsReceivedAfterReset);		
                     pStat->activePlps[Loop].res[0] = SMS_ENDIAN_SWAP32(pStat->activePlps[Loop].res[0]);
				 }
			  }
		   }
		   else
		   {
			  UINT32* pData;
			  SMSHOSTLIB_STATISTICS_ISDBT_ST* pStat;
			  ULONG Loop;

			  pData = (UINT32*)&iicRxBuf[8];
              if(pData[0] == 0)
			  {          
                 pStat = (SMSHOSTLIB_STATISTICS_ISDBT_ST*)&pData[1];

				 pStat->StatisticsType = SMS_ENDIAN_SWAP32(pStat->StatisticsType);		
				 pStat->FullSize = SMS_ENDIAN_SWAP32(pStat->FullSize);			
				 pStat->IsRfLocked = SMS_ENDIAN_SWAP32(pStat->IsRfLocked);			
				 pStat->IsDemodLocked = SMS_ENDIAN_SWAP32(pStat->IsDemodLocked);			
				 pStat->IsExternalLNAOn = SMS_ENDIAN_SWAP32(pStat->IsExternalLNAOn);			
				 pStat->SNR = SMS_ENDIAN_SWAP32(pStat->SNR);						
				 pStat->RSSI = SMS_ENDIAN_SWAP32(pStat->RSSI);					
				 pStat->InBandPwr = SMS_ENDIAN_SWAP32(pStat->InBandPwr);				
				 pStat->CarrierOffset = SMS_ENDIAN_SWAP32(pStat->CarrierOffset);			
				 pStat->Frequency = SMS_ENDIAN_SWAP32(pStat->Frequency);				
				 pStat->Bandwidth = SMS_ENDIAN_SWAP32(pStat->Bandwidth);			
				 pStat->TransmissionMode = SMS_ENDIAN_SWAP32(pStat->TransmissionMode);		
				 pStat->ModemState = SMS_ENDIAN_SWAP32(pStat->ModemState);				
				 pStat->GuardInterval = SMS_ENDIAN_SWAP32(pStat->GuardInterval);			
				 pStat->SystemType = SMS_ENDIAN_SWAP32(pStat->SystemType);				
				 pStat->PartialReception = SMS_ENDIAN_SWAP32(pStat->PartialReception);		
				 pStat->NumOfLayers = SMS_ENDIAN_SWAP32(pStat->NumOfLayers);				
				 pStat->SegmentNumber = SMS_ENDIAN_SWAP32(pStat->SegmentNumber);			
				 pStat->TuneBW = SMS_ENDIAN_SWAP32(pStat->TuneBW);					
				 for(Loop=0;Loop<3;Loop++)
				 {
					 pStat->LayerInfo[Loop].CodeRate = SMS_ENDIAN_SWAP32(pStat->LayerInfo[Loop].CodeRate);
					 pStat->LayerInfo[Loop].Constellation = SMS_ENDIAN_SWAP32(pStat->LayerInfo[Loop].Constellation);
					 pStat->LayerInfo[Loop].BER = SMS_ENDIAN_SWAP32(pStat->LayerInfo[Loop].BER);	
					 pStat->LayerInfo[Loop].BERErrorCount = SMS_ENDIAN_SWAP32(pStat->LayerInfo[Loop].BERErrorCount);
					 pStat->LayerInfo[Loop].BERBitCount = SMS_ENDIAN_SWAP32(pStat->LayerInfo[Loop].BERBitCount);	
					 pStat->LayerInfo[Loop].PreBER = SMS_ENDIAN_SWAP32(pStat->LayerInfo[Loop].PreBER); 		
					 pStat->LayerInfo[Loop].TS_PER = SMS_ENDIAN_SWAP32(pStat->LayerInfo[Loop].TS_PER);			
					 pStat->LayerInfo[Loop].ErrorTSPackets = SMS_ENDIAN_SWAP32(pStat->LayerInfo[Loop].ErrorTSPackets);	
					 pStat->LayerInfo[Loop].TotalTSPackets = SMS_ENDIAN_SWAP32(pStat->LayerInfo[Loop].TotalTSPackets);	
					 pStat->LayerInfo[Loop].TILdepthI = SMS_ENDIAN_SWAP32(pStat->LayerInfo[Loop].TILdepthI);	
					 pStat->LayerInfo[Loop].NumberOfSegments = SMS_ENDIAN_SWAP32(pStat->LayerInfo[Loop].NumberOfSegments);
					 pStat->LayerInfo[Loop].TMCCErrors = SMS_ENDIAN_SWAP32(pStat->LayerInfo[Loop].TMCCErrors);	
				 }
			     pStat->Reserved1 = SMS_ENDIAN_SWAP32(pStat->Reserved1);
			     pStat->ExtAntenna = SMS_ENDIAN_SWAP32(pStat->ExtAntenna);				
			 	 pStat->ReceptionQuality = SMS_ENDIAN_SWAP32(pStat->ReceptionQuality);
				 pStat->EwsAlertActive = SMS_ENDIAN_SWAP32(pStat->EwsAlertActive);		
				 pStat->LNAOnOff = SMS_ENDIAN_SWAP32(pStat->LNAOnOff);	
			     pStat->RfAgcLevel = SMS_ENDIAN_SWAP32(pStat->RfAgcLevel);
				 pStat->BbAgcLevel = SMS_ENDIAN_SWAP32(pStat->BbAgcLevel);				
				 pStat->FwErrorsCounter = SMS_ENDIAN_SWAP32(pStat->FwErrorsCounter);			
				 pStat->MRC_SNR = SMS_ENDIAN_SWAP32(pStat->MRC_SNR);				
				 pStat->SNRFullRes = SMS_ENDIAN_SWAP32(pStat->SNRFullRes);				
				 pStat->layer_in_hier1 = SMS_ENDIAN_SWAP32(pStat->layer_in_hier1);
				 pStat->layer_in_hier2 = SMS_ENDIAN_SWAP32(pStat->layer_in_hier2);
				 pStat->MRC_InBandPwr = SMS_ENDIAN_SWAP32(pStat->MRC_InBandPwr);		
				 pStat->MRC_Rssi = SMS_ENDIAN_SWAP32(pStat->MRC_Rssi);
			  }
		   }
		}
		else
		if(pMsg->xMsgHeader.msgType == MSG_SMS_I2C_SHORT_STAT_IND)
		{
		   Short_Statistics_ST* pStat; 

		   pStat = (Short_Statistics_ST*)&iicRxBuf[8];

           pStat->IsDemodLocked = SMS_ENDIAN_SWAP32(pStat->IsDemodLocked);
           pStat->InBandPwr = SMS_ENDIAN_SWAP32(pStat->InBandPwr);
           pStat->BER = SMS_ENDIAN_SWAP32(pStat->BER);
           pStat->SNR = SMS_ENDIAN_SWAP32(pStat->SNR);
           pStat->TotalTSPackets = SMS_ENDIAN_SWAP32(pStat->TotalTSPackets);
           pStat->ErrorTSPackets = SMS_ENDIAN_SWAP32(pStat->ErrorTSPackets);
		}
#endif

		return 0xffff;
	}
	else
	{
	    if((pMsg->xMsgHeader.msgType < MSG_LAST_MSG_TYPE) && (pMsg->xMsgHeader.msgType > MSG_TYPE_BASE_VAL))
		{
			if(pMsg->xMsgHeader.msgType == MSG_SMS_HO_PER_SLICES_IND)
			{
			   SMS4470DebugPrintf("(SMS4470_read_response) got [MSG_SMS_HO_PER_SLICES_IND] message !\n");

	           if((Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == DVBT_DVBT2_DUAL_WORKING_MODE && Sms4470DeviceGroupInformation[Index].SMS4470CurrentDemodulationModeForDvbtDvbt2DualWorkingMode == SMS4470_DVBT_DEMODULATION_MODE) ||
	              Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == DVBT_WORKING_MODE)
                  SMS4470_HandlePerSlicesIndication(pMsg->msgData,&Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTReceptionStatistic);
				  PlatformPorting_Frontend_Reception_Statistics((void*)(&Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTReceptionStatistic));
			}
			else
			if(pMsg->xMsgHeader.msgType == MSG_SMS_HO_INBAND_POWER_IND)
			{
			   SMS4470DebugPrintf("(SMS4470_read_response) got [MSG_SMS_HO_INBAND_POWER_IND] message !\n");
			}
			else
            if(pMsg->xMsgHeader.msgType == MSG_SMS_TRANSMISSION_IND)
			{
			   SMS4470DebugPrintf("(SMS4470_read_response) got [MSG_SMS_TRANSMISSION_IND] message !\n");

	           if((Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == DVBT_DVBT2_DUAL_WORKING_MODE && Sms4470DeviceGroupInformation[Index].SMS4470CurrentDemodulationModeForDvbtDvbt2DualWorkingMode == SMS4470_DVBT_DEMODULATION_MODE) ||
	              Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == DVBT_WORKING_MODE)
			   {
                  memcpy(&Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic,&iicRxBuf[8],sizeof(TRANSMISSION_STATISTICS_ST));

#ifdef SMS_BIGENDIAN
				  Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.Frequency           = SMS_ENDIAN_SWAP32(Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.Frequency);
				  Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.Bandwidth           = SMS_ENDIAN_SWAP32(Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.Bandwidth);
				  Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.TransmissionMode    = SMS_ENDIAN_SWAP32(Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.TransmissionMode);
				  Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.GuardInterval       = SMS_ENDIAN_SWAP32(Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.GuardInterval);
				  Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.CodeRate            = SMS_ENDIAN_SWAP32(Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.CodeRate);
				  Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.LPCodeRate          = SMS_ENDIAN_SWAP32(Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.LPCodeRate);
				  Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.Hierarchy           = SMS_ENDIAN_SWAP32(Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.Hierarchy);
				  Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.Constellation       = SMS_ENDIAN_SWAP32(Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.Constellation);
				  Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.CellId              = SMS_ENDIAN_SWAP32(Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.CellId);
				  Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.DvbhSrvIndHP        = SMS_ENDIAN_SWAP32(Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.DvbhSrvIndHP);
				  Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.DvbhSrvIndLP        = SMS_ENDIAN_SWAP32(Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.DvbhSrvIndLP);
				  Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.IsDemodLocked       = SMS_ENDIAN_SWAP32(Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.IsDemodLocked);					  
#endif
			   }
			}
			else
            if(pMsg->xMsgHeader.msgType == MSG_SMS_SIGNAL_DETECTED_IND)
			{
			   SMS4470DebugPrintf("(SMS4470_read_response) got [MSG_SMS_SIGNAL_DETECTED_IND] message !\n");
			}
			else
            if(pMsg->xMsgHeader.msgType == MSG_SMS_NO_SIGNAL_IND)
			{
			   Sms4470DeviceGroupInformation[Index].SMS4470DetectNoSignalFlag = TRUE;
			   SMS4470DebugPrintf("(SMS4470_read_response) got [MSG_SMS_NO_SIGNAL_IND] message !\n");
			}
		}
	}

//	SMS4470DebugPrintf("(SMS4470_read_response) pMsg->xMsgHeader.msgLength : %d\n",pMsg->xMsgHeader.msgLength);
    
	return pMsg->xMsgHeader.msgLength;
}

#define CORRECT_STAT_RSSI(_stat) (_stat).RSSI *= -1
#define CORRECT_STAT_MRC_RSSI(_stat) (_stat).MRC_RSSI *= -1

VOID SMS4470_HandlePerSlicesIndication(PUBYTE pData,RECEPTION_STATISTICS_ST* pReceptionStatistic)
{
	UINT32 i;
	UINT32* pMsgData = (UINT32*)pData;
	UINT32 RecQualSum = 0;
	UINT32 Snr;
	UINT32 InBandPower;
	UINT32 TsPackets;
	UINT32 EtsPackets;
	UINT32 Constellation;
	UINT32 HpCode;

	Snr             = pMsgData[1];
#ifdef SMS_BIGENDIAN
	Snr             = SMS_ENDIAN_SWAP32(Snr);
#endif
	InBandPower     = (INT32)pMsgData[2];
#ifdef SMS_BIGENDIAN
	InBandPower     = SMS_ENDIAN_SWAP32(InBandPower);
#endif
	TsPackets       = pMsgData[3];
#ifdef SMS_BIGENDIAN
	TsPackets       = SMS_ENDIAN_SWAP32(TsPackets);
#endif
	EtsPackets      = pMsgData[4];
#ifdef SMS_BIGENDIAN
	EtsPackets      = SMS_ENDIAN_SWAP32(EtsPackets);
#endif
	Constellation   = pMsgData[5];
#ifdef SMS_BIGENDIAN
	Constellation   = SMS_ENDIAN_SWAP32(Constellation);
#endif
	HpCode          = pMsgData[6];
#ifdef SMS_BIGENDIAN
	HpCode          = SMS_ENDIAN_SWAP32(HpCode);
#endif

	pReceptionStatistic->IsRfLocked			= pMsgData[16];				
#ifdef SMS_BIGENDIAN
	pReceptionStatistic->IsRfLocked         = SMS_ENDIAN_SWAP32(pReceptionStatistic->IsRfLocked);
#endif
	pReceptionStatistic->IsDemodLocked		= pMsgData[17];	
#ifdef SMS_BIGENDIAN
	pReceptionStatistic->IsDemodLocked      = SMS_ENDIAN_SWAP32(pReceptionStatistic->IsDemodLocked);
#endif
	pReceptionStatistic->ModemState			= pMsgData[12];	
#ifdef SMS_BIGENDIAN
	pReceptionStatistic->ModemState         = SMS_ENDIAN_SWAP32(pReceptionStatistic->ModemState);
#endif
	pReceptionStatistic->SNR			    = pMsgData[1];						
#ifdef SMS_BIGENDIAN
	pReceptionStatistic->SNR                = SMS_ENDIAN_SWAP32(pReceptionStatistic->SNR);
#endif
	pReceptionStatistic->BER				= pMsgData[13]; 	
#ifdef SMS_BIGENDIAN
	pReceptionStatistic->BER                = SMS_ENDIAN_SWAP32(pReceptionStatistic->BER);
#endif
	pReceptionStatistic->RSSI				= pMsgData[14];
#ifdef SMS_BIGENDIAN
	pReceptionStatistic->RSSI               = SMS_ENDIAN_SWAP32(pReceptionStatistic->RSSI);
#endif
	CORRECT_STAT_RSSI(*pReceptionStatistic); 

	pReceptionStatistic->InBandPwr			= (INT32)pMsgData[2];		
#ifdef SMS_BIGENDIAN
	pReceptionStatistic->InBandPwr          = SMS_ENDIAN_SWAP32(pReceptionStatistic->InBandPwr);
#endif
	pReceptionStatistic->CarrierOffset		= (INT32)pMsgData[15];	
#ifdef SMS_BIGENDIAN
	pReceptionStatistic->CarrierOffset      = SMS_ENDIAN_SWAP32(pReceptionStatistic->CarrierOffset);
#endif
	pReceptionStatistic->TotalTSPackets		= pMsgData[3];
#ifdef SMS_BIGENDIAN
	pReceptionStatistic->TotalTSPackets     = SMS_ENDIAN_SWAP32(pReceptionStatistic->TotalTSPackets);
#endif
	pReceptionStatistic->ErrorTSPackets		= pMsgData[4];
#ifdef SMS_BIGENDIAN
	pReceptionStatistic->ErrorTSPackets     = SMS_ENDIAN_SWAP32(pReceptionStatistic->ErrorTSPackets);
#endif

	//TSPER
	if ((TsPackets + EtsPackets) > 0)
	{
		pReceptionStatistic->TS_PER = (EtsPackets * 100) / (TsPackets + EtsPackets);		
	}
	else
	{
		pReceptionStatistic->TS_PER = 0;
	}

	pReceptionStatistic->BERBitCount			= pMsgData[18];						
#ifdef SMS_BIGENDIAN
	pReceptionStatistic->BERBitCount            = SMS_ENDIAN_SWAP32(pReceptionStatistic->BERBitCount);
#endif
	pReceptionStatistic->BERErrorCount			= pMsgData[19];
#ifdef SMS_BIGENDIAN
	pReceptionStatistic->BERErrorCount          = SMS_ENDIAN_SWAP32(pReceptionStatistic->BERErrorCount);
#endif

	pReceptionStatistic->MRC_SNR				= pMsgData[20];
#ifdef SMS_BIGENDIAN
	pReceptionStatistic->MRC_SNR                = SMS_ENDIAN_SWAP32(pReceptionStatistic->MRC_SNR);
#endif
	pReceptionStatistic->MRC_InBandPwr			= pMsgData[21];
#ifdef SMS_BIGENDIAN
	pReceptionStatistic->MRC_InBandPwr          = SMS_ENDIAN_SWAP32(pReceptionStatistic->MRC_InBandPwr);
#endif
	pReceptionStatistic->MRC_RSSI				= pMsgData[22];
#ifdef SMS_BIGENDIAN
	pReceptionStatistic->MRC_RSSI               = SMS_ENDIAN_SWAP32(pReceptionStatistic->MRC_RSSI);
#endif
	CORRECT_STAT_MRC_RSSI(*pReceptionStatistic); 

	pReceptionStatistic->RefDevPPM				= pMsgData[23];
#ifdef SMS_BIGENDIAN
	pReceptionStatistic->RefDevPPM              = SMS_ENDIAN_SWAP32(pReceptionStatistic->RefDevPPM);
#endif
	pReceptionStatistic->FreqDevHz				= pMsgData[24];
#ifdef SMS_BIGENDIAN
	pReceptionStatistic->FreqDevHz              = SMS_ENDIAN_SWAP32(pReceptionStatistic->FreqDevHz);
#endif
}
//**********************************************************************************************************
//**********************************************************************************************************
//**********************************************************************************************************
BOOL SMS4470_dual_mode_SwitchStandard(ULONG FrontendGroupID)
{
	ULONG                   Index;
	UINT32                  i;
	UINT32                  ResponseResult;
	SmsMsgHdr_ST            SmsMsg;
	UINT16                  TempMsgLength;

    SMS4470FunctionNamePrintf("(SMS4470_dual_mode_SwitchStandard)\n");

	if(FrontendGroupID == FRONTEND_TWO_DIVERSITY_GROUP_ID)
	   Index = 0;
	else
	   Index = 1;

    SmsMsg.msgType   = MSG_SW_SWITCH_STANDARD_REQ;
	SmsMsg.msgSrcId  = SMS_HOST_LIB;
	SmsMsg.msgDstId  = HIF_TASK;
	SmsMsg.msgLength = sizeof(SmsMsgHdr_ST);
	SmsMsg.msgFlags  = MSG_HDR_FLAG_STATIC_MSG;

   
SMS4470DebugPrintf("(SMS4470_dual_mode_SwitchStandard) Send command (MSG_SW_SWITCH_STANDARD_REQ) to SMS4470\n");

    TempMsgLength = SmsMsg.msgLength;
#ifdef SMS_BIGENDIAN
    SmsMsg.msgType     = SMS_ENDIAN_SWAP16(SmsMsg.msgType);
    SmsMsg.msgLength   = SMS_ENDIAN_SWAP16(SmsMsg.msgLength);
    SmsMsg.msgFlags    = SMS_ENDIAN_SWAP16(SmsMsg.msgFlags);
#endif

//    SMS4470SysDelay(5);
	SMS4470I2CWrite(FrontendGroupID,(UINT8*)&SmsMsg,TempMsgLength);
	
   
SMS4470DebugPrintf("(SMS4470_dual_mode_SwitchStandard) Get respence from SMS4470\n");

//	memset(Sms4470DeviceGroupInformation[Index].Sms4470MessageRxBuffer,0xaa,SIANO_IIC_RX_BUFFER_SIZE);

	for(i=0; i<CriticalCommandWaitResponseLoop; i++)
	{
        SMS4470SysDelay(10);
		ResponseResult = SMS4470_read_response(FrontendGroupID,MSG_SW_SWITCH_STANDARD_RES,Sms4470DeviceGroupInformation[Index].Sms4470MessageRxBuffer,SIANO_IIC_RX_BUFFER_SIZE);
	    if(ResponseResult == 0xffff)
		{
		   break;
		}
	    else
	    if(ResponseResult == 0xf000)
		{
           i = CriticalCommandWaitResponseLoop;
		   break;
		}
	}

	if(i == CriticalCommandWaitResponseLoop)
	{
       SMS4470ErrorPrintf("(SMS4470_dual_mode_SwitchStandard) xxxxxxxxx Get response failed !\n");

       return FALSE;
	}

	if(SMS4470_wait_device_redetected(FrontendGroupID) != TRUE)
       return FALSE;

//SMS4470ErrorPrintf("(SMS4470_dual_mode_SwitchStandard) End\n");

	return TRUE;
}
//**********************************************************************************************************
//**********************************************************************************************************
//**********************************************************************************************************
BOOL SMS4470_check_signal(ULONG FrontendGroupID,BOOL* pLockStatus)
{
	ULONG            Index;
    UINT32           i;
	UINT32           ResponseResult;
    BOOL             LockFlag = FALSE;
    SmsMsgData_ST*   pRespondseMsg;
	SmsMsgData_ST    SmsMsg = {{0}};
	UINT16           TempMsgLength;
    
    SMS4470FunctionNamePrintf("(SMS4470_check_signal)\n");

	if(FrontendGroupID == FRONTEND_TWO_DIVERSITY_GROUP_ID)
	   Index = 0;
	else
	   Index = 1;
  
	if((Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == DVBT_DVBT2_DUAL_WORKING_MODE && Sms4470DeviceGroupInformation[Index].SMS4470CurrentDemodulationModeForDvbtDvbt2DualWorkingMode == SMS4470_DVBT2_DEMODULATION_MODE) ||
	   Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == DVBT2_WORKING_MODE)
	{

  
																//SMS4470ErrorPrintf("(SMS4470_check_signal) check signal for DVB-T2\n");


																	   SmsMsg.xMsgHeader.msgType   = MSG_SMS_GET_STATISTICS_EX_REQ;
																	   SmsMsg.xMsgHeader.msgSrcId  = SMS_HOST_LIB; // DVBT_BDA_CONTROL_MSG_ID; // 0;
																	   SmsMsg.xMsgHeader.msgDstId  = HIF_TASK;               
																	   SmsMsg.xMsgHeader.msgLength = sizeof(SmsMsg);
																	   SmsMsg.xMsgHeader.msgFlags  = MSG_HDR_FLAG_STATIC_MSG; // 0;
    

																	   TempMsgLength = SmsMsg.xMsgHeader.msgLength;
																#ifdef SMS_BIGENDIAN
																	   SmsMsg.xMsgHeader.msgType     = SMS_ENDIAN_SWAP16(SmsMsg.xMsgHeader.msgType);
																	   SmsMsg.xMsgHeader.msgLength   = SMS_ENDIAN_SWAP16(SmsMsg.xMsgHeader.msgLength);
																	   SmsMsg.xMsgHeader.msgFlags    = SMS_ENDIAN_SWAP16(SmsMsg.xMsgHeader.msgFlags);
																#endif

																//       SMS4470SysDelay(5); 
																	   SMS4470I2CWrite(FrontendGroupID,(UINT8*)&SmsMsg,TempMsgLength);
	   
																	   for(i=0 ; i<SpecialCommandWaitResponseLoop ; i++)
																	   {
																		   SMS4470SysDelay(10);
																		   ResponseResult = SMS4470_read_response(FrontendGroupID,MSG_SMS_GET_STATISTICS_EX_RES, Sms4470DeviceGroupInformation[Index].Sms4470MessageRxBuffer, SIANO_IIC_RX_BUFFER_SIZE);
																		   if(ResponseResult == 0xffff) 
																		   {
																			  break;
																		   }
																		   else
																		   if(ResponseResult == 0xf000)
																		   {
																			  i = SpecialCommandWaitResponseLoop;
																			  break;
																		   }
																	   }
    
																	   if(i == SpecialCommandWaitResponseLoop)
																	   {
																		  SMS4470ErrorPrintf("(SMS4470_check_signal) xxxxxxxxx Get response failed !\n");
																		  OSPorting_Frontend_failflag(16, 16);
																		  *pLockStatus = FALSE;

																		  return FALSE;
																	   }

																	   SMSHOSTLIB_STATISTICS_DVBT2_ST* pStat;    
     
																	   pRespondseMsg = (SmsMsgData_ST*)Sms4470DeviceGroupInformation[Index].Sms4470MessageRxBuffer;

																	   if(pRespondseMsg->msgData[0] == 0)
																	   {          
																		  pStat = (SMSHOSTLIB_STATISTICS_DVBT2_ST*)&pRespondseMsg->msgData[1];

																		  SMS4470DebugPrintf("(SMS4470_check_signal) IsModemLocked : 0x%x\n",pStat->ReceptionData.IsModemLocked);
        
																		  if(pStat->ReceptionData.IsModemLocked == 0x01)
																		  {
																			 *pLockStatus = TRUE;

																			 return TRUE;
																		  }
																	   }
																	   else
																	   {
																		  SMS4470DebugPrintf("(SMS4470_check_signal) pRespondseMsg->msgData[0] != 0\n");
																	   }
	}
	else
	if((Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == DVBT_DVBT2_DUAL_WORKING_MODE && Sms4470DeviceGroupInformation[Index].SMS4470CurrentDemodulationModeForDvbtDvbt2DualWorkingMode == SMS4470_DVBT_DEMODULATION_MODE) ||
	   Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == DVBT_WORKING_MODE)
	{

  
//SMS4470ErrorPrintf("(SMS4470_check_signal) check signal for DVB-T\n");


       for(i=0 ; i<SpecialCommandWaitResponseLoop ; i++)
	   {
           SMS4470SysDelay(10);   //YH: ctrl polling blocking delay
		   ResponseResult = SMS4470_read_response(FrontendGroupID,MSG_SMS_I2C_SHORT_STAT_IND, Sms4470DeviceGroupInformation[Index].Sms4470MessageRxBuffer, SIANO_IIC_RX_BUFFER_SIZE);
           if(ResponseResult == 0xffff) 
		   {
              break;
		   }
		   else
		   if(ResponseResult == 0xf000)
		   {
              i = SpecialCommandWaitResponseLoop;
		      break;
		   }
	   }
    
       if(i == SpecialCommandWaitResponseLoop)
	   {
	      SMS4470ErrorPrintf("(SMS4470_check_signal) xxxxxxxxx Get response failed !\n");

		  *pLockStatus = FALSE;

          return FALSE;
	   }

       Short_Statistics_ST* pStat;
        
       pRespondseMsg = (SmsMsgData_ST*)Sms4470DeviceGroupInformation[Index].Sms4470MessageRxBuffer;
        
       pStat = (Short_Statistics_ST*)&pRespondseMsg->msgData[0];
	   PlatformPorting_Frontend_Short_Statistics((void *)pStat);

       if(pStat->IsDemodLocked == 0x01)
	   {
		   *pLockStatus = FALSE;  //testing
		  if(Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTReceptionStatistic.IsDemodLocked)
		  {
			  *pLockStatus = FALSE;  //testing
			 if(Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.IsDemodLocked)
			 {
		        *pLockStatus = TRUE;

                return TRUE;
			 }
			 *pLockStatus = FALSE;  //testing
		  }
		  *pLockStatus = FALSE;  //testing
	   }
	   *pLockStatus = FALSE;  //testing
	}
	else
	{

  
											//SMS4470ErrorPrintf("(SMS4470_check_signal) check signal for ISDB-T\n");


												   SmsMsg.xMsgHeader.msgType   = MSG_SMS_GET_STATISTICS_EX_REQ;
												   SmsMsg.xMsgHeader.msgSrcId  = SMS_HOST_LIB; // DVBT_BDA_CONTROL_MSG_ID; // 0;
												   SmsMsg.xMsgHeader.msgDstId  = HIF_TASK;                
												   SmsMsg.xMsgHeader.msgLength = sizeof(SmsMsg);
												   SmsMsg.xMsgHeader.msgFlags  = MSG_HDR_FLAG_STATIC_MSG; // 0;
    

												   TempMsgLength = SmsMsg.xMsgHeader.msgLength;
											#ifdef SMS_BIGENDIAN
												   SmsMsg.xMsgHeader.msgType     = SMS_ENDIAN_SWAP16(SmsMsg.xMsgHeader.msgType);
												   SmsMsg.xMsgHeader.msgLength   = SMS_ENDIAN_SWAP16(SmsMsg.xMsgHeader.msgLength);
												   SmsMsg.xMsgHeader.msgFlags    = SMS_ENDIAN_SWAP16(SmsMsg.xMsgHeader.msgFlags);
											#endif

											//       SMS4470SysDelay(5); 
												   SMS4470I2CWrite(FrontendGroupID,(UINT8*)&SmsMsg,TempMsgLength);
   
												   for(i=0 ; i<SpecialCommandWaitResponseLoop ; i++)
												   {
													   SMS4470SysDelay(10);
													   ResponseResult = SMS4470_read_response(FrontendGroupID,MSG_SMS_GET_STATISTICS_EX_RES, Sms4470DeviceGroupInformation[Index].Sms4470MessageRxBuffer, SIANO_IIC_RX_BUFFER_SIZE);
													   if(ResponseResult == 0xffff) 
													   {
														  break;
													   }
													   else
													   if(ResponseResult == 0xf000)
													   {
														  i = SpecialCommandWaitResponseLoop;
														  break;
													   }
												   }
    
												   if(i == SpecialCommandWaitResponseLoop)
												   {
													  SMS4470ErrorPrintf("(SMS4470_check_signal) xxxxxxxxx Get response failed !\n");

													  *pLockStatus = FALSE;

													  return FALSE;
												   }

												   SMSHOSTLIB_STATISTICS_ISDBT_ST* pStat;    
     
												   pRespondseMsg = (SmsMsgData_ST*)Sms4470DeviceGroupInformation[Index].Sms4470MessageRxBuffer;

												   if(pRespondseMsg->msgData[0] == 0)
												   {          
													  pStat = (SMSHOSTLIB_STATISTICS_ISDBT_ST*)&pRespondseMsg->msgData[1];
        
													  SMS4470DebugPrintf("(SMS4470_check_signal) IsDemodLocked : 0x%x\n",pStat->IsDemodLocked);

													  if(pStat->IsDemodLocked == 0x01)
													  {
														 *pLockStatus = TRUE;

														 return TRUE;
													  }
												   }
												   else
												   {
													  SMS4470DebugPrintf("(SMS4470_check_signal) pRespondseMsg->msgData[0] != 0\n");
												   }
	}

	*pLockStatus = FALSE;

	return TRUE;
}
//**********************************************************************************************************
//**********************************************************************************************************
//**********************************************************************************************************
BOOL SMS4470_GetReceptionStatistics(ULONG FrontendGroupID,PSTATISTICS_INFORMATION pStatistics)
{
	ULONG           Index;
    UINT32          i;
	UINT32          ResponseResult;
    UINT32          MaxPlpData;
    UINT32          MaxActivePlp;
	SmsMsgData_ST   SmsMsg = {{0}};
    SmsMsgData_ST*  pRespondseMsg;
	BOOL            PlpValidCheckFlag;
	BOOL            ActivePlpMatchCheckFlag;
	UINT16          TempMsgLength;
    
    SMS4470FunctionNamePrintf("(SMS4470_GetReceptionStatistics)\n");

    pStatistics->IsSignalLocked = FALSE;
	pStatistics->StatisticAvailableFlag = FALSE;

	if(FrontendGroupID == FRONTEND_TWO_DIVERSITY_GROUP_ID)
	   Index = 0;
	else
	   Index = 1;

	if((Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == DVBT_DVBT2_DUAL_WORKING_MODE && Sms4470DeviceGroupInformation[Index].SMS4470CurrentDemodulationModeForDvbtDvbt2DualWorkingMode == SMS4470_DVBT2_DEMODULATION_MODE) ||
	   Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == DVBT2_WORKING_MODE)
	{
	   // for DVB-T2

       SmsMsg.xMsgHeader.msgType   = MSG_SMS_GET_STATISTICS_EX_REQ;
       SmsMsg.xMsgHeader.msgSrcId  = SMS_HOST_LIB; // DVBT_BDA_CONTROL_MSG_ID; // 0;
       SmsMsg.xMsgHeader.msgDstId  = HIF_TASK;                
       SmsMsg.xMsgHeader.msgLength = sizeof(SmsMsg);
       SmsMsg.xMsgHeader.msgFlags  = MSG_HDR_FLAG_STATIC_MSG; // 0;

       TempMsgLength = SmsMsg.xMsgHeader.msgLength;
#ifdef SMS_BIGENDIAN
       SmsMsg.xMsgHeader.msgType     = SMS_ENDIAN_SWAP16(SmsMsg.xMsgHeader.msgType);
       SmsMsg.xMsgHeader.msgLength   = SMS_ENDIAN_SWAP16(SmsMsg.xMsgHeader.msgLength);
       SmsMsg.xMsgHeader.msgFlags    = SMS_ENDIAN_SWAP16(SmsMsg.xMsgHeader.msgFlags);
#endif

//       SMS4470SysDelay(5); 
	   SMS4470I2CWrite(FrontendGroupID,(UINT8*)&SmsMsg,TempMsgLength);
   
       for(i=0 ; i<SpecialCommandWaitResponseLoop ; i++)
	   {
           SMS4470SysDelay(10);
		   ResponseResult = SMS4470_read_response(FrontendGroupID,MSG_SMS_GET_STATISTICS_EX_RES, Sms4470DeviceGroupInformation[Index].Sms4470MessageRxBuffer, SIANO_IIC_RX_BUFFER_SIZE);
           if(ResponseResult == 0xffff) 
		   {
              break;
		   }
		   else
		   if(ResponseResult == 0xf000)
		   {
              i = SpecialCommandWaitResponseLoop;
		      break;
		   }
	   }
    
       if(i == SpecialCommandWaitResponseLoop)
	   {
	      SMS4470ErrorPrintf("(SMS4470_GetReceptionStatistics) xxxxxxxxx Get response failed !\n");

          return TRUE;
	   }

       SMSHOSTLIB_STATISTICS_DVBT2_ST* pStat;    
     
       pRespondseMsg = (SmsMsgData_ST*)Sms4470DeviceGroupInformation[Index].Sms4470MessageRxBuffer;

       if(pRespondseMsg->msgData[0] == 0)
	   {          
          pStat = (SMSHOSTLIB_STATISTICS_DVBT2_ST*)&pRespondseMsg->msgData[1];
        
	      if(pStat->ReceptionData.IsModemLocked == 0x01)
		  {
             pStatistics->IsSignalLocked = TRUE;

             SMS4470StatisticPrintf("********************************************************************************\n");
             SMS4470StatisticPrintf("***********************     SMS4470 Signal Locked !     ************************\n");
             SMS4470StatisticPrintf("********************************************************************************\n");
             SMS4470StatisticPrintf("************************* Statistics Data Information **************************\n");

	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) IsModemLocked : 0x%x\n",pStat->ReceptionData.IsModemLocked);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) txStatistics.Frequency : %d\n",pStat->ReceptionData.txStatistics.Frequency);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) txStatistics.Bandwidth : %d\n",pStat->ReceptionData.txStatistics.Bandwidth);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) txStatistics.res[0] : 0x%x\n",pStat->ReceptionData.txStatistics.res[0]);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) txStatistics.res[1] : 0x%x\n",pStat->ReceptionData.txStatistics.res[1]);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) txStatistics.res[2] : 0x%x\n",pStat->ReceptionData.txStatistics.res[2]);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) carrierOffset : %d\n",pStat->ReceptionData.carrierOffset);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) inbandPower : %d\n",pStat->ReceptionData.inbandPower);  
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) extLna : %d\n",pStat->ReceptionData.extLna);     
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) totalFrames : %d\n",pStat->ReceptionData.totalFrames);    			  
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) SNR : %d\n",pStat->ReceptionData.SNR); 						//!< dB     
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) RSSI : %d\n",pStat->ReceptionData.RSSI); 					//!< dBm     
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) FER : %d\n",pStat->ReceptionData.FER);	
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) CellId : 0x%x\n",pStat->ReceptionData.CellId);	
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) netId : 0x%x\n",pStat->ReceptionData.netId);	
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) receptionQuality : %d\n",pStat->ReceptionData.receptionQuality);	        
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) bwt_ext : 0x%x\n",pStat->ReceptionData.bwt_ext);	
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) fftMode : 0x%x\n",pStat->ReceptionData.fftMode);       
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) guardInterval : 0x%x\n",pStat->ReceptionData.guardInterval);       
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) pilotPattern : 0x%x\n",pStat->ReceptionData.pilotPattern);       
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) bitRate : %d\n",pStat->ReceptionData.bitRate);	  
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) extended : 0x%x\n",pStat->ReceptionData.extended); 
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) toneReservation : 0x%x\n",pStat->ReceptionData.toneReservation);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) l1PostSize : %d\n",pStat->ReceptionData.l1PostSize);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) numOfAuxs : %d\n",pStat->ReceptionData.numOfAuxs);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) numOfPlps : %d\n",pStat->ReceptionData.numOfPlps);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) liteMode : 0x%x\n",pStat->ReceptionData.liteMode);

	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) MRC_SNR : %d\n",pStat->ReceptionData.MRC_SNR); 						//!< dB     
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) SNRFullRes : %d\n",pStat->ReceptionData.SNRFullRes); 				    //!< dB x 65536     
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) MRC_InBandPwr : %d\n",pStat->ReceptionData.MRC_InBandPwr); 		    //!< In band power in dBM    
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) MRC_Rssi : %d\n",pStat->ReceptionData.MRC_Rssi); 				   

	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) commonPlpNotSupported : 0x%x\n",pStat->ReceptionData.commonPlpNotSupported);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) l1modulation : 0x%x\n",pStat->ReceptionData.l1modulation);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) numdatasymbols : %d\n",pStat->ReceptionData.numdatasymbols);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) res[0] : 0x%x\n",pStat->ReceptionData.res[0]);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) res[1] : 0x%x\n",pStat->ReceptionData.res[1]);

	         if(pStat->ReceptionData.numOfPlps != 0)
			 {
			    PlpValidCheckFlag = TRUE;
                for(i=0;i<pStat->ReceptionData.numOfPlps;i++)
				{
			        if(pStat->PlpData[i].plpStatistics.plpId == -1)
					{
                       PlpValidCheckFlag = FALSE;
				       break;
					}
				}
                if(PlpValidCheckFlag)
				{
	               SMS4470StatisticPrintf("********************************* PLP ID ***************************************\n");
                   if(pStat->ReceptionData.numOfPlps > DVBT2_MAX_PLPS_LITE)
			          MaxPlpData = DVBT2_MAX_PLPS_LITE;
			       else
				      MaxPlpData = pStat->ReceptionData.numOfPlps;
                   for(i=0;i<MaxPlpData;i++)
				   {
                       SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) plpId(%d) : %d\n",i,pStat->PlpData[i].plpStatistics.plpId);
                       SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) plpType(%d) : 0x%x\n",i,pStat->PlpData[i].plpStatistics.plpType);
                       SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) plpPayloadType(%d) : 0x%x\n",i,pStat->PlpData[i].plpStatistics.plpPayloadType);

                       SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) ffFlag(%d) : 0x%x\n",i,pStat->PlpData[i].plpStatistics.ffFlag);
                       SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) firstRfIdx(%d) : 0x%x\n",i,pStat->PlpData[i].plpStatistics.firstRfIdx);
                       SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) firstFrameIdx(%d) : 0x%x\n",i,pStat->PlpData[i].plpStatistics.firstFrameIdx);
                       SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) plpGroupId(%d) : 0x%x\n",i,pStat->PlpData[i].plpStatistics.plpGroupId);
                       SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) plpCod(%d) : 0x%x\n",i,pStat->PlpData[i].plpStatistics.plpCod);

                       SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) plpMod(%d) : 0x%x\n",i,pStat->PlpData[i].plpStatistics.plpMod);
                       SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) plpRotation(%d) : 0x%x\n",i,pStat->PlpData[i].plpStatistics.plpRotation);
                       SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) plpFecType(%d) : 0x%x\n",i,pStat->PlpData[i].plpStatistics.plpFecType);
                       SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) plpNumBlocksMax(%d) : 0x%x\n",i,pStat->PlpData[i].plpStatistics.plpNumBlocksMax);
                       SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) frameInterval(%d) : 0x%x\n",i,pStat->PlpData[i].plpStatistics.frameInterval);
                       SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) timeIlLength(%d) : 0x%x\n",i,pStat->PlpData[i].plpStatistics.timeIlLength);
                       SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) timeIlType(%d) : 0x%x\n",i,pStat->PlpData[i].plpStatistics.timeIlType);
                       SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) inbandA_Flag(%d) : 0x%x\n",i,pStat->PlpData[i].plpStatistics.inbandA_Flag);
                       SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) inbandB_Flag(%d) : 0x%x\n",i,pStat->PlpData[i].plpStatistics.inbandB_Flag);

                       SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) plpMode(%d) : 0x%x\n",i,pStat->PlpData[i].plpStatistics.plpMode);
                       SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) staticFlag(%d) : 0x%x\n",i,pStat->PlpData[i].plpStatistics.staticFlag);
                       SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) staticPaddingFlag(%d) : 0x%x\n",i,pStat->PlpData[i].plpStatistics.staticPaddingFlag);
                       SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) res[0](%d) : 0x%x\n",i,pStat->PlpData[i].plpStatistics.res[0]);
                       SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) res[1](%d) : 0x%x\n",i,pStat->PlpData[i].plpStatistics.res[1]);
                       SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) res[2](%d) : 0x%x\n",i,pStat->PlpData[i].plpStatistics.res[2]);
				   }

	               SMS4470StatisticPrintf("************************* Active PLP Information *******************************\n");
                   if(pStat->ReceptionData.numOfPlps > DVBT2_ACTIVE_PLPS_LITE)
			          MaxActivePlp = DVBT2_ACTIVE_PLPS_LITE;
			       else
				      MaxActivePlp = pStat->ReceptionData.numOfPlps;
				   ActivePlpMatchCheckFlag = TRUE;
			       for(i=0;i<MaxActivePlp;i++)
				   {
                       if(i == 0)
					   {
                          if(Sms4470DeviceGroupInformation[Index].SMS4470DVBT2LastPlpIdSetup != 0xFF)
						  {
                             if(pStat->activePlps[i].plpId != 0xFFFFFFFF)
							 {
					            if(pStat->activePlps[i].plpId != Sms4470DeviceGroupInformation[Index].SMS4470DVBT2LastPlpIdSetup)
								{

                                   SMS4470InfoPrintf("(SMS4470_GetReceptionStatistics) Working around for unmatch PLP ID\n");

	                               SMS4470_OpenPlp(FrontendGroupID,Sms4470DeviceGroupInformation[Index].SMS4470DVBT2LastPlpIdSetup);

								   ActivePlpMatchCheckFlag = FALSE;
								}
							 }
						  }
					   }
					  
	             	   SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) plpId(%d) : 0x%x\n",i,pStat->activePlps[i].plpId);
	             	   SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) plpType(%d) : 0x%x\n",i,pStat->activePlps[i].plpType);
	             	   SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) plpEfficiencyMode(%d) : 0x%x\n",i,pStat->activePlps[i].plpEfficiencyMode);
	             	   SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) dnp(%d) : 0x%x\n",i,pStat->activePlps[i].dnp);
	             	   SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) issyi(%d) : 0x%x\n",i,pStat->activePlps[i].issyi);
	             	   SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) crcErrors(%d) : 0x%x\n",i,pStat->activePlps[i].crcErrors);
	             	   SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) numOfLdpcIters(%d) : 0x%x\n",i,pStat->activePlps[i].numOfLdpcIters);
	             	   SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) totalNumBBFramesReceived(%d) : %d\n",i,pStat->activePlps[i].totalNumBBFramesReceived);
	             	   SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) totalNumErrBBFramesReceived(%d) : %d\n",i,pStat->activePlps[i].totalNumErrBBFramesReceived);
	             	   SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) totalNumTsPktsReceived(%d) : %d\n",i,pStat->activePlps[i].totalNumTsPktsReceived);
	             	   SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) totalNumTsPktsTransmitted(%d) : %d\n",i,pStat->activePlps[i].totalNumTsPktsTransmitted);
	             	   SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) totalNumErrTsPktsReceived(%d) : %d\n",i,pStat->activePlps[i].totalNumErrTsPktsReceived);
	             	   SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) numOfOverflow(%d) : %d\n",i,pStat->activePlps[i].numOfOverflow);
	             	   SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) numOfUnderflow(%d) : %d\n",i,pStat->activePlps[i].numOfUnderflow);
	             	   SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) dejitterBufferSize(%d) : %d\n",i,pStat->activePlps[i].dejitterBufferSize);
	             	   SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) totalNumOfPktsInserted(%d) : %d\n",i,pStat->activePlps[i].totalNumOfPktsInserted);
	             	   SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) totalNumTsPktsForwarded(%d) : %d\n",i,pStat->activePlps[i].totalNumTsPktsForwarded);
	             	   SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) totalPostLdpcErr(%d) : %d\n",i,pStat->activePlps[i].totalPostLdpcErr);
	             	   SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) numTsPktsReceivedAfterReset(%d) : %d\n",i,pStat->activePlps[i].numTsPktsReceivedAfterReset);
	             	   SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) numErrTsPktsReceivedAfterReset(%d) : %d\n",i,pStat->activePlps[i].numErrTsPktsReceivedAfterReset);
	             	   SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) res[0](%d) : %d\n",i,pStat->activePlps[i].res[0]);

			 	   }
                   if(ActivePlpMatchCheckFlag)
				   {
                      pStatistics->StatisticAvailableFlag = TRUE;
                      pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.SNR = pStat->ReceptionData.MRC_SNR;
                      pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.InBandPower = pStat->ReceptionData.MRC_InBandPwr;
                      pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.ErrorTSPackets = pStat->activePlps[0].totalNumErrTsPktsReceived;
                      pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.TotalTSPackets = pStat->activePlps[0].totalNumTsPktsReceived;
                      pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.CellID = pStat->ReceptionData.CellId;
                      pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.NetworkID = pStat->ReceptionData.netId;
					  if(pStat->ReceptionData.liteMode)
					     pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.T2LiteActive = TRUE;
					  else
						 pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.T2LiteActive = FALSE;
					  // fix-me, the fftMode value maybe need to double check !
                      if((pStat->ReceptionData.fftMode & 0x0E) == 0)
                         pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.TransmissionMode = FFT_MODE_2K;
					  else
                      if((pStat->ReceptionData.fftMode & 0x0E) == 2)
                         pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.TransmissionMode = FFT_MODE_8K;
					  else
                      if((pStat->ReceptionData.fftMode & 0x0E) == 4)
                         pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.TransmissionMode = FFT_MODE_4K;
					  else
                      if((pStat->ReceptionData.fftMode & 0x0E) == 6)
					  {
						 if(pStat->ReceptionData.liteMode)
                            pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.TransmissionMode = FFT_MODE_16K;
						 else
                            pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.TransmissionMode = FFT_MODE_1K;
					  }
					  else
                      if((pStat->ReceptionData.fftMode & 0x0E) == 8)
                         pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.TransmissionMode = FFT_MODE_16K;
					  else
                      if((pStat->ReceptionData.fftMode & 0x0E) == 10)
					  {
						 if(pStat->ReceptionData.liteMode)
						    pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.TransmissionMode = FFT_MODE_RESERVED;
						 else
                            pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.TransmissionMode = FFT_MODE_32K;
					  }
					  else
                      if((pStat->ReceptionData.fftMode & 0x0E) == 12)
                         pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.TransmissionMode = FFT_MODE_8K;
					  else
                      if((pStat->ReceptionData.fftMode & 0x0E) == 14)
					  {
						 if(pStat->ReceptionData.liteMode)
						    pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.TransmissionMode = FFT_MODE_RESERVED;
						 else
                            pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.TransmissionMode = FFT_MODE_32K;
					  }
					  else
					     pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.TransmissionMode = UNKNOWN_FFT_MODE;					
					  if(pStat->ReceptionData.guardInterval == 0)
						 pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.GuardInterval = GUARD_INTERVAL_1_32;
					  else
					  if(pStat->ReceptionData.guardInterval == 1)
						 pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.GuardInterval = GUARD_INTERVAL_1_16;
					  else
					  if(pStat->ReceptionData.guardInterval == 2)
						 pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.GuardInterval = GUARD_INTERVAL_1_8;
					  else
					  if(pStat->ReceptionData.guardInterval == 3)
						 pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.GuardInterval = GUARD_INTERVAL_1_4;
					  else
					  if(pStat->ReceptionData.guardInterval == 4)
						 pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.GuardInterval = GUARD_INTERVAL_1_128;
					  else
					  if(pStat->ReceptionData.guardInterval == 5)
						 pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.GuardInterval = GUARD_INTERVAL_19_128;
					  else
					  if(pStat->ReceptionData.guardInterval == 6)
						 pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.GuardInterval = GUARD_INTERVAL_19_256;
					  else
						 pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.GuardInterval = UNKNOWN_GUARD_INTERVAL;
                      if(pStat->ReceptionData.pilotPattern == 1)
                         pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PilotPattern = PILOT_PATTERN_1;
					  else
                      if(pStat->ReceptionData.pilotPattern == 2)
                         pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PilotPattern = PILOT_PATTERN_2;
					  else
                      if(pStat->ReceptionData.pilotPattern == 3)
                         pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PilotPattern = PILOT_PATTERN_3;
					  else
                      if(pStat->ReceptionData.pilotPattern == 4)
                         pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PilotPattern = PILOT_PATTERN_4;
					  else
                      if(pStat->ReceptionData.pilotPattern == 5)
                         pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PilotPattern = PILOT_PATTERN_5;
					  else
                      if(pStat->ReceptionData.pilotPattern == 6)
                         pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PilotPattern = PILOT_PATTERN_6;
					  else
                      if(pStat->ReceptionData.pilotPattern == 7)
                         pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PilotPattern = PILOT_PATTERN_7;
					  else
                      if(pStat->ReceptionData.pilotPattern == 8)
                         pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PilotPattern = PILOT_PATTERN_8;
					  else
                         pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PilotPattern = UNKNOWN_PILOT_PATTERN;
					  pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformationAvailableFlag = TRUE;
                      pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.NumberOfPlps = MaxPlpData;
					  for(i=0;i<MaxPlpData;i++)
					  {
                          pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].PlpID = pStat->PlpData[i].plpStatistics.plpId;
                          pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].PlpGroupID = pStat->PlpData[i].plpStatistics.plpGroupId;
						  if(pStat->PlpData[i].plpStatistics.plpType == 0)
							 pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].PlpType = COMMON_PLP;
						  else
						  if(pStat->PlpData[i].plpStatistics.plpType == 1)
							 pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].PlpType = DATA_TYPE1_PLP;
						  else
						  if(pStat->PlpData[i].plpStatistics.plpType == 2)
							 pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].PlpType = DATA_TYPE2_PLP;
						  else
							 pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].PlpType = UNKNOWN_PLP_TYPE;
                          if(pStat->PlpData[i].plpStatistics.plpMode == 0)
                             pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].PlpMode = PLP_NOT_SPECIFIED_MODE;
                          else
                          if(pStat->PlpData[i].plpStatistics.plpMode == 1)
                             pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].PlpMode = PLP_NORMAL_EFFICIENCY_MODE;
                          else
                          if(pStat->PlpData[i].plpStatistics.plpMode == 2)
                             pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].PlpMode = PLP_HIGH_EFFICIENCY_MODE;
                          else
                          if(pStat->PlpData[i].plpStatistics.plpMode == 3)
                             pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].PlpMode = PLP_RESERVED_MODE;
                          else
							 pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].PlpMode = UNKNOWN_PLP_MODE;
                          if(pStat->PlpData[i].plpStatistics.plpFecType == 0)
							 pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].FecFrameType = FEC_FRAME_16K_LDPC;
						  else
                          if(pStat->PlpData[i].plpStatistics.plpFecType == 1)
							 pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].FecFrameType = FEC_FRAME_64K_LDPC;
						  else
							 pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].FecFrameType = UNKNOWN_FEC_FRAME;

						  if(pStat->PlpData[i].plpStatistics.plpCod == 0)
                             pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].CodeRate = CODE_RATE_1_2;
						  else
						  if(pStat->PlpData[i].plpStatistics.plpCod == 1)
                             pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].CodeRate = CODE_RATE_3_5;
						  else
						  if(pStat->PlpData[i].plpStatistics.plpCod == 2)
                             pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].CodeRate = CODE_RATE_2_3;
						  else
						  if(pStat->PlpData[i].plpStatistics.plpCod == 3)
                             pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].CodeRate = CODE_RATE_3_4;
						  else
						  if(pStat->PlpData[i].plpStatistics.plpCod == 4)
                             pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].CodeRate = CODE_RATE_4_5;
						  else
						  if(pStat->PlpData[i].plpStatistics.plpCod == 5)
                             pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].CodeRate = CODE_RATE_5_6;
						  else
						  if(pStat->PlpData[i].plpStatistics.plpCod == 6)
                             pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].CodeRate = CODE_RATE_1_3;
						  else
						  if(pStat->PlpData[i].plpStatistics.plpCod == 7)
                             pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].CodeRate = CODE_RATE_2_5;
						  else
                             pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].CodeRate = UNKNOWN_CODE_RATE;
                          if(pStat->PlpData[i].plpStatistics.plpMod == 0)
                             pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].Constellation = CONSTELLATION_QPSK;
						  else
                          if(pStat->PlpData[i].plpStatistics.plpMod == 1)
                             pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].Constellation = CONSTELLATION_16_QAM;
						  else
                          if(pStat->PlpData[i].plpStatistics.plpMod == 2)
                             pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].Constellation = CONSTELLATION_64_QAM;
						  else
                          if(pStat->PlpData[i].plpStatistics.plpMod == 3)
                             pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].Constellation = CONSTELLATION_256_QAM;
						  else
                             pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].Constellation = UNKNOWN_CONSTELLATION;
                          if(pStat->PlpData[i].plpStatistics.plpRotation)
							 pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].PlpRotationEnable = TRUE;
						  else
							 pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].PlpRotationEnable = FALSE;
                          pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].PlpNumberBlocksMax = pStat->PlpData[i].plpStatistics.plpNumBlocksMax;
                          pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].FrameInterval = pStat->PlpData[i].plpStatistics.frameInterval;
						  pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].TimeIlLength = pStat->PlpData[i].plpStatistics.timeIlLength;
						  pStatistics->DTV_STATISTIC.DVBT2StatisticInfo.PlpInformation.PlpStatistic[i].TimeIlType = pStat->PlpData[i].plpStatistics.timeIlType;
					  }
				   }

	         	   SMS4470StatisticPrintf("********************************************************************************\n");
	         	   SMS4470StatisticPrintf("********************************************************************************\n");
	         	   SMS4470StatisticPrintf("********************************************************************************\n");
				}
			 }
		  }
	      else
		  {
             SMS4470StatisticPrintf("********************************************************************************\n");
             SMS4470StatisticPrintf("*************************     SMS4470 No Signal !     **************************\n");
             SMS4470StatisticPrintf("********************************************************************************\n");
		  }	  
       }
	   else
	   {
          SMS4470StatisticPrintf("(SMS4470_check_signal) pRespondseMsg->msgData[0] != 0\n");

	   }
	}
	else
	if((Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == DVBT_DVBT2_DUAL_WORKING_MODE && Sms4470DeviceGroupInformation[Index].SMS4470CurrentDemodulationModeForDvbtDvbt2DualWorkingMode == SMS4470_DVBT_DEMODULATION_MODE) ||
	   Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == DVBT_WORKING_MODE)
	{ 
	   // for DVB-T
       for(i=0 ; i<SpecialCommandWaitResponseLoop ; i++)
       {
           SMS4470SysDelay(10);
		   ResponseResult = SMS4470_read_response(FrontendGroupID,MSG_SMS_I2C_SHORT_STAT_IND, Sms4470DeviceGroupInformation[Index].Sms4470MessageRxBuffer, SIANO_IIC_RX_BUFFER_SIZE);
           if(ResponseResult == 0xffff) 
		   {
              break;
		   }
		   else
		   if(ResponseResult == 0xf000)
		   {
              i = SpecialCommandWaitResponseLoop;
		      break;
		   }
       }
    
       if(i == SpecialCommandWaitResponseLoop)
       {
	      SMS4470ErrorPrintf("(SMS4470_GetReceptionStatistics) xxxxxxxxx Get response failed !\n");

          return TRUE;
       }

       Short_Statistics_ST* pStat;
        
       pRespondseMsg = (SmsMsgData_ST*)Sms4470DeviceGroupInformation[Index].Sms4470MessageRxBuffer;
        
       pStat = (Short_Statistics_ST*)&pRespondseMsg->msgData[0];

       if(pStat->IsDemodLocked == 0x01)
	   {   
		  pStatistics->IsSignalLocked = TRUE;

          SMS4470StatisticPrintf("********************************************************************************\n");
          SMS4470StatisticPrintf("***********************     SMS4470 Signal Locked !     ************************\n");
          SMS4470StatisticPrintf("********************************************************************************\n");
          SMS4470StatisticPrintf("************************* Statistics Data Information **************************\n");
          SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) IsModemLocked : 0x%x\n",pStat->IsDemodLocked);
          SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) InBandPwr : %d\n",pStat->InBandPwr);
          SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) BER : %d\n",pStat->BER);
          SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) SNR : %d\n",pStat->SNR);
          SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) TotalTSPackets : %d\n",pStat->TotalTSPackets);
          SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) ErrorTSPackets : %d\n",pStat->ErrorTSPackets);

		  if(Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTReceptionStatistic.IsDemodLocked)
		  {
             SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) DVBTReceptionStatistic.IsRfLocked : 0x%x\n",Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTReceptionStatistic.IsRfLocked);
             SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) DVBTReceptionStatistic.IsDemodLocked : 0x%x\n",Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTReceptionStatistic.IsDemodLocked);

             SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) DVBTReceptionStatistic.ModemState : 0x%x\n",Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTReceptionStatistic.ModemState);
             SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) DVBTReceptionStatistic.SNR : %d\n",Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTReceptionStatistic.SNR);
             SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) DVBTReceptionStatistic.BER : %d\n",Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTReceptionStatistic.BER);
             SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) DVBTReceptionStatistic.BERErrorCount : %d\n",Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTReceptionStatistic.BERErrorCount);
             SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) DVBTReceptionStatistic.BERBitCount : %d\n",Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTReceptionStatistic.BERBitCount);
             SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) DVBTReceptionStatistic.TS_PER : %d\n",Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTReceptionStatistic.TS_PER);
			 SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) DVBTReceptionStatistic.MFER : %d\n",Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTReceptionStatistic.MFER);
             SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) DVBTReceptionStatistic.RSSI : %d\n",Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTReceptionStatistic.RSSI);
             SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) DVBTReceptionStatistic.InBandPwr : %d\n",Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTReceptionStatistic.InBandPwr);
             SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) DVBTReceptionStatistic.CarrierOffset : %d\n",Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTReceptionStatistic.CarrierOffset);
             SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) DVBTReceptionStatistic.ErrorTSPackets : %d\n",Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTReceptionStatistic.ErrorTSPackets);
             SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) DVBTReceptionStatistic.TotalTSPackets : %d\n",Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTReceptionStatistic.TotalTSPackets);

             SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) DVBTReceptionStatistic.RefDevPPM : %d\n",Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTReceptionStatistic.RefDevPPM);
             SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) DVBTReceptionStatistic.FreqDevHz : %d\n",Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTReceptionStatistic.FreqDevHz);

             SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) DVBTReceptionStatistic.MRC_SNR : %d\n",Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTReceptionStatistic.MRC_SNR);
             SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) DVBTReceptionStatistic.MRC_RSSI : %d\n",Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTReceptionStatistic.MRC_RSSI);
             SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) DVBTReceptionStatistic.MRC_InBandPwr : %d\n",Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTReceptionStatistic.MRC_InBandPwr);

		     if(Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.IsDemodLocked)
			 {
                SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) DVBTTransmissionStatistic.Frequency : %d\n",Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.Frequency);
                SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) DVBTTransmissionStatistic.Bandwidth : %d\n",Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.Bandwidth);
                SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) DVBTTransmissionStatistic.TransmissionMode : 0x%x\n",Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.TransmissionMode);
                SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) DVBTTransmissionStatistic.GuardInterval : 0x%x\n",Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.GuardInterval);
                SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) DVBTTransmissionStatistic.CodeRate : 0x%x\n",Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.CodeRate);
                SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) DVBTTransmissionStatistic.LPCodeRate : 0x%x\n",Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.LPCodeRate);
                SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) DVBTTransmissionStatistic.Hierarchy : 0x%x\n",Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.Hierarchy);
                SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) DVBTTransmissionStatistic.Constellation : 0x%x\n",Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.Constellation);
	            // DVB-H TPS parameters
                SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) DVBTTransmissionStatistic.CellId : 0x%x\n",Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.CellId);
                SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) DVBTTransmissionStatistic.DvbhSrvIndHP : 0x%x\n",Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.DvbhSrvIndHP);
                SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) DVBTTransmissionStatistic.DvbhSrvIndLP : 0x%x\n",Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.DvbhSrvIndLP);
                SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) DVBTTransmissionStatistic.IsDemodLocked : 0x%x\n",Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.IsDemodLocked);

                pStatistics->StatisticAvailableFlag = TRUE;
#if 1
                pStatistics->DTV_STATISTIC.DVBTStatisticInfo.SNR = pStat->SNR;
                pStatistics->DTV_STATISTIC.DVBTStatisticInfo.InBandPower = pStat->InBandPwr;
                pStatistics->DTV_STATISTIC.DVBTStatisticInfo.ErrorTSPackets = pStat->ErrorTSPackets;
                pStatistics->DTV_STATISTIC.DVBTStatisticInfo.TotalTSPackets = pStat->TotalTSPackets; 
#else
                pStatistics->DTV_STATISTIC.DVBTStatisticInfo.SNR = Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTReceptionStatistic.SNR;
                pStatistics->DTV_STATISTIC.DVBTStatisticInfo.InBandPower = Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTReceptionStatistic.InBandPwr;
                pStatistics->DTV_STATISTIC.DVBTStatisticInfo.ErrorTSPackets = Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTReceptionStatistic.ErrorTSPackets;
                pStatistics->DTV_STATISTIC.DVBTStatisticInfo.TotalTSPackets = Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTReceptionStatistic.TotalTSPackets; 
#endif
			    if(Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.TransmissionMode == 0)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.TransmissionMode = FFT_MODE_2K;
			    else
			    if(Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.TransmissionMode == 1)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.TransmissionMode = FFT_MODE_8K;
			    else
			    if(Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.TransmissionMode == 2)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.TransmissionMode = FFT_MODE_4K;
			    else
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.TransmissionMode = UNKNOWN_FFT_MODE;
			    if(Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.GuardInterval == 3)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.GuardInterval = GUARD_INTERVAL_1_4;
			    else
			    if(Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.GuardInterval == 2)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.GuardInterval = GUARD_INTERVAL_1_8;
			    else
			    if(Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.GuardInterval == 1)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.GuardInterval = GUARD_INTERVAL_1_16;
			    else
			    if(Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.GuardInterval == 0)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.GuardInterval = GUARD_INTERVAL_1_32;
			    else
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.GuardInterval = UNKNOWN_GUARD_INTERVAL;
			    if(Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.CodeRate == 0)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.CodeRate = CODE_RATE_1_2;
			    else
			    if(Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.CodeRate == 1)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.CodeRate = CODE_RATE_2_3;
			    else
			    if(Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.CodeRate == 2)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.CodeRate = CODE_RATE_3_4;
			    else
			    if(Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.CodeRate == 3)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.CodeRate = CODE_RATE_5_6;
			    else
			    if(Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.CodeRate == 4)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.CodeRate = CODE_RATE_7_8;
			    else
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.CodeRate = UNKNOWN_CODE_RATE;
			    if(Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.LPCodeRate == 0)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.LPCodeRate = CODE_RATE_1_2;
			    else
			    if(Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.LPCodeRate == 1)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.LPCodeRate = CODE_RATE_2_3;
			    else
			    if(Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.LPCodeRate == 2)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.LPCodeRate = CODE_RATE_3_4;
			    else
			    if(Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.LPCodeRate == 3)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.LPCodeRate = CODE_RATE_5_6;
			    else
			    if(Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.LPCodeRate == 4)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.LPCodeRate = CODE_RATE_7_8;
			    else
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.LPCodeRate = UNKNOWN_CODE_RATE;
			    if(Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.Constellation == 0)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.Constellation = CONSTELLATION_QPSK;
			    else
			    if(Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.Constellation == 1)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.Constellation = CONSTELLATION_16_QAM;
			    else
			    if(Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.Constellation == 2)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.Constellation = CONSTELLATION_64_QAM;
			    else
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.Constellation = UNKNOWN_CONSTELLATION;
			    if(Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.Hierarchy == 0)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.Hierarchical = HIERARCHY_NONE;
			    else
			    if(Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.Hierarchy == 1)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.Hierarchical = HIERARCHY_ALPHA_1;
			    else
			    if(Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.Hierarchy == 2)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.Hierarchical = HIERARCHY_ALPHA_2;
			    else
			    if(Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.Hierarchy == 2)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.Hierarchical = HIERARCHY_ALPHA_4;
			    else
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.Hierarchical = UNKNOWN_HIERARCHY;
                pStatistics->DTV_STATISTIC.DVBTStatisticInfo.DVBH_FrameErrorRate = Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTReceptionStatistic.MFER;
                pStatistics->DTV_STATISTIC.DVBTStatisticInfo.DVBHCellID = Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.CellId;
	            if((Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.DvbhSrvIndHP & 0x02) == 0x02)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.DVBH_TimeSlicing_HP = TRUE;
				else
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.DVBH_TimeSlicing_HP = FALSE;
	            if((Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.DvbhSrvIndHP & 0x01) == 0x01)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.DVBH_MPE_FEC_HP = TRUE;
				else
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.DVBH_MPE_FEC_HP = FALSE;
	            if((Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.DvbhSrvIndLP & 0x02) == 0x02)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.DVBH_TimeSlicing_LP = TRUE;
				else
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.DVBH_TimeSlicing_LP = FALSE;
	            if((Sms4470DeviceGroupInformation[Index].SMS4470CurrentRespondentDVBTTransmissionStatistic.DvbhSrvIndLP & 0x01) == 0x01)
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.DVBH_MPE_FEC_LP = TRUE;
				else
                   pStatistics->DTV_STATISTIC.DVBTStatisticInfo.DVBH_MPE_FEC_LP = FALSE;
		     }
		  }
	   }
	   else
	   {
          SMS4470StatisticPrintf("********************************************************************************\n");
          SMS4470StatisticPrintf("*************************     SMS4470 No Signal !     **************************\n");
          SMS4470StatisticPrintf("********************************************************************************\n");
	   }
    }
	else
	{
	   // for ISDB-T
       SmsMsg.xMsgHeader.msgType   = MSG_SMS_GET_STATISTICS_EX_REQ;
       SmsMsg.xMsgHeader.msgSrcId  = SMS_HOST_LIB; // DVBT_BDA_CONTROL_MSG_ID; // 0;
       SmsMsg.xMsgHeader.msgDstId  = HIF_TASK;                
       SmsMsg.xMsgHeader.msgLength = sizeof(SmsMsg);
       SmsMsg.xMsgHeader.msgFlags  = MSG_HDR_FLAG_STATIC_MSG; // 0;

       TempMsgLength = SmsMsg.xMsgHeader.msgLength;
#ifdef SMS_BIGENDIAN
       SmsMsg.xMsgHeader.msgType     = SMS_ENDIAN_SWAP16(SmsMsg.xMsgHeader.msgType);
       SmsMsg.xMsgHeader.msgLength   = SMS_ENDIAN_SWAP16(SmsMsg.xMsgHeader.msgLength);
       SmsMsg.xMsgHeader.msgFlags    = SMS_ENDIAN_SWAP16(SmsMsg.xMsgHeader.msgFlags);
#endif
	   
//       SMS4470SysDelay(5); 
	   SMS4470I2CWrite(FrontendGroupID,(UINT8*)&SmsMsg,TempMsgLength);
   
       for(i=0 ; i<SpecialCommandWaitResponseLoop ; i++)
	   {
           SMS4470SysDelay(10);
		   ResponseResult = SMS4470_read_response(FrontendGroupID,MSG_SMS_GET_STATISTICS_EX_RES, Sms4470DeviceGroupInformation[Index].Sms4470MessageRxBuffer, SIANO_IIC_RX_BUFFER_SIZE);
           if(ResponseResult == 0xffff)
		   {
              break;
		   }
		   else
		   if(ResponseResult == 0xf000)
		   {
              i = SpecialCommandWaitResponseLoop;
		      break;
		   }
	   }
    
       if(i == SpecialCommandWaitResponseLoop)
	   {
	      SMS4470ErrorPrintf("(SMS4470_GetReceptionStatistics) xxxxxxxxx Get response failed !\n");

          return FALSE;
	   }

       SMSHOSTLIB_STATISTICS_ISDBT_ST* pStat;    
     
       pRespondseMsg = (SmsMsgData_ST*)Sms4470DeviceGroupInformation[Index].Sms4470MessageRxBuffer;

       if(pRespondseMsg->msgData[0] == 0)
	   {          
          pStat = (SMSHOSTLIB_STATISTICS_ISDBT_ST*)&pRespondseMsg->msgData[1];
        
	      if(pStat->IsDemodLocked == 0x01)
	      {
			 pStatistics->IsSignalLocked = TRUE;

             SMS4470StatisticPrintf("********************************************************************************\n");
             SMS4470StatisticPrintf("***********************     SMS4470 Signal Locked !    *************************\n");
             SMS4470StatisticPrintf("********************************************************************************\n");
             SMS4470StatisticPrintf("************************* Statistics Data Information **************************\n");

	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) StatisticsType : 0x%x\n",pStat->StatisticsType);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) FullSize : %d\n",pStat->FullSize);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) IsRfLocked : 0x%x\n",pStat->IsRfLocked);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) IsDemodLocked : 0x%x\n",pStat->IsDemodLocked);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) IsExternalLNAOn : 0x%x\n",pStat->IsExternalLNAOn);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) SNR : %d\n",pStat->SNR);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) RSSI : %d\n",pStat->RSSI);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) InBandPwr : %d\n",pStat->InBandPwr);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) CarrierOffset : %d\n",pStat->CarrierOffset);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) Frequency : %d\n",pStat->Frequency);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) Bandwidth : %d\n",pStat->Bandwidth);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) TransmissionMode : 0x%x\n",pStat->TransmissionMode);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) ModemState : 0x%x\n",pStat->ModemState);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) GuardInterval : 0x%x\n",pStat->GuardInterval);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) SystemType : 0x%x\n",pStat->SystemType);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) PartialReception : 0x%x\n",pStat->PartialReception);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) NumOfLayers : %d\n",pStat->NumOfLayers);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) SegmentNumber : %d\n",pStat->SegmentNumber);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) TuneBW : 0x%x\n",pStat->TuneBW);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) LayerInfo[0].CodeRate 0x%x\n",pStat->LayerInfo[0].CodeRate);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) LayerInfo[0].Constellation 0x%x\n",pStat->LayerInfo[0].Constellation);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) LayerInfo[0].BER %d\n",pStat->LayerInfo[0].BER);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) LayerInfo[0].BERErrorCount %d\n",pStat->LayerInfo[0].BERErrorCount);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) LayerInfo[0].BERBitCount %d\n",pStat->LayerInfo[0].BERBitCount);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) LayerInfo[0].PreBER %d\n",pStat->LayerInfo[0].PreBER);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) LayerInfo[0].TS_PER %d\n",pStat->LayerInfo[0].TS_PER);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) LayerInfo[0].ErrorTSPackets %d\n",pStat->LayerInfo[0].ErrorTSPackets);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) LayerInfo[0].TotalTSPackets %d\n",pStat->LayerInfo[0].TotalTSPackets);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) LayerInfo[0].TILdepthI 0x%x\n",pStat->LayerInfo[0].TILdepthI);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) LayerInfo[0].NumberOfSegments %d\n",pStat->LayerInfo[0].NumberOfSegments);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) LayerInfo[0].TMCCErrors 0x%x\n",pStat->LayerInfo[0].TMCCErrors);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) LayerInfo[1].CodeRate 0x%x\n",pStat->LayerInfo[1].CodeRate);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) LayerInfo[1].Constellation 0x%x\n",pStat->LayerInfo[1].Constellation);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) LayerInfo[1].BER %d\n",pStat->LayerInfo[1].BER);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) LayerInfo[1].BERErrorCount %d\n",pStat->LayerInfo[1].BERErrorCount);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) LayerInfo[1].BERBitCount %d\n",pStat->LayerInfo[1].BERBitCount);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) LayerInfo[1].PreBER %d\n",pStat->LayerInfo[1].PreBER);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) LayerInfo[1].TS_PER %d\n",pStat->LayerInfo[1].TS_PER);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) LayerInfo[1].ErrorTSPackets %d\n",pStat->LayerInfo[1].ErrorTSPackets);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) LayerInfo[1].TotalTSPackets %d\n",pStat->LayerInfo[1].TotalTSPackets);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) LayerInfo[1].TILdepthI 0x%x\n",pStat->LayerInfo[1].TILdepthI);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) LayerInfo[1].NumberOfSegments %d\n",pStat->LayerInfo[1].NumberOfSegments);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) LayerInfo[1].TMCCErrors 0x%x\n",pStat->LayerInfo[1].TMCCErrors);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) LayerInfo[2].CodeRate 0x%x\n",pStat->LayerInfo[2].CodeRate);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) LayerInfo[2].Constellation 0x%x\n",pStat->LayerInfo[2].Constellation);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) LayerInfo[2].BER %d\n",pStat->LayerInfo[2].BER);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) LayerInfo[2].BERErrorCount %d\n",pStat->LayerInfo[2].BERErrorCount);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) LayerInfo[2].BERBitCount %d\n",pStat->LayerInfo[2].BERBitCount);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) LayerInfo[2].PreBER %d\n",pStat->LayerInfo[2].PreBER);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) LayerInfo[2].TS_PER %d\n",pStat->LayerInfo[2].TS_PER);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) LayerInfo[2].ErrorTSPackets %d\n",pStat->LayerInfo[2].ErrorTSPackets);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) LayerInfo[2].TotalTSPackets %d\n",pStat->LayerInfo[2].TotalTSPackets);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) LayerInfo[2].TILdepthI 0x%x\n",pStat->LayerInfo[2].TILdepthI);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) LayerInfo[2].NumberOfSegments %d\n",pStat->LayerInfo[2].NumberOfSegments);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) LayerInfo[2].TMCCErrors 0x%x\n",pStat->LayerInfo[2].TMCCErrors);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) ExtAntenna : 0x%x\n",pStat->ExtAntenna);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) ReceptionQuality : %d\n",pStat->ReceptionQuality);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) EwsAlertActive : 0x%x\n",pStat->EwsAlertActive);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) LNAOnOff : 0x%x\n",pStat->LNAOnOff);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) RfAgcLevel : %d\n",pStat->RfAgcLevel);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) BbAgcLevel : %d\n",pStat->BbAgcLevel);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) FwErrorsCounter : %d\n",pStat->FwErrorsCounter);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) MRC_SNR : %d\n",pStat->MRC_SNR);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) SNRFullRes : %d\n",pStat->SNRFullRes);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) layer_in_hier1 : 0x%x\n",pStat->layer_in_hier1);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) layer_in_hier2 : 0x%x\n",pStat->layer_in_hier2);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) MRC_InBandPwr : %d\n",pStat->MRC_InBandPwr);
	         SMS4470StatisticPrintf("(SMS4470_GetReceptionStatistics) MRC_Rssi : %d\n",pStat->MRC_Rssi);

             pStatistics->StatisticAvailableFlag = TRUE;
             pStatistics->DTV_STATISTIC.ISDBTStatisticInfo.SNR = pStat->MRC_SNR;
             pStatistics->DTV_STATISTIC.ISDBTStatisticInfo.InBandPower = pStat->MRC_InBandPwr;
			 if(pStat->PartialReception)
                pStatistics->DTV_STATISTIC.ISDBTStatisticInfo.PartialReception = TRUE;
			 else
                pStatistics->DTV_STATISTIC.ISDBTStatisticInfo.PartialReception = FALSE;
			 if(pStat->EwsAlertActive)
                pStatistics->DTV_STATISTIC.ISDBTStatisticInfo.EWSAlertActive = TRUE;
			 else
                pStatistics->DTV_STATISTIC.ISDBTStatisticInfo.EWSAlertActive = FALSE;
			 if(pStat->TransmissionMode == 1)
				pStatistics->DTV_STATISTIC.ISDBTStatisticInfo.TransmissionMode = TRANSMISSION_MODE_1;
			 else
			 if(pStat->TransmissionMode == 2)
				pStatistics->DTV_STATISTIC.ISDBTStatisticInfo.TransmissionMode = TRANSMISSION_MODE_2;
			 else
			 if(pStat->TransmissionMode == 3)
				pStatistics->DTV_STATISTIC.ISDBTStatisticInfo.TransmissionMode = TRANSMISSION_MODE_3;
			 else
                pStatistics->DTV_STATISTIC.ISDBTStatisticInfo.TransmissionMode = UNKNOWN_TRANSMISSION_MODE;
             if(pStat->GuardInterval == 4)
				pStatistics->DTV_STATISTIC.ISDBTStatisticInfo.GuardInterval = GUARD_INTERVAL_1_4;
			 else
             if(pStat->GuardInterval == 8)
				pStatistics->DTV_STATISTIC.ISDBTStatisticInfo.GuardInterval = GUARD_INTERVAL_1_8;
			 else
             if(pStat->GuardInterval == 16)
				pStatistics->DTV_STATISTIC.ISDBTStatisticInfo.GuardInterval = GUARD_INTERVAL_1_16;
			 else
             if(pStat->GuardInterval == 32)
				pStatistics->DTV_STATISTIC.ISDBTStatisticInfo.GuardInterval = GUARD_INTERVAL_1_32;
			 else
				pStatistics->DTV_STATISTIC.ISDBTStatisticInfo.GuardInterval = UNKNOWN_GUARD_INTERVAL;
             for(i=0;i<3;i++)
			 {
                 pStatistics->DTV_STATISTIC.ISDBTStatisticInfo.LayerStatistic[i].NumberOfSegments = pStat->LayerInfo[i].NumberOfSegments;
                 pStatistics->DTV_STATISTIC.ISDBTStatisticInfo.LayerStatistic[i].ErrorTSPackets = pStat->LayerInfo[i].ErrorTSPackets; 
                 pStatistics->DTV_STATISTIC.ISDBTStatisticInfo.LayerStatistic[i].TotalTSPackets = pStat->LayerInfo[i].TotalTSPackets;
	             if(pStat->LayerInfo[i].CodeRate == 0)
                    pStatistics->DTV_STATISTIC.ISDBTStatisticInfo.LayerStatistic[i].CodeRate = CODE_RATE_1_2;
				 else
	             if(pStat->LayerInfo[i].CodeRate == 1)
                    pStatistics->DTV_STATISTIC.ISDBTStatisticInfo.LayerStatistic[i].CodeRate = CODE_RATE_2_3;
				 else
	             if(pStat->LayerInfo[i].CodeRate == 2)
                    pStatistics->DTV_STATISTIC.ISDBTStatisticInfo.LayerStatistic[i].CodeRate = CODE_RATE_3_4;
				 else
	             if(pStat->LayerInfo[i].CodeRate == 3)
                    pStatistics->DTV_STATISTIC.ISDBTStatisticInfo.LayerStatistic[i].CodeRate = CODE_RATE_5_6;
				 else
	             if(pStat->LayerInfo[i].CodeRate == 4)
                    pStatistics->DTV_STATISTIC.ISDBTStatisticInfo.LayerStatistic[i].CodeRate = CODE_RATE_7_8;
				 else
                    pStatistics->DTV_STATISTIC.ISDBTStatisticInfo.LayerStatistic[i].CodeRate = UNKNOWN_CODE_RATE;
	             if(pStat->LayerInfo[i].Constellation == 0)
                    pStatistics->DTV_STATISTIC.ISDBTStatisticInfo.LayerStatistic[i].Constellation = CONSTELLATION_QPSK;
				 else
	             if(pStat->LayerInfo[i].Constellation == 1)
                    pStatistics->DTV_STATISTIC.ISDBTStatisticInfo.LayerStatistic[i].Constellation = CONSTELLATION_16_QAM;
				 else
	             if(pStat->LayerInfo[i].Constellation == 2)
                    pStatistics->DTV_STATISTIC.ISDBTStatisticInfo.LayerStatistic[i].Constellation = CONSTELLATION_64_QAM;
				 else
                    pStatistics->DTV_STATISTIC.ISDBTStatisticInfo.LayerStatistic[i].Constellation = UNKNOWN_CONSTELLATION;
			     if(pStat->LayerInfo[i].TILdepthI == 0)
                    pStatistics->DTV_STATISTIC.ISDBTStatisticInfo.LayerStatistic[i].TimeInterleaver = TIME_INTERLEAVER_0;
				 else
			     if(pStat->LayerInfo[i].TILdepthI == 1)
                    pStatistics->DTV_STATISTIC.ISDBTStatisticInfo.LayerStatistic[i].TimeInterleaver = TIME_INTERLEAVER_1;
				 else
			     if(pStat->LayerInfo[i].TILdepthI == 2)
                    pStatistics->DTV_STATISTIC.ISDBTStatisticInfo.LayerStatistic[i].TimeInterleaver = TIME_INTERLEAVER_2;
				 else
			     if(pStat->LayerInfo[i].TILdepthI == 4)
                    pStatistics->DTV_STATISTIC.ISDBTStatisticInfo.LayerStatistic[i].TimeInterleaver = TIME_INTERLEAVER_4;
				 else
                    pStatistics->DTV_STATISTIC.ISDBTStatisticInfo.LayerStatistic[i].TimeInterleaver = UNKNOWN_TIME_INTERLEAVER;
			 }
	      }
	      else
	      {
             SMS4470StatisticPrintf("********************************************************************************\n");
             SMS4470StatisticPrintf("*************************     SMS4470 No Signal !     **************************\n");
             SMS4470StatisticPrintf("********************************************************************************\n");
	      }	    
       }
       else
   	   {
          SMS4470StatisticPrintf("(SMS4470_check_signal) pRespondseMsg->msgData[0] != 0\n");
	   }
	}

	return TRUE;
}
//**********************************************************************************************************
//**********************************************************************************************************
//**********************************************************************************************************
BOOL SMS4470_CheckDVBT2PhysicalLayerPipeInformation(ULONG FrontendGroupID,BOOL* pDoesPhysicalLayerPipeInformationExistFlag)
{
	ULONG           Index;
    UINT32          i;
	UINT32          ResponseResult;
	SmsMsgData_ST   SmsMsg = {{0}};
    SmsMsgData_ST*  pRespondseMsg;
	BOOL            CheckFlag;
	UINT16          TempMsgLength;
    
    SMS4470FunctionNamePrintf("(SMS4470_CheckDVBT2PhysicalLayerPipeInformation)\n");

	if(FrontendGroupID == FRONTEND_TWO_DIVERSITY_GROUP_ID)
	   Index = 0;
	else
	   Index = 1;

    *pDoesPhysicalLayerPipeInformationExistFlag = FALSE;

	if((Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == DVBT_DVBT2_DUAL_WORKING_MODE && Sms4470DeviceGroupInformation[Index].SMS4470CurrentDemodulationModeForDvbtDvbt2DualWorkingMode == SMS4470_DVBT2_DEMODULATION_MODE) ||
	   Sms4470DeviceGroupInformation[Index].Sms4470CurrentWorkingMode == DVBT2_WORKING_MODE)
	{
       SmsMsg.xMsgHeader.msgType   = MSG_SMS_GET_STATISTICS_EX_REQ;
       SmsMsg.xMsgHeader.msgSrcId  = SMS_HOST_LIB; // DVBT_BDA_CONTROL_MSG_ID; // 0;
       SmsMsg.xMsgHeader.msgDstId  = HIF_TASK;                // Jacky Han Modified For Diversity Function
       SmsMsg.xMsgHeader.msgLength = sizeof(SmsMsg);
       SmsMsg.xMsgHeader.msgFlags  = MSG_HDR_FLAG_STATIC_MSG; // 0;

   
SMS4470DebugPrintf("(SMS4470_CheckDVBT2PhysicalLayerPipeInformation) Send command (MSG_SMS_GET_STATISTICS_EX_REQ) to SMS4470\n");

       TempMsgLength = SmsMsg.xMsgHeader.msgLength;
#ifdef SMS_BIGENDIAN
       SmsMsg.xMsgHeader.msgType     = SMS_ENDIAN_SWAP16(SmsMsg.xMsgHeader.msgType);
       SmsMsg.xMsgHeader.msgLength   = SMS_ENDIAN_SWAP16(SmsMsg.xMsgHeader.msgLength);
       SmsMsg.xMsgHeader.msgFlags    = SMS_ENDIAN_SWAP16(SmsMsg.xMsgHeader.msgFlags);
#endif

//       SMS4470SysDelay(5); 
	   SMS4470I2CWrite(FrontendGroupID,(UINT8*)&SmsMsg,TempMsgLength);
    
	
SMS4470DebugPrintf("(SMS4470_CheckDVBT2PhysicalLayerPipeInformation) Get respence from SMS4470\n");
    
       for(i=0 ; i<SpecialCommandWaitResponseLoop ; i++)
	   {
           SMS4470SysDelay(10);
		   ResponseResult = SMS4470_read_response(FrontendGroupID,MSG_SMS_GET_STATISTICS_EX_RES, Sms4470DeviceGroupInformation[Index].Sms4470MessageRxBuffer, SIANO_IIC_RX_BUFFER_SIZE);
           if(ResponseResult == 0xffff)
		   {
              break;
		   }
		   else
		   if(ResponseResult == 0xf000)
		   {
              i = SpecialCommandWaitResponseLoop;
		      break;
		   }
	   }
    
       if(i == SpecialCommandWaitResponseLoop)
	   {
	      SMS4470ErrorPrintf("(SMS4470_CheckDVBT2PhysicalLayerPipeInformation) xxxxxxxxx Get response failed !\n");

          return FALSE;
	   }

       SMSHOSTLIB_STATISTICS_DVBT2_ST* pStat;
        
       pRespondseMsg = (SmsMsgData_ST*)Sms4470DeviceGroupInformation[Index].Sms4470MessageRxBuffer;
        
       if(pRespondseMsg->msgData[0] == 0)
	   {          
          pStat = (SMSHOSTLIB_STATISTICS_DVBT2_ST*)&pRespondseMsg->msgData[1];
        
	      if(pStat->ReceptionData.IsModemLocked == 0x01)
		  {
	         if(pStat->ReceptionData.numOfPlps != 0)
			 {
			    CheckFlag = TRUE;
                for(i=0;i<pStat->ReceptionData.numOfPlps;i++)
				{
			        if(pStat->PlpData[i].plpStatistics.plpId == -1)
					{
                       CheckFlag = FALSE;
				       break;
					}
				}
                if(CheckFlag)
				{
				   *pDoesPhysicalLayerPipeInformationExistFlag = TRUE;

                   if(Sms4470DeviceGroupInformation[Index].SMS4470DVBT2LastPlpIdSetup != 0xFF)
				   {
                      if(pStat->activePlps[0].plpId != 0xFFFFFFFF)
					  {
					     if(pStat->activePlps[0].plpId != Sms4470DeviceGroupInformation[Index].SMS4470DVBT2LastPlpIdSetup)
						 {

                            SMS4470InfoPrintf("(SMS4470_CheckDVBT2PhysicalLayerPipeInformation) Working around for unmatch PLP ID\n");

	                        SMS4470_OpenPlp(FrontendGroupID,Sms4470DeviceGroupInformation[Index].SMS4470DVBT2LastPlpIdSetup);
						 }
					  }
				   }
				}
			 }
		  }
	   }
	   else
	   {
          SMS4470DebugPrintf("(SMS4470_CheckDVBT2PhysicalLayerPipeInformation) pRespondseMsg->msgData[0] != 0\n");
	   }

	   return TRUE;
	}
	else
	   return TRUE;
}
//**********************************************************************************************************
//**********************************************************************************************************
//**********************************************************************************************************
BOOL SMS4470_OpenPlp(ULONG FrontendGroupID,UINT32 plpId)
{
	ULONG               Index;
    UINT32              i;
	UINT32              ResponseResult;
	SmsMsgData_ST       SmsMsg = {{0}};
	UINT16              TempMsgLength;
    
    SMS4470FunctionNamePrintf("(SMS4470_OpenPlp)\n");
    SMS4470DebugPrintf("(SMS4470_OpenPlp) plpId : 0x%x\n",plpId);

	if(FrontendGroupID == FRONTEND_TWO_DIVERSITY_GROUP_ID)
	   Index = 0;
	else
	   Index = 1;

	SmsMsg.xMsgHeader.msgType  = MSG_SMS_DVBT2_OPEN_PLP_REQ;
	SmsMsg.xMsgHeader.msgSrcId = SMS_HOST_LIB; // 0;
    SmsMsg.xMsgHeader.msgDstId = HIF_TASK;
    SmsMsg.xMsgHeader.msgFlags = MSG_HDR_FLAG_STATIC_MSG; // 0;
	SmsMsg.xMsgHeader.msgLength = (UINT16)sizeof(SmsMsgData_ST);
	SmsMsg.msgData[0] = plpId;

   
SMS4470DebugPrintf("(SMS4470_OpenPlp) Send command (MSG_SMS_DVBT2_OPEN_PLP_REQ) to SMS4470\n");

    TempMsgLength = SmsMsg.xMsgHeader.msgLength;
#ifdef SMS_BIGENDIAN
    SmsMsg.xMsgHeader.msgType     = SMS_ENDIAN_SWAP16(SmsMsg.xMsgHeader.msgType);
    SmsMsg.xMsgHeader.msgLength   = SMS_ENDIAN_SWAP16(SmsMsg.xMsgHeader.msgLength);
    SmsMsg.xMsgHeader.msgFlags    = SMS_ENDIAN_SWAP16(SmsMsg.xMsgHeader.msgFlags);
    SmsMsg.msgData[0]             = SMS_ENDIAN_SWAP32(SmsMsg.msgData[0]);
#endif

//       SMS4470SysDelay(5);
	SMS4470I2CWrite(FrontendGroupID,(UINT8*)&SmsMsg,TempMsgLength);

	
SMS4470DebugPrintf("(SMS4470_OpenPlp) Get respence from SMS4470\n");
    
    for(i=0 ; i<SpecialCommandWaitResponseLoop ; i++)
	{
        SMS4470SysDelay(10);
		ResponseResult = SMS4470_read_response(FrontendGroupID,MSG_SMS_DVBT2_OPEN_PLP_RES, Sms4470DeviceGroupInformation[Index].Sms4470MessageRxBuffer, SIANO_IIC_RX_BUFFER_SIZE);
        if(ResponseResult == 0xffff)
		{
           break;
		}
		else
		if(ResponseResult == 0xf000)
		{
           i = SpecialCommandWaitResponseLoop;
		   break;
		}
	}
    
    if(i == SpecialCommandWaitResponseLoop)
	{
	   SMS4470ErrorPrintf("(SMS4470_OpenPlp) xxxxxxxxx Get response failed !\n");
	}  

    return TRUE;
}

//**********************************************************************************************************
//**********************************************************************************************************
//**********************************************************************************************************
BOOL SMS4470_ClosePlp(ULONG FrontendGroupID,UINT32 plpId)
{
	ULONG               Index;
    UINT32              i;
	UINT32              ResponseResult;
	SmsMsgData_ST       SmsMsg = {{0}};
	UINT16              TempMsgLength;
    
    SMS4470FunctionNamePrintf("(SMS4470_ClosePlp)\n");
    SMS4470DebugPrintf("(SMS4470_ClosePlp) plpId : 0x%x\n",plpId);

	if(FrontendGroupID == FRONTEND_TWO_DIVERSITY_GROUP_ID)
	   Index = 0;
	else
	   Index = 1;

	SmsMsg.xMsgHeader.msgType  = MSG_SMS_DVBT2_CLOSE_PLP_REQ;
    SmsMsg.xMsgHeader.msgSrcId = SMS_HOST_LIB; // 0;
    SmsMsg.xMsgHeader.msgDstId = HIF_TASK;
    SmsMsg.xMsgHeader.msgFlags = MSG_HDR_FLAG_STATIC_MSG; // 0;
    SmsMsg.xMsgHeader.msgLength = (UINT16)sizeof(SmsMsgData_ST);
    SmsMsg.msgData[0] = plpId;

   
SMS4470DebugPrintf("(SMS4470_ClosePlp) Send command (MSG_SMS_DVBT2_CLOSE_PLP_REQ) to SMS4470\n");

    TempMsgLength = SmsMsg.xMsgHeader.msgLength;
#ifdef SMS_BIGENDIAN
    SmsMsg.xMsgHeader.msgType     = SMS_ENDIAN_SWAP16(SmsMsg.xMsgHeader.msgType);
    SmsMsg.xMsgHeader.msgLength   = SMS_ENDIAN_SWAP16(SmsMsg.xMsgHeader.msgLength);
    SmsMsg.xMsgHeader.msgFlags    = SMS_ENDIAN_SWAP16(SmsMsg.xMsgHeader.msgFlags);
    SmsMsg.msgData[0]             = SMS_ENDIAN_SWAP32(SmsMsg.msgData[0]);
#endif

//       SMS4470SysDelay(5);
	SMS4470I2CWrite(FrontendGroupID,(UINT8*)&SmsMsg,TempMsgLength);

	
SMS4470DebugPrintf("(SMS4470_ClosePlp) Get respence from SMS4470\n");
    
    for(i=0 ; i<SpecialCommandWaitResponseLoop ; i++)
	{
        SMS4470SysDelay(10);
		ResponseResult = SMS4470_read_response(FrontendGroupID,MSG_SMS_DVBT2_CLOSE_PLP_RES, Sms4470DeviceGroupInformation[Index].Sms4470MessageRxBuffer, SIANO_IIC_RX_BUFFER_SIZE);
        if(ResponseResult == 0xffff)
		{
           break;
		}
		else
		if(ResponseResult == 0xf000)
		{
           i = SpecialCommandWaitResponseLoop;
		   break;
		}
	}
     
    if(i == SpecialCommandWaitResponseLoop)
	{
	   SMS4470ErrorPrintf("(SMS4470_ClosePlp) xxxxxxxxx Get response failed !\n");
	}

    return TRUE;
}
//********************************************************************************************
//********************************************************************************************
//********************************************************************************************
BOOL SMS4470_AddPidFilter(ULONG FrontendGroupID,UINT32 PID)
{
	ULONG           Index;
    UINT32          i;
	UINT32          ResponseResult;
	SmsMsgData_ST   SmsMsg = {{0}};
	UINT16          TempMsgLength;
        
    SMS4470FunctionNamePrintf("(SMS4470_AddPidFilter)\n");
    SMS4470DebugPrintf("(SMS4470_AddPidFilter) PID : 0x%x\n",PID);

	if(FrontendGroupID == FRONTEND_TWO_DIVERSITY_GROUP_ID)
	   Index = 0;
	else
	   Index = 1;

	SmsMsg.xMsgHeader.msgType  = MSG_SMS_ADD_PID_FILTER_REQ;
	SmsMsg.xMsgHeader.msgSrcId = SMS_HOST_LIB; // 0;
	SmsMsg.xMsgHeader.msgDstId = HIF_TASK;
	SmsMsg.xMsgHeader.msgFlags = MSG_HDR_FLAG_STATIC_MSG; // 0;
	SmsMsg.xMsgHeader.msgLength = (UINT16)sizeof(SmsMsgData_ST);
	SmsMsg.msgData[0] = PID;

   
SMS4470DebugPrintf("(SMS4470_AddPidFilter) Send command (MSG_SMS_ADD_PID_FILTER_REQ) to SMS4470\n");

    TempMsgLength = SmsMsg.xMsgHeader.msgLength;
#ifdef SMS_BIGENDIAN
    SmsMsg.xMsgHeader.msgType     = SMS_ENDIAN_SWAP16(SmsMsg.xMsgHeader.msgType);
    SmsMsg.xMsgHeader.msgLength   = SMS_ENDIAN_SWAP16(SmsMsg.xMsgHeader.msgLength);
    SmsMsg.xMsgHeader.msgFlags    = SMS_ENDIAN_SWAP16(SmsMsg.xMsgHeader.msgFlags);
    SmsMsg.msgData[0]             = SMS_ENDIAN_SWAP32(SmsMsg.msgData[0]);
#endif

//    SMS4470SysDelay(5);
	SMS4470I2CWrite(FrontendGroupID,(UINT8*)&SmsMsg,TempMsgLength);

	
SMS4470DebugPrintf("(SMS4470_AddPidFilter) Get respence from SMS4470\n");
    
    for (i=0 ; i<SpecialCommandWaitResponseLoop ; i++)
    {
        SMS4470SysDelay(10);
		ResponseResult = SMS4470_read_response(FrontendGroupID,MSG_SMS_ADD_PID_FILTER_RES, Sms4470DeviceGroupInformation[Index].Sms4470MessageRxBuffer, SIANO_IIC_RX_BUFFER_SIZE);
        if(ResponseResult == 0xffff)
		{
           break;
		}
		else
		if(ResponseResult == 0xf000)
		{
           i = SpecialCommandWaitResponseLoop;
		   break;
		}
    }
    
    if(i == SpecialCommandWaitResponseLoop)
    {
	   SMS4470ErrorPrintf("(SMS4470_AddPidFilter) xxxxxxxxx Get response failed !\n");
	   OSPorting_Frontend_failflag(128,128);
    }

    return TRUE;
}

//********************************************************************************************
//********************************************************************************************
//********************************************************************************************
BOOL SMS4470_RemovePidFilter(ULONG FrontendGroupID,UINT32 PID)
{
	ULONG           Index;
    UINT32          i;
	UINT32          ResponseResult;
	SmsMsgData_ST   SmsMsg = {{0}};
	UINT16          TempMsgLength;
    
    SMS4470FunctionNamePrintf("(SMS4470_RemovePidFilter)\n");
    SMS4470DebugPrintf("(SMS4470_RemovePidFilter) PID : 0x%x\n",PID);

	if(FrontendGroupID == FRONTEND_TWO_DIVERSITY_GROUP_ID)
	   Index = 0;
	else
	   Index = 1;

	SmsMsg.xMsgHeader.msgType  = MSG_SMS_REMOVE_PID_FILTER_REQ;
	SmsMsg.xMsgHeader.msgSrcId = SMS_HOST_LIB; // 0;
	SmsMsg.xMsgHeader.msgDstId = HIF_TASK;
	SmsMsg.xMsgHeader.msgFlags = MSG_HDR_FLAG_STATIC_MSG; // 0;
	SmsMsg.xMsgHeader.msgLength = (UINT16)sizeof(SmsMsgData_ST);
	SmsMsg.msgData[0] = PID;

   
SMS4470DebugPrintf("(SMS4470_RemovePidFilter) Send command (MSG_SMS_ADD_PID_FILTER_REQ) to SMS4470\n");

    TempMsgLength = SmsMsg.xMsgHeader.msgLength;
#ifdef SMS_BIGENDIAN
    SmsMsg.xMsgHeader.msgType     = SMS_ENDIAN_SWAP16(SmsMsg.xMsgHeader.msgType);
    SmsMsg.xMsgHeader.msgLength   = SMS_ENDIAN_SWAP16(SmsMsg.xMsgHeader.msgLength);
    SmsMsg.xMsgHeader.msgFlags    = SMS_ENDIAN_SWAP16(SmsMsg.xMsgHeader.msgFlags);
    SmsMsg.msgData[0]             = SMS_ENDIAN_SWAP32(SmsMsg.msgData[0]);
#endif

//    SMS4470SysDelay(5);
	SMS4470I2CWrite(FrontendGroupID,(UINT8*)&SmsMsg,TempMsgLength);

	
SMS4470DebugPrintf("(SMS4470_RemovePidFilter) Get respence from SMS4470\n");
    
    for (i=0 ; i<SpecialCommandWaitResponseLoop ; i++)
    {
        SMS4470SysDelay(10);
		ResponseResult = SMS4470_read_response(FrontendGroupID,MSG_SMS_REMOVE_PID_FILTER_RES, Sms4470DeviceGroupInformation[Index].Sms4470MessageRxBuffer, SIANO_IIC_RX_BUFFER_SIZE);
        if(ResponseResult == 0xffff)
		{
           break;
		}
		else
		if(ResponseResult == 0xf000)
		{
           i = SpecialCommandWaitResponseLoop;
		   OSPorting_Frontend_failflag(64,64);
		   break;
		}
    }
    
    if(i == SpecialCommandWaitResponseLoop)
    {
	   SMS4470ErrorPrintf("(SMS4470_RemovePidFilter) xxxxxxxxx Get response failed !\n");
	   OSPorting_Frontend_failflag(64,64);
    }

    return TRUE;
}
BOOL SMS4470_DVBT2LiteControl(ULONG FrontendGroupID,BOOL T2LiteActiveFlag)
{
	ULONG           Index;
    UINT32          i;
	UINT32          ResponseResult;
	SmsMsgData_ST   SmsMsg = {{0}};
	UINT16          TempMsgLength;
    
    SMS4470FunctionNamePrintf("(SMS4470_DVBT2LiteControl)\n");
    SMS4470DebugPrintf("(SMS4470_DVBT2LiteControl) T2LiteActiveFlag : 0x%x\n",T2LiteActiveFlag);

	if(FrontendGroupID == FRONTEND_TWO_DIVERSITY_GROUP_ID)
	   Index = 0;
	else
	   Index = 1;

	SmsMsg.xMsgHeader.msgType  = MSG_SMS_DVBT2_T2LITE_REQ;
	SmsMsg.xMsgHeader.msgSrcId = SMS_HOST_LIB; // 0;
	SmsMsg.xMsgHeader.msgDstId = HIF_TASK;
	SmsMsg.xMsgHeader.msgFlags = MSG_HDR_FLAG_STATIC_MSG; // 0;
	SmsMsg.xMsgHeader.msgLength = (UINT16)sizeof(SmsMsgData_ST);
	SmsMsg.msgData[0] = T2LiteActiveFlag;

   
SMS4470DebugPrintf("(SMS4470_DVBT2LiteControl) Send command (MSG_SMS_ADD_PID_FILTER_REQ) to SMS4470\n");

    TempMsgLength = SmsMsg.xMsgHeader.msgLength;
#ifdef SMS_BIGENDIAN
    SmsMsg.xMsgHeader.msgType     = SMS_ENDIAN_SWAP16(SmsMsg.xMsgHeader.msgType);
    SmsMsg.xMsgHeader.msgLength   = SMS_ENDIAN_SWAP16(SmsMsg.xMsgHeader.msgLength);
    SmsMsg.xMsgHeader.msgFlags    = SMS_ENDIAN_SWAP16(SmsMsg.xMsgHeader.msgFlags);
    SmsMsg.msgData[0]             = SMS_ENDIAN_SWAP32(SmsMsg.msgData[0]);
#endif

//    SMS4470SysDelay(5);
	SMS4470I2CWrite(FrontendGroupID,(UINT8*)&SmsMsg,TempMsgLength);

	
SMS4470DebugPrintf("(SMS4470_DVBT2LiteControl) Get respence from SMS4470\n");
    
    for (i=0 ; i<SpecialCommandWaitResponseLoop ; i++)
    {
        SMS4470SysDelay(10);
		ResponseResult = SMS4470_read_response(FrontendGroupID,MSG_SMS_DVBT2_T2LITE_RES, Sms4470DeviceGroupInformation[Index].Sms4470MessageRxBuffer, SIANO_IIC_RX_BUFFER_SIZE);
        if(ResponseResult == 0xffff)
		{
           break;
		}
		else
		if(ResponseResult == 0xf000)
		{
           i = SpecialCommandWaitResponseLoop;
		   break;
		}
    }
    
    if(i == SpecialCommandWaitResponseLoop)
    {
	   SMS4470ErrorPrintf("(SMS4470_DVBT2LiteControl) xxxxxxxxx Get response failed !\n");
    }

    return TRUE;
}
//********************************************************************************************
//********************************************************************************************
//********************************************************************************************
//********************************************************************************************
//********************************************************************************************
//********************************************************************************************
VOID SMS4470ReadResponseWithDelay(ULONG FrontendGroupID,ULONG Milliseconds)
{
	ULONG           Index;
    UINT32          i;
	UINT32          ResponseResult;

    SMS4470FunctionNamePrintf("(SMS4470ReadResponseWithDelay)\n");

	if(FrontendGroupID == FRONTEND_TWO_DIVERSITY_GROUP_ID)
	   Index = 0;
	else
	   Index = 1;

	if(Milliseconds < 10)
	   Milliseconds = 10;

    for(i=0 ; i<(Milliseconds/10) ; i++)
	{
        SMS4470SysDelay(10);
		ResponseResult = SMS4470_read_response(FrontendGroupID,(MSG_LAST_MSG_TYPE - 2), Sms4470DeviceGroupInformation[Index].Sms4470MessageRxBuffer, SIANO_IIC_RX_BUFFER_SIZE);
        if(ResponseResult == 0xffff || ResponseResult == 0xf000) 
		{
           break;
		} 	 	      
	}
}
//********************************************************************************************
//********************************************************************************************
//********************************************************************************************
VOID SMS4470SysDelay(ULONG Milliseconds)
{
    OSPorting_Frontend_Sleep(Milliseconds);       
}
BOOL SMS4470I2CWrite(ULONG FrontendGroupID,PUBYTE pData,ULONG DataLength)
{
	BOOL Result;

    Result = PlatformPorting_Frontend_I2C_Write(FrontendGroupID,DEMODULATOR_DEVICE_ADDRESS,pData,DataLength);
	if(Result == FALSE)
	{
       SMS4470ErrorPrintf("(SMS4470I2CWrite) ********* I2C Write Failed *********\n");
	}

	return Result;
}
BOOL SMS4470I2CWrite_FW(ULONG FrontendGroupID,PUBYTE pData,ULONG DataLength)
{
	BOOL Result;    

    Result = PlatformPorting_Frontend_I2C_Write_FW(FrontendGroupID,DEMODULATOR_DEVICE_ADDRESS,pData,DataLength);
	if(Result == FALSE)
	{
       SMS4470ErrorPrintf("(SMS4470I2CWrite_FW) ********* I2C Write FW Failed *********\n");
	}

	return Result;
}
BOOL SMS4470I2CRead(ULONG FrontendGroupID,ULONG ReadLength, PUBYTE pBuffer)
{
	BOOL Result;

    Result = PlatformPorting_Frontend_I2C_Read(FrontendGroupID,DEMODULATOR_DEVICE_ADDRESS,ReadLength,pBuffer);
	if(Result == FALSE)
	{
       SMS4470ErrorPrintf("(SMS4470I2CRead) ********* I2C Read Failed *********\n");
	}

	return Result;
}
//********************************************************************************************
//********************************************************************************************
//********************************************************************************************

