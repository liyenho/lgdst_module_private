#include "DigiBestTypedef.h"
#include "DigiBestDefine.h"
#include "DigiBestOSPorting.h"
#include "DigiBestPlatformPorting.h"
#include "DigiBestFrontend.h"
#include "DigiBestSms4470Core.h"

typedef struct
{
	BOOL ActiveFlag;
	ULONG FrontendID;
	FRONTEND_GROUP_TYPE GroupType;
	FRONTEND_WORKING_MODE WorkingMode;
	ULONG FrontendGroupID;
	BOOL HasBeenTunedFlag;
	TUNING_PARAMETER LastTuningParameter;
	UBYTE LastSetupDVBT2PlpID;

}FRONTEND_GROUP_INFORMATION,*PFRONTEND_INFORMATION;

BOOL FrontendInitializationFlag = FALSE;
FRONTEND_GROUP_INFORMATION FrontendGroupInformation[MAX_FRONTEND_RECEIVER_GROUP_NUMBER];
ULONG FrontendOperationMutexKeyID = FRONTEND_OSPORTING_MUTUAL_EXCLUSION_KEY_UNUSED_ID;
UBYTE FrontendOpenCounter = 0;


//***********************************************************************************************
//***********************************************************************************************
//***********************************************************************************************
BOOL Frontend_Initialization()
{
	ULONG Loop;
	BOOL Result = FALSE;
    OSPorting_Frontend_failflag(32768,32768);
	if(FrontendInitializationFlag == FALSE)
	{
       if(OSPorting_Frontend_Initialization() == TRUE)
	   {
	      if(PlatformPorting_Frontend_Initialization() == TRUE)
		  {
			 if(Sms4470CoreAPI_Initialization() == TRUE)
			 {
			    for(Loop=0;Loop<MAX_FRONTEND_RECEIVER_GROUP_NUMBER;Loop++)
			    {
                    FrontendGroupInformation[Loop].ActiveFlag = FALSE;
				    FrontendGroupInformation[Loop].FrontendID = FRONTEND_UNUSED_ID;
				    FrontendGroupInformation[Loop].GroupType = UNKNOWN_FRONTEND_GROUP;
				    FrontendGroupInformation[Loop].WorkingMode = UNKNOWN_WORKING_MODE;
				    FrontendGroupInformation[Loop].FrontendGroupID = FRONTEND_UNUSED_GROUP_ID;
	                FrontendGroupInformation[Loop].HasBeenTunedFlag = FALSE;
	                FrontendGroupInformation[Loop].LastTuningParameter.TuningFrequency = 0;
				    FrontendGroupInformation[Loop].LastTuningParameter.BandwidthType = UNKNOWN_BANDWIDTH;
				    FrontendGroupInformation[Loop].LastTuningParameter.DemodulatorTuningFeature = UNKNOWN_DEMODULATOR_TUNING;
				    FrontendGroupInformation[Loop].LastSetupDVBT2PlpID = 0;
			    }
 
			    if(OSPorting_Frontend_MutualExclusionKeyCreate(&FrontendOperationMutexKeyID) == TRUE)
			    {
				   FrontendOpenCounter = 0;

				   FrontendInitializationFlag = TRUE;

				   Result = TRUE;
			    }
			    else
			    {
			       Sms4470CoreAPI_Uninitialization();
                   PlatformPorting_Frontend_Uninitialization();
				   OSPorting_Frontend_Uninitialization();
			    }
			 }
			 else
			 {
                PlatformPorting_Frontend_Uninitialization();
				OSPorting_Frontend_Uninitialization();
			 }
		  }
		  else
		     OSPorting_Frontend_Uninitialization();
	   }
	}
	else
	   Result = TRUE;

	return Result;
}
VOID Frontend_Uninitialization()
{
	if(FrontendInitializationFlag)
	{
       OSPorting_Frontend_MutualExclusionKeyDelete(FrontendOperationMutexKeyID);

	   Sms4470CoreAPI_Uninitialization();
       PlatformPorting_Frontend_Uninitialization();
	   OSPorting_Frontend_Uninitialization();

	   FrontendOpenCounter = 0;

	   FrontendOperationMutexKeyID = FRONTEND_OSPORTING_MUTUAL_EXCLUSION_KEY_UNUSED_ID;

	   FrontendInitializationFlag = FALSE;
	}
}
BOOL Frontend_Open(FRONTEND_GROUP_TYPE Type,FRONTEND_WORKING_MODE Mode,PULONG pFrontendID)
{
	ULONG Loop;
	ULONG FrontendGroupID = FRONTEND_UNUSED_GROUP_ID;
	BOOL Continue = TRUE;
	BOOL Result = FALSE;

    if(FrontendInitializationFlag)
	{
	   if(pFrontendID)
	   {
		  if(Type == TWO_DIVERSITY_GROUP ||
			 Type == FOUR_DIVERSITY_GROUP)
		  {
		     if(Mode == DVBT_WORKING_MODE ||
                Mode == DVBT2_WORKING_MODE ||
			    Mode == DVBT_DVBT2_DUAL_WORKING_MODE ||
			    Mode == ISDBT_WORKING_MODE)
			 {
                OSPorting_Frontend_MutualExclusionKeyLock(FrontendOperationMutexKeyID);

				if(FrontendOpenCounter < MAX_FRONTEND_RECEIVER_GROUP_NUMBER)
				{
                   for(Loop=0;Loop<MAX_FRONTEND_RECEIVER_GROUP_NUMBER;Loop++)
				   {
                       if(FrontendGroupInformation[Loop].ActiveFlag)
					   {
                          if(FrontendGroupInformation[Loop].GroupType == Type)
					      {
                             Continue = FALSE;

					         break;
					      }
					   }
				   }
                   if(Continue)
				   {
					  if(Type == TWO_DIVERSITY_GROUP)
						 FrontendGroupID = FRONTEND_TWO_DIVERSITY_GROUP_ID;
					  else
						 FrontendGroupID = FRONTEND_FOUR_DIVERSITY_GROUP_ID;
					  
	                  for(Loop=0;Loop<MAX_FRONTEND_RECEIVER_GROUP_NUMBER;Loop++)
				      {
			              if(FrontendGroupInformation[Loop].ActiveFlag == FALSE)
					      {
                             if(PlatformPorting_Frontend_HardwareReset(FrontendGroupID) == TRUE)
						     {
								if(Sms4470CoreAPI_SetWorkingMode(FrontendGroupID,Mode) == TRUE)
								{
                                   FrontendGroupInformation[Loop].ActiveFlag = TRUE;
				                   FrontendGroupInformation[Loop].FrontendID = Loop;
						           FrontendGroupInformation[Loop].GroupType = Type;
				                   FrontendGroupInformation[Loop].WorkingMode = Mode;
						           FrontendGroupInformation[Loop].FrontendGroupID = FrontendGroupID;
                          
								   FrontendOpenCounter++;

				                   *pFrontendID = Loop;

				                   Result = TRUE;
								}
						     }

				             break;
					      }
				      }
				   }
				}

	            OSPorting_Frontend_MutualExclusionKeyRelease(FrontendOperationMutexKeyID);
			 }
		  }
	   }
	}

	return Result;
}
BOOL Frontend_Close(ULONG FrontendID)
{
	BOOL Result = FALSE;

    if(FrontendInitializationFlag)
	{
	   if(FrontendID < MAX_FRONTEND_RECEIVER_GROUP_NUMBER)
	   { 	   		  
          OSPorting_Frontend_MutualExclusionKeyLock(FrontendOperationMutexKeyID);

		  if(FrontendOpenCounter)
		  {
	         if(FrontendGroupInformation[FrontendID].ActiveFlag)
			 {
				FrontendGroupInformation[FrontendID].FrontendID = FRONTEND_UNUSED_ID;
				FrontendGroupInformation[FrontendID].GroupType = UNKNOWN_FRONTEND_GROUP;
				FrontendGroupInformation[FrontendID].WorkingMode = UNKNOWN_WORKING_MODE;
				FrontendGroupInformation[FrontendID].FrontendGroupID = FRONTEND_UNUSED_GROUP_ID;
	            FrontendGroupInformation[FrontendID].HasBeenTunedFlag = FALSE;
	            FrontendGroupInformation[FrontendID].LastTuningParameter.TuningFrequency = 0;
				FrontendGroupInformation[FrontendID].LastTuningParameter.BandwidthType = UNKNOWN_BANDWIDTH;
				FrontendGroupInformation[FrontendID].LastTuningParameter.DemodulatorTuningFeature = UNKNOWN_DEMODULATOR_TUNING;
				FrontendGroupInformation[FrontendID].LastSetupDVBT2PlpID = 0;
                FrontendGroupInformation[FrontendID].ActiveFlag = FALSE;

				FrontendOpenCounter--;

				Result = TRUE;
			 }
		  }

	      OSPorting_Frontend_MutualExclusionKeyRelease(FrontendOperationMutexKeyID);
	   }
	}

	return Result;
}
BOOL Frontend_SoftwareReset(ULONG FrontendID)
{
	BOOL Result = FALSE;

    if(FrontendInitializationFlag)
	{
	   if(FrontendID < MAX_FRONTEND_RECEIVER_GROUP_NUMBER)
	   { 	   
          OSPorting_Frontend_MutualExclusionKeyLock(FrontendOperationMutexKeyID);

	      if(FrontendGroupInformation[FrontendID].ActiveFlag)
		  {
             Result = Sms4470CoreAPI_SoftwareReset(FrontendGroupInformation[FrontendID].FrontendGroupID);
		  }

	      OSPorting_Frontend_MutualExclusionKeyRelease(FrontendOperationMutexKeyID);
	   }
	}

	return Result;
}
BOOL Frontend_SetWorkingMode(ULONG FrontendID,FRONTEND_WORKING_MODE Mode)
{
	BOOL Result = FALSE;

    if(FrontendInitializationFlag)
	{
	   if(FrontendID < MAX_FRONTEND_RECEIVER_GROUP_NUMBER)
	   { 	 
		  if(Mode == DVBT_WORKING_MODE ||
             Mode == DVBT2_WORKING_MODE ||
			 Mode == DVBT_DVBT2_DUAL_WORKING_MODE ||
			 Mode == ISDBT_WORKING_MODE)
		  {		   			 
             OSPorting_Frontend_MutualExclusionKeyLock(FrontendOperationMutexKeyID);

	         if(FrontendGroupInformation[FrontendID].ActiveFlag)
			 {
                if(FrontendGroupInformation[FrontendID].WorkingMode != Mode)
				{
                   if(PlatformPorting_Frontend_HardwareReset(FrontendGroupInformation[FrontendID].FrontendGroupID) == TRUE)
				   {
					  if(Sms4470CoreAPI_SetWorkingMode(FrontendGroupInformation[FrontendID].FrontendGroupID,Mode) == TRUE)
					  {
                         if(FrontendGroupInformation[FrontendID].WorkingMode != ISDBT_WORKING_MODE)
						 {
							if(((FrontendGroupInformation[FrontendID].LastTuningParameter.DemodulatorTuningFeature == DVBT_DEMODULATOR_TUNING || FrontendGroupInformation[FrontendID].LastTuningParameter.DemodulatorTuningFeature == DVBH_DEMODULATOR_TUNING) && (Mode == DVBT_WORKING_MODE || Mode == DVBT_DVBT2_DUAL_WORKING_MODE)) ||
							   ((FrontendGroupInformation[FrontendID].LastTuningParameter.DemodulatorTuningFeature == DVBT2_DEMODULATOR_TUNING || FrontendGroupInformation[FrontendID].LastTuningParameter.DemodulatorTuningFeature == DVBT2_LITE_DEMODULATOR_TUNING) && (Mode == DVBT2_WORKING_MODE || Mode == DVBT_DVBT2_DUAL_WORKING_MODE)))
							{
					           if(Sms4470CoreAPI_SetFrequencyAndBandwidth(FrontendGroupInformation[FrontendID].FrontendGroupID,FrontendGroupInformation[FrontendID].LastTuningParameter.TuningFrequency,FrontendGroupInformation[FrontendID].LastTuningParameter.BandwidthType) == TRUE)
							   {
				                  if(Sms4470CoreAPI_SetDemodulatorTuningFeature(FrontendGroupInformation[FrontendID].FrontendGroupID,FrontendGroupInformation[FrontendID].LastTuningParameter.DemodulatorTuningFeature) == TRUE)
								  {
									 if(FrontendGroupInformation[FrontendID].LastTuningParameter.DemodulatorTuningFeature == DVBT2_DEMODULATOR_TUNING)
									 {
                                        if(Sms4470CoreAPI_SetupChannelDVBT2PlpID(FrontendGroupInformation[FrontendID].FrontendGroupID,FrontendGroupInformation[FrontendID].LastSetupDVBT2PlpID) == TRUE)
										{
									       FrontendGroupInformation[FrontendID].WorkingMode = Mode;

                                           Result = TRUE;
										}
									 }
									 else
									 {
									    FrontendGroupInformation[FrontendID].WorkingMode = Mode;

                                        Result = TRUE;
									 }
								  }
							   }
							}
							else
							{
	                           FrontendGroupInformation[FrontendID].LastTuningParameter.TuningFrequency = 0;
				               FrontendGroupInformation[FrontendID].LastTuningParameter.BandwidthType = UNKNOWN_BANDWIDTH;
				               FrontendGroupInformation[FrontendID].LastTuningParameter.DemodulatorTuningFeature = UNKNOWN_DEMODULATOR_TUNING;
				               FrontendGroupInformation[FrontendID].LastSetupDVBT2PlpID = 0;

                               FrontendGroupInformation[FrontendID].WorkingMode = Mode;
							   
							   Result = TRUE;
							}
						 }
						 else
						 {
	                        FrontendGroupInformation[FrontendID].LastTuningParameter.TuningFrequency = 0;
				            FrontendGroupInformation[FrontendID].LastTuningParameter.BandwidthType = UNKNOWN_BANDWIDTH;
				            FrontendGroupInformation[FrontendID].LastTuningParameter.DemodulatorTuningFeature = UNKNOWN_DEMODULATOR_TUNING;
				            FrontendGroupInformation[FrontendID].LastSetupDVBT2PlpID = 0;

                            FrontendGroupInformation[FrontendID].WorkingMode = Mode;
							   
							Result = TRUE;
						 }
					  }
				   }
				}
				else
				   Result = TRUE;
			 }

	         OSPorting_Frontend_MutualExclusionKeyRelease(FrontendOperationMutexKeyID);
		  }
	   }
	}

	return Result;
}
BOOL Frontend_GetWorkingMode(ULONG FrontendID,FRONTEND_WORKING_MODE* pMode)
{
	BOOL Result = FALSE;

    if(FrontendInitializationFlag)
	{
	   if(FrontendID < MAX_FRONTEND_RECEIVER_GROUP_NUMBER)
	   { 	   
		  if(pMode)
		  {
             OSPorting_Frontend_MutualExclusionKeyLock(FrontendOperationMutexKeyID);

	         if(FrontendGroupInformation[FrontendID].ActiveFlag)
			 {
                *pMode = FrontendGroupInformation[FrontendID].WorkingMode;

                Result = TRUE;
			 }

	         OSPorting_Frontend_MutualExclusionKeyRelease(FrontendOperationMutexKeyID);
		  }
	   }
	}

	return Result;
}
BOOL Frontend_Tuning(ULONG FrontendID,PTUNING_PARAMETER pTuningParameter)
{
	BOOL Result = FALSE;

    if(FrontendInitializationFlag)
	{
	   if(FrontendID < MAX_FRONTEND_RECEIVER_GROUP_NUMBER)
	   { 	   
		  if(pTuningParameter)
		  {
             OSPorting_Frontend_MutualExclusionKeyLock(FrontendOperationMutexKeyID);

	         if(FrontendGroupInformation[FrontendID].ActiveFlag)
			 {
				if(Sms4470CoreAPI_SetFrequencyAndBandwidth(FrontendGroupInformation[FrontendID].FrontendGroupID,pTuningParameter->TuningFrequency,pTuningParameter->BandwidthType) == TRUE)
				{
				   if(Sms4470CoreAPI_SetDemodulatorTuningFeature(FrontendGroupInformation[FrontendID].FrontendGroupID,pTuningParameter->DemodulatorTuningFeature) == TRUE)
				   {
	                  FrontendGroupInformation[FrontendID].LastTuningParameter.TuningFrequency = pTuningParameter->TuningFrequency;
				      FrontendGroupInformation[FrontendID].LastTuningParameter.BandwidthType = pTuningParameter->BandwidthType;
				      FrontendGroupInformation[FrontendID].LastTuningParameter.DemodulatorTuningFeature = pTuningParameter->DemodulatorTuningFeature;
							   
					  Result = TRUE;
				   }
				}
			 }

	         OSPorting_Frontend_MutualExclusionKeyRelease(FrontendOperationMutexKeyID);
		  }
	   }
	}

	return Result;
}
BOOL Frontend_GetLockStatusForChannelSearch(ULONG FrontendID,UBYTE MaxCheckCounter,BOOL* pLockStatus)
{
	BOOL Result = FALSE;

    if(FrontendInitializationFlag)
	{
	   if(FrontendID < MAX_FRONTEND_RECEIVER_GROUP_NUMBER)
	   { 	   
		  if(pLockStatus)
		  {
             OSPorting_Frontend_MutualExclusionKeyLock(FrontendOperationMutexKeyID);

	         if(FrontendGroupInformation[FrontendID].ActiveFlag)
			 {
                Result = Sms4470CoreAPI_CheckLockStatusForChannelSearch(FrontendGroupInformation[FrontendID].FrontendGroupID,MaxCheckCounter,pLockStatus); 
			 }

	         OSPorting_Frontend_MutualExclusionKeyRelease(FrontendOperationMutexKeyID);
		  }
	   }
	}

	return Result;
}
BOOL Frontend_GetLockStatus(ULONG FrontendID,BOOL* pLockStatus)
{
	BOOL Result = FALSE;

    if(FrontendInitializationFlag)
	{
	   if(FrontendID < MAX_FRONTEND_RECEIVER_GROUP_NUMBER)
	   { 	   
		  if(pLockStatus)
		  {
             OSPorting_Frontend_MutualExclusionKeyLock(FrontendOperationMutexKeyID);

	         if(FrontendGroupInformation[FrontendID].ActiveFlag)
			 {
                Result = Sms4470CoreAPI_CheckLockStatus(FrontendGroupInformation[FrontendID].FrontendGroupID,pLockStatus); 
			 }

	         OSPorting_Frontend_MutualExclusionKeyRelease(FrontendOperationMutexKeyID);
		  }
	   }
	}

	return Result;
}



BOOL Frontend_GetStatistics(ULONG FrontendID,PSTATISTICS_INFORMATION pStatistics)
{
	BOOL Result = FALSE;

    if(FrontendInitializationFlag)
	{
	   if(FrontendID < MAX_FRONTEND_RECEIVER_GROUP_NUMBER)
	   { 	   
		  if(pStatistics)
		  {
             OSPorting_Frontend_MutualExclusionKeyLock(FrontendOperationMutexKeyID);

	         if(FrontendGroupInformation[FrontendID].ActiveFlag)
			 {
                Result = Sms4470CoreAPI_GetStatistics(FrontendGroupInformation[FrontendID].FrontendGroupID,pStatistics); 
			 }

	         OSPorting_Frontend_MutualExclusionKeyRelease(FrontendOperationMutexKeyID);
		  }
	   }
	}

	return Result;
}
BOOL Frontend_SetupChannelDVBT2PlpID(ULONG FrontendID,UBYTE PlpID)
{
	BOOL Result = FALSE;

    if(FrontendInitializationFlag)
	{
	   if(FrontendID < MAX_FRONTEND_RECEIVER_GROUP_NUMBER)
	   { 	   
          OSPorting_Frontend_MutualExclusionKeyLock(FrontendOperationMutexKeyID);

	      if(FrontendGroupInformation[FrontendID].ActiveFlag)
		  {
             if(Sms4470CoreAPI_SetupChannelDVBT2PlpID(FrontendGroupInformation[FrontendID].FrontendGroupID,PlpID) == TRUE)
			 {
				FrontendGroupInformation[FrontendID].LastSetupDVBT2PlpID = PlpID;
							   
			    Result = TRUE;
			 }
		  }

	      OSPorting_Frontend_MutualExclusionKeyRelease(FrontendOperationMutexKeyID);
	   }
	}

	return Result;
}
#ifdef FRONTEND_ENABLE_REGULAR_TIMER_CALLBACK
BOOL Frontend_TimerCallback(ULONG FrontendID)                // if enable the "FRONTEND_ENABLE_REGULAR_TIMER_CALLBACK" compiler option,
{                                                            // then this function should be called every 250ms or 500ms once by a timer or task   
	BOOL Result = FALSE;

    if(FrontendInitializationFlag)
	{
	   if(FrontendID < MAX_FRONTEND_RECEIVER_GROUP_NUMBER)
	   { 	   
          OSPorting_Frontend_MutualExclusionKeyLock(FrontendOperationMutexKeyID);

	      if(FrontendGroupInformation[FrontendID].ActiveFlag)
		  {
             Result = Sms4470CoreAPI_TimerCallback(FrontendGroupInformation[FrontendID].FrontendGroupID);
		  }

	      OSPorting_Frontend_MutualExclusionKeyRelease(FrontendOperationMutexKeyID);
	   }
	}

	return Result;
}
#endif
//***********************************************************************************************
//***********************************************************************************************
//***********************************************************************************************
