#ifndef _DIGIBESTSMS4470CORE_H_
#define _DIGIBESTSMS4470CORE_H_

#define SMS4470_SERIAL_TRANSPORT_STREAM_OUTPUT                  // compiler option
//#define SMS4470_DUAL_MODE_ENABLE_WORKING_AROUND               // compiler option
//#define SMS_BIGENDIAN                                         // compiler option

#define MSG_HDR_DEFAULT_DYNAMIC_MSG	        0x0000	// Message is dynamic 
#define MSG_HDR_FLAG_STATIC_MSG             0x0001

#define SMS_HOST_ID_BASE                    100
#define SMS_HOST_LIB                        (SMS_HOST_ID_BASE + 50)
#define SMS_HOST_LIB_INTERNAL               (SMS_HOST_ID_BASE + 51)
#define SMS_HOST_LIB_INTERNAL2              (SMS_HOST_ID_BASE + 52)

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

#define HIF_TASK					        11

#define CommonCommandWaitResponseLoop       200 //150
#define SpecialCommandWaitResponseLoop      200
#define CriticalCommandWaitResponseLoop     200 //250

#define SIANO_IIC_TX_BUFFER_SIZE            0x200
#define SIANO_IIC_RX_BUFFER_SIZE            0x1200

#define SMS_MAX_PAYLOAD_SIZE	            240

#define DVBT_BDA_CONTROL_MSG_ID             201


#define I2C_SEC_CTR				(1)			// for I2C polling
#define SMS_GPIO_NONE			(32)		// for I2C polling
#define	SPI_PULSE_WIDTH			(20)

//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------

// Endianness swap macros
#ifdef SMS_BIGENDIAN

#define SMS_ENDIAN_SWAP16(x)										\
					( (((x) >> 8) & 0x00FF) |					\
					  (((x) << 8) & 0xFF00) )

#define SMS_ENDIAN_SWAP32(x)										\
					( (((x) >> 24) & 0x000000FF) |				\
					  (((x) >>  8) & 0x0000FF00) |				\
					  (((x) <<  8) & 0x00FF0000) |				\
					  (((x) << 24) & 0xFF000000) )

#else

#define SMS_ENDIAN_SWAP16(x)	(x)
#define SMS_ENDIAN_SWAP32(x)	(x)

#endif

//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------

typedef enum
{
    SMS4470_DVBT_DEMODULATION_MODE,
	SMS4470_DVBT2_DEMODULATION_MODE,
	UNKNOWN_SMS4470_DEMODULATION_MODE

}SMS4470_DVBT_DVBT2_DUAL_WORKING_MODE_DEMODULATION_MODE;

typedef enum MsgTypes_E
{
	MSG_TYPE_BASE_VAL = 500,

	// Multi2 Values
	MSG_SMS_DECRYPT_MULTI2_INIT_REQ	 = 501,	// Initialize HW Multi2
											// Format:	32 byte - system key, 8 byte - CBC
											// Direction: Host->SMS
	MSG_SMS_DECRYPT_MULTI2_INIT_RES	 = 502,	// The response to MSG_SMS_TRMP_MULTI2_INIT_REQ
											// Format:	4 byte - status
											// Direction: SMS->Host			

	// Meron ATSC-MH Reusabe Values
	MSG_SMS_ATSC_GET_KEYS_REQ = 501,				//
	MSG_SMS_ATSC_GET_KEYS_RES = 502,				//


	MSG_SMS_GET_VERSION_REQ = 503,			// Get version
											// Format: None
											// Direction: Host->SMS

	MSG_SMS_GET_VERSION_RES = 504,			// The response to MSG_SMS_GET_VERSION_REQ
											// Format:	8-bit - Version string
											// Direction: SMS->Host

	MSG_SMS_MULTI_BRIDGE_CFG	= 505,		// Multi bridge configuration message
											// Format: 
											//		32 Bit Config type
											//		Rest - depends on config type.

	MSG_SMS_GET_WIFI_STATISTICS_REQ	= 506,	//	This message requests WiFi statistics from from the FW.
	MSG_SMS_GET_WIFI_STATISTICS_RES	= 507,	//	This message requests WiFi statistics from from the FW.

	MSG_SMS_CHARGER_NOTIFICATION_IND = 508,		// Charger info notification
											// Format:	32 bits, boolean type, 1 = charger connected
											//			0 = charger disconnected.
											//			32 bits, contains the current supplied by the charger.
											//          Values: 2100mA, 1000mA, 500mA, 100mA, 0
											// Direction: FW --> Host.

	MSG_SMS_GPIO_SET_LEVEL_REQ = 509,		// Set GPIO level high / low
											// Format: Data[0] = UINT32 PinNum
											//		   Data[1] = UINT32 NewLevel
											// Direction: Host-->FW

	MSG_SMS_GPIO_SET_LEVEL_RES = 510,		// The response to MSG_SMS_GPIO_SET_LEVEL_REQ
											// Direction: FW-->Host

	MSG_SMS_GPIO_GET_LEVEL_REQ = 511,		// Get GPIO level high / low
											// Format: Data[0] = UINT32 PinNum
											//		   Data[1] = 0
											// Direction: Host-->FW
											  
	MSG_SMS_GPIO_GET_LEVEL_RES = 512,		// The response to MSG_SMS_GPIO_GET_LEVEL_REQ
											// Direction: FW-->Host

	MSG_SMS_EEPROM_BURN_IND = 513,				//

	MSG_SMS_LOG_ENABLE_CHANGE_REQ = 514,	// Change the state of (enable/disable) log messages flow from SMS to Host (MSG_SMS_LOG_ITEM)
											// Format: 32-bit address value for g_log_enable
											// Direction: Host->SMS

	MSG_SMS_LOG_ENABLE_CHANGE_RES = 515,	// A reply to MSG_SMS_LOG_ENABLE_CHANGE_REQ
											// Format: 32-bit address value for g_log_enable
											// Direction: SMS->Host

	MSG_SMS_SET_MAX_TX_MSG_LEN_REQ = 516,	// Set the maximum length of a receiver message
											// Format: 32-bit value of length in bytes, must be modulo of 4
	MSG_SMS_SET_MAX_TX_MSG_LEN_RES = 517,	// ACK/ERR for MSG_SMS_SET_MAX_TX_MSG_LEN_REQ

	MSG_SMS_SPI_HALFDUPLEX_TOKEN_HOST_TO_DEVICE	= 518,	// SPI Half-Duplex protocol
	MSG_SMS_SPI_HALFDUPLEX_TOKEN_DEVICE_TO_HOST	= 519,  //

	// DVB-T MRC background scan messages
	MSG_SMS_BACKGROUND_SCAN_FLAG_CHANGE_REQ	= 520, 
	MSG_SMS_BACKGROUND_SCAN_FLAG_CHANGE_RES = 521, 
	MSG_SMS_BACKGROUND_SCAN_SIGNAL_DETECTED_IND		= 522,  
	MSG_SMS_BACKGROUND_SCAN_NO_SIGNAL_IND			= 523,  

	MSG_SMS_CONFIGURE_RF_SWITCH_REQ		= 524,
	MSG_SMS_CONFIGURE_RF_SWITCH_RES		= 525,

	MSG_SMS_MRC_PATH_DISCONNECT_REQ		= 526,
	MSG_SMS_MRC_PATH_DISCONNECT_RES		= 527,


	MSG_SMS_RECEIVE_1SEG_THROUGH_FULLSEG_REQ = 528, // Application: ISDB-T on SMS2270
													// Description: In full segment application, 
													//				enable reception of 1seg service even if full segment service is not received
	MSG_SMS_RECEIVE_1SEG_THROUGH_FULLSEG_RES = 529, // Application: ISDB-T on SMS2270
													// Description: In full segment application, 
													//				enable reception of 1seg service even if full segment service is not received
	MSG_SMS_RECEIVE_VHF_VIA_VHF_INPUT_REQ = 530,	// Application: All on SMS2270
													// Enable VHF signal (170-240MHz) via VHF input
	
	MSG_SMS_RECEIVE_VHF_VIA_VHF_INPUT_RES = 531,	// Application: All on SMS2270
													// Enable VHF signal (170-240MHz) via VHF input

	
	MSG_SMS_PROD_TEST_RESULT_REQ	= 532,	
													

	MSG_WR_REG_RFT_REQ   =533,			// Write value to a given RFT register
										// Format: 32-bit address of register, following header
										//		   32-bit of value, following address
										// Direction: Host->SMS

	MSG_WR_REG_RFT_RES   =534,			// Response to MSG_WR_REG_RFT_REQ message
										// Format: Status of write operation, following header
										// Direction: SMS->Host

	MSG_RD_REG_RFT_REQ   =535,			// Read the value of a given RFT register
										// Format: 32-bit address of the register, following header
										// Direction: Host->SMS

	MSG_RD_REG_RFT_RES   =536,			// Response to MSG_RD_REG_RFT_RES message
										// Format: 32-bit value of register, following header
										// Direction: SMS->Host

	MSG_RD_REG_ALL_RFT_REQ=537,			// Read all 16 RFT registers
										// Format: N/A (nothing after the header)
										// Direction: Host->SMS

	MSG_RD_REG_ALL_RFT_RES=538,			// Response to MSG_RD_REG_ALL_RFT_REQ message
										// Format: For each register, 32-bit address followed by 32-bit value (following header)
										// Direction: SMS->Host

	MSG_HELP_INT          =539,			// Internal (SmsMonitor) message
										// Format: N/A (nothing after header)
										// Direction: Host->Host

	MSG_RUN_SCRIPT_INT    =540,			// Internal (SmsMonitor) message
										// Format: Name of script(file) to run, immediately following header
										// direction: N/A

	MSG_SMS_EWS_INBAND_REQ = 541,		//	Format: UINT32 Data[0]: 
										//	0 - EWS packets are not sent
										//	1 - when EWS exists EWS messages are sent in-band with the data, over TS packets 
										//	2 - data is not sent until EWS achieved. Ones achieved, output data as regular
										//	UINT32 Data[1] (optional): 	
										//	0 - don't set GPIO
										//	1 - set GPIO for each EWS event
										//	2 - set GPIO only if EWS is about earthquake. Also enable AC buffer reading
										//	UINT32 Data[2] (optional):
										//	GPIO number (relevant only if Data[1] is not 0)

	MSG_SMS_EWS_INBAND_RES = 542,		// Description: Response to MSG_SMS_EWS_INBAND_REQ
	
	MSG_SMS_RFS_SELECT_REQ = 543,		// Application: ISDB-T on SMS2130
										// Description: select RFS resistor value (if the HW of two 60kohm paralel resistor exist)
										// Format: Data[0] = UINT32 GPIO number, Data[1] = UINT32 selected RFS value, 0: select 30kohm, 1: select 60kohm
										// Direction: Host-->FW
										 
	MSG_SMS_RFS_SELECT_RES = 544,		// Application: ISDB-T on SMS2130
										// Description: Response to MSG_SMS_RFS_SELECT_REQ
										// Direction: FW-->Host
	
	MSG_SMS_MB_GET_VER_REQ = 545,			// 
	MSG_SMS_MB_GET_VER_RES = 546,			//  
	MSG_SMS_MB_WRITE_CFGFILE_REQ = 547,		// 
	MSG_SMS_MB_WRITE_CFGFILE_RES = 548,		//
	MSG_SMS_MB_READ_CFGFILE_REQ = 549,		// 
	MSG_SMS_MB_READ_CFGFILE_RES = 550,		//
	MSG_SMS_KITA_DELAY_IPEF = 551,			// Direction: Host->FW, delay iperf task

	MSG_SMS_RD_MEM_REQ    =552,			// A request to read address in memory
										// Format: 32-bit of address, followed by 32-bit of range (following header)
										// Direction: Host->SMS

	MSG_SMS_RD_MEM_RES    =553,			// The response to MSG_SMS_RD_MEM_REQ
										// Format: 32-bit of data X range, following header
										// Direction: SMS->Host

	MSG_SMS_WR_MEM_REQ    =554,			// A request to write data to memory
										// Format:	32-bit of address
										//			32-bit of range (in bytes)
										//			32-bit of value
										// Direction: Host->SMS

	MSG_SMS_WR_MEM_RES    =555,			// Response to MSG_SMS_WR_MEM_REQ
										// Format: 32-bit of result
										// Direction: SMS->Host

	MSG_SMS_UPDATE_MEM_REQ = 556,
	MSG_SMS_UPDATE_MEM_RES = 557,
	
	MSG_SMS_ISDBT_ENABLE_FULL_PARAMS_SET_REQ = 558, // Application: ISDB-T on SMS2270
													// Description: A request to enable the recpetion of mode 1, 2 
													// and guard 1/32 which are disabled by default
	MSG_SMS_ISDBT_ENABLE_FULL_PARAMS_SET_RES = 559, // Application: ISDB-T on SMS2270
													// Description: A response to MSG_SMS_ISDBT_ENABLE_FULL_PARAMS_SET_REQ
													
	
	MSG_SMS_ISDBT_STAT_IND = 560,			//ISDBT MRC send statistics to master
	
	MSG_SMS_RF_TUNE_REQ=561,			// Application: CMMB, DVBT/H 
										// A request to tune to a new frequency
										// Format:	32-bit - Frequency in Hz
										//			32-bit - Bandwidth (in CMMB always use BW_8_MHZ)
										//			32-bit - Crystal (Use 0 for default, always 0 in CMMB)
										// Direction: Host->SMS

	MSG_SMS_RF_TUNE_RES=562,			// Application: CMMB, DVBT/H 
										// A response to MSG_SMS_RF_TUNE_REQ
										// In DVBT/H this only indicates that the tune request
										// was received.
										// In CMMB, the response returns after the demod has determined
										// if there is a valid CMMB transmission on the frequency
										//
										// Format:
										//	DVBT/H:
										//		32-bit Return status. Should be SMSHOSTLIB_ERR_OK.
										//	CMMB:
										//		32-bit CMMB signal status - SMSHOSTLIB_ERR_OK means that the 
										//					frequency has a valid CMMB signal
										// 
										// Direction: SMS->Host
	
	MSG_SMS_ISDBT_ENABLE_HIGH_MOBILITY_REQ = 563,	// Application: ISDB-T on SMS2270
														// Description: A request to enable high mobility performance
														//
	MSG_SMS_ISDBT_ENABLE_HIGH_MOBILITY_RES = 564,	// Application: ISDB-T on SMS2270
														// Description: A response to MSG_SMS_ISDBT_ENABLE_HIGH_MOBILITY_REQ

	MSG_SMS_ISDBT_SB_RECEPTION_REQ = 565,	// Application: ISDB-T on SMS2270
											// Description: A request to receive independent 1seg transmission via tune to 13seg.
	//
	MSG_SMS_ISDBT_SB_RECEPTION_RES = 566,	// Application: ISDB-T on SMS2270
											// Description: A response to MSG_SMS_ISDBT_SB_RECEPTION_REQ

	MSG_SMS_GENERIC_EPROM_WRITE_REQ = 567,		//Write to EPROM.
	MSG_SMS_GENERIC_EPROM_WRITE_RES = 568,					//

	MSG_SMS_GENERIC_EPROM_READ_REQ = 569,			// A request to read from the EPROM
	MSG_SMS_GENERIC_EPROM_READ_RES = 570,			// 

	MSG_SMS_EEPROM_WRITE_REQ=571,		// A request to program the EEPROM
										// Format:	32-bit - Section status indication (0-first,running index,0xFFFFFFFF -last)
										//			32-bit - (optional) Image CRC or checksum
										//			32-bit - Total image length, in bytes, immediately following this DWORD
										//			32-bit - Actual section length, in bytes, immediately following this DWORD
										// Direction: Host->SMS

	MSG_SMS_EEPROM_WRITE_RES=572,		// The status response to MSG_SMS_EEPROM_WRITE_REQ
										// Format:	32-bit of the response
										// Direction: SMS->Host

	//MSG_SMS_RESERVED1 =573, 			// 
	MSG_SMS_CUSTOM_READ_REQ =574,			// 
	MSG_SMS_CUSTOM_READ_RES =575,			// 
	MSG_SMS_CUSTOM_WRITE_REQ =576,			// 
	MSG_SMS_CUSTOM_WRITE_RES =577,			//

	MSG_SMS_INIT_DEVICE_REQ=578,		// A request to init device
										// Format: 32-bit - device mode (DVBT,DVBH,TDMB,DAB, DRM)
										//		   32-bit - Crystal
										//		   32-bit - Clk Division
										//		   32-bit - Ref Division
										// Direction: Host->SMS

	MSG_SMS_INIT_DEVICE_RES=579,		// The response to MSG_SMS_INIT_DEVICE_REQ
										// Format:	32-bit - status
										// Direction: SMS->Host

	MSG_SMS_ATSC_SET_ALL_IP_REQ =580,			
	MSG_SMS_ATSC_SET_ALL_IP_RES =581,			

	MSG_SMS_ATSC_START_ENSEMBLE_REQ = 582,
	MSG_SMS_ATSC_START_ENSEMBLE_RES = 583,

	MSG_SMS_SET_OUTPUT_MODE_REQ	= 584,
	MSG_SMS_SET_OUTPUT_MODE_RES	= 585,

	MSG_SMS_ATSC_IP_FILTER_GET_LIST_REQ = 586,
	MSG_SMS_ATSC_IP_FILTER_GET_LIST_RES = 587,

	//MSG_SMS_RESERVED1 =588,			//

	MSG_SMS_SUB_CHANNEL_START_REQ =589,	// DAB
	MSG_SMS_SUB_CHANNEL_START_RES =590,	// DAB

	MSG_SMS_SUB_CHANNEL_STOP_REQ =591,	// DAB
	MSG_SMS_SUB_CHANNEL_STOP_RES =592,	// DAB

	MSG_SMS_ATSC_IP_FILTER_ADD_REQ = 593,
	MSG_SMS_ATSC_IP_FILTER_ADD_RES = 594,
	MSG_SMS_ATSC_IP_FILTER_REMOVE_REQ = 595,
	MSG_SMS_ATSC_IP_FILTER_REMOVE_RES = 596,
	MSG_SMS_ATSC_IP_FILTER_REMOVE_ALL_REQ = 597,
	MSG_SMS_ATSC_IP_FILTER_REMOVE_ALL_RES = 598,

	MSG_SMS_WAIT_CMD =599,				// Internal (SmsMonitor) message
										// Format: Name of script(file) to run, immediately following header
										// direction: N/A
	//MSG_SMS_RESERVED1 = 600,			// 

	MSG_SMS_ADD_PID_FILTER_REQ=601,		// Application: DVB-T/DVB-H
										// Add PID to filter list
										// Format: 32-bit PID
										// Direction: Host->SMS

	MSG_SMS_ADD_PID_FILTER_RES=602,		// Application: DVB-T/DVB-H
										// The response to MSG_SMS_ADD_PID_FILTER_REQ
										// Format:	32-bit - Status
										// Direction: SMS->Host

	MSG_SMS_REMOVE_PID_FILTER_REQ=603,	// Application: DVB-T/DVB-H
										// Remove PID from filter list
										// Format: 32-bit PID
										// Direction: Host->SMS

	MSG_SMS_REMOVE_PID_FILTER_RES=604,	// Application: DVB-T/DVB-H
										// The response to MSG_SMS_REMOVE_PID_FILTER_REQ
										// Format:	32-bit - Status
										// Direction: SMS->Host

	MSG_SMS_FAST_INFORMATION_CHANNEL_REQ=605,// Application: DAB
										     // A request for a of a Fast Information Channel (FIC)
											 // Direction: Host->SMS

	MSG_SMS_FAST_INFORMATION_CHANNEL_RES=606,// Application: DAB, ATSC M/H
										     // Forwarding of a Fast Information Channel (FIC)
											 // Format:	Sequence counter and FIC bytes with Fast Information Blocks { FIBs  as described in "ETSI EN 300 401 V1.3.3 (2001-05)":5.2.1 Fast Information Block (FIB))
											 // Direction: SMS->Host

	MSG_SMS_DAB_CHANNEL=607,			// Application: All
										// Forwarding of a played channel
										// Format:	H.264
										// Direction: SMS->Host

	MSG_SMS_GET_PID_FILTER_LIST_REQ=608,// Application: DVB-T
										// Request to get current PID filter list
										// Format: None
										// Direction: Host->SMS

	MSG_SMS_GET_PID_FILTER_LIST_RES=609,// Application: DVB-T
										// The response to MSG_SMS_GET_PID_FILTER_LIST_REQ
										// Format:	array of 32-bit of PIDs
										// Direction: SMS->Host

	MSG_SMS_POWER_DOWN_REQ = 610,		// Request from the host to the chip to enter minimal power mode (as close to zero as possible)
										// In Kita App it is used to set up timer to shut down Kita for debug
	MSG_SMS_POWER_DOWN_RES = 611,   	//

	MSG_SMS_ATSC_SLT_EXIST_IND = 612, 		// Application: ATSC M/H
	MSG_SMS_ATSC_NO_SLT_IND  = 613,	// Indication of SLT existence in the parade

	//MSG_SMS_RESERVED1 = 614,			//

	MSG_SMS_GET_STATISTICS_REQ=615,		// Application: DVB-T / DAB / DRM
										// Request statistics information 
										// In DVB-T uses only at the driver level (BDA)
										// Direction: Host->FW

	MSG_SMS_GET_STATISTICS_RES=616,		// Application: DVB-T / DAB / DRM
										// The response to MSG_SMS_GET_STATISTICS_REQ
										// Format:	SmsMsgStatisticsInfo_ST
										// Direction: SMS->Host

	MSG_SMS_SEND_DUMP=617,				// uses for - Dump msgs
										// Direction: SMS->Host

	MSG_SMS_SCAN_START_REQ=618,			// Application: CMMB
										// Start Scan
										// Format:
										//			32-bit - Bandwidth
										//			32-bit - Scan Flags
										//			32-bit - Param Type
										// In CMMB Param type must be 0 - because of CMRI spec, 
										//	and only range is supported.
										//
										// In other standards:
										// If Param Type is SCAN_PARAM_TABLE:
										//			32-bit - Number of frequencies N
										//			N*32-bits - List of frequencies
										// If Param Type is SCAN_PARAM_RANGE:
										//			32-bit - Start Frequency
										//			32-bit - Gap between frequencies
										//			32-bit - End Frequency
										// Direction: Host->SMS

	MSG_SMS_SCAN_START_RES=619,			// Application: CMMB
										// Scan Start Reply
										// Format:	32-bit - ACK/NACK
										// Direction: SMS->Host

	MSG_SMS_SCAN_STOP_REQ=620,			// Application: CMMB
										// Stop Scan
										// Direction: Host->SMS

	MSG_SMS_SCAN_STOP_RES=621,			// Application: CMMB
										// Scan Stop Reply
										// Format:	32-bit - ACK/NACK
										// Direction: SMS->Host

	MSG_SMS_SCAN_PROGRESS_IND=622,		// Application: CMMB
										// Scan progress indications
										// Format:
										//		32-bit RetCode: SMSHOSTLIB_ERR_OK means that the frequency is Locked
										//		32-bit Current frequency 
										//		32-bit Number of frequencies remaining for scan
										//		32-bit NetworkID of the current frequency - if locked. If not locked - 0.
										
	MSG_SMS_SCAN_COMPLETE_IND=623,		// Application: CMMB
										// Scan completed
										// Format: Same as SCAN_PROGRESS_IND

	MSG_SMS_LOG_ITEM = 624,             // Application: All
										// Format:	SMSHOSTLIB_LOG_ITEM_ST.
										// Actual size depend on the number of parameters
										// Direction: Host->SMS

	MSG_SMS_DRM_SDC_IND = 625,			// Application: DRM
										//DRM SDC DATA
										// Direction: FW-->Host

	MSG_SMS_ISDBT_LAYERS_SELECTION_REQ = 626,   //select 2 out of 3 layers for ISDB-T
	MSG_SMS_ISDBT_LAYERS_SELECTION_RES = 627,

	MSG_SMS_DAB_SUBCHANNEL_RECONFIG_REQ = 628,	// Application: DAB
	MSG_SMS_DAB_SUBCHANNEL_RECONFIG_RES = 629,	// Application: DAB

	// Handover - start (630)
	MSG_SMS_HO_PER_SLICES_IND		= 630,		// Application: DVB-H 
												// Direction: FW-->Host

	MSG_SMS_HO_INBAND_POWER_IND		= 631,		// Application: DVB-H 
												// Direction: FW-->Host

	MSG_SMS_MANUAL_DEMOD_REQ		= 632,		// Application: DVB-H
												// Debug msg 
												// Direction: Host-->FW

	MSG_SMS_DRM_FAC_IND				= 633,		// Application: DRM
												//DRM FAC DATA
												// Direction: FW-->Host


	MSG_SMS_DVBH_CLEAR_PID_LIST_REQ  = 634,			//Application: DVBH
													//Clear pid list
													// Direction: Host->SMS

	MSG_SMS_DVBH_CLEAR_PID_LIST_RES  = 635,			//Application: DVBH
													//Clear pid list response
													//Direction: SMS->Host

	MSG_SMS_HO_TUNE_ON_REQ			= 636,		// Application: DVB-H  
	MSG_SMS_HO_TUNE_ON_RES			= 637,		// Application: DVB-H  
	MSG_SMS_HO_TUNE_OFF_REQ			= 638,		// Application: DVB-H 	
	MSG_SMS_HO_TUNE_OFF_RES			= 639,		// Application: DVB-H  
	MSG_SMS_HO_PEEK_FREQ_REQ		= 640,		// Application: DVB-H 
	MSG_SMS_HO_PEEK_FREQ_RES		= 641,		// Application: DVB-H  
	MSG_SMS_HO_PEEK_FREQ_IND		= 642,		// Application: DVB-H 
	// Handover - end (642)

	MSG_SMS_MB_ATTEN_SET_REQ		= 643,		// 
	MSG_SMS_MB_ATTEN_SET_RES		= 644,		//

	MSG_SMS_SET_SINGLE_XTAL_REQ		= 645,		//
	MSG_SMS_SET_SINGLE_XTAL_RES		= 646,		//		
	MSG_SMS_SWITCH_INTERFACE_REQ	= 647,		// Application: iAP Eeprom
												// Switching between UART and USB
												// Format: 32bit - interface (COMM_UART, COMM_USB)
												// Direction: Host/ADR -> Eeprom

	MSG_SMS_SWITCH_INTERFACE_RES	= 648,		// Application: iAP Eeprom
												// Format: 32 bit - Status
												// Direction: Eeprom -> Host/ADR	

	MSG_SMS_ENABLE_STAT_IN_I2C_REQ = 649,		// Application: DVB-T (backdoor)
												// Enable async statistics in I2C polling 
												// Direction: Host->FW

	MSG_SMS_ENABLE_STAT_IN_I2C_RES = 650,		// Application: DVB-T
												// Response to MSG_SMS_ENABLE_STAT_IN_I2C_REQ
												// Format: N/A
												// Direction: FW->Host

   MSG_SMS_BGS_DATA_TX_ENABLE_REQ	= 651,		// Application: All
												// Enable slave device sending 
												// data on BGS SPI interface
												// Format: 1=Enable, 0=Disable
												// Direction: Host->FW

   MSG_SMS_BGS_DATA_TX_ENABLE_RES	= 652,		// Application: All
												// Enable slave device sending 
												// data on BGS SPI interface
												// response.
												// Format: 32-bit - Status
												// Direction: FW->Host

  	MSG_SMS_GET_STATISTICS_EX_REQ   = 653,		// Application: ISDBT / FM
												// Request for statistics 
												// Direction: Host-->FW

	MSG_SMS_GET_STATISTICS_EX_RES   = 654,		// Application: ISDBT / FM
												// Format:
												// 32 bit ErrCode
												// The rest: A mode-specific statistics struct starting
												// with a 32 bits type field.
												// Direction: FW-->Host

	MSG_SMS_SLEEP_RESUME_COMP_IND	= 655,		// Application: All
												// Indicates that a resume from sleep has been completed
												// Uses for Debug only
												// Direction: FW-->Host

	MSG_SMS_SWITCH_HOST_INTERFACE_REQ	= 656,		// Application: All
	MSG_SMS_SWITCH_HOST_INTERFACE_RES	= 657,		// Request the FW to switch off the current host I/F and activate a new one
													// Format: one UINT32 parameter in SMSHOSTLIB_COMM_TYPES_E format

	MSG_SMS_RF_PARAMS_SET_REQ			= 658,			//	Set external LNA/RF switch parameters
														//	Direction: Host ->FW
														//	Format: 32 bit - enum for external LNA/FF switch
														//			32 bit - GPIO
														//			32 bit - Polarity
														//			32 bit - Gain on
														//			32 bit - Gain off
	MSG_SMS_RF_PARAMS_SET_RES			= 659,			//	Response to MSG_SMS_RF_PARAMS_SET_REQ	
														//	Format: 32 bit - status (SMS_S_OK/ SMS_E_BAD_PARAMS)

	MSG_SMS_DATA_DOWNLOAD_REQ		= 660,		// Application: All
												// Direction: Host-->FW

	MSG_SMS_DATA_DOWNLOAD_RES		= 661,		// Application: All
												// Direction: FW-->Host

	MSG_SMS_DATA_VALIDITY_REQ		= 662,		// Application: All
												// Direction: Host-->FW
												
	MSG_SMS_DATA_VALIDITY_RES		= 663,		// Application: All
												// Direction: FW-->Host
												
	MSG_SMS_SWDOWNLOAD_TRIGGER_REQ	= 664,		// Application: All
												// Direction: Host-->FW
												
	MSG_SMS_SWDOWNLOAD_TRIGGER_RES	= 665,		// Application: All
												// Direction: FW-->Host

	MSG_SMS_SWDOWNLOAD_BACKDOOR_REQ	= 666,		// Application: All
												// Direction: Host-->FW
	
	MSG_SMS_SWDOWNLOAD_BACKDOOR_RES	= 667,		// Application: All
												// Direction: FW-->Host

	MSG_SMS_GET_VERSION_EX_REQ		= 668,		// Application: All Except CMMB
												// Direction: Host-->FW

	MSG_SMS_GET_VERSION_EX_RES		= 669,		// Application: All Except CMMB
												// Direction: FW-->Host

	MSG_SMS_CLOCK_OUTPUT_CONFIG_REQ = 670,		// Application: All 
												// Request to clock signal output from SMS
												// Format: 32-bit - Enable/Disable clock signal
												//         32-bit - Requested clock frequency
												// Direction: Host-->FW

	MSG_SMS_CLOCK_OUTPUT_CONFIG_RES = 671,		// Application: All
												// Response to clock signal output config request
												// Format: 32-bit - Status
												// Direction: FW-->Host

  	MSG_SMS_DEVICE_REMOVED_IND	= 672,			// Application: All
												// Description: FW indicate that SMS device from SPI chain has been removed - All applications
												// Direction: FW->Host

	MSG_SMS_DEVICE_DETECTED_IND	= 673,			// Application: All
												// Description: FW indicate that SMS device from SPI chain has been detected - All applications
												// Direction: FW->Host
	
	MSG_SMS_TS_SMOOTH_SET_REQ			= 674,	// Application: DVB-T2
			                                    // Description: SET value of smoothing in TS
			                                    // Direction: Host->FW

	MSG_SMS_TS_SMOOTH_SET_RES			= 675,	// Application: DVB-T2
												// Description: respones to SET value of smoothing in TS
												// Direction: FW->Host

	MSG_SMS_TS_SMOOTH_GET_REQ			= 676,	// Application: DVB-T2
												// Description: GET value of smoothing in TS
												// Direction: Host->FW

	MSG_SMS_TS_SMOOTH_GET_RES			= 677,	//Application: DVB-T2
												// Description: retun value of smoothing in TS
												// Direction: FW->Host

	MSG_SMS_CFG_SPI_GET_REQ             = 678,  // A request get SPI interface as the DATA(!) output interface										// 								// Direction: SMS->Host/Bridg
	MSG_SMS_CFG_SPI_GET_RES             = 679,  // Response to MSG_SMS_CFG_SPI_GET_REQ

	MSG_SMS_CFG_PINS_REQ		    	= 680,  // Application: Sienna
												// Request to Get MB config PINS as seen by device
												// Direction: Host-->FW
	MSG_SMS_CFG_PINS_RES			    = 681,  // Application: Sienna

	MSG_SMS_TASK_STACK_USAGE_REQ		= 682,	// Application: All
												// Description: Get all tasks stack usage watermark
												// Direction: Host-->FW	
	MSG_SMS_TASK_STACK_USAGE_RES		= 683,  // Application: All
												// Description: Response for get all tasks stack usage watermark
												// Direction: Host-->FW	

    MSG_SMS_EWBS_IND = 684,						//Application: Isdbt (Kita)
												//Indication showing the tmmc status and the EID pids
												//Format: [0] = tmmc status, [1] and [2] are the EID pids
												//Direction: FW-->Host

	MSG_SMS_I2C_SET_FREQ_REQ		= 685,		// Application: All 
												// Request to start I2C configure with new clock Frequency
												// Format: 32-bit - Requested clock frequency
												// Direction: Host-->FW

	MSG_SMS_I2C_SET_FREQ_RES		= 686,		// Application: All 
												// Response to MSG_SMS_I2C_SET_FREQ_REQ
												// Format: 32-bit - Status
												// Direction: FW-->Host

	MSG_SMS_GENERIC_I2C_REQ			= 687,		// Application: All 
												// Request to write buffer through I2C
												// Format: 32-bit - device address
												//		   32-bit - write size
												//		   32-bit - requested read size
												//		   n * 8-bit - write buffer

	MSG_SMS_GENERIC_I2C_RES			= 688,		// Application: All 
												// Response to MSG_SMS_GENERIC_I2C_REQ
												// Format: 32-bit - Status
												//		   32-bit - read size
												//         n * 8-bit - read data


	MSG_SMS_TS_CFG_GET_REQ = 689,           // Request to Get TS config from device
								            // Direction: SMS->DEVICE	
	MSG_SMS_TS_CFG_GET_RES = 690,		    // Response to Get TS config from device (return with values)

	MSG_SMS_GET_INTERFACE_REQ			= 691,		// Application: iAP Eeprom
													// Get interface (USB or UART)
													// Format: N/A
													// Direction: Host/ADR -> Eeprom

	MSG_SMS_GET_INTERFACE_RES			= 692,		// Application: iAP Eeprom
													// Format: 32 bit - Status
													//		   32 bit - Interface type according to SMSHOSTLIB_COMM_TYPES_E 
													//			(COMM_USB/COMM_UART)
													// Direction: Eeprom -> Host/ADR
	
	MSG_SMS_DVBT_BDA_DATA			= 693,		// Application: All (BDA)
												// Direction: FW-->Host
												
	MSG_SMS_IAP_DATA_ENABLE_REQ		= 694,		// Application: iAP Eeprom
												//Request to switch the device power mode.
												// Format: 32 bit - The desired power state
												//					0 : Low power mode
												//					1 : High power mode
												// Direction:  Host/ADR -> EEPROM

	MSG_SMS_IAP_DATA_ENABLE_RES		= 695,		// Application: iAP Eeprom
												// Response to switch power mode request.
												// Format: 32 bit - Status
												// Direction: Eeprom -> Host/ADR
	MSG_SMS_RESET_REQ				= 696,		//
	MSG_SMS_RESET_RES				= 697,		//

	//MSG_SMS_RESERVED1				= 698,		//


	MSG_SMS_DATA_MSG				= 699,		// Application: All
												// Direction: FW-->Host

	///  NOTE: Opcodes targeted for Stellar cannot exceed 700
	MSG_TABLE_UPLOAD_REQ			= 700,		// Request for PSI/SI tables in DVB-H
												// Format: 
												// Direction Host->SMS

	MSG_TABLE_UPLOAD_RES			= 701,		// Reply to MSG_TABLE_UPLOAD_REQ
												// Format: 
												// Direction SMS->Host

	// reload without reseting the interface
	MSG_SW_RELOAD_START_REQ			= 702,		// Request to prepare to reload 
	MSG_SW_RELOAD_START_RES			= 703,		// Response to 
	MSG_SW_RELOAD_EXEC_REQ			= 704,		// Request to start reload
	MSG_SW_RELOAD_EXEC_RES			= 705,		// Response to MSG_SW_RELOAD_EXEC_REQ

	MSG_SMS_PRE_RESET_SET_REQ		= 706,		// Application: Siena A01/A1 - All
												// Request to prepare for HW reset
												// Description: Before invoking an HW reset, the Siena A0/A1 required to switch to Ring Oscillator to avoid reset stuck
												// This message is one way with no response, as interfaces are shut down due to RO usage
												// Direction: Host->FW

	MSG_SMS_3_LAYERS_IND			= 707,		// Application: ISDBT, 3 layers
												// Description:Indication sent to host to notice that all si tables are collected
												//and host may choose channel from any layer
												// Direction: FW->Host

	MSG_SMS_3_LAYERS_ADD_PID_REQ	= 708,		// Application: ISDBT, 3 layers
												// Description:host request to transmit layer with specific pid (layer including this pid will be transmitted) 
												// Direction: Host->FW

	MSG_SMS_3_LAYERS_ADD_PID_RES	= 709,		// Application: ISDBT, 3 layers
												// Description:Response to MSG_SMS_3_LAYERS_ADD_PID_REQ
												// Direction: FW->Host

	MSG_SMS_SPI_INT_LINE_SET_REQ	= 710,		//
	MSG_SMS_SPI_INT_LINE_SET_RES	= 711,		//

	MSG_SMS_GPIO_CONFIG_EX_REQ		= 712,		//
	MSG_SMS_GPIO_CONFIG_EX_RES		= 713,		//

	MSG_SMS_GET_MAC_ADDR_REQ		= 714,		// Retrieve Mac address
												// Format: none
												// Direction: Host->SMS
	MSG_SMS_GET_MAC_ADDR_RES 		= 715,		// Response to MSG_SMS_GET_MAC_ADDR_REQ
												// Format: array of 6 bytes contains mac address
												// Direction: SMS->Host

	MSG_SMS_WATCHDOG_ACT_REQ		= 716,		//
	MSG_SMS_WATCHDOG_ACT_RES		= 717,		//

	MSG_SMS_LOOPBACK_REQ			= 718,		//
	MSG_SMS_LOOPBACK_RES			= 719,		//  

	MSG_SMS_RAW_CAPTURE_START_REQ	= 720,  	//
	MSG_SMS_RAW_CAPTURE_START_RES	= 721,  	//

	MSG_SMS_RAW_CAPTURE_ABORT_REQ	= 722,  	//
	MSG_SMS_RAW_CAPTURE_ABORT_RES	= 723,  	//

	MSG_SMS_MERON_SET_WIFI_TIMER_CFG_REQ	=  724,  	// Set Wifi shutdown timer
														// Format: 32-bit - time in seconds
														// Direction: Host->SMS
	MSG_SMS_MERON_SET_WIFI_TIMER_CFG_RES	=  725,  	// Response to MSG_SMS_MERON_SET_WIFI_TIMER_CFG_REQ											
														// Direction: SMS->Host

	MSG_SMS_MERON_GET_WIFI_TIMER_CFG_REQ				=  726,  	// Get Wifi shutdown timer
																	// Direction: Host->SMS
	MSG_SMS_MERON_GET_WIFI_TIMER_CFG_RES				=  727,  	// Response to MSG_SMS_MERON_GET_WIFI_TIMER_CFG_REQ		
																	// Format: 32-bit - time in seconds
																	// Direction: SMS->Host
	

	MSG_SMS_RAW_CAPTURE_COMPLETE_IND = 728, 	//

	MSG_SMS_DATA_PUMP_IND			= 729,  	// USB debug - _TEST_DATA_PUMP 
	MSG_SMS_DATA_PUMP_REQ			= 730,  	// USB debug - _TEST_DATA_PUMP 
	MSG_SMS_DATA_PUMP_RES			= 731,  	// USB debug - _TEST_DATA_PUMP 

	MSG_SMS_FLASH_DL_REQ			= 732,		// A request to program the FLASH
												// Format:	32-bit - Section status indication (0-first,running index,0xFFFFFFFF -last)
												//			32-bit - (optional) Image CRC or checksum
												//			32-bit - Total image length, in bytes, immediately following this DWORD
												//			32-bit - Actual section length, in bytes, immediately following this DWORD
												// Direction: Host->SMS

	MSG_SMS_FLASH_DL_RES			= 733,		// The status response to MSG_SMS_FLASH_DL_REQ
												// Format:	32-bit of the response
												// Direction: SMS->Host

	MSG_SMS_EXEC_TEST_1_REQ			= 734,		// USB debug - _TEST_DATA_PUMP 
	MSG_SMS_EXEC_TEST_1_RES			= 735,  	// USB debug - _TEST_DATA_PUMP 

	MSG_SMS_ENBALE_TS_INTERFACE_REQ	= 736,		// A request set TS interface as the DATA(!) output interface
												// Format:	32-bit - Requested Clock speed in Hz(0-disable)
												//			32-bit - transmission mode (Serial or Parallel)
												// Direction: Host->SMS

	MSG_SMS_ENBALE_TS_INTERFACE_RES	= 737,  	//

	MSG_SMS_SPI_SET_BUS_WIDTH_REQ	= 738,  	//
	MSG_SMS_SPI_SET_BUS_WIDTH_RES	= 739,  	//

	MSG_SMS_SEND_EMM_REQ 			= 740,  	//  Request to process Emm from API
	MSG_SMS_SEND_EMM_RES			= 741,  	//	Response to MSG_SMS_SEND_EMM_REQ

	MSG_SMS_DISABLE_TS_INTERFACE_REQ = 742, 	//
	MSG_SMS_DISABLE_TS_INTERFACE_RES = 743, 	//

	MSG_SMS_IS_BUF_FREE_REQ			= 744,    	//Request to check is CaBuf is free for EMM from API
	MSG_SMS_IS_BUF_FREE_RES			= 745,    	//Response to MSG_SMS_IS_BUF_FREE_RES

	MSG_SMS_EXT_ANTENNA_REQ			= 746,  	//Activate external antenna search algorithm 
	MSG_SMS_EXT_ANTENNA_RES			= 747,  	//confirmation 


	// CMMB Obsolete Values
	MSG_SMS_CMMB_GET_NET_OF_FREQ_REQ_OBSOLETE= 748,		// Obsolete
	MSG_SMS_CMMB_GET_NET_OF_FREQ_RES_OBSOLETE= 749,	    // Obsolete

	// EWS Reusable Values
	MSG_SMS_EWS_MSG_READ_REQ		= 748,		// Read EWS msg (used when EWS alarm received).	
	MSG_SMS_EWS_MSG_READ_RES		= 749,

	MSG_SMS_BATTERY_LEVEL_REQ		= 750,		// Request to get battery charge level
												// Format:	32-bit - Interface(SPI1/SPI2/I2C1_R/I2C1_W)
												//			32-bit - DeviceAdd(for I2C case)
												//			32-bit - RegAdd(for I2C case)
												//			32-bit - WriteVal(for I2C case)
												// In GenAir case, no parameters is needed
												// Direction: Host->SMS
	MSG_SMS_BATTERY_LEVEL_RES		= 751,		// Response to MSG_SMS_BATTERY_LEVEL_REQ
												// Format:	32-bit - status
												//			32-bit - battery level
												// In GenAir case
												//			32-bit - status
												//			32-bit - battery level according to SmsBatteryLevel_EN enum
												//			32-bit - charger status(0-disconnected, 1-connected)
												// Direction: SMS->Host
											
	// CMMB Obsolete Values
	MSG_SMS_CMMB_INJECT_TABLE_REQ_OBSOLETE	= 752,		// Obsolete
	MSG_SMS_CMMB_INJECT_TABLE_RES_OBSOLETE	= 753,		// Obsolete
	MSG_SMS_SET_EXPECTED_TABLES_CHKSUM_REQ	= 752, // Application: DRM
	MSG_SMS_SET_EXPECTED_TABLES_CHKSUM_RES	= 753, // Set expected checksum and version of tables.
	
	MSG_SMS_FM_RADIO_BLOCK_IND		= 754,		// Application: FM_RADIO
												// Description: RDS blocks
												// Format: Data[0] = 	
												// Direction: FW-->Host

	MSG_SMS_HOST_NOTIFICATION_IND 	= 755,		// Application: CMMB
												// Description: F/W notification to host
												// Data[0]:	SMSHOSTLIB_CMMB_HOST_NOTIFICATION_TYPE_ET
												// Direction: FW-->Host

	// CMMB Obsolete Values
	MSG_SMS_CMMB_GET_CONTROL_TABLE_REQ_OBSOLETE	= 756,	// Obsolete
	MSG_SMS_CMMB_GET_CONTROL_TABLE_RES_OBSOLETE = 757,	// Obsolete

	// ISDB-T Reusable Values
	MSG_SMS_SET_MULTI2_SC_KEY_REQ	= 756,	// Application -ISDBT
											// Direction: Host-->FW
	MSG_SMS_SET_MULTI2_SC_KEY_RES	= 757,	// Response to MSG_SMS_SET_MULTI2_SC_KEY_REQ
											// Direction: FW-->Host

	MSG_SMS_FLASH_SET_FACTORY_DEFAULT_REQ		= 758,	// 
	MSG_SMS_FLASH_SET_FACTORY_DEFAULT_RES		= 759,	// 

	MSG_SMS_CMMB_GET_NETWORKS_REQ	= 760,	// Data[0]: Reserved - has to be 0
	MSG_SMS_CMMB_GET_NETWORKS_RES	= 761,	// Data[0]: RetCode
											// Data[1]: Number of networks (N)
											// Followed by N * SmsCmmbNetworkInfo_ST

	MSG_SMS_CMMB_START_SERVICE_REQ	= 762,	// Data[0]: UINT32 Reserved 0xFFFFFFFF (was NetworkLevel)
											// Data[1]: UINT32 Reserved 0xFFFFFFFF (was NetworkNumber)
											// Data[2]: UINT32 ServiceId

	MSG_SMS_CMMB_START_SERVICE_RES	= 763,	// Data[0]: UINT32 RetCode
											// Data[1]: UINT32 ServiceHandle
											// Data[2]: UINT32 Service sub frame index
											//		The index of the sub frame that contains the service
											//      inside the multiplex frame. Usually 0.
											// Data[1]: UINT32 Service ID
											//		The started service ID 

	MSG_SMS_CMMB_STOP_SERVICE_REQ	= 764,	// Data[0]: UINT32 ServiceHandle
	MSG_SMS_CMMB_STOP_SERVICE_RES	= 765,	// Data[0]: UINT32 RetCode

	MSG_SMS_WIFI_CHANNEL_SET_REQ	= 766,	// Request for changing WiFi channel
	MSG_SMS_WIFI_CHANNEL_SET_RES	= 767,	// Request for changing WiFi channel
	

	MSG_SMS_CMMB_ADD_CHANNEL_FILTER_REQ		= 768,	// Data[0]: UINT32 Channel ID
	MSG_SMS_CMMB_ADD_CHANNEL_FILTER_RES		= 769,	// Data[0]: UINT32 RetCode

	MSG_SMS_CMMB_REMOVE_CHANNEL_FILTER_REQ	= 770,	// Data[0]: UINT32 Channel ID
	MSG_SMS_CMMB_REMOVE_CHANNEL_FILTER_RES	= 771,	// Data[0]: UINT32 RetCode

	MSG_SMS_CMMB_START_CONTROL_INFO_REQ		= 772,	// Format:	
													// Data[0]: UINT32 Reserved 0xFFFFFFFF (was NetworkLevel)
													// Data[1]: UINT32 Reserved 0xFFFFFFFF (was NetworkNumber)

	MSG_SMS_CMMB_START_CONTROL_INFO_RES		= 773,	// Format:	Data[0]: UINT32 RetCode

	MSG_SMS_CMMB_STOP_CONTROL_INFO_REQ		= 774,	// Format: No Payload
	MSG_SMS_CMMB_STOP_CONTROL_INFO_RES		= 775,	// Format: Data[0]: UINT32 RetCode

	MSG_SMS_ISDBT_TUNE_REQ			= 776,	// Application Type: ISDB-T
											// Description: A request to tune to a new frequency
											// Format:	Data[0]:	UINT32 Frequency
											//			Data[1]:	UINT32 Bandwidth
											//			Data[2]:	UINT32 Crystal
											//			Data[3]:	UINT32 Segment number
											// Direction: Host->SMS

	MSG_SMS_ISDBT_TUNE_RES			= 777,	// Application Type: ISDB-T
											// Data[0]:	UINT32 RetCode
											// Direction: SMS->Host

	MSG_SMS_ENABLE_POWER_REDUCTION_REQ = 778,
	MSG_SMS_ENABLE_POWER_REDUCTION_RES = 779,

	//Obsolete MRC Indications
	MSG_SMS_MRC_SIGNAL_DETECTED_IND	= 780,  // Obsolete
	MSG_SMS_MRC_NO_SIGNAL_IND		= 781,  // Obsolete


	MSG_SMS_TRANSMISSION_IND		= 782,  // Application Type: DVB-T/DVB-H 
											// Description: Send statistics info using the following structure:
											// TRANSMISSION_STATISTICS_ST
											//	 Data[0] = UINT32 Frequency																
											//   Data[1] = UINT32 Bandwidth				
											//   Data[2] = UINT32 TransmissionMode		
											//   Data[3] = UINT32 GuardInterval			
											//   Data[4] = UINT32 CodeRate				
											//   Data[5] = UINT32 LPCodeRate				
											//   Data[6] = UINT32 Hierarchy				
											//   Data[7] = UINT32 Constellation			
											//   Data[8] = UINT32 CellId					
											//   Data[9] = UINT32 DvbhSrvIndHP			
											//   Data[10]= UINT32 DvbhSrvIndLP			
											//   Data[11]= UINT32 IsDemodLocked			
											// Direction: FW-->Host
												
	MSG_SMS_PID_STATISTICS_IND		= 783,	// Application Type: DVB-H 
											// Description: Send PID statistics info using the following structure:
											// PID_DATA_ST
											//	 Data[0] = UINT32 pid
											//   Data[1] = UINT32 num rows 
											//   Data[2] = UINT32 size  
											//   Data[3] = UINT32 padding_cols
											//   Data[4] = UINT32 punct_cols
											//   Data[5] = UINT32 duration
											//   Data[6] = UINT32 cycle
											//   Data[7] = UINT32 calc_cycle
											//   Data[8] = UINT32 tot_tbl_cnt 
											//   Data[9] = UINT32 invalid_tbl_cnt 
											//   Data[10]= UINT32 tot_cor_tbl
											// Direction: FW-->Host

	MSG_SMS_POWER_DOWN_IND			= 784,	// Application Type: DVB-H 
											// Description: Indicates start of the power down to sleep mode procedure
											//  data[0] - requestId, 
											//  data[1] - message quarantine time
											// Direction: FW-->Host

	MSG_SMS_POWER_DOWN_CONF			= 785,	// Application Type: DVB-H 
											// Description: confirms the power down procedure, 
											// data[0] - requestId, 
											// data[1] - quarantine time
											// Direction: Host-->FW 

	MSG_SMS_POWER_UP_IND			= 786,	// Application Type: DVB-H 
											// Description: Indicates end of sleep mode,       
											// data[0] - requestId
											// Direction: FW-->Host

	MSG_SMS_POWER_UP_CONF			= 787,	// Application Type: DVB-H 
											// Description: confirms the end of sleep mode,    
											// data[0] - requestId
											// Direction: Host-->FW 

	MSG_SMS_CONFIG_SDIO_REQ			= 788,	// Application: CMMB
											// Description: Change working interface to SDIO.
											// Format: Data[0] = UINT32 Num of bits.

	MSG_SMS_CONFIG_SDIO_RES			= 789,	// Application: CMMB
											// Description: response to the previous request
											// Direction: FW-->Host


	MSG_SMS_POWER_MODE_SET_REQ		= 790,	// Application: DVB-H 
											// Description: set the inter slice power down (sleep) mode (Enable/Disable)
											// Format: Data[0] = UINT32 sleep mode
											// Direction: Host-->FW 

	MSG_SMS_POWER_MODE_SET_RES		= 791,	// Application: DVB-H
											// Description: response to the previous request
											// Direction: FW-->Host

	MSG_SMS_DEBUG_HOST_EVENT_REQ	= 792,	// Application: CMMB (Internal) 
											// Description: An opaque event host-> FW for debugging internal purposes (CMMB)
											// Format:	data[0] = Event type (enum)
											//			data[1] = Param

	MSG_SMS_DEBUG_HOST_EVENT_RES	= 793,	// Application: CMMB (Internal)
											// Description: Response. 
											// Format:  data[0] = RetCode, 
											//			data[1] = RetParam


	MSG_SMS_NEW_CRYSTAL_REQ			= 794,	// Application: All 
											// report crystal input to FW
											// Format:  data[0] = UINT32 crystal 
											// Direction: Host-->FW 

	MSG_SMS_NEW_CRYSTAL_RES			= 795,  // Application Type: All 
											// Response to MSG_SMS_NEW_CRYSTAL_REQ
											// Direction: FW-->Host

	MSG_SMS_CONFIG_SPI_REQ			= 796,	// Application: All 
											// Configure SPI interface (also activates I2C slave interface)
											// Format:	data[0] = SPI Controller (UINT32)
											//			data[1] = SPI Mode - Master/Slave (UINT32)
											//			data[2] = SPI Type - Mot/TI (UINT32)
											//			data[3] = SPI Width - 8bit/32bit (UINT32)
											//			data[4] = SPI Clock - in Hz (UINT32)
											// Direction: Host-->FW

	MSG_SMS_CONFIG_SPI_RES			= 797,	// Application: All 
											// Response to MSG_SMS_CONFIG_SPI_RES
											// Direction: FW-->Host

	MSG_SMS_I2C_SHORT_STAT_IND		= 798,	// Application Type: DVB-T/ISDB-T 
											// Format: ShortStatMsg_ST
											//		Data[0] = UINT16 msgType
											//		Data[1] = UINT8	msgSrcId
											//		Data[2] = UINT8	msgDstId
											//		Data[3] = UINT16	msgLength	
											//		Data[4] = UINT16	msgFlags
											//  The following parameters relevant in DVB-T only - in isdb-t should be Zero
											//		Data[5] = UINT32 IsDemodLocked;
											//		Data[6] = UINT32 InBandPwr;
											//		Data[7] = UINT32 BER;
											//		Data[8] = UINT32 SNR;
											//		Data[9] = UINT32 TotalTsPackets;
											//		Data[10]= UINT32 ErrorTSPackets;
											// Direction: FW-->Host

	//MSG_SMS_RESERVED1				= 799,	//

	MSG_SMS_START_IR_REQ			= 800,  // Application: All
											// Description: request to start sampling IR controller
											// Format: Data[0] = irController;
											//		   Data[1] = irTimeout;
											// Direction: Host-->FW

	MSG_SMS_START_IR_RES			= 801,  // Application: All
											// Response to MSG_SMS_START_IR_REQ
											// Direction: FW-->Host

	MSG_SMS_IR_SAMPLES_IND			= 802,  // Application: All
											// Send IR samples to Host
											// Format: Data[] = 128 * UINT32 
											// Direction: FW-->Host
	
	MSG_SMS_CMMB_CA_SERVICE_IND		= 803,	// Format:	UINT32 data[0] UINT32 Indication type, according to
											//					SmsCaServiceIndicationTypes_EN enum
											//			UINT32 data[1] UINT32 Service ID

	MSG_SMS_SLAVE_DEVICE_DETECTED	= 804,  // Application: DVB-T MRC
											// Description: FW indicate that Slave exist in MRC - DVB-T application
											// Direction: FW->Host

	MSG_SMS_INTERFACE_LOCK_IND		= 805,	// Application: All
											// Description: firmware requests that the host does not transmit anything on the interface
											// Direction: FW->Host

	MSG_SMS_INTERFACE_UNLOCK_IND	= 806,	// Application: All
											// Description: firmware signals that the host may resume transmission
											// Direction: FW->Host


	MSG_SMS_BCAS_DEINIT_REQ			= 807,	// 
	MSG_SMS_BCAS_DEINIT_RES			= 808,	// 

	
	MSG_SMS_TCMD_RX_REPORT_IND		= 809,	//

	MSG_SMS_SEND_ROSUM_BUFF_REQ		= 810,  // Application: Rosum
											// Description: Host send buffer to Rosum internal module in FW 
											// Format: msg structure is proprietary to rosum, size can be up to 240
											// Direction: Host-->FW

	MSG_SMS_SEND_ROSUM_BUFF_RES		= 811,  // Application: Rosum
											// Response to MSG_SMS_SEND_ROSUM_BUFF_RES
											// Direction: FW->Host

	MSG_SMS_ROSUM_BUFF				= 812,  // Application: Rosum
											// Description: Rosum internal module in FW  send buffer to Host
											// Format: msg structure is proprietary to rosum, size can be up to 240
											// Direction: FW->Host

	MSG_SMS_GENERIC_DATA_TRANSFER_REQ	= 813,	// Application: All
												// Description: Host send data to FW foe general porposes.
												//				The FW knows where to write the data by data type.
												// Format: String
												// Direction: Host-->FW 

	MSG_SMS_GENERIC_DATA_TRANSFER_RES	= 814,	// Application: All
												//Response to MSG_SMS_GENERIC_DATA_TRANSFER_REQ
												// Direction: FW-->Host  

	MSG_SMS_SET_AES128_KEY_REQ		= 815,  // Application: ISDB-T
											// Description: Host send key for AES128
											// Format: String
											// Direction: Host-->FW

	MSG_SMS_SET_AES128_KEY_RES		= 816,  // Application: ISDB-T
											// Description: response to MSG_SMS_SET_AES128_KEY_REQ
											// Direction: FW-->Host

	MSG_SMS_MBBMS_WRITE_REQ			= 817,	// MBBMS-FW communication message - downstream
	MSG_SMS_MBBMS_WRITE_RES			= 818,	// MBBMS-FW communication message - downstream response
	MSG_SMS_MBBMS_READ_IND			= 819,	// MBBMS-FW communication message - upstream

	MSG_SMS_IQ_STREAM_START_REQ		= 820,  // Application: Streamer
	MSG_SMS_IQ_STREAM_START_RES		= 821,  // Application: Streamer
	MSG_SMS_IQ_STREAM_STOP_REQ		= 822,  // Application: Streamer
	MSG_SMS_IQ_STREAM_STOP_RES		= 823,  // Application: Streamer
	MSG_SMS_IQ_STREAM_DATA_BLOCK	= 824,  // Application: Streamer

	MSG_SMS_GET_EEPROM_VERSION_REQ  = 825,	// Request to get EEPROM version string

	MSG_SMS_GET_EEPROM_VERSION_RES  = 826,	// Response to get EEPROM version string request
											// Format: 32-bit - Status
											//         32-bit - Length of string
											//         N*bytes - EEPROM version string

	MSG_SMS_SIGNAL_DETECTED_IND		= 827,  // Application: DVB-T/ISDB-T/TDMB
											// Description: Indication on good signal - after Tune 
											// Direction: FW-->Host

	MSG_SMS_NO_SIGNAL_IND			= 828,  // Application: DVB-T/ISDB-T/TDMB
											// Description: Indication on bad signal - after Tune 
											// Direction: FW-->Host

	MSG_SMS_LOCAL_SIGNAL_IND		= 829,	// Application: ISDB-T GenAir Internal
											// Description: Sends Signal indications via MsgProc
											// Format: 32-bit - Signal indication type (SIGNAL_DETECTED/NO_SIGNAL)
											// Direction: FW-->FW 

	MSG_SMS_MRC_SHUTDOWN_SLAVE_REQ	= 830,	// Application: DVB-T MRC
											// Description: Power down MRC slave to save power
											// Direction: Host-->FW

	MSG_SMS_MRC_SHUTDOWN_SLAVE_RES	= 831,	// Application: DVB-T MRC
											// Description: response to MSG_SMS_MRC_SHUTDOWN_SLAVE_REQ 
											// Direction: FW-->Host

	MSG_SMS_MRC_BRINGUP_SLAVE_REQ	= 832,	// Application: DVB-T MRC
											// Description: Return back the MRC slave to operation
											// Direction: Host-->FW

	MSG_SMS_MRC_BRINGUP_SLAVE_RES	= 833,  // Application: DVB-T MRC
											// Description: response to MSG_SMS_MRC_BRINGUP_SLAVE_REQ 
											// Direction: FW-->Host

	MSG_SMS_EXTERNAL_LNA_CTRL_REQ   = 834,  // APPLICATION: DVB-T 
											// Description: request from driver to control external LNA
											// Direction: Host-->FW

	MSG_SMS_EXTERNAL_LNA_CTRL_RES   = 835,  // APPLICATION: DVB-T 
											// Description: response to MSG_SMS_EXTERNAL_LNA_CTRL_REQ
											// Direction: FW-->Host

	MSG_SMS_SET_PERIODIC_STATISTICS_REQ		= 836,	// Application: CMMB
													// Description: Enable/Disable periodic statistics.
													// Format:	32 bit enable flag. 0 - Disable, 1- Enable 
													// Direction: Host-->FW

	MSG_SMS_SET_PERIODIC_STATISTICS_RES		= 837,  // Application: CMMB
													// Description: response to MSG_SMS_SET_PERIODIC_STATISTICS_REQ 
													// Direction: FW-->Host

	MSG_SMS_CMMB_SET_AUTO_OUTPUT_TS0_REQ	= 838,	// Application: CMMB
													// Description: Enable/Disable auto output of TS0
													// Format: 32 bit enable flag. 0 - Disable, 1- Enable 
													// Direction: Host-->FW

	MSG_SMS_CMMB_SET_AUTO_OUTPUT_TS0_RES	= 839,  // Application: CMMB
													// Description: response to MSG_SMS_CMMB_SET_AUTO_OUTPUT_TS0_REQ 
													// Direction: FW-->Host
	MSG_SMS_MRC_MODE_CHANGE_REQ		= 840,	
	MSG_SMS_MRC_MODE_CHANGE_RES		= 841,

	MSG_SMS_MRC_RESET_REQ			= 842,
	MSG_SMS_MRC_RESET_RES			= 843,

	MSG_SMS_MF_DMA_ACTIVATE_REQ		= 844,
	MSG_SMS_MF_DMA_ACTIVATE_RES		= 845,

	MSG_SMS_SEND_CIPHER_REQ			= 846,	// Application: CMMB Player protection
											// Description: request to validate that the player is working with SMS device
											// Format  : see SMSHOSTLIB_VALIDATE_ST
	MSG_SMS_SEND_CIPHER_RES			= 847,	// Description: CMMB Player protection response
											// 
											// Format  : 16 bytes of Cipher	
	MSG_SMS_ADD_PID_NOTIFICATION	= 848,	// Description: ISDBT BCAS - notification about automatic added tables pids into pid list
											// Direction: FW-->Host
											// Format  : added pid

	MSG_SMS_EWBS_FILTER_RESULT_IND		= 849, // Application: Kita EWBS (ISDBT)
											//Description: notification if according to the filters set in initialization of ewbs module, the channel is good for ewbs or not
											// Direction: FW-->HOST	
											//Format: 1 for ok, 0 for not pass the filter

	LOCAL_TUNE						= 850,	// Application: DVB-T (Internal)
											// Description: Internal message sent by the demod after tune/resync
											// Direction: FW-->FW	

	LOCAL_IFFT_H_ICI				= 851,  // Application: DVB-T (Internal)
											// Direction: FW-->FW

	MSG_RESYNC_REQ					= 852,	// Application: DVB-T (Internal)
											// Description: Internal resync request used by the MRC master
											// Direction: FW-->FW

	MSG_SMS_CMMB_GET_MRC_STATISTICS_REQ		= 853,	// Application: CMMB (Internal)
													// Description: MRC statistics request (internal debug, not exposed to users)
                                                    // Format 
                                                    // 32-bit   IsDemodLocked;			//!< 0 - not locked, 1 - locked
                                                    // 32-bit   SNR dB
                                                    // 32-bit   RSSI dBm
                                                    // 32-bit   InBandPwr In band power in dBM
                                                    // 32-bit   CarrierOffset Carrier Offset in Hz
													// Direction: Host-->FW
	MSG_SMS_CMMB_GET_MRC_STATISTICS_RES		= 854,	// Description: MRC statistics response (internal debug, not exposed to users)
													// Direction: FW-->Host

	MSG_SMS_LOG_EX_ITEM				= 855,  // Application: All
											// Format:	32-bit - number of log messages
											//			followed by N  SMSHOSTLIB_LOG_ITEM_ST  
											// Direction: FW-->Host

	MSG_SMS_DEVICE_DATA_LOSS_IND	= 856,  // Application: LBS
											// Description: Indication on data loss on the device level
											// Direction: FW-->Host

	MSG_SMS_MRC_WATCHDOG_TRIGGERED_IND	= 857,  // 

	MSG_SMS_USER_MSG_REQ			= 858,  // Application: All
											// Description: Data message for Data Cards internal 
											// Direction: Host-->Data card 

	MSG_SMS_USER_MSG_RES			= 859,  // Application: All 
											// Data message response from Data card to host.
											// Direction: Data card-->Host

	MSG_SMS_SMART_CARD_INIT_REQ		= 860, 	// ISO-7816 SmartCard access routines
	MSG_SMS_SMART_CARD_INIT_RES		= 861,  //
	MSG_SMS_SMART_CARD_WRITE_REQ	= 862,  //
	MSG_SMS_SMART_CARD_WRITE_RES	= 863,  //
	MSG_SMS_SMART_CARD_READ_IND		= 864,  //
	MSG_SMS_BCAS_ERR_NOTIFICATION	= 865,	// Notification about incorrect response from BCAS

	MSG_SMS_TSE_ENABLE_REQ			= 866,	// Application: DVB-T/ISDB-T 
											// Description: Send this command in case the Host wants to handle TS with Error Bit enable
											// Direction: Host-->FW

	MSG_SMS_TSE_ENABLE_RES			= 867,	// Application: DVB-T/ISDB-T 
											// Description: Response to MSG_SMS_TSE_ENABLE_REQ 
											// Direction: FW-->Host

	MSG_SMS_CMMB_GET_SHORT_STATISTICS_REQ	= 868,  // Application: CMMB
													// Description: Short statistics for CMRI standard.
													// Direction: Host-->FW
													// supported only in Venice

	MSG_SMS_CMMB_GET_SHORT_STATISTICS_RES	= 869,  // Description: Short statistics response
													// Format: SMSHOSTLIB_CMMB_SHORT_STATISTICS_ST
													// (No return code).

	MSG_SMS_LED_CONFIG_REQ			= 870,	// Application: DVB-T/ISDB-T
											// Description: uses for LED reception indication
											// Format: Data[0] = UINT32 GPIO number
											// Direction: Host-->FW

	MSG_SMS_LED_CONFIG_RES			= 871,	// Application: DVB-T/ISDB-T
											// Description: Response to MSG_SMS_LED_CONFIG_REQ
											// Direction: FW-->Host

	// Chen Temp for PCTV PWM FOR ANTENNA
	MSG_PWM_ANTENNA_REQ				= 872,  // antenna array reception request
	MSG_PWM_ANTENNA_RES				= 873,  // antenna array reception response
	
	MSG_SMS_CMMB_SMD_SN_REQ			= 874,  // Application: CMMB
											// Description: Get SMD serial number
											// Direction: Host-->FW
											// supported only by SMD firmware 

								
	MSG_SMS_CMMB_SMD_SN_RES			= 875,  // Application: CMMB
											// Description: Get SMD serial number response
											// Format: 
											// UINT32 RetCode
											// UINT8 SmdSerialNumber[SMS_CMMB_SMD_SN_LEN==8]

	MSG_SMS_CMMB_SET_CA_CW_REQ		= 876,  // Application: CMMB
											// Description: Set current and next CA control words 
											//	for firmware descrambler
											// Format: SMSHOSTLIB_CA_CW_PAIR_ST

	MSG_SMS_CMMB_SET_CA_CW_RES		= 877,  // Application: CMMB
											// Description: Set control words response
											// Format: UINT32 RetCode

	MSG_SMS_CMMB_SET_CA_SALT_REQ	= 878,  // Application: CMMB
											// Description: Set Set CA salt key for 
											// firmware descrambler
											// Format: SMSHOSTLIB_CA_SALT_ST
	MSG_SMS_CMMB_SET_CA_SALT_RES	= 879,	// Application: CMMB
											// Description: Set salt keys response
											// Format: UINT32 RetCode
	
	//NSCD injector(Internal debug fw versions only)
	MSG_SMS_NSCD_INIT_REQ			= 880, //NSCD injector(Internal debug fw versions only)
	MSG_SMS_NSCD_INIT_RES			= 881, //NSCD injector(Internal debug fw versions only)
	MSG_SMS_NSCD_PROCESS_SECTION_REQ= 882, //NSCD injector(Internal debug fw versions only)
	MSG_SMS_NSCD_PROCESS_SECTION_RES= 883, //NSCD injector(Internal debug fw versions only)
	MSG_SMS_DBD_CREATE_OBJECT_REQ	= 884, //NSCD injector(Internal debug fw versions only)
	MSG_SMS_DBD_CREATE_OBJECT_RES	= 885, //NSCD injector(Internal debug fw versions only)
	MSG_SMS_DBD_CONFIGURE_REQ		= 886, //NSCD injector(Internal debug fw versions only)
	MSG_SMS_DBD_CONFIGURE_RES		= 887, //NSCD injector(Internal debug fw versions only)
	MSG_SMS_DBD_SET_KEYS_REQ		= 888, //NSCD injector(Internal debug fw versions only)
	MSG_SMS_DBD_SET_KEYS_RES		= 889, //NSCD injector(Internal debug fw versions only)
	MSG_SMS_DBD_PROCESS_HEADER_REQ	= 890, //NSCD injector(Internal debug fw versions only)
	MSG_SMS_DBD_PROCESS_HEADER_RES	= 891, //NSCD injector(Internal debug fw versions only)
	MSG_SMS_DBD_PROCESS_DATA_REQ	= 892, //NSCD injector(Internal debug fw versions only)
	MSG_SMS_DBD_PROCESS_DATA_RES	= 893, //NSCD injector(Internal debug fw versions only)
	MSG_SMS_DBD_PROCESS_GET_DATA_REQ= 894, //NSCD injector(Internal debug fw versions only)
	MSG_SMS_DBD_PROCESS_GET_DATA_RES= 895, //NSCD injector(Internal debug fw versions only) 
	MSG_SMS_NSCD_OPEN_SESSION_REQ	= 896, //NSCD injector(Internal debug fw versions only)
	MSG_SMS_NSCD_OPEN_SESSION_RES	= 897, //NSCD injector(Internal debug fw versions only)
	MSG_SMS_SEND_HOST_DATA_TO_DEMUX_REQ		= 898, // CMMB Data to Demux injector (Internal debug fw versions only)
	MSG_SMS_SEND_HOST_DATA_TO_DEMUX_RES		= 899, // CMMB Data to Demux injector (Internal debug fw versions only)

	//NSCD injector Reusable Codes
	MSG_SMS_WIFI_GET_SSID_AND_MAC_ADDR_REQ	= 880,	//Set the delay to get WiFi statistics for the GenAir Project 
	MSG_SMS_WIFI_GET_SSID_AND_MAC_ADDR_RES	= 881,	//(A lot of Statistics might interfere with WiFi utilization) 

	MSG_SMS_MB_EXPOSE_SLAVE_REQ        = 882,
	MSG_SMS_MB_EXPOSE_SLAVE_RES        = 883,

	MSG_SMS_SET_SSID_REQ			= 884,		// Change SSID
												// Format: payload-null terminated string
												// Direction: Host->SMS
	MSG_SMS_SET_SSID_RES			= 885,		// Response to MSG_SMS_SET_SSID_REQ											
												// Direction: SMS->Host

	MSG_SMS_TCMD_CONT_TX_REQ        = 886,		// Application Type: DVB-T Meron Telec mode
												// Description: Activate TX continuous command for WiFi certification test
												// Format: Data[0]: UINT32 mode
												//		   Data[1]: UINT32 frequency
												//		   Data[2]: UINT32 data rate
												//		   Data[3]: UINT32 tx power
												//		   Data[4]: UINT32 packet size
												// Direction: Host->SMS

	MSG_SMS_TCMD_CONT_TX_RES        = 887,		// Application Type: DVB-T Meron Telec mode
												// Description: Response to MSG_SMS_TCMD_CONT_TX_REQ
												// Direction: SMS->Host

	MSG_SMS_TCMD_CONT_RX_REQ		= 888,		// Application Type: DVB-T Meron Telec mode
												// Description: Activate RX continuous command for WiFi certification test
												// Format: Data[0]: UINT32 action
												//		   Data[1]: UINT32 frequency
												// Direction: Host->SMS

	MSG_SMS_TCMD_CONT_RX_RES       	= 889,		// Application Type: DVB-T Meron Telec mode
												// Description: Response to MSG_SMS_TCMD_CONT_RX_REQ
												// Direction: SMS->Host
	
	//WIFI Internal Messages Carrier
	MSG_SMS_WIFI_INTERNAL_MSG_REQ	= 890,	// Application Type: WiFi (Meron/Kita/Makalu)
											// Description: Message that carries internal WiFi Messages
											// Format: Data[0]: UINT32 MsgWiFiTypes_E ID
											//		   Data[1]: UINT32 Len
											//		   Data[2 - (Len+2)]: [Content]
											// Direction: Host->SMS
	MSG_SMS_WIFI_INTERNAL_MSG_RES	= 891,	// Application Type: WiFi (Meron/Kita/Makalu)
											// Description: Message that carries internal WiFi Messages
											// Format: Data[0]: UINT32 MsgWiFiTypes_E ID
											//		   Data[1]: UINT32 Len
											//		   Data[2 - (Len+2)]: [Content]
											// Direction: SMS->Host

	MSG_SMS_INTERFACE_TX_DROP_SET_REQ = 892,// Application: All
											// Description: Set I/F to drop all Tx packets - 
											//				has no response.
											// Format: Data[0]: interface_tx_packets_drop_val = 1/0

	MSG_SMS_INTERFACE_TX_DROP_SET_RES = 893,// Application: All
											// Description: Set I/F to drop all Tx packets response - 
											//				has no response.
											// Format: Data[0]: interface_tx_packets_drop_val = 1/0
	MSG_SMS_RESET_STATISTICS_REQ		= 894,	//
	MSG_SMS_RESET_STATISTICS_RES		= 895,	//
	MSG_SW_SWITCH_STANDARD_REQ			= 896,
	MSG_SW_SWITCH_STANDARD_RES			= 897,
	//MSG_SMS_RESERVED1				= 898,	//
	//MSG_SMS_RESERVED1				= 899,	//


	//MSG_SMS_RESERVED1 = 900,			//   Note: Stellar ROM limits this number to 700, other chip sets to 900

	/************************************************************************/
	/*********** Siena and forward only *************************************/
	/************************************************************************/

	MSG_SMS_COM_INTERFACE_CHANGE_NOTIFICATION = 901,// This message is not supported by the hostlib and is between the
												    // eeprom and the ADR only.

	MSG_SMS_DVBT2_OPEN_PLP_REQ		=  902, // Application: DVB-T2
											// Description: Select the PLP ID to be handled 
											// Direction: Host-->FW
	MSG_SMS_DVBT2_OPEN_PLP_RES		=  903, // Application: DVB-T2
											// Description: Select the PLP ID to be handled 
											// Direction: FW-->Host

	MSG_SMS_DVBT2_CLOSE_PLP_REQ		=  904, // Application: DVB-T2
											// Description: Select the PLP ID to be closed 
											// Direction: Host-->FW
	MSG_SMS_DVBT2_CLOSE_PLP_RES		=  905, // Application: DVB-T2
											// Description: Select the PLP ID to be closed 
											// Direction: FW-->Host

	MSG_SMS_DVBT2_T2LITE_REQ		=  906,	// Application: DVB-T2
											// Description: Enable/Disable T2 lite
											// Direction: FW-->Host 
	MSG_SMS_DVBT2_T2LITE_RES		=  907,	// Application: DVB-T2
											// Description: Enable/Disable T2 lite
											// Direction: FW-->Host 
	MSG_SMS_DVBT2_TRANSMISSION_IND	=  908, // Application Type: DVB-T2
											// Direction: FW-->Internal (MRC) 

	//MSG_SMS_RESERVED1 = 909,			//

	//MSG_SMS_RESERVED1 = 910,			//
	//MSG_SMS_RESERVED1 = 911,			//

	//MSG_SMS_RESERVED1 = 912,			//
	//MSG_SMS_RESERVED1 = 913,			//

	//MSG_SMS_RESERVED1 = 914,			//
	//MSG_SMS_RESERVED1 = 915,			//

	//MSG_SMS_RESERVED1 = 916,			//
	//MSG_SMS_RESERVED1 = 917,			//

	//MSG_SMS_RESERVED1 = 918,			//
	//MSG_SMS_RESERVED1 = 919,			//

	// ... //

	MSG_LAST_MSG_TYPE				= 1000  // Note: Stellar ROM limits this number to 700, other chip sets to 900, Siena sets to 1000	

}MsgTypes_ET;

/************************************************************************/
/*                         Jacky Han Typedef                            */ 
/************************************************************************/

typedef unsigned char                    __u8;
typedef unsigned short                   __u16;
typedef unsigned long                    __u32;
typedef long                             __s32;
typedef unsigned char                    UINT8;
typedef unsigned short                   UINT16;
typedef unsigned long                    UINT32;
typedef long                             INT32;
typedef unsigned char                    u8;
typedef unsigned short                   u16;
typedef unsigned long                    u32;

/************************************************************************/
/* Defines, types and structures taken fron Siano SmsHostLibTypes.h,    */
/* specifying required API with FW                  */
/************************************************************************/
typedef enum
{
    SMSHOSTLIB_DEVMD_DVBT,
    SMSHOSTLIB_DEVMD_DVBH,
    SMSHOSTLIB_DEVMD_DAB_TDMB,
    SMSHOSTLIB_DEVMD_DAB_TDMB_DABIP,
    SMSHOSTLIB_DEVMD_DVBT_BDA,
    SMSHOSTLIB_DEVMD_ISDBT,
    SMSHOSTLIB_DEVMD_ISDBT_BDA,
    SMSHOSTLIB_DEVMD_CMMB,
    SMSHOSTLIB_DEVMD_RAW_TUNER,
    SMSHOSTLIB_DEVMD_FM_RADIO,
    SMSHOSTLIB_DEVMD_FM_RADIO_BDA,
    SMSHOSTLIB_DEVMD_ATSC,
    SMSHOSTLIB_DEVMD_ATV,
    SMSHOSTLIB_DEVMD_DVBT2,
	SMSHOSTLIB_DEVMD_DRM,
    SMSHOSTLIB_DEVMD_DVBT2_BDA,
    SMSHOSTLIB_DEVMD_MAX,
    SMSHOSTLIB_DEVMD_NONE = 0xFFFFFFFF
} SMSHOSTLIB_DEVICE_MODES_E;

typedef enum SmsTsiMode_E
{
	TSI_SERIAL_MAIN,			//	TSI_SERIAL_ON_SDIO
	TSI_SERIAL_SECONDARY,		//	TSI_SERIAL_ON_HIF
	TSI_PARALLEL_MAIN,			//	TSI_PARALLEL_ON_HIF
	TSI_PARALLEL_SECONDARY,
	TSI_SERIAL_MAIN_WITH_SYNC_EXTEND = 16,
	TSI_MAX_MODE
}SmsTsiMode_ET;

typedef enum SMS_TSI_SIGNAL_ACTIVE
{
    TSI_SIGNALS_ACTIVE_LOW,
    TSI_SIGNALS_ACTIVE_HIGH,
    TSI_MAX_SIG_ACTIVE
}sms_tsi_signal_active_e;

typedef enum SMS_TSI_SENS_POLAR
{
    TSI_SIG_OUT_FALL_EDGE,
    TSI_SIG_OUT_RISE_EDGE,
    TSI_MAX_CLK_POLAR
}sms_tsi_sens_polar_e;

typedef enum SMS_TSI_BIT_ORDER
{
    TSI_BIT7_IS_MSB,
    TSI_BIT0_IS_MSB,
    TSI_MAX_BIT_ORDER
}sms_tsi_bit_order_e;

typedef enum SMS_TSI_FORMAT
{
    TSI_TRANSPARENT,
    TSI_ENCAPSULATED,
    TSI_MAX_FORMAT
}sms_tsi_format_e;

typedef enum SMS_TSI_ERR_ACTIVE
{
    TSI_ERR_NOT_ACTIVE,
    TSI_ERR_ACTIVE,
    TSI_MAX_ERR_ACTIVE
}sms_tsi_err_active_e;

typedef enum SMS_TSI_CLOCK_KEEP_GO
{
    TSI_CLK_STAY_LOW_GO_NO_PKT,
    TSI_CLK_KEEP_GO_NO_PKT,
    TSI_MAX_CLK_ON
}sms_tsi_clock_keep_go_e;

typedef struct SmsMsgHdr_S
{
	UINT16 	msgType;
	UINT8	msgSrcId;
	UINT8	msgDstId;
	UINT16	msgLength;	// Length is of the entire message, including header
	UINT16	msgFlags;
} SmsMsgHdr_ST;

typedef struct SmsMsgData_S
{
	SmsMsgHdr_ST	xMsgHeader;
	UINT32			msgData[1];
} SmsMsgData_ST;

typedef struct SmsMsgData2Args_S
{
	SmsMsgHdr_ST	xMsgHeader;
	UINT32			msgData[2];
} SmsMsgData2Args_ST;

typedef struct SmsMsgData3Args_S
{
	SmsMsgHdr_ST	xMsgHeader;
	UINT32			msgData[3];
} SmsMsgData3Args_ST;

typedef struct SmsMsgData4Args_S
{
	SmsMsgHdr_ST	xMsgHeader;
	UINT32			msgData[4];
} SmsMsgData4Args_ST;

typedef struct SmsDataDownload_S
{
    SmsMsgHdr_ST		xMsgHeader;
    UINT32				MemAddr;
    UINT32				Payload[SMS_MAX_PAYLOAD_SIZE/4];
} SmsDataDownload_ST;

typedef struct SMS_INTR_LINE
{
    SmsMsgHdr_ST     xMsgHeader;
    UINT32           Controler;
    UINT32           GpioNum;
    UINT32           PulseWidth;
} sms_intr_line_t;

typedef enum SmsTsiElectrical_E
{
      TSI_ELEC_LOW,           // slew rate 0.45 V/ns, drive 2.8 mA 
      TSI_ELEC_NORMAL,        // slew rate 1.7 V/ns, drive 7 mA
      TSI_ELEC_HIGH,          // slew rate 3.3 V/ns, drive 10 mA

}SmsTsiElectrical_ET;

typedef enum IoVoltage_E
{
      IOC_VOLTAGE_0,
      IOC_VOLTAGE_1_8,
      IOC_VOLTAGE_3_3
} IoVoltage_ET;

typedef enum
{
	SMSHOSTLIB_TSI_ERR_NOT_ACTIVE	= 0,
	SMSHOSTLIB_TSI_ERR_ACTIVE		= 1
}ERR_ACTIVE_ET;

typedef enum
{
	SMSHOSTLIB_TSI_CLK_STAY_LOW_GO_NO_PKT	= 0,
	SMSHOSTLIB_TSI_CLK_KEEP_GO_NO_PKT		= 1
}CLOCK_KEEP_GO_ET;

#define Electrical_ET  SmsTsiElectrical_ET

typedef struct SMS_MSG_TS_ENABLE
{
    SmsMsgHdr_ST      xMsgHeader;
    UINT32            TsClock;                // 0 - TS Clock Speed in Hz
    UINT32            eTsiMode;               // 1 - TS Mode of operation Serial (on SDIO or HIF Pins), or Parallel
    UINT32            eTsiSignals;            // 2 - Level of Valid, Sync and Error signals when active
    UINT32            nTsiPcktDelay;          // 3 - number of delay bytes between TS packets (for 204bytes mode set to 16)
    UINT32            eTsClockPolarity;       // 4 - Clock edge to sample data
    UINT32            TsBitOrder;             // 5 - Bit order in TS output
    UINT32            EnableControlOverTs;    // 6 - Enable Control messages over TS interface
    UINT32            TsiEncapsulationFormat; // 7 - TS encapsulation method
    UINT32            TsiPaddingPackets;      // 8 - Number of TS padding packets appended to control messages
    UINT32            eTsiElectrical;         // 9 - Set slew rate
    UINT32            IoVoltage;              // 10 - Set IO voltage
	UINT32            eTsiErrActive;          // 11 - Set ErrActive status
	UINT32            eTsiClockKeepGo;        // 12 - Set TS clock keep go with no packet or not

} SmsTsEnable_ST;

typedef struct ShortStatMsg_S
{
    SmsMsgHdr_ST     xMsgHeader;
	//Statistics parameters
    UINT32           IsDemodLocked;
    UINT32           InBandPwr;
    UINT32           BER;
    UINT32           SNR;
    UINT32           TotalTsPackets;
    UINT32           ErrorTSPackets;
} ShortStatMsg_ST; 

typedef struct Short_Statistics_S
{
    //Statistics parameters
    UINT32 IsDemodLocked;
    UINT32 InBandPwr;
    UINT32 BER;
    UINT32 SNR;
    UINT32 TotalTSPackets;
    UINT32 ErrorTSPackets;
} Short_Statistics_ST;

//! DVBT Statistics
typedef struct TRANSMISSION_STATISTICS_S
{
	UINT32 Frequency;				//!< Frequency in Hz
	UINT32 Bandwidth;				//!< Bandwidth in MHz
	UINT32 TransmissionMode;		//!< FFT mode carriers in Kilos
	UINT32 GuardInterval;			//!< Guard Interval from SMSHOSTLIB_GUARD_INTERVALS_ET
	UINT32 CodeRate;				//!< Code Rate from SMSHOSTLIB_CODE_RATE_ET
	UINT32 LPCodeRate;				//!< Low Priority Code Rate from SMSHOSTLIB_CODE_RATE_ET
	UINT32 Hierarchy;				//!< Hierarchy from SMSHOSTLIB_HIERARCHY_ET
	UINT32 Constellation;			//!< Constellation from SMSHOSTLIB_CONSTELLATION_ET

	// DVB-H TPS parameters
	UINT32 CellId;					//!< TPS Cell ID in bits 15..0, bits 31..16 zero; if set to 0xFFFFFFFF cell_id not yet recovered
	UINT32 DvbhSrvIndHP;			//!< DVB-H service indication info, bit 1 - Time Slicing indicator, bit 0 - MPE-FEC indicator
	UINT32 DvbhSrvIndLP;			//!< DVB-H service indication info, bit 1 - Time Slicing indicator, bit 0 - MPE-FEC indicator
	UINT32 IsDemodLocked;			//!< 0 - not locked, 1 - locked

}TRANSMISSION_STATISTICS_ST;

typedef struct RECEPTION_STATISTICS_S
{
	UINT32 IsRfLocked;				//!< 0 - not locked, 1 - locked
	UINT32 IsDemodLocked;			//!< 0 - not locked, 1 - locked
	UINT32 IsExternalLNAOn;			//!< 0 - external LNA off, 1 - external LNA on

	UINT32 ModemState;				//!< from SMSHOSTLIB_DVB_MODEM_STATE_ET
	INT32  SNR;						//!< dB
	UINT32 BER;						//!< Post Viterbi BER [1E-5]
	UINT32 BERErrorCount;			//!< Number of erroneous SYNC bits.
	UINT32 BERBitCount;				//!< Total number of SYNC bits.
	UINT32 TS_PER;					//!< Transport stream PER, 0xFFFFFFFF indicate N/A
	UINT32 MFER;					//!< DVB-H frame error rate in percentage, 0xFFFFFFFF indicate N/A, valid only for DVB-H
	INT32  RSSI;					//!< dBm
	INT32  InBandPwr;				//!< In band power in dBM
	INT32  CarrierOffset;			//!< Carrier Offset in bin/1024
	UINT32 ErrorTSPackets;			//!< Number of erroneous transport-stream packets
	UINT32 TotalTSPackets;			//!< Total number of transport-stream packets

	INT32  RefDevPPM;
	INT32  FreqDevHz;

	INT32  MRC_SNR;					//!< dB //in non MRC application: maximum dvbt cnt
	INT32  MRC_RSSI;				//!< dBm
	INT32  MRC_InBandPwr;			//!< In band power in dBM, in Non MRC application: dvbt buffer max count. Should be less than 345*188

	UINT32 ErrorTSPacketsAfterReset;			//!< Number of erroneous transport-stream packets from the last reset
	UINT32 TotalTSPacketsAfterReset;			//!< Total number of transport-stream packets from the last reset

}RECEPTION_STATISTICS_ST;

// Statistics information returned as response for SmsLiteMsGetStatistics_Req for DVB applications, SMS1100 and up
typedef struct SMSHOSTLIB_STATISTICS_DVBT_S
{
	// Reception
	RECEPTION_STATISTICS_ST ReceptionData;

	// Transmission parameters
	TRANSMISSION_STATISTICS_ST TransmissionData;

	UINT32 ReceptionQuality;
} SMSHOSTLIB_STATISTICS_DVBT_ST;

//! DVBT2 Statistics
typedef struct TRANSMISSION_STATISTICS_DVBT2_S
{
	UINT32 Frequency;				//!< Frequency in Hz
	UINT32 Bandwidth;				//!< Bandwidth in MHz
	UINT32 res[3];
}TRANSMISSION_STATISTICS_DVBT2_ST;

typedef struct RECEPTION_STATISTICS_DVBT2_S
{
		UINT32	IsModemLocked;				//!< 0 - not locked, 1 - locked
		TRANSMISSION_STATISTICS_DVBT2_ST  txStatistics;
		INT32 carrierOffset;
		INT32 inbandPower;  
		UINT32 extLna;     
		UINT32 totalFrames;          
		INT32  SNR;						//!< dB     
        INT32  RSSI;					//!< dBm         
		UINT32 FER;	
		UINT32 CellId;	
		UINT32 netId;	
		UINT32 receptionQuality;        
		UINT32 bwt_ext;
		UINT32 fftMode;       
		UINT32 guardInterval;       
		UINT32 pilotPattern;       
		UINT32 bitRate;   
		UINT32 extended;
		UINT32 toneReservation;
		UINT32 l1PostSize;
		UINT32 numOfAuxs;
		UINT32 numOfPlps;
		UINT32 liteMode;

		INT32  MRC_SNR;					// !< dB
		UINT32 SNRFullRes;				// !< dB x 65536	
		INT32  MRC_InBandPwr;			// !< In band power in dBM
		INT32  MRC_Rssi;

		UINT8  commonPlpNotSupported;
		UINT8  l1modulation;
		UINT16 numdatasymbols;
		UINT32 res[2];

}RECEPTION_STATISTICS_DVBT2_ST;

typedef struct
{
		UINT32 plpId;
		UINT32 plpType;
		UINT32 plpEfficiencyMode;
		UINT32 dnp;
		UINT32 issyi;
		UINT32 crcErrors;
		UINT32 numOfLdpcIters;
		UINT32 totalNumBBFramesReceived;  // Total number of BB frames received.
		UINT32 totalNumErrBBFramesReceived; // Total number of error BB frames received.
		UINT32 totalNumTsPktsReceived;  // Total number of TS packets received.
		UINT32 totalNumTsPktsTransmitted;  // Total number of TS packets transmitted to the TSI.
		UINT32 totalNumErrTsPktsReceived;  // Total number of error TS packets received.
		UINT32 numOfOverflow; 
		UINT32 numOfUnderflow;
		UINT32 dejitterBufferSize;
		UINT32 totalNumOfPktsInserted;
		UINT32 totalNumTsPktsForwarded;
		UINT32 totalPostLdpcErr;
		UINT32 numTsPktsReceivedAfterReset;			//!< Total number of transport-stream packets from the last reset
		UINT32 numErrTsPktsReceivedAfterReset;		//!< Number of erroneous transport-stream packets from the last reset
		UINT32 res[1];
}  ACTIVE_PLP_STATISTICS_ST;

typedef struct DVBT2_PLP_STATISTICS_DATA_S
{
		UINT32 plpId;	
		UINT32 plpType;	
		UINT32 plpPayloadType;	
		UINT32 ffFlag;		
		UINT32 firstRfIdx;	
		UINT32 firstFrameIdx;  
		UINT32 plpGroupId;		
		UINT32 plpCod ;	
		UINT32 plpMod ;
		UINT32 plpRotation;		
		UINT32 plpFecType;	
		UINT32 plpNumBlocksMax;	
		UINT32 frameInterval;		
		UINT32 timeIlLength;	
		UINT32 timeIlType;	
		UINT32 inbandA_Flag;		
		UINT32 inbandB_Flag;	
		UINT32 plpMode;	
		UINT32 staticFlag;	
		UINT32 staticPaddingFlag;	
		UINT32 res[3];
} DVBT2_PLP_STATISTICS_DATA_ST;

typedef struct PLP_DATA_S
{
	DVBT2_PLP_STATISTICS_DATA_ST plpStatistics;

}PLP_DATA_ST;

typedef struct DVBT2_GENERAL_INFO_S
{
	UINT32 smoothing;
	UINT32 res[3];
}DVBT2_GENERAL_INFO_ST;

#define	DVBT2_MAX_PLPS_LITE												(8)
#define	DVBT2_ACTIVE_PLPS_LITE										    (2)

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

typedef struct SMSHOSTLIB_ISDBT_LAYER_STAT_S
{
	// Per-layer information
	UINT32 CodeRate;			//!< Code Rate from SMSHOSTLIB_CODE_RATE_ET, 255 means layer does not exist
	UINT32 Constellation;		//!< Constellation from SMSHOSTLIB_CONSTELLATION_ET, 255 means layer does not exist
	UINT32 BER;					//!< Post Viterbi BER [1E-5], 0xFFFFFFFF indicate N/A
	UINT32 BERErrorCount;		//!< Post Viterbi Error Bits Count
	UINT32 BERBitCount;			//!< Post Viterbi Total Bits Count
	UINT32 PreBER; 				//!< Pre Viterbi BER [1E-5], 0xFFFFFFFF indicate N/A
	UINT32 TS_PER;				//!< Transport stream PER [%], 0xFFFFFFFF indicate N/A
	UINT32 ErrorTSPackets;		//!< Number of erroneous transport-stream packets
	UINT32 TotalTSPackets;		//!< Total number of transport-stream packets
	UINT32 TILdepthI;			//!< Time interleaver depth I parameter, 255 means layer does not exist
	UINT32 NumberOfSegments;	//!< Number of segments in layer A, 255 means layer does not exist
	UINT32 TMCCErrors;			//!< TMCC errors

} SMSHOSTLIB_ISDBT_LAYER_STAT_ST;

typedef struct SMSHOSTLIB_STATISTICS_ISDBT_S
{
	UINT32 StatisticsType;			//!< Enumerator identifying the type of the structure.  Values are the same as SMSHOSTLIB_DEVICE_MODES_E
	//!< This field MUST always first in any statistics structure

	UINT32 FullSize;				//!< Total size of the structure returned by the modem.  If the size requested by
	//!< the host is smaller than FullSize, the struct will be truncated

	// Common parameters
	UINT32 IsRfLocked;				//!< 0 - not locked, 1 - locked
	UINT32 IsDemodLocked;			//!< 0 - not locked, 1 - locked
	UINT32 IsExternalLNAOn;			//!< 0 - external LNA off, 1 - external LNA on

	// Reception quality
	INT32  SNR;						//!< dB
	INT32  RSSI;					//!< dBm
	INT32  InBandPwr;				//!< In band power in dBM
	INT32  CarrierOffset;			//!< Carrier Offset in Hz

	// Transmission parameters
	UINT32 Frequency;				//!< Frequency in Hz
	UINT32 Bandwidth;				//!< Bandwidth in MHz
	UINT32 TransmissionMode;		//!< ISDB-T transmission mode
	UINT32 ModemState;				//!< 0 - Acquisition, 1 - Locked
	UINT32 GuardInterval;			//!< Guard Interval, 1 divided by value
	UINT32 SystemType;				//!< ISDB-T system type (ISDB-T / ISDB-Tsb)
	UINT32 PartialReception;		//!< TRUE - partial reception, FALSE otherwise
	UINT32 NumOfLayers;				//!< Number of ISDB-T layers in the network
	UINT32 SegmentNumber;			//!< Segment number for ISDB-Tsb
	UINT32 TuneBW;					//!< Tuned bandwidth - BW_ISDBT_1SEG / BW_ISDBT_3SEG

	// Per-layer information
	// Layers A, B and C
	SMSHOSTLIB_ISDBT_LAYER_STAT_ST	LayerInfo[3];	//!< Per-layer statistics, see SMSHOSTLIB_ISDBT_LAYER_STAT_ST

	// Interface information
	UINT32 Reserved1;				// Was SmsToHostTxErrors - obsolete .

	// Proprietary information	
	UINT32 ExtAntenna;				// Obsolete field.

	UINT32 ReceptionQuality;

	// EWS
	UINT32 EwsAlertActive;			//!< Signals if EWS alert is currently on

	// LNA on/off					//!< Internal LNA state: 0: OFF, 1: ON
	UINT32 LNAOnOff;
	
	// RF AGC Level					// !< RF AGC level [linear units], full gain = 65535 (20dB)
	UINT32 RfAgcLevel;

	// BB AGC Level
	UINT32 BbAgcLevel;				// !< Baseband AGC level [linear units], full gain = 65535 (71.5dB)

	UINT32 FwErrorsCounter;			// !< FW Application errors - should be always zero
	UINT8 FwErrorsHistoryArr[8];	// !< Last FW errors IDs - first is most recent, last is oldest
									// !< This field was ExtAntenna, and was not used
	INT32  MRC_SNR;					// !< dB
	UINT32 SNRFullRes;				// !< dB x 65536	
	
	UINT32 layer_in_hier1;
	UINT32 layer_in_hier2;

	INT32  MRC_InBandPwr;			// !< In band power in dBM
	INT32  MRC_Rssi;

} SMSHOSTLIB_STATISTICS_ISDBT_ST;

typedef struct SMSHOSTLIB_VERSIONING_S
{
	UINT8			Major;
	UINT8			Minor;
	UINT8			Patch;
	UINT8			FieldPatch;
} SMSHOSTLIB_VERSIONING_ST;

typedef struct SMSHOSTLIB_VERSION_S
{
	UINT16						ChipModel;				//!< e.g. 0x1102 for SMS-1102 "Nova"
	UINT8						Step;					//!< 0 - Step A
	UINT8						MetalFix;				//!< 0 - Metal 0
	UINT8						FirmwareId;				//!< 0xFF - ROM or see #SMSHOSTLIB_DEVICE_MODES_E
	UINT8						SupportedProtocols;		/*!< Bitwise OR combination of supported
	                                                    protocols, see #SMSHOSTLIB_DEVICE_MODES_E */
	SMSHOSTLIB_VERSIONING_ST	FwVer;					//!< Firmware version
	SMSHOSTLIB_VERSIONING_ST	RomVer;					//!< ROM version
	UINT8						TextLabel[34];			//!< Text label
	SMSHOSTLIB_VERSIONING_ST	RFVer;					//!< RF tuner version
	UINT32						PkgVer;                 //!< SMS11xx Package Version	
	UINT32						Reserved[9];            //!< Reserved for future use
                                                        //!< Reserved[0] used for MRC_mode
}SMSHOSTLIB_VERSION_ST;

typedef struct SmsVersionRes_S
{
	SmsMsgHdr_ST				xMsgHeader;
	SMSHOSTLIB_VERSION_ST	    xVersion;
} SmsVersionRes_ST;

struct SmsFirmware_S 
{
	u32 CheckSum;
	u32 Length;
	u32 StartAddress;
	u8 Payload[1];                               //xingyu add
}SmsFirmware_ST;


typedef enum SMSHOSTLIB_MRC_MODE_E
{
	MRC_MODE,
	MULTI_FREQUENCY_MODE,
}SMSHOSTLIB_MRC_MODE_EN;


#define SMSHOSTLIB_DNLD_NO_PRE_TASKS_SHUTDOWN_BASE	10

typedef enum SMSHOSTLIB_RELOAD_TYPE_ET
{	
	SMSHOSTLIB_DNLD_ALL_SLAVES_INCHAIN								= 0,
	SMSHOSTLIB_DNLD_TO_SPECIFIC										= 1,
	SMSHOSTLIB_DNLD_ALL_SLAVES_INCHAIN_ASYNC						= 2,

	SMSHOSTLIB_DNLD_TO_SPECIFIC_NO_PRE_TASKS_SHUTDOWN				= SMSHOSTLIB_DNLD_NO_PRE_TASKS_SHUTDOWN_BASE + SMSHOSTLIB_DNLD_TO_SPECIFIC,
	SMSHOSTLIB_DNLD_ALL_SLAVES_INCHAIN_ASYNC_NO_PRE_TASKS_SHUTDOWN	= SMSHOSTLIB_DNLD_NO_PRE_TASKS_SHUTDOWN_BASE + SMSHOSTLIB_DNLD_ALL_SLAVES_INCHAIN_ASYNC,

	SMSHOSTLIB_MAX_DNLD_MRC

}SMSHOSTLIB_RELOAD_TYPE_E;

//********************************************************************************************
//********************************************************************************************
//********************************************************************************************
//********************************************************************************************
//********************************************************************************************
//********************************************************************************************

#ifdef __cplusplus
extern "C" {
#endif

//**********************************************************************************************
//**********************************************************************************************
//**********************************************************************************************
extern BOOL Sms4470CoreAPI_Initialization();
extern VOID Sms4470CoreAPI_Uninitialization();
extern BOOL Sms4470CoreAPI_HardwareReset(ULONG FrontendGroupID);
extern BOOL Sms4470CoreAPI_SetWorkingMode(ULONG FrontendGroupID,FRONTEND_WORKING_MODE Mode);
extern BOOL Sms4470CoreAPI_SoftwareReset(ULONG FrontendGroupID);
extern BOOL Sms4470CoreAPI_SetFrequencyAndBandwidth(ULONG FrontendGroupID,ULONG RF,BANDWIDTH_TYPE BandWidth);
extern BOOL Sms4470CoreAPI_SetDemodulatorTuningFeature(ULONG FrontendGroupID,DEMODULATOR_TUNING_FEATURE DemodulatorTuningFeature);
extern BOOL Sms4470CoreAPI_CheckLockStatusForChannelSearch(ULONG FrontendGroupID,UBYTE MaxCheckCounter,BOOL* pLockStatus);
extern BOOL Sms4470CoreAPI_CheckLockStatus(ULONG FrontendGroupID,BOOL* pLockStatus);
extern BOOL Sms4470CoreAPI_GetStatistics(ULONG FrontendGroupID,PSTATISTICS_INFORMATION pStatistics);
extern BOOL Sms4470CoreAPI_SetupChannelDVBT2PlpID(ULONG FrontendGroupID,UBYTE PlpID);
#ifdef FRONTEND_ENABLE_REGULAR_TIMER_CALLBACK
extern BOOL Sms4470CoreAPI_TimerCallback(ULONG FrontendGroupID);
#endif
//**********************************************************************************************
//**********************************************************************************************
//**********************************************************************************************
extern BOOL SMS4470_tune(ULONG FrontendGroupID,ULONG Frequency,BANDWIDTH_TYPE BandWidth,DEMODULATOR_TUNING_FEATURE DemodulatorTuningFeature);
extern BOOL SMS4470_get_version(ULONG FrontendGroupID);
extern BOOL SMS4470_dual_mode_download_dvbt2_firmware_data(ULONG FrontendGroupID,UINT8* pFwImage);
extern BOOL SMS4470_dual_mode_download_dvbt_firmware_data(ULONG FrontendGroupID,UINT8* pFwImage);
extern BOOL SMS4470_single_mode_download_firmware_data(ULONG FrontendGroupID,UINT8* pFwImage);
extern BOOL SMS4470_wait_device_redetected(ULONG FrontendGroupID);
extern BOOL SMS4470_set_polling_mode(ULONG FrontendGroupID);
extern BOOL SMS4470_enable_ts_interface(ULONG FrontendGroupID);
extern BOOL SMS4470_send_device_init(ULONG FrontendGroupID,DEMODULATOR_TUNING_FEATURE DemodulatorTuningFeature);
extern UINT32 SMS4470_read_response(ULONG FrontendGroupID,UINT32 response, UINT8* iicRxBuf,UINT16 MaxiicRxBufSize);
extern VOID SMS4470_HandlePerSlicesIndication(PUBYTE pData,RECEPTION_STATISTICS_ST* pReceptionStatistic);
extern BOOL SMS4470_dual_mode_SwitchStandard(ULONG FrontendGroupID);
extern BOOL SMS4470_check_signal(ULONG FrontendGroupID,BOOL* pLockStatus);
extern BOOL SMS4470_GetReceptionStatistics(ULONG FrontendGroupID,PSTATISTICS_INFORMATION pStatistics);
extern BOOL SMS4470_CheckDVBT2PhysicalLayerPipeInformation(ULONG FrontendGroupID,BOOL* pDoesPhysicalLayerPipeInformationExistFlag);
extern BOOL SMS4470_OpenPlp(ULONG FrontendGroupID,UINT32 plpId);
extern BOOL SMS4470_ClosePlp(ULONG FrontendGroupID,UINT32 plpId);
extern BOOL SMS4470_AddPidFilter(ULONG FrontendGroupID,UINT32 PID);
extern BOOL SMS4470_RemovePidFilter(ULONG FrontendGroupID,UINT32 PID);
extern BOOL SMS4470_DVBT2LiteControl(ULONG FrontendGroupID,BOOL T2LiteActiveFlag);
//********************************************************************************************
//********************************************************************************************
//********************************************************************************************
VOID SMS4470ReadResponseWithDelay(ULONG FrontendGroupID,ULONG Milliseconds);
//********************************************************************************************
//********************************************************************************************
//********************************************************************************************
extern VOID SMS4470SysDelay(ULONG Milliseconds);
extern BOOL SMS4470I2CWrite(ULONG FrontendGroupID,PUBYTE pData, ULONG DataLength);
extern BOOL SMS4470I2CWrite_FW(ULONG FrontendGroupID,PUBYTE pData, ULONG DataLength);
extern BOOL SMS4470I2CRead(ULONG FrontendGroupID,ULONG ReadLength, PUBYTE pBuffer);
//**********************************************************************************************
//**********************************************************************************************
//**********************************************************************************************

#ifdef __cplusplus
}
#endif

//********************************************************************************************
//********************************************************************************************
//********************************************************************************************
//********************************************************************************************
//********************************************************************************************
//********************************************************************************************


#endif

