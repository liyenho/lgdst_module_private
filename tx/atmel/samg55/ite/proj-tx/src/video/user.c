#include <asf.h>
#include <string.h>
//#include <unistd.h>
#include "user.h"
#include "delay.h"
#include <stdio.h>
#include <assert.h>
#include "main.h"

extern void twi_sms4470_handler(const uint32_t id, const uint32_t index);
 extern /*static*/ twi_packet_t packet_tx;
 extern twi_packet_t packet_rx;
 //extern volatile bool i2c_read_done;
 extern volatile context_it951x ctx_951x;

static DCtable dc_table[7];
static DCtable ofs_table[7];

static RFGainTable rfGain_table[50];
uint32_t IT9510User_i2cSwitchToADRF6755 (  IT9510INFO*    modulator)
{
	return IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh2_o, 1);
}


uint32_t IT9510User_i2cSwitchToOrion (  IT9510INFO*    modulator)
{
#if USE_UART
	char dbg_msg[80], dbg_len, *pdbg ;
	sprintf(dbg_msg, "enter %s\n", __func__);
	dbg_len = strlen(dbg_msg);
	TRANSFER_LOG
#endif
	IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh7_o, 0);
	IT9510User_delay(20);

	IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh8_o, 1);
	IT9510User_delay(20);

#if USE_UART
	sprintf(pdbg, "exit %s, %x\n", __func__, ModulatorError_NO_ERROR);
	dbg_len += strlen(pdbg);
	TRANSFER_LOG
#endif
	return ModulatorError_NO_ERROR;
}

uint32_t IT9510User_i2cSwitchToOrion2(  IT9510INFO*    modulator)
{
#if USE_UART
	char dbg_msg[80], dbg_len, *pdbg ;
	sprintf(dbg_msg, "enter %s\n", __func__);
	dbg_len = strlen(dbg_msg);
	TRANSFER_LOG
#endif
	IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh8_o, 0);
	IT9510User_delay(20);

	IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh7_o, 1);
	IT9510User_delay(20);

#if USE_UART
	sprintf(pdbg, "exit %s, %x\n", __func__, ModulatorError_NO_ERROR);
	dbg_len += strlen(pdbg);
	TRANSFER_LOG
#endif
	return ModulatorError_NO_ERROR;
}

uint32_t IT9510User_getDeviceType (
	  IT9510INFO*    modulator,
	  uint8_t*		  deviceType
){
	uint32_t error = ModulatorError_NO_ERROR;
	uint8_t temp;
#if USE_UART
	char dbg_msg[80], dbg_len, *pdbg ;
	sprintf(dbg_msg, "enter %s\n", __func__);
	dbg_len = strlen(dbg_msg);
	TRANSFER_LOG
#endif

	error = IT9510_readRegister (modulator, Processor_LINK, 0x4979, &temp);//has eeprom ??
	if((temp == 1) && (error == ModulatorError_NO_ERROR)){
		error = IT9510_readRegister (modulator, Processor_LINK, 0x49D5, &temp);
		if(error == ModulatorError_NO_ERROR){
			*deviceType = temp;
		}else if(temp == 0){ // No eeprom
			*deviceType = IT9510User_DEVICETYPE;
		}
	}
	else
	{
		*deviceType = 1;
	}

#if USE_UART
	sprintf(pdbg, "exit %s, %x\n", __func__, error);
	dbg_len += strlen(pdbg);
	TRANSFER_LOG
#endif
	return(error);
}

uint32_t IT9510User_memoryCopy (
      IT9510INFO*    modulator,
      void*           dest,
      void*           src,
      uint32_t           count
) {
    /*
     *  ToDo:  Add code here
     *
     *  //Pseudo code
     *  memcpy(dest, src, (size_t)count);
     *  return (0);
     */
	memcpy(dest, src, (size_t)count);
    return (ModulatorError_NO_ERROR);
}

uint32_t IT9510User_delay (
      uint32_t           dwMs
) {
    /*
     *  ToDo:  Add code here
     *
     *  //Pseudo code
     *  delay(dwMs);
     *  return (0);
     */
	if(dwMs > 0)
		delay_us(dwMs*250);
    return (ModulatorError_NO_ERROR);
}

uint32_t IT9510User_printf (const char* format,...){

	perror(format);
	return (ModulatorError_NO_ERROR);
}

uint32_t IT9510User_enterCriticalSection (
	void
) {
    /*
     *  ToDo:  Add code here
     *
     *  //Pseudo code
     *  return (0);
     */
    return (ModulatorError_NO_ERROR);
}

uint32_t IT9510User_leaveCriticalSection (
	void
) {
    /*
     *  ToDo:  Add code here
     *
     *  //Pseudo code
     *  return (0);
     */
    return (ModulatorError_NO_ERROR);
}

uint32_t IT9510User_mpegConfig (
      IT9510INFO*    modulator
) {
    /*
     *  ToDo:  Add code here
     *
     */
    return (ModulatorError_NO_ERROR);
}

uint32_t IT9510User_busTx (
      IT9510INFO*    modulator,
      uint32_t           bufferLength,
      uint8_t*           buffer
) {
	 uint32_t     error = Error_I2C_WRITE_FAILED;
	 int32_t 		i, msg[80]; // access buffer
	 dev_access *pt = (dev_access*)msg;
#if USE_UART
	char dbg_msg[80+255*(2+1)], *pdbg;
	int dbg_len;  // can't be char data type
	sprintf(dbg_msg, "enter %s\n", __func__);
	dbg_len = strlen(dbg_msg);
	TRANSFER_LOG
	LOG_BIN_DATA
#endif

	for (i = 0; i < IT9510User_MAXFRAMESIZE; i++) {
		 pt->dcnt = bufferLength;
		 pt->addr = IT951X_ADDRESS;
		 memcpy(pt->data, buffer, bufferLength);
		//IT951X_WRITE:
			TWI_WRITE(pt->addr,pt->data,pt->dcnt)
			//break;
		if (ctx_951x.it951x_err_wr == TWI_SUCCESS) {
			error = ModulatorError_NO_ERROR;
			break;
		}
		IT9510User_delay(10);
	}
#if USE_UART
	sprintf(pdbg, "exit %s, %x\n", __func__, error);
	dbg_len += strlen(pdbg);
	TRANSFER_LOG
#endif
    return (error);
}
uint32_t IT9510User_busTx2 (
      IT9510INFO*    modulator,
      uint32_t           bufferLength,
      uint8_t*           buffer
) {
   return IT9510User_busTx (
      				modulator,
      				bufferLength,
      				buffer);
}


uint32_t IT9510User_busRx (
      IT9510INFO*    modulator,
      uint32_t           bufferLength,
      uint8_t*           buffer
) {
	 uint32_t     error = Error_I2C_READ_FAILED;
	 int32_t 		i, msg[80]; // access buffer
	 dev_access *pr = (dev_access*)msg;
#if USE_UART
	char dbg_msg[80+255*(2+1)], *pdbg;
	int dbg_len ; // can't be char data type
	sprintf(dbg_msg, "enter %s\n", __func__);
	dbg_len = strlen(dbg_msg);
	TRANSFER_LOG
#endif

	 for (i = 0; i < IT9510User_MAXFRAMESIZE; i++) {
		 pr->dcnt = bufferLength;
		 pr->addr = IT951X_ADDRESS;
		//IT951X_READ:
			_TWI_READ_(pr->addr,pr->data,pr->dcnt)
			//break;
		if (ctx_951x.it951x_err_rd == TWI_SUCCESS) {
			error = ModulatorError_NO_ERROR;
			break;
		}
		IT9510User_delay(10);
	}
	 memcpy(buffer, pr->data, bufferLength);
#if USE_UART
	LOG_BIN_DATA
	sprintf(pdbg, "exit %s, %x\n", __func__, error);
	dbg_len += strlen(pdbg);
	TRANSFER_LOG
#endif
	 return (error);
}
uint32_t IT9510User_busRx2 (
      IT9510INFO*    modulator,
      uint32_t           bufferLength,
      uint8_t*           buffer
) {
	 return IT9510User_busRx (
      					modulator,
      					bufferLength,
      					buffer);
}


uint32_t IT9510User_setBus (
      IT9510INFO*    modulator
) {
	/*
     *  ToDo:  Add code here
     *
     *  // If no error happened return 0, else return error code.
     *  return (0);
     */
    uint32_t error = ModulatorError_NO_ERROR;

    return(error);
}

uint32_t IT9510User_Initialization  (
      IT9510INFO*    modulator
) {
	/*
     *  ToDo:  Add code here
     *
     *  // If no error happened return 0, else return error code.
     *  return (0);
     */
#if USE_UART
	char dbg_msg[80], dbg_len, *pdbg ;
	sprintf(dbg_msg, "enter %s\n", __func__);
	dbg_len = strlen(dbg_msg);
	TRANSFER_LOG
#endif
	uint32_t error = 0;
	//uint8_t deviceType = 0;

	modulator->isExtLo = False;
/*
	error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh8_en, 1);	//U/V filter control pin
	if (error) goto exit;
	error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh8_on, 1);
	if (error) goto exit;
*/
	if((modulator->tsInterfaceType == SERIAL_TS_INPUT) || (modulator->tsInterfaceType == PARALLEL_TS_INPUT)){
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh1_o, 1); //RX(IT9133) rest
		if (error) goto exit;

		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh1_en, 1);
		if (error) goto exit;
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh1_on, 1);
		if (error) goto exit;
		IT9510User_delay(5);

		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh5_o, 0); //new U/V filter control pin
		if (error) goto exit;
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh5_en, 1);
		if (error) goto exit;
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh5_on, 1);
		if (error) goto exit;
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh3_en, 1); //new U/V filter control pin
		if (error) goto exit;
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh3_on, 1);
		if (error) goto exit;

		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh4_en, 1);
		if (error) goto exit;
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh4_on, 1);
		if (error) goto exit;
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh6_en, 1);
		if (error) goto exit;
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh6_on, 1);
		if (error) goto exit;
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh7_en, 1);
		if (error) goto exit;
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh7_on, 1);
		if (error) goto exit;
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh8_en, 1);
		if (error) goto exit;
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh8_on, 1);
		if (error) goto exit;

		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh8_o, 0);
		if (error) goto exit;

		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh7_o, 1);
		if (error) goto exit;
	}

	error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh2_en, 1);
	if (error) goto exit;
	error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh2_on, 1);
	if (error) goto exit;


//#ifdef _ORION_CAL_H
	//----- init orion
	//IT9510User_getDeviceType(modulator, &deviceType);
	//if(deviceType == 0xD9 || deviceType == 0x99){
/*		IT9510User_i2cSwitchToOrion(modulator);
		if(modulator->isExtLo == True)
			error = orion_cal_init(modulator, 20000);
		else
			error = orion_cal_init(modulator, 12000);

		if (error) goto exit;
*/
	//}
//#endif
	error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh2_o, 0); //RF out power down
	if (error) goto exit;
	IT9510User_LoadRFGainTable(modulator);
	IT9510User_LoadDCCalibrationTable(modulator);
exit:
#if USE_UART
	sprintf(pdbg, "exit %s, %x\n", __func__, error);
	dbg_len += strlen(pdbg);
	TRANSFER_LOG
#endif
    return (error);

 }


uint32_t IT9510User_Finalize  (
      IT9510INFO*    modulator
) {
	/*
     *  ToDo:  Add code here
     *
     *  // If no error happened return 0, else return error code.
     *  return (0);
     */
#if 0
	uint32_t error = 0;
	if(modulator->busId == Bus_USB)
		error = Usb2_exitDriver((Modulator*)modulator);
	else if(modulator->busId == Bus_9035U2I)
		error = Af9035u2i_exitDriver((Modulator*)modulator);
	else

		error = ModulatorError_INVALID_BUS_TYPE;
#endif
    return (ModulatorError_NO_ERROR);

 }


uint32_t IT9510User_acquireChannel (
	  IT9510INFO*    modulator,
	  uint16_t          bandwidth,
	  uint32_t         frequency
){

	/*
     *  ToDo:  Add code here
     *
     *  // If no error happened return 0, else return error code.
     *  return (0);
     */
	uint32_t error = 0;

	//control U/V filter by gpioh8
/*
	if(frequency < 250000){ // <=250000 v-filter gpio set to Lo
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh8_o, 0);
		if (error) goto exit;

	}else{
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh8_o, 1);
		if (error) goto exit;
	}

	//control new 4CH U/V filter by gpioh3 & 5
	if(frequency <= 150000){ //
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh3_o, 1);
		if (error) goto exit;
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh5_o, 1);
		if (error) goto exit;
	}else if((frequency > 150000)&&(frequency <= 250000)){
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh3_o, 1);
		if (error) goto exit;
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh5_o, 0);
		if (error) goto exit;
	}else if((frequency > 250000)&&(frequency <= 450000)){
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh3_o, 0);
		if (error) goto exit;
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh5_o, 1);
		if (error) goto exit;
	}else if(frequency > 450000){
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh3_o, 0);
		if (error) goto exit;
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh5_o, 0);
		if (error) goto exit;
	}
*/
	uint8_t bIndex = 0;
	int gain_value;

	if (IT9510User_getOutputGainInfo(modulator, frequency + LO_Frequency, &bIndex) == 0)
			{
			// setting output gain
			// analog gain
			gain_value = modulator->rfGainInfo.ptrGaintable[bIndex].alanogGainValue;
			error = IT9510_writeRegister(modulator, Processor_OFDM, 0xFBBC, gain_value);
			if (error) goto exit;

			// digital gain
			gain_value = modulator->rfGainInfo.ptrGaintable[bIndex].digitalGainValue;
			error = IT9510_adjustOutputGain(modulator, &gain_value);
			if (error) goto exit;
		}
exit:
	return (error);
}


uint32_t IT9510User_acquireChannelDual(
	  IT9510INFO*    modulator,
	  uint16_t          bandwidth,
	  uint32_t         frequency
	){

	/*
	*  ToDo:  Add code here
	*
	*  // If no error happened return 0, else return error code.
	*  return (0);
	*/
	uint32_t error = 0;
exit:

	return (error);

}

uint32_t IT9510User_setTxModeEnable (
	  IT9510INFO*            modulator,
	  uint8_t                    enable
) {
	/*
     *  ToDo:  Add code here
     *
     *  // If no error happened return 0, else return error code.
     *  return (0);
     */
#if USE_UART
	char dbg_msg[80], dbg_len, *pdbg ;
	sprintf(dbg_msg, "enter %s\n", __func__);
	dbg_len = strlen(dbg_msg);
	TRANSFER_LOG
#endif
	uint32_t error = ModulatorError_NO_ERROR;
	uint32_t         frequency = 0;
	if(enable){

//#ifdef _ORION_CAL_H
		if(modulator->isExtLo == True)
			frequency =  (modulator->frequency/1000)*1000; // fixed Accuracy issue
		else
			frequency =  modulator->frequency;
		//----- orion cal dc

			IT9510User_i2cSwitchToOrion(modulator);
		//	IT9510User_delay(20);

			//printf("IT9510 orion initialize.\n");
			if(modulator->isExtLo == True)
				error = orion_cal_init(modulator, 20000);
			else
				error = orion_cal_init(modulator, 12000);
			if (error) goto exit;

		// cal dc with orion iqik module, Michael, 20140502

			//IT9510User_delay(200);
			IT9510User_delay(10);
			//printf("IT9510 orion_calDC.\n");
			if(modulator->isExtLo == True)
				error = orion_calDC(modulator, frequency, False);
			else
				error = orion_calDC(modulator, frequency, True);
			if (error) goto exit;

			IT9510User_i2cSwitchToOrion2(modulator);

//#endif

	}else{

	}
	error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh2_o, enable);
	if (error) goto exit;
exit :
#if USE_UART
	sprintf(pdbg, "exit %s, %x\n", __func__, error);
	dbg_len += strlen(pdbg);
	TRANSFER_LOG
#endif
	return (error);
}

uint32_t IT9510User_LoadDCCalibrationTable (
	  IT9510INFO*            modulator
) {
    uint32_t error = ModulatorError_NO_ERROR;
	DCInfo dcInfo;
	uint8_t DCvalue[16];
	uint8_t OFS_I_value[7];
	uint8_t OFS_Q_value[7];
	uint8_t eeprom = 0;
	uint8_t DCcalibration = 0;
	uint8_t index = 0;
#if USE_UART
	char dbg_msg[80], dbg_len, *pdbg ;
	sprintf(dbg_msg, "enter %s\n", __func__);
	dbg_len = strlen(dbg_msg);
	TRANSFER_LOG
#endif
//	uint8_t i =0;

	//-------------set DC Calibration table
	error = IT9510_readRegister (modulator, Processor_LINK, 0x4979, &eeprom);//has eeprom ??
	if (error) goto exit;
	error = IT9510_readRegister (modulator, Processor_LINK, 0x49D6, &DCcalibration);//has DCcalibration ??
	if (error) goto exit;

	if((eeprom ==1) && ((DCcalibration & 0x80) == 0x80)){
		; //puts("found eeprom and chip supported DC calibration as well");
		error = IT9510_readRegisters (modulator, Processor_LINK, 0x49D6, 12, DCvalue);//DC calibration value
		if (error) goto exit;

		error = IT9510_readRegisters (modulator, Processor_LINK, 0x49CC, 5, OFS_I_value);//OFS_I calibration value
		if (error) goto exit;

		error = IT9510_readRegisters (modulator, Processor_LINK, 0x49E2, 5, OFS_Q_value);//OFS_Q calibration value
		if (error) goto exit;

		error = IT9510_readRegister (modulator, Processor_LINK, 0x49C9, &DCvalue[12]);//DC_6_i calibration value
		if (error) goto exit;
		error = IT9510_readRegister (modulator, Processor_LINK, 0x49D1, &DCvalue[13]);//DC_6_q calibration value
		if (error) goto exit;
		error = IT9510_readRegister (modulator, Processor_LINK, 0x49CB, &DCvalue[14]);//DC_7_i calibration value
		if (error) goto exit;
		error = IT9510_readRegister (modulator, Processor_LINK, 0x49E7, &DCvalue[15]);//DC_7_q calibration value
		if (error) goto exit;

		error = IT9510_readRegister (modulator, Processor_LINK, 0x49AB, &OFS_I_value[5]);//OFS_6_i calibration value
		if (error) goto exit;
		error = IT9510_readRegister (modulator, Processor_LINK, 0x49E8, &OFS_Q_value[5]);//OFS_6_q calibration value
		if (error) goto exit;
		error = IT9510_readRegister (modulator, Processor_LINK, 0x49C3, &OFS_I_value[6]);//OFS_7_i calibration value
		if (error) goto exit;
		error = IT9510_readRegister (modulator, Processor_LINK, 0x49E9, &OFS_Q_value[6]);//OFS_7_q calibration value
		if (error) goto exit;
		; //puts("set up I/Q-DC calibration table");
		for(index = 1; index<8; index++){

			if(index == 1){
				dc_table[index-1].startFrequency = 200000;
				ofs_table[index-1].startFrequency = 200000;
			} else if(index == 2){
				dc_table[index-1].startFrequency = 325000;
				ofs_table[index-1].startFrequency = 325000;
			} else if(index == 3){
				dc_table[index-1].startFrequency = 500000;
				ofs_table[index-1].startFrequency = 500000;
			} else if(index == 4){
				dc_table[index-1].startFrequency = 700000;
				ofs_table[index-1].startFrequency = 700000;
			} else if(index == 5){
				dc_table[index-1].startFrequency = 875000;
				ofs_table[index-1].startFrequency = 875000;
			} else if(index == 6){
				dc_table[index-1].startFrequency = 1250000;
				ofs_table[index-1].startFrequency = 1250000;
			} else if(index == 7){
				dc_table[index-1].startFrequency = 2400000;
				ofs_table[index-1].startFrequency = 2400000;
			}

			dc_table[index-1].i = DCvalue[(index*2)];
			if(((DCvalue[0] >> (index-1) ) &0x01) == 0)
				dc_table[index-1].i = dc_table[index-1].i * -1;

			dc_table[index-1].q = DCvalue[1 + (index*2)];
			if(((DCvalue[1] >> (index-1) ) &0x01) == 0)
				dc_table[index-1].q = dc_table[index-1].q * -1;


			ofs_table[index-1].i = OFS_I_value[index-1];
			ofs_table[index-1].q = OFS_Q_value[index-1];
		}

		dcInfo.ptrDCtable = dc_table;
		dcInfo.ptrOFStable = ofs_table;
		dcInfo.tableGroups = 7;

		error = IT9510_setDCtable(modulator, dcInfo);
	}
exit:
#if USE_UART
	sprintf(pdbg, "exit %s, %x\n", __func__, error);
	dbg_len += strlen(pdbg);
	TRANSFER_LOG
#endif
	return error;
}

uint32_t IT9510User_rfPowerOn (
	  IT9510INFO*            modulator,
	  Bool                   isPowerOn
){
	uint32_t error = ModulatorError_NO_ERROR;
#if USE_UART
	char dbg_msg[80], dbg_len, *pdbg ;
	sprintf(dbg_msg, "enter %s\n", __func__);
	dbg_len = strlen(dbg_msg);
	TRANSFER_LOG
#endif

	error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh2_o, (uint8_t)isPowerOn);

#if USE_UART
	sprintf(pdbg, "exit %s, %x\n", __func__, error);
	dbg_len += strlen(pdbg);
	TRANSFER_LOG
#endif
	return error;

}

uint16_t EepromProtocolChecksum(uint8_t* p, uint8_t count)
{
	uint16_t sum;
	uint8_t i;

	sum = 0;
	for (i = 0; i < count; i++) {
		if (i & 1)
			sum += p[i];		// add low byte.
		else
			sum += ((uint8_t)p[i]) << 8;	// add high byte.
	}

	return ~sum;
}

uint32_t IT9510User_LoadRFGainTable(
		IT9510INFO*            modulator
		)
{
	uint32_t error = ModulatorError_NO_ERROR;
	RFGainInfo rfGain_Info;
	uint8_t eeprom = 0;
	uint16_t tempAddress = 0;
	uint8_t tempLength = 16;
	uint8_t tab_count, i, tmp_val;
	uint16_t tmp_data1, tmp_data2;
	uint8_t eepromData[256];
	uint16_t chk_tmp = 0, chk_page = 0;
	const uint8_t header_len = 4;
	const uint8_t sec_unit_size = 4; // eeprom: 2 byte for freq, 1 byte for digital and analog gain
	uint8_t tab_size = sizeof(rfGain_table) / sizeof(RFGainTable);

	rfGain_Info.tableIsValid = False;
	error = IT9510_writeRegister(modulator, Processor_LINK, 0x4979, 0x1);
	if (error) goto exit;

	error = IT9510_writeRegister(modulator, Processor_LINK, 0x496D, 0xa8);
	if (error) goto exit;

	//-------------get RF Gain table
	error = IT9510_readRegister(modulator, Processor_LINK, 0x4979, &eeprom);//has eeprom ??
	if (error) goto exit;

	if (eeprom == 0)
	{
		error = ModulatorError_NO_SUCH_TABLE;
		goto exit;
	}

	// read 256 byte from eeprom
	for (i = 0; i<16; i++) {
		tempAddress = i * 16;
		// get page 2 data
		error = IT9510_readEepromValues(modulator, 0x200 + tempAddress, tempLength, &eepromData[tempAddress]);
		if (error)	goto exit;
	}

	// checksum whether correct
	chk_tmp = EepromProtocolChecksum(eepromData, 256 - 2);

	chk_page = (eepromData[254] << 8) + eepromData[255];

	if (chk_tmp != chk_page)
	{
		error = ModulatorError_WRONG_CHECKSUM;
		goto exit;
	}
	// check eeprom data
	if (eepromData[0] == 0)
	{
		error = ModulatorError_NO_SUCH_TABLE;
		goto exit;
	}

	// get the table count
	tab_count = eepromData[1];

	if (tab_count > tab_size || tab_count == 1)
	{
		error = ModulatorError_INVALID_DATA_LENGTH;
		goto exit;
	}

	for (i = 0; i < tab_count; i++)
	{
		// get freq
		rfGain_table[i].rawFrequency = (eepromData[header_len + i * sec_unit_size] << 8) +
										eepromData[header_len + i * sec_unit_size + 1];
		// get alalog gain
		rfGain_table[i].alanogGainValue = eepromData[header_len + i * sec_unit_size + 2];

		//get digital gain
		tmp_val = eepromData[header_len + i * sec_unit_size + 3];
		rfGain_table[i].digitalGainValue = tmp_val & 0x80 ? ((tmp_val & 0x7F) * -1) : tmp_val;
	}

	// calculate the start frequency for gain control
	for (i = 0; i < tab_count - 1; i++)
	{
		tmp_data1 = rfGain_table[i].rawFrequency;
		tmp_data2 = rfGain_table[i + 1].rawFrequency;

		rfGain_table[i].startFrequency = (tmp_data1 + tmp_data2) * 1000 / 2; // use K Hz
	}

	rfGain_Info.tableIsValid = True;
	rfGain_Info.tableCount = tab_count;
	rfGain_Info.ptrGaintable = rfGain_table;

	error = IT9510_setRFGaintable(modulator, rfGain_Info);

exit:
	return error;
}

uint32_t IT9510User_getOutputGainInfo(
		IT9510INFO*    modulator,
		uint32_t			frequency,
		uint8_t*			index
		)
{
	uint32_t error = ModulatorError_NO_ERROR;
	uint8_t tab_count, i, tmpVal;

	RFGainInfo rfGain_Info = modulator->rfGainInfo;

	if (rfGain_Info.tableIsValid == False)
	{
		error = ModulatorError_NOT_SUPPORT;
		goto exit;
	}

	tab_count  = rfGain_Info.tableCount;

	// search the start frequency
	tmpVal = 0;

	for (i = 0; i < tab_count - 1; i++)
	{
		if (rfGain_Info.ptrGaintable[i].startFrequency == frequency)
		{
			tmpVal = i + 1;
			break;
		}
		else if (rfGain_Info.ptrGaintable[i].startFrequency > frequency)
		{
			tmpVal = i;
			break;
		}
	}

	// check frequency whether more than max table value
	if (i == tab_count - 1 && tmpVal == 0)
		tmpVal = i;

	// return output gain index
	*index = tmpVal;

exit:
	return error;
}

uint32_t IT9510User_adjustOutputGain(
		IT9510INFO*    modulator,
		int			  *gain
		)
{
	uint32_t error = ModulatorError_NO_ERROR;


	uint8_t bIndex = 0;
	uint32_t frequency = modulator->frequency;

	frequency = modulator->frequency + LO_Frequency; // RFFC2072's LO_Frequency


	if (IT9510User_getOutputGainInfo(modulator, frequency, &bIndex) == 0)
	{
		// save the digital gain when different with eeprom
		modulator->rfGainInfo.ptrGaintable[bIndex].digitalGainValue = *gain;
	}


	return error;
}
