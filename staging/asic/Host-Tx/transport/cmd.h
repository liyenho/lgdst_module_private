#ifndef __CMD_H__
#define __CMD_H__


/**
 * Bus types
 */
#define Bus_I2C             1
#define Bus_USB             2
#define Bus_SPI             3
#define Bus_SDIO            4
#define Bus_USB11           5
#define Bus_I2M             6  /** I2C bus for Mercury      */
#define Bus_I2U             7  /** I2C bus for Mercury USB  */
#define Bus_I2U2            8  /** I2C bus for Mercury USB (new, buffer size is 55) */
#define Bus_9035U2I         9  /** I2C bus for IT9130 USB */


/**
 * Define commands
 */
#define Command_REG_DEMOD_READ          0x0000
#define Command_REG_DEMOD_WRITE         0x0001
#define Command_REG_TUNER_READ          0x0002
#define Command_REG_TUNER_WRITE         0x0003
#define Command_REG_EEPROM_READ         0x0004
#define Command_REG_EEPROM_WRITE        0x0005
#define Command_VAR_READ                0x0008
#define Command_VAR_WRITE               0x0009

#define Command_DATA_READ               0x0006

#define Command_PLATFORM_GET            0x000A
#define Command_PLATFORM_SET            0x000B
#define Command_IP_CACHE                0x000D
#define Command_IP_ADD                  0x000E
#define Command_IP_REMOVE               0x000F
#define Command_PID_ADD                 0x0010
#define Command_PID_REMOVE              0x0011
#define Command_SIPSI_GET               0x0012  /** Get SI/PSI table for specific PID "once". */
#define Command_SIPSI_MPE_RESET         0x0013
#define Command_H_PID_ADD               0x0015
#define Command_H_PID_REMOVE            0x0016
#define Command_ABORT                   0x0017
#define Command_IR_GET                  0x0018
#define Command_IR_SET                  0x0019
#define Command_FW_DOWNLOAD_BEGIN       0x0024
#define Command_FW_DOWNLOAD             0x0021
#define Command_FW_DOWNLOAD_END         0x0025
#define Command_QUERYINFO               0x0022
#define Command_BOOT                    0x0023
#define Command_REBOOT                  0x0023
#define Command_RUN_CODE                0x0026
#define Command_SCATTER_READ            0x0028
#define Command_SCATTER_WRITE           0x0029
#define Command_GENERIC_READ            0x002A
#define Command_GENERIC_WRITE           0x002B
#define Command_SHORT_REG_DEMOD_READ    0x02
#define Command_SHORT_REG_DEMOD_WRITE   0X03
#define Command_SHORT_REG_TUNER_READ    0x04
#define Command_SHORT_REG_TUNER_WRITE   0X05

#define Command_SERVICES_GET            0x0083
#define Command_COMPONENT_ADD           0x0086
#define Command_COMPONENT_REMOVE        0x0087
#define Command_FIG_ADD                 0x0088
#define Command_FIG_REMOVE              0x0089

#define Bus_MAX_WRITE_SIZE              254
#define Bus_MAX_READ_SIZE               254

#define User_MAX_PKT_SIZE               255
#define User_RETRY_MAX_LIMIT        500
#define IT913X_ADDRESS        				0x38 /*0x38*/ // (gpioh6, gpioh5)= (0,0)

#define Cmd_buildCommand(command, processor, chip)  (command + (uint16_t) (processor << 12) + (uint16_t) (chip << 12))
/**
 * The type defination of Processor.
 */
typedef enum {
    Processor_LINK = 0,
    Processor_OFDM = 8
} Processor;

/**
 *
 */
uint32_t Cmd_writeRegisters (
    uint8_t            chip,
    Processor       processor,
    uint32_t           registerAddress,
    uint8_t            registerAddressLength,
    uint32_t           writeBufferLength,
    uint8_t*           writeBuffer
);


/**
 *
 */
uint32_t Cmd_writeTunerRegisters (
    uint8_t            chip,
    uint8_t            tunerAddress,
    uint16_t            registerAddress,
    uint8_t            registerAddressLength,
    uint8_t            writeBufferLength,
    uint8_t*           writeBuffer
);


/**
 *
 */
uint32_t Cmd_readRegisters (
    uint8_t            chip,
    Processor       processor,
    uint32_t           registerAddress,
    uint8_t            registerAddressLength,
    uint32_t           readBufferLength,
    uint8_t*           readBuffer
);


/**
 *
 */
uint32_t Cmd_readTunerRegisters (
    uint8_t            chip,
    uint8_t            tunerAddress,
    uint16_t            registerAddress,
    uint8_t            registerAddressLength,
    uint8_t            readBufferLength,
    uint8_t*           readBuffer
);


/**
 *
 */
uint32_t Cmd_loadFirmware (
    uint32_t           length,
    uint8_t*           firmware
);


/**
 *
 */
uint32_t Cmd_reboot (
    uint8_t            chip
);


/**
 *
 */
uint32_t Cmd_sendCommand (
    uint16_t            command,
    uint8_t            chip,
    Processor       processor,
    uint32_t           writeBufferLength,
    uint8_t*           writeBuffer,
    uint32_t           readBufferLength,
    uint8_t*           readBuffer
);

uint32_t Standard_readRegisters (
	uint8_t            chip,
	Processor       processor,
	uint32_t           registerAddress,
	uint8_t            bufferLength,
	uint8_t*           buffer
);

uint32_t Standard_writeRegisters (
	uint8_t            chip,
	Processor       processor,
	uint32_t           registerAddress,
	uint8_t            bufferLength,
	uint8_t*           buffer
);

uint32_t Standard_getFirmwareVersion (
	Processor       processor,
	uint32_t*          version
);
#endif