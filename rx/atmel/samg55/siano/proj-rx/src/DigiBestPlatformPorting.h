#ifndef __DIGIBESTPLATFORMPORTING_H
#define __DIGIBESTPLATFORMPORTING_H

#ifdef __cplusplus
extern "C" {
#endif

extern BOOL PlatformPorting_Frontend_Initialization();
extern VOID PlatformPorting_Frontend_Uninitialization();
extern BOOL PlatformPorting_Frontend_I2C_Write(ULONG FrontendGroupID,UBYTE I2CDeviceAddress,PUBYTE pWriteBuffer,ULONG WriteLength);
extern BOOL PlatformPorting_Frontend_I2C_Write_FW(ULONG FrontendGroupID,UBYTE I2CDeviceAddress,PUBYTE pFirmwareWriteBuffer,ULONG FirmwareWriteLength);
extern BOOL PlatformPorting_Frontend_I2C_Read(ULONG FrontendGroupID,UBYTE I2CDeviceAddress,ULONG ReadLength,PUBYTE pReadBuffer);
extern BOOL PlatformPorting_Frontend_HardwareReset(ULONG FrontendGroupID);
extern BOOL PlatformPorting_Frontend_Fw_Download_Usb_Read(void* *pbuff, int size);
extern BOOL PlatformPorting_Frontend_Reception_Statistics(void* pRecptStat);
extern BOOL PlatformPorting_Frontend_Short_Statistics(void* pStat);

#ifdef __cplusplus
}
#endif

#endif

