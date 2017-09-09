#include "DigiBestTypedef.h"
#include <string.h>
#include <asf.h>
#include "conf_usb.h"
#include "conf_example.h"
#include "ui.h"
#include "uart.h"
#include "delay.h"
#include "DigiBestFrontend.h"
#if (SAMG55)
#include "flexcom.h"
#endif

extern void fordigibest_twi_wr(unsigned int* buffer, int length);
extern void fordigibest_twi_rd(unsigned int* buffer, int length);
extern void fordigibest_get_fw_hdr(unsigned int* fw_hdr);
extern void fordigibest_usb_read_buf1(void* *pbuff, int size0);
extern void fordigibest_reception_statistics(void* pRecptStat_src);
extern void fordigibest_short_statistics(void* pShortStat_src);


BOOL PlatformPorting_Frontend_Initialization()
{
// please implement your own platform portig initialization here !
    return TRUE;
}
VOID PlatformPorting_Frontend_Uninitialization()
{
// please implement your own platform portig uninitialization here !
}
BOOL PlatformPorting_Frontend_I2C_Write(ULONG FrontendGroupID,UBYTE I2CDeviceAddress,PUBYTE pWriteBuffer,ULONG WriteLength)
{
  // please implement your own I2C write here !
  fordigibest_twi_wr((unsigned int*)pWriteBuffer, WriteLength);
  return TRUE;
}
BOOL PlatformPorting_Frontend_I2C_Write_FW(ULONG FrontendGroupID,UBYTE I2CDeviceAddress,PUBYTE pFirmwareWriteBuffer,ULONG FirmwareWriteLength)
{
// please implement your own I2C write firmware here !
fordigibest_twi_wr((unsigned int*)pFirmwareWriteBuffer, FirmwareWriteLength);
	return TRUE;
}
BOOL PlatformPorting_Frontend_I2C_Read(ULONG FrontendGroupID,UBYTE I2CDeviceAddress,ULONG ReadLength,PUBYTE pReadBuffer)
{
// please implement your own I2C read firmware here !
    fordigibest_twi_rd((unsigned int*)pReadBuffer, ReadLength);

	return TRUE;
}
BOOL PlatformPorting_Frontend_HardwareReset(ULONG FrontendGroupID)
{
// please implement your own frontend hardware reset here !
	pio_set_output(PIOB, PIO_PB15, LOW, DISABLE, ENABLE); 
	delay_ms(10);
	pio_set(PIOB, PIO_PB15);
    delay_ms(30);  //YH: required, for Siano to get ready
	   
	return TRUE;
}

BOOL PlatformPorting_Frontend_Fw_Download_Usb_Read(void* *pbuff, int size)
{
  fordigibest_usb_read_buf1(pbuff, size);
	
	return TRUE;
}

BOOL PlatformPorting_Frontend_Reception_Statistics(void* pRecptStat)
{
	fordigibest_reception_statistics(pRecptStat);
	
	return TRUE;
}

BOOL PlatformPorting_Frontend_Short_Statistics(void * pStat)
{
	fordigibest_short_statistics(pStat);
		
	return TRUE;
}

