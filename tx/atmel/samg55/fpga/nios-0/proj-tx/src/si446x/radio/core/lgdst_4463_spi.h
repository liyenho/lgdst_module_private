#ifndef LGDST_4463_SPI
#define LGDST_4463_SPI

#include <compiler.h>  // for U8/16/32 definition???
#define RF4463_TX

#ifdef RF4463_TX
	#define RF4463_TX_MODE 1
	#define SI4463_SPI SPI2_MASTER_BASE //Kevin
#else
	#define RF4463_TX_MODE 0
	#define SI4463_SPI SPI7_MASTER_BASE //Kevin
#endif

#define RF4463_PWRDN 1
#define RF4463_NSEL  2
#define RF4463_NIRQ  3
#define RF4463_GPI00 4
#define RF4463_GPI01 5

/*****************************************************************************
 *  Global Function Declarations
 *****************************************************************************/
bool si4463_wrbytes(U8 *buf, U8 size);
bool si4463_rdbytes(U8 *buf, U8 size);
U8 si4463_get_cts();
void SBIT_W(U8 IO_name, U8 IO_value);
U8 SBIT_R(U8 IO_name);

#endif /* LGDST_4463_SPI */