#include "bsp.h"
#include "lgdst_4463_spi.h"

/*!
 * This function is used to read/write one byte from/to SPI1.
 *
 * @param[in] biDataIn    Data to be sent.
 * @return  Read value of the SPI port after writing on it.
 */
/*
U8 SpiReadWrite(U8 biDataIn)
{  
  SEGMENT_VARIABLE(bValue, U8, SEG_DATA);
  BIT gRestoreEA;


  // disable interrupts during SPI transfer 
  //...


  // Send SPI data using double buffered write
  //SPIF1 = 0;                                    // clear SPIF
  //SPI1DAT = ( biDataIn );                       // write data register
  //while (!TXBMT1);                              // wait on TXBMT
  //while ((SPI1CFG & M_SPIBSY1) == M_SPIBSY1);   // wait on SPIBSY
  //bValue = SPI1DAT;                             // read value
  //SPIF1 = 0;                                    // leave SPIF cleared
  bValue = 0xff;

  // Restore interrupts after SPI transfer
  //...

  return bValue;
}
*/

/*!
 * This function is used to send data over SPI0 no response expected.
 *
 *  @param[in] biDataInLength  The length of the data.
 *  @param[in] *pabiDataIn     Pointer to the first element of the data.
 *
 *  @return None
 */

void SpiWriteData(U8 biDataInLength, U8 *pabiDataIn)
{
	si4463_wrbytes(pabiDataIn, biDataInLength);
  //while (biDataInLength--) {
	//Kevin, please add
    //SpiReadWrite(*pabiDataIn++);
  //}
}

/*!
 * This function is used to read data from SPI0.
 *
 *  \param[in] biDataOutLength  The length of the data.
 *  \param[out] *paboDataOut    Pointer to the first element of the response.
 *
 *  \return None
 */
void SpiReadData(U8 biDataOutLength, U8 *paboDataOut)
{
 	si4463_rdbytes(paboDataOut, biDataOutLength);
  //while (biDataOutLength--) {
	  //Kevin, please add
    //*paboDataOut++ = SpiReadWrite(0xFF);
  //}
}
