

#ifndef __SPI1
#define __SPI1

/* MODULE SPI1. */
#include "dsp56f807.h"


#ifndef __BWUserType_TPtrByte
#define __BWUserType_TPtrByte
  typedef byte* TPtrByte ; /* Pointer to byte */
#endif

void IOReg_PutVal(Int16 RegAddress, Int8 RegBitAddress, bool Val);
bool IOReg_GetVal(Int16 RegAddress, Int8 RegBitAddress);



byte SPI1_RecvChar(byte *Chr);
/*
** ===================================================================
**     Method      :  SPI1_RecvChar (bean SWSPI)
**
**     Description :
**         If any data received, this method returns one character,
**         otherwise it returns error code (it does not wait for
**         data).
**     Parameters  :
**         NAME            - DESCRIPTION
**       * Chr             - Pointer to received character.
**     Returns     :
**         ---             - Error code, possible codes:
**                           ERR_OK - OK
**                           ERR_RXEMPTY - No data in receiver
**                           ERR_OVERRUN - Overrun error is detected
** ===================================================================
*/

byte SPI1_SendChar(byte Chr);
/*
** ===================================================================
**     Method      :  SPI1_SendChar (bean SWSPI)
**
**     Description :
**         Send one character to the channel.
**     Parameters  :
**         NAME            - DESCRIPTION
**         Chr             - Character to send.
**     Returns     :
**         ---             - Error code, possible codes:
**                           ERR_OK - OK
** ===================================================================
*/

byte SPI1_SendWord(Int16 Chr);
/*
** ===================================================================
**     Method      :  SPI1_SendChar (bean SWSPI)
**
**     Description :
**         Send one character to the channel.
**     Parameters  :
**         NAME            - DESCRIPTION
**         Chr             - Character to send.
**     Returns     :
**         ---             - Error code, possible codes:
**                           ERR_OK - OK
** ===================================================================
*/

void SPI1_Init(void);
/*
** ===================================================================
**     Method      :  SPI1_Init (bean SWSPI)
**
**     Description :
**         This method is internal. It is used by Processor Expert
**         only.
** ===================================================================
*/


/* END SPI1. */

#endif /* ifndef __SPI1 */
/*
** ###################################################################
**
**     This file was created by UNIS Processor Expert 2.95 [03.58]
**     for the Freescale 56800 series of microcontrollers.
**
** ###################################################################
*/
