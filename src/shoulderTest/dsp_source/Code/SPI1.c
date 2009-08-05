
#include "SPI1.h"
#include "asc.h"


static byte CLKshift;
static byte CLKsampl;
static byte InputBuffer;
static byte SerFlag;  

#define OVERRUN_ERR  1                 /* Overrun error flag bit   */
#define CHAR_IN_RX   8                 /* Char is in RX buffer     */

/*                                       
void IOReg_PutVal(Int16 RegAddress, Int8 RegBitAddress, bool Val)
{
  if (Val) {                           // Is it one to be written? 
    setRegBits(RegAddress,RegBitAddress); // Set bit on port
  }
  else {                               // Is it zero to be written? 
    clrRegBits(RegAddress,RegBitAddress); // Clear bit on port 
  }
}

bool IOReg_GetVal(Int16 RegAddress, Int8 RegBitAddress)
{
	
	return getRegBits(RegAddress,RegBitAddress);
}
*/

byte SPI1_RecvChar(byte *Chr)
{
  if(!(SerFlag & CHAR_IN_RX))          /* Is char. received */
    return ERR_RXEMPTY;
  *Chr = InputBuffer;                  /* Store the character */
  if(SerFlag & OVERRUN_ERR) {          /* Is "overrun" occured? */
    SerFlag &= ~(OVERRUN_ERR|CHAR_IN_RX); /* If yes, clear flags */
    return OVERRUN_ERR;                /* ... and return error */
  }
  else {
    SerFlag &= ~CHAR_IN_RX;            /* If no, clear flag */
    return ERR_OK;                     /* ... and return */
  }
}

byte SPI1_SendChar(byte Chr)
{
 	setReg(SPDTR, 0x0000 | Chr );
 	return ERR_OK;
}


byte SPI1_SendWord(Int16 Chr)
{
 	Int16 temp = 0;
 	
 	setReg(SPDTR, Chr);
 	
    temp=getReg(SPSCR);
 	while ((temp & 0x2000) == 0)
 	{
 		temp=getReg(SPSCR);
 	}
 	temp=getReg(SPDRR);

	return ERR_OK;	
}


void SPI1_Init(void)
{

    //MISO 
	setRegBits(GPIO_E_PER,0x40);
	//MOSI  
	setRegBits(GPIO_E_PER,0x20);
	//SCLK   
	setRegBits(GPIO_E_PER,0x10); 
	
    // 0000 0000 1111 0011 0xF3 /32 baudrate
	setReg(SPSCR,0xF3);
	// 0000 0000 0000 1111
	setReg(SPDSR,0x0F);
	
	
	
	
	
}



