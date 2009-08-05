
#include "SPI1.h"
#include "asc.h"

static byte CLKshift;
static byte CLKsampl;
static byte InputBuffer;
static byte SerFlag;

#define OVERRUN_ERR  1                 /* Overrun error flag bit   */
#define CHAR_IN_RX   8                 /* Char is in RX buffer     */

/***************************************************************************/
// simple delay loop. 
/***************************************************************************/
void wait(byte n);
void wait(byte n)
{
	unsigned int i=0;
	for (i=0;i<n;i++)
	{
	asm
		 {
		 	NOP
		 }
	}
}

/***************************************************************************/
/**
 * This method reads a byte from the SPI bus
 * @param Chr is the read byte
 * @return ERR_OK always
 ***************************************************************************/
byte SPI1_RecvChar(byte *Chr)
{
	int i=0;
	byte ToRead=0;  
	*Chr=0;
    //SCLK   
 	setRegBits(GPIO_E_DR,0x10);
 	wait(4); 
	for(i=7;i>=0;i--)
	{	
	  //SCLK   
 	  clrRegBits(GPIO_E_DR,0x10); 
 	  wait(4);
 	  ToRead= getRegBits(GPIO_E_DR,0x40);
	  wait(1);
	  *Chr |=(ToRead & 1)<<i;
	  //SCLK   
 	  setRegBits(GPIO_E_DR,0x10);
 	  wait(4);  
	}
	wait(4); 
	//SCLK   
 	setRegBits(GPIO_E_DR,0x10);
}

/***************************************************************************/
/**
 * This method sends a byte to the SPI bus
 * @param Chr is the byte to be sent. 
 * @return ERR_OK always
 ***************************************************************************/
byte SPI1_SendChar(byte Chr)
{
	int i=0;
	byte ToSend=0;
	//SCLK   
 	setRegBits(GPIO_E_DR,0x10);
 	wait(4); 
	for(i=7;i>=0;i--)
	{
	  ToSend=(Chr >>i) & 0x1;	
	  //SCLK   
 	  clrRegBits(GPIO_E_DR,0x10); 
 	  wait(4);
 	  if (ToSend==0x1)
 	  {
 	  	//MOSI  
	  	setRegBits(GPIO_E_DR,0x20);
 	  }
 	  else
	  {
	  	//MOSI  
	  	clrRegBits(GPIO_E_DR,0x20);
	  }
	  wait(1);
	  //SCLK   
 	  setRegBits(GPIO_E_DR,0x10);
 	  wait(4); 
 	  
	}
}


/***************************************************************************/
/**
 * This method inits the SPI interface
 ***************************************************************************/
void SPI1_Init(void)
{
    //MISO 
    clrRegBits(GPIO_E_DDR,0x40);   
	clrRegBits(GPIO_E_PER,0x40);
	//MOSI  
	setRegBits(GPIO_E_DDR,0x20);   
	clrRegBits(GPIO_E_PER,0x20);
	//SCLK   
	setRegBits(GPIO_E_DDR,0x10);   
	clrRegBits(GPIO_E_PER,0x10); 	
}



