#include "EEPROM.h"
#include "SPI1.h"

// EEPROM M95160 array:2048*8
void EEPROM_Init()
{
	
	SPI1_Init();  
	//CS configuration
	//E2PCS: GPIOE2
	setRegBits(GPIO_E_DDR,0x04);   
	clrRegBits(GPIO_E_PER,0x04); 
	
	//E2PWP: GPIOE3  Hardware Write Protection 0=write protect
	setRegBits(GPIO_E_DDR,0x08);   
	clrRegBits(GPIO_E_PER,0x08); 
	
	// set E2PCS
	setRegBits(GPIO_E_DR,0x04);   
	// clear E2PWP: write protect 
	clrRegBits(GPIO_E_DR,0x08); 
	
	
}


/*
# define WREN  0b00000110
# define WRDI  0b00000100
# define RDSR  0b00000101
# define WRSR  0b00000001
# define READ  0b00000011
# define WRITE 0b00000010
*/

byte EEPROM_WREN()
{
	byte ret=0;
	// clr E2PCS
	clrRegBits(GPIO_E_DR,0x04); 
	ret=SPI1_SendChar(WREN);
	// set E2PCS
	setRegBits(GPIO_E_DR,0x04); 
	return ret;
}

byte EEPROM_WRDI()
{   
	byte ret=0;
	// clr E2PCS
	clrRegBits(GPIO_E_DR,0x04); 
 	ret=SPI1_SendChar(WRDI);	
	// set E2PCS
	setRegBits(GPIO_E_DR,0x04);
	return ret; 
}

byte EEPROM_RDSR(byte *SR)
{
	byte ret=0;
	// clr E2PCS
	clrRegBits(GPIO_E_DR,0x04); 
	ret=SPI1_SendChar(RDSR);
	ret+=SPI1_RecvChar(SR); //potrebbe essere necessario mettere un loop di attesa
					        // del dato con timeout	o un check sul buffer
	// set E2PCS
	setRegBits(GPIO_E_DR,0x04); 
	return ret;
}
byte EEPROM_WRSR(byte WSR)
{
	byte ret=0;
	// set E2PWP: write protect 
	setRegBits(GPIO_E_DR,0x08); 
	EEPROM_WREN();
	// clr E2PCS
	clrRegBits(GPIO_E_DR,0x04); 
	ret=SPI1_SendChar(WRSR);
	ret+=SPI1_SendChar(WSR);
	// set E2PCS
	setRegBits(GPIO_E_DR,0x04);
	// clear E2PWP: write protect 
	clrRegBits(GPIO_E_DR,0x08); 
	return ret; 
}

byte EEPROM_READ(byte * DataRead,UInt16 Address, UInt16 NData)
{
	byte ret=0;
    byte data=0;
	UInt16 i=0;
	// clr E2PCS
	clrRegBits(GPIO_E_DR,0x04); 
	SPI1_SendChar(READ);
	ret=SPI1_SendChar(Address);
	for(i=0;i<NData;i++)
	{
		ret+=SPI1_RecvChar(&data);//potrebbe essere necessario mettere un loop di attesa
					              // del dato con timeout o un check 	
		DataRead[i]=data;
	}
	// set E2PCS
	setRegBits(GPIO_E_DR,0x04);	
	return ret; 
}
byte EEPROM_WRITE(byte * DataWrite, UInt16 Address, UInt16 NData)
{
	byte ret=0;
	UInt16 i=0;
	// set E2PWP: write protect 
	setRegBits(GPIO_E_DR,0x08); 
	EEPROM_WREN();
	// clr E2PCS
	clrRegBits(GPIO_E_DR,0x04); 
	SPI1_SendChar(WRITE);
	ret=SPI1_SendChar(Address);
	for(i=0;i<NData;i++)
	{
		ret+=SPI1_SendChar(DataWrite[i]);
	}
	// set E2PCS
	setRegBits(GPIO_E_DR,0x04);
	// clear E2PWP: write protect 
	clrRegBits(GPIO_E_DR,0x08); 
	
	return ret; 
}

