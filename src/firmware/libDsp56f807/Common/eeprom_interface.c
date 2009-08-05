#include "eeprom_interface.h"
#include "SPI1.h"

// EEPROM M95160 array:2048*8

/***************************************************************************/
/**
 * This method inits the eeprom interface
 ***************************************************************************/
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
	setRegBits(GPIO_E_DR,0x08); 
}

///write enable command
#define WREN  0x6
///write disable command
#define WRDI  0x4
/// ?? command
#define RDSR  0x5
/// ?? command
#define WRSR  0x1
/// read command
#define READ  0x3
/// write command
#define WRITE 0x2

/***************************************************************************/
/**
 * This method enables the writing on the eeprom
 * @return ERR_OK if the operation has been complete successfully
 ***************************************************************************/
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

/***************************************************************************/
/**
 * This method disables the writing on the eeprom
 * @return ERR_OK if the operation has been complete successfully
 ***************************************************************************/
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

/***************************************************************************/
/**
 * This method reads the status register of the eeprom
 * @param SR is the byte that represents the status register of the eeprom
 * @return ERR_OK if the operation has been complete successfully
 ***************************************************************************/
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

/***************************************************************************/
/**
 * This method writes the status register of the eeprom
 * @param WSR is the byte to write on the eeprom
 * @return ERR_OK if the operation has been complete successfully
 ***************************************************************************/
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

/***************************************************************************/
/**
 * This method reads an array of bytes from the eeprom
 * @param DataRead is the pointer to the array of data to be filled
 * @param Address is the eeprom address to be read
 * @param NData is the number of bytes to be read 
 * @return ERR_OK if the operation has been complete successfully
 ***************************************************************************/
byte EEPROM_READ(byte * DataRead,UInt16 Address, UInt16 NData)
{
	byte ret=0;
    byte data=0;
	UInt16 i=0;
	// clr E2PCS
	clrRegBits(GPIO_E_DR,0x04); 
	SPI1_SendChar(READ);
	ret=SPI1_SendChar((Address>>8)&0xFF);
	ret=SPI1_SendChar((Address)&0xFF);
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

/***************************************************************************/
/**
 * This method writes an array of bytes to the eeprom
 * @param DataWrite is the pointer to the array of data to be written
 * @param Address is the eeprom address to be witten
 * @param NData is the length in bytes of the array to be written  
 * @return ERR_OK if the operation has been complete successfully
 ***************************************************************************/
byte EEPROM_WRITE(byte * DataWrite, UInt16 Address, UInt16 NData)
{
	byte ret=0;
	UInt16 i=0;
	// set E2PWP: write protect 
	setRegBits(GPIO_E_DR,0x08); 
	EEPROM_WREN();
//	wait(2);
	// clr E2PCS
	clrRegBits(GPIO_E_DR,0x04); 
	SPI1_SendChar(WRITE);
	ret=SPI1_SendChar((Address>>8)&0xFF);
	ret=SPI1_SendChar((Address)&0xFF);
	for(i=0;i<NData;i++)
	{
		ret+=SPI1_SendChar(DataWrite[i]);
	}
	// set E2PCS
	setRegBits(GPIO_E_DR,0x04);
//	// clear E2PWP: write protect 
//	clrRegBits(GPIO_E_DR,0x08); 
	
	return ret; 
}

