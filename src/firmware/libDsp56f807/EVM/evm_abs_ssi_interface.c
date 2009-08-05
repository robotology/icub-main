#include "dsp56f807.h"
#include "abs_ssi_interface.h"

/***************************************************************************/
/**
 *	this function inits the SSI bus for the absolute position sensor
 ***************************************************************************/ 
void init_position_abs_ssi()
{
	//SSI1 Initilization
	
	//SCLK init
	setRegBits(GPIO_E_DDR,0x10);   
	clrRegBits(GPIO_E_PER,0x10); 	
	//DATAIN init
	clrRegBits(GPIO_E_DDR,0x20);
	clrRegBits(GPIO_E_PER,0x20);

	//SPIEN1: GPIOB1	
	setRegBits(GPIO_B_DDR,0x02);   
	clrRegBits(GPIO_B_PER,0x02); 
	//SPIEN2: GPIOB3
	setRegBits(GPIO_B_DDR,0x08);   
	clrRegBits(GPIO_B_PER,0x08); 
	//SPIEN3: GPIOB5	
	setRegBits(GPIO_B_DDR,0x20);   
	clrRegBits(GPIO_B_PER,0x20); 
	//SPIEN4: GPIOB7
	setRegBits(GPIO_B_DDR,0x80);   
	clrRegBits(GPIO_B_PER,0x80);
	
	setRegBits(GPIO_E_DR,0x04);
	setRegBits(GPIO_B_DR,0x02);
	setRegBits(GPIO_B_DR,0x08);
	setRegBits(GPIO_B_DR,0x20);
	setRegBits(GPIO_B_DR,0x80);
	
}

/***************************************************************************/
/**
 * this function reads the current _position which will be used in the PID.
 * Measurament is given by an SSI absolute position sensor.
 * @param jnt is the joint number 
 * @return  the reading of the sensor 
 ***************************************************************************/
UInt16 get_position_abs_ssi(byte jnt)
{	
	UInt16 value=0;
	UInt16 i,j;
	UInt16 bit=0;
	byte   mask=0;
	
	if      (jnt==0) mask =0x02;
	else if (jnt==1) mask =0x08;
	else if (jnt==2) mask =0x20;
	else if (jnt==3) mask =0x80;
		
	setRegBits(GPIO_E_DR,GPIO_E4); //CLK
	clrRegBits(GPIO_B_DR,mask);	//ENABLE
	for(i=0;i<18;i++)
	{
		clrRegBits(GPIO_E_DR,0x10); //CLK
		for (j=0;j<5;j++)
		{
		 asm
		 {
		 	NOP
		 }
		}// delay
		setRegBits(GPIO_E_DR,0x10); //CLK
		for (j=0;j<5;j++)
		{
		 asm
		 {
		 	NOP
		 }
		}// delay
		if (i<12)
		{
			bit=getRegBits(GPIO_E_DR,0x20); //DATAIN
			value=value | (bit <<(11-i)); 	
		}		
	}
	setRegBits(GPIO_B_DR,mask); //DISABLE
	return (UInt16) value;
}
