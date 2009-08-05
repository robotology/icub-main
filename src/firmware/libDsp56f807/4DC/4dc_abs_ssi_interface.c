#include "dsp56f807.h"
#include "abs_ssi_interface.h"

/***************************************************************************/
/**
 *	Globals
 ***************************************************************************/ 
UInt16 max_real_position[2] = {4095, 4095};

/***************************************************************************/
/**
 *	this function inits the SSI bus for the absolute position sensor
 ***************************************************************************/ 
void init_position_abs_ssi()
{
	//SSI1 Initilization
	
	//SCLK init
	setRegBits(GPIO_E_DDR,GPIO_E4);   
	clrRegBits(GPIO_E_PER,GPIO_E4); 	
	//DATAIN init
	clrRegBits(GPIO_E_DDR,GPIO_E6);
	clrRegBits(GPIO_E_PER,GPIO_E6);

	//SPIEN1: GPIOA0	
	setRegBits(GPIO_A_DDR,GPIO_A0);   
	clrRegBits(GPIO_A_PER,GPIO_A0); 
	//SPIEN2: GPIOA1
	setRegBits(GPIO_A_DDR,GPIO_A1);   
	clrRegBits(GPIO_A_PER,GPIO_A1); 
	//SPIEN3: GPIOA2	
	setRegBits(GPIO_A_DDR,GPIO_A2);   
	clrRegBits(GPIO_A_PER,GPIO_A2); 
	//SPIEN4: GPIOA3
	setRegBits(GPIO_A_DDR,GPIO_A3);   
	clrRegBits(GPIO_A_PER,GPIO_A3); 
	
	setRegBits(GPIO_E_DR,0x04);
	setRegBits(GPIO_A_DR,GPIO_A0);
	setRegBits(GPIO_A_DR,GPIO_A1);
	setRegBits(GPIO_A_DR,GPIO_A2);
	setRegBits(GPIO_A_DR,GPIO_A3);
	
}

/***************************************************************************/
/**
 *	
 ***************************************************************************/ 
void set_max_position(byte jnt, UInt16 max_pos)
{
	max_real_position[jnt]=max_pos;
}

/***************************************************************************/
/**
 *	
 ***************************************************************************/ 
UInt16 get_max_position(byte jnt)
{
	return max_real_position[jnt];
}

/***************************************************************************/
/**
 *	
 ***************************************************************************/ 
void    set_current_as_middle_position(byte jnt)
{
	UInt16 value=get_absolute_real_position_abs_ssi(jnt);
	max_real_position[jnt]=value;
}

/***************************************************************************/
/**
 * this function reads the current _position which will be used in the PID.
 * Measurament is given by an SSI absolute position sensor.
 * @param jnt is the joint number 
 * @return  the reading of the sensor 
 ***************************************************************************/
UInt16 get_absolute_real_position_abs_ssi(byte jnt)
{	
	UInt16 value=0;
	UInt16 i,j;
	UInt16 bit=0;
	byte   mask=(1<<jnt);
		
	setRegBits(GPIO_E_DR,GPIO_E4); //CLK
	clrRegBits(GPIO_A_DR,mask);	//ENABLE
	for(i=0;i<18;i++)
	{
		clrRegBits(GPIO_E_DR,GPIO_E4); //CLK
		for (j=0;j<5;j++)
		{
		 asm
		 {
		 	NOP
		 }
		}// delay
		setRegBits(GPIO_E_DR,GPIO_E4); //CLK
		for (j=0;j<5;j++)
		{
		 asm
		 {
		 	NOP
		 }
		}// delay
		if (i<12)
		{
			bit=getRegBits(GPIO_E_DR,GPIO_E6); //DATAIN
			value=value | (bit <<(11-i)); 	
		}		
	}
	setRegBits(GPIO_A_DR,mask); //DISABLE
	return (UInt16) value;
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
	UInt16 value = get_absolute_real_position_abs_ssi(jnt);
	
	if ((value >= max_real_position[jnt] ) && (value <= 4095))
	{
		value=value- max_real_position[jnt]; 
	}
	else
	{
		value=value+ (4095-max_real_position[jnt])+1;
	}
			
	return (UInt16) value;
}
