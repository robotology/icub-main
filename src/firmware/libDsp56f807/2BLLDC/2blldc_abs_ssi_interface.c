#include "dsp56f807.h"
#include "abs_ssi_interface.h"

/***************************************************************************/
/**
 *	Globals
 ***************************************************************************/ 
UInt16 max_real_position[2] = {4095, 4095};
Int32 encoder_zeros[2] = {0,0};
Int32 old_value[2] = {0,0};
bool  status_ocf = 0;
bool  status_cof = 0;
bool  status_lin = 0;
bool  status_inc = 0;
bool  status_dec = 0;

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
	//DATAIN init MISO
	clrRegBits(GPIO_E_DDR,GPIO_E6);
	clrRegBits(GPIO_E_PER,GPIO_E6);

	//SPIEN1: GPIOA0	
	setRegBits(GPIO_A_DDR,GPIO_A0);   
	clrRegBits(GPIO_A_PER,GPIO_A0); 
	//SPIEN2: GPIOA1
	setRegBits(GPIO_A_DDR,GPIO_A1);   
	clrRegBits(GPIO_A_PER,GPIO_A1); 
	
	setRegBits(GPIO_E_DR,0x04);
	setRegBits(GPIO_A_DR,GPIO_A0);
	setRegBits(GPIO_A_DR,GPIO_A1);
	
	max_real_position[0]=4095;
	max_real_position[1]=4095;	

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
		if (i==12)
		{
		 	status_ocf=getRegBits(GPIO_E_DR,GPIO_E6);	 	
		}
		if (i==13)
		{
		 	status_cof=getRegBits(GPIO_E_DR,GPIO_E6);	 	
		}
		if (i==14)
		{
			status_lin=getRegBits(GPIO_E_DR,GPIO_E6); //DATAIN	
		}
		if (i==15)
		{
			status_inc=getRegBits(GPIO_E_DR,GPIO_E6); //DATAIN	
		}
		if (i==16)
		{
			status_dec=getRegBits(GPIO_E_DR,GPIO_E6); //DATAIN	
		}
	}
	setRegBits(GPIO_A_DR,mask); //ENABLE	
	return (UInt16) value;
}

void get_status_abs_ssi(bool* s_ocf,bool* s_cof,bool* s_lin, bool* s_inc, bool* s_dec,byte jnt)
{
	(*s_ocf)= status_ocf;
	(*s_cof)= status_cof;
	(*s_lin)= status_lin;
	(*s_inc)= status_inc;
	(*s_dec)= status_dec;
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

/***************************************************************************/
/**
 * this function uses an SSI absolute position sensor as a relative 
 * multiturn encoder. When it is called the first time, the measurment
 * is given in the 0-4095 range. Then this number can become <0 or >4095
 * every time the sensore detects a zero crossing.
 * @param jnt is the joint number 
 * @return  the reading of the sensor 
 ***************************************************************************/
Int32 get_relative_position_abs_ssi(byte jnt)
{	
	static Int16 turns[2] = {0,0};
			
	Int32 value = (Int32) get_absolute_real_position_abs_ssi(jnt);//-encoder_zeros[jnt];
	
	if ((value - old_value[jnt]) > 2000 ) turns[jnt]--;
	
	else if ((value - old_value[jnt]) < -2000) turns[jnt]++;

	old_value[jnt]= value;
				
	return (value+turns[jnt]*4095);
}

Int32 init_relative_position_abs_ssi()
{
	old_value[0]=get_absolute_real_position_abs_ssi(0);
	old_value[1]=get_absolute_real_position_abs_ssi(1);
}

Int32 set_relative_position_abs_ssi(byte jnt,Int32 offset)
{
	encoder_zeros[jnt]=offset;
}