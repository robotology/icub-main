#include "SSI1.h"

	UInt16 value=0;
	UInt16 bit=0;
	UInt16 i,j;


void SSI1_Init()
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

UInt16 SSI1_GetVal(byte mask)
{	
	value=0;
	setRegBits(GPIO_E_DR,0x10); //CLK
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

