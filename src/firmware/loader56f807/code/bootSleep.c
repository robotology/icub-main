//#pragma define_section sectionBootVar ".bss" RWX
//#pragma section sectionBootVar begin
//int firstBoot = 2;
//#pragma section sectionBootVar end

#pragma define_section sectionBootSleep "sectionBootSleep.text" RWX
#pragma section sectionBootSleep begin

#include "Cpu.h"
#include "IO_Map.h"
//#include "bootSleep.h"
#include "56F80x_init.h"

void _sleepEntryPoint();
/*
** ===================================================================
**     Function      :  _sleepEntryPoint
**
**     Description :
**      This function is executed only at the 
**		startup of the device. A suitable sleep
**		time allows all the devices to be correctly
**		powered up. Removing the sleep time might be
**		risky in situations were the power supply ramp
**		is slow.
** ===================================================================
*/

void _sleepEntryPoint(void)
{	
    /*** ### 56F807 "Cpu" init code ... ***/
  	/* System clock initialization */
	setReg(PLLCR, (PLLCR_LCKON_MASK | PLLCR_ZSRC0_MASK)); /* Enable PLL, LCKON and select clock source from prescaler */
	/* CLKOSR: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,CLKOSEL=0 */
	setReg16(CLKOSR, 0);                 /* CLKO = ZCLOCK */ 
	/* PLLDB: LORTP=0,PLLCOD=0,PLLCID=1,??=0,PLLDB=39 */
	setReg16(PLLDB, 295);                /* Set the clock prescalers */ 
	while(!getRegBit(PLLSR, LCK0)){}     /* Wait for PLL lock */
	setReg(PLLCR, (PLLCR_LCKON_MASK | PLLCR_ZSRC1_MASK)); /* Select clock source from postscaler */
	/* External bus initialization */
	/* BCR: ??=0,??=0,??=0,??=0,??=0,??=0,DRV=1,??=0,WSX=12,WSP=12 */
	setReg16(BCR, 716);                  /* Bus control register */ 

	asm(move #1,y1)						/* stores y1=1 for jumping
										 * to mySleep after initialization */
	
	asm(jmp init_56800_);               /* Jump to C startup code */
}

#pragma section sectionBootSleep end