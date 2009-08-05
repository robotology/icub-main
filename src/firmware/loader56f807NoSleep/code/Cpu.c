/* MODULE Cpu. */
#include "Timer.h"
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "Cpu.h"


/* Global variables */
volatile word SR_lock=0;               /* Lock */
volatile word SR_reg;                  /* Current value of the SR reegister */


/*
** ===================================================================
**     Method      :  _EntryPoint (bean 56F807)
**
**     Description :
**         This method is internal. It is used by Processor Expert
**         only.
** ===================================================================
*/
extern void init_56800_(void);         /* Forward declaration of external startup function declared in startup file */

/*** !!! Here you can place your own code using property "User data declarations" on the build options tab. !!! ***/

void _EntryPoint(void)
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
 

  asm(JMP init_56800_);                /* Jump to C startup code */
}

/*
** ===================================================================
**     Method      :  PE_low_level_init (bean 56F807)
**
**     Description :
**         This method is internal. It is used by Processor Expert
**         only.
** ===================================================================
*/
void PE_low_level_init(void)
{
  /* SYS_CNTL: ??=0,??=0,??=0,??=0,TMR_PD=0,CTRL_PD=0,ADR_PD=0,DATA_PD=0,??=0,??=0,??=0,BOOTMAP_B=0,LVIE27=0,LVIE22=0,PD=0,RPD=0 */
  setReg16(SYS_CNTL, 0);               /* Enable/Disable pullup registers for the given devices */ 
  /* Common initialization of the CPU registers */
  /* GPIO_E_PER: PE|=3 */
  setReg16Bits(GPIO_E_PER, 3);          
  /* ### FreeCntr "Timer" init code ... */
  Timer_Init();
  __DI();                              /* Disable interrupts. Only level 1 is allowed */
}

/*
** ===================================================================
**     Method      :  Cpu_Interrupt (bean 56F807)
**
**     Description :
**         This method is internal. It is used by Processor Expert
**         only.
** ===================================================================
*/
/*
#pragma interrupt
void Cpu_Interrupt(void)
{
  asm(DEBUG);                          // Halt the core and placing it in the debug processing state 
}
*/

/* END Cpu. */

