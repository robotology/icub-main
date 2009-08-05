//#pragma define_section mySection "mySection.text" RWX
//#pragma section mySection begin

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

void _downloadEntryPoint(void)
{
	asm(move #0,y1);					/* stores y1=0 for jumping
										 * to main() after initialization */
	asm(JMP init_56800_);               /* Jump to C startup code */
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

//#pragma section mySection end