/** ###################################################################
**     Filename  : Timer.C
**     Settings  :
**         Timer name                  : TMRC01 (32-bit)
**         Compare name                : TMRC01_Compare
**         Counter shared              : No
**
**         High speed mode
**             Prescaler               : divide-by-1
**             Clock                   : 40000000 Hz
**           Period
**             Xtal ticks              : 8000000
**             microseconds            : 1000000
**             milliseconds            : 1000
**             seconds                 : 1
**             seconds (real)          : 1.0000000
**             Hz                      : 1
**           Frequency of counting (Bus clock / prescaler)
**             Hz                      : 40000000
**
**         Initialization:
**              Timer                  : Disabled
**
**         Timer registers
**              Counter                : TMRC1_CNTR [3341]
**              Mode                   : TMRC1_CTRL [3342]
**              Run                    : TMRC1_CTRL [3342]
**              Prescaler              : TMRC1_CTRL [3342]
**              Compare                : TMRC1_CMP1 [3336]
**
** ###################################################################*/

/* MODULE Timer. */

#include "Timer.h"


/*
** ===================================================================
**     Method      :  Timer_Init 
**
**     Description :
**         This method is internal. It is used by Processor Expert
**         only.
** ===================================================================
*/
void Timer_Init(void)
{
  /* TMRC0_CTRL: CM=0,PCS=0,SCS=0,ONCE=0,LENGTH=1,DIR=0,Co_INIT=0,OM=0 */
  setReg(TMRC0_CTRL,32);               /* Stop all functions of the timer */
  /* TMRC1_CTRL: CM=7,PCS=4,SCS=0,ONCE=0,LENGTH=1,DIR=0,Co_INIT=0,OM=0 */
  setReg(TMRC1_CTRL,59424);            /* Set up cascade counter mode */
  setReg(TMRC1_CNTR,0);                /* Reset counter register */
  setReg(TMRC0_CNTR,0);
  setReg(TMRC1_LOAD,0);                /* Reset load register */
  setReg(TMRC0_LOAD,0);
  setReg(TMRC1_CMP1,639);             /* Store given value to the compare registers */
  setReg(TMRC0_CMP1,62499);
  setRegBitGroup(TMRC0_CTRL,PCS,8);  /* Store given value to the prescaler */
  setReg(TMRC0_CNTR,0);                /* Reset counter */
  setReg(TMRC1_CNTR,0);
}

/* END Timer. */

