

#ifndef __brushless_commh__
#define __brushless_commh__

#include "dsp56f807.h"

typedef struct sDutyControl_tag
{
	byte    Dir;	
	Int16   Duty;
} sDutyControlBL;



#define  MIN_DUTY 6  // MIN_DUTY has to be bigger then DEAD_TIME  8 a 200KHz 8 a 50KHz 8 a 30KHz at 48V it moves even at min duty
#define  MAX_DUTY 1250// 190 a 200KHz  760 a 50KHz 1250 a 30KHz 
#define  DEAD_TIME 4 //deadtime  
#define  STEP 10     // 1 a 200KHz  4 a 50KHz 6 a 30KHz   


#define  PWMFREQ 1333 // 200 = 200KHz  800= 50KHz 1333= 30KHz


void TD0_interrupt(void);

void TD0_Enable(void);


void TD0_Disable(void);

//It initialize the brushless commutation for axis_number axis (1 or 2 axis)
void Init_Brushless_Comm(byte axis_number);  //  
void TD0_init (void);
void PWM_generate_BLL(byte , Int16);
Int32 get_speed(byte axis);
Int32 get_commutations(byte axis);
void set_commutations(byte axis, Int32 value);

#endif 