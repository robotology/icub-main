
#include "currents_interface.h"
#include "ad.h"
#include "pwm_interface.h"

Int32 MAX_CURRENT=1600;   //MAX current in milliAmpere
Int32 MAX_I2T_CURRENT=2000000000;
//Int32 _current_limit_I2T=1600; //NOT USED, we are using max_allowed_current  value in mA 
Int32 _current[4] =		 {0,0,0,0};					/* current through the transistors*/
Int32 _current_old[4] =  {0,0,0,0};					/* current at t-1*/
Int32 _filt_current[4] = {0,0,0,0};     			/* filtered current through the transistors*/
Int32 _max_allowed_current[4] = {1600000,1600000,1600000,1600000};	/* limit on the current in micro-ampere*/							
float _conversion_factor[4] = {0.0488,0.0488,0.0488,0.0488} ;	// 484.85*3.3/32760	

/*************************************************************************** 
 * this function checks if the current consumption has exceeded a threshold
 * for more than 200 ms using a filtered verion of the current reading.
 ***************************************************************************/
void init_currents( void )
{
	AD_init ();
	AD_enableIntTriggerA ();
	AD_enableIntTriggerB ();	
}

/*************************************************************************** 
 * this function checks if the current consumption has exceeded a threshold
 * for more than 200 ms using a filtered verion of the current reading.
 ***************************************************************************/
word check_current(byte jnt, bool sign)
{
	word temp;
	Int32 temporary;

	switch (jnt)
	{
		case 0: AD_getChannel16A (1, &temp); break;
		case 1: AD_getChannel16A (5, &temp); break;	
		case 2: AD_getChannel16B (1, &temp); break;
		case 3: AD_getChannel16B (5, &temp); break;			
	}		
	
	temporary = (Int32) temp;
	if (!sign)	temporary = -temporary;
	
	_current_old[jnt] = _current[jnt];

	_current[jnt] = temporary * _conversion_factor[jnt];
	return _current[jnt];
}

/********************************************************* 
 * this function filters the current (AD value).
 *********************************************************/
void compute_filtcurr(byte jnt)
{
	/*
	The filter is the following:
	_filt_current = a_1 * _filt_current_old 
				  + a_2 * (_current_old + _current).
	Parameters a_1 and a_2 are computed on the sample time
	(Ts = 1 ms) and the rising time (ts = 200ms).Specifically
	we have:
	a_1 = (2*tau - Ts) / (2*tau + Ts)
	a_2 = Ts / (2*tau + Ts)
	where tau = ts/2.3. Therefore:
	a_1 = 0.9886
	a_2 = 0.0057
	For numerical reasons we prefer to compute _filt_current
	in micro-ampere choosing a_2 = 3.8 instead of a_2 = 0.0038
	*/
	
	Int32 current;
	Int32 current_old;
	static dword filt_current_old[4] = {0,0,0,0};
	
	/*take the absolute value 
	in order to consider both positive and
	negative currents*/

	if (_current[jnt] < 0)
		current = -_current[jnt];
	else
		current = _current[jnt];
	
	if (_current_old[jnt] < 0)
		current_old = -_current_old[jnt];
	else
		current_old = _current_old[jnt];

	
	filt_current_old[jnt] = _filt_current[jnt];
	_filt_current[jnt] = 0.9886 * filt_current_old[jnt] + 5.7 * (current_old + current);
	
}
