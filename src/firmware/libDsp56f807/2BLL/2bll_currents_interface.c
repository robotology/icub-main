
#include "currents_interface.h"
#include "ad.h"
#include "pwm_interface.h"
#include "brushless_comm.h"


Int32 MAX_CURRENT=7000;   //MAX current in milliAmpere
Int32 MAX_I2T_CURRENT=2000000000;
Int32 _current_limit_I2T=7000; //NOT USED, we are using max_allowed_current  value in mA 
Int32 _current[4] =		 {0,0,0,0};					/* current through the transistors in mA*/
Int32 _current_old[4] =  {0,0,0,0};					/* current at t-1*/
Int16 _current_debug[4]={0,0,0,0};
Int32 _filt_current[4] = {0,0,0,0};     			/* filtered current through the transistors*/
Int32 _max_allowed_current[4] = {7000,7000,7000,7000};	/* limit on the current in milli-ampere*/							
float _conversion_factor[4] ={0.46,0.46,0.46,0.46};		/* limit on the current as set by the interface (later converted into the filter parameter) */
Int32 _current_offset[4]= {3376,3376,3376,3376};
extern sDutyControlBL DutyCycle[2];
byte   first_time[2]={true,true};
/*************************************************************************** 
 * this function checks if the current consumption has exceeded a threshold
 * for more than 200 ms using a filtered verion of the current reading.
 ***************************************************************************/
void init_currents( void )
{
	AD_init ();
	AD_enableIntTriggerA ();
	AD_enableIntTriggerB ();	
	first_time[0]=true;
	first_time[1]=true;	
}

byte set_current_offset(byte jnt)
{
    word temp;
    byte ret;
   	if (jnt==0)
   	{
   		AD_getChannel16A (1, &temp);
   		if (temp!=15)
   		{
   			_current_offset[0]= (Int32) temp;
   			return ERR_OK;
   		}
   	}
	else
	{
		AD_getChannel16B (1, &temp);
		if (temp!=15)
   		{
	    	_current_offset[1]= (Int32) temp;
	    	return ERR_OK;
   		}
	}
	return ERR_VALUE;
}

/*************************************************************************** 
 * this function checks if the current consumption has exceeded a threshold
 * for more than 200 ms using a filtered verion of the current reading.
 ***************************************************************************/
word check_current(byte jnt, bool sign)
{
//	#warning "_current now is Iss the Imot=Iss/deltaPWM;"
	word temp;
	Int32 temporary;	
	
	switch (jnt)
	{
		case 0: 
		{
			AD_getChannel16A (1, &temp);
			if (first_time[0])
				{
					if(set_current_offset(0)==ERR_OK) first_time[0]=false;	
				}	
		}
		break;
		
		case 1:
		{
			AD_getChannel16B (1, &temp);
			if (first_time[1])
				{
					if(set_current_offset(1)==ERR_OK) first_time[1]=false;	
				}
		} 
		break;			
	}
	if ( temp < _current_offset[jnt])	temporary = 0; //4080 was calculated,   3328 was found experimentally
	else  temporary = ((Int32)(temp))-_current_offset[jnt];
	if (!sign)	temporary = -temporary;
	_current_old[jnt] = _current[jnt];
	
	_current_debug[jnt]=temp;
	if (DutyCycle[jnt].Duty<=(2*DEAD_TIME)+20)
	{
		_current[jnt]=500;
	}
	else
	{
 		 _current[jnt]=  (temporary * (float) _conversion_factor[jnt])*((Int32)((PWMFREQ<<4)/((Int32)(DutyCycle[jnt].Duty-2*DEAD_TIME))))/16;//see BLP and BLL manual		
	}
   																							   
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
	
	
	//provo a fare un filtro di 100ms 
	//ts=100ms Ts=1ms
	/*
	a_1 = (2*tau - Ts) / (2*tau + Ts)
	a_2 = Ts / (2*tau + Ts)
	where tau = ts/2.3=43.47. Therefore:
	a_1 = 0.977
	a_2 = 0.011 
	*/
	Int32 current;
	Int32 current_old;
	static Int32 filt_current_old[4] = {0,0,0,0};
	
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
	//_filt_current[jnt] = 0.9886 * filt_current_old[jnt] + 5.7 * (current_old + current);
	
	// filter with 200ms filter
	_filt_current[jnt] =(Int32) (0.9886 * (float) filt_current_old[jnt] + 5.7 * (current_old + current));
		
}

void compute_i2t(byte jnt)
{
	Int32 _increment=0;
	Int32 current;
		if (_current[jnt] < 0)
		current = -_current[jnt];
	else
		current = _current[jnt];
	
	_increment=current-_max_allowed_current[jnt];
	
	if (_increment>0)
	{
		_filt_current[jnt]+=((_increment*_increment)>>4);	
	}
	else 
	{
		_filt_current[jnt]+=((-_increment*_increment)>>4);
	}
	if (_filt_current[jnt]<0) _filt_current[jnt]=0;

}