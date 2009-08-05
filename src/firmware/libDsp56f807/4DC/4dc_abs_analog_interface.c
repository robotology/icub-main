#include "dsp56f807.h"
#include "ad.h"
#include "abs_analog_interface.h"

/***************************************************************************/
/**
 * this function inits the analog absolute position sensor interface
 ***************************************************************************/
void init_position_abs_analog(void)
{
	AD_init ();
	AD_enableIntTriggerA ();
	AD_enableIntTriggerB ();	
}

/***************************************************************************/
/**
 * this function reads the current _position which will be used in the PID.
 * Measurament is given by an analog absolute position sensor.
 * @param jnt is the joint number
 * @return  the reading of the sensor 
 ***************************************************************************/
Int16 get_position_abs_analog(byte jnt)
{
	word temporary = 0;
	
	if 		(jnt == 0)
		AD_getChannel16A (0, &temporary);
	else if (jnt == 1)
		AD_getChannel16A (4, &temporary);
	else if (jnt == 2)
		AD_getChannel16B (0, &temporary);
	else if (jnt == 3)
		AD_getChannel16B (4, &temporary);
	
	return temporary;
}

/***************************************************************************/
/**
 * this function calculates the filtering of a time-variant value
 * (i.e. the readings absolute position sensor)
 * @param value is the current reading to be filtered
 * @return  the filtered value
 ***************************************************************************/
Int32 compute_filt_pos(Int16 value, byte jnt)
{
	/*
	The filter is the following:

	_filt_abs_pos =  K * (position + position_old) + a_1 * _filt_abs_pos_old

	Parameter a_1 is computed to filter out the noise which affect the
	signal. In the current filter we filter out all frequencies greater than 
	5Hz. This choice is related to the fact that the sensor is already designed
	with a (hardware) low pass filter whose crossover freqeuncy is at 5Hz. All 
	residual frequencies are therefore caused by transmission noises.
	
	Specifically we have:
	a_1 = 32427*2^(-15).
	K	= 11186546*2^(-31)
	Variables are represented as follows:
	a_1							Int16		16 bit with 15 fractional bits
	K							Int32		32 bit with 31 fractional bits
	_abs_pos					Int16		16 bit with 15 fractional bits
	K * _abs_pos_old			Int32		32 bit with 31 fractional bits
	_filt_abs_pos				Int32		32 bit with 31 fractional bits
	_filt_abs_pos_old			Int32		32 bit with 31 fractional bits

	NOTE: The variable _abs_pos_old will contain the numerical vaulue of
	the product _abs_pos[t-1]*K. A more suitable name for the variable would
	be _K_times_abs_pos_old; this name is not used being too long.
	*/

	Int16 a_1 = 0x7EAB;
	Int32 K   = 0x00AAB172;
	Int32 tmp;
	Int32 tmp2;
	
	static Int32 old_value[4] = {0,0,0,0};
	static Int32 old_filt[4] = 	{0,0,0,0};
	static Int32 filt[4] = 		{0,0,0,0};
	
	tmp = L_mult_ls(K, value);
	tmp2 = L_mult_ls(old_filt[jnt], a_1);
	tmp2 = L_add(tmp2, old_value[jnt]);
	filt[jnt] = L_add(tmp, tmp2);

	old_value[jnt]= tmp;
	old_filt[jnt] = filt[jnt];
		
	return filt[jnt];
}
