/*
 * firmware/controller application.
 *
 */

#include "can1.h"
#include "dsp56f807.h"
#include "options.h"
#include "asc.h"
#include "pid.h"
#include "controller.h"
#include "trajectory.h"

#include "encoders_interface.h"
#include "calibration.h"

#ifndef VERSION
#	error "No valid version specified"
#endif

/************************************************************ 
 * this function checks if the calibration is terminated
 * and if calibration is terminated resets the encoder
 ************************************************************/
void check_in_position_calib(byte jnt)
{
	Int32 temporary_long;
	bool temporary_cond1;
	bool temporary_cond2; 
	
	/* final consideration reached? and ... */
	temporary_long = (Int32) extract_h(_filt_abs_pos[jnt]);
	temporary_cond1 = (__abs( temporary_long - _abs_pos_calibration[jnt]) < INPOSITION_CALIB_THRESHOLD);
	temporary_cond2 = (_speed[jnt] == 0);
	/* ... control mode is calibration? and ... */
	temporary_cond1 = temporary_cond1 && (_control_mode[jnt] == MODE_CALIB_ABS_POS_SENS);
	temporary_cond2 = temporary_cond2 && (_control_mode[jnt] == MODE_CALIB_HARD_STOPS);
	/* ... trajecotry ended? */
	temporary_cond1 = temporary_cond1 && _ended[jnt];
	temporary_cond2 = temporary_cond2 && (_counter_calib > 1000);
		
	if (temporary_cond1 | temporary_cond2)
	{
		
		AS1_printStringEx ("Calibration sequence terminated \r\n");
		_control_mode[jnt] = MODE_POSITION;
		//Reset the encoder
		set_position_encoder (jnt, 0);
		
		_position[jnt] = 0;
		_position_old[jnt] = 0;
		_desired[jnt] = 0;
		_integral[jnt] = 0;
		//Keep the system in the current configuration
		_set_point[jnt] = _position[jnt];
		init_trajectory (jnt, _position[jnt], _position[jnt], 1);
		_calibrated[jnt] = true;
	}
}



/**************************************************************** 
 * calibration procedure, depends on the firmware version.
 ****************************************************************/
byte calibrate (byte channel, byte type, Int16 param1,Int16 param2, Int16 param3)
{

#if VERSION == 0x0113
	if (_control_mode[channel] != MODE_IDLE && IS_DONE(channel))
	{
		if (channel == 0)
		{
			_control_mode[channel] = MODE_CALIB_ABS_POS_SENS;
			_abs_pos_calibration[channel] = param1;
			
			_set_point[channel] = _abs_pos_calibration[channel];
			_set_vel[channel] = 1;
			init_trajectory (channel, (Int32) extract_h(_filt_abs_pos[channel]), _set_point[channel], _set_vel[channel]);	
		}
		else
		{
			_control_mode[channel] = MODE_CALIB_HARD_STOPS;
			
			_counter_calib = 0;
			_pwm_calibration[channel] = param1;
		}

		AS1_printStringEx ("Calibration sequence started \r\n");
	}

#elif VERSION == 0x0112
	if (_control_mode[channel] != MODE_IDLE && IS_DONE(channel))
	{
		_control_mode[channel] = MODE_CALIB_ABS_POS_SENS;
		_abs_pos_calibration[channel] = param1;
		
		_set_point[channel] = _abs_pos_calibration[channel];
		_set_vel[channel] = 1;
		init_trajectory (channel, (Int32) extract_h(_filt_abs_pos[channel]), _set_point[channel], _set_vel[channel]);	

		AS1_printStringEx ("Calibration sequence started \r\n");
	}
#else
	if (_control_mode[channel] != MODE_IDLE && IS_DONE(channel))
	{
		_control_mode[channel] = MODE_CALIB_HARD_STOPS;
			
		_counter_calib = 0;
		_pwm_calibration[channel] = param1;
		
		AS1_printStringEx ("Calibration sequence started \r\n");
		AS1_printWord16AsChars(param1);
	}
#endif


	/* need to change this return val */
	return ERR_SPEED;
}
