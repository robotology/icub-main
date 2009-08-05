/*
 * firmware/controller application.
 *
 */

#include "dsp56f807.h"
#include "options.h"
#include "asc.h"
#include "pid.h"
#include "trajectory.h"
#include "pwm_interface.h"
#include "can1.h"
#include "calibration.h"
#include "encoders_interface.h"
#include "controller.h"

#ifndef VERSION   
#	error "No valid version specified"
#endif

/************************************************************ 
 * this function checks if the calibration is terminated
 * and if calibration is terminated resets the encoder
 ************************************************************/
static Int32 temporary_position[JN]={0,0};
static byte refresh[JN]={0,0};


void check_in_position_calib(byte jnt)
{
	Int32 temporary_long;
	bool temporary_cond1;
	bool temporary_cond2; 
	temporary_cond2 = (_position[jnt] ==temporary_position[jnt]);
	temporary_cond2 = temporary_cond2 && (_control_mode[jnt] == MODE_CALIB_HARD_STOPS);
	/* ... trajectory ended? */
	temporary_cond2 = temporary_cond2 && (_counter_calib >= 1000);
	
	refresh[jnt]++;
	if (refresh[jnt]>=20)
	{
		temporary_position[jnt]=_position_old[jnt];
		refresh[jnt]=0;
	}	
	if (temporary_cond2)
	{
	#ifdef DEBUG_CALIBRATION
		can_printf ("Calibration sequence terminated \r\n");
		
		if (_counter_calib >= 1000)
		can_printf ("_counter_calib reached 1000\r\n");
		
	#endif
		_control_mode[jnt] = MODE_POSITION;
		//Reset the encoder	
		set_position_encoder (jnt, 0);
		_position[jnt] = 0;
		_position_old[jnt] = 0;
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
  // this board has only brushless motor with digital absolute
  // position sensors. So, the only operation required for the 
  // calibration in to set an "offset" value that maps the output
  // of the abs position sensor, in order to avoid the passage on 0.
	if (type==CALIB_HARD_STOPS)
	{
#if VERSION==0x0161 
	#ifdef DEBUG_CALIBRATION    
		can_printf("Calibration Encoder started \r\n");
	#endif		
		if (_control_mode[channel] != MODE_IDLE && IS_DONE(channel))
		{
			_control_mode[channel] = MODE_CALIB_HARD_STOPS;	
			_counter_calib = 0;
			_pwm_calibration[channel] = param1;
			if (param2!=0)
			{
		 		_velocity_calibration[channel]=param2;
			}
			else
			{
				_velocity_calibration[channel]=1;
			}
		#ifdef DEBUG_CALIBRATION
			can_printf("Calibration HARD_STOPS started %d %d \r\n", param1,param2);
		#endif				
		}	
#endif
	}

	if (type==CALIB_ABS_DIGITAL)
	{
#if VERSION==0x0162

	#ifdef DEBUG_CALIBRATION	
		can_printf("Calibration Absolute encoder started \r\n");
	#endif		
		if (param3 >=0 && param3 <=4095) set_max_position(channel, param3);	
		if (param2>0)
		{
		    _position[channel] = get_position_abs_ssi(channel);
			_set_point[channel] = param1;
			init_trajectory (channel, _position[channel], _set_point[channel], param2);
			_calibrated[channel] = true;
		#ifdef DEBUG_CALIBRATION
			can_printf ("Calibration ABS_DIGITAL terminated \r\n");
		#endif		
		}
		if (param2==0)
		{
			_control_mode[channel]=MODE_IDLE;	
			_pad_enabled[channel] = false;
			PWM_outputPadDisable(channel);
		#ifdef DEBUG_CALIBRATION			
			can_printf ("Calibration ABS_DIGITAL aborted\r\n");
			can_printf ("Offset setted\r\n");		
		#endif					
		}
#endif
return 0;
	}
}
