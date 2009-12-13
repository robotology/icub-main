/*
 * firmware/controller application.
 *
 */

#include "dsp56f807.h"
#include "options.h"
#include "asc.h"
#include "pid.h"
#include "controller.h"
#include "trajectory.h"
#include "pwm_interface.h"

#include "abs_ssi_interface.h"
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
  //this function is not used for a board with brushess motors
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
	if (type==CALIB_ABS_DIGITAL)
	{

#if (VERSION==0x0151 || VERSION==0x0152 || VERSION==0x0154 || VERSION==0x0174) 

		AS1_printStringEx ("Calibration ABS_DIGITAL started \r\n");
		if (param3 >=0 && param3 <=4095) set_max_position(channel, param3);	
		if (param2>0)
		{
		    _position[channel] = get_position_abs_ssi(channel);
			_set_point[channel] = param1;
			init_trajectory (channel, _position[channel], _set_point[channel], param2);
			_in_position[channel] = false;
			_calibrated[channel] = true;
	//		AS1_printStringEx ("Calibration ABS_DIGITAL terminated \r\n");
		}
		if (param2==0)
		{
			_control_mode[channel]=MODE_IDLE;	
			_pad_enabled[channel] = false;
			PWM_outputPadDisable(channel);
	//		AS1_printStringEx ("Calibration ABS_DIGITAL aborted\r\n");
	//		AS1_printStringEx ("Offset setted\r\n");			
		}
#elif VERSION==0x0153 || VERSION==0x0173
	//  	AS1_printStringEx ("Calibration ABS_DIGITAL started \r\n");
		if (param3 >=0 && param3 <=4095) set_max_position(channel, param3);	
		if (param2>0 && channel==0)
		{
			set_relative_position_abs_ssi_turns(channel, 0);
			_position[channel] = get_relative_position_abs_ssi(channel);
			#if VERSION==0x0153
				_position[channel] = _position[channel]+ ((float)_cpl_pos_prediction[0]*6.5/3.8);  
				_position[channel] = _position[channel]- ((float) _cpl_pos_prediction[1]*6.5/3.8);    
			#elif VERSION==0x0173
				_position[channel] = (((float) _position[channel])*0.6153F);  
				_position[channel] = _position[channel]+ _cpl_pos_prediction[0];
				_position[channel] = _position[channel]- _cpl_pos_prediction[1];			
			#endif			
			_set_point[channel] = param1;
			init_trajectory (channel, _position[channel], _set_point[channel], param2);
			_in_position[channel] = false;
			_calibrated[channel] = true;
	//		AS1_printStringEx ("Calibration ABS_DIGITAL terminated \r\n");
		}
		if (param2>0 &&channel==1)
		{
		    _position[channel] = get_position_abs_ssi(channel);
			_set_point[channel] = param1;
			init_trajectory (channel, _position[channel], _set_point[channel], param2);
			_in_position[channel] = false;
			_calibrated[channel] = true;
	//		AS1_printStringEx ("Calibration ABS_DIGITAL terminated \r\n");
		}
		if (param2==0)
		{
			_control_mode[channel]=MODE_IDLE;	
			_pad_enabled[channel] = false;
			PWM_outputPadDisable(channel);
	//		AS1_printStringEx ("Calibration ABS_DIGITAL aborted\r\n");
	//		AS1_printStringEx ("Offset setted\r\n");			
		}
#endif
	}   
}
