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
#include "abs_ssi_interface.h"
#include "can1.h"
#include "can_interface.h"
#include "pwm_interface.h"

#include "encoders_interface.h"
#include "calibration.h"

#ifndef VERSION
#	error "No valid version specified"
#endif

#if ((VERSION == 0x0120) || (VERSION == 0x0121) || (VERSION == 0x0128) || (VERSION == 0x0130))
Int16 _max_position_enc_tmp[JN] = INIT_ARRAY (0);
/* max allowd position for encoder while 
controlling with absolute position sensors*/
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
	bool temporary_cond3; 
	
	/* increase the counter for the calibration (wait for the movement to start)*/
	_counter_calib +=1;
	/* final consideration reached? and ... */
	temporary_long = (Int32) extract_h(_filt_abs_pos[jnt]);
	temporary_cond1 = (__abs( temporary_long - _abs_pos_calibration[jnt]) < INPOSITION_CALIB_THRESHOLD);
	temporary_cond2 = (_position[jnt] == _position_old[jnt]);
	/* ... control mode is calibration? and ... */
	temporary_cond1 = temporary_cond1 && (_control_mode[jnt] == MODE_CALIB_ABS_POS_SENS);
	temporary_cond2 = temporary_cond2 && (_control_mode[jnt] == MODE_CALIB_HARD_STOPS);
	temporary_cond3 =                    (_control_mode[jnt] == MODE_CALIB_ABS_AND_INCREMENTAL);
	/* ... trajecotry ended? */
	temporary_cond1 = temporary_cond1 && _ended[jnt];
	temporary_cond2 = temporary_cond2 && (_counter_calib > 1000);
	temporary_cond3 = temporary_cond3 && _ended[jnt];
		
	if (temporary_cond1 | temporary_cond2 | temporary_cond3)
	{
		
#if VERSION != 0x0119
		#ifdef DEBUG_CALIBRATION
		AS1_printStringEx ("Calibration sequence terminated \r\n");
		#endif
		_control_mode[jnt] = MODE_POSITION;
		//Reset the encoder	
		set_position_encoder (jnt, 0);

#if ((VERSION == 0x0120) || (VERSION == 0x0121))
		if (jnt==3)
		{
			_max_position_enc[jnt] = _max_position_enc_tmp[jnt];
			#ifdef DEBUG_CALIBRATION
			can_printf("maxPosEnc: %d", _max_position_enc[jnt]);
			#endif			
			_calibrated[jnt] = true;
			return;
		}
#endif		
#if ((VERSION == 0x0128))
		if (jnt!=0)
		{
			_max_position_enc[jnt] = _max_position_enc_tmp[jnt];
			#ifdef DEBUG_CALIBRATION
			can_printf("maxPosEnc: %d", _max_position_enc[jnt]);
			#endif			
			_calibrated[jnt] = true;
			return;
		}		
#endif
#if ((VERSION == 0x0130))
		{
			_max_position_enc[jnt] = _max_position_enc_tmp[jnt];
			#ifdef DEBUG_CALIBRATION
			can_printf("maxPosEnc: %d", _max_position_enc[jnt]);
			#endif			
			_calibrated[jnt] = true;
			return;
		}						
#endif
		
#elif VERSION ==0x0119
		#ifdef DEBUG_CALIBRATION
		AS1_printStringEx ("Calibration sequence terminated \r\n");
		#endif
		_control_mode[jnt] = MODE_POSITION;	

		if (jnt == 0)
		{
			//Reset the encoder	
			set_position_encoder (jnt, 0);
		}
		if (jnt==1)
		{
			set_position_encoder (2, _position[2]);
			set_position_encoder (1, 0);
			_position[2] = _position[1] + _position[2];
		}
		if (jnt==2)
		{
			set_position_encoder (2, -_position[1]);		
			_position[2] = _position[1] + _position[2];		
		}	
#endif
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
/********
 *0x0111*
 ********/
#if VERSION == 0x0111
	 
	if (type==CALIB_ABS_POS_SENS)
	{
		#ifdef DEBUG_CALIBRATION	
 		AS1_printStringEx ("Calibration ABS_POS aborted \r\n");
		#endif 		
	}

	if (type==CALIB_HARD_STOPS)
	{
		if (_control_mode[channel] != MODE_IDLE && IS_DONE(channel))
		{
			_control_mode[channel] = MODE_CALIB_HARD_STOPS;	
			_counter_calib = 0;
			_pwm_calibration[channel] = param1;
			if (param2!=0)
		 		_velocity_calibration[channel]=param2;
			else
				_velocity_calibration[channel]=1;
			#ifdef DEBUG_CALIBRATION			
			AS1_printStringEx ("Calibration HARD_STOPS started \r\n");
			AS1_printStringEx ("param1: ");
			AS1_printWord16AsChars (param1);
			AS1_printStringEx ("param2: ");	
			AS1_printWord16AsChars (param2);
			#endif
		}	
	}
/********
 *0x0112*
 ********/
#elif VERSION == 0x0112
	if (type==CALIB_ABS_POS_SENS)
	{
		if (_control_mode[channel] != MODE_IDLE && IS_DONE(channel))
		{
			_control_mode[channel] = MODE_CALIB_ABS_POS_SENS;
			_abs_pos_calibration[channel] = param1;		
			_set_point[channel] = _abs_pos_calibration[channel];
			_set_vel[channel] = param2;
			init_trajectory (channel, (Int32) extract_h(_filt_abs_pos[channel]), _set_point[channel], _set_vel[channel]);	
			#ifdef DEBUG_CALIBRATION			
	 		AS1_printStringEx ("Calibration ABS_POS started \r\n");
	 		AS1_printStringEx ("param1: ");
			AS1_printWord16AsChars (param1);
			AS1_printStringEx ("param2: ");	
			AS1_printWord16AsChars (param2);
			#endif			
		}
	}
	if (type==CALIB_HARD_STOPS)
	{
		#ifdef DEBUG_CALIBRATION	
		AS1_printStringEx ("Calibration HARD_STOPS aborted \r\n");	
		#endif
	}
/********
 *0x0114*
 ********/
#elif VERSION ==0x0114
	#ifdef DEBUG_CALIBRATION
	if (type==CALIB_ABS_POS_SENS)
	{
 		AS1_printStringEx ("Calibration ABS_POS aborted \r\n");
	}
	if (type==CALIB_HARD_STOPS)
	{
 		AS1_printStringEx ("Calibration HARD_STOPS aborted \r\n");
	}
	#endif		
/********
 *0x0115*
 ********/
#elif VERSION ==0x0115
	 
	if (type==CALIB_ABS_POS_SENS)
	{
		#ifdef DEBUG_CALIBRATION	
 		AS1_printStringEx ("Calibration ABS_POS aborted \r\n");
		#endif
	}

	if (type==CALIB_HARD_STOPS)
	{
		if (_control_mode[channel] != MODE_IDLE && IS_DONE(channel))
		{
			_control_mode[channel] = MODE_CALIB_HARD_STOPS;	
			_counter_calib = 0;
			_pwm_calibration[channel] = param1;
			if (param2!=0)
		 		_velocity_calibration[channel]=param2;
			else
				_velocity_calibration[channel]=1;
			#ifdef DEBUG_CALIBRATION			
			AS1_printStringEx ("Calibration HARD_STOPS started \r\n");		
			#endif
		}	
	}
/********
 *0x0116*
 ********/		
#elif VERSION ==0x0116	 
	#ifdef DEBUG_CALIBRATION
	if (type==CALIB_ABS_POS_SENS)
	{
 		AS1_printStringEx ("Calibration ABS_POS aborted \r\n");
	}
	#endif

	if (type==CALIB_HARD_STOPS)
	{
		if (_control_mode[channel] != MODE_IDLE && IS_DONE(channel))
		{
			_control_mode[channel] = MODE_CALIB_HARD_STOPS;	
			_counter_calib = 0;
			_pwm_calibration[channel] = param1;
			if (param2!=0)
		 		_velocity_calibration[channel]=param2;
			else
				_velocity_calibration[channel]=1;
			#ifdef DEBUG_CALIBRATION			
			AS1_printStringEx ("Calibration HARD_STOPS started \r\n");	
			AS1_printStringEx ("param1: ");
			AS1_printWord16AsChars (param1);
			AS1_printStringEx ("param2: ");	
			AS1_printWord16AsChars (param2);
			#endif
		}	
	}
/********
 *0x0117*
 ********/	
#elif VERSION ==0x0117
	#ifdef DEBUG_CALIBRATION
	if (type==CALIB_ABS_POS_SENS)
	{
 		AS1_printStringEx ("Calibration ABS_POS aborted \r\n");
	}
	if (type==CALIB_HARD_STOPS)
	{
 		AS1_printStringEx ("Calibration HARD_STOPS aborted \r\n");					
	}
	#endif
/********
 *0x0118*
 ********/
#elif VERSION ==0x0118
	#ifdef DEBUG_CALIBRATION
	if (type==CALIB_ABS_POS_SENS)
	{
 		AS1_printStringEx ("Calibration ABS_POS aborted \r\n");
	}
	#endif
	if ((type==CALIB_HARD_STOPS) && (channel!=0))
	{
		if (_control_mode[channel] != MODE_IDLE && IS_DONE(channel))
		{
			_control_mode[channel] = MODE_CALIB_HARD_STOPS;	
			_counter_calib = 0;
			_pwm_calibration[channel] = param1;
			if (param2!=0)
		 		_velocity_calibration[channel]=param2;
			else
				_velocity_calibration[channel]=1;
			#ifdef DEBUG_CALIBRATION			
			AS1_printStringEx ("Calibration HARD_STOPS started \r\n");
			AS1_printStringEx ("param1: ");
			AS1_printWord16AsChars (param1);
			AS1_printStringEx ("param2: ");	
			AS1_printWord16AsChars (param2);	
			#endif
		}	
	}
	if ((type==CALIB_ABS_DIGITAL) &&  (channel==0))
	{
		#ifdef DEBUG_CALIBRATION
		AS1_printStringEx ("Calibration ABS_DIGITAL started \r\n");
		AS1_printStringEx ("param1: ");
		AS1_printWord16AsChars (param1);
		AS1_printStringEx ("param2: ");	
		AS1_printWord16AsChars (param2);
		#endif
		if (param2>0)
		{
			_set_point[channel] = param1;
			init_trajectory(channel, _position[channel], _set_point[channel], param2);
			_calibrated[channel] = true;
			#ifdef DEBUG_CALIBRATION			
			AS1_printStringEx ("moving from: ");	
			AS1_printWord16AsChars (_position[channel]);	
			AS1_printStringEx ("moving to: ");	
			AS1_printWord16AsChars (_set_point[channel]);	
			AS1_printStringEx ("Calibration ABS_DIGITAL terminated \r\n");
			#endif
		}
		if (param2==0)
		{
			_control_mode[channel]=MODE_IDLE;	
			_pad_enabled[channel] = false;
			#ifdef DEBUG_CALIBRATION			
			AS1_printStringEx ("Calibration ABS_DIGITAL aborted\r\n");
			#endif			
		} 			
	}
/********
 *0x0119*
 ********/	
#elif VERSION ==0x0119
	byte channel1;
	byte channel2; 
	if ((type==CALIB_HARD_STOPS) && (channel==0))
	{
		if (_control_mode[channel] != MODE_IDLE && IS_DONE(channel))
		{
			_control_mode[channel] = MODE_CALIB_HARD_STOPS;	
			_counter_calib = 0;
			_pwm_calibration[channel] = param1;
			if (param2!=0)
		 		_velocity_calibration[channel]=param2;
			else
				_velocity_calibration[channel]=1;
			#ifdef DEBUG_CALIBRATION			
			AS1_printStringEx ("Calibration HARD_STOPS started \r\n");
			AS1_printStringEx ("param1: ");
			AS1_printWord16AsChars (param1);
			AS1_printStringEx ("param2: ");	
			AS1_printWord16AsChars (param2);
			#endif			
		}	
	}
	if (type==CALIB_ABS_POS_SENS)
	{
		#ifdef DEBUG_CALIBRATION	
 		AS1_printStringEx ("Calibration ABS_POS aborted \r\n");
		#endif
	}
	if (type==CALIB_HARD_STOPS_DIFF)
	{
		if (channel==1)
		{
			channel1=1; //maybe it could be change somehow 
			channel2=2;
		}
		else
		{
			channel1=2; //maybe it could be change somehow 
			channel2=1;
		}
		if ((_control_mode[channel1] != MODE_IDLE && IS_DONE(channel1)) && 
		   (_control_mode[channel2] != MODE_IDLE && IS_DONE(channel2)))
		{
			_control_mode[channel1] = MODE_CALIB_HARD_STOPS;
			_counter_calib = 0;
			_pwm_calibration[channel1] = param1;
			_pwm_calibration[channel2] = param1;
			if (param2!=0)
		 		_velocity_calibration[channel1]=param2;
			else
				_velocity_calibration[channel1]=1;
			#ifdef DEBUG_CALIBRATION			
			AS1_printStringEx ("Calibration CALIB_HARD_STOPS_DIFF started \r\n");	
			AS1_printStringEx ("param1: ");
			AS1_printWord16AsChars (param1);
			AS1_printStringEx ("param2: ");	
			AS1_printWord16AsChars (param2);
			#endif
		}	
	}	
	if (type==CALIB_ABS_DIGITAL)
	{
		#ifdef DEBUG_CALIBRATION	
		AS1_printStringEx ("Calibration ABS_DIGITAL started \r\n");
		AS1_printStringEx ("param1: ");
		AS1_printWord16AsChars (param1);
		AS1_printStringEx ("param2: ");	
		AS1_printWord16AsChars (param2);
		#endif		
		if (channel==3)
		{
			if (param2>0)
			{
				_set_point[channel] = param1;
				init_trajectory(channel, _position[channel], _set_point[channel], param2);
				_calibrated[channel] = true;
				#ifdef DEBUG_CALIBRATION				
				AS1_printStringEx ("moving from: ");	
				AS1_printWord16AsChars (_position[channel]);	
				AS1_printStringEx ("moving to: ");	
				AS1_printWord16AsChars (_set_point[channel]);	
				AS1_printStringEx ("Calibration ABS_DIGITAL terminated \r\n");
				#endif				
			}
			if (param2==0)
			{
				_control_mode[channel]=MODE_IDLE;	
				_pad_enabled[channel] = false;
				#ifdef DEBUG_CALIBRATION				
				AS1_printStringEx ("Calibration ABS_DIGITAL aborted\r\n");
				#endif				
			} 		
			
		}	
	}
/********
 *0x0129*
 ********/	
#elif VERSION ==0x0129

	if ((type==CALIB_HARD_STOPS) && (channel==0))
	{
		if (_control_mode[channel] != MODE_IDLE && IS_DONE(channel))
		{
			_control_mode[channel] = MODE_CALIB_HARD_STOPS;	
			_counter_calib = 0;
			_pwm_calibration[channel] = param1;
			if (param2!=0)
		 		_velocity_calibration[channel]=param2;
			else
				_velocity_calibration[channel]=1;
			#ifdef DEBUG_CALIBRATION			
			AS1_printStringEx ("Calibration HARD_STOPS started \r\n");
			AS1_printStringEx ("param1: ");
			AS1_printWord16AsChars (param1);
			AS1_printStringEx ("param2: ");	
			AS1_printWord16AsChars (param2);
			#endif			
		}	
	}

	if (type==CALIB_ABS_POS_SENS)
	{
		#ifdef DEBUG_CALIBRATION	
 		AS1_printStringEx ("Calibration ABS_POS aborted \r\n");
		#endif
	}

	if (type==CALIB_ABS_DIGITAL )
	{
		#ifdef DEBUG_CALIBRATION	
		can_printf ("Calibration started  %d \r\n",channel);
		#endif
		
			#ifdef DEBUG_CALIBRATION	
		can_printf ("BC ");
    	can_print_dword(_position[channel]);
		#endif	
		if (param3 >=0 && param3 <=4095) set_max_position(channel-1, param3);	
		#ifdef DEBUG_CALIBRATION	
		can_printf ("AC ");
		can_print_dword(_position[channel]);
		#endif
		if (param2>0)
		{
		
		    _position[channel] = get_position_abs_ssi(channel-1);
			_set_point[channel] = param1;
			init_trajectory (channel, _position[channel], _set_point[channel], param2);
			_in_position[channel] = false;
			_calibrated[channel] = true;
				#ifdef DEBUG_CALIBRATION
			can_printf("Calibration ABS_DIGITAL terminated %d \r\n", channel);
				#endif
		}
		if (param2==0)
		{
			_control_mode[channel]=MODE_IDLE;	
			_pad_enabled[channel] = false;
			PWM_outputPadDisable(channel);
		#ifdef DEBUG_CALIBRATION
			can_printf("Calibration ABS_DIGITAL aborted \r\n");
				#endif			
		}		
	}
/********	********
 *0x0120*	*0x0121*
 ********	********/

#elif ((VERSION == 0x0120) || (VERSION ==0x0121))

	if (type==CALIB_ABS_POS_SENS)
	{

		#ifdef DEBUG_CALIBRATION	
 		AS1_printStringEx ("Calibration ABS_POS aborted \r\n");
		#endif
	}
	if ((type==CALIB_HARD_STOPS) && (channel!=3))
	{
		if (_control_mode[channel] != MODE_IDLE && IS_DONE(channel))
		{
			_control_mode[channel] = MODE_CALIB_HARD_STOPS;	
			_counter_calib = 0;
			_pwm_calibration[channel] = param1;
			if (param2!=0)
		 		_velocity_calibration[channel]=param2;
			else
				_velocity_calibration[channel]=1;
		#ifdef DEBUG_CALIBRATION			
			can_printf ("Calibration HARD_STOPS started %d , %d \r\n",param1,param2);
		
		#endif

		}	
	}
	if ((type==CALIB_ABS_AND_INCREMENTAL) &&  (channel==3))
	{

		#ifdef DEBUG_CALIBRATION
			can_printf("Starting the calibration %d", type);	
		#endif

		if (param2>0)
		{
		    _control_mode[channel] = MODE_CALIB_ABS_AND_INCREMENTAL;
		    _set_point[channel] = param1;
			_max_position_enc_tmp[channel] = param3;

			#ifdef DEBUG_CALIBRATION		    

		    can_printf("Params are: %d, %d, %d", param1, param2, param3);

		#endif	
			init_trajectory(channel, _position[channel], _set_point[channel], param2);
		#ifdef DEBUG_CALIBRATION		
			
			AS1_printStringEx ("moving from: ");	
			AS1_printWord16AsChars (_position[channel]);	
			AS1_printStringEx ("moving to: ");	
			AS1_printWord16AsChars (_set_point[channel]);	
		#endif
		}
		if (param2==0)
		{
			_control_mode[channel]=MODE_IDLE;	
			_pad_enabled[channel] = false;
		#ifdef DEBUG_CALIBRATION			
			can_printf ("Calibration ABS_DIGITAL aborted\r\n");
		#endif			
		} 			
	}	

/********	  
 *0x0128*	 
 ********/
 
#elif ((VERSION == 0x0128) )

	if (type==CALIB_ABS_POS_SENS)
	{
		#ifdef DEBUG_CALIBRATION	
 			can_printf("Calibration ABS_POS aborted \r\n");
		#endif
	}
	if ((type==CALIB_HARD_STOPS) )
	{
		#ifdef DEBUG_CALIBRATION	
 			can_printf("Calibration HARD_STOPS aborted \r\n");
		#endif
	}
	if ((type==CALIB_ABS_DIGITAL) && (channel==0) )
	{
		#ifdef DEBUG_CALIBRATION
			can_printf("Calibration ABS_DIGITAL started \r\n");
		#endif
		if (param2>0)
		{

			_set_point[channel] = param1;
			init_trajectory(channel, _position[channel], _set_point[channel], param2);
			_calibrated[channel] = true;
		#ifdef DEBUG_CALIBRATION	
			can_printf("moving....");		
		#endif

		}
		if (param2==0)
		{
			_control_mode[channel]=MODE_IDLE;	
			_pad_enabled[channel] = false;

		#ifdef DEBUG_CALIBRATION			

			AS1_printStringEx ("Calibration ABS_DIGITAL aborted\r\n");

		#endif			

		} 			
	}	

	if ((type==CALIB_ABS_AND_INCREMENTAL) &&  (channel!=0))
	{
		#ifdef DEBUG_CALIBRATION
	  		can_printf("Starting the calibration %d", type);
		#endif
		if (param2>0)
		{
		    _control_mode[channel] = MODE_CALIB_ABS_AND_INCREMENTAL;
		    _set_point[channel] = param1;
			_max_position_enc_tmp[channel] = param3;
		#ifdef DEBUG_CALIBRATION		    
		    can_printf("Params are: %d, %d, %d", param1, param2, param3);
			can_printf ("_position");
		    can_print_dword(_position[channel]);
		    can_printf ("_set_point");
		    can_print_dword(_set_point[channel]);
		#endif	
			init_trajectory(channel, _position[channel], _set_point[channel], param2);
		}
		if (param2==0)
		{
			_control_mode[channel]=MODE_IDLE;	
			_pad_enabled[channel] = false;
		#ifdef DEBUG_CALIBRATION			
			can_printf ("Calibration ABS_DIGITAL aborted\r\n");
		#endif			
		} 			
	}
	
	/********	  
 *0x0130*	 
 ********/
 
#elif ((VERSION == 0x0130) )

if (type==CALIB_ABS_POS_SENS)
	{
		#ifdef DEBUG_CALIBRATION	
 			can_printf("Calibration ABS_POS aborted \r\n");
		#endif
	}
if ((type==CALIB_HARD_STOPS) )
	{
		#ifdef DEBUG_CALIBRATION	
 			can_printf("Calibration HARD_STOPS aborted \r\n");
		#endif
	}
	if ((type==CALIB_ABS_AND_INCREMENTAL) )
	{
		#ifdef DEBUG_CALIBRATION
			can_printf("Starting the calibration %d", type);	
		#endif
		if (param2>0)
		{
		    _control_mode[channel] = MODE_CALIB_ABS_AND_INCREMENTAL;
		    _set_point[channel] = param1;
			_max_position_enc_tmp[channel] = param3;
		#ifdef DEBUG_CALIBRATION		    
		    can_printf("Params are: %d, %d, %d", param1, param2, param3);
			can_printf ("_position");
		    can_print_dword(_position[channel]);
		    can_printf ("_set_point");
		    can_print_dword(_set_point[channel]);
		#endif	
			init_trajectory(channel, _position[channel], _set_point[channel], param2);
		#ifdef DEBUG_CALIBRATION		
			
			AS1_printStringEx ("moving from: ");	
			AS1_printWord16AsChars (_position[channel]);	
			AS1_printStringEx ("moving to: ");	
			AS1_printWord16AsChars (_set_point[channel]);	
		#endif
		}
		if (param2==0)
		{
			_control_mode[channel]=MODE_IDLE;	
			_pad_enabled[channel] = false;
		#ifdef DEBUG_CALIBRATION			
			can_printf ("Calibration ABS_DIGITAL aborted\r\n");
		#endif			
		} 			
	}	
#endif
	/* need to change this return val */
	return ERR_SPEED;
	
}
