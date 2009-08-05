#include "dsp56f807.h"
#include "options.h"
#include "pid.h"
#include "pwm_interface.h"
#include "currents_interface.h"
#include "trajectory.h"
#include "asc.h"
#include "can1.h"


/* stable global data */
#ifndef VERSION
#	error "No valid version specified"
#endif

#if VERSION == 0x0164
/* analog feedback */
#define INPOSITION_THRESHOLD 150
#else
/* digital encoder feedback */
#define INPOSITION_THRESHOLD 			60
#endif
#define INPOSITION_CALIB_THRESHOLD 		 1
bool _in_position[JN] = INIT_ARRAY (true);

Int16  _velocity_calibration[JN]=INIT_ARRAY (0);	// vel value during calibration with hard stops 

// GENERAL VARIABLES
byte    _control_mode[JN] = INIT_ARRAY (MODE_IDLE); // control mode (e.g. position, velocity, etc.) 
Int16   _fault[JN] = INIT_ARRAY (0);				// amp fault memory 
Int16   _counter = 0;								// used to count cycles, it resets now and then to generate periodic events 
Int16   _counter_calib = 0;							// used in calibration to count the number of cycles
Int16   _pwm_calibration[JN] = INIT_ARRAY (0);		// pid value during calibration with hard stops 
bool    _calibrated[JN] = INIT_ARRAY (false);
bool    _ended[];									// trajectory completed flag 
bool    _verbose = false;
bool    _pending_request = false;					// whether a request to another card is pending 
Int16   _timeout = 0;								// used to timeout requests 
//Rec_Pid _received_pid[JN];

// DEBUG VARIABLES
Int16 _debug1[JN] = INIT_ARRAY (0); 		 	// general purpouse debug
Int16 _debug2[JN] = INIT_ARRAY (0);				// general purpouse debug
byte  _t1c =0;


// POSITION VARIABLES
Int32 _abs_pos_calibration[JN] = INIT_ARRAY (0); // absolute position to be reached during calibration
Int32 _filt_abs_pos[JN] = INIT_ARRAY (0);		 // filtered absolute position sensor position
Int32 _position[JN] = INIT_ARRAY (0);			 // encoder position 
Int32 _position_old[JN] = INIT_ARRAY (0);		 // do I need to add bits for taking into account multiple rotations 

Int32 _real_position[JN]= INIT_ARRAY (0);
Int32 _real_position_old[JN]= INIT_ARRAY (0);
Int32 _desired[JN] = INIT_ARRAY (0);		 
Int16 _desired_absolute[JN] = INIT_ARRAY (0); // PD ref value for the calibration 
Int32 _set_point[JN] = INIT_ARRAY (0);  	  // set point for position [user specified] 

Int32 _min_position[JN] = INIT_ARRAY (-DEFAULT_MAX_POSITION);
Int32 _max_position[JN] = INIT_ARRAY (DEFAULT_MAX_POSITION);
Int32 _max_real_position[JN]=INIT_ARRAY (0);


// SPEED VARIABLES
Int16 _speed[JN] = INIT_ARRAY (0);			 	 		// encoder speed 
Int16 _speed_old[JN] = INIT_ARRAY (0);					// encoder old speed 
Int32 _comm_speed[JN] = INIT_ARRAY (0);		     		// brushless commutation speed 
Int16 _accu_desired_vel[JN] = INIT_ARRAY (0);			// accumultor for the fractional part of the desired vel 
Int16 _desired_vel[JN] = INIT_ARRAY (0);				// speed reference value, computed by the trajectory gen. 
Int16 _set_vel[JN] = INIT_ARRAY (DEFAULT_VELOCITY);		// set point for velocity [user specified] 
Int16 _max_vel[JN] = INIT_ARRAY (DEFAULT_MAX_VELOCITY);	// assume this limit is symmetric 
Int32 _vel_shift[JN] = INIT_ARRAY (4);


// ACCELERATION VARIABLES
Int16  _accel[JN] = INIT_ARRAY (0);			 			 // encoder acceleration 
Int16  _set_acc[JN] = INIT_ARRAY (DEFAULT_ACCELERATION); // set point for acceleration [too low!] 


// TORQUE VARIABLES
Int32 _desired_torque[JN] = INIT_ARRAY (0); 	// PID ref value, computed by the trajectory generator 


// POSITION PID VARIABLES
Int16  _error[JN] = INIT_ARRAY (0);				// actual feedback error 
Int16  _error_old[JN] = INIT_ARRAY (0);			// error at t-1 
Int16  _absolute_error[JN] = INIT_ARRAY (0);	// actual feedback error from absolute sensors
Int16  _absolute_error_old[JN] = INIT_ARRAY (0);// error at t-1 
Int16  _pid[JN] = INIT_ARRAY (0);				// pid result 
Int16  _pid_limit[JN] = INIT_ARRAY (0);			// pid limit 
Int32  _pd[JN] = INIT_ARRAY (0);              	// pd portion of the pid
float  _integral[JN] = INIT_ARRAY (0.0);		// store the sum of the integral component 
Int16  _integral_limit[JN] = INIT_ARRAY (0x7fff);

Int16  _kp[JN] = INIT_ARRAY (10);				// PID gains: proportional... 
Int16  _kd[JN] = INIT_ARRAY (40);				// ... derivative  ...
Int16  _ki[JN] = INIT_ARRAY (0);				// ... integral
Int16  _ko[JN] = INIT_ARRAY (0);				// offset 
Int16  _kr[JN] = INIT_ARRAY (3);				// scale factor (negative power of two) 

#if VERSION == 0x0170 || VERSION == 0x0171
// TORQUE PID
Int16  _error_torque[JN] ;						// actual feedback error 
Int16  _error_old_torque[JN] ;					// error at t-1 
Int16  _pid_torque[JN] ;						// pid result 
Int16  _pid_limit_torque[JN] ;					// pid limit 
Int32  _pd_torque[JN] ;           			  	// pd portion of the pid
Int32  _integral_torque[JN] ;					// store the sum of the integral component 
Int16  _integral_limit_torque[JN] ;

Int16  _kp_torque[JN] = INIT_ARRAY (10);		// PID gains: proportional... 
Int16  _kd_torque[JN] = INIT_ARRAY (40);		// ... derivative  ...
Int16  _ki_torque[JN] = INIT_ARRAY (0);			// ... integral
Int16  _ko_torque[JN] = INIT_ARRAY (0);			// offset 
Int16  _kr_torque[JN] = INIT_ARRAY (3);			// scale factor (negative power of two) 
#endif

											
#if VERSION == 0x0166
// CURRENT PID
Int32 _desired_current[JN] = INIT_ARRAY (0);	// PID ref value, computed by the trajectory generator 
Int16  _error_current[JN] = INIT_ARRAY (0);		// current error
Int16  _error_current_old[JN] = INIT_ARRAY (0);	// current error at t-1 
Int16  _kp_current[JN] = INIT_ARRAY (40);		// PID gains: proportional ... 
Int16  _kd_current[JN] = INIT_ARRAY (30);		// ... derivative  ...
Int16  _ki_current[JN] = INIT_ARRAY (1);		// integral
Int16  _kr_current[JN] = INIT_ARRAY (6);		// scale factor (negative power of two) 
Int32  _integral_current[JN] = INIT_ARRAY (0);	// store the sum of the integral component 
Int16  _current_limit[JN] = INIT_ARRAY (250);	// pid current limit 
Int32  _pd_current[JN] = INIT_ARRAY (0);        // pd portion of the current pid
#endif


#if VERSION == 0x0161
Int16 _version = 0x0161;
#elif VERSION == 0x0162
Int16 _version = 0x0162;
#elif VERSION == 0x0163
Int16 _version = 0x0163;
#elif VERSION == 0x0164
Int16 _version = 0x0164;
#elif VERSION == 0x0165
Int16 _version = 0x0165;
#elif VERSION == 0x0166
Int16 _version = 0x0166;
#endif

#if VERSION == 0x0163
Int32  _other_position[JN] = INIT_ARRAY (0);	/* the position of the synchronized card */
Int32  _adjustment[JN] = INIT_ARRAY (0);		/* the actual adjustment (compensation) */
Int32  _delta_adj[JN] = INIT_ARRAY (0);			/* velocity over the adjustment */
#endif

#ifdef SMOOTH_PID_CTRL
float _pid_old[JN] = INIT_ARRAY (0);			/* pid control at previous time step */
float _filt_pid[JN] = INIT_ARRAY (0);			/* filtered pid control*/
#endif


/*
 * compute PWM in different modalities
 */
Int32 compute_pwm(byte j)
{
	Int32 PWMoutput = 0;
	Int32 Ioutput = 0;
		
	switch (_control_mode[j])
	{
	case MODE_POSITION:
	case MODE_VELOCITY:
	case MODE_CALIB_ABS_POS_SENS:
		compute_desired(j);
		PWMoutput = compute_pid2(j);
		PWMoutput = PWMoutput + _ko[j];
		_pd[j] = _pd[j] + _ko[j];
		break;
			
	case MODE_CALIB_HARD_STOPS:
		PWMoutput = _pwm_calibration[j];
		_counter_calib +=1;		
		break;
			
	case MODE_HANDLE_HARD_STOPS:
		can_printf("MODE HANDLE HARD STOP");
	    _pad_enabled[j] = false;
	    PWM_outputPadDisable(j);
		_control_mode[j] = MODE_IDLE;
		break;	
		
	case MODE_IDLE:
		PWMoutput=0;
		break; 
	}
	
#ifdef SMOOTH_PID_CTRL
	PWMoutput = compute_filtpid(j, PWMoutput);
#endif
	return PWMoutput;
}

/*
 * compute PID control (integral is implemented).
 */
Int32 compute_pid2(byte j)
{
	Int32 ProportionalPortion, DerivativePortion, IntegralPortion;
	Int32 IntegralError;
	
	Int32 PIDoutput;
	Int32 InputError;
	byte i=0;
	byte k=0;
	static Int32 DerPort[2][10]={{0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0}};	
	Int16 Coeff[10]={1,1,1,1,1,1,1,1,1,1};
	Int16 Sum_Coeff=10; //should be equal to the sum of Coeffs
	
	/* the error @ previous cycle */
	
	_error_old[j] = _error[j];

	InputError = L_sub(_desired[j], _position[j]);
		
	if (InputError > MAX_16)
		_error[j] = MAX_16;
	else
	if (InputError < MIN_16) 
		_error[j] = MIN_16;
	else
	{
		_error[j] = extract_l(InputError);
	}
			
	/* Proportional */
	ProportionalPortion = ((Int32) _error[j]) * ((Int32)_kp[j]);
	
	if (ProportionalPortion>=0)
	{
		ProportionalPortion = ProportionalPortion >> _kr[j]; 
	}
	else
	{
		ProportionalPortion = -(-ProportionalPortion >> _kr[j]);
	}
	
	/* Derivative */	
		/* Derivative */	
	DerivativePortion = ((Int32) (_error[j]-_error_old[j])) * ((Int32) _kd[j]);

	if (DerivativePortion>=0)
	{
		DerivativePortion = DerivativePortion >> _kr[j]; 
	}
	else
	{
		DerivativePortion = -(-DerivativePortion >> _kr[j]);
	}
		
	/* Integral */
	IntegralError = ( (Int32) _error[j]) * ((Int32) _ki[j]);

	if (IntegralError>=0)
	{
		IntegralError = IntegralError >> (_kr[j]); // integral reduction 
	}
	else
	{
		IntegralError = -(-IntegralError >> (_kr[j])); // integral reduction 
	}
	
	if (IntegralError > MAX_16)
		IntegralError = (Int32) MAX_16;
	if (IntegralError < MIN_16) 
		IntegralError = (Int32) MIN_16;
	
#if VERSION == 0x0166

	_integral_current[j] = _integral_current[j] + IntegralError;
	IntegralPortion = _integral_current[j];

	_pd_current[j] = L_add(ProportionalPortion, DerivativePortion);
	PIDoutput = L_add(_pd_current[j], IntegralPortion);

#else

	_integral[j] = _integral[j] + IntegralError;
	IntegralPortion = (Int32) _integral[j];
	/* Accumulator saturation */
	if (IntegralPortion >= _integral_limit[j])
	{
		IntegralPortion = _integral_limit[j];
		_integral[j] = (float) _integral_limit[j];
	}		
	else
		if (IntegralPortion < - (_integral_limit[j]))
		{
			IntegralPortion = - (_integral_limit[j]);
			_integral[j] = (float) (-_integral_limit[j]);
		}

	_pd[j] = L_add(ProportionalPortion, DerivativePortion);
	PIDoutput = L_add(_pd[j], IntegralPortion);

#endif
					
	return PIDoutput;
}


/*
 * compute PID control (integral implemented).
 */
#if VERSION == 0x0166

Int32 compute_current_pid(byte j)
{
	Int32 ProportionalPortion, DerivativePortion, IntegralPortion;
	Int32 IntegralError;
	Int32 PIDoutput;
	Int32 InputError;
		
	/* the error @ previous cycle */
	_error_current_old[j] = _error_current[j];

	InputError = L_sub(_desired_current[j], _current[j]);
		
	if (InputError > MAX_16)
		_error_current[j] = MAX_16;
	else
	if (InputError < MIN_16) 
		_error_current[j] = MIN_16;
	else
	{
		_error_current[j] = extract_l(InputError);
	}		

	/* Proportional */
	ProportionalPortion = ((Int32) _error_current[j]) * ((Int32)_kp_current[j]);
	ProportionalPortion = ProportionalPortion >> _kr_current[j];
	/* Derivative */	
	DerivativePortion = ((Int32) (_error_current[j]-_error_current_old[j])) * ((Int32) _kd_current[j]);
	DerivativePortion = DerivativePortion >>  _kr_current[j];
	/* Integral */
	IntegralError = ( (Int32) _error_current[j]) * ((Int32) _ki_current[j]);
	IntegralError = IntegralError >> _kr_current[j];
	
	if (IntegralError > MAX_16)
		IntegralError = (Int32) MAX_16;
	if (IntegralError < MIN_16) 
		IntegralError = (Int32) MIN_16;
	
	_integral[j] = L_add(_integral[j], IntegralError);
	IntegralPortion = _integral[j];
		
	_pd[j] = L_add(ProportionalPortion, DerivativePortion);
	PIDoutput = L_add(_pd[j], IntegralPortion);
			
	return PIDoutput;
}
#endif

/* 
 * Compute PD for calibrating with the absolute postion sensors
 */
Int32 compute_pid_abs(byte j)
{
	Int32 ProportionalPortion, DerivativePortion;
	Int32 PIDoutput;
	Int16 InputError;
	Int16 Kp = 1;
	Int16 Kd = 10;
		
	/* the error @ previous cycle */
	_absolute_error_old[j] = _absolute_error[j];
	/* the errore @ current cycle */
	_absolute_error[j] = _desired_absolute[j] - extract_h(_filt_abs_pos[j]);
	//_absolute_error[j] = 0x5a0 - extract_h(_filt_abs_pos[j]);	

	/* Proportional */
	ProportionalPortion = _absolute_error[j] * Kp;
	/* Derivative */	
	DerivativePortion = (_absolute_error[j]-_absolute_error_old[j]) * Kd;
	
	PIDoutput = (ProportionalPortion + DerivativePortion);
	//AS1_printDWordAsCharsDec (PIDoutput/70);		
	

	return (PIDoutput >> 1);
}

/* 
 * this function filters the current (AD value).
 */
#ifdef SMOOTH_PID_CTRL
 
Int32 compute_filtpid(byte jnt, Int32 PID)
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
	a_1 = 0.9773
	a_2 = 0.0114
	*/
	float pid;
	static float filt_pid_old[JN] = INIT_ARRAY(0); 
	
	pid = (float) PID;
	filt_pid_old[jnt] = _filt_pid[jnt];
	_filt_pid[jnt] = 0.9773 * filt_pid_old[jnt] + 0.0114 * (_pid_old[jnt] + pid);
	_pid_old[jnt] = pid;
	return (Int32) _filt_pid[jnt];
	//return (Int32) pid;
}

#endif

/*
 * a step in the trajectory generation for velocity control. 
 */
Int32 step_velocity (byte jj)
{
	Int32 u0;
	Int16 dv, da;
	Int16 _tmp_desired_vel;
	Int16 _tmp_diff_vel;
	
	/* dv is a signed 16 bit value, need to be checked for overflow */
	if (_set_vel[jj] < -_max_vel[jj])
		_set_vel[jj] = -_max_vel[jj];
	else
	if (_set_vel[jj] > _max_vel[jj])
		_set_vel[jj] = _max_vel[jj];
	
	dv = _set_vel[jj] - _desired_vel[jj];
	da = _set_acc[jj] * CONTROLLER_PERIOD;
	
	if (__abs(dv) < da)
	{
		_desired_vel[jj] = _set_vel[jj];
	}
	else
	if (dv > 0)
	{
		_desired_vel[jj] += da;
	}
	else
	{
		_desired_vel[jj] -= da;
	}
	
	//since the desired velocity is expressed in
	//[16*encoders_tics/ms] we divide the desired
	//velocity by 16 with a shift of 4 bits
	//(more in general: _desired_vel[jj] which has a default value of 4)
	_tmp_desired_vel = (__abs(_desired_vel[jj]) >> _vel_shift[jj]);
	if (_desired_vel[jj] > 0)
		u0 =   _tmp_desired_vel * CONTROLLER_PERIOD;
	else
		u0 = - _tmp_desired_vel * CONTROLLER_PERIOD;
	
	//the additional 4 bits (which have not been used
	//in computing the desired velocity) are accumulated
	_tmp_diff_vel = __abs(_desired_vel[jj]) - (_tmp_desired_vel << _vel_shift[jj]);
	if (_desired_vel[jj] > 0)
		_accu_desired_vel[jj] = _accu_desired_vel[jj] + _tmp_diff_vel;
	else
		_accu_desired_vel[jj] = _accu_desired_vel[jj] - _tmp_diff_vel;
	
	//if accumulated additional bits overflow (i.e.
	//the fifth...sixteenth bits are different from zero)
	//the overflown part is added to the output
	//finally the accumulator is updated, taking into
	//account that the overflown bit have been considered
	_tmp_desired_vel = (__abs(_accu_desired_vel[jj]) >> _vel_shift[jj]);
	if (_desired_vel[jj] > 0)
	{
		u0 = u0 + _tmp_desired_vel * CONTROLLER_PERIOD;
		_accu_desired_vel[jj] = _accu_desired_vel[jj] - (_tmp_desired_vel<<_vel_shift[jj]);
	}
		
	else
	{
		u0 = u0 - _tmp_desired_vel * CONTROLLER_PERIOD;
		_accu_desired_vel[jj] = _accu_desired_vel[jj] + (_tmp_desired_vel<<_vel_shift[jj]);
	}
	
	return u0;
}


/*
 * helper function to generate desired position.
 */
void compute_desired(byte i)
{		
 	Int32 previous_desired;
	
	if (_control_mode[i] != MODE_IDLE)
	{
		previous_desired = _desired[i];
		
		/* compute trajectory and control mode */
		switch (_control_mode[i])
		{
		case MODE_POSITION:
			_desired[i] = step_trajectory (i);
			break;
			
		case MODE_CALIB_ABS_POS_SENS:
		
			_desired_absolute[i] = step_trajectory (i);
			
			/* The following lines handle two possible situations:
				(1) the absolute position sensor increseas 
					when the encoder increases
				(2) the absolute position sensor increseas 
					when the encoder increases */
		/*		
			if (VERSION == 0x0112 && i == 0)
				_desired[i] = _desired[i] - compute_pid_abs (i);
			else
				_desired[i] = _desired[i] + compute_pid_abs (i);
		*/	
			break;
							
		case MODE_VELOCITY:
			_desired[i] += step_trajectory_delta (i);
			_desired[i] += step_velocity (i);
			break;
		}
		
		check_desired_within_limits(i, previous_desired);
	}
}

/***************************************************************** 
 * Sets back the desired position to a legal one
 * in case joint limits have been exceeded.
 *****************************************************************/
void check_desired_within_limits(byte i, Int32 previous_desired)
{
	if (_control_mode[i]!=MODE_CALIB_ABS_POS_SENS)
	{
		if (_desired[i] < _min_position[i] && (_desired[i] - previous_desired) < 0) 
		{
			_desired[i] = _min_position[i];
			if (_control_mode[i] == MODE_VELOCITY)
				_set_vel[i] = 0;
		}
		if (_desired[i] > _max_position[i] && (_desired[i] - previous_desired) > 0)
		{
			_desired[i] = _max_position[i];
			if (_control_mode[i] == MODE_VELOCITY)
				_set_vel[i] = 0;
		}
	}
}

/***************************************************************** 
 * this function checks if the trajectory is terminated
 * and if trajectory is terminated sets the variable _in_position
 *****************************************************************/
bool check_in_position(byte jnt)
{
	if (_control_mode[jnt] == MODE_POSITION)
	{
		if (__abs(_position[jnt] - _set_point[jnt]) < INPOSITION_THRESHOLD && _ended[jnt])
			return true;
		else
			return false;
	}				
}