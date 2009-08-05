#include "dsp56f807.h"
#include "options.h"
#include "pid.h"
#include "pwm_interface.h"
#include "currents_interface.h"
#include "trajectory.h"

/* stable global data */
bool _calibrated[JN] = INIT_ARRAY (false);
bool _pad_enabled[JN] = INIT_ARRAY (false);
bool _verbose = false;

#ifndef VERSION
#	error "No valid version specified"
#endif

#if VERSION == 0x0114
/* analog feedback */
#define INPOSITION_THRESHOLD 150
#else
/* digital encoder feedback */
#define INPOSITION_THRESHOLD 			60
#endif
#define INPOSITION_CALIB_THRESHOLD 		 1
bool _in_position[JN] = INIT_ARRAY (true);

byte _control_mode[JN] = INIT_ARRAY (MODE_IDLE);
											/* control mode (e.g. position, velocity, etc.) */

Int32 _abs_pos_calibration[JN] = INIT_ARRAY (0); /* absolute position to be reached during calibration*/
Int32 _filt_abs_pos[JN] = INIT_ARRAY (0);		 /* filtered absolute position sensor position*/
Int32 _position[JN] = INIT_ARRAY (0);			 /* encoder position */
Int32 _position_old[JN] = INIT_ARRAY (0);		 /* do I need to add bits for taking into account multiple rotations */
Int32 _speed[JN] = INIT_ARRAY (0);			 	 /* encoder speed */

Int32 _desired[JN] = INIT_ARRAY (0);		  /* PID ref value, computed by the trajectory generator */
Int16 _desired_absolute[JN] = INIT_ARRAY (0); /* PD ref value for the calibration */
Int32 _set_point[JN] = INIT_ARRAY (0);  	  /* set point for position [user specified] */

Int32 _min_position[JN] = INIT_ARRAY (-DEFAULT_MAX_POSITION);
Int32 _max_position[JN] = INIT_ARRAY (DEFAULT_MAX_POSITION);
											/* software position limits */
										
Int16  _desired_vel[JN] = INIT_ARRAY (0);			/* speed reference value, computed by the trajectory gen. */
Int16  _set_vel[JN] = INIT_ARRAY (DEFAULT_VELOCITY);	
											/* set point for velocity [user specified] */
Int16  _max_vel[JN] = INIT_ARRAY (DEFAULT_MAX_VELOCITY);
											/* assume this limit is symmetric */
										
Int16  _set_acc[JN] = INIT_ARRAY (DEFAULT_ACCELERATION);
											/* set point for acceleration [too low!] */
Int32  _integral[JN] = INIT_ARRAY (0);		/* store the sum of the integral component */
Int16  _integral_limit[JN] = INIT_ARRAY (0x7fff);

Int16  _error[JN] = INIT_ARRAY (0);				/* actual feedback error */
Int16  _error_old[JN] = INIT_ARRAY (0);			/* error at t-1 */
Int16  _absolute_error[JN] = INIT_ARRAY (0);	/* actual feedback error from absolute sensors*/
Int16  _absolute_error_old[JN] = INIT_ARRAY (0);/* error at t-1 */
Int16  _fault[JN] = INIT_ARRAY (0);				/* amp fault memory */

Int16  _pid[JN] = INIT_ARRAY (0);				/* pid result */
Int16  _pid_limit[JN] = INIT_ARRAY (0);			/* pid limit */
Int16  _pwm_calibration[JN] = INIT_ARRAY (0);	/* pid value during calibration with hard stops */
Int32  _pd[JN] = INIT_ARRAY (0);              	/* pd portion of the pid*/

Int16  _kp[JN] = INIT_ARRAY (10);				/* PID gains: proportional... */
Int16  _kd[JN] = INIT_ARRAY (40);				/* ... derivative  ...*/
Int16  _ki[JN] = INIT_ARRAY (0);				/* ... integral*/
Int16  _ko[JN] = INIT_ARRAY (0);				/* offset */
Int16  _kr[JN] = INIT_ARRAY (3);				/* scale factor (negative power of two) */

Int16 _counter = 0;							/* used to count cycles, it resets now and then */
											/* to generate periodic events */
Int16 _counter_calib = 0;					/* used in calibration to count the number of cycles*/											
											
#if VERSION == 0x0116
Int32 _desired_current[JN] = INIT_ARRAY (0);	/* PID ref value, computed by the trajectory generator */
Int16  _error_current[JN] = INIT_ARRAY (0);		/* current error*/
Int16  _error_current_old[JN] = INIT_ARRAY (0);	/* current error at t-1 */
Int16  _kp_current[JN] = INIT_ARRAY (40);		/* PID gains: proportional ... */
Int16  _kd_current[JN] = INIT_ARRAY (20);		/* ... derivative  ...*/
Int16  _ki_current[JN] = INIT_ARRAY (1);		/* integral*/
Int16  _kr_current[JN] = INIT_ARRAY (6);		/* scale factor (negative power of two) */
Int32  _integral_current[JN] = INIT_ARRAY (0);	/* store the sum of the integral component */
Int16  _current_limit[JN] = INIT_ARRAY (350);	/* pid current limit */
Int32  _pd_current[JN] = INIT_ARRAY (0);         /* pd portion of the current pid*/
#endif

/*
 * version specific global variables.
 */
#if VERSION == 0x0111
Int16 _version = 0x0111;
#elif VERSION == 0x0112
Int16 _version = 0x0112;
#elif VERSION == 0x0113
Int16 _version = 0x0113;
#elif VERSION == 0x0114
Int16 _version = 0x0114;
#elif VERSION == 0x0115
Int16 _version = 0x0115;
#elif VERSION == 0x0116
Int16 _version = 0x0116;
#endif

#if VERSION == 0x0113
Int32  _other_position[JN] = INIT_ARRAY (0);	/* the position of the synchronized card */
Int32  _adjustment[JN] = INIT_ARRAY (0);		/* the actual adjustment (compensation) */
Int32  _delta_adj[JN] = INIT_ARRAY (0);			/* velocity over the adjustment */
#endif

#ifdef SMOOTH_PID_CTRL
float _pid_old[JN] = INIT_ARRAY (0);			/* pid control at previous time step */
float _filt_pid[JN] = INIT_ARRAY (0);			/* filtered pid control*/
#endif

bool _pending_request = false;			/* whether a request to another card is pending */
Int16  _timeout = 0;					/* used to timeout requests */

bool _ended[];							/* trajectory completed flag */

/*
 * compute PWM in different modalities
 */
Int32 compute_pwm(byte j)
{
	Int32 PWMoutput = 0;
	Int32 Ioutput = 0;
		
	switch (_control_mode[j])
	{
#if VERSION == 0x0116
/*
	case MODE_POSITION:
	case MODE_VELOCITY:
	case MODE_CALIB_ABS_POS_SENS:
		compute_desired(j);
		Ioutput = compute_pid2(j);		
		ENFORCE_CURRENT_LIMITS(j, Ioutput);
		PWMoutput = compute_current_pid(j);
		break;
*/
#else

	case MODE_POSITION:
	case MODE_VELOCITY:
	case MODE_CALIB_ABS_POS_SENS:
		
		//generate trajectory
		compute_desired(j);
		
		/*
		// use this to disable trajectory generator
		// compute desired(j);
		_desired[j]=_set_point[j]; 
		*/
		
		PWMoutput = compute_pid2(j);
		break;

#endif
			
	case MODE_CALIB_HARD_STOPS:
		PWMoutput = _pwm_calibration[j];
		_counter_calib +=1;		
		break;
			
	case MODE_HANDLE_HARD_STOPS:
	    _pad_enabled[j] = false;
	    PWM_outputPadDisable(j);
		_control_mode[j] = MODE_IDLE;
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
	ProportionalPortion = ProportionalPortion >> _kr[j];
	/* Derivative */	
	DerivativePortion = ((Int32) (_error[j]-_error_old[j])) * ((Int32) _kd[j]);
	DerivativePortion = DerivativePortion >>  _kr[j];
	/* Integral */
	IntegralError = ( (Int32) _error[j]) * ((Int32) _ki[j]);
	IntegralError = IntegralError >> _kr[j];
	
	if (IntegralError > MAX_16)
		IntegralError = (Int32) MAX_16;
	if (IntegralError < MIN_16) 
		IntegralError = (Int32) MIN_16;

#if VERSION == 0x0116

	_integral_current[j] = L_add(_integral_current[j], IntegralError);
	IntegralPortion = _integral_current[j];

	_pd_current[j] = L_add(ProportionalPortion, DerivativePortion);
	PIDoutput = L_add(_pd_current[j], IntegralPortion);

#else

	_integral[j] = L_add(_integral[j], IntegralError);
	IntegralPortion = _integral[j];

	_pd[j] = L_add(ProportionalPortion, DerivativePortion);
	PIDoutput = L_add(_pd[j], IntegralPortion);

#endif
					
	return PIDoutput;
}


/*
 * compute PID control (integral implemented).
 */
#if VERSION == 0x0116

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
	
	u0 = _desired[jj] + _desired_vel[jj] * CONTROLLER_PERIOD;
	return u0;
}

/*
 * helper function to generate desired position.
 */
void compute_desired(byte i)
{		
 	Int32 cd;
	
	if (_control_mode[i] != MODE_IDLE)
	{
		cd = _desired[i];
		
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
				
			if (VERSION == 0x0112 && i == 0)
				_desired[i] = _desired[i] - compute_pid_abs (i);
			else
				_desired[i] = _desired[i] + compute_pid_abs (i);
			
			break;
							
		case MODE_VELOCITY:
			_desired[i] = step_velocity (i);
			if (_desired[i] < _min_position[i] && (_desired[i] - cd) < 0) 
			{
				_desired[i] = _min_position[i];
				_set_vel[i] = 0;
			}
			else
			if (_desired[i] > _max_position[i] && (_desired[i] - cd) > 0)
			{
				_desired[i] = _max_position[i];
				_set_vel[i] = 0;
			}
			break;
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

/***************************************************************** 
 * 
 *****************************************************************/
/*void ENFORCE_LIMITS2(byte j, Int32* PID)
{ 
	// Anti reset wind up scheme 
	if (*PID > _pid_limit[j]) 
    { 
    	if ( _ki[j] != 0) 
		{ 
    		_integral[j] =  ((Int32) _pid_limit[j]) - _pd[j]; 
		} 
	} 
	else 
	{
		if (*PID < -_pid_limit[j]) 
		{ 
			if ( _ki[j] != 0) 
			{ 
	    		_integral[j] =  ((Int32) (-_pid_limit[j])) - _pd[j]; 
			} 
		} 
	}
	// Accumulator saturation  \
	if (_integral[j] >= (Int32)_integral_limit[j]) 
    { 
		_integral[j] = (Int32) _integral_limit[j]; 
	} 
	else 
	{ 
		if (_integral[j] < - ((Int32)_integral_limit[j])) 
		{ 
			_integral[j] = - ((Int32)_integral_limit[j]); 
		} 
	} 
	// Control saturation  
	if (*PID > (Int32) _pid_limit[j]) 
    	_pid[j] = _pid_limit[j]; 
	else 
	{
	if ( *PID < - ((Int32) _pid_limit[j])) 
		_pid[j] =  -_pid_limit[j]; 
	else 
		_pid[j] = (Int16)(*PID); 
	}
	// Control hard limit reached ? i.e. abnormal high current ? 
	if (_control_mode[j] == MODE_POSITION) 
	{
		if (_filt_current[j] > _max_allowed_current[j] / 2) 
		{
//			AS1_printStringEx ("R: "); 
			_control_mode[j] = MODE_HANDLE_HARD_STOPS; 
//			AS1_printWord16AsChars(_pid[j]); 
			_pid[j] = 0; 
			*PID=0;
		}
	}
}
*/