
#ifndef __pidh__
#define __pidh__

#include "dsp56f807.h"
#include "controller.h"
#include "options.h"

#ifndef VERSION
#	error "No valid version specified"
#endif

/******************************************************/
// pid prototypes 
/******************************************************/

Int32 compute_pwm(byte j);
Int32 compute_pid2(byte j);
Int32 compute_current_pid(byte j);
Int32 compute_pid_abs(byte j);
Int32 compute_filtpid(byte jnt, Int32 PID);
Int32 step_velocity (byte jj);
void compute_desired(byte i);
bool check_in_position(byte jnt);
void check_desired_within_limits(byte i, Int32 previous_desired);

/******************************************************/
// stable global data 
/******************************************************/

extern bool _calibrated[JN] ;
extern bool _pad_enabled[JN] ;
extern bool _verbose ;

#if VERSION == 0x0188
// analog feedback 
#define INPOSITION_THRESHOLD 150
#else
// digital encoder feedback 
#define INPOSITION_THRESHOLD 			60
#endif
#define INPOSITION_CALIB_THRESHOLD 		 1
extern bool _in_position[JN] ;

#define IS_DONE(jj) (_ended[jj])
extern bool _ended[];

extern byte _control_mode[JN] ;
											// control mode (e.g. position, velocity, etc.) 
extern Int32 _abs_pos_calibration[JN] ;		// absolute position to be reached during calibration
extern Int32 _filt_abs_pos[JN] ;			// filtered absolute position sensor position
extern Int32 _position[JN] ;				// encoder position 
extern Int32 _position_old[JN] ;			// do I need to add bits for taking into account multiple rotations 
extern Int16 _speed[JN] ;					// encoder speed 

extern Int16 _speed_old[JN]; 	//previous speed
extern Int16 _accel[JN]; 		//encoder estimated acceleration

extern Int32 _comm_speed[JN] ;			    // brushless commutation speed 
extern Int32 _real_position[JN];
extern Int32 _real_position_old[JN];
extern Int32 _desired[JN] ;				// PID ref value, computed by the trajectory generator 
extern Int16 _desired_absolute[JN] ;		// PD ref value for the calibration 
extern Int32 _set_point[JN] ;			// set point for position [user specified] 

extern Int32 _min_position[JN] ;
extern Int32 _max_position[JN] ;		// software position limits 
extern Int32 _vel_shift[JN]	;
extern Int32 _max_real_position[JN];										
extern Int16  _desired_vel[JN] ;			// speed reference value, computed by the trajectory gen. 
extern Int16  _set_vel[JN] ;
											// set point for velocity [user specified] 
extern Int16  _max_vel[JN] ;
											// assume this limit is symmetric 
										
extern Int16  _set_acc[JN] ;
											// set point for acceleration [too low!] 
extern Int32  _integral[JN] ;			// store the sum of the integral component 
extern Int16  _integral_limit[JN] ;

extern Int16  _error[JN] ;				// actual feedback error 
extern Int16  _error_old[JN] ;			// error at t-1 
extern Int16  _absolute_error[JN] ;		// actual feedback error from absolute sensors
extern Int16  _absolute_error_old[JN] ;	// error at t-1 
extern Int16  _fault[JN] ;				// amp fault memory 
extern Int16  _pid[JN] ;					// pid result 
extern Int16  _pid_limit[JN] ;			// pid limit 
extern Int16  _pwm_calibration[JN] ;		// pid value during calibration with hard stops 
extern Int32  _pd[JN] ;              	// pd portion of the pid

extern Int16  _kp[JN] ;				// PID gains: proportional... 
extern Int16  _kd[JN] ;				// ... derivative  ...
extern Int16  _ki[JN] ;					// ... integral
extern Int16  _ko[JN] ;					// offset 
extern Int16  _kr[JN] ;					// scale factor (negative power of two) 

extern Int16 _counter ;							// used to count cycles, it resets now and then 
											// to generate periodic events 
extern Int16 _counter_calib ;					// used in calibration to count the number of cycles											
											
#if VERSION == 0x0186
extern Int32 _desired_current[JN] ;		// PID ref value, computed by the trajectory generator 
extern Int16  _error_current[JN] ;		// current error
extern Int16  _error_current_old[JN] ;	// current error at t-1 
extern Int16  _kp_current[JN] ;		// PID gains: proportional ... 
extern Int16  _kd_current[JN] ;		// ... derivative  ...
extern Int16  _ki_current[JN] ;			// integral
extern Int16  _kr_current[JN] ;			// scale factor (negative power of two) 
extern Int32  _integral_current[JN] ;	// store the sum of the integral component 
extern Int16  _current_limit[JN] ;	// pid current limit 
extern Int32  _pd_current[JN] ;          // pd portion of the current pid
#endif

/*
 * version specific global variables.
 */

extern Int16 _version ;

#if VERSION == 0x0183
extern Int32  _other_position[JN] ;	// the position of the synchronized card 
extern Int32  _adjustment[JN] ;		// the actual adjustment (compensation) 
extern Int32  _delta_adj[JN] ;		// velocity over the adjustment 
#endif

#ifdef SMOOTH_PID_CTRL
extern float _pid_old[JN] ;			// pid control at previous time step 
extern float _filt_pid[JN] ;			// filtered pid control
#endif

extern bool _pending_request ;			// whether a request to another card is pending 
extern Int16  _timeout ;					// used to timeout requests 




/******************************************************/
// macro
/******************************************************/

//void enforce_PIDlimits(byte j, Int32 PIDoutput)
#define ENFORCE_CURRENT_LIMITS(j, I) \
{ \
	/* Anti reset wind up scheme */ \
	if (I > _current_limit[j]) \
    { \
    	if ( _ki[j] != 0) \
		{ \
    		_integral_current[j] =  ((Int32) _current_limit[j]) - _pd_current[j]; \
		} \
	} \
	else \
	{\
		if (I < -_current_limit[j]) \
		{ \
			if ( _ki[j] != 0) \
			{ \
	    		_integral_current[j] =  ((Int32) (-_current_limit[j])) - _pd_current[j]; \
			} \
		} \
	}\
	/* Control saturation */ \
	if (I > (Int32) _current_limit[j]) \
    	_desired_current[j] = _current_limit[j]; \
	else \
	{\
	if ( I < - ((Int32) _current_limit[j])) \
		_desired_current[j] =  -_current_limit[j]; \
	else \
		_desired_current[j] = (Int16)(I); \
	}\
}


//void enforce_PIDlimits(byte j, Int32 PIDoutput)
#define ENFORCE_LIMITS(j, PID) \
{ \
	/* Anti reset wind up scheme */ \
	if (PID > _pid_limit[j]) \
    { \
    	if ( _ki[j] != 0) \
		{ \
    		_integral[j] =  ((Int32) _pid_limit[j]) - _pd[j]; \
		} \
	} \
	else \
	{\
		if (PID < -_pid_limit[j]) \
		{ \
			if ( _ki[j] != 0) \
			{ \
	    		_integral[j] =  ((Int32) (-_pid_limit[j])) - _pd[j]; \
			} \
		} \
	}\
	/* Control saturation */ \
	if (PID > (Int32) _pid_limit[j]) \
    	_pid[j] = _pid_limit[j]; \
	else \
	{\
	if ( PID < - ((Int32) _pid_limit[j])) \
		_pid[j] =  -_pid_limit[j]; \
	else \
		_pid[j] = (Int16)(PID); \
	}\
	/* Control hard limit reached ? i.e. abnormal high current ? */ \
	if (_control_mode[j] == MODE_POSITION) \
	{\
		if (_filt_current[j] > _max_allowed_current[j] ) \
		{\
			_control_mode[j] = MODE_HANDLE_HARD_STOPS; \
			_pid[j] = 0; \
			if      (j == 0) PWMoutput[0] = 0; \
			else if (j == 1) PWMoutput[1] = 0; \
			else if (j == 2) PWMoutput[2] = 0; \
			else if (j == 3) PWMoutput[3] = 0; \
		}\
	}\
}



#endif