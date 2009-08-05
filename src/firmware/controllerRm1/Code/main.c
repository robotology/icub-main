/*
 * firmware/controller application.
 *
 */

#include "dsp56f807.h"
#include "asc.h"
#include "can1.h"
#include "qd0.h"
#include "qd1.h"
#include "pwmc0.h"
#include "pwmc1.h"
#include "ti1.h"
#include "ifsh1.h"
#include "ad.h"

#include "controller.h"
#include "messages.h"

/*
 * prototypes.
 */
void isrIRQA ();
int get_flash_addr (void);
extern void bootStart(void);  /* Defined in linker.cmd file */ 

/*
 * test irs.
 */
#pragma interrupt saveall
void isrIRQA ()
{
	AS1_sendCharSafe('*');
}

/* stable global data */
bool _calibrated[JN] = { false, false };
bool _pad_enabled[JN] = { false, false };
bool _verbose = false;

#if VERSION == 0x0114
/* analog feedback */
#define INPOSITION_THRESHOLD 150
#else
/* digital encoder feedback */
#define INPOSITION_THRESHOLD 			60
#endif
#define INPOSITION_CALIB_THRESHOLD 		 1
bool _in_position[JN] = { true, true };

byte _control_mode[JN] = { MODE_IDLE, MODE_IDLE };
											/* control mode (e.g. position, velocity, etc.) */

Int16 _abs_position[JN] = { 0, 0 };			/* absolute position sensor position*/
Int32 _abs_position_old[JN] = { 0, 0 };		/* scaled absolute position at previous time step*/
Int32 _abs_pos_calibration[JN] = {0,0};		/* absolute position to be reached during calibration*/
Int32 _filt_abs_pos[JN] = { 0, 0 };			/* filtered absolute position sensor position*/
Int32 _filt_abs_pos_old[JN] = { 0, 0 };		/* filtered absolute position at previous time step*/
Int32 _position[JN] = { 0, 0 };				/* encoder position */
Int32 _position_old[JN] = { 0, 0 };			/* do I need to add bits for taking into account multiple rotations */
Int32 _speed[JN] = { 0, 0 };				/* encoder speed */

Int32 _desired[JN] = { 0, 0 };				/* PID ref value, computed by the trajectory generator */
Int16 _desired_absolute[JN] = { 0, 0 };		/* PD ref value for the calibration */
Int32 _set_point[JN] = { 0, 0 };			/* set point for position [user specified] */

Int32 _min_position[JN] = { -DEFAULT_MAX_POSITION, -DEFAULT_MAX_POSITION };
Int32 _max_position[JN] = { DEFAULT_MAX_POSITION, DEFAULT_MAX_POSITION };
											/* software position limits */
										
Int16  _desired_vel[JN] = { 0, 0 };			/* speed reference value, computed by the trajectory gen. */
Int16  _accu_desired_vel[JN] = { 0, 0 };	/* accumultor for the fractional part of the desired vel */
Int16  _set_vel[JN] = { DEFAULT_VELOCITY, DEFAULT_VELOCITY };	
											/* set point for velocity [user specified] */
Int16  _max_vel[JN] = { DEFAULT_MAX_VELOCITY, DEFAULT_MAX_VELOCITY };
											/* assume this limit is symmetric */
										
Int16  _set_acc[JN] = { DEFAULT_ACCELERATION, DEFAULT_ACCELERATION };
											/* set point for acceleration [too low!] */
Int32  _integral[JN] = { 0, 0 };			/* store the sum of the integral component */
Int16  _integral_limit[JN] = { 0x7fff, 0x7fff };

Int16  _error[JN] = { 0, 0 };				/* actual feedback error */
Int16  _error_old[JN] = { 0, 0 };			/* error at t-1 */
Int16  _absolute_error[JN] = { 0, 0 };		/* actual feedback error from absolute sensors*/
Int16  _absolute_error_old[JN] = { 0, 0 };	/* error at t-1 */
Int16  _fault[JN] = { 0, 0 };				/* amp fault memory */

Int16  _pid[JN] = { 0, 0 };					/* pid result */
Int16  _pid_limit[JN] = { 0, 0 };			/* pid limit */
Int16  _pwm_calibration[JN] = { 0, 0 };		/* pid value during calibration with hard stops */
Int32  _pd[JN] = { 0, 0 };              	/* pd portion of the pid*/

Int16  _kp[JN] = { 10, 10 };				/* PID gains: proportional... */
Int16  _kd[JN] = { 40, 40 };				/* ... derivative  ...*/
Int16  _ki[JN] = { 0, 0 };					/* ... integral*/
Int16  _ko[JN] = { 0, 0 };					/* offset */
Int16  _kr[JN] = { 3, 3 };					/* scale factor (negative power of two) */

Int16 _counter = 0;							/* used to count cycles, it resets now and then */
											/* to generate periodic events */
Int16 _counter_calib = 0;					/* used in calibration to count the number of cycles*/											
											
Int16 _flash_addr = 0;
byte _write_buffer = 0;						/* the current CAN bus buffer, buffers alternate */

Int32 _current[JN] = { 0, 0 };				/* current through the transistors*/
Int32 _current_old[JN] = { 0, 0 };			/* current at t-1*/

#if VERSION == 0x0116
Int32 _desired_current[JN] = { 0, 0 };		/* PID ref value, computed by the trajectory generator */
Int16  _error_current[JN] = { 0, 0 };		/* current error*/
Int16  _error_current_old[JN] = { 0, 0 };	/* current error at t-1 */
Int16  _kp_current[JN] = { 40, 40 };		/* PID gains: proportional ... */
Int16  _kd_current[JN] = { 30, 30 };		/* ... derivative  ...*/
Int16  _ki_current[JN] = { 1, 1 };			/* integral*/
Int16  _kr_current[JN] = { 6, 6 };			/* scale factor (negative power of two) */
Int32  _integral_current[JN] = { 0, 0 };	/* store the sum of the integral component */
Int16  _current_limit[JN] = { 250, 250 };	/* pid current limit */
Int32  _pd_current[JN] = { 0, 0 };          /* pd portion of the current pid*/
#endif

dword _filt_current[JN] = { 0, 0 };     	/* filtered current through the transistors*/
dword _filt_current_old[JN] = { 0, 0 }; 	/* filtered current at t-1*/
dword _max_allowed_current[JN] = { 4000000, 4000000 }; 
											/* limit on the current in micro-ampere*/
										
dword _broadcast_mask = 0;					/* specifies which broadcast messages are to be sent */
float _conversion_factor[JN] = { 0f, 0f };	/* limit on the current as set by the interface (later converted into the filter parameter) */

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
Int32  _other_position[JN] = { 0, 0 };	/* the position of the synchronized card */
Int32  _adjustment[JN] = { 0, 0 };		/* the actual adjustment (compensation) */
Int32  _delta_adj[JN] = { 0, 0 };		/* velocity over the adjustment */
#endif

#ifdef SMOOTH_PID_CTRL
float _pid_old[JN] = { 0, 0 };			/* pid control at previous time step */
float _filt_pid[JN] = { 0, 0 };			/* filtered pid control*/
float _filt_pid_old[JN] = { 0, 0 };		/* filtered pid control at previous time step*/
#endif

bool _pending_request = false;			/* whether a request to another card is pending */
Int16  _timeout = 0;					/* used to timeout requests */

/* CAN bus communication global vars */
canmsg_t can_fifo[CAN_FIFO_LEN];
Int16 write_p = 0;
Int16 read_p = -1;					/* -1 means empty, last_read == fifo_ptr means full */
canmsg_t _canmsg;					/* buffer to prepare messages for send */

byte	_board_ID = DEFAULT_BOARD_ID;	/* */
byte	_general_board_error = ERROR_NONE;

volatile bool _wait = true;				/* wait on timer variable */

extern bool _ended[];					/* trajectory completed flag */
#define IS_DONE(jj) (_ended[jj])

/* Local prototypes */
Int32 compute_pwm(byte j);
Int32 compute_pid2(byte j);
Int32 compute_pid_abs(byte j);
void compute_desired(byte j);
void print_version(void);
void check_current(byte j);
void compute_filtcurr(byte j);
void compute_filt_pos(byte j);
void set_can_masks(void);
void check_in_position_calib(byte j);
void check_in_position(byte j);
void get_postion(byte jnt);
void get_abs_postion(byte jnt);
void decouple_positions(void);

#if VERSION == 0x0116
Int32 compute_current_pid(byte j);
#endif

#ifdef SMOOTH_PID_CTRL
Int32 compute_filtpid(byte j, Int32 PID);
#endif


void set_can_masks()
{
	Int32 acceptance_code = 0x0;
	Int32 acceptance_code2 = 0x0;
	word temporary;
		
#if VERSION == 0x0113
#define NEIGHBOR 11	
	CAN1_setAcceptanceMask (0xff011f1e, 0x1f1e1f1e);
	temporary = (NEIGHBOR << 1);
	temporary |= (0x0020);
	acceptance_code = L_deposit_h (temporary); 
	temporary = _board_ID >> 3;
	temporary |= (_board_ID << 13);
	temporary &= (0xff1f);
	acceptance_code |= temporary;
	acceptance_code2 = 0xe0e10000;
	temporary = _board_ID >> 3;
	temporary |= (_board_ID << 13);
	temporary |= (0x00e0);
	acceptance_code2 |= temporary;	
	CAN1_setAcceptanceCode (acceptance_code, acceptance_code2);
#undef NEIGHBOR
#else
	CAN1_setAcceptanceMask (0x1f1e1f1e, 0x1f1e1f1e);
	//acceptance_code = 0x00100010;
	temporary = _board_ID >> 3;
	temporary |= (_board_ID << 13);
	temporary &= (0xff1f);
	acceptance_code = L_deposit_h (temporary);
	acceptance_code |= temporary;
	acceptance_code2 = 0xe0e10000;
	temporary = _board_ID >> 3;
	temporary |= (_board_ID << 13);
	temporary |= (0x00e0);
	acceptance_code2 |= temporary;	
	CAN1_setAcceptanceCode (acceptance_code, acceptance_code2);
#endif	
}

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
	/* Accumulator saturation */ \
	if (_integral[j] >= (Int32)_integral_limit[j]) \
    { \
		_integral[j] = (Int32) _integral_limit[j]; \
	} \
	else \
	{ \
		if (_integral[j] < - ((Int32)_integral_limit[j])) \
		{ \
			_integral[j] = - ((Int32)_integral_limit[j]); \
		} \
	} \
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
		if (_filt_current[j] > _max_allowed_current[j] / 2) \
		{\
			AS1_printStringEx ("R: "); \
			_control_mode[j] = MODE_HANDLE_HARD_STOPS; \
			AS1_printWord16AsChars(_pid[j]); \
			_pid[j] = 0; \
			if (j == 0)\
			{\
				PWMoutput0 = 0;\
			}\
			else\
			{\
				PWMoutput1 = 0;\
			}\
		}\
	}\
}


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

	case MODE_POSITION:
	case MODE_VELOCITY:
	case MODE_CALIB_ABS_POS_SENS:
		compute_desired(j);
		Ioutput = compute_pid2(j);		
		ENFORCE_CURRENT_LIMITS(j, Ioutput);
		PWMoutput = compute_current_pid(j);
		break;

#else

	case MODE_POSITION:
	case MODE_VELOCITY:
	case MODE_CALIB_ABS_POS_SENS:
		compute_desired(j);
		PWMoutput = compute_pid2(j);
#if (VERSION == 0x0112 || VERSION == 0x0113)
		PWMoutput = PWMoutput + _ko[j];
		_pd[j] = _pd[j] + _ko[j];
#endif		
		break;

#endif
			
	case MODE_CALIB_HARD_STOPS:
		PWMoutput = _pwm_calibration[j];
		_counter_calib +=1;		
		break;
			
	case MODE_HANDLE_HARD_STOPS:
	    _pad_enabled[j] = false;
	    if (j == 0)
			PWMC0_outputPadDisable();
	    else
	    	PWMC1_outputPadDisable();
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
	
	pid = (float) PID;
	_filt_pid_old[jnt] = _filt_pid[jnt];
	_filt_pid[jnt] = 0.9773 * _filt_pid_old[jnt] + 0.0114 * (_pid_old[jnt] + pid);
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
	_tmp_desired_vel = (__abs(_desired_vel[jj]) >> 4);
	if (_desired_vel[jj] > 0)
		u0 =   _tmp_desired_vel * CONTROLLER_PERIOD;
	else
		u0 = - _tmp_desired_vel * CONTROLLER_PERIOD;
	
	//the additional 4 bits (which have not been used
	//in computing the desired velocity) are accumulated
	_tmp_diff_vel = __abs(_desired_vel[jj]) - (_tmp_desired_vel << 4);
	if (_desired_vel[jj] > 0)
		_accu_desired_vel[jj] = _accu_desired_vel[jj] + _tmp_diff_vel;
	else
		_accu_desired_vel[jj] = _accu_desired_vel[jj] - _tmp_diff_vel;
	
	//if accumulated additional bits overflow (i.e.
	//the fifth...sixteenth bits are different from zero)
	//the overflown part is added to the output
	//finally the accumulator is updated, taking into
	//account that the overflown bit have been considered
	_tmp_desired_vel = (__abs(_accu_desired_vel[jj]) >> 4);
	if (_desired_vel[jj] > 0)
	{
		u0 = u0 + _tmp_desired_vel * CONTROLLER_PERIOD;
		_accu_desired_vel[jj] = _accu_desired_vel[jj] - (_tmp_desired_vel<<4);
	}
		
	else
	{
		u0 = u0 - _tmp_desired_vel * CONTROLLER_PERIOD;
		_accu_desired_vel[jj] = _accu_desired_vel[jj] + (_tmp_desired_vel<<4);
	}
	
	return u0;
}


/*
 * flash memory functions.
 * LATER: add all relevant variables.
 */
byte writeToFlash (word addr)
{
	dword ptr = (dword)addr;
	byte i, err;
	word tmp;
	bool gerr = false;

	IFsh1_eraseFlash (addr);

	tmp = BYTE_W(_board_ID, 0);
	err = IFsh1_setWordFlash(ptr, tmp);
	gerr |= (err != ERR_OK);
	ADP(ptr,2);
	err = IFsh1_setWordFlash(ptr, _version);
	gerr |= (err != ERR_OK);
	ADP(ptr,2);
	
	for (i = 0; i < JN; i++)
	{
		err = IFsh1_setWordFlash(ptr, _kp[i]);
		gerr |= (err != ERR_OK);
		ADP(ptr,2);
		err = IFsh1_setWordFlash(ptr, _kd[i]);
		gerr |= (err != ERR_OK);
		ADP(ptr,2);
		err = IFsh1_setWordFlash(ptr, _ki[i]);
		gerr |= (err != ERR_OK);
		ADP(ptr,2);
		err = IFsh1_setWordFlash(ptr, _ko[i]);
		gerr |= (err != ERR_OK);
		ADP(ptr,2);
		err = IFsh1_setWordFlash(ptr, _kr[i]);
		gerr |= (err != ERR_OK);
		ADP(ptr,2);
		err = IFsh1_setWordFlash(ptr, _integral_limit[i]);
		gerr |= (err != ERR_OK);
		ADP(ptr,2);
		err = IFsh1_setWordFlash(ptr, _pid_limit[i]);
		gerr |= (err != ERR_OK);
		ADP(ptr,2);
		
		err = IFsh1_setLongFlash(ptr, _min_position[i]);
		gerr |= (err != ERR_OK);
		ADP(ptr,4);
		err = IFsh1_setLongFlash(ptr, _max_position[i]);
		gerr |= (err != ERR_OK);
		ADP(ptr,4);
	}

	if (gerr)
		AS1_printStringEx ("Error while writing to flash memory, pls try again\r\n");
	
	return ERR_OK;
}

byte readFromFlash (word addr)
{
	dword ptr = (dword)addr;
	word tmp;
	int i;

	IFsh1_getWordFlash(ptr, &tmp);
	_board_ID = BYTE_H(tmp) & 0x0f;
	ADP(ptr,2);
	//IFsh1_getWordFlash(ptr, (unsigned int *)&_version);
	ADP(ptr,2);

	for (i = 0; i < JN; i++)
	{
		IFsh1_getWordFlash(ptr, (word *)(_kp+i));
		ADP(ptr,2);
		IFsh1_getWordFlash(ptr, (word *)(_kd+i));
		ADP(ptr,2);
		IFsh1_getWordFlash(ptr, (word *)(_ki+i));
		ADP(ptr,2);
		IFsh1_getWordFlash(ptr, (word *)(_ko+i));
		ADP(ptr,2);
		IFsh1_getWordFlash(ptr, (word *)(_kr+i));
		ADP(ptr,2);
		IFsh1_getWordFlash(ptr, (word *)(_integral_limit+i));
		ADP(ptr,2);
		IFsh1_getWordFlash(ptr, (word *)(_pid_limit+i));
		ADP(ptr,2);
		
		IFsh1_getLongFlash(ptr, (dword *)(_min_position+i));
		ADP(ptr,4);
		IFsh1_getLongFlash(ptr, (dword *)(_max_position+i));
		ADP(ptr,4);
	}
	
	return ERR_OK;
}


dword BYTE_C(byte x4, byte x3, byte x2, byte x1)
{
	dword ret;
	word *p = (word *)&ret;
	*p++ = __shl(x3,8) | x4;
	*p++ = __shl(x1,8) | x2;
	return ret;
}

#define DUTYCYCLE(axis, ch, value) \
 if (axis == 0) \
	PWMC0_setDuty (ch, value); \
 else \
	PWMC1_setDuty (ch, value);
 
#define LOADDUTYCYCLE(axis) \
 if (axis == 0) \
	PWMC0_load(); \
 else \
	PWMC1_load();
 
/*
 * helper function to generate desired position.
 */
void compute_desired(byte i)
{		
 	Int32 cd;
 	Int32 _min_position_coupled = 0;
 	Int32 _max_position_coupled = 0;
 	Int32 _tmp_desired_vel;
 	float tmp;
	
	if (_control_mode[i] != MODE_IDLE)
	{
		cd = _desired[i];
		
		/* compute trajectory and control mode */
		switch (_control_mode[i])
		{
		case MODE_POSITION:
			_desired[i] += step_trajectory_vel (i);
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
			_desired[i] += step_trajectory_vel (i);
			_tmp_desired_vel = step_velocity (i);
			_desired[i] += _tmp_desired_vel;
			//AS1_printDWordAsCharsDec (_tmp_desired_vel);
			//AS1_printStringEx ("\r\n");
			break;
		}
		
		/*
		 *	Sets back the desired position to a legal one
		 *	in case joint limits have been exceeded.
		 */
#if VERSION == 0x0113
		if (i == 0 && _control_mode[i]!=MODE_CALIB_ABS_POS_SENS)
		{		
			tmp = (((float) _adjustment[0])*0.2683);
			_min_position_coupled = _min_position[i] + (Int32) tmp; 
			if (_desired[i] < _min_position_coupled && (_desired[i] - cd) < 0) 
			{
				_desired[i] = _min_position_coupled;
				if (_control_mode[i] == MODE_VELOCITY)
					_set_vel[i] = 0;
				
#ifdef DEBUG_LIMITS
				AS1_printStringEx ("L");
#endif
			}
			
			_max_position_coupled = _max_position[i] + (Int32) tmp; 
			if (_desired[i] > _max_position_coupled && (_desired[i] - cd) > 0)
			{
				_desired[i] = _max_position_coupled;
				if (_control_mode[i] == MODE_VELOCITY)
					_set_vel[i] = 0;
				
#ifdef DEBUG_LIMITS
				AS1_printStringEx ("U");
#endif
			}
#ifdef DEBUG_LIMITS
			if (_verbose && _counter == 0)
			{
				AS1_printDWordAsCharsDec (_min_position_coupled);
				AS1_printStringEx (" ");
				AS1_printDWordAsCharsDec (_max_position_coupled);
				AS1_printStringEx ("\r\n");
			}
#endif		
		}
#endif
		if (_control_mode[i]!=MODE_CALIB_ABS_POS_SENS)
		{
			if (_desired[i] < _min_position[i] && (_desired[i] - cd) < 0) 
			{
				_desired[i] = _min_position[i];
				if (_control_mode[i] == MODE_VELOCITY)
					_set_vel[i] = 0;
			}
			if (_desired[i] > _max_position[i] && (_desired[i] - cd) > 0)
			{
				_desired[i] = _max_position[i];
				if (_control_mode[i] == MODE_VELOCITY)
					_set_vel[i] = 0;
			}
		}
	}
}

/*
 * helper function to generate PWM values according to controller status.
 */
void generatePwm (byte i)
{
	if (_control_mode[i] != MODE_IDLE)
	{
		/* set PWM, _pid becomes the PWM value */
		if (_pid[i] >= 0)
		{
			DUTYCYCLE (i, 0, (unsigned char)(_pid[i] & 0x7fff));
			DUTYCYCLE (i, 2, 0);
			DUTYCYCLE (i, 4, 0);
		}
		else
		{
			DUTYCYCLE (i, 0, 0);
			DUTYCYCLE (i, 2, (unsigned char)((-_pid[i]) & 0x7fff));
			DUTYCYCLE (i, 4, 0);
		}
		
		LOADDUTYCYCLE(i);
	} /* end of !IDLE */
	else
	{
		DUTYCYCLE (i, 0, 0);
		DUTYCYCLE (i, 2, 0);
		DUTYCYCLE (i, 4, 0);

		LOADDUTYCYCLE(i);
	}
}

/*
 * prints the version number of the firmware to the serial port.
 */
void print_version(void)
{
	AS1_printStringEx ("\r\n\n");
	AS1_printStringEx ("Firmware - ver ");
#if VERSION == 0x0111
	AS1_printStringEx ("1.11");
#elif VERSION == 0x0112
	AS1_printStringEx ("1.12");
#elif VERSION == 0x0113
	AS1_printStringEx ("1.13");
#elif VERSION == 0x0114
	AS1_printStringEx ("1.14");
#elif VERSION == 0x0115
	AS1_printStringEx ("1.15");
#elif VERSION == 0x0116
	AS1_printStringEx ("1.16");
#else
#	error "No valid version specified"
#endif
	AS1_printStringEx ("\r\n");
}

/* 
 * send broadcast messages according to mask 
 * bit 1: position
 * bit 2: velocity + acceleration (not yet implemented)
 * bit 3: fault bits
 * bit 4: e.g. PWMA_PMFSA register
 * bit 5: current feedback + position error
 *
 */
void can_send_broadcast(void)
{
	int iretval; 
	bool send;
	
	if (!_broadcast_mask)
		return;
	
	if ((_broadcast_mask & 0x02) && _counter == 0)
	{
		_canmsg.CAN_messID = 0x100;
		_canmsg.CAN_messID |= (_board_ID) << 4;
		_canmsg.CAN_messID |= CAN_BCAST_POSITION;

		_canmsg.CAN_data[0] = BYTE_4(_position[0]);
		_canmsg.CAN_data[1] = BYTE_3(_position[0]);
		_canmsg.CAN_data[2] = BYTE_2(_position[0]);
		_canmsg.CAN_data[3] = BYTE_1(_position[0]);
		_canmsg.CAN_data[4] = BYTE_4(_position[1]);
		_canmsg.CAN_data[5] = BYTE_3(_position[1]);
		_canmsg.CAN_data[6] = BYTE_2(_position[1]);
		_canmsg.CAN_data[7] = BYTE_1(_position[1]);
			
		_canmsg.CAN_length = 8;
		_canmsg.CAN_frameType = DATA_FRAME;
		if (CAN1_sendFrame (1, _canmsg.CAN_messID, _canmsg.CAN_frameType, _canmsg.CAN_length, _canmsg.CAN_data) != ERR_OK)
			AS1_printStringEx("send err\r\n");
	}

	if ((_broadcast_mask & 0x04) && _counter == 1)
	{
		/* CHANGED: send PID (control) value */
		_canmsg.CAN_messID = 0x100;
		_canmsg.CAN_messID |= (_board_ID) << 4;
		_canmsg.CAN_messID |= CAN_BCAST_PID_VAL;

		_canmsg.CAN_data[0] = BYTE_H(_pid[0]);
		_canmsg.CAN_data[1] = BYTE_L(_pid[0]);
		_canmsg.CAN_data[2] = BYTE_H(_pid[1]);
		_canmsg.CAN_data[3] = BYTE_L(_pid[1]);
		
		_canmsg.CAN_length = 4;
		_canmsg.CAN_frameType = DATA_FRAME;
		if (CAN1_sendFrame (1, _canmsg.CAN_messID, _canmsg.CAN_frameType, _canmsg.CAN_length, _canmsg.CAN_data) != ERR_OK)
			AS1_printStringEx("send err\r\n");
	}
	
	if ((_broadcast_mask & 0x08) && _counter == 2)
	{
		/* if a new fault is detected then sends a message */
		_canmsg.CAN_messID = 0x100;
		_canmsg.CAN_messID |= (_board_ID) << 4;
		_canmsg.CAN_messID |= CAN_BCAST_FAULT;
		_canmsg.CAN_data[0] = 0;
		_canmsg.CAN_data[1] = 0;
		_canmsg.CAN_data[2] = 0;
		_canmsg.CAN_data[3] = 0;
		send = false;
			
		iretval = getReg (PWMA_PMFSA);
		
		if (_fault[0] == 0 && iretval != 0)
		{
			// new fault, copy iretval in _fault.
			_fault[0] = iretval;
			_canmsg.CAN_data[0] = BYTE_H(iretval);
			_canmsg.CAN_data[1] = BYTE_L(iretval);
			send = true;
		}
		else
		if (_fault[0] != 0 && iretval == 0)
		{
			// reset fault.
			_fault[0] = 0;
			_canmsg.CAN_data[0] = BYTE_H(iretval);
			_canmsg.CAN_data[1] = BYTE_L(iretval);
			send = true;
		}
		
		iretval = getReg (PWMB_PMFSA);
		
		if (_fault[1] == 0 && iretval != 0)
		{
			// new fault, copy iretval in _fault.
			_fault[1] = iretval;
			_canmsg.CAN_data[2] = BYTE_H(iretval);
			_canmsg.CAN_data[3] = BYTE_L(iretval);
			send = true;
		}
		else
		if (_fault[1] != 0 && iretval == 0)
		{
			// reset fault.
			_fault[1] = 0;
			_canmsg.CAN_data[2] = BYTE_H(iretval);
			_canmsg.CAN_data[3] = BYTE_L(iretval);
			send = true;
		}

		// if new fault, send message.		
		if (send)
		{
			_canmsg.CAN_length = 4;
			_canmsg.CAN_frameType = DATA_FRAME;
			if (CAN1_sendFrame (1, _canmsg.CAN_messID, _canmsg.CAN_frameType, _canmsg.CAN_length, _canmsg.CAN_data) != ERR_OK)
				AS1_printStringEx("send err\r\n");
		}
	}
	
	if ((_broadcast_mask & 0x10) && _counter == 3)
	{
		_canmsg.CAN_messID = 0x100;
		_canmsg.CAN_messID |= (_board_ID) << 4;
		_canmsg.CAN_messID |= CAN_BCAST_CURRENT;

		_canmsg.CAN_data[0] = BYTE_H(_current[0]);
		_canmsg.CAN_data[1] = BYTE_L(_current[0]);
		_canmsg.CAN_data[2] = BYTE_H(_current[1]);
		_canmsg.CAN_data[3] = BYTE_L(_current[1]);
		
		_canmsg.CAN_data[4] = BYTE_H(_error[0]);
		_canmsg.CAN_data[5] = BYTE_L(_error[0]);
		_canmsg.CAN_data[6] = BYTE_H(_error[1]);
		_canmsg.CAN_data[7] = BYTE_L(_error[1]);
			
		_canmsg.CAN_length = 8;
		_canmsg.CAN_frameType = DATA_FRAME;
		if (CAN1_sendFrame (1, _canmsg.CAN_messID, _canmsg.CAN_frameType, _canmsg.CAN_length, _canmsg.CAN_data) != ERR_OK)
			AS1_printStringEx("send err\r\n");		
	}
}

/* defined by the linker */
extern _data_ROM2_addr;
asm int get_flash_addr (void)
{
	move #_data_ROM2_addr, y0
	rts
}

/* used only for displaying on the terminal */
byte channel = 0;

/* 
	This is the main controller loop.
	sequences of operations:
		- reads from CAN bus or serial port 1.
		- reads encoders (or ADC).
		- computes the control value (a PID in this version).
		- checks limits and other errors.
		- sets PWM
		- does extra functions (e.g. communicate with neighboring cards).
*/
void main(void)
{
//	Int32 acceptance_code = 0x0, acceptance_code2 = 0x0;
	Int32 PWMoutput1, PWMoutput0;
	word temporary;
	
	/* gets the address of flash memory from the linker */
	_flash_addr = get_flash_addr();

	/* enable interrupts */
	setReg(SYS_CNTL, 0);
	setRegBits(IPR, 0xfe12);	/* enable all interrupts and IRQA, IRQB */
	__ENIGROUP (52, 4);
	__ENIGROUP (53, 4);
	__ENIGROUP (50, 4);
	__ENIGROUP (51, 4);
	__ENIGROUP (13, 4);
	__ENIGROUP (14, 7);
	__ENIGROUP (15, 7);
	__ENIGROUP (16, 7);
	__ENIGROUP (17, 7);
	__ENIGROUP (42, 4);
	__ENIGROUP (55, 4);
	__ENIGROUP (54, 4);
		
	AS1_init ();
	CAN1_init ();
	QD0_init ();
	QD1_init ();	
	PWMC0_init ();
	PWMC1_init ();
	TI1_init ();
	IFsh1_init ();
	
	TIC_init ();
	AD_init ();

	__EI();

	readFromFlash (_flash_addr);
	writeToFlash (_flash_addr);	

	AD_enableIntTriggerA ();
	AD_enableIntTriggerB ();
	
	print_version ();
	AS1_printStringEx ("\r\n");
	
	/* initialization */
	QD0_initPosition ();
	QD1_initPosition ();
	_calibrated[0] = false;
	_calibrated[1] = false;

	/* reads the PID parameters from flash memory */
	//readFromFlash (_flash_addr);

	/* CAN masks/filters init, note the reverse order of mask comparison (see manual) */
	set_can_masks ();
	
	/* reset trajectory generation */
	abort_trajectory (0, 0);
	abort_trajectory (1, 0);
	
	/* main control loop */
	for(_counter = 0;; _counter ++) 
	{
		if (_counter >= CAN_SYNCHRO_STEPS) _counter = 0;
		while (_wait) ;
		
		/* read commands from CAN or serial board */
		serial_interface ();
		can_interface ();
		
		/* instructions are executed for both axes and only the PWM isn't 
		   used if the specific axis is not used/calibrated
		   we're supposed to guarantee a constant loop period 
		   the serial line and other devices shouldn't be doing
		   much processing in case they receive external commands */

		/* read encoders, 32 bit values */
		_position_old[0] = _position[0];
		_position_old[1] = _position[1];
		
		get_postion(0);
		get_postion(1);
		
		/* read absolute position sensors*/
#if VERSION == 0x0112
		get_abs_postion(0);
		get_abs_postion(1);
#elif VERSION == 0x0113
		get_abs_postion(0);
#endif
		/* decouple position*/
		decouple_positions();
		
		/* this can be useful to estimate speed later on */
		_speed[0] = _position[0] - _position_old[0];
		_speed[1] = _position[1] - _position_old[1];
		
		/* in position? */
		check_in_position(0); 
		check_in_position(1);
		
		/* in reference configuration for calibration? */
		check_in_position_calib(0); 
		check_in_position_calib(1);
					
		/* computes controls */
		PWMoutput0 = compute_pwm(0);
		PWMoutput1 = compute_pwm(1);
		
		/*couple controls*/
#if VERSION == 0x0115

		PWMoutput0 = (PWMoutput0 + PWMoutput1) >> 1;
		PWMoutput1 = PWMoutput0 - PWMoutput1;	
		PWMoutput1 = -PWMoutput1;
		
		if (_control_mode[0] == MODE_IDLE || _control_mode[1] == MODE_IDLE)
		{
			PWMoutput0 = 0;
			PWMoutput1 = 0;
		}
			
		_pd[0] = (_pd[0] + _pd[1]) >> 1;
		_pd[1] = _pd[0] - _pd[1];	
		_pd[1] = -_pd[1];	
#endif
		
		/* saturates controls if necessary */
		ENFORCE_LIMITS(0,PWMoutput0);
		ENFORCE_LIMITS(1,PWMoutput1);
		
		/* set PWM duty cycle */
		generatePwm (0);
		generatePwm (1);
		
		/* Check Current value */
		check_current(0);
		check_current(1);		

		/* do extra functions, communicate, etc. */
		/* LATER */
		can_send_broadcast();
		
		/* tells that the control cycle is completed */
		_wait = true;	
		
	} /* end for(;;) */
}


/* 
 * this function reads the current _position which will be used in 
 * the PID. Measurament is given by the enocder or by an analog signal
 * depending on the the firmware version.
 */
void get_postion(byte jnt)
{
	word temporary;

#if VERSION == 0x0114
	if (jnt == 0)
		AD_getChannel16A (2, &temporary);
	else
		AD_getChannel16B (2, &temporary);
	_position[jnt] = L_deposit_l(temporary)-HALL_EFFECT_SENS_ZERO;
#else
	if (jnt == 0)
		QD0_getPosition ((dword *)_position);
	else
		QD1_getPosition ((dword *)(_position+1));
#endif

#ifdef DEBUG_TRAJECTORY
		if (_verbose && _counter == 0)
		{
			AS1_printDWordAsCharsDec (_position[0]);
			AS1_printStringEx (" ");
			AS1_printDWordAsCharsDec (_position[1]);
			AS1_printStringEx ("\r\n");
		}
#endif
}

/* 
 * this function reads the absolute position sensors.
 */

void get_abs_postion(byte jnt)
{
	word temporary;

	/* read absolute position sensors ... */
	if (jnt == 0)
		AD_getChannel16A (2, &temporary);
	else
		AD_getChannel16B (2, &temporary);
	_abs_position[jnt] = temporary >> 3;
	
	/*... and filter the measured value*/
	compute_filt_pos(jnt);
	
#ifdef DEBUG_CALIBRATION
	if (_verbose && _counter == 0)
	{	
		AS1_printWord16AsChars(_abs_position[0]);
		AS1_printStringEx (" ");
		AS1_printWord16AsChars(extract_h(_filt_abs_pos[0]));
		AS1_printStringEx ("\r\n");
		AS1_printWord16AsChars(_abs_position[1]);
		AS1_printStringEx (" ");
		AS1_printWord16AsChars(extract_h(_filt_abs_pos[1]));
		AS1_printStringEx ("\r\n");
	}
#endif
}

/* 
 * this function decouple encoder readings.
 */
void decouple_positions(void)
{
	float tmp;
	
#if VERSION == 0x0112
		/* (de)couple encoder readings */
		_position[0] = L_sub(_position[0], _position[1]);
				
		
#elif VERSION == 0x0113		
		/* beware of the first cycle when _old has no meaning */
		tmp = ((float) _position[0]) + 
				22.0/41.0*((float)_adjustment[0]) - 
				11.0/41.0*((float)_adjustment[1]);
		_position[0] = (Int32) tmp;
		//Previous version was:
		//
		//_position[0] = L_add(_position[0], _adjustment[0] >> 1);
		//_position[0] = L_sub(_position[0], _adjustment[0] / 7);
		//_position[0] = L_sub(_position[0], _adjustment[1] >> 2);  // last >>2 must be 11/41
		
				
		_adjustment[0] = L_add(_adjustment[0], _delta_adj[0]);
		_adjustment[1] = L_add(_adjustment[1], _delta_adj[1]);
				
#elif VERSION == 0x0115
		_position[0] = _position[0] - _position[1];
		_position[1] = _position[0] + 2*_position[1];	
#endif

#ifdef DEBUG_TRAJECTORY
		if (_verbose && _counter == 0)
		{
			AS1_printDWordAsCharsDec (_position[0]);
			AS1_printStringEx (" ");
			AS1_printDWordAsCharsDec (_adjustment[0]);
			AS1_printStringEx (" ");
			AS1_printDWordAsCharsDec (_adjustment[1]);
			AS1_printStringEx ("\r\n");
		}
#endif
}


/* 
 * this function checks if the current consumption has exceeded a threshold
 * for more than 200 ms using a filtered verion of the current reading.
 */
void check_current(byte jnt)
{
	word temp;
	Int32 temporary;

	if (jnt==0)
		AD_getChannel16A (1, &temp);
	else
		AD_getChannel16B (1, &temp);
	if (temp < 0x160)
		temp = 0;
	temporary = (Int32) temp;
	if (_pid[jnt] < 0)
		temporary = -temporary;
	
	_current_old[jnt] = _current[jnt];
	_current[jnt] = temporary * _conversion_factor[jnt];


#ifdef DEBUG_CURRENT
	if (_verbose && _counter == 0)
	{
		AS1_printDWordAsChars (_filt_current[1]);
		AS1_printStringEx (" ");
		AS1_printWord16AsChars (_current[1]);
		AS1_printStringEx ("\r\n");
	}
#endif
		
	compute_filtcurr(jnt);
	if (_filt_current[jnt] > _max_allowed_current[jnt])
	{
		_control_mode[jnt] = MODE_IDLE;
		if (jnt ==0)
		{
			_pad_enabled[0] = false;
			PWMC0_outputPadDisable();
			AS1_printStringEx ("Big current ch-0!");
		}
		else
		{
			_pad_enabled[1] = false;
			PWMC1_outputPadDisable();
			AS1_printStringEx ("Big current ch-1!");
		}			
	}
}


/* 
 * this function filters the current (AD value).
 */
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
	
	word current;
	word current_old;
	
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

	
	_filt_current_old[jnt] = _filt_current[jnt];
	_filt_current[jnt] = 0.9886 * _filt_current_old[jnt] + 5.7 * (_current_old[jnt] + _current[jnt]);
}


/* 
 * this function checks if the calibration is terminated
 * and if calibration is terminated resets the encoder
 */
void check_in_position_calib(byte jnt)
{
	Int32 temporary_long;
	bool temporary_cond1;
	bool temporary_cond2; 
	
	/* final consideration reached? and ... */
	temporary_long = (Int32) extract_h(_filt_abs_pos[jnt]);
	temporary_cond1 = (__abs( temporary_long - _abs_pos_calibration[jnt]) < INPOSITION_CALIB_THRESHOLD);
	temporary_cond2 = (_position[jnt] == _position_old[jnt]);
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
		if (jnt == 0)
			QD0_setPosition (0);
		else
			QD1_setPosition (0);
		_position[jnt] = 0;
		_position_old[jnt] = 0;
		_desired[jnt] = 0;
		_integral[jnt] = 0;
		//Keep the system in the current configuration
		_set_point[jnt] = _position[jnt];
		init_trajectory_vel (jnt, _position[jnt], _position[jnt], 1);
		_calibrated[jnt] = true;
	}
}

/* 
 * this function checks if the trajectory is terminated
 * and if trajectory is terminated sets the variable _in_position
 */
void check_in_position(byte jnt)
{
	if (_control_mode[jnt] == MODE_POSITION)
	{
		if (__abs(_position[jnt] - _set_point[jnt]) < INPOSITION_THRESHOLD && _ended[jnt])
			_in_position[jnt] = true;
		else
			_in_position[jnt] = false;
	}				
}

/* 
 * this function filters the output
 * of the absolute postion sensor 
 */
void compute_filt_pos(byte jnt)
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
		
	tmp = L_mult_ls(K, _abs_position[jnt]);
	tmp2 = L_mult_ls(_filt_abs_pos_old[jnt], a_1);
	tmp2 = L_add(tmp2, _abs_position_old[jnt]);
	_filt_abs_pos[jnt] = L_add(tmp, tmp2);

	_abs_position_old[jnt] = tmp;
	_filt_abs_pos_old[jnt] = _filt_abs_pos[jnt];
}

/* 
 * calibration procedure, depends on the firmware version.
 */
byte calibrate (byte channel, Int16 param)
{
#if VERSION == 0x0113
	if (_control_mode[channel] != MODE_IDLE && IS_DONE(channel))
	{
		if (channel == 0)
		{
			_control_mode[channel] = MODE_CALIB_ABS_POS_SENS;
			_abs_pos_calibration[channel] = param;
			
			_set_point[channel] = _abs_pos_calibration[channel];
			_set_vel[channel] = 1;
			init_trajectory (channel, (Int32) extract_h(_filt_abs_pos[channel]), _set_point[channel], _set_vel[channel]);	
		}
		else
		{
			_control_mode[channel] = MODE_CALIB_HARD_STOPS;
			
			_counter_calib = 0;
			_pwm_calibration[channel] = param;
		}

		AS1_printStringEx ("Calibration sequence started \r\n");
	}

#elif VERSION == 0x0112
	if (_control_mode[channel] != MODE_IDLE && IS_DONE(channel))
	{
		_control_mode[channel] = MODE_CALIB_ABS_POS_SENS;
		_abs_pos_calibration[channel] = param;
		
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
		_pwm_calibration[channel] = param;
		
		AS1_printStringEx ("Calibration sequence started \r\n");
	}
#endif


	/* need to change this return val */
	return ERR_SPEED;
}

#define BEGIN_SPECIAL_MSG_TABLE(x) \
	switch (x & 0x7F) \
	{ \
		default: \
			break;

#define END_SPECIAL_MSG_TABLE \
	}
	
/* message table macros */
#define BEGIN_MSG_TABLE(x) \
	CAN_TEMP16 = (word)extract_l(x); \
	switch (CAN_TEMP16 & 0x7f) \
	{ \
		default: \
			return ERR_OK; \
			break;

#define HANDLE_MSG(x, y) \
		case x: \
			y(CAN_DATA) ; \
			break;
	
#define END_MSG_TABLE \
	}

	
void print_can (byte data[], byte len, char c)
{
	int i;
	AS1_printString (&c, 1);
	AS1_printStringEx (": ");
	
	for (i = 0; i < len; i++)
	{
		AS1_printByteAsChars (data[i]);
		AS1_printStringEx (" ");
	}
	AS1_printStringEx ("\r\n");
}

#define SEND_BOOL(x) ((x) ? AS1_printStringEx("1") : AS1_printStringEx("0"))

void print_can_error(CAN1_TError *e)
{
	AS1_printStringEx ("f: ");
	SEND_BOOL(e->errName.BusOff);
	SEND_BOOL(e->errName.TxPassive);
	SEND_BOOL(e->errName.RxPassive);
	SEND_BOOL(e->errName.TxWarning);
	SEND_BOOL(e->errName.RxWarning);
	SEND_BOOL(e->errName.OverRun);
	AS1_printStringEx ("\r\n");
}

CAN1_TError err;

/* */
byte can_interface (void)
{
	bool done = false;
	word i;
	dword IdTx;
	
	CAN_DI;
	if (read_p != -1)
	{
		CAN_EI;
		while (!done)
		{
			canmsg_t *p;
			CAN_DI;	
			p = can_fifo + read_p;
			
			/* makes a private copy of the message */
			_canmsg.CAN_data[0] = p->CAN_data[0];
			_canmsg.CAN_data[1] = p->CAN_data[1];
			_canmsg.CAN_data[2] = p->CAN_data[2];
			_canmsg.CAN_data[3] = p->CAN_data[3];
			_canmsg.CAN_data[4] = p->CAN_data[4];
			_canmsg.CAN_data[5] = p->CAN_data[5];
			_canmsg.CAN_data[6] = p->CAN_data[6];
			_canmsg.CAN_data[7] = p->CAN_data[7];
			_canmsg.CAN_messID = p->CAN_messID;
			_canmsg.CAN_frameType = p->CAN_frameType;
			_canmsg.CAN_frameFormat = p->CAN_frameFormat;
			_canmsg.CAN_length = p->CAN_length;
			
			if (read_p == write_p)
			{
				read_p = -1;	/* empty */
				done = true;
			}
			else
			{
				read_p ++;
				if (read_p >= CAN_FIFO_LEN)
					read_p = 0;
				//done = true;
			}
			CAN_EI;
			
#ifdef DEBUG_CAN_MSG
		if (_verbose)
		{
			AS1_printStringEx ("id: ");
			AS1_printDWordAsChars (_canmsg.CAN_messID);
			AS1_printStringEx (" ");
			print_can (_canmsg.CAN_data, _canmsg.CAN_length, 'i');
			CAN1_getError (&err);
			print_can_error (&err);
		}
#endif

			if (_write_buffer == 0)
				_write_buffer = 2;
			else
				_write_buffer = 0;

#define CAN_BUFFER _write_buffer
#define CAN_DATA _canmsg.CAN_data
#define CAN_FRAME_TYPE _canmsg.CAN_frameType
#define CAN_FRAME_FMT _canmsg.CAN_frameFormat
#define CAN_LEN _canmsg.CAN_length
#define CAN_ID _canmsg.CAN_messID
#define CAN_TEMP16 i

			/* special message, not too neat */
#if VERSION == 0x0113
			if ((_canmsg.CAN_messID & 0x00000700) == 0x0100)
			{
				switch (_canmsg.CAN_messID & 0x000f)
				{
					case CAN_BCAST_POSITION:
						CAN_SET_ACTIVE_ENCODER_POSITION_HANDLER(0)
						break;
					
					default:
						break;
				}
			}
			else
#else
			if ((_canmsg.CAN_messID & 0x00000700) == 0x0100)
			{
			}
			else
#endif
			/* special message for the can loader */ 
			if ((_canmsg.CAN_messID & 0x00000700) == 0x0700)
			{
				if ((_canmsg.CAN_length == 1)) 
				{
	 				IdTx = (_canmsg.CAN_messID & 0x0700);			
					IdTx |= (L_deposit_l (_board_ID) << 4);
					IdTx |= ((_canmsg.CAN_messID >> 4) & 0x000F);
					
			    	switch (_canmsg.CAN_data[0]) 
			    	{
						case 0xFF:
							_canmsg.CAN_data[0] = 0xFF;
							_canmsg.CAN_data[1] = 0;  // board type (always 0 for motor control card).
							_canmsg.CAN_data[2] = (_version & 0xff00) >> 8;  // firmware version.	
							_canmsg.CAN_data[3] = _version & 0x00ff; 		 // firmware revision.
							CAN1_sendFrame (1, IdTx, DATA_FRAME, 4, _canmsg.CAN_data);
							break;
							
					    case 4:
		    				if (_board_ID == (_canmsg.CAN_messID & 0x000F)) 
		    				{
								_canmsg.CAN_data[0] = 4;
								_canmsg.CAN_data[1] = 1; 
								CAN1_sendFrame (1, IdTx, DATA_FRAME, 2, _canmsg.CAN_data);
							}	
							break;
							
					    case 0:
		    				if (_board_ID == (_canmsg.CAN_messID & 0x000F))
		    					asm(jsr bootStart);	/// check whether this has to be a JMP rather than a JSR.
		    				break;
		    		}	
				}
			}
			else
			{
			BEGIN_MSG_TABLE (_canmsg.CAN_data[0])
			HANDLE_MSG (CAN_NO_MESSAGE, CAN_NO_MESSAGE_HANDLER)
			HANDLE_MSG (CAN_CONTROLLER_RUN, CAN_CONTROLLER_RUN_HANDLER)
			HANDLE_MSG (CAN_CONTROLLER_IDLE, CAN_CONTROLLER_IDLE_HANDLER)
			HANDLE_MSG (CAN_TOGGLE_VERBOSE, CAN_TOGGLE_VERBOSE_HANDLER)
			HANDLE_MSG (CAN_CALIBRATE_ENCODER, CAN_CALIBRATE_ENCODER_HANDLER)
			HANDLE_MSG (CAN_ENABLE_PWM_PAD, CAN_ENABLE_PWM_PAD_HANDLER)
			HANDLE_MSG (CAN_DISABLE_PWM_PAD, CAN_DISABLE_PWM_PAD_HANDLER)
			HANDLE_MSG (CAN_GET_CONTROL_MODE, CAN_GET_CONTROL_MODE_HANDLER)
			HANDLE_MSG (CAN_MOTION_DONE, CAN_MOTION_DONE_HANDLER)
			
			HANDLE_MSG (CAN_WRITE_FLASH_MEM, CAN_WRITE_FLASH_MEM_HANDLER)
			HANDLE_MSG (CAN_READ_FLASH_MEM, CAN_READ_FLASH_MEM_HANDLER)
			
			HANDLE_MSG (CAN_GET_ENCODER_POSITION, CAN_GET_ENCODER_POSITION_HANDLER)
			HANDLE_MSG (CAN_SET_DESIRED_POSITION, CAN_SET_DESIRED_POSITION_HANDLER)
			HANDLE_MSG (CAN_GET_DESIRED_POSITION, CAN_GET_DESIRED_POSITION_HANDLER)
			HANDLE_MSG (CAN_SET_DESIRED_VELOCITY, CAN_SET_DESIRED_VELOCITY_HANDLER)
			HANDLE_MSG (CAN_GET_DESIRED_VELOCITY, CAN_GET_DESIRED_VELOCITY_HANDLER)
			HANDLE_MSG (CAN_SET_DESIRED_ACCELER, CAN_SET_DESIRED_ACCELER_HANDLER)
			HANDLE_MSG (CAN_GET_DESIRED_ACCELER, CAN_GET_DESIRED_ACCELER_HANDLER)

			HANDLE_MSG (CAN_SET_ENCODER_POSITION, CAN_SET_ENCODER_POSITION_HANDLER)
			HANDLE_MSG (CAN_GET_ENCODER_VELOCITY, CAN_GET_ENCODER_VELOCITY_HANDLER)
			HANDLE_MSG (CAN_SET_COMMAND_POSITION, CAN_SET_COMMAND_POSITION_HANDLER)
			
			HANDLE_MSG (CAN_SET_P_GAIN, CAN_SET_P_GAIN_HANDLER)
			HANDLE_MSG (CAN_GET_P_GAIN, CAN_GET_P_GAIN_HANDLER)
			HANDLE_MSG (CAN_SET_D_GAIN, CAN_SET_D_GAIN_HANDLER)
			HANDLE_MSG (CAN_GET_D_GAIN, CAN_GET_D_GAIN_HANDLER)
			HANDLE_MSG (CAN_SET_I_GAIN, CAN_SET_I_GAIN_HANDLER)
			HANDLE_MSG (CAN_GET_I_GAIN, CAN_GET_I_GAIN_HANDLER)
			HANDLE_MSG (CAN_SET_ILIM_GAIN, CAN_SET_ILIM_GAIN_HANDLER)
			HANDLE_MSG (CAN_GET_ILIM_GAIN, CAN_GET_ILIM_GAIN_HANDLER)
			HANDLE_MSG (CAN_SET_OFFSET, CAN_SET_OFFSET_HANDLER)
			HANDLE_MSG (CAN_GET_OFFSET, CAN_GET_OFFSET_HANDLER)
			HANDLE_MSG (CAN_SET_SCALE, CAN_SET_SCALE_HANDLER)
			HANDLE_MSG (CAN_GET_SCALE, CAN_GET_SCALE_HANDLER)

			HANDLE_MSG (CAN_POSITION_MOVE, CAN_POSITION_MOVE_HANDLER)
			HANDLE_MSG (CAN_VELOCITY_MOVE, CAN_VELOCITY_MOVE_HANDLER)
		
			HANDLE_MSG (CAN_GET_PID_OUTPUT, CAN_GET_PID_OUTPUT_HANDLER)
			HANDLE_MSG (CAN_GET_PID_ERROR, CAN_GET_PID_ERROR_HANDLER)
			
			HANDLE_MSG (CAN_SET_MIN_POSITION, CAN_SET_MIN_POSITION_HANDLER)
			HANDLE_MSG (CAN_GET_MIN_POSITION, CAN_GET_MIN_POSITION_HANDLER)
			HANDLE_MSG (CAN_SET_MAX_POSITION, CAN_SET_MAX_POSITION_HANDLER)
			HANDLE_MSG (CAN_GET_MAX_POSITION, CAN_GET_MAX_POSITION_HANDLER)
			HANDLE_MSG (CAN_SET_MAX_VELOCITY, CAN_SET_MAX_VELOCITY_HANDLER)
			HANDLE_MSG (CAN_GET_MAX_VELOCITY, CAN_GET_MAX_VELOCITY_HANDLER)
		
			HANDLE_MSG (CAN_SET_TLIM, CAN_SET_TLIM_HANDLER)
			HANDLE_MSG (CAN_GET_TLIM, CAN_GET_TLIM_HANDLER)
			HANDLE_MSG (CAN_SET_CURRENT_LIMIT, CAN_SET_CURRENT_LIMIT_HANDLER)
			HANDLE_MSG (CAN_SET_BCAST_POLICY, CAN_SET_BCAST_POLICY_HANDLER)
			HANDLE_MSG (CAN_GET_ERROR_STATUS, CAN_GET_ERROR_STATUS_HANDLER)

//			HANDLE_MSG (CAN_GET_ACTIVE_ENCODER_POSITION, CAN_GET_ACTIVE_ENCODER_POSITION_HANDLER)
			
			END_MSG_TABLE		
			}

#ifdef DEBUG_CAN_MSG
			if (_verbose)
			{
				AS1_printStringEx ("id: ");
				AS1_printDWordAsChars (_canmsg.CAN_messID);
				AS1_printStringEx (" ");
				print_can (_canmsg.CAN_data, _canmsg.CAN_length, 'o'); 
			}
#endif
	
		} /* end of while() */
		
	} /* end of if () */
	else
	{
		CAN_EI;
	}
			
	return ERR_OK;
}

byte c = 0;

#define ASK_PARM(msg, var1) \
	AS1_printStringEx (msg); \
	AS1_printStringEx (" ["); \
	AS1_printWord16AsChars (*var1); \
	AS1_printStringEx ("] : "); \
	AS1_getStringEx (buffer, SMALL_BUFFER_SIZE, true); \
	iretval = AS1_atoi (buffer, AS1_strlen(buffer, SMALL_BUFFER_SIZE)); \
	*var1 = iretval;

#define ASK_PARM_32(msg, var1) \
	AS1_printStringEx (msg); \
	AS1_printStringEx (" ["); \
	AS1_printDWordAsChars (*var1); \
	AS1_printStringEx ("] : "); \
	AS1_getStringEx (buffer, SMALL_BUFFER_SIZE, true); \
	iretval = AS1_atoi (buffer, AS1_strlen(buffer, SMALL_BUFFER_SIZE)); \
	*var1 = (dword)iretval;

/* test/debug serial port interface (on AS1) */
byte serial_interface (void)
{
#ifdef DEBUG_SERIAL
	byte tx, rx;
#endif
	Int32 acceptance_code;
	byte d = 0;
	char buffer[SMALL_BUFFER_SIZE];
	int  iretval = 0;
	
	if (c == 0)
		AS1_recvChar(&c);
	
	switch (c)
	{
		default:
			c = 0;
			break;
		
		case 'h':
		case 'H':
		case '\r':
			print_version ();
			AS1_printStringEx ("h, H: help\r\n");
			AS1_printStringEx ("a, set card address\r\n");
			AS1_printStringEx ("b, print card address\r\n");
			AS1_printStringEx ("w2, write control params to FLASH mem\r\n");
			AS1_printStringEx ("w3, read control params from FLASH mem\r\n");
			AS1_printStringEx ("v, toggle verbose flag\r\n");
			AS1_printStringEx ("n, print fault registers\r\n");

#ifdef DEBUG_SERIAL			
			AS1_printStringEx ("f, print CAN bus error values\r\n");
			AS1_printStringEx ("e, enable controller channel\r\n");
			AS1_printStringEx ("g, set pid gain\r\n");
			AS1_printStringEx ("s, show pid gain\r\n");
			AS1_printStringEx ("x1, start trajectory generation\r\n");
			AS1_printStringEx ("x2, stop trajectory generation\r\n");
			AS1_printStringEx ("x3, enable/disable PWM\r\n");
			AS1_printStringEx ("c, toggle channel 0/1\r\n");
#endif
			c = 0;
			break;

#ifdef DEBUG_SERIAL
		case 'f':
			CAN1_getErrorValues (&rx, &tx);
			AS1_printStringEx ("rx = ");
			AS1_printByteAsChars (rx);
			AS1_printStringEx ("\r\ntx = ");
			AS1_printByteAsChars (tx);
			AS1_printStringEx ("\r\n");
			c = 0;
			break;
	
		case 'c':
			if (channel == 0)
			{
				channel = 1;
				AS1_printStringEx ("channel = 1\r\n");
			}
			else
			{
				channel = 0;
				AS1_printStringEx ("channel = 0\r\n");
			}			
			c = 0;
			break;
			
		case 'e':
			if (channel == 0)
				AS1_printStringEx ("channel = 0\r\n");
			else
				AS1_printStringEx ("channel = 1\r\n");
				
			if (_control_mode[channel] == MODE_IDLE)
			{
				_control_mode[channel] = MODE_POSITION;
				AS1_printStringEx ("mode = position\r\n");
			}
			else
			{
				_control_mode[channel] = MODE_IDLE;
				AS1_printStringEx ("mode = idle\r\n");
			}
			c = 0;
			break;
			
		case 'x':
			if (AS1_recvChar(&d) == ERR_OK)
			{
				if (d == '1')
				{
					if (channel == 0)
						AS1_printStringEx ("channel = 0\r\n");
					else
						AS1_printStringEx ("channel = 1\r\n");
						
					AS1_printStringEx ("position: ");
					AS1_getStringEx (buffer, SMALL_BUFFER_SIZE, true);
					iretval = AS1_atoi (buffer, AS1_strlen(buffer, SMALL_BUFFER_SIZE)); 
					
					AS1_printStringEx ("move: ");
					_set_point[channel] = 0;
					_set_point[channel] = L_deposit_l(iretval);
					AS1_printDWordAsCharsDec (iretval);
					_set_vel[channel] = 10;
					_set_acc[channel] = 0;
					AS1_printStringEx (" 10\r\n");
					init_trajectory_vel (channel, _position[channel], _set_point[channel], _set_vel[channel]);
				}
				else
				if (d == '2')
				{
					if (channel == 0)
						AS1_printStringEx ("channel = 0\r\n");
					else
						AS1_printStringEx ("channel = 1\r\n");
						
					abort_trajectory (channel, _position[channel]);
					AS1_printStringEx ("trajectory aborted\r\n");
				}
				else
				if (d == '3')
				{
					if (channel == 0)
					{
						AS1_printStringEx ("channel = 0\r\n");
						if (_pad_enabled[channel] == true)
							PWMC0_outputPadDisable();
						else
							PWMC0_outputPadEnable();
					}
					else
					{
						AS1_printStringEx ("channel = 1\r\n");
						if (_pad_enabled[channel] == true)
							PWMC1_outputPadDisable();
						else
							PWMC1_outputPadEnable();
					}
						
					_pad_enabled[channel] = !_pad_enabled[channel];
				}

				c = 0;
			}
		
			break;
			
		case 'g':
			if (channel == 0)
				AS1_printStringEx ("channel = 0\r\n");
			else
				AS1_printStringEx ("channel = 1\r\n");
		
			ASK_PARM("p gain", &_kp[channel]);
			ASK_PARM("d gain", &_kd[channel]);
			ASK_PARM("scale factor", &_kr[channel]);
			ASK_PARM("offset", &_ko[channel]);
			ASK_PARM("limit", &_pid_limit[channel]);
			
			ASK_PARM("max position", &_max_position[channel]);
			_min_position[channel] = -_max_position[channel];
					
			c = 0;
			break;
			
		case 's':
			if (channel == 0)
				AS1_printStringEx ("channel = 0\r\n");
			else
				AS1_printStringEx ("channel = 1\r\n");
			AS1_printStringEx ("gain, p= ");
			AS1_printWord16AsChars (_kp[channel]);
			AS1_printStringEx (" d= ");
			AS1_printWord16AsChars (_kd[channel]);
			AS1_printStringEx (" scale= ");
			AS1_printWord16AsChars (_kr[channel]);
			AS1_printStringEx (" offset= ");
			AS1_printWord16AsChars (_ko[channel]);
			AS1_printStringEx (" limit= ");
			AS1_printWord16AsChars (_pid_limit[channel]);
			AS1_printStringEx ("\r\n");

			AS1_printStringEx ("max position= ");
			AS1_printDWordAsChars (_max_position[channel]);
			AS1_printStringEx ("\r\n");
						
			c = 0;
			break;
#endif

		case 'n':
			AS1_printStringEx ("pwm fault 0 = ");
			iretval = getReg (PWMA_PMFSA);
			AS1_printUWord16AsChars (iretval);
			
			AS1_printStringEx (" dismap 0 = ");
			iretval = getReg (PWMA_PMDISMAP1);
			AS1_printUWord16AsChars (iretval);
			
			AS1_printStringEx ("  pwm fault 1 = ");
			iretval = getReg (PWMB_PMFSA);
			AS1_printUWord16AsChars (iretval);

			AS1_printStringEx (" dismap 1 = ");
			iretval = getReg (PWMB_PMDISMAP1);
			AS1_printUWord16AsChars (iretval);
			
			AS1_printStringEx ("\r\n");
			c = 0;
			break;

		case 'v':
			_verbose = !_verbose;
			if (_verbose)
				AS1_printStringEx ("verbose is now ON\r\n");
			else
				AS1_printStringEx ("verbose is now OFF\r\n");
				
			c = 0;
			break;
			
		case 'a':
			AS1_printStringEx ("address [1-15]: ");
			AS1_getStringEx (buffer, SMALL_BUFFER_SIZE, true);
			iretval = AS1_atoi (buffer, AS1_strlen(buffer, SMALL_BUFFER_SIZE)); 
			AS1_printStringEx ("address is ");
			AS1_printWord16AsChars (iretval);
			AS1_printStringEx ("\r\n");
			
			if (iretval >= 1 && iretval <= 15)
				_board_ID = iretval & 0x0f;
			
			/* CAN masks/filters init, see main() */
			set_can_masks ();
			c = 0;
			break;

		case 'b':
			AS1_printStringEx ("address is ");
			iretval = BYTE_W(_board_ID, 0);
			AS1_printWord16AsChars (iretval);
			AS1_printStringEx ("\r\n");
			
			c = 0;
			break;
		
		case 'w':
			if (AS1_recvChar(&d) == ERR_OK)
			{
				if (d == '2')
				{
					AS1_printStringEx ("writing to FLASH mem\r\n");
					writeToFlash (_flash_addr);
					AS1_printStringEx ("done!\r\n");
				}
				else
				if (d == '3')
				{
					AS1_printStringEx ("reading from FLASH mem\r\n");
					readFromFlash (_flash_addr);
					AS1_printStringEx ("done!\r\n");
				}

				c = 0;
			}
			break;
		
	}	/* end switch/case */
}

