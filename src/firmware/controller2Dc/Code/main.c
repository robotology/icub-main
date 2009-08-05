/*
 * firmware/controller application.
 *
 */

#include "dsp56f807.h"
#include "options.h"
#include "asc.h"
#include "ti1.h"
#include "pid.h"
#include "calibration.h"
#include "trajectory.h"
#include "can_interface.h"
#include "serial_interface.h"

#include "leds_interface.h"
#include "currents_interface.h"
#include "flash_interface.h"
#include "pwm_interface.h"
#include "faults_interface.h"
#include "temp_sens_interface.h"

#include "encoders_interface.h"
#include "eeprom_interface.h"
#include "abs_analog_interface.h"
#include "abs_ssi_interface.h"

byte	_board_ID = DEFAULT_BOARD_ID;	
char    _additional_info [32];
word    _build_number = 1;
bool    mainLoopOVF=false;
//********************
// Local prototypes 
//********************

void decouple_positions(void);
void send_debug(void);

#ifndef VERSION
#	error "No valid version specified"
#endif

/***********************************************************************
	This is the main controller loop.
	sequences of operations:
		- reads from CAN bus or serial port 1.
		- reads encoders (or ADC).
		- computes the control value (a PID in this version).
		- checks limits and other errors.
		- sets PWM
		- does extra functions (e.g. communicate with neighboring cards).
 ***********************************************************************/
void main(void)
{
	Int32 PWMoutput [JN];
	word temporary;
	byte i=0;
	byte test=0;
	Int32 t1val=0;
	/* gets the address of flash memory from the linker */
	_flash_addr = get_flash_addr();
		
	/* enable interrupts */
	setReg(SYS_CNTL, 0);
	
	// IPL channels from 0 to 6 enabled
	setRegBits(IPR, 0xfe12);	/* enable all interrupts and IRQA, IRQB */

	// enable FAULT
	__ENIGROUP (61, 4);
	__ENIGROUP (60, 4);
		
	// enable SCI
	__ENIGROUP (52, 5);
	__ENIGROUP (53, 5);
	__ENIGROUP (50, 5);
	__ENIGROUP (51, 5);
	
    // enable data flash
	__ENIGROUP (13, 4);
	
	// enable CAN	
	__ENIGROUP (14, 6);
	__ENIGROUP (15, 6);
	__ENIGROUP (16, 6);
	__ENIGROUP (17, 6);
	
	// enable ADCA/ADCB
	__ENIGROUP (55, 4);
	__ENIGROUP (54, 4);
	
	// enable timers
	// TIMER_A
	__ENIGROUP (45, 7);
	__ENIGROUP (44, 7);
	__ENIGROUP (43, 7);
	__ENIGROUP (42, 7);
	// TIMER_B
	__ENIGROUP (41, 7);
	__ENIGROUP (40, 7);
	__ENIGROUP (39, 7);
	__ENIGROUP (38, 7);
	// TIMER_B
	__ENIGROUP (37, 7);
	__ENIGROUP (36, 7);
	__ENIGROUP (35, 7);
	__ENIGROUP (34, 7);
	// TIMER_D
	__ENIGROUP (33, 7);
	__ENIGROUP (32, 7);
	__ENIGROUP (31, 7);
	__ENIGROUP (30, 7);

	__EI();
	
	flash_interface_init  (JN);			
	readFromFlash (_flash_addr);  
	
	__DI();
	init_leds  			  ();				
	serial_interface_init (JN);
	can_interface_init    (JN);
    init_pwm			  	();
    if (VERSION == 0x0112 || VERSION == 0x0113)
    	init_faults           	(false,true,true);
    else
    	init_faults           	(true,true,true);
    init_position_encoder 	();
	TI1_init 			  	();
	init_position_abs_analog();
	init_currents         	();
	EEPROM_Init();
//	EEPROM_WREN();
	
// variable initialization
	
	mainLoopOVF=false;	
	_count=0;


	__EI();
 	
	print_version ();
	
	
	/* initialization */
	for (i=0; i<JN; i++) _calibrated[i] = false;
	
	/* reset trajectory generation */
	for (i=0; i<JN; i++) abort_trajectory (i, 0);
	
	/* initialize speed and acceleration to zero (useful later on) */
	for (i=0; i<JN; i++) _position[i] = 0;
	for (i=0; i<JN; i++) _speed_old[i] = 0;
	for (i=0; i<JN; i++) _accel[i] = 0;
	
	/* main control loop */
	for(_counter = 0;; _counter ++) 
	{
		if (_counter >= CAN_SYNCHRO_STEPS) _counter = 0;
		while (_wait) ;
		_count=0;
		/* memorize old position (for next cycle) */
		for (i=0; i<JN; i++) _position_old[i] = _position[i];
		
		/* read commands from CAN or serial board */
		serial_interface ();
		can_interface ();
		
		/* instructions are executed for both axes and only the PWM isn't 
		   used if the specific axis is not used/calibrated
		   we're supposed to guarantee a constant loop period 
		   the serial line and other devices shouldn't be doing
		   much processing in case they receive external commands */
		
		
#if VERSION == 0x0114
		for (i=0; i<JN; i++) _position[i] = get_position_abs_analog(i)-HALL_EFFECT_SENS_ZERO;		
#else
		for (i=0; i<JN; i++) _position[i] = get_position_encoder(i);
#endif
		
		/* read absolute position sensors*/
#if VERSION == 0x0112
		for (i=0; i<JN; i++)
		_filt_abs_pos[i] = compute_filt_pos(get_position_abs_analog(i)>>3,i);
#elif VERSION == 0x0113
		_filt_abs_pos[0] = compute_filt_pos(get_position_abs_analog(0)>>3,0);
#endif
	
		/* decouple position*/
		decouple_positions();
		
		
		/* this can be useful to estimate speed later on */
		for (i=0; i<JN; i++) _speed[i] = (Int16)(_position[i] - _position_old[i]);
		
		/* this can be useful to estimate acceleration later on */
		for (i=0; i<JN; i++) _accel[i] = (_speed[i] - _speed_old[i]);
		
		/* memorize old position and velocity (for next cycle) */
		for (i=0; i<JN; i++) _speed_old[i] = _speed[i];
		
		/* in position? */
		for (i=0; i<JN; i++) _in_position[i] = check_in_position(i); 
				
		/* in reference configuration for calibration? */
		for (i=0; i<JN; i++) check_in_position_calib(i); 
							
		/* computes controls */
		for (i=0; i<JN; i++) PWMoutput[i] = compute_pwm(i);
		
		/*couple controls*/
#if VERSION == 0x0115

		PWMoutput[0] = (PWMoutput[0] + PWMoutput[1]) >> 1;
		PWMoutput[1] = PWMoutput[0] - PWMoutput[1];	
		PWMoutput[1] = -PWMoutput[1];
		
		if (_control_mode[0] == MODE_IDLE || _control_mode[1] == MODE_IDLE)
		{
			PWMoutput[0] = 0;
			PWMoutput[1] = 0;
		}
			
		_pd[0] = (_pd[0] + _pd[1]) >> 1;
		_pd[1] = _pd[0] - _pd[1];	
		_pd[1] = -_pd[1];	
#endif

		
		/* saturates controls if necessary */
		for (i=0; i<JN; i++) ENFORCE_LIMITS(i,PWMoutput[i]);
					
		/* set PWM duty cycle */
		for (i=0; i<JN; i++)
		{
			if (_control_mode[i] != MODE_IDLE)
				PWM_generate (i, _pid[i]); 
			else
				PWM_generate (i, 0);
		}
		
		/* Check Current */
		for (i=0; i<JN; i++) 
		{
			check_current(i, (_pid[i] > 0));		
			compute_filtcurr(i);
			if (_filt_current[i] > _max_allowed_current[i])
			{
				_control_mode[i] = MODE_IDLE;	
				_pad_enabled[i] = false;
				PWM_outputPadDisable(i);
				AS1_printStringEx ("Big cur ch-");
				AS1_printWord16AsChars (i);
				AS1_printStringEx ("\r\n");		
			}			
		}

		/* do extra debug */
		send_debug();
		
		/* do extra functions, communicate, etc. */
		can_send_broadcast();
		
		//	Check for the MAIN LOOP duration
 
			
		t1val= (UInt16) TI1_getCounter(); 	
		if (	_count>0)
		{				
		mainLoopOVF=true;
		_count=0;
		}
		else
		mainLoopOVF=false;
		/* tells that the control cycle is completed */
		_wait = true;	
		
	} /* end for(;;) */
}
	
/***************************************************************************/
/**
 * this function decouple encoder readings.
 ***************************************************************************/
void decouple_positions(void)
{
#if VERSION == 0x0112
		/* (de)couple encoder readings */
		_position[0] = L_sub(_position[0], _position[1]);
				
		
#elif VERSION == 0x0113		
		/* beware of the first cycle when _old has no meaning */		
		_position[0] = (Int32) ((float) _position[0]) + 
						22.0/41.0*((float)_adjustment[0]) - 
						11.0/41.0*((float)_adjustment[1]);
		/*
		_position[0] = L_add(_position[0], _adjustment[0] >> 1);
		_position[0] = L_sub(_position[0], _adjustment[0] / 7);
		_position[0] = L_sub(_position[0], _adjustment[1] >> 2);  // last >>2 must be 11/41
		*/
				
		_adjustment[0] = L_add(_adjustment[0], _delta_adj[0]);
		_adjustment[1] = L_add(_adjustment[1], _delta_adj[1]);
				
#elif VERSION == 0x0115
		_position[0] = _position[0] - _position[1];
		_position[1] = _position[0] + 2*_position[1];	
#endif

}


/***************************************************************************/
/**
 * this function sends debug info on the serial port.
 ***************************************************************************/
void send_debug(void)
{
	Int16 i  = 0;
	Int16 tmp16 = 0;
	Int32 tmp32 = 0;
	static Int16 step = 0;
	
#ifdef DEBUG_CALIBRATION
	if (_verbose && _counter==0)
	{		
		for (i=0; i<JN; i++) 
		{
			tmp16 = get_position_abs_analog(i)>>3;
			//tmp16 = extract_h(tmp32);
			AS1_printWord16AsChars (tmp16);
		}
		for (i=0; i<JN; i++) 
		{
			tmp32 = _filt_abs_pos[i];
			tmp16 = extract_h(tmp32);		
			AS1_printWord16AsChars (tmp16);	
		}
		AS1_printStringEx ("\r\n");
	}
#endif

#ifdef DEBUG_CURRENT
	if (_verbose && _counter==0)
	{
		for (i=0; i<JN; i++) 
		{
			tmp16 = check_current(i, (_pid[i] > 0));
			tmp32 = _filt_current[i];
			AS1_printWord16AsChars (_pid[i]);	
			//AS1_printStringEx (" ");
			//AS1_printDWordAsCharsDec (_filt_current[i]);
		}
		AS1_printStringEx ("\r\n");
	}
#endif

#ifdef DEBUG_DESIRED
	if (_verbose)
	{
		for (i=0; i<JN; i++) 
		{
			tmp32 = _desired[i];
			AS1_printDWordAsChars (tmp32);	
		}
		AS1_printStringEx ("\r\n");
	}
#endif

#ifdef DEBUG_TRAJECTORY
		if (_verbose && _counter == 0)
		{
			AS1_printWord16AsChars (_position[0]);
			AS1_printStringEx (" ");
			AS1_printWord16AsChars (_adjustment[0]);
			AS1_printStringEx (" ");
			AS1_printDWordAsCharsDec (_adjustment[1]);
			AS1_printStringEx ("\r\n");
		}
#endif

#ifdef DEBUG_IDENTIFICATION
		if (_verbose && (step == 0))
		{
			tmp16 = check_current(i, (_pid[0] > 0));
			
			AS1_printDWordAsCharsDec ((Int32)_pid[0]);
			AS1_printStringEx (" ");
			//AS1_printWord16AsChars (tmp16);
			//AS1_printStringEx (" ");
			AS1_printDWordAsCharsDec (_position[0]);
			AS1_printStringEx ("\r\n");			
			//AS1_printDWordAsCharsDec (_adjustment[0]);
			//AS1_printStringEx (" ");
			//AS1_printDWordAsCharsDec (_adjustment[1]);
			//AS1_printStringEx ("\r\n");
			step++;
		}
		else
			step = 0;
#endif

}






