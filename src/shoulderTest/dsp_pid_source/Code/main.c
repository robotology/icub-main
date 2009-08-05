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

//#include "leds_interface.h"
#include "currents_interface.h"
#include "flash_interface.h"
#include "pwm_interface.h"
//#include "faults_interface.h"
#include "temp_sens_interface.h"

//#include "encoders_interface.h"
//#include "abs_analog_interface.h"
#include "abs_ssi_interface.h"

#include "pwm_a.h"
#include "pwm_b.h"


byte	_board_ID = 12;//DEFAULT_BOARD_ID;	

//********************
// Local prototypes 
//********************

void decouple_positions(void);

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
	Int32 margin_position[JN];
	Int32 speed_factor[JN];
	Int32 PID_R= 2;
	Int32 kpp=1;
	
	margin_position[0]=300;
	margin_position[1]=300;
	margin_position[2]=300;
	margin_position[3]=300;
	speed_factor[0]=2;
	speed_factor[1]=2;
	speed_factor[2]=2;
	speed_factor[3]=2;
	
	/* gets the address of flash memory from the linker */
	_flash_addr = get_flash_addr();
		
	/* enable interrupts */
	setReg(SYS_CNTL, 0);
	
	// IPL channels from 0 to 6 enabled
	// external interrupts IRQA and IRQB disabled
	setRegBits(IPR, 0b1111111000000000); 

	// enable FAULT
	__ENIGROUP (61, 4);
	__ENIGROUP (60, 4);
		
	// enable SCI
	__ENIGROUP (52, 4);
	__ENIGROUP (53, 4);
	__ENIGROUP (50, 4);
	__ENIGROUP (51, 4);
	
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
	readFromFlash (_flash_addr, &_board_ID);  
	
	__DI();
	
//	init_leds  			  ();				
	serial_interface_init (JN);
	can_interface_init    (JN);
//	init_currents         (JN);
    PWM_A_init ();
	PWM_B_init ();
	
	PWM_A_outputPadEnable(0x3F00);
	PWM_B_outputPadEnable(0x3F00);
	
	PWM_A_setDuty(0,666);
	PWM_A_setDuty(2,666);
	PWM_A_setDuty(4,666);
	PWM_B_setDuty(0,666);
	PWM_B_setDuty(2,1333);
	
	PWM_A_load();
	PWM_B_load();	
	
    init_position_abs_ssi ();
 //   init_faults           (true,true);	 
 //   init_position_encoder ();
 //   init_temp_sens        ();
	TI1_init 			  ();

	__EI();
	
	print_version ();
	
	/* initialization */
	for (i=0; i<JN; i++) _calibrated[i] = false;
	
	/* reset trajectory generation */
	for (i=0; i<JN; i++) abort_trajectory (i, 0);
	
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
		//for (i=0; i<JN; i++) _position_old[i] = _position[i];
		
		for (i=0; i<JN; i++) _position[i] = get_position_abs_ssi(i);


#ifdef DEBUG_TRAJECTORY
		if (_verbose && _counter == 0)
		{
			for (i=0; i<JN; i++)
			{
				AS1_printDWordAsCharsDec (_position[i]);
				AS1_printStringEx (" ");	
			}
			AS1_printStringEx ("\r\n");
		}
#endif
	
		/* decouple position*/
		//decouple_positions();
		
		/* this can be useful to estimate speed later on */
		for (i=0; i<JN; i++) 
		{
			if (_counter == 0)
			{
				_speed[i] = _position[i] - _position_old[i];
				_position_old[i] = _position[i];
			}
		}
		
		/* in position? */
		for (i=0; i<JN; i++) _in_position[i] = check_in_position(i); 
				
		/* in reference configuration for calibration? */
		//for (i=0; i<JN; i++) check_in_position_calib(i); 
							
		/* computes controls */
		for (i=0; i<JN; i++) 
		{
			PWMoutput[i] = compute_pwm(i);
		}
		
		/* saturates controls if necessary */
		for (i=0; i<JN; i++)
		{
			ENFORCE_LIMITS			(i,PWMoutput[i]);
			CHECK_ABNORMAL_CURRENT	(i,_pid[i]);
		}
		

			
		for (i=0; i<JN; i++) 
		{	
			if      (_pid[i] < -666) _pid[i]=-666;
			else if (_pid[i] > 666)  _pid[i]= 666;
				
			_pid[i]+=666;		
		}			
		
		for (i=0; i<JN; i++)
		{
			//margin_position[i]=_speed[i]*speed_factor[i];
			margin_position[i]=0;
			//margin_position[i]=100*_speed[i];
			if (margin_position[i]<0)margin_position[i]=-margin_position[i];
		}
		
		for (i=0; i<JN; i++)
		{
			if (_position[i]<100 && _position_old[i] > 4000) 
			{
			//	PWM_outputPadDisable(i);
				_pid[i]=666;	
				abort_trajectory (i, 0);
				_control_mode[i] = MODE_IDLE; 
			}
			if (_position_old[i] <100 && _position[i] > 4000)
			{
			//	PWM_outputPadDisable(i);
				_pid[i]=666;	
				abort_trajectory (i, 0);
				_control_mode[i] = MODE_IDLE; 
			}	
			/*
			if (_position[i]>_max_position[i]-margin_position[i] &&
				_set_point[i]>_max_position[i]-margin_position[i])
			{	
				
	//			PID_R=(_position[i]-(_max_position[i]-margin_position[i]))*kpp;		
				
				_pid[i]>666?_pid[i]=666-PID_R:666+PID_R;
				abort_trajectory (i, 0);
				
	//			_set_point[i]= _max_position[i]-margin_position[i];
			}

			else if (_position[i]<_min_position[i]+margin_position[i] &&
					_set_point[i]<_min_position[i]+margin_position[i])
			{
	
	//			PID_R=(_position[i]-(_min_position[i]+margin_position[i]))*-kpp;
					
				_pid[i]>666?_pid[i]=666-PID_R:666+PID_R;
				abort_trajectory (i, 0); 
	
	//			_set_point[i]= _min_position[i]+margin_position[i];
			}
			*/
			if (_control_mode[i] == MODE_MARGIN_REACHED)
			{
				_set_point[i]=_desired[i]=_position[i];	
			}
			
			if (_position[i]>_max_position[i]-margin_position[i] &&
				_set_point[i]>_max_position[i]-margin_position[i] &&
				_control_mode[i] != MODE_MARGIN_REACHED  )
			{	
				_pid[i]=666;
				abort_trajectory (i, 0);
				_control_mode[i] = MODE_MARGIN_REACHED ; 
				
			}

			else if (_position[i]<_min_position[i]+margin_position[i] &&
					_set_point[i]<_min_position[i]+margin_position[i] &&
					_control_mode[i] != MODE_MARGIN_REACHED )
			{
				_pid[i]=666;
				abort_trajectory (i, 0); 
				_control_mode[i] = MODE_MARGIN_REACHED ; 	
				
			}
			
		}
							
		/* set PWM duty cycle */
		PWM_A_setDuty(0,_pid[0]);
		PWM_A_setDuty(2,_pid[1]);
		PWM_A_setDuty(4,_pid[2]);
		PWM_B_setDuty(0,_pid[3]);
		PWM_B_setDuty(2,1333);
		
		PWM_A_load();
		PWM_B_load();
			
//		PWM_generate (0, _control_mode[0], _pid[0]); 
//		PWM_generate (1, _control_mode[1], _pid[1]);
//		PWM_generate (2, _control_mode[2], _pid[2]);
//		PWM_generate (3, _control_mode[3], _pid[3]);
	
		/* do extra functions, communicate, etc. */
		can_send_broadcast();
		
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
		_position[0] = L_add(_position[0], _adjustment[0] >> 1);
		_position[0] = L_sub(_position[0], _adjustment[0] / 7);
		_position[0] = L_sub(_position[0], _adjustment[1] >> 2);  // last >>2 must be 11/41
				
		_adjustment[0] = L_add(_adjustment[0], _delta_adj[0]);
		_adjustment[1] = L_add(_adjustment[1], _delta_adj[1]);
				
#elif VERSION == 0x0115
		_position[0] = _position[0] - _position[1];
		_position[1] = _position[0] + 2*_position[1];	
#endif
}









