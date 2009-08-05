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
#include "ad.h"
#include "can1.h" 
#include "leds_interface.h"
#include "currents_interface.h"
#include "flash_interface.h"
#include "pwm_interface.h"
#include "faults_interface.h"
#include "temp_sens_interface.h"
#include "brushess_comm.h"

#include "encoders_interface.h"

#include "pwm_a.h"
#include "pwm_b.h"


byte	_board_ID = 15;	
char    _additional_info [32];
word    _build_number = 7;
bool    mainLoopOVF=false;
int     _countBoardStatus =0;
Int16   _flash_version=0; 
UInt8   BUS_OFF=false;

#ifdef TEMPERATURE_SENSOR
	//temperature sensor variables
	Int16    TempSens[]= {0,0};
	UInt8    overtemp[]= {0,0};
	UInt8    errortemp[]={0,0};

#endif

//********************
// Local prototypes 
//********************

void decouple_positions(void);

#ifndef VERSION
#	error "No valid version specified"
#endif


#ifdef ENCODER_SHIFT_11BITS
	#define Filter_Bit(x) ((x>>1)<<1)
#else
	#define Filter_Bit(x) (x)
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
	word temp;
	word temporary;
	byte i=0;
	UInt16 *value=0;
	Int32 temp_swap = 0;
	Int32 PID_R= 2;
	Int32 kpp=1;
	Int16 test =0;
	byte first_step=0;
	Int32 t1val=0;	
	//temperature sensor variables
	Int16  TempSens[]=	 {0,0};
	byte   TempSensCount1 = 0;
	UInt32 TempSensCount2 = 0;
	
	bool  status_lin = 0;
	bool  status_inc = 0;
	bool  status_dec = 0;
	bool  status_ocf = 0;
	bool  status_cof = 0;
	
	
	/* gets the address of flash memory from the linker */
	_flash_addr = get_flash_addr();
		
	/* enable interrupts */
	setReg(SYS_CNTL, 0);
	
	// IPL channels from 0 to 6 enabled
	// external interrupts IRQA and IRQB disabled
	setRegBits(IPR, 0xFE00); 

	// enable FAULT
	__ENIGROUP (61, 3);
	__ENIGROUP (60, 3);
		
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
	__ENIGROUP (45, 7); //
	__ENIGROUP (44, 7); //
	__ENIGROUP (43, 7); //
	__ENIGROUP (42, 7); // TI1 1ms delay main loop
	// TIMER_B
	__ENIGROUP (41, 7); //
	__ENIGROUP (40, 7); //
	__ENIGROUP (39, 7); //
	__ENIGROUP (38, 7);
	// TIMER_C
	__ENIGROUP (37, 1); //
	__ENIGROUP (36, 1); //
	__ENIGROUP (35, 1); //AD triggered with the PWM 
	__ENIGROUP (34, 1);
	// TIMER_D
	__ENIGROUP (33, 7); // 1ms timer for Brushess PWM ramp 
	__ENIGROUP (32, 1);
	__ENIGROUP (31, 1);
	__ENIGROUP (30, 1);

//variable init	
	mainLoopOVF=false;
	_countBoardStatus=0;
	BUS_OFF=false;
	_count=0;

	__EI();
	
	flash_interface_init  (JN);			
	readFromFlash (_flash_addr);  
	
	__DI();
   
	init_leds  			  ();			
	serial_interface_init (JN);
	can_interface_init    (JN);

	Init_Brushess_Comm    ();	


//
    init_faults           (true,true,true);	 
   init_position_encoder ();
//  init_temp_sens        ();
    TI1_init 			  ();
	__EI();
	

//	print_version ();
	
	/* initialization */
	for (i=0; i<JN; i++) _calibrated[i] = false;
	
	/* reset trajectory generation */
	for (i=0; i<JN; i++) abort_trajectory (i, 0);
	
	
	/* initialize speed and acceleration to zero (useful later on) */
	for (i=0; i<JN; i++) _position_old[i] = 0;
	for (i=0; i<JN; i++) _speed_old[i] = 0;
	for (i=0; i<JN; i++) _accel[i] = 0;
	


	/* main control loop */
	for(_counter = 0;; _counter ++) 
	{
		if (_counter >= CAN_SYNCHRO_STEPS) _counter = 0;
		led3_on;
		while (_wait) ;
		
		test=test+1;
		_count=0;
		led3_off;
	//	serial_interface();
	// BUS_OFF check
		if (getCanBusOffstatus() )
		{
			for (i=0; i<JN; i++) _control_mode[i]=MODE_IDLE;
			led1_off
		}
		else
			led1_on
		
		//DEBUG ADC
		
//		can_printf("adc %d",_adc_debug);
		
		can_interface();
	
	    //Position calculation
	    // This is used to have a shift of the zero-cross out of the 
	    // joint workspace
	    //
	    // max_real_position is the limit of the joint starting from 
	    // 4095 and going to decrease this number without zero-cross
	    // untill the joint limit is reached
	    	    
	    for (i=0; i<JN; i++) 
		{		
			_position_old[i]=_position[i];
			_position[i]=get_position_encoder(i);
		}
		
		// decoupling the position
		 	
		decouple_positions();

		
		/* this can be useful to estimate speed later on */
		if (_counter == 0)
		{		
			/* this can be useful to estimate speed later on */
			for (i=0; i<JN; i++) _speed[i] = (Int16)(_position[i] - _position_old[i]);
			
			/* this can be useful to estimate acceleration later on */
			for (i=0; i<JN; i++) _accel[i] = (_speed[i] - _speed_old[i]);
			
			/* memorize old position and velocity (for next cycle) */
			for (i=0; i<JN; i++) _position_old[i] = _position[i];
			for (i=0; i<JN; i++) _speed_old[i] = _speed[i];
		}
			
		/* in position? */
		for (i=0; i<JN; i++) _in_position[i] = check_in_position(i); 
				
		/* in reference configuration for calibration? */
		for (i=0; i<JN; i++) check_in_position_calib(i); 
	
						
		/* computes controls */
		for (i=0; i<JN; i++) PWMoutput[i] = compute_pwm(i);
		
		/* saturates controls if necessary */
		for (i=0; i<JN; i++)
		{
			ENFORCE_LIMITS			(i,PWMoutput[i]);
		}

				
		/* generate PWM */		
		for (i=0; i<JN; i++)
		{
			if (_pad_enabled[i] == false) 
			{			
				_control_mode[i] = MODE_IDLE;
			}
			else
			{
	
					PWM_generate(i,_pid[i]);
				
			}		
			
		}

				/* check temperature sensor */
		#ifdef TEMPERATURE_SENSOR
		TempSensCount1++;
		TempSensCount2++;
		
		// communicate with temperature sensor every 5millisecs
		if (TempSensCount1==5)
		{
			if (MeasureTempSens()==TEMPERATURE_STATUS_OK)
				{
					TempSens[0]=GetTempSens(0);			
					TempSens[1]=GetTempSens(1);		
#ifdef DEBUG_TEMP_SENS			
					can_printf("temp %d %d",TempSens[0],TempSens[1]);
#endif				
				}
			else
				{
#ifdef DEBUG_TEMP_SENS
					can_printf("ERR: Temp sens not ready! \r\n");
#endif		
					TempSens[0]=0;			
					TempSens[1]=0;		
				}	
			TempSensCount1=0;
		}

		//print this warning message every 30secs		
		if (TempSensCount2==30000)
		{
			for (i=0; i<JN; i++) 
			{
				if (TempSens[i] > 35) 
				{	
#ifdef DEBUG_CAN_MSG	
					can_printf("WARN: high temp ax%d: %d",i,TempSens[i]);
#endif
				}
				if (TempSens[i] > 70)	
				{
#ifdef DEBUG_CAN_MSG
					can_printf("WARN: VERY HIGH temp ax%d: %d",i,TempSens[i]);	
#endif			
				}
			}
			TempSensCount2 = 0;		
		}
   
		//check for overtemperature
		for (i=0; i<JN; i++) 
		{
			if ((TempSens[i] > 75) && _pad_enabled[i] == true)
			{
				_control_mode[i] = MODE_IDLE;	
				_pad_enabled[i] = false;
#ifdef DEBUG_CAN_MSG
				can_printf("WARN: VERY HIGH temp ax%d: %d! disabling pwm...",i,TempSens[i]);	
#endif				
				PWM_outputPadDisable(i);				
			}
		}
		#endif
		
		
/*****************************************************************
/*
/*      Check Current is done in the Ti1 Interrupt routine  
/*
/*****************************************************************/


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

		/* do extra functions, communicate, etc. */
		can_send_broadcast();
			//	Check for the MAIN LOOP duration 

//	Check for the MAIN LOOP duration
 
			
		t1val= (UInt16) TI1_getCounter(); 	
		if (	_count>0)
		{
			
		#ifdef DEBUG_CAN_MSG
    		can_printf("MainLoop OVF");
		#endif		
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
		
#if VERSION == 0x0163		
		/* beware of the first cycle when _old has no meaning */		
		_position[0] = _position[0]+ ((float)_adjustment[0]*6.5/3.8);  
		_position[0] = _position[0]- ((float) _adjustment[1]*6.5/3.8);
		/*
		|M1| |  1     0    0   |  |T1|
		|T2|=|  0     1    0   |* |T2|     with a=3.8/6
		|M3| | 1/a  -1/a   1   |  |T3|
		*/
		_adjustment[0] = L_add(_adjustment[0], _delta_adj[0]);
		_adjustment[1] = L_add(_adjustment[1], _delta_adj[1]);
				
#elif VERSION == 0x0165
		_position[0] = _position[0] - _position[1];
		_position[1] = _position[0] + 2*_position[1];	
		
/*#elif VERSION == 0x0162
		
	/*  Waist Differential coupling 
		|Me1| |  1     1 |  |Je1|
		|Me2|=|  1    -1 |* |Je2|    */

/*	_position[0] =_position[0] -  _position[1];
	_position[1] =_position[0] +2*_position[1];
*/
#endif
}









