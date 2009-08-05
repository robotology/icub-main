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

#include "encoders_interface.h"

#include "phase_hall_sens.h"
#include "brushless_comm.h"

#include "pwm_a.h"


byte	_board_ID = 15;	
char    _additional_info [32];
UInt8    mainLoopOVF=0;
word    _build_number = 9;
int     _countBoardStatus =0;


#ifdef TEMPERATURE_SENSOR
	//temperature sensor variables
	Int16    TempSens[] = {0,0};
	UInt8    overtemp[] = {0,0};
	UInt8    errortemp[]= {0,0};

#endif 

//**********************
// externs
//**********************
extern sDutyControlBL DutyCycle[2];
extern sDutyControlBL DutyCycleReq[2];

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
	word temporary;
	byte i=0;
	UInt16 *value=0;
	Int32 t1val=0;
	Int32 temp_swap = 0;
	Int32 PID_R= 2;
	Int32 kpp=1;
	Int16 test =0;
	byte first_step=0;
	
	//velocity and acceleration estimation
	byte j=0;

	byte headPos=0; //current pos/vel
	byte tailPos=0; //tail is (head+1)%winSize
	byte headVel=0; //current pos/vel
	byte tailVel=0; //tail is (head+1)%winSize
	byte winSizePos=35;
	byte winSizeVel=55;
	//Int32 windSizePos=35;
	//Int32 windSizeVel=50;
	Int32 positionWindow[35][JN]; //max window size: 254
	Int32 velocityWindow[55][JN]; //max window size: 254
		
#ifdef TEMPERATURE_SENSOR
	byte   TempSensCount1 = 0;
	UInt32 TempSensCount2 = 0;
	byte   temp_sens_status=0;
	overtemp[0]=0;
	overtemp[1]=0;
	errortemp[0]=0;
	errortemp[1]=0;

#endif
	
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
	__ENIGROUP (45, 7); //HallX0
	__ENIGROUP (44, 7); //HallY0
	__ENIGROUP (43, 7); //HallZ0
	__ENIGROUP (42, 7); //TI1 1ms delay main loop
	// TIMER_B
	__ENIGROUP (41, 7); //
	__ENIGROUP (40, 7); //
	__ENIGROUP (39, 7); //
	__ENIGROUP (38, 7);
	// TIMER_C
	__ENIGROUP (37, 1); 
	__ENIGROUP (36, 1);
	__ENIGROUP (35, 1);
	__ENIGROUP (34, 1);
	// TIMER_D
	__ENIGROUP (33, 7); //1ms delay duty cycle
	__ENIGROUP (32, 1);
	__ENIGROUP (31, 1);
	__ENIGROUP (30, 1);

	__EI();
	
	flash_interface_init  (JN);			
	readFromFlash (_flash_addr);  
	
	__DI();
   
	init_leds  			  ();	
	Init_Brushless_Comm	  ();
				
	serial_interface_init (JN);
	can_interface_init    (JN);
	init_currents         ();

//
    init_faults           (true,true,true);	 
    init_position_encoder ();
    init_temp_sens        ();
	TI1_init 			  ();

//variable init	
	mainLoopOVF=0;
	_countBoardStatus=0;
	_count=0;
	highcurrent[0]=0;
	highcurrent[1]=0;
	
	__EI();
	  
//	print_version ();
	
	/* initialization */
	for (i=0; i<JN; i++) _calibrated[i] = false;
	
	/* reset trajectory generation */
	for (i=0; i<JN; i++) abort_trajectory (i, 0);
	
	///////////////////////////////////////	//
	//////////////////////////////////////
	
	/* initialize speed and acceleration to zero (useful later on) */
	for (i=0; i<JN; i++) _position_old[i] = 0;
	for (i=0; i<JN; i++) _speed[i] = 0;
	for (i=0; i<JN; i++) _accel[i] = 0;
	
	/* reset the recursive windows for storage of position and velocity data */
	/* (for velocity and position estimates) */
	for(i=0;i<JN;i++)
	{
		for(j=0;j<winSizePos;j++)
		{
			positionWindow[j][i]=0;	
		}
		for(j=0;j<winSizeVel;j++)
		{
			velocityWindow[j][i]=0;	
		}	
	}
	
	
	//set_relative_position_abs_ssi(1,get_absolute_real_position_abs_ssi(1));
	/* main control loop */
	for(_counter = 0;; _counter ++) 
	{
		if (_counter >= CAN_SYNCHRO_STEPS) _counter = 0;
		led0_off
		while (_wait) ;
		_count=0;
		led0_on
//		serial_interface();
		can_interface();
	
	    //Position calculation
	    // This is used to have a shift of the zero-cross out of the 
	    // joint workspace
	    //
	    // max_real_position is the limit of the joint starting from 
	    // 4095 and going to decrease this number without zero-cross
	    // untill the joint limit is reached	


		_position_old[0]=_position[0]; /* kept for calibration check */

		_position[0]=get_position_encoder(1);
		_position_old[1]=_position[1]; /* kept for calibration check */
		_position[1]=0;
			

		// decoupling the position
		 	
		decouple_positions();

		//if (_counter == 0)
		//{	
		
		//tail= (tail+1)%winSize;
		tailPos=headPos+1; if(tailPos==winSizePos) tailPos=0;
		tailVel=headVel+1; if(tailVel==winSizeVel) tailVel=0;
		
		/* store the new position in the circular buffer */
		for (i=0; i<JN; i++) positionWindow[headPos][i]=_position[i];
				
		/* compute speed and acceleration, and store the speed in the circular buffer */		
		for (i=0; i<JN; i++)
		{
			velocityWindow[headVel][i]= (positionWindow[headPos][i] - positionWindow[tailPos][i] );/* /(windSize)*/
			_speed[i]= (Int16)(velocityWindow[headVel][i]);
			_accel[i]= (Int16)((velocityWindow[headVel][i] - velocityWindow[tailVel][i]));
		}

		
		/* advance in the circular buffer */
		//head= (head+1)%winSize;
		headPos=headPos+1; if(headPos==winSizePos) headPos=0;
		headVel=headVel+1; if(headVel==winSizeVel) headVel=0;
		
		/* memorize old position - kept for calibration check */
		for (i=0; i<JN; i++) _position_old[i] = _position[i];
		
	
		//}
		
					
		/* in position? */
		for (i=0; i<JN; i++) _in_position[i] = check_in_position(i); 
				
		/* in reference configuration for calibration? */
		//for (i=0; i<JN; i++) check_in_position_calib(i); 
	
	
//		decouple PWM	

		
	/* computes controls */
		 PWMoutput[0] = compute_pwm(0);
	
	/* saturates controls if necessary */
		for (i=0; i<JN-1; i++)
		{
			ENFORCE_LIMITS			(i,PWMoutput[i]);
			if      (_pid[i] < -MAX_DUTY) _pid[i]=-MAX_DUTY;
			else if (_pid[i] > MAX_DUTY)  _pid[i]= MAX_DUTY;
		}
	
		
		/* check temperature sensor */
#ifdef TEMPERATURE_SENSOR
		TempSensCount1++;
		TempSensCount2++;
		
		// communicate with temperature sensor every 5millisecs
		if (TempSensCount1==5)
		{
			//read temperature sensors
			temp_sens_status=MeasureTempSens();
			
			
			if (temp_sens_status==TEMPERATURE_STATUS_OK)
			{
				TempSens[0]=GetTempSens(0);			
				TempSens[1]=GetTempSens(1);	
						
				errortemp[0]=false;
				errortemp[1]=false;	
			}
			if (temp_sens_status== TEMPERATURE_STATUS_ERR)
			{
				TempSens[0]=0;			
				errortemp[0]=true;	
				TempSens[1]=0;			
				errortemp[1]=true;	
			}
			if (temp_sens_status== TEMPERATURE_STATUS_ERR1)
			{
				TempSens[0]=0;			
				errortemp[0]=true;	
			}
			if (temp_sens_status== TEMPERATURE_STATUS_ERR2)
			{			
				TempSens[1]=0;	
				errortemp[1]=true;	
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
			overtemp[i]=false;
			if ((TempSens[i] > 75) && _pad_enabled[i] == true)
			{
				_control_mode[i] = MODE_IDLE;	
				_pad_enabled[i] = false;
				
				overtemp[i]=true;
			#ifdef DEBUG_CAN_MSG
			
				can_printf("WARN: VERY HIGH temp ax%d: %d! disabling pwm...",i,TempSens[i]);	
			
			#endif				
				PWM_outputPadDisable(i);				
			}
		}	
#endif
		
				
		/* generate PWM */		
		for (i=0; i<JN; i++)
		{
			if (_pad_enabled[i] == false) _control_mode[i] = MODE_IDLE;
			else	
			
				PWM_generate(i,_pid[i]);
			
		}
	
		/* Check Current done in T1 */


	
		/* do extra functions, communicate, etc. */
		can_send_broadcast();
	
		
//	Check for the MAIN LOOP duration
 
			
		t1val= (UInt16) TI1_getCounter(); 	
		if (	_count>0)
		{	
			mainLoopOVF=1;
			_count=0;
		}
		else
		mainLoopOVF=0;
//		can_printf("V");
//		can_print_dword(t1val);	
		
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
		
#if VERSION == 0x0183		
		/* beware of the first cycle when _old has no meaning */		
		_position[0] = _position[0]+ (float) (((float) _adjustment[0])*1.7105F);  
		_position[0] = _position[0]- (float) (((float) _adjustment[1])*1.7105F);
		/*
		|M1| |  1     0    0   |  |T1|
		|T2|=|  0     1    0   |* |T2|     with a=3.8/6.5 i.e. a=1/1.7105
		|M3| | 1/a  -1/a   1   |  |T3|
		*/
		_adjustment[0] = L_add(_adjustment[0], _delta_adj[0]);
		_adjustment[1] = L_add(_adjustment[1], _delta_adj[1]);
				
#elif VERSION == 0x0185
		_position[0] = _position[0] - _position[1];
		_position[1] = _position[0] + 2*_position[1];	
		
/*#elif VERSION == 0x0182
		
	/*  Waist Differential coupling 
		|Me1| |  1     1 |  |Je1|
		|Me2|=|  1    -1 |* |Je2|    */

/*	_position[0] =_position[0] -  _position[1];
	_position[1] =_position[0] +2*_position[1];
*/
#endif
}









