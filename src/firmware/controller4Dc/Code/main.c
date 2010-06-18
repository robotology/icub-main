/*
 * firmware/controller application.
 * 
 */

#include "dsp56f807.h"
#include "options.h"
#include "asc.h"
#include "can1.h"
#include "ti1.h"
#include "pid.h"
#include "filters.h"

#include "calibration.h"
#include "trajectory.h"
#include "can_interface.h"
#include "serial_interface.h"

#include "leds_interface.h"
#include "currents_interface.h"
#include "flash_interface.h"
#include "pwm_interface.h"
#include "faults_interface.h"

#include "encoders_interface.h"
#include "eeprom_interface.h"
#include "abs_analog_interface.h"
#include "abs_ssi_interface.h"
#include "strain_board.h"
	
byte	_board_ID = 15;	

char    _additional_info [32];
word    _build_number = 33;
UInt8    mainLoopOVF[2]={0,0};
int     _countBoardStatus[2] ={0,0};
UInt8   highcurrent[4]={false,false,false,false};
Int16   _flash_version=0; 
UInt8   mais_counter=0;
UInt16  mais_init_request=0;
#define  MAX_MAIS_COUNTER 15
//#define  MAIS_INIT_REQUEST 500

 
#if ((VERSION==0x0120) || (VERSION==0x0121) || (VERSION==0x0128) || (VERSION==0x0130))

byte _initMAIS[8];
UInt16 msec;
Int16 idtx=0;
Int32 cur_enc_pos[JN]={0,0,0,0};
Int32 cur_hall_pos[JN]={0,0,0,0};
Int32 prev_enc_pos[JN]={0,0,0,0};
Int32 prev_hall_pos[JN]={0,0,0,0};
byte enc_gap[JN]={500,500,500,500};
Int16 hall_gap[JN]={5,5,5,5};;
Int16 _counter_check[JN]={0,0,0,0};
Int16 enc_temp=0;
Int16 hall_temp=0;

#endif 
//********************
// Local prototypes
//********************

void decouple_positions(void);
void check_range_torque (byte, Int16, Int32 *);
void check_encoder_hall_drift(byte jnt);
/*
 * version specific global variables.
 */
#if   VERSION == 0x0111
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
#elif VERSION == 0x0117
Int16 _version = 0x0117;
#elif VERSION == 0x0118
Int16 _version = 0x0118;
#elif VERSION == 0x0119
Int16 _version = 0x0119;
#elif VERSION == 0x0120
Int16 _version = 0x0120;
#elif VERSION == 0x0121
Int16 _version = 0x0121;
#elif VERSION == 0x0128
Int16 _version = 0x0128;
#elif VERSION == 0x0129
Int16 _version = 0x0129;
#elif VERSION == 0x0130
Int16 _version = 0x0130;
#elif VERSION == 0x0125
Int16 _version = 0x0125;
#elif VERSION == 0x0131
Int16 _version = 0x0131;
#endif

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
	Int32 temp_swap = 0;

	byte i=0;
	byte j=0;
	word test=0;
	Int32 t1val=0; 
	Int16 _safeband[JN]=INIT_ARRAY (-5); //it is a value for reducing the JOINT limit of 2*_safeband [tick encoder]
	

	/* gets the address of flash memory from the linker */
	_flash_addr = get_flash_addr();
		
	/* enable interrupts */
	setReg(SYS_CNTL, 0);
	
	// IPL channels from 0 to 6 enabled
	// external interrupts IRQA and IRQB disabled
	setRegBits(IPR,0xFE00); 

	// enable FAULT
	__ENIGROUP (61, 7);
	__ENIGROUP (60, 7);
		
	// enable SCI
	__ENIGROUP (52, 3);
	__ENIGROUP (53, 3);
	__ENIGROUP (50, 3);
	__ENIGROUP (51, 3);
	
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
	// TIMER_C
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
	if (_version==_flash_version)
	{
		
	}
	else
	{
		writeToFlash(_flash_addr);
	}
	__DI();
				
	serial_interface_init (JN);
	can_interface_init    (JN);
	init_currents         ();
    init_pwm			  ();	 
    init_faults           (true,true,true);	 
    init_position_encoder ();
   	#if (VERSION==0x0129)		

	init_position_abs_ssi();

	// reset of the ABS_SSI
	// this is needed because the AS5045 gives the first value wrong !!!
    for (i=0; i<JN; i++)	_position[i]=(Int32) get_position_abs_ssi(i);
    for (i=0; i<JN; i++)    _max_real_position[i]=4095;
    for (i=0; i<JN; i++) 	_safeband[i] =-5; //5 ticks => 1 grado di AEA.
	

	#endif
	TI1_init 			  ();
	init_leds  			  ();
	EEPROM_Init();
	
	mainLoopOVF[0]=false;
	mainLoopOVF[1]=false;	
	_count=0;
	__EI();
	
	for(j=0;j<JN;j++)
	{
	_received_pid[j].rec_pid=0;
	}
	
	
#if ((VERSION==0x0120) || (VERSION==0x0121) || (VERSION==0x0128) || (VERSION==0x0130))	

	for (i=0; i<JN; i++)
	{
		_counter_check[i]=0;
		cur_enc_pos[i]=0;
		cur_hall_pos[i]=0;
		prev_enc_pos[i]=0;
		prev_hall_pos[i]=0;
	}
#endif
	
	print_version ();
	
	
	/* initialization */
	for (i=0; i<JN; i++) _calibrated[i] = false;
	
	/* reset trajectory generation */
	for (i=0; i<JN; i++) abort_trajectory (i, 0);
	
	/* initialize speed and acceleration to zero (useful later on) */
	for (i=0; i<JN; i++) _position_old[i] = 0;
	for (i=0; i<JN; i++) _speed_old[i] = 0;
	for (i=0; i<JN; i++) _accel[i] = 0;


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%		
// 											MAIN CONTROL LOOP 	                                                                   
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	for(_counter = 0;; _counter ++) 
	{
		if (_counter >= CAN_SYNCHRO_STEPS) _counter = 0;
		turn_led_off(1);
		while (_wait) ;
		turn_led_on(1);
		_count=0;
		
//******************************************************************************************/		
// 											BUS_OFF CHECK	  
//                             read commands from CAN or serial board                                          
//******************************************************************************************/

		for (i=0; i<JN; i++) 
		{
			if (getCanBusOffstatus())
			{
				turn_led_on(0);
				turn_led_on(1);
				_control_mode[i]=MODE_IDLE; 
			}
		}
//-------------------------------------------------------------------------------------------
		
		
//******************************************************************************************/		
// 											CAN INTERFACE	                                /
//******************************************************************************************/ 		
 	
		can_interface ();	
//-------------------------------------------------------------------------------------------

		
//******************************************************************************************/		
// 											LED TOGGLE                                       /
//******************************************************************************************/ 		

		test++;
		if (test==1000)
		{
			turn_led_off(0);
		}
		if (test==2000)
		{
			turn_led_on(0);
		
			test=0;
		}
//-------------------------------------------------------------------------------------------

//******************************************************************************************/		
// 											MAIS CHECK                                      /
//******************************************************************************************/ 		
#if ( (VERSION==0x0121) || (VERSION==0x0128) || (VERSION==0x0130))	

		//updates the 1000 msec counter
		msec==1000 ? msec=0 : msec++;

		if (mais_counter>=MAX_MAIS_COUNTER)
		{
			for (i=0; i<JN; i++)
			{
				_control_mode[i] = MODE_IDLE;
			}
			if (msec>=500) can_printf("MAIS is not broadcasting");	
		}
		else
		    mais_counter++; //this variable is set to zero whenever a message from the MAIS is received
		
		#ifdef DEBUG_ANA_INC
		for (i=0; i<JN; i++)
			if(msec==250*i)
		    	can_printf("[%d] enc_temp: %d", i, (Int16) get_position_encoder(i));
		#endif
		
#endif
//-------------------------------------------------------------------------------------------



	
	/* instructions are executed for both axes and only the PWM isn't 
		   used if the specific axis is not used/calibrated
		   we're supposed to guarantee a constant loop period 
		   the serial line and other devices shouldn't be doing
		   much processing in case they receive external commands */

//******************************************************************************************/		
// 										POSITIONS UPDATE                                       
// 								 update old position, 32 bit values 
//******************************************************************************************/ 		

		for (i=0; i<JN; i++) _position_old[i] = _position[i];
		for (i=0; i<JN; i++) _speed_old[i] = _speed[i];				
//-------------------------------------------------------------------------------------------

//******************************************************************************************/		
// 										READ POSITIONS                                        
// 								 update positions, 32 bit values 
//******************************************************************************************/ 		
		
#if VERSION == 0x0114		
		_position_old[0] = get_position_abs_analog(0)-HALL_EFFECT_SENS_ZERO;
		_position_old[1] = get_position_abs_analog(1)-HALL_EFFECT_SENS_ZERO;
#else
		for (i=0; i<JN; i++) _position[i] = get_position_encoder(i);
#endif
		
		/* read absolute position sensors*/
#if VERSION == 0x0112
		_filt_abs_pos[0] = compute_filt_pos(get_position_abs_analog(0)>>3,0);
		_filt_abs_pos[1] = compute_filt_pos(get_position_abs_analog(1)>>3,1);
#elif VERSION == 0x0113
		_filt_abs_pos[0] = compute_filt_pos(get_position_abs_analog(0)>>3,0);
#endif
#if VERSION == 0x0117
		_position[3]= extract_h( compute_filt_pos(get_position_abs_analog(3)>>3,3));	
#endif
#if VERSION == 0x0118
		_position[0]= extract_h( compute_filt_pos(get_position_abs_analog(0)>>3,0));	
#endif
#if VERSION == 0x0119
		_position[3]= extract_h( compute_filt_pos(get_position_abs_analog(3)>>3,3));	
#endif
#if VERSION == 0x0120
		_position[3]= extract_h( compute_filt_pos(get_position_abs_analog(0)>>3,0))+
		extract_h( compute_filt_pos(get_position_abs_analog(1)>>3,1))+
		extract_h( compute_filt_pos(get_position_abs_analog(2)>>3,2));	
#endif
#if VERSION == 0x0121
		_position[3]= _adjustment[3];			
#endif

#if VERSION == 0x0128
		_position[0]= extract_h( compute_filt_pos(get_position_abs_analog(0)>>3,0));	
		_position[1]= _adjustment[1];
		_position[2]= _adjustment[2];
		_position[3]= _adjustment[3];					
#endif

#if VERSION == 0x0129
	    _position[0]= get_position_encoder(0);	
		_position[1]= get_position_abs_ssi(0);
		_position[2]= get_position_abs_ssi(1);
		_position[3]= extract_h( compute_filt_pos(get_position_abs_analog(3)>>3,3));					
#endif


#if VERSION == 0x0130
		for (i=0; i<JN; i++) _position[i]=_adjustment[i];					
#endif
//-------------------------------------------------------------------------------------------

//******************************************************************************************/		
// 										DECOUPLE POSITIONS                                        
// 							    	 for differential joints 
//******************************************************************************************/ 		

		decouple_positions();
//-------------------------------------------------------------------------------------------
		
                          //  ALARM !!! now _position is virtual _position //	

//******************************************************************************************/		
// 							  		VELOCITY and ACCELEARATION                                         
// 							this can be useful to estimate speed later on 	     
//******************************************************************************************/ 		
	
		for (i=0; i<JN; i++) 
		{
			_speed[i] = (Int16)(_position[i] - _position_old[i]);
			_accel[i] = (_speed[i] - _speed_old[i]);	
			_in_position[i] = check_in_position(i);
		}
//-------------------------------------------------------------------------------------------

//******************************************************************************************/		
// 							  	CHECK FOR CALIBRATION                                       
// 							     in reference configuration for calibration?
//******************************************************************************************/ 		
	
		for (i=0; i<JN; i++)
		{
			if (_control_mode[i] & 0x70)    //MODE_CALIB
			check_in_position_calib(i); 
		}
//-------------------------------------------------------------------------------------------
		
//******************************************************************************************/		
// 							  		COMPUTE CONTROLS                                    
// 							     /* computes controls */
//******************************************************************************************/ 		

		for (i=0; i<JN; i++) PWMoutput[i] = compute_pwm(i);		
//-------------------------------------------------------------------------------------------

//******************************************************************************************/		
// 							  		COMPUTE CONTROLS FOR COUPLED JOINTS                                    
// 							     /* computes controls */
//******************************************************************************************/ 		
#if VERSION == 0x0115 

		if ( ! ((_control_mode[0] == MODE_CALIB_HARD_STOPS ) ||
			    (_control_mode[1] == MODE_CALIB_HARD_STOPS ) ) )
		{
			/*
			PWMoutput[0] = (PWMoutput[0] + PWMoutput[1]) >> 1;
			PWMoutput[1] = PWMoutput[0] - PWMoutput[1];	
			PWMoutput[1] = -PWMoutput[1];
			*/
			temp_swap 	 = PWMoutput[0];
			PWMoutput[0] = (PWMoutput[0] + PWMoutput[1]) >> 1;
			PWMoutput[1] = (temp_swap    - PWMoutput[1]) >> 1;	
			
				if (_control_mode[0] == MODE_IDLE || 
				_control_mode[1] == MODE_IDLE)
			{
				PWMoutput[0] = 0;
				PWMoutput[1] = 0;
			}
			/*
			_pd[0] = (_pd[0] + _pd[1]) >> 1;
			_pd[1] = _pd[0] - _pd[1];	
			_pd[1] = -_pd[1];
			*/
			temp_swap = _pd[0];
			_pd[0] = (_pd[0] 	+ _pd[1]) >> 1;
			_pd[1] = (temp_swap - _pd[1]) >> 1;		
		}		

		if (_control_mode[0] == MODE_CALIB_HARD_STOPS) PWMoutput[1] = 0;
		if (_control_mode[1] == MODE_CALIB_HARD_STOPS) PWMoutput[0] = 0;	
#endif

		/*differential controls*/
#if VERSION == 0x0119 
			//PWMoutput[1] = PWMoutput[1];	//omitted
			PWMoutput[2] = (PWMoutput[2] - PWMoutput[1]);
			if (_control_mode[1] == MODE_IDLE || _control_mode[2] == MODE_IDLE)
			{
				PWMoutput[1] = 0;
	 			PWMoutput[2] = 0;
			}
			//_pd[1] = _pd[1];				//omitted
			_pd[2] = _pd[2] - _pd[1];
			
			if ((_control_mode[1] == MODE_CALIB_HARD_STOPS) || (_control_mode[2] == MODE_CALIB_HARD_STOPS))
			{
				if 	(PWMoutput[1]> _pwm_calibration[1])
				  	PWMoutput[1] = _pwm_calibration[1];
				if 	(PWMoutput[1] < -_pwm_calibration[1])	
					PWMoutput[1] = -_pwm_calibration[1];
				if 	(PWMoutput[2]> _pwm_calibration[2])
				  	PWMoutput[2] = _pwm_calibration[2];
				if 	(PWMoutput[2] < -_pwm_calibration[2])	
					PWMoutput[2] = -_pwm_calibration[2];		
			}	
#endif 

		/*differential controls*/
#if VERSION == 0x0129 
	/*  Wrist Differential coupling 
		|Me1| |  1    -1 |  |Je1|
		|Me2|=| -1     0 |* |Je2|    */
	


			PWMoutput[1] = PWMoutput[1]-PWMoutput[2];	
			PWMoutput[2] = -PWMoutput[2];
			if (_control_mode[1] == MODE_IDLE || _control_mode[2] == MODE_IDLE)
			{
				PWMoutput[1] = 0;
	 			PWMoutput[2] = 0;
			}
			_pd[1] = _pd[1]-_pd[2];			
			_pd[2] = -_pd[2];
			
			if ((_control_mode[1] == MODE_CALIB_HARD_STOPS) || (_control_mode[2] == MODE_CALIB_HARD_STOPS))
			{
				if 	(PWMoutput[1]> _pwm_calibration[1])
				   	PWMoutput[1] = _pwm_calibration[1];
				if 	(PWMoutput[1] < -_pwm_calibration[1])	
					PWMoutput[1] = -_pwm_calibration[1];
				if 	(PWMoutput[2]> _pwm_calibration[2])
				  	PWMoutput[2] = _pwm_calibration[2];
				if 	(PWMoutput[2] < -_pwm_calibration[2])	
					PWMoutput[2] = -_pwm_calibration[2];		
			}	
#endif 
//-------------------------------------------------------------------------------------------


		/* PWM filtering */
		for (i=0; i<JN; i++) 
		{
			if (_control_mode[i] == MODE_TORQUE ||
			 	_control_mode[i] == MODE_IMPEDANCE)
				PWMoutput[i] = lpf_ord1_3hz (PWMoutput[i], i);	
		}
//******************************************* LIMIT CHECK FOR FORCE CONTORL******************/
		// Protection for joints out of the admissible range during force control
		for (i=0; i<JN; i++)  check_range_torque(i, _safeband[i], PWMoutput);


//******************************************************************************************/		
// 							  		CONTROL SATURATIONS                                  
// 							/* saturates controls if necessary */
//******************************************************************************************/ 				

#if VERSION==0x0125		
/*
		if(_strain_init[5]!=0 && _strain_init[4]!=0)
		{
			_Feq = 5*1333*(_strain[5]-_strain[4]); 
		}

		for (i=0; i<JN; i++) ENFORCE_LIMITS(i,1*_Feq);
*/
#else
		for (i=0; i<JN; i++) ENFORCE_LIMITS(i,PWMoutput[i]);
#endif

#if VERSION==0x0120
		if (_calibrated[3]==true) 
		{
			temp_swap = get_position_encoder(3);
			ENFORCE_ENC_LIMITS((_pid[3]), temp_swap, ((Int32)_max_position_enc[3]));
		}
		// check for encoder drift
		//check_encoder_hall_drift(3);
#endif	

#if VERSION==0x0121   //to limit the position of the encoder up to a certain number
		if (_calibrated[3]==true) 
		{
			temp_swap = get_position_encoder(3);
			ENFORCE_ENC_LIMITS((_pid[3]), temp_swap, ((Int32)_max_position_enc[3]));
		}
		// check for encoder drift
		check_encoder_hall_drift(3);
#endif	 	

#if VERSION==0x0128   //to limit the position of the encoder up to a certain number

		for (i=1;i<4;i++)
		{
			if (_calibrated[i]==true) 
			{
				temp_swap = get_position_encoder(i);
				ENFORCE_ENC_LIMITS((_pid[i]), temp_swap, ((Int32)_max_position_enc[i]));
			}
			// check for encoder drift
			//check_encoder_hall_drift(i);
		}
#endif	

#if VERSION==0x0130   //to limit the position of the encoder up to a certain number

		for (i=0;i<4;i++)
		{
			if (_calibrated[i]==true) 
			{
				temp_swap = get_position_encoder(i);
				ENFORCE_ENC_LIMITS((_pid[i]), temp_swap, ((Int32)_max_position_enc[i]));
			}
			// check for encoder drift
			//check_encoder_hall_drift(i);
		}
#endif	
//-------------------------------------------------------------------------------------------


//******************************************************************************************/		
// 							  		PWM generate                                   
// 									/* generate PWM */
//******************************************************************************************/ 				
		for (i=0; i<JN; i++)
		{
			if (_pad_enabled[i] == false) _control_mode[i] = MODE_IDLE;
			else
			PWM_generate(i,_pid[i]);
		}
//-------------------------------------------------------------------------------------------

//******************************************************************************************/		
// 							  		CHECK CURRENT                                  
// 									
//******************************************************************************************/ 		
	
		for (i=0; i<JN; i++) 
		{
			check_current(i, (_pid[i] > 0));
			
			/*  TEMPORARY removed the I2T current filter
			 *  which was a little bit too strong for the
			 *  pronosupination joint. 
			 */
#if VERSION != 0x0119
			compute_i2t(i);
			if (_filt_current[i] > MAX_I2T_CURRENT)
#else
			compute_filtcurr(i);
			if ((_filt_current[i] > (_max_allowed_current[i]*1000)))
#endif			
			{
				_control_mode[i] = MODE_IDLE;	
				_pad_enabled[i] = false;
				highcurrent[i]=true;
				PWM_outputPadDisable(i);
				can_printf("BIG CURR AXIS:%d ",i);
			} 			
			else
			{
				highcurrent[i]=false;
			}
		}

//******************************************************************************************/		
// 							  		CAN SEND BROADCAST                                 
// 									
//******************************************************************************************/ 		

		can_send_broadcast();
//-------------------------------------------------------------------------------------------

//******************************************************************************************/		
// 							  		Check for the MAIN LOOP duration                                 
// 									
//******************************************************************************************/ 		

	
		t1val= (UInt16) TI1_getCounter(); 	
		if (	_count>0)
		{
			
		#ifdef DEBUG_CAN_MSG
    		can_printf("MainLoop OVF");
		#endif		
		mainLoopOVF[0]=true;
		mainLoopOVF[1]=true;
		_count=0;
		}
		
		/* tells that the control cycle is completed */
		_wait = true;	
//-------------------------------------------------------------------------------------------


	} /* end for(;;) */
}/* end main()*/











	
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
		//_position[0] = _position[0] - _position[1];
		//_position[1] = _position[0] + 2*_position[1];
		//vergence / version inverted
		_position[0] = _position[0] + _position[1];
		_position[1] = _position[0] - 2*_position[1];
		
#elif VERSION == 0x0119
		//_position[1] = _position[1];		//omitted
		_position[2] = _position[1] + _position[2];		
#endif
}

/***************************************************************************/
/**
 * This function checks if the joint is in range during torque control mode
 ***************************************************************************/
void check_range_torque(byte i, Int16 band, Int32 *PWM)
{
	static UInt32 TrqLimitCount =0;
 	if (_control_mode[i] == MODE_TORQUE ||
	  	_control_mode[i] == MODE_IMPEDANCE)
 		{
	 		if  (_position[i] > _max_position[i])
	 		{
	 			if ((_position[i]-_position_old[i])>=0)
				{
					PWM[i] = 0;	
				}
				TrqLimitCount++;	 
				if (TrqLimitCount>=500)
				{
					#ifdef DEBUG_CONTROL_MODE
						can_printf("MODE TORQUE OUT LIMITS ax:%d", i);	
						TrqLimitCount=0;
				    #endif 
				}			
	 		}
	 		if  (_position[i] < _min_position[i])   
	 		{
	 			if ((_position[i]-_position_old[i])<=0)
				{
					PWM[i] = 0;			
				}				
				TrqLimitCount++;
				if (TrqLimitCount>=500)
				{
					#ifdef DEBUG_CONTROL_MODE
						can_printf("MODE TORQUE OUT LIMITS ax:%d", i);	
						TrqLimitCount=0;
				    #endif 
				}
	 		} 				
 		}
}

/***************************************************************************/
/**
/* check if the encoder is moving while the hall effect sensor is not
/**
/***************************************************************************/
#if ((VERSION==0x0121) || (VERSION==0x0128) || (VERSION==0x0130))

void check_encoder_hall_drift( byte jnt)
{

	cur_enc_pos[jnt]=get_position_encoder(jnt);
	cur_hall_pos[jnt]=_position[jnt];
	
	enc_temp=(Int16)  (cur_enc_pos[jnt]-prev_enc_pos[jnt]);
	hall_temp=(Int16) (cur_hall_pos[jnt]-prev_hall_pos[jnt]);
	if ((enc_temp)<0) enc_temp=-enc_temp;
	if ((hall_temp)<0) hall_temp=-hall_temp;
	
	if (_counter_check[jnt]==300)
	{
		if (enc_temp>enc_gap[jnt]) 
		{
#ifdef DEBUG_CAN_MSG
			can_printf("enc_temp: %d",enc_temp);
#endif
			if (hall_temp<hall_gap[jnt]) 																																																					
			{
#ifdef DEBUG_CAN_MSG
				can_printf("jnt %d check_enc_hall:",jnt);	
				can_printf("hall_temp: %d",hall_temp);
#endif
				abort_trajectory (jnt, _position[jnt]);
			}
		}
		
		// update of the variables
		
		prev_enc_pos[jnt]=cur_enc_pos[jnt];
		prev_hall_pos[jnt]=cur_hall_pos[jnt];
		
		_counter_check[jnt]=0;
	}
	else 
	{
		if (enc_temp<=enc_gap[jnt])
		{
			//reset the _counter_check
			_counter_check[jnt]=0;
		}   
		else	
			_counter_check[jnt]++;
	}
}
#endif
