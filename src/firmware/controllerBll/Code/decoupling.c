#include "decoupling.h"
#include "abs_ssi_interface.h"
#include "pwm_interface.h"
#include "pid.h"
#include "can1.h" 


/***************************************************************************/
/**
 * this function decouples encoder readings.
 ***************************************************************************/
void decouple_positions(void)
{
	byte timeout_cpl_pos = 100;
#ifdef DEBUG_CAN_MSG
	static UInt8 count=0;
#endif			
	
#if   VERSION == 0x0153
	_cpl_pos_counter++;
	if (_cpl_pos_counter < timeout_cpl_pos && (get_error_abs_ssi(0)==ERR_OK))
	{
		/* beware of the first cycle when _old has no meaning */		
		_position[0] = _position[0]+ (float) (((float) _cpl_pos_prediction[0])*1.625F);  
		_position[0] = _position[0]- (float) (((float) _cpl_pos_prediction[1])*1.625F);
		/*
		|M1| |  1     0    0   |  |T1|
		|T2|=|  0     1    0   |* |T2|     with a=40/65 i.e. a=1/1.625
		|M3| | 1/a  -1/a   1   |  |T3|
		*/
		_cpl_pos_prediction[0] = L_add(_cpl_pos_prediction[0], _cpl_pos_delta[0]);
		_cpl_pos_prediction[1] = L_add(_cpl_pos_prediction[1], _cpl_pos_delta[1]);
		
		#ifdef DEBUG_CPL_BOARD
		if(count==255)
		{
			can_printf("cplPos:(%d,%d)", (Int16) _cpl_pos_prediction[0], (Int16) _cpl_pos_prediction[1]);
			count=0;
		}			
		count++;
		#endif	

	}
	else
	{
		_control_mode[0] = MODE_IDLE;	
		_pad_enabled[0] = false;
		PWM_outputPadDisable(0);
	
		#ifdef DEBUG_CAN_MSG
		if(count==255)
		{
			can_printf("No cpl pos info");
			count=0;
		}			
		count++;				
		#endif			
	}
#elif   VERSION == 0x0157 
	_cpl_pos_counter++;
	if (_cpl_pos_counter < timeout_cpl_pos  && (get_error_abs_ssi(0)==ERR_OK))
	{
		/* beware of the first cycle when _old has no meaning */		
		_position[0] = (((float) _position[0])*0.6153F);  
		_position[0] = _position[0]+ _cpl_pos_prediction[0];
		_position[0] = _position[0]- _cpl_pos_prediction[1];
		/*
		|M1| |  1     0    0   |  |T1|     pulley diameter
		|T2|=|  0     1    0   |* |T2|     with a=40/65 i.e. a=0.6153
		|M3| |  1    -1    a   |  |T3|
		*/
		_cpl_pos_prediction[0] = L_add(_cpl_pos_prediction[0], _cpl_pos_delta[0]);
		_cpl_pos_prediction[1] = L_add(_cpl_pos_prediction[1], _cpl_pos_delta[1]);
	}
	else
	{
		_control_mode[0] = MODE_IDLE;	
		_pad_enabled[0] = false;
		PWM_outputPadDisable(0);

		#ifdef DEBUG_CAN_MSG
		if(count==255)
		{
			can_printf("No cpl pos info");
			count=0;
		}			
		count++;				
		#endif			
	}
#elif VERSION == 0x0155
//		_position[0] = _position[0] - _position[1];
//		_position[1] = _position[0] + 2*_position[1];	
		
/*#elif VERSION == 0x0152
		
	/*  Waist Differential coupling 
		|Me1| |  1     1 |  |Je1|
		|Me2|=|  1    -1 |* |Je2|    */

/*	_position[0] =_position[0] -  _position[1];
	_position[1] =_position[0] +2*_position[1];
*/
#endif
}

#ifndef USE_NEW_DECOUPLING
/***************************************************************************/
/**
 * this function decouples PWM.
 ***************************************************************************/
void decouple_dutycycle(Int32 *pwm)
{
	float tempf = 0;
	Int32 temp32 = 0;
	static UInt8 count=0;
	byte timeout_cpl_pid = 100;

#if VERSION == 0x0150

	/* Version 0x0150 relizes the shoulder coupling (here '_c' denotes 
	 * the coupled board variables).The applied coupling is the following:
	 *
	 * 			[    Jm1,      0,      0]
	 * tau_m = 	[ -Jm2/a,  Jm2/a,      0] tau_j
	 * 			[ -Jm3/a,  Jm3/a,  Jm3/a]
	 *
	 * 			[    R1/K1,      0,      0]
	 * u_m = 	[        0,  R2/K2,      0] tau_m
	 * 			[        0,      0,  R3/K3]
	 *
	 *			[    R1/K1*Jm1,            0,            0]                    			[ 1         0         0     ]
	 * u_m =	[ -R2/K2*Jm2/a,  R2/K2*Jm2/a,            0] tau_j =  1.0e-03 * 0.1519 * [-1.6455    1.6455    0     ]
	 *			[ -R3/K3*Jm3/a,  R3/K3*Jm3/a,  R3/K3*Jm3/a]                    			[-1.6455    1.6455    1.6455]
	 *
	 * where:
	 * tau_m 	: torques applied by the motors (tau_m_c[0], tau_m_c[1], tau_m[0])
	 * tau_j 	: virtual torque at the joints  (tau_j_c[0], tau_j_c[1], tau_j[0])
	 * R		: motor resitance (mean of the three phases) R1=0.8967 VS R2=R3=0.8363
	 * K		: motor constant torque K1=0.0500  K2=K3=0.0280
	 * Jm		: inertia of the motors Jm1=8.47E-06 Jm2=Jm3=5.15E-06 
	 * 
	 * This solution follows from the following coupling:
	 *		[  1,  0,  0]
	 * qj =	[  1,  a,  0] qm = Tjm qm
	 *		[  0, -a,  a]
	 * where:
	 *
	 * qm 	: position of the motors (qm_c[0], qm_c[1], qm[0])
	 * qj 	: position of the motors (qj_c[0], qj_c[1], qj[0])
	 * 
	 * and from the assumption that dynamics of motors are not
	 * effected by the kinematic coupling (due to high gear boxes):
	 *
	 *	[  Jm1,    0,    0]
	 * 	[    0,  Jm2,    0] d2qm = tau_m
	 *	[    0,    0,  Jm3]
	 *
	 * where:
	 *
	 * Jm 	: motor inertia
	 * tau_m: motor torque 
	 */
	if (_control_mode[1] == MODE_POSITION)
	{
	
		#ifdef DEBUG_CPL_BOARD
		if(count==255)
		{
			can_printf("Pid:(%d,%d)", (Int16) pwm[0], (Int16) pwm[1]);
			count=0;
		}			
		count++;
		#endif	
	
		tempf = ((float)(_pd[1] - _pd[0]));
		tempf = tempf * a_coeff;
		_pd[1] = (Int32) tempf;

		tempf = ((float)(pwm[1] - pwm[0]));
		tempf = tempf * a_coeff;
	 	pwm[1] = (Int32) tempf;
				
		
		#ifdef DEBUG_CPL_BOARD
		if(count==255)
		{
			can_printf("cplPid:(%d,%d)", (Int16) pwm[0], (Int16) pwm[1]);
			count=0;
		}			
		count++;
		#endif	
	}
		
	

#elif VERSION == 0x0152
	/*  Waist Differential coupling 
		|Me1| |  1    -1 |  |Je1|
		|Me2|=|  1     1 |* |Je2|    */
	
	temp32 	     = pwm[0];
	pwm[0] = (pwm[0] - pwm[1])>>1;
	pwm[1] = (temp32         + pwm[1])>>1;	
				
	if (_control_mode[0] == MODE_IDLE || 
		_control_mode[1] == MODE_IDLE)
	{
		pwm[0] = 0;
		pwm[1] = 0;
	}
	temp32   = _pd[0];
	_pd[0] = (_pd[0] - _pd[1])>>1;
	_pd[1] = (temp32   + _pd[1])>>1;
		
#elif VERSION == 0x0153 || VERSION == 0x0157
	/* Version 0x0153 relizes the shoulder coupling (here '_c' denotes 
	 * the coupled board variables).The applied coupling is the following:
	 *
	 * 			[    Jm1,      0,      0]
	 * tau_m = 	[ -Jm2/a,  Jm2/a,      0] tau_j
	 * 			[ -Jm3/a,  Jm3/a,  Jm3/a]
	 *
	 * 			[    R1/K1,      0,      0]
	 * u_m = 	[        0,  R2/K2,      0] tau_m
	 * 			[        0,      0,  R3/K3]
	 *
	 *			[    R1/K1*Jm1,            0,            0]                    			[1          0              0]
	 * u_m =	[ -R2/K2*Jm2/a,  R2/K2*Jm2/a,            0] tau_j =  1.0e-03 * 0.1519 * [-1.6455    1.6455         0] tau_j
	 *			[ -R3/K3*Jm3/a,  R3/K3*Jm3/a,  R3/K3*Jm3/a]                    			[-1.6455    1.6455    1.6455]
	 *
	 * where:
	 * tau_m 	: torques applied by the motors (tau_m_c[0], tau_m_c[1], tau_m[0])
	 * tau_j 	: virtual torque at the joints  (tau_j_c[0], tau_j_c[1], tau_j[0])
	 * R		: motor resitance (mean of the three phases) R1=0.8967 VS R2=R3=0.8363
	 * K		: motor constant torque K1=0.0500  K2=K3=0.0280
	 * Jm		: inertia of the motors Jm1=8.47E-06 Jm2=Jm3=5.15E-06 
	 * 
	 * This solution follows from the following coupling:
	 *		[  1,  0,  0]
	 * qj =	[  1,  a,  0] qm = Tjm qm
	 *		[  0, -a,  a]
	 * where:
	 *
	 * qm 	: position of the motors (qm_c[0], qm_c[1], qm[0])
	 * qj 	: position of the motors (qj_c[0], qj_c[1], qj[0])
	 * 
	 * and from the assumption that dynamics of motors are not
	 * effected by the kinematic coupling (due to high gear boxes):
	 *
	 *	[  Jm1,    0,    0]
	 * 	[    0,  Jm2,    0] d2qm = tau_m
	 *	[    0,    0,  Jm3]
	 *
	 * where:
	 *
	 * Jm 	: motor inertia
	 * tau_m: motor torque 
	 */
	_cpl_pid_counter++;
	if (_cpl_pid_counter < timeout_cpl_pid)
	{
		if (_control_mode[0] == MODE_POSITION)
		{
	   		tempf = (float)(_pd[0]);
			tempf = tempf * a_coeff;
		    temp32 = (Int32) _cpl_pid_prediction[1] + (Int32) (tempf);
		    _pd[0] = temp32;
		    
			tempf = (float)(pwm[0]);
			tempf = tempf * a_coeff;
			temp32 = (Int32) _cpl_pid_prediction[1] + (Int32) (tempf);
		    pwm[0] = temp32;	    
			#ifdef DEBUG_CPL_BOARD
				if(count==255)
				{
					can_printf("cplPid:%d(%d,%d)", (Int16) pwm[0], (Int16) (tempf), _cpl_pid_prediction[1]);
					count=0;
				}			
				count++;
			#endif	
		    //update the prediction for coupled board duty
		    _cpl_pid_prediction[0] = _cpl_pid_prediction[0] + _cpl_pid_delta[0];
		    _cpl_pid_prediction[1] = _cpl_pid_prediction[1] + _cpl_pid_delta[1];
		}
	}
	else
	{
		_control_mode[0] = MODE_IDLE;	
		_pad_enabled[0] = false;
		PWM_outputPadDisable(0);

		#ifdef DEBUG_CAN_MSG
		if(count==255)
		{
			can_printf("No cpl pid info");
			count=0;
		}			
		count++;				
		#endif			
	}
#endif			
}

/***************************************************************************/
/**
 * this function decouples PWM (new version).
 ***************************************************************************/
/*void decouple_dutycycle_new_motor(Int32 *pwm)
{
	float tempf;
	Int32 temp32 = 0;
	static UInt8 count=0;
	byte timeout_cpl_pid = 100;

#ifdef DEBUG_CPL_BOARD
    // Here the pid value are printed BEFORE decoupling 
	if(count==255)
	{
		can_printf("Pid:(%d,%d)", (Int16) pwm[0], (Int16) pwm[1]);
		count=0;
	}			
	count++;
#endif

#if VERSION == 0x0150
	// ----- JOINT 0 -----	
	if (_control_mode[0] == MODE_POSITION)
	{
		// do nothing, alread decoupled
	}
	else if (_control_mode[0] == MODE_TORQUE ||
	    _control_mode[0] == MODE_IMPEDANCE_POS ||
	    _control_mode[0] == MODE_IMPEDANCE_VEL )
	{	
		_pd[0] = (_pd[0] + _pd[1]);
		pwm[0] = (pwm[0] + pwm[1]);	
	}	
	
	// ----- JOINT 1 -----
	if (_control_mode[1] == MODE_POSITION)
	{	
		tempf = ((float)(_pd[1] - _pd[0]));
		tempf = tempf * a_coeff;
		_pd[1] = (Int32) tempf;

		tempf = ((float)(pwm[1] - pwm[0]));
		tempf = tempf * a_coeff;
	 	pwm[1] = (Int32) tempf;				
	}
	else if (_control_mode[1] == MODE_TORQUE ||
	         _control_mode[1] == MODE_IMPEDANCE_POS ||
	         _control_mode[1] == MODE_IMPEDANCE_VEL )
	{
   		tempf = (float)(_pd[1]);
		tempf = tempf * b_coeff;
		tempf = tempf - (float)(_cpl_pid_prediction[0]);
	    temp32 = (Int32) (tempf);
	    _pd[1] = temp32;
	    
		tempf = (float)(pwm[1]);
		tempf = tempf * b_coeff;
		tempf = tempf - (float)(_cpl_pid_prediction[0]);
		temp32 = (Int32) (tempf);
	    pwm[1] = temp32;	        
	}

    //update the prediction for coupled board duty
    _cpl_pid_prediction[0] = _cpl_pid_prediction[0] + _cpl_pid_delta[0];
    _cpl_pid_prediction[1] = _cpl_pid_prediction[1] + _cpl_pid_delta[1];		

	// watchdog on coupling broadcast: if no message is received, turn off the controller	
	_cpl_pid_counter++;
	if (_cpl_pid_counter >= timeout_cpl_pid)
	{
		_control_mode[1] = MODE_IDLE;	
		_pad_enabled[1] = false;
		PWM_outputPadDisable(0);

		#ifdef DEBUG_CAN_MSG
			if(count==255)
			{
				can_printf("No cpl pid info");
				count=0;
			}			
			count++;				
		#endif			
	}
	
#elif VERSION == 0x0152
	//  Waist Differential coupling 
	//	|Me1| |  1    -1 |  |Je1|
	//	|Me2|=|  1     1 |* |Je2|    
	
	temp32 	     = pwm[0];
	pwm[0] = (pwm[0] - pwm[1])>>1;
	pwm[1] = (temp32         + pwm[1])>>1;	
				
	if (_control_mode[0] == MODE_IDLE || 
		_control_mode[1] == MODE_IDLE)
	{
		pwm[0] = 0;
		pwm[1] = 0;
	}
	temp32   = _pd[0];
	_pd[0] = (_pd[0] - _pd[1])>>1;
	_pd[1] = (temp32   + _pd[1])>>1;
		
#elif VERSION == 0x0153 || VERSION == 0x0157
	// ----- JOINT 0 ONLY -----
	if (_control_mode[0] == MODE_POSITION)
	{
   		tempf = (float)(_pd[0]);
		tempf = tempf * a_coeff;
	    temp32 = (Int32) _cpl_pid_prediction[1] + (Int32) (tempf);
	    _pd[0] = temp32;
	    
		tempf = (float)(pwm[0]);
		tempf = tempf * a_coeff;
		temp32 = (Int32) _cpl_pid_prediction[1] + (Int32) (tempf);
	    pwm[0] = temp32;	    
	}
	else if (_control_mode[0] == MODE_TORQUE ||
	 		 _control_mode[0] == MODE_IMPEDANCE_POS ||
	 		 _control_mode[0] == MODE_IMPEDANCE_VEL)
	{		
		tempf = (float)(_pd[0]);
		tempf = tempf * b_coeff;
		_pd[0] = (Int32) tempf;
		
		tempf = (float)(pwm[0]);
		tempf = tempf * b_coeff;
		pwm[0] = (Int32) tempf;					
	}

	//update the prediction for coupled board duty
	_cpl_pid_prediction[0] = _cpl_pid_prediction[0] + _cpl_pid_delta[0];
	_cpl_pid_prediction[1] = _cpl_pid_prediction[1] + _cpl_pid_delta[1];
	    
	// watchdog on coupling broadcast: if no message is received, turn off the controller	
	_cpl_pid_counter++;
	if (_cpl_pid_counter >= timeout_cpl_pid)
	{
		_control_mode[0] = MODE_IDLE;	
		_pad_enabled[0] = false;
		PWM_outputPadDisable(0);

		#ifdef DEBUG_CAN_MSG
			if(count==255)
			{
				can_printf("No cpl pid info");
				count=0;
			}			
			count++;				
		#endif			
	}
#endif

#ifdef DEBUG_CPL_BOARD
    // Here the pid value are printed AFTER decoupling 
	if(count==255)
	{
		can_printf("cplPid:(%d,%d)", (Int16) pwm[0], (Int16) pwm[1]);
		count=0;
	}			
	count++;
#endif		
}
*/
#else  //ifndef USE_NEW_DECOUPLING

/***************************************************************************/
/**
 * this function decouples PWM (new version joint version).
 ***************************************************************************/
#define a_coeff 1.6455F
#define b_coeff 1.6455F
#define t_coeff 0,6077F
 
void decouple_dutycycle_new_joint(Int32 *pwm)
{
	float tempf = 0;
	Int32 temp32 = 0;
	static UInt8 count=0;
	byte timeout_cpl_pid = 100;
	
	Int32 pd_out [2]  = {0,0};
	Int32 pwm_out [2] = {0,0};
	   
	pd_out[0]=_pd[0];
	pd_out[1]=_pd[1];
	pwm_out[0]=pwm[0];
	pwm_out[1]=pwm[1];

#if VERSION == 0x0150

	/*
		  TORQUE COUPLING MATRIX
		  
		  		  [1  1  0]
		  tau_m = [0  b -b] tau_j
		  		  [0  0  b]   
		  
		  b = 1.6455		  
	*/	
	  
	/* Version 0x0150 relizes the shoulder coupling (here '_c' denotes 
	 * the coupled board variables).The applied coupling is the following:
	 *
	 * 			[    Jm1,      0,      0]
	 * tau_m = 	[ -Jm2/a,  Jm2/a,      0] tau_j
	 * 			[ -Jm3/a,  Jm3/a,  Jm3/a]
	 *
	 * 			[    R1/K1,      0,      0]
	 * u_m = 	[        0,  R2/K2,      0] tau_m
	 * 			[        0,      0,  R3/K3]
	 *
	 *			[    R1/K1*Jm1,            0,            0]                    			[ 1         0         0     ]
	 * u_m =	[ -R2/K2*Jm2/a,  R2/K2*Jm2/a,            0] tau_j =  1.0e-03 * 0.1519 * [-1.6455    1.6455    0     ]
	 *			[ -R3/K3*Jm3/a,  R3/K3*Jm3/a,  R3/K3*Jm3/a]                    			[-1.6455    1.6455    1.6455]
	 *
	 * where:
	 * tau_m 	: torques applied by the motors (tau_m_c[0], tau_m_c[1], tau_m[0])
	 * tau_j 	: virtual torque at the joints  (tau_j_c[0], tau_j_c[1], tau_j[0])
	 * R		: motor resitance (mean of the three phases) R1=0.8967 VS R2=R3=0.8363
	 * K		: motor constant torque K1=0.0500  K2=K3=0.0280
	 * Jm		: inertia of the motors Jm1=8.47E-06 Jm2=Jm3=5.15E-06 
	 * 
	 * This solution follows from the following coupling:
	 *		[  1,  0,  0]
	 * qj =	[  1,  a,  0] qm = Tjm qm
	 *		[  0, -a,  a]
	 * where:
	 *
	 * qm 	: position of the motors (qm_c[0], qm_c[1], qm[0])
	 * qj 	: position of the motors (qj_c[0], qj_c[1], qj[0])
	 * 
	 * and from the assumption that dynamics of motors are not
	 * effected by the kinematic coupling (due to high gear boxes):
	 *
	 *	[  Jm1,    0,    0]
	 * 	[    0,  Jm2,    0] d2qm = tau_m
	 *	[    0,    0,  Jm3]
	 *
	 * where:
	 *
	 * Jm 	: motor inertia
	 * tau_m: motor torque 
	 */

	// ----- JOINT 0 -----	
	if (_control_mode[0] == MODE_POSITION)
	{
		// do nothing, alread decoupled
		//	 	    [ 1  0  0]
		//  tau_m = [-t  t  0] tau_j
		// 		    [-t  t  t]   

	}
	else if (_control_mode[0] == MODE_TORQUE ||
	    _control_mode[0] == MODE_IMPEDANCE_POS ||
	    _control_mode[0] == MODE_IMPEDANCE_VEL )
	{	
		//	 	    [ 1  1  0]
		//  tau_m = [ 0  b -b] tau_j
		// 		    [ 0  0  b]  
		
		pd_out[0]  = (_pd[0] + _pd[1]);
		pwm_out[0] = (pwm[0] + pwm[1]);	
	}	
	
	// ----- JOINT 1 -----
	if (_control_mode[1] == MODE_POSITION)
	{
		//	 	    [ 1  0  0]
		//  tau_m = [-t  t  0] tau_j
		// 		    [-t  t  t]   
	
		tempf = ((float)(- _pd[0] + _pd[1]));
		tempf = tempf * a_coeff;
		pd_out[1] = (Int32) tempf;

		tempf = ((float)(- pwm[0] + pwm[1]));
		tempf = tempf * a_coeff;
	 	pwm_out[1] = (Int32) tempf;				
	}
	else if (_control_mode[1] == MODE_TORQUE ||
	         _control_mode[1] == MODE_IMPEDANCE_POS ||
	         _control_mode[1] == MODE_IMPEDANCE_VEL )
	{
		//	 	    [ 1  1  0]
		//  tau_m = [ 0  b -b] tau_j
		// 		    [ 0  0  b]   
		  		  
   		tempf = (float)(_pd[1]) - (float)(_cpl_pid_prediction[0]);
		tempf = tempf * b_coeff;
	    temp32 = (Int32) (tempf);
	    pd_out[1] = temp32;
	    
		tempf = (float)(pwm[1]) - (float)(_cpl_pid_prediction[0]);
		tempf = tempf * b_coeff;
		temp32 = (Int32) (tempf);
	    pwm_out[1] = temp32;	        
	}

    //update the prediction for coupled board duty
    _cpl_pid_prediction[0] = _cpl_pid_prediction[0] + _cpl_pid_delta[0];
    _cpl_pid_prediction[1] = _cpl_pid_prediction[1] + _cpl_pid_delta[1];		

	// watchdog on coupling broadcast: if no message is received, turn off the controller	
	_cpl_pid_counter++;
	if (_cpl_pid_counter >= timeout_cpl_pid)
	{
		_control_mode[0] = MODE_IDLE;	
		_pad_enabled[0] = false;
		PWM_outputPadDisable(0);
		_control_mode[1] = MODE_IDLE;	
		_pad_enabled[1] = false;
		PWM_outputPadDisable(1);

		#ifdef DEBUG_CAN_MSG
			if(count==255)
			{
				can_printf("No cpl pid info");
				count=0;
			}			
			count++;				
		#endif			
	}
	
#elif VERSION == 0x0152
	/*  Waist Differential coupling 
		|Me1| |  1    -1 |  |Je1|
		|Me2|=|  1     1 |* |Je2|    */
	
	pwm_out[0] = (pwm[0] - pwm[1])>>1;
	pwm_out[1] = (pwm[0] + pwm[1])>>1;	

	pd_out[0] = (_pd[0] - _pd[1])>>1;
	pd_out[1] = (_pd[0] + _pd[1])>>1;
					
	if (_control_mode[0] == MODE_IDLE || 
		_control_mode[1] == MODE_IDLE)
	{
		pwm_out[0] = 0;
		pwm_out[1] = 0;
	}

		
#elif VERSION == 0x0153 || VERSION == 0x0157
	/*
		  TORQUE COUPLING MATRIX
		  
		  		  [1  1  0]
		  tau_m = [0  b -b] tau_j
		  		  [0  0  b]   	  
	*/	  
	
	/* Version 0x0153 relizes the shoulder coupling (here '_c' denotes 
	 * the coupled board variables).The applied coupling is the following:
	 *
	 * 			[    Jm1,      0,      0]
	 * tau_m = 	[ -Jm2/a,  Jm2/a,      0] tau_j
	 * 			[ -Jm3/a,  Jm3/a,  Jm3/a]
	 *
	 * 			[    R1/K1,      0,      0]
	 * u_m = 	[        0,  R2/K2,      0] tau_m
	 * 			[        0,      0,  R3/K3]
	 *
	 *			[    R1/K1*Jm1,            0,            0]                    			[1          0              0]
	 * u_m =	[ -R2/K2*Jm2/a,  R2/K2*Jm2/a,            0] tau_j =  1.0e-03 * 0.1519 * [-1.6455    1.6455         0] tau_j
	 *			[ -R3/K3*Jm3/a,  R3/K3*Jm3/a,  R3/K3*Jm3/a]                    			[-1.6455    1.6455    1.6455]
	 *
	 * where:
	 * tau_m 	: torques applied by the motors (tau_m_c[0], tau_m_c[1], tau_m[0])
	 * tau_j 	: virtual torque at the joints  (tau_j_c[0], tau_j_c[1], tau_j[0])
	 * R		: motor resitance (mean of the three phases) R1=0.8967 VS R2=R3=0.8363
	 * K		: motor constant torque K1=0.0500  K2=K3=0.0280
	 * Jm		: inertia of the motors Jm1=8.47E-06 Jm2=Jm3=5.15E-06 
	 * 
	 * This solution follows from the following coupling:
	 *		[  1,  0,  0]
	 * qj =	[  1,  a,  0] qm = Tjm qm
	 *		[  0, -a,  a]
	 * where:
	 *
	 * qm 	: position of the motors (qm_c[0], qm_c[1], qm[0])
	 * qj 	: position of the motors (qj_c[0], qj_c[1], qj[0])
	 * 
	 * and from the assumption that dynamics of motors are not
	 * effected by the kinematic coupling (due to high gear boxes):
	 *
	 *	[  Jm1,    0,    0]
	 * 	[    0,  Jm2,    0] d2qm = tau_m
	 *	[    0,    0,  Jm3]
	 *
	 * where:
	 *
	 * Jm 	: motor inertia
	 * tau_m: motor torque 
	 */

	// ----- JOINT 0 ONLY -----
	if (_control_mode[0] == MODE_POSITION)
	{
		//	 	    [ 1  0  0]
		//  tau_m = [-t  t  0] tau_j
		// 		    [-t  t  t]   
		
   	    tempf = (float) (-_cpl_pid_prediction[0]) + (float) (_cpl_pid_prediction[1]) + (float)(_pd[0]);
		tempf = tempf * a_coeff;
	    temp32 = (Int32) (tempf);
	    pd_out[0] = temp32;
	    
		tempf = (float) (-_cpl_pid_prediction[0]) + (float) (_cpl_pid_prediction[1]) + (float)(pwm[0]);
		tempf = tempf * a_coeff;
	    temp32 = (Int32) (tempf);
	    pwm_out[0] = temp32;	    
	}
	else if (_control_mode[0] == MODE_TORQUE ||
	 		 _control_mode[0] == MODE_IMPEDANCE_POS ||
	 		 _control_mode[0] == MODE_IMPEDANCE_VEL)
	{
		//	 	    [ 1  1  0]
		//  tau_m = [ 0  s -s] tau_j
		// 		    [ 0  0  s]  
		  		
		tempf = (float)(_pd[0]);
		tempf = tempf * b_coeff;
		pd_out[0] = (Int32) tempf;
		
		tempf = (float)(pwm[0]);
		tempf = tempf * b_coeff;
		pwm_out[0] = (Int32) tempf;					
	}

	//update the prediction for coupled board duty
	_cpl_pid_prediction[0] = _cpl_pid_prediction[0] + _cpl_pid_delta[0];
	_cpl_pid_prediction[1] = _cpl_pid_prediction[1] + _cpl_pid_delta[1];
	    
	// watchdog on coupling broadcast: if no message is received, turn off the controller	
	_cpl_pid_counter++;
	if (_cpl_pid_counter >= timeout_cpl_pid)
	{
		_control_mode[0] = MODE_IDLE;	
		_pad_enabled[0] = false;
		PWM_outputPadDisable(0);

		#ifdef DEBUG_CAN_MSG
			if(count==255)
			{
				can_printf("No cpl pid info");
				count=0;
			}			
			count++;				
		#endif			
	}
#endif

#ifdef DEBUG_CPL_BOARD
	if(count==255)
	{
		// before decoupling
		can_printf("Pid:(%d,%d)", (Int16) pwm[0], (Int16) pwm[1]);
		// after decoupling
		can_printf("cplPid:(%d,%d)", (Int16) pwm_out[0], (Int16) pwm_out[1]);
		count=0;
	}			
	count++;
#endif	

	//*** WRITE THE OUTPUT ***
	pwm[0]=pwm_out[0];
	pwm[1]=pwm_out[1];
	_pd[0]=pd_out[0];
	_pd[1]=pd_out[1];
		
}

#endif //USE_NEW_DECOUPLING
