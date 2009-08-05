
#include "brushless_comm.h"
#include "phase_hall_sens.h"
#include "pwm_interface.h"
#include "pwm_a.h"
#include "pwm_b.h"
#include "asc.h"
#include "can1.h"
#include "leds_interface.h"
#include "currents_interface.h"
// variabili per l'inseguimento del duty cycle
sDutyControlBL DutyCycle[2];
sDutyControlBL DutyCycleReq[2];

extern sPwmControlBL *pTable0;
extern sPwmControlBL *pTable1;
extern UInt16 status0 ;
extern UInt16 status1 ;
extern UInt16 old_status0 ;
extern UInt16 old_status1 ;
extern sPwmControlBL PWMState[2];
extern Int32 comm_enc[2]; 
extern UInt8 hall_error[2];
Int32 comm_count[2] = {0, 0};

//debug

/* extracting from a short */
#define BYTE_L(x) (__shr(x, 8))
#define BYTE_H(x) (x & 0xff)
canmsg_t _can;					// buffer to prepare messages for send 
//end 

//**********************************************************************
sPwmControlBL bldcCommutationTableComp[8] = {{0x003F, 0x0000} ,
					 {0x000F, 0x0002} ,
					 {0x0033, 0x0020} ,
					 {0x0033, 0x0002} ,
					 {0x003C, 0x0008} ,
					 {0x000F, 0x0008} ,
					 {0x003C, 0x0020} ,
					 {0x003F, 0x0000} };
																							 
sPwmControlBL bldcCommutationTableCompInv[8] = {{0x003F, 0x0000} ,
					{0x003C, 0x0020} ,
		 			{0x000F, 0x0008} ,
					{0x003C, 0x0008} ,
					{0x0033, 0x0002} ,
					{0x0033, 0x0020} ,
					{0x000F, 0x0002} ,
					{0x003F, 0x0000} };
					
//**********************************************************************
void TD0_Enable(void)
{
	setRegBits (TMRD0_CTRL, 0x2000);
}

void TD0_Disable(void)
{
	clrRegBits (TMRD0_CTRL, 0x2000);	
}

//**********************************************************************
/**
 * returns the speed of the brushless
 */
Int32 get_speed(byte axis)
{
	if (axis>=0 && axis <2)	return comm_count[axis];
}

//*********************************
/**
/* returns the hall sensor commutations
*/
Int32 get_commutations(byte axis)
{
	if (axis>=0 && axis <2)	return comm_enc[axis];
}

//*********************************
/**
/* returns the hall sensor commutations
*/
void set_commutations(byte axis, Int32 value)
{
	if (axis>=0 && axis <2)	comm_enc[axis]=value;
}


//**********************************************************************
/**
 * initializes the counter/timer. the timer is initialized w/ 1ms period.
 */
void TD0_init (void)
{
	/* TMRD0_CTRL: CM=0,PCS=0,SCS=0,ONCE=0,LENGTH=1,DIR=0,Co_INIT=0,OM=0 */
	setReg (TMRD0_CTRL, 0x20);           /* Stop all functions of the timer */

	/* TMRD0_SCR: TCF=0,TCFIE=1,TOF=0,TOFIE=0,IEF=0,IEFIE=0,IPS=0,INPUT=0,Capture_Mode=0,MSTR=0,EEOF=0,VAL=0,FORCE=0,OPS=0,OEN=0 */
	setReg (TMRD0_SCR, 0x4000);
	setReg (TMRD0_LOAD, 0);                /* Reset load register */
	setReg (TMRD0_CMP1, 39999);            /* Store appropriate value to the compare register according to the selected high speed CPU mode */

	clrRegBits (TMRD0_CTRL, 0x1e00);
	setRegBits (TMRD0_CTRL, 4096);    /* Set prescaler register according to the selected high speed CPU mode */
	setReg 	   (TMRD0_CNTR, 0); 		   /* Reset counter */
		
	clrRegBits (TMRD0_CTRL, 0xe000);
	TD0_Enable();						   /* counter on! */
}

/**
 * isr timer. 
 */
#pragma interrupt saveall
void TD0_interrupt(void)
{
	UInt8 tmp,val;
	static Int16 counter = 0;
	UInt16 hallstatus0=0;
	UInt16 hallstatus1=0;
	UInt8 hallflag[2]={0,0};
	
//#ifdef DEBUG_CAN_MSG 
	hallstatus0=(getReg(QD0_IMR) >> 5) &0x07;
	if ((hallstatus0==0x00) || (hallstatus0==0x07)) 
	{
		hall_error[0] |=HALL_ERROR_TD0;
	}

	hallflag[0]=getRegBits(TMRA0_SCR, TMRA0_SCR_IEF_MASK)+getRegBits(TMRA1_SCR, TMRA1_SCR_IEF_MASK) +getRegBits(TMRA2_SCR, TMRA2_SCR_IEF_MASK);
	if ((hallstatus0!=status0)  && (hallflag[0]==0))
	{
		old_status0=hallstatus0;
		hall_error[0] |=HALL_ERROR_TD0;
		PWMState[0] = pTable0[hallstatus0];
		tmp = getReg(PWMA_PMOUT) & 0x8000;
		val=tmp | PWMState[0].MaskOut | (PWMState[0].Mask<<8);
		setReg(PWMA_PMOUT,val);
	#ifdef DEBUG_CAN_MSG 
		can_printf("hall0 mismatch");
	#endif
	}
	
	hallstatus1=(getReg(QD1_IMR) >> 5) &0x07;
	if ((hallstatus1==0x00) || (hallstatus1==0x07))  
	{
		hall_error[1]=HALL_ERROR_TD0;
	}
	hallflag[1]=getRegBits(TMRB0_SCR, TMRB0_SCR_IEF_MASK)+getRegBits(TMRB1_SCR, TMRB1_SCR_IEF_MASK)+ getRegBits(TMRB2_SCR, TMRB2_SCR_IEF_MASK);	

	if ((hallstatus1!=status1) && (hallflag[1]==0))
	{
	
		old_status1=hallstatus1;
		hall_error[1]=HALL_ERROR_TD0;		
		PWMState[1] = pTable1[hallstatus1];
		tmp = getReg(PWMB_PMOUT) & 0x8000;
		val=tmp | PWMState[1].MaskOut | (PWMState[1].Mask<<8);
		setReg(PWMB_PMOUT,val); 
	#ifdef DEBUG_CAN_MSG
		can_printf("hall1 mismatch");
	#endif	
	}		
//#endif
	
	if (DutyCycleReq[0].Duty < MIN_DUTY)
		DutyCycleReq[0].Duty=MIN_DUTY;
	if (DutyCycleReq[0].Duty > MAX_DUTY)
		DutyCycleReq[0].Duty=MAX_DUTY;
	if (DutyCycleReq[1].Duty < MIN_DUTY)
		DutyCycleReq[1].Duty=MIN_DUTY;
	if (DutyCycleReq[1].Duty > MAX_DUTY)
		DutyCycleReq[1].Duty=MAX_DUTY;

	if (DutyCycleReq[0].Dir != DutyCycle[0].Dir) 
	{
		if ((DutyCycle[0].Duty-STEP) < (MIN_DUTY)) //then direction change 
		{
			DutyCycle[0].Duty=DutyCycle[0].Duty-STEP;
			
			if (DutyCycle[0].Duty<(-MIN_DUTY))
			{
		 		DutyCycle[0].Duty=-DutyCycle[0].Duty;
				if (DutyCycle[0].Duty>DutyCycleReq[0].Duty) DutyCycle[0].Duty=DutyCycleReq[0].Duty; 	
			}
			else
			{
				DutyCycle[0].Duty=MIN_DUTY;
			}
			
			if (DutyCycleReq[0].Dir) pTable0 = bldcCommutationTableCompInv;
			else pTable0 = bldcCommutationTableComp; 
		    // after a direction change the pwm must be changed immediately, before a hall sensor changed  
			hallstatus0=(getReg(QD0_IMR) >> 1) &0x07;
			PWMState[0] = pTable0[hallstatus0];
			tmp = getReg(PWMA_PMOUT) & 0x8000;
			val=tmp | PWMState[0].MaskOut | (PWMState[0].Mask<<8);
			setReg(PWMA_PMOUT,val);			
			DutyCycle[0].Dir = DutyCycleReq[0].Dir;
			
		}		
		else //no direction change
		{	
			DutyCycle[0].Duty=DutyCycle[0].Duty-STEP;
		}
	}
	else {
			if (DutyCycleReq[0].Duty > DutyCycle[0].Duty) 
			{
				if (DutyCycleReq[0].Duty-DutyCycle[0].Duty>=STEP)
					DutyCycle[0].Duty=DutyCycle[0].Duty+STEP;
				else
					DutyCycle[0].Duty=DutyCycleReq[0].Duty;
			}
			else if (DutyCycleReq[0].Duty < DutyCycle[0].Duty) 
			{
				if (DutyCycle[0].Duty-DutyCycleReq[0].Duty>=STEP)
					DutyCycle[0].Duty=DutyCycle[0].Duty-STEP;
				else
					DutyCycle[0].Duty=DutyCycleReq[0].Duty;
			}
		
	//	else TD0_Disable();
	}
	
	if (DutyCycleReq[1].Dir != DutyCycle[1].Dir) 
	{
		if ((DutyCycle[1].Duty-STEP) < (MIN_DUTY)) //then direction change 
		{
			DutyCycle[1].Duty=DutyCycle[1].Duty-STEP;
			
			if (DutyCycle[1].Duty<(-MIN_DUTY))
			{
		 		DutyCycle[1].Duty=-DutyCycle[1].Duty;
				if (DutyCycle[1].Duty>DutyCycleReq[1].Duty) DutyCycle[1].Duty=DutyCycleReq[1].Duty; 	
			}
			else
			{
				DutyCycle[1].Duty=MIN_DUTY;
			}
			
			if (DutyCycleReq[1].Dir) pTable1 = bldcCommutationTableCompInv;
			else pTable1 = bldcCommutationTableComp; 
		    
			// after a direction change the pwm must be changed immediately, before a hall sensor changed  

		
			hallstatus1=(getReg(QD1_IMR) >> 1) &0x07;
			PWMState[1] = pTable1[hallstatus1];
			tmp = getReg(PWMB_PMOUT) & 0x8000;
			val=tmp | PWMState[1].MaskOut | (PWMState[1].Mask<<8);
			setReg(PWMB_PMOUT,val); 
	    	
			DutyCycle[1].Dir = DutyCycleReq[1].Dir;
		}		
		else //no direction change 
		{
			DutyCycle[1].Duty=DutyCycle[1].Duty-STEP;
		}
	}
	else {
	 
			if (DutyCycleReq[1].Duty > DutyCycle[1].Duty) 
			{
				if (DutyCycleReq[1].Duty-DutyCycle[1].Duty>=STEP)
					DutyCycle[1].Duty=DutyCycle[1].Duty+STEP;
				else
					DutyCycle[1].Duty=DutyCycleReq[1].Duty;
			}
			else if (DutyCycleReq[1].Duty < DutyCycle[1].Duty) 
			{
				if (DutyCycle[1].Duty-DutyCycleReq[1].Duty>=STEP)
					DutyCycle[1].Duty=DutyCycle[1].Duty-STEP;
				else
					DutyCycle[1].Duty=DutyCycleReq[1].Duty;
			}
		
	//	else TD0_Disable();
	}
	
	PWM_generate_BLL(0, DutyCycle[0].Duty); 	
	PWM_generate_BLL(1, DutyCycle[1].Duty); 
		
	clrRegBit (TMRD0_SCR, TCF);            /* Reset interrupt request flag */
	
}

//*********************************************************
void Init_Brushless_Comm(byte axis_number)
{
	UInt8 tmp;
	if (axis_number==2)
	{
	
	DutyCycle[0].Duty = MIN_DUTY;
	DutyCycle[0].Dir = 0;
	DutyCycleReq[0].Duty = MIN_DUTY;
	DutyCycleReq[0].Dir = 0;
	pTable0 = bldcCommutationTableComp;

	DutyCycle[1].Duty = MIN_DUTY;
	DutyCycle[1].Dir = 0;
	DutyCycleReq[1].Duty = MIN_DUTY;
	DutyCycleReq[1].Dir = 0;
	pTable1 = bldcCommutationTableComp;
		
	Init_Hall_Effect_0();
	Init_Hall_Effect_1();

	// inizializzazione dello stato del motore
	status0=(getReg(QD0_IMR) >> 1) &0x07;
	status1=(getReg(QD1_IMR) >> 1) &0x07;
	
	PWMState[0] = pTable0[status0];
	old_status0 = status0;

	PWMState[1] = pTable1[status1];
	old_status1 = status1;	
	
	//Init PWM
	PWM_A_init ();
	PWM_B_init ();
	
	// write mask to PWM Channel Control Register / 
	tmp = getReg(PWMA_PMOUT) & 0x8000;
	// Set output control enable to PWMOUT 
	setReg(PWMA_PMOUT,tmp | PWMState[0].MaskOut | (PWMState[0].Mask<<8)); 

	tmp = getReg(PWMB_PMOUT) & 0x8000;
	// Set output control enable to PWMOUT 
	setReg(PWMB_PMOUT,tmp | PWMState[1].MaskOut | (PWMState[1].Mask<<8)); 
	}
	
	if (axis_number==1)
	{
		
	DutyCycle[0].Duty = MIN_DUTY;
	DutyCycle[0].Dir = 0;
	DutyCycleReq[0].Duty = MIN_DUTY;
	DutyCycleReq[0].Dir = 0;
	pTable0 = bldcCommutationTableComp;
		
	Init_Hall_Effect_0();

	// inizializzazione dello stato del motore
	status0=(getReg(QD0_IMR) >> 1) &0x07;

	PWMState[0] = pTable0[status0];
	old_status0 = status0;
	
	//Init PWM
	PWM_A_init ();
	// write mask to PWM Channel Control Register / 
	tmp = getReg(PWMA_PMOUT) & 0x8000;
	// Set output control enable to PWMOUT 
	setReg(PWMA_PMOUT,tmp | PWMState[0].MaskOut | (PWMState[0].Mask<<8)); 

	}
	//Init Current
	init_currents();
	
	// Init duty cycle timer
	TD0_init();
}
