
#include "brushless_comm.h"
#include "phase_hall_sens.h"
#include "pwm_interface.h"
#include "pwm_a.h"
#include "pwm_b.h"

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
Int32 comm_count[2] = {0, 0};

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
	if (axis>0 && axis <2)	return comm_count[axis];
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
	setRegBits (TMRD0_CTRL, 0x08 << 9);    /* Set prescaler register according to the selected high speed CPU mode */
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
	static Int16 counter = 0;	
	if (counter == 20)
	{
		//saves the number of commutations occurred during this period
		comm_count[0]=comm_enc[0];
		comm_count[1]=comm_enc[1];
		
		//zero the commutation econder
		comm_enc[0]=0;
		comm_enc[1]=0;
		
		counter = 0;
	}	
	else counter++;
	
	if (DutyCycleReq[0].Dir != DutyCycle[0].Dir) 
	{
		if (DutyCycle[0].Duty <= (MIN_DUTY)) 
		{
			if (DutyCycleReq[0].Dir) pTable0 = bldcCommutationTableCompInv;
			else pTable0 = bldcCommutationTableComp; 
			DutyCycle[0].Dir = DutyCycleReq[0].Dir;
		}		
		else 
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
	
	//++++++
	
	if (DutyCycleReq[1].Dir != DutyCycle[1].Dir) 
	{
		if (DutyCycle[1].Duty <= (MIN_DUTY)) 
		{
			if (DutyCycleReq[1].Dir) pTable1 = bldcCommutationTableCompInv;
			else pTable1 = bldcCommutationTableComp; 
			DutyCycle[1].Dir = DutyCycleReq[1].Dir;
		}		
		else 
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
void Init_Brushless_Comm()
{
	UInt8 tmp;
	
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
	status0=getReg(QD0_IMR) >> 5;
	status1=getReg(QD1_IMR) >> 5;
	
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

	// Init duty cycle timer
	TD0_init();
}
