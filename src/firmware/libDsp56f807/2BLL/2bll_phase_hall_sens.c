
#include "phase_hall_sens.h"
#include "pwm_interface.h"
#include "qd0.h"
#include "qd1.h" 
#include "leds_interface.h"

volatile   UInt16 status0 = 0;
volatile   UInt16 status1 = 0;
volatile   UInt16 old_status0 = 0;
volatile   UInt16 old_status1 = 0;
volatile   UInt8 hall_error[2]={0,0};


//*********************************************************
// definizione varibili																			 
volatile sPwmControlBL *pTable0;
volatile sPwmControlBL *pTable1;

const UWord16 DIRECTION_TABLE[8] = {0, 5, 3, 1, 6, 4, 2, 0};
const UWord16 DIRECTION_TABLE_INV[8] = {0, 3, 6, 2, 5, 1, 4, 0};

volatile Int32 comm_enc[2]={0,0}; 
sPwmControlBL PWMState[2];

//*********************************************************
// Local Prototypes
void Interrupt_Phase_X0(void);
void Interrupt_Phase_Y0(void);
void Interrupt_Phase_Z0(void);

void Interrupt_Phase_X1(void);
void Interrupt_Phase_Y1(void);
void Interrupt_Phase_Z1(void);

//*********************************************************
void Init_Hall_Effect_0(void)
{
  
  // TMRA0_CTRL: CM=1,PCS=8,SCS=0,ONCE=0,LENGTH=0,DIR=0,Co_INIT=0,OM=0 
  //                    CCCPPPPSS
  setReg (TMRA0_CTRL, 0x3000);              
  // TMRA0_SCR: TCF=0,TCFIE=0,TOF=0,TOFIE=0,IEF=0,IEFIE=1,IPS=0,INPUT=0,Capture_Mode=3,MSTR=0,EEOF=0,VAL=0,FORCE=0,OPS=0,OEN=0 
  //                    CCOOEEIIPP
  setReg (TMRA0_SCR,  0x4C0);
  setReg (TMRA0_CNTR,0);                // Reset counter register 
  setReg (TMRA0_LOAD,0);                // Reset load register 
  setReg (TMRA0_CAP,0);                 // Reset capture register 	

  // TMRA1_CTRL: CM=1,PCS=8,SCS=1,ONCE=0,LENGTH=0,DIR=0,Co_INIT=0,OM=0 
  //                    CCCPPPPSS
  setReg (TMRA1_CTRL, 0x3080);              
  // TMRA1_SCR: TCF=0,TCFIE=0,TOF=0,TOFIE=0,IEF=0,IEFIE=1,IPS=0,INPUT=0,Capture_Mode=3,MSTR=0,EEOF=0,VAL=0,FORCE=0,OPS=0,OEN=0 
  //                    CCOOEEIIPP
  setReg (TMRA1_SCR,  0x4C0);
  setReg (TMRA1_CNTR,0);                // Reset counter register 
  setReg (TMRA1_LOAD,0);                // Reset load register 
  setReg (TMRA1_CAP,0);                 // Reset capture register 	
  
  // TMRA2_CTRL: CM=1,PCS=8,SCS=2,ONCE=0,LENGTH=0,DIR=0,Co_INIT=0,OM=0 
  //                    CCCPPPPSS
  setReg (TMRA2_CTRL, 0x3100);              
  // TMRA2_SCR: TCF=0,TCFIE=0,TOF=0,TOFIE=0,IEF=0,IEFIE=1,IPS=0,INPUT=0,Capture_Mode=3,MSTR=0,EEOF=0,VAL=0,FORCE=0,OPS=0,OEN=0 
  //                    CCOOEEIIPP
  setReg (TMRA2_SCR,  0x4C0);
  setReg (TMRA2_CNTR,0);                // Reset counter register 
  setReg (TMRA2_LOAD,0);                // Reset load register 
  setReg (TMRA2_CAP,0);                 // Reset capture register 	

}				

//*********************************************************
void Init_Hall_Effect_1(void)
{

	
  // TMRB0_CTRL: CM=1,PCS=8,SCS=0,ONCE=0,LENGTH=0,DIR=0,Co_INIT=0,OM=0 
  //                    CCCPPPPSS
  setReg (TMRB0_CTRL, 0x3000);              
  // TMRB0_SCR: TCF=0,TCFIE=0,TOF=0,TOFIE=0,IEF=0,IEFIE=1,IPS=0,INPUT=0,Capture_Mode=1,MSTR=0,EEOF=0,VAL=0,FORCE=0,OPS=0,OEN=0 
  //                    CCOOEEIIPP
  setReg (TMRB0_SCR,  0x4C0);
  setReg (TMRB0_CNTR,0);                // Reset counter register 
  setReg (TMRB0_LOAD,0);                // Reset load register 
  setReg (TMRB0_CAP,0);                 // Reset capture register 	

  // TMRB1_CTRL: CM=1,PCS=8,SCS=1,ONCE=0,LENGTH=0,DIR=0,Co_INIT=0,OM=0 
  //                    CCCPPPPSS
  setReg (TMRB1_CTRL, 0x3080);              
  // TMRB1_SCR: TCF=0,TCFIE=0,TOF=0,TOFIE=0,IEF=0,IEFIE=1,IPS=0,INPUT=0,Capture_Mode=1,MSTR=0,EEOF=0,VAL=0,FORCE=0,OPS=0,OEN=0 
  //                    CCOOEEIIPP
  setReg (TMRB1_SCR,  0x4C0);
  setReg (TMRB1_CNTR,0);                // Reset counter register 
  setReg (TMRB1_LOAD,0);                // Reset load register 
  setReg (TMRB1_CAP,0);                 // Reset capture register 	
  
  // TMRB2_CTRL: CM=1,PCS=8,SCS=2,ONCE=0,LENGTH=0,DIR=0,Co_INIT=0,OM=0 
  //                    CCCPPPPSS
  setReg (TMRB2_CTRL, 0x3100);              
  // TMRB2_SCR: TCF=0,TCFIE=0,TOF=0,TOFIE=0,IEF=0,IEFIE=1,IPS=0,INPUT=0,Capture_Mode=1,MSTR=0,EEOF=0,VAL=0,FORCE=0,OPS=0,OEN=0 
  //                    CCOOEEIIPP  
  setReg (TMRB2_SCR,  0x4C0);
  setReg (TMRB2_CNTR,0);                // Reset counter register 
  setReg (TMRB2_LOAD,0);                // Reset load register 
  setReg( TMRB2_CAP,0);                 // Reset capture register
  
}


//*********************************************************
#pragma interrupt saveall
void Interrupt_Phase_X0(void)
{	
	UInt8 tmp,val,deb;
	deb=(getReg(QD0_IMR) >> 5) &0x7;
	// conteggio fonti rispetto direzione	
	if ((deb == DIRECTION_TABLE[old_status0]) || (deb == DIRECTION_TABLE_INV[old_status0]))
	{
		status0=(getReg(QD0_IMR) >> 5) &0x7;
		if (status0==deb)
		{
		// write mask to PWM Channel Control Register / 
    	PWMState[0]= pTable0[status0];
		tmp = getReg(PWMA_PMOUT) & 0x8000;
    	val=tmp | PWMState[0].MaskOut | (PWMState[0].Mask<<8);
		setReg(PWMA_PMOUT,val); 
		old_status0 = status0;	
		if (status0 == DIRECTION_TABLE[old_status0]) comm_enc[0]++;
		else if (status0 == DIRECTION_TABLE_INV[old_status0]) comm_enc[0]--;
		}
		else
		{
			hall_error[0] |=HALL_ERROR_GLITCH;
		}

	}
	else hall_error[0]  |=HALL_ERROR_TABLE;

	if (((getReg(QD0_IMR) >> 5) &0x07)!=status0) 
	{
		hall_error[0] |=HALL_ERROR_GLITCH;
	}
	clrRegBits(TMRA0_SCR, TMRA0_SCR_IEF_MASK);

}
#pragma interrupt saveall
void Interrupt_Phase_Y0(void)
{
	UInt8 tmp,val,deb;
	deb=(getReg(QD0_IMR) >> 5) &0x7;
	// conteggio fonti rispetto direzione	
	if ((deb == DIRECTION_TABLE[old_status0]) || (deb == DIRECTION_TABLE_INV[old_status0]))
	{
		status0=(getReg(QD0_IMR) >> 5) &0x7;
		if (status0==deb)
		{
		// write mask to PWM Channel Control Register / 
    	PWMState[0]= pTable0[status0];
		tmp = getReg(PWMA_PMOUT) & 0x8000;
    	val=tmp | PWMState[0].MaskOut | (PWMState[0].Mask<<8);
		setReg(PWMA_PMOUT,val); 
		old_status0 = status0;	
		if (status0 == DIRECTION_TABLE[old_status0]) comm_enc[0]++;
		else if (status0 == DIRECTION_TABLE_INV[old_status0]) comm_enc[0]--;
		}
		else
		{
			hall_error[0] |=HALL_ERROR_GLITCH;
		}

	}
	else hall_error[0]  |=HALL_ERROR_TABLE;

	if (((getReg(QD0_IMR) >> 5) &0x07)!=status0) 
	{
		hall_error[0] |=HALL_ERROR_GLITCH;
	}
	clrRegBits(TMRA1_SCR, TMRA1_SCR_IEF_MASK);
}
#pragma interrupt saveall
void Interrupt_Phase_Z0(void)
{
	UInt8 tmp,val,deb;
	deb=(getReg(QD0_IMR) >> 5) &0x7;
	// conteggio fonti rispetto direzione	
	if ((deb == DIRECTION_TABLE[old_status0]) || (deb == DIRECTION_TABLE_INV[old_status0]))
	{
		status0=(getReg(QD0_IMR) >> 5) &0x7;
		if (status0==deb)
		{
		// write mask to PWM Channel Control Register / 
    	PWMState[0]= pTable0[status0];
		tmp = getReg(PWMA_PMOUT) & 0x8000;
    	val=tmp | PWMState[0].MaskOut | (PWMState[0].Mask<<8);
		setReg(PWMA_PMOUT,val); 
		old_status0 = status0;	
		if (status0 == DIRECTION_TABLE[old_status0]) comm_enc[0]++;
		else if (status0 == DIRECTION_TABLE_INV[old_status0]) comm_enc[0]--;
		}
		else
		{
			hall_error[0] |=HALL_ERROR_GLITCH;
		}

	}
	else hall_error[0]  |=HALL_ERROR_TABLE;

	if (((getReg(QD0_IMR) >> 5) &0x07)!=status0) 
	{
		hall_error[0] |=HALL_ERROR_GLITCH;
	}

	
	clrRegBits(TMRA2_SCR, TMRA2_SCR_IEF_MASK);
}

//*********************************************************
#pragma interrupt saveall
void Interrupt_Phase_X1(void)
{
	UInt8 tmp,val,deb;
	deb=(getReg(QD1_IMR) >> 5) &0x07;
	
	// conteggio fonti rispetto direzione	
	if ((deb == DIRECTION_TABLE[old_status1]) || (deb == DIRECTION_TABLE_INV[old_status1]))
	{
		status1=(getReg(QD1_IMR) >> 5) &0x07;
		if (status1==deb)
		{
			PWMState[1] = pTable1[status1];
			// write mask to PWM Channel Control Register / 
			tmp = getReg(PWMB_PMOUT) & 0x8000;
			val=tmp | PWMState[1].MaskOut | (PWMState[1].Mask<<8);
			setReg(PWMB_PMOUT,val);
			if (status1 == DIRECTION_TABLE[old_status1]) comm_enc[1]++;
			else if (status1 == DIRECTION_TABLE_INV[old_status1]) comm_enc[1]--;
			old_status1 = status1;	
		}
		else
			{
				hall_error[1] |=HALL_ERROR_GLITCH;
			}

	}
	else hall_error[1] |=HALL_ERROR_TABLE;
	
	if (((getReg(QD1_IMR) >> 5) &0x07)!=status1) 
	{
		hall_error[1] |=HALL_ERROR_GLITCH;
	}
	
	clrRegBits(TMRB0_SCR, TMRB0_SCR_IEF_MASK);
}
#pragma interrupt saveall
void Interrupt_Phase_Y1(void)
{
	UInt8 tmp,val,deb;
	deb=(getReg(QD1_IMR) >> 5) &0x07;
	
	// conteggio fonti rispetto direzione	
	if ((deb == DIRECTION_TABLE[old_status1]) || (deb == DIRECTION_TABLE_INV[old_status1]))
	{
		status1=(getReg(QD1_IMR) >> 5) &0x07;
		if (status1==deb)
		{
	 		PWMState[1] = pTable1[status1];
			// write mask to PWM Channel Control Register / 
			tmp = getReg(PWMB_PMOUT) & 0x8000;
			val=tmp | PWMState[1].MaskOut | (PWMState[1].Mask<<8);
			setReg(PWMB_PMOUT,val);
			if (status1 == DIRECTION_TABLE[old_status1]) comm_enc[1]++;
			else if (status1 == DIRECTION_TABLE_INV[old_status1]) comm_enc[1]--;
			old_status1 = status1;	
		}
		else
			{
				hall_error[1] |=HALL_ERROR_GLITCH;
			}

	}
	else hall_error[1] |=HALL_ERROR_TABLE;
	
	if (((getReg(QD1_IMR) >> 5) &0x07)!=status1) 
	{
		hall_error[1] |=HALL_ERROR_GLITCH;
	}
	clrRegBits(TMRB1_SCR, TMRB1_SCR_IEF_MASK);
}
#pragma interrupt saveall
void Interrupt_Phase_Z1(void)
{
	UInt8 tmp,val,deb;
	deb=(getReg(QD1_IMR) >> 5) &0x07;
	
	// conteggio fonti rispetto direzione	
	if ((deb == DIRECTION_TABLE[old_status1]) || (deb == DIRECTION_TABLE_INV[old_status1]))
	{
		status1=(getReg(QD1_IMR) >> 5) &0x07;
		if (status1==deb)
		{
			PWMState[1] = pTable1[status1];
			// write mask to PWM Channel Control Register / 
			tmp = getReg(PWMB_PMOUT) & 0x8000;
			val=tmp | PWMState[1].MaskOut | (PWMState[1].Mask<<8);
			setReg(PWMB_PMOUT,val);
			if (status1 == DIRECTION_TABLE[old_status1]) comm_enc[1]++;
			else if (status1 == DIRECTION_TABLE_INV[old_status1]) comm_enc[1]--;
			old_status1 = status1;	
		}
		else
			{
				hall_error[1] |=HALL_ERROR_GLITCH;
			}

	}
	else hall_error[1] |=HALL_ERROR_TABLE;
	
	if (((getReg(QD1_IMR) >> 5) &0x07)!=status1) 
	{
		hall_error[1] |=HALL_ERROR_GLITCH;
	}
	clrRegBits(TMRB2_SCR, TMRB2_SCR_IEF_MASK);
}

UInt8 Get_Sens0_Status(void)
{
	return status0;
}

UInt8 Get_Sens1_Status(void)
{
	return status1;
}
UInt8 getHallStatus(UInt16 channel)
{
	return hall_error[channel];
}

void setHallStatus(UInt16 channel,UInt8 val)
{
	hall_error[channel]=val;
}

