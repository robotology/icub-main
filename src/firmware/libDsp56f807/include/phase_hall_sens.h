
#ifndef __phase_hall_sensh__
#define __phase_hall_sensh__


#include "dsp56f807.h"

//  definizione strutture dati
typedef struct sPwmControlBL_tag
{
	UWord16    Mask;
	UWord16    MaskOut;
} sPwmControlBL;

#define  HALL_ERROR_TD0   	0x1  //when there is a mismatch between Hall reading in the hall_interrupt and the 1ms timer
#define  HALL_ERROR_TABLE 	0x2  //when the reading of the hall in the hall_interrupt is not congruent with the table  
#define  HALL_ERROR_GLITCH	0x4 //when there is a mismatch between the two hall reading in the hall_interrupt 



/**************************************************************************************/
/**
 * Inits the Hall Effect Sensor, channel 0
 */
/**************************************************************************************/
void Init_Hall_Effect_0(void);

/**************************************************************************************/
/**
 * Inits the Hall Effect Sensor, channel 1
 */
/**************************************************************************************/
void Init_Hall_Effect_1(void);

/**************************************************************************************/
/**
 * Return the Status of the Halla Effect Sensor, channel 0
 * @return a byte with the 3 lsb bits indicating the status of the three sensors (0b00000xyz) 
 */
/**************************************************************************************/
UInt8 Get_Sens0_Status(void);

/**************************************************************************************/
/**
 * Return the Status of the Halla Effect Sensor, channel 0
 * @return a byte with the 3 lsb bits indicating the status of the three sensors (0b00000xyz) 
 */
/**************************************************************************************/
UInt8 Get_Sens1_Status(void);

UInt8 getHallStatus(UInt16 channel);
void setHallStatus(UInt16 channel,UInt8 val);
#endif 
