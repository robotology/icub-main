#include "dsp56f807.h"
#include "qd0.h"
#include "qd1.h"
#include "encoders_interface.h"



/***************************************************************************/
/**
 *	this function inits the quadrature decoder for encoder sensors
 ***************************************************************************/ 
void init_position_encoder(void)
{
	QD0_init ();
	QD1_init ();
	QD0_ResetPosition ();
	QD1_ResetPosition ();
}

/***************************************************************************/
/**
 * this function reads the current _position which will be used in the PID.
 * Measurament is given by the quadrature encoder
 * @param   jnt is the joint number 
 * @return  the reading of the sensor
 ***************************************************************************/
Int32 get_position_encoder(byte jnt)
{
	dword temp=0;
	
	switch (jnt)
	{
		case 0:
			QD0_getPosition (&temp); //old: (dword *)(temp)
		break;		
		case 1:
			QD1_getPosition (&temp);
		break;
	}
	
	return temp;
}

/***************************************************************************/
/**
 * this function set the current position of the specified encoder
 * @param   jnt is the joint number 
 * @param  position is the new position of the econder
 ***************************************************************************/
void set_position_encoder(byte jnt,dword position)
{
	switch (jnt)
	{
		case 0:
			QD0_setPosition (position); 
		break;		
		case 1:
			QD1_setPosition (position);
		break;
	}
}