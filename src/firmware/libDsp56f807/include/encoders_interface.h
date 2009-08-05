
#ifndef __encoders_interface_h__
#define __encoders_interface_h__

/***************************************************************************/
/**
 *	this function inits the encoder sensors
 ***************************************************************************/ 
void  init_position_encoder(void);

/***************************************************************************/
/**
 * this function reads the current position which will be used in the PID.
 * Measurament is given by the quadrature encoder (joints 0,1) or
 * software (joints 2,3)
 * @param   jnt is the joint number 
 * @return  the reading of the sensor
 ***************************************************************************/
Int32 get_position_encoder(byte jnt);

/***************************************************************************/
/**
 * this function set the current position of the specified encoder
 * @param  jnt is the joint number 
 * @param  position is the new position of the encoder
 ***************************************************************************/
void  set_position_encoder(byte jnt, dword position);

#endif 
