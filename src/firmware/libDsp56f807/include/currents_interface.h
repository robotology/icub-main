#ifndef __currents_interface_h__
#define __currents_interface_h__

#include "dsp56f807.h"

extern Int32 _current [];				// current through the transistors
extern Int32 _current_old [] ;			// current at t-1
extern Int32 _filt_current [] ;     	// filtered current through the transistors
extern Int32 _max_allowed_current  [] ; // limit on the current in micro-ampere
extern float _conversion_factor  [] ;	// limit on the current as set by the interface (later converted into the filter parameter) 
extern Int32 _current_offset[];         //current offset at the beginning

extern Int16 _current_debug [];

extern Int32 MAX_CURRENT;   //MAX current in microAmpere
extern Int32 MAX_I2T_CURRENT;   //MAX current for I2T




void init_currents( void );
void compute_filtcurr(byte jnt);
word check_current(byte jnt, bool sign);
byte set_current_offset(byte jnt);
void compute_i2t(byte jnt);


#endif 
