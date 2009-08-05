
#ifndef __abs_position_sens_analog_h__
#define __abs_position_sens_analog_h__

void init_position_abs_analog(void);

Int16 get_position_abs_analog(byte jnt);
Int32 compute_filt_pos(Int16 value, byte jnt);

#endif 
  