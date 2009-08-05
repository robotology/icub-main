
#ifndef __abs_position_sens_ssi_h__
#define __abs_position_sens_ssi_h__

void 	init_position_abs_ssi(void);
UInt16  get_position_abs_ssi(byte jnt);
Int32   get_relative_position_abs_ssi(byte jnt);
Int32   init_relative_position_abs_ssi();
Int32   set_relative_position_abs_ssi(byte jnt,Int32 offset);
void    set_relative_position_abs_ssi_turns(byte jnt, Int16 turn);
UInt16  get_absolute_real_position_abs_ssi(byte jnt);
void    set_max_position(byte jnt, UInt16 max_pos);
UInt16  get_max_position(byte jnt);
void    set_current_as_middle_position(byte jnt);
void get_status_abs_ssi(bool* s_ocf,bool* s_cof,bool* s_lin, bool* s_inc, bool* s_dec, byte jnt);
byte get_error_abs_ssi(byte jnt);
#endif 
