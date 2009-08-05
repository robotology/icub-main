#ifndef calibration_h
#define calibration_h

void check_in_position_calib(byte jnt);
byte calibrate (byte channel, byte type, Int16 param1,Int16 param2, Int16 param3);

#if ((VERSION == 0x0120) || (VERSION == 0x0121)  || (VERSION == 0x0128)  || (VERSION == 0x0130))
extern Int16 _max_position_enc_tmp[JN];
/* max allowd position for encoder while 
controlling with absolute position sensors*/
#endif

#endif
