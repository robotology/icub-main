#ifndef calibration_h
#define calibration_h

void check_in_position_calib(byte jnt);
byte calibrate (byte channel, byte type, Int16 param1,Int16 param2, Int16 param3);

#endif
