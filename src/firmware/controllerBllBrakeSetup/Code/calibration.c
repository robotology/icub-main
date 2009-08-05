/*
 * firmware/controller application.
 *
 */

#include "dsp56f807.h"
#include "options.h"
#include "asc.h"
#include "pid.h"
#include "controller.h"
#include "trajectory.h"
#include "pwm_interface.h"

#include "abs_ssi_interface.h"
#include "calibration.h"

#ifndef VERSION   
#	error "No valid version specified"
#endif

/************************************************************ 
 * this function checks if the calibration is terminated
 * and if calibration is terminated resets the encoder
 ************************************************************/
void check_in_position_calib(byte jnt)
{
  //this function is not used for a board with brushess motors
}   


      
/**************************************************************** 
 * calibration procedure, depends on the firmware version.
 ****************************************************************/
byte calibrate (byte channel, byte type, Int16 param1,Int16 param2, Int16 param3)
{
  // this board has only brushless motor with digital absolute
  // position sensors. So, the only operation required for the 
  // calibration in to set an "offset" value that maps the output
  // of the abs position sensor, in order to avoid the passage on 0.

}
