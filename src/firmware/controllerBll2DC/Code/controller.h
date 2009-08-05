/////////////////////////////////////////////////////////////////////////
///                                                                   ///
///       YARP - Yet Another Robotic Platform (c) 2001-2003           ///
///                                                                   ///
///                    #Add our name(s) here#                         ///
///                                                                   ///
///     "Licensed under the Academic Free License Version 1.0"        ///
///                                                                   ///
/// The complete license description is contained in the              ///
/// licence.template file included in this distribution in            ///
/// $YARP_ROOT/conf. Please refer to this file for complete           ///
/// information about the licensing of YARP                           ///
///                                                                   ///
/// DISCLAIMERS: LICENSOR WARRANTS THAT THE COPYRIGHT IN AND TO THE   ///
/// SOFTWARE IS OWNED BY THE LICENSOR OR THAT THE SOFTWARE IS         ///
/// DISTRIBUTED BY LICENSOR UNDER A VALID CURRENT LICENSE. EXCEPT AS  ///
/// EXPRESSLY STATED IN THE IMMEDIATELY PRECEDING SENTENCE, THE       ///
/// SOFTWARE IS PROVIDED BY THE LICENSOR, CONTRIBUTORS AND COPYRIGHT  ///
/// OWNERS "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, ///
/// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   ///
/// FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO      ///
/// EVENT SHALL THE LICENSOR, CONTRIBUTORS OR COPYRIGHT OWNERS BE     ///
/// LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN   ///
/// ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN ///
/// CONNECTION WITH THE SOFTWARE.                                     ///
///                                                                   ///
/////////////////////////////////////////////////////////////////////////

///
/// $Id: controller.h,v 1.3 2009/03/03 07:36:43 babybot Exp $
///
///
  
#ifndef __controllerh__
#define __controllerh__

#ifndef __ONLY_DEF
#include "dsp56f807.h"
#endif

/* 
 * the purpose of including this file on Linux/Winnt/Qnx is to 
 *     get the definition of messages and params of the dsp controller
 * define __ONLY_DEF before inclusion on Linux/Winnt/Qnx
 */

//board_types
#define BOARD_TYPE_2DC    0
#define BOARD_TYPE_4DC    1
#define BOARD_TYPE_2BLL   2
#define BOARD_TYPE_EVM    3
#define BOARD_TYPE_2BLLDC 4


#define CURRENT_BOARD_TYPE  BOARD_TYPE_2BLLDC
#define DEFAULT_BOARD_ID    15
#define SMALL_BUFFER_SIZE 	32		/* for serial I/O */
#define CONTROLLER_PERIOD 	1		/* espressed in ms */
#define JN 					2		/* number of axes */

#define MODE_IDLE					0
#define MODE_POSITION 				1
#define MODE_VELOCITY				2
#define MODE_TORQUE					3
#define MODE_CALIB_ABS_POS_SENS		0x10
#define MODE_CALIB_HARD_STOPS		0x20
#define MODE_HANDLE_HARD_STOPS		0x30
#define MODE_MARGIN_REACHED    		0x40

#define MODE_CONTROLLED				1
#define MODE_CALIB				    2

//   Calibration Type Messages 
#define CALIB_HARD_STOPS            0 
#define CALIB_ABS_POS_SENS          1
#define CALIB_HARD_STOPS_DIFF       2 
#define CALIB_ABS_DIGITAL           3

#define DEFAULT_VELOCITY 10
#define DEFAULT_ACCELERATION 10
#define DEFAULT_MAX_POSITION 5000
#define DEFAULT_MAX_VELOCITY 0x7fff

#ifndef __ONLY_DEF
/* deals with the endianism - byte 4 is the MSB on the Pentium */
#define BYTE_1(x) (__shr(__extract_h(x), 8))
#define BYTE_2(x) (__extract_h(x) & 0xff)
#define BYTE_3(x) (__shr(__extract_l(x), 8))
#define BYTE_4(x) (__extract_l(x) & 0xff)

/* extracting from a short */
#define BYTE_L(x) (__shr(x, 8))
#define BYTE_H(x) (x & 0xff)

/* same as above, deals also with endianism */
dword BYTE_C(byte x4, byte x3, byte x2, byte x1);
	 
#define BYTE_W(x2, x1) (__shl(x1,8) | x2)

#if (JN == 2)
	#define INIT_ARRAY(x) {x,x}
#elif (JN == 4)
	#define INIT_ARRAY(x) {x,x,x,x}
#else
	#error unable to init array for JN != 2,4
#endif
    
/* clrRegBit(CANRIER, RXFIE), setRegBit(CANRIER, RXFIE) */

#define CAN_SYNCHRO_STEPS 5
#define BOARDSTATUSTIME 5000 //the time in ms between a BCAST_STATUS message and another

#endif

#endif
