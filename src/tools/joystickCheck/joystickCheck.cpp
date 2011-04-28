/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Marco Randazzo
 * email:  marco.randazzo@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

/** 
\defgroup joystickCheck joystickCheck
 
@ingroup icub_tools
 
This module returns the number of joysticks detected on the system.
 
Copyright (C) 2011 RobotCub Consortium
 
Author: Marco Randazzo

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
 
This module returns the number of joysticks detected on the system.

\section portsa_sec Ports Accessed
None. 
 
\section portsc_sec Ports Created 
None. 

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files 
None. 
 
\section conf_file_sec Configuration Files
None. 

\section tested_os_sec Tested OS
Windows, Linux

\author Marco Randazzo
*/ 

#include <iostream>
#include <SDL.h>

int main( int argc, char **argv ) 
{
		// start SDL subsystem
		if ( SDL_InitSubSystem ( SDL_INIT_JOYSTICK ) < 0 )
		{
			fprintf ( stderr, "JoystickCheck: Unable to initialize joystick system: %s\n", SDL_GetError() );
			return -1;
		}

		// get the list of available joysticks
		fprintf ( stderr, "\n");
		int joy_id=0;
		int joystick_num = SDL_NumJoysticks ();
		if (joystick_num == 0)
		{
			fprintf ( stderr, "JoystickCheck: no joysticks found\n");
			return 0;
		}
		else if (joystick_num == 1)
		{
			fprintf ( stderr, "JoystickCheck: one joystick found \n");
			return 1;
		}
		else
		{
			fprintf ( stderr, "JoystickCheck: multiple (%d) joysticks found\n",joystick_num);
			return joystick_num;
		}
}