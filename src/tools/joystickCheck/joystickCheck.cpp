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
 
This module is used to check if one joystick is currently used.
 
Copyright (C) 2011 RobotCub Consortium
 
Author: Marco Randazzo

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
 
The module returns 1 to the O.S. if at least one axis/button of the default joystick is pressed, 0 otherwise (no axis/buttons pressed or no joysticks active).

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
#include <yarp/os/Time.h>

int main( int argc, char **argv ) 
{
		// start SDL subsystem
		if ( SDL_InitSubSystem ( SDL_INIT_VIDEO | SDL_INIT_JOYSTICK ) < 0 )
		{
			fprintf ( stderr, "JoystickCheck: Unable to initialize joystick system: %s\n", SDL_GetError() );
			return -1;
		}
		SDL_JoystickEventState ( SDL_ENABLE );
		// get the list of available joysticks
		fprintf ( stderr, "\n");
		int joy_id=0;
		int joystick_num = SDL_NumJoysticks ();
		if (joystick_num == 0)
		{
			fprintf ( stderr, "JoystickCheck: no joysticks found.\n");
			return 0;
		}
		else if (joystick_num > 0)
		{
			fprintf ( stderr, "JoystickCheck: (%d) joystick(s) found.\n",joystick_num);
			int joy_id = 0;
			SDL_Joystick* joy1 = SDL_JoystickOpen ( joy_id );
			if ( joy1 == NULL )
			{
				printf ( "Could not open default joystick.\n" );
				return 0;
			}

			int numAxes    = SDL_JoystickNumAxes    ( joy1 );
			int numBalls   = SDL_JoystickNumBalls   ( joy1 );
			int numHats    = SDL_JoystickNumHats    ( joy1 );
			int numButtons = SDL_JoystickNumButtons ( joy1 );

                       SDL_Event event;
                       for (int trial=0; trial < 100; trial++ )
                       {  
                        //   yarp::os::Time::delay(0.010);//remove me?
                            while(SDL_PollEvent(&event))
                            {
                                switch(event.type)
                                    {  
                                        case SDL_JOYAXISMOTION:
                                        //printf ( "event found axis %d %d \n", event.jaxis.axis ,event.jaxis.value);
                                        break;
                                    }
                             }
                       }

                       for (int trial=0; trial < 1000; trial++ )
                       {  
                            yarp::os::Time::delay(0.010);
                            while(SDL_PollEvent(&event))
                            {
                                switch(event.type)
                                    {  
                                        case SDL_JOYAXISMOTION:
                                       // printf ( "event found axis %d %d \n", event.jaxis.axis ,event.jaxis.value);
                                        if ( ( event.jaxis.value < -20000 ) || (event.jaxis.value > 20000 ) )
                                        {  
                                		printf ( "joysticks activity detected.\n" );                                           
                                                return 1;
                                        }
                                        break;
                                    }
                             }
                       }

                        printf ( "stop here.\n" );
                        /*for (int trial=0; trial < 100; trial++ )
			{
                                yarp::os::Time::delay(0.010);
				SDL_JoystickUpdate ();                      
				for ( int i=0; i < numAxes; ++i )
				{
					rawMeasure[i] = SDL_JoystickGetAxis ( joy1, i );
                                        printf ( "%d %d %d\n",trial,i,rawMeasure[i] );
                                }
        			if (rawMeasure[2] && rawMeasure[5] >= 20000)
				{
						printf ( "found one joystick currently in use.\n" );
						return 1;
				}
			}*/
			printf ( "no joysticks are currently used.\n" );
			return 0;
		}

        return 0;   // pattacini: to prevent MSVC from warning up
}
