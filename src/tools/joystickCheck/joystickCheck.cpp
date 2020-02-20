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
#include <yarp/os/Log.h>
#include <math.h>

int main( int argc, char **argv )
{
    // start SDL subsystem
    SDL_JoystickEventState ( SDL_QUERY );
    if ( SDL_InitSubSystem ( SDL_INIT_JOYSTICK) < 0 )
    {
        yError ("JoystickCheck: Unable to initialize joystick system: %s\n", SDL_GetError() );
        return 0;
    }

    // get the list of available joysticks
    fprintf ( stderr, "\n");
    int joystick_num = SDL_NumJoysticks ();

    // if not joysticks are found, quit immediately
    if (joystick_num <= 0)
    {
        yError ( "JoystickCheck: no joysticks found.\n");
        return 0;
    }

    // open the first joystick
    fprintf ( stderr, "JoystickCheck: (%d) joystick(s) found.\n",joystick_num);
    int joy_id = 0;
    SDL_Joystick* joy1 = SDL_JoystickOpen ( joy_id );
    if ( joy1 == NULL )
    {
        yError ( "Could not open default joystick.\n" );
        return 0;
    }
    int numAxes    = SDL_JoystickNumAxes    ( joy1 );
    int numBalls   = SDL_JoystickNumBalls   ( joy1 );
    int numHats    = SDL_JoystickNumHats    ( joy1 );
    int numButtons = SDL_JoystickNumButtons ( joy1 );
    bool first =true;
    bool active=false;
    double* firstAxes    = new double [numAxes];
    double* firstBalls   = new double [numBalls];
    double* firstHats    = new double [numHats];
    double* firstButtons = new double [numButtons];
    double* valAxes    = new double [numAxes];
    double* valBalls   = new double [numBalls];
    double* valHats    = new double [numHats];
    double* valButtons = new double [numButtons];

    //check for user activity on the opened joystick
    for (int trial=0; trial < 1000; trial++ )
    {
        yarp::os::Time::delay(0.010);
        SDL_JoystickUpdate();

        if (first==true)
        {
            for (int i=0; i < numAxes; i++)
                firstAxes[i] = (double)SDL_JoystickGetAxis(joy1, i);
            //for (int i=0; i < numBalls; i++)
            //    firstBalls[i] = (double)SDL_JoystickGetBall(joy1, i, dx, dy);
            for (int i=0; i < numHats; i++)
                firstHats[i] = (double)SDL_JoystickGetHat(joy1, i);
            for (int i=0; i < numButtons; i++)
                firstButtons[i] = (double)SDL_JoystickGetButton(joy1, i);
        }
        else
        {
            for (int i=0; i < numAxes; i++)
            {
                valAxes[i] = (double)SDL_JoystickGetAxis(joy1, i);
                if (fabs(valAxes[i]-firstAxes[i])>25000)
                {
                    active=true;
                    break;
                }
            }
            //for (int i=0; i < numBalls; i++)
            //    valAxes[i] = (double)SDL_JoystickGetBall(joy1, i, dx, dy);
            for (int i=0; i < numHats; i++)
            {
                valHats[i] = (double)SDL_JoystickGetHat(joy1, i);
                if (fabs(valHats[i]-firstHats[i])>100)
                {
                    active=true;
                    break;
                }
            }
            for (int i=0; i < numButtons; i++)
            {
                valButtons[i] = (double)SDL_JoystickGetButton(joy1, i);
                if (fabs(valButtons[i]-firstButtons[i])>0.5)
                {
                    active=true;
                    break;
                }
            }
        }
        first = false;
        if (active==true)
        {
            yInfo ( "joysticks activity detected.\n" );
            return 101;
        }
    }

    yInfo ( "no joysticks are currently used.\n" );
    return 0;
}
