
// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2008 Sarah Degallier Ludovic Righetti BIRG EPFL Lausanne
 * RobotCub Consortium, European Commission FP6 Project IST-004370
 * email:   sarah.degallier@robotcub.org
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
 * @ingroup icub_module
 *
 * \defgroup icub_DrumManager DrumManager
 *
 *This module is part of the application \ref icub_drummingEPFL "drummingEPFL"
 *
 *\section intro_sec Description 
 *
 * This module transforms a score into the appropriate parameters for the dynamical systems generating the trajectories (\in \ref icub_DrumGenerator "DrumGenerator"). It sends the parameters at timing corresponding to the beat of each of the part.If you want to  use the whole drumming application, please refer to \ref icub_drummingEPFL "drummingEPFL". 
 *
 *\section lib_sec Libraries
 *
 *No external libraries
 *
 *
 *\section parameters_sec Parameters
 *
 * --config-path $ICUB_ROOT/app/DrummingEpfl/conf 
 *
 *\section portssa_sec Ports Accessed 
 *
 * Input ports\n
 * <ul>
 * <li> for each active \a part (i.e left_arm, right_arm, left_leg, right_leg, head) of the robot: /part/parameters/in (created by the \ref icub_DrumGenerator "DrumGenerator" module)
 *<li> for each active \a part of the robot: /part/sound/in (created by the \ref icub_DrumGenerator "DrumGenerator" module)
 *</ul>
 *
 * Output ports\n
 * <ul>
 * <li> for each active \a part of the robot: /part/check_motion/out (created by the \ref icub_DrumGenerator "DrumGenerator" module)
 *<li> a port /midiDrum/server/out (created by the \ref icub_midiDrum "midiDrum" module)
 * <li> for each active \a part of the robot: /part/score/out (created by the \ref icub_guiDemo "guiDemo" module)
* <li> for each active \a part of the robot: /part/phase/out (created by the \ref icub_guiDemo "guiDemo" module)
 * <li> a port /interactive/out (created by the \ref icub_guiDemo "guiDemo" module)
 * </ul>
 *  
 *
 *\section portsc_sec Ports Created
 *
 * Input ports\n
 * <ul>
 *<li> For each active \a part of the robot, a corresponding port /part/check_motion/in receives two integers:
 * <ul>
 * <li> the current beat of the \a part 
 * <li> a value 0 or 1 to check if the motion is finished (not used yet);
 * </ul>
 * <li> For each \a part of the robot, a corresponding port /part/score/in receives a vector of integers corresponding to the target positions at the different time steps;
* <li> For each \a part of the robot, a corresponding port /part/phase/in receives a vector of doubles corresponding to the phase shift of each part at the different time steps;
 * <li> A port /interactive/in receives a vector corresponding to the frequency at the different time steps. \n
 *<li> A port /midiDrum/in receives information on the sound feedback, the third  value corresponds to the instrument and the fourth one to the velocity of the impact. 
 * </ul>
 * Output ports\n
 * <ul>
 * <li> For each \a part of the robot, a corresponding port /part/parameters/out sends 2*ndofs+1 doubles (where ndofs is the number of dofs of \a part)
 * <ul> 
 * <li> the amplitude of the movement (1 if drumming; -5 if idle) and the target position for each dof
 * <li> the phase shift of the \a part (for the the legs the phase shift is applied to the right one, the left one always stays synchronized with the clock)
 * </ul>
 *<li> For each \a part of the robot, a corresponding port /part/sound/out sends one integer (1 if feedback should be enabled, 0 otherwise)
 *</ul>
 *
 *\section conf_file_sec Configuration Files
 *
 * For each active part, this module requires:
 *<ul>
 *<li> partConfig.ini
 *<li> partTargets.ini
 *</ul>
 *
 * Examples of such files can be found at \in src/drummingEPFL/config/left_armConfig.ini and at \in src/drummingEPFL/config/left_armTargets.ini.  
 *
 *
 *\section tested_os_sec Tested OS
 *
 * This module has been tested on Linux and Windows. 
 *
 *\section example_sec Example Instantiation of the Module
 *
 * ./DrumManager --config-path $ICUB_ROOT/app/DrummingEpfl/conf/
 *
 * This file can be edited at \in src/drummingEPFL/DrumManager/main.cpp 
 *
 *\authors Sarah Degallier Ludovic Righetti
 *
 **/

#include "drum.h"
#undef main 

int main(int argc,char **argv) 
{
    Network yarp;
    
    drum *MyGMP;
    MyGMP = new drum();

    Property prop;
    prop.fromCommand(argc, argv);

    if (!prop.check("config-path"))
        {
            fprintf(stderr, "Please specify --config-path path to config files\n");
            return -1;
        }
    
    sprintf(MyGMP->pathToConfig, "%s", prop.find("config-path").asString().c_str());
    fprintf(stderr, "Using config files from %s\n",MyGMP->pathToConfig);

    //initialize
    ACE_OS::printf("connecting ports....\n");
    MyGMP->doConnect(); //connect ports
    ACE_OS::printf("...done\n getting parameters...\n");
    MyGMP->getConfig(); //get info from files
    ACE_OS::printf("...done\n");
    

    //run
    MyGMP->run();

    //close     
    ACE_OS::printf("Closing...\n");
    delete MyGMP;

    return 1;
}

 
