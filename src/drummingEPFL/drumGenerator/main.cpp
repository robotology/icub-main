/**
 * @ingroup icub_module
 *
 * \defgroup icub_DrumGenerator DrumGenerator
 *
 * This module is part of the application \ref icub_drummingEPFL "drummingEPFL"
 *
 *\section intro_sec Description 
 *
 * This module transforms a set of parameters for the dynamical systems into target trajectories for a specific interface of the robot (head, left_arm, right_arm, left_leg, right_leg). The command are then sent to the corresponding \ref icub_velocityControl "velocityControl" module. If you want to  use the whole drumming application, please refer to \ref icub_drummingEPFL "drummingEPFL".
 *
 *\section lib_sec Libraries
 *
 *No external libraries. 
 *
 *
 *\section parameters_sec Parameters
 *
 * For instance for the left arm:  \n
 * --part left_arm --file  $ICUB_ROOT/app/DrummingEpfl/conf/left_armConfig.ini \n
 * 
 * The same type of command can be used for right_arm, left_leg, right_leg and head. 
 *
 *\section portssa_sec Ports Accessed 
 *
 * Output ports\n
 * <ul>
 * <li> for each active \a part (i.e left_arm, right_arm, left_leg, right_leg, head) of the robot: /part/parameters/out (created by the \ref icub_DrumManager "DrumManager" module)
 *<li> for each active \a part of the robot: /part/sound/out (created by the \ref icub_DrumManager "DrumManager" module)
 *</ul>
 *  
 *
 *\section portsc_sec Ports Created
 *
 *  Input ports\n
 * <ul>
 * <li> For each \a part of the robot, a corresponding port /part/parameters/in receives from the \ref icub_DrumManager "DrumManager" 2*ndofs+1 doubles (where ndofs is the number of dofs of \a part)
 * <ul> 
 * <li> the amplitude of the movement (1 if drumming; -5 if idle) and the target position for each dof
 * <li> the phase shift of the \a part (for the the legs the phase shift is applied to the right one, the left one always stays synchronized with the clock)
 * </ul>
 *<li> For each \a part of the robot, a corresponding port /part/sound/in receives from the \icub_DrumManager "DrumManager" one integer (1 if feedback 
 *</ul>
 *
 * Output ports\n
 * <ul>
 *<li> For each active \a part of the robot, a corresponding port /part/check_motion/out sends to the \ref icub_DrumManager "DrumManager" two integers:
 * <ul>
 * <li> the current beat of the \a part 
 * <li> a value 0 or 1 to check if the motion is finished (not used yet);
 * </ul>
 *</ul>
 *
 *\section conf_file_sec Configuration Files
 *
 * For each active  \a part, this module requires:
 *<ul>
 *<li> partConfig.ini
 *</ul>
 *
 * Examples of such files can be found at \in src/drummingEPFL/config/left_armConfig.ini
 *
 *
 *\section tested_os_sec Tested OS
 *
 * This module has been tested on Linux and Windows. 
 *
 *\section example_sec Example Instantiation of the Module
 *
 * ./DrumGenerator  --part left_arm --file  $ICUB_ROOT/app/DrummingEpfl/conf/left_armConfig.ini \n
 *
 * This file can be edited at \in src/drummingEPFL/DrumGenerator/main.cpp 
 *
 *\authors Sarah Degallier Ludovic Righetti
 *
 **/



#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/Time.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Module.h>
#include "DrumGeneratorModule.h"

using namespace yarp::os;
using namespace yarp::dev;

int main(int argc, char *argv[])
{
    Network yarp;

    //create and run the Drum generator module
    DrumGeneratorModule mod;
  
    return mod.runModule(argc,argv);

}
