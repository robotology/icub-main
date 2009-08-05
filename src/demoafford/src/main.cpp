/**
 * @ingroup icub_module
 *
 * \defgroup icub_demoAfford demoAfford
 *
 * This module is part of the application \ref demoaff "demoaff"
 *
 *\section intro_sec Description 
 *
 * This module imitates an observed effect on a particular object. This is done using  a predefined Bayesian network that contains the relations between the robot available actions, the object features and the effects of each action upon objects. The robot is presented with two alternative objects it has to select from. The current version uses preprogrammed actions and positions.
 *
 *\section lib_sec Libraries
 *
 *pnl. 
 *
 *
 *\section parameters_sec Parameters
 *
 * No parameters.
 *
 *\section portssa_sec Ports Accessed 
 *
 * Input ports\n
 * 
 *  /camshiftplus/all/o 
 *  /artracker/debout
 *
 * Output ports\n 
 *
 *  /camshiftplus/roi/i 
 *
 *\section portsc_sec Ports Created
 *
 *  Input ports\n
 * /demoAff/objectinfo
 * /demoAff/marks 
 *
 * Output ports\n
 *  /demoAff/synccamshift
 *
 *\section conf_file_sec Configuration Files
 *
 *
 * --file affordance Bayes net
 *  example: --file $ICUB_ROOT/conf/BNaffordances.txt
 * Examples of such files can be found at \in conf/BNaffordances.txt
 *
 *
 *\section tested_os_sec Tested OS
 *
 * This module has been tested on Linux. Windows portability issues may arise due to its dependency on the pnl library.
 *
 *\section example_sec Example Instantiation of the Module
 *
 * ./demoAfford  --file  $ICUB_ROOT/conf/BNaffordances.txt \n
 *
 * This file can be edited at \in src/demoAfford/main.cpp 
 *
 *\authors Luis Montesano, Manuel Lopes
 *
 **/

// yarp
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>

// iCub
#include <iCub/demoAff.h>

using namespace std;
using namespace yarp::os;
//using namespace iCub::contrib;


int main(int argc, char *argv[]) {

    Network yarp;
    DemoAff module;
    module.setName("/demoAff"); // set default name of module
    return module.runModule(argc,argv);
}
