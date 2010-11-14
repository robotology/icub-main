
/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Andrea Del Prete, Alexander Schmitz
 * email:   andrea.delprete@iit.it, alexander.schmitz@iit.it
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
*
@ingroup icub_module
\defgroup icub_skinDriftCompensation skinDriftCompensation

This module reads the raw tactile sensor values, compensates the drift of the sensors 
and writes the compensated values on its output port.


\section intro_sec Description
When launched the module executes the sensors calibration (composed by a "big" calibration and a "small" calibration), 
assuming that the sensors are not in contact with anything during this phase. 
The big calibration is performed by sending a message to the icubInterface. 
The small calibration gathers the sensors data for 5 sec, computes the mean (called baseline) and the 95 percentile 
(that will be used as touch threshold for the raw data) for every taxel.
After the calibration the module starts reading the raw data, computing the difference between the read values 
and the baseline, and outputs the results. 
If no touch is detected (i.e. the compensated values are under the touch threshold) then the baseline is updated.
If the automatic calibration is allowed, when the touch threshold almost reaches one of the two limits 
(either 0 or 255), then the big and small calibrations are executed.
By default the automatic calibration is forbidden.


\section lib_sec Libraries
YARP.


\section parameters_sec Parameters

<b>Command-line Parameters</b> 

The following key-value pairs can be specified as command-line parameters by prefixing -- to the key 
(e.g. --from file.ini. The value part can be changed to suit your needs; the default values are shown below.
 - \c from \c skinDriftCompensationLeft.ini \n 
   specifies the configuration file
 - \c context \c graspingDemo/conf \n
    specifies the sub-path from \c $ICUB_ROOT/app to the configuration file
 - \c name \c skinDriftCompensation \n   
    specifies the name of the module (used to form the stem of module port names)  
 - \c robot \c icub \n          
    specifies the name of the robot (used to form the root of robot port names)

<b>Configuration File Parameters </b>
 The following key-value pairs can be specified as parameters in the configuration file 
 (they can also be specified as command-line parameters if you so wish). 
 The value part can be changed to suit your needs; the default values are shown below.
 - \c hand \c right \n    
   specifies which hand sensors has to be read
 - \c minBaseline \c 3 \n  
   if the baseline of one sensor (at least) reaches this value, then the calibration is executed (if allowed)
 - \c zeroUpRawData \c false \n
   if true the raw data are considered from zero up, otherwise from 255 down
 

\section portsa_sec Ports Accessed
- /icub/skin/right_hand  or  /icub/skin/left_hand
- /icub/skin/right_hand/rpc:i  or  /icub/skin/left_hand/rpc:i


\section portsc_sec Ports Created
<b>Output ports </b>
- /icub/skin/left_hand_comp  or  /icub/skin/right_hand_comp: 
	yarp::os::Vector output port streaming the compensated tactile data

<b>Input ports: </b>
All the port names listed below will be prefixed by \c /moduleName or whatever else is specified by the name parameter.\n
- /skinComp/right  or  /skinComp/left :
	port used by the IAnalogSensor interface for connecting with the sensors for reading the raw sensor data 
	and sending calibration signals to the microprocessor
- /rpc:i: input ports to control the module, accepts a yarp::os::Bottle which contains one of these commands:
	- “forbid calibration”: prevent the module from executing the automatic sensor calibration
	- “allow calibration”: enable the automatic sensor calibration
	- “force calibration”: force the sensor calibration
	- "get percentile": return a yarp::os::Bottle containing the 95 percentile values of the tactile sensors
	- "help": get a list of the commands accepted by this module
	- "quit": quit the module


\section in_files_sec Input Data Files
None.


\section out_data_sec Output Data Files
None.
 

\section conf_file_sec Configuration Files
None.


\section tested_os_sec Tested OS
Linux and Windows.


\section example_sec Example Instantiation of the Module
skinDriftCompensation --context graspingDemo/conf --from skinDriftCompensationRight.ini


\author Andrea Del Prete, Alexander Schmitz

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at ICUB_HOME/main/src/modules/skinDriftCompensation/include/iCub/skinDriftCompensation/SkinDriftCompensation.h.
**/


#ifndef __ICUB_SKINDRIFTCOMPENSATION_H__
#define __ICUB_SKINDRIFTCOMPENSATION_H__

#include <iostream>
#include <string>

#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>

#include "iCub/skinDriftCompensation/CompensationThread.h"
 
using namespace std;
using namespace yarp::os; 
using namespace yarp::sig;

namespace iCub{

namespace skinDriftCompensation{

class SkinDriftCompensation:public RFModule
{
public:

	// the last element of the enum (COMMANDS_COUNT) represents the total number of commands accepted by this module
	typedef enum { forbid_calibration, allow_calibration, force_calibration, get_percentile, 
		help, quit, COMMANDS_COUNT} SkinDriftCompCommand;
   
	bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
	bool interruptModule();                       // interrupt, e.g., the ports 
	bool close();                                 // close and shut down the module
	bool respond(const Bottle& command, Bottle& reply);
	double getPeriod(); 
	bool updateModule();

private:

	// module constants
	static const string COMMAND_LIST[];						// list of commands received through the rpc port
	static const string COMMAND_DESC[];						// descriptions of the commands

	/* module parameters */
	string moduleName;
	string robotName;

	// names of the ports
	string compensatedTactileDataPortName;
	string handlerPortName;

	/* class variables */
	BufferedPort<Vector> compensatedTactileDataPort;	
	Port handlerPort;									// a port to handle messages

	bool calibrationAllowed;								// if false the thread is not allowed to run the calibration
	bool forceCalibration;								// if true a calibration is executed as soon as possible, 
														// after that the variable is set to false

	/* pointer to a new thread to be created and started in configure() and stopped in close() */
	CompensationThread *myThread;

	bool identifyCommand(Bottle commandBot, SkinDriftCompCommand &com);

};

} //namespace iCub

} //namespace skinDriftCompensation

#endif // __ICUB_SKINDRIFTCOMPENSATION_H__
//empty line to make gcc happy

