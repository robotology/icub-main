
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

