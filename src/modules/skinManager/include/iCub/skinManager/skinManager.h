 
/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
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
\defgroup icub_skinManager skinManager

This module reads the raw tactile sensor values, compensates for the (thermal) drift of the sensors (basically it is a high pass filter)
and writes the compensated values on output ports.
The module can manage many input ports at the same time (see parameter "inputPorts").
For each input port the compensated tactile data are written on the corresponding output port (see parameter "outputPorts").
Optionally, the module also can apply a smoothing filter (low pass filter) and/or a binarization filter to the data.
The \ref icub_skinManagerGui can be used to control and monitor an instance of the skinManager module.
If the 3d position of the tactile sensors is provided, then this module computes the skinContacts (see library skinDynLib),
 which can be used by \ref wholeBodyDynamics to compute the contact forces.

\section intro_sec Description
When launched the module executes the skin sensor calibration, assuming that the sensors are not in contact 
with anything during this phase. 
The calibration resets the taxel baselines, gathers the sensor data for 5 sec, computes the mean (i.e. the baseline) 
and the 95 percentile (i.e. the touch threshold) for every taxel.

After the calibration the module starts reading the raw data, computing the difference between the read values 
and the baseline, and outputs the results. 
If no touch is detected (i.e. the compensated values are under the touch threshold) then the baseline is updated
in order to follow the drift (if any).

The binarization filter is really simple.
Every taxel has a touch threshold, given by its 95% percentile plus a safety threshold (2 by default).
If the read value is greater than the corrisponding touch threshold the output is set to 100, otherwise to 0.
The binarization filter can be used for stressing the touch detection, especially in cases where the touch is very light.

The smoothing filter performs an exponential moving average in order to reduce the sensor noise.
The intensity of the filter can be tuned by setting the parameter alpha, also called "smoothing factor".
The smoothed output is a weighted average of the current input and the previous output:
y(t) = (1-alpha)*x(t) + alpha*y(t-1)

If the module is properly configured then it performs contact clustering, that is it determines how many contacts are
detected by the tactile sensors. When tactile sensors that are neighbors detect contact, they are clustered together.
When tactile sensors that are not neighbors detect contact, they are considered as independent contacts. The result of
this operation is a skinContactList (see skinDynLib) that is written on the output port "\moduleName\skin_events:o".
Tipically this port is connected to an input port of the module \ref wholeBodyDynamics which uses this data
to estimate the contact forces.

\section lib_sec Libraries
YARP.


\section parameters_sec Parameters

<b>Command-line Parameters</b> 

The following key-value pairs can be specified as command-line parameters by prefixing -- to the key 
(e.g. --from file.ini). The value part can be changed to suit your needs; the default values are shown below.
 - \c from \c [driftCompLeft.ini] \n 
   specifies the configuration file
 - \c context \c [skinGui/conf] \n
    specifies the sub-path from \c $ICUB_ROOT/app to the configuration file
 - \c name \c [skinManager] \n   
    specifies the name of the module (used to form the stem of module port names)  
 - \c robot \c [icub] \n          
    specifies the name of the robot (used only to distiguish the simulator and the real robot)

<b>Configuration File Parameters </b>

 The following key-value pairs can be specified as parameters in the configuration file 
 (they can also be specified as command-line parameters if you so wish). 
 The value part can be changed to suit your needs; the default values are shown below.
 - \c inputPorts \c [emptyList] \n
   list of the input ports from which the module has to read the tactile data.
   For each input port there has to be a corresponding output port specified in the "outputPorts" parameter.
 - \c outputPorts \c [emptyList] \n
   list of the output ports on which the module has to write the compensated tactile data.
   For each output port there has to be a corresponding input port specified in the "inputPorts" parameter.
 - \c period \c [20] \n
   period of the compensating thread expressed in ms.
 - \c minBaseline \c [3] \n  
   if the baseline of one sensor (at least) reaches this value, then a warning message is sent on the info output port.
 - \c zeroUpRawData \c [false] \n
   if true the input tactile data are considered from zero up, otherwise from 255 down
 - \c binarization \c [not active]\n
   if specified the output tactile data are binarized: 0 indicates no touch, whereas 100 indicates touch
 - \c smoothFilter \c [not active]\n
   if specified the output tactile data are filtered with an exponential moving average (where alpha is the smooth factor):
    \t- y(t) = (1-alpha)*x(t) + alpha*y(t-1)
 - \c smoothFactor \c [0.5] \n
   alpha value of the smoothing filter, in [0, 1] where 0 is no smoothing at all and 1 is the max smoothing possible.
.
An optional section called SKIN_EVENTS may be specified in the configuration file.
These are the parameters of this section:
 - \c bodyParts \c [emptyList] \n
    list of the body parts corresponding to the specified inputPorts (see common.h in skinDynLib for the definition of BodyPart).
    For each input port there has to be a body part, e.g. UNKNOWN_BODY_PART, HEAD, TORSO, LEFT_ARM, RIGHT_ARM, LEFT_LEG, RIGHT_LEG.
 - \c skinParts \c [emptyList] \n
    list of the skin part corresponding to the specified inputPorts (see skinDynLib for the definition of SkinPart).
    For each input port there has to be a skin part, e.g. UNKNOWN_SKIN_PART, HAND, FOREARM, UPPER_ARM.
 - \c linkList \c [emptyList] \n
    list of the link number corresponding to the specified inputPorts.
    For each input port there has to be a link number (2: upperArm link, 4: forearm link, 6: hand link).
 - \c taxelPositionFiles [emptyList] \n
    list of the files containing the 3d position and orientation of the tactile sensors for each input port.
    The files contain a line for each tactile sensor. In each line there are 6 values, the first three are
    the position, the last three are the orientation.
    For each input port there has to be a taxel position file (which may not exist though).
 - \c maxNeighborDist \c 0.015 \n
    maximum distance between two neighbor tactile sensors (in meters).
 

\section portsa_sec Ports Accessed
All the ports listed in the "inputPorts" parameter and the corresponding rpc ports.
For instance if in the "inputPorts" parameter it is specified the port
- /icub/skin/right_hand
.
then also the port
- /icub/skin/right_hand/rpc:i
.
will be accessed.


\section portsc_sec Ports Created
<b>Output ports </b>
- Every port specified in the "outputPorts" parameter: outputs a yarp::os::Vector containing the compensated tactile data.
- "/"+moduleName+"/monitor:o": \n 
    outputs a yarp::os::Bottle containing streaming information regarding the compensation status 
    (used to communicate with the \ref icub_skinManagerGui). The first value is the data frequency, while
    all the following ones represent the drift compensated so far for each taxel.
- "/"+moduleName+"/info:o": \n 
    outputs a yarp::os::Bottle containing occasional information regarding the compensation status 
    such as warning or error messages (used to communicate with the \ref icub_skinManagerGui). Possible messages may regard 
    an error in the sensor reading or an excessive drift of the baseline of a taxel.\n
- "/"+moduleName+"/skin_events:o": \n
    outputs a iCub::skinDynLib::skinContactList containing the list of contacts.

<b>Input ports</b>
- For each port specified in the "inputPorts" parameter a local port is created with the name
  "/"+moduleName+index+"/input", where "index" is an increasing counter starting from 0.
- "/"+moduleName+"/rpc:i": input port to control the module (alternatively the \ref icub_skinManagerGui can be used). 
    This port accepts a yarp::os::Bottle that contains one of these commands:
	- “calib”: force the sensor calibration
	- "get touch thr": return a yarp::os::Bottle containing the 95 percentile values of the tactile sensors
	- "set binarization": enable or disable the binarization (specifying the value on/off)
	- "get binarization": get the binarization filter state (on, off)
	- "set smooth filter": enable or disable the smooth filter (specifying the value on/off)
	- "get smooth filter": get the smooth filter state (on, off)
	- "set smooth factor": set the value of the smooth factor (in [0,1])
	- "get smooth factor": get the smooth factor value
	- "set threshold": set the safety threshold that is added to the touch thresholds (int in [0, 254])
	- "get threshold": get the safety threshold that is added to the touch thresholds (int in [0, 254])
	- "set gain": set the compensation gain
	- "get gain": get the compensation gain
	- "set contact gain": set the contact compensation gain
	- "get contact gain": get the contact compensation gain
	- "is calibrating": tell whether the skin calibration is in progress
	- "get pose": get taxel pose(s) with input params: body part, skin part, taxel index (if taxel index is not specified return all taxel positions),
	- "set pose": set taxel pose(s) with input params: body part, skin part, taxel index, pose(s) (if taxel index is not specified set all taxel positions),
	- "get info": get information about the module (module name, robot name, input ports, num of taxels)
	- "help": get a list of the commands accepted by this module
	- "quit": quit the module
    .
	All the commands accepted by this module are defined in rpcSkinManager.h .

\section in_files_sec Input Data Files
None.


\section out_data_sec Output Data Files
None.
 

\section conf_file_sec Configuration Files
None.


\section tested_os_sec Tested OS
Linux and Windows.


\section example_sec Example Instantiation of the Module
skinManager --context skinGui/conf --from driftCompRight.ini


\author Andrea Del Prete (andrea.delprete@iit.it), Alexander Schmitz

Copyright (C) 2010 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at ICUB_HOME/main/src/modules/skinManager/include/iCub/skinManager/skinManager.h.
**/


#ifndef __ICUB_skinManager_H__
#define __ICUB_skinManager_H__

#include <iostream>
#include <string>

#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>

#include "iCub/skinManager/compensationThread.h"
#include "iCub/skinDynLib/rpcSkinManager.h"
 
using namespace std;
using namespace yarp::os; 
using namespace yarp::sig;

namespace iCub{

namespace skinManager{

class skinManager:public RFModule
{
public:
	bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
	bool interruptModule();                       // interrupt, e.g., the ports 
	bool close();                                 // close and shut down the module
	bool respond(const Bottle& command, Bottle& reply);
	double getPeriod(); 
	bool updateModule();

private:
	// module default values
	static const bool CALIBRATION_ALLOWED_DEFAULT;
	static const int MIN_BASELINE_DEFAULT;
	static const int PERIOD_DEFAULT;
	static const int ADD_THRESHOLD_DEFAULT;
	static const float SMOOTH_FACTOR_DEFAULT;
	static const float COMPENSATION_GAIN_DEFAULT;
    static const float CONTACT_COMPENSATION_GAIN_DEFAULT;
	static const string MODULE_NAME_DEFAULT;
	static const string ROBOT_NAME_DEFAULT;
	static const string ZERO_UP_RAW_DATA_DEFAULT;
	static const string RPC_PORT_DEFAULT;

	/* module parameters */
	string moduleName;
	string robotName;

	/* ports */
	Port handlerPort;									// a port to handle messages

	/* pointer to a new thread to be created and started in configure() and stopped in close() */
	CompensationThread *myThread;

    void addToBottle(Bottle& b, const Vector& v);
    void addToBottle(Bottle& b, const vector<Vector>& v);
    bool bottleToVector(const yarp::os::Bottle& b, yarp::sig::Vector& v);
	bool identifyCommand(Bottle commandBot, SkinManagerCommand &com, Bottle& params);

};

} //namespace iCub

} //namespace skinManager

#endif // __ICUB_skinManager_H__
//empty line to make gcc happy

