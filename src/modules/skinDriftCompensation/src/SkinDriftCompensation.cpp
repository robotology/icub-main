
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

#include <sstream>			// string stream
#include "iCub/skinDriftCompensation/SkinDriftCompensation.h"
 
using namespace iCub::skinDriftCompensation;

// module default values
const int SkinDriftCompensation::MIN_BASELINE_DEFAULT = 3;
const int SkinDriftCompensation::ADD_THRESHOLD_DEFAULT = 2;
const int SkinDriftCompensation::PERIOD_DEFAULT = 50;
const float SkinDriftCompensation::SMOOTH_FACTOR_DEFAULT = 0.5;
const float SkinDriftCompensation::COMPENSATION_GAIN_DEFAULT = 0.2f;
const string SkinDriftCompensation::MODULE_NAME_DEFAULT = "skinDriftCompensation";
const string SkinDriftCompensation::ROBOT_NAME_DEFAULT = "icub";
//const string SkinDriftCompensation::HAND_DEFAULT = "right";
const string SkinDriftCompensation::ZERO_UP_RAW_DATA_DEFAULT = "notFound";
const string SkinDriftCompensation::RPC_PORT_DEFAULT = "/rpc";

// the order of the command in this list MUST correspond to the order of the enum SkinDriftCompensation::SkinDriftCompCommand
const string SkinDriftCompensation::COMMAND_LIST[]  = {
	"forbid calibration",	"allow calibration",	"force calibration", 
	"get percentile",		"set binarization",		"get binarization", 
	"set smooth filter",	"get smooth filter",	"set smooth factor",	
	"get smooth factor",	"set threshold",        "get threshold", 
    "set gain",             "get gain",
    "is calibrating",       "get info",		        "help",					
    "quit"};

// the order in COMMAND_DESC must correspond to the order in COMMAND_LIST
const string SkinDriftCompensation::COMMAND_DESC[]  = {
	"forbid the automatic calibration (by default it is already forbidden)", 
	"allow the automatic calibration", 
	"force the calibration (for 5 sec no touch should happens)", 
	"get the 95 percentile of the tactile data", 
	"enable or disable the binarization filter (255 touch, 0 no touch)",
	"get the binarization filter state (on, off)",
	"enable or disable the smooth filter",
	"get the smooth filter state (on, off)",
	"set the value of the smooth factor (in [0,1])",
	"get the smooth factor value",
    "set the safety threshold that is added to the touch thresholds (int in [0, 254])",
    "get the safety threshold that is added to the touch threshold",
    "set the compensation gain", 
    "get the compensation gain",
	"tell whether the skin calibration is in progress",
    "get information about the module",
	"get this list", 
	"quit the module"};

bool SkinDriftCompensation::configure(yarp::os::ResourceFinder &rf)
{    
	/* Process all parameters from both command-line and .ini file */

	/* get the module name which will form the stem of all module port names */
	moduleName			= rf.check("name", Value(MODULE_NAME_DEFAULT.c_str()), "module name (string)").asString();
	robotName			= rf.check("robot", Value(ROBOT_NAME_DEFAULT.c_str()), "name of the robot (string)").asString();
	/* before continuing, set the module name before getting any other parameters, 
	* specifically the port names which are dependent on the module name*/
	setName(moduleName.c_str());

	/* get some other values from the configuration file */
	float minBaseline		= (float)rf.check("minBaseline", Value(MIN_BASELINE_DEFAULT), 
	   "If the baseline reaches this value then, if allowed, a calibration is executed (float in [0,255])").asDouble();
	int period				= (int)rf.check("period", Value(PERIOD_DEFAULT), 
	   "Period of the thread in ms (positive int)").asInt();
	bool binarization		= rf.check("binarization");
	bool smoothFilter		= rf.check("smoothFilter");
	float smoothFactor		= (float)rf.check("smoothFactor", Value(SMOOTH_FACTOR_DEFAULT), 
	   "Determine the smoothing intensity (float in [0,1])").asDouble();
	float compGain			= (float)rf.check("compensationGain", Value(COMPENSATION_GAIN_DEFAULT), 
	   "Gain of the compensation algorithm (float)").asDouble();
	int addThreshold		= (int)rf.check("addThreshold", Value(ADD_THRESHOLD_DEFAULT), 
	   "Value added to all the touch thresholds (positive int)").asInt();
	
	bool zeroUpRawData = true;
	string zeroUpRawDataStr		= rf.check("zeroUpRawData", Value(ZERO_UP_RAW_DATA_DEFAULT.c_str()), 
	   "if true the raw data are considered from zero up, otherwise from 255 down (string)").asString().c_str();
	if(zeroUpRawDataStr.compare("true")==0){
		zeroUpRawData = true;
	}else if(zeroUpRawDataStr.compare("false")==0){
		zeroUpRawData = false;
	}

	/*
	* attach a port of the same name as the module (prefixed with a /) to the module
	* so that messages received from the port are redirected to the respond method
	*/
	handlerPortName = "/";
	handlerPortName += getName(rf.check("handlerPort", Value(RPC_PORT_DEFAULT.c_str())).asString());
	if (!handlerPort.open(handlerPortName.c_str())) {
		cout << getName() << ": Unable to open port " << handlerPortName << endl;  
		return false;
	}
	attach(handlerPort);                  // attach to port	


	/* create the thread and pass pointers to the module parameters */
	myThread = new CompensationThread(moduleName, &rf, robotName, compGain, addThreshold, minBaseline, 
		zeroUpRawData, period, binarization, smoothFilter, smoothFactor);
	/* now start the thread to do the work */
	myThread->start(); // this calls threadInit() and it if returns true, it then calls run()

	return true ;      // let the RFModule know everything went well
					  // so that it will then run the module
}


bool SkinDriftCompensation::interruptModule()
{
	handlerPort.interrupt();

	return true;
}


bool SkinDriftCompensation::close()
{
	/* stop the thread */
	if(myThread){
		myThread->stop();
        delete myThread;
    }    	
	handlerPort.close();   

	return true;
}


bool SkinDriftCompensation::respond(const Bottle& command, Bottle& reply) 
{
	stringstream temp;
	string helpMessage =  string(getName().c_str()) + " commands are: ";
	reply.clear();

	SkinDriftCompCommand com;
	if(!identifyCommand(command, com)){
		reply.addString("Unknown command. Input 'help' to get a list of the available commands.");
		return true;
	}

	switch( com ){
		case quit:
			reply.addString("quitting");
			return false;

		case help:
			reply.addString("many");				// print every string added to the bottle on a new line
			reply.addString(helpMessage.c_str());
			for(unsigned int i=0; i< COMMANDS_COUNT; i++){
				reply.addString( ("- "+COMMAND_LIST[i]+": "+COMMAND_DESC[i]).c_str() );
			}
			return true;

		case forbid_calibration:
            reply.addString("Command deprecated!");
			return true;

		case allow_calibration:
			reply.addString("Command deprecated!");
			return true;

		case force_calibration:
			myThread->forceCalibration();
			break;

		case get_percentile:
			{
			Vector touchThreshold = myThread->getTouchThreshold();
			for(int i=0; i< touchThreshold.size(); i++) 
				reply.addDouble(touchThreshold[i]);
			return true;
			}

		case set_binarization:
			{
			if(command.size()<3){
				reply.addString("Binarization state missing! Specify either on or off.");
				return true;
			}
			string value = command.get(2).asString().c_str();
			if(value.compare("on")==0)
				myThread->setBinarization(true);
			else if(value.compare("off")==0)
				myThread->setBinarization(false);
			else{
				reply.addString("Value not recognized.");
				return true;
			}
			break;
			}

		case get_binarization:
			if(myThread->getBinarization())
				reply.addString("on");
			else
				reply.addString("off");
			return true;

		case set_smooth_filter:
			{
			if(command.size()<4){
				reply.addString("Smooth filter state missing! Specify either on or off.");
				return true;
			}
			string value = command.get(3).asString().c_str();
			if(value.compare("on")==0)
				myThread->setSmoothFilter(true);
			else if(value.compare("off")==0)
				myThread->setSmoothFilter(false);
			else{
				reply.addString("Value not recognized.");
				return true;
			}
			break;
			}

		case get_smooth_filter:
			if(myThread->getSmoothFilter())
				reply.addString("on");
			else
				reply.addString("off");
			return true;

		case set_smooth_factor:
			{
			if(command.size()<4 || (!command.get(3).isDouble() && !command.get(3).isInt())){
				reply.addString("New smooth factor value missing or not a number! Smooth factor not updated.");
				return true;
			}

			stringstream temp;
			if(myThread->setSmoothFactor((float)command.get(3).asDouble())){				
				temp<< "New smooth factor set: "<< command.get(3).asDouble();				
			}
			else{
				temp<< "ERROR in setting new smooth factor: "<< command.get(3).asDouble();
			}
			reply.addString( temp.str().c_str());
			return true;
			}

		case get_smooth_factor:
			reply.addDouble(myThread->getSmoothFactor());
			return true;

        case set_threshold:
            {
			if(command.size()<3 || (!command.get(2).isInt())){
				reply.addString("New threshold value missing or not an integer! Threshold not updated.");
				return true;
			}

			stringstream temp;
			if(myThread->setAddThreshold(command.get(2).asInt())){				
				temp<< "New threshold set: "<< command.get(2).asInt();				
			}
			else{
				temp<< "ERROR in setting new threshold: "<< command.get(2).asInt();
			}
			reply.addString( temp.str().c_str());
			return true;
			}

        case get_threshold:
            reply.addInt(myThread->getAddThreshold());
            return true;

        case set_gain:
            {
			if(command.size()<3 || (!command.get(2).isDouble())){
				reply.addString("New gain value missing or not a double! Gain not updated.");
				return true;
			}

			stringstream temp;
			if(myThread->setCompensationGain(command.get(2).asDouble())){	
				temp<< "New gain set: "<< command.get(2).asDouble();				
			}
			else{
				temp<< "ERROR in setting new gain: "<< command.get(2).asDouble();
			}
			reply.addString( temp.str().c_str());
			return true;
			}

        case get_gain:
            reply.addDouble(myThread->getCompensationGain());
            return true;

		case is_calibrating:
			if(myThread->isCalibrating())
				reply.addString("yes");
			else
				reply.addString("no");
			return true;

        case get_info:
            reply.append(myThread->getInfo());
            return true;

		default:
			reply.addString("ERROR: This command is known but it is not managed in the code.");
			return true;
	}

	reply.addString( (COMMAND_LIST[com]+" command received.").c_str());

	return true;	
}


/**
  * Identify the command in the bottle and return the correspondent enum value.
  */
bool SkinDriftCompensation::identifyCommand(Bottle commandBot, SkinDriftCompensation::SkinDriftCompCommand &com){
	for(unsigned int i=0; i<COMMANDS_COUNT; i++){
		stringstream stream(COMMAND_LIST[i]);
		string word;
		int j=0;
		bool found = true;

		while(stream>>word){
			if (commandBot.get(j).asString() != word.c_str()){
				found=false;
				break;
			}
			j++;
		}
		if(found){
			com = (SkinDriftCompCommand)i;
			return true;
		}
	}

	return false;
}

bool SkinDriftCompensation::updateModule(){ return true;}
double SkinDriftCompensation::getPeriod(){ return 0.1;}

