
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
const bool SkinDriftCompensation::CALIBRATION_ALLOWED_DEFAULT = false;
const int SkinDriftCompensation::MIN_BASELINE_DEFAULT = 3;
const int SkinDriftCompensation::PERIOD_DEFAULT = 50;
const string SkinDriftCompensation::MODULE_NAME_DEFAULT = "skinDriftCompensation";
const string SkinDriftCompensation::ROBOT_NAME_DEFAULT = "icub";
const string SkinDriftCompensation::HAND_DEFAULT = "right";
const string SkinDriftCompensation::ZERO_UP_RAW_DATA_DEFAULT = "false";
const string SkinDriftCompensation::RPC_PORT_DEFAULT = "/rpc";

// the order of the command in this list MUST correspond to the order of the enum SkinDriftCompensation::SkinDriftCompCommand
const string SkinDriftCompensation::COMMAND_LIST[]  = {
	"forbid calibration",	"allow calibration",	"force calibration", 
	"get percentile",		"help",					"quit"};

// the order in COMMAND_DESC must correspond to the order in COMMAND_LIST
const string SkinDriftCompensation::COMMAND_DESC[]  = {
	"forbid the automatic calibration (by default it is already forbidden)", 
	"allow the automatic calibration", 
	"force the calibration (for 5 sec no touch should happens)", 
	"get the 95 percentile of the tactile data", 
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

	bool rightHand;
	string hand			= rf.check("hand", Value(HAND_DEFAULT.c_str()), "Hand to take as reference (string)").asString().c_str();
	if(hand.compare("right")==0){
		rightHand = true;
	}else if(hand.compare("left")==0){
		rightHand = false;
	}else{
		return false;
	}

	/* get some other values from the configuration file */
	float minBaseline		= (float)rf.check("minBaseline", Value(MIN_BASELINE_DEFAULT), 
	   "If the baseline reaches this value then, if allowed, a calibration is executed (float in [0,255])").asDouble();
	int period				= (int)rf.check("period", Value(PERIOD_DEFAULT), 
	   "Period of the thread in ms (positive int)").asInt();
	
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
	calibrationAllowed = CALIBRATION_ALLOWED_DEFAULT;
	myThread = new CompensationThread(&rf, robotName, &minBaseline, &calibrationAllowed, 
		zeroUpRawData, rightHand, period);
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
	if(myThread)
		myThread->stop();

	//compensatedTactileDataPort.close();
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
			calibrationAllowed = false;
			break;

		case allow_calibration:
			calibrationAllowed = true;
			break;

		case force_calibration:
			myThread->forceCalibration();
			break;

		case get_percentile:
			{
			VectorOf<float> touchThreshold = myThread->getTouchThreshold();
			for(int i=0; i< touchThreshold.size(); i++) 
				reply.addDouble(touchThreshold[i]);
			return true;
			}

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

