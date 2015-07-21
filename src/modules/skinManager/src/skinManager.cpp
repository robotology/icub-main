
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
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include "iCub/skinManager/skinManager.h"
 
using std::string;
using std::vector;

using yarp::os::Bottle;
using yarp::sig::Vector;

using namespace iCub::skinManager;

// module default values
const int skinManager::MIN_BASELINE_DEFAULT = 3;
const int skinManager::ADD_THRESHOLD_DEFAULT = 2;
const int skinManager::PERIOD_DEFAULT = 50;
const bool skinManager::SMOOTH_FILTER_DEFAULT = false;
const float skinManager::SMOOTH_FACTOR_DEFAULT = 0.5;
const float skinManager::COMPENSATION_GAIN_DEFAULT = 0.2f;
const float skinManager::CONTACT_COMPENSATION_GAIN_DEFAULT = 0.0f;
const string skinManager::MODULE_NAME_DEFAULT = "skinManager";
const string skinManager::ROBOT_NAME_DEFAULT = "icub";
const bool skinManager::ZERO_UP_RAW_DATA_DEFAULT = false;
const bool skinManager::BINARIZATION_DEFAULT = false;
const string skinManager::RPC_PORT_DEFAULT = "/rpc";

bool skinManager::configure(yarp::os::ResourceFinder &rf) {    
    /* Process all parameters from both command-line and .ini file */

    /* get the module name which will form the stem of all module port names */
    moduleName			= rf.check("name", Value(MODULE_NAME_DEFAULT.c_str()), "module name (string)").asString();
    robotName			= rf.check("robot", Value(ROBOT_NAME_DEFAULT.c_str()), "name of the robot (string)").asString();
    /* before continuing, set the module name before getting any other parameters, 
    * specifically the port names which are dependent on the module name*/
    setName(moduleName.c_str());

    /* get some other values from the configuration file */
    int period				= (int)rf.check("period", Value(PERIOD_DEFAULT), 
       "Period of the thread in ms (positive int)").asInt();
    float minBaseline		= (float)rf.check("minBaseline", Value(MIN_BASELINE_DEFAULT), 
       "If the baseline reaches this value then, if allowed, a calibration is executed (float in [0,255])").asDouble();
    float compGain			= (float)rf.check("compensationGain", Value(COMPENSATION_GAIN_DEFAULT), 
       "Gain of the compensation algorithm (float)").asDouble();
    float contCompGain		= (float)rf.check("contactCompensationGain", Value(CONTACT_COMPENSATION_GAIN_DEFAULT), 
       "Gain of the compensation algorithm during contact (float)").asDouble();
    int addThreshold		= (int)rf.check("addThreshold", Value(ADD_THRESHOLD_DEFAULT), 
       "Value added to all the touch thresholds (positive int)").asInt();
    
    bool zeroUpRawData = rf.check("zeroUpRawData", ZERO_UP_RAW_DATA_DEFAULT, 
        "if true the raw data are considered from zero up, otherwise from 255 down (bool)").asBool();
    bool smoothFilter = rf.check("smoothFilter", SMOOTH_FILTER_DEFAULT,
            "if true then the smoothing filter is active (bool)").asBool();
    float smoothFactor		= (float) rf.check("smoothFactor", Value(SMOOTH_FACTOR_DEFAULT), 
       "Determine the smoothing intensity (float in [0,1])").asDouble();
    bool binarization = rf.check("binarization", BINARIZATION_DEFAULT,
            "if true then the binarization is active (bool)").asBool();

    /*
    * attach a port of the same name as the module (prefixed with a /) to the module
    * so that messages received from the port are redirected to the respond method
    */
    string handlerPortName = "/";
    handlerPortName += getName(rf.check("handlerPort", Value(RPC_PORT_DEFAULT.c_str())).asString());
    if (!handlerPort.open(handlerPortName.c_str())) {
	    cout << getName() << ": Unable to open port " << handlerPortName << endl;  
	    return false;
    }
    attach(handlerPort);                  // attach to port	
    handlerPort.setRpcMode(true);


    /* ****** Threads                                            ****** */
    // Compensator thread
    /* create the thread and pass pointers to the module parameters */
    myThread = new CompensationThread(moduleName, &rf, robotName, compGain, contCompGain, addThreshold, minBaseline, 
	    zeroUpRawData, period, binarization, smoothFilter, smoothFactor);
    /* now start the thread to do the work */
    if(!myThread->start()) { // this calls threadInit() and it if returns true, it then calls run()
        yError() << "[SkinManager] Could not start the compensator thread.";
        return false;
    }

    // Skin diagnostics thread
    thSkinDiagnostics = new SkinDiagnosticsReadThread(20, rf);
    if (!thSkinDiagnostics->start()) {
        yError() << "[SkinManager] Could not start the skin diagnostics thread.";
        return false;
    }
    return true;
}


bool skinManager::interruptModule()
{
	handlerPort.interrupt();

	return true;
}


bool skinManager::close()
{
    /* stop the threads */
    if(myThread) {
        myThread->stop();
        delete myThread;
    }
    if (thSkinDiagnostics) {
        thSkinDiagnostics->stop();
        delete thSkinDiagnostics;
    }

    handlerPort.close();   

    return true;
}


bool skinManager::respond(const Bottle& command, Bottle& reply) {
	stringstream temp;
	reply.clear();

	SkinManagerCommand com;
    Bottle params;
    if(command.get(0).isInt()){
        // if first value is int then it is the id of the command
        com = (SkinManagerCommand)command.get(0).asInt();
        yInfo("[rpc] Command received: %s", SkinManagerCommandList[com].c_str());
        params = command.tail();
    }
	else if(!identifyCommand(command, com, params)){
		reply.addString("Unknown command. Input 'help' to get a list of the available commands.");
		return true;
	}

	switch( com ){
		case quit:
			reply.addString("quitting");
			return false;

		case help:
            reply.addVocab(Vocab::encode("many"));				// print every string added to the bottle on a new line
			reply.addString((string(getName().c_str()) + " commands are: ").c_str());
			for(unsigned int i=0; i< SkinManagerCommandSize; i++){
				reply.addString( ("- "+SkinManagerCommandList[i]+": "+SkinManagerCommandDesc[i]).c_str() );
			}
			return true;

		case calibrate:
			myThread->calibrate();
			break;

		case get_touch_thr:
			{
			Vector touchThreshold = myThread->getTouchThreshold();
			for(size_t i=0; i< touchThreshold.size(); i++) 
				reply.addDouble(touchThreshold[i]);
			return true;
			}

		case set_binarization:
			{
			if(params.size()<1){
				reply.addString("Binarization state missing! Specify either on or off.");
				return true;
			}
			string value = params.get(0).asString().c_str();
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
			if(params.size()<1){
				reply.addString("Smooth filter state missing! Specify either on or off.");
				return true;
			}
			string value = params.get(0).asString().c_str();
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
			if(params.size()<1 || (!params.get(0).isDouble() && !params.get(0).isInt())){
				reply.addString("New smooth factor value missing or not a number! Smooth factor not updated.");
				return true;
			}

			stringstream temp;
			if(myThread->setSmoothFactor((float)params.get(0).asDouble())){
				temp<< "New smooth factor set: "<< params.get(0).asDouble();				
			}
			else{
				temp<< "ERROR in setting new smooth factor: "<< params.get(0).asDouble();
			}
			reply.addString( temp.str().c_str());
			return true;
			}

		case get_smooth_factor:
			reply.addDouble(myThread->getSmoothFactor());
			return true;

        case set_threshold:
            {
			if(params.size()<1 || (!params.get(0).isInt())){
				reply.addString("New threshold value missing or not an integer! Threshold not updated.");
				return true;
			}

			stringstream temp;
			if(myThread->setAddThreshold(params.get(0).asInt())){				
				temp<< "New threshold set: "<< params.get(0).asInt();				
			}
			else{
				temp<< "ERROR in setting new threshold: "<< params.get(0).asInt();
			}
			reply.addString( temp.str().c_str());
			return true;
			}

        case get_threshold:
            reply.addInt(myThread->getAddThreshold());
            return true;

        case set_gain:
            {
			if(params.size()<1 || (!params.get(0).isDouble())){
				reply.addString("New gain value missing or not a double! Gain not updated.");
				return true;
			}

			stringstream temp;
			if(myThread->setCompensationGain(params.get(0).asDouble())){	
				temp<< "New gain set: "<< params.get(0).asDouble();				
			}
			else{
				temp<< "ERROR in setting new gain: "<< params.get(0).asDouble();
			}
			reply.addString( temp.str().c_str());
			return true;
			}

        case get_gain:
            reply.addDouble(myThread->getCompensationGain());
            return true;

        case set_cont_gain:
            {
			if(params.size()<1 || (!params.get(0).isDouble())){
				reply.addString("New gain value missing or not a double! Contact gain not updated.");
				return true;
			}

			stringstream temp;
			if(myThread->setContactCompensationGain(params.get(0).asDouble())){	
				temp<< "New contact gain set: "<< params.get(0).asDouble();				
			}
			else{
				temp<< "ERROR in setting new contact gain: "<< params.get(0).asDouble();
			}
			reply.addString( temp.str().c_str());
			return true;
			}

        case get_cont_gain:
            reply.addDouble(myThread->getContactCompensationGain());
            return true;

        case get_max_neigh_dist:
            reply.addDouble(myThread->getMaxNeighborDistance());
            return true;

        case set_max_neigh_dist:
            {
			if(params.size()<1 || (!params.get(0).isDouble())){
				reply.addString("New max neighbor distance value missing or not a double! Not updated.");
				return true;
			}

			stringstream temp;
			if(myThread->setMaxNeighborDistance(params.get(0).asDouble())){	
				temp<< "New max neighbor distance set: "<< params.get(0).asDouble();				
			}
			else{
				temp<< "ERROR in setting new max neighbor distance: "<< params.get(0).asDouble();
			}
			reply.addString( temp.str().c_str());
			return true;
			}

        case get_skin_parts:
            {
                vector<SkinPart> spl = myThread->getSkinParts();
                for(vector<SkinPart>::const_iterator it=spl.begin(); it!=spl.end(); it++){
                    reply.addInt(*it);
                    reply.addString(SkinPart_s[*it].c_str());
                }
                return true;
            }

        case enable_skin_part:
            if(!(params.size()>0 && params.get(0).isInt())){
                reply.addString(("ERROR: SkinPart is not specified. Params read are: "+string(params.toString().c_str())).c_str());
                return true;
            }
            if(myThread->enableSkinPart((SkinPart)params.get(0).asInt()))
                reply.addString("SkinPart enabled");
            else
                reply.addString("SkinPart not found");
            return true;

        case disable_skin_part:
            if(!(params.size()>0 && params.get(0).isInt())){
                reply.addString(("ERROR: SkinPart is not specified. Params read are: "+string(params.toString().c_str())).c_str());
                return true;
            }
            if(myThread->disableSkinPart((SkinPart)params.get(0).asInt()))
                reply.addString("SkinPart disabled");
            else
                reply.addString("SkinPart not found");
            return true;

        case is_skin_enabled:
            if(!(params.size()>0 && params.get(0).isInt())){
                reply.addString(("ERROR: SkinPart is not specified. Params read are: "+string(params.toString().c_str())).c_str());
                return true;
            }
            if(myThread->isSkinEnabled((SkinPart)params.get(0).asInt()))
                reply.addString("yes");
            else
                reply.addString("no");
            return true;

		case is_calibrating:
			if(myThread->isCalibrating())
				reply.addString("yes");
			else
				reply.addString("no");
			return true;

        case get_pose:
            {
                if(!(params.size()>0 && params.get(0).isInt())){
                    reply.addString(("ERROR: SkinPart is not specified. Params read are: "+string(params.toString().c_str())).c_str());
                    return true;
                }
                SkinPart sp = (SkinPart) params.get(0).asInt();
                if(params.size()>1 && params.get(1).isInt()){
                    unsigned int taxelId = params.get(1).asInt();
                    Vector res = myThread->getTaxelPose(sp, taxelId);
                    if(res.size()>0)
                        addToBottle(reply, res);
                    else
                        reply.addString("No poses for the specified skin part");
                }
                else{
                    vector<Vector> res = myThread->getTaxelPoses(sp);
                    addToBottle(reply, res);
                }
                return true;
            }

        case set_pose:
            {
                if(!(params.size()>6 && params.get(0).isInt() )){
                    reply.addString(("ERROR: SkinPart is not specified. Params read are: "+string(params.toString().c_str())).c_str());
                    return true;
                }
                SkinPart sp = (SkinPart) params.get(0).asInt();
                if(params.get(1).isInt()){
                    unsigned int taxelId = params.get(1).asInt();
                    Vector pose;
                    if(!bottleToVector(params.tail().tail(), pose)){
                        reply.addString("ERROR while reading the taxel pose");
                        return true;
                    }
                    if(myThread->setTaxelPose(sp, taxelId, pose))
                        reply.addInt(skin_manager_ok);
                    else{
                        reply.addInt(skin_manager_error);
                        reply.addString("ERROR: pose was not set");
                    }
                }
                else{
                    Vector poses;
                    if(!bottleToVector(params.tail(), poses)){
                        reply.addString("ERROR while reading the taxel poses");
                        return true;
                    }
                    if(myThread->setTaxelPoses(sp, poses))
                        reply.addInt(skin_manager_ok);
                    else{
                        reply.addInt(skin_manager_error);
                        reply.addString("ERROR: pose was not set");
                    }
                }
                return true;
            }
		case get_position:
            {
                if(!(params.size()>0 && params.get(0).isInt())){
                    reply.addString(("ERROR: SkinPart is not specified. Params read are: "+string(params.toString().c_str())).c_str());
                    return true;
                }
                SkinPart sp = (SkinPart) params.get(0).asInt();
                if(params.size()>1 && params.get(1).isInt()){
                    unsigned int taxelId = params.get(1).asInt();
                    Vector res = myThread->getTaxelPosition(sp, taxelId);
                    if(res.size()>0)
                        addToBottle(reply, res);
                    else
                        reply.addString("No position for the specified skin part");
                }
                else{
                    vector<Vector> res = myThread->getTaxelPositions(sp);
                    if(res.empty())
                        reply.addString("Specified skin part has not been found.");
                    else
                        addToBottle(reply, res);
                }
                return true;
            }
		case set_position:
			{
                if(!(params.size()>3 && params.get(0).isInt() )){
                    reply.addString(("ERROR: SkinPart is not specified. Params read are: "+string(params.toString().c_str())).c_str());
                    return true;
                }
                SkinPart sp = (SkinPart) params.get(0).asInt();
                if(params.get(1).isInt()){
                    unsigned int taxelId = params.get(1).asInt();
                    Vector pose;
                    if(!bottleToVector(params.tail().tail(), pose)){
                        reply.addString("ERROR while reading the taxel position");
                        return true;
                    }
                    if(myThread->setTaxelPosition(sp, taxelId, pose))
                        reply.addInt(skin_manager_ok);
                    else{
                        reply.addInt(skin_manager_error);
                        reply.addString("ERROR: position was not set");
                    }
                }
                else{
                    Vector poses;
                    if(!bottleToVector(params.tail(), poses)){
                        reply.addString("ERROR while reading the taxel positions");
                        return true;
                    }
                    if(myThread->setTaxelPositions(sp, poses))
                        reply.addInt(skin_manager_ok);
                    else{
                        reply.addInt(skin_manager_error);
                        reply.addString("ERROR: position was not set");
                    }
                }
                return true;
            }
		case get_confidence:
			{
				if(!(params.size()>0 && params.get(0).isInt())){
                    reply.addString(("ERROR: SkinPart is not specified. Params read are: "+string(params.toString().c_str())).c_str());
                    return true;
                }
                SkinPart sp = (SkinPart) params.get(0).asInt();
                if(params.size()>1 && params.get(1).isInt()){
                    unsigned int taxelId = params.get(1).asInt();
                    double res = myThread->getPoseConfidence(sp, taxelId);
					reply.addDouble(res);
                }
                else{
                    Vector res = myThread->getPoseConfidences(sp);
                    if(res.size()==0)
                        reply.addString("Specified skin part has not been found");
                    else
                        addToBottle(reply, res);
                }
                return true;

			}
        case get_info:
            reply.append(myThread->getInfo());
            return true;

		default:
			reply.addString("ERROR: This command is known but it is not managed in the code.");
			return true;
	}

	reply.addString( (SkinManagerCommandList[com]+" command received.").c_str());

	return true;	
}

void skinManager::addToBottle(Bottle& b, const Vector& v){
    for(unsigned int i=0; i<v.size(); i++)
        b.addDouble(v[i]);
}

void skinManager::addToBottle(Bottle& b, const vector<Vector>& v){
    for(unsigned int i=0; i<v.size(); i++)
        for(unsigned int j=0; j<v[i].size(); j++)
            b.addDouble(v[i][j]);
}

bool skinManager::bottleToVector(const yarp::os::Bottle& b, yarp::sig::Vector& v){
    for(int i=0; i<b.size(); i++)
        if(b.get(i).isDouble() || b.get(i).isInt())
            v.push_back(b.get(i).asDouble());
        else
            return false;
    return true;
}

/**
  * Identify the command in the bottle and return the correspondent enum value.
  */
bool skinManager::identifyCommand(Bottle commandBot, SkinManagerCommand &com, Bottle& params) {
	for(unsigned int i=0; i<SkinManagerCommandSize; i++){
		stringstream stream(SkinManagerCommandList[i]);
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
			com = (SkinManagerCommand)i;
            params = commandBot.tail();
            for(int k=1; k<j; k++)
                params = params.tail();
			return true;
		}
	}

	return false;
}

bool skinManager::updateModule() { 
    double avgTime, stdDev, period;
    period = myThread->getRate();
    myThread->getEstPeriod(avgTime, stdDev);
    double avgTimeUsed, stdDevUsed;
    myThread->getEstUsed(avgTimeUsed, stdDevUsed);     // real duration of run()
    if(avgTime > 1.3 * period){
        yWarning("Thread too slow. Real period: %3.3f+/-%3.3f. Expected period: %3.3f.\n", avgTime, stdDev, period);
        yWarning("Duration of 'run' method: %3.3f+/-%3.3f.\n", avgTimeUsed, stdDevUsed);
    }
    return true;
}

double skinManager::getPeriod(){ return 1.0;}

