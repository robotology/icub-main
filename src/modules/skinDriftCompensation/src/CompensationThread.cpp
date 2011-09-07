
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
#include <yarp/os/Time.h>
#include "math.h"
#include "memory.h"
#include "iCub/skinDriftCompensation/CompensationThread.h"

#define FOR_ALL_PORTS(i) for(unsigned int i=0;i<portNum;i++)

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::skinDriftCompensation;


CompensationThread::CompensationThread(string name, ResourceFinder* rf, string robotName, double _compensationGain, double _contactCompensationGain, 
                                       int addThreshold, float minBaseline, bool zeroUpRawData, 
									   int period, bool binarization, bool smoothFilter, float smoothFactor)
									   : 
										RateThread(period), moduleName(name), compensationGain(_compensationGain), 
                                            contactCompensationGain(_contactCompensationGain), 
											ADD_THRESHOLD(addThreshold), PERIOD(period), robotName(robotName), 
											binarization(binarization), smoothFilter(smoothFilter), smoothFactor(smoothFactor)
{
   this->rf								= rf;
   this->minBaseline					= minBaseline;
   this->zeroUpRawData					= zeroUpRawData;
   initializationFinished               = false;
}

bool CompensationThread::threadInit() 
{
    fprintf(stderr, "THREAD INIT\n\n");

   /* initialize variables and create data-structures if needed */
	lastTimestamp = Time::now();
	frequencyOld = 1.0/PERIOD;

	readErrorCounter = 0;
	state = calibration;
	calibrationCounter = 0;
    CAL_SAMPLES = 1000*CAL_TIME/PERIOD; // samples needed for calibration

    // open the output ports for communicating with the gui
    string monitorPortName = "/" + moduleName + "/monitor:o";   // output streaming data
    string infoPortName = "/" + moduleName + "/info:o";         // output occasional data
	if(!monitorPort.open(monitorPortName.c_str())){
		stringstream msg; msg<< "Unable to open port " << monitorPortName << endl;
        sendInfoMsg(msg.str());
        initializationFinished = true;
		return false;
	}
    if(!infoPort.open(infoPortName.c_str())){
		stringstream msg; msg<< "Unable to open port " << infoPortName << endl;
        sendInfoMsg(msg.str());
        initializationFinished = true;
		return false;
	}

	// open the input and output ports for the skin data
    if(!rf->check("outputPorts") || !rf->check("inputPorts")){
        stringstream msg; msg<< "Input ports and/or output ports missing. Closing the module.";
        sendInfoMsg(msg.str());
        initializationFinished = true;
        return false;
    }

	Bottle* outputPortList = rf->find("outputPorts").asList();
	Bottle* inputPortList = rf->find("inputPorts").asList();
	portNum = outputPortList->size();
	if(portNum<=0 || portNum!=inputPortList->size()){
        stringstream msg;
		msg<< "No input port specified or mismatching number of input and output ports ("
			<< portNum<< " out ports; "<< inputPortList->size()<< " in ports).";
        sendInfoMsg(msg.str());
        initializationFinished = true;
		return false;
	}
	
    compensators.resize(portNum);
    compWorking.resize(portNum);
    compensatorCounter = portNum;
    SKIN_DIM = 0;
    cout<< portNum<< " input ports found in the configuration file\n";
    FOR_ALL_PORTS(i){
		string outputPortName = outputPortList->get(i).asString().c_str();
		string inputPortName = inputPortList->get(i).asString().c_str();
        cout<< "\nInput port: "<< inputPortName<< " -> Output port: "<< outputPortName<< endl;
        stringstream name;
        name<< moduleName<< i;
		compensators[i] = new Compensator(name.str(), robotName, outputPortName, inputPortName, &infoPort,
                         compensationGain, contactCompensationGain, ADD_THRESHOLD, minBaseline, zeroUpRawData, binarization, 
                         smoothFilter, smoothFactor);
        SKIN_DIM += compensators[i]->getNumTaxels();
	}

    // remove the compensators that did not open correctly
    FOR_ALL_PORTS(i){
        compWorking[i] = compensators[i]->isWorking();
        if(!compWorking[i]){
            stringstream msg;
            msg<< "[ERROR] Compensator "<< compensators[i]->getInputPortName().c_str()
                << " did not open correctly. Removing the port.";
            sendInfoMsg(msg.str());
            if(compensatorCounter==1){
                msg.str("");
                msg<< "No input port left. Stopping the thread.";
                sendInfoMsg(msg.str());
                this->threadRelease();
                initializationFinished = true;
                return false;
            }
            compensatorCounter--;
            SKIN_DIM -= compensators[i]->getNumTaxels();            
        }
    }	


    // configure the SKIN_EVENT if the corresponding section exists
    skinEventsOn = false;
	Bottle &skinEventsConf = rf->findGroup("SKIN_EVENTS");
	if(!skinEventsConf.isNull()){
        cout<< "SKIN_EVENTS section found\n";
        
        //maxNeighbourDistance = skinEventsConf.check("maxNeighbourDistance", 0.01).asDouble();
        if(skinEventsConf.check("linkList")){
            Bottle* linkList = skinEventsConf.find("linkList").asList();
            if(linkList->size() != portNum){
                stringstream msg;
                msg<< "ERROR: the number of link id is not equal to the number of input ports ("
                    << linkList->size()<< "!="<< portNum<< ")";
                sendInfoMsg(msg.str());
            }else{
                FOR_ALL_PORTS(i){
                    compensators[i]->setLinkNum(linkList->get(i).asInt());
                }
                string eventPortName = "/" + moduleName + "/skin_events:o";  // output skin events
	            if(!skinEventsPort.open(eventPortName.c_str())){
                    stringstream msg; msg << "Unable to open port " << eventPortName << endl;
                    sendInfoMsg(msg.str());
                }else{
                    skinEventsOn = true;
                }
            }
        }

        if(skinEventsConf.check("bodyParts")){
            Bottle* bodyPartList = skinEventsConf.find("bodyParts").asList();
            if(bodyPartList->size() != portNum){
                stringstream msg;
                msg<< "ERROR: the number of body part ids is not equal to the number of input ports ("
                    << bodyPartList->size()<< "!="<< portNum<< "). Body parts will not be set.";
                sendInfoMsg(msg.str());
            }else{
                FOR_ALL_PORTS(i){
                    cout<< "Body part "<< BodyPart_s[bodyPartList->get(i).asInt()]<< endl;
                    compensators[i]->setBodyPart((BodyPart)bodyPartList->get(i).asInt());
                }                
            }
        }

        if(skinEventsConf.check("skinParts")){
            Bottle* skinPartList = skinEventsConf.find("skinParts").asList();
            if(skinPartList->size() != portNum){
                stringstream msg;
                msg<< "ERROR: the number of skin part ids is not equal to the number of input ports ("
                    << skinPartList->size()<< "!="<< portNum<< "). Skin parts will not be set.";
                sendInfoMsg(msg.str());
            }else{
                FOR_ALL_PORTS(i){
                    cout<< "Skin part "<< SkinPart_s[skinPartList->get(i).asInt()]<< endl;
                    compensators[i]->setSkinPart((SkinPart)skinPartList->get(i).asInt());
                }                
            }
        }
    
        if(skinEventsConf.check("taxelPositionFiles")){
            Bottle *taxelPosFiles = skinEventsConf.find("taxelPositionFiles").asList();
            if(portNum!=taxelPosFiles->size()){
                stringstream msg;
                msg<< "Mismatching number of taxel position files and input ports ("
                    <<portNum<< " in ports; "<< taxelPosFiles->size()<< " taxel position files). ";
                msg<< "Taxel positions will not be set.";
                sendInfoMsg(msg.str());
            }
            else{
                double maxNeighborDist = rf->check("maxNeighborDist", Value(0.12)).asDouble();
                printf("Max neighbor distance: %f\n", maxNeighborDist);
                FOR_ALL_PORTS(i){
	                string taxelPosFile = taxelPosFiles->get(i).asString().c_str();
	                string filePath(rf->findFile(taxelPosFile.c_str()));
	                compensators[i]->setTaxelPositions(filePath.c_str(), maxNeighborDist);
	            }
            }
        }
	}
    if(skinEventsOn)
        sendInfoMsg("Skin events ENABLED.");
    else
        sendInfoMsg("Skin events DISABLED.");

    initializationFinished = true;
	return true;
}




void CompensationThread::forceCalibration(){
	stateSem.wait();
	if(state != calibration){
		state = calibration;
		calibrationCounter = 0;
	}
	stateSem.post();
}

void CompensationThread::run(){
	stateSem.wait();

	if( state == compensation){
		// It reads the raw data, computes the difference between the read values and the baseline 
		// and outputs these values
        FOR_ALL_PORTS(i){
            if(compWorking[i]){
		        if(compensators[i]->readRawAndWriteCompensatedData()){
			        //If the read succeeded, update the baseline
			        compensators[i]->updateBaseline();
		        }
            }
        }

        if(skinEventsOn){
            sendSkinEvents();
        }
	}
	else if(state == calibration){
        FOR_ALL_PORTS(i){    
            if(compWorking[i]){
		        if(calibrationCounter==0)
			        compensators[i]->calibrationInit();
        		
		        compensators[i]->calibrationDataCollection();

		        if(calibrationCounter==CAL_SAMPLES){
			        compensators[i]->calibrationFinish();
			        state = compensation;
		        }
            }
        }
        calibrationCounter++;
	}
	else{
		stateSem.post();
        sendInfoMsg("[ERROR] Unknown state in CompensationThread. Suspending the thread.\n");
		this->suspend();
		return;
    }	
    stateSem.post();
	sendMonitorData();
    checkErrors();
}

void CompensationThread::sendSkinEvents(){
    skinContactList &skinEvents = skinEventsPort.prepare();
    skinEvents.clear();

    FOR_ALL_PORTS(i){
        if(compWorking[i]){
            skinContactList temp = compensators[i]->getContacts();
            skinEvents.insert(skinEvents.end(), temp.begin(), temp.end());            
        }
    }
    printf("%d contacts (timestamp %.3f)\n", skinEvents.size(), Time::now());
    skinEventsPort.write();     // send something anyway (if there is no contact the bottle is empty)
}


void CompensationThread::checkErrors(){
    FOR_ALL_PORTS(i){
        if(compWorking[i]){
            compWorking[i] = compensators[i]->isWorking();
            if(!compWorking[i]){    // read failed too many times in a row, remove the port
                if(compensatorCounter==1){
                    fprintf(stderr, "No input port left. Stopping the compensation thread\n");
                    this->threadRelease();
		            this->suspend();
                    return;
                }
                
                compensatorCounter--;
                SKIN_DIM -= compensators[i]->getNumTaxels();    // remove the taxel from the total count                                
            }
        }
    }

    unsigned int taxInd, compInd;
    double baseline, initialBaseline;
    if(doesBaselineExceed(compInd, taxInd, baseline, initialBaseline)){
        stringstream msg;
        msg<< "Baseline of the taxel "<< taxInd<< " of port "<< compensators[compInd]->getInputPortName()
            << " saturated (current baseline="<< baseline<< "; initial baseline="<< initialBaseline<< 
            ")! A skin calibration is suggested.";
        sendInfoMsg(msg.str());
    }
}

bool CompensationThread::doesBaselineExceed(unsigned int &compInd, unsigned int &taxInd, double &baseline, double &initialBaseline){
    stateSem.wait();
    CompensationThreadState currentState = state;
    stateSem.post();
    if(currentState==compensation){
        FOR_ALL_PORTS(i){
            if(compWorking[i] && compensators[i]->doesBaselineExceed(taxInd, baseline, initialBaseline)){
                compInd = i;
                return true;
            }
        }
    }    
    return false;
}

void CompensationThread::threadRelease() 
{
    FOR_ALL_PORTS(i){
        delete compensators[i];
    }
    portNum = 0;
    state = compensation;   // to prevent the GUI from looping calling isCalibrating() on the thread

    monitorPort.interrupt();
    infoPort.interrupt();
    monitorPort.close();
    infoPort.close();
}


// send the data on the monitor port
void CompensationThread::sendMonitorData(){
	// update the frequency
	double currentTimestamp = Time::now();
	double timeBetweenRead = currentTimestamp - lastTimestamp;
	lastTimestamp = currentTimestamp;
	frequency = 0.5*(1.0/timeBetweenRead) + 0.5*frequencyOld;	// moving average
	frequencyOld = frequency;

	// send the monitor data (if there is at least a connection)
    if(monitorPort.getOutputCount()>0){
	    Bottle &b = monitorPort.prepare();
	    b.clear();
	    b.addDouble(frequency); // data frequency
        
        stateSem.wait();
        if(state==compensation){    // during calibration don't send this data
            // for each taxel add how much the baseline has changed so far (i.e. the drift)
            FOR_ALL_PORTS(i){
                if(compWorking[i]){
                    Vector temp = compensators[i]->getBaselines();
                    for(int j=0; j<temp.size(); j++){ 
                        b.addDouble(temp[j]);
                        if(temp[j]<0){
                            stringstream ss;
                            ss<<"port "<<compensators[i]->getSkinPartName()<<"; tax "<< j<<"; baseline "<< temp[j]<< "; d: "<< 
                                compensators[i]->getCompData()[j]<<"; raw: "<< compensators[i]->getRawData()[j]<< 
                                "; touchThr: "<< compensators[i]->getTouchThreshold()[j];
                            sendInfoMsg(ss.str());
                        }
                    }
                }
            }
            FOR_ALL_PORTS(i){
                if(compWorking[i]){
                    Vector temp = compensators[i]->getRawData();
                    for(int j=0; j<temp.size(); j++) b.addDouble(temp[j]);
                }
            }
        }
        stateSem.post();
        //printf("Writing %d data on monitor port\n", b.size());
	    monitorPort.write();
    }
}

void CompensationThread::sendInfoMsg(string msg){    
    printf("\n");
    printf("%s", msg.c_str());
    Bottle& b = infoPort.prepare();
    b.clear();
    b.addString(msg.c_str());
    infoPort.write();
}

Bottle CompensationThread::getInfo(){
    Bottle res;
    if(initializationFinished){    // check whether the thread has been initialized
        Bottle& nameB = res.addList();
		nameB.addString("Name: ");
		nameB.addString(moduleName.c_str());
		Bottle& robotB = res.addList();
        robotB.addString("Robot Name: "); 
		robotB.addString(robotName.c_str());
        Bottle& portB = res.addList();
        string compName;
        FOR_ALL_PORTS(i){
            compName = compensators[i]->getInputPortName().c_str();
            if(!compWorking[i])
                compName = compName + " (NOT WORKING)";
            portB.addString(compName.c_str());
			portB.addInt(compensators[i]->getNumTaxels());
        }
    }else{
        res.addString("Module initialization has not been completed yet.");
    }
    return res;
}

void CompensationThread::setBinarization(bool value){
    binarization = value;
    FOR_ALL_PORTS(i){
	    compensators[i]->setBinarization(value);
    }
}
void CompensationThread::setSmoothFilter(bool value){
	if(smoothFilter != value){
		stateSem.wait();
        smoothFilter = value;
        FOR_ALL_PORTS(i){
            compensators[i]->setSmoothFilter(value);
        }
		stateSem.post();
	}
}
bool CompensationThread::setSmoothFactor(float value){
	if(value<0 || value>1)
		return false;
	if(value==1) 
		value = 0.99f;	// otherwise with 1 the values don't update
	smoothFactor = value;
    FOR_ALL_PORTS(i){
        compensators[i]->setSmoothFactor(value);
    }
	return true;
}
bool CompensationThread::setAddThreshold(unsigned int thr){
    bool res = true;
    FOR_ALL_PORTS(i){
        res = res && compensators[i]->setAddThreshold(thr);
    }
    if(res)
        ADD_THRESHOLD = thr;
    return res;
}

bool CompensationThread::setCompensationGain(double gain){
    bool res = true;
    FOR_ALL_PORTS(i){
        res = res && compensators[i]->setCompensationGain(gain);
    }
    if(res)
        compensationGain = gain;
    return res;
}

bool CompensationThread::setContactCompensationGain(double gain){
    bool res = true;
    FOR_ALL_PORTS(i){
        res = res && compensators[i]->setContactCompensationGain(gain);
    }
    if(res)
        contactCompensationGain = gain;
    return res;
}

Vector CompensationThread::getTouchThreshold(){
    Vector res(SKIN_DIM);
    int currentDim=0;
    FOR_ALL_PORTS(i){
        if(compWorking[i]){
	        Vector temp = compensators[i]->getTouchThreshold();
            memcpy(res.data()+currentDim, temp.data(), temp.size()*sizeof(double));
            currentDim += temp.size();
        }
    }
	return res;
}

unsigned int CompensationThread::getAddThreshold(){
    return ADD_THRESHOLD;
}

double CompensationThread::getCompensationGain(){
    return compensationGain;
}

double CompensationThread::getContactCompensationGain(){
    return contactCompensationGain;
}

bool CompensationThread::getBinarization(){
	return binarization;
}
bool CompensationThread::getSmoothFilter(){
	return smoothFilter;
}
bool CompensationThread::isCalibrating(){
	stateSem.wait();
	bool res = state==calibration;
	stateSem.post();
	return res;
}
float CompensationThread::getSmoothFactor(){
	return smoothFactor;
}

