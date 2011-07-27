
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


CompensationThread::CompensationThread(string name, ResourceFinder* rf, string robotName, float maxDrift, int addThreshold, 
									   float minBaseline, bool zeroUpRawData, 
									   int period, bool binarization, bool smoothFilter, float smoothFactor)
									   : 
										RateThread(period), moduleName(name), MAX_DRIFT(maxDrift), 
											ADD_THRESHOLD(addThreshold), PERIOD(period), robotName(robotName), 
											binarization(binarization), smoothFilter(smoothFilter), smoothFactor(smoothFactor)
{
   this->rf								= rf;
   this->minBaseline					= minBaseline;
   this->zeroUpRawData					= zeroUpRawData;
}

bool CompensationThread::threadInit() 
{
    fprintf(stderr, "THREAD INIT\n\n");

   /* initialize variables and create data-structures if needed */
	lastTimestamp = Time::now();
	frequencyOld = 1.0/PERIOD;

	CHANGE_PER_TIMESTEP = MAX_DRIFT/PERIOD;
	readErrorCounter = 0;
	state = calibration;
	calibrationCounter = 0;
    CAL_SAMPLES = 1000*CAL_TIME/PERIOD; // samples needed for calibration

    // open the output ports for communicating with the gui
    string monitorPortName = "/" + moduleName + "/monitor:o";   // output streaming data
    string infoPortName = "/" + moduleName + "/info:o";         // output occasional data
	if(!monitorPort.open(monitorPortName.c_str())){
		cout << "Unable to open port " << monitorPortName << endl;
		return false;
	}
    if(!infoPort.open(infoPortName.c_str())){
		cout << "Unable to open port " << infoPortName << endl;
		return false;
	}

	// open the input and output ports for the skin data
    if(!rf->check("outputPorts") || !rf->check("inputPorts")){
        cout<< "Input ports and/or output ports missing. Closing the module.\n";
        return false;
    }

	Bottle* outputPortList = rf->find("outputPorts").asList();
	Bottle* inputPortList = rf->find("inputPorts").asList();
	portNum = outputPortList->size();
	if(portNum<=0 || portNum!=inputPortList->size()){
		cout<< "No input port specified or mismatching number of input and output ports ("
			<< portNum<< " out ports; "<< inputPortList->size()<< " in ports)\n";
		return false;
	}
	
    compensators.resize(portNum);
    SKIN_DIM = 0;
    cout<< portNum<< " input ports found in the configuration file\n";
    FOR_ALL_PORTS(i){
		string outputPortName = outputPortList->get(i).asString().c_str();
		string inputPortName = inputPortList->get(i).asString().c_str();
        cout<< "\nInput port: "<< inputPortName<< " -> Output port: "<< outputPortName<< endl;
        stringstream name;
        name<< moduleName<< i;
		compensators[i] = new Compensator(name.str(), robotName, outputPortName, inputPortName, 
                         CHANGE_PER_TIMESTEP, ADD_THRESHOLD, minBaseline, zeroUpRawData, binarization, 
                         smoothFilter, smoothFactor);
        SKIN_DIM += compensators[i]->getNumTaxels();
	}

    // remove the compensators that did not open correctly
    FOR_ALL_PORTS(i){
        if(!compensators[i]->isWorking()){
            fprintf(stderr, "[ERROR] Compensator %s did not open correctly. Removing the port.\n", 
                compensators[i]->getName().c_str());
            if(portNum==1){
                fprintf(stderr, "No input port left. Stopping the thread\n");
                return false;
            }            
            portNum--;
            SKIN_DIM -= compensators[i]->getNumTaxels();
            compensators.erase(compensators.begin()+i);
            i--;
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
                fprintf(stderr, "ERROR: the number of link id does not match the number of input ports (%d!=%d)\n", 
                    linkList->size(), portNum);
            }else{
                FOR_ALL_PORTS(i){
                    compensators[i]->setLinkId(linkList->get(i).asInt());
                }
                string eventPortName = "/" + moduleName + "/skin_events:o";  // output skin events
	            if(!skinEventsPort.open(eventPortName.c_str())){
		            cout << "Unable to open port " << eventPortName << endl;
		            skinEventsOn = false;
                }else{
                    skinEventsOn = true;
                }
            }
        }
	}
    if(skinEventsOn)
        fprintf(stderr, "Skin events ENABLED.\n");
    else
        fprintf(stderr, "Skin events DISABLED.\n");

	return true;
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
		    if(compensators[i]->readRawAndWriteCompensatedData()){
			    //If the read succeeded, update the baseline
			    compensators[i]->updateBaseline();
		    }
        }

        if(skinEventsOn){
            sendSkinEvents();
        }
	}
	else if(state == calibration){
        FOR_ALL_PORTS(i){            
		    if(calibrationCounter==0)
			    compensators[i]->calibrationInit();
    		
		    compensators[i]->calibrationDataCollection();

		    if(calibrationCounter==CAL_SAMPLES){
			    compensators[i]->calibrationFinish();
			    state = compensation;
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
    Bottle &skinEvents = skinEventsPort.prepare();
    skinEvents.clear();
    bool contact = false;
    FOR_ALL_PORTS(i){
        if(compensators[i]->isThereContact()){
            contact = true;
            Bottle &c = skinEvents.addList();
            c.addString("part");
            c.addString(compensators[i]->getInputPortName().c_str());
            c.addString("link");
            c.addInt(compensators[i]->getLinkId());
        }
    }
    if(contact)
        skinEventsPort.write();
}


void CompensationThread::checkErrors(){
    FOR_ALL_PORTS(i){
        readErrorCounter = compensators[i]->getErrorCounter();
        if(readErrorCounter >= MAX_READ_ERROR){    // read failed too many times in a row, remove the port
            stringstream msg;
            msg<< "[ERROR] "<< MAX_READ_ERROR<< " successive errors reading port "<< 
                compensators[i]->getInputPortName()<< ". Closing compensator "<< compensators[i]->getName()<< ".\n";
            sendInfoMsg(msg.str());

            if(portNum==1){
                fprintf(stderr, "No input port left. Stopping the thread\n");
		        this->suspend();
                return;
            }
            
            portNum--;
            SKIN_DIM -= compensators[i]->getNumTaxels();    // remove the taxel from the total count
            compensators.erase(compensators.begin()+i);
            i--;
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
            if(compensators[i]->doesBaselineExceed(taxInd, baseline, initialBaseline)){
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

    monitorPort.interrupt();
    infoPort.interrupt();
    monitorPort.close();
    infoPort.close();
}

Vector CompensationThread::getTouchThreshold(){
    Vector res(SKIN_DIM);
    int currentDim=0;
    FOR_ALL_PORTS(i){
	    Vector temp = compensators[i]->getTouchThreshold();
        memcpy(res.data()+currentDim, temp.data(), temp.size()*sizeof(double));
        currentDim += temp.size();
    }
	return res;
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
// send the data on the monitor port
void CompensationThread::sendMonitorData(){
	// update the frequency
	double currentTimestamp = Time::now();
	double timeBetweenRead = currentTimestamp - lastTimestamp;
	lastTimestamp = currentTimestamp;
	frequency = 0.5*(1.0/timeBetweenRead) + 0.5*frequencyOld;	// moving average
	frequencyOld = frequency;

	// send the monitor data
	Bottle &b = monitorPort.prepare();
	b.clear();
	b.addDouble(frequency); // data frequency
    
    stateSem.wait();
    if(state==compensation){    // during calibration don't send this data
        // for each taxel add how much the baseline has changed so far (i.e. the drift)
        FOR_ALL_PORTS(i){
            Vector temp = compensators[i]->getCompensation();
            for(int j=0; j<temp.size(); j++){
                b.addDouble(temp[j]);
            }
        }
    }
    stateSem.post();
	monitorPort.write();
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
    if(this->getIterations()>1){    // check whether the thread has been initialized
        Bottle& nameB = res.addList();
		nameB.addString("Name: "); 
		nameB.addString(moduleName.c_str());
		Bottle& robotB = res.addList();
        robotB.addString("Robot Name: "); 
		robotB.addString(robotName.c_str());        
        Bottle& portB = res.addList();
        FOR_ALL_PORTS(i){
            portB.addString(compensators[i]->getInputPortName().c_str());
			portB.addInt(compensators[i]->getNumTaxels());
        }
    }
    return res;
}
