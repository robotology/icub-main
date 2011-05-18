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
#include <algorithm>
#include "iCub/skinDriftCompensation/Compensator.h"


using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::skinDriftCompensation;

Compensator::Compensator(string name, string robotName, string outputPortName, string inputPortName, 
                         float changePerTimestep, int addThreshold, float _minBaseline, bool _zeroUpRawData, bool _binarization, 
                         bool _smoothFilter, float _smoothFactor)
									   : 
										CHANGE_PER_TIMESTEP(changePerTimestep), ADD_THRESHOLD(addThreshold), 
                                            minBaseline(_minBaseline), binarization(_binarization), smoothFilter(_smoothFilter), 
                                            smoothFactor(_smoothFactor), robotName(robotName), name(name)
{
    this->zeroUpRawData = _zeroUpRawData;
    _isWorking = init(name, robotName, outputPortName, inputPortName);
}

Compensator::~Compensator(){
    if(tactileSensorDevice)
		tactileSensorDevice->close();

    compensatedTactileDataPort.interrupt();
    compensatedTactileDataPort.close();
}

bool Compensator::init(string name, string robotName, string outputPortName, string inputPortName){
    if (!compensatedTactileDataPort.open(outputPortName.c_str())) {
	    cout<< "Unable to open output port "<< outputPortName<< endl;
	    return false;  // unable to open
    }

    Property options;
    stringstream localPortName;
    localPortName<< "/"<< name<< "/input";
    options.put("robot",  robotName.c_str());
    options.put("local",  localPortName.str().c_str());
    options.put("remote",  inputPortName.c_str());	
    options.put("device", "analogsensorclient");
     
    // create a new device driver
    tactileSensorDevice = new PolyDriver(options);
    if (!tactileSensorDevice->isValid()){
	    printf("Device not available.  Here are the known devices:\n");
	    printf("%s", Drivers::factory().toString().c_str());
	    return false;
    }
    // open the sensor interface	
    bool ok = tactileSensorDevice->view(tactileSensor);
    if (!ok) {
	    printf("Problems acquiring interfaces\n");
	    return false;
    }
    
    SKIN_DIM = tactileSensor->getChannels();
    if(SKIN_DIM<=0){
		fprintf(stderr, "Error while reading the number of channels of the tactile sensor device\n");
		SKIN_DIM = 192;
	}
    readErrorCounter = 0;
    baselines.resize(SKIN_DIM);
    touchThresholds.resize(SKIN_DIM);
    touchDetected.resize(SKIN_DIM);
    compensatedData.resize(SKIN_DIM);
    compensatedDataOld.resize(SKIN_DIM);

    return true;
}



void Compensator::calibrationInit(){   
	// take the semaphore so that the touchThreshold can't be read during the calibration phase
	touchThresholdSem.wait();

	// send a command to the microcontroller for calibrating the skin sensors
	if(robotName!="icubSim"){	// this feature isn't implemented in the simulator and causes a runtime error
		tactileSensor->calibrateSensor();
	}

	// initialize
    calibrationRead = 0;
    saturatedTaxels.resize(0);
	start_sum.assign(SKIN_DIM, 0);
	skin_empty.assign(SKIN_DIM, vector<int>(MAX_SKIN+1, 0));
}

void Compensator::calibrationDataCollection(){	
	//calibrationCounter++; 

    Vector skin_values(SKIN_DIM);
    if(!readInputData(skin_values))
        return;
	calibrationRead++;
		
	for (unsigned int j=0; j<SKIN_DIM; j++) {
		if (zeroUpRawData==false)
			skin_values[j] = MAX_SKIN - skin_values[j];
		
		if(skin_values[j]<0 || skin_values[j]>MAX_SKIN){
			fprintf(stderr, "Error while reading the tactile data! Data out of range: %d\n", (int)skin_values[j]);
		}
		else{
			skin_empty[j][int(skin_values[j])]++;
			start_sum[j] += int(skin_values[j]);
		}
	}	
}

void Compensator::calibrationFinish(){
	
	//vector<float> standard_dev(SKIN_DIM, 0);
	//get percentile
	for (unsigned int i=0; i<SKIN_DIM; i++) {
		//avg start value
		baselines[i] = start_sum[i]/calibrationRead;
		
		//cumulative values
		for (int j=1; j<=MAX_SKIN; j++) {
			//standard_dev[i] += fabs(j-baselines[i]) * skin_empty[i][j];
			skin_empty[i][j] += skin_empty[i][j-1] ;			
		}
		//standard_dev[i] /= (CAL_TIME*PERIOD);

		//when do we cross the threshold?
		for (int j=0; j<=MAX_SKIN; j++) {
			if (skin_empty[i][j] > (calibrationRead*0.95)) {
                // the threshold can not be less than MIN_TOUCH_THR
				touchThresholds[i] = max<double>(MIN_TOUCH_THR, (double)j - baselines[i]);
				j = MAX_SKIN;   // break
			}
		}
	}
    // store the initial baseline to compute the drift compensated later
    initialBaselines = baselines;
	
	// release the semaphore so that as of now the touchThreshold can be read
	touchThresholdSem.post();

	// set the "old output value" for the smoothing filter to the baseline value to get a smooth start
	compensatedDataOld = baselines;

	// print to console
	fprintf(stderr, "\n[%s] Baselines:\n", name.c_str());
	for (unsigned int i=0; i<SKIN_DIM; i++) {
		if(!(i%12)) fprintf(stderr, "\n");
		fprintf(stderr,"%4.1f ", baselines[i]);		
	}

	/*fprintf(stderr, "\nStandard dev:\n");
	for (int i=0; i<SKIN_DIM; i++) {
		if(!(i%12)) fprintf(stderr, "\n");
		fprintf(stderr,"%3.1f ", standard_dev[i]);
	}*/

	fprintf(stderr,"\n[%s] Thresholds (95 percentile):\n", name.c_str());
	for (unsigned int i=0; i<SKIN_DIM; i++) {
		if(!(i%12)) fprintf(stderr, "\n");
		fprintf(stderr,"%3.1f ", touchThresholds[i]);		
	}
	fprintf(stderr,"\n");
}

bool Compensator::readInputData(Vector& skin_values){
    int err;
    if((err=tactileSensor->read(skin_values))!=IAnalogSensor::AS_OK){
        readErrorCounter++;
        if(err == IAnalogSensor::AS_TIMEOUT)
            fprintf(stderr, "[%s]: Timeout error reading tactile sensor.\n", compensatedTactileDataPort.getName().c_str(), err);
        else if(err == IAnalogSensor::AS_OVF)
            fprintf(stderr, "[%s]: Ovf error reading tactile sensor.\n", compensatedTactileDataPort.getName().c_str(), err);
        else if(err == IAnalogSensor::AS_ERROR)
            fprintf(stderr, "[%s]: Generic error reading tactile sensor.\n", compensatedTactileDataPort.getName().c_str(), err);
	    return false;
    }

    if(skin_values.size() != SKIN_DIM){
        readErrorCounter++;
        fprintf(stderr, "Unexpected size of the input array (raw tactile data): %d\n", skin_values.size());
        return false;
    }
    

    readErrorCounter = 0;
    return true;
}
unsigned int Compensator::getErrorCounter(){
    return readErrorCounter;
}

bool Compensator::readRawAndWriteCompensatedData(){
    Vector rawData(SKIN_DIM);
    if(!readInputData(rawData))
        return false;
	
	Vector& compensatedData2Send = compensatedTactileDataPort.prepare();
    compensatedData2Send.clear();   // local variable with data to send
	compensatedData.clear();        // global variable with data to store
	
	double d;
	for(unsigned int i=0; i<SKIN_DIM; i++){
	    // baseline compensation
	    if( zeroUpRawData == false){
		    d = (double)( MAX_SKIN - rawData(i) - baselines[i]);
	    }else{
		    d = (double)(rawData(i) - baselines[i]);
	    }
	    compensatedData.push_back(d);	// save the data before applying filtering

        // detect touch (before applying filtering, so the compensation algorithm is not affected by the filters)
        if(d > touchThresholds[i] + ADD_THRESHOLD){
		    touchDetected[i] = true;
	    }else{
		    touchDetected[i] = false;
	    }	   

        // smooth filter
        if(smoothFilter){
		    smoothFactorSem.wait();
		    d = (1-smoothFactor)*d + smoothFactor*compensatedDataOld(i);
		    smoothFactorSem.post();
		    compensatedDataOld(i) = d;	// update old value
	    }

	    // binarization filter
        if(binarization){
            // here we don't use the touchDetected array because, if the smooth filter is on,
            // we want to use the filtered values
	        if(d > touchThresholds[i] + ADD_THRESHOLD)
			    d = BIN_TOUCH;
	        else
		        d = BIN_NO_TOUCH;
        }else if(d<0) // if negative, set it to zero
		    d=0;
	    compensatedData2Send.push_back(d);
	}

	if(compensatedData2Send.size() != SKIN_DIM){
		fprintf(stderr, "Unexpected size of the output array (compensated tactile data): %d\n", compensatedData2Send.size());
		return false;
	}

	compensatedTactileDataPort.write();
	return true;
}

void Compensator::updateBaseline(){
	double mean_change = 0, change;
    unsigned int non_touching_taxels = 0;
	double d; 

    for(unsigned int j=0; j<SKIN_DIM; j++) {
        if(!touchDetected[j]){
			non_touching_taxels++;										//for changing the taxels where we detected touch
			d = compensatedData(j);

            // old algorithm
			//if(d > 0.5) {
			//	baselines[j]		+= CHANGE_PER_TIMESTEP;
			//	mean_change			+= CHANGE_PER_TIMESTEP;				//for changing the taxels where we detected touch
			//}else if(d < -0.5) {
			//	baselines[j]		-= CHANGE_PER_TIMESTEP;
			//	mean_change			-= CHANGE_PER_TIMESTEP;				//for changing the taxels where we detected touch
			//}

            // new algorithm
			if(fabs(d)>0.5){
                change          = CHANGE_PER_TIMESTEP*d/touchThresholds[j];
				baselines[j]    += change;
                mean_change     += change;
			}
		}
    }
    
    //for compensating the taxels where we detected touch
    if (non_touching_taxels>0 && non_touching_taxels<SKIN_DIM && mean_change!=0){
        mean_change /= non_touching_taxels;
        for(unsigned int j=0; j<SKIN_DIM; j++) {
            if (touchDetected[j]) {
                baselines[j]		+= mean_change;
            }
        }
    }
}

bool Compensator::doesBaselineExceed(unsigned int &taxelIndex, double &baseline, double &initialBaseline){
    vector<unsigned int>::iterator it;
	for(unsigned int i=0; i<SKIN_DIM; i++){                
	    if(baselines[i]<minBaseline || baselines[i]>MAX_SKIN-minBaseline){
            it = find(saturatedTaxels.begin(), saturatedTaxels.end(), i);
            if(it==saturatedTaxels.end()){  // if the taxel hasn't been already signalled
			    //fprintf(stderr, "Baseline %d exceeds: %f\n", i, baselines[i]);
                saturatedTaxels.push_back(i);
                baseline = baselines[i];
                initialBaseline = initialBaselines[i];
                taxelIndex = i;
			    return true;
            }
        }
	}
	return false;
}



void Compensator::setBinarization(bool value){
	binarization = value;
}
void Compensator::setSmoothFilter(bool value){
	if(smoothFilter != value){
		smoothFilter = value;
		if(value){
			// set the old output value of the smooth filter to the last read, to get a smooth start
			compensatedDataOld = compensatedData;			
		}
	}
}
bool Compensator::setSmoothFactor(float value){
	if(value<0 || value>1)
		return false;
	if(value==1) 
		value = 0.99f;	// otherwise with 1 the values don't update
	smoothFactorSem.wait();
	smoothFactor = value;
	smoothFactorSem.post();
	return true;
}

unsigned int Compensator::getNumTaxels(){
    if(_isWorking)
        return SKIN_DIM;
    return 0;
}

Vector Compensator::getTouchThreshold(){
	touchThresholdSem.wait();
	Vector res = touchThresholds;
	touchThresholdSem.post();
	return res;
}

Vector Compensator::getCompensation(){
    Vector res(baselines.size());
    for(int i=0; i<res.size(); i++){
        res[i] = baselines[i] - initialBaselines[i];
    }
    return res;
}

bool Compensator::getBinarization(){
	return binarization;
}
bool Compensator::getSmoothFilter(){
	return smoothFilter;
}

float Compensator::getSmoothFactor(){
    smoothFactorSem.wait();
    float res=smoothFactor;
    smoothFactorSem.post();
	return res;
}

string Compensator::getName(){
    return name;
}

string Compensator::getInputPortName(){
    return tactileSensorDevice->getValue("remote").asString().c_str();
}

bool Compensator::isWorking(){
    return _isWorking;
}
