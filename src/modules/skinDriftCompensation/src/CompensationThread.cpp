
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
#include "iCub/skinDriftCompensation/CompensationThread.h"


using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::skinDriftCompensation;


CompensationThread::CompensationThread(ResourceFinder* rf, string robotName, float* minBaseline, bool *calibrationAllowed, 
									   bool zeroUpRawData, bool rightHand, int period, bool binarization, bool smoothFilter, 
									   float smoothFactor)
									   : RateThread(period), PERIOD(period), robotName(robotName), binarization(binarization),
									   smoothFilter(smoothFilter), smoothFactor(smoothFactor)
{
   this->rf								= rf;
   this->minBaseline					= minBaseline;
   this->calibrationAllowed				= calibrationAllowed;
   this->zeroUpRawData					= zeroUpRawData;
   this->rightHand						= rightHand;
}

bool CompensationThread::threadInit() 
{
    fprintf(stderr, "THREAD INIT\n\n");

   /* initialize variables and create data-structures if needed */
	MAX_DRIFT = 0.1f;								// the maximal drift that is being compensated every second
	CHANGE_PER_TIMESTEP = MAX_DRIFT/PERIOD;
	readErrorCounter = 0;
	state = calibration;
	calibrationCounter = 0;
	calibrationRead = 0;

	// open the output port
	string compensatedTactileDataPortName;
	if(rightHand){
		compensatedTactileDataPortName		= "/"+ robotName+ "/skin/right_hand_comp";
	}else{
		compensatedTactileDataPortName		= "/"+ robotName+ "/skin/left_hand_comp";
	}
	if (!compensatedTactileDataPort.open(compensatedTactileDataPortName.c_str())) {
		cout << "Unable to open port " << compensatedTactileDataPortName << endl;
		return false;  // unable to open
	}

	// open the analog sensor interface for the skin
	Property options;
	options.put("robot",  robotName.c_str());
	if(rightHand){
		options.put("part",   "righthand");         //skin part that you want to control
		options.put("local",  "/skinComp/right");
		options.put("remote",  ("/"+robotName+"/skin/right_hand").c_str());
	}else{
		options.put("part",   "lefthand");          //skin part that you want to control
		options.put("local",  "/skinComp/left");
		options.put("remote",  ("/"+robotName+"/skin/left_hand").c_str());
	}
	options.put("device", "analogsensorclient");	//important! It’s different from remote_controlboard that you use to control motors!
	 
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
	if(SKIN_DIM==0){
		fprintf(stderr, "Error while reading the number of channels of the tactile sensor device\n");
		SKIN_DIM = 192;
	}
	touchDetected.resize(SKIN_DIM);
	touchThresholds.resize(SKIN_DIM);
    baselines.resize(SKIN_DIM);

	return true;
}


void CompensationThread::setBinarization(bool value){
	binarization = value;
}
void CompensationThread::setSmoothFilter(bool value){
	if(smoothFilter != value){
		stateSem.wait();
		smoothFilter = value;
		if(value){				
			// set the old output value of the smooth filter to the last read, to get a smooth start
			compensatedDataOld = compensatedData;			
		}
		stateSem.post();
	}
}
bool CompensationThread::setSmoothFactor(float value){
	if(value<0 || value>1)
		return false;
	smoothFactorSem.wait();
	smoothFactor = value;
	smoothFactorSem.post();
	return true;
}
void CompensationThread::forceCalibration(){
	stateSem.wait();
	if(state != calibration){
		state = calibration;
		calibrationCounter = 0;	
		calibrationRead = 0;
	}
	stateSem.post();
}

void CompensationThread::run(){
	stateSem.wait();

	if( state == compensation){
		// It reads the raw data, computes the difference between the read values and the baseline 
		// and outputs these values
		if(readRawAndWriteCompensatedData()){
			//If the read succeeded, update the baseline
			updateBaseline();
		}

		//If calibration is allowed AND at least one baseline is less than minBaseline (or greater than 255-minBaseline)
		if( (*calibrationAllowed)==true && doesBaselineExceed()){
			state = calibration;
			calibrationCounter = 0;	
			calibrationRead = 0;
		}
	}
	else if(state == calibration){
		if(calibrationCounter==0)
			calibrationInit();
		
		calibrationDataCollection();

		if(calibrationCounter==PERIOD*CAL_TIME){
			calibrationFinish();
			state = compensation;
		}
	}
	else{
		stateSem.post();
		fprintf(stderr, "\n[ERROR] Unknown state in CompensationThread. Suspending the thread.\n");
		this->suspend();
		return;
	}

	stateSem.post();


	if(readErrorCounter >= MAX_READ_ERROR){    // read failed too many times in a row, so suspend the thread
        fprintf(stderr, "[ERROR] %d successive errors reading the sensor data. Suspending the thread.", MAX_READ_ERROR);
		this->suspend();
    }
}

void CompensationThread::calibrationInit(){   
	fprintf(stderr, "CALIBRATING.......................");
	// take the semaphore so that the touchThreshold can't be read during the calibration phase
	touchThresholdSem.wait();

	// send a command to the microcontroller for calibrating the skin sensors
	if(robotName!="icubSim")	// this feature isn't implemented in the simulator and causes a runtime error
		tactileSensor->calibrateSensor();
	fprintf(stderr, "Chip calibration executed\n");

	// initialize
	start_sum.assign(SKIN_DIM, 0);
	skin_empty.assign(SKIN_DIM, vector<int>(MAX_SKIN+1, 0));
}

void CompensationThread::calibrationDataCollection(){	
	calibrationCounter++; 

	Vector skin_values;
	int err;
	if((err=tactileSensor->read(skin_values))!=IAnalogSensor::AS_OK){
        readErrorCounter++;
		fprintf(stderr, "Error reading tactile sensor: %d\n", err);
		return;
	}
	
	readErrorCounter = 0;
	calibrationRead++;
		
	for (int j=0; j<SKIN_DIM; j++) {
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

void CompensationThread::calibrationFinish(){
	
	//vector<float> standard_dev(SKIN_DIM, 0);
	//get percentile
	for (int i=0; i<SKIN_DIM; i++) {
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
				touchThresholds[i] = max<double>(0.0, (double)j - baselines[i]);	// the threshold can not be less than zero
				j = MAX_SKIN;
			}
		}
	}
	
	// release the semaphore so that as of now the touchThreshold can be read
	touchThresholdSem.post();

	// set the "old output value" for the smoothing filter to the baseline value to get a smooth start
	compensatedDataOld = baselines;

	// print to console
	fprintf(stderr, "\nBaselines:\n");
	for (int i=0; i<SKIN_DIM; i++) {
		if(!(i%12)) fprintf(stderr, "\n");
		fprintf(stderr,"%4.1f ", baselines[i]);		
	}

	/*fprintf(stderr, "\nStandard dev:\n");
	for (int i=0; i<SKIN_DIM; i++) {
		if(!(i%12)) fprintf(stderr, "\n");
		fprintf(stderr,"%3.1f ", standard_dev[i]);
	}*/

	fprintf(stderr,"\nThresholds (95 percentile):\n");
	for (int i=0; i<SKIN_DIM; i++) {
		if(!(i%12)) fprintf(stderr, "\n");
		fprintf(stderr,"%3.1f ", touchThresholds[i]);		
	}
	fprintf(stderr,"\n");
}

bool CompensationThread::readRawAndWriteCompensatedData(){
	Vector rawData;				// raw tactile data
	if(tactileSensor->read(rawData)!=IAnalogSensor::AS_OK){
        readErrorCounter++;
		return false;
	}
	readErrorCounter = 0;    

	if(rawData.size() != SKIN_DIM){
		fprintf(stderr, "Unexpected size of the input array (raw tactile data): %d\n", rawData.size());
		return false;
	}
	Vector& compensatedData2Send = compensatedTactileDataPort.prepare();
    compensatedData2Send.clear();
	compensatedData.clear();
	
	double d;
	for(int i=0; i<SKIN_DIM; i++){
		// baseline compensation
		if( zeroUpRawData == false){
			d = (double)( MAX_SKIN - rawData(i) - baselines[i]);
		}else{
			d = (double)(rawData(i) - baselines[i]);
		}
		compensatedData.push_back(d);	// save the data before applying filtering

		// smooth filter
		if(smoothFilter){
			smoothFactorSem.wait();
			d = smoothFactor*d + (1-smoothFactor)*compensatedDataOld(i);
			smoothFactorSem.post();
			compensatedDataOld(i) = d;	// update old value
		}
		
        
		if(d<0) 
			d=0;		

		// binarization filter
		if(d > touchThresholds[i] + ADD_THRESHOLD){
			touchDetected[i] = true;
			if(binarization)
				d = BIN_TOUCH;
		}
		else{
			touchDetected[i] = false;
			if(binarization)
				d = BIN_NO_TOUCH;
		}

		compensatedData2Send.push_back(d);
	}

	if(compensatedData2Send.size() != SKIN_DIM){
		fprintf(stderr, "Unexpected size of the output array (compensated tactile data): %d\n", compensatedData2Send.size());
		return false;
	}

	compensatedTactileDataPort.write();
	return true;
}

void CompensationThread::updateBaseline(){
	float mean_change = 0;
    int non_touching_taxels = 0;
	double d; 

    for(int j=0; j<SKIN_DIM; j++) {
        if(!touchDetected[j]){
			non_touching_taxels++;										//for changing the taxels where we detected touch
			d = compensatedData(j);

			if(d > 0.5) {
				baselines[j]		+= CHANGE_PER_TIMESTEP;
				mean_change			+= CHANGE_PER_TIMESTEP;				//for changing the taxels where we detected touch
			}else if(d < -0.5) {
				baselines[j]		-= CHANGE_PER_TIMESTEP;
				mean_change			-= CHANGE_PER_TIMESTEP;				//for changing the taxels where we detected touch
			}
		}
    }
    
    //for changing the taxels where we detected touch
    if (non_touching_taxels > 0)
        mean_change /= non_touching_taxels;
    for(int j=0; j<SKIN_DIM; j++) {
        if (touchDetected[j]) {
            baselines[j]		+= mean_change;
        }
    }
}

bool CompensationThread::doesBaselineExceed(){
	for(int i=0; i<SKIN_DIM; i++){
		if(baselines[i]<(*minBaseline) || baselines[i]>MAX_SKIN-(*minBaseline)){
			fprintf(stderr, "Baseline %d exceeds: %f\n", i, baselines[i]);
			return true;
        }
	}
	return false;
}



void CompensationThread::threadRelease() 
{
	/* close device driver and port */
	if(tactileSensorDevice)
		tactileSensorDevice->close();

	compensatedTactileDataPort.interrupt();
	compensatedTactileDataPort.close();
}

Vector CompensationThread::getTouchThreshold(){
	touchThresholdSem.wait();
	Vector res = touchThresholds;
	touchThresholdSem.post();
	return res;
}

bool CompensationThread::getBinarization(){
	return binarization;
}
bool CompensationThread::getSmoothFilter(){
	return smoothFilter;
}
float CompensationThread::getSmoothFactor(){
	return smoothFactor;
}
void CompensationThread::log(string s, bool endLine){
	cout << "[SKIN DRIFT COMP THREAD]: " << s;
	if(endLine)
		cout << endl;
}
