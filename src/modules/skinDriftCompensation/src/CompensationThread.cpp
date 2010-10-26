
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


CompensationThread::CompensationThread(ResourceFinder* rf, string robotName, float* minBaseline, 
									   bool *calibrationAllowed, bool *forceCalibration, bool zeroUpRawData, bool rightHand)
									   : RateThread(FREQUENCY)
{
   this->rf								= rf;
   this->robotName						= robotName;
   this->minBaseline					= minBaseline;
   this->calibrationAllowed				= calibrationAllowed;
   this->forceCalibration				= forceCalibration;
   this->zeroUpRawData					= zeroUpRawData;
   this->rightHand						= rightHand;
}

bool CompensationThread::threadInit() 
{
    fprintf(stderr, "THREAD INIT\n\n");

   /* initialize variables and create data-structures if needed */
	MAX_DRIFT = 0.1f;								// the maximal drift that is being compensated every second
	CHANGE_PER_TIMESTEP = MAX_DRIFT/FREQUENCY;
    *forceCalibration = false;

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

	// temporary
    /*rawTactileDataPort = new BufferedPort<Vector>();
	if(*rightHand){
        rawTactileDataPort->open(("/"+ robotName+ "/skin_comp/right_hand:i").c_str());
		Network::connect(("/"+robotName+"/skin/right_hand").c_str(), rawTactileDataPort->getName().c_str());
	}
	else{
		rawTactileDataPort->open(("/"+ robotName+ "/skin_comp/left_hand:i").c_str());
		Network::connect(("/"+robotName+"/skin/left_hand").c_str(), rawTactileDataPort->getName().c_str());
	}*/

	// First of all, a big and a small calibration are executed, assuming that the sensors are not in contact with anything. 
	// The big calibration is performed by sending a message to the icubInterface. 
	// The small calibration gathers the sensors data for 5 sec, computes the mean (that is the baseline) 
	// and the 95 percentile (that will be used as touch threshold) for every taxel.
	runCalibration();

	return true;
}

void CompensationThread::run(){	

	// It reads the raw data,
	// computes the difference between the read values and the baseline and outputs these values
	if( !readRawAndWriteCompensatedData() )
		return;

	//If the read values are under the touch threshold 
	if(!globalTouchDetected){
		//then the baseline is updated
		updateBaseline();
	}

	//If calibration is allowed AND at least one baseline is less than minBaseline (or greater than 255-minBaseline)
	if( (*calibrationAllowed)==true && doesBaselineExceed()){
		// then the big and small calibrations are executed
		runCalibration();
	}else if( (*forceCalibration) == true){
		runCalibration();
		*forceCalibration = false;
	}
}

void CompensationThread::runCalibration(){
    fprintf(stderr, "CALIBRATING.......................");
	// take the semaphore so that the touchThreshold can't be read during the calibration phase
	touchThresholdSem.wait();

	// send a command to the microcontroller for calibrating the skin sensors
	if(robotName!="icubSim")	// this feature isn't implemented in the simulator and causes a runtime error
		tactileSensor->calibrateSensor();

	Vector input;
	int err;
	if((err=tactileSensor->read(input)) != IAnalogSensor::AS_OK){
		fprintf(stderr, "ERROR while reading the sensor data. Error value: %d", err);
		this->suspend();
		return;
	}

	//input = *rawTactileDataPort->read();
    fprintf(stderr,"First Input:\n");
    for (int i=0; i<SKIN_DIM; i++) {
		if(!(i%12)) fprintf(stderr, "\n");
    	fprintf(stderr,"%3.0f ", input[i]);
    }


	//collect skin data for some time, and compute the 95% percentile
	vector<float> start_sum(SKIN_DIM, 0);
    vector< vector<int> > skin_empty(SKIN_DIM, vector<int>(MAX_SKIN+1, 0));

	//collect data
	for (int i=0; i<CAL_TIME*FREQUENCY; i++) {
        int retV = tactileSensor->read(input);    	

		if (retV == IAnalogSensor::AS_OK) {            
			Vector skin_values(SKIN_DIM);
			for (int j=0; j<SKIN_DIM; j++) {
				if (zeroUpRawData==true)
					skin_values[j] = input[j];
				else{
					skin_values[j] = MAX_SKIN - input[j];
				}
				if(skin_values[j]<0 || skin_values[j]>MAX_SKIN){
					fprintf(stderr, "Error while reading the tactile data! Data out of range: %d\n", (int)skin_values[j]);
				}
				else{
					skin_empty[j][int(skin_values[j])]++;
					start_sum[j] += int(skin_values[j]);
				}
			}
			/*fprintf(stderr,"Input:\n");
            for (int i=0; i<SKIN_DIM; i++) {
            	fprintf(stderr,"%3.0f ", skin_values[i]);
            }*/
		}
		else{
			fprintf(stderr, "Error while reading the tactile data! Error code: %d\n", retV);
		}
		Time::delay((float)1/FREQUENCY);
	}

	vector<float> standard_dev(SKIN_DIM, 0);
	//get percentile
    for (int i=0; i<SKIN_DIM; i++) {
        //avg start value
		baselines[i] = start_sum[i]/(CAL_TIME*FREQUENCY);
		
		//cumulative values
		for (int j=1; j<=MAX_SKIN; j++) {
			standard_dev[i] += fabs(j-baselines[i]) * skin_empty[i][j];
			skin_empty[i][j] += skin_empty[i][j-1] ;			
		}
		standard_dev[i] /= (CAL_TIME*FREQUENCY);

		//when do we cross the threshold?
    	for (int j=0; j<=MAX_SKIN; j++) {
			if (skin_empty[i][j] > (CAL_TIME*FREQUENCY*0.95)) {
				touchThresholds[i] = max<float>(0.0, (float)j - baselines[i]);	// the threshold can not be less than zero
				j = MAX_SKIN;
			}
    	}
    }
	
	// release the semaphore so that as of now the touchThreshold can be read
	touchThresholdSem.post();

	//printf
    fprintf(stderr, "\nBaselines:\n");
	for (int i=0; i<SKIN_DIM; i++) {
		if(!(i%12)) fprintf(stderr, "\n");
    	fprintf(stderr,"%4.1f ", baselines[i]);		
    }

	fprintf(stderr, "\nStandard dev:\n");
	for (int i=0; i<SKIN_DIM; i++) {
		if(!(i%12)) fprintf(stderr, "\n");
    	fprintf(stderr,"%3.1f ", standard_dev[i]);
    }

    fprintf(stderr,"\nThresholds (95 percentile):\n");
	for (int i=0; i<SKIN_DIM; i++) {
		if(!(i%12)) fprintf(stderr, "\n");
    	fprintf(stderr,"%3.1f ", touchThresholds[i]);		
    }
    fprintf(stderr,"\n");
}

bool CompensationThread::readRawAndWriteCompensatedData(){
	globalTouchDetected = false;
	int res = tactileSensor->read(rawData);
	if(res != IAnalogSensor::AS_OK){
		fprintf(stderr, "ERROR while reading the sensor data. Error value: %d", res);
		this->suspend();
		return false;
	}
	//rawData = *(rawTactileDataPort->read());
	if(rawData.size() != SKIN_DIM){
		fprintf(stderr, "Unexpected size of the input array (raw tactile data): %d\n", rawData.size());
		return false;
	}
	Vector& compensatedData2 = compensatedTactileDataPort.prepare();
    compensatedData2.clear();
	
	float d;
	for(int i=0; i<SKIN_DIM; i++){		
		/*if( zeroUpRawData == false){
			d = (float)( MAX_SKIN - rawData(i) - baselines[i] - touchThresholds[i]);
		}else{
			d = (float)(rawData(i) - baselines[i] - touchThresholds[i]);
		}*/

		if( zeroUpRawData == false){
			d = (float)( MAX_SKIN - rawData(i) - baselines[i]);
		}else{
			d = (float)(rawData(i) - baselines[i]);
		}
        
		if(d<0) 
			d=0;

		compensatedData2.push_back((int)d);

		if(d > touchThresholds[i]){
		//if(d > 0){
			touchDetected[i] = true;
			globalTouchDetected = true;
		}
		else
			touchDetected[i] = false;
	}

	if(compensatedData2.size() != SKIN_DIM){
		fprintf(stderr, "Unexpected size of the output array (compensated tactile data): %d\n", compensatedData2.size());
		return false;
	}

	compensatedTactileDataPort.write();
    compensatedData = compensatedData2;
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
				baselines[j]		-= CHANGE_PER_TIMESTEP;
				mean_change			-= CHANGE_PER_TIMESTEP;				//for changing the taxels where we detected touch
			}else if(d < -0.5) {
				baselines[j]		+= CHANGE_PER_TIMESTEP;
				mean_change			+= CHANGE_PER_TIMESTEP;				//for changing the taxels where we detected touch
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

	/*if(rawTactileDataPort){
		rawTactileDataPort->interrupt();
		rawTactileDataPort->close();
	}*/
}

VectorOf<float> CompensationThread::getTouchThreshold(){
	touchThresholdSem.wait();
	VectorOf<float> res = touchThresholds;
	touchThresholdSem.post();
	return res;
}

void CompensationThread::log(string s, bool endLine){
	cout << "[SKIN DRIFT COMP THREAD]: " << s;
	if(endLine)
		cout << endl;
}
