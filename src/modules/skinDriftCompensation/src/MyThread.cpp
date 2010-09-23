//#include <iomanip>			// io manipulator (setw, setfill)
//#include <sstream>			// string stream
//#include <fstream>			// ifstream, ofstream, fstream

#include <yarp/os/Time.h>

#include "iCub/MyThread.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

MyThread::MyThread(BufferedPort<Vector>* compensatedTactileDataPort, string robotName, float* minBaseline, 
				   bool *calibrationAllowed, bool *forceCalibration, bool *zeroUpRawData, bool* rightHand)
{
   this->compensatedTactileDataPort		= compensatedTactileDataPort;
   this->robotName						= robotName;
   this->minBaseline					= minBaseline;
   this->calibrationAllowed				= calibrationAllowed;
   this->forceCalibration				= forceCalibration;
   this->zeroUpRawData					= zeroUpRawData;
   this->rightHand						= rightHand;
}

bool MyThread::threadInit() 
{
   /* initialize variables and create data-structures if needed */
	MAX_DRIFT = 0.1;								// the maximal drift that is being compensated every second
	CHANGE_PER_TIMESTEP = MAX_DRIFT/FREQUENCY;

	
	//This is the tricky part: you should already have something like the following code this in your module
	//you have to create a new “options” distinguished from the one that you already have and create
	//a new Polydriver that use these new options. I’m not sure if the following three options are
	//enough or you need additional parameters. Try to compare it with the code that opens the motor interface in your code.
	Property options;
	options.put("robot",  "icub");
	if(*rightHand){
		options.put("part",   "righthand");         //skin part that you want to control
		options.put("local",  "/skinComp/right");
		options.put("remote",  ("/"+robotName+"/skin/righthand").c_str());
	}else{
		options.put("part",   "lefthand");          //skin part that you want to control
		options.put("local",  "/skinComp/left");
		options.put("remote",  ("/"+robotName+"/skin/lefthand").c_str());
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
	
	return true;
}

void MyThread::run(){
	// First of all, a big and a small calibration are executed, assuming that the sensors are not in contact with anything. 
	// The big calibration is performed by sending a message to the icubInterface. 
	// The small calibration gathers the sensors data for 5 sec, computes the mean (that is the baseline) 
	// and the 95 percentile (that will be used as touch threshold) for every taxel.
	runCalibration();

	//Then it starts cycling.
	while (!isStopping()) { // the thread continues to run until isStopping() returns true		
		//It reads the raw data 
		// computes the difference between the read values and the touch threshold (95percentile) and outputs these values
		readRawAndWriteCompensatedData();

		//If the read values are under the touch threshold 
		if(!touchDetected){
			//then the touch threshold is updated
			updateBaselineAndThreshold();
		}

		//If calibration is allowed AND at least one baseline is less than minBaseline (or greater than 255-minBaseline)
		if(*calibrationAllowed && doesBaselineExceed()){
			// then the big and small calibrations are executed
			runCalibration();
		}else if(*forceCalibration){
			runCalibration();
			*forceCalibration = false;
		}
	}
}

void MyThread::runCalibration(){
	// send a command to the microcontroller for calibrating the skin sensors
	tactileSensor->calibrateSensor();

	Vector input;
	tactileSensor->read(input);
    fprintf(stderr,"First Input:\n");
    for (int i=0; i<SKIN_DIM; i++) {
    	fprintf(stderr,"%f ", input[i]);
    }


	//collect skin data for some time, and compute the 95% percentile
	float start_sum[SKIN_DIM];
    int skin_empty[SKIN_DIM][MAX_SKIN+1];
    for (int i=0; i<SKIN_DIM; i++) {
        start_sum[i] = 0;
		for (int j=0; j<MAX_SKIN+1; j++) {
    		skin_empty[i][j] = 0;
    	}
    }
	//collect data
	for (int i=0; i<CAL_TIME*FREQUENCY; i++) {
        int retV = tactileSensor->read(input);
    	fprintf(stderr,"Return value: %d\t", retV);
		if (true || retV==0) {
            fprintf(stderr,"Input:\n");
            for (int i=0; i<SKIN_DIM; i++) {
            	fprintf(stderr,"%f ", input[i]);
            }
			Vector skin_values;
			skin_values.resize(SKIN_DIM);
			for (int j=0; j<SKIN_DIM; j++) {
				if(*zeroUpRawData)
					skin_values[j] = input(j);
				else
					skin_values[j] = MAX_SKIN - input(j);
				skin_empty[j][int(skin_values[j])]++;
                start_sum[j] += int(skin_values[j]);
			}
		}
		Time::delay((float)1/FREQUENCY);
	}

	//get percentile
    for (int i=0; i<SKIN_DIM; i++) {
        //avg start value
		baselines[i] = start_sum[i]/(CAL_TIME*FREQUENCY);
		
		//cumulative values
		for (int j=1; j<=MAX_SKIN; j++) {
			skin_empty[i][j] += skin_empty[i][j-1] ;
		}
		//when do we cross the treshhold?
    	for (int j=0; j<=MAX_SKIN; j++) {
			if (skin_empty[i][j] > (CAL_TIME*FREQUENCY*0.95)) {
				touchThresholds[i] = j;
				j = MAX_SKIN;
			}
    	}
    }
	//printf
    fprintf(stderr, "\nBaselines:\n");
	for (int i=0; i<SKIN_DIM; i++) {
    	fprintf(stderr,"%f ", baselines[i]);
    }
    fprintf(stderr,"\nThresholds:\n");
	for (int i=0; i<SKIN_DIM; i++) {
    	fprintf(stderr,"%f ", touchThresholds[i]);
    }
    fprintf(stderr,"\n");
}

void MyThread::readRawAndWriteCompensatedData(){
	tactileSensor->read(rawData);
	compensatedData = compensatedTactileDataPort->prepare();
	
	float d;
	for(int i=0; i<SKIN_DIM; i++){		
		if(! *zeroUpRawData){
			d = MAX_SKIN - rawData(i) - touchThresholds[i];
		}else{
			d = rawData(i) - touchThresholds[i];
		}
		compensatedData.push_back(d);

		if(d>0)
			touchDetected[i] = true;
		else
			touchDetected[i] = false;
	}

	compensatedTactileDataPort->write();
}

void MyThread::updateBaselineAndThreshold(){
	float mean_change = 0;
    int non_touching_taxels = 0;
	float d;

    for(int j=0; j<SKIN_DIM; j++) {
        if(!touchDetected[j]){
			non_touching_taxels++;										//for changing the taxels where we detected touch
			d = compensatedData(j);

			if(d > 0.5) {
				baselines[j]		-= CHANGE_PER_TIMESTEP;
				touchThresholds[j]	-= CHANGE_PER_TIMESTEP;
				mean_change			-= CHANGE_PER_TIMESTEP;						//for changing the taxels where we detected touch
			}else if(d < -0.5) {
				baselines[j]		+= CHANGE_PER_TIMESTEP;
				touchThresholds[j]	+= CHANGE_PER_TIMESTEP;
				mean_change			+= CHANGE_PER_TIMESTEP;						//for changing the taxels where we detected touch
			}
		}
    }
    
    //for changing the taxels where we detected touch
    if (non_touching_taxels > 0)
        mean_change /= non_touching_taxels;
    for(int j=0; j<SKIN_DIM; j++) {
        if (touchDetected[j]) {
            baselines[j]		+= mean_change;
			touchThresholds[j]	+= mean_change;
        }
    }
}

bool MyThread::doesBaselineExceed(){
	for(int i=0; i<SKIN_DIM; i++){
		if(baselines[i]<*minBaseline || baselines[i]>MAX_SKIN-(*minBaseline))
			return true;
	}
	return false;
}



void MyThread::threadRelease() 
{
   /* for example, delete dynamically created data-structures */
}


void MyThread::log(string s, bool endLine){
	cout << "[SKIN DRIFT COMP THREAD " << this->getKey() << "]: " << s;
	if(endLine)
		cout << endl;
}
