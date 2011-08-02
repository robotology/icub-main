
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
#ifndef __COMP_H__
#define __COMP_H__

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <fstream>//duarte code

#include <yarp/sig/Vector.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/PolyDriver.h>

using namespace std;
using namespace yarp::os; 
using namespace yarp::sig;
using namespace yarp::dev;

namespace iCub{

namespace skinDriftCompensation{    

class Compensator
{
	/* class methods */
public:
	Compensator(string name, string robotName, string outputPortName, string inputPortName, BufferedPort<Bottle>* _infoPort,
                         double _compensationGain, int addThreshold, float _minBaseline, bool _zeroUpRawData, bool _binarization, 
                         bool _smoothFilter, float _smoothFactor, unsigned int _linkId = 0);
    ~Compensator();
	    
	void calibrationInit();
	void calibrationDataCollection();
	void calibrationFinish();
    bool readRawAndWriteCompensatedData();
	void updateBaseline();
	bool doesBaselineExceed(unsigned int &taxelIndex, double &baseline, double &initialBaseline);
    bool isThereContact();
    bool isWorking();

	void setBinarization(bool value);
	void setSmoothFilter(bool value);
	bool setSmoothFactor(float value);
    void setLinkId(unsigned int linkId);
    bool setAddThreshold(unsigned int thr);
    bool setCompensationGain(double gain);
    bool setTaxelPositions(const char *filePath);

	Vector getTouchThreshold();
	bool getBinarization();
	bool getSmoothFilter();
	float getSmoothFactor();
    unsigned int getLinkId();
    unsigned int getAddThreshold();
    double getCompensationGain();    
    unsigned int getNumTaxels();
    Vector getCompensation();
    string getName();
    string getInputPortName();    
	Vector getContactCOP(); // get the contact center of pressure
    string getPartName();    

private:

	/* class constants */	
    static const int MAX_READ_ERROR = 100;		// max number of read errors before suspending the compensator
	static const int MAX_SKIN = 255;			// max value you can read from the skin sensors
    static const int MIN_TOUCH_THR = 1;			// min value assigned to the touch thresholds (i.e. the 95% percentile)
	static const int BIN_TOUCH = 100;			// output value of the binarization filter when touch is detected
	static const int BIN_NO_TOUCH = 0;			// output value of the binarization filter when no touch is detected
	
	unsigned int ADD_THRESHOLD;					// value added to the touch threshold of every taxel	
	unsigned int SKIN_DIM;						// number of taxels (for the hand it is 192)
	double compensationGain;					// the maximal drift that is being compensated every cycle	
    string robotName;
    string name;                                // name of the compensator
    string partName;                            // name of the part of the skin
	double **taxelPosOri;						//taxel position and orientation {xPos, yPos, zPos, xOri, yOri, zOri}

	/* class variables */
	vector<bool> touchDetected;					// true if touch has been detected in the last read of the taxel
	vector<bool> touchDetectedFilt;             // true if touch has been detected after applying the filtering
    Vector touchThresholds;						// threshold for discriminating between "touch" and "no touch"
	Semaphore touchThresholdSem;				// semaphore for controlling the access to the touchThreshold
    Vector initialBaselines;					// mean of the raw tactile data computed during calibration
    Vector baselines;							// mean of the raw tactile data 
												// (considering only the samples read when no touch is detected)
	
	// calibration variables
    int calibrationRead;						// count the calibration reads
	vector<float> start_sum;					// sum of the values read during the calibration
    vector< vector<int> > skin_empty;			// distribution of the values read during the calibration
	
	int readErrorCounter;						// it counts the number of successive errors
    Vector compensatedData;			    		// compensated tactile data (that is rawData-touchThreshold)
	Vector compensatedDataOld;			    	// compensated tactile data of the previous step (used for smoothing filter)
	IAnalogSensor* tactileSensor;	        	// interface for executing the tactile sensor calibration
	PolyDriver* tactileSensorDevice;

    bool _isWorking;                            // true if the compensator is working fine
    vector<unsigned int> saturatedTaxels;       // list of all the taxels whose baseline exceeded

    // input parameters
	bool zeroUpRawData;				// if true the raw data are considered from zero up, otherwise from 255 down
    float minBaseline;				// min baseline value regarded as "safe"
	bool binarization;				// if true binarize the compensated output value (0: no touch, 255: touch)
	bool smoothFilter;				// if true the smooth filter is on, otherwise it is off
	float smoothFactor;				// intensity of the smooth filter action
	Semaphore smoothFactorSem;

    // SKIN EVENTS
    unsigned int linkId;

	/* ports */
	BufferedPort<Vector> compensatedTactileDataPort;	// output port
    BufferedPort<Bottle>* infoPort;					    // info output port

	
	/* class private methods */	    
    bool init(string name, string robotName, string outputPortName, string inputPortName);
    bool readInputData(Vector& skin_values);
    void sendInfoMsg(string msg);
	
};

} //namespace iCub

} //namespace skinDriftCompensation

#endif