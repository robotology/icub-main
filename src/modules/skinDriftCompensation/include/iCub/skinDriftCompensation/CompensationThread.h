
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

#include <iostream>
#include <string>
#include <vector>

#include <yarp/sig/Vector.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/PolyDriver.h>

#include <yarp/os/Network.h>	//temporary

using namespace std;
using namespace yarp::os; 
using namespace yarp::sig;
using namespace yarp::dev;

namespace iCub{

namespace skinDriftCompensation{

class CompensationThread : public RateThread
{
private:

	/* class constants */	
	static const int MAX_SKIN = 255;			// max value you can read from the skin sensors
	static const int CAL_TIME = 5;				// calibration time in sec
	static const int ADD_THRESHOLD = 2;			// value added to the threshold of every taxel 
	const int PERIOD;
	static const int ADD_THRESHOLD = 2;			// value added to the touch threshold of every taxel
	int SKIN_DIM;								// number of taxels in one hand (192)
	float MAX_DRIFT;							// the maximal drift that is being compensated every second
	float CHANGE_PER_TIMESTEP;					// the maximal drift that is being compensated every cycle

	/* class variables */
	vector<bool> touchDetected;					// true if touch has been detected in the last read of the taxel
	VectorOf<float> touchThresholds;			// threshold for discriminating between "touch" and "no touch"
	Semaphore touchThresholdSem;				// semaphore for controlling the access to the touchThreshold
    VectorOf<float> baselines;					// mean of the raw tactile data 
												// (considering only the samples read when no touch is detected)
	Vector rawData;								// raw tactile data
	Vector compensatedData;			    		// compensated tactile data (that is rawData-touchThreshold)
	IAnalogSensor *tactileSensor;				// interface for executing the tactile sensor calibration
	PolyDriver* tactileSensorDevice;

	ResourceFinder* rf;

	// input parameters
	string robotName;
	float *minBaseline;				// if the baseline value is less than this, then a calibration is executed (if allowed)
	bool *calibrationAllowed;		// if false the thread is not allowed to run the calibration
	bool *forceCalibration;			// if true a calibration is executed as soon as possible; 
									// after that the variable is set to false
	bool zeroUpRawData;				// if true the raw data are considered from zero up, otherwise from 255 down
	bool rightHand;					// if true then calibrate the right hand, otherwise the left hand

	/* ports */
	BufferedPort<Vector> compensatedTactileDataPort;	// output port

	/* class private methods */
	void log(string s, bool endLine=true);
	
	void runCalibration();
	bool readRawAndWriteCompensatedData();
	void updateBaseline();
	bool doesBaselineExceed();

public:

	/* class methods */

	CompensationThread(ResourceFinder* rf, string robotName, float* minBaseline, bool *calibrationAllowed, 
		bool *forceCalibration, bool zeroUpRawData, bool rightHand, int period);
	bool threadInit();
	void threadRelease();
	void run(); 
	VectorOf<float> getTouchThreshold();
};

} //namespace iCub

} //namespace skinDriftCompensation
