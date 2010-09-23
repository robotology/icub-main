#include <iostream>
#include <string>

#include <yarp/sig/Vector.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Thread.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/PolyDriver.h>

#include <yarp/os/Network.h>	//temporary

using namespace std;
using namespace yarp::os; 
using namespace yarp::sig;
using namespace yarp::dev;

class MyThread : public Thread
{
private:
	/* class constants */
	static const int SKIN_DIM = 48;				// number of taxels in one hand (192)
	static const int MAX_SKIN = 255;			// max value you can read from the skin sensors
	static const int FREQUENCY = 50;			// how many measurements every second?
	static const int CAL_TIME = 5;				// calibration time in sec	
	float MAX_DRIFT;							// the maximal drift that is being compensated every second
	float CHANGE_PER_TIMESTEP;					// the maximal drift that is being compensated every cycle

   /* class variables */
	bool touchDetected[SKIN_DIM];				// true if touch has been detected in the last read of the taxel
	float touchThresholds[SKIN_DIM];			// threshold for discriminating between "touch" and "no touch"
    float baselines[SKIN_DIM];					// mean of the raw tactile data 
												// (considering only the samples read when no touch is detected)
	Vector rawData;								// raw tactile data
	Vector compensatedData;						// compensated tactile data (that is rawData-touchThreshold)
	IAnalogSensor *tactileSensor;				// interface for executing the tactile sensor calibration
	PolyDriver* tactileSensorDevice;

	// input parameters
	string robotName;
	float *minBaseline;				// if the baseline value is less than this, then a calibration is executed (if allowed)
	bool *calibrationAllowed;		// if false the thread is not allowed to run the calibration
	bool *forceCalibration;			// if true a calibration is executed as soon as possible; 
									// after that the variable is set to false
	bool *zeroUpRawData;			// if true the raw data are considered from zero up, otherwise from 255 down
	bool *rightHand;				// if true then calibrate the right hand, otherwise the left hand

	/* ports */
	BufferedPort<Vector>* rawTactileDataPort;
	BufferedPort<Vector>* compensatedTactileDataPort;

	/* class private methods */
	void log(string s, bool endLine=true);
	
	void runCalibration();
	void readRawAndWriteCompensatedData();
	void updateBaselineAndThreshold();
	bool doesBaselineExceed();

public:

	/* class methods */

	MyThread(BufferedPort<Vector>* compensatedTactileDataPort, string robotName, 
		float* minBaseline, bool *calibrationAllowed, bool *forceCalibration, bool *zeroUpRawData, bool *rightHand);
	bool threadInit();     
	void threadRelease();
	void run(); 
};