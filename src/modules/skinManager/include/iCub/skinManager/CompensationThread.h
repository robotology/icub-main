 
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

#ifndef __COMPTHREAD_H__
#define __COMPTHREAD_H__

#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include <yarp/sig/Vector.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/PolyDriver.h>

#include "iCub/skinManager/Compensator.h"
#include "iCub/skinDynLib/skinContactList.h"

using namespace std;
using namespace yarp::os; 
using namespace yarp::sig;
using namespace yarp::dev;
using namespace iCub::skinDynLib;

namespace iCub{

namespace skinManager{

class CompensationThread : public RateThread
{
public:
	typedef enum { calibration, compensation} CompensationThreadState;

	/* class methods */

	CompensationThread(string name, ResourceFinder* rf, string robotName, double _compensationGain, double _contactCompensationGain,
        int addThreshold, float minBaseline, bool zeroUpRawData, int period, bool binarization, bool smoothFilter, float smoothFactor);
	bool threadInit();
	void threadRelease();
	void run(); 
	
	void calibrate();
	void setBinarization(bool value);
	void setSmoothFilter(bool value);
	bool setSmoothFactor(float value);
    bool setAddThreshold(unsigned int thr);
    bool setCompensationGain(double gain);
    bool setContactCompensationGain(double gain);
    bool setTaxelPosition(BodyPart bp, SkinPart sp, unsigned int taxelId, Vector position);
    bool setTaxelPositions(BodyPart bp, SkinPart sp, vector<Vector> positions);
    bool setTaxelOrientation(BodyPart bp, SkinPart sp, unsigned int taxelId, Vector orientation);
    bool setTaxelOrientations(BodyPart bp, SkinPart sp, vector<Vector> orientations);
    bool setTaxelPose(BodyPart bp, SkinPart sp, unsigned int taxelId, Vector pose);
    bool setTaxelPoses(BodyPart bp, SkinPart sp, vector<Vector> poses);
    bool setTaxelPoses(BodyPart bp, SkinPart sp, Vector poses);

	Vector getTouchThreshold();
	bool getBinarization();
	bool getSmoothFilter();
	float getSmoothFactor();
    unsigned int getAddThreshold();
    double getCompensationGain();
    double getContactCompensationGain();
	bool isCalibrating();
    Vector getTaxelPosition(BodyPart bp, SkinPart sp, unsigned int taxelId);
    vector<Vector> getTaxelPositions(BodyPart bp, SkinPart sp);
    vector<Vector> getTaxelPositions(BodyPart bp);
    vector<Vector> getTaxelPositions();
    Vector getTaxelOrientation(BodyPart bp, SkinPart sp, unsigned int taxelId);
    vector<Vector> getTaxelOrientations(BodyPart bp, SkinPart sp);
    vector<Vector> getTaxelOrientations(BodyPart bp);
    vector<Vector> getTaxelOrientations();
    Vector getTaxelPose(BodyPart bp, SkinPart sp, unsigned int taxelId);
    vector<Vector> getTaxelPoses(BodyPart bp, SkinPart sp);
    vector<Vector> getTaxelPoses(BodyPart bp);
    vector<Vector> getTaxelPoses();
    Bottle getInfo();


private:

	/* class constants */	
	static const int CAL_TIME = 5;				// calibration time in sec	

    /* class variables */
    int CAL_SAMPLES;                            // num of samples needed for calibration
	int ADD_THRESHOLD;							// value added to the touch threshold of every taxel	
	unsigned int SKIN_DIM;						// total number of taxels
	double compensationGain;				    // the gain of the compensation algorithm
    double contactCompensationGain;				// the gain of the compensation algorithm during contact
	
    bool initializationFinished;
	
	// calibration variables
	int calibrationCounter;						// count the calibration cycles	
	int readErrorCounter;						// it counts the number of successive errors

	ResourceFinder* rf;

	// input parameters
	string moduleName;
	string robotName;
	float minBaseline;				// if the baseline value is less than this, then a calibration is executed (if allowed)
	bool zeroUpRawData;				// if true the raw data are considered from zero up, otherwise from 255 down
	bool binarization;				// if true binarize the compensated output value (0: no touch, 255: touch)
	bool smoothFilter;				// if true the smooth filter is on, otherwise it is off
	float smoothFactor;				// intensity of the smooth filter action
	unsigned int portNum;			// number of input ports (that is the same as the number of output ports)

    vector<Compensator*> compensators;
    vector<bool> compWorking;           // true if the relative compensator is working, false otherwise
    unsigned int compensatorCounter;    // count the number of compensators that are working 

    // SKIN EVENTS
    bool skinEventsOn;

	/* ports */
    BufferedPort<skinContactList> skinEventsPort;   // skin events output port
	BufferedPort<Vector> monitorPort;				// monitoring output port (streaming)
    BufferedPort<Bottle> infoPort;					// info output port

	CompensationThreadState state;			// state of the thread (calibration, compensation)
	Semaphore stateSem;

	/* class private methods */	
    void checkErrors();
    bool doesBaselineExceed(unsigned int &compInd, unsigned int &taxInd, double &baseline, double &initialBaseline);
	void sendMonitorData();
    void sendInfoMsg(string msg);
    void sendSkinEvents();

};

} //namespace iCub

} //namespace skinManager

#endif

