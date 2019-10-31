 
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

#include <mutex>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include <yarp/sig/Vector.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/PolyDriver.h>

#include "iCub/skinManager/compensator.h"
#include "iCub/skinDynLib/skinContactList.h"

using namespace std;
using namespace yarp::os; 
using namespace yarp::sig;
using namespace yarp::dev;
using namespace iCub::skinDynLib;

namespace iCub{

namespace skinManager{

class CompensationThread : public PeriodicThread
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
    bool setMaxNeighborDistance(double dist);
    bool setTaxelPosition(SkinPart sp, unsigned int taxelId, const Vector &position);
    bool setTaxelPositions(SkinPart sp, const Vector &positions);
    bool setTaxelOrientation(SkinPart sp, unsigned int taxelId, const Vector &orientation);
    bool setTaxelOrientations(SkinPart sp, const vector<Vector> &orientations);
    bool setTaxelPose(SkinPart sp, unsigned int taxelId, const Vector &pose);
    bool setTaxelPoses(SkinPart sp, const vector<Vector> &poses);
    bool setTaxelPoses(SkinPart sp, const Vector &poses);

    Vector getTouchThreshold();
    bool getBinarization();
    bool getSmoothFilter();
    float getSmoothFactor();
    unsigned int getAddThreshold();
    double getCompensationGain();
    double getContactCompensationGain();
    double getMaxNeighborDistance(){ return maxNeighDist; }
    bool isCalibrating();

    Vector getTaxelPosition(SkinPart sp, unsigned int taxelId);
    vector<Vector> getTaxelPositions(SkinPart sp=SKIN_PART_ALL);
    Vector getTaxelOrientation(SkinPart sp, unsigned int taxelId);
    vector<Vector> getTaxelOrientations(SkinPart sp=SKIN_PART_ALL);
    Vector getTaxelPose(SkinPart sp, unsigned int taxelId);
    vector<Vector> getTaxelPoses(SkinPart sp=SKIN_PART_ALL);
    double getPoseConfidence(SkinPart sp, unsigned int taxelId);
    Vector getPoseConfidences(SkinPart sp);

    bool enableSkinPart(SkinPart sp);
    bool disableSkinPart(SkinPart sp);
    bool isSkinEnabled(SkinPart sp);
    vector<SkinPart> getSkinParts();
    Bottle getInfo();


private:

    /* class constants */
    static const int CAL_TIME = 5;              // calibration time in sec

    /* class variables */
    int CAL_SAMPLES;                            // num of samples needed for calibration
    int ADD_THRESHOLD;                          // value added to the touch threshold of every taxel
    unsigned int SKIN_DIM;                      // total number of taxels
    double compensationGain;                    // the gain of the compensation algorithm
    double contactCompensationGain;             // the gain of the compensation algorithm during contact
    
    bool initializationFinished;
    
    // calibration variables
    int calibrationCounter;                     // count the calibration cycles
    int readErrorCounter;                       // it counts the number of successive errors

    ResourceFinder* rf;

    // input parameters
    string moduleName;
    string robotName;
    float minBaseline;              // if the baseline value is less than this, then a calibration is executed (if allowed)
    bool zeroUpRawData;             // if true the raw data are considered from zero up, otherwise from 255 down
    bool binarization;              // if true binarize the compensated output value (0: no touch, 255: touch)
    bool smoothFilter;              // if true the smooth filter is on, otherwise it is off
    float smoothFactor;             // intensity of the smooth filter action
    double maxNeighDist;            // max neighbor distance used for computing taxel neighbors
    unsigned int portNum;           // number of input ports (that is the same as the number of output ports)

    vector<Compensator*> compensators;
    vector<bool> compEnable;            // true if the related compensator is enabled, false otherwise
    vector<bool> compWorking;           // true if the related compensator is working, false otherwise
    unsigned int compensatorCounter;    // count the number of compensators that are working 

    // SKIN EVENTS
    bool skinEventsOn;

    /* ports */
    BufferedPort<skinContactList> skinEventsPort;   // skin events output port
    BufferedPort<Vector> monitorPort;               // monitoring output port (streaming)
    BufferedPort<Bottle> infoPort;                  // info output port

    CompensationThreadState state;                  // state of the thread (calibration, compensation)
    mutex stateSem;

    /* class private methods */
    void checkErrors();
    bool doesBaselineExceed(unsigned int &compInd, unsigned int &taxInd, double &baseline, double &initialBaseline);
    void sendMonitorData();
    void sendDebugMsg(string msg);
    void sendErrorMsg(string msg);
    void sendSkinEvents();

};

} //namespace iCub

} //namespace skinManager

#endif

