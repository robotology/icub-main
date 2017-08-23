
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
#include <list>
#include <fstream>//duarte code
#include <deque>

#include <yarp/sig/Vector.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Log.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/PolyDriver.h>

#include "iCub/skinDynLib/skinContact.h"
#include "iCub/skinDynLib/skinContactList.h"
#include "iCub/skinDynLib/rpcSkinManager.h"
#include "iCub/skinDynLib/common.h"

using namespace std;
using namespace yarp::os; 
using namespace yarp::sig;
using namespace yarp::dev;
using namespace iCub::skinDynLib;

namespace iCub{

namespace skinManager{    

class Compensator
{
private:
    /* class constants */
    static const int MAX_READ_ERROR = 100;      // max number of read errors before suspending the compensator
    static const int MAX_SKIN = 255;            // max value you can read from the skin sensors
    static const int MIN_TOUCH_THR = 1;         // min value assigned to the touch thresholds (i.e. the 95% percentile)
    static const double BIN_TOUCH;              // output value of the binarization filter when touch is detected
    static const double BIN_NO_TOUCH;           // output value of the binarization filter when no touch is detected
    
    // INIT
    unsigned int skinDim;                       // number of taxels (for the hand it is 192)
    string robotName;
    string name;                                // name of the compensator
    SkinPart skinPart;                          // id of the part of the skin (e.g. left_hand, right_forearm, left_upper_arm)
    BodyPart bodyPart;                          // id of the body part
    unsigned int linkNum;                       // number of the link
    string linkName;                            // name of the link to which the skin patch is attached
    string frameName;                           // name of the frame in which all skin quantities are expressed

    // SKIN CONTACTS
    vector< list<int> >     neighborsXtaxel;    // list of neighbors for each taxel    
    vector<Vector>          taxelPos;           // taxel positions {xPos, yPos, zPos}
    vector<Vector>          taxelOri;           // taxel normals {xOri, yOri, zOri}
    Vector                  taxelPoseConfidence;// taxels pose estimation confidence
    double                  maxNeighDist;       // max distance between two neighbor taxels
    Semaphore               poseSem;            // mutex to access taxel poses

    // COMPENSATION
    vector<bool> touchDetected;                 // true if touch has been detected in the last read of the taxel
    vector<bool> touchDetectedFilt;             // true if touch has been detected after applying the filtering
    vector<bool> subTouchDetected;              // true if the taxel value has gone under the baseline (because of touch in neighbouring taxels)
    Vector rawData;                             // data read from the skin
    Vector touchThresholds;                     // thresholds for discriminating between "touch" and "no touch"
    Semaphore touchThresholdSem;                // semaphore for controlling the access to the touchThreshold
    Vector initialBaselines;                    // mean of the raw tactile data computed during calibration
    Vector baselines;                           // mean of the raw tactile data
    Vector compensatedData;                     // compensated tactile data (that is rawData-touchThreshold)
    Vector compensatedDataOld;                  // compensated tactile data of the previous step (used for smoothing filter)
    Vector compensatedDataFilt;                 // compensated tactile data after smooth filter
    
    // CALIBRATION
    int calibrationRead;                        // count the calibration reads
    vector<float> start_sum;                    // sum of the values read during the calibration
    vector< vector<int> > skin_empty;           // distribution of the values read during the calibration
    
    // DEVICE
    IAnalogSensor* tactileSensor;               // interface for executing the tactile sensor calibration
    PolyDriver* tactileSensorDevice;

    // ERROR MANAGEMENT
    bool _isWorking;                            // true if the compensator is working fine
    int readErrorCounter;                       // it counts the number of successive errors
    vector<unsigned int> saturatedTaxels;       // list of all the taxels whose baseline exceeded

    // input parameters
    unsigned int addThreshold;          // value added to the touch threshold of every taxel    
    double compensationGain;            // proportional gain of the compensation algorithm
    double contactCompensationGain;     // proportional gain of the compensation algorithm during contact
    bool zeroUpRawData;                 // if true the raw data are considered from zero up, otherwise from 255 down
    float minBaseline;                  // min baseline value regarded as "safe"
    bool binarization;                  // if true binarize the compensated output value (0: no touch, 255: touch)
    bool smoothFilter;                  // if true the smooth filter is on, otherwise it is off
    float smoothFactor;                 // intensity of the smooth filter action
    Semaphore smoothFactorSem;

    /* ports */
    BufferedPort<Vector> compensatedTactileDataPort;    // output port
    BufferedPort<Bottle>* infoPort;                     // info output port
    BufferedPort<Vector> inputPort;
    Stamp timestamp;                                    // timestamp of last data read from inputPort

    
    /* class private methods */        
    bool init(string name, string robotName, string outputPortName, string inputPortName);
    bool readInputData(Vector& skin_values);
    void sendInfoMsg(string msg);
    void computeNeighbors();
    void updateNeighbors(unsigned int taxelId);

    /* class methods */
public:
    Compensator(string name, string robotName, string outputPortName, string inputPortName, BufferedPort<Bottle>* _infoPort,
                         double _compensationGain, double _contactCompensationGain, int addThreshold, float _minBaseline, bool _zeroUpRawData, 
                         bool _binarization, bool _smoothFilter, float _smoothFactor, unsigned int _linkId = 0);
    ~Compensator();
        
    void calibrationInit();
    void calibrationDataCollection();
    void calibrationFinish();
    bool readRawAndWriteCompensatedData();
    void updateBaseline();
    bool doesBaselineExceed(unsigned int &taxelIndex, double &baseline, double &initialBaseline);
    skinContactList getContacts();
    bool isWorking(){ return _isWorking; }

    void setBinarization(bool value){ binarization = value; }
    void setSmoothFilter(bool value);
    bool setSmoothFactor(float value);    
    bool setAddThreshold(unsigned int thr);
    bool setCompensationGain(double gain);
    bool setContactCompensationGain(double gain);
    bool setMaxNeighborDistance(double d);
    bool setTaxelPosesFromFile(const char *filePath);
    bool setTaxelPosesFromFileOld(const char *filePath);
    bool setTaxelPoses(const vector<Vector> &poses);
    bool setTaxelPose(unsigned int taxelId, const Vector &pose);
    bool setTaxelPositions(const Vector &positions);
    bool setTaxelPosition(unsigned int taxelId, const Vector &position);
    bool setTaxelOrientations(const vector<Vector> &orientations);
    bool setTaxelOrientation(unsigned int taxelId, const Vector &orientation);
    void setSkinPart(SkinPart _skinPart);

    Vector getTouchThreshold();
    bool getBinarization(){     return binarization; }
    bool getSmoothFilter(){     return smoothFilter; }
    float getSmoothFactor();
    unsigned int getAddThreshold(){         return addThreshold; }
    double getCompensationGain(){           return compensationGain; }
    double getContactCompensationGain(){    return contactCompensationGain; }
    Vector getTaxelPosition(unsigned int taxelId);
    vector<Vector> getTaxelPositions();
    Vector getTaxelOrientation(unsigned int taxelId);
    vector<Vector> getTaxelOrientations();
    Vector getTaxelPose(unsigned int taxelId);
    vector<Vector> getTaxelPoses();
    double getPoseConfidence(unsigned int taxelId);
    Vector getPoseConfidences();
    unsigned int getNumTaxels();
    Vector getCompensation();
    Vector getBaselines(){      return baselines; }
    Vector getRawData(){        return rawData; }
    Vector getCompData(){       return compensatedData; }
    Stamp getTimestamp(){       return timestamp; }
    
    string getName(){           return name; }
    string getInputPortName(){  return tactileSensorDevice->getValue("remote").asString().c_str(); }
    string getSkinPartName(){   return SkinPart_s[skinPart]; }
    SkinPart getSkinPart(){     return skinPart; }
    string getBodyPartName(){   return BodyPart_s[bodyPart]; }
    BodyPart getBodyPart(){     return bodyPart; }
    unsigned int getLinkNum(){  return linkNum; }

};

template <class T>
inline std::string toString(const T& t){
    std::stringstream ss;
    ss << t;
    return ss.str();
}

} //namespace iCub

} //namespace skinManager

#endif

