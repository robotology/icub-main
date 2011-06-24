/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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

/** 
 * @defgroup SpringyFingers springyFingers
 *  
 * @ingroup PerceptiveModels 
 *  
 * Abstract layers for dealing with perceptive models framework.
 *
 * @author Ugo Pattacini 
 *  
 * Copyright (C) 2011 RobotCub Consortium
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0. 
 *
 * @section intro_sec Description
 *
 * ... 
 *  
 */ 

#ifndef __PERCEPTIVEMODELS_SPRINGYFINGERS_H__
#define __PERCEPTIVEMODELS_SPRINGYFINGERS_H__

#include <yarp/os/Value.h>
#include <yarp/os/Property.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>

#include <iCub/learningMachine/FixedRangeScaler.h>
#include <iCub/learningMachine/LSSVMLearner.h>
#include <iCub/perception/sensors.h>
#include <iCub/perception/nodes.h>
#include <iCub/perception/models.h>


namespace iCub
{

namespace perception
{

/**
* @ingroup SpringyFingers
*
*/
class SpringyFinger : public Node
{
protected:
    mutable iCub::learningmachine::FixedRangeScaler scaler;
    mutable iCub::learningmachine::LSSVMLearner     lssvm;
    double  calibratingVelocity;
    double  outputGain;
    bool    calibrated;

    bool extractSensorsData(yarp::sig::Vector &in, yarp::sig::Vector &out) const;

public:
    bool fromProperty(const yarp::os::Property &options);
    void toProperty(yarp::os::Property &options) const;
    bool calibrate(const yarp::os::Property &options);
    bool getSensorsData(yarp::os::Value &data) const;
    bool getOutput(yarp::os::Value &out) const;

    void   setCalibVel(const double vel) { calibratingVelocity=vel;    }
    double getCalibVel() const           { return calibratingVelocity; }
    bool   isCalibrated() const          { return calibrated;          }
};


/**
* @ingroup SpringyFingers
*
*/
class SpringyFingersModel : public virtual Model
{
private:
    std::string type;
    std::string robot;
    int verbose;

    SensorInterface sensIF[5];
    SensorPort      sensPort[12];
    SpringyFinger   fingers[5];
    bool configured;

    yarp::os::BufferedPort<yarp::os::Bottle> *port;
    yarp::dev::PolyDriver                     driver;

    int printMessage(const int level, const char *format, ...) const;
    void calibrateFinger(SpringyFinger &finger, const int joint, const yarp::sig::Vector &qmin,
                         const yarp::sig::Vector &qmax);
    void close();

public:
    SpringyFingersModel();

    bool fromProperty(const yarp::os::Property &options);
    void toProperty(yarp::os::Property &options) const;
    bool calibrate(const yarp::os::Property &options);
    bool getOutput(yarp::os::Value &out) const;

    bool isCalibrated() const;

    virtual ~SpringyFingersModel();
};


}

}

#endif


