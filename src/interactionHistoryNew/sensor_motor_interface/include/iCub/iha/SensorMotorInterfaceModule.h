// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
// vim:expandtab:tabstop=4:shiftwidth=4:softtabstop=4:

/*
 * Copyright (C) 2008 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Assif Mirza
 * email:   assif.mirza@robotcub.org
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

#ifndef __SENSOR_MOTOR_INTERFACE_MODULE_H__
#define __SENSOR_MOTOR_INTERFACE_MODULE_H__

#include <stdio.h>
#include <string>
#include <iostream>

#include <yarp/String.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <iCub/iha/debug.h>

#include <iCub/iha/FaceDetectLoop.h>
#include <iCub/iha/ImageReadLoop.h>
#include <iCub/iha/GazeReadLoop.h>
#include <iCub/iha/SoundSensorReadLoop.h>
//#include <iCub/iha/SensorLoop.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

namespace iCub {
    namespace contrib {
        class SensorMotorInterfaceModule;
    }
}

using namespace iCub::contrib;

/**
 *
 * Sensor Motor Interface Module class
 *
 * \brief See \ref icub_iha_SensorMotorInterface
 */
class iCub::contrib::SensorMotorInterfaceModule : public yarp::os::Module {

private:

    // ports
    yarp::os::BufferedPort<yarp::os::Bottle> quitPort;
    yarp::os::BufferedPort<yarp::os::Bottle> sensorMotorPort;  // to read the encoders positions
    yarp::os::BufferedPort<yarp::sig::ImageOf< yarp::sig::PixelRgb> > imagePort; // for reading images
    yarp::os::BufferedPort<yarp::sig::Vector> faceCoordsPort; // for reading coordinates of current detected face
    yarp::os::BufferedPort<yarp::os::Bottle> gazeCoordsPort; // for reading coordinates of gaze
    yarp::os::BufferedPort<yarp::os::Bottle> soundSensorPort; // sound sensor data from sound server
    yarp::os::Port sensorOutputPort; // place where final sensor array is written
    yarp::os::Port memOutputPort; // place where sensor readings for short term memory
                                  // are written
    yarp::os::Port statusPort;

    // parameters read from ini file/command line
    int num_image_sensors_x;
    int num_image_sensors_y;
    bool echo_output;
    bool use_gaze;

  

    // Sensor Loop Object pointers
    ImageReadLoop* imageReadLoop;
    FaceDetectLoop* faceDetectLoop;
    //RewardReadLoop* rewardReadLoop;
    SoundSensorReadLoop* soundSensorReadLoop;
    GazeReadLoop* gazeReadLoop;

    // image sensor array
	int *imageSensorarray;

public:

    SensorMotorInterfaceModule();
    virtual ~SensorMotorInterfaceModule();
    
    virtual bool open(Searchable& config);
    virtual bool close();
    virtual bool interruptModule();
    virtual bool updateModule();
    virtual bool respond(const Bottle &command,Bottle &reply);

};


#endif
