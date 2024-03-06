/*
 * Copyright (C) 2024 iCub Facility - Istituto Italiano di Tecnologia
 * Author:  Jacopo Losi
 * email:   jacopo.losi@iit.it
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


#ifndef __MOTORTEMPERATUREPUBLISHER__
#define __MOTORTEMPERATUREPUBLISHER__

#include <yarp/os/RFModule.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Bottle.h>
#include <yarp/dev/ControlBoardHelper.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/os/BufferedPort.h>

class MotorTemperaturePublisher: public yarp::os::RFModule
{
private:
    yarp::dev::IMotor *_imot;
    yarp::os::BufferedPort<yarp::os::Bottle>  _outputPort;

    double *_motorTemperatures;
    double *_motorTemperatureLimits;
    int _nmotors;
    int _nEnabledMotors = 0;

    std::string _portPrefix="/5-setup";
    double _updatePeriod = 1; //seconds
    std::string _robotName= "icub";
    yarp::sig::Vector _listOfJoints = 0;

    yarp::dev::PolyDriver _motionControlDevice;

    bool sendData2OutputPort(double *temperatures);
    bool alloc(int nm);
    bool dealloc();

public:

    MotorTemperaturePublisher();
    ~MotorTemperaturePublisher() override;

    MotorTemperaturePublisher(const MotorTemperaturePublisher&) = default;
    MotorTemperaturePublisher(MotorTemperaturePublisher&&) = default;
    MotorTemperaturePublisher& operator=(const MotorTemperaturePublisher&) = default;
    MotorTemperaturePublisher& operator=(MotorTemperaturePublisher&&) = default;

    bool configure(yarp::os::ResourceFinder &rf);
    bool close();
    double getPeriod();
    bool updateModule();
};

#endif
