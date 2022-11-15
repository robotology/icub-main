/*
 * Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef SKINWRAPPER_H_
#define SKINWRAPPER_H_


//#include "RobotInterfaceRemap.h"
//#include "extractPath.h"

#include <string>
#include <sstream>

#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/os/Property.h>

#include <yarp/dev/IGenericSensor.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>
#include <yarp/dev/IMultipleWrapper.h>

//#include <list>
#include <vector>
#include <iostream>
#include <string>
#include <sstream>
#include <yarp/sig/Vector.h>

#include <yarp/os/PeriodicThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Stamp.h>

#include <yarp/os/LogStream.h>

class skinWrapper : public yarp::dev::DeviceDriver,
                    public yarp::dev::IMultipleWrapper
{
private:
    // Up to day the skinwrapper is able to handle (attach to) just one analog sensor device
    int period;
    yarp::dev::IAnalogSensor *analog;
    int numPorts;
    yarp::dev::IMultipleWrapper *multipleWrapper;

//    yarp::sig::Vector wholeData;      // may be useful if one the skin wrapper has to get data from more than one device...

public:
    std::string id;
    yarp::dev::PolyDriver driver;

    skinWrapper();
    ~skinWrapper();

//    bool open(yarp::os::Property &params, yarp::os::Property &params);
    bool open(yarp::os::Searchable &params);
    bool close();
    void calibrate();
    yarp::sig::Vector * getData();

    void setId(const std::string &i)
    {
        id=i;
    }

    bool attachAll(const yarp::dev::PolyDriverList &p);
    bool detachAll();
};

#endif /* SKINWRAPPER_H_ */
