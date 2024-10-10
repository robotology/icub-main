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


#ifndef __FAKERAWDATAPUBLISHERTESTER__
#define __FAKERAWDATAPUBLISHERTESTER__

// yarp includes
#include <yarp/os/Stamp.h>
#include <yarp/os/RFModule.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>

// Interface for the getter of the raw data 
// It calls the rawValuesDataStruct generated w/ thrift
#include <iCub/IRawValuesPublisher.h>

class FakeRawDataPublisherTester: public yarp::os::RFModule
{
private:
    iCub::debugLibrary::IRawValuesPublisher*_iravap;
    yarp::os::BufferedPort<yarp::os::Bottle>  _outputPort;

    std::map<std::string, std::vector<std::int32_t>> _rawDataValuesMap;
    yarp::os::Stamp _stamp;

    std::string m_portPrefix="/raw_data";
    double _updatePeriod = 0.5; //seconds
    std::string _robotName= "fakedevicetest";

    yarp::dev::PolyDriver _rawDataPublisherDevice;


public:

    FakeRawDataPublisherTester();
    ~FakeRawDataPublisherTester() override;

    FakeRawDataPublisherTester(const FakeRawDataPublisherTester&) = default;
    FakeRawDataPublisherTester(FakeRawDataPublisherTester&&) = default;
    FakeRawDataPublisherTester& operator=(const FakeRawDataPublisherTester&) = default;
    FakeRawDataPublisherTester& operator=(FakeRawDataPublisherTester&&) = default;

    bool configure(yarp::os::ResourceFinder &rf);
    bool close();
    double getPeriod();
    bool updateModule();
};

#endif
