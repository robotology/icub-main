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

#include "FakeRawDataPublisherTester.h"

#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

using namespace std;
using namespace yarp::os;


bool FakeRawDataPublisherTester::configure(yarp::os::ResourceFinder& rf)
{
    // Read configuration file
    Bottle &conf_group = rf.findGroup("GENERAL");
    if (conf_group.isNull())
    {
        yWarning() << "Missing GENERAL group! The module uses the default values";
    }
    else
    {
        if(conf_group.check("portprefix")) { m_portPrefix = conf_group.find("portprefix").asString(); }
        if(conf_group.check("period")) { _updatePeriod = conf_group.find("period").asFloat64(); }
        if(conf_group.check("robotname")) { _robotName = conf_group.find("robotname").asString(); }
        
    }
    
    // Create remote motion control device
    Property options;
    options.put("device", "rawValuesPublisherClient");
    options.put("remote", "/" + _robotName + m_portPrefix);
    options.put("local",  "/" + _robotName + m_portPrefix);


    yDebug() << "++++ config:\n" 
        << "\t portprefix: " << m_portPrefix << "\n"
        << "\t period: " << _updatePeriod << "\n"
        << "\t robotname: " << _robotName << "\n";

    _rawDataPublisherDevice.open(options);

    if (!_rawDataPublisherDevice.isValid())
    {
        yError() << "Unable to open device driver. Aborting...";
        return false;
    }
    
    if (!_rawDataPublisherDevice.view(_iravap) || _iravap==nullptr)
    {
        yError() << "Unable to open motor raw interface. Aborting...";
        return false;
    }
    
    // open the communication port for streaming data
    if(!_outputPort.open(m_portPrefix + "/rawdata_publisher:o"))
    {
        yError() << "Error opening output port for rawdata publisher";
        return false;
    }

    // Initialize the local map as an empty map --> will be filled later
    _rawDataValuesMap = {};

    return true;
}

bool FakeRawDataPublisherTester::close()
{
    // Closing port explicitely
    yInfo() << "Calling close functionality\n";
    
    return true;
}

double FakeRawDataPublisherTester::getPeriod()
{
    return _updatePeriod;
}

bool FakeRawDataPublisherTester::updateModule()
{
    Bottle &b = _outputPort.prepare();
    b.clear();

    if(!_iravap->getRawDataMap(_rawDataValuesMap))
    {
        yError() << "Map of raw values to be streamed cannot be retrieved. Unpreparing...\n";
        _outputPort.unprepare();
        return true;
    }
    else
    {
        for (auto [key, value] : _rawDataValuesMap)
        {
            b.addString(key);
            Bottle &bKeyContent = b.addList();
            for (auto el : value)
            {
                bKeyContent.addInt32(el);
            }
        }
        
        _stamp.update();
        _outputPort.setEnvelope(_stamp);
        _outputPort.write();
    }
    
    return true;
}


FakeRawDataPublisherTester::FakeRawDataPublisherTester(): _iravap(nullptr)
{;}

FakeRawDataPublisherTester::~FakeRawDataPublisherTester()
{;}
