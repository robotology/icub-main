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

#include "MotorTemperaturePublisher.h"

#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Stamp.h>

using namespace std;
using namespace yarp::os;


bool MotorTemperaturePublisher::configure(yarp::os::ResourceFinder& rf)
{
    // Read configuration file
    Bottle &conf_group = rf.findGroup("GENERAL");
    Bottle* jointsBottle = nullptr;
    if (conf_group.isNull())
    {
        yWarning() << "Missing GENERAL group! The module uses the default values";
    }
    else
    {
        if(conf_group.check("portprefix")) { m_portPrefix = conf_group.find("portprefix").asString(); }
        if(conf_group.check("period")) { _updatePeriod = conf_group.find("period").asFloat64(); }
        if(conf_group.check("robotname")) { _robotName = conf_group.find("robotname").asString(); }
        if (conf_group.check("listofjoints"))
        {
            jointsBottle = conf_group.find("listofjoints").asList();
            _nEnabledMotors = jointsBottle->size();
            for(int i=0; i < _nEnabledMotors; i++) _listOfJoints.push_back(jointsBottle->get(i).asInt32());
        }
        
    }
    
    // Create remote motion control device
    Property options;
    options.put("device", "remote_controlboard");
    options.put("remote", "/"+ _robotName + m_portPrefix);
    options.put("local", m_portPrefix + "/mc");


    yDebug() << "++++ config:\n" 
        << "\t portprefix: " << m_portPrefix << "\n"
        << "\t period: " << _updatePeriod << "\n"
        << "\t robotname: " << _robotName << "\n"
        << "\t listofjoints: " << jointsBottle->toString() << "\n";

    _motionControlDevice.open(options);

    if (!_motionControlDevice.isValid())
    {
        yError() << "Unable to open device driver. Aborting...";
        return false;
    }
    
    if (!_motionControlDevice.view(_imot) || _imot==nullptr)
    {
        yError() << "Unable to open motor raw interface. Aborting...";
        return false;
    }
    
    if (!_imot->getNumberOfMotors(&_nmotors))
    {
        yError() << "Unable to retrieve the number of motors";
        return false;
    }
    else
    {
        yDebug() << "Working with" << _nmotors << "motors";
        yDebug() << "Enabling" << _nEnabledMotors << "motors of the subpart";
    }
    
    // Allocate memory for pointer
    if (!alloc(_nEnabledMotors))
    {
        yError() << "Error allocating memory for pointers. Aborting...";
        return false;
    }
    
    
    // open the communication port towards motor controller module
    if(!_outputPort.open(m_portPrefix +"/motor_temperatures:o"))
    {
        yError() << "Error opening output port for motor control";
        return false;
    }

    for (uint8_t i = 0; i < _listOfJoints.size(); i++)
    {
        if (!_imot->getTemperatureLimit(i, &_motorTemperatureLimits[i]))
        {
            yError() << "Unable to get motor temperature Limits. Aborting...";
            return false;
        }
        else
        {
            yDebug() << "Limit for motor#" << i << "value:" << _motorTemperatureLimits[i];
        }
        
    }

    return true;
}

bool MotorTemperaturePublisher::close()
{
    // Closing port explicitely
    yInfo() << "Calling close functionality\n";

    // Deallocating memory for pointers
    if (!dealloc())
    {
        yError() << "Error deallocating memory for pointer. Failing...";
        return false;
    }
    
    return true;
}

double MotorTemperaturePublisher::getPeriod()
{
    return _updatePeriod;
}

bool MotorTemperaturePublisher::updateModule()
{
    
    for (int i = 0; i < _listOfJoints.size(); i++)
    {
    	_motorTemperatures[i]= 0;
        int jointNib = (int)_listOfJoints[i];
        if (!_imot->getTemperature(jointNib, &_motorTemperatures[jointNib]))
        {
            yError() << "Unable to get motor " << jointNib << " temperature.\n";
        }
    }

    sendData2OutputPort(_motorTemperatures);
    
    return true;
}


MotorTemperaturePublisher::MotorTemperaturePublisher(): _imot(nullptr)
{;}

MotorTemperaturePublisher::~MotorTemperaturePublisher()
{;}

// Private methods
bool MotorTemperaturePublisher::sendData2OutputPort(double * temperatures)
{
    static yarp::os::Stamp stamp;

    stamp.update();

    Bottle &b = _outputPort.prepare();
    _outputPort.setEnvelope(stamp);

    b.clear();

    b.addFloat64(stamp.getTime());
    for (size_t i = 0; i < _nEnabledMotors; i++)
    {
        b.addFloat64(temperatures[i]);
	    uint8_t allarm=0;
	if(temperatures[i] >= _motorTemperatureLimits[i])
		allarm=1;
	    b.addInt8(allarm);
    }
    _outputPort.write();
    
    
    return true;
}

bool MotorTemperaturePublisher::alloc(int nm)
{
    _motorTemperatures = allocAndCheck<double>(nm);
    _motorTemperatureLimits = allocAndCheck<double>(nm);

    return true;

}

bool MotorTemperaturePublisher::dealloc()
{
    checkAndDestroy(_motorTemperatures);
    checkAndDestroy(_motorTemperatureLimits);

    return true;
}
