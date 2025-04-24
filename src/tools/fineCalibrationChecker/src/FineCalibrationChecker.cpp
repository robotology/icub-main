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
// -*- mode: C++; tab-width: 4; c-basic-offset: 4; indent-tabs-mode: nil -*-

// lib includes
#include <yarp/os/Bottle.h>
#include <yarp/os/Property.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Log.h>

// APIs
#include "FineCalibrationChecker.h"

// Constructor
FineCalibrationChecker::FineCalibrationChecker()
{
    // Initialization (if needed)
}

bool FineCalibrationChecker::configure(yarp::os::ResourceFinder& rf)
{
    // Read configuration file
    Bottle &conf_group = rf.findGroup("GENERAL");
    if (conf_group.isNull())
    {
        yWarning() << "Missing GENERAL group! The module uses the default values";
    }
    else
    {
        // Read parameters from the configuration file
        if (conf_group.check("devicename")) { m_deviceName = conf_group.find("devicename").asString(); }
        if (conf_group.check("portprefix")) { m_portPrefix = conf_group.find("portprefix").asString(); }
        if (conf_group.check("period")) { m_updatePeriod = conf_group.find("period").asFloat64(); }
        if (conf_group.check("robotname")) { m_robotName = conf_group.find("robotname").asString(); }
    }

    // Initialize remote raw values publisher device
    Property deviceProperties;
    deviceProperties.put("device", m_deviceName);
    deviceProperties.put("remote", "/" + m_robotName + m_portPrefix);
    deviceProperties.put("local", "/" + m_robotName + m_portPrefix);
    
    yDebug() << m_deviceName << "++++ config:\n"
        << "\t portprefix: " << m_portPrefix << "\n"
        << "\t period: " << m_updatePeriod << "\n"
        << "\t robotname: " << m_robotName << "\n";

    _fineCalibrationCheckerDevice.open(deviceProperties);

    if (!_fineCalibrationCheckerDevice.isValid())
    {
        yError() << m_deviceNname << "Unable to open device driver. Aborting...";
        return false;
    }
    
    if (!_fineCalibrationCheckerDevice.view(_iravap) || _iravap == nullptr)
    {
        yError() << m_deviceName << "Unable to open raw values publisher interface. Aborting...";
        return false;
    }
    
    // Initialize parametric calibrator device
    if (!_fineCalibrationCheckerDevice.view(_icalib) || _icalib == nullptr)
    {
        yError() << m_deviceName << "Unable to open parametric calibrator interface. Aborting...";
        return false;
    }

    return true;
}

// Public methods
void FineCalibrationChecker::runCalibration()
{
    // Placeholder implementation
    return;
}

bool FineCalibrationChecker::isCalibrationSuccessful() const
{
    // Placeholder implementation
    return true;
}