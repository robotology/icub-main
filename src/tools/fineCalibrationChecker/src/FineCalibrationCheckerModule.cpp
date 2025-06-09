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
#include "FineCalibrationCheckerModule.h"

namespace
{
    YARP_LOG_COMPONENT(FineCalibrationCheckerModuleCOMPONENT, "yarp.device.fineCalibrationCheckerModule")
}

bool FineCalibrationCheckerModule::configure(yarp::os::ResourceFinder& rf)
{
    // Create the thread using std::make_unique
    checkerThread = std::make_unique<FineCalibrationCheckerThread>(rf);

    // Start the thread
    if (!checkerThread->start()) {
        yCError(FineCalibrationCheckerModuleCOMPONENT) << "Failed to start FineCalibrationCheckerThread.";
        checkerThread.reset(); // Release the unique_ptr if starting fails
        return false;
    }

    return true;
}

bool FineCalibrationCheckerModule::close()
{
    // Stop the thread if it exists
    if (checkerThread) 
    {
        checkerThread->stop();
        checkerThread.reset(); // Automatically deletes the thread
    }

    return true;
}

bool FineCalibrationCheckerModule::updateModule()
{
    // This method is called periodically by the RFModule
    // You can add any periodic tasks here if needed
    yCDebug(FineCalibrationCheckerModuleCOMPONENT) << "Module is running happily";
    return true;
}

double FineCalibrationCheckerModule::getPeriod()
{
    // Return the period for the module
    return 1.0; // Example: 1 second
}

