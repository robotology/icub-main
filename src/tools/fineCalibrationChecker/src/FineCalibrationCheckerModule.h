/*
 * Copyright (C) 2025 iCub Facility - Istituto Italiano di Tecnologia
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
/**
 * @ingroup icub_tools
 * \defgroup fineCalibrationChecker fineCalibrationChecker
 *
 * Tool to check and analyze the accuracy of joint calibration on ergoCub and iCub robots.
 *
 * This module provides users with information about the accuracy of the joints calibration procedure, allowing the collection of data that might help in analyzing miscalibration errors on a specific system. 
 * It interfaces with YARP devices and robot control boards.
 *
 * The tool is composed of two main parts:
 * - A module that manages a thread containing all the core functionalities of the application.
 * - A thread that performs the data collection and calibration checking.
 *
 * \section disclaimer Disclaimer
 * This software is intended for use on the ergoCub, iCub platforms and related bench setups made with same components. 
 * Use on other platforms is not guaranteed.
 * Complete documentation to the tool can be found at (https://robotology.github.io/robotology-documentation/doc/html/group__fineCalibrationChecker.html)
*/

#ifndef FINE_CALIBRATION_CHECKER_MODULE_H
#define FINE_CALIBRATION_CHECKER_MODULE_H

// std includes
#include <memory> // For std::unique_ptr

// yarp includes
#include <yarp/os/RFModule.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ICalibrator.h>
#include <yarp/dev/IRemoteCalibrator.h>
#include <yarp/dev/IControlCalibration.h>

// icub includes
#include <iCub/IRawValuesPublisher.h>

// local includes
#include "FineCalibrationCheckerThread.h"

class FineCalibrationCheckerModule : public yarp::os::RFModule
{
public:
    // Constructor
    FineCalibrationCheckerModule() = default;

    // Destructor
    ~FineCalibrationCheckerModule() = default;

    // Copy constructor
    FineCalibrationCheckerModule(const FineCalibrationCheckerModule& other) = default;

    // Copy assignment operator
    FineCalibrationCheckerModule& operator=(const FineCalibrationCheckerModule& other) = default;

    // Move constructor
    FineCalibrationCheckerModule(FineCalibrationCheckerModule&& other) noexcept = default;

    // Move assignment operator
    FineCalibrationCheckerModule& operator=(FineCalibrationCheckerModule&& other) noexcept = default;

    // Public methods

    // Overridden methods from RFModule
    bool configure(yarp::os::ResourceFinder& rf) override;
    bool close() override;
    double getPeriod() override; // TODO: add comment in implementation, should not do anything
    bool updateModule() override; // TODO: add comment in implementation, should not do anything

private:

    // Private variables
    std::unique_ptr<FineCalibrationCheckerThread> checkerThread;
};

#endif // FINE_CALIBRATION_CHECKER_MODULE_H