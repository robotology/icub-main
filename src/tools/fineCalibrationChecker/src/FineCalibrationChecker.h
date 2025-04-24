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

#ifndef FINE_CALIBRATION_CHECKER_H
#define FINE_CALIBRATION_CHECKER_H

// yarp includes
#include <yarp/os/RFModule.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ICalibrator.h>

// icub includes
#include <iCub/IRawValuesPublisher.h>

class FineCalibrationChecker : public yarp::os::RFModule
{
public:
    // Constructor
    FineCalibrationChecker() = default;

    // Destructor
    ~FineCalibrationChecker() = default;

    // Copy constructor
    FineCalibrationChecker(const FineCalibrationChecker& other) = default;

    // Copy assignment operator
    FineCalibrationChecker& operator=(const FineCalibrationChecker& other) = default;

    // Move constructor
    FineCalibrationChecker(FineCalibrationChecker&& other) noexcept = default;

    // Move assignment operator
    FineCalibrationChecker& operator=(FineCalibrationChecker&& other) noexcept = default;

    // Public methods
    bool configure(yarp::os::ResourceFinder& rf) override;
    void runCalibration();
    bool isCalibrationSuccessful() const;

private:
    // Private members
    bool calibrationStatus;

    // Pointer to the raw values publisher interface
    iCub::debugLibrary::IRawValuesPublisher* _iravap;

    // Pointer to the parametric calibrator interface
    yarp::dev::ICalibrator* _icalib;

    // Clinet driver to communicate with interfaces
    yarp::dev::PolyDriver _fineCalibrationCheckerDevice;
};

#endif // FINE_CALIBRATION_CHECKER_H