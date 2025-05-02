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

#ifndef FINE_CALIBRATION_CHECKER_THREAD_H
#define FINE_CALIBRATION_CHECKER_THREAD_H

// YARP includes
#include <yarp/os/Thread.h>

class FineCalibrationCheckerThread : public yarp::os::Thread
{
public:
    // Constructor
    FineCalibrationCheckerThread() = default;

    // Destructor
    ~FineCalibrationCheckerThread() = default;

    // Copy constructor
    FineCalibrationCheckerThread(const FineCalibrationCheckerThread& other) = default;

    // Copy assignment operator
    FineCalibrationCheckerThread& operator=(const FineCalibrationCheckerThread& other) = default;

    // Move constructor
    FineCalibrationCheckerThread(FineCalibrationCheckerThread&& other) noexcept = default;

    // Move assignment operator
    FineCalibrationCheckerThread& operator=(FineCalibrationCheckerThread&& other) noexcept = default;

    // Overridden methods from yarp::os::Thread
    void run() override;
    void onStop() override;
    bool threadInit() override;
    void threadRelease() override;

    bool isCalibrationSuccessful() const;

private:
    // Private members
    bool calibrationStatus;

    // Pointer to the raw values publisher interface
    iCub::debugLibrary::IRawValuesPublisher* _iravap;

    // Pointer to the parametric calibrator and its controller interface
    yarp::dev::IRemoteCalibrator* _iremotecalib;
    yarp::dev::IControlCalibration* _icontrolcalib;

    // Clinet driver to communicate with interfaces
    yarp::dev::PolyDriver _fineCalibrationCheckerDevice;

    
    void runCalibration();
};

#endif // FINE_CALIBRATION_CHECKER_THREAD_H