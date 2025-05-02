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

// std includes
#include <vector>
#include <string>
#include <map>
#include <memory>

// yarp includes
#include <yarp/os/Bottle.h>
#include <yarp/os/Thread.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IRemoteCalibrator.h>
#include <yarp/dev/IControlCalibration.h>
#include <yarp/dev/IMotor.h>

// iCub includes
#include <iCub/IRawValuesPublisher.h>

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

    // Parameterized constructor
    FineCalibrationCheckerThread(yarp::os::ResourceFinder& rf);

    // Overridden methods from yarp::os::Thread
    void run() override;
    void onStop() override;
    bool threadInit() override;
    void threadRelease() override;

    bool isCalibrationSuccessful() const;

private:
    // Private members

    // Configuration parameters
    std::string _portPrefix = "/fineCalibrationChecker";
    std::string _robotName= "";
    std::string _deviceName= "fineCalibrationChecker";
    yarp::os::Bottle* _subpartsList = nullptr;
    yarp::os::Bottle* _jointsList = nullptr;
    std::vector<std::string> _robotSubpartsWrapper = {"setup_mc", "head", "left_arm", "right_arm", "torso", "left_leg", "right_leg"};
    std::vector<std::string> _robotSubpartsList = {};
    bool calibrationStatus;
    bool configured = false;
    std::map<std::string, std::vector<std::int32_t>> rawDataValuesMap;

    // Pointer to the raw values publisher interface
    iCub::debugLibrary::IRawValuesPublisher* _iravap;//TODO: we need also to make another device for this interface not implemented by remotecontrolboardremapper

    // Pointer to the parametric calibrator and its controller interface
    yarp::dev::IRemoteCalibrator* _iremotecalib;
    yarp::dev::IControlCalibration* _icontrolcalib;
    yarp::dev::IMotor* _imot;

    // Clinet driver to communicate with interfaces
    std::unique_ptr<yarp::dev::PolyDriver> _fineCalibrationCheckerDevice;
    std::unique_ptr<yarp::dev::PolyDriver> _rawValuesOublisherDevice;

    bool configureCalibration();
    void runCalibration();

    // Utility methods
    void configureDevicesMap(std::vector<std::string> list);
};

#endif // FINE_CALIBRATION_CHECKER_THREAD_H