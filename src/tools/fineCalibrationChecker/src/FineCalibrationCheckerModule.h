/*
 * Copyright (C) 2024 iCub Facility - Istituto Italiano di Tecnologia
 * All rights reserved.
 * 
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
*/
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
 *
 * \section intro_sec Description
 * The module inherits from the YARP interface yarp::os::RFModule and manages a yarp::os::Thread, 
 * keeping it running with a specific period until the user decides to stop the process.
 *
 * The thread implements yarp::os::Thread and contains the core logic of the tool, including:
 * - Parsing configuration data from a config.ini file
 * - Initializing data structures for storing calibration data
 * - Parsing a .csv file with input position data (zeroPositionsData.csv)
 * - Opening devices and connecting to remote control boards and ports
 * - Running the calibration check
 * - Outputting the results
 *
 * \section features Main Features
 * - Reads configuration parameters and joint information from a configuration file.
 * - Connects to remote control boards and a raw values publisher to access joint and encoder data.
 * - Executes a calibration routine, including checking golden hard-stop positions and comparing them with live encoder readings.
 * - Evaluates the difference (delta) between expected and measured joint positions, logs the results, and generates output files and images for analysis.
 * - Runs as a threaded YARP module, allowing for asynchronous calibration and monitoring.
 *
 * This tool is useful for developers and technicians who need to verify and fine-tune the mechanical calibration of the robot’s joints, ensuring accurate and reliable operation.
 * 
 * \section params Parameters
 * --devicename  The name of the device (default: fineCalibrationChecker).
 * --robotname   The name of the robot to be controlled (default: torsoPitchMj1).
 * --remoteRawValuesPort  The name of the remote port streaming the raw encoder data to connect to (default: /torsoPitchMj1/setup_rawval).
 * --remoteControlBoardsList  List of the remote control boards to connect to (default: (torso-pitch-mj1-setup)).
 * --axesNamesList  List of the axes names to connect to (default: (torso_pitch_mj1_real_aksim2)).
 *
 * \section tested_os_sec Tested OS
 * Linux
 * \author Jacopo Losi jacopo.losi@iit.it
 * 
 */
// -*- mode: C++; tab-width: 4; c-basic-offset: 4; indent-tabs-mode: nil -*-

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