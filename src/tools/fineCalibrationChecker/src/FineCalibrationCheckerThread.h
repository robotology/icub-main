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

#include <opencv2/opencv.hpp>

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


struct ItemData {
    std::string name;
    int32_t val1, val2, val3, val4;
};

class FineCalibrationCheckerThread : public yarp::os::Thread
{
public:
    // Enum for device status
    enum class deviceStatus 
    {
        INITIALIZED = 0,
        OPENED = 1,
        CONFIGURED = 2,
        CALIBRATED = 3,
        END_POSITION_CHECKED = 4,
        IN_HOME_POSITION = 5,
        UNKNOWN = 244,
        NONE = 255
    };

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
    std::string _remoteRawValuesPort = "";
    
    deviceStatus _deviceStatus = deviceStatus::NONE;
    
    yarp::os::Bottle _controlBoardsList = yarp::os::Bottle();
    yarp::os::Bottle _axesNamesList = yarp::os::Bottle();
    std::vector<std::string> _robotSubpartsWrapper = {"setup_mc", "head", "left_arm", "right_arm", "torso", "left_leg", "right_leg"};
    std::map<std::string, std::vector<std::int32_t>> rawDataValuesMap;
    iCub::rawValuesKeyMetadataMap rawDataMetadata;
    std::map<std::string, std::array<int32_t, 2>> axesRawGoldenPositionsResMap;
    std::string _rawValuesTag = "eoprot_tag_mc_joint_status_addinfo_multienc";

    // Pointers to interfaces
    yarp::dev::IRemoteCalibrator* _iremotecalib;
    yarp::dev::IControlCalibration* _icontrolcalib;
    yarp::dev::IMotor* _imot;
    iCub::debugLibrary::IRawValuesPublisher* _iravap;

    // Client drivers to communicate with interfaces
    std::unique_ptr<yarp::dev::PolyDriver> _fineCalibrationCheckerDevice;
    std::unique_ptr<yarp::dev::PolyDriver> _rawValuesOublisherDevice;

    bool configureCalibration();
    bool runCalibration();
    void evaluateHardStopPositionDelta(const std::string& key, const std::string& inputFileName, const std::string& outputFileName);
    void generateOutputImage(int frameWidth, int frameHeight, const std::vector<ItemData>& items);
    // Utility methods
    void configureDevicesMap(std::vector<std::string> list);
    cv::Scalar getColorForDelta(int32_t delta, int32_t threshold_1, int32_t threshold_2);
};

#endif // FINE_CALIBRATION_CHECKER_THREAD_H