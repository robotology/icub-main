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
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IMultipleWrapper.h>
#include <yarp/dev/IControlCalibration.h>
#include <yarp/dev/IMotor.h>
#include <yarp/dev/IEncoders.h>

// iCub includes
#include <iCub/IRawValuesPublisher.h>


struct ItemData {
    std::string name;
    int64_t val1, val2, val3, val4;
};

class FineCalibrationChecker : public yarp::dev::DeviceDriver,
                                    public yarp::os::Thread,
                                    public yarp::dev::IMultipleWrapper
{
public:
    // Enum for device status
    enum class deviceStatus
    {
        INITIALIZED = 0,
        OPENED = 1,
        CONFIGURED = 2,
        STARTED = 3,
        CALIBRATED = 4,
        IN_ZERO_POSITION = 5,
        CHECK_COMPLETED = 6,
        UNKNOWN = 244,
        NONE = 255
    };

    // Constructor
    FineCalibrationChecker();

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

    // Overridden methods from yarp::dev::DeviceDriver
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // Overridden methods from yarp::os::Thread
    void run() override;
    void onStop() override;
    bool threadInit() override;

    // Overridden methods from yarp::dev::IMultipleWrapper
    bool attachAll(const yarp::dev::PolyDriverList& device2attach) override;
    bool detachAll() override;

    bool isCalibrationSuccessful() const;

private:
    // Private members

    // Configuration parameters
    std::string _portPrefix = "/fineCalibrationChecker";
    std::string _robotName= "";
    std::string _deviceName= "fineCalibrationChecker";
    std::string _remoteRawValuesPort = "";
    bool _withGui = false;

    deviceStatus _deviceStatus = deviceStatus::NONE;

    yarp::os::Bottle _axesNamesList = yarp::os::Bottle();
    yarp::os::Bottle _goldPositionsList = yarp::os::Bottle();
    yarp::os::Bottle _encoderResolutionsList = yarp::os::Bottle();
    yarp::os::Bottle _calibrationDeltasList = yarp::os::Bottle();

    std::vector<std::string> _robotSubpartsWrapper = {"setup_mc", "head", "left_arm", "right_arm", "torso", "left_leg", "right_leg"};
    std::map<std::string, std::vector<std::int32_t>> rawDataValuesMap;
    iCub::rawValuesKeyMetadataMap rawDataMetadata;
    std::map<std::string, std::array<int32_t, 2>> axesRawGoldenPositionsResMap;
    std::string _rawValuesTag = "eoprot_tag_mc_joint_status_addinfo_multienc";

    // Pointers to interfaces
    struct
    {
        yarp::dev::IMultipleWrapper* _imultwrap{ nullptr };
        yarp::dev::IControlCalibration* _icontrolcalib { nullptr };
        yarp::dev::IMotor* _imot { nullptr };
        yarp::dev::IEncoders* _ienc { nullptr };
    } remappedControlBoardInterfaces;

    iCub::debugLibrary::IRawValuesPublisher* _iravap;

    // Client drivers to communicate with interfaces
    std::unique_ptr<yarp::dev::PolyDriver> _remappedControlBoardDevice;
    std::unique_ptr<yarp::dev::PolyDriver> _rawValuesPublisherDevice;

    void evaluateHardStopPositionDelta(const std::string& key, const std::string& outputFileName);
    void generateOutputImage(int frameWidth, int frameHeight, const std::vector<ItemData>& items);

    // Utility methods
    cv::Scalar getColorForDelta(int32_t delta, int32_t threshold_1, int32_t threshold_2);

    bool attachToAllControlBoards(const yarp::dev::PolyDriverList& polyList);
};

#endif // FINE_CALIBRATION_CHECKER_THREAD_H