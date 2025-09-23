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

// std includes
#include <iostream>
#include <algorithm>
#include <unordered_map>
#include <set>
#include <utility>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <chrono>

// yarp includes
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/Log.h>
#include <yarp/os/Searchable.h>

// APIs
#include "FineCalibrationChecker.h"


namespace
{
    YARP_LOG_COMPONENT(FineCalibrationCheckerCOMPONENT, "yarp.device.FineCalibrationChecker")
}

FineCalibrationChecker::FineCalibrationChecker()
    : yarp::os::Thread(), _deviceName("fineCalibrationChecker"), _portPrefix("/fineCalibrationChecker"),
      _robotName("icub"), _remoteRawValuesPort("/icub/rawvalues"), _axesNamesList(yarp::os::Bottle()),
      _goldPositionsList(yarp::os::Bottle()), _encoderResolutionsList(yarp::os::Bottle()), _calibrationDeltasList(yarp::os::Bottle()), _deviceStatus(deviceStatus::NONE)
{
    // Initialize device driver as empty PolyDriver
    _remappedControlBoardDevice = std::make_unique<yarp::dev::PolyDriver>();
    _remappedRawValuesPublisherDevice = std::make_unique<yarp::dev::PolyDriver>();
}

bool FineCalibrationChecker::open(yarp::os::Searchable& config)
{
    // Read configuration file
    yarp::os::Property property;
    property.fromString(config.toString().c_str());
    if (property.isNull())
    {
        yCError(FineCalibrationCheckerCOMPONENT) << "Failed to read configuration file. Stopping device...";
        return false;
    }
    else
    {
        // Read parameters from the configuration file
        if (property.check("devicename")) { _deviceName = property.find("devicename").asString(); }
        if (property.check("robotname")) { _robotName = property.find("robotname").asString(); }
        if (property.check("remoteRawValuesPort")) { _remoteRawValuesPort = property.find("remoteRawValuesPort").asString(); }
        if (property.check("axesNamesList"))
        {
            yarp::os::Bottle* _jointsList = property.find("axesNamesList").asList();
            yarp::os::Bottle &axesNames = _axesNamesList.addList();

            for (size_t i = 0; i < _jointsList->size(); i++)
            {
                axesNames.addString(_jointsList->get(i).asString());
            }
        }
        if(property.check("goldPositions"))
        {
            yarp::os::Bottle* _goldPositions = property.find("goldPositions").asList();
            yarp::os::Bottle &goldPositions = _goldPositionsList.addList();

            for (size_t i = 0; i < _goldPositions->size(); i++)
            {
                goldPositions.addInt32(_goldPositions->get(i).asInt32());
            }
        }
        if(property.check("calibrationDeltas"))
        {
            yarp::os::Bottle* _calibrationDeltas = property.find("calibrationDeltas").asList();
            yarp::os::Bottle &calibrationDeltas = _calibrationDeltasList.addList();

            for (size_t i = 0; i < _calibrationDeltas->size(); i++)
            {
                calibrationDeltas.addFloat64(_calibrationDeltas->get(i).asFloat64());
            }
        }
        if(property.check("encoderResolutions"))
        {
            yarp::os::Bottle* _encoderResolutions = property.find("encoderResolutions").asList();
            yarp::os::Bottle &encoderResolutions = _encoderResolutionsList.addList();

            for (size_t i = 0; i < _encoderResolutions->size(); i++)
            {
                encoderResolutions.addInt32(_encoderResolutions->get(i).asInt32());
            }
        }
        if (property.check("axesSigns"))
        {
            yarp::os::Bottle* _axesSigns = property.find("axesSigns").asList();
            yarp::os::Bottle &axesSigns = _axesSignsList.addList();

            for (size_t i = 0; i < _axesSigns->size(); i++)
            {
                axesSigns.addInt32(_axesSigns->get(i).asInt32());
            }
        }
        

        // Use pointer to list to simplify the listing
        yarp::os::Bottle* axes = _axesNamesList.get(0).asList();
        yarp::os::Bottle* goldpos = _goldPositionsList.get(0).asList();
        yarp::os::Bottle* encres = _encoderResolutionsList.get(0).asList();
        yarp::os::Bottle* caldeltas = _calibrationDeltasList.get(0).asList();
        yarp::os::Bottle* signs = _axesSignsList.get(0).asList();

        // Check list sizes. They must be equal
        if (axes->size() != goldpos->size() ||
            axes->size() != encres->size() ||
            axes->size() != caldeltas->size() ||
            axes->size() != signs->size())
        {
            yCError(FineCalibrationCheckerCOMPONENT) << "Axes names, gold positions and encoder resolutions lists must have the same size. Stopping device...";
            return false;
        }
        else
        {
            yCDebug(FineCalibrationCheckerCOMPONENT) << "Axes names list:" << _axesNamesList.toString();
            yCDebug(FineCalibrationCheckerCOMPONENT) << "Gold positions list:" << goldpos->toString();
            yCDebug(FineCalibrationCheckerCOMPONENT) << "Encoder resolutions list:" << encres->toString();
            yCDebug(FineCalibrationCheckerCOMPONENT) << "Calibration deltas list:" << caldeltas->toString();
            yCDebug(FineCalibrationCheckerCOMPONENT) << "Axes signs list:" << signs->toString();
        }


        for (size_t i = 0; i < axes->size(); i++)
        {
            yCDebug(FineCalibrationCheckerCOMPONENT) << "Adding to MAP key:" << axes->get(i).asString()
                << "GP:" << goldpos->get(i).asInt32() << "ER:" << encres->get(i).asInt32() << "CD:" << caldeltas->get(i).asFloat64();
            axesRawGoldenPositionsResMap[axes->get(i).asString()] = {goldpos->get(i).asInt32(), encres->get(i).asInt32()};
            yCDebug(FineCalibrationCheckerCOMPONENT) << "Array added to MAP:" << axesRawGoldenPositionsResMap.at(axes->get(i).asString())[0]
                << "and" << axesRawGoldenPositionsResMap.at(axes->get(i).asString())[1];
        }
    }

    _withGui = property.check("withGui", yarp::os::Value(false)).asBool();

    yCDebug(FineCalibrationCheckerCOMPONENT) << _deviceName << "Initialized device driver";
    _deviceStatus = deviceStatus::INITIALIZED;

    // Open ports to control boards and network wrapper servers
    // Open the port to the remote control boards and connect the remappedControlBoardDevice to it
    yarp::os::Property deviceProperties;

    deviceProperties.put("device", "controlboardremapper");
    deviceProperties.put("axesNames", _axesNamesList.get(0));

    yCDebug(FineCalibrationCheckerCOMPONENT) << _deviceName << "Configuring device driver with properties:" << deviceProperties.toString();

    _remappedControlBoardDevice->open(deviceProperties);

    if (!_remappedControlBoardDevice->isValid())
    {
        yCError(FineCalibrationCheckerCOMPONENT) << _deviceName << "Unable to open device driver. Aborting...";
        return false;
    }

    // Open port to the remote raw values publisher server and connect the client network wrapper to it
    if (_remoteRawValuesPort.empty())
    {
        yCError(FineCalibrationCheckerCOMPONENT) << "Remote raw values port is empty. Cannot open device driver. Stopping device...";
        return false;
    }
    else
    {
        yCDebug(FineCalibrationCheckerCOMPONENT) << _deviceName << "Remote raw values port:" << _remoteRawValuesPort;

    }

     yarp::os::Property rawValuesDeviceProperties;
    rawValuesDeviceProperties.put("device", "rawvaluespublisherremapper");
    rawValuesDeviceProperties.put("axesNames", _axesNamesList.get(0));
    rawValuesDeviceProperties.put("remote", _remoteRawValuesPort);
    rawValuesDeviceProperties.put("local", "/" + _deviceName + "/rawValuesPublisherRemapper");

    _remappedRawValuesPublisherDevice->open(rawValuesDeviceProperties);

    if (!_remappedRawValuesPublisherDevice->isValid())
    {
        yCError(FineCalibrationCheckerCOMPONENT) << _deviceName << "Unable to open raw values device driver. Aborting...";
        return false;
    }

    yCDebug(FineCalibrationCheckerCOMPONENT) << _deviceName << "Opened all devices successfully";

    _deviceStatus = deviceStatus::OPENED;
    return true;
}

bool FineCalibrationChecker::close()
{
    // Close the device driver and all the opened ports
    if(_remappedControlBoardDevice->close())
    {
        yCDebug(FineCalibrationCheckerCOMPONENT) << "Closed device" << _deviceName;
    }
    else
    {
        yCError(FineCalibrationCheckerCOMPONENT) << "Unable to close device" << _deviceName;
    }

    if (_remappedRawValuesPublisherDevice->close())
    {
        yCDebug(FineCalibrationCheckerCOMPONENT) << "Closed raw values publisher device";
    }
    else
    {
        yCError(FineCalibrationCheckerCOMPONENT) << "Unable to close raw values publisher device";
    }
    yCDebug(FineCalibrationCheckerCOMPONENT) << _deviceName << "Closed all devices successfully";

    return true;
}

bool FineCalibrationChecker::threadInit()
{
    // Initialize the thread --> can add the view on the interfaces here
    if (!_remappedControlBoardDevice->view(remappedControlBoardInterfaces._imot) || remappedControlBoardInterfaces._imot == nullptr)
    {
        yCError(FineCalibrationCheckerCOMPONENT) << _deviceName << "Unable to open motor interface. Aborting...";
        return false;
    }

    if (!_remappedControlBoardDevice->view(remappedControlBoardInterfaces._ienc) || remappedControlBoardInterfaces._ienc == nullptr)
    {
        yCError(FineCalibrationCheckerCOMPONENT) << _deviceName << "Unable to open encoders interface. Aborting...";
        return false;
    }

    if (!_remappedControlBoardDevice->view(remappedControlBoardInterfaces._icontrolcalib) || remappedControlBoardInterfaces._icontrolcalib == nullptr)
    {
        yCError(FineCalibrationCheckerCOMPONENT) << _deviceName << "Unable to open control calibration interface. Aborting...";
        return false;
    }

    if (!_remappedRawValuesPublisherDevice->view(remappedRawValuesPublisherInterfaces._iravap) || remappedRawValuesPublisherInterfaces._iravap == nullptr)
    {
        yCError(FineCalibrationCheckerCOMPONENT) << _deviceName << "Unable to open raw values publisher interface. Aborting...";
        return false;
    }


    // Configuring raw values metadata
    rawDataMetadata = {};
    remappedRawValuesPublisherInterfaces._iravap->getMetadataMap(rawDataMetadata);
    yCDebug(FineCalibrationCheckerCOMPONENT) << _deviceName << "Configured raw values with metadata";
    for (auto [k, m] : rawDataMetadata.metadataMap)
    {
        yCDebug(FineCalibrationCheckerCOMPONENT) << _deviceName << "\n"
            << "\t Key: " << k << "\n"
            << "\t axesName: " << m.axesNames << "\n"
            << "\t rawValueNames: " << m.rawValueNames
        ;
    }

    yCDebug(FineCalibrationCheckerCOMPONENT) << _deviceName << "Opened remote calibrator and control calibration interfaces successfully";

    _deviceStatus = deviceStatus::CONFIGURED;
    return true;

}

void FineCalibrationChecker::run()
{
    // Run the calibration thread
    yCDebug(FineCalibrationCheckerCOMPONENT) << _deviceName << "Thread started to run";

    // Use chrono for time-based debug prints
    static auto lastTimerLog = std::chrono::steady_clock::now();
    static auto shoutdownTimer = std::chrono::steady_clock::now();

    while(!this->isStopping())
    {
        if (_deviceStatus == deviceStatus::CONFIGURED)
        {
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTimerLog).count() > 1000)
            {
                yCDebug(FineCalibrationCheckerCOMPONENT) << _deviceName << "Device configured, waiting for attachAll() to be called.";
                lastTimerLog = now;
            }
            yarp::os::Time::delay(0.1); // Prevent busy-waiting.
            continue;
        }
        else if(_deviceStatus == deviceStatus::STARTED)
        {
            // Get number of axes
            int numAxes = 0;
            if(!_axesNamesList.isNull() && _axesNamesList.size() > 0 && _axesNamesList.get(0).isList())
            {
                numAxes = _axesNamesList.get(0).asList()->size();
            }
            // Check if calibration is done by calling IControlCalibration APIs
            bool calibDone = true;
            for (size_t i = 0; i < numAxes; i++)
            {
                calibDone &= remappedControlBoardInterfaces._icontrolcalib->calibrationDone(i);
                // TODO: add list of axis that did not perform calibration to be rechecked
            }
            if (!calibDone)
            {
                yCWarning(FineCalibrationCheckerCOMPONENT) << _deviceName << "Calib not complete for all axis. Waiting for calibration to finish...";
                yarp::os::Time::delay(0.1); // Prevent busy-waiting.
                continue;
            }
            else
            {
                yCDebug(FineCalibrationCheckerCOMPONENT) << _deviceName << "Calibration done successfully";
                _deviceStatus = deviceStatus::CALIBRATED;
            }
        }
        else if(_deviceStatus == deviceStatus::CALIBRATED)
        {
            // TODO: Check that all joints reached the startup/zero position
            _deviceStatus = deviceStatus::IN_ZERO_POSITION;
        }
        else if(_deviceStatus == deviceStatus::IN_ZERO_POSITION)
        {
            if(!remappedRawValuesPublisherInterfaces._iravap->getRawDataMap(rawDataValuesMap))
            {
                yCWarning(FineCalibrationCheckerCOMPONENT) << "embObjMotionControl::IRawValuesPublisher warning : raw_data_values map was not read correctly";
            }
            else
            {
                yCDebug(FineCalibrationCheckerCOMPONENT) << "Get raw values from encoders:";
                for (auto [key,value] : rawDataValuesMap)
                {
                    yCDebug(FineCalibrationCheckerCOMPONENT) << "\t key:" << key << "value:" << value;
                }

                // Here we have to evaluate the delta between the raw golen position
                // and the raw position read at the hard stop per each axis
                // TODO: input file not needed anymore. RobeRemoved
                evaluateHardStopPositionDelta(_rawValuesTag, "zeroPositionsDataDelta.csv");
                _deviceStatus = deviceStatus::CHECK_COMPLETED;
            }
        }
        else if(_deviceStatus == deviceStatus::CHECK_COMPLETED)
        {
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - shoutdownTimer).count() > 5000)
            {
                yCDebug(FineCalibrationCheckerCOMPONENT) << _deviceName << "Operation completed successfully. Waiting yarprobotinterface to stop the thread...";
                break; // Exit the loop to stop the thread
            }
        }
        else
        {
            yCError(FineCalibrationCheckerCOMPONENT) << _deviceName << "Device is in unknown state. Stopping thread...";
            break; // Exit the loop to stop the thread

        }
    }

    yCDebug(FineCalibrationCheckerCOMPONENT) << _deviceName << "Thread stopping";
    // Perform any cleanup or finalization tasks here
    // For example, you can close the device or release resources

}

void FineCalibrationChecker::onStop()
{
    //nothing to do for now
}

bool FineCalibrationChecker::attachAll(const yarp::dev::PolyDriverList& device2attach)
{
    // Attach all devices to the FineCalibrationChecker
    yCDebug(FineCalibrationCheckerCOMPONENT) << _deviceName << "Attaching all devices";

    // The _imultwrap view should be place before calling the attachToAllControlBoards method
    // otherwise IMultipleWrapper::attachAll cannot be used
    if (!_remappedControlBoardDevice->view(remappedControlBoardInterfaces._imultwrap) || remappedControlBoardInterfaces._imultwrap == nullptr)
    {
        yCError(FineCalibrationCheckerCOMPONENT) << _deviceName << "Unable to open multiple wrapper interface. Aborting...";
        return false;
    }

    if (!_remappedControlBoardDevice->isValid())
    {
        yCError(FineCalibrationCheckerCOMPONENT) << _deviceName << "Device is not valid. Cannot attach.";
        return false;
    }

    if(!_remappedRawValuesPublisherDevice->view(remappedRawValuesPublisherInterfaces._imultwrap) || remappedRawValuesPublisherInterfaces._imultwrap == nullptr)
    {
        yCError(FineCalibrationCheckerCOMPONENT) << _deviceName << "Unable to open multiple wrapper interface for raw values publisher. Aborting...";
        return false;
    }

    if(!_remappedRawValuesPublisherDevice->isValid())
    {
        yCError(FineCalibrationCheckerCOMPONENT) << _deviceName << "Raw values publisher device is not valid. Cannot attach.";
        return false;
    }

    // Attach the device to the thread
    if (!this->attachToAllControlBoards(device2attach))
    {
        yCError(FineCalibrationCheckerCOMPONENT) << _deviceName << "Failed to attach all devices.";
        return false;
    }

    yCDebug(FineCalibrationCheckerCOMPONENT) << _deviceName << "Successfully attached all devices. Starting the thread...";
    // Start the thread
    if (!this->start())
    {
        yCError(FineCalibrationCheckerCOMPONENT) << _deviceName << "Failed to start the thread.";
        return false;
    }
    yCDebug(FineCalibrationCheckerCOMPONENT) << _deviceName << "Thread started successfully";
    _deviceStatus = deviceStatus::STARTED;

    return true;
}

bool FineCalibrationChecker::detachAll()
{
    // Stop the thread and detach all devices
    if (this->isRunning())
    {
        yCDebug(FineCalibrationCheckerCOMPONENT) << _deviceName << "Stopping the thread before detaching all devices";
        this->stop();
    }
    if(!remappedControlBoardInterfaces._imultwrap->detachAll() || !remappedRawValuesPublisherInterfaces._imultwrap->detachAll())
    {
        yCError(FineCalibrationCheckerCOMPONENT) << _deviceName << "Failed to detach all devices";
        return false;
    }
    else
    {
        yCDebug(FineCalibrationCheckerCOMPONENT) << _deviceName << "Detached all devices successfully";
    }
    return  true;
}

// Private methods

bool FineCalibrationChecker::isCalibrationSuccessful() const
{
    // Check if the calibration was successful
    return (_deviceStatus == deviceStatus::CALIBRATED);
}

bool FineCalibrationChecker::attachToAllControlBoards(const yarp::dev::PolyDriverList& polyList)
{
    // Attach all control boards to the FineCalibrationChecker
    yCDebug(FineCalibrationCheckerCOMPONENT) << _deviceName << "Attaching all control boards";
    if (!_remappedControlBoardDevice->isValid())
    {
        yCError(FineCalibrationCheckerCOMPONENT) << _deviceName << "Device is not valid. Cannot attach.";
        return false;
    }

    // Setup the control board list given the list of PolyDrivers devices we need to attach to.
    yarp::dev::PolyDriverList controlBoardsList;
    for (size_t i = 0; i < polyList.size(); i++)
    {
        yCDebug(FineCalibrationCheckerCOMPONENT) << _deviceName << "Attaching control board" << polyList[i]->key;
        controlBoardsList.push(const_cast<yarp::dev::PolyDriverDescriptor&>(*polyList[i]));
    }

    yCDebug(FineCalibrationCheckerCOMPONENT) << _deviceName << "Control boards list size:" << controlBoardsList.size();

    // Attach the list of control boards to the control board remapper
    if(!remappedControlBoardInterfaces._imultwrap->attachAll(controlBoardsList))
    {
        yCError(FineCalibrationCheckerCOMPONENT) << _deviceName << "Failed to attach all control boards";
        return false;
    }

    if(!remappedRawValuesPublisherInterfaces._imultwrap->attachAll(controlBoardsList))
    {
        yCError(FineCalibrationCheckerCOMPONENT) << _deviceName << "Failed to attach all control boards to raw values publisher";
        return false;
    }

    yCDebug(FineCalibrationCheckerCOMPONENT) << _deviceName << "Successfully attached all control boards";

    return true;
}

void FineCalibrationChecker::evaluateHardStopPositionDelta(const std::string& key, const std::string& outputFileName)
{

    // Add comments for magic numbers
    const int32_t RAW_VALUES_STRIDE = 3; // Number of raw values per joint in the raw data vector
    const int64_t ICUB_DEGREES_RANGE = (65535); // Range of iCub degrees for rescaling (2^16-1)
    // Get the directory of the output file
    std::filesystem::path outputPath(outputFileName);
    int64_t goldPosition = 0;
    int64_t rawPosition = 0;
    int64_t resolution = 0;
    int64_t rescaledPos = 0;
    double delta = 0;
    std::vector<ItemData> sampleItems = {};

    yarp::os::Bottle* caldeltas = _calibrationDeltasList.get(0).asList();
    yarp::os::Bottle* axesSigns = _axesSignsList.get(0).asList();

    std::ofstream outFile(outputPath);
    if (!outFile.is_open())
    {
        yCError(FineCalibrationCheckerCOMPONENT) << "Unable to open output file:" << outputPath.string();
        return;
    }
    // Add headers to the output CSV file
    outFile << "AxisName,GoldPosition,RescaledPosition,RawPosition,Delta\n";
    yCDebug(FineCalibrationCheckerCOMPONENT) << "Evaluating deltas.....";
    if(auto it = rawDataValuesMap.find(key); it != rawDataValuesMap.end())
    {
        std::vector<std::string> axesNames = {};
        std::vector<std::int32_t> rawData = {};

        // Fill axesNames and rawData vectors
        axesNames = rawDataMetadata.metadataMap.at(it->first).axesNames;
        rawData = it->second;

        std::vector<double> homePositions(axesNames.size(), 0);
        std::vector<double> calibrationDelta(axesNames.size(), 0.0);

        
        for (size_t i = 0; i < axesNames.size(); ++i)
        {
            goldPosition = 0;
            rawPosition = 0;
            resolution = 0;
            delta = 0.0;
            yCDebug(FineCalibrationCheckerCOMPONENT) << "Evaluating axis:" << axesNames[i];
            if (auto it = axesRawGoldenPositionsResMap.find(axesNames[i]); it != axesRawGoldenPositionsResMap.end())
            {
                calibrationDelta[i] = caldeltas->get(i).asFloat64(); // Get the calibration delta for the axis
                double pos = 0.0;
                remappedControlBoardInterfaces._ienc->getEncoder(i, &pos); // Update home position by calling the IEncoders API
                //homePositions[i] = (pos > 0) ? pos : -pos; // Update home position for the axis
                homePositions[i] = (axesSigns->get(i).asInt32() > 0) ? pos : -pos; // Update home position for the axis
                goldPosition = it->second[0];
                resolution = it->second[1];
                rawPosition = rawData[RAW_VALUES_STRIDE*i]; // This because the raw values for tag eoprot_tag_mc_joint_status_addinfo_multienc
                                            // are stored in a vector whose legth is joints_number*3, where each sub-array is made such
                                            // [raw_val_primary_enc, raw_val_secondary_enc, rraw_val_auxiliary_enc]
                                            // and we want the first value for each joint
                rescaledPos = rawPosition * ICUB_DEGREES_RANGE / resolution; // Rescale the encoder raw position to iCubDegrees
                // Gold position and rescaled position are in iCubDegrees --> thus between 0 and 65535
                // Home position is in degrees --> thus between -180 and 180
                // Delta is in degrees
                // The issue here is that the home position can be negative or positive, while the gold and rescaled positions are always positive
                // and when we calculate their difference we are not sure if we are aligned with the home position sign convention
                // Thus we calculate the delta as the absolute value of the difference between gold and rescaled positions
                // divided by the iCub degrees range and multiplied by 360 to convert it to degrees
                // and then we add the home position to it
                // This is the reason why we want the home position to be always positive at line 540
                delta = static_cast<double>((goldPosition - rescaledPos) / (ICUB_DEGREES_RANGE/360)) + homePositions[i]; // Calculate the delta in degrees
                calibrationDelta[i] = (axesSigns->get(i).asInt32() > 0) ? calibrationDelta[i] : -calibrationDelta[i]; // Apply the sign to the calibration delta
                delta += calibrationDelta[i]; // Add the calibration delta to the delta
                yCDebug(FineCalibrationCheckerCOMPONENT) << "GP:" << goldPosition << "HP:" << homePositions[i] << "RSP:" << rescaledPos << "RWP:" << rawPosition << "DD:" << delta;
            }
            else
            {
                yCWarning(FineCalibrationCheckerCOMPONENT) << "This device axes has not ben requested to be checked. Continue...";
                continue;
            }

            // Write to output CSV file
            outFile << axesNames[i] << "," << goldPosition << "," << rescaledPos << "," << rawPosition << "," << delta << "\n";
            sampleItems.push_back({axesNames[i], goldPosition, rescaledPos, rawPosition, delta});
        }
    }
    else
    {
        yCError(FineCalibrationCheckerCOMPONENT) << "Key" << key << "not found in rawDataValuesMap";
        outFile << "Key not found in rawDataValuesMap\n";
    }

    outFile.close();
    yCDebug(FineCalibrationCheckerCOMPONENT) << "Output CSV written to:" << outputPath.string();
    // Generate output image
    if(_withGui) {
        generateOutputImage(1800, 400, sampleItems);
    }
}

void FineCalibrationChecker::generateOutputImage(int frameWidth, int frameHeight, const std::vector<ItemData>& items)
{
    cv::Mat image = cv::Mat::zeros(frameHeight, frameWidth, CV_8UC3);
    image.setTo(cv::Scalar(255, 255, 255)); // White background

    int lineHeight = 30;
    int padding = 10;
    int textOffset = 20;
    int colWidthAvg = (frameWidth - 2 * padding) / 5; // Average width for each column

    // Define headers and column widths (in pixels)
    std::vector<std::string> headers = {"AxisName", "GoldPosition[iCubDegrees]", "RescaledPosition[iCubDegrees]", "RawPosition[Ticks]", "Delta[Degrees]"};
    std::vector<int> colWidths(5, colWidthAvg); // Creates a vector of size 5, all values set to colWidthAvg
    std::vector<int> colX(headers.size());
    colX[0] = padding + 5;
    for (size_t i = 1; i < headers.size(); ++i) {
        colX[i] = colX[i-1] + colWidths[i-1];
    }

    // Draw header row
    int headerY = padding;
    cv::Scalar headerBgColor(220, 220, 220); // Light gray background
    // Draw the filled background for the header row (light gray)
    cv::rectangle(image,
                  cv::Point(padding, headerY),
                  cv::Point(frameWidth - padding, headerY + lineHeight - 5),
                  headerBgColor, cv::FILLED);
    // Draw a border around the header row (darker gray)
    cv::rectangle(image,
                  cv::Point(padding, headerY),
                  cv::Point(frameWidth - padding, headerY + lineHeight - 5),
                  cv::Scalar(100, 100, 100), 1);
    for (size_t c = 0; c < headers.size(); ++c) {
        cv::putText(image, headers[c], cv::Point(colX[c], headerY + textOffset),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);
    }

    // Draw data rows
    for (size_t i = 0; i < items.size(); ++i)
    {
        int y = padding + (i + 1) * lineHeight; // +1 to account for header
        // Draw background rectangle for the row
        cv::Scalar bgColor = getColorForDelta(items[i].val4, 1.0, 3.0); // Green, Orange, Red based on delta thresholds
        cv::rectangle(image,
                      cv::Point(padding, y),
                      cv::Point(frameWidth - padding, y + lineHeight - 5),
                      bgColor, cv::FILLED);

        // Draw border for the row
        cv::rectangle(image,
                      cv::Point(padding, y),
                      cv::Point(frameWidth - padding, y + lineHeight - 5),
                      cv::Scalar(200, 200, 200), 1);

        // Draw each column value aligned with header
        std::vector<std::string> rowVals = {
            items[i].name,
            std::to_string(items[i].val1),
            std::to_string(items[i].val2),
            std::to_string(items[i].val3),
            std::to_string(items[i].val4)
        };
        for (size_t c = 0; c < rowVals.size(); ++c) {
            cv::putText(image, rowVals[c], cv::Point(colX[c], y + textOffset),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);
        }
    }

    cv::imwrite("output_frame.png", image);
    //TODO: remove openCV highgui functions since they are not thread safe and showld be called only in the main thread
    cv::imshow("Output Frame", image);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

cv::Scalar FineCalibrationChecker::getColorForDelta(double delta, double threshold_1, double threshold_2)
{
    if (std::abs(delta) > threshold_2) return cv::Scalar(0, 0, 255);    // Red
    else if (std::abs(delta) > threshold_1) return cv::Scalar(0, 165, 255); // Orange
    else return cv::Scalar(0, 255, 0); // Green
}

