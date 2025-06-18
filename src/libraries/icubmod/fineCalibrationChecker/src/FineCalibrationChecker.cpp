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
      _goldPositionsList(yarp::os::Bottle()), _encoderResolutionsList(yarp::os::Bottle()), _deviceStatus(deviceStatus::NONE)
{
    // Initialize device driver as empty PolyDriver
    _remappedControlBoardDevice = std::make_unique<yarp::dev::PolyDriver>();
    _rawValuesPublisherDevice = std::make_unique<yarp::dev::PolyDriver>();
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
        if(property.check("encoderResolutions"))
        {
            yarp::os::Bottle* _encoderResolutions = property.find("encoderResolutions").asList();
            yarp::os::Bottle &encoderResolutions = _encoderResolutionsList.addList();
            
            for (size_t i = 0; i < _encoderResolutions->size(); i++)
            {
                encoderResolutions.addInt32(_encoderResolutions->get(i).asInt32());
            }
        }

        // Use pointer to list to simplify the listing
        yarp::os::Bottle* axes = _axesNamesList.get(0).asList();
        yarp::os::Bottle* goldpos = _goldPositionsList.get(0).asList();
        yarp::os::Bottle* encres = _encoderResolutionsList.get(0).asList();

        // Check list sizes. They must be equal
        if (axes->size() != goldpos->size() || 
            axes->size() != encres->size())
        {
            yCError(FineCalibrationCheckerCOMPONENT) << "Axes names, gold positions and encoder resolutions lists must have the same size. Stopping device...";
            return false;
        }
        else
        {
            yCDebug(FineCalibrationCheckerCOMPONENT) << "Axes names list:" << _axesNamesList.toString();
            yCDebug(FineCalibrationCheckerCOMPONENT) << "Gold positions list:" << goldpos->toString();
            yCDebug(FineCalibrationCheckerCOMPONENT) << "Encoder resolutions list:" << encres->toString();
        }

        
        for (size_t i = 0; i < axes->size(); i++)
        {
            yCDebug(FineCalibrationCheckerCOMPONENT) << "Adding to MAP key:" << axes->get(i).asString()
                << "GP:" << goldpos->get(i).asInt32() << "ER:" << encres->get(i).asInt32();
            axesRawGoldenPositionsResMap[axes->get(i).asString()] = {goldpos->get(i).asInt32(), encres->get(i).asInt32()};
            yCDebug(FineCalibrationCheckerCOMPONENT) << "Array added to MAP:" << axesRawGoldenPositionsResMap.at(axes->get(i).asString())[0]
                << "and" << axesRawGoldenPositionsResMap.at(axes->get(i).asString())[1];
        }
    }
    
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
    rawValuesDeviceProperties.put("device", "rawValuesPublisherClient");
    rawValuesDeviceProperties.put("remote", _remoteRawValuesPort);
    rawValuesDeviceProperties.put("local", "/" + _deviceName + "/rawValuesPublisherClient");

    _rawValuesPublisherDevice->open(rawValuesDeviceProperties);

    if (!_rawValuesPublisherDevice->isValid())
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

    if (_rawValuesPublisherDevice->close())
    {
        yCDebug(FineCalibrationCheckerCOMPONENT) << "Closed raw values publisher device";
    }
    else
    {
        yCError(FineCalibrationCheckerCOMPONENT) << "Unable to close raw values publisher device";
    }
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

    if (!_remappedControlBoardDevice->view(remappedControlBoardInterfaces._icontrolcalib) || remappedControlBoardInterfaces._icontrolcalib == nullptr)
    {
        yCError(FineCalibrationCheckerCOMPONENT) << _deviceName << "Unable to open control calibration interface. Aborting...";
        return false;
    }

    if (!_rawValuesPublisherDevice->view(_iravap) || _iravap == nullptr)
    {
        yCError(FineCalibrationCheckerCOMPONENT) << _deviceName << "Unable to open raw values publisher interface. Aborting...";
        return false;
    }
    

    // Configuring raw values metadata
    rawDataMetadata = {};
    _iravap->getMetadataMap(rawDataMetadata);
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
    static auto lastTimer2Log = std::chrono::steady_clock::now();

    while(!this->isStopping())
    {
        if (_deviceStatus == deviceStatus::CONFIGURED)
        {
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTimerLog).count() > 1000) 
            {
                yCDebug(FineCalibrationCheckerCOMPONENT) << _deviceName << "Running calibration thread with deviceStatus:" << (uint8_t)_deviceStatus;
                lastTimerLog = now;
            }
            yCDebug(FineCalibrationCheckerCOMPONENT) << _deviceName << "Waiting the thread to complete the initialization. Continue...";
            continue;
        }
        else if(_deviceStatus == deviceStatus::STARTED)
        {
            // Get number of axes
            int numAxes = _axesNamesList.size();
            // Check if calibration is done by calling IControlCalibration APIs
            bool calibDone = true;
            for (size_t i = 0; i < numAxes; i++)
            {
                //calibDone &= remappedControlBoardInterfaces._icontrolcalib->calibrationDone(i);
                // TODO: add list of axis that did not perform calibration to be rechecked
            }
            if (!calibDone)
            {
                yCWarning(FineCalibrationCheckerCOMPONENT) << _deviceName << "Calib not complete for all axis. Waiting for calibration to finish...";
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
            if(!_iravap->getRawDataMap(rawDataValuesMap))
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
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTimer2Log).count() > 5000) 
            {
                yCDebug(FineCalibrationCheckerCOMPONENT) << _deviceName << "Operation completed successfully. Waiting for yarprobotinterface stop";
                lastTimer2Log = now;
            }
        }
        else
        {
            yCError(FineCalibrationCheckerCOMPONENT) << _deviceName << "Device is in unknown state. Stopping thread...";
            this->stop();
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
    if(!remappedControlBoardInterfaces._imultwrap->detachAll())
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

    yCDebug(FineCalibrationCheckerCOMPONENT) << _deviceName << "Successfully attached all control boards";

    return true;
}

void FineCalibrationChecker::evaluateHardStopPositionDelta(const std::string& key, const std::string& outputFileName)
{
    // Get the directory of the output file
    std::filesystem::path outputPath(outputFileName);
    int32_t goldPosition = 0;
    int32_t rawPosition = 0;
    int32_t resolution = 0;
    int32_t rescaledPos = 0;
    int32_t delta = 0;
    std::vector<ItemData> sampleItems = {};

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
        _iravap->getAxesNames(it->first, axesNames);
        _iravap->getRawData(it->first, rawData);

        for (size_t i = 0; i < axesNames.size(); ++i)
        {
            goldPosition = 0;
            rawPosition = 0;
            resolution = 0;
            delta = 0;
            yCDebug(FineCalibrationCheckerCOMPONENT) << "Evaluating axis:" << axesNames[i];
            yCDebug(FineCalibrationCheckerCOMPONENT) << "First value added to MAP:" << axesRawGoldenPositionsResMap.at(axesNames[i])[0] 
                << "and second value:" << axesRawGoldenPositionsResMap.at(axesNames[i])[1];
            if (auto it = axesRawGoldenPositionsResMap.find(axesNames[i]); it != axesRawGoldenPositionsResMap.end())
            {
                goldPosition = it->second[0];
                resolution = it->second[1];
                rawPosition = rawData[3*i]; // This because the raw values for tag eoprot_tag_mc_joint_status_addinfo_multienc 
                                            // are stored in a vector whose legth is joints_number*3, where each sub-array is made such
                                            // [raw_val_primary_enc, raw_val_secondary_enc, rraw_val_auxiliary_enc] 
                                            // and we want the first value for each joint
                rescaledPos = goldPosition * resolution / 65535; // Rescale the position (in iCubDegrees) to the encoder full resolution             
                delta = std::abs(rescaledPos - rawPosition);

                yCDebug(FineCalibrationCheckerCOMPONENT) << "GP:" << goldPosition << "RSP:" << rescaledPos << "RWP:" << rawPosition << "DD:" << delta;
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

    generateOutputImage(1000, 400, sampleItems);
}

void FineCalibrationChecker::configureDevicesMap(std::vector<std::string> list)
{
    // Configure the devices map based on the provided list of subpart names

    // std::vector<std::string> input = list; // Example input list
    // std::vector<std::string> desired_order = _robotSubpartsWrapper; // Example desired order

    // // Step 1: Create the order map
    // std::unordered_map<std::string, int> order_map;
    // for (size_t i = 0; i < _robotSubpartsWrapper.size(); ++i) {
    //     order_map[_robotSubpartsWrapper[i]] = i;
    // }

    // // Step 2: Separate known and unknown elements
    // std::vector<std::string> known_items;
    // std::vector<std::string> unknown_items;

    // for (const auto& item : list) {
    //     if (order_map.count(item)) {
    //         known_items.push_back(item);
    //     } else {
    //         unknown_items.push_back(item);  // Handle unknowns
    //     }
    // }

    // // Step 3: Sort known items using desired order
    // std::sort(known_items.begin(), known_items.end(), [&](const std::string& a, const std::string& b) {
    //     return order_map[a] < order_map[b];
    // });

    // // Step 4: Optionally detect missing elements
    // std::set<std::string> input_set(list.begin(), list.end());
    // std::vector<std::string> missing_items;
    // for (const auto& expected : _robotSubpartsWrapper) {
    //     if (!input_set.count(expected)) {
    //         missing_items.push_back(expected);
    //     }
    // }

    // // Output results
    // std::cout << "Sorted known items:\n";
    // for (const auto& item : known_items) std::cout << item << " ";
    // std::cout << "\n";

    // if (!unknown_items.empty()) {
    //     std::cout << "Unknown items (not in desired order):\n";
    //     for (const auto& item : unknown_items) std::cout << item << " ";
    //     std::cout << "\n";
    // }

    // if (!missing_items.empty()) {
    //     std::cout << "Missing expected items:\n";
    //     for (const auto& item : missing_items) std::cout << item << " ";
    //     std::cout << "\n";
    // }

}

void FineCalibrationChecker::generateOutputImage(int frameWidth, int frameHeight, const std::vector<ItemData>& items)
{
    cv::Mat image = cv::Mat::zeros(frameHeight, frameWidth, CV_8UC3);
    image.setTo(cv::Scalar(255, 255, 255)); // White background

    int lineHeight = 30;
    int padding = 10;
    int textOffset = 20;

    // Define headers and column widths (in pixels)
    std::vector<std::string> headers = {"AxisName", "GoldPosition", "RescaledPosition", "RawPosition", "Delta"};
    std::vector<int> colWidths = {140, 180, 200, 180, 120}; // Adjust as needed for your data
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
        cv::Scalar bgColor = getColorForDelta(items[i].val4, 1, 3);
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

cv::Scalar FineCalibrationChecker::getColorForDelta(int32_t delta, int32_t threshold_1, int32_t threshold_2)
{
    if (delta > threshold_2) return cv::Scalar(0, 0, 255);    // Red
    else if (delta > threshold_1) return cv::Scalar(0, 165, 255); // Orange
    else return cv::Scalar(0, 255, 0); // Green
}

