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

// std includes
#include <iostream>
#include <algorithm>
#include <unordered_map>
#include <set>
#include <utility>
#include <fstream>
#include <sstream>
#include <filesystem>

// yarp includes
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/Log.h>
#include <yarp/os/Searchable.h>

// APIs
#include "FineCalibrationCheckerThread.h"

namespace
{
    YARP_LOG_COMPONENT(FineCalibrationCheckerThreadCOMPONENT, "yarp.device.fineCalibrationCheckerThread")
}

FineCalibrationCheckerThread::FineCalibrationCheckerThread(yarp::os::ResourceFinder &rf)
    : yarp::os::Thread(), _deviceName("fineCalibrationChecker"), _portPrefix("/fineCalibrationChecker"),
      _robotName("icub"), _controlBoardsList(yarp::os::Bottle()), _axesNamesList(yarp::os::Bottle()), _deviceStatus(deviceStatus::NONE)
{
    // Initialize device driver as empty PolyDriver
    _fineCalibrationCheckerDevice = std::make_unique<yarp::dev::PolyDriver>();
    _rawValuesOublisherDevice = std::make_unique<yarp::dev::PolyDriver>();

    // Read configuration file
    yarp::os::Searchable &conf_group = rf.findGroup("GENERAL");
    if (conf_group.isNull())
    {
        yCWarning(FineCalibrationCheckerThreadCOMPONENT) << "Missing GENERAL group! The module uses the default values";
    }
    else
    {
        // Read parameters from the configuration file
        if (conf_group.check("devicename")) { _deviceName = conf_group.find("devicename").asString(); }
        if (conf_group.check("robotname")) { _robotName = conf_group.find("robotname").asString(); }
        if (conf_group.check("remoteControlBoardsList")) 
        {
            yarp::os::Bottle* subpartsList = conf_group.find("remoteControlBoardsList").asList();
            yarp::os::Bottle &controlBoards = _controlBoardsList.addList();
            for (size_t i = 0; i < subpartsList->size(); i++)
            {
                controlBoards.addString("/" + _robotName + "/" + subpartsList->get(i).asString());
            }
        }
        if (conf_group.check("axesNamesList")) 
        {
            yarp::os::Bottle* _jointsList = conf_group.find("axesNamesList").asList();
            yarp::os::Bottle &axesNames = _axesNamesList.addList();
            for (size_t i = 0; i < _jointsList->size(); i++)
            {
                axesNames.addString(_jointsList->get(i).asString());
            }
        }
    }

    // Read raw golden hard-stop positions from csv file and save them in map
    std::ifstream file("zeroPositionsData.csv");
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string key, value;
        if (std::getline(iss, key, ',') && std::getline(iss, value)) {
            axesRawGoldenPositionsMap[key] = std::stoi(value);
        }
        yCDebug(FineCalibrationCheckerThreadCOMPONENT) << "Zero GOLDEN position for joint " << key << " is " << axesRawGoldenPositionsMap[key];
    }
    
    _deviceStatus = deviceStatus::INITIALIZED;
}

bool FineCalibrationCheckerThread::threadInit()
{
    // Just for debug. Check the list of controlBoards and axesNames
    yCDebug(FineCalibrationCheckerThreadCOMPONENT) << _deviceName << "Configuring controlboards" << _controlBoardsList.toString();
    yCDebug(FineCalibrationCheckerThreadCOMPONENT) << _deviceName << "Configuring joint" << _axesNamesList.toString();
    

    // Initialize the remote control boards
    yarp::os::Property deviceProperties;

    deviceProperties.put("device", "remotecontrolboardremapper");
    deviceProperties.put("axesNames", _axesNamesList.get(0));
    deviceProperties.put("remoteControlBoards", _controlBoardsList.get(0));
    deviceProperties.put("localPortPrefix", "/" + _deviceName);

    yCDebug(FineCalibrationCheckerThreadCOMPONENT) << _deviceName << "Configuring device driver with properties:" << deviceProperties.toString();

    _fineCalibrationCheckerDevice->open(deviceProperties);

    if (!_fineCalibrationCheckerDevice->isValid())
    {
        yCError(FineCalibrationCheckerThreadCOMPONENT) << _deviceName << "Unable to open device driver. Aborting...";
        return false;
    }

    yarp::os::Property rawValuesDeviceProperties;
    rawValuesDeviceProperties.put("device", "rawValuesPublisherClient");
    rawValuesDeviceProperties.put("remote", "/" + _robotName + "/setup_rawval");
    rawValuesDeviceProperties.put("local", "/" + _deviceName + "/rawValuesPublisherClient");

    _rawValuesOublisherDevice->open(rawValuesDeviceProperties);

    if (!_rawValuesOublisherDevice->isValid())
    {
        yCError(FineCalibrationCheckerThreadCOMPONENT) << _deviceName << "Unable to open raw values device driver. Aborting...";
        return false;
    }    
    
    yCDebug(FineCalibrationCheckerThreadCOMPONENT) << _deviceName << "Opened all devices successfully";

    _deviceStatus = deviceStatus::OPENED;
    return true;

}

void FineCalibrationCheckerThread::run()
{
    // Run the calibration thread
    yCDebug(FineCalibrationCheckerThreadCOMPONENT) << _deviceName << "Thread started to run";

    while(!this->isStopping())
    {

        if(_deviceStatus == deviceStatus::OPENED)
        {
            yCDebug(FineCalibrationCheckerThreadCOMPONENT) << "Configuring calibration device";
            if(!configureCalibration())
            {
                yCError(FineCalibrationCheckerThreadCOMPONENT) << "Failed to configure device";
            }
            else
            {
                _deviceStatus = deviceStatus::CONFIGURED;
            }
        }
        else if(_deviceStatus == deviceStatus::CONFIGURED)
        {
            if(!runCalibration())
            {
                yCError(FineCalibrationCheckerThreadCOMPONENT) << "Failed to calibrate axesNames attached to the device";
            }
            else
            {
                _deviceStatus = deviceStatus::CALIBRATED;
            }
        }
        else
        {
            if((_icontrolcalib->calibrationDone(0)) && (_deviceStatus == deviceStatus::CALIBRATED))
            {
                if(!_iravap->getRawDataMap(rawDataValuesMap))
                {
                    yCWarning(FineCalibrationCheckerThreadCOMPONENT) << "telemetryDeviceDumper warning : raw_data_values was not read correctly";
                }
                else
                {
                    yCDebug(FineCalibrationCheckerThreadCOMPONENT) << "Get raw values from encoders:";
                    for (auto [key,value] : rawDataValuesMap)
                    {
                        yCDebug(FineCalibrationCheckerThreadCOMPONENT) << "\t key:" << key << "value:" << value;
                    }
                    
                    // Here we have to evaluate the delta between the raw golen position 
                    // and the raw position read at the hard stop per each axis
                    evaluateHardStopPositionDelta(_rawValuesTag, "zeroPositionsData.csv", "zeroPositionsDataDelta.csv");

                    _deviceStatus = deviceStatus::END_POSITION_CHECKED;
                }
                if(_deviceStatus == deviceStatus::END_POSITION_CHECKED)
                {
                    if(!_iremotecalib->homingSingleJoint(0))
                    {
                        yCError(FineCalibrationCheckerThreadCOMPONENT) << "Failed to homing axesNames attached to the device";
                    }
                    else
                    {
                        yCDebug(FineCalibrationCheckerThreadCOMPONENT) << "Successfully homing axesNames attached to the device";
                        _deviceStatus = deviceStatus::IN_HOME_POSITION;
                    }
                }
            }
        } 
    }
    
    yCDebug(FineCalibrationCheckerThreadCOMPONENT) << _deviceName << "Thread stopping";
    // Perform any cleanup or finalization tasks here
    // For example, you can close the device or release resources
    
}

void FineCalibrationCheckerThread::threadRelease()
{
    // Release resources and close the device
    
    // Close all devices
    if(this->isStopping())
    {
        if(_fineCalibrationCheckerDevice->close())
        {
            yCDebug(FineCalibrationCheckerThreadCOMPONENT) << "Closed device" << _deviceName;
        }
        else
        {
            yCError(FineCalibrationCheckerThreadCOMPONENT) << "Unable to close device" << _deviceName;
        }
    }
    
}

void FineCalibrationCheckerThread::onStop()
{
    //nothing to do for now
}

// Private methods
bool FineCalibrationCheckerThread::isCalibrationSuccessful() const
{
    // Check if the calibration was successful
    return (_deviceStatus == deviceStatus::CALIBRATED);
}


bool FineCalibrationCheckerThread::configureCalibration()
{
    // Configure the calibration parameters here
    // For example, you can set the calibration parameters for each subpart
    // Configure the calibration parameters for the specified subpart

    if (!_fineCalibrationCheckerDevice->view(_imot) || _imot == nullptr)
    {
        yCError(FineCalibrationCheckerThreadCOMPONENT) << _deviceName << "Unable to open motor interface. Aborting...";
        return false;
    }
    // Initialize parametric calibrator device
    if (!_fineCalibrationCheckerDevice->view(_iremotecalib) || _iremotecalib == nullptr)
    {
        yCError(FineCalibrationCheckerThreadCOMPONENT) << _deviceName << "Unable to open remote calibrator interface. Aborting...";
        return false;
    }

    if (!_fineCalibrationCheckerDevice->view(_icontrolcalib) || _icontrolcalib == nullptr)
    {
        yCError(FineCalibrationCheckerThreadCOMPONENT) << _deviceName << "Unable to open control calibration interface. Aborting...";
        return false;
    }

    if (!_rawValuesOublisherDevice->view(_iravap) || _iravap == nullptr)
    {
        yCError(FineCalibrationCheckerThreadCOMPONENT) << _deviceName << "Unable to open raw values publisher interface. Aborting...";
        return false;
    }
    

    // Configuring raw values metadata
    rawDataMetadata = {};
    _iravap->getMetadataMap(rawDataMetadata);
    yCDebug(FineCalibrationCheckerThreadCOMPONENT) << _deviceName << "Configured raw values with metadata";
    for (auto [k, m] : rawDataMetadata.metadataMap)
    {
        yCDebug(FineCalibrationCheckerThreadCOMPONENT) << _deviceName
            << "\t Key: " << k << "\n"
            << "\t axesName: " << m.axesNames << "\n"
            << "\t rawValueNames: " << m.rawValueNames;
    }
     
    yCDebug(FineCalibrationCheckerThreadCOMPONENT) << _deviceName << "Opened remote calibrator and control calibration interfaces successfully";
    return true;
}

bool FineCalibrationCheckerThread::runCalibration()
{
    // Implement the calibration logic here
    // For example, you can call methods on _iremotecalib and _icontrolcalib to perform calibration tasks

    int motors = 0;
    _imot->getNumberOfMotors(&motors);
    bool isCalibratorSet = false;
    yCDebug(FineCalibrationCheckerThreadCOMPONENT) << _deviceName << "Starting calibration process on joint" << motors;
    if(_iremotecalib->isCalibratorDevicePresent(&isCalibratorSet))
    {
        yCDebug(FineCalibrationCheckerThreadCOMPONENT) << _deviceName << "Calibration device present!. Go on with calibration procedure";
        
        if (_iremotecalib->calibrateSingleJoint(0))
        {
            yCDebug(FineCalibrationCheckerThreadCOMPONENT) << _deviceName << "Calibration of joint ended successfully!";
        }
        else
        {
            yCError(FineCalibrationCheckerThreadCOMPONENT) << _deviceName << "Unable to start calibration!. Closing!";
            return false;
        }
    }
    else
    {
        yCError(FineCalibrationCheckerThreadCOMPONENT) << _deviceName << "Calibration device not set yet. Closing!";
        return false;
    }
    
    return true;
}

void FineCalibrationCheckerThread::evaluateHardStopPositionDelta(const std::string& key, const std::string& inputFileName, const std::string& outputFileName)
{
    // Get the directory of the input file
    std::filesystem::path inputPath(inputFileName);
    std::filesystem::path outputPath = inputPath.parent_path() / outputFileName;
    int32_t goldPosition = 0;
    int32_t rawPosition = 0;
    int32_t delta = 0;
    std::vector<ItemData> sampleItems = {};

    std::ofstream outFile(outputPath);
    if (!outFile.is_open()) 
    {
        yCError(FineCalibrationCheckerThreadCOMPONENT) << "Unable to open output file:" << outputPath.string();
        return;
    }

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
            delta = 0;
            if (auto it = axesRawGoldenPositionsMap.find(axesNames[i]); it != axesRawGoldenPositionsMap.end())
            {
                goldPosition = it->second;
                rawPosition = rawData[3*i]; // This because the raw values for tag eoprot_tag_mc_joint_status_addinfo_multienc 
                                            // are stored in a vector whose legth is joints_number*3, where each sub-array is made such
                                            // [raw_val_primary_enc, raw_val_secondary_enc, rraw_val_auxiliary_enc] 
                                            // and we want the first value for each joint
                delta = std::abs(goldPosition - rawPosition);
            }
            
            // Write to output CSV file
            outFile << axesNames[i] << "," << goldPosition << "," << rawPosition << "," << delta << "\n";
            sampleItems.push_back({axesNames[i], goldPosition, rawPosition, delta});
        }
    }
    else
    {
        yCError(FineCalibrationCheckerThreadCOMPONENT) << "Key" << key << "not found in rawDataValuesMap";
        outFile << "Key not found in rawDataValuesMap\n";
    }

    outFile.close();
    yCDebug(FineCalibrationCheckerThreadCOMPONENT) << "Output CSV written to:" << outputPath.string();

    generateOutputImage(300, 150, sampleItems);
}

void FineCalibrationCheckerThread::configureDevicesMap(std::vector<std::string> list)
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

void FineCalibrationCheckerThread::generateOutputImage(int cellWidth, int cellHeight, const std::vector<ItemData>& items)
{

    const int cols = 1, rows = 1; // Adjust the number of columns and rows as needed
    // const int cellWidth = 300, cellHeight = 150;
    const int canvasWidth = cols * cellWidth;
    const int canvasHeight = rows * cellHeight;

    cv::Mat image = cv::Mat::zeros(canvasHeight, canvasWidth, CV_8UC3);
    image.setTo(cv::Scalar(255, 255, 255)); // White background


    for (int row = 0; row < rows; ++row) 
    {
        for (int col = 0; col < cols; ++col) 
        {
            int x = col * cellWidth;
            int y = row * cellHeight;

            int lineHeight = 20;
            int padding = 10;
            cv::Point topLeft = cv::Point(x, y);
            // Draw each item as a row
            for (size_t i = 0; i < items.size(); ++i) 
            {
                const ItemData& item = items[i];
                int y = topLeft.y + padding + i * (lineHeight + 5);
                cv::Point p1(topLeft.x + padding, y);
                cv::Point p2(topLeft.x + cellWidth - padding, y + lineHeight);

                // Background color based on val3
                cv::Scalar bgColor = getColorForDelta(item.val3, 1, 3);
                cv::rectangle(image, p1, p2, bgColor, cv::FILLED);

                // Text content
                std::string text = item.name + "  " +
                                std::to_string(item.val1) + " " +
                                std::to_string(item.val2) + " " +
                                std::to_string(item.val3);

                cv::putText(image, text, cv::Point(p1.x + 5, y + lineHeight - 5),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
            }

            // Draw border of the cell
            cv::rectangle(image, topLeft,
                        cv::Point(topLeft.x + cellWidth, topLeft.y + cellHeight),
                        cv::Scalar(200, 200, 200), 2);
        }
    }

    cv::imwrite("grid_output.png", image); 
    cv::imshow("Grid Output", image);     // Open a window with the rendered image
    cv::waitKey(0);                       // Wait until any key is pressed
    cv::destroyAllWindows();             // Clean up

}

cv::Scalar FineCalibrationCheckerThread::getColorForDelta(int32_t delta, int32_t threshold_1, int32_t threshold_2)
{
    if (delta > threshold_2) return cv::Scalar(0, 0, 255);    // Red
    else if (delta > threshold_1) return cv::Scalar(0, 165, 255); // Orange
    else return cv::Scalar(0, 255, 0); // Green
}

