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

// yarp includes
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/Log.h>

// APIs
#include "FineCalibrationCheckerThread.h"

namespace
{
    YARP_LOG_COMPONENT(FineCalibrationCheckerThreadCOMPONENT, "yarp.device.fineCalibrationCheckerThread")
}

FineCalibrationCheckerThread::FineCalibrationCheckerThread(yarp::os::ResourceFinder &rf)
    : yarp::os::Thread(), _deviceName("fineCalibrationChecker"), _portPrefix("/fineCalibrationChecker"),
      _robotName("icub"), _subpartsList(nullptr), _jointsList(nullptr), calibrationStatus(false)
{
    _fineCalibrationCheckerDevice = std::make_unique<yarp::dev::PolyDriver>();
    _rawValuesOublisherDevice = std::make_unique<yarp::dev::PolyDriver>();
    configured = false;
    // Read configuration file
    yarp::os::Bottle &conf_group = rf.findGroup("GENERAL");
    if (conf_group.isNull())
    {
        yWarning() << "Missing GENERAL group! The module uses the default values";
    }
    else
    {
        // Read parameters from the configuration file
        if (conf_group.check("devicename")) { _deviceName = conf_group.find("devicename").asString(); }
        if (conf_group.check("robotname")) { _robotName = conf_group.find("robotname").asString(); }
        if (conf_group.check("remoteControlBoardsList")) { _subpartsList = conf_group.find("remoteControlBoardsList").asList(); }
        if (conf_group.check("axesNamesList")) { _jointsList = conf_group.find("axesNamesList").asList(); }
    }
}

bool FineCalibrationCheckerThread::threadInit()
{
    // TODO: think to eventually move this initialization of calibrationStatus elsewhere
    calibrationStatus = false;

    _robotSubpartsList.resize(_subpartsList->size());
    for (size_t i = 0; i < _subpartsList->size(); i++)
    {
        _robotSubpartsList[i] = _subpartsList->get(i).asString();
        yCDebug(FineCalibrationCheckerThreadCOMPONENT) << _deviceName << "Configuring controlboard" << i << _robotSubpartsList[i];
    }
    for (size_t i = 0; i < _jointsList->size(); i++)
    {
        yCDebug(FineCalibrationCheckerThreadCOMPONENT) << _deviceName << "Configuring joint" << i << _jointsList->get(i).asString();
    }
    //configureDevicesMap(_robotSubpartsList);

    // ************ END OF CONFIGURATION ************

    // Initialize the remote control boards
    yarp::os::Property deviceProperties;
    yarp::os::Bottle axesNames;
    yarp::os::Bottle &axesNamesList = axesNames.addList();
    yarp::os::Bottle remoteControlBoards;
    yarp::os::Bottle &remoteControlBoardsList = remoteControlBoards.addList();

    deviceProperties.put("device", "remotecontrolboardremapper");
    for (size_t i = 0; i < _jointsList->size(); i++)
    {
        axesNamesList.addString(_jointsList->get(i).asString());
    }
    for (size_t i = 0; i < _robotSubpartsList.size(); i++)
    {
        remoteControlBoardsList.addString( "/" +_robotName + "/" + _subpartsList->get(i).asString());
    }
    
    deviceProperties.put("axesNames", axesNames.get(0));
    deviceProperties.put("remoteControlBoards", remoteControlBoards.get(0));
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

    return true;

}

void FineCalibrationCheckerThread::run()
{
    static int debug_counter;
    // Run the calibration thread
    while(!this->isStopping())
    {
        //yCDebug(FineCalibrationCheckerThreadCOMPONENT) << _deviceName << "Thread running";
        if(!configured)
        {
            yCDebug(FineCalibrationCheckerThreadCOMPONENT) << "Configuring calibration";
            // Perform calibration logic here
            // For example, you can call methods on _iremotecalib and _icontrolcalib to perform calibration tasks
            
            configureCalibration();
        }
        else if(!calibrationStatus)
        {
            runCalibration();
            debug_counter = 0;
        }
        else
        {
            if(_icontrolcalib->calibrationDone(0))
            {
                if(!_iravap->getRawDataMap(rawDataValuesMap))
                {
                    yCWarning(FineCalibrationCheckerThreadCOMPONENT) << "telemetryDeviceDumper warning : raw_data_values was not read correctly";
                }
                else
                {
                    if(++debug_counter > 1000)
                    {
                        yCDebug(FineCalibrationCheckerThreadCOMPONENT) << "Get raw values from encoders:";
                        for (auto [key,value] : rawDataValuesMap)
                        {
                            yCDebug(FineCalibrationCheckerThreadCOMPONENT) << "\t key:" << key << "value:" << value;
                        }
                        debug_counter = 0;
                    }
                }
            }
            else
            {
                yCWarning(FineCalibrationCheckerThreadCOMPONENT) << "telemetryDeviceDumper warning : calibration not done";
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
    // if (this->isStopping())
    // {
        
        if(_fineCalibrationCheckerDevice->close())
        {
            yCDebug(FineCalibrationCheckerThreadCOMPONENT) << "Closed device" << _deviceName;
        }
        else
        {
            yCError(FineCalibrationCheckerThreadCOMPONENT) << "Unable to close device" << _deviceName;
        }
         
    // }
}

void FineCalibrationCheckerThread::onStop()
{
    //nothing to do for now
}

// Private methods
bool FineCalibrationCheckerThread::isCalibrationSuccessful() const
{
    // Check if the calibration was successful
    return calibrationStatus;
}


bool FineCalibrationCheckerThread::configureCalibration()
{
    // Configure the calibration parameters here
    // For example, you can set the calibration parameters for each subpart
    // Configure the calibration parameters for the specified subpart
    configured = false;

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
    iCub::rawValuesKeyMetadataMap metadata = {}; // I just need to call it once while configuring (I think) 
    _iravap->getMetadataMap(metadata);
    yCDebug(FineCalibrationCheckerThreadCOMPONENT) << _deviceName << "Configured raw values with metadata";
    for (auto [k, m] : metadata.metadataMap)
    {
        yCDebug(FineCalibrationCheckerThreadCOMPONENT) << _deviceName << "Key: " << k << "\n"
            << "\t rawValueNames: " << m.rawValueNames;
    }
     
    yCDebug(FineCalibrationCheckerThreadCOMPONENT) << _deviceName << "Opened remote calibrator and control calibration interfaces successfully";
    configured = true;
    return true;
}

void FineCalibrationCheckerThread::runCalibration()
{
    // Implement the calibration logic here
    // For example, you can call methods on _iremotecalib and _icontrolcalib to perform calibration tasks

    int motors = 2;
    _imot->getNumberOfMotors(&motors);
    yDebug() << _deviceName << "Starting calibration process on joint" << motors;
    if(_iremotecalib->isCalibratorDevicePresent(&calibrationStatus))
    {
        yDebug() << _deviceName << "Calibration device present!";
        
        if (_iremotecalib->calibrateSingleJoint(0))
        {
            yDebug() << _deviceName << "Calibration started!";
            calibrationStatus = true;
        }
        else
        {
            yError() << _deviceName << "Unable to start calibration!";
            calibrationStatus = false;
            return;
        }
    }
    else
    {
        yError() << _deviceName << "Calibration failed!";
        calibrationStatus = false;
    }
    
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
