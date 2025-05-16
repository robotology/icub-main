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

// Standard includes
#include <iostream>
#include <algorithm>
#include <unordered_map>
#include <set>
#include <utility>

// YARP includes
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/Log.h>

// APIs
#include "FineCalibrationCheckerThread.h"

FineCalibrationCheckerThread::FineCalibrationCheckerThread(yarp::os::ResourceFinder &rf)
    : yarp::os::Thread(), _deviceName("fineCalibrationChecker"), _portPrefix("/fineCalibrationChecker"),
      _robotName("icub"), _subpartsList(nullptr), _jointsList(nullptr), calibrationStatus(false)
{
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
        yDebug() << _deviceName << "Subpart " << i << ": " << _robotSubpartsList[i];
    }
    configureDevicesMap(_robotSubpartsList);

    // ************ END OF CONFIGURATION ************

    // Initialize remote raw values publisher device
    yarp::os::Property deviceProperties;
    deviceProperties.put("device", "remotecontrolboardremapper");
    yarp::os::Bottle remoteControlBoards;
    yarp::os::Bottle &remoteControlBoardsList = remoteControlBoards.addList();
    remoteControlBoardsList.addString("/" + _robotName + "/" + _subpartsList->get(0).asString());
    deviceProperties.put("remoteControlBoards",remoteControlBoards.get(0));
    deviceProperties.put("localPortPrefix", "/" + _deviceName);

    yDebug() << _deviceName << "++++ config:\n"
        << "\t robotname: " << _robotName << "\n"
        << "\t portprefix: " << _subpartsList->get(0).asString() << "\n";

    // Open the device driver
    for (size_t i = 0; i < _subpartsList->size(); i++)
    {

        _fineCalibrationCheckerDevicesMap[_robotSubpartsList[i]]->open(deviceProperties);

        if (!_fineCalibrationCheckerDevicesMap[_robotSubpartsList[i]]->isValid())
        {
            yError() << _deviceName << "Unable to open device driver. Aborting...";
            return false;
        }

        /**
        if (!_fineCalibrationCheckerDevicesMap[_robotSubpartsList[i]].view(_iravap) || _iravap == nullptr)
        {
            yError() << _deviceName << "Unable to open raw values publisher interface. Aborting...";
            return false;
        }

        // Initialize parametric calibrator device
        if (!_fineCalibrationCheckerDevicesMap[_robotSubpartsList[i]].view(_iremotecalib) || _iremotecalib == nullptr)
        {
            yError() << _deviceName << "Unable to open remote calibrator interface. Aborting...";
            return false;
        }

        if (!_fineCalibrationCheckerDevicesMap[_robotSubpartsList[i]].view(_icontrolcalib) || _icontrolcalib == nullptr)
        {
            yError() << _deviceName << "Unable to open control calibration interface. Aborting...";
            return false;
        }

        if (!_fineCalibrationCheckerDevicesMap[_robotSubpartsList[i]].view(_imot) || _imot == nullptr)
        {
            yError() << _deviceName << "Unable to open motor interface. Aborting...";
            return false;
        }
        */
    }

    // Configuring raw values metadata
    // iCub::rawValuesKeyMetadataMap metadata = {}; // I just need to call it once while configuring (I think) 
    // _iravap->getMetadataMap(metadata);
    // yDebug() << _deviceName << "Configured raw values with metadata";
    /**
    for (auto [k, m] : metadata.metadataMap)
    {
        yDebug() << _deviceName << "Key: " << k << "\n"
            << "\t rawValueType: " << m.rawValueType << "\n"
            << "\t rawValueSize: " << m.rawValueSize << "\n"
            << "\t rawValueNames: " << yarp::os::join(m.rawValueNames, ", ");
    }
     */
    return true;

}

void FineCalibrationCheckerThread::run()
{
    // Run the calibration thread
    // if (_fineCalibrationCheckerDevice.isValid())
    // {
    //     // Perform calibration logic here
    //     // For example, you can call methods on _iremotecalib and _icontrolcalib to perform calibration tasks
    //     for (size_t i = 0; i < _subpartsList->size(); i++)
    //     {
    //         this->configureCalibration(_subpartsList[i]);
    //         this->runCalibration();
    //     }
        
    // }
}

void FineCalibrationCheckerThread::threadRelease()
{
    // Release resources and close the device
    // if (_fineCalibrationCheckerDevice.isValid())
    // {
    //     _fineCalibrationCheckerDevice.close();
    // }
}

void FineCalibrationCheckerThread::onStop()
{
    // Stop the calibration thread
    // if (_fineCalibrationCheckerDevice.isValid())
    // {
    //     _iremotecalib->stopCalibration();
    // }
}

bool FineCalibrationCheckerThread::open(yarp::os::Searchable &config)
{

    return true;
}

bool FineCalibrationCheckerThread::close()
{
    return true;
}

// Private methods
bool FineCalibrationCheckerThread::isCalibrationSuccessful() const
{
    // Check if the calibration was successful
    return calibrationStatus;
}


void FineCalibrationCheckerThread::configureCalibration(std::string subpartName)
{
    // Configure the calibration parameters here
    // For example, you can set the calibration parameters for each subpart
    // Configure the calibration parameters for the specified subpart
    yDebug() << _deviceName << "Configuring calibration for subpart " << subpartName;

    for (size_t i = 0; i < _subpartsList->size(); i++)
    {
        // Set calibration parameters for each subpart
        // You can use _iremotecalib and _icontrolcalib to set the parameters
        yDebug() << _deviceName << "Configuring calibration for subpart " << _subpartsList->get(i).asString();


        _fineCalibrationCheckerDevicesMap[subpartName]->view(_iremotecalib);
        _fineCalibrationCheckerDevicesMap[subpartName]->view(_icontrolcalib);

        runCalibration();
    }
}

void FineCalibrationCheckerThread::runCalibration()
{
    // Implement the calibration logic here
    // For example, you can call methods on _iremotecalib and _icontrolcalib to perform calibration tasks

    // yDebug() << _deviceName << "Starting calibration process"; 
    // if(_iremotecalib->calibrateWholePart())
    // {
    //     yDebug() << _deviceName << "Calibration for subpart " << _subpartList[i] << " started!";
        
    //     int motors = 0;
    //     _imot->getNumberOfMotors(&motors);
    //     for (size_t i = 0; i < motors; i++)
    //     {
    //         /* code */
    //     }
        
    //     if (_icontrolcalib->calibrationDone())
    //     {
    //         yDebug() << _deviceName << "Calibration for subpart " << _subpartList[i] << " completed successfully!";
    //     }
    //     else
    //     {
    //         yError() << _deviceName << "Calibration for subpart " << _subpartList[i] << " failed!";
    //     }
        
    //     calibrationStatus = true; // Set the calibration status based on the result of the calibration process

    //     //TODO: we need to check if the raw position read overlaps with the expected position
    //     bool ok;
    //     ok = iravap->getRawDataMap(rawDataValuesMap);
    //     if (!ok)
    //     {
    //         yWarning() << "telemetryDeviceDumper warning : raw_data_values was not read correctly";
    //     }
    //     else
    //     {
    //         for (auto [key,value] : rawDataValuesMap)
    //         {
    //             bufferManager.push_back(value, "raw_data_values::"+key);
    //         }
    //     }
    // }
    // else
    // {
    //     yError() << _deviceName << "Calibration for subpart " << _subpartList[i] << " failed!";
    //     calibrationStatus = false;
    // }
    
}

void FineCalibrationCheckerThread::configureDevicesMap(std::vector<std::string> list)
{
    // Configure the devices map based on the provided list of subpart names

    std::vector<std::string> input = list; // Example input list
    std::vector<std::string> desired_order = _robotSubpartsWrapper; // Example desired order

    // Step 1: Create the order map
    std::unordered_map<std::string, int> order_map;
    for (size_t i = 0; i < _robotSubpartsWrapper.size(); ++i) {
        order_map[_robotSubpartsWrapper[i]] = i;
    }

    // Step 2: Separate known and unknown elements
    std::vector<std::string> known_items;
    std::vector<std::string> unknown_items;

    for (const auto& item : list) {
        if (order_map.count(item)) {
            known_items.push_back(item);
        } else {
            unknown_items.push_back(item);  // Handle unknowns
        }
    }

    // Step 3: Sort known items using desired order
    std::sort(known_items.begin(), known_items.end(), [&](const std::string& a, const std::string& b) {
        return order_map[a] < order_map[b];
    });

    // Step 4: Optionally detect missing elements
    std::set<std::string> input_set(list.begin(), list.end());
    std::vector<std::string> missing_items;
    for (const auto& expected : _robotSubpartsWrapper) {
        if (!input_set.count(expected)) {
            missing_items.push_back(expected);
        }
    }

    // Output results
    std::cout << "Sorted known items:\n";
    for (const auto& item : known_items) std::cout << item << " ";
    std::cout << "\n";

    if (!unknown_items.empty()) {
        std::cout << "Unknown items (not in desired order):\n";
        for (const auto& item : unknown_items) std::cout << item << " ";
        std::cout << "\n";
    }

    if (!missing_items.empty()) {
        std::cout << "Missing expected items:\n";
        for (const auto& item : missing_items) std::cout << item << " ";
        std::cout << "\n";
    }

    for (const auto& item : known_items)
    {
        _fineCalibrationCheckerDevicesMap.insert(std::make_pair(item, new yarp::dev::PolyDriver));
    }
}
