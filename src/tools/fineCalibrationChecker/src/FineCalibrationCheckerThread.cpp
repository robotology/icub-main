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
// APIs
#include "FineCalibrationCheckerThread.h"

bool FineCalibrationCheckerThread::threadInit()
{
    // TODO: think to eventually move this initialization of calibrationStatus elsewhere
    calibrationStatus = false;

    // Read configuration file
    Bottle &conf_group = rf.findGroup("GENERAL");
    if (conf_group.isNull())
    {
        yWarning() << "Missing GENERAL group! The module uses the default values";
    }
    else
    {
        // Read parameters from the configuration file
        if (conf_group.check("devicename")) { _deviceName = conf_group.find("devicename").asString(); }
        if (conf_group.check("robotname")) { _robotName = conf_group.find("robotname").asString(); }
        if (conf_group.check("subpartslist")) { _subpartsList = conf_group.find("subpartslist").asList(); }
        if (conf_group.check("jointslist")) { _jointsList = conf_group.find("jointslist").asList(); }
    }

    // Initialize remote raw values publisher device
    Property deviceProperties;
    deviceProperties.put("device", m_deviceName);
    deviceProperties.put("remote", "/" + m_robotName + m_portPrefix);
    deviceProperties.put("local", "/" + m_robotName + m_portPrefix);

    yDebug() << m_deviceName << "++++ config:\n"
        << "\t portprefix: " << m_portPrefix << "\n"
        << "\t robotname: " << m_robotName << "\n";

    _robotSubpartsList.resize(_subpartsList.size());
    for (size_t i = 0; i < _subpartsList.size(); i++)
    {
        _robotSubpartsList[i] = _subpartsList->get(i).asString();
        yDebug() << m_deviceName << "Subpart " << i << ": " << _robotSubpartsList[i];
    }
    configureDevicesMap(_robotSubpartsList);

    // ************ END OF CONFIGURATION ************
    
    // Open the device driver
    for (size_t i = 0; i < _subpartsList.size(); i++)
    {

        _fineCalibrationCheckerDevice.open(deviceProperties);

        if (!_fineCalibrationCheckerDevice.isValid())
        {
            yError() << m_deviceNname << "Unable to open device driver. Aborting...";
            return false;
        }

        if (!_fineCalibrationCheckerDevice.view(_iravap) || _iravap == nullptr)
        {
            yError() << m_deviceName << "Unable to open raw values publisher interface. Aborting...";
            return false;
        }

        // Initialize parametric calibrator device
        if (!_fineCalibrationCheckerDevice.view(_iremotecalib) || _iremotecalib == nullptr)
        {
            yError() << m_deviceName << "Unable to open remote calibrator interface. Aborting...";
            return false;
        }

        if (!_fineCalibrationCheckerDevice.view(_icontrolcalib) || _icontrolcalib == nullptr)
        {
            yError() << m_deviceName << "Unable to open control calibration interface. Aborting...";
            return false;
        }

        if (!_fineCalibrationCheckerDevice.view(_imot) || _imot == nullptr)
        {
            yError() << m_deviceName << "Unable to open motor interface. Aborting...";
            return false;
        }
    }

    // Configuring raw values metadata
    rawValuesKeyMetadataMap metadata = {}; // I just need to call it once while configuring (I think) 
    iravap->getMetadataMap(metadata);
    yDebug() << m_deviceName << "Configured raw values with metadata";
    for (auto [k, m] : metadata.metadataMap)
    {
        yDebug() << m_deviceName << "Key: " << k << "\n"
            << "\t rawValueType: " << m.rawValueType << "\n"
            << "\t rawValueSize: " << m.rawValueSize << "\n"
            << "\t rawValueNames: " << yarp::os::join(m.rawValueNames, ", ");
    }

    return true;

}

void FineCalibrationCheckerThread::run()
{
    // Run the calibration thread
    if (_fineCalibrationCheckerDevice.isValid())
    {
        // Perform calibration logic here
        // For example, you can call methods on _iremotecalib and _icontrolcalib to perform calibration tasks
        for (size_t i = 0; i < _subpartsList.size(); i++)
        {
            this->configureCalibration(_subpartsList[i]);
            this->runCalibration();
        }
        
    }
}

void FineCalibrationCheckerThread::threadRelease()
{
    // Release resources and close the device
    if (_fineCalibrationCheckerDevice.isValid())
    {
        _fineCalibrationCheckerDevice.close();
    }
}

void FineCalibrationCheckderThread::onStop()
{
    // Stop the calibration thread
    if (_fineCalibrationCheckerDevice.isValid())
    {
        _iremotecalib->stopCalibration();
    }
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
    yDebug() << m_deviceName << "Configuring calibration for subpart " << subpartName;

    for (size_t i = 0; i < _subpartsList.size(); i++)
    {
        // Set calibration parameters for each subpart
        // You can use _iremotecalib and _icontrolcalib to set the parameters
        yDebug() << m_deviceName << "Configuring calibration for subpart " << _subpartsList[i];

        yarp::dev::PolyDriver driver = _fineCalibrationCheckerDevices[subpartName];
        driver.view(_iremotecalib);
        driver.view(_icontrolcalib);

        runCalibration();
    }
}

void FineCalibrationCheckerThread::runCalibration()
{
    // Implement the calibration logic here
    // For example, you can call methods on _iremotecalib and _icontrolcalib to perform calibration tasks

    yDebug() << m_deviceName << "Starting calibration process"; 
    if(_iremotecalib->calibrateWholePart())
    {
        yDebug() << m_deviceName << "Calibration for subpart " << _subpartList[i] << " started!";
        
        int motors = 0;
        _imot->getNumberOfMotors(&motors);
        for (size_t i = 0; i < motors; i++)
        {
            /* code */
        }
        
        if (_icontrolcalib->calibrationDone())
        {
            yDebug() << m_deviceName << "Calibration for subpart " << _subpartList[i] << " completed successfully!";
        }
        else
        {
            yError() << m_deviceName << "Calibration for subpart " << _subpartList[i] << " failed!";
        }
        
        calibrationStatus = true; // Set the calibration status based on the result of the calibration process

        //TODO: we need to check if the raw position read overlaps with the expected position
        bool ok;
        ok = iravap->getRawDataMap(rawDataValuesMap);
        if (!ok)
        {
            yWarning() << "telemetryDeviceDumper warning : raw_data_values was not read correctly";
        }
        else
        {
            for (auto [key,value] : rawDataValuesMap)
            {
                bufferManager.push_back(value, "raw_data_values::"+key);
            }
        }
    }
    else
    {
        yError() << m_deviceName << "Calibration for subpart " << _subpartList[i] << " failed!";
        calibrationStatus = false;
    }
    
}

FineCalibrationCheckerThread::configureDevicesMap(std::vector<std::string> list)
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
        _fineCalibrationCheckerDevices.insert(std::pair{item, yarp::dev::PolyDriver()});
    }
}
