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

// APIs
#include "FineCalibrationCheckerThread.h"

bool FineCalibrationCheckerThread::threadInit()
{
    // Read configuration file
    Bottle &conf_group = rf.findGroup("GENERAL");
    if (conf_group.isNull())
    {
        yWarning() << "Missing GENERAL group! The module uses the default values";
    }
    else
    {
        // Read parameters from the configuration file
        if (conf_group.check("devicename")) { m_deviceName = conf_group.find("devicename").asString(); }
        if (conf_group.check("portprefix")) { m_portPrefix = conf_group.find("portprefix").asString(); }
        if (conf_group.check("period")) { m_updatePeriod = conf_group.find("period").asFloat64(); }
        if (conf_group.check("robotname")) { m_robotName = conf_group.find("robotname").asString(); }
    }

    // Initialize remote raw values publisher device
    Property deviceProperties;
    deviceProperties.put("device", m_deviceName);
    deviceProperties.put("remote", "/" + m_robotName + m_portPrefix);
    deviceProperties.put("local", "/" + m_robotName + m_portPrefix);

    yDebug() << m_deviceName << "++++ config:\n"
        << "\t portprefix: " << m_portPrefix << "\n"
        << "\t period: " << m_updatePeriod << "\n"
        << "\t robotname: " << m_robotName << "\n";

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


    return true;

}

/**
TODO: Keep 
    this in mind which can be a good insight if we wanna define a vector of Device Drivers 
    instead of alwats instantiate different device driver separately
#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>
#include <algorithm>
#include <set>

int main() {
    std::vector<std::string> input = {"left_arm", "right_arm", "tail", "left_leg", "torso"}; // "head" is missing, "tail" is unknown
    std::vector<std::string> desired_order = {"head", "left_arm", "right_arm", "torso", "left_leg", "right_leg"};

    // Step 1: Create the order map
    std::unordered_map<std::string, int> order_map;
    for (size_t i = 0; i < desired_order.size(); ++i) {
        order_map[desired_order[i]] = i;
    }

    // Step 2: Separate known and unknown elements
    std::vector<std::string> known_items;
    std::vector<std::string> unknown_items;

    for (const auto& item : input) {
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
    std::set<std::string> input_set(input.begin(), input.end());
    std::vector<std::string> missing_items;
    for (const auto& expected : desired_order) {
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

    return 0;
}

 */