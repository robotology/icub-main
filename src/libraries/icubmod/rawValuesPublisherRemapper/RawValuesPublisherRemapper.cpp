/*
 * SPDX-FileCopyrightText: 2006-2025 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "RawValuesPublisherRemapper.h"


#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>

namespace {
YARP_LOG_COMPONENT(RAWVALUESPUBLISHERREMAPPER, "yarp.device.rawvaluespublisherremapper")
}

// Private methods

bool RawValuesPublisherRemapper::open(yarp::os::Searchable& config)
{
    yarp::os::Property prop;
    prop.fromString(config.toString());

    m_verbose = (prop.check("verbose","if present, give detailed output"));
    if (m_verbose)
    {
        yCInfo(RAWVALUESPUBLISHERREMAPPER, "Running with verbose output\n");
    }

    if(!parseParams(prop))
    {
        yCError(RAWVALUESPUBLISHERREMAPPER) << "Error parsing configuration parameters";
        return false;
    }

    yCDebug(RAWVALUESPUBLISHERREMAPPER) << "RawValuesPublisherRemapper device started";
    for (const auto& name : m_axesNames)
    {
        //TODO: debug print to be removed once the remapper will be fully implemented
        yCDebug(RAWVALUESPUBLISHERREMAPPER) << "Axes Name: " << name;
    }
    return true;
}

bool RawValuesPublisherRemapper::close()
{
    return detachAll();
}

bool RawValuesPublisherRemapper::attachAll(const yarp::dev::PolyDriverList& drivers)
{
    if (drivers.size() < 1)
    {
        yCError(RAWVALUESPUBLISHERREMAPPER) << "attachAll: cannot attach to less than one device";
        return false;
    }
    yCDebug(RAWVALUESPUBLISHERREMAPPER) << "Attaching to " << drivers.size() << " devices";
    m_remappedControlBoards.resize(drivers.size());
    for (size_t i = 0; i < drivers.size(); i++)
    {
        yarp::dev::PolyDriver* poly = drivers[i]->poly;
        if (!poly)
        {
            yCError(RAWVALUESPUBLISHERREMAPPER) << "NullPointerException when getting the polyDriver at attachAll.";
            detachAll();
            return false;
        }

        yCDebug(RAWVALUESPUBLISHERREMAPPER) << "Attaching to device " << drivers[i]->key.c_str();

        // View all the interfaces
        if (!poly->view(m_remappedControlBoards[i]))
        {
            yCError(RAWVALUESPUBLISHERREMAPPER) << "Failure in viewing raw values publisher interface for device " << drivers[i]->key.c_str();
            detachAll();
            return false;
        }
        else
        {
            yCDebug(RAWVALUESPUBLISHERREMAPPER) << "Successfully viewed raw values publisher interface";
            std::vector<std::string> keys;
            if (m_remappedControlBoards[i]->getKeys(keys))
            {
                yCDebug(RAWVALUESPUBLISHERREMAPPER) << "Device " << drivers[i]->key.c_str() << " has " << keys.size() << " keys";
            }
            else
            {
                yCWarning(RAWVALUESPUBLISHERREMAPPER) << "Failed to get keys from device " << drivers[i]->key.c_str();
            }
            // Debug print keys
            yCDebug(RAWVALUESPUBLISHERREMAPPER) << "Keys from device " << drivers[i]->key.c_str() << ":";
            for (const auto& key : keys)
            {
                yCDebug(RAWVALUESPUBLISHERREMAPPER) << "Key: " << key;
            }
        }
    }

    return true;
}

bool RawValuesPublisherRemapper::detachAll()
{
    m_remappedControlBoards.resize(0);
    return true;
}

bool RawValuesPublisherRemapper::getRawDataMap(std::map<std::string, std::vector<std::int32_t>>& map)
{
    map.clear();
    bool allOk = true;
    for (size_t i = 0; i < m_remappedControlBoards.size(); i++)
    {
        if (!m_remappedControlBoards[i]) 
        {
            yCWarning(RAWVALUESPUBLISHERREMAPPER) << "Null pointer in m_remappedControlBoards at index " << i;
            allOk = false;
            continue;
        }
        std::map<std::string, std::vector<std::int32_t>> temp_map;
        if (!m_remappedControlBoards[i]->getRawDataMap(temp_map))
        {
            yCWarning(RAWVALUESPUBLISHERREMAPPER) << "Failed to get raw data map from control board " << i;
            allOk = false;
            continue;
        }
        // If key already present in the map, append the values
        // to avoid overwriting data from previous control boards
        // This is useful when multiple control boards publish
        // data under the same key
        for (const auto& [k, v] : temp_map)
        {
            if (map.find(k) != map.end())
            {   
                // Key already exists, append values
                if(m_verbose)
                {
                    yCDebug(RAWVALUESPUBLISHERREMAPPER) << "Key " << k << " already exists in the combined map. Appending values.";
                }
                map[k].insert(map[k].end(), v.begin(), v.end());
            }
            else
            {
                // Key does not exist, create new entry
                map[k] = v;
            }
        } 
    }
    return allOk;
}

bool RawValuesPublisherRemapper::getRawData(std::string key, std::vector<std::int32_t>& data)
{
    bool found = false;
    data.clear();
    for (size_t i = 0; i < m_remappedControlBoards.size(); i++)
    {
        if (!m_remappedControlBoards[i]) 
        {
            yCWarning(RAWVALUESPUBLISHERREMAPPER) << "Null pointer in m_remappedControlBoards at index " << i;
            continue;
        }
        std::vector<std::int32_t> temp_data;
        if (m_remappedControlBoards[i]->getRawData(key, temp_data))
        {
            data.insert(data.end(), temp_data.begin(), temp_data.end());
            found = true;
        }
    }
    if (!found) {
        yCWarning(RAWVALUESPUBLISHERREMAPPER) << "Key not found: " << key;
    }
    return found;
}

bool RawValuesPublisherRemapper::getKeys(std::vector<std::string>& keys)
{
    keys.clear();
    bool allOk = true;
    std::set<std::string> unique_keys;
    for (size_t i = 0; i < m_remappedControlBoards.size(); i++)
    {
        if (!m_remappedControlBoards[i]) {
            yCWarning(RAWVALUESPUBLISHERREMAPPER) << "Null pointer in m_remappedControlBoards at index " << i;
            allOk = false;
            continue;
        }
        std::vector<std::string> temp_keys;
        if (!m_remappedControlBoards[i]->getKeys(temp_keys))
        {
            yCWarning(RAWVALUESPUBLISHERREMAPPER) << "Failed to get keys from control board " << i;
            allOk = false;
            continue;
        }
        unique_keys.insert(temp_keys.begin(), temp_keys.end());
    }
    keys.assign(unique_keys.begin(), unique_keys.end());
    return allOk;
}

int RawValuesPublisherRemapper::getNumberOfKeys()
{
    std::vector<std::string> keys;
    if (getKeys(keys)) {
        return static_cast<int>(keys.size());
    }
    return -1;
}

bool RawValuesPublisherRemapper::getMetadataMap(iCub::rawValuesKeyMetadataMap& metamap)
{
    yCDebug(RAWVALUESPUBLISHERREMAPPER) << "Getting metadata map from all attached control boards:" << m_remappedControlBoards.size();
    bool allOk = true;
    for (size_t i = 0; i < m_remappedControlBoards.size(); i++)
    {
        iCub::rawValuesKeyMetadataMap temp_metamap = {};
        if (!m_remappedControlBoards[i]) 
        {
            yCWarning(RAWVALUESPUBLISHERREMAPPER) << "Null pointer in m_remappedControlBoards at index " << i;
            allOk = false;
            continue;
        }
        if (!m_remappedControlBoards[i]->getMetadataMap(temp_metamap))
        {
            yCWarning(RAWVALUESPUBLISHERREMAPPER) << "Failed to get metadata map from control board " << i;
            allOk = false;
            continue;
        }
        else
        {
            yCDebug(RAWVALUESPUBLISHERREMAPPER) << "Successfully got metadata map from control board " << i << "with elements:";
            if(m_verbose)
            {
                for (const auto& [k, v] : temp_metamap.metadataMap)
                {
                    yarp::os::Bottle axesNamesBottle, rawValueNamesBottle;
                    for(const auto& axisName : v.axesNames) { axesNamesBottle.addString(axisName); }
                    for(const auto& rawValueName : v.rawValueNames) { rawValueNamesBottle.addString(rawValueName); }
                    yCDebug(RAWVALUESPUBLISHERREMAPPER) << "\tKey: " << k << " AxesNames: " << axesNamesBottle.toString() << " RawValueNames: " << rawValueNamesBottle.toString();
                }
            }
        }
        for (const auto& [k, v] : temp_metamap.metadataMap)
        {
            if (metamap.metadataMap.find(k) != metamap.metadataMap.end())
            {
                if(m_verbose)
                {
                    yCDebug(RAWVALUESPUBLISHERREMAPPER) << "Key " << k << " already exists in the combined metadata map. Adding only values.";
                }
                    // Append axesNames and rawValueNames to the existing entry
                auto& existingMeta = metamap.metadataMap[k];
                existingMeta.axesNames.insert(existingMeta.axesNames.end(), v.axesNames.begin(), v.axesNames.end());
                existingMeta.rawValueNames.insert(existingMeta.rawValueNames.end(), v.rawValueNames.begin(), v.rawValueNames.end());
                // Update size
                existingMeta.size = static_cast<int>(existingMeta.rawValueNames.size());
            }
            else
            {
                yCDebug(RAWVALUESPUBLISHERREMAPPER) << "Inserting new key " << k << " in the combined metadata map.";
                // Copying entire metadata entry
                metamap.metadataMap[k] = v;
            }
        }
        
    }
    return allOk;
}

bool RawValuesPublisherRemapper::getKeyMetadata(std::string key, iCub::rawValuesKeyMetadata& meta)
{
    bool found = false;
    meta = {};
    for (size_t i = 0; i < m_remappedControlBoards.size(); i++)
    {
        if (!m_remappedControlBoards[i]) 
        {
            yCWarning(RAWVALUESPUBLISHERREMAPPER) << "Null pointer in m_remappedControlBoards at index " << i;
            continue;
        }
        iCub::rawValuesKeyMetadata temp_meta;
        if (m_remappedControlBoards[i]->getKeyMetadata(key, temp_meta))
        {
            if (!found) 
            {
                meta = temp_meta;
            } 
            else 
            {
                meta.axesNames.insert(meta.axesNames.end(), temp_meta.axesNames.begin(), temp_meta.axesNames.end());
                meta.rawValueNames.insert(meta.rawValueNames.end(), temp_meta.rawValueNames.begin(), temp_meta.rawValueNames.end());
                meta.size = static_cast<int>(meta.rawValueNames.size());
            }
            found = true;
        }
    }
    if (!found) {
        yCWarning(RAWVALUESPUBLISHERREMAPPER) << "Metadata not found for key: " << key;
    }
    return found;
}

bool RawValuesPublisherRemapper::getAxesNames(std::string key, std::vector<std::string>& axesNames)
{
    bool found = false;
    axesNames.clear();
    for (size_t i = 0; i < m_remappedControlBoards.size(); i++)
    {
        if (!m_remappedControlBoards[i]) {
            yCWarning(RAWVALUESPUBLISHERREMAPPER) << "Null pointer in m_remappedControlBoards at index " << i;
            continue;
        }
        std::vector<std::string> temp_axesNames;
        if (m_remappedControlBoards[i]->getAxesNames(key, temp_axesNames))
        {
            axesNames.insert(axesNames.end(), temp_axesNames.begin(), temp_axesNames.end());
            found = true;
        }
    }
    if (!found) 
    {
        yCWarning(RAWVALUESPUBLISHERREMAPPER) << "Axes names not found for key: " << key;
    }
    return found;
}
