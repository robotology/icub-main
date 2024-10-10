// -*- Mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2024 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Jacopo Losi, Valentina Gaggero
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

// api

#include "FakeRawValuesPublisher.h"

// yarp includes
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/LogComponent.h>

namespace 
{
    YARP_LOG_COMPONENT(FAKERAWVALUESPUBLISHER, "iCub.debugLibrary.fakeRawvaluespublisher")
}

using namespace iCub::debugLibrary;

// Static inline functions
static inline bool NOT_YET_IMPLEMENTED(const char* txt)
{
    yCError(FAKERAWVALUESPUBLISHER) << txt << "has not yet been emplemented";
    return true;
}

static inline bool DEPRECATED(const char* txt)
{
    yCError(FAKERAWVALUESPUBLISHER) << txt << "has been deprecated";
    return true;
}

#undef DEBUG_RAW_VALUES_MACRO 

FakeRawValuesPublisher::FakeRawValuesPublisher()
{
    m_numberOfJomos = 4;
    // set sawtooth amplitude to default value of 128. It will be overridden by the conf value
    m_sawtoothThreshold = 128;
    m_sawthootTestVal = 0;
    m_rawValuesVectorTag = "rawJomoEncoderValues";
    m_rawDataAuxVector = {};
    m_rawValuesMetadataMap = {};
}

bool FakeRawValuesPublisher::getRawDataMap(std::map<std::string, std::vector<std::int32_t>> &map)
{
    for (auto it = m_rawValuesMetadataMap.begin(); it != m_rawValuesMetadataMap.end(); it++)
    {   
        if(!getRawData_core(it->first, m_rawDataAuxVector)) 
        {
            yCError(FAKERAWVALUESPUBLISHER) << "getRawData_core() failed. Cannot retrieve all data from local memory";
            return false;
        }
        map.insert({it->first, m_rawDataAuxVector});
    }
    return true;
}

bool FakeRawValuesPublisher::getRawData_core(std::string key, std::vector<std::int32_t> &data)
{
    //Here I need to be sure 100% the key exists!!! 
    // It must exists since the call is made while iterating over the map
    data.clear();
    m_sawthootTestVal = (m_sawthootTestVal < m_sawtoothThreshold) ? (++m_sawthootTestVal) : 0;
    for (uint8_t i = 0; i < m_rawValuesMetadataMap[key].size; i++)
    {
        data.push_back(m_sawthootTestVal);
    }
    return true;
}

bool FakeRawValuesPublisher::getRawData(std::string key, std::vector<std::int32_t> &data)
{   
    if (m_rawValuesMetadataMap.find(key) != m_rawValuesMetadataMap.end())
    {
        getRawData_core(key, data);
    }
    else
    {
        yCError(FAKERAWVALUESPUBLISHER) << "Request key:" << key << "is not available. Cannot retrieve get raw data.";
        return false;
    }

    return true;
}

bool FakeRawValuesPublisher::getKeys(std::vector<std::string> &keys)
{
    keys.clear();
    for (const auto &p : m_rawValuesMetadataMap)
    {
        keys.push_back(p.first);
    }
    
    return true;
}

int FakeRawValuesPublisher::getNumberOfKeys()
{
    return m_rawValuesMetadataMap.size();
}


bool FakeRawValuesPublisher::getMetadataMap(rawValuesKeyMetadataMap &metamap)
{
    
    #ifdef DEBUG_RAW_VALUES_MACRO
    for (auto [k, v] : m_rawValuesMetadataMap)
    {
        yCDebug(FAKERAWVALUESPUBLISHER) << "size of elements name at key:" << k << "is:" << v.rawValueNames.size(); 
        for (size_t e = 0; e < v.rawValueNames.size(); e++)
        {
            yCDebug(FAKERAWVALUESPUBLISHER) << "GOT to rawValueNames:" << v.rawValueNames[e];
        }
        
    }
    #endif

    if (m_rawValuesMetadataMap.empty())
    {
        yCError(FAKERAWVALUESPUBLISHER) << "embObjMotionControl Map is empty. No reason to proceed...";
        return false;
    }
    
    metamap.metadataMap = m_rawValuesMetadataMap;
    return true;
}
bool FakeRawValuesPublisher::getKeyMetadata(std::string key, rawValuesKeyMetadata &meta)
{
    if(m_rawValuesMetadataMap.find(key) != m_rawValuesMetadataMap.end())
    {
        meta = m_rawValuesMetadataMap[key];
    }
    else
    {
        yCError(FAKERAWVALUESPUBLISHER) << "Requested key" << key << "is not available in the map. Exiting";
        return false;
    }
    return true;
}

bool FakeRawValuesPublisher::open(yarp::os::Searchable& config)
{
    if (!parseParams(config))
    {
        yCError(FAKERAWVALUESPUBLISHER) << "Failed to parse parameters from configuration";
        return false;
    }

    m_numberOfJomos = m_njomos;
    // Override sawtooth threshold value with the one passed by the configuration file
    m_sawtoothThreshold = m_threshold;

    // Instantiate map of raw values vectors
    m_rawValuesMetadataMap.insert({m_rawValuesVectorTag, rawValuesKeyMetadata({}, m_numberOfJomos)});
    for (int i = 0; i < m_numberOfJomos;  i++)
    {
        m_rawValuesMetadataMap[m_rawValuesVectorTag].rawValueNames.push_back("fake_jomo_"+std::to_string(i));
    }
    
    m_rawDataAuxVector.resize(m_numberOfJomos);
    return true;
}

bool FakeRawValuesPublisher::close()
{
    yCInfo(FAKERAWVALUESPUBLISHER) << "Closing the device."; 
    return true;
}