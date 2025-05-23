/*
 * SPDX-FileCopyrightText: 2006-2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "RawValuesPublisherClient.h"

// yarp includes
#include <yarp/os/LogComponent.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Network.h>

namespace {
    YARP_LOG_COMPONENT(RAWVALUESPUBLISHERCLIENT, "iCub.debugLibrary.rawvaluespublisherclient")
}

#undef DEBUG_RAW_VALUES_MACRO 

using namespace yarp::os;

// Static inline functions
static inline bool NOT_YET_IMPLEMENTED(const char* txt)
{
    yCError(RAWVALUESPUBLISHERCLIENT) << txt << "has not yet been implemented";
    return true;
}

static inline bool DEPRECATED(const char* txt)
{
    yCError(RAWVALUESPUBLISHERCLIENT) << txt << "has been deprecated";
    return true;
}

void RawValuesStreamingDataInputPort::onRead(rawValuesDataVectorsMap& rawdata)
{
    // saving read data from server output port in the local map
    std::lock_guard<std::mutex> guard(dataMutex);
    receivedRawDataMap = rawdata.vectorsMap;
}

bool RawValuesPublisherClient::open(yarp::os::Searchable& config)
{
    if (!parseParams(config))
    {
        return false;
    }
    
    std::string localStreamingPortName = m_local + "/rawdata:i";
    std::string remoteStreamingPortName = m_remote + "/rawdata:o";
    std::string localRPCPortName = m_local + "/rpc:i";
    std::string remoteRPCPortName = m_remote + "/rpc:o";
    
    // Open ports
    // Open RPC port
    if(!m_rpcPort.open(localRPCPortName))
    {
        yCError(RAWVALUESPUBLISHERCLIENT) << "Failure opening RPC port" << localStreamingPortName;
        close();
        return false;
    }

    // Open streaming port
    if(!m_streamingPort.open(localStreamingPortName))
    {
        yCError(RAWVALUESPUBLISHERCLIENT) << "Failure opening streaming port" << localStreamingPortName;
        close();
        return false;
    }
    m_streamingPort.useCallback();
    
    // Connect ports
    if (!m_externalConnection) 
    {
        // Connect RPC port
        if (!yarp::os::Network::connect(localRPCPortName, remoteRPCPortName, m_carrier))
        {
            yCError(RAWVALUESPUBLISHERCLIENT) <<
                "Failure in connecting remote port" << remoteRPCPortName <<
                "to local port" << localRPCPortName;

            yCError(RAWVALUESPUBLISHERCLIENT) <<
                "Check that the specified RawValuesPublisherServer is up. Closing.";
            
            close();
            return false;
        }
        
        // Connect Streaming port
        if(!yarp::os::Network::connect(remoteStreamingPortName, localStreamingPortName, m_carrier))
        {
            yCError(RAWVALUESPUBLISHERCLIENT) <<
                "Failure in connecting remote port" << remoteStreamingPortName <<
                "to local port" << localStreamingPortName;

            yCError(RAWVALUESPUBLISHERCLIENT) <<
                "Check that the specified RawValuesPublisherServer is up. Closing.";
            
            close();
            return false;
        }

        if (!m_RPCInterface.yarp().attachAsClient(m_rpcPort)) {
            yCError(RAWVALUESPUBLISHERCLIENT, "Failure opening Thrift-based RPC interface.");
            return false;
        }
    }
    m_streamingPort.receivedRawDataMap = {};

    yCInfo(RAWVALUESPUBLISHERCLIENT) << "Open completes";
    return true;
}

bool RawValuesPublisherClient::close()
{
    m_streamingPort.close();
    m_rpcPort.close();

    yCInfo(RAWVALUESPUBLISHERCLIENT) << "Close completes";

    return true;
}

bool RawValuesPublisherClient::getRawDataMap(std::map<std::string, std::vector<std::int32_t>> &map)
{
    std::lock_guard<std::mutex> guard(m_streamingPort.dataMutex);
    map = m_streamingPort.receivedRawDataMap;
    
    return true;
}

bool RawValuesPublisherClient::getRawData(std::string key, std::vector<std::int32_t> &data)
{
    std::lock_guard<std::mutex> guard(m_streamingPort.dataMutex);
    if(m_streamingPort.receivedRawDataMap.find(key) != m_streamingPort.receivedRawDataMap.end())
    {
        data = m_streamingPort.receivedRawDataMap[key];
    }
    else
    {
        yCError(RAWVALUESPUBLISHERCLIENT) << "Requested key:" << key << "does not exist in the raw data map.";
        return false;
    }

    return true;
}

bool RawValuesPublisherClient::getKeys(std::vector<std::string> &keys)
{
    return NOT_YET_IMPLEMENTED("getKeys");
}

int RawValuesPublisherClient::getNumberOfKeys()
{
    return NOT_YET_IMPLEMENTED("getNumberOfKeys()");
}

bool RawValuesPublisherClient::getMetadataMap(rawValuesKeyMetadataMap &metamap)
{
    metamap = m_RPCInterface.getMetadata();
    
    return true;
}
bool RawValuesPublisherClient::getKeyMetadata(std::string key, rawValuesKeyMetadata &meta)
{
    std::map<std::string, rawValuesKeyMetadata> metamap = (m_RPCInterface.getMetadata()).metadataMap;
    if(metamap.find(key) != metamap.end())
    {
        meta = metamap[key];
    }
    else
    {
        yCError(RAWVALUESPUBLISHERCLIENT) << "Requested key" << key << "is not available in the map. Exiting";
        return false;
    }

    return true;
}

bool RawValuesPublisherClient::getAxesNames(std::string key, std::vector<std::string> &axesNames)
{
    axesNames.clear();
    rawValuesKeyMetadata metadata;
    this->getKeyMetadata(key, metadata);

    if (metadata.axesNames.empty())
    {
        yCError(RAWVALUESPUBLISHERCLIENT) << "No axes names found for key" << key;
        return false;
    }

    axesNames.assign(metadata.axesNames.begin(), metadata.axesNames.end());
    return true;
}