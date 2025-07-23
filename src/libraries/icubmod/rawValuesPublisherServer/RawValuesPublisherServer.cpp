/*
 * SPDX-FileCopyrightText: 2006-2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "RawValuesPublisherServer.h"

// yarp includes
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/LogComponent.h>

namespace {
    YARP_LOG_COMPONENT(RAWVALUESPUBLISHERSERVER, "iCub.debugLibrary.rawvaluespublisherserver")
}

#undef DEBUG_RAW_VALUES_MACRO

using namespace yarp::os;


RawValuesPublisherServer::RawValuesPublisherServer() :
    PeriodicThread(0.02)
{;}

RawValuesPublisherServer::~RawValuesPublisherServer() = default;

bool RawValuesPublisherServer::open(yarp::os::Searchable& config)
{
    if (!parseParams(config))
    {
        return false;
    }
    // set the period. m_period var is defined in the <device_name>_ParamsParser.h file
    // and updates at config file parsing as per m_name var.
    m_threadPeriodInS = m_period / 1000.0;

    if(m_threadPeriodInS <= 0)
    {
        yCError(RAWVALUESPUBLISHERSERVER)
            << "Period parameter is present with value:"
            << m_threadPeriodInS << "but it is not a positive integer. Closing...";
        return false;
    }

    // set streaming port
    m_streamingPortName = m_name + "/rawdata:o";
    // Open port
    if(!m_streamingRawDataPort.open(m_streamingPortName))
    {
        yCError(RAWVALUESPUBLISHERSERVER)
            << "Error opening streaming raw data port:" << m_streamingPortName;
        close();
        return false;
    }

    m_rpcPortName = m_name + "/rpc:o";

    // Attach to port
    if(!this->yarp().attachAsServer(m_rpcRawDataPort))
    {
        yCError(RAWVALUESPUBLISHERSERVER)
            << "Failure in attaching RPC port to thrift RPC interface";
        close();
        return false;
    }
    // Open port
    if(!m_rpcRawDataPort.open(m_rpcPortName))
    {
        yCError(RAWVALUESPUBLISHERSERVER)
            << "Failure in opening rpc port:" << m_rpcPortName;
        close();
        return false;
    }

    yCInfo(RAWVALUESPUBLISHERSERVER) << "Open ports completes";
    return true;
}

bool RawValuesPublisherServer::close()
{
    return(this->detachAll());
}

void RawValuesPublisherServer::threadRelease()
{
    return;
}

bool RawValuesPublisherServer::attachAll(const yarp::dev::PolyDriverList &p)
{
    if (p.size() != 1 )
    {
        yCError(RAWVALUESPUBLISHERSERVER)
            << "Trying to expose" << p.size() << "device(s). Expected only one device to be exposed to YARP ports. Closing...";
        close();
        return false;
    }

    yarp::dev::PolyDriver* poly = p[0]->poly;

    if(!poly)
    {
        yCError(RAWVALUESPUBLISHERSERVER)
            << "NullPointerException when getting the polyDriver at attachAll.";
        close();
        return false;
    }

    // View all the interfaces
    if(!poly->view(m_iRawValuesPublisher))
    {
        yCError(RAWVALUESPUBLISHERSERVER)
            << "Failure in viewing raw values publisher interface";
        close();
        return false;
    }

    // Set rate period
    if(!this->setPeriod(m_threadPeriodInS))
    {
        yCError(RAWVALUESPUBLISHERSERVER)
            << "Failure in setting periodic thread period";
        close();
        return false;
    }

    // Populate the RPC data to be served on the RPC port
    if(!populateMetadata(m_mapOfMetadata))
    {
        yCError(RAWVALUESPUBLISHERSERVER, "Failure in getMetadata()");
        close();
        return false;
    }

    // Start periodic thread
    if(!this->start())
    {
        yCError(RAWVALUESPUBLISHERSERVER)
            << "Failure in starting periodic thread";
        close();
        return false;
    }

    yCInfo(RAWVALUESPUBLISHERSERVER) << "Attach completes";

    return true;
}

bool RawValuesPublisherServer::detachAll()
{
    // Stop periodicThread if running
    if (this->isRunning())
    {
        this->stop();
    }

    m_rpcRawDataPort.close();
    m_streamingRawDataPort.close();

    yCInfo(RAWVALUESPUBLISHERSERVER) << "Detach complete";

    return true;
}

rawValuesKeyMetadataMap RawValuesPublisherServer::getMetadata()
{
    return m_mapOfMetadata;
}

void RawValuesPublisherServer::run()
{
    rawValuesDataVectorsMap &rdm = m_streamingRawDataPort.prepare();
    rdm.vectorsMap.clear();

    if(!m_iRawValuesPublisher->getRawDataMap(rdm.vectorsMap))
    {
        m_streamingRawDataPort.unprepare();
        return;
    }

    m_stamp.update();
    m_streamingRawDataPort.setEnvelope(m_stamp);
    m_streamingRawDataPort.write();
}

// Private nmethods
bool RawValuesPublisherServer::populateMetadata(rawValuesKeyMetadataMap &metamap)
{
    if(!m_iRawValuesPublisher->getMetadataMap(metamap))
    {
        return false;
    }

    #ifdef DEBUG_RAW_VALUES_MACRO
    for (auto [k,v] : metamap.metadataMap)
    {
        yCDebug(RAWVALUESPUBLISHERSERVER) << "Metadata at key:" << k << "with size:" << v.size;
        for (size_t i = 0; i < v.size; i++)
        {
            yCDebug(RAWVALUESPUBLISHERSERVER) << "Metadata element:" << v.rawValueNames[i];
        }

    }
    #endif


    return true;
}