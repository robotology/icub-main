/*
 * SPDX-FileCopyrightText: 2006-2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef RAWVALUESPUBLISHER_NETWORKSERVER_H
#define RAWVALUESPUBLISHER_NETWORKSERVER_H

// std includes
#include <map>

// yarp includes
#include <yarp/os/Network.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IMultipleWrapper.h>
#include <yarp/sig/Vector.h>

// Interface for the getter of the raw data 
// It calls the rawValuesDataStruct generated w/ thrift
#include <iCub/IRawValuesPublisher.h>

// ParamParser generated classes
#include "RawValuesPublisherServer_ParamsParser.h"

/*TIPS:
 *  1. follow Multipleanalog server as example https://github.com/robotology/yarp/blob/bde9cd2b1069d207a77a845935b2c3063c74c58d/src/devices/networkWrappers/multipleanalogsensorsserver/MultipleAnalogSensorsServer.h#L33
 *  2. as fist MVP implement only Buffered Port and NOT RPC
 *  3. avoid the config parameters parser
 *  4. use the thrift file to create the RawValue data struct. As first step it'll contain only one uint32_t value.
 *  See https://github.com/robotology/icub-main/tree/ab93283ba6d654396b92e081d960ace777a9e656/src/tools/depth2kin es example
 *
 * */
using namespace iCub;

class RawValuesPublisherServer :
    public yarp::os::PeriodicThread,
    public yarp::dev::DeviceDriver,
    public yarp::dev::IMultipleWrapper,
    public RawValuesPublisherMetadata,
    public RawValuesPublisherServer_ParamsParser
{

private:
    yarp::os::Stamp m_stamp;
    double m_threadPeriodInS = 0.1;
    yarp::os::BufferedPort<rawValuesDataVectorsMap> m_streamingRawDataPort; // streams data continuously at each server run
    yarp::os::Port m_rpcRawDataPort; // public data on client request
    std::string m_streamingPortName = "";
    std::string m_rpcPortName = "";
    
    std::vector<std::string> m_vectorOfKeys;
    rawValuesKeyMetadataMap m_mapOfMetadata;
    
    // Interface of the wrapped device
    iCub::debugLibrary::IRawValuesPublisher* m_iRawValuesPublisher{nullptr};

    bool populateMetadata(rawValuesKeyMetadataMap &metamap);

public:
    RawValuesPublisherServer();
    ~RawValuesPublisherServer();

    /* DevideDriver methods */
    bool open(yarp::os::Searchable &params) override;
    bool close() override;

    /* PeriodThread methods */
    void threadRelease() override;
    void run() override;

    /* IMultipleWrapper methods */
    bool attachAll(const yarp::dev::PolyDriverList &p) override;
    bool detachAll() override;

    /* RawValuesPublisherMetadata */
    rawValuesKeyMetadataMap getMetadata() override;
};

#endif
