/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef RAWVALUESPUBLISHER_NETWORKCLIENT_H
#define RAWVALUESPUBLISHER_NETWORKCLIENT_H

#include <yarp/os/BufferedPort.h>
#include <yarp/dev/DeviceDriver.h>
#include <iCub/IRawValuesPublisher.h>

// ParamParser generated classes
#include "RawValuesPublisherClient_ParamsParser.h"

using namespace iCub;
class RawValuesStreamingDataInputPort :
        public yarp::os::BufferedPort<rawValuesDataVectorsMap>
{
public:
    std::map<std::string, std::vector<std::int32_t>> receivedRawDataMap;
    mutable std::mutex dataMutex;

    using yarp::os::BufferedPort<rawValuesDataVectorsMap>::onRead;
    void onRead(rawValuesDataVectorsMap &rawdata) override;
};

class RawValuesPublisherClient :
    public yarp::dev::DeviceDriver,
    public iCub::debugLibrary::IRawValuesPublisher,
    public RawValuesPublisherClient_ParamsParser
{
    private:
        RawValuesStreamingDataInputPort m_streamingPort;
        yarp::os::Port m_rpcPort;

        RawValuesPublisherMetadata m_RPCInterface;

    public:
        /* DevideDriver methods */
        bool open(yarp::os::Searchable& config) override;
        bool close() override;

        /* IRawValuesPublisher methods */
        virtual bool getRawDataMap(std::map<std::string, std::vector<std::int32_t>> &map) override;
        virtual bool getRawData(std::string key, std::vector<std::int32_t> &data) override;
        virtual bool getKeys(std::vector<std::string> &keys) override;
        virtual int  getNumberOfKeys() override;
        virtual bool getMetadataMap(rawValuesKeyMetadataMap &metamap) override;
        virtual bool getKeyMetadata(std::string key, rawValuesKeyMetadata &meta) override;
        virtual bool getAxesNames(std::string key, std::vector<std::string> &axesNames) override;
};

#endif
