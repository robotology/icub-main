/*
 * SPDX-FileCopyrightText: 2006-2025 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef RAW_VALUES_PUBLISHER_REMAPPER_H
#define RAW_VALUES_PUBLISHER_REMAPPER_H

#include <iCub/IRawValuesPublisher.h>
#include <yarp/dev/IMultipleWrapper.h>
#include "RawValuesPublisherRemapper_ParamsParser.h"


class RawValuesPublisherRemapper :
    public yarp::dev::DeviceDriver,
    public yarp::dev::IMultipleWrapper,
    public iCub::debugLibrary::IRawValuesPublisher,
    public RawValuesPublisherRemapper_ParamsParser
{

    private:
    bool m_verbose{false};
    std::vector<iCub::debugLibrary::IRawValuesPublisher*> m_remappedControlBoards {nullptr};
    
public:

    RawValuesPublisherRemapper() = default;
    RawValuesPublisherRemapper(const RawValuesPublisherRemapper&) = default;
    RawValuesPublisherRemapper& operator=(const RawValuesPublisherRemapper&) = default;
    RawValuesPublisherRemapper(RawValuesPublisherRemapper&&) = default;
    RawValuesPublisherRemapper& operator=(RawValuesPublisherRemapper&&) = default;
    ~RawValuesPublisherRemapper() override = default;

    /* DeviceDriver methods */
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    /** MultipeWrapper methods */
    bool attachAll(const yarp::dev::PolyDriverList &p) override;
    bool detachAll() override;

    /* IRawValuesPublisher methods */
    bool getRawDataMap(std::map<std::string, std::vector<std::int32_t>> &map) override;
    bool getRawData(std::string key, std::vector<std::int32_t> &data) override;
    bool getKeys(std::vector<std::string> &keys) override;
    int  getNumberOfKeys() override;
    bool getMetadataMap(iCub::rawValuesKeyMetadataMap &metamap) override;
    bool getKeyMetadata(std::string key, iCub::rawValuesKeyMetadata &meta) override;
    bool getAxesNames(std::string key, std::vector<std::string> &axesNames) override;
};

#endif // RAW_VALUES_PUBLISHER_REMAPPER_H