/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 * Author: Luca Tricerri
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include <embObjMultipleFTsensors.h>
#include <ethManager.h>

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

#include <iostream>
#include <string>

#include "EOnv_hid.h"
#include "EoAnalogSensors.h"
#include "EoProtocol.h"
#include "EoProtocolAS.h"
#include "EoProtocolMN.h"

#ifdef WIN32
#pragma warning(once : 4355)
#endif

using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;

embObjMultipleFTsensors::embObjMultipleFTsensors()
{
    yInfo() << "MultipleFTSensor has been created";
    device_ = std::make_shared<yarp::dev::embObjDevPrivData>("embObjMultipleFTsensors");
}

embObjMultipleFTsensors::embObjMultipleFTsensors(std::shared_ptr<yarp::dev::embObjDevPrivData> device) : device_(device)
{
}

embObjMultipleFTsensors::~embObjMultipleFTsensors()
{
    close();
}

bool embObjMultipleFTsensors::initialised()
{
    return device_->isOpen();
}

bool embObjMultipleFTsensors::open(yarp::os::Searchable& config)
{
    yInfo() << "embObjMultipleFTsensors::open(): preparing ETH resource";
    if (!device_->prerareEthService(config, this))
        return false;

    yInfo() << device_->getBoardInfo() << " embObjMultipleFTsensors::open(): browsing xml files which describe the service";
    ServiceParserMultipleFt parser;
    if (!parser.parse(config))
    {
        yError() << device_->getBoardInfo() << "open() fails to parse xml... cannot continue ";
        return false;
    }

    yInfo() << device_->getBoardInfo() << " embObjMultipleFTsensors::open(): verify the presence of the board and if its protocol version is correct";
    if (!device_->res->verifyEPprotocol(eoprot_endpoint_analogsensors))
    {
        yError() << device_->getBoardInfo() << " open() fails to verifyEPprotocol... cannot continue ";
        cleanup();
        return false;
    }

    yInfo() << device_->getBoardInfo() << " embObjMultipleFTsensors::open(): verify and activate the FT service";
    eOmn_serv_parameter_t ftData;
    ftData.configuration.type = eomn_serv_AS_ft;
    ftData.configuration.diagnosticsmode = eomn_serv_diagn_mode_NONE;
    ftData.configuration.diagnosticsparam = 0;
    parser.toEomn(ftData.configuration.data.as.ft);
    if (!device_->res->serviceVerifyActivate(eomn_serv_category_ft, &ftData, 5.0))
    {
        yError() << device_->getBoardInfo() << " open() fails to serviceVerifyActivate... cannot continue ";
        cleanup();
        return false;
    }

    yInfo() << device_->getBoardInfo() << " embObjMultipleFTsensors::open(): configure the FT service";
    if (false == sendConfig2boards(parser, device_->res))
    {
        yError() << device_->getBoardInfo() << " open() fails to sendConfig2boards... cannot continue";
        cleanup();
        return false;
    }

    yInfo() << device_->getBoardInfo() << " embObjMultipleFTsensors::open(): impose the network variable which the ETH bord must stream up";
    if (false == initRegulars(parser, device_->res))
    {
        yError() << device_->getBoardInfo() << " open() fails to initRegulars... cannot continue";
        cleanup();
        return false;
    }

    yInfo() << device_->getBoardInfo() << " embObjMultipleFTsensors::open(): start the FT service";
    if (!device_->res->serviceStart(eomn_serv_category_ft))
    {
        yError() << device_->getBoardInfo() << " open() fails to serviceStart... cannot continue";
        cleanup();
        return false;
    }
    else
    {
        if (device_->isVerbose())
        {
            yDebug() << device_->getBoardInfo() << " open() correctly starts service";
        }
    }

    yInfo() << device_->getBoardInfo() << " embObjMultipleFTsensors::open(): start streaming of FT data";
    if (!sendStart2boards(parser, device_->res))
    {
        yError() << device_->getBoardInfo() << " open() fails to sendStart2boards... cannot continue";
        cleanup();
        return false;
    }

    device_->setOpen(true);
    return true;
}

bool embObjMultipleFTsensors::sendConfig2boards(ServiceParserMultipleFt& parser, eth::AbstractEthResource* deviceRes)
{
    auto& ftInfos = parser.getFtInfo();
    int index = 0;
    for (const auto& [id, data] : ftInfos)
    {
        eOprotID32_t id32 = eo_prot_ID32dummy;
        eOas_ft_config_t cfg;
        cfg.ftperiod = data.ftAcquisitionRate;
        cfg.temperatureperiod = data.temperatureAcquisitionRate;
        cfg.mode = data.useCalibration;
        cfg.calibrationset = 0;
        id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_ft, index, eoprot_tag_as_ft_config);

        if (false == deviceRes->setcheckRemoteValue(id32, &cfg, 10, 0.010, 0.050))
        {
            yError() << device_->getBoardInfo() << " sendConfig2boards() while try to configure ftPeriod=" << cfg.ftperiod;
            return false;
        }

        if (device_->isVerbose())
        {
            yDebug() << device_->getBoardInfo() << " sendConfig2boards() correctly configured boards with ftPeriod=" << cfg.ftperiod;
        }
        ++index;

        eOprotIndex_t eoprotIndex = eoprot_ID2index(id32);
        ftSensorsData_[eoprotIndex] = {{0, 0, 0, 0, 0, 0}, 0, id};
    }
    return true;
}

bool embObjMultipleFTsensors::sendStart2boards(ServiceParserMultipleFt& parser, eth::AbstractEthResource* deviceRes)
{
    eOprotID32_t id32 = eo_prot_ID32dummy;

    uint8_t enable = 1;

    const auto& ftInfos = parser.getFtInfo();
    int index = 0;
    for (const auto& [id, data] : ftInfos)
    {
        id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_ft, index, eoprot_tag_as_ft_cmmnds_enable);

        if (false == deviceRes->setcheckRemoteValue(id32, &enable, 10, 0.010, 0.050))
        {
            yError() << device_->getBoardInfo() << " sendStart2boards() while try to enable the boards transmission";
            return false;
        }

        if (device_->isVerbose())
        {
            yDebug() << device_->getBoardInfo() << " sendStart2boards() correctly enabled the boards transmission";
        }
        ++index;
    }
    return true;
}

bool embObjMultipleFTsensors::initRegulars(ServiceParserMultipleFt& parser, eth::AbstractEthResource* deviceRes)
{
    // configure regular rops

    vector<eOprotID32_t> id32v;
    eOprotID32_t id32 = eo_prot_ID32dummy;

    const auto& ftInfos = parser.getFtInfo();
    int index = 0;
    for (const auto& [id, data] : ftInfos)
    {
        id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_ft, index, eoprot_tag_as_ft_status_timedvalue);
        id32v.push_back(id32);
        ++index;
    }

    if (false == deviceRes->serviceSetRegulars(eomn_serv_category_ft, id32v))
    {
        yError() << device_->getBoardInfo() << " initRegulars() fails to add its variables to regulars: cannot proceed any further";
        return false;
    }

    if (device_->isVerbose())
    {
        yDebug() << device_->getBoardInfo() << " initRegulars() added" << id32v.size() << "regular rops ";
        char nvinfo[128];
        for (size_t r = 0; r < id32v.size(); r++)
        {
            uint32_t item = id32v.at(r);
            eoprot_ID2information(item, nvinfo, sizeof(nvinfo));
            yDebug() << device_->getBoardInfo() << "\t it added regular rop for" << nvinfo;
        }
    }

    return true;
}

eth::iethresType_t embObjMultipleFTsensors::type()
{
    return eth::iethres_analogft;
}

bool embObjMultipleFTsensors::update(eOprotID32_t id32, double timestamp, void* rxdata)
{
    if (!device_->isOpen())
        return false;

    eOprotIndex_t eoprotIndex = eoprot_ID2index(id32);
    if (eoprotIndex > 3)
    {
        yError() << device_->getBoardInfo() << " update() index too big";
        return false;
    }

    eOprotEntity_t entity = eoprot_ID2entity(id32);
    if (entity != eoprot_entity_as_ft)
    {
        yError() << device_->getBoardInfo() << " update() wrong entity";
        return false;
    }

    eOprotTag_t tag = eoprot_ID2tag(id32);
    if (tag != eoprot_tag_as_ft_status_timedvalue)
    {
        yError() << device_->getBoardInfo() << " update() wrong tag";
        return false;
    }

    eOas_ft_timedvalue_t* data = (eOas_ft_timedvalue_t*)rxdata;

    std::unique_lock<std::shared_mutex> lck(mutex_);

    for (int index = 0; index < eoas_ft_6axis; ++index)
    {
        ftSensorsData_[eoprotIndex].data_[index] = data->values[index];
    }
    ftSensorsData_[eoprotIndex].timeStamp_ = data->age;

    temperaturesensordata_[eoprotIndex].data_ = data->temperature;
    temperaturesensordata_[eoprotIndex].timeStamp_ = data->age;
    return true;
}

bool embObjMultipleFTsensors::close()
{
    yDebug() << device_->getBoardInfo() << " close board";
    cleanup();
    return true;
}

void embObjMultipleFTsensors::cleanup(void)
{
    device_->cleanup(static_cast<eth::IethResource*>(this));
}

bool embObjMultipleFTsensors::getSixAxisForceTorqueSensorMeasure(size_t sensorIndex, yarp::sig::Vector& out, double& timestamp) const
{
    if (!device_->isOpen())
        return false;

    std::shared_lock<std::shared_mutex> lck(mutex_);

    if (ftSensorsData_.find(sensorIndex) == ftSensorsData_.end())
    {
        yError() << device_->getBoardInfo() << " getSixAxisForceTorqueSensorMeasure() fails data for index:" << sensorIndex << " not found";
        return false;
    }

    FtData sensorData = ftSensorsData_.at(sensorIndex);

    out.resize(ftChannels_);
    for (size_t k = 0; k < ftChannels_; k++)
    {
        out[k] = sensorData.data_[k];
    }
    timestamp = ftSensorsData_.at(sensorIndex).timeStamp_;
    return true;
}

size_t embObjMultipleFTsensors::getNrOfSixAxisForceTorqueSensors() const
{
    return ftSensorsData_.size();
}

yarp::dev::MAS_status embObjMultipleFTsensors::getSixAxisForceTorqueSensorStatus(size_t sensorindex) const
{
    return yarp::dev::MAS_OK;
}

bool embObjMultipleFTsensors::getSixAxisForceTorqueSensorName(size_t sensorindex, std::string& name) const
{
    name = ftSensorsData_.at(sensorindex).sensorName_;
    return true;
}

bool embObjMultipleFTsensors::getSixAxisForceTorqueSensorFrameName(size_t sensorindex, std::string& frameName) const
{
    frameName = "";  // Unused
    return true;
}

size_t embObjMultipleFTsensors::getNrOfTemperatureSensors() const
{
    return temperaturesensordata_.size();
}

yarp::dev::MAS_status embObjMultipleFTsensors::getTemperatureSensorStatus(size_t sensorindex) const
{
    return yarp::dev::MAS_OK;
}

bool embObjMultipleFTsensors::getTemperatureSensorName(size_t sensorindex, std::string& name) const
{
    name = ftSensorsData_.at(sensorindex).sensorName_;
    return true;
}

bool embObjMultipleFTsensors::getTemperatureSensorFrameName(size_t sensorindex, std::string& frameName) const
{
    frameName = "";  // Unused
    return true;
}

bool embObjMultipleFTsensors::getTemperatureSensorMeasure(size_t sensorIndex, double& out, double& timestamp) const
{
    if (!device_->isOpen())
        return false;

    std::shared_lock<std::shared_mutex> lck(mutex_);

    if (temperaturesensordata_.find(sensorIndex) == temperaturesensordata_.end())
    {
        yError() << device_->getBoardInfo() << " getTemperatureSensorMeasure() fails data for index:" << sensorIndex << " not found";
        return false;
    }

    out = temperaturesensordata_.at(sensorIndex).data_;
    timestamp = temperaturesensordata_.at(sensorIndex).timeStamp_;
    return true;
}

bool embObjMultipleFTsensors::getTemperatureSensorMeasure(size_t sensorIndex, yarp::sig::Vector& out, double& timestamp) const
{
    double value{0};
    getTemperatureSensorMeasure(sensorIndex, value, timestamp);
    out.resize(1);
    out[0] = value;
    return true;
}
