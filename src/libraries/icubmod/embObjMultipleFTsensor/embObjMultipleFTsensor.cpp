
/*
 * Copyright (C) 2020 iCub Tech - Istituto Italiano di Tecnologia
 * Author:  Marco Accame
 * email:   marco.accame@iit.it
 */

// general purpose stuff.
#include <string.h>

#include <iostream>
#include <string>

// Yarp Includes
#include <ace/Log_Msg.h>
#include <ace/config.h>
#include <stdarg.h>
#include <stdio.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>

// specific to this device driver.
#include <embObjMultipleFTsensor.h>
#include <ethManager.h>
#include <yarp/os/NetType.h>

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

embObjMultipleFTsensor::embObjMultipleFTsensor() : device_("embObjMultipleFTsensor")
{
}

embObjMultipleFTsensor::~embObjMultipleFTsensor()
{
	close();
}

bool embObjMultipleFTsensor::initialised()
{
	return device_.isOpen();
}

bool embObjMultipleFTsensor::open(yarp::os::Searchable& config)
{
	// 1) prepare eth service verifing if the eth manager is available and parsing info about the eth board.

	yInfo() << "embObjMultipleFTsensor::open(): preparing ETH resource";

	if (!device_.prerareEthService(config, this))
		return false;

	yInfo() << "embObjMultipleFTsensor::open(): browsing xml files which describe the service";

	// 2) read stuff from config file
	ServiceParserMultipleFt parser;
	if (!parser.parse(config))
	{
		yError() << "embObjMultipleFTsensor missing some configuration parameter. Check logs and your config file.";
		return false;
	}

	eOmn_serv_parameter_t ftData;
	ftData.configuration.data.as.ft = parser.toEomn();

	// 3) prepare data vector
	{
		//m_data.resize(eOas_pos_data_maxnumber, 0.0);
	}

	yInfo() << "embObjMultipleFTsensor::open(): verify the presence of the board and if its protocol version is correct";

	// 4) verify analog sensor protocol and then verify-Activate the POS service
	if (!device_.res->verifyEPprotocol(eoprot_endpoint_analogsensors))
	{
		cleanup();
		return false;
	}

	yInfo() << "embObjMultipleFTsensor::open(): verify and activate the POS service";

	// const eOmn_serv_parameter_t* servparam = &serviceConfig.ethservice;

	if (!device_.res->serviceVerifyActivate(eomn_serv_category_ft, &ftData, 5.0))
	{
		yError() << device_.getBoardInfo() << "open() has an error in call of ethResources::serviceVerifyActivate() ";
		cleanup();
		return false;
	}

	// printServiceConfig();

	yInfo() << "embObjMultipleFTsensor::open(): configure the POS service";

	if (false == sendConfig2boards(parser))
	{
		cleanup();
		return false;
	}

	yInfo() << "embObjMultipleFTsensor::open(): impose the network variable which the ETH bord must stream up";

	// Set variable to be signaled
	if (false == initRegulars(parser))
	{
		cleanup();
		return false;
	}

	yInfo() << "embObjMultipleFTsensor::open(): start the POS service";

	if (!device_.res->serviceStart(eomn_serv_category_ft))
	{
		yError() << device_.getBoardInfo() << "open() fails to start as service.... cannot continue";
		cleanup();
		return false;
	}
	else
	{
		if (device_.isVerbose())
		{
			yDebug() << device_.getBoardInfo() << "open() correctly starts service";
		}
	}

	yInfo() << "embObjMultipleFTsensor::open(): start streaming of POS data";

	sendStart2boards(parser);

	device_.setOpen(true);

	return true;
}

bool embObjMultipleFTsensor::sendConfig2boards(ServiceParserMultipleFt& parser)
{
	auto& ftInfos = parser.getFtInfo();
	int index = 0;
	for (const auto& [id, data] : ftInfos)
	{
		eOprotID32_t id32 = eo_prot_ID32dummy;
		eOas_ft_config_t cfg;
		cfg.ftperiod = data.ftAcquisitionRate;			 // TODO LUCA check
		cfg.temperatureperiod = data.ftAcquisitionRate;	 // TODO LUCA check
		id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_ft, index, eoprot_tag_as_ft_config);

		if (false == device_.res->setcheckRemoteValue(id32, &cfg, 10, 0.010, 0.050))
		{
			yError() << device_.getBoardInfo() << "FATAL error in sendConfig2boards() while try to configure datarate=" << cfg.ftperiod;
			return false;
		}

		if (device_.isVerbose())
		{
			yDebug() << device_.getBoardInfo() << ": sendConfig2boards() correctly configured boards with datarate=" << cfg.ftperiod;
		}
		++index;
	}
	return true;
}

bool embObjMultipleFTsensor::sendStart2boards(ServiceParserMultipleFt& parser)
{
	eOprotID32_t id32 = eo_prot_ID32dummy;

	uint8_t enable = 1;

	const auto& ftInfos = parser.getFtInfo();
	int index = 0;
	for (const auto& [id, data] : ftInfos)
	{
		id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_ft, index, eoprot_tag_as_ft_cmmnds_enable);

		if (false == device_.res->setcheckRemoteValue(id32, &enable, 10, 0.010, 0.050))
		{
			yError() << device_.getBoardInfo() << "FATAL error in sendStart2boards() while try to enable the boards transmission";
			return false;
		}

		if (device_.isVerbose())
		{
			yDebug() << device_.getBoardInfo() << ": sendStart2boards() correctly enabled the boards transmission";
		}
		++index;
	}
	return true;
}

bool embObjMultipleFTsensor::initRegulars(ServiceParserMultipleFt& parser)
{
	// configure regular rops

	vector<eOprotID32_t> id32v(0);
	eOprotID32_t id32 = eo_prot_ID32dummy;

	// we need to choose the id32 to put inside the vector

	const auto& ftInfos = parser.getFtInfo();
	int index = 0;
	for (const auto& [id, data] : ftInfos)
	{
		id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_ft, index, eoprot_tag_as_ft_status_timedvalue);
		id32v.push_back(id32);
		++index;
	}

	// now we send the vector

	if (false == device_.res->serviceSetRegulars(eomn_serv_category_pos, id32v))
	{
		yError() << device_.getBoardInfo() << "initRegulars() fails to add its variables to regulars: cannot proceed any further";
		return false;
	}

	if (device_.isVerbose())
	{
		yDebug() << device_.getBoardInfo() << "initRegulars() added" << id32v.size() << "regular rops ";
		char nvinfo[128];
		for (size_t r = 0; r < id32v.size(); r++)
		{
			uint32_t item = id32v.at(r);
			eoprot_ID2information(item, nvinfo, sizeof(nvinfo));
			yDebug() << "\t it added regular rop for" << nvinfo;
		}
	}

	return true;
}

eth::iethresType_t embObjMultipleFTsensor::type()
{
	return eth::iethres_analogpos;
}

bool embObjMultipleFTsensor::update(eOprotID32_t id32, double timestamp, void* rxdata)
{
	// called by feat_manage_analogsensors_data() which is called by:
	// eoprot_fun_UPDT_as_pos_status
	if (!device_.isOpen())
		return false;

	eOprotIndex_t eoprotIndex = eoprot_ID2index(id32);
	eOas_ft_timedvalue_t* data = (eOas_ft_timedvalue_t*)rxdata;
	
	std::unique_lock<std::shared_mutex> lck(mutex_);

	for (int index = 0; index < eoas_ft_6axis; ++index)
	{
		ftData_[eoprotIndex].data_[index] = data->values[index];
	}
	ftData_[eoprotIndex].timeStamp_ = data->age;

	temperature_[eoprotIndex].data_ = data->temperature;
	temperature_[eoprotIndex].timeStamp_ = data->age;
	return true;
}

bool embObjMultipleFTsensor::close()
{
	cleanup();
	return true;
}

void embObjMultipleFTsensor::cleanup(void)
{
	device_.cleanup(static_cast<eth::IethResource*>(this));
}

bool embObjMultipleFTsensor::getSixAxisForceTorqueSensorMeasure(size_t sensorIndex, yarp::sig::Vector& out, double& timestamp) const
{
	if (!device_.isOpen())
		return false;

	std::shared_lock<std::shared_mutex> lck(mutex_);

	if (ftData_.find(sensorIndex) == ftData_.end())
	{
		yError() << device_.getBoardInfo() << "getSixAxisForceTorqueSensorMeasure() fails data for index:" << sensorIndex << " not found";
		return false;
	}

	FtData sensorData = ftData_.at(sensorIndex);

	out.resize(ftChannels_);
	for (size_t k = 0; k < ftChannels_; k++)
	{
		out[k] = sensorData.data_[k] /*+ offset_[k]*/;//TODO LUCA manage offset 
	}
	timestamp = ftData_.at(sensorIndex).timeStamp_;
	return true;
}

size_t embObjMultipleFTsensor::getNrOfSixAxisForceTorqueSensors() const
{
	return ftData_.size();
}

yarp::dev::MAS_status embObjMultipleFTsensor::getSixAxisForceTorqueSensorStatus(size_t sens_index) const
{
	return yarp::dev::MAS_OK;
}

bool embObjMultipleFTsensor::getSixAxisForceTorqueSensorName(size_t sens_index, std::string& name) const
{
	name = "";	// TODO LUCA
	return true;
}

bool embObjMultipleFTsensor::getSixAxisForceTorqueSensorFrameName(size_t sens_index, std::string& frameName) const
{
	frameName = "";	 // TODO LUCA
	return true;
}

size_t embObjMultipleFTsensor::getNrOfTemperatureSensors() const
{
	return temperature_.size();
}

yarp::dev::MAS_status embObjMultipleFTsensor::getTemperatureSensorStatus(size_t sens_index) const
{
	return yarp::dev::MAS_OK;
}

bool embObjMultipleFTsensor::getTemperatureSensorName(size_t sens_index, std::string& name) const
{
	name = "";	// TODO LUCA
	return true;
}

bool embObjMultipleFTsensor::getTemperatureSensorFrameName(size_t sens_index, std::string& frameName) const
{
	frameName = "";	 // TODO LUCA
	return true;
}

bool embObjMultipleFTsensor::getTemperatureSensorMeasure(size_t sensorIndex, double& out, double& timestamp) const
{
	if (!device_.isOpen())
		return false;

	std::shared_lock<std::shared_mutex> lck(mutex_);

	if (temperature_.find(sensorIndex) == temperature_.end())
	{
		yError() << device_.getBoardInfo() << "getTemperatureSensorMeasure() fails data for index:" << sensorIndex << " not found";
		return false;
	}

	out = temperature_.at(sensorIndex).data_;
	timestamp = temperature_.at(sensorIndex).timeStamp_;
	return true;
}

bool embObjMultipleFTsensor::getTemperatureSensorMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
	// TODO LUCA
	return true;
}