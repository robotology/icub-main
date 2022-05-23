/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 * Author: Luca Tricerri
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include <embObjCanBatterysensor.h>
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

embObjCanBatterysensor::embObjCanBatterysensor()
{
	yInfo() << "MultipleFTSensors has been created";
	device_ = std::make_shared<yarp::dev::embObjDevPrivData>("embObjCanBatterysensor");
}

embObjCanBatterysensor::embObjCanBatterysensor(std::shared_ptr<yarp::dev::embObjDevPrivData> device) : device_(device)
{
}

embObjCanBatterysensor::~embObjCanBatterysensor()
{
	close();
}

bool embObjCanBatterysensor::initialised()
{
	return device_->isOpen();
}

bool embObjCanBatterysensor::open(yarp::os::Searchable &config)
{
	yInfo() << "embObjCanBatterysensor::open(): preparing ETH resource";
	if (!device_->prerareEthService(config, this))
		return false;

	yInfo() << device_->getBoardInfo() << " embObjCanBatterysensor::open(): browsing xml files which describe the service";
	ServiceParserCanBattery parser;
	if (!parser.parse(config))
	{
		yError() << device_->getBoardInfo() << "open() fails to parse xml... cannot continue ";
		return false;
	}

	yInfo() << device_->getBoardInfo() << " embObjCanBatterysensor::open(): verify the presence of the board and if its protocol version is correct";
	if (!device_->res->verifyEPprotocol(eoprot_endpoint_analogsensors))
	{
		yError() << device_->getBoardInfo() << " open() fails to verifyEPprotocol... cannot continue ";
		cleanup();
		return false;
	}

	yInfo() << device_->getBoardInfo() << " embObjCanBatterysensor::open(): verify and activate the FT service";
	eOmn_serv_parameter_t canBatteryData;
	canBatteryData.configuration.type = eomn_serv_AS_canbattery;
	canBatteryData.configuration.diagnosticsmode = eomn_serv_diagn_mode_NONE;
	canBatteryData.configuration.diagnosticsparam = 0;
	parser.toEomn(canBatteryData.configuration.data.as.canbattery);
	if (!device_->res->serviceVerifyActivate(eomn_serv_category_canbattery, &canBatteryData, 5.0))
	{
		yError() << device_->getBoardInfo() << " open() fails to serviceVerifyActivate... cannot continue ";
		cleanup();
		return false;
	}

	yInfo() << device_->getBoardInfo() << " embObjCanBatterysensor::open(): configure the FT service";
	if (false == sendConfig2boards(parser, device_->res))
	{
		yError() << device_->getBoardInfo() << " open() fails to sendConfig2boards... cannot continue";
		cleanup();
		return false;
	}

	yInfo() << device_->getBoardInfo() << " embObjCanBatterysensor::open(): impose the network variable which the ETH bord must stream up";
	if (false == initRegulars(parser, device_->res))
	{
		yError() << device_->getBoardInfo() << " open() fails to initRegulars... cannot continue";
		cleanup();
		return false;
	}

	yInfo() << device_->getBoardInfo() << " embObjCanBatterysensor::open(): start the FT service";
	if (!device_->res->serviceStart(eomn_serv_category_canbattery))
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

	yInfo() << device_->getBoardInfo() << " embObjCanBatterysensor::open(): start streaming of FT data";
	if (!sendStart2boards(parser, device_->res))
	{
		yError() << device_->getBoardInfo() << " open() fails to sendStart2boards... cannot continue";
		cleanup();
		return false;
	}

	device_->setOpen(true);
	return true;
}

bool embObjCanBatterysensor::getBatteryVoltage(double &voltage)
{
	return true;
}
bool embObjCanBatterysensor::getBatteryCurrent(double &current)
{
	return true;
}
bool embObjCanBatterysensor::getBatteryCharge(double &charge)
{
	return true;
}
bool embObjCanBatterysensor::getBatteryStatus(Battery_status &status)
{
	return true;
}
bool embObjCanBatterysensor::getBatteryTemperature(double &temperature)
{
	return true;
}

bool embObjCanBatterysensor::getBatteryInfo(std::string &battery_info)
{
	return true;
}

bool embObjCanBatterysensor::sendConfig2boards(ServiceParserCanBattery &parser, eth::AbstractEthResource *deviceRes)
{
	auto &canBattery = parser.getBatteryInfo();

	eOprotID32_t id32 = eo_prot_ID32dummy;
	eOas_canbattery_config_t cfg;
	cfg.period = canBattery.acquisitionRate;
	id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_canbattery, 0, eoprot_tag_as_canbattery_config);

	if (false == deviceRes->setcheckRemoteValue(id32, &cfg, 10, 0.010, 0.050))
	{
		yError() << device_->getBoardInfo() << " sendConfig2boards() while try to configure fperiod=" << cfg.period;
		return false;
	}

	if (device_->isVerbose())
	{
		yDebug() << device_->getBoardInfo() << " sendConfig2boards() correctly configured boards with period=" << cfg.period;
	}

	eOprotIndex_t eoprotIndex = eoprot_ID2index(id32);
	std::unique_lock<std::shared_mutex> lck(mutex_);
	//canBatteryData_ = {0, 0, 0, 0, 0, 0, 0, id};todo luca
	return true;
}

bool embObjCanBatterysensor::sendStart2boards(ServiceParserCanBattery &parser, eth::AbstractEthResource *deviceRes)
{
	eOprotID32_t id32 = eo_prot_ID32dummy;

	uint8_t enable = 1;

	const auto &batteryInfos = parser.getBatteryInfo();

	id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_canbattery, 0, eoprot_tag_as_canbattery_cmmnds_enable);

	if (false == deviceRes->setcheckRemoteValue(id32, &enable, 10, 0.010, 0.050))
	{
		yError() << device_->getBoardInfo() << " sendStart2boards() while try to enable the boards transmission";
		return false;
	}

	if (device_->isVerbose())
	{
		yDebug() << device_->getBoardInfo() << " sendStart2boards() correctly enabled the boards transmission";
	}
	return true;
}

bool embObjCanBatterysensor::initRegulars(ServiceParserCanBattery &parser, eth::AbstractEthResource *deviceRes)
{
	// configure regular rops

	vector<eOprotID32_t> id32v;
	eOprotID32_t id32 = eo_prot_ID32dummy;

	const auto &batteryInfos = parser.getBatteryInfo();
	id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_canbattery, 0, eoprot_tag_as_canbattery_status_timedvalue);
	id32v.push_back(id32);

	if (false == deviceRes->serviceSetRegulars(eomn_serv_category_canbattery, id32v))
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
/*
eth::iethresType_t embObjCanBatterysensor::type()
{
	return eth::iethres_analogft;
}

bool embObjCanBatterysensor::update(eOprotID32_t id32, double timestamp, void *rxdata)
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

	eOas_ft_timedvalue_t *data = (eOas_ft_timedvalue_t *)rxdata;
	if (!checkUpdateTimeout(id32, data->age))
	{
		return false;
	}

	std::unique_lock<std::shared_mutex> lck(mutex_);

	for (int index = 0; index < eoas_ft_6axis; ++index)
	{
		ftSensorsData_[eoprotIndex].data_[index] = data->values[index];
	}
	ftSensorsData_[eoprotIndex].timeStamp_ = data->age;

	temperaturesensordata_[eoprotIndex].data_ = data->temperature;
	temperaturesensordata_[eoprotIndex].timeStamp_ = calculateBoardTime(data->age);
	return true;
}

bool embObjCanBatterysensor::close()
{
	cleanup();
	return true;
}
*/
void embObjCanBatterysensor::cleanup(void)
{
	device_->cleanup(static_cast<eth::IethResource *>(this));
}
/*
bool embObjCanBatterysensor::getSixAxisForceTorqueSensorMeasure(size_t sensorIndex, yarp::sig::Vector &out, double &timestamp) const
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

size_t embObjCanBatterysensor::getNrOfSixAxisForceTorqueSensors() const
{
	return ftSensorsData_.size();
}

yarp::dev::MAS_status embObjCanBatterysensor::getSixAxisForceTorqueSensorStatus(size_t sensorindex) const
{
	return masStatus_[sensorindex];
}

bool embObjCanBatterysensor::getSixAxisForceTorqueSensorName(size_t sensorindex, std::string &name) const
{
	std::shared_lock<std::shared_mutex> lck(mutex_);
	name = ftSensorsData_.at(sensorindex).sensorName_;
	return true;
}

bool embObjCanBatterysensor::getSixAxisForceTorqueSensorFrameName(size_t sensorindex, std::string &frameName) const
{
	frameName = "";	 // Unused
	return true;
}

size_t embObjCanBatterysensor::getNrOfTemperatureSensors() const
{
	return temperaturesensordata_.size();
}

yarp::dev::MAS_status embObjCanBatterysensor::getTemperatureSensorStatus(size_t sensorindex) const
{
	return masStatus_[sensorindex];
}

bool embObjCanBatterysensor::getTemperatureSensorName(size_t sensorindex, std::string &name) const
{
	std::shared_lock<std::shared_mutex> lck(mutex_);
	name = ftSensorsData_.at(sensorindex).sensorName_;
	return true;
}

bool embObjCanBatterysensor::getTemperatureSensorFrameName(size_t sensorindex, std::string &frameName) const
{
	frameName = "";	 // Unused
	return true;
}

bool embObjCanBatterysensor::getTemperatureSensorMeasure(size_t sensorIndex, double &out, double &timestamp) const
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

bool embObjCanBatterysensor::getTemperatureSensorMeasure(size_t sensorIndex, yarp::sig::Vector &out, double &timestamp) const
{
	double value{0};
	getTemperatureSensorMeasure(sensorIndex, value, timestamp);
	out.resize(1);
	out[0] = value;
	return true;
}

bool embObjCanBatterysensor::checkUpdateTimeout(eOprotID32_t id32, eOabstime_t current)
{
	if (!checkUpdateTimeoutFlag_)
	{
		return true;
	}

	eOabstime_t diff = current - timeoutUpdate_[id32];
	if (timeoutUpdate_[id32] != 0 && current > timeoutUpdate_[id32] + updateTimeout_)
	{
		yError() << device_->getBoardInfo() << " update timeout for index:" << eoprot_ID2index(id32);
		timeoutUpdate_[id32] = current;
		masStatus_[eoprot_ID2index(id32)] = MAS_TIMEOUT;
		return false;
	}
	masStatus_[eoprot_ID2index(id32)] = MAS_OK;
	timeoutUpdate_[id32] = current;
	return true;
}

double embObjCanBatterysensor::calculateBoardTime(eOabstime_t current)
{
	if (!useBoardTimeFlag_)
	{
		return yarp::os::Time::now();
	}

	// Simulate real board time
	if (firstYarpTimestamp_ == 0)
	{
		firstYarpTimestamp_ = yarp::os::Time::now();
		firstCanTimestamp_ = current;
	}
	double realtime = firstYarpTimestamp_ + (double)(current - firstCanTimestamp_) / 1000000;  // Simulate real board time
	return realtime;
}
*/