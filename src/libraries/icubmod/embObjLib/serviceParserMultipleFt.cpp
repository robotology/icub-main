/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 * Author: Luca Tricerri
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "serviceParserMultipleFt.h"

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

ServiceParserMultipleFt::ServiceParserMultipleFt()
{
}

bool ServiceParserMultipleFt::checkPropertyCanBoards(const Bottle &property, bool &formaterror)
{
	Bottle propertyCanBoard = Bottle(property.findGroup("CANBOARDS"));
	if (propertyCanBoard.isNull())
	{
		yError() << "ServiceParserMultipleFt::check() cannot find PROPERTIES.CANBOARDS";
		return false;
	}

	Bottle propertyCanBoardType = propertyCanBoard.findGroup("type");
	if (propertyCanBoardType.isNull())
	{
		yError() << "ServiceParserMultipleFt::check() cannot find "
					"PROPERTIES.CANBOARDS.type";
		return false;
	}

	Bottle propertyCanBoardProtocol = Bottle(propertyCanBoard.findGroup("PROTOCOL"));
	if (propertyCanBoardProtocol.isNull())
	{
		yError() << "ServiceParserMultipleFt::check() cannot find "
					"PROPERTIES.CANBOARDS.PROTOCOL";
		return false;
	}

	Bottle propertyCanBoardProtocolMajor = Bottle(propertyCanBoardProtocol.findGroup("major"));
	if (propertyCanBoardProtocolMajor.isNull())
	{
		yError() << "ServiceParserMultipleFt::check() cannot find "
					"PROPERTIES.CANBOARDS.PROTOCOL.major";
		return false;
	}

	Bottle propertyCanBoardProtocolMinor = Bottle(propertyCanBoardProtocol.findGroup("minor"));
	if (propertyCanBoardProtocolMinor.isNull())
	{
		yError() << "ServiceParserMultipleFt::check() cannot find "
					"PROPERTIES.CANBOARDS.PROTOCOL.minor";
		return false;
	}

	Bottle propertyCanBoardFirmware = Bottle(propertyCanBoard.findGroup("FIRMWARE"));
	if (propertyCanBoardFirmware.isNull())
	{
		yError() << "ServiceParserMultipleFt::check() cannot find "
					"PROPERTIES.CANBOARDS.FIRMWARE";
		return false;
	}

	Bottle propertyCanBoardFirmwareMajor = Bottle(propertyCanBoardFirmware.findGroup("major"));
	if (propertyCanBoardFirmwareMajor.isNull())
	{
		yError() << "ServiceParserMultipleFt::check() cannot find "
					"PROPERTIES.CANBOARDS.FIRMWARE.major";
		return false;
	}

	Bottle propertyCanBoardFirmwareMinor = Bottle(propertyCanBoardFirmware.findGroup("minor"));
	if (propertyCanBoardFirmwareMinor.isNull())
	{
		yError() << "ServiceParserMultipleFt::check() cannot find "
					"PROPERTIES.CANBOARDS.FIRMWARE.minor";
		return false;
	}

	Bottle propertyCanBoardFirmwareBuild = Bottle(propertyCanBoardFirmware.findGroup("build"));
	if (propertyCanBoardFirmwareBuild.isNull())
	{
		yError() << "ServiceParserMultipleFt::check() cannot find "
					"PROPERTIES.CANBOARDS.FIRMWARE.build";
		return false;
	}

	// Check size
	if (propertyCanBoardType.size() != propertyCanBoardProtocolMajor.size())
	{
		yError() << "ServiceParserMultipleFt::checkSettings --> protocol major size";
		return false;
	}
	if (propertyCanBoardProtocolMajor.size() != propertyCanBoardProtocolMinor.size())
	{
		yError() << "ServiceParserMultipleFt::checkSettings --> protocol minor size";
		return false;
	}
	if (propertyCanBoardProtocolMinor.size() != propertyCanBoardFirmwareMajor.size())
	{
		yError() << "ServiceParserMultipleFt::checkSettings --> firmware major size";
		return false;
	}
	if (propertyCanBoardFirmwareMajor.size() != propertyCanBoardFirmwareMinor.size())
	{
		yError() << "ServiceParserMultipleFt::checkSettings --> firmware minor size";
		return false;
	}
	if (propertyCanBoardFirmwareMinor.size() != propertyCanBoardFirmwareBuild.size())
	{
		yError() << "ServiceParserMultipleFt::checkSettings --> firmware build size";
		return false;
	}

	size_t size = propertyCanBoardType.size();
	for (int index = 1; index < size; ++index)
	{
		std::string boardType = propertyCanBoardType.get(index).asString();
		int protocolMajor = propertyCanBoardProtocolMajor.get(index).asInt32();
		int protocolMinor = propertyCanBoardProtocolMinor.get(index).asInt32();
		int firmwareMajor = propertyCanBoardFirmwareMajor.get(index).asInt32();
		int firmwareMinor = propertyCanBoardFirmwareMinor.get(index).asInt32();
		int firmwareBuild = propertyCanBoardFirmwareBuild.get(index).asInt32();

		for (auto &[key, current] : ftInfo_)
		{
			if (current.board != boardType)
				continue;
			current.majorProtocol = protocolMajor;
			current.minorProtocol = protocolMinor;
			current.majorFirmware = firmwareMajor;
			current.minorFirmware = firmwareMinor;
			current.buildFirmware = firmwareBuild;
		}
	}

	return true;
}

bool ServiceParserMultipleFt::checkPropertySensors(const Bottle &property, bool &formaterror)
{
	Bottle propertySensors = Bottle(property.findGroup("SENSORS"));
	if (propertySensors.isNull())
	{
		yError() << "ServiceParserMultipleFt::checkPropertySensors() cannot find "
					"PROPERTIES.SENSORS";
		return false;
	}

	Bottle propertySensorsId = Bottle(propertySensors.findGroup("id"));
	if (propertySensorsId.isNull())
	{
		yError() << "ServiceParserMultipleFt::checkPropertySensors() cannot find "
					"PROPERTIES.SENSORS.id";
		return false;
	}

	Bottle propertySensorsBoard = Bottle(propertySensors.findGroup("board"));
	if (propertySensorsBoard.isNull())
	{
		yError() << "ServiceParserMultipleFt::checkPropertySensors() cannot find "
					"PROPERTIES.SENSORS.type";
		return false;
	}

	Bottle propertySensorsLocation = Bottle(propertySensors.findGroup("location"));
	if (propertySensorsLocation.isNull())
	{
		yError() << "ServiceParserMultipleFt::checkPropertySensors() cannot find "
					"PROPERTIES.SENSORS.location";
		return false;
	}

	// CHECK size
	if (propertySensorsId.size() != propertySensorsBoard.size())
	{
		yError() << "ServiceParserMultipleFt::checkPropertySensors --> board size";
		return false;
	}
	if (propertySensorsBoard.size() != propertySensorsLocation.size())
	{
		yError() << "ServiceParserMultipleFt::checkPropertySensors --> location size";
		return false;
	}

	size_t sensorSize = propertySensorsId.size();
	for (size_t index = 1 /*first is tagname*/; index < sensorSize; index++)
	{
		std::string id = propertySensorsId.get(index).asString();
		std::string board = propertySensorsBoard.get(index).asString();
		std::string location = propertySensorsLocation.get(index).asString();
		if (ftInfo_.find(id) == ftInfo_.end())
			continue;
		auto &currentFt = ftInfo_.at(id);

		try
		{
			currentFt.port = std::stoi(location.substr(3, 1));
		}
		catch (const std::exception &e)
		{
			yError() << "ServiceParser::checkPropertySensors() invalid can port";
			return false;
		}
		try
		{
			currentFt.address = std::stoi(location.substr(5, location.size() - 5));
		}
		catch (const std::exception &e)
		{
			yError() << "ServiceParser::checkPropertySensors() invalid can address";
			return false;
		}

		currentFt.board = board;
	}

	// TODO check missing SENSORS for SETTINGS

	return true;
}

bool ServiceParserMultipleFt::checkSettings(const Bottle &service, bool &formaterror)
{
	Bottle settings = Bottle(service.findGroup("SETTINGS"));
	if (settings.isNull())
	{
		yError() << "ServiceParserMultipleFt::checkSettings() cannot find SETTINGS";
		return false;
	}
	formaterror = false;

	Bottle settingsFtPeriod = Bottle(settings.findGroup("ftPeriod"));
	if (settingsFtPeriod.isNull())
	{
		yError() << "ServiceParserMultipleFt::checkSettings() cannot find "
					"SETTINGS.ftPeriod";
		return false;
	}

	Bottle settingsTemperaturePeriod = Bottle(settings.findGroup("temperaturePeriod"));
	if (settingsTemperaturePeriod.isNull())
	{
		yError() << "ServiceParserMultipleFt::parseService() for embObjMultipleFTsensor "
					"device cannot find SETTINGS.temperaturePeriod";
		return false;
	}

	Bottle settingsUseCalibration = Bottle(settings.findGroup("useCalibration"));
	if (settingsUseCalibration.isNull())
	{
		yError() << "ServiceParserMultipleFt::checkSettings() cannot find "
					"FT_SETTINGS.useCalibration";
		return false;
	}

	Bottle settingsEnabledSensors = Bottle(settings.findGroup("enabledSensors"));
	if (settingsEnabledSensors.isNull())
	{
		yError() << "ServiceParserMultipleFt::checkSettings() cannot find "
					"SETTINGS.enabledSensors";
		return false;
	}

	// Check size
	if (settingsEnabledSensors.size() != settingsFtPeriod.size())
	{
		yError() << "ServiceParserMultipleFt::checkSettings --> FtPeriod size";
		return false;
	}
	if (settingsFtPeriod.size() != settingsTemperaturePeriod.size())
	{
		yError() << "ServiceParserMultipleFt::checkSettings --> "
					"temperaturePeriod size";
		return false;
	}
	if (settingsTemperaturePeriod.size() != settingsUseCalibration.size())
	{
		yError() << "ServiceParserMultipleFt::checkSettings --> usecalibration size";
		return false;
	}

	// TODO Luca check acquisition rate
	size_t enabledSensorSize = settingsEnabledSensors.size();
	if (enabledSensorSize > 4)
	{
		yError() << "ServiceParserMultipleFt::checkSettings --> too mutch sensors";
		return false;
	}

	for (size_t index = 1 /*first is tagname*/; index < enabledSensorSize; index++)
	{
		std::string id = settingsEnabledSensors.get(index).asString();
		uint8_t acquisitionRate = (uint8_t)settingsFtPeriod.get(index).asInt8();
		uint8_t acquisitionTempRate = (uint8_t)settingsTemperaturePeriod.get(index).asInt8();
		bool useCalibration = settingsUseCalibration.get(index).asBool();
		eOas_ft_mode_t calib = eoas_ft_mode_calibrated;
		if (!useCalibration)
		{
			calib = eoas_ft_mode_raw;
		}
		FtInfo currentSensor{acquisitionRate, acquisitionTempRate, calib};
		ftInfo_[id] = currentSensor;
	}

	// Duplicate name check
	if (settingsEnabledSensors.size() - 1 != ftInfo_.size())
	{
		yError() << "ServiceParserMultipleFt::checkSettings --> id duplicate name";
		return false;
	}

	return true;
}

bool ServiceParserMultipleFt::checkServiceType(const Bottle &service, bool &formaterror)
{
	if (false == service.check("type"))
	{
		yError() << "ServiceParserMultipleFt::check() cannot find SERVICE.type";
		return false;
	}
	std::string serviceType = service.find("type").asString();
	eOmn_serv_type_t serviceTypeEomn = eomn_string2servicetype(serviceType.c_str());

	if (eomn_serv_AS_ft != serviceTypeEomn)
	{
		yError() << "ServiceParserMultipleFt::check() has found wrong SERVICE.type = " << serviceType << "it must be eomn_serv_AS_ft";
		return false;
	}
	return true;
}

bool ServiceParserMultipleFt::checkCanMonitor(const Bottle &service, bool &formaterror)
{
	Bottle canMonitor = Bottle(service.findGroup("CANMONITOR"));
	if (canMonitor.isNull())
	{
		yError() << "ServiceParserMultipleFt::check() cannot find PROPERTIES.CANMONITOR";
		return false;
	}

	if (false == canMonitor.check("checkPeriod"))
	{
		yError() << "ServiceParserMultipleFt::check() cannot find canMonitor.checkrate";
		return false;
	}
	if (false == canMonitor.check("ratePeriod"))
	{
		yError() << "ServiceParserMultipleFt::check() cannot find "
					"canMonitor.periodicreportrate";
		return false;
	}
	if (false == canMonitor.check("reportMode"))
	{
		yError() << "ServiceParserMultipleFt::check() cannot find canMonitor.reportmode";
		return false;
	}

	// TODO check validita valori
	int checkPeriod = canMonitor.find("checkPeriod").asInt();
	if (checkPeriod > 254)
	{
		yError() << "ServiceParserMultipleFt::check() wrong canMonitor.checkPeriod "
					"too big";
		return false;
	}
	int periodicreportrate = canMonitor.find("ratePeriod").asInt();
	if (periodicreportrate > 65535)
	{
		yError() << "ServiceParserMultipleFt::check() wrong canMonitor.ratePeriod "
					"too big";
		return false;
	}
	std::string reportmode = canMonitor.find("reportMode").asString();
	eObrd_canmonitor_reportmode_t reportmodeEobrd = eoboards_string2reportmode(reportmode.c_str(), true);

	if (reportmodeEobrd == eobrd_canmonitor_reportmode_unknown)
	{
		yError() << "ServiceParserMultipleFt::check() wrong canMonitor.reportmode";
		return false;
	}

	canMonitor_ = {((int8_t)checkPeriod), reportmodeEobrd, ((int16_t)periodicreportrate)};

	return true;
}

bool ServiceParserMultipleFt::parse(const yarp::os::Searchable &config)
{
	bool formaterror = false;

	Bottle service(config.findGroup("SERVICE"));
	if (service.isNull())
	{
		yError() << "ServiceParser::check() cannot find SERVICE group";
		return false;
	}

	Bottle properties = Bottle(service.findGroup("PROPERTIES"));
	if (properties.isNull())
	{
		yError() << "ServiceParser::check() cannot find PROPERTIES";
		return false;
	}

	if (!checkServiceType(service, formaterror))
		return false;

	if (!checkSettings(service, formaterror))
		return false;

	if (!checkPropertySensors(properties, formaterror))
		return false;

	if (!checkPropertyCanBoards(properties, formaterror))
		return false;

	if (!checkCanMonitor(service, formaterror))
		return false;

	return true;
}

eOmn_serv_config_data_as_ft_t ServiceParserMultipleFt::toEomn() const
{
	eOmn_serv_config_data_as_ft_t out;
	out.canmonitorconfig = canMonitor_;

	EOarray *ar = eo_array_New(/*ftInfo_.size() TODO LUCA */eOas_ft_sensors_maxnumber, sizeof(eOas_ft_sensordescriptor_t), (void *)(&(out.arrayofsensors)));

	for (const auto &[key, value] : ftInfo_)
	{
		eOas_ft_sensordescriptor_t item = value.toEomn();
		eo_array_PushBack(ar, &item);
	}
	return out;
}

std::map<std::string, FtInfo> &ServiceParserMultipleFt::getFtInfo()
{
	return ftInfo_;
}
