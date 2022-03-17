/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 * Author: Luca Tricerri
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "serviceParserMultipleFt.h"

#include "serviceParser.h"

ServiceParserMultipleFt::ServiceParserMultipleFt()
{
}

bool ServiceParserMultipleFt::checkPropertyCanBoards(const Bottle& property, bool& formaterror)
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
		yError() << "ServiceParserMultipleFt::check() cannot find PROPERTIES.CANBOARDS.type";
		return false;
	}

	Bottle propertyCanBoardProtocol = Bottle(propertyCanBoard.findGroup("PROTOCOL"));
	if (propertyCanBoardProtocol.isNull())
	{
		yError() << "ServiceParserMultipleFt::check() cannot find PROPERTIES.CANBOARDS.PROTOCOL";
		return false;
	}

	Bottle propertyCanBoardProtocolMajor = Bottle(propertyCanBoardPROTOCOL.findGroup("major"));
	if (propertyCanBoardProtocolMajor.isNull())
	{
		yError() << "ServiceParserMultipleFt::check() cannot find PROPERTIES.CANBOARDS.PROTOCOL.major";
		return false;
	}

	Bottle propertyCanBoardProtocolMinor = Bottle(propertyCanBoardPROTOCOL.findGroup("minor"));
	if (propertyCanBoardProtocolMinor.isNull())
	{
		yError() << "ServiceParserMultipleFt::check() cannot find PROPERTIES.CANBOARDS.PROTOCOL.minor";
		return false;
	}

	Bottle propertyCanBoardFirmware = Bottle(b_PROPERTIES_CANBOARDS.findGroup("FIRMWARE"));
	if (propertyCanBoardFirmware.isNull())
	{
		yError() << "ServiceParserMultipleFt::check() cannot find PROPERTIES.CANBOARDS.FIRMWARE";
		return false;
	}

	Bottle propertyCanBoardFirmwareMajor = Bottle(propertyCanBoardFIRMWARE.findGroup("major"));
	if (propertyCanBoardFirmwareMajor.isNull())
	{
		yError() << "ServiceParserMultipleFt::check() cannot find PROPERTIES.CANBOARDS.FIRMWARE.major";
		return false;
	}

	Bottle propertyCanBoardFirmwareMinor = Bottle(propertyCanBoardFIRMWARE.findGroup("minor"));
	if (propertyCanBoardFirmwareMinor.isNull())
	{
		yError() << "ServiceParserMultipleFt::check() cannot find PROPERTIES.CANBOARDS.FIRMWARE.minor";
		return false;
	}

	Bottle propertyCanBoardFirmwareBuild = Bottle(propertyCanBoardFIRMWARE.findGroup("build"));
	if (propertyCanBoardFirmwareBuild.isNull())
	{
		yError() << "ServiceParserMultipleFt::check() cannot find PROPERTIES.CANBOARDS.FIRMWARE.build";
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
		std::string boardType = propertyCanBoardType.get(i).asString();
		int protocolMajor = propertyCanBoardProtocolMajor.get(i).asInt32();
		int protocolMinor = propertyCanBoardProtocolMinor.get(i).asInt32();
		int firmwareMajor = propertyCanBoardFirmwareMajor.get(i).asInt32();
		int firmwareMinor = propertyCanBoardFirmwareMinor.get(i).asInt32();
		int firmwareBuild = propertyCanBoardFirmwareBuild.get(i).asInt32();

		for (auto& current : ftInfo_)
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

bool ServiceParserMultipleFt::checkPropertySensors(const Bottle& property, bool& formaterror)
{
	Bottle propertySensors = Bottle(property.findGroup("SENSORS"));
	if (propertySensors.isNull())
	{
		yError() << "ServiceParserMultipleFt::checkPropertySensors() cannot find PROPERTIES.SENSORS";
		return false;
	}

	Bottle propertySensorsId = Bottle(propertySensors.findGroup("id"));
	if (propertySensorsId.isNull())
	{
		yError() << "ServiceParserMultipleFt::checkPropertySensors() cannot find PROPERTIES.SENSORS.id";
		return false;
	}

	Bottle propertySensorsBoard = Bottle(propertySensors.findGroup("board"));
	if (propertySensorsBoard.isNull())
	{
		yError() << "ServiceParserMultipleFt::checkPropertySensors() cannot find PROPERTIES.SENSORS.type";
		return false;
	}

	Bottle propertySensorsLocation = Bottle(propertySensors.findGroup("location"));
	if (propertySensorsLocation.isNull())
	{
		yError() << "ServiceParserMultipleFt::checkPropertySensors() cannot find PROPERTIES.SENSORS.location";
		return false;
	}

	size_t sensorSize = propertySensorsId.size();

	// CHECK size
	if (settingsEnabledSensors.size() != propertySensorsBoard.size())
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
	for (size_t index = 1 /*first is tagname*/; index < sensorSize; i++)
	{
		std::string id = propertySensorsId.get(i).asString();
		std::string board = propertySensorsBoard.get(i).asString();
		std::string location = propertySensorsLocation.get(i).asString();

		if (ftInfo_.find(id) == ftInfo_.end())
			continue;
		auto currentFt = ftInfo_.at(id);
		currentFt.location = location;
		currentFt.board = board;
	}

	return true;
}

bool ServiceParserMultipleFt::checkSettings(const Bottle& service, bool& formaterror)
{
	Bottle settings = Bottle(service.findGroup("SETTINGS"));
	if (settings.isNull())
	{
		yError() << "ServiceParserMultipleFt::checkSettings() cannot find SETTINGS";
		return false;
	}
	formaterror = false;

	Bottle settingsAcquisitionRate = Bottle(settings.findGroup("acquisitionRate"));
	if (settingsAcquisitionRate.isNull())
	{
		yError() << "ServiceParserMultipleFt::checkSettings() cannot find SETTINGS.acquisitionRate";
		return false;
	}

	Bottle settingsAcquisitionTempRate = Bottle(settings.findGroup("temperature-acquisitionRate"));
	if (settingsAcquisitionTempRate.isNull())
	{
		yError() << "ServiceParserMultipleFt::parseService() for embObjMultipleFTsensor device cannot find SETTINGS.temperature-acquisitionRate";
		return false;
	}

	Bottle settingsUseCalibration = Bottle(settings.findGroup("useCalibration"));
	if (settingsUseCalibration.isNull())
	{
		yError() << "ServiceParserMultipleFt::checkSettings() cannot find FT_SETTINGS.useCalibration";
		return false;
	}

	Bottle settingsEnabledSensors = Bottle(settings.findGroup("enabledSensors"));
	if (settingsEnabledSensors.isNull())
	{
		yError() << "ServiceParserMultipleFt::checkSettings() cannot find SETTINGS.enabledSensors";
		return false;
	}

	// Check size
	if (settingsEnabledSensors.size() != settingsAcquisitionRate.size())
	{
		yError() << "ServiceParserMultipleFt::checkSettings --> ft-acquisitionRate size";
		return false;
	}
	if (settingsAcquisitionRate.size() != settingsAcquisitionTempRate.size())
	{
		yError() << "ServiceParserMultipleFt::checkSettings --> temperature-acquisitionRate size";
		return false;
	}
	if (settingsAcquisitionTempRate.size() != settingsUseCalibration.size())
	{
		yError() << "ServiceParserMultipleFt::checkSettings --> usecalibration size";
		return false;
	}

	// TODO Luca check acquisition rate
	size_t enabledSensorSize = settingsEnabledSensors.size();
	for (size_t index = 1 /*first is tagname*/; index < enabledSensorSize; i++)
	{
		std::string id = settingsEnabledSensors.get(i).asString();
		int acquisitionRate = settingsAcquisitionRate.get(i).asInt32();
		int acquisitionTempRate = settingsAcquisitionTempRate.get(i).asInt32();
		int useCalibration = settingsUseCalibration.get(i).asInt32();
		FtInfo currentSensor{acquisitionRate, acquisitionTempRate, useCalibration};
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

bool ServiceParserMultipleFt::checkServiceType(const Bottle& service, bool& formaterror)
{
	if (false == service.check("type"))
	{
		yError() << "ServiceParserMultipleFt::check() cannot find SERVICE.type";
		return false;
	}

	Bottle b_type(service.find("type").asString());
	if (false == convert(b_type.toString(), asService_.type, formaterror))
	{
		yError() << "ServiceParserMultipleFt::check() has found unknown SERVICE.type = " << b_type.toString();
		return false;
	}
	if (eomn_serv_AS_ft != asService_.type)
	{
		yError() << "ServiceParserMultipleFt::check() has found wrong SERVICE.type = " << asService_.type << "it must be"
				 << "TODO: tostring() function";
		return false;
	}
	return true;
}

bool ServiceParserMultipleFt::checkCanMonitor(const Bottle& service, bool& formaterror)
{
	Bottle canMonitor = Bottle(service.findGroup("CANMONITOR"));
	if (canMonitor.isNull())
	{
		yError() << "ServiceParserMultipleFt::check() cannot find PROPERTIES.CANMONITOR";
		return false;
	}

	if (false == canMonitor.check("checkrate"))
	{
		yError() << "ServiceParserMultipleFt::check() cannot find canMonitor.checkrate";
		return false;
	}
	if (false == canMonitor.check("reportmode"))
	{
		yError() << "ServiceParserMultipleFt::check() cannot find canMonitor.reportmode";
		return false;
	}
	if (false == canMonitor.check("periodicreportrate"))
	{
		yError() << "ServiceParserMultipleFt::check() cannot find canMonitor.periodicreportrate";
		return false;
	}

	int checkrate=canMonitor.find("checkrate").asInt32());
	std::string reportmode=canMonitor.find("reportmode").asString());
	if (stringToReport.find("reportmode") == stringToFind.end())
	{
		yError() << "ServiceParserMultipleFt::check() wrong canMonitor.reportmode";
		return false;
	}
	int periodicreportrate=canMonitor.find("periodicreportrate").asInt32());

	canMonitor_ = {checkrate, stringToReport.at(reportmode), periodicreportrate};

	return true;
}

bool ServiceParserMultipleFt::parse(yarp::os::Searchable& config)
{
	bool formaterror = false;
	if (eomn_serv_AS_ft != type)
	{
		yError() << "ServiceParser::checkAlalogForMultipleFT() is called with wrong type";
		return false;
	}

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
	out.canmonitorconfig={canMonitor_.checkrate,canMonitor_.reportmode,canMonitor_.periodicreportrate};

	for (auto& current : ftInfo_)
	{
		//out.arrayofsensors=
	}
}
