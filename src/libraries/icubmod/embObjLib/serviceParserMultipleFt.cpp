#include "serviceParserMultipleFt.h"

#include "serviceParser.h"

ServiceParserMultipleFt::ServiceParserMultipleFt(servAScollector_t& asService) : asService_(asService)
{
}

bool ServiceParserMultipleFt::checkPropertyCanBoards(const Bottle& property, bool& formaterror)
{
	Bottle b_PROPERTIES_CANBOARDS = Bottle(property.findGroup("CANBOARDS"));
	if (b_PROPERTIES_CANBOARDS.isNull())
	{
		yError() << "ServiceParserMultipleFt::check() cannot find PROPERTIES.CANBOARDS";
		return false;
	}

	// now get type, PROTOCOL.major/minor, FIRMWARE.major/minor/build and see their sizes. the must be all equal.
	// for mais and strain and so far for intertials it must be numboards = 1.
	Bottle b_PROPERTIES_CANBOARDS_type = b_PROPERTIES_CANBOARDS.findGroup("type");
	if (b_PROPERTIES_CANBOARDS_type.isNull())
	{
		yError() << "ServiceParserMultipleFt::check() cannot find PROPERTIES.CANBOARDS.type";
		return false;
	}

	Bottle b_PROPERTIES_CANBOARDS_PROTOCOL = Bottle(b_PROPERTIES_CANBOARDS.findGroup("PROTOCOL"));
	if (b_PROPERTIES_CANBOARDS_PROTOCOL.isNull())
	{
		yError() << "ServiceParserMultipleFt::check() cannot find PROPERTIES.CANBOARDS.PROTOCOL";
		return false;
	}
	Bottle b_PROPERTIES_CANBOARDS_PROTOCOL_major = Bottle(b_PROPERTIES_CANBOARDS_PROTOCOL.findGroup("major"));
	if (b_PROPERTIES_CANBOARDS_PROTOCOL_major.isNull())
	{
		yError() << "ServiceParserMultipleFt::check() cannot find PROPERTIES.CANBOARDS.PROTOCOL.major";
		return false;
	}
	Bottle b_PROPERTIES_CANBOARDS_PROTOCOL_minor = Bottle(b_PROPERTIES_CANBOARDS_PROTOCOL.findGroup("minor"));
	if (b_PROPERTIES_CANBOARDS_PROTOCOL_minor.isNull())
	{
		yError() << "ServiceParserMultipleFt::check() cannot find PROPERTIES.CANBOARDS.PROTOCOL.minor";
		return false;
	}
	Bottle b_PROPERTIES_CANBOARDS_FIRMWARE = Bottle(b_PROPERTIES_CANBOARDS.findGroup("FIRMWARE"));
	if (b_PROPERTIES_CANBOARDS_FIRMWARE.isNull())
	{
		yError() << "ServiceParserMultipleFt::check() cannot find PROPERTIES.CANBOARDS.FIRMWARE";
		return false;
	}
	Bottle b_PROPERTIES_CANBOARDS_FIRMWARE_major = Bottle(b_PROPERTIES_CANBOARDS_FIRMWARE.findGroup("major"));
	if (b_PROPERTIES_CANBOARDS_FIRMWARE_major.isNull())
	{
		yError() << "ServiceParserMultipleFt::check() cannot find PROPERTIES.CANBOARDS.FIRMWARE.major";
		return false;
	}
	Bottle b_PROPERTIES_CANBOARDS_FIRMWARE_minor = Bottle(b_PROPERTIES_CANBOARDS_FIRMWARE.findGroup("minor"));
	if (b_PROPERTIES_CANBOARDS_FIRMWARE_minor.isNull())
	{
		yError() << "ServiceParserMultipleFt::check() cannot find PROPERTIES.CANBOARDS.FIRMWARE.minor";
		return false;
	}
	Bottle b_PROPERTIES_CANBOARDS_FIRMWARE_build = Bottle(b_PROPERTIES_CANBOARDS_FIRMWARE.findGroup("build"));
	if (b_PROPERTIES_CANBOARDS_FIRMWARE_build.isNull())
	{
		yError() << "ServiceParserMultipleFt::check() cannot find PROPERTIES.CANBOARDS.FIRMWARE.build";
		return false;
	}

	size_t tmp = b_PROPERTIES_CANBOARDS_type.size();
	int numboards = tmp - 1;  // first position of bottle contains the tag "type"

	// check if all other fields have the same size.
	if ((tmp != b_PROPERTIES_CANBOARDS_PROTOCOL_major.size()) || (tmp != b_PROPERTIES_CANBOARDS_PROTOCOL_minor.size()) || (tmp != b_PROPERTIES_CANBOARDS_FIRMWARE_major.size()) ||
		(tmp != b_PROPERTIES_CANBOARDS_FIRMWARE_minor.size()) || (tmp != b_PROPERTIES_CANBOARDS_FIRMWARE_build.size()))
	{
		yError() << "ServiceParserMultipleFt::check() in PROPERTIES.CANBOARDS some param has inconsistent length";
		return false;
	}

	asService_.properties.canboards.resize(0);

	formaterror = false;
	for (int i = 0; i < numboards; i++)
	{
		servCanBoard_t item;

		convert(b_PROPERTIES_CANBOARDS_type.get(i + 1).asString(), item.type, formaterror);
		convert(b_PROPERTIES_CANBOARDS_PROTOCOL_major.get(i + 1).asInt32(), item.protocol.major, formaterror);
		convert(b_PROPERTIES_CANBOARDS_PROTOCOL_minor.get(i + 1).asInt32(), item.protocol.minor, formaterror);

		convert(b_PROPERTIES_CANBOARDS_FIRMWARE_major.get(i + 1).asInt32(), item.firmware.major, formaterror);
		convert(b_PROPERTIES_CANBOARDS_FIRMWARE_minor.get(i + 1).asInt32(), item.firmware.minor, formaterror);
		convert(b_PROPERTIES_CANBOARDS_FIRMWARE_build.get(i + 1).asInt32(), item.firmware.build, formaterror);

		asService_.properties.canboards.push_back(item);
	}

	if (true == formaterror)
	{
		yError() << "ServiceParserMultipleFt::check() has detected an illegal format for some of the params of PROPERTIES.CANBOARDS some param has inconsistent length";
		return false;
	}

	return true;
}

bool ServiceParserMultipleFt::checkPropertySensors(const Bottle& property, bool& formaterror)
{
	Bottle b_PROPERTIES_SENSORS = Bottle(property.findGroup("SENSORS"));
	if (b_PROPERTIES_SENSORS.isNull())
	{
		yError() << "ServiceParserMultipleFt::check() cannot find PROPERTIES.SENSORS";
		return false;
	}
	Bottle b_PROPERTIES_SENSORS_id = Bottle(b_PROPERTIES_SENSORS.findGroup("id"));
	if (b_PROPERTIES_SENSORS_id.isNull())
	{
		yError() << "ServiceParserMultipleFt::check() cannot find PROPERTIES.SENSORS.id";
		return false;
	}
	Bottle b_PROPERTIES_SENSORS_type = Bottle(b_PROPERTIES_SENSORS.findGroup("type"));
	if (b_PROPERTIES_SENSORS_type.isNull())
	{
		yError() << "ServiceParserMultipleFt::check() cannot find PROPERTIES.SENSORS.type";
		return false;
	}
	Bottle b_PROPERTIES_SENSORS_location = Bottle(b_PROPERTIES_SENSORS.findGroup("location"));
	if (b_PROPERTIES_SENSORS_location.isNull())
	{
		yError() << "ServiceParserMultipleFt::check() cannot find PROPERTIES.SENSORS.location";
		return false;
	}

	b_PROPERTIES_SENSORS_boardtype.clear();

	size_t tmp = b_PROPERTIES_SENSORS_id.size();
	int numsensors = tmp - 1;  // first position of bottle contains the tag "id"

	// check if all other fields have the same size.
	if ((tmp != b_PROPERTIES_SENSORS_type.size()) || (tmp != b_PROPERTIES_SENSORS_location.size()))
	{
		yError() << "ServiceParserMultipleFt::check() in PROPERTIES.SENSORS some param has inconsistent length";
		return false;
	}

	asService_.properties.sensors.resize(0);

	formaterror = false;
	for (int i = 0; i < numsensors; i++)
	{
		servAnalogSensor_t item;
		item.type = eoas_none;
		item.location.any.place = eobrd_place_none;

		convert(b_PROPERTIES_SENSORS_id.get(i + 1).asString(), item.id, formaterror);
		convert(b_PROPERTIES_SENSORS_type.get(i + 1).asString(), item.type, formaterror);
		convert(b_PROPERTIES_SENSORS_location.get(i + 1).asString(), item.location, formaterror);
		item.boardtype = eobrd_none;

		asService_.properties.sensors.push_back(item);
	}

	// in here we could decide to return false if any previous conversion function has returned error
	// bool fromStringToBoolean(string str, bool &anyerror); // inside: if error then .... be sure to set error = true. dont set it to false.

	if (true == formaterror)
	{
		yError() << "ServiceParserMultipleFt::check() has detected an illegal format for some of the params of PROPERTIES.SENSORS some param has inconsistent length";
		return false;
	}

	return true;
}

bool ServiceParserMultipleFt::checkSettings(const Bottle& service,  bool& formaterror)
{
	Bottle settings = Bottle(service.findGroup("SETTINGS"));
	if (settings.isNull())
	{
		yError() << "ServiceParserMultipleFt::check() cannot find SETTINGS";
		return false;
	}

	Bottle b_SETTINGS_acquisitionRate = Bottle(settings.findGroup("acquisitionRate"));
	if (b_SETTINGS_acquisitionRate.isNull())
	{
		yError() << "ServiceParserMultipleFt::check() cannot find SETTINGS.acquisitionRate";
		return false;
	}

	Bottle b_SETTINGS_temp = Bottle(settings.findGroup("temperature-acquisitionRate"));
	if (b_SETTINGS_temp.isNull())
	{
		yError() << "ServiceParserMultipleFt::parseService() for embObjMultipleFTsensor device cannot find SETTINGS.temperature-acquisitionRate";
		return false;
	}
	asService_.settings.temperatureAcquisitionrate = b_SETTINGS_temp.get(1).asInt32();

	Bottle b_FT_SETTINGS_useCalibration = Bottle(settings.findGroup("useCalibration"));
	if (b_FT_SETTINGS_useCalibration.isNull())
	{
		yError() << "ServiceParserMultipleFt::check() cannot find FT_SETTINGS.useCalibration";
		return false;
	}

	formaterror = false;
	convert(b_FT_SETTINGS_useCalibration.get(1).asString(), asService_.settings.useCalibration, formaterror);

	if (true == formaterror)
	{
		yError() << "ServiceParserMultipleFt::check() has detected an illegal format for paramf FT_SETTINGS.useCalibration";
		return false;
	}

	Bottle b_SETTINGS_enabledSensors = Bottle(settings.findGroup("enabledSensors"));
	if (b_SETTINGS_enabledSensors.isNull())
	{
		yError() << "ServiceParserMultipleFt::check() cannot find SETTINGS.enabledSensors";
		return false;
	}
	size_t s = b_SETTINGS_enabledSensors.size();
	size_t numenabledsensors = (0 == s) ? (0) : (s - 1);  // first position of bottle contains the tag "enabledSensors"

	// the enabled must be <= the sensors.
	int tmp = asService_.properties.sensors.size();
	if (numenabledsensors > asService_.properties.sensors.size())
	{
		yError() << "ServiceParserMultipleFt::check() in SETTINGS.enabledSensors there are too many items with respect to supported sensors:" << numenabledsensors << "vs."
				 << asService_.properties.sensors.size();
		return false;
	}

	// TODO check acquisition range
	convert(b_SETTINGS_acquisitionRate.get(1).asInt32(), asService_.settings.acquisitionrate, formaterror);

	asService_.settings.enabledsensors.resize(0);
	for (size_t i = 0; i < numenabledsensors; i++)
	{
		servAnalogSensor_t founditem;

		std::string s_enabled_id = b_SETTINGS_enabledSensors.get(i + 1).asString();
		//            const char *str = s_enabled_id.c_str();
		//            std::string cpp_str = str;

		// we must now search inside the whole vector<> asService_.properties.sensors if we find an id which matches s_enabled_id ....
		// if we dont, ... we issue a warning.
		// if we find, ... we do a pushback of it inside
		bool found = false;
		// i decide to use a brute force search ... for now
		for (size_t n = 0; n < asService_.properties.sensors.size(); n++)
		{
			servAnalogSensor_t item = asService_.properties.sensors.at(n);
			// if(item.id == cpp_str)
			if (item.id == s_enabled_id)
			{
				found = true;
				founditem = item;
				break;
			}
		}

		if (true == found)
		{
			asService_.settings.enabledsensors.push_back(founditem);
		}
	}

	// in here we issue an error if we dont have at least one enabled sensor

	if (0 == asService_.settings.enabledsensors.size())
	{
		yError() << "ServiceParserMultipleFt::check() could not find any item in SETTINGS.enabledSensors which matches what in PROPERTIES.SENSORS.id";
		return false;
	}
	return true;
}

bool ServiceParserMultipleFt::checkServiceType(const Bottle& service,  bool& formaterror)
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

bool ServiceParserMultipleFt::checkCanMonitor(const Bottle& service,bool& formaterror)
{
	Bottle b_CAN_MONITOR = Bottle(service.findGroup("CANMONITOR"));
	if (b_CAN_MONITOR.isNull())
	{
		yError() << "ServiceParserMultipleFt::check() cannot find PROPERTIES.CANMONITOR";
		return false;
	}

	if (false == b_CAN_MONITOR.check("checkrate"))
	{
		yError() << "ServiceParserMultipleFt::check() cannot find b_CAN_MONITOR.checkrate";
		return false;
	}
	if (false == b_CAN_MONITOR.check("reportmode"))
	{
		yError() << "ServiceParserMultipleFt::check() cannot find b_CAN_MONITOR.reportmode";
		return false;
	}
	if (false == b_CAN_MONITOR.check("periodicreportrate"))
	{
		yError() << "ServiceParserMultipleFt::check() cannot find b_CAN_MONITOR.periodicreportrate";
		return false;
	}

	int checkrate=b_CAN_MONITOR.find("checkrate").asInt32());
	std::string reportmode=b_CAN_MONITOR.find("reportmode").asString());
	if (stringToReport.find("reportmode") == stringToFind.end())
	{
		yError() << "ServiceParserMultipleFt::check() wrong b_CAN_MONITOR.reportmode";
		return false;
	}
	int periodicreportrate=b_CAN_MONITOR.find("periodicreportrate").asInt32());

	asService_.canMonitor = {checkrate, stringToReport.at(reportmode), periodicreportrate};

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

	if (!checkServiceType(service, type, formaterror))
		return false;

	if (!checkPropertyCanBoards(properties, formaterror))
		return false;

	if (!checkPropertySensors(properties, type, formaterror))
		return false;

	if (!checkSettings(service, type, formaterror))
		return false;

	if (!checkCanMonitor(service, type, formaterror))
		return false;

	return true;
}
