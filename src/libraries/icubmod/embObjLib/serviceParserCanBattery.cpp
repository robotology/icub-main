/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 * Author: Luca Tricerri
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */
#include "serviceParserCanBattery.h"

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

ServiceParserCanBattery::ServiceParserCanBattery()
{
}

bool ServiceParserCanBattery::checkPropertyCanBoards(const Bottle &property)
{
    Bottle propertyCanBoard = Bottle(property.findGroup("CANBOARDS"));
    if (propertyCanBoard.isNull())
    {
        yError() << "ServiceParserCanBattery::check() cannot find PROPERTIES.CANBOARDS";
        return false;
    }

    Bottle propertyCanBoardType = propertyCanBoard.findGroup("type");
    if (propertyCanBoardType.isNull())
    {
        yError() << "ServiceParserCanBattery::check() cannot find "
                    "PROPERTIES.CANBOARDS.type";
        return false;
    }

    Bottle propertyCanBoardProtocol = Bottle(propertyCanBoard.findGroup("PROTOCOL"));
    if (propertyCanBoardProtocol.isNull())
    {
        yError() << "ServiceParserCanBattery::check() cannot find "
                    "PROPERTIES.CANBOARDS.PROTOCOL";
        return false;
    }

    Bottle propertyCanBoardProtocolMajor = Bottle(propertyCanBoardProtocol.findGroup("major"));
    if (propertyCanBoardProtocolMajor.isNull())
    {
        yError() << "ServiceParserCanBattery::check() cannot find "
                    "PROPERTIES.CANBOARDS.PROTOCOL.major";
        return false;
    }

    Bottle propertyCanBoardProtocolMinor = Bottle(propertyCanBoardProtocol.findGroup("minor"));
    if (propertyCanBoardProtocolMinor.isNull())
    {
        yError() << "ServiceParserCanBattery::check() cannot find "
                    "PROPERTIES.CANBOARDS.PROTOCOL.minor";
        return false;
    }

    Bottle propertyCanBoardFirmware = Bottle(propertyCanBoard.findGroup("FIRMWARE"));
    if (propertyCanBoardFirmware.isNull())
    {
        yError() << "ServiceParserCanBattery::check() cannot find "
                    "PROPERTIES.CANBOARDS.FIRMWARE";
        return false;
    }

    Bottle propertyCanBoardFirmwareMajor = Bottle(propertyCanBoardFirmware.findGroup("major"));
    if (propertyCanBoardFirmwareMajor.isNull())
    {
        yError() << "ServiceParserCanBattery::check() cannot find "
                    "PROPERTIES.CANBOARDS.FIRMWARE.major";
        return false;
    }

    Bottle propertyCanBoardFirmwareMinor = Bottle(propertyCanBoardFirmware.findGroup("minor"));
    if (propertyCanBoardFirmwareMinor.isNull())
    {
        yError() << "ServiceParserCanBattery::check() cannot find "
                    "PROPERTIES.CANBOARDS.FIRMWARE.minor";
        return false;
    }

    Bottle propertyCanBoardFirmwareBuild = Bottle(propertyCanBoardFirmware.findGroup("build"));
    if (propertyCanBoardFirmwareBuild.isNull())
    {
        yError() << "ServiceParserCanBattery::check() cannot find "
                    "PROPERTIES.CANBOARDS.FIRMWARE.build";
        return false;
    }

    // Check size
    if (propertyCanBoardType.size() != 2)
    {
        yError() << "ServiceParserCanBattery::checkPropertyCanBoards --> protocol type size";
        return false;
    }
    if (propertyCanBoardProtocolMajor.size() != 2)
    {
        yError() << "ServiceParserCanBattery::checkPropertyCanBoards --> protocol major size";
        return false;
    }
    if (propertyCanBoardProtocolMinor.size() != 2)
    {
        yError() << "ServiceParserCanBattery::checkPropertyCanBoards --> firmware minor size";
        return false;
    }
    if (propertyCanBoardFirmwareMajor.size() != 2)
    {
        yError() << "ServiceParserCanBattery::checkPropertyCanBoards --> firmware major size";
        return false;
    }
    if (propertyCanBoardFirmwareMinor.size() != 2)
    {
        yError() << "ServiceParserCanBattery::checkPropertyCanBoards --> firmware minor size";
        return false;
    }
    if (propertyCanBoardFirmwareBuild.size() != 2)
    {
        yError() << "ServiceParserCanBattery::checkPropertyCanBoards --> firmware build size";
        return false;
    }

    std::string boardType = propertyCanBoardType.get(1).asString();

    eObrd_type_t currentBoard = checkBoardType(boardType);
    if (currentBoard == eobrd_unknown)
        return false;

    batteryInfo_.majorProtocol = propertyCanBoardProtocolMajor.get(1).asInt32();
    batteryInfo_.minorProtocol = propertyCanBoardProtocolMinor.get(1).asInt32();
    batteryInfo_.majorFirmware = propertyCanBoardFirmwareMajor.get(1).asInt32();
    batteryInfo_.minorFirmware = propertyCanBoardFirmwareMinor.get(1).asInt32();
    batteryInfo_.buildFirmware = propertyCanBoardFirmwareBuild.get(1).asInt32();

    return true;
}

bool ServiceParserCanBattery::checkPropertySensors(const Bottle &property)
{
    Bottle propertySensors = Bottle(property.findGroup("SENSORS"));
    if (propertySensors.isNull())
    {
        yError() << "ServiceParserCanBattery::checkPropertySensors() cannot find "
                    "PROPERTIES.SENSORS";
        return false;
    }

    Bottle propertySensorsId = Bottle(propertySensors.findGroup("id"));
    if (propertySensorsId.isNull())
    {
        yError() << "ServiceParserCanBattery::checkPropertySensors() cannot find "
                    "PROPERTIES.SENSORS.id";
        return false;
    }

    Bottle propertySensorsBoard = Bottle(propertySensors.findGroup("board"));
    if (propertySensorsBoard.isNull())
    {
        yError() << "ServiceParserCanBattery::checkPropertySensors() cannot find "
                    "PROPERTIES.SENSORS.type";
        return false;
    }

    Bottle propertySensorsLocation = Bottle(propertySensors.findGroup("location"));
    if (propertySensorsLocation.isNull())
    {
        yError() << "ServiceParserCanBattery::checkPropertySensors() cannot find "
                    "PROPERTIES.SENSORS.location";
        return false;
    }

    // CHECK size
    if (propertySensorsId.size() != 2)
    {
        yError() << "ServiceParserCanBattery::checkPropertySensors --> id size";
        return false;
    }
    if (propertySensorsBoard.size() != 2)
    {
        yError() << "ServiceParserCanBattery::checkPropertySensors --> board size";
        return false;
    }
    if (propertySensorsLocation.size() != 2)
    {
        yError() << "ServiceParserCanBattery::checkPropertySensors --> location size";
        return false;
    }

    std::string id = propertySensorsId.get(1).asString();
    std::string board = propertySensorsBoard.get(1).asString();
    eObrd_type_t currentBoard = checkBoardType(board);
    if (currentBoard == eobrd_unknown)
        return false;
    std::string location = propertySensorsLocation.get(1).asString();

    try
    {
        batteryInfo_.port = std::stoi(location.substr(3, 1));
    }
    catch (const std::exception &)
    {
        yError() << "ServiceParser::checkPropertySensors() invalid can port";
        return false;
    }
    try
    {
        batteryInfo_.address = std::stoi(location.substr(5, location.size() - 5));
    }
    catch (const std::exception &)
    {
        yError() << "ServiceParser::checkPropertySensors() invalid can address";
        return false;
    }

    batteryInfo_.board = currentBoard;

    return true;
}

bool ServiceParserCanBattery::checkSettings(const Bottle &service)
{
    Bottle settings = Bottle(service.findGroup("SETTINGS"));
    if (settings.isNull())
    {
        yError() << "ServiceParserCanBattery::checkSettings() cannot find SETTINGS";
        return false;
    }

    Bottle settingsBatteryPeriod = Bottle(settings.findGroup("acquisitionRate"));
    if (settingsBatteryPeriod.isNull())
    {
        yError() << "ServiceParserCanBattery::checkSettings() cannot find "
                    "SETTINGS.acquisitionRate";
        return false;
    }

    Bottle settingsEnabledSensors = Bottle(settings.findGroup("enabledSensors"));
    if (settingsEnabledSensors.isNull())
    {
        yError() << "ServiceParserCanBattery::checkSettings() cannot find "
                    "SETTINGS.enabledSensors";
        return false;
    }

    // Check size
    int tmp = settingsBatteryPeriod.size();
    std::string s = settingsBatteryPeriod.toString();
    if (settingsEnabledSensors.size() != 2)
    {
        yError() << "ServiceParserCanBattery::checkSettings --> enabledSensors";
        return false;
    }
    if (settingsBatteryPeriod.size() != 2)
    {
        yError() << "ServiceParserCanBattery::checkSettings --> acquisitionRate"
                    "temperaturePeriod size";
        return false;
    }

    uint8_t acquisitionRate = (uint8_t)settingsBatteryPeriod.get(1).asInt8();
    batteryInfo_.acquisitionRate = acquisitionRate;
    return true;
}

bool ServiceParserCanBattery::checkServiceType(const Bottle &service)
{
    if (false == service.check("type"))
    {
        yError() << "ServiceParserCanBattery::check() cannot find SERVICE.type";
        return false;
    }
    std::string serviceType = service.find("type").asString();
    eOmn_serv_type_t serviceTypeEomn = eomn_string2servicetype(serviceType.c_str());

    if (eomn_serv_AS_battery != serviceTypeEomn)
    {
        yError() << "ServiceParserCanBattery::check() has found wrong SERVICE.type = " << serviceType << "it must be eomn_serv_AS_battery";
        return false;
    }
    return true;
}

bool ServiceParserCanBattery::parse(const yarp::os::Searchable &config)
{
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

    if (!checkServiceType(service))
        return false;

    if (!checkSettings(service))
        return false;

    if (!checkPropertySensors(properties))
        return false;

    if (!checkPropertyCanBoards(properties))
        return false;

    return true;
}

bool ServiceParserCanBattery::toEomn(eOmn_serv_config_data_as_battery_t &out) const
{
    EOarray *ar = eo_array_New(eOas_battery_sensors_maxnumber, sizeof(eOas_battery_sensordescriptor_t), (void *)(&(out.arrayofsensors)));

    eOas_battery_sensordescriptor_t item;
    if (!batteryInfo_.toEomn(item))
    {
        yError() << "ServiceParserCanBattery::toEomn() wrong data for sensor";
        return false;
    }
    eo_array_PushBack(ar, &item);

    return true;
}

BatteryInfo &ServiceParserCanBattery::getBatteryInfo()
{
    return batteryInfo_;
}

eObrd_type_t ServiceParserCanBattery::checkBoardType(const std::string &boardType)
{
    eObrd_type_t type = eoboards_string2type2(boardType.c_str(), eobool_true);
    if (!eoas_bms_isboardvalid(eoboards_type2cantype(type)))
    {
        type = eoboards_string2type2(boardType.c_str(), eobool_false);
        if (!eoas_bms_isboardvalid(eoboards_type2cantype(type)))
        {
            yError() << "checkBoardType --> unsupported board type:" << boardType;
            return type;
        }
    }
    return type;
}
