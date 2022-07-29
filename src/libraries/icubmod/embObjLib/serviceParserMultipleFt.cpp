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

bool ServiceParserMultipleFt::checkPropertyCanBoards(const Bottle &property)
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
        yError() << "ServiceParserMultipleFt::checkPropertyCanBoards() cannot find "
                    "PROPERTIES.CANBOARDS.type";
        return false;
    }

    Bottle propertyCanBoardProtocol = Bottle(propertyCanBoard.findGroup("PROTOCOL"));
    if (propertyCanBoardProtocol.isNull())
    {
        yError() << "ServiceParserMultipleFt::checkPropertyCanBoards() cannot find "
                    "PROPERTIES.CANBOARDS.PROTOCOL";
        return false;
    }

    Bottle propertyCanBoardProtocolMajor = Bottle(propertyCanBoardProtocol.findGroup("major"));
    if (propertyCanBoardProtocolMajor.isNull())
    {
        yError() << "ServiceParserMultipleFt::checkPropertyCanBoards() cannot find "
                    "PROPERTIES.CANBOARDS.PROTOCOL.major";
        return false;
    }

    Bottle propertyCanBoardProtocolMinor = Bottle(propertyCanBoardProtocol.findGroup("minor"));
    if (propertyCanBoardProtocolMinor.isNull())
    {
        yError() << "ServiceParserMultipleFt::checkPropertyCanBoards() cannot find "
                    "PROPERTIES.CANBOARDS.PROTOCOL.minor";
        return false;
    }

    Bottle propertyCanBoardFirmware = Bottle(propertyCanBoard.findGroup("FIRMWARE"));
    if (propertyCanBoardFirmware.isNull())
    {
        yError() << "ServiceParserMultipleFt::checkPropertyCanBoards() cannot find "
                    "PROPERTIES.CANBOARDS.FIRMWARE";
        return false;
    }

    Bottle propertyCanBoardFirmwareMajor = Bottle(propertyCanBoardFirmware.findGroup("major"));
    if (propertyCanBoardFirmwareMajor.isNull())
    {
        yError() << "ServiceParserMultipleFt::checkPropertyCanBoards() cannot find "
                    "PROPERTIES.CANBOARDS.FIRMWARE.major";
        return false;
    }

    Bottle propertyCanBoardFirmwareMinor = Bottle(propertyCanBoardFirmware.findGroup("minor"));
    if (propertyCanBoardFirmwareMinor.isNull())
    {
        yError() << "ServiceParserMultipleFt::checkPropertyCanBoards() cannot find "
                    "PROPERTIES.CANBOARDS.FIRMWARE.minor";
        return false;
    }

    Bottle propertyCanBoardFirmwareBuild = Bottle(propertyCanBoardFirmware.findGroup("build"));
    if (propertyCanBoardFirmwareBuild.isNull())
    {
        yError() << "ServiceParserMultipleFt::checkPropertyCanBoards() cannot find "
                    "PROPERTIES.CANBOARDS.FIRMWARE.build";
        return false;
    }

    // Check size
    if (propertyCanBoardType.size() != propertyCanBoardProtocolMajor.size())
    {
        yError() << "ServiceParserMultipleFt::checkPropertyCanBoards --> protocol major size";
        return false;
    }
    if (propertyCanBoardProtocolMajor.size() != propertyCanBoardProtocolMinor.size())
    {
        yError() << "ServiceParserMultipleFt::checkPropertyCanBoards --> protocol minor size";
        return false;
    }
    if (propertyCanBoardProtocolMinor.size() != propertyCanBoardFirmwareMajor.size())
    {
        yError() << "ServiceParserMultipleFt::checkPropertyCanBoards --> firmware major size";
        return false;
    }
    if (propertyCanBoardFirmwareMajor.size() != propertyCanBoardFirmwareMinor.size())
    {
        yError() << "ServiceParserMultipleFt::checkPropertyCanBoards --> firmware minor size";
        return false;
    }
    if (propertyCanBoardFirmwareMinor.size() != propertyCanBoardFirmwareBuild.size())
    {
        yError() << "ServiceParserMultipleFt::checkPropertyCanBoards --> firmware build size";
        return false;
    }

    size_t size = propertyCanBoardType.size();
    for (int index = 1; index < size; ++index)
    {
        std::string boardType = propertyCanBoardType.get(index).asString();

        eObrd_type_t currentBoard = checkBoardType(boardType);
        if (currentBoard == eobrd_unknown)
            return false;

        int protocolMajor = propertyCanBoardProtocolMajor.get(index).asInt32();
        int protocolMinor = propertyCanBoardProtocolMinor.get(index).asInt32();
        int firmwareMajor = propertyCanBoardFirmwareMajor.get(index).asInt32();
        int firmwareMinor = propertyCanBoardFirmwareMinor.get(index).asInt32();
        int firmwareBuild = propertyCanBoardFirmwareBuild.get(index).asInt32();

        for (auto &[key, current] : ftInfo_)
        {
            if (current.board != currentBoard)
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

bool ServiceParserMultipleFt::checkPropertySensors(const Bottle &property)
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
        eObrd_type_t currentBoard = checkBoardType(board);
        if (currentBoard == eobrd_unknown)
            return false;

        std::string location = propertySensorsLocation.get(index).asString();
        if (ftInfo_.find(id) == ftInfo_.end())
            continue;
        auto &currentFt = ftInfo_.at(id);

        try
        {
            currentFt.port = std::stoi(location.substr(3, 1));
        }
        catch (const std::exception &)
        {
            yError() << "ServiceParser::checkPropertySensors() invalid can port";
            return false;
        }
        try
        {
            currentFt.address = std::stoi(location.substr(5, location.size() - 5));
        }
        catch (const std::exception &)
        {
            yError() << "ServiceParser::checkPropertySensors() invalid can address";
            return false;
        }

        currentFt.board = currentBoard;
    }

    // Check missing SENSORS but enabled in SETTINGS
    for (const auto &[id, data] : ftInfo_)
    {
        bool found = false;
        for (int index = 0; index < propertySensorsId.size(); ++index)
        {
            if (id == propertySensorsId.get(index).asString())
            {
                found = true;
            }
        }
        if (!found)
        {
            yError() << "ServiceParser::checkPropertySensors() try to enable not existing sensor:" << id;
            return false;
        }
    }

    return true;
}

bool ServiceParserMultipleFt::checkSettings(const Bottle &service)
{
    Bottle settings = Bottle(service.findGroup("SETTINGS"));
    if (settings.isNull())
    {
        yError() << "ServiceParserMultipleFt::checkSettings() cannot find SETTINGS";
        return false;
    }

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
        yError() << "ServiceParserMultipleFt::checkSettings() for embObjMultipleFTsensors "
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
        uint32_t acquisitionTempRate = (uint32_t)settingsTemperaturePeriod.get(index).asInt32();
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

bool ServiceParserMultipleFt::checkServiceType(const Bottle &service)
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

bool ServiceParserMultipleFt::checkCanMonitor(const Bottle &service)
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

    int checkPeriod = canMonitor.find("checkPeriod").asInt32();
    if (checkPeriod > 254)
    {
        yError() << "ServiceParserMultipleFt::check() wrong canMonitor.checkPeriod "
                    "too big";
        return false;
    }
    int periodicreportrate = canMonitor.find("ratePeriod").asInt32();
    if (periodicreportrate > 65535)
    {
        yError() << "ServiceParserMultipleFt::check() wrong canMonitor.ratePeriod "
                    "too big";
        return false;
    }
    std::string reportmode = canMonitor.find("reportMode").asString();
    eObrd_canmonitor_reportmode_t reportmodeEobrd = (eObrd_canmonitor_reportmode_t)eoboards_string2reportmode(reportmode.c_str(), 1);

    if (reportmodeEobrd == eobrd_canmonitor_reportmode_unknown)
    {
        yError() << "ServiceParserMultipleFt::check() wrong canMonitor.reportmode";
        return false;
    }

    canMonitor_ = {(uint8_t)checkPeriod, (uint8_t)reportmodeEobrd, (uint16_t)periodicreportrate};

    return true;
}

bool ServiceParserMultipleFt::parse(const yarp::os::Searchable &config)
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

    if (!checkCanMonitor(service))
        return false;

    return true;
}

bool ServiceParserMultipleFt::toEomn(eOmn_serv_config_data_as_ft_t &out) const
{
    out.canmonitorconfig = canMonitor_;

    EOarray *ar = eo_array_New(eOas_ft_sensors_maxnumber, sizeof(eOas_ft_sensordescriptor_t), (void *)(&(out.arrayofsensors)));

    for (const auto &[key, value] : ftInfo_)
    {
        eOas_ft_sensordescriptor_t item;
        if (!value.toEomn(item))
        {
            yError() << "ServiceParserMultipleFt::toEomn() wrong data for sensor";
            return false;
        }
        eo_array_PushBack(ar, &item);
    }
    return true;
}

std::map<std::string, FtInfo> &ServiceParserMultipleFt::getFtInfo()
{
    return ftInfo_;
}

eObrd_type_t ServiceParserMultipleFt::checkBoardType(const std::string &boardType)
{
    eObrd_type_t type = eoboards_string2type2(boardType.c_str(), eobool_true);
    if (!eoas_ft_isboardvalid(eoboards_type2cantype(type)))
    {
        type = eoboards_string2type2(boardType.c_str(), eobool_false);
        if (!eoas_ft_isboardvalid(eoboards_type2cantype(type)))
        {
            yError() << "checkBoardType --> unsupported board type:" << boardType;
            return type;
        }
    }
    return type;
}
