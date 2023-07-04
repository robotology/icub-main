/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 * Author: Luca Tricerri
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include <embObjBattery.h>
#include <ethManager.h>

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

#include <iostream>
#include <string>
#include <sstream>
#include <cmath>

#include "EOnv_hid.h"
#include "EoProtocol.h"
#include "EoProtocolAS.h"
#include "EoProtocolMN.h"
#include "EoAnalogSensors.h"

#include "embot_core_binary.h"

#ifdef WIN32
#pragma warning(once : 4355)
#endif

using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;

void CanBatteryData::decode(eOas_battery_timedvalue_t *data, double timestamp)
{
    temperature_ = data->temperature / 10;  // in steps of 0.1 celsius degree (pos and neg).
    voltage_ = std::trunc(10 * data->voltage) / 10;
    current_ = data->current;
    charge_ = data->charge;
    status_ = data->status;
    timeStamp_ = timestamp;
}

embObjBattery::embObjBattery()
{
    yInfo() << "CanBatterySensors has been created";
    device_ = std::make_shared<yarp::dev::embObjDevPrivData>("embObjBattery");
}

embObjBattery::embObjBattery(std::shared_ptr<yarp::dev::embObjDevPrivData> device) : device_(device)
{
}

embObjBattery::~embObjBattery()
{
    close();
}

bool embObjBattery::initialised()
{
    return device_->isOpen();
}

bool embObjBattery::open(yarp::os::Searchable &config)
{
    yInfo() << "embObjBattery::open(): preparing ETH resource";
    if (!device_->prerareEthService(config, this))
        return false;

    yInfo() << device_->getBoardInfo() << " embObjBattery::open(): browsing xml files which describe the service";
    ServiceParserCanBattery parser;
    if (!parser.parse(config))
    {
        yError() << device_->getBoardInfo() << "open() fails to parse xml... cannot continue ";
        return false;
    }

    yInfo() << device_->getBoardInfo() << " embObjBattery::open(): verify the presence of the board and if its protocol version is correct";
    if (!device_->res->verifyEPprotocol(eoprot_endpoint_analogsensors))
    {
        yError() << device_->getBoardInfo() << " open() fails to verifyEPprotocol... cannot continue ";
        cleanup();
        return false;
    }

    yInfo() << device_->getBoardInfo() << " embObjBattery::open(): verify and activate the FT service";
    eOmn_serv_parameter_t canBatteryData;
    canBatteryData.configuration.type = eomn_serv_AS_battery;
    canBatteryData.configuration.diagnosticsmode = eomn_serv_diagn_mode_NONE;
    canBatteryData.configuration.diagnosticsparam = 0;
    parser.toEomn(canBatteryData.configuration.data.as.battery);
    if (!device_->res->serviceVerifyActivate(eomn_serv_category_battery, &canBatteryData, 5.0))
    {
        yError() << device_->getBoardInfo() << " open() fails to serviceVerifyActivate... cannot continue ";
        cleanup();
        return false;
    }

    yInfo() << device_->getBoardInfo() << " embObjBattery::open(): configure the FT service";
    if (false == sendConfig2boards(parser, device_->res))
    {
        yError() << device_->getBoardInfo() << " open() fails to sendConfig2boards... cannot continue";
        cleanup();
        return false;
    }

    yInfo() << device_->getBoardInfo() << " embObjBattery::open(): impose the network variable which the ETH bord must stream up";
    if (false == initRegulars(parser, device_->res))
    {
        yError() << device_->getBoardInfo() << " open() fails to initRegulars... cannot continue";
        cleanup();
        return false;
    }

    yInfo() << device_->getBoardInfo() << " embObjBattery::open(): start the FT service";
    if (!device_->res->serviceStart(eomn_serv_category_battery))
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

    yInfo() << device_->getBoardInfo() << " embObjBattery::open(): start streaming of FT data";
    if (!sendStart2boards(parser, device_->res))
    {
        yError() << device_->getBoardInfo() << " open() fails to sendStart2boards... cannot continue";
        cleanup();
        return false;
    }

    device_->setOpen(true);
    return true;
}

bool embObjBattery::sendConfig2boards(ServiceParserCanBattery &parser, eth::AbstractEthResource *deviceRes)
{
    auto &canBattery = parser.getBatteryInfo();

    eOprotID32_t id32 = eo_prot_ID32dummy;
    eOas_battery_config_t cfg{0, 0};
    cfg.period = canBattery.acquisitionRate;
    id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_battery, 0, eoprot_tag_as_battery_config);

    if (false == deviceRes->setcheckRemoteValue(id32, &cfg, 10, 0.010, 0.050))
    {
        yError() << device_->getBoardInfo() << " sendConfig2boards() while try to configure fperiod=" << cfg.period;
        return false;
    }

    if (device_->isVerbose())
    {
        yDebug() << device_->getBoardInfo() << " sendConfig2boards() correctly configured boards with period=" << cfg.period;
    }
    return true;
}

bool embObjBattery::sendStart2boards(ServiceParserCanBattery &parser, eth::AbstractEthResource *deviceRes)
{
    eOprotID32_t id32 = eo_prot_ID32dummy;

    uint8_t enable = 1;

    const auto &batteryInfos = parser.getBatteryInfo();

    id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_battery, 0, eoprot_tag_as_battery_cmmnds_enable);

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

bool embObjBattery::initRegulars(ServiceParserCanBattery &parser, eth::AbstractEthResource *deviceRes)
{
    // configure regular rops

    vector<eOprotID32_t> id32v;
    eOprotID32_t id32 = eo_prot_ID32dummy;

    const auto &batteryInfos = parser.getBatteryInfo();
    id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_battery, 0, eoprot_tag_as_battery_status_timedvalue);
    id32v.push_back(id32);

    if (false == deviceRes->serviceSetRegulars(eomn_serv_category_battery, id32v))
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

eth::iethresType_t embObjBattery::type()
{
    return eth::iethres_analogbattery;
}

bool embObjBattery::update(eOprotID32_t id32, double timestamp, void *rxdata)
{
    if (!device_->isOpen())
        return false;
    eOprotIndex_t eoprotIndex = eoprot_ID2index(id32);
    if (eoprotIndex > 1)
    {
        yError() << device_->getBoardInfo() << " update() index too big";
        return false;
    }
    eOprotEntity_t entity = eoprot_ID2entity(id32);
    if (entity != eoprot_entity_as_battery)
    {
        yError() << device_->getBoardInfo() << " update() wrong entity";
        return false;
    }

    eOprotTag_t tag = eoprot_ID2tag(id32);
    if (tag != eoprot_tag_as_battery_status_timedvalue)
    {
        yError() << device_->getBoardInfo() << " update() wrong tag";
        return false;
    }

    eOas_battery_timedvalue_t *data = (eOas_battery_timedvalue_t *)rxdata;
    if (!checkUpdateTimeout(id32, data->age))
    {
        return false;
    }

    if (!isPastFirstPrint && (data->age == 0))
    {
        yDebug("CAN DATA NOT YET AVAILABLE");
        isPastFirstPrint = true;
    }
    else if (!isCanDataAvailable && (data->age != 0))
    {
        if (updateStatusStringStream(data->status, canBatteryData_.prevStatus_, true))
        {
            yDebug() << "First Status are:\n" << statusStreamBMS.str() << statusStreamBAT.str() << "\n";
        }
        canBatteryData_.prevStatus_ = data->status;
        isCanDataAvailable = true;
    }
    else if (data->status != canBatteryData_.prevStatus_)
    {
        if (updateStatusStringStream(data->status, canBatteryData_.prevStatus_, false))
        {
            yDebug() << "Status changed to:\n" << statusStreamBMS.str() << statusStreamBAT.str() << "\n";
        }
        canBatteryData_.prevStatus_ = data->status;
    }

    std::unique_lock<std::shared_mutex> lck(mutex_);
    canBatteryData_.decode(data, calculateBoardTime(data->age));

    return true;
}

bool embObjBattery::close()
{
    cleanup();
    return true;
}

void embObjBattery::cleanup(void)
{
    device_->cleanup(static_cast<eth::IethResource *>(this));
}

bool embObjBattery::checkUpdateTimeout(eOprotID32_t id32, eOabstime_t current)
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

bool embObjBattery::updateStatusStringStream(const uint32_t &currStatus, const uint32_t &prevStatus, bool isFirstLoop)
{
    // Initialize the first time the static map
    static const std::array<std::pair<eOas_battery_alarm_status_t, std::string>, eoas_battery_alarm_status_numberof> s_boards_map_of_battery_alarm_status = 
    {
        {{eoas_bms_general_alarm_lowvoltage, "eoas_bms_general_alarm_lowvoltage"},
        {eoas_bms_general_alarm_highvoltage, "eoas_bms_general_alarm_highvoltage"},
        {eoas_bms_general_alarm_overcurrent_discharge, "eoas_bms_general_alarm_overcurrent_discharge"},
        {eoas_bms_general_alarm_overcurrent_charge, "eoas_bms_general_alarm_overcurrent_charge"},
        {eoas_bms_general_alarm_lowSOC, "eoas_bms_general_alarm_lowSOC"},
        {eoas_bms_general_alarm_lowtemperature, "eoas_bms_general_alarm_lowtemperature"},
        {eoas_bms_general_alarm_hightemperature, "eoas_bms_general_alarm_hightemperature"},
        {eoas_bat_status_hsm_mosfet_broken, "eoas_bat_status_hsm_mosfet_broken"},
        {eoas_bat_status_hsm_mosfet_normal, "eoas_bat_status_hsm_mosfet_normal"},
        {eoas_bat_status_hsm_overcurrent_overvoltage, "eoas_bat_status_hsm_overcurrent_overvoltage"},
        {eoas_bat_status_hsm_normal, "eoas_bat_status_hsm_normal"},
        {eoas_bat_status_hsm_voltage_power_good, "eoas_bat_status_hsm_voltage_power_good"},
        {eoas_bat_status_hsm_voltage_not_guaranteed, "eoas_bat_status_hsm_voltage_not_guaranteed"},
        {eoas_bat_status_hsm_status_on, "eoas_bat_status_hsm_status_on"},
        {eoas_bat_status_hsm_status_off, "eoas_bat_status_hsm_status_off"},
        {eoas_bat_status_motor_regulator_overcurrent, "eoas_bat_status_motor_regulator_overcurrent"},
        {eoas_bat_status_motor_regulator_normal, "eoas_bat_status_motor_regulator_normal"},
        {eoas_bat_status_motor_on, "eoas_bat_status_motor_on"},
        {eoas_bat_status_motor_off, "eoas_bat_status_motor_off"},
        {eoas_bat_status_board_regulator_overcurrent, "eoas_bat_status_board_regulator_overcurrent"},
        {eoas_bat_status_board_regulator_normal, "eoas_bat_status_board_regulator_normal"},
        {eoas_bat_status_board_on, "eoas_bat_status_board_on"},
        {eoas_bat_status_board_off, "eoas_bat_status_board_off"},
        {eoas_bat_status_btn_2_start_up_phase, "eoas_bat_status_btn_2_start_up_phase"},
        {eoas_bat_status_btn_2_stable_op, "eoas_bat_status_btn_2_stable_op"},
        {eoas_bat_status_btn_1_start_up_phase, "eoas_bat_status_btn_1_start_up_phase"},
        {eoas_bat_status_btn_1_stable_op, "eoas_bat_status_btn_1_stable_op"}}
    };

    // Clear buffer for BAT and BMS
    statusStreamBMS.str("");
    statusStreamBAT.str("");

    bool isBmsSignatureBit = embot::core::binary::bit::check(currStatus, 0);

    if(isBmsSignatureBit)
    {
        // And add header string
        statusStreamBMS << "STATUS_BMS:" << "\n";
        if (!(embot::core::binary::mask::check(currStatus, static_cast<uint32_t>(0x0000ffff))))
        {
            statusStreamBMS << "\tNo Faults. All Alarms Bit Down\n";
        }
        else
        {
            for (uint8_t i = 1; i < eoas_bms_alarm_numberof; i++)
            {
                std::string tmpString = (embot::core::binary::bit::check(currStatus, i)) ?  (s_boards_map_of_battery_alarm_status.at(i)).second : "";

                if (tmpString != "")
                {
                    statusStreamBMS << "\t" << tmpString << "\n";
                }
            }
        }
    }
    else
    {
        uint8_t map_pos = 0;
        statusStreamBAT << "STATUS_BAT:" << "\n";

        for (const auto& [k,v] : s_boards_map_of_battery_alarm_status)
        {
            std::string tmpString = "";
            uint8_t bit_pos = (uint8_t)k;

            if (bit_pos > eoas_bms_alarm_numberof && bit_pos < eoas_battery_alarm_status_numberof)
            {
                if (isFirstLoop)
                {
                    tmpString = embot::core::binary::bit::check(currStatus, bit_pos) ? v : (s_boards_map_of_battery_alarm_status.at(map_pos+1)).second;
                }
                else
                {
                    if (embot::core::binary::bit::check(currStatus, bit_pos) != embot::core::binary::bit::check(prevStatus, bit_pos))
                    {
                        tmpString = embot::core::binary::bit::check(currStatus, bit_pos) ? v : (s_boards_map_of_battery_alarm_status.at(map_pos+1)).second;
                    }
                }
                if (tmpString != "")
                {
                    statusStreamBAT << "\t" << tmpString << "\n";
                }
            }
            ++map_pos;
        }
        
    }

    return true;
}

double embObjBattery::calculateBoardTime(eOabstime_t current)
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

bool embObjBattery::getBatteryVoltage(double &voltage)
{
    voltage = canBatteryData_.voltage_;
    return true;
}

bool embObjBattery::getBatteryCurrent(double &current)
{
    current = (canBatteryData_.current_ != 0) ? canBatteryData_.current_ : NAN;
    return true;
}

bool embObjBattery::getBatteryCharge(double &charge)
{
    charge = canBatteryData_.charge_;
    return true;
}

bool embObjBattery::getBatteryStatus(Battery_status &status)
{
    status = static_cast<Battery_status>(canBatteryData_.status_);
    return true;
}

bool embObjBattery::getBatteryTemperature(double &temperature)
{
    temperature = (canBatteryData_.temperature_ != 0) ? canBatteryData_.temperature_ : NAN;
    return true;
}

bool embObjBattery::getBatteryInfo(std::string &battery_info)
{
    std::stringstream ss;
    ss << "{\"temperature\":" << canBatteryData_.temperature_ << ",\"voltage\":" << canBatteryData_.voltage_ << ",\"charge\":" << canBatteryData_.charge_ << ",\"status\":" << canBatteryData_.status_
       << ",\"ts\":" << canBatteryData_.timeStamp_ << "}" << std::endl;

    battery_info = ss.str();
    return true;
}

bool CanBatteryData::operator==(const CanBatteryData &other) const
{
    if (temperature_ != other.temperature_)
        return false;
    if ((int)(voltage_ * 10) != (int)(other.voltage_ * 10))  // Only one digit after dot
        return false;
    if (current_ != other.current_)
        return false;
    if (charge_ != other.charge_)
        return false;
    if (status_ != other.status_)
        return false;
    if (timeStamp_ != other.timeStamp_)
        return false;
    if (sensorName_ != other.sensorName_)
        return false;
    return true;
}

bool CanBatteryData::operator!=(const CanBatteryData &other) const
{
    return !operator==(other);
}
