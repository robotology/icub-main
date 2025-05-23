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
#include <string_view>
#include <cmath>
#include <array>

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
    current_ = std::trunc(10 * data->current) / 10;
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

    canBatteryData_.sensorName_ = eoboards_type2string(parser.getBatteryInfo().board);
    canBatteryData_.sensorType_ = parser.getBatteryInfo().board;
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
        yDebug() << "First Status are:\n" << updateStatusStringStream(data->status, canBatteryData_.prevStatus_, true);
        
        canBatteryData_.prevStatus_ = data->status;
        isCanDataAvailable = true;
    }
    else if (data->status != canBatteryData_.prevStatus_)
    {
        yDebug() << "Status changed to:\n" << updateStatusStringStream(data->status, canBatteryData_.prevStatus_, false);
        
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

std::string embObjBattery::updateStatusStringStream(const uint16_t &currStatus, const uint16_t &prevStatus, bool isFirstLoop)
{
    // Initialize the first time the static map
    static const std::array<std::pair<eOas_battery_alarm_status_t, std::string_view>, eoas_battery_alarm_status_numberof> s_boards_map_of_battery_alarm_status =
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

    // Clear and reserve space for buffer for BAT and BMS
    std::string statusstring = {};
    statusstring.reserve(512);

    if (canBatteryData_.sensorType_ == eobrd_cantype_bms)
    {
        for (uint8_t i = 0; i < eoas_bms_alarm_numberof; i++)
        {
            if((embot::core::binary::bit::check(currStatus, i)))
            {
                statusstring.append("\t");
                statusstring.append(s_boards_map_of_battery_alarm_status.at(i).second);
                statusstring.append("\n");
            }
        }
    }
    else if(canBatteryData_.sensorType_ == eobrd_cantype_bat)
    {
        uint8_t bit_pos = 0;
        for (uint8_t i = eoas_bms_alarm_numberof; i < eoas_battery_alarm_status_numberof; i = i+2)
        {
            if ((embot::core::binary::bit::check(currStatus, bit_pos) != embot::core::binary::bit::check(prevStatus, bit_pos)) || isFirstLoop)
            {
                statusstring.append("\t");
                if((embot::core::binary::bit::check(currStatus, bit_pos)))
                {
                    statusstring.append(s_boards_map_of_battery_alarm_status.at(i).second);
                } 
                else
                {
                    statusstring.append(s_boards_map_of_battery_alarm_status.at(i+1).second);
                }
                statusstring.append("\n");
            }
            
            ++bit_pos;
        }
    }

    if(statusstring.empty())
    {
        statusstring.append("\tNo Faults Detected. All Alarms Bit Down\n");
    }

    
    return statusstring;
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

YARP_DEV_RETURN_VALUE_TYPE_CH312 embObjBattery::getBatteryVoltage(double &voltage)
{
    voltage = canBatteryData_.voltage_;
    return YARP_DEV_RETURN_VALUE_OK_CH312;
}

YARP_DEV_RETURN_VALUE_TYPE_CH312 embObjBattery::getBatteryCurrent(double &current)
{
    current = canBatteryData_.current_;
    return YARP_DEV_RETURN_VALUE_OK_CH312;
}

YARP_DEV_RETURN_VALUE_TYPE_CH312 embObjBattery::getBatteryCharge(double &charge)
{
    charge = (canBatteryData_.charge_ != 0.0) ? canBatteryData_.charge_ : NAN;
    return YARP_DEV_RETURN_VALUE_OK_CH312;
}

YARP_DEV_RETURN_VALUE_TYPE_CH312 embObjBattery::getBatteryStatus(Battery_status &status)
{
    status = static_cast<Battery_status>(canBatteryData_.status_);
    return YARP_DEV_RETURN_VALUE_OK_CH312;
}

YARP_DEV_RETURN_VALUE_TYPE_CH312 embObjBattery::getBatteryTemperature(double &temperature)
{
    temperature = (canBatteryData_.temperature_ != 0) ? canBatteryData_.temperature_ : NAN;
    return YARP_DEV_RETURN_VALUE_OK_CH312;
}

YARP_DEV_RETURN_VALUE_TYPE_CH312 embObjBattery::getBatteryInfo(std::string &battery_info)
{
    std::stringstream ss;
    ss << "{\"temperature\":" << canBatteryData_.temperature_ << ",\"voltage\":" << canBatteryData_.voltage_ << ",\"current\":" << canBatteryData_.current_ << ",\"charge\":" << canBatteryData_.charge_ << ",\"status\":" << canBatteryData_.status_
       << ",\"ts\":" << canBatteryData_.timeStamp_ << "}" << std::endl;

    battery_info = ss.str();
    return YARP_DEV_RETURN_VALUE_OK_CH312;
}

bool CanBatteryData::operator==(const CanBatteryData &other) const
{
    if (temperature_ != other.temperature_)
        return false;
    if ((int)(voltage_ * 10) != (int)(other.voltage_ * 10))  // Only one digit after dot
        return false;
    if ((int)(current_ * 10) != (int)(other.current_ * 10))
        return false;
    if (charge_ != other.charge_)
        return false;
    if (status_ != other.status_)
        return false;
    if (timeStamp_ != other.timeStamp_)
        return false;
    if (sensorName_ != other.sensorName_)
        return false;
    if (sensorType_ != other.sensorType_)
        return false;
    
    return true;
}

bool CanBatteryData::operator!=(const CanBatteryData &other) const
{
    return !operator==(other);
}
