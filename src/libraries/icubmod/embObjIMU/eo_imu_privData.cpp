/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 * Author: Valentina Gaggero
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */
#include "eo_imu_privData.h"
#include <stdexcept>

#include "EoProtocolAS.h"
#include "EOnv_hid.h"
#include "EOconstarray.h"

using namespace yarp::dev;



PositionMaps::PositionMaps()
{
    memset(positionmap, 0xff, sizeof(positionmap));
}

PositionMaps::~PositionMaps(void) {;}

bool PositionMaps::init(servConfigImu_t &servCfg)
{
    std::uint8_t numberof[eoas_sensors_numberof];
    memset(numberof, 0, sizeof(numberof));

    // now we list the service config to fill the map withe proper indices.

    const eOas_inertial3_arrayof_descriptors_t* tmp = &servCfg.ethservice.configuration.data.as.inertial3.arrayofdescriptor;
    EOconstarray* array = eo_constarray_Load(reinterpret_cast<const EOarray*>(tmp));
    uint8_t size = eo_constarray_Size(array);
    for(uint8_t i=0; i<size; i++)
    {
        eOas_inertial3_descriptor_t *des = (eOas_inertial3_descriptor_t*)eo_constarray_At(array, i);
        if(nullptr != des)
        {
            // use des.
            if(des->typeofsensor < eoas_sensors_numberof)
            {
                if(des->on.any.place == eobrd_place_can)
                {
                    positionmap[des->typeofsensor][des->on.can.port][des->on.can.addr] = numberof[des->typeofsensor];
                    numberof[des->typeofsensor]++;
                }
                else if(des->on.any.place == eobrd_place_eth)
                {
                    // must manage the case of gyro on ems
                }
            }
        }
        else
        {
            return false;
        }
    }
    return true;
}

bool PositionMaps::getIndex(const eOas_inertial3_data_t* data, uint8_t& index, eOas_sensor_t& type)
{
    uint8_t canbus, canaddress;
    if(!getCanAddress(data, canbus, canaddress))
        return false;

    if(data->typeofsensor >= eoas_sensors_numberof)
    {   // it is not a valid index
        return false;
    }

    index = positionmap[data->typeofsensor][canbus][canaddress];

    type = static_cast<eOas_sensor_t>(data->typeofsensor);
    return (0xff == index) ? false : true;
}

bool PositionMaps::getIndex(eOas_sensor_t type, uint8_t canbus, uint8_t canaddress, uint8_t& index)
{
    if(canbus >= eOcanports_number)
    {
        return false;
    }

    if(canaddress > 0x0f)
        return false;

    index = positionmap[type][canbus][canaddress];
    return (0xff == index) ? false : true;
}


bool PositionMaps::getCanAddress(const eOas_inertial3_data_t *data, uint8_t &canbus, uint8_t &canaddress)
{
    if(nullptr == data)
        return false;

    canbus = data->id >> 4;

    if(canbus >= eOcanports_number)
    {
        return false;
    }

    canaddress = data->id & 0x0f;

    return true;
}


SensorsData::SensorsData()
{
    errorstring = "embObjIMU";

    mysens.resize(eoas_sensors_numberof);
    for(int t=0; t<eoas_sensors_numberof; t++)
    { mysens[t].resize(0); }
}


void SensorsData::init(servConfigImu_t &servCfg, string error_string)
{
    errorstring = error_string;

    const eOas_inertial3_arrayof_descriptors_t* tmp = &servCfg.ethservice.configuration.data.as.inertial3.arrayofdescriptor;
    EOconstarray* array = eo_constarray_Load(reinterpret_cast<const EOarray*>(tmp));
    uint8_t size = eo_constarray_Size(array);
    for(uint8_t i=0; i<size; i++)
    {
        eOas_inertial3_descriptor_t *des = (eOas_inertial3_descriptor_t*)eo_constarray_At(array, i);
        if(nullptr != des)
        {
            // use des.
            if(des->typeofsensor < eoas_sensors_numberof)
            {
                sensorInfo_t newSensor;
                newSensor.name = servCfg.id[i];
                newSensor.framename = newSensor.name;
                if(des->typeofsensor == eoas_imu_qua)
                    newSensor.values.resize(4);
                else
                    newSensor.values.resize(3);
                newSensor.state = 0;
                mysens[des->typeofsensor].push_back(newSensor);
            }
        }
    }
}

bool SensorsData::outOfRangeErrorHandler(const std::out_of_range& oor) const
{
    yError() << errorstring.c_str() << "Out of Range error: " << oor.what();
    return false;
}

size_t SensorsData::getNumOfSensors(eOas_sensor_t type) const
{
    std::lock_guard<std::mutex> lck (mutex);
    return mysens[type].size();
}

uint8_t SensorsData::getSensorStatus(size_t sens_index, eOas_sensor_t type) const
{
    try
    {
        std::lock_guard<std::mutex> lck (mutex);
//         sensorInfo_t info = mysens[type].at(sens_index);
//         return info.state;
        return mysens[type].at(sens_index).state;
    }
    catch (const std::out_of_range& oor)
    {
        outOfRangeErrorHandler(oor);
        return yarp::dev::MAS_ERROR;
    }

}
bool SensorsData::getSensorName(size_t sens_index, eOas_sensor_t type, std::string &name) const
{
    try
    {
        std::lock_guard<std::mutex> lck (mutex);
        name = mysens[type].at(sens_index).name;
        return true;
    }
    catch (const std::out_of_range& oor)
    {
        return outOfRangeErrorHandler(oor);
    }

}
bool SensorsData::getSensorFrameName(size_t sens_index, eOas_sensor_t type, std::string &frameName) const
{
    try
    {
        std::lock_guard<std::mutex> lck (mutex);
        frameName = mysens[type].at(sens_index).framename;
        return true;
    }
    catch (const std::out_of_range& oor)
    {
        return outOfRangeErrorHandler(oor);
    }
}


bool SensorsData::getSensorMeasure(size_t sens_index, eOas_sensor_t type, yarp::sig::Vector& out, double& timestamp) const
{
    try
    {   std::lock_guard<std::mutex> lck (mutex);
        out = mysens[type].at(sens_index).values;
        switch(type)
        {
            case eoas_imu_acc:
            {
                for(int i=0; i<out.size(); i++)
                    out[i] = measConverter.convertAcc_raw2metric(out[i]);
            }break;

            case  eoas_imu_mag:
            {    for(int i=0; i<out.size(); i++)
                out[i] = measConverter.convertMag_raw2metric(out[i]);
            }break;

            case eoas_imu_gyr:
            {    for(int i=0; i<out.size(); i++)
                out[i] = measConverter.convertGyr_raw2metric(out[i]);
            }break;

            case eoas_imu_eul:
            {
                for(int i=0; i<out.size(); i++)
                    out[i] = measConverter.convertEul_raw2metric(out[i]);
            }

            default: break;
        };
        timestamp = mysens[type].at(sens_index).timestamp;
    }
    catch (const std::out_of_range& oor)
    {
        return outOfRangeErrorHandler(oor);
    }

}

bool SensorsData::update(eOas_sensor_t type, uint8_t index, eOas_inertial3_data_t *newdata)
{
    std::lock_guard<std::mutex> lck (mutex);

    sensorInfo_t *info = &(mysens[type][index]);

    info->values[0] = newdata->x;
    info->values[1] = newdata->y;
    info->values[2] = newdata->z;
    info->timestamp = yarp::os::Time::now();

    return true;

}

bool SensorsData::updateStatus(eOas_sensor_t type, uint8_t index, eOas_inertial3_sensorstatus_t &status)
{
    std::lock_guard<std::mutex> lck (mutex);

    sensorInfo_t *info = &(mysens[type][index]);

    switch(type)
    {
        case eoas_imu_acc:
            info->state = status.calib.acc;
        break;

        case eoas_imu_mag:
            info->state = status.calib.mag;
            break;

        case eoas_imu_gyr:
            info->state = status.calib.gyr;
            break;
        case eoas_imu_eul:
        case eoas_imu_qua:
        case eoas_imu_lia:
            info->state = status.calib.acc + status.calib.gyr + status.calib.mag;
            break;
        default:
            info->state = status.general;
            break;
    }

    //yError() << "UPDATE STATUS OF SENSOR " << index << "with type "<< eoas_sensor2string(type) <<"with value " << status.general <<"info.state= " << info->state;
    return true;

}

eo_imu_privData::eo_imu_privData(std::string name):embObjDevPrivData(name)
{;}
eo_imu_privData::~eo_imu_privData()
{;}

bool eo_imu_privData::fromConfig(yarp::os::Searchable &config, servConfigImu_t &servCfg)
{
    ServiceParser* parser = new ServiceParser;
    bool ret = parser->parseService(config, servCfg);
    delete parser;
    if(!ret)
        yError() << "embObjIMU" << getBoardInfo() << ": missing some configuration parameter. Check logs and your config file.";
    return ret;;
}

bool eo_imu_privData::sendConfing2board(servConfigImu_t &servCfg)
{
    eOas_inertial3_config_t cfg ={0};
    cfg.datarate = servCfg.acquisitionrate;
    cfg.enabled = 0;

    for(size_t i=0; i<servCfg.inertials.size(); i++)
    {
        eo_common_word_bitset(&cfg.enabled, i);
    }

    eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_inertial3, 0, eoprot_tag_as_inertial3_config);
    if(false == res->setcheckRemoteValue(id32, &cfg, 10, 0.010, 0.050))
    {
        yError() << "embObjIMU" << getBoardInfo() << "FATAL: sendConfing2board() had an error while calling setcheckRemoteValue() for config the board";
        return false;
    }
    else
    {
        if(behFlags.verbosewhenok)
        {
            yError() << "embObjIMU" << getBoardInfo() << "sendConfing2board() correctly configured enabled sensors with period" << cfg.datarate;
        }
    }

    return true;
}

// configure regular rops
bool eo_imu_privData::initRegulars(void)
{
    vector<eOprotID32_t> id32v(0);
    eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_inertial3, 0, eoprot_tag_as_inertial3_status);
    // put it inside vector
    id32v.push_back(id32);

    if(false == res->serviceSetRegulars(eomn_serv_category_inertials3, id32v))
    {
        yError() << "embObjIMU" << getBoardInfo() <<"initRegulars() fails to add its variables to regulars: cannot proceed any further";
        return false;
    }
    else
    {
        if(isVerbose())
        {
            yError() << "embObjIMU" << getBoardInfo()  <<"initRegulars() added" << id32v.size() << " regular rops to the board";
            char nvinfo[128];
            for (size_t r = 0; r<id32v.size(); r++)
            {
                uint32_t item = id32v.at(r);
                eoprot_ID2information(item, nvinfo, sizeof(nvinfo));
                yDebug() << "\t it added regular rop for" << nvinfo;
            }
        }
    }

    return true;
}



yarp::dev::MAS_status eo_imu_privData::sensorState_eo2yarp(eOas_sensor_t type, uint8_t eo_state)
{
    debugPrintStateNotOK(type, eo_state);
    //In the future I'll put here a translation of state from firmware to yarp interface.
    //Currently I return always ok for testing porpouse


    return yarp::dev::MAS_OK;
}


#undef DEBUG_PRINT_STATE
#undef DEBUG_PRINT_STATE_FULL

void eo_imu_privData::debugPrintStateNotOK(eOas_sensor_t type, uint8_t eo_state)
{
    static int count[eoas_sensors_numberof]={0};

    count[type]++;
    if(count[type]< 100) //the print there is about every 1 second
        return;

#if defined(DEBUG_PRINT_STATE)
    switch(type)
    {
        case eoas_imu_acc:
        case eoas_imu_mag:
        case eoas_imu_gyr:
            if(0 == eo_state)
            {
                yError() << getBoardInfo() << "sensor " << eoas_sensor2string(type) << "is not calibrated because its calib status reg is =" << eo_state<< "(reg: 0 = not calib, 3 = fully calib)";
            }
#if defined(DEBUG_PRINT_STATE_FULL)
            else if(1 == eo_state)
            {
                yWarning() << getBoardInfo() << "sensor " << eoas_sensor2string(type) << "is poorly calibrated because its calib status reg is =" << eo_state<< "(reg: 0 = not calib, 3 = fully calib)";
            }
            else
            {
                yInfo() << getBoardInfo() << "sensor " << eoas_sensor2string(type) << "is calibrated because its calib status reg is =" << eo_state<< "(reg: 0 = not calib, 3 = fully calib)";
            }
#endif
            break;
        case eoas_imu_eul:
        case eoas_imu_qua:
        case eoas_imu_lia:
            if(eo_state < 3)
            {
                yError() << getBoardInfo() << "sensor " << eoas_sensor2string(type) << "is not calibrated because sum of the 3 calib status regs is =" << eo_state<< "(reg: 0 = not calib, 3=fully calib)";
            }
#if defined(DEBUG_PRINT_STATE_FULL)
            else if(eo_state < 6)
            {
                yWarning() << getBoardInfo() << "sensor " << eoas_sensor2string(type) << "is poorly calibrated because sum of the 3 calib status regs is =" << eo_state<< "(reg: 0 = not calib, 3 = fully calib)";
            }
            else
            {
                yInfo() << getBoardInfo() << "sensor " << eoas_sensor2string(type) << "is calibrated because sum of the 3 calib status regs is =" << eo_state<< "(reg: 0 = not calib, 3 = fully calib)";
            }
#endif
            break;
    }
#endif //defined(DEBUG_PRINT_STATE)

    count[type]=0;

}
