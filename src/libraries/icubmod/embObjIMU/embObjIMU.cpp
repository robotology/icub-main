/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include <string>
#include <yarp/os/Thread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/NetType.h>
#include <embObjIMU.h>

#include "EoAnalogSensors.h"
#include "EoManagement.h"
#include "EoProtocol.h"
#include "EoProtocolMN.h"
#include "EoProtocolAS.h"

#include "EOconstarray.h"

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

/**
 * This device implements the embObjIMU sensor
 * @author Valentina Gaggero
 */
embObjIMU::embObjIMU()
{
    ethManager = nullptr;
    res = nullptr;
    parser = nullptr;
    opened = false;
    ConstString tmp = NetworkBase::getEnvironment("ETH_VERBOSEWHENOK");
    if (tmp != "")
    {
       verbosewhenok = (bool)NetType::toInt(tmp);
    }
    else
    {
        verbosewhenok = false;
    }
}

embObjIMU::~embObjIMU()
{
    close();
}

std::string embObjIMU::getBoardInfo(void) const
{
    if(nullptr == res)
    {
        return " BOARD name_unknown (IP unknown) ";
    }
    else
    {
        return ("BOARD " + res->getProperties().boardnameString +  " (IP "  + res->getProperties().ipv4addrString + ") ");
    }
}

bool embObjIMU::fromConfig(yarp::os::Searchable &config)
{
    if(false == parser->parseService(config, servCfg))
    {
        return false;
    }
    
    return true;
}

void embObjIMU::cleanup(void)
{
    if(ethManager == NULL) return;
    
    int ret = ethManager->releaseResource2(res, this);
    res = NULL;
    if(ret == -1)
        ethManager->killYourself();
    opened = false;
}

bool embObjIMU::sendConfing2board(void)
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
        if(verbosewhenok)
        {
            yError() << "embObjIMU" << getBoardInfo() << "sendConfing2board() correctly configured enabled sensors with period" << cfg.datarate;
        }
    }
    
    return true;

}

bool embObjIMU::initRegulars(void)
{
    // configure regular rops

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
        if(verbosewhenok)
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

bool embObjIMU::open(yarp::os::Searchable &config)
{
    // - first thing to do is verify if the eth manager is available. then i parse info about the eth board.
    
    ethManager = eth::TheEthManager::instance();
    if(NULL == ethManager)
    {
        yFatal() << "embObjIMU" << getBoardInfo() << "open() fails to instantiate ethManager";
        return false;
    }
    
    eOipv4addr_t ipv4addr;
    string boardIPstring;
    string boardName;
    
    if(false == ethManager->verifyEthBoardInfo(config, ipv4addr, boardIPstring, boardName))
    {
        yError() << "embObjIMU" << getBoardInfo() << "open(): object TheEthManager fails in parsing ETH propertiex from xml file";
        return false;
    }
    
    
    // - now all other things
    
    if(NULL == parser)
    {
        parser = new ServiceParser;
    }
    
    // read stuff from config file
    if(!fromConfig(config))
    {
        yError() << "embObjIMU" << getBoardInfo() << ": missing some configuration parameter. Check logs and your config file.";
        return false;
    }
    
    
    // -- instantiate EthResource etc.
    
    res = ethManager->requestResource2(this, config);
    if(NULL == res)
    {
        yError() << "embObjIMU" << getBoardInfo() << "open() fails because could not instantiate the ethResource ... unable to continue";
        return false;
    }
    
    
    if(!res->verifyEPprotocol(eoprot_endpoint_analogsensors))
    {
        cleanup();
        return false;
    }
    
    const eOmn_serv_parameter_t* servparam = &servCfg.ethservice;
    
    if(false == res->serviceVerifyActivate(eomn_serv_category_inertials3, servparam, 5.0))
    {
        yError() << "embObjIMU" << getBoardInfo() << "open() has an error in call of ethResources::serviceVerifyActivate() ";

        cleanup();
        return false;
    }
    
    
    // configure the sensor(s)
    
    if(false == sendConfing2board())
    {
        cleanup();
        return false;
    }
    
    
    if(false == initRegulars())
    {
        cleanup();
        return false;
    }
    
    
    if(false == res->serviceStart(eomn_serv_category_inertials3))
    {
        yError() << "embObjIMU" << getBoardInfo() << "open() fails to start as service.... cannot continue";
        cleanup();
        return false;
    }
    else
    {
        if(verbosewhenok)
        {
            yDebug() << "embObjIMU" << getBoardInfo() << "open() correctly starts service";
        }
    }

    // build data structure used to handle rx packets
    buildmaps();
    
    
    {   // start the configured sensors. so far, we must keep it in here. later on we can remove this command
        
        uint8_t enable = 1;
        
        eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_inertial3, 0, eoprot_tag_as_inertial3_cmmnds_enable);
        if(false == res->setRemoteValue(id32, &enable))
        {
            yError() << "embObjIMU" << getBoardInfo() << "open() fails to command the start transmission of the configured inertials";
            cleanup();
            return false;
        }
    }
    
    opened = true;
    return true;
}

bool embObjIMU::close()
{
    cleanup();
    return true;
}



bool embObjIMU::outOfRangeErrorHandler(const std::out_of_range& oor) const
{
    yError() << "embObjIMU" << getBoardInfo()  << "Out of Range error: " << oor.what();
    return false;
}

size_t embObjIMU::getNumOfSensors(eOas_sensor_t type) const
{
    std::lock_guard<std::mutex> lck (mutex);
    return sensorsData[type].size();
}

yarp::dev::MAS_status embObjIMU::getSensorStatus(size_t sens_index, eOas_sensor_t type) const
{
    try
    {
        std::lock_guard<std::mutex> lck (mutex);
        return sensorsData[type].at(sens_index).state;
    }
    catch (const std::out_of_range& oor) 
    {
        outOfRangeErrorHandler(oor);
        return yarp::dev::MAS_ERROR;
    }
    
}
bool embObjIMU::getSensorName(size_t sens_index, eOas_sensor_t type, yarp::os::ConstString &name) const
{
    try
    {
        std::lock_guard<std::mutex> lck (mutex);
        name = sensorsData[type].at(sens_index).name;
        return true;
    }
    catch (const std::out_of_range& oor) 
    {
        return outOfRangeErrorHandler(oor);
    }
    
}
bool embObjIMU::getSensorFrameName(size_t sens_index, eOas_sensor_t type, yarp::os::ConstString &frameName) const
{
    try
    {
        std::lock_guard<std::mutex> lck (mutex);
        frameName = sensorsData[type].at(sens_index).framename;
        return true;
    }
    catch (const std::out_of_range& oor) 
    {
        return outOfRangeErrorHandler(oor);
    }
}


bool embObjIMU::getSensorMeasure(size_t sens_index, eOas_sensor_t type, yarp::sig::Vector& out, double& timestamp) const
{
    try
    {   std::lock_guard<std::mutex> lck (mutex);
        out.resize(0);
        for(int i=0; i<sensorsData[type].at(sens_index).values.size(); i++)
            out.push_back(sensorsData[type].at(sens_index).values[i]);// TODO Verifica la copia o usa il vector di yarp
        timestamp = sensorsData[type].at(sens_index).timestamp;
        return true;
    }
    catch (const std::out_of_range& oor) 
    {
        return outOfRangeErrorHandler(oor);
    }
    
}


size_t embObjIMU::getNrOfThreeAxisGyroscopes() const
{
    return getNumOfSensors(eoas_imu_gyr);
}

yarp::dev::MAS_status embObjIMU::getThreeAxisGyroscopeStatus(size_t sens_index) const
{
    return getSensorStatus(sens_index, eoas_imu_gyr);
}

bool embObjIMU::getThreeAxisGyroscopeName(size_t sens_index, yarp::os::ConstString &name) const
{
    return getSensorName(sens_index, eoas_imu_gyr, name);
}

bool embObjIMU::getThreeAxisGyroscopeFrameName(size_t sens_index, yarp::os::ConstString &frameName) const
{
    return getSensorFrameName(sens_index, eoas_imu_gyr, frameName);
}

bool embObjIMU::getThreeAxisGyroscopeMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    return getSensorMeasure(sens_index, eoas_imu_gyr, out, timestamp);
}

size_t embObjIMU::getNrOfThreeAxisLinearAccelerometers() const
{
    return getNumOfSensors(eoas_imu_acc);
}

yarp::dev::MAS_status embObjIMU::getThreeAxisLinearAccelerometerStatus(size_t sens_index) const
{
    return getSensorStatus(sens_index, eoas_imu_acc);
}

bool embObjIMU::getThreeAxisLinearAccelerometerName(size_t sens_index, yarp::os::ConstString &name) const
{
    return getSensorName(sens_index, eoas_imu_acc, name);
}

bool embObjIMU::getThreeAxisLinearAccelerometerFrameName(size_t sens_index, yarp::os::ConstString &frameName) const
{
    return getSensorFrameName(sens_index, eoas_imu_acc, frameName);
}

bool embObjIMU::getThreeAxisLinearAccelerometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    return getSensorMeasure(sens_index, eoas_imu_acc, out, timestamp);
}

size_t embObjIMU::getNrOfThreeAxisMagnetometers() const
{
    return getNumOfSensors(eoas_imu_mag);
}

yarp::dev::MAS_status embObjIMU::getThreeAxisMagnetometerStatus(size_t sens_index) const
{
    return getSensorStatus(sens_index, eoas_imu_mag);
}

bool embObjIMU::getThreeAxisMagnetometerName(size_t sens_index, yarp::os::ConstString &name) const
{
    return getSensorName(sens_index, eoas_imu_mag, name);
}

bool embObjIMU::getThreeAxisMagnetometerFrameName(size_t sens_index, yarp::os::ConstString &frameName) const
{
    return getSensorFrameName(sens_index, eoas_imu_mag, frameName);
}

bool embObjIMU::getThreeAxisMagnetometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    return getSensorMeasure(sens_index, eoas_imu_mag, out, timestamp);}

size_t embObjIMU::getNrOfOrientationSensors() const
{
    return getNumOfSensors(eoas_imu_eul);
}

yarp::dev::MAS_status embObjIMU::getOrientationSensorStatus(size_t sens_index) const
{
    return getSensorStatus(sens_index, eoas_imu_eul);
}

bool embObjIMU::getOrientationSensorName(size_t sens_index, yarp::os::ConstString &name) const
{
    return getSensorName(sens_index, eoas_imu_eul, name);
}

bool embObjIMU::getOrientationSensorFrameName(size_t sens_index, yarp::os::ConstString &frameName) const
{
    return getSensorFrameName(sens_index, eoas_imu_eul, frameName);
}

bool embObjIMU::getOrientationSensorMeasureAsRollPitchYaw(size_t sens_index, yarp::sig::Vector& rpy_out, double& timestamp) const
{
    return getSensorMeasure(sens_index, eoas_imu_eul, rpy_out, timestamp);
    
}


bool embObjIMU::initialised()
{
    return opened;
}

eth::iethresType_t embObjIMU::type()
{
    return eth::iethres_analoginertial3;
}


yarp::dev::MAS_status embObjIMU::sensorState_eo2yarp(uint8_t eo_state)
{
    //TODO
    return yarp::dev::MAS_OK;
}

bool embObjIMU::update(eOprotID32_t id32, double timestamp, void* rxdata)
{
    std::lock_guard<std::mutex> lck (mutex);
    
    eOas_inertial3_status_t *i3s  = (eOas_inertial3_status_t*)rxdata;

    EOconstarray* arrayofvalues = eo_constarray_Load(reinterpret_cast<const EOarray*>(&i3s->arrayofdata));

    uint8_t numofIntem2update = eo_constarray_Size(arrayofvalues);

    for(int i=0; i<numofIntem2update; i++)
    {
        eOas_inertial3_data_t *data = (eOas_inertial3_data_t*) eo_constarray_At(arrayofvalues, i);
        if(data == NULL)
        {
            yError() << "embObjIMU" << getBoardInfo() << "update(): I have to update " << numofIntem2update << "items, but the " << i << "-th item is null.";
            continue; 
            //NOTE: I signal this strange situation with an arror for debug porpouse...maybe we can convert in in warning when the device is stable....
        }
        uint8_t index;
        eOas_sensor_t type;
        bool validdata =  getIndex(data, index, type);
        
        if(!validdata)
        {
            yError("NOT VALID value[%i] is: seq = %d, timestamp = %d, type = %s, id = %d, v= ((%d), %d, %d, %d), status = %d",
                    i,
                    data->seq,
                    data->timestamp,
                    eoas_sensor2string(static_cast<eOas_sensor_t>(data->typeofsensor)),
                    data->id,
                    data->w, data->x, data->y, data->z,
                    data->status);
            continue;
        }
        
        sensorInfo_t *info = &sensorsData[type][index];
        
        info->values[0] = data->x;
        info->values[1] = data->y;
        info->values[2] = data->z;
        info->state = sensorState_eo2yarp(data->status);
        info->timestamp = yarp::os::Time::now();
    }

}

//this function can be called inside update function to print the received data
void embObjIMU::updateDebugPrints(eOprotID32_t id32, double timestamp, void* rxdata)
{
    std::lock_guard<std::mutex> lck (mutex);
    static int prog = 1;
    static double prevtime =  yarp::os::Time::now();
    
    double delta = yarp::os::Time::now() - prevtime;
    double millidelta = 1000.0 *delta;
    long milli = static_cast<long>(millidelta);
    
    
    eOas_inertial3_status_t *i3s  = (eOas_inertial3_status_t*)rxdata;
    
    EOconstarray* arrayofvalues = eo_constarray_Load(reinterpret_cast<const EOarray*>(&i3s->arrayofdata));
    
    uint8_t n = eo_constarray_Size(arrayofvalues);
    
    if(n > 0)
    {
        prog++;
    //        if(0 == (prog%20))
        {
            yDebug() << "embObjIMU::update(): received" << n << "values after" << milli << "milli";

            for(int i=0; i<n; i++)
            {
                eOas_inertial3_data_t *data = (eOas_inertial3_data_t*) eo_constarray_At(arrayofvalues, i);
                if(NULL == data)
                {
                    yDebug() << "embObjIMU::update(): NULL";
                }
                else
                {
                    uint8_t pos = 0xff;
                    eOas_sensor_t type;
                    getIndex(data, pos, type);
                    yDebug("value[%i] is: seq = %d, timestamp = %d, type = %s, id = %d, v= ((%d), %d, %d, %d), status = %d, pos = %d",
                            i,
                            data->seq,
                            data->timestamp,
                            eoas_sensor2string(static_cast<eOas_sensor_t>(type)),
                            data->id,
                            data->w, data->x, data->y, data->z,
                            data->status,
                            pos);
                }
            }

        }

    }
}


bool embObjIMU::buildmaps(void)
{
    //initis internals maps
    
    memset(positionmap, 0xff, sizeof(positionmap));
    std::uint8_t numberof[eoas_sensors_numberof];
    memset(numberof, 0, sizeof(numberof));

    sensorsData.resize(eoas_sensors_numberof);
    for(int t=0; t<eoas_sensors_numberof; t++)
    { sensorsData[t].resize(0); }
    
    // and now we list the service config to fill the map withe proper indices.
    
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
                    yDebug() << "embObjIMU" << getBoardInfo() << "one more" << eoas_sensor2string(static_cast<eOas_sensor_t>(des->typeofsensor));
                }
                else if(des->on.any.place == eobrd_place_eth)
                {
                    // must manage the case of gyro on ems
                }

                sensorInfo_t newSens;
                newSens.name = servCfg.id[i];
                newSens.framename = newSens.name;
                newSens.values.resize(3);
                newSens.state = yarp::dev::MAS_OK;
                sensorsData[des->typeofsensor].push_back(newSens);
            }

        }
    }

    return true;
}


// bool embObjIMU::getIndex(const eOas_inertial3_descriptor_t *des, uint8_t &index, eOas_sensor_t &type)
// {
//     if(nullptr == des)
//     {
//         return false;
//     }
//     uint8_t can = (des->on.can.port <<4 | des->on.can.addr);
//     getIndex_core(des->typeofsensor, can,
//     if(des->typeofsensor >= eoas_sensors_numberof)
//     {   // it is not a valid index
//         return false;
//     }
// 
//     uint8_t canbus = data->id >> 4;
// 
//     if(canbus >= eOcanports_number)
//     {
//         return false;
//     }
// 
//     uint8_t canaddress = data->id & 0x0f;
// 
//     index = positionmap[data->typeofsensor][canbus][canaddress];
// 
//     type = data->typeofsensor;
//     return (0xff == index) ? false : true;
// 
// }


bool embObjIMU::getIndex(const eOas_inertial3_data_t *data, uint8_t &index, eOas_sensor_t &type)
{
    if(nullptr == data)
    {
        return false;
    }
    
    if(data->typeofsensor >= eoas_sensors_numberof)
    {   // it is not a valid index
        return false;
    }
    
    uint8_t canbus = data->id >> 4;
    
    if(canbus >= eOcanports_number)
    {
        return false;
    }
    
    uint8_t canaddress = data->id & 0x0f;
    
    index = positionmap[data->typeofsensor][canbus][canaddress];
    
    type = static_cast<eOas_sensor_t>(data->typeofsensor);
    return (0xff == index) ? false : true;
    
}

// bool embObjIMU::getIndex_core(eOas_sensor_t type, uint8_t idcan, uint8_t &index)
// {
//     
//     if(Type >= eoas_sensors_numberof)
//     {   // it is not a valid index
//         return false;
//     }
//     
//     uint8_t canbus = idcan >> 4;
//     
//     if(canbus >= eOcanports_number)
//     {
//         return false;
//     }
//     
//     uint8_t canaddress = idcan & 0x0f;
//     
//     index = positionmap[type][canbus][canaddress];
//     
//     return (0xff == index) ? false : true;
//     
// }
// eof

