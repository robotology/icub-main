/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include <string>
#include <yarp/os/Time.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/NetType.h>
#include <yarp/sig/Vector.h>
#include <embObjIMU.h>

#include "EoAnalogSensors.h"
#include "EoManagement.h"
#include "EoProtocol.h"
#include "EoProtocolMN.h"
#include "EoProtocolAS.h"

#include "EOconstarray.h"

#include <mutex>
#include <stdexcept>
#include <ethManager.h>
#include <abstractEthResource.h>
#include "FeatureInterface.h"

#include "embObjGeneralDevPrivData.h"

#include "imuMeasureConverter.h"

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

// data used for handling the received messsages
class PositionMaps
{
    std::uint8_t positionmap[eoas_sensors_numberof][eOcanports_number][16];
public:
    PositionMaps();
    ~PositionMaps();
    bool init(servConfigImu_t &servCfg);
    bool getIndex(const eOas_inertial3_data_t *data, uint8_t &index, eOas_sensor_t &type);
};

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

typedef struct
{
    std::string name;
    std::string framename;
    yarp::sig::Vector values;
    uint8_t state; 
    double timestamp;
} sensorInfo_t;

class SensorsData
{
private:
    std::vector<std::vector<sensorInfo_t>> mysens;
    mutable std::mutex mutex;
    string errorstring;
    
public:
    ImuMeasureConverter measConverter;
    SensorsData();
    void init(servConfigImu_t &servCfg, string error_string);
    bool update(eOas_sensor_t type, uint8_t index, eOas_inertial3_data_t *newdata);
    bool outOfRangeErrorHandler(const std::out_of_range& oor) const;
    
    size_t getNumOfSensors(eOas_sensor_t type) const;
    uint8_t getSensorStatus(size_t sens_index, eOas_sensor_t type) const;
    bool getSensorName(size_t sens_index, eOas_sensor_t type, yarp::os::ConstString &name) const;
    bool getSensorFrameName(size_t sens_index, eOas_sensor_t type, yarp::os::ConstString &frameName) const;
    bool getSensorMeasure(size_t sens_index, eOas_sensor_t type, yarp::sig::Vector& out, double& timestamp) const;
};

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
                newSensor.state = 0; //unused
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
        return mysens[type].at(sens_index).state;
    }
    catch (const std::out_of_range& oor) 
    {
        outOfRangeErrorHandler(oor);
        return yarp::dev::MAS_ERROR;
    }
    
}
bool SensorsData::getSensorName(size_t sens_index, eOas_sensor_t type, yarp::os::ConstString &name) const
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
bool SensorsData::getSensorFrameName(size_t sens_index, eOas_sensor_t type, yarp::os::ConstString &frameName) const
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
            
            default: break;
        };
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
    info->state = newdata->status;
    info->timestamp = yarp::os::Time::now();
    return true;
    
}















class eo_imu_privData : public yarp::dev::embObjDevPrivData
{
public:
    SensorsData sens;
    PositionMaps maps;
    
    eo_imu_privData();
    ~eo_imu_privData();

};

eo_imu_privData::eo_imu_privData()
{;}
eo_imu_privData::~eo_imu_privData()
{;}








#define GET_privData(x) (*((static_cast<eo_imu_privData*>(x))))


/**
 * This device implements the embObjIMU sensor
 * @author Valentina Gaggero
 */
embObjIMU::embObjIMU()
{
    mPriv = new eo_imu_privData();

}

embObjIMU::~embObjIMU()
{
    close();
    delete &GET_privData(mPriv);
}

std::string embObjIMU::getBoardInfo(void) const
{
   return GET_privData(mPriv).getBoardInfo();
}

bool embObjIMU::fromConfig(yarp::os::Searchable &config, servConfigImu_t &servCfg)
{
    ServiceParser* parser = new ServiceParser;
    bool ret = parser->parseService(config, servCfg);
    delete parser;
    return ret;;
}

void embObjIMU::cleanup(void)
{
    if(GET_privData(mPriv).ethManager == NULL) return;
    
    int ret = GET_privData(mPriv).ethManager->releaseResource2(GET_privData(mPriv).res, this);
    GET_privData(mPriv).res = NULL;
    if(ret == -1)
        GET_privData(mPriv).ethManager->killYourself();
    GET_privData(mPriv).behFlags.opened = false;
}

bool embObjIMU::sendConfing2board(servConfigImu_t &servCfg)
{
    eOas_inertial3_config_t cfg ={0};
    cfg.datarate = servCfg.acquisitionrate;
    cfg.enabled = 0;
    
    for(size_t i=0; i<servCfg.inertials.size(); i++)
    {
        eo_common_word_bitset(&cfg.enabled, i);
    }

    eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_inertial3, 0, eoprot_tag_as_inertial3_config);
    if(false == GET_privData(mPriv).res->setcheckRemoteValue(id32, &cfg, 10, 0.010, 0.050))
    {
        yError() << "embObjIMU" << getBoardInfo() << "FATAL: sendConfing2board() had an error while calling setcheckRemoteValue() for config the board";
        return false;
    }
    else
    {
        if(GET_privData(mPriv).behFlags.verbosewhenok)
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


    if(false == GET_privData(mPriv).res->serviceSetRegulars(eomn_serv_category_inertials3, id32v))
    {
        yError() << "embObjIMU" << getBoardInfo() <<"initRegulars() fails to add its variables to regulars: cannot proceed any further";
        return false;
    }
    else
    {
        if(GET_privData(mPriv).behFlags.verbosewhenok)
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
    
    GET_privData(mPriv).ethManager = eth::TheEthManager::instance();
    if(NULL == GET_privData(mPriv).ethManager)
    {
        yFatal() << "embObjIMU" << getBoardInfo() << "open() fails to instantiate ethManager";
        return false;
    }
    
    eOipv4addr_t ipv4addr;
    string boardIPstring;
    string boardName;
    
    if(false == GET_privData(mPriv).ethManager->verifyEthBoardInfo(config, ipv4addr, boardIPstring, boardName))
    {
        yError() << "embObjIMU" << getBoardInfo() << "open(): object TheEthManager fails in parsing ETH propertiex from xml file";
        return false;
    }
    
    
    // - now all other things
    
    // read stuff from config file
    servConfigImu_t servCfg;
    if(!fromConfig(config, servCfg))
    {
        yError() << "embObjIMU" << getBoardInfo() << ": missing some configuration parameter. Check logs and your config file.";
        return false;
    }
    
    
    // -- instantiate EthResource etc.
    
    GET_privData(mPriv).res = GET_privData(mPriv).ethManager->requestResource2(this, config);
    if(NULL == GET_privData(mPriv).res)
    {
        yError() << "embObjIMU" << getBoardInfo() << "open() fails because could not instantiate the ethResource ... unable to continue";
        return false;
    }
    
    
    if(!GET_privData(mPriv).res->verifyEPprotocol(eoprot_endpoint_analogsensors))
    {
        cleanup();
        return false;
    }
    
    const eOmn_serv_parameter_t* servparam = &servCfg.ethservice;
    
    if(false == GET_privData(mPriv).res->serviceVerifyActivate(eomn_serv_category_inertials3, servparam, 5.0))
    {
        yError() << "embObjIMU" << getBoardInfo() << "open() has an error in call of ethResources::serviceVerifyActivate() ";

        cleanup();
        return false;
    }
    
    //init conversion factor
    //TODO: currently the conversion factors are not read from xml files, but configured here.
    //please read IMUbosh datasheet for more information
    
    servCfg.convFactors.accFactor = 100.0; // 1 m/sec2 = 100 binary units
    servCfg.convFactors.magFactor = 16.0;  // 1 microT = 16 binary units
    servCfg.convFactors.gyrFactor = 16.0;  // 1 degree/sec = 16 binary units
    //eul angles don't need a conversion.
    GET_privData(mPriv).sens.measConverter.Initialize(servCfg.convFactors.accFactor, servCfg.convFactors.gyrFactor,  servCfg.convFactors.magFactor);
    
    // configure the sensor(s)
    
    if(false == sendConfing2board(servCfg))
    {
        cleanup();
        return false;
    }
    
    
    if(false == initRegulars())
    {
        cleanup();
        return false;
    }
    
    
    if(false == GET_privData(mPriv).res->serviceStart(eomn_serv_category_inertials3))
    {
        yError() << "embObjIMU" << getBoardInfo() << "open() fails to start as service.... cannot continue";
        cleanup();
        return false;
    }
    else
    {
        if(GET_privData(mPriv).behFlags.verbosewhenok)
        {
            yDebug() << "embObjIMU" << getBoardInfo() << "open() correctly starts service";
        }
    }

    // build data structure used to handle rx packets
    GET_privData(mPriv).maps.init(servCfg);
    
    
    {   // start the configured sensors. so far, we must keep it in here. later on we can remove this command
        
        uint8_t enable = 1;
        
        eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_inertial3, 0, eoprot_tag_as_inertial3_cmmnds_enable);
        if(false == GET_privData(mPriv).res->setRemoteValue(id32, &enable))
        {
            yError() << "embObjIMU" << getBoardInfo() << "open() fails to command the start transmission of the configured inertials";
            cleanup();
            return false;
        }
    }
    
    GET_privData(mPriv).sens.init(servCfg, getBoardInfo());
    GET_privData(mPriv).behFlags.opened = true;
    return true;
}

bool embObjIMU::close()
{
    cleanup();
    return true;
}





size_t embObjIMU::getNrOfThreeAxisGyroscopes() const
{
    return GET_privData(mPriv).sens.getNumOfSensors(eoas_imu_gyr);
}

yarp::dev::MAS_status embObjIMU::getThreeAxisGyroscopeStatus(size_t sens_index) const
{
    return sensorState_eo2yarp(GET_privData(mPriv).sens.getSensorStatus(sens_index, eoas_imu_gyr));
}

bool embObjIMU::getThreeAxisGyroscopeName(size_t sens_index, yarp::os::ConstString &name) const
{
    return GET_privData(mPriv).sens.getSensorName(sens_index, eoas_imu_gyr, name);
}

bool embObjIMU::getThreeAxisGyroscopeFrameName(size_t sens_index, yarp::os::ConstString &frameName) const
{
    return GET_privData(mPriv).sens.getSensorFrameName(sens_index, eoas_imu_gyr, frameName);
}

bool embObjIMU::getThreeAxisGyroscopeMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    return GET_privData(mPriv).sens.getSensorMeasure(sens_index, eoas_imu_gyr, out, timestamp);
}

size_t embObjIMU::getNrOfThreeAxisLinearAccelerometers() const
{
    return GET_privData(mPriv).sens.getNumOfSensors(eoas_imu_acc);
}

yarp::dev::MAS_status embObjIMU::getThreeAxisLinearAccelerometerStatus(size_t sens_index) const
{
    return sensorState_eo2yarp(GET_privData(mPriv).sens.getSensorStatus(sens_index, eoas_imu_acc));
}

bool embObjIMU::getThreeAxisLinearAccelerometerName(size_t sens_index, yarp::os::ConstString &name) const
{
    return GET_privData(mPriv).sens.getSensorName(sens_index, eoas_imu_acc, name);
}

bool embObjIMU::getThreeAxisLinearAccelerometerFrameName(size_t sens_index, yarp::os::ConstString &frameName) const
{
    return GET_privData(mPriv).sens.getSensorFrameName(sens_index, eoas_imu_acc, frameName);
}

bool embObjIMU::getThreeAxisLinearAccelerometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    return GET_privData(mPriv).sens.getSensorMeasure(sens_index, eoas_imu_acc, out, timestamp);
}

size_t embObjIMU::getNrOfThreeAxisMagnetometers() const
{
    return GET_privData(mPriv).sens.getNumOfSensors(eoas_imu_mag);
}

yarp::dev::MAS_status embObjIMU::getThreeAxisMagnetometerStatus(size_t sens_index) const
{
    return sensorState_eo2yarp(GET_privData(mPriv).sens.getSensorStatus(sens_index, eoas_imu_mag));
}

bool embObjIMU::getThreeAxisMagnetometerName(size_t sens_index, yarp::os::ConstString &name) const
{
    return GET_privData(mPriv).sens.getSensorName(sens_index, eoas_imu_mag, name);
}

bool embObjIMU::getThreeAxisMagnetometerFrameName(size_t sens_index, yarp::os::ConstString &frameName) const
{
    return GET_privData(mPriv).sens.getSensorFrameName(sens_index, eoas_imu_mag, frameName);
}

bool embObjIMU::getThreeAxisMagnetometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    return GET_privData(mPriv).sens.getSensorMeasure(sens_index, eoas_imu_mag, out, timestamp);
}

size_t embObjIMU::getNrOfOrientationSensors() const
{
    return GET_privData(mPriv).sens.getNumOfSensors(eoas_imu_eul);
}

yarp::dev::MAS_status embObjIMU::getOrientationSensorStatus(size_t sens_index) const
{
    return sensorState_eo2yarp(GET_privData(mPriv).sens.getSensorStatus(sens_index, eoas_imu_eul));
}

bool embObjIMU::getOrientationSensorName(size_t sens_index, yarp::os::ConstString &name) const
{
    return GET_privData(mPriv).sens.getSensorName(sens_index, eoas_imu_eul, name);
}

bool embObjIMU::getOrientationSensorFrameName(size_t sens_index, yarp::os::ConstString &frameName) const
{
    return GET_privData(mPriv).sens.getSensorFrameName(sens_index, eoas_imu_eul, frameName);
}

bool embObjIMU::getOrientationSensorMeasureAsRollPitchYaw(size_t sens_index, yarp::sig::Vector& rpy_out, double& timestamp) const
{
    return GET_privData(mPriv).sens.getSensorMeasure(sens_index, eoas_imu_eul, rpy_out, timestamp);
    
}


bool embObjIMU::initialised()
{
    return GET_privData(mPriv).behFlags.opened;
}

eth::iethresType_t embObjIMU::type()
{
    return eth::iethres_analoginertial3;
}


yarp::dev::MAS_status embObjIMU::sensorState_eo2yarp(uint8_t eo_state)
{
    /*Note: 9 means that gyro, acc and mag are completely calibrated
     For more information see IMUbosgh BNO055 data sheet page 68 */
    if(eo_state == 9)
        return yarp::dev::MAS_OK;
    else
        return  yarp::dev::MAS_ERROR;
}

bool embObjIMU::update(eOprotID32_t id32, double timestamp, void* rxdata)
{
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
        bool validdata =  GET_privData(mPriv).maps.getIndex(data, index, type);
        
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
        
        GET_privData(mPriv).sens.update(type, index, data);
    }
    return true;

}

//this function can be called inside update function to print the received data
void embObjIMU::updateDebugPrints(eOprotID32_t id32, double timestamp, void* rxdata)
{
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
                    GET_privData(mPriv).maps.getIndex(data, pos, type);
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

// eof

