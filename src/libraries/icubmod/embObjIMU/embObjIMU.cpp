/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 * Author: Valentina Gaggero
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include <string>
#include <mutex>
#include <stdexcept>
#include <yarp/os/Time.h>

#include <yarp/os/LogStream.h>

#include <embObjIMU.h>

#include "EoAnalogSensors.h"
#include "EoProtocolAS.h"
#include "EOconstarray.h"


#include "FeatureInterface.h"
#include "eo_imu_privData.h"



using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;







#define GET_privData(x) (*((static_cast<eo_imu_privData*>(x))))


/**
 * This device implements the embObjIMU sensor
 * @author Valentina Gaggero
 */
embObjIMU::embObjIMU()
{
    mPriv = new eo_imu_privData("embObjIMU");

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

void embObjIMU::cleanup(void)
{
    GET_privData(mPriv).cleanup(static_cast <eth::IethResource*> (this));
}



bool embObjIMU::open(yarp::os::Searchable &config)
{
    // - first thing to do is verify if the eth manager is available then i parse info about the eth board.

    if(! GET_privData(mPriv).prerareEthService(config, this))
        return false;

    // read stuff from config file

    servConfigImu_t servCfg;
    if(!GET_privData(mPriv).fromConfig(config, servCfg))
        return false;

    if(!GET_privData(mPriv).res->verifyEPprotocol(eoprot_endpoint_analogsensors))
    {
        cleanup();
        return false;
    }

    
    const eOmn_serv_parameter_t* servparam = &servCfg.ethservice;
    
    if(false == GET_privData(mPriv).res->serviceVerifyActivate(eomn_serv_category_inertials3, servparam, 5.0))
    {
        yError() << getBoardInfo() << "open() has an error in call of ethResources::serviceVerifyActivate() ";
        cleanup();
        return false;
    }
    
    //init conversion factor
    //TODO: currently the conversion factors are not read from xml files, but configured here.
    //please read IMUbosh datasheet for more information
    
    servCfg.convFactors.accFactor = 100.0; // 1 m/sec2 = 100 binary units
    servCfg.convFactors.magFactor = 16.0 * 1000000.0;  // 1 microT = 16 binary units
    servCfg.convFactors.gyrFactor = 16.0;  // 1 degree/sec = 16 binary units
    servCfg.convFactors.eulFactor = 16.0; //  1 degree  = 16 binary units
    //eul angles don't need a conversion.
    GET_privData(mPriv).sens.measConverter.Initialize(servCfg.convFactors.accFactor, servCfg.convFactors.gyrFactor,  servCfg.convFactors.magFactor, servCfg.convFactors.eulFactor);
    
    // configure the sensor(s)
    
    if(false == GET_privData(mPriv).sendConfing2board(servCfg))
    {
        cleanup();
        return false;
    }
    
    
    if(false == GET_privData(mPriv).initRegulars())
    {
        cleanup();
        return false;
    }
    
    
    if(false == GET_privData(mPriv).res->serviceStart(eomn_serv_category_inertials3))
    {
        yError() << getBoardInfo() << "open() fails to start as service.... cannot continue";
        cleanup();
        return false;
    }
    else
    {
        if(GET_privData(mPriv).isVerbose())
        {
            yDebug() << getBoardInfo() << "open() correctly starts service";
        }
    }

    // build data structure used to handle rx packets
    GET_privData(mPriv).maps.init(servCfg);
    
    
    {   // start the configured sensors. so far, we must keep it in here. later on we can remove this command
        
        uint8_t enable = 1;
        
        eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_inertial3, 0, eoprot_tag_as_inertial3_cmmnds_enable);
        if(false == GET_privData(mPriv).res->setRemoteValue(id32, &enable))
        {
            yError() << getBoardInfo() << "open() fails to command the start transmission of the configured inertials";
            cleanup();
            return false;
        }
    }
    
    GET_privData(mPriv).sens.init(servCfg, getBoardInfo());
    GET_privData(mPriv).setOpen(true);
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
    return  GET_privData(mPriv).sensorState_eo2yarp(eoas_imu_gyr, GET_privData(mPriv).sens.getSensorStatus(sens_index, eoas_imu_gyr));
}

bool embObjIMU::getThreeAxisGyroscopeName(size_t sens_index, std::string &name) const
{
    return GET_privData(mPriv).sens.getSensorName(sens_index, eoas_imu_gyr, name);
}

bool embObjIMU::getThreeAxisGyroscopeFrameName(size_t sens_index, std::string &frameName) const
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
    return  GET_privData(mPriv).sensorState_eo2yarp(eoas_imu_acc, GET_privData(mPriv).sens.getSensorStatus(sens_index, eoas_imu_acc));
}

bool embObjIMU::getThreeAxisLinearAccelerometerName(size_t sens_index, std::string &name) const
{
    return GET_privData(mPriv).sens.getSensorName(sens_index, eoas_imu_acc, name);
}

bool embObjIMU::getThreeAxisLinearAccelerometerFrameName(size_t sens_index, std::string &frameName) const
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
    return  GET_privData(mPriv).sensorState_eo2yarp(eoas_imu_mag, GET_privData(mPriv).sens.getSensorStatus(sens_index, eoas_imu_mag));
}

bool embObjIMU::getThreeAxisMagnetometerName(size_t sens_index, std::string &name) const
{
    return GET_privData(mPriv).sens.getSensorName(sens_index, eoas_imu_mag, name);
}

bool embObjIMU::getThreeAxisMagnetometerFrameName(size_t sens_index, std::string &frameName) const
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
    return  GET_privData(mPriv).sensorState_eo2yarp(eoas_imu_eul, GET_privData(mPriv).sens.getSensorStatus(sens_index, eoas_imu_eul));
}

bool embObjIMU::getOrientationSensorName(size_t sens_index, std::string &name) const
{
    return GET_privData(mPriv).sens.getSensorName(sens_index, eoas_imu_eul, name);
}

bool embObjIMU::getOrientationSensorFrameName(size_t sens_index, std::string &frameName) const
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



#undef DEBUG_PRINT_NONVALIDDATA
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
            yError() << getBoardInfo() << "update(): I have to update " << numofIntem2update << "items, but the " << i << "-th item is null.";
            continue; 
            //NOTE: I signal this strange situation with an arror for debug porpouse...maybe we can convert in in warning when the device is stable....
        }
        uint8_t index;
        eOas_sensor_t type;
        bool validdata =  GET_privData(mPriv).maps.getIndex(data, index, type);
        
        if(!validdata)
        {

#if defined(DEBUG_PRINT_NONVALIDDATA)
            yError("NOT VALID value[%i] is: seq = %d, timestamp = %d, type = %s, id = %d, v= ((%d), %d, %d, %d), status = %x",
                    i,
                    data->seq,
                    data->timestamp,
                    eoas_sensor2string(static_cast<eOas_sensor_t>(data->typeofsensor)),
                    data->id,
                    data->w, data->x, data->y, data->z,
                    data->status.general);
#endif
            continue;
        }

        if(type == eoas_imu_status)
        {
            //updateAllsensorOnSameBoad(data)
            uint8_t canbus, canaddress, i;
            PositionMaps::getCanAddress(data, canbus, canaddress);
            for (uint8_t t=eoas_imu_acc; t<=eoas_imu_status; t++)
            {
                uint8_t i;
                if(GET_privData(mPriv).maps.getIndex(static_cast<eOas_sensor_t>(t), canbus, canaddress, i))
                {
                    GET_privData(mPriv).sens.updateStatus(static_cast<eOas_sensor_t>(t), i, data->status);
                    //yError() << "UPDATE STATUS OF SENSOR " << i << "with type "<< eoas_sensor2string(static_cast<eOas_sensor_t>(t)) << "can port=" << canbus << "can addr=" << canaddress << "status=" << data->status.general;
                }

            }
        }
        else
        {
            GET_privData(mPriv).sens.update(type, index, data);
        }
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
                    yDebug("value[%i] is: seq = %d, timestamp = %d, type = %s, id = %d, v= ((%d), %d, %d, %d), status = %x, pos = %d",
                            i,
                            data->seq,
                            data->timestamp,
                            eoas_sensor2string(static_cast<eOas_sensor_t>(type)),
                            data->id,
                            data->w, data->x, data->y, data->z,
                            data->status.general,
                            pos);
                }
            }

        }

    }
}

// eof

