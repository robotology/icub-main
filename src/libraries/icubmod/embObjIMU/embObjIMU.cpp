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
//using namespace yarp::math;

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

    // verbosewhenok = true;

    // al others:
    nchannels = 0;
    dummy_value = 0;
}

embObjIMU::~embObjIMU()
{
    close();
}

std::string embObjIMU::getBoardInfo(void)
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
        yError() << "FATAL: embObjIMU::sendConfing2board() had an error while calling setcheckRemoteValue() for config in BOARD" << res->getProperties().boardnameString << "with IP" << res->getProperties().ipv4addrString;
        return false;
    }
    else
    {
        if(verbosewhenok)
        {
            yDebug() << "embObjIMU::sendConfing2board() correctly configured enabled sensors with period" << cfg.datarate << "in BOARD" << res->getProperties().boardnameString << "with IP" << res->getProperties().ipv4addrString;
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
        yError() << "embObjIMU::initRegulars() fails to add its variables to regulars: cannot proceed any further";
        return false;
    }
    else
    {
        if(verbosewhenok)
        {
            yDebug() << "embObjIMU::initRegulars() added" << id32v.size() << "regular rops to BOARD" << res->getProperties().boardnameString << "with IP" << res->getProperties().ipv4addrString;
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
        yFatal() << "embObjIMU::open() fails to instantiate ethManager";
        return false;
    }
    
    eOipv4addr_t ipv4addr;
    string boardIPstring;
    string boardName;
    
    if(false == ethManager->verifyEthBoardInfo(config, ipv4addr, boardIPstring, boardName))
    {
        yError() << "embObjIMU::open(): object TheEthManager fails in parsing ETH propertiex from xml file";
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
        yError() << "embObjIMU missing some configuration parameter. Check logs and your config file.";
        return false;
    }
    
    
    // -- instantiate EthResource etc.
    
    res = ethManager->requestResource2(this, config);
    if(NULL == res)
    {
        yError() << "embObjIMU::open() fails because could not instantiate the ethResource for BOARD w/ IP = " << boardIPstring << " ... unable to continue";
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
        yError() << "embObjIMU::open() has an error in call of ethResources::serviceVerifyActivate() for BOARD " << boardName << "IP " << boardIPstring;

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
        yError() << "embObjIMU::open() fails to start as service for BOARD" << res->getProperties().boardnameString << "IP" << res->getProperties().ipv4addrString << ": cannot continue";
        cleanup();
        return false;
    }
    else
    {
        if(verbosewhenok)
        {
            yDebug() << "embObjIMU::open() correctly starts service of BOARD" << res->getProperties().boardnameString << "IP" << res->getProperties().ipv4addrString;
        }
    }

    // build data structure used to handle rx packets
    buildmaps();
    
    
    {   // start the configured sensors. so far, we must keep it in here. later on we can remove this command
        
        uint8_t enable = 1;
        
        eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_inertial3, 0, eoprot_tag_as_inertial_cmmnds_enable);
        if(false == res->setRemoteValue(id32, &enable))
        {
            yError() << "embObjIMU::open() fails to command the start transmission of the configured inertials";
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


#if defined(MORPH_IT_INTO_ANALOGSENSOR)

int embObjIMU::read(yarp::sig::Vector &out)
{
    // This method gives analogdata to the analogServer

    if(false == opened)
    {
        return IAnalogSensor::AS_ERROR;
    }

//    mutex.wait();


//    // errors are not handled for now... it'll always be OK!!
//    if (status != IAnalogSensor::AS_OK)
//    {
//        switch (status)
//        {
//            case IAnalogSensor::AS_OVF:
//            {
//              counterSat++;
//            }  break;
//            case IAnalogSensor::AS_ERROR:
//            {
//              counterError++;
//            } break;
//            case IAnalogSensor::AS_TIMEOUT:
//            {
//             counterTimeout++;
//            } break;
//            default:
//            {
//              counterError++;
//            } break;
//        }
//        mutex.post();
//        return status;
//    }

//    out.resize(analogdata.size());
//    for (size_t k = 0; k<analogdata.size(); k++)
//    {
//        out[k] = analogdata[k];
//    }


 //   mutex.post();

    return IAnalogSensor::AS_OK;

}


//void embObjIMU::resetCounters()
//{
////    counterSat = 0;
////    counterError = 0;
////    counterTimeout = 0;
//}


//void embObjIMU::getCounters(unsigned int &sat, unsigned int &err, unsigned int &to)
//{
////    sat = counterSat;
////    err = counterError;
////    to = counterTimeout;
//}


int embObjIMU::getState(int ch)
{
    printf("getstate\n");
    return AS_OK;
}

int embObjIMU::getChannels()
{
    return 6;
}


int embObjIMU::calibrateSensor()
{
    return AS_OK;
}


int embObjIMU::calibrateSensor(const yarp::sig::Vector& value)
{
    return AS_OK;
}

int embObjIMU::calibrateChannel(int ch)
{
    return AS_OK;
}


int embObjIMU::calibrateChannel(int ch, double v)
{
    return AS_OK;
}

#else

bool embObjIMU::read(Vector &out)
{
#if 0
    if(out.size() != nchannels)
        out.resize(nchannels);

    out.zero();

    // Euler angle
    for(unsigned int i=0; i<3; i++)
    {
        out[i] = dummy_value;
    }

    // accelerations
    for(unsigned int i=0; i<3; i++)
    {
        out[3+i] = accels[i];
    }

    // gyro
    for(unsigned int i=0; i<3; i++)
    {
        out[6+i] = dummy_value;
    }

    // magnetometer
    for(unsigned int i=0; i<3; i++)
    {
        out[9+i] = dummy_value;
    }
#endif

    return true;
}

bool embObjIMU::getChannels(int *nc)
{
    *nc=nchannels;
    return true;
}

bool embObjIMU::calibrate(int ch, double v)
{
    yWarning("Not implemented yet\n");
    return false;
}

yarp::os::Stamp embObjIMU::getLastInputStamp()
{
    return lastStamp;
}

yarp::dev::MAS_status embObjIMU::genericGetStatus(size_t sens_index) const
{
    if (sens_index!=0) {
        return yarp::dev::MAS_status::MAS_ERROR;
    }

    return yarp::dev::MAS_status::MAS_OK;
}

bool embObjIMU::genericGetSensorName(size_t sens_index, yarp::os::ConstString &name) const
{
    if (sens_index!=0) {
        return false;
    }

    name = m_sensorName;
    return true;
}

bool embObjIMU::genericGetFrameName(size_t sens_index, yarp::os::ConstString &frameName) const
{
    if (sens_index!=0) {
        return false;
    }

    frameName = m_frameName;
    return true;
}

size_t embObjIMU::getNrOfThreeAxisGyroscopes() const
{
    return 1;
}

yarp::dev::MAS_status embObjIMU::getThreeAxisGyroscopeStatus(size_t sens_index) const
{
    return genericGetStatus(sens_index);
}

bool embObjIMU::getThreeAxisGyroscopeName(size_t sens_index, yarp::os::ConstString &name) const
{
    return genericGetSensorName(sens_index, name);
}

bool embObjIMU::getThreeAxisGyroscopeFrameName(size_t sens_index, yarp::os::ConstString &frameName) const
{
    return genericGetFrameName(sens_index, frameName);
}

bool embObjIMU::getThreeAxisGyroscopeMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    if (sens_index!=0) {
        return false;
    }

#if 0
    out.resize(3);
    out[0] = dummy_value;
    out[1] = dummy_value;
    out[2] = dummy_value;

    // Workaround for https://github.com/robotology/yarp/issues/1610
    yarp::os::Stamp copyStamp(lastStamp);
    timestamp = copyStamp.getTime();
#endif

    return true;
}

size_t embObjIMU::getNrOfThreeAxisLinearAccelerometers() const
{
    return 1;
}

yarp::dev::MAS_status embObjIMU::getThreeAxisLinearAccelerometerStatus(size_t sens_index) const
{
    return genericGetStatus(sens_index);
}

bool embObjIMU::getThreeAxisLinearAccelerometerName(size_t sens_index, yarp::os::ConstString &name) const
{
    return genericGetSensorName(sens_index, name);
}

bool embObjIMU::getThreeAxisLinearAccelerometerFrameName(size_t sens_index, yarp::os::ConstString &frameName) const
{
    return genericGetFrameName(sens_index, frameName);
}

bool embObjIMU::getThreeAxisLinearAccelerometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    if (sens_index!=0) {
        return false;
    }

    out.resize(3);
    out[0] = accels[0];
    out[1] = accels[1];
    out[2] = accels[2];

    // Workaround for https://github.com/robotology/yarp/issues/1610
    yarp::os::Stamp copyStamp(lastStamp);
    timestamp = copyStamp.getTime();

    return true;
}

size_t embObjIMU::getNrOfThreeAxisMagnetometers() const
{
    return 1;
}

yarp::dev::MAS_status embObjIMU::getThreeAxisMagnetometerStatus(size_t sens_index) const
{
    return genericGetStatus(sens_index);
}

bool embObjIMU::getThreeAxisMagnetometerName(size_t sens_index, yarp::os::ConstString &name) const
{
    return genericGetSensorName(sens_index, name);
}

bool embObjIMU::getThreeAxisMagnetometerFrameName(size_t sens_index, yarp::os::ConstString &frameName) const
{
    return genericGetFrameName(sens_index, frameName);
}

bool embObjIMU::getThreeAxisMagnetometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    if (sens_index!=0) {
        return false;
    }
#if 0
    out.resize(3);
    out[0] = dummy_value;
    out[1] = dummy_value;
    out[2] = dummy_value;

    // Workaround for https://github.com/robotology/yarp/issues/1610
    yarp::os::Stamp copyStamp(lastStamp);
    timestamp = copyStamp.getTime();
#endif
    return true;
}

size_t embObjIMU::getNrOfOrientationSensors() const
{
    return 1;
}

yarp::dev::MAS_status embObjIMU::getOrientationSensorStatus(size_t sens_index) const
{
    return genericGetStatus(sens_index);
}

bool embObjIMU::getOrientationSensorName(size_t sens_index, yarp::os::ConstString &name) const
{
    return genericGetSensorName(sens_index, name);
}

bool embObjIMU::getOrientationSensorFrameName(size_t sens_index, yarp::os::ConstString &frameName) const
{
    return genericGetFrameName(sens_index, frameName);
}

bool embObjIMU::getOrientationSensorMeasureAsRollPitchYaw(size_t sens_index, yarp::sig::Vector& rpy_out, double& timestamp) const
{
    if (sens_index!=0) {
        return false;
    }

#if 0
    rpy_out.resize(3);
    rpy_out[0] = dummy_value;
    rpy_out[1] = dummy_value;
    rpy_out[2] = dummy_value;

    // Workaround for https://github.com/robotology/yarp/issues/1610
    yarp::os::Stamp copyStamp(lastStamp);
    timestamp = copyStamp.getTime();
#endif

    return true;
}

#endif


bool embObjIMU::initialised()
{
    return opened;
}

eth::iethresType_t embObjIMU::type()
{
    return eth::iethres_analoginertial3;
}

// TODO
// we must put the received eOas_inertial3_status_t into a proper place.
// we can detect what data is by looking at eOas_inertial3_status_t::typeofsensor (accel, gyro, etc)
// we need to put eOas_inertial3_status_t::x, ::y, ::z into the relevant vector in a given order
// the order is determined by eOas_inertial3_status_t::id which contains the can address CANx:y, where y = id & 0xf and x = (id >> 4) + 1
// the solution is to build for each sensor type a uint8_t map[2][16] which maps the address into a position of teh vector.
// we build these maps when we send the service configuration down to the board.



bool embObjIMU::update(eOprotID32_t id32, double timestamp, void* rxdata)
{
//    mutex.wait();

    // important note: use mutex to protect what we write in here which is later read by the ::get* methods

//    mutex.post();

    static int prog = 1;
    static double prevtime =  yarp::os::Time::now();

    double delta = yarp::os::Time::now() - prevtime;
    double millidelta = 1000.0 *delta;
    long milli = static_cast<long>(millidelta);


    eOas_inertial3_status_t *i3s  = (eOas_inertial3_status_t*)rxdata;

    EOconstarray* arrayofvalues = eo_constarray_Load(reinterpret_cast<const EOarray*>(&i3s->arrayofdata));

    uint8_t n = eo_constarray_Size(arrayofvalues);

    //EOarray * array = reinterpret_cast<EOarray*>(&i3s->arrayofdata);

    //uint8_t n = eo_array_Size(array);

    //yDebug() << "embObjIMU::update(): size =" << n << "values";

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
                    getIndex(data, pos);
                    yDebug("value[%i] is: seq = %d, timestamp = %d, type = %s, id = %d, v= ((%d), %d, %d, %d), status = %d, pos = %d",
                           i,
                           data->seq,
                           data->timestamp,
                           eoas_sensor2string(static_cast<eOas_sensor_t>(data->typeofsensor)),
                           data->id,
                           data->w, data->x, data->y, data->z,
                           data->status,
                           pos);
                }
            }

        }

    }


    return true;
}


bool embObjIMU::buildmaps(void)
{
    memset(positionmap, 0xff, sizeof(positionmap));
    memset(numberof, 0, sizeof(numberof));

    // and now we list the service config to fill the map withe proper indices.
    // tobedone


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
                    yDebug() << "one more" << eoas_sensor2string(static_cast<eOas_sensor_t>(des->typeofsensor));
                }
                else if(des->on.any.place == eobrd_place_eth)
                {
                    // must manage the case of gyro on ems
                }
            }
        }

    }

    return true;
}


bool embObjIMU::getIndex(const eOas_inertial3_data_t *data, uint8_t &index)
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

    return (0xff == index) ? false : true;

}

// eof

