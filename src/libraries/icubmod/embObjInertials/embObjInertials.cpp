
// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2012 Robotcub Consortium
* Author: Alberto Cardellino
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

// general purpose stuff.
#include <string>
#include <iostream>
#include <string.h>

// Yarp Includes
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <stdarg.h>
#include <stdio.h>
#include <yarp/dev/PolyDriver.h>
#include <ace/config.h>
#include <ace/Log_Msg.h>


// specific to this device driver.
#include <embObjInertials.h>
#include <ethManager.h>
#include <yarp/os/LogStream.h>
#include "EoAnalogSensors.h"
#include "EOnv_hid.h"

#include "EoProtocol.h"
#include "EoProtocolMN.h"
#include "EoProtocolAS.h"

#include <yarp/os/NetType.h>

#ifdef WIN32
#pragma warning(once:4355)
#endif



using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;


inline bool NOT_YET_IMPLEMENTED(const char *txt)
{
    yWarning() << std::string(txt) << " not yet implemented for embObjInertials\n";
    return false;
}

// generic function that checks is key1 is present in input bottle and that the result has size elements
// return true/false
bool embObjInertials::extractGroup(Bottle &input, Bottle &out, const std::string &key1, const std::string &txt, int size)
{
    size++;  // size includes also the name of the parameter
    Bottle &tmp=input.findGroup(key1.c_str(), txt.c_str());
    if (tmp.isNull())
    {
        yError ("%s not found\n", key1.c_str());
        return false;
    }

    if(tmp.size()!=size)
    {
        yError("%s incorrect number of entries\n", key1.c_str());
        return false;
    }

    out=tmp;

    return true;
}


bool embObjInertials::fromConfig(yarp::os::Searchable &_config)
{
    Bottle xtmp;

    // Analog Sensor stuff
    Bottle config = _config.findGroup("GENERAL");
    if (!extractGroup(config, xtmp, "Period","transmitting period of the sensors", 1))
    {
        yError() << "embObjInertials Using default value = 0 (disabled)";
        _period = 0;
    }
    else
    {
        _period = xtmp.get(1).asInt();
        yDebug() << "embObjInertials::fromConfig() detects embObjInertials Using value of" << _period;
    }


#if 0
    if (!extractGroup(config, xtmp, "NumberOfSensors","Number of sensors managed", 1))
    {
        yWarning("embObjInertials: Using default value = 1 for _numofsensors\n");
        _numofsensors = 1;
    }
    else
    {
        _numofsensors = xtmp.get(1).asInt();
    }
#endif


    _numofsensors = 1;

    // if we have a AS_Type_INERTIAL_MTB, then we may have more than one. for sure not zero.

    {
        Bottle tmp;
        _numofsensors = 0;

        tmp = config.findGroup("enabledAccelerometers");
        _numofsensors += (tmp.size()-1); // sensors holds strings "enabledAccelerometers" and then all the others, thus i need a -1

        tmp = config.findGroup("enabledGyroscopes");
        _numofsensors += (tmp.size()-1); // sensors holds strings "enabledGyroscopes" and then all the others, thus i need a -1

#if 0
        if (!extractGroup(config, xtmp, "enabledAccelerometers", "Position of managed sensors expressed as strings", 1))
        {
            yWarning("embObjInertials: cannot find enabledAccelerometers\n");
            _numofsensors = 1;
        }
        else
        {
            _numofsensors = xtmp.size();
        }
#endif

    }


    return true;
}


embObjInertials::embObjInertials()
{
    _numofsensors = 1;
    _period=0;

    analogdata.resize(0);

    memset(_fromInertialPos2DataIndexAccelerometers, 255, sizeof(_fromInertialPos2DataIndexAccelerometers));
    memset(_fromInertialPos2DataIndexGyroscopes, 255, sizeof(_fromInertialPos2DataIndexGyroscopes));


    timeStamp = 0;
    counterSat = 0;
    counterError = 0;
    counterTimeout = 0;

    status = IAnalogSensor::AS_OK;

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


embObjInertials::~embObjInertials()
{
    analogdata.resize(0);
}

bool embObjInertials::initialised()
{
    return opened;
}

bool embObjInertials::open(yarp::os::Searchable &config)
{
    // - first thing to do is verify if the eth manager is available. then i parse info about the eth board.

    ethManager = TheEthManager::instance();
    if(NULL == ethManager)
    {
        yFatal() << "embObjInertials::open() fails to instantiate ethManager";
        return false;
    }


    if(false == ethManager->verifyEthBoardInfo(config, NULL, boardIPstring, sizeof(boardIPstring)))
    {
        yError() << "embObjInertials::open(): object TheEthManager fails in parsing ETH propertiex from xml file";
        return false;
    }
    // add specific info about this device ...

    // when we split this device in three ones ... _as_type will be lost and we can move code tagged with tag__XXX0123_ in here


    // - now all other things


//    bool ret = false;

//    std::string str;
//    if(config.findGroup("GENERAL").find("verbose").asBool())
//        str=config.toString().c_str();
//    else
//        str="\n";
//    yTrace() << str;

    // read stuff from config file
    if(!fromConfig(config))
    {
        yError() << "embObjInertials missing some configuration parameter. Check logs and your config file.";
        return false;
    }

    // and prepare analogdata
    {
        // must be of size: 2+inertials_Channels*_numofsensors, and 0.0-initted
        analogdata.resize(2+inertials_Channels*_numofsensors, 0.0);
        // but the first two positions contain: number of sensors, number of channels
        analogdata[0] = _numofsensors;
        analogdata[1] = inertials_Channels;
    }


    // -- instantiate EthResource etc.

    res = ethManager->requestResource2(this, config);
    if(NULL == res)
    {
        yError() << "embObjInertials::open() fails because could not instantiate the ethResource for BOARD w/ IP = " << boardIPstring << " ... unable to continue";
        return false;
    }


    if(!res->verifyEPprotocol(eoprot_endpoint_analogsensors))
    {
        cleanup();
        return false;
    }


    if(false == res->serviceVerifyActivate(eomn_serv_category_inertials, NULL))
    {
        yError() << "embObjInertials::open() has an error in call of ethResources::serviceVerifyActivate() for BOARD" << res->getName() << "IP" << res->getIPv4string();
        cleanup();
        return false;
    }


    // configure the service: aka, send to the remote board information about the whereabouts of the can boards mais, strain, mtb which offers the service.

    if(false == configServiceInertials(config))
    {
        cleanup();
        return false;
    }

    // configure the sensor(s)

    if(false == sendConfig2MTBboards(config))
    {
        cleanup();
        return false;
    }


    if(false == initRegulars())
    {
        cleanup();
        return false;
    }


    if(false == res->serviceStart(eomn_serv_category_inertials))
    {
        yError() << "embObjInertials::open() fails to start as service for BOARD" << res->getName() << "IP" << res->getIPv4string() << ": cannot continue";
        cleanup();
        return false;
    }
    else
    {
        if(verbosewhenok)
        {
            yDebug() << "embObjInertials::open() correctly starts service of BOARD" << res->getName() << "IP" << res->getIPv4string();
        }
    }



    {   // start the configured sensors. so far, we must keep it in here. later on we can remove this command

        eOas_inertial_commands_t startCommand = {0};
        startCommand.enable = 1;

        uint32_t id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_inertial, 0, eoprot_tag_as_inertial_cmmnds_enable);
        if(!res->addSetMessage(id32, (uint8_t*) &startCommand))
        {
            yError() << "embObjInertials::open() fails to command the start transmission of the configured inertials";
            cleanup();
            return false;
        }
    }

    opened = true;
    return true;
}


bool embObjInertials::isEpManagedByBoard()
{    
    return res->isEPsupported(eoprot_endpoint_analogsensors);
}





bool embObjInertials::configServiceInertials(Searchable& globalConfig)
{
    // find SERVICES-INERTIALS. if not found, exit mildly from function. in a first stage the remote eth board will already be configured.

    Bottle tmp = globalConfig.findGroup("SERVICES");
    Bottle config = tmp.findGroup("INERTIALS");


    // prepare the config of the inertial sensors: datarate and the mask of the enabled ones.

    eOas_inertial_serviceconfig_t inertialServiceConfig = {0}; // by this initialisation, there is no sensor at all in the two can buses



    // meglio sarebbe mettere un controllo sul fatto che ho trovato InertialsCAN1mapping e che ha size coerente

    Bottle canmap;
    int numofentries = 0;

    // CAN1
    canmap = config.findGroup("InertialsCAN1mapping");
    numofentries = canmap.size()-1;

    for(int i=0; i<numofentries; i++)
    {
        eOas_inertial_position_t pos = eoas_inertial_pos_none;

        yarp::os::ConstString strpos = canmap.get(i+1).asString();

        pos = getLocationOfInertialSensor(strpos);  // prendi la posizione dalla stringa strpos: fai una funzione apposita
        inertialServiceConfig.canmapofsupportedsensors[0][i] = pos;
    }

    // CAN2
    canmap = config.findGroup("InertialsCAN2mapping");
    numofentries = canmap.size()-1;

    for(int i=0; i<numofentries; i++)
    {
        eOas_inertial_position_t pos = eoas_inertial_pos_none;

        yarp::os::ConstString strpos = canmap.get(i+1).asString();

        pos = getLocationOfInertialSensor(strpos);  // prendi la posizione dalla stringa strpos: fai una funzione apposita
        inertialServiceConfig.canmapofsupportedsensors[1][i] = pos;
    }

    // configure the service

    eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_inertial, 0, eoprot_tag_as_inertial_config_service);
    if(false == res->setRemoteValueUntilVerified(id32, &inertialServiceConfig, sizeof(inertialServiceConfig), 10, 0.010, 0.050, 2))
    {
        yError() << "FATAL: embObjInertials::configServiceInertials() had an error while calling setRemoteValueUntilVerified() for config in BOARD" << res->getName() << "with IP" << res->getIPv4string();
        return false;
    }
    else
    {
        if(verbosewhenok)
        {
            yDebug() << "embObjInertials::configServiceInertials() correctly configured the service in BOARD" << res->getName() << "with IP" << res->getIPv4string();
        }
    }

    return true;
}


bool embObjInertials::sendConfig2MTBboards(Searchable& globalConfig)
{
    eOprotID32_t id32 = eo_prot_ID32dummy;

    // configuration specific for skin-inertial device

    Bottle config = globalConfig.findGroup("GENERAL");


    // prepare the config of the inertial sensors: datarate and the mask of the enabled ones.

    eOas_inertial_sensorsconfig_t inertialSensorsConfig = {0};

    inertialSensorsConfig.accelerometers = 0;
    inertialSensorsConfig.gyroscopes = 0;
    inertialSensorsConfig.datarate = _period;


    // meglio sarebbe mettere un controllo sul fatto che ho trovato enabledAccelerometers e che ha size coerente

    Bottle sensors;

    sensors = config.findGroup("enabledAccelerometers");

    int numofaccelerometers = sensors.size()-1;    // sensors holds strings "enabledAccelerometers" and then all the others, thus i need a -1

    for(int i=0; i<numofaccelerometers; i++)
    {
        eOas_inertial_position_t pos = eoas_inertial_pos_none;

        yarp::os::ConstString strpos = sensors.get(i+1).asString();

        pos = getLocationOfInertialSensor(strpos);  // prendi la posizione dalla stringa strpos: fai una funzione apposita

        if(eoas_inertial_pos_none != pos)
        {
            _fromInertialPos2DataIndexAccelerometers[pos] = i;
            inertialSensorsConfig.accelerometers   |= EOAS_ENABLEPOS(pos);
        }
    }

    sensors = config.findGroup("enabledGyroscopes");

    int numofgyroscopess = sensors.size()-1;    // sensors holds strings "enabledGyroscopes" and then all the others, thus i need a -1

    for(int i=0; i<numofgyroscopess; i++)
    {
        eOas_inertial_position_t pos = eoas_inertial_pos_none;

        yarp::os::ConstString strpos = sensors.get(i+1).asString();

        pos = getLocationOfInertialSensor(strpos);  // prendi la posizione dalla stringa strpos: fai una funzione apposita

        if(eoas_inertial_pos_none != pos)
        {
            _fromInertialPos2DataIndexGyroscopes[pos] = numofaccelerometers+i;
            inertialSensorsConfig.gyroscopes   |= EOAS_ENABLEPOS(pos);
        }
    }

    // configure the sensors (datarate and position)

    id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_inertial, 0, eoprot_tag_as_inertial_config_sensors);
    if(false == res->setRemoteValueUntilVerified(id32, &inertialSensorsConfig, sizeof(inertialSensorsConfig), 10, 0.010, 0.050, 2))
    {
        yError() << "FATAL: embObjInertials::sendConfig2MTBboards() had an error while calling setRemoteValueUntilVerified() for config in BOARD" << res->getName() << "with IP" << res->getIPv4string();
        return false;
    }
    else
    {
        if(verbosewhenok)
        {
            yDebug() << "embObjInertials::sendConfig2MTBboards() correctly configured enabled sensors with period" << _period << "in BOARD" << res->getName() << "with IP" << res->getIPv4string();
        }
    }


//    // start the configured sensors

//    eOmc_inertial_commands_t startCommand = {0};
//    startCommand.enable = 1;

//    id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_inertial, 0, eoprot_tag_as_inertial_cmmnds_enable);
//    if(!res->addSetMessage(id32, (uint8_t*) &startCommand))
//    {
//        yError() << "ethResources::sendConfig2MTBboards() fails to command the start transmission of the inertials";
//        return false;
//    }

    return true;
}



eOas_inertial_position_t embObjInertials::getLocationOfInertialSensor(yarp::os::ConstString &strpos)
{
    eOas_inertial_position_t ret = eoas_inertial_pos_none;

    if(strpos == "none")
    {
        ret = eoas_inertial_pos_none;
    }

    else if(strpos == "l_hand")
    {
        ret = eoas_inertial_pos_l_hand;
    }
    else if(strpos == "l_forearm_1")
    {
        ret = eoas_inertial_pos_l_forearm_1;
    }
    else if(strpos == "l_forearm_2")
    {
        ret = eoas_inertial_pos_l_forearm_2;
    }
    else if(strpos == "l_upper_arm_1")
    {
        ret = eoas_inertial_pos_l_upper_arm_1;
    }
    else if(strpos == "l_upper_arm_2")
    {
        ret = eoas_inertial_pos_l_upper_arm_2;
    }
    else if(strpos == "l_upper_arm_3")
    {
        ret = eoas_inertial_pos_l_upper_arm_3;
    }
    else if(strpos == "l_upper_arm_4")
    {
        ret = eoas_inertial_pos_l_upper_arm_4;
    }
    else if(strpos == "l_foot_1")
    {
        ret = eoas_inertial_pos_l_foot_1;
    }
    else if(strpos == "l_foot_2")
    {
        ret = eoas_inertial_pos_l_foot_2;
    }
    else if(strpos == "l_lower_leg_1")
    {
        ret = eoas_inertial_pos_l_lower_leg_1;
    }
    else if(strpos == "l_lower_leg_2")
    {
        ret = eoas_inertial_pos_l_lower_leg_2;
    }
    else if(strpos == "l_lower_leg_3")
    {
        ret = eoas_inertial_pos_l_lower_leg_3;
    }
    else if(strpos == "l_lower_leg_4")
    {
        ret = eoas_inertial_pos_l_lower_leg_4;
    }
    else if(strpos == "l_upper_leg_1")
    {
        ret = eoas_inertial_pos_l_upper_leg_1;
    }
    else if(strpos == "l_upper_leg_2")
    {
        ret = eoas_inertial_pos_l_upper_leg_2;
    }
    else if(strpos == "l_upper_leg_3")
    {
        ret = eoas_inertial_pos_l_upper_leg_3;
    }
    else if(strpos == "l_upper_leg_4")
    {
        ret = eoas_inertial_pos_l_upper_leg_4;
    }
    else if(strpos == "l_upper_leg_5")
    {
        ret = eoas_inertial_pos_l_upper_leg_5;
    }
    else if(strpos == "l_upper_leg_6")
    {
        ret = eoas_inertial_pos_l_upper_leg_6;
    }
    else if(strpos == "l_upper_leg_7")
    {
        ret = eoas_inertial_pos_l_upper_leg_7;
    }

    else if(strpos == "r_hand")
    {
        ret = eoas_inertial_pos_r_hand;
    }
    else if(strpos == "r_forearm_1")
    {
        ret = eoas_inertial_pos_r_forearm_1;
    }
    else if(strpos == "r_forearm_2")
    {
        ret = eoas_inertial_pos_r_forearm_2;
    }
    else if(strpos == "r_upper_arm_1")
    {
        ret = eoas_inertial_pos_r_upper_arm_1;
    }
    else if(strpos == "r_upper_arm_2")
    {
        ret = eoas_inertial_pos_r_upper_arm_2;
    }
    else if(strpos == "r_upper_arm_3")
    {
        ret = eoas_inertial_pos_r_upper_arm_3;
    }
    else if(strpos == "r_upper_arm_4")
    {
        ret = eoas_inertial_pos_r_upper_arm_4;
    }
    else if(strpos == "r_foot_1")
    {
        ret = eoas_inertial_pos_r_foot_1;
    }
    else if(strpos == "r_foot_2")
    {
        ret = eoas_inertial_pos_r_foot_2;
    }
    else if(strpos == "r_lower_leg_1")
    {
        ret = eoas_inertial_pos_r_lower_leg_1;
    }
    else if(strpos == "r_lower_leg_2")
    {
        ret = eoas_inertial_pos_r_lower_leg_2;
    }
    else if(strpos == "r_lower_leg_3")
    {
        ret = eoas_inertial_pos_r_lower_leg_3;
    }
    else if(strpos == "r_lower_leg_4")
    {
        ret = eoas_inertial_pos_r_lower_leg_4;
    }
    else if(strpos == "r_upper_leg_1")
    {
        ret = eoas_inertial_pos_r_upper_leg_1;
    }
    else if(strpos == "r_upper_leg_2")
    {
        ret = eoas_inertial_pos_r_upper_leg_2;
    }
    else if(strpos == "r_upper_leg_3")
    {
        ret = eoas_inertial_pos_r_upper_leg_3;
    }
    else if(strpos == "r_upper_leg_4")
    {
        ret = eoas_inertial_pos_r_upper_leg_4;
    }
    else if(strpos == "r_upper_leg_5")
    {
        ret = eoas_inertial_pos_r_upper_leg_5;
    }
    else if(strpos == "r_upper_leg_6")
    {
        ret = eoas_inertial_pos_r_upper_leg_6;
    }
    else if(strpos == "r_upper_leg_7")
    {
        ret = eoas_inertial_pos_r_upper_leg_7;
    }


    return(ret);

}


bool embObjInertials::initRegulars()
{
    // configure regular rops

    vector<eOprotID32_t> id32v(0);
    eOprotID32_t id32 = eo_prot_ID32dummy;

    // we need to choose the protoid to put inside the vector

    id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_inertial, 0, eoprot_tag_as_inertial_status);

    // put it inside vector

    id32v.push_back(id32);


    if(false == res->serviceSetRegulars(eomn_serv_category_inertials, id32v))
    {
        yError() << "embObjInertials::initRegulars() fails to add its variables to regulars: cannot proceed any further";
        return false;
    }
    else
    {
        if(verbosewhenok)
        {
            yDebug() << "embObjInertials::initRegulars() added" << id32v.size() << "regular rops to BOARD" << res->getName() << "with IP" << res->getIPv4string();
            char nvinfo[128];
            for(int r=0; r<id32v.size(); r++)
            {
                uint32_t item = id32v.at(r);
                eoprot_ID2information(item, nvinfo, sizeof(nvinfo));
                yDebug() << "\t it added regular rop for" << nvinfo;
            }
        }
    }

    return true;
}


/*! Read a vector from the sensor.
 * @param out a vector containing the sensor's last readings.
 * @return AS_OK or return code. AS_TIMEOUT if the sensor timed-out.
 **/
int embObjInertials::read(yarp::sig::Vector &out)
{
    // This method gives analogdata to the analogServer

    if(false == opened)
    {
        return false;
    }

    mutex.wait();


    // errors are not handled for now... it'll always be OK!!
    if (status != IAnalogSensor::AS_OK)
    {
        switch (status)
        {
            case IAnalogSensor::AS_OVF:
            {
              counterSat++;
            }  break;
            case IAnalogSensor::AS_ERROR:
            {
              counterError++;
            } break;
            case IAnalogSensor::AS_TIMEOUT:
            {
             counterTimeout++;
            } break;
            default:
            {
              counterError++;
            } break;
        }
        mutex.post();
        return status;
    }

    out.resize(analogdata.size());
    for(int k=0; k<analogdata.size(); k++)
    {
        out[k] = analogdata[k];
    }


    mutex.post();

    return status;

}


void embObjInertials::resetCounters()
{
    counterSat = 0;
    counterError = 0;
    counterTimeout = 0;
}


void embObjInertials::getCounters(unsigned int &sat, unsigned int &err, unsigned int &to)
{
    sat = counterSat;
    err = counterError;
    to = counterTimeout;
}


int embObjInertials::getState(int ch)
{
    printf("getstate\n");
    return AS_OK;
}

int embObjInertials::getChannels()
{
    return analogdata.size();
}


int embObjInertials::calibrateSensor()
{
    return AS_OK;
}


int embObjInertials::calibrateSensor(const yarp::sig::Vector& value)
{
    return AS_OK;
}

int embObjInertials::calibrateChannel(int ch)
{
    return AS_OK;
}


int embObjInertials::calibrateChannel(int ch, double v)
{
    return AS_OK;
}


iethresType_t embObjInertials::type()
{
    return iethres_analoginertial;
}


bool embObjInertials::update(eOprotID32_t id32, double timestamp, void* rxdata)
{
    id32 = id32;
    timestamp = timestamp;

    if(false == opened)
    {
        return false;
    }

    eOas_inertial_status_t *status = (eOas_inertial_status_t*) rxdata;

    if((eoas_inertial_pos_none == status->data.position) || (status->data.position >= eoas_inertial_pos_max_numberof))
    {   // we dont have any info to manage or the received position is WRONG
        return(true);
    }

    mutex.wait();

    uint8_t dataindex = 255;

    if(eoas_inertial_type_accelerometer == status->data.type)
    {
        dataindex = _fromInertialPos2DataIndexAccelerometers[status->data.position];
    }
    else if(eoas_inertial_type_gyroscope == status->data.type)
    {
        dataindex = _fromInertialPos2DataIndexGyroscopes[status->data.position];
    }


    if(255 != dataindex)
    {
        // we now use dataindex to offset the array.
        uint8_t firstpos = 2 + dataindex*6;

        analogdata[firstpos+0] = (double) status->data.position;
        analogdata[firstpos+1] = (double) status->data.type;
        analogdata[firstpos+2] = (double) status->data.timestamp;

        analogdata[firstpos+3] = (double) status->data.x;
        analogdata[firstpos+4] = (double) status->data.y;
        analogdata[firstpos+5] = (double) status->data.z;
    }

    mutex.post();

    return true;
}


bool embObjInertials::close()
{
    opened = false;

    cleanup();
    return true;
}

void embObjInertials::cleanup(void)
{
    if(ethManager == NULL) return;

    int ret = ethManager->releaseResource2(res, this);
    res = NULL;
    if(ret == -1)
        ethManager->killYourself();
}

// eof

