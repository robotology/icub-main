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
    return true;
}

bool embObjIMU::initRegulars(void)
{
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
    
    if(false == res->serviceVerifyActivate(eomn_serv_category_inertials, servparam, 5.0))
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
    
    
    if(false == res->serviceStart(eomn_serv_category_inertials))
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
    
    
    {   // start the configured sensors. so far, we must keep it in here. later on we can remove this command
        
        uint8_t enable = 1;
        
        eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_inertial3, 0, 0); //TODO
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

bool embObjIMU::read(Vector &out)
{
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

    out.resize(3);
    out[0] = dummy_value;
    out[1] = dummy_value;
    out[2] = dummy_value;

    // Workaround for https://github.com/robotology/yarp/issues/1610
    yarp::os::Stamp copyStamp(lastStamp);
    timestamp = copyStamp.getTime();

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

    out.resize(3);
    out[0] = dummy_value;
    out[1] = dummy_value;
    out[2] = dummy_value;

    // Workaround for https://github.com/robotology/yarp/issues/1610
    yarp::os::Stamp copyStamp(lastStamp);
    timestamp = copyStamp.getTime();

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

    rpy_out.resize(3);
    rpy_out[0] = dummy_value;
    rpy_out[1] = dummy_value;
    rpy_out[2] = dummy_value;

    // Workaround for https://github.com/robotology/yarp/issues/1610
    yarp::os::Stamp copyStamp(lastStamp);
    timestamp = copyStamp.getTime();

    return true;
}


bool embObjIMU::initialised()
{
    return opened;
}
eth::iethresType_t embObjIMU::type()
{
    return eth::iethres_analoginertial3;
}
bool embObjIMU::update(eOprotID32_t id32, double timestamp, void* rxdata)
{
    return true;
}