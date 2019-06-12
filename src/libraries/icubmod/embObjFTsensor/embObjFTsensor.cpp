
// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2012 Robotcub Consortium
* Author: Valentina Gaggero
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

// general purpose stuff.
#include <string>
#include <iostream>
#include <string.h>

// Yarp Includes
#include <yarp/os/Time.h>
#include <yarp/os/Semaphore.h>


// specific to this device driver.
#include "embObjFTsensor.h"
#include "eo_ftsens_privData.h"

#include "EoAnalogSensors.h"
#include "EOconstarray.h"
#include "EoProtocolAS.h"


#ifdef WIN32
#pragma warning(once:4355)
#endif




using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;


#define GET_privData(x) (*((static_cast<eo_ftsens_privData*>(x))))

embObjFTsensor::embObjFTsensor()
{
    mPriv = new eo_ftsens_privData("embObjFTsensor");
}


embObjFTsensor::~embObjFTsensor()
{
    close();
    delete &GET_privData(mPriv);
}


std::string embObjFTsensor::getBoardInfo(void) const
{
    return GET_privData(mPriv).getBoardInfo();
}

bool embObjFTsensor::initialised()
{
    return GET_privData(mPriv).isOpen();
}


bool embObjFTsensor::enableTemperatureTransmission(bool enable)
{
    uint8_t cmd;
    (enable) ? cmd =1: cmd=0;

    eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_temperature, 0, eoprot_tag_as_temperature_cmmnds_enable);
    if(false == GET_privData(mPriv).res->setRemoteValue(id32, &cmd))
    {
        yError() << getBoardInfo() << "fails send command enableTemperatureTransmission(" << enable << ")";
        return false;
    }
    return true;
}

//----------------------- DeviceDriver -------------------

bool embObjFTsensor::open(yarp::os::Searchable &config)
{
    // - first thing to do is verify if the eth manager is available then i parse info about the eth board.

    if(! GET_privData(mPriv).prerareEthService(config, this))
        return false;

    // read stuff from config file
    
    servConfigFTsensor_t serviceConfig;
    if(!GET_privData(mPriv).fromConfig(config, serviceConfig))
        return false;

    if(!GET_privData(mPriv).res->verifyEPprotocol(eoprot_endpoint_analogsensors))
    {
        cleanup();
        return false;
    }


#if defined(EMBOBJSTRAIN_USESERVICEPARSER)

    //Fill temperature service data in servparamtemp: some of these data are copied from the serviceconfig of ft because both services used tha same board.
    eOmn_serv_parameter_t servparamtemp;
    bool ret = GET_privData(mPriv).fillTemperatureEthServiceInfo(serviceConfig.ethservice, servparamtemp);
    if(!ret)
        return false;

    const eOmn_serv_parameter_t* servparamtemp_ptr = &servparamtemp;
    const eOmn_serv_parameter_t* servparamstrain = &serviceConfig.ethservice;

#else
    const eOmn_serv_parameter_t* servparamstrain = NULL;
    const eOmn_serv_parameter_t* servparamtemp_ptr = NULL;
#endif

    if(false == GET_privData(mPriv).res->serviceVerifyActivate(eomn_serv_category_strain, servparamstrain, 5.0))
    {
        yError() << getBoardInfo() << "open() has an error in call of ethResources::serviceVerifyActivate()";
        cleanup();
        return false;
    }

    if(false == GET_privData(mPriv).res->serviceVerifyActivate(eomn_serv_category_temperatures, servparamtemp_ptr, 5.0))
    {
        yError() << getBoardInfo() << "open() has an error in call of ethResources::serviceVerifyActivate()";
        cleanup();
        return false;
    }

    // we always prepare the fullscales.
    if(false == GET_privData(mPriv).fillScaleFactor(serviceConfig))
    {
        yError() << getBoardInfo() << "open() has failed in calling  embObjFTsensor::fillScaleFactor()";
        return false;
    }

    if(false == GET_privData(mPriv).sendConfig2Strain(serviceConfig))
    {
        cleanup();
        return false;
    }

    if(false == GET_privData(mPriv).initRegulars(serviceConfig))
    {
        cleanup();
        return false;
    }


    if(false == GET_privData(mPriv).res->serviceStart(eomn_serv_category_strain))
    {
        yError() << getBoardInfo() << "open() fails to start service strain";
        cleanup();
        return false;
    }
    else
    {
        if(GET_privData(mPriv).isVerbose())
        {
            yDebug()  << getBoardInfo() << "open() correctly starts as service strain";
        }
    }

    if(false == GET_privData(mPriv).res->serviceStart(eomn_serv_category_temperatures))
    {
        yError() << getBoardInfo() << "open() fails to start service temperature";
        cleanup();
        return false;
    }
    else
    {
        if(GET_privData(mPriv).isVerbose())
        {
            yDebug()  << getBoardInfo() << "open() correctly starts as service temperature";
        }
    }

    // start the configured sensors. so far, we must keep it in here. later on we can remove this command
    enableTemperatureTransmission(true);

    GET_privData(mPriv).setOpen(true);
    return true;
}

bool embObjFTsensor::close()
{
    cleanup();
    return true;
}

void embObjFTsensor::cleanup(void)
{

    // disable temperature
    enableTemperatureTransmission(false);

    GET_privData(mPriv).cleanup(static_cast <eth::IethResource*> (this));
}



//-------------------------------  IethResource --------------------------------
bool embObjFTsensor::update(eOprotID32_t id32, double timestamp, void* rxdata)
{
    bool ret = false;

    if(false == GET_privData(mPriv).isOpen())
    {
        return ret;;
    }
    eOprotEntity_t entity = eoprot_ID2entity(id32);

    switch(entity)
    {
        case eoas_entity_strain:
        {
            ret = updateStrainValues(id32, timestamp, rxdata);
        }break;

        case eoas_entity_temperature:
        {
            ret = updateTemperatureValues(id32, timestamp, rxdata);
        }break;
        default:
        {
            ret = false;
            yError() << getBoardInfo() << "update() failed ";
        }
    };
    return ret;
}

bool embObjFTsensor::updateStrainValues(eOprotID32_t id32, double timestamp, void* rxdata)
{
    id32 = id32;
    timestamp = timestamp;


    // called by feat_manage_analogsensors_data() which is called by:
    // eoprot_fun_UPDT_as_strain_status_calibratedvalues() or eoprot_fun_UPDT_as_strain_status_uncalibratedvalues()
    // the void* parameter inside this function is a eOas_arrayofupto12bytes_t*
    // and can be treated as a EOarray

    EOarray *array = (EOarray*)rxdata;
    uint8_t size = eo_array_Size(array);
    uint8_t itemsize = eo_array_ItemSize(array); // marco.accame: must be 2, as the code after uses this convention
    if((0 == size) || (2 != itemsize))
    {
        return false;
    }

    // lock analogdata
    GET_privData(mPriv).mutex.wait();
    GET_privData(mPriv).timestampAnalogdata = yarp::os::Time::now();
    for (size_t k = 0; k< GET_privData(mPriv).analogdata.size(); k++)
    {
        // Get the kth element of the array as a 2 bytes msg
        char* tmp = (char*) eo_array_At(array, k);
        // marco.accame: i am nervous about iterating for strain_Channels instead of size of array....
        //               thus i add a protection. if k goes beyond size of array, eo_array_At() returns NULL.
        if(NULL != tmp)
        {
            uint8_t msg[2] = {0};
            memcpy(msg, tmp, 2);
            // Got from canBusMotionControl
            GET_privData(mPriv).analogdata[k] = (short)( ( (((unsigned short)(msg[1]))<<8)+msg[0]) - (unsigned short) (0x8000) );

            if(true ==  GET_privData(mPriv).useCalibValues)
            {
                GET_privData(mPriv).analogdata[k] =  GET_privData(mPriv).analogdata[k]* GET_privData(mPriv).scaleFactor[k]/float(0x8000);
            }
        }
    }

    // unlock analogdata
    GET_privData(mPriv).mutex.post();

    return true;
}

bool embObjFTsensor::updateTemperatureValues(eOprotID32_t id32, double timestamp, void* rxdata)
{
    eOas_temperature_status_t *temp_st = (eOas_temperature_status_t *)rxdata;

    EOconstarray* arrayofvalues = eo_constarray_Load(reinterpret_cast<const EOarray*>(&(temp_st->arrayofdata)));

    uint8_t numofIntem2update = eo_constarray_Size(arrayofvalues);

    if(numofIntem2update>1)
        yError() << getBoardInfo() << "updateTemperature: I expect 1 item, but I received " << numofIntem2update ;

    for(int i=0; i<numofIntem2update; i++)
    {
        eOas_temperature_data_t *data = (eOas_temperature_data_t*) eo_constarray_At(arrayofvalues, i);
        if(data == NULL)
        {
            yError() << getBoardInfo() << "update(): I have to update " << numofIntem2update << "items, but the " << i << "-th item is null.";
            continue;
            //NOTE: I signal this strange situation with an arror for debug porpouse...maybe we can convert in in warning when the device is stable....
        }
        else
        {
            //yDebug() << "embObjFTStrain" << getBoardInfo() << ": val "<< i<< "/" << numofIntem2update << ": value=" << data->value << ", time=" << data->timestamp;
            GET_privData(mPriv).mutex.wait();
            GET_privData(mPriv).lastTemperature = static_cast<float>(data->value);
            GET_privData(mPriv).timestampTemperature = yarp::os::Time::now();
            GET_privData(mPriv).mutex.post();
        }
    }
    return true;
}


// -----------------------------  yarp::dev::IAnalogSensor --------------------------------------


/*! Read a vector from the sensor.
 * @param out a vector containing the sensor's last readings.
 * @return AS_OK or return code. AS_TIMEOUT if the sensor timed-out.
 **/
int embObjFTsensor::read(yarp::sig::Vector &out)
{
    // This method gives analogdata to the analogServer

    if(false == GET_privData(mPriv).isOpen())
    {
        return false;
    }

    GET_privData(mPriv).mutex.wait();

    out.resize(GET_privData(mPriv).analogdata.size());
    for (size_t k = 0; k<GET_privData(mPriv).analogdata.size(); k++)
    {
        out[k] = GET_privData(mPriv).analogdata[k] + GET_privData(mPriv).offset[k];
    }


    GET_privData(mPriv).mutex.post();

    return IAnalogSensor::AS_OK;;
}

int embObjFTsensor::getState(int ch)
{
    return AS_OK;
}


int embObjFTsensor::getChannels()
{
    return GET_privData(mPriv).strain_Channels;
}


int embObjFTsensor::calibrateSensor()
{
    GET_privData(mPriv).mutex.wait();
    for (size_t i = 0; i <  GET_privData(mPriv).analogdata.size(); i++)
    {
        GET_privData(mPriv).offset[i] = - GET_privData(mPriv).analogdata[i];
    }
    GET_privData(mPriv).mutex.post();
    return AS_OK;
}

int embObjFTsensor::calibrateSensor(const yarp::sig::Vector& value)
{
    return AS_OK;
}

int embObjFTsensor::calibrateChannel(int ch)
{
    return AS_OK;
}

int embObjFTsensor::calibrateChannel(int ch, double v)
{
    return AS_OK;
}

eth::iethresType_t embObjFTsensor::type()
{
    return eth::iethres_analogstrain;
}



// ---------------------- ITemperatureSensors --------------------------------------------------------
size_t embObjFTsensor::getNrOfTemperatureSensors() const
{
    return 1;
}

yarp::dev::MAS_status embObjFTsensor::getTemperatureSensorStatus(size_t sens_index) const
{
    return yarp::dev::MAS_OK;
}

bool embObjFTsensor::getTemperatureSensorName(size_t sens_index, std::string &name) const
{
    name = GET_privData(mPriv).devicename;
    return true;
}

bool embObjFTsensor::getTemperatureSensorFrameName(size_t sens_index, std::string &frameName) const
{
    frameName = GET_privData(mPriv).devicename;
    return true;
}

bool embObjFTsensor::getTemperatureSensorMeasure(size_t sens_index, double& out, double& timestamp) const
{
    GET_privData(mPriv).mutex.wait();
    out = GET_privData(mPriv).lastTemperature/10.0; //I need to convert from tenths of degree centigrade to degree centigrade
    timestamp =  GET_privData(mPriv).timestampTemperature;
    GET_privData(mPriv).mutex.post();
    return true;
}

bool embObjFTsensor::getTemperatureSensorMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    GET_privData(mPriv).mutex.wait();
    out.resize(1);
    out[0] = GET_privData(mPriv).lastTemperature/10.0; //I need to convert from tenths of degree centigrade to degree centigrade
    timestamp =  GET_privData(mPriv).timestampTemperature;
    GET_privData(mPriv).mutex.post();
    return true;
}

//------------------------- ISixAxisForceTorqueSensors -------------------------

size_t embObjFTsensor::getNrOfSixAxisForceTorqueSensors() const
{
    return 1;
}

yarp::dev::MAS_status embObjFTsensor::getSixAxisForceTorqueSensorStatus(size_t sens_index) const
{
    return yarp::dev::MAS_OK;
}

bool embObjFTsensor::getSixAxisForceTorqueSensorName(size_t sens_index, std::string &name) const
{
    name = GET_privData(mPriv).devicename;
    return true;
}

bool embObjFTsensor::getSixAxisForceTorqueSensorFrameName(size_t sens_index, std::string &frameName) const
{
    frameName = GET_privData(mPriv).devicename;
    return true;
}

bool embObjFTsensor::getSixAxisForceTorqueSensorMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    if(false == GET_privData(mPriv).isOpen())
    {
        return false;
    }

    GET_privData(mPriv).mutex.wait();

    out.resize(GET_privData(mPriv).analogdata.size());
    for (size_t k = 0; k<GET_privData(mPriv).analogdata.size(); k++)
    {
        out[k] = GET_privData(mPriv).analogdata[k] + GET_privData(mPriv).offset[k];
    }

    timestamp =  GET_privData(mPriv).timestampAnalogdata;

    GET_privData(mPriv).mutex.post();

    return true;
}

// eof

