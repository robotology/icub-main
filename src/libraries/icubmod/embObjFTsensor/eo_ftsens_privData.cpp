/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 * Author: Valentina Gaggero
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */
#include "eo_ftsens_privData.h"
#include "EoProtocolAS.h"
#include "EOnv_hid.h"

using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;


eo_ftsens_privData::eo_ftsens_privData(std::string name):
                    embObjDevPrivData(name),
                    useCalibValues(false),
                    useTemperature(false),
                    scaleFactorIsFilled(false),
                    lastTemperature(0),
                    timestampTemperature(0.0),
                    timestampAnalogdata(0.0)
{
    analogdata.resize(strain_Channels, 0.0);
    offset.resize(strain_Channels, 0.0);
    scaleFactor.resize(strain_Channels, 1.0);
}


eo_ftsens_privData::~eo_ftsens_privData()
{    analogdata.resize(0);
    offset.resize(0);
    scaleFactor.resize(0);
}


bool eo_ftsens_privData::fromConfig(yarp::os::Searchable &_config, servConfigFTsensor_t &serviceConfig)
{
    
    ServiceParser* parser = new ServiceParser;
    bool ret = parser->parseService(_config, serviceConfig);
    delete parser;
    
    if(!ret)
    {
        yError() << getBoardInfo() << "is missing some configuration parameter. Check logs and your config file.";
        return false;
    }
    
    useCalibValues = serviceConfig.useCalibration;
    if(serviceConfig.temperatureAcquisitionrate > 0)
        useTemperature = true;
    devicename = serviceConfig.nameOfStrain;
    
    if(isVerbose())
        printServiceConfig(serviceConfig);
    return ret;
}




//#warning --> marco.accame: review function embObjFTsensor::fillScaleFactor() as in comment below
// it is better to change the behaviour of the function so that: 1. we send the request, 2. we wait for the sig<> and unblock a mutex
// current implementation relies on a wait of 1 sec and check of non-zero length of an array: smart but not the rigth way to do it.

// EVEN better: in serviceVerifyActivate() we allow the retrieval of a parameter which the ETH board sends back. in such a param it is contained
// the fullscales values ...

// marco.accame on 09 jan 2018: much better using an ask(id32_fullscale) and making this variable proxied inside the ETH board ...

bool eo_ftsens_privData::fillScaleFactor(servConfigFTsensor_t &serviceConfig)
{
    // if we already have set the scalefactor ...
    if(true == scaleFactorIsFilled)
    {
        return true;
    }
    
    // at first we set the scale factors to 1, so that we are sure they have a safe value. it redundant, as we already set it to 1.0
    for(size_t i = 0; i<scaleFactor.size(); i++)
    {
        scaleFactor[i] = 1.0f;
    }
    
    // if we dont need calibration we are done
    if(false == useCalibValues)
    {
        if(isVerbose())
        {
            yDebug() << getBoardInfo() << "fillScaleFactor(): we DONT use calibration, thus all scale factors are set to 1.0";
        }
        
        scaleFactorIsFilled = true;
        return true;
    }
    
    // if we need calibration, then we need to ask the fullscales directly to the strain
    
    
    // marco.accame on 11 apr 2014:
    // added the code under ifdef 1. the reason is that one cannot rely on validity of data structures inside the EOnv, as in protocol v2 the requirement is that
    // the initialisation is not specialised and is ... all zeros. if the user wants to init to proper values must redefine the relevant INIT funtion.
    // in this case, the eoprot_fun_INIT_as_strain_status_fullscale().
    // extern void eoprot_fun_INIT_as_strain_status_fullscale(const EOnv* nv)
    // {
    //     eOas_arrayofupto12bytes_t fullscale_values = {0};
    //     eo_array_New(6, 2, &fullscale_values); // itemsize = 2, capacity = 6
    //     eo_nv_Set(nv, &fullscale_values, eobool_true, eo_nv_upd_dontdo);    
    // }
    // moreover, even if properly initted, it is required to set the size to 0 because the size being not 0 is the check of reception of a message.
    
    
    bool gotFullScaleValues = false;
    
    
    // Check initial size of array...  it should be zero.
    int timeout, NVsize;
    EOnv tmpNV;
    EOnv *p_tmpNV = NULL;
    eOas_arrayofupto12bytes_t fullscale_values = {0};
    // force it to be an empty array of itemsize 2 and capacity 6. 
    // the reason is that the eoprot_tag_as_strain_status_fullscale contains 3 forces and 3 torques each of 2 bytes. see eOas_strain_status_t in EoAnalogSensors.h
    eo_array_New(6, 2, &fullscale_values);
    
    eOprotID32_t id32_fullscale = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_strain, 0, eoprot_tag_as_strain_status_fullscale);
    
    
    // at first we impose that the local value of fullscales is zero.
    // we also force the change because this variable is readonly
    const bool overrideROprotection = true;
    res->setLocalValue(id32_fullscale, &fullscale_values, overrideROprotection);
    
    
    // Prepare analog sensor
    eOas_strain_config_t strainConfig = {0};
    strainConfig.datarate               = serviceConfig.acquisitionrate;
    strainConfig.mode                   = eoas_strainmode_acquirebutdonttx;
    strainConfig.signaloncefullscale    = eobool_true;
    
    eOprotID32_t id32_strain_config = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_strain, 0, eoprot_tag_as_strain_config);
    
    timeout = 5;
    
    // wait for response
    while(!gotFullScaleValues && (timeout != 0))
    {
        res->setRemoteValue(id32_strain_config, &strainConfig);
        SystemClock::delaySystem(1.0);
        // read fullscale values
        res->getLocalValue(id32_fullscale, &fullscale_values);
        // If data arrives, size is bigger than zero
        //#warning --> marco.accame says: to wait for 1 sec and read size is ok. a different way is to ... wait for a semaphore incremented by the reply of the board. think of it!
        NVsize = eo_array_Size((EOarray *)&fullscale_values);
        
        if(0 != NVsize)
        {
            gotFullScaleValues = true;
            break;
        }
        
        timeout--;
        if(isVerbose())
        {
            yWarning() << getBoardInfo() << "filling ScaleFactor ....";
        }
    }
    
    if((false == gotFullScaleValues) && (0 == timeout))
    {
        yError() << getBoardInfo()  << "fillScaleFactor(): ETH Analog sensor: request for calibration parameters timed out ";
        return false;
    }
    
    if((strain_Channels != NVsize))
    {
        yError()  << getBoardInfo() << "Analog sensor Calibration data has a different size from channels number in configuration file ";
        return false;
    }
    
    
    if(gotFullScaleValues)
    {
        if(isVerbose())
        {
            yWarning() << getBoardInfo() << "fillScaleFactor() detected that already has full scale values";
            yDebug()   << getBoardInfo() << "fillScaleFactor(): Fullscale values are: size=" <<  eo_array_Size((EOarray *)&fullscale_values) << "  numchannel=" <<  strain_Channels;
        }
        
        for (size_t i = 0; i<scaleFactor.size(); i++)
        {
            // Get the k-th element of the array as a 2 bytes msg
            uint8_t *msg = (uint8_t *) eo_array_At((EOarray *) &fullscale_values, i);
            if(NULL == msg)
            {
                yError() << getBoardInfo() << "fillScaleFactor() doesn't receive data for channel " << i;
                return false;
            }
            // Got from CanBusMotionControl... here order of bytes seems inverted with respect to calibratedValues or uncalibratedValues (see callback of can strain messages inside the FW of ETHBOARD)
            scaleFactor[i] = ((uint16_t)(msg[0]<<8) | msg[1]);
            //yError() << " scale factor[" << i << "] = " << scaleFactor[i];
            if(isVerbose())
            {
                yDebug() << getBoardInfo() << "fillScaleFactor(): channel " << i << "full scale value " << scaleFactor[i];
            }
        }
        
        scaleFactorIsFilled = true;
    }
    
    return scaleFactorIsFilled;
}


bool eo_ftsens_privData::initRegulars(servConfigFTsensor_t &serviceConfig)
{
    vector<eOprotID32_t> id32v(0); //vector with id of nv to configure as regulars
    eOprotID32_t id32 = eo_prot_ID32dummy;

    //1) set regulars for ft (strain) service
    if(true == serviceConfig.useCalibration)
    {
        id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_strain, 0, eoprot_tag_as_strain_status_calibratedvalues);
    }
    else
    {
        id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_strain, 0, eoprot_tag_as_strain_status_uncalibratedvalues);
    }

    id32v.push_back(id32);

    if(!serviceSetRegulars(eomn_serv_category_strain, id32v))
        return false;

    //2) set regulars for temperature service
    id32v.resize(0);
    id32 = eo_prot_ID32dummy;

    id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_temperature, 0, eoprot_tag_as_temperature_status);
    id32v.push_back(id32);

    if(!serviceSetRegulars(eomn_serv_category_temperatures, id32v))
        return false;

    return true;
}


void eo_ftsens_privData::printServiceConfig(servConfigFTsensor_t &serviceConfig)
{    
    char loc[20] = {0};
    char fir[20] = {0};
    char pro[20] = {0};
    const char * boardname = (NULL != res) ? (res->getProperties().boardnameString.c_str()) : ("NOT-ASSIGNED-YET");
    const char * ipv4 = (NULL != res) ? (res->getProperties().ipv4addrString.c_str()) : ("NOT-ASSIGNED-YET");
    const char * boardtype = eoboards_type2string2(static_cast<eObrd_type_t>(serviceConfig.ethservice.configuration.data.as.strain.boardtype.type), eobool_true);
    ServiceParser *parser =  new ServiceParser();
    parser->convert(serviceConfig.ethservice.configuration.data.as.strain.canloc, loc, sizeof(loc));
    parser->convert(serviceConfig.ethservice.configuration.data.as.strain.boardtype.firmware, fir, sizeof(fir));
    parser->convert(serviceConfig.ethservice.configuration.data.as.strain.boardtype.protocol, pro, sizeof(pro));
    
    yInfo() << "The embObjFTsensor device using BOARD" << boardname << " w/ IP" << ipv4 << "has the following service config:";
    yInfo() << "- acquisitionrate =" << serviceConfig.acquisitionrate;
    yInfo() << "- useCalibration =" << serviceConfig.useCalibration;
    yInfo() << "- STRAIN of type" << boardtype << "named" << serviceConfig.nameOfStrain << "@" << loc << "with required protocol version =" << pro << "and required firmware version =" << fir;
    delete parser;
}

bool eo_ftsens_privData::sendConfig2Strain(servConfigFTsensor_t &serviceConfig)
{
    eOas_strain_config_t strainConfig = {0};
    
    strainConfig.datarate = serviceConfig.acquisitionrate;
    strainConfig.signaloncefullscale = eobool_false;
    strainConfig.mode = (true == serviceConfig.useCalibration) ? (eoas_strainmode_txcalibrateddatacontinuously) : (eoas_strainmode_txuncalibrateddatacontinuously);
    
    // version with read-back
    
    eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_strain, 0, eoprot_tag_as_strain_config);
    
    if(false == res->setcheckRemoteValue(id32, &strainConfig, 10, 0.010, 0.050))
    {
        yError() << getBoardInfo() << "FATAL: sendConfig2Strain() had an error while calling setcheckRemoteValue() for strain config ";
        return false;
    }
    else
    {
        if(isVerbose())
        {
            yDebug() << getBoardInfo() << "sendConfig2Strain() correctly configured strain coinfig ";
        }
    }
    
    //configure the service of temperature 
    id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_temperature, 0, eoprot_tag_as_temperature_config);
    
    
    eOas_temperature_config_t tempconfig = {0};
    if(serviceConfig.temperatureAcquisitionrate > 0)
    {
        tempconfig.enabled = 1;
        tempconfig.datarate = serviceConfig.temperatureAcquisitionrate/1000;
    }


    if(false == res->setcheckRemoteValue(id32, &tempconfig, 10, 0.010, 0.050))
    {
        yError() << getBoardInfo() << "FATAL: sendConfig2Strain(temperature) had an error while calling setcheckRemoteValue() for strain config ";
        return false;
    }
    else
    {
        if(isVerbose())
        {
            yDebug() << getBoardInfo() << "sendConfig2Strain() correctly configured strain coinfig ";
        }
    }
    return true;
}

bool eo_ftsens_privData::fillTemperatureEthServiceInfo(eOmn_serv_parameter_t &ftSrv, eOmn_serv_parameter_t &tempSrv)
{
//     const eOmn_serv_parameter_t* servparamstrain = &serviceConfig.ethservice;
//     eOmn_serv_parameter_t servparamtemp;
//     const eOmn_serv_parameter_t* servparamtemp_ptr = &servparamtemp;

    tempSrv.configuration.type = eomn_serv_AS_temperatures;

    EOarray* array = eo_array_New(eOas_temperature_descriptors_maxnumber, sizeof(eOas_temperature_descriptor_t), &(tempSrv.configuration.data.as.temperature.arrayofdescriptor));
    eOas_temperature_descriptor_t descr= {0};
    descr.typeofboard = ftSrv.configuration.data.as.strain.boardtype.type; //eobrd_strain2 ;
    descr.typeofsensor = eoas_temperature_t1;
    descr.on.can.place = eobrd_place_can;
    descr.on.can.port = ftSrv.configuration.data.as.strain.canloc.port;
    descr.on.can.addr = ftSrv.configuration.data.as.strain.canloc.addr;
    eo_array_PushBack(array, &descr);

    eOas_temperature_setof_boardinfos_t * boardInfoSet_ptr = &tempSrv.configuration.data.as.temperature.setofboardinfos;
    eOresult_t res = eoas_temperature_setof_boardinfos_clear(boardInfoSet_ptr);
    if(res != eores_OK)
    {
        yError() << getBoardInfo() << "Error in eoas_temperature_setof_boardinfos_clear()";
        return false;
    }

    eObrd_info_t boardInfo = {0};
    boardInfo.type =  ftSrv.configuration.data.as.strain.boardtype.type;
    memcpy(&boardInfo.protocol , &(ftSrv.configuration.data.as.strain.boardtype.protocol), sizeof(eObrd_protocolversion_t));
    memcpy(&boardInfo.firmware, &(ftSrv.configuration.data.as.strain.boardtype.firmware), sizeof(eObrd_firmwareversion_t));
    res = eoas_temperature_setof_boardinfos_add(boardInfoSet_ptr, &boardInfo);
    if(eores_OK != res)
    {
        yError() << getBoardInfo() << "Error in eoas_temperature_setof_boardinfos_add()";
        return false;
    }

    return true;

}