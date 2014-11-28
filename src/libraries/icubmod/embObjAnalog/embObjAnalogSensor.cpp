
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
#include <stdarg.h>
#include <stdio.h>
#include <yarp/dev/PolyDriver.h>
#include <ace/config.h>
#include <ace/Log_Msg.h>


// specific to this device driver.
#include <embObjAnalogSensor.h>
#include <ethManager.h>
#include <yarp/os/LogStream.h>
#include "EoAnalogSensors.h"
#include "EOnv_hid.h"

#include "EoProtocol.h"
#include "EoProtocolMN.h"
#include "EoProtocolAS.h"

#ifdef WIN32
#pragma warning(once:4355)
#endif



using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;


inline bool NOT_YET_IMPLEMENTED(const char *txt)
{
    yWarning() << std::string(txt) << " not yet implemented for embObjAnalogSensor\n";
    return false;
}

// generic function that checks is key1 is present in input bottle and that the result has size elements
// return true/false
static inline bool extractGroup(Bottle &input, Bottle &out, const std::string &key1, const std::string &txt, int size)
{
    size++;  // size includes also the name of the parameter
    Bottle &tmp=input.findGroup(key1.c_str(), txt.c_str());
    if (tmp.isNull())
    {
        fprintf(stderr, "%s not found\n", key1.c_str());
        return false;
    }

    if(tmp.size()!=size)
    {
        fprintf(stderr, "%s incorrect number of entries\n", key1.c_str());
        return false;
    }

    out=tmp;

    return true;
}

bool embObjAnalogSensor::fromConfig(yarp::os::Searchable &_config)
{
    Bottle xtmp;
    int format=0;

  // embObj parameters, in ETH group
    Value val =_config.findGroup("ETH").check("Ems",Value(1), "Board number");
    if(val.isInt())
        _fId.boardNumber = val.asInt();
    else
    {
        _as_type=AS_NONE;
        yError () << "embObjAnalogSensor: EMS Board number identifier not found\n";
        return false;
    }

    // Analog Sensor stuff
    Bottle config = _config.findGroup("GENERAL");
    if (!extractGroup(config, xtmp, "Period","transmetting period of the sensor", 1))
    {
        yError() << "embObjAnalogSensor Using default value = 0 (disabled)";
        _period = 0;
        _as_type=AS_NONE;
    }
    else
    {
        _period = xtmp.get(1).asInt();
        yDebug() << "  (!!)-> embObjAnalogSensor::fromConfig() detects embObjAnalogSensor Using value of" << _period;
    }

    if (!extractGroup(config, xtmp, "Channels","Number of channels of the Analog Sensor", 1))
    {
        fprintf(stderr, "embObjAnalogSensor: Using default value = 0 (disabled)\n");
        _channels = 0;
        _period   = 0;
        _as_type=AS_NONE;
    }
    else
    {
        _channels = xtmp.get(1).asInt();
    }


    if (!extractGroup(config, xtmp, "Format","data format of the Analog Sensor", 1))
    {
        fprintf(stderr, "embObjAnalogSensor: Using default value = 0 (disabled)\n");
        _channels = 0;
        _period   = 0;
        _as_type=AS_NONE;
    }
    else
    {
        format = xtmp.get(1).asInt();
    }



    if((_channels==NUMCHANNEL_STRAIN) && (format==FORMATDATA_STRAIN))
    {
        _as_type=AS_STRAIN;
    }
    else if((_channels==NUMCHANNEL_MAIS) && (format==FORMATDATA_MAIS))
    {
        _as_type=AS_MAIS;
    }
    else
    {
        _as_type=AS_NONE;
        yError() << "embObjAnalogSensor incorrect config!channels="<< _channels <<" format="<< format;
        return false;
    }

    if(AS_STRAIN == _as_type)
    {
        if (!extractGroup(config, xtmp, "UseCalibration","Calibration parameters are needed", 1))
        {
            fprintf(stderr, "embObjAnalogSensor: Using default value = 0 (Don't use calibration)\n");
            _useCalibration = 0;
        }
        else
        {
            _useCalibration = xtmp.get(1).asInt();
        }
    }
    return true;
};


embObjAnalogSensor::embObjAnalogSensor(): data(0)
{
    _useCalibration=0;
    _channels=0;
    _period=0;
    _as_type=AS_NONE;
    //_format=ANALOG_FORMAT_NONE;

    scaleFactor=0;

    timeStamp=0;
    counterSat=0;
    counterError=0;
    counterTimeout=0;

    status=IAnalogSensor::AS_OK;

    opened = false;
}

embObjAnalogSensor::~embObjAnalogSensor()
{
    if (!data)
        delete data;
    if (!scaleFactor)
        delete scaleFactor;
}

bool embObjAnalogSensor::initialised()
{
    return opened;
}

bool embObjAnalogSensor::open(yarp::os::Searchable &config)
{
    std::string str;
    if(config.findGroup("GENERAL").find("verbose").asBool())
        str=config.toString().c_str();
    else
        str="\n";
    yTrace() << str;

    // Read stuff from config file
    if(!fromConfig(config))
    {
        yError() << "embObjAnalogSensor missing some configuration parameter. Check logs and your config file.";
        return false;
    }

    // Tmp variables
    Bottle          groupEth;
    ACE_TCHAR       address[64];
    ACE_UINT16      port;
    bool            ret;

    Bottle groupTransceiver = Bottle(config.findGroup("TRANSCEIVER"));
    if(groupTransceiver.isNull())
    {
        yError() << "embObjAnalogSensor::open(): Can't find TRANSCEIVER group in xml config files";
        return false;
    }

    Bottle groupProtocol = Bottle(config.findGroup("PROTOCOL"));
    if(groupProtocol.isNull())
    {
        yError() << "embObjAnalogSensor::open(): Can't find PROTOCOL group in xml config files";
        return false;
    }

    // Get both PC104 and EMS ip addresses and port from config file
    groupEth  = Bottle(config.findGroup("ETH"));
    Bottle parameter1( groupEth.find("PC104IpAddress").asString() );    // .findGroup("IpAddress");
    port      = groupEth.find("CmdPort").asInt();              // .get(1).asInt();
    snprintf(_fId.pc104IPaddr.string, sizeof(_fId.pc104IPaddr.string), "%s", parameter1.toString().c_str());
    _fId.pc104IPaddr.port = port;

    Bottle parameter2( groupEth.find("IpAddress").asString() );    // .findGroup("IpAddress");
    snprintf(_fId.boardIPaddr.string, sizeof(_fId.boardIPaddr.string), "%s", parameter2.toString().c_str());
    _fId.boardIPaddr.port = port;

    sscanf(_fId.boardIPaddr.string,"\"%d.%d.%d.%d", &_fId.boardIPaddr.ip1, &_fId.boardIPaddr.ip2, &_fId.boardIPaddr.ip3, &_fId.boardIPaddr.ip4);
    sscanf(_fId.pc104IPaddr.string,"\"%d.%d.%d.%d", &_fId.pc104IPaddr.ip1, &_fId.pc104IPaddr.ip2, &_fId.pc104IPaddr.ip3, &_fId.pc104IPaddr.ip4);

    snprintf(_fId.boardIPaddr.string, sizeof(_fId.boardIPaddr.string), "%u.%u.%u.%u:%u", _fId.boardIPaddr.ip1, _fId.boardIPaddr.ip2, _fId.boardIPaddr.ip3, _fId.boardIPaddr.ip4, _fId.boardIPaddr.port);
    snprintf(_fId.pc104IPaddr.string, sizeof(_fId.pc104IPaddr.string), "%u.%u.%u.%u:%u", _fId.pc104IPaddr.ip1, _fId.pc104IPaddr.ip2, _fId.pc104IPaddr.ip3, _fId.pc104IPaddr.ip4, _fId.pc104IPaddr.port);

    // debug info
    memset(info, 0x00, sizeof(info));
    snprintf(info, sizeof(info), "embObjAnalogSensor: referred to EMS: %d at address %s\n", _fId.boardNumber, address);
    snprintf(_fId.name, sizeof(_fId.name), "%s", info);       // Saving User Friendly Id

    // Set dummy values
    _fId.boardNumber  = FEAT_boardnumber_dummy;

    Value val = config.findGroup("ETH").check("Ems", Value(1), "Board number");
    if(val.isInt())
        _fId.boardNumber = val.asInt();
    else
    {
        yError () << "embObjAnalogSensor: EMS Board number identifier not found for IP" << _fId.pc104IPaddr.string;
        return false;
    }


    ethManager = TheEthManager::instance();
    if(NULL == ethManager)
    {
        yFatal() << "embObjAnalogSensor::open() cannot instantiate ethManager";
        return false;
    }

    // N.B.: use a dynamic_cast<> to extract correct interface when using this pointer
    _fId.interface = this;

    _fId.endpoint = eoprot_endpoint_analogsensors;
    if(AS_STRAIN == _as_type)
        _fId.type = ethFeatType_AnalogStrain;
    else if(AS_MAIS == _as_type)
        _fId.type = ethFeatType_AnalogMais;

    /* Once I'm ok, ask for resources, through the _fId struct I'll give the ip addr, port and
    *  and boardNum to the ethManager in order to create the ethResource requested.
    */
    res = ethManager->requestResource(config, groupTransceiver, groupProtocol, _fId);
    if(NULL == res)
    {
        yError() << "embObjAnalogSensor::open() fails because could not instantiate the ethResource board" << _fId.boardNumber << " ... unable to continue";
        return false;
    }

    if(false == res->isEPmanaged(eoprot_endpoint_analogsensors))
    {
        yError() << "embObjAnalogSensor::open() detected that EMS "<< _fId.boardNumber << " does not support analog sensors";
        return false;
    }

    if(false == res->verifyBoard(groupProtocol))
    {
        yError() << "embObjAnalogSensor::open() fails in function verifyBoard() for board " << _fId.boardNumber << ": CANNOT PROCEED ANY FURTHER";
        return false;
    }
    else
    {
        yWarning() << "(OK)-> embObjAnalogSensor::open() has succesfully verified that board "<< _fId.boardNumber << " is communicating correctly";
    }

    if(!res->verifyEPprotocol(groupProtocol, eoprot_endpoint_analogsensors))
    {
        yError() << "embObjAnalogSensor::open() fails in function verifyEPprotocol() for board "<< _fId.boardNumber << ": either the board cannot be reached or it does not have the same eoprot_endpoint_management and/or eoprot_endpoint_analogsensors protocol version: DO A FW UPGRADE";
        return false;
    }
    else
    {
        yWarning() << "(OK)-> embObjAnalogSensor::open() has succesfully verified that board "<< _fId.boardNumber << " has same protocol version for analogsensors as robotInterface";
    }

//    // marco.accame on 04 sept 2014: we could add a verification about the entities of analog ... MAYBE in the future
//
//    uint8_t numberofmaisboards = eoprot_entity_numberof_get(featIdBoardNum2nvBoardNum(_fId.boardNumber), eoprot_endpoint_analogsensors, eoprot_entity_as_mais);
//    if(false == res->verifyENTITYnumber(groupProtocol, eoprot_endpoint_analogsensors, eoprot_entity_as_mais, numberofmaisboards))
//    {   // JUST AN EXAMPLE OF HOW TO DO IT FOR THE MAIS.
//        yError() << "embObjAnalogSensor::init() fails in function verifyENTITYnumber() for board "<< _fId.boardNumber << " and entity eoprot_entity_as_mais: VERIFY their number in board, and in XML files";
//        return false;
//    }
//    uint8_t numberofstrainboards = eoprot_entity_numberof_get(featIdBoardNum2nvBoardNum(_fId.boardNumber), eoprot_endpoint_analogsensors, eoprot_entity_as_strain);
//    if(false == res->verifyENTITYnumber(groupProtocol, eoprot_endpoint_analogsensors, eoprot_entity_as_strain, numberofstrainboards))
//    {   // JUST AN EXAMPLE OF HOW TO DO IT FOR THE STRAIN.
//        yError() << "embObjAnalogSensor::init() fails in function verifyENTITYnumber() for board "<< _fId.boardNumber << " and entity eoprot_entity_as_strain: VERIFY their number in board, and in XML files";
//        return false;
//    }


    data = new AnalogData(_channels, _channels+1);
    scaleFactor = new double[_channels];
    int i=0;
    for (i=0; i<_channels; i++) scaleFactor[i]=1;

    // Real values will be read from the sensor itself during its initalization hereafter
    for(int i=0; i<_channels; i++)
    {
        scaleFactor[i]=1;
    }


//#warning --> marco.accame on 04sept14: both embObjAnalogSensors and embObjMotionControl initialises the mais board. but potentially w/ different params (mc uses: datarate = 10 and mode = eoas_maismode_txdatacontinuously).

    switch(_as_type)
    {
        case AS_MAIS:
        {
            ret = sendConfig2Mais();
        } break;
        
        case AS_STRAIN:
        {
            ret = sendConfig2Strain();
        } break;
        
        default:
        {
            //i should not be here. if AS_NONE then i should get error in fromConfig function
            ret = false;
        }
    }
    if(!ret)
        return false;


    // Set variable to be signalled
    if(! init())
        return false;


    if(false == res->goToRun())
    {
        yError() << "embObjAnalogSensor::open() fails to start control loop of board" << _fId.boardNumber << ": cannot continue";
        return false;
    }
    else
    {
        yWarning() << "(OK)-> embObjAnalogSensor::open() correctly activated control loop of BOARD" << _fId.boardNumber;
    }

    opened = true;

    return true;
}

bool embObjAnalogSensor::isEpManagedByBoard()
{

    if(eobool_true == eoprot_endpoint_configured_is(res->get_protBRDnumber(), eoprot_endpoint_analogsensors))
    {   // we can extend to the entity by evaluating eoprot_entity_configured_is(res->get_protBRDnumber(), eoprot_endpoint_analogsensors, eoprot_entity_as_xxx)
        return true;
    }
    
    return false;
}

bool embObjAnalogSensor::sendConfig2Strain(void)
{
    eOas_strain_config_t strainConfig = {0};

    strainConfig.datarate = _period;
    strainConfig.signaloncefullscale = eobool_false;

    if(_useCalibration)
    {
        if( ! getFullscaleValues() )
        {
            yError() << "embObjAnalogSensor::sendConfig2Strain() has failed in calling  embObjAnalogSensor::getFullscaleValues()";
            return false;
        }
        strainConfig.mode = eoas_strainmode_txcalibrateddatacontinuously;
    }
    else
    {
        strainConfig.mode = eoas_strainmode_txuncalibrateddatacontinuously;
    }


#if 1
    // version with read-back

    eOprotID32_t id32 =  eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_strain, 0, eoprot_tag_as_strain_config);

    if(false == res->setRemoteValueUntilVerified(id32, &strainConfig, sizeof(strainConfig), 10, 0.010, 0.050, 2))
    {
        yError() << "FATAL: embObjAnalogSensor::sendConfig2Strain() had an error while calling setRemoteValueUntilVerified() for strain config in BOARD" << res->get_protBRDnumber()+1;
        return false;
    }
    else
    {
        yWarning() << "(OK)-> embObjAnalogSensor::sendConfig2Strain() correctly configured strain coinfig in BOARD" << res->get_protBRDnumber()+1;
    }

    return true;

#else

    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_strain, 0, eoprot_tag_as_strain_config);
    if(!res->addSetMessage(protoid, (uint8_t *) &strainConfig))
    {
        yError() << "embObjAnalogSensor::sendConfig2Strain() could not send a set message for eoprot_tag_as_strain_config";
    }

    return true;
#endif

}

bool embObjAnalogSensor::sendConfig2Mais(void)
{
#if 1
    // version with read-back

    eOprotID32_t id32 = eo_prot_ID32dummy;

    // -- mais datarate

    uint8_t datarate  = _period;  // set mais datarate = 1millisec
    id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_mais, 0, eoprot_tag_as_mais_config_datarate);

    if(false == res->setRemoteValueUntilVerified(id32, &datarate, sizeof(datarate), 10, 0.010, 0.050, 2))
    {
        yError() << "FATAL: embObjAnalogSensor::sendConfig2Mais() had an error while calling setRemoteValueUntilVerified() for mais datarate in BOARD" << res->get_protBRDnumber()+1;
        return false;
    }
    else
    {
        yWarning() << "(OK)-> embObjAnalogSensor::sendConfig2Mais() correctly configured mais datarate at value" << datarate << "in BOARD" << res->get_protBRDnumber()+1;
    }

    // -- mais tx mode

    eOenum08_t maismode  = eoas_maismode_txdatacontinuously; // use eOas_maismode_t for value BUT USE   for type (their sizes can be different !!)
    id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_mais, 0, eoprot_tag_as_mais_config_mode);

    if(false == res->setRemoteValueUntilVerified(id32, &maismode, sizeof(maismode), 10, 0.010, 0.050, 2))
    {
        yError() << "FATAL: embObjAnalogSensor::sendConfig2Mais() had an error while calling setRemoteValueUntilVerified() for mais mode in BOARD" << res->get_protBRDnumber()+1;
        return false;
    }
    else
    {
        yWarning() << "(OK)-> embObjAnalogSensor::sendConfig2Mais() correctly configured mais mode at value" << maismode << "in BOARD" << res->get_protBRDnumber()+1;
    }

    return true;

#else
    uint8_t datarate  = _period;

    // set mais datarate = 1millisec
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_mais, 0, eoprot_tag_as_mais_config_datarate);


    if(!res->addSetMessage(protoid, &datarate))
    {
        yError() << "embObjAnalogSensor::sendConfig2Mais() could not send a set message fot datarate";
        yError() << "embObjAnalogSensor::sendConfig2Mais() however did not return false. IS CORRECT?";
    }

    // set tx mode continuosly
    eOas_maismode_t     maismode  = eoas_maismode_txdatacontinuously;
    protoid = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_mais, 0, eoprot_tag_as_mais_config_mode);


    if(!res->addSetMessage(protoid, (uint8_t *) &maismode))
    {
        yError() << "embObjAnalogSensor::sendConfig2Mais() could not send a set message fot maismode";
        yError() << "embObjAnalogSensor::sendConfig2Mais() however did not return false. IS CORRECT?";
    }

#warning --> marco.accame: maybe we can add a readback of the datarate and maismode to be sure of successful operation

    return true;
#endif
}

#warning --> marco.accame: review function embObjAnalogSensor::getFullscaleValues() as in comment below
// it is better to change the behaviour of the function so that: 1. we send the request, 2. we wait for the sig<> and unblock a mutex
// current implementation relies on a wait of 1 sec and check of non-zero length of an array: smart but not the rigth way to do it.
bool embObjAnalogSensor::getFullscaleValues()
{
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
    
#if 1
   // Check initial size of array...  it should be zero.
    bool gotFullScaleValues = false;
    int timeout, NVsize;
    uint16_t tmpNVsize;
    EOnv tmpNV;
    EOnv *p_tmpNV = NULL;
    eOas_arrayofupto12bytes_t fullscale_values = {0};
    // force it to be an empty array of itemsize 2 and capacity 6. 
    // the reason is that the eoprot_tag_as_strain_status_fullscale contains 3 forces and 3 torques each of 2 bytes. see eOas_strain_status_t in EoAnalogSensors.h
    eo_array_New(6, 2, &fullscale_values);

    eOprotID32_t protoid_fullscale = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_strain, 0, eoprot_tag_as_strain_status_fullscale);

    if(eobool_false == eoprot_id_isvalid(featIdBoardNum2nvBoardNum(_fId.boardNumber), protoid_fullscale))
    {
        char str[128];
        eoprot_ID2information(protoid_fullscale, str, sizeof(str));
        yError() << "embObjAnalogSensor::getFullscaleValues() detected that is not valid this ID32:" << str;
        return false;
    }
        
    // now we impose that the value of the EOnv described by eoprot_tag_as_strain_status_fullscale is what in fullscale_values.
    // we need to do that because the ram of the EOnv is ... of zero value unless one overrides the relevant INIT function.
    // if of zero value, then ... capacity is not 6 and itemsize is not 2.
    p_tmpNV = res->getNVhandler(protoid_fullscale, &tmpNV);
    eo_nv_Set(p_tmpNV, &fullscale_values, eobool_true, eo_nv_upd_dontdo); 

    // now we are sure that we have a value of the array inside the EOnv which is consistent and ... of zero size.

#else
    // Check initial size of array...  it should be zero.
    bool gotFullScaleValues = false;
    int timeout, NVsize;
    uint16_t tmpNVsize;
    EOnv tmpNV, *p_tmpNV;
    eOas_arrayofupto12bytes_t fullscale_values;
/*  can't do a reset if array is not correctly initialized with eo_new_blablabla function.
 * This needs knowing how data are actually stored, bytes dimension etc... which is quite low level knowledge to be placed here.
    I rely on the fact that iitialization of the variable is correctly done by transceiver.
     eo_array_New(&fullscale_values)
     eo_array_Reset((EOarray*) &fullscale_values);

     or better, just check that initalization has been done as expected, i.e. initial size is zero.
*/
     eOprotID32_t protoid_fullscale = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_strain, 0, eoprot_tag_as_strain_status_fullscale);

    if(eobool_false == eoprot_id_isvalid(featIdBoardNum2nvBoardNum(_fId.boardNumber), protoid_fullscale))
        yError() << "nvid not found";

    p_tmpNV = res->getNVhandler(protoid_fullscale, &tmpNV);

    // tmpNVsize is the actual dimension of the array expressed in bytes, it is NOT the number of elements the array contains
    res->readBufferedValue(protoid_fullscale, (uint8_t *)&fullscale_values, &tmpNVsize);

    // when initialized, size should be zero... check it
    NVsize = eo_array_Size((EOarray*) p_tmpNV->ram);
    if(0 != NVsize)
        yError() << "Initial size of array is different from zero (" << NVsize << ") for board" << _fId.boardNumber;
     
#endif
        
    // Prepare analog sensor
    eOas_strain_config_t strainConfig = {0};
    strainConfig.datarate               = _period;
    strainConfig.mode                   = eoas_strainmode_acquirebutdonttx;
    strainConfig.signaloncefullscale    = eobool_true;

    eOprotID32_t protoid_strain_config = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_strain, 0, eoprot_tag_as_strain_config);

    timeout = 5;

    // wait for response
    while(!gotFullScaleValues && (timeout != 0))
    {
        res->addSetMessage(protoid_strain_config, (uint8_t *) &strainConfig);
        Time::delay(1.0);
        // read fullscale values
        res->readBufferedValue(protoid_fullscale, (uint8_t *) &fullscale_values, &tmpNVsize);
        // If data arrives, size is bigger than zero
        //#warning --> marco.accame says: to wait for 1 sec and read size is ok. a different way is to ... wait for a semaphore incremented by the reply of the board. think of it!
        NVsize = eo_array_Size((EOarray *)&fullscale_values);

        if(0 != NVsize)
        {
            gotFullScaleValues = true;
            break;
        }

        timeout--;
        yDebug() << "  (!!)-> embObjAnalogSensor::getFullscaleValues(): for board " << _fId.boardNumber << ": full scale val not arrived yet... retrying in 1 sec";
    }

    if((false == gotFullScaleValues) && (0 == timeout))
    {
        yError() << "embObjAnalogSensor::getFullscaleValues(): ETH Analog sensor: request for calibration parameters timed out for board" << _fId.boardNumber;
        return false;
    }

    if((NVsize != _channels))
    {
        yError() << "Analog sensor Calibration data has a different size from channels number in configuration file for board" << _fId.boardNumber << "Aborting";
        return false;
    }

    uint8_t *msg;


    if(gotFullScaleValues)
    {
        yWarning() << "(!!)-> embObjAnalogSensor::getFullscaleValues() detected that already has full scale values for board" << _fId.boardNumber;

        yDebug() << "  (!!)-> embObjAnalogSensor::getFullscaleValues(): Fullscale values for board " << _fId.boardNumber << "are: size=" <<  eo_array_Size((EOarray *)&fullscale_values) << "  numchannel=" <<  _channels;


        for(int i=0; i<_channels; i++)
        {
            // Get the k-th element of the array as a 2 bytes msg
            msg = (uint8_t *) eo_array_At((EOarray *) &fullscale_values, i);
            if(NULL == msg)
            {
                yError() << "I don't receive data for channel " << i;
                return false;
            }
            // Got from CanBusMotionControl... here order of bytes seems inverted with respect to calibratedValues or uncalibratedValues (see callback)
            scaleFactor[i]= 0;
            scaleFactor[i]= ((uint16_t)(msg[0]<<8) | msg[1]);
            //scaleFactor[i]=i;
            //yError() << " scale factor[" << i << "] = " << scaleFactor[i];
            yDebug() << "  (!!)-> embObjAnalogSensor::getFullscaleValues(): channel " << i << "full scale value " << scaleFactor[i];
        }
    }

    return true;  
}


bool embObjAnalogSensor::init()
{
    yTrace();

    
    // - configure regular rops

    vector<eOprotID32_t> id32v(0);
    eOprotID32_t protoid = eo_prot_ID32dummy;

    // we need to choose the protoid to put inside the vector

    switch(_as_type)
    {
        case AS_MAIS:
        {
            protoid = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_mais, 0, eoprot_tag_as_mais_status_the15values);
        } break;
        
        case AS_STRAIN:
        {
            if(_useCalibration)
                protoid = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_strain, 0, eoprot_tag_as_strain_status_calibratedvalues);
            else
                protoid = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_strain, 0, eoprot_tag_as_strain_status_uncalibratedvalues);
        } break;
        
        default:
        {
            protoid = eo_prot_ID32dummy;
        }
    }

    if(eobool_false == eoprot_id_isvalid(featIdBoardNum2nvBoardNum(_fId.boardNumber), protoid))
    {
        char str[128];
        eoprot_ID2information(protoid, str, sizeof(str));
        yError () << "embObjAnalogSensor::init() recognised invalid ID to put in regulars: " << str;
        return false;
    }

    // put it inside vector

    id32v.push_back(protoid);

    // configure the regulars
    if(false == res->addRegulars(id32v, true))
    {
        yError() << "embObjAnalogSensor::init() fails to add its variables to regulars: cannot proceed any further";
        return false;
    }
    else
    {
        yWarning() << "(OK)-> embObjAnalogSensor::init() added" << id32v.size() << "regular rops to BOARD" << res->get_protBRDnumber()+1;
        char nvinfo[128];
        for(int r=0; r<id32v.size(); r++)
        {
            uint32_t id32 = id32v.at(r);
            eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
            yWarning() << "(OK)->\t it added regular rop for" << nvinfo;
        }
    }

    Time::delay(0.005);  // 5 ms (m.a.a-delay: before it was 0)
   

    return true;
}


/*! Read a vector from the sensor.
 * @param out a vector containing the sensor's last readings.
 * @return AS_OK or return code. AS_TIMEOUT if the sensor timed-out.
 **/
int embObjAnalogSensor::read(yarp::sig::Vector &out)
{
    // This method gives data to the analogServer

      mutex.wait();

      if (!data)
      {
          mutex.post();
          return false;
      }

      // errors are not handled for now... it'll always be OK!!
      if (status!=IAnalogSensor::AS_OK)
      {
          switch (status)
          {
              case IAnalogSensor::AS_OVF:
                  {
                      counterSat++;
                  }
                  break;
              case IAnalogSensor::AS_ERROR:
                  {
                      counterError++;
                  }
                  break;
              case IAnalogSensor::AS_TIMEOUT:
                  {
                     counterTimeout++;
                  }
                  break;
              default:
              {
                  counterError++;
              }
          }
          mutex.post();
          return status;
      }

      out.resize(data->size());
      for(int k=0;k<data->size();k++)
      {
          out[k]=(*data)[k];
      }

      mutex.post();
    
    return status;
}

int embObjAnalogSensor::getState(int ch)
{
    printf("getstate\n");
    return AS_OK;
}

int embObjAnalogSensor::getChannels()
{
    return data->size();
}

int embObjAnalogSensor::calibrateSensor()
{
    return AS_OK;
}

int embObjAnalogSensor::calibrateSensor(const yarp::sig::Vector& value)
{
    return AS_OK;
}

int embObjAnalogSensor::calibrateChannel(int ch)
{
    return AS_OK;
}

int embObjAnalogSensor::calibrateChannel(int ch, double v)
{
    return AS_OK;
}

bool embObjAnalogSensor::update(eOprotID32_t id32, double timestamp, void* rxdata)
{
    bool ret;

#warning --> marco.accame: retrieve the entity from id32 and see is it is mais or strain.
    switch(_as_type)
    {
        case AS_MAIS:
        {
            ret = fillDatOfMais(rxdata);
        } break;
        
        case AS_STRAIN:
        {
            ret = fillDatOfStrain(rxdata);
        } break;
        
        default:
        {
            //i should not be here. if AS_NONE then i should get error in fromConfig function
            ret = false;
        }
    }

    return ret;
}



bool embObjAnalogSensor::fillDatOfStrain(void *as_array_raw)
{
    // called by  embObjAnalogSensor::fillData() which is called by handle_AS_data() which is called by handle_data() which is called by:
    // eoprot_fun_UPDT_as_strain_status_calibratedvalues() or eoprot_fun_UPDT_as_strain_status_uncalibratedvalues() 
    // the void* parameter inside this function is a eOas_arrayofupto12bytes_t*   
    // and can be treated as a EOarray
    EOarray *array = (EOarray*)as_array_raw;
    uint8_t size = eo_array_Size(array);
    uint8_t itemsize = eo_array_ItemSize(array); // marco.accame: must be 2, as the code after uses this convention
    if(0 == size)
    {
        return false;
    }

    // lock data
    mutex.wait();

    double *_buffer = this->data->getBuffer();

    if(NULL == _buffer)
    {
        // unlock data
        mutex.post();
        return false;
    }

  
    for(int k=0; k<_channels; k++)
    {
        // Get the kth element of the array as a 2 bytes msg
        char* tmp = (char*) eo_array_At(array, k);
        // marco.accame: i am nervous about iterating for _channels instead of size of array.... 
        //               thus i add a protection. if k goes beyond size of array, eo_array_At() returns NULL.
        if(NULL != tmp)
        {
            uint8_t msg[2] = {0};
            memcpy(msg, tmp, 2);
            // Got from canBusMotionControl
            _buffer[k]= (short)( ( (((unsigned short)(msg[1]))<<8)+msg[0]) - (unsigned short) (0x8000) );

            if (_useCalibration == 1)
            {
                _buffer[k]=_buffer[k]*scaleFactor[k]/float(0x8000);
            }
        }
    }
     
    // unlock data
    mutex.post();

    return true;
}





bool embObjAnalogSensor::fillDatOfMais(void *as_array_raw)
{
    // called by  embObjAnalogSensor::fillData() which is called by handle_AS_data() which is called by handle_data() which is called by:
    // eoprot_fun_UPDT_as_mais_status_the15values()
    // the void* parameter inside this function is a eOas_arrayofupto36bytes_t*   
    // and can be treated as a EOarray

    EOarray *array = (EOarray*)as_array_raw;
    uint8_t size = eo_array_Size(array);
    uint8_t itemsize = eo_array_ItemSize(array); // marco.accame: must be 1, as the code after uses this convention
    if(0 == size)
    {
        return false;
    }

    mutex.wait();

    double *_buffer = this->data->getBuffer();

    if(NULL == _buffer)
    {
        mutex.post();
        return false;
    }

    //NOTE: here i suppose that size of array is equal to num of channel or to 0 if sensor did not sent something
    //this is an agreement with firmware.
    for(int k=0; k<size; k++)
    {
        uint8_t val = *((uint8_t*)eo_array_At(array, k));       // marco.accame: i see that we treat the array as if containing items of 1 byte.
        // Get the kth element of the array
        _buffer[k] = (double)val;
    }

    mutex.post();

    return true;
}




bool embObjAnalogSensor::close()
{
    int ret = ethManager->releaseResource(_fId);
    if(ret == -1)
        ethManager->killYourself();
    res = NULL;
    return true;
}

// eof

