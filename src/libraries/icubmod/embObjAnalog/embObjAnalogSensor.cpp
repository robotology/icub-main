
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
#include <Debug.h>
#include "EoAnalogSensors.h"
#include "EOnv_hid.h"

#include "EoProtocol.h"
#include "EoProtocolMN.h"
#include "EoProtocolAS.h"

#ifdef WIN32
#pragma warning(once:4355)
#endif

const int REPORT_PERIOD = 6;            // seconds
const double BCAST_STATUS_TIMEOUT = 6;  // seconds

#define NUMCHANNEL_STRAIN 6
#define NUMCHANNEL_MAIS 15
#define FORMATDATA_STRAIN 16
#define FORMATDATA_MAIS 8


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
        _fId.boardNum =val.asInt();
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
        yDebug() << "embObjAnalogSensor Using value of" << _period;
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
}

embObjAnalogSensor::~embObjAnalogSensor()
{
    if (!data)
        delete data;
    if (!scaleFactor)
        delete scaleFactor;
}

bool embObjAnalogSensor::open(yarp::os::Searchable &config)
{
    std::string str;
    if(config.findGroup("GENERAL").find("verbose").asBool())
        str=config.toString().c_str();
    else
        str="\n";
    yTrace() <<str;

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

    Bottle groupProtocol = Bottle(config.findGroup("PROTOCOL"));
    if(groupProtocol.isNull())
    {
        yWarning() << "embObjAnalogSensor: Can't find PROTOCOL group in config files ... using max capabilities";
        //return false;
    }

    // Get both PC104 and EMS ip addresses and port from config file
    groupEth  = Bottle(config.findGroup("ETH"));
    Bottle parameter1( groupEth.find("PC104IpAddress").asString() );    // .findGroup("IpAddress");
    port      = groupEth.find("CmdPort").asInt();              // .get(1).asInt();
    snprintf(_fId.PC104ipAddr.string, sizeof(_fId.PC104ipAddr.string), "%s", parameter1.toString().c_str());
    _fId.PC104ipAddr.port = port;

    Bottle parameter2( groupEth.find("IpAddress").asString() );    // .findGroup("IpAddress");
    snprintf(_fId.EMSipAddr.string, sizeof(_fId.EMSipAddr.string), "%s", parameter2.toString().c_str());
    _fId.EMSipAddr.port = port;

    sscanf(_fId.EMSipAddr.string,"\"%d.%d.%d.%d", &_fId.EMSipAddr.ip1, &_fId.EMSipAddr.ip2, &_fId.EMSipAddr.ip3, &_fId.EMSipAddr.ip4);
    sscanf(_fId.PC104ipAddr.string,"\"%d.%d.%d.%d", &_fId.PC104ipAddr.ip1, &_fId.PC104ipAddr.ip2, &_fId.PC104ipAddr.ip3, &_fId.PC104ipAddr.ip4);

    snprintf(_fId.EMSipAddr.string, sizeof(_fId.EMSipAddr.string), "%u.%u.%u.%u:%u", _fId.EMSipAddr.ip1, _fId.EMSipAddr.ip2, _fId.EMSipAddr.ip3, _fId.EMSipAddr.ip4, _fId.EMSipAddr.port);
    snprintf(_fId.PC104ipAddr.string, sizeof(_fId.PC104ipAddr.string), "%u.%u.%u.%u:%u", _fId.PC104ipAddr.ip1, _fId.PC104ipAddr.ip2, _fId.PC104ipAddr.ip3, _fId.PC104ipAddr.ip4, _fId.PC104ipAddr.port);

    // debug info
    memset(info, 0x00, sizeof(info));
    snprintf(info, sizeof(info), "embObjAnalogSensor: referred to EMS: %d at address %s\n", _fId.boardNum, address);
    snprintf(_fId.name, sizeof(_fId.name), "%s", info);       // Saving User Friendly Id

    // Set dummy values
    _fId.boardNum  = 255;

    Value val = config.findGroup("ETH").check("Ems", Value(1), "Board number");
    if(val.isInt())
        _fId.boardNum = val.asInt();
    else
    {
        yError () << "embObjAnalogSensor: EMS Board number identifier not found for IP" << _fId.PC104ipAddr.string;
        return false;
    }

    _fId.ep = eoprot_endpoint_analogsensors;

    ethManager = TheEthManager::instance();
    if(NULL == ethManager)
    {
        yFatal() << "Unable to instantiate ethManager";
        return false;
    }

    //N.B.: use a dynamic_cast to extract correct interface when using this pointer
    _fId.handle = (this);

    /* Once I'm ok, ask for resources, through the _fId struct I'll give the ip addr, port and
    *  and boradNum to the ethManagerin order to create the ethResource requested.
    * I'll Get back the very same sturct filled with other data useful for future handling
    * like the EPvector and EPhash_function */
    res = ethManager->requestResource(config, &_fId);
    if(NULL == res)
    {
        yError() << "EMS device not instantiated... unable to continue";
        return false;
    }

    if(!isEpManagedByBoard())
    {
        yError() << "EMS "<< _fId.boardNum << "is not connected to a analog sensor";
        return false;
    }

    data=new AnalogData(_channels, _channels+1);
    scaleFactor=new double[_channels];
    int i=0;
    for (i=0; i<_channels; i++) scaleFactor[i]=1;

    // Real values will be read from the sensor itself during its initalization hereafter
    for(int i=0; i<_channels; i++)
    {
        scaleFactor[i]=1;
    }
    // Tell EMS to go into config state, otherwise something doesn't work correctly.
//#warning "go to config message before getting strain fullscale... investigate more


//     res->goToConfig();
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

    res->goToRun();

    yTrace()<< "EmbObj Analog Sensor for board"<< _fId.boardNum << "instantiated correctly";
    return true;
}

bool embObjAnalogSensor::isEpManagedByBoard()
{
#if 1
    // marco.accame on 11 apr 2014:
    // use this code
    if(eobool_true == eoprot_endpoint_configured_is(res->get_protBRDnumber(), eoprot_endpoint_analogsensors))
    {   // we can extend to the entity by evaluating eoprot_entity_configured_is(res->get_protBRDnumber(), eoprot_endpoint_analogsensors, eoprot_entity_as_xxx)
        return true;
    }
    
    return false;

#else
    eOprotID32_t protoid;
    
    switch(_as_type)
    {
        case AS_MAIS:
        {
            protoid = eoprot_ID_get((eOprotEndpoint_t)_fId.ep, eoprot_entity_as_mais, 0, eoprot_tag_as_mais_config);
        }break;
        case AS_STRAIN:
        {
            protoid = eoprot_ID_get((eOprotEndpoint_t)_fId.ep, eoprot_entity_as_strain, 0, eoprot_tag_as_strain_config);
        }break;
        default:
        {
            //never i should be here, because if analos sensor type is unknown, then function fromConfig should be retrun with error!
            return false;
        }
    }

    // marco.accame: 
    // we can use:                              if(eobool_true == eoprot_endpoint_configured_is(res->get_protBRDnumber(), eoprot_endpoint_analogsensors))
    // we can extend to the entity by doing:   ... eoprot_entity_configured_is()
    EOnv nv;
    if(NULL == res->getNVhandler(protoid, &nv))
    {
        return false;
    }
    return true;
#endif    
}

bool embObjAnalogSensor::sendConfig2Strain(void)
{
    eOas_strain_config_t strainConfig;
    strainConfig.datarate = _period;
    strainConfig.signaloncefullscale = eobool_false;

    if(_useCalibration)
    {
        if( ! getFullscaleValues() )
        {
//             yError() << "EmbObj AnalogSensor, problem while getting fullscale values!";
            return false;
        }
        strainConfig.mode = eoas_strainmode_txcalibrateddatacontinuously;
    }
    else
    {
        strainConfig.mode = eoas_strainmode_txuncalibrateddatacontinuously;
    }


    eOprotID32_t protoid = eoprot_ID_get((eOprotEndpoint_t)_fId.ep, eoprot_entity_as_strain, 0, eoprot_tag_as_strain_config);
    res->addSetMessage(protoid, (uint8_t *) &strainConfig);

    return true;

}
bool embObjAnalogSensor::sendConfig2Mais(void)
{
    uint8_t datarate  = _period;

    // set mais datarate = 1millisec
    eOprotID32_t protoid = eoprot_ID_get((eOprotEndpoint_t)_fId.ep, eoprot_entity_as_mais, 0, eoprot_tag_as_mais_config_datarate);

    if(eobool_false == eoprot_id_isvalid(featIdBoardNum2nvBoardNum(_fId.boardNum), protoid))
    {
        yError () << " NVID not found( maisNVindex_mconfig__datarate, " << _fId.name << "board number " << _fId.boardNum << "at line" << __LINE__ << ")";
        return false;
    }

    if(!res->addSetMessage(protoid, &datarate))
    {
        yError() << "while setting mais datarate";
    }

    //set tx mode continuosly
    eOas_maismode_t     maismode  = eoas_maismode_txdatacontinuously;
    protoid = eoprot_ID_get((eOprotEndpoint_t)_fId.ep, eoprot_entity_as_mais, 0, eoprot_tag_as_mais_config_mode);

    if(eobool_false == eoprot_id_isvalid(featIdBoardNum2nvBoardNum(_fId.boardNum), protoid))
    {
        yError () << "NVID not found( maisNVindex_mconfig__mode, " << _fId.name << "board number " << _fId.boardNum << "at line" << __LINE__ << ")";
        return false;
    }

    if(!res->addSetMessage(protoid, (uint8_t *) &maismode))
    {
        yError() << "while setting mais maismode";
    }

    return true;
}

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

    eOprotID32_t protoid_fullscale = eoprot_ID_get((eOprotEndpoint_t)_fId.ep, eoprot_entity_as_strain, 0, eoprot_tag_as_strain_status_fullscale);

    if(eobool_false == eoprot_id_isvalid(featIdBoardNum2nvBoardNum(_fId.boardNum), protoid_fullscale))
        yError() << "nvid not found";
        
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
     eOprotID32_t protoid_fullscale = eoprot_ID_get((eOprotEndpoint_t)_fId.ep, eoprot_entity_as_strain, 0, eoprot_tag_as_strain_status_fullscale);

    if(eobool_false == eoprot_id_isvalid(featIdBoardNum2nvBoardNum(_fId.boardNum), protoid_fullscale))
        yError() << "nvid not found";

    p_tmpNV = res->getNVhandler(protoid_fullscale, &tmpNV);

    // tmpNVsize is the actual dimension of the array expressed in bytes, it is NOT the number of elements the array contains
    res->readBufferedValue(protoid_fullscale, (uint8_t *)&fullscale_values, &tmpNVsize);

    // when initialized, size should be zero... check it
    NVsize = eo_array_Size((EOarray*) p_tmpNV->ram);
    if(0 != NVsize)
        yError() << "Initial size of array is different from zero (" << NVsize << ") for board" << _fId.boardNum;
     
#endif
        
    // Prepare analog sensor
    eOas_strain_config_t strainConfig = {0};
    strainConfig.datarate               = _period;
    strainConfig.mode                   = eoas_strainmode_acquirebutdonttx;
    strainConfig.signaloncefullscale    = eobool_true;

    eOprotID32_t protoid_strain_config = eoprot_ID_get((eOprotEndpoint_t)_fId.ep, eoprot_entity_as_strain, 0, eoprot_tag_as_strain_config);
    //res->addSetMessage(protoid_strain_config, (uint8_t *) &strainConfig);
    timeout = 5;

    // wait for response
    while(!gotFullScaleValues && (timeout != 0))
    {
        res->addSetMessage(protoid_strain_config, (uint8_t *) &strainConfig);
        Time::delay(1);
        // read fullscale values
        res->readBufferedValue(protoid_fullscale, (uint8_t *) &fullscale_values, &tmpNVsize);
        // If data arrives, size is bigger than zero
        NVsize = eo_array_Size((EOarray *)&fullscale_values);

        if(0 != NVsize)
        {
            gotFullScaleValues = true;
            break;
        }

        timeout--;
        yDebug() << "board " << _fId.boardNum << ": full scale val not arrived yet... retrying in 1 sec";
    }

    if((false == gotFullScaleValues) && (0 == timeout))
    {
        yError() << "ETH Analog sensor: request for calibration parameters timed out for board" << _fId.boardNum;
        return false;
    }

    if((NVsize != _channels))
    {
        yError() << "Analog sensor Calibration data has a different size from channels number in configuration file for board" << _fId.boardNum << "Aborting";
        return false;
    }

    uint8_t *msg;


    if(gotFullScaleValues)
    {
        yWarning() << "GOT full scale values for board" << _fId.boardNum;

        yDebug() << "Fullscale values for board " << _fId.boardNum << "are: size=" <<  eo_array_Size((EOarray *)&fullscale_values) << "  numchannel=" <<  _channels;


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
            yDebug() << "channel " << i << "full scale value " << scaleFactor[i];
        }
    }

    return true;
  
}

// marco.accame on 11 apr 2014:
// changed the way the eOmn_ropsigcfg_command_t is prepared, as it is now in embObjMotionControl.cpp
bool embObjAnalogSensor::init()
{
    yTrace();
    
    // configure the ep of the internal feature-interface to be eoprot_endpoint_analogsensors
    _fId.ep = eoprot_endpoint_analogsensors;

    // configure the remote board to regularly send some variables

#define NEW_MN_COMMANDS

#if     defined(EOPROT_USE_MN_VERSION_1_0)

    eOmn_ropsigcfg_command_t cmdconfig  = {0};  
    eOropSIGcfg_t sigcfg                = {0};  
    eOprotID32_t IDcmdconfig            = eoprot_ID_get(eoprot_endpoint_management, eoprot_entity_mn_comm, 0, eoprot_tag_mn_comm_cmmnds_ropsigcfg);
    EOarray *array                      = eo_array_New(NUMOFROPSIGCFG, sizeof(eOropSIGcfg_t), &cmdconfig.array); 

    cmdconfig.cmmnd                 = ropsigcfg_cmd_append;
    cmdconfig.plustime              = 0;
    cmdconfig.plussign              = 0;
    cmdconfig.filler01              = 0;
    cmdconfig.signature             = eo_rop_SIGNATUREdummy;  

#else

    eOmn_cmd_config_t cmdconfig     = {0};
    eOropSIGcfg_t sigcfg            = {0};
    eOprotID32_t IDcmdconfig        = eoprot_ID_get(eoprot_endpoint_management, eoprot_entity_mn_comm, 0, eoprot_tag_mn_comm_cmmnds_command_config);
    uint16_t targetcapacity         = (sizeof(cmdconfig.array)-sizeof(eOarray_head_t)) / sizeof(eOropSIGcfg_t);
    EOarray *array                  = eo_array_New(targetcapacity, sizeof(eOropSIGcfg_t), cmdconfig.array);

    cmdconfig.opcpar.opc            = eomn_opc_config_REGROPs_append;
    cmdconfig.opcpar.plustime       = 0;
    cmdconfig.opcpar.plussign       = 0;
    cmdconfig.opcpar.dummy01        = 0;
    cmdconfig.opcpar.signature      = eo_rop_SIGNATUREdummy;

#endif     


    // now we prepare the sigcfg we want to configure on the remote board
    eOprotID32_t protoid = eo_prot_ID32dummy;
    switch(_as_type)
    {
        case AS_MAIS:
        {
            protoid = eoprot_ID_get((eOprotEndpoint_t)_fId.ep, eoprot_entity_as_mais, 0, eoprot_tag_as_mais_status_the15values);
        } break;
        
        case AS_STRAIN:
        {
            if(_useCalibration)
                protoid = eoprot_ID_get((eOprotEndpoint_t)_fId.ep, eoprot_entity_as_strain, 0, eoprot_tag_as_strain_status_calibratedvalues);
            else
                protoid = eoprot_ID_get((eOprotEndpoint_t)_fId.ep, eoprot_entity_as_strain, 0, eoprot_tag_as_strain_status_uncalibratedvalues);
        } break;
        
        default:
        {
            protoid = eo_prot_ID32dummy;
        }
    }

    if(eobool_false == eoprot_id_isvalid(featIdBoardNum2nvBoardNum(_fId.boardNum), protoid))
    {
        yError () << " EmbObj Analog Sensor NVID not found for EndPoint" << _fId.ep <<" at line " << __LINE__;
        return false;
    }
   
    sigcfg.id32 = protoid;
    
    // and we put the sigcfg inside the array
    if(eores_OK != eo_array_PushBack(array, &sigcfg))
    {
        yError () << " EmbObj Analog Sensor while loading ropSig Array  at line " << __LINE__;
        return false;
    }

    // send message to configure the spontaneous signalling in the remote board
    if( !res->addSetMessage(IDcmdconfig, (uint8_t *) &cmdconfig) )
    {
        yError() << "while setting rop sig cfg";
        return false;
    }
    
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

bool embObjAnalogSensor::fillData(void *as_array_raw, eOprotID32_t id32)
{
    bool ret;

    switch(_as_type)
    {
        case AS_MAIS:
        {
            ret = fillDatOfMais(as_array_raw);
        } break;
        
        case AS_STRAIN:
        {
            ret = fillDatOfStrain(as_array_raw);
        } break;
        
        default:
        {
            //i should not be here. if AS_NONE then i should get error in fromConfig function
            ret = false;
        }
    }

    return ret;
}





#undef as_array_raw_DBG
bool embObjAnalogSensor::fillDatOfStrain(void *as_array_raw)
{
    // called by  embObjAnalogSensor::fillData() which is called by handle_AS_data() which is called by handle_data() which is called by:
    // eoprot_fun_UPDT_as_strain_status_calibratedvalues() or eoprot_fun_UPDT_as_strain_status_uncalibratedvalues() 
    // the void* parameter inside this function is a eOas_arrayofupto12bytes_t*   
    // and can be treated as a EOarray
    
    mutex.wait();
    
#ifdef as_array_raw_DBG
    printf("\nembObj Analog Sensor fill_as_data\n");
#endif

    //eOas_arrayofupto12bytes_t  *as_array = (eOas_arrayofupto12bytes_t*) as_array_raw;
    EOarray *array = (EOarray*)as_array_raw;
    
    uint8_t msg[2];
    double *_buffer = data->getBuffer();

#ifdef as_array_raw_DBG
     printf("_useCalibration: %d\n", _useCalibration);
#endif   
  
    for(int k=0; k<_channels; k++)
    {
        // Get the kth element of the array as a 2 bytes msg
        memcpy(msg, (char*) eo_array_At(array, k), 2);
        // Got from canBusMotionControl
        _buffer[k]= (short)( ( (((unsigned short)(msg[1]))<<8)+msg[0]) - (unsigned short) (0x8000) );

#ifdef as_array_raw_DBG 
        // use it for test
        uint16_t testAAA = 0;
        uint16_t testBBB = 0;
        int16_t testCCC = 0;
        testAAA = msg[1];
        testBBB = testAAA << 8;
        testCCC = (int16_t) (testBBB + msg[0]);

//        if (k==4)
//        {
//        printf("0x%04X(%X %X) vs 0x%04X\n", (int16_t)_buffer[k], (uint8_t)msg[0], (uint8_t)msg[1], (uint16_t)testCCC);
//        printf("%d(%d %d) vs %d\n\n", (int16_t)_buffer[k], (uint8_t)msg[0], (uint8_t)msg[1], (uint16_t)testCCC);
//        }
#endif

        if (_useCalibration == 1)
        {
            _buffer[k]=_buffer[k]*scaleFactor[k]/float(0x8000);
        }
    }
    
#ifdef as_array_raw_DBG    
     printf("\n");
#endif
     
    mutex.post();
    return AS_OK;
}





bool embObjAnalogSensor::fillDatOfMais(void *as_array_raw)
{
    // called by  embObjAnalogSensor::fillData() which is called by handle_AS_data() which is called by handle_data() which is called by:
    // eoprot_fun_UPDT_as_mais_status_the15values()
    // the void* parameter inside this function is a eOas_arrayofupto36bytes_t*   
    // and can be treated as a EOarray

    //eOas_arrayofupto36bytes_t  *as_array = (eOas_arrayofupto36bytes_t*) as_array_raw;
    EOarray *array = (EOarray*)as_array_raw;
    uint8_t size = eo_array_Size(array);
    uint8_t itemsize = eo_array_ItemSize(array); // marco.accame: must be 1, as the code after uses this convention
    if(0 == size)
    {
        return false;
    }

    mutex.wait();
    double *_buffer = data->getBuffer();

    //NOTE: here i suppose that size of array is equal to num of channel or to 0 if sensor did not sent something
    //this is an agreement with firmware.
    for(int k=0; k<size; k++)
    {
        uint8_t val = *((uint8_t*)eo_array_At(array, k));       // marco.accame: i see that we treat the array as if containing items of 1 byte.
        // Get the kth element of the array
        _buffer[k] = (double)val;
    }
    mutex.post();

    return AS_OK;
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

