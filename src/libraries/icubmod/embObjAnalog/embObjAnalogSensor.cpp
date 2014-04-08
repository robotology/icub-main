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

#ifdef WIN32
#pragma warning(once:4355)
#endif

const int REPORT_PERIOD=6; //seconds
const double BCAST_STATUS_TIMEOUT=6; //seconds

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

//generic function that check is key1 is present in input bottle and that the result has size elements
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
    if(config.findGroup("GENERAL").find("Verbose").asInt())
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


    // Get both PC104 and EMS ip addresses and port from config file
    groupEth  = Bottle(config.findGroup("ETH"));
    Bottle parameter1( groupEth.find("PC104IpAddress").asString() );    // .findGroup("IpAddress");
    port      = groupEth.find("CmdPort").asInt();              // .get(1).asInt();
    sprintf(_fId.PC104ipAddr.string, "%s", parameter1.toString().c_str(), port);
    _fId.PC104ipAddr.port = port;

    Bottle parameter2( groupEth.find("IpAddress").asString() );    // .findGroup("IpAddress");
    sprintf(_fId.EMSipAddr.string, "%s", parameter2.toString().c_str());
    _fId.EMSipAddr.port = port;

    sscanf(_fId.EMSipAddr.string,"\"%d.%d.%d.%d", &_fId.EMSipAddr.ip1, &_fId.EMSipAddr.ip2, &_fId.EMSipAddr.ip3, &_fId.EMSipAddr.ip4);
    sscanf(_fId.PC104ipAddr.string,"\"%d.%d.%d.%d", &_fId.PC104ipAddr.ip1, &_fId.PC104ipAddr.ip2, &_fId.PC104ipAddr.ip3, &_fId.PC104ipAddr.ip4);

    sprintf(_fId.EMSipAddr.string,"%u.%u.%u.%u:%u", _fId.EMSipAddr.ip1, _fId.EMSipAddr.ip2, _fId.EMSipAddr.ip3, _fId.EMSipAddr.ip4, _fId.EMSipAddr.port);
    sprintf(_fId.PC104ipAddr.string,"%u.%u.%u.%u:%u", _fId.PC104ipAddr.ip1, _fId.PC104ipAddr.ip2, _fId.PC104ipAddr.ip3, _fId.PC104ipAddr.ip4, _fId.PC104ipAddr.port);

    //   Debug info
    memset(info, 0x00, SIZE_INFO);
    sprintf(info, "embObjAnalogSensor: referred to EMS: %d at address %s\n", _fId.boardNum, address);
    sprintf(_fId.name, "%s", info);       // Saving User Friendly Id

    // Set dummy values
    _fId.boardNum  = 255;

    Value val =config.findGroup("ETH").check("Ems",Value(1), "Board number");
    if(val.isInt())
        _fId.boardNum =val.asInt();
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
    res = ethManager->requestResource(&_fId);
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

#define MSG010979 "WARNING->go to config message before getting strain fullscale... investigate more"
#if defined(_MSC_VER)
    #pragma message(MSG010979)
#else
    #warning MSG010989
#endif

//     res->goToConfig();
    switch(_as_type)
    {
        case AS_MAIS:
        {
            ret = sendConfig2Mais();
        }break;
        case AS_STRAIN:
        {
            ret = sendConfig2Strain();
        }break;
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
    eOprotID32_t protoid;
    EOnv         nv;

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


    if(NULL == res->getNVhandler(protoid, &nv))
    {
        return false;
    }
    return true;
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

    //set mais datarate = 1millisec
    eOprotID32_t protoid = eoprot_ID_get((eOprotEndpoint_t)_fId.ep, eoprot_entity_as_mais, 0, eoprot_tag_as_mais_config_datarate);
    if(EOK_uint16dummy == protoid)
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
    if(EOK_uint16dummy == protoid)
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
    // Check inital size of array...  it should be zero.
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
    if(EOK_uint16dummy == protoid_fullscale)
        yError() << "nvid not found";

    p_tmpNV = res->getNVhandler(protoid_fullscale, &tmpNV);

    // tmpNVsize is the actual dimension of the array expressed in bytes, it is NOT the number of elements the array contains
    res->readBufferedValue(protoid_fullscale, (uint8_t *)&fullscale_values, &tmpNVsize);

    // when initialized, size shuold be zero... check it
    NVsize = eo_array_Size((EOarray*) p_tmpNV->ram);
    if(0 != NVsize)
        yError() << "Initial size of array is different from zero (" << NVsize << ") for board" << _fId.boardNum;

     // Prepare analog sensor
    eOas_strain_config_t strainConfig;
    strainConfig.datarate = _period;
    strainConfig.mode = eoas_strainmode_acquirebutdonttx;
    strainConfig.signaloncefullscale = eobool_true;

    eOprotID32_t protoid_strain_config = eoprot_ID_get((eOprotEndpoint_t)_fId.ep, eoprot_entity_as_strain, 0, eoprot_tag_as_strain_config);
    res->addSetMessage(protoid_strain_config, (uint8_t *) &strainConfig);

    timeout = 5;

    // wait for rensponse
    while(!gotFullScaleValues && (timeout != 0))
    {
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
        yDebug() << "full scale val not arrived yet... retrying in 1 sec";
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

bool embObjAnalogSensor::init()
{
    yTrace();
    eOprotID32_t              protoid;

    // Configure values to be sent regularly
    eOmn_ropsigcfg_command_t  *ropsigcfgassign;           // pointer to the type, to handle content
    EOnv                      nv_ropsigcfgassign;         // memory to hold stuff
    EOnv                      *nv_ropsigcfgassign_ptr;    // pointer to the nv_ropsigcfgassign
    eOropSIGcfg_t             sigcfg;                     // struct for the single element to be signalled
    EOarray                   *array;                     // array containing nvids to be signalled

    _fId.ep = eoprot_endpoint_analogsensors;

    eOprotID32_t protoid_ropsigcfgassign = eoprot_ID_get(eoprot_endpoint_management, eoprot_entity_mn_comm, 0, eoprot_tag_mn_comm_cmmnds_ropsigcfg );
    nv_ropsigcfgassign_ptr = res->getNVhandler(protoid_ropsigcfgassign, &nv_ropsigcfgassign);

    ropsigcfgassign = (eOmn_ropsigcfg_command_t*) nv_ropsigcfgassign_ptr->ram;
    array = (EOarray*) &ropsigcfgassign->array;
    eo_array_Reset(array);
    array->head.capacity = NUMOFROPSIGCFG;
    array->head.itemsize = sizeof(eOropSIGcfg_t);
    ropsigcfgassign->cmmnd = ropsigcfg_cmd_append;

    switch(_as_type)
    {
        case AS_MAIS:
        {
            protoid = eoprot_ID_get((eOprotEndpoint_t)_fId.ep, eoprot_entity_as_mais, 0, eoprot_tag_as_mais_status_the15values);
        }break;
        case AS_STRAIN:
        {
            if(_useCalibration)
                protoid = eoprot_ID_get((eOprotEndpoint_t)_fId.ep, eoprot_entity_as_strain, 0, eoprot_tag_as_strain_status_calibratedvalues);
            else
                protoid = eoprot_ID_get((eOprotEndpoint_t)_fId.ep, eoprot_entity_as_strain, 0, eoprot_tag_as_strain_status_uncalibratedvalues);
        }break;
        default:
        {
            protoid=EOK_uint16dummy;
        }
    }

    if(EOK_uint16dummy == protoid)
    {
        yError () << " EmbObj Analog Sensor NVID not found for EndPoint" << _fId.ep <<" at line " << __LINE__;
        return false;
    }

    sigcfg.id32 = protoid;
    if(eores_OK != eo_array_PushBack(array, &sigcfg))
    {
        yError () << " EmbObj Analog Sensor while loading ropSig Array  at line " << __LINE__;
        return false;
    }

    // Send message
    if( !res->addSetMessage(protoid_ropsigcfgassign, (uint8_t *) ropsigcfgassign) )
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

bool embObjAnalogSensor::fillData(void *as_array_raw)
{
    bool ret;

    switch(_as_type)
    {
        case AS_MAIS:
        {
            ret = fillDatOfMais(as_array_raw);
        }break;
        case AS_STRAIN:
        {
            ret = fillDatOfStrain(as_array_raw);
        }break;
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
    // Called by eoCallback.
    mutex.wait();
//     printf("\nembObj Analog Sensor fill_as_data\n");
    // do the decode16 code
    eOas_arrayofupto12bytes_t  *as_array = (eOas_arrayofupto12bytes_t*) as_array_raw;
    uint8_t msg[2];
    double *_buffer = data->getBuffer();

//     printf("_useCalibration: %d\n", _useCalibration);
    for(int k=0; k<_channels; k++)
    {
        // Get the kth element of the array as a 2 bytes msg
        memcpy(msg, (char*) eo_array_At((EOarray*) as_array, k), 2);
        // Got from canBusMotionControl
        _buffer[k]= (short)( ( (((unsigned short)(msg[1]))<<8)+msg[0]) - (unsigned short) (0x8000) );

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
//

        if (_useCalibration==1)
        {
            _buffer[k]=_buffer[k]*scaleFactor[k]/float(0x8000);
        }
    }
//     printf("\n");
    mutex.post();
    return AS_OK;
}





bool embObjAnalogSensor::fillDatOfMais(void *as_array_raw)
{
    // Called by eoCallback.

    eOas_arrayofupto36bytes_t  *as_array = (eOas_arrayofupto36bytes_t*) as_array_raw;
    uint8_t size = eo_array_Size((EOarray *)as_array);
    if(0 == size)
    {
        return false;
    }

    mutex.wait();
    double *_buffer = data->getBuffer();

    //NOTE: here i suppose that size of array is equal to num of channel or to 0 if sensor did not sent something
    //this is an agreement with firmware.
    for(int k=0; k<size; k++)
    //for(int k=0; k<_channels; k++)
    {
        uint8_t val = *((uint8_t*)eo_array_At((EOarray*) as_array, k));
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


