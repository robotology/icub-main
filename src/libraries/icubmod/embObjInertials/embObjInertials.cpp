
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


#if defined(EMBOBJINERTIALS_PUBLISH_OLDSTYLE)

typedef enum
{
    eoas_inertial1_type_none          = 0,
    eoas_inertial1_type_accelerometer = 1,
    eoas_inertial1_type_gyroscope     = 2,
    eoas_inertial1_type_emsgyro       = 3
} eOas_inertial1_type_t;

enum { eoas_inertial1_pos_max_numberof = 63 };

enum { eoas_inertial1_pos_offsetleft = 0, eoas_inertial1_pos_offsetright = 24, eoas_inertial1_pos_offsetcentral = 48 };

typedef enum
{
    eoas_inertial1_pos_none                  = 0,

    // left arm
    eoas_inertial1_pos_l_hand                = 1+eoas_inertial1_pos_offsetleft,       // label 1B7    canloc = (CAN2, 14)
    eoas_inertial1_pos_l_forearm_1           = 2+eoas_inertial1_pos_offsetleft,       // label 1B8    canloc = (CAN2, 12)
    eoas_inertial1_pos_l_forearm_2           = 3+eoas_inertial1_pos_offsetleft,       // label 1B9    canloc = (CAN2, 13)
    eoas_inertial1_pos_l_upper_arm_1         = 4+eoas_inertial1_pos_offsetleft,       // label 1B10   canloc = (CAN2,  9)
    eoas_inertial1_pos_l_upper_arm_2         = 5+eoas_inertial1_pos_offsetleft,       // label 1B11   canloc = (CAN2, 11)
    eoas_inertial1_pos_l_upper_arm_3         = 6+eoas_inertial1_pos_offsetleft,       // label 1B12   canloc = (CAN2, 10)
    eoas_inertial1_pos_l_upper_arm_4         = 7+eoas_inertial1_pos_offsetleft,       // label 1B13   canloc = (CAN2,  8)
    // left leg
    eoas_inertial1_pos_l_foot_1              = 8+eoas_inertial1_pos_offsetleft,       // label 10B12  canloc = (CAN2, 13)
    eoas_inertial1_pos_l_foot_2              = 9+eoas_inertial1_pos_offsetleft,       // label 10B13  canloc = (CAN2, 12)
    eoas_inertial1_pos_l_lower_leg_1         = 10+eoas_inertial1_pos_offsetleft,      // label 10B8   canloc = (CAN2,  8)
    eoas_inertial1_pos_l_lower_leg_2         = 11+eoas_inertial1_pos_offsetleft,      // label 10B9   canloc = (CAN2,  9)
    eoas_inertial1_pos_l_lower_leg_3         = 12+eoas_inertial1_pos_offsetleft,      // label 10B10  canloc = (CAN2, 10)
    eoas_inertial1_pos_l_lower_leg_4         = 13+eoas_inertial1_pos_offsetleft,      // label 10B11  canloc = (CAN2, 11)
    eoas_inertial1_pos_l_upper_leg_1         = 14+eoas_inertial1_pos_offsetleft,      // label 10B1   canloc = (CAN1,  1)
    eoas_inertial1_pos_l_upper_leg_2         = 15+eoas_inertial1_pos_offsetleft,      // label 10B2   canloc = (CAN1,  2)
    eoas_inertial1_pos_l_upper_leg_3         = 16+eoas_inertial1_pos_offsetleft,      // label 10B3   canloc = (CAN1,  3)
    eoas_inertial1_pos_l_upper_leg_4         = 17+eoas_inertial1_pos_offsetleft,      // label 10B4   canloc = (CAN1,  4)
    eoas_inertial1_pos_l_upper_leg_5         = 18+eoas_inertial1_pos_offsetleft,      // label 10B5   canloc = (CAN1,  5)
    eoas_inertial1_pos_l_upper_leg_6         = 19+eoas_inertial1_pos_offsetleft,      // label 10B6   canloc = (CAN1,  6)
    eoas_inertial1_pos_l_upper_leg_7         = 20+eoas_inertial1_pos_offsetleft,      // label 10B7   canloc = (CAN1,  7)

    // right arm
    eoas_inertial1_pos_r_hand                = 1+eoas_inertial1_pos_offsetright,      // label 2B7    canloc = (CAN2, 14)
    eoas_inertial1_pos_r_forearm_1           = 2+eoas_inertial1_pos_offsetright,      // label 2B8    canloc = (CAN2, 12)
    eoas_inertial1_pos_r_forearm_2           = 3+eoas_inertial1_pos_offsetright,      // label 2B9    canloc = (CAN2, 13)
    eoas_inertial1_pos_r_upper_arm_1         = 4+eoas_inertial1_pos_offsetright,      // label 2B10   canloc = (CAN2,  9)
    eoas_inertial1_pos_r_upper_arm_2         = 5+eoas_inertial1_pos_offsetright,      // label 2B11   canloc = (CAN2, 11)
    eoas_inertial1_pos_r_upper_arm_3         = 6+eoas_inertial1_pos_offsetright,      // label 2B12   canloc = (CAN2, 10)
    eoas_inertial1_pos_r_upper_arm_4         = 7+eoas_inertial1_pos_offsetright,      // label 2B13   canloc = (CAN2,  8)
    // right leg
    eoas_inertial1_pos_r_foot_1              = 8+eoas_inertial1_pos_offsetright,      // label 11B12  canloc = (CAN2, 13)
    eoas_inertial1_pos_r_foot_2              = 9+eoas_inertial1_pos_offsetright,      // label 11B13  canloc = (CAN2, 12)
    eoas_inertial1_pos_r_lower_leg_1         = 10+eoas_inertial1_pos_offsetright,     // label 11B8   canloc = (CAN2,  8)
    eoas_inertial1_pos_r_lower_leg_2         = 11+eoas_inertial1_pos_offsetright,     // label 11B9   canloc = (CAN2,  9)
    eoas_inertial1_pos_r_lower_leg_3         = 12+eoas_inertial1_pos_offsetright,     // label 11B10  canloc = (CAN2, 10)
    eoas_inertial1_pos_r_lower_leg_4         = 13+eoas_inertial1_pos_offsetright,     // label 11B11  canloc = (CAN2, 11)
    eoas_inertial1_pos_r_upper_leg_1         = 14+eoas_inertial1_pos_offsetright,     // label 11B1   canloc = (CAN1,  1)
    eoas_inertial1_pos_r_upper_leg_2         = 15+eoas_inertial1_pos_offsetright,     // label 11B2   canloc = (CAN1,  2)
    eoas_inertial1_pos_r_upper_leg_3         = 16+eoas_inertial1_pos_offsetright,     // label 11B3   canloc = (CAN1,  3)
    eoas_inertial1_pos_r_upper_leg_4         = 17+eoas_inertial1_pos_offsetright,     // label 11B5   canloc = (CAN1,  5)
    eoas_inertial1_pos_r_upper_leg_5         = 18+eoas_inertial1_pos_offsetright,     // label 11B4   canloc = (CAN1,  4)
    eoas_inertial1_pos_r_upper_leg_6         = 19+eoas_inertial1_pos_offsetright,     // label 11B6   canloc = (CAN1,  6)
    eoas_inertial1_pos_r_upper_leg_7         = 20+eoas_inertial1_pos_offsetright,     // label 11B7   canloc = (CAN1,  7)

    // central parts
    eoas_inertial1_pos_chest_1               = 1+eoas_inertial1_pos_offsetcentral,    // 9B7
    eoas_inertial1_pos_chest_2               = 2+eoas_inertial1_pos_offsetcentral,    // 9B8
    eoas_inertial1_pos_chest_3               = 3+eoas_inertial1_pos_offsetcentral,    // 9B9
    eoas_inertial1_pos_chest_4               = 4+eoas_inertial1_pos_offsetcentral,    // 9B10

    eOas_inertial1_pos_jolly_1               = 60,
    eOas_inertial1_pos_jolly_2               = 61,
    eOas_inertial1_pos_jolly_3               = 62,
    eOas_inertial1_pos_jolly_4               = 63

} eOas_inertial1_position_t;

const uint8_t fromip2indexof_thepositions[28] = { 0, 0, 1, 0, 2, 0, 0, 0, 0, 0, 3, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 0, 1, 0, 0, 2 };

const eOas_inertial1_position_t thepositions[6][2][16] =
{
    {   // none
        {   // can1
            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,
            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,
            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,
            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none
        },
        {   // can2
            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,
            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,
            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,
            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none
        }
    },
    {   // eb2
        {   // can1
            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,
            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,
            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,
            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none
        },
        {   // can2
            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,
            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,
            eoas_inertial1_pos_l_upper_arm_4,   eoas_inertial1_pos_l_upper_arm_1,   eoas_inertial1_pos_l_upper_arm_3,   eoas_inertial1_pos_l_upper_arm_2,
            eoas_inertial1_pos_l_forearm_1,     eoas_inertial1_pos_l_forearm_2,     eoas_inertial1_pos_l_hand,          eoas_inertial1_pos_none
        }
    },
    {   // eb4
        {   // can1
            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,
            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,
            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,
            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none
        },
        {   // can2
            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,
            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,
            eoas_inertial1_pos_r_upper_arm_4,   eoas_inertial1_pos_r_upper_arm_1,   eoas_inertial1_pos_r_upper_arm_3,   eoas_inertial1_pos_r_upper_arm_2,
            eoas_inertial1_pos_r_forearm_1,     eoas_inertial1_pos_r_forearm_2,     eoas_inertial1_pos_r_hand,          eoas_inertial1_pos_none
        }
    },
    {   // eb10
        {   // can1
            eoas_inertial1_pos_none,            eoas_inertial1_pos_l_upper_leg_1,   eoas_inertial1_pos_l_upper_leg_2,   eoas_inertial1_pos_l_upper_leg_3,
            eoas_inertial1_pos_l_upper_leg_4,   eoas_inertial1_pos_l_upper_leg_5,   eoas_inertial1_pos_l_upper_leg_6,   eoas_inertial1_pos_l_upper_leg_7,
            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,
            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none
        },
        {   // can2
            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,
            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,
            eoas_inertial1_pos_l_lower_leg_1,   eoas_inertial1_pos_l_lower_leg_2,   eoas_inertial1_pos_l_lower_leg_3,   eoas_inertial1_pos_l_lower_leg_4,
            eoas_inertial1_pos_l_foot_2,        eoas_inertial1_pos_l_foot_1,        eoas_inertial1_pos_none,            eoas_inertial1_pos_none
        }
    },
    {   // eb11
        {   // can1
            eoas_inertial1_pos_none,            eoas_inertial1_pos_r_upper_leg_1,   eoas_inertial1_pos_r_upper_leg_2,   eoas_inertial1_pos_r_upper_leg_3,
            eoas_inertial1_pos_r_upper_leg_5,   eoas_inertial1_pos_r_upper_leg_4,   eoas_inertial1_pos_r_upper_leg_6,   eoas_inertial1_pos_r_upper_leg_7,
            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,
            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none
        },
        {   // can2
            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,
            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,
            eoas_inertial1_pos_r_lower_leg_1,   eoas_inertial1_pos_r_lower_leg_2,   eoas_inertial1_pos_r_lower_leg_3,   eoas_inertial1_pos_r_lower_leg_4,
            eoas_inertial1_pos_r_foot_2,        eoas_inertial1_pos_r_foot_1,        eoas_inertial1_pos_none,            eoas_inertial1_pos_none
        }
    },
    {   // eb22
        {   // can1
            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,
            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_chest_1,
            eoas_inertial1_pos_chest_2,         eoas_inertial1_pos_chest_3,         eoas_inertial1_pos_chest_4,         eoas_inertial1_pos_none,
            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none
        },
        {   // can2
            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,
            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,
            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,
            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none,            eoas_inertial1_pos_none
        }
    }
};

#endif



using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;


inline bool NOT_YET_IMPLEMENTED(const char *txt)
{
    yWarning() << std::string(txt) << " not yet implemented for embObjInertials\n";
    return false;
}


bool embObjInertials::fromConfig(yarp::os::Searchable &_config)
{

    if(false == parser->parseService(_config, serviceConfig))
    {
        return false;
    }

#if defined(EMBOBJINERTIALS_PUBLISH_OLDSTYLE)

    vector<eOas_inertial1_position_t> positions;
    vector<eOas_inertial1_type_t> types;

    int size = serviceConfig.inertials.size();
    positions.resize(size, eoas_inertial1_pos_none);
    types.resize(size, eoas_inertial1_type_none);


    for(int i=0; i<size; i++)
    {
        eOas_inertial1_type_t type = eoas_inertial1_type_none;
        switch(serviceConfig.inertials.at(i).type)
        {
            case eoas_inertial_accel_mtb_int:
            case eoas_inertial_accel_mtb_ext:
            {
                type = eoas_inertial1_type_accelerometer;
            } break;

            case eoas_inertial_gyros_mtb_ext:
            {
                type = eoas_inertial1_type_gyroscope;
            } break;

            case eoas_gyros_st_l3g4200d:
            {
                type = eoas_inertial1_type_emsgyro;
            } break;

            default:
            {
                type = eoas_inertial1_type_none;
            } break;
        }

        types[i] = type;

        if(eoas_inertial1_type_emsgyro == type)
        {
            positions[i] = eOas_inertial1_pos_jolly_1;
        }
        else
        {
            // i decide to repeat the code in here because it must give error only for mtb-based positions.
            uint8_t ip4 = 10;
            eo_common_ipv4addr_to_decimal(ipv4addr, NULL, NULL, NULL, &ip4);
            if(ip4 > sizeof(fromip2indexof_thepositions))
            {
                yError() << "embObjInertials::fromConfig() is using a non supported IP address:" << boardIPstring;
                return false;
            }
            uint8_t index = fromip2indexof_thepositions[ip4];


            eObrd_location_t on = serviceConfig.inertials.at(i).on;

            eOas_inertial1_position_t pos = eoas_inertial1_pos_none;
            pos = thepositions[index][on.can.port][on.can.addr];

            positions[i] = pos;
        }
    }

    // prepare analogdata
    {
        // must be of size: 2+ inertials_Channels*erviceConfig.inertials.size(), and 0.0-initted
        analogdata.resize(2 + inertials_Channels*serviceConfig.inertials.size(), -1.0);
        // format is (n, 6) { (pos, type), (t, x, y, z) }_n
        // now: (t, x, y, z) stay -1 if nothing comes up.
        // the others must be initted now because they dont change
        analogdata[0] = serviceConfig.inertials.size();
        analogdata[1] = inertials_Channels;
        for(int i=0; i<serviceConfig.inertials.size(); i++)
        {
            int indexpos = 2 + i*inertials_Channels;
            int indextype = 2 + i*inertials_Channels + 1;
            analogdata[indexpos] = positions[i];
            analogdata[indextype] = types[i];
        }

    }

#else

    // prepare analogdata
    {
        // must be of size:  inertials_Channels*erviceConfig.inertials.size(), and -1.0-initted
        // format is { (t, x, y, z) }_n
        // now: (t, x, y, z) stay -1 if nothing comes up.
        analogdata.resize(inertials_Channels*serviceConfig.inertials.size(), -1.0);
    }

#endif

    return true;
}


embObjInertials::embObjInertials()
{
    analogdata.resize(0);

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

    parser = NULL;
    res = NULL;
}


embObjInertials::~embObjInertials()
{
    analogdata.resize(0);

    if(NULL != parser)
    {
        delete parser;
        parser = NULL;
    }
}

bool embObjInertials::initialised()
{
    return opened;
}

bool embObjInertials::open(yarp::os::Searchable &config)
{
    // - first thing to do is verify if the eth manager is available. then i parse info about the eth board.

    ethManager = eth::TheEthManager::instance();
    if(NULL == ethManager)
    {
        yFatal() << "embObjInertials::open() fails to instantiate ethManager";
        return false;
    }


    if(false == ethManager->verifyEthBoardInfo(config, ipv4addr, boardIPstring, boardName))
    {
        yError() << "embObjInertials::open(): object TheEthManager fails in parsing ETH propertiex from xml file";
        return false;
    }
    // add specific info about this device ...


    // - now all other things

    if(NULL == parser)
    {
        parser = new ServiceParser;
    }

    // read stuff from config file
    if(!fromConfig(config))
    {
        yError() << "embObjInertials missing some configuration parameter. Check logs and your config file.";
        return false;
    }


    // -- instantiate EthResource etc.

    res = ethManager->requestResource2(this, config);
    if(NULL == res)
    {
        yError() << "embObjInertials::open() fails because could not instantiate the ethResource for BOARD w/ IP = " << boardIPstring << " ... unable to continue";
        return false;
    }

    printServiceConfig();


    if(!res->verifyEPprotocol(eoprot_endpoint_analogsensors))
    {
        cleanup();
        return false;
    }

    const eOmn_serv_parameter_t* servparam = &serviceConfig.ethservice;

    if(false == res->serviceVerifyActivate(eomn_serv_category_inertials, servparam, 5.0))
    {
        yError() << "embObjInertials::open() has an error in call of ethResources::serviceVerifyActivate() for BOARD" << res->getProperties().boardnameString << "IP" << res->getProperties().ipv4addrString;
        printServiceConfig();
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
        yError() << "embObjInertials::open() fails to start as service for BOARD" << res->getProperties().boardnameString << "IP" << res->getProperties().ipv4addrString << ": cannot continue";
        cleanup();
        return false;
    }
    else
    {
        if(verbosewhenok)
        {
            yDebug() << "embObjInertials::open() correctly starts service of BOARD" << res->getProperties().boardnameString << "IP" << res->getProperties().ipv4addrString;
        }
    }


    {   // start the configured sensors. so far, we must keep it in here. later on we can remove this command

        uint8_t enable = 1;

        eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_inertial, 0, eoprot_tag_as_inertial_cmmnds_enable);
        if(false == res->setRemoteValue(id32, &enable))
        {
            yError() << "embObjInertials::open() fails to command the start transmission of the configured inertials";
            cleanup();
            return false;
        }
    }

    opened = true;
    return true;
}



bool embObjInertials::sendConfig2MTBboards(Searchable& globalConfig)
{
    eOas_inertial_config_t config = {0};
    config.datarate = serviceConfig.acquisitionrate;
    if(config.datarate > 200)
    {
        config.datarate = 200;
    }
    if(config.datarate < 10)
    {
        config.datarate = 10;
    }
    if(config.datarate != serviceConfig.acquisitionrate)
    {
        yWarning() << "embObjInertials::sendConfig2MTBboards() has detected a wrong acquisition rate =" << serviceConfig.acquisitionrate << "and clipped it to be" << config.datarate;
    }

    config.enabled=0;
    for(size_t i=0; i<serviceConfig.inertials.size(); i++)
    {
        eo_common_dword_bitset(&config.enabled, i);
    }

    eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_inertial, 0, eoprot_tag_as_inertial_config);
    if(false == res->setcheckRemoteValue(id32, &config, 10, 0.010, 0.050))
    {
        yError() << "FATAL: embObjInertials::sendConfig2MTBboards() had an error while calling setcheckRemoteValue() for config in BOARD" << res->getProperties().boardnameString << "with IP" << res->getProperties().ipv4addrString;
        return false;
    }
    else
    {
        if(verbosewhenok)
        {
            yDebug() << "embObjInertials::sendConfig2MTBboards() correctly configured enabled sensors with period" << serviceConfig.acquisitionrate << "in BOARD" << res->getProperties().boardnameString << "with IP" << res->getProperties().ipv4addrString;
        }
    }

    return true;
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
            yDebug() << "embObjInertials::initRegulars() added" << id32v.size() << "regular rops to BOARD" << res->getProperties().boardnameString << "with IP" << res->getProperties().ipv4addrString;
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
    for (size_t k = 0; k<analogdata.size(); k++)
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


eth::iethresType_t embObjInertials::type()
{
    return eth::iethres_analoginertial;
}


bool embObjInertials::update(eOprotID32_t id32, double timestamp, void* rxdata)
{
    id32 = id32;
    timestamp = timestamp;
    // timestamp is the time of reception inside EthReceiver, whereas status->data.timestamp is the time of the remote ETH board

    if(false == opened)
    {
        return false;
    }

    eOas_inertial_status_t *status = (eOas_inertial_status_t*) rxdata;

    if(status->data.id >= serviceConfig.inertials.size())
    {   // we dont have any info to manage the received position ... or the remote board did not have to send up anything meaningful
        return(true);
    }

    mutex.wait();

#if defined(EMBOBJINERTIALS_PUBLISH_OLDSTYLE)

    int firstpos = 2 + inertials_Channels*status->data.id;
    // firstpos+0 is pos, firstpos+1 is type. they dont change over time

    analogdata[firstpos+2] = (double) status->data.timestamp;

    analogdata[firstpos+3] = (double) status->data.x;
    analogdata[firstpos+4] = (double) status->data.y;
    analogdata[firstpos+5] = (double) status->data.z;

#else

    int firstpos = inertials_Channels*status->data.id;

    analogdata[firstpos+0] = (double) status->data.timestamp;

    analogdata[firstpos+1] = (double) status->data.x;
    analogdata[firstpos+2] = (double) status->data.y;
    analogdata[firstpos+3] = (double) status->data.z;

#endif

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


void embObjInertials::printServiceConfig(void)
{
    char loc[20] = {0};
    char fir[20] = {0};
    char pro[20] = {0};

    const char * boardname = (NULL != res) ? (res->getProperties().boardnameString.c_str()) : ("NOT-ASSIGNED-YET");
    const char * ipv4 = (NULL != res) ? (res->getProperties().ipv4addrString.c_str()) : ("NOT-ASSIGNED-YET");


    yInfo() << "The embObjInertials device using BOARD" << boardname << "w/ IP" << ipv4 << "has the following service config:";
    yInfo() << "- acquisitionrate =" << serviceConfig.acquisitionrate;
    yInfo() << "- number of sensors =" << serviceConfig.inertials.size() << "defined as follows:";
    for (size_t i = 0; i<serviceConfig.inertials.size(); i++)
    {
        eOas_inertial_descriptor_t des = serviceConfig.inertials.at(i);
        string id = serviceConfig.id.at(i);
        string strtype = string(eoas_sensor2string((eOas_sensor_t)des.type)); // from sensor type to string

        parser->convert(des.on, loc, sizeof(loc));
        parser->convert(serviceConfig.ethservice.configuration.data.as.inertial.mtbversion.firmware, fir, sizeof(fir));
        parser->convert(serviceConfig.ethservice.configuration.data.as.inertial.mtbversion.protocol, pro, sizeof(pro));

        yInfo() << "  - id =" << id << "type =" << strtype << "on MTB w/ loc =" << loc << "with required protocol version =" << pro << "and required firmware version =" << fir;
    }
}

// eof

