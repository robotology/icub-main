// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


/* Copyright (C) 2012  iCub Facility, Istituto Italiano di Tecnologia
 * Author: Valentina Gaggero
 * email: valentina.gaggero@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef SKIN_CONFIG_READER
#define SKIN_CONFIG_READER

#include <string>


//#include <ace/ACE.h>
#include <Debug.h>
#include <stdint.h>





#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

//#include "skinParams.h"


//default values for SkinBoardCfgParam
#define sk_period_default                40 //millisec
#define sk_skintype_default              0
#define sk_noLoad_default                0xf0

class SkinBoardCfgParam
{
public:
    uint8_t                     period;
    uint8_t                     noLoad;
    uint8_t                     skinType;

public:
    void setDefaultValues(void)
    {
        period                  = sk_period_default;
        noLoad                  = sk_noLoad_default;
        skinType                = sk_skintype_default;
    };
    void debugPrint(void)
    {
        yDebug() << "period=" << period << "; noLoad=" << noLoad << "; skinType=" << skinType;
    }

};




//default values for triangles
#define skT_enabled_default      true
#define skT_shift_default        0
#define skT_cdcOffset_default    0x2200
class SkinTriangleCfgParam
{
public:
    bool                          enabled;
    uint8_t                       shift;
    uint16_t                      cdcOffset;

    void setDefaultValues(void)
    {
        enabled                   = skT_enabled_default;
        shift                     = skT_shift_default;
        cdcOffset                 = skT_cdcOffset_default;
    }

    void debugPrint(void)
    {
        yDebug() << "enabled=" << enabled << "; shift=" << shift << "; cdcOffset=" << cdcOffset;
    }
};


class SpecialSkinBoardCfgParam
{
public:
    int                 patch; //in eth version it means number of patch where board is connected to, while in can means canDeviceNumber
    int                 boardAddrStart;
    int                 boardAddrEnd; //in eth version it means number of patch where board is connected to, while in can means canDeviceNumber
    SkinBoardCfgParam   cfg;

    void debugPrint(void)
    {
        yDebug() << "patch num " << patch << ": startAddr=" << boardAddrStart << "endAddr=" << boardAddrEnd << "with cfg:";
        cfg.debugPrint();
    }
};

class SpecialSkinTriangleCfgParam
{
public:
    int                  patch; //in eth version it means number of patch where board is connected to, while in can means canDeviceNumber
    int                  boardAddr;
    int                  triangleStart;
    int                  triangleEnd;
    SkinTriangleCfgParam cfg;

    void debugPrint(void)
    {
        yDebug() << "boardAddr " << boardAddr<< " on patch num " << patch << ": trisngleStart=" << triangleStart << "endAddr=" << triangleEnd << "with cfg:";
        cfg.debugPrint();
    }
};

#define CONFIG_READER_NAME_LEN 30
class SkinConfigReader
{
private:
    char                _name[CONFIG_READER_NAME_LEN];
    
public:
    SkinConfigReader(char *name);
    SkinConfigReader();
    bool isDefaultBoardCfgPresent(yarp::os::Searchable& config);
    bool readDefaultBoardCfg(yarp::os::Searchable& config, SkinBoardCfgParam *boardCfg);
    bool isDefaultTriangleCfgPresent(yarp::os::Searchable& config);
    bool readDefaultTriangleCfg(yarp::os::Searchable& config, SkinTriangleCfgParam *triangCfg);
    bool readSpecialBoardCfg(yarp::os::Searchable& config, SpecialSkinBoardCfgParam *boardCfg, int *numofcfg);
    bool readSpecialTriangleCfg(yarp::os::Searchable& config, SpecialSkinTriangleCfgParam *triangleCfg, int *numofcfg);
};

#endif
