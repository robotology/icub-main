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


#ifndef __SKIN_PARAMS_H__
#define __SKIN_PARAMS_H__

#include <stdio.h>

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
};

#endif

