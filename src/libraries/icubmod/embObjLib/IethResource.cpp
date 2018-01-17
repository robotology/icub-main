// -*- Mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


/*
 * Copyright (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
 * Author:  Marco Accame
 * email:   marco.accame@iit.it
 * website: www.robotcub.org
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


// --------------------------------------------------------------------------------------------------------------------
// - public interface
// --------------------------------------------------------------------------------------------------------------------

#include "IethResource.h"



// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------




// --------------------------------------------------------------------------------------------------------------------
// - pimpl: private implementation (see scott meyers: item 22 of effective modern c++, item 31 of effective c++
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - the class
// --------------------------------------------------------------------------------------------------------------------

// - class IethResource

using namespace eth;

const char * IethResource::names[iethresType_numberof+1] =
{
    "resManagement", "resAnalogMais", "resAnalogStrain", "resMotionControl",
    "resSkin", "resAnalogVirtual", "resAnalogInertial", "resAnalogmultienc", "resNONE"
};

const char * IethResource::stringOfType()
{
    iethresType_t t = this->type();

    if(iethres_none == t)
    {
        return names[iethresType_numberof];
    }

    return names[t];
}


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------





