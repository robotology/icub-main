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

/*
 This file contains the default values for the optional configuration paramters.
 In the future these values could be asked to the firmware
 */

#ifndef MOTIONCONTROLDEFAULTVALUES
#define MOTIONCONTROLDEFAULTVALUES
namespace eomc_defaultValue 
{
    namespace DeadZone
    {
        const double jointWithAEA = 0.0494;
        const double jointWithAMO = 0.0055;
    }
}
#endif
