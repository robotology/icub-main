// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author:  Eric Sauser
 * email:   eric.sauser@a3.epfl.ch
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

#ifndef ICUBADDKINCHAIN_H_
#define ICUBADDKINCHAIN_H_

#include <iCub/iKinFwd.h>

namespace iKin
{
    class iCubWrist;
};
class iKin::iCubWrist : public iKin::iKinLimb
{
protected:
    virtual void _allocate_limb(const std::string &_type);

public:
    /**
    * Default constructor. 
    */
    iCubWrist();

    /**
    * Constructor. 
    * @param _type is a string to discriminate between "left" and 
    *              "right" arm
    */
    iCubWrist(const std::string &_type);

    /**
    * Creates a new Arm from an already existing Arm object.
    * @param arm is the Arm to be copied.
    */
    iCubWrist(const iCubWrist &arm);
};



#endif
