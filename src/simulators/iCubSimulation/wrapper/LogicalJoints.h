/* 
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Paul Fitzpatrick
* email:  paulfitz@alum.mit.edu
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

#ifndef ICUBSIMULATION_LOGICALJOINTS_INC
#define ICUBSIMULATION_LOGICALJOINTS_INC

#include "LogicalJoint.h"

#include <yarp/dev/DeviceDriver.h>

/**
 *
 * Route control for the robots controlled joints to their implementation.
 *
 */
class LogicalJoints : public yarp::dev::DeviceDriver {
public:
    virtual ~LogicalJoints() {}

    /**
     *
     * Access the control for a logical joint, based on part and axis 
     * number.
     *
     */
    virtual LogicalJoint& control(int part, int axis) = 0;
};


#endif
