// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Alessandro Scalzo
 * email: alessandro.scalzo@iit.it
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

#ifndef __ICUB_TEST_MOTORS_01122009__
#define __ICUB_TEST_MOTORS_01122009__

#include <vector>
#include <string>

#include <yarp/os/Searchable.h>
#include <yarp/os/Value.h>

#include "DriverInterface.h"
#include "Test.h"
#include "TestMotorsReportEntry.h"

class iCubTestMotors : public iCubTest
{
public:
    iCubTestMotors(yarp::os::Searchable& configuration);

    virtual ~iCubTestMotors();

    iCubTestReport* run();

protected:
    iCubDriver::iCubPart m_Part;
    int m_NumJoints;
    double *m_aTargetVal;
    double *m_aMaxErr;
    double *m_aMinErr;
    double *m_aRefVel;
    double *m_aRefAcc;
    double *m_aTimeout;
};

#endif
