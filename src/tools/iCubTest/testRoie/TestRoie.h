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
#include <yarp/os/Time.h>

#include "DriverInterface.h"
#include "Test.h"
#include "TestRoieReportEntry.h"

class iCubTestRoie : public iCubTest
{
public:
    iCubTestRoie(yarp::os::Searchable& configuration);

    virtual ~iCubTestRoie();

    iCubTestReport* run();

protected:
    iCubDriver  m_icubDriver;
    iCubPart    m_part;
    std::string m_robot;
    int     m_NumJoints;
    int    *m_aCycles;
    double *m_aMaxPos;
    double *m_aMinPos;
    double *m_aRefVel;
    double *m_aTolerance;
};

#endif
