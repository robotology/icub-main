// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
 * Authors: Marco Randazzo
 * email:  marco.randazzo@iit.it
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


#ifndef __ICUB_TEST_MOTORS_STICTION_INCREMENTAL__
#define __ICUB_TEST_MOTORS_STICTION_INCREMENTAL__

#include <vector>
#include <string>

#include <yarp/os/Searchable.h>
#include <yarp/os/Value.h>

#include "DriverInterface.h"
#include "Test.h"
#include "TestMotorsStictionIncrementalReportEntry.h"

class iCubTestMotorsStictionIncremental : public iCubTest
{
public:
    iCubTestMotorsStictionIncremental(yarp::os::Searchable& configuration);

    virtual ~iCubTestMotorsStictionIncremental();

    iCubTestReport* run();

protected:
    iCubDriver  m_icubDriver;
    iCubPart    m_part;
    std::string m_robot;
    int     m_NumJoints;
};

#endif
