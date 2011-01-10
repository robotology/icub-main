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

#include <yarp/os/Value.h>

#include "Test.h"

iCubTest::iCubTest(yarp::os::Searchable& configuration)
{
    if (configuration.check("name"))
    {
        m_Name=configuration.find("name").asString();
    }
    if (configuration.check("part"))
    {
        m_PartCode=configuration.find("part").asString();
    }
    if (configuration.check("description"))
    {
        m_Description=configuration.find("description").asString();
    }

    m_bIsCritical=configuration.check("critical");

    m_bSuccess=false;
}