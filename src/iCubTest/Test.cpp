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
    yarp::os::Value name;
    yarp::os::Value part;
    yarp::os::Value description;

    name=configuration.find("name");
    if (!name.isNull())
    {
        m_Name=name.asString();
    }

    part=configuration.find("part");
    if (!part.isNull())
    {
        m_PartCode=part.asString();
    }

    description=configuration.find("description");
    if (!description.isNull())
    {
        m_Description=description.asString();
    }

    m_bIsCritical=configuration.check("critical");
    m_bSuccess=false;
}