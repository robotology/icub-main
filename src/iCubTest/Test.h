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


#ifndef __ICUB_TEST_01122009__
#define __ICUB_TEST_01122009__

#include <vector>
#include <string>
#include <time.h>

#include <yarp/os/Searchable.h>
#include <yarp/os/Value.h>

#include "TestReport.h"

class iCubTest
{
public:
    iCubTest(yarp::os::Searchable& configuration)
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

    virtual ~iCubTest()
    {
    }
    
    virtual iCubTestReport* run()=0;

    bool getSuccess()
    {
        return m_bSuccess;
    }

    bool isCritical()
    {
        return m_bIsCritical;
    }

protected:
    bool m_bSuccess;
    bool m_bIsCritical;

    std::string m_Name;
    std::string m_Description;
    std::string m_PartCode;
};

#endif
