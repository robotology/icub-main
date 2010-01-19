// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2009 Robotcub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Author Alessandro Scalzo alessandro@liralab.it
 */

#ifndef __ICUB_TEST_01122009__
#define __ICUB_TEST_01122009__

// sostituire stringhe // OK
// nome classe file    // OK
// ritornare i report come return di run() // OK
// risultato dei test su consolle
// numero di fallimenti nei valori di ritorno dei test // OK
// criticità nei fallimenti // oK
// nome: iCubDriver // OK
// 

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
    
    virtual iCubTestReport run()=0;

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
