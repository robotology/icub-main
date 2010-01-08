// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2009 Robotcub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Author Alessandro Scalzo alessandro@liralab.it
 */

#ifndef __ICUB_TEST_01122009__
#define __ICUB_TEST_01122009__

#include <vector>

#include <yarp/os/impl/String.h>
#include <yarp/os/Searchable.h>
#include <yarp/os/Value.h>

#include "TestOutput.h"

class iCubTest
{
public:
    iCubTest(yarp::os::Searchable& configuration)
    {
        m_bSuccess=false;
            
        yarp::os::Value name=configuration.find("name");
        if (!name.isNull())
        {
            m_sName=name.asString();
        }

        yarp::os::Value part=configuration.find("part");
        if (!part.isNull())
        {
            m_sPart=part.asString();
        }

        yarp::os::Value description=configuration.find("description");
        if (!description.isNull())
        {
            m_sDescription=description.asString();
        }
    }

    virtual ~iCubTest()
    {
        m_aOutput.clear();
    }
    
    virtual bool run()=0;

    void SetDateTime()
    {
        time_t rawtime;
        struct tm * timeinfo;
        time(&rawtime);
        timeinfo=localtime(&rawtime);
        m_sDateTime=yarp::os::impl::String(asctime(timeinfo));
    }

    void PrintReport(XMLPrinter& printer)
    {
        printer.XMLopen("test");
            printer.XML("description",m_sDescription);
            printer.XMLopen("references");
                printer.XML("datetime",m_sDateTime);
                printer.XML("part",m_sPart);
            printer.XMLclose();
            printer.XML("outcome",yarp::os::impl::String(m_bSuccess?"SUCCESS":"FAILURE"));

            for (unsigned int i=0; i<m_aOutput.size(); ++i)
            {
                m_aOutput[i].Print(printer);
            }

        printer.XMLclose();
    }


protected:
    bool m_bSuccess;
    yarp::os::impl::String m_sName;
    yarp::os::impl::String m_sDateTime;
    yarp::os::impl::String m_sDescription;
    yarp::os::impl::String m_sPart;

    std::vector<iCubTestOutput> m_aOutput;
};

#endif
