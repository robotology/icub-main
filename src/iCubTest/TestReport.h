// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2009 Robotcub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Author Alessandro Scalzo alessandro@liralab.it
 */

#ifndef __ICUB_TEST_REPORT_01122009__
#define __ICUB_TEST_REPORT_01122009__

#include <vector>
#include <string>
#include <time.h>

#include <yarp/os/Searchable.h>
#include <yarp/os/Value.h>

#include "TestOutput.h"

class iCubTestReport
{
public:
    iCubTestReport(yarp::os::Searchable& configuration)
    {
        setDateTime();

        m_bSuccess=false;
            
        yarp::os::Value name=configuration.find("name");
        if (!name.isNull())
        {
            m_Name=name.asString();
        }

        yarp::os::Value part=configuration.find("part");
        if (!part.isNull())
        {
            m_Part=part.asString();
        }

        yarp::os::Value description=configuration.find("description");
        if (!description.isNull())
        {
            m_Description=description.asString();
        }
    }

    virtual ~iCubTestReport()
    {
        m_aOutput.clear();
    }

    void printReport(XMLPrinter& printer)
    {
        printer.xmlOpen("test");
            printer.xml("description",m_Description);
            printer.xmlOpen("references");
                printer.xml("datetime",m_DateTime);
                printer.xml("part",m_Part);
            printer.xmlClose();
            printer.xml("outcome",std::string(m_bSuccess?"SUCCESS":"FAILURE"));

            for (unsigned int i=0; i<m_aOutput.size(); ++i)
            {
                m_aOutput[i].print(printer);
            }

        printer.xmlClose();
    }

    void setSuccess(bool success)
    {
        m_bSuccess=success;
    }

    void setDateTime()
    {
        time_t rawTime;
        struct tm * timeInfo;
        time(&rawTime);
        timeInfo=localtime(&rawTime);
        m_DateTime=std::string(asctime(timeInfo));
    }

    void addEntry(iCubTestOutput& output)
    {
        m_aOutput.push_back(output);
    }

protected:
    bool m_bSuccess;

    std::string m_Name;
    std::string m_DateTime;
    std::string m_Description;
    std::string m_Part;

    std::vector<iCubTestOutput> m_aOutput;
};

#endif