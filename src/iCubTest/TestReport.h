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

#include "TestReportEntry.h"
#include "TestPartReportEntry.h"


class iCubTestReport
{
public:
    iCubTestReport(std::string name,std::string part,std::string description)
    {
        time_t rawTime;
        struct tm * timeInfo;
        time(&rawTime);
        timeInfo=localtime(&rawTime);

        m_DateTime=std::string(asctime(timeInfo));
        m_Name=name;
        m_PartCode=part;
        m_Description=description;

        m_bSuccess=false;
        m_Failures=0;
        
        m_apEntries.clear();

        fprintf(stderr,"*************************\n");
        fprintf(stderr,"test name: %s\n",m_Name.c_str());
        fprintf(stderr,"part code: %s\n",m_PartCode.c_str());
        fprintf(stderr,"description: %s\n",m_Description.c_str());
    }

    void incFailures()
    {
        ++m_Failures;
    }

    int getFailures()
    {
        return m_Failures;
    }

    virtual ~iCubTestReport()
    {
        for (unsigned int i=0; i<m_apEntries.size(); ++i)
        {
            if (m_apEntries[i])
            {
                delete m_apEntries[i];
                m_apEntries[i]=NULL;
            }
        }

        m_apEntries.clear();
    }

    void printReport(XMLPrinter& printer)
    {
        char failuresStr[16];

        printer.xmlOpen("test");
            printer.xml("description",m_Description);
            printer.xmlOpen("references");
                printer.xml("datetime",m_DateTime);
                printer.xml("part",m_PartCode);
            printer.xmlClose();
            printer.xml("outcome",std::string(m_bSuccess?"SUCCESS":"FAILURE"));
            sprintf(failuresStr,"%d",m_Failures);
            printer.xml("failures",std::string(failuresStr));

            for (unsigned int i=0; i<m_apEntries.size(); ++i)
            {
                m_apEntries[i]->print(printer);
            }

        printer.xmlClose();
    }

    void setSuccess(bool success)
    {
        m_bSuccess=success;
    }

    void addEntry(iCubTestReportEntry* pEntry)
    {
        m_apEntries.push_back(pEntry);
        pEntry->printStdio();
    }

protected:
    bool m_bSuccess;

    int m_Failures;

    std::string m_Name;
    std::string m_DateTime;
    std::string m_Description;
    std::string m_PartCode;

    std::vector<iCubTestReportEntry*> m_apEntries;
};

#endif
