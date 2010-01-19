// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2009 Robotcub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Author Alessandro Scalzo alessandro@liralab.it
 */

#ifndef __ICUB_TEST_SET_01122009__
#define __ICUB_TEST_SET_01122009__

#include <string>

#include <yarp/os/Searchable.h>
#include <yarp/os/Value.h>

#include <vector>

#include "TestXMLPrinter.h"
#include "Test.h"

class iCubTestSet
{
public:
    iCubTestSet(yarp::os::Searchable& configuration)
    {   
        m_bSuccess=false;
         
        yarp::os::Value user=configuration.find("user");
        if (!user.isNull())
        {
            m_User=user.asString();
        }

        yarp::os::Value comment=configuration.find("comment");
        if (!comment.isNull())
        {
            m_Comment=comment.asString();
        }

        yarp::os::Value outfile=configuration.find("outfile");
        if (!outfile.isNull())
        {
            m_Outfile=outfile.asString();
        }
    }
    
    ~iCubTestSet()
    {
        m_apTest.clear();
        m_aReport.clear();
    }
    
    void addTest(iCubTest* pTest)
    {
        m_apTest.push_back(pTest);
    }

    void run()
    {
        m_bSuccess=true;

        m_aReport.clear();
        
        for (unsigned int i=0; i<m_apTest.size(); ++i)
        {
            m_aReport.push_back(m_apTest[i]->run());

            m_bSuccess = m_bSuccess && m_apTest[i]->getSuccess();

            if (!m_apTest[i]->getSuccess() && m_apTest[i]->isCritical())
            {
                return;
            }
        }
    }

    void printReport()
    {
        int numFailures=0;
        char failuresStr[16];

        XMLPrinter printer(m_Outfile);

        printer.xmlOpen("report");
            printer.xml("user",m_User);
            printer.xml("comment",m_Comment);
            printer.xml("success",std::string(m_bSuccess?"YES":"NO"));

            for (unsigned int i=0; i<m_aReport.size(); ++i)
            {
                numFailures+=m_aReport[i].getFailures();   
            }

            sprintf(failuresStr,"%d",numFailures);

            printer.xml("failures-total",std::string(failuresStr));

            for (unsigned int i=0; i<m_aReport.size(); ++i)
            {
                m_aReport[i].printReport(printer);   
            }

        printer.xmlClose();
    }

protected:
    bool m_bSuccess;
    std::string m_Comment;
    std::string m_User;
    std::string m_Outfile;
    std::vector<iCubTest*> m_apTest;
    std::vector<iCubTestReport> m_aReport;
};

#endif
