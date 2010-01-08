// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2009 Robotcub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Author Alessandro Scalzo alessandro@liralab.it
 */

#ifndef __ICUB_TEST_SET_01122009__
#define __ICUB_TEST_SET_01122009__

#include <yarp/os/impl/String.h>
//#include <yarp/os/Time.h>
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
            m_sUser=user.asString();
        }

        yarp::os::Value comment=configuration.find("comment");
        if (!comment.isNull())
        {
            m_sComment=comment.asString();
        }

        yarp::os::Value outfile=configuration.find("outfile");
        if (!outfile.isNull())
        {
            m_sOutfile=outfile.asString();
        }
    }
    
    ~iCubTestSet()
    {
        m_apTest.clear();
    }
    
    void AddTest(iCubTest* pTest)
    {
        m_apTest.push_back(pTest);
    }

    void run()
    {
        m_bSuccess=true;
        
        for (unsigned int i=0; i<m_apTest.size(); ++i)
        {
            m_bSuccess = m_bSuccess && m_apTest[i]->run();
        }
    }

    void PrintReport()
    {
        XMLPrinter printer(m_sOutfile);

        printer.XMLopen("report");
            printer.XML("user",m_sUser);
            printer.XML("comment",m_sComment);
            printer.XML("success",yarp::os::impl::String(m_bSuccess?"YES":"NO"));

            for (unsigned int i=0; i<m_apTest.size(); ++i)
            {
                m_apTest[i]->PrintReport(printer);   
            }

        printer.XMLclose();
    }

protected:
    bool m_bSuccess;
    yarp::os::impl::String m_sComment;
    yarp::os::impl::String m_sUser;
    yarp::os::impl::String m_sOutfile;
    std::vector<iCubTest*> m_apTest;
};

#endif
