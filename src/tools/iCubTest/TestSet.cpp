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

#include "TestXMLPrinter.h"
#include "TestSet.h"

#include <stdio.h>

iCubTestSet::iCubTestSet(yarp::os::Searchable& configuration)
{   
    m_bSuccess=false;
    m_numFailures=0;

    if (configuration.check("user"))
    {
        m_User=configuration.find("user").asString();
    }
    if (configuration.check("comment"))
    {
        m_Comment=configuration.find("comment").asString();
    }
    if (configuration.check("outfile"))
    {
        m_Outfile=configuration.find("outfile").asString();
    }
}

iCubTestSet::~iCubTestSet()
{
    for (unsigned int t=0; t<m_apTest.size(); ++t)
    {
        if (m_apTest[t])
        {
            delete m_apTest[t];
            m_apTest[t]=NULL;
        }
    }

    for (unsigned int t=0; t<m_apReport.size(); ++t)
    {
        if (m_apReport[t])
        {
            delete m_apReport[t];
            m_apReport[t]=NULL;
        }
    }

    m_apTest.clear();
    m_apReport.clear();
}

int iCubTestSet::run()
{
    m_bSuccess=true;
    m_numFailures=0;

    m_apReport.clear();

    for (unsigned int i=0; i<m_apTest.size(); ++i)
    {
        m_apReport.push_back(m_apTest[i]->run());

        m_bSuccess=m_bSuccess && m_apTest[i]->getSuccess();
        m_numFailures+=m_apReport[i]->getFailures();   

        if (!m_apTest[i]->getSuccess() && m_apTest[i]->isCritical())
        {
            break;
        }
    }

    return m_numFailures;
}

void iCubTestSet::printReport()
{
    XMLPrinter printer(m_Outfile);

    char failuresStr[16];
    sprintf(failuresStr,"%d",m_numFailures);

    printer.xmlOpen("report");
    printer.xml("user",m_User);
    printer.xml("comment",m_Comment);
    printer.xml("success",std::string(m_bSuccess?"YES":"NO"));

    printer.xml("failures-total",std::string(failuresStr));

    for (unsigned int i=0; i<m_apReport.size(); ++i)
    {
        m_apReport[i]->printReport(printer);   
    }

    printer.xmlClose();
}
