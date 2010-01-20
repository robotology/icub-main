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

#include "TestReport.h"

iCubTestReport::iCubTestReport(std::string name,std::string part,std::string description)
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

iCubTestReport::~iCubTestReport()
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

void iCubTestReport::printReport(XMLPrinter& printer)
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
