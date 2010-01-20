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

#ifndef __ICUB_TEST_REPORT_01122009__
#define __ICUB_TEST_REPORT_01122009__

#include <vector>
#include <string>
#include <time.h>

#include <yarp/os/Searchable.h>
#include <yarp/os/Value.h>

#include "TestReportEntry.h"
#include "TestMotorsReportEntry.h"

class iCubTestReport
{
public:
    iCubTestReport(std::string name,std::string part,std::string description);

    virtual ~iCubTestReport();

    void incFailures()
    {
        ++m_Failures;
    }

    int getFailures()
    {
        return m_Failures;
    }

    void printReport(XMLPrinter& printer);

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
