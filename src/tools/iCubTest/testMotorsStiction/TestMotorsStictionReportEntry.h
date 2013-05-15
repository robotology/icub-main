// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
 * Authors: Marco Randazzo
 * email:  marco.randazzo@iit.it
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

#ifndef __ICUB_TEST_MOTORS_STICTION_REPORT_ENTRY_01122009__
#define __ICUB_TEST_MOTORS_STICTION_REPORT_ENTRY_01122009__

#include "TestReportEntry.h"

class iCubTestMotorsStictionReportEntry : public iCubTestReportEntry
{
public:
    iCubTestMotorsStictionReportEntry(){}
    virtual ~iCubTestMotorsStictionReportEntry(){}

    virtual void print(XMLPrinter& printer);
    
    virtual void printStdio();

    std::string m_Name;
    std::string m_Result;
    std::string m_PWM;
    std::string m_MinLim;
    std::string m_MinLimReached;
    std::string m_MaxLim;
    std::string m_MaxLimReached;
    std::string m_Tolerance;
    std::string m_Timeout;
};

#endif
