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

#include "TestMotorsStictionReportEntry.h"

void iCubTestMotorsStictionReportEntry::print(XMLPrinter& printer)
{
    printer.xmlOpen("output");
    if (!m_Name.empty())
    {
        printer.xml("name",m_Name.c_str());
    }
    if (!m_Result.empty())
    {
        printer.xml("result",m_Result.c_str());
    }
    if (!m_PWM.empty())
    {
        printer.xml("PWM",m_PWM.c_str());
    }
    if (!m_MinLim.empty())
    {
        printer.xml("MinLim",m_MinLim.c_str());
    }
    if (!m_MinLimReached.empty())
    {
        printer.xml("MinLimReached",m_MinLimReached.c_str());
    }
    if (!m_MaxLim.empty())
    {
        printer.xml("MaxLim",m_MaxLim.c_str());
    }
    if (!m_MaxLimReached.empty())
    {
        printer.xml("MaxLimReached",m_MaxLimReached.c_str());
    }
    if (!m_Tolerance.empty())
    {
        printer.xml("Tolerance",m_Tolerance.c_str());
    }
    if (!m_Timeout.empty())
    {
        printer.xml("Timeout",m_Timeout.c_str());
    }
    printer.xmlClose(); 
}

void iCubTestMotorsStictionReportEntry::printStdio()
{
    fprintf(stderr,"+\n");

    if (!m_Name.empty())
    {
        fprintf(stderr,"name %s\n",m_Name.c_str());
    }
    if (!m_Result.empty())
    {
        fprintf(stderr,"result %s\n",m_Result.c_str());
    }
    if (!m_MinLim.empty())
    {
        fprintf(stderr,"MinLim %s\n",m_MinLim.c_str());
    }
    if (!m_MinLimReached.empty())
    {
        fprintf(stderr,"MinLimReached %s\n",m_MinLimReached.c_str());
    }
    if (!m_MaxLim.empty())
    {
        fprintf(stderr,"MaxLim %s\n",m_MaxLim.c_str());
    }
    if (!m_MaxLimReached.empty())
    {
        fprintf(stderr,"MaxLimReached %s\n",m_MaxLimReached.c_str());
    }
    if (!m_Tolerance.empty())
    {
        fprintf(stderr,"Tolerance %s\n",m_Tolerance.c_str());
    }
    if (!m_Timeout.empty())
    {
        fprintf(stderr,"Timeout %s\n",m_Timeout.c_str());
    }
}
