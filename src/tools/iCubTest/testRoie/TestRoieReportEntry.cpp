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

#include "TestRoieReportEntry.h"

void iCubTestRoieReportEntry::print(XMLPrinter& printer)
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
    if (!m_Variation.empty())
    {
        printer.xml("variation",m_Variation.c_str());
    }
    if (!m_Tolerance.empty())
    {
        printer.xml("tolerance",m_Tolerance.c_str());
    }
    if (!m_Cycles.empty())
    {
        printer.xml("cyles",m_Cycles.c_str());
    }
    if (!m_MinVal.empty())
    {
        printer.xml("rangemin",m_MinVal.c_str());
    }
    if (!m_MaxVal.empty())
    {
        printer.xml("rangemax",m_MaxVal.c_str());
    }
    if (!m_RefVel.empty())
    {
        printer.xml("referenceVel",m_RefVel.c_str());
    }

    printer.xmlClose(); 
}

void iCubTestRoieReportEntry::printStdio()
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
    if (!m_Variation.empty())
    {
        fprintf(stderr,"variation %s\n",m_Variation.c_str());
    }
    if (!m_Tolerance.empty())
    {
        fprintf(stderr,"tolerance %s\n",m_Tolerance.c_str());
    }
    if (!m_Cycles.empty())
    {
        fprintf(stderr,"cyles %s\n",m_Cycles.c_str());
    }
    if (!m_MinVal.empty())
    {
        fprintf(stderr,"rangemin %s\n",m_MinVal.c_str());
    }
    if (!m_MaxVal.empty())
    {
        fprintf(stderr,"rangemax %s\n",m_MaxVal.c_str());
    }
    if (!m_RefVel.empty())
    {
        fprintf(stderr,"referenceVel %s\n",m_RefVel.c_str());
    }
}
