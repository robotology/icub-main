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

#include "TestMotorsStictionIncrementalReportEntry.h"

void iCubTestMotorsStictionIncrementalReportEntry::print(XMLPrinter& printer)
{
    printer.xmlOpen("output");
    if (!m_Name.empty())
    {
        printer.xml("name",m_Name.c_str());
    }
    if (!m_trial.empty())
    {
        printer.xml("trial",m_trial.c_str());
    }
    if (!m_PWM.empty())
    {
        printer.xml("PWM",m_PWM.c_str());
    }
    if (!m_PWM.empty())
    {
        printer.xml("displacement",m_displacement.c_str());
    }
    if (!m_PWM.empty())
    {
        printer.xml("speed",m_speed.c_str());
    }

    printer.xmlClose(); 
}

void iCubTestMotorsStictionIncrementalReportEntry::printStdio()
{
    fprintf(stderr,"+\n");

    if (!m_Name.empty())
    {
        fprintf(stderr,"name %s\n",m_Name.c_str());
    }
    if (!m_trial.empty())
    {
        fprintf(stderr,"trial %s\n",m_trial.c_str());
    }
    if (!m_PWM.empty())
    {
        fprintf(stderr,"PWM %s\n",m_PWM.c_str());
    }
    if (!m_PWM.empty())
    {
        fprintf(stderr,"displacament %s\n",m_displacement.c_str());
    }
    if (!m_PWM.empty())
    {
        fprintf(stderr,"speed %s\n",m_speed.c_str());
    }
}
