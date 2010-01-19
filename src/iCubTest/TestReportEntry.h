// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2009 Robotcub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Author Alessandro Scalzo alessandro@liralab.it
 */

#ifndef __ICUB_TEST_REPORT_ENTRY_01122009__
#define __ICUB_TEST_REPORT_ENTRY_01122009__

#include <string>

#include "TestXMLPrinter.h"

class iCubTestReportEntry
{
public:
    iCubTestReportEntry(){}
    virtual ~iCubTestReportEntry(){}

    virtual void print(XMLPrinter& printer)=0;
    virtual void printStdio()=0;
};

#endif
