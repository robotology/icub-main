// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2009 Robotcub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Author Alessandro Scalzo alessandro@liralab.it
 */

#ifndef __ICUB_TEST_OUTPUT_01122009__
#define __ICUB_TEST_OUTPUT_01122009__

#include <yarp/os/impl/String.h>

#include "TestXMLPrinter.h"

class iCubTestOutput
{
public:
    iCubTestOutput(){}
    void Print(XMLPrinter& printer)
    {
        printer.XMLopen("output");
            if (!m_sName.is_empty())   printer.XML("name",m_sName.c_str());
            if (!m_sResult.is_empty()) printer.XML("result",m_sResult.c_str());
            if (!m_sTarget.is_empty()) printer.XML("target",m_sTarget.c_str());
            if (!m_sValue.is_empty())  printer.XML("value",m_sValue.c_str());
            if (!m_sMinVal.is_empty()) printer.XML("rangemin",m_sMinVal.c_str());
            if (!m_sMaxVal.is_empty()) printer.XML("rangemax",m_sMaxVal.c_str());
        printer.XMLclose(); 
    }

    yarp::os::impl::String m_sName;
    yarp::os::impl::String m_sResult;
    yarp::os::impl::String m_sTarget;
    yarp::os::impl::String m_sValue;
    yarp::os::impl::String m_sMinVal;
    yarp::os::impl::String m_sMaxVal;
};

#endif
