// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2009 Robotcub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Author Alessandro Scalzo alessandro@liralab.it
 */

#ifndef __ICUB_TEST_OUTPUT_01122009__
#define __ICUB_TEST_OUTPUT_01122009__

#include <string>

#include "TestXMLPrinter.h"

class iCubTestOutput
{
public:
    iCubTestOutput(){}
    void Print(XMLPrinter& printer)
    {
        printer.XMLopen("output");
            if (!m_sName.empty())   printer.XML("name",m_sName.c_str());
            if (!m_sResult.empty()) printer.XML("result",m_sResult.c_str());
            if (!m_sTarget.empty()) printer.XML("target",m_sTarget.c_str());
            if (!m_sValue.empty())  printer.XML("value",m_sValue.c_str());
            if (!m_sMinVal.empty()) printer.XML("rangemin",m_sMinVal.c_str());
            if (!m_sMaxVal.empty()) printer.XML("rangemax",m_sMaxVal.c_str());
        printer.XMLclose(); 
    }

    std::string m_sName;
    std::string m_sResult;
    std::string m_sTarget;
    std::string m_sValue;
    std::string m_sMinVal;
    std::string m_sMaxVal;
};

#endif
