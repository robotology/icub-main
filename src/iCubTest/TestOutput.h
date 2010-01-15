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
    void print(XMLPrinter& printer)
    {
        printer.xmlOpen("output");
            if (!m_Name.empty())   printer.xml("name",m_Name.c_str());
            if (!m_Result.empty()) printer.xml("result",m_Result.c_str());
            if (!m_Target.empty()) printer.xml("target",m_Target.c_str());
            if (!m_Value.empty())  printer.xml("value",m_Value.c_str());
            if (!m_MinVal.empty()) printer.xml("rangemin",m_MinVal.c_str());
            if (!m_MaxVal.empty()) printer.xml("rangemax",m_MaxVal.c_str());
        printer.xmlClose(); 
    }

    std::string m_Name;
    std::string m_Result;
    std::string m_Target;
    std::string m_Value;
    std::string m_MinVal;
    std::string m_MaxVal;
};

#endif
