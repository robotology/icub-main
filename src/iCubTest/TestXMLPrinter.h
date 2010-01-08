// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2009 Robotcub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Author Alessandro Scalzo alessandro@liralab.it
 */

#ifndef __ICUB_TEST_XML_PRINTER_01122009__
#define __ICUB_TEST_XML_PRINTER_01122009__

#include <vector>

#include <yarp/os/impl/String.h>

class XMLPrinter
{
public:
    XMLPrinter(yarp::os::impl::String& filename)
    {
        m_pFile=fopen(filename.c_str(),"wb");

        m_Tabs=0;
        m_Stack.clear();
    }
    
    ~XMLPrinter()
    {
        if (m_pFile)
        {
            fclose(m_pFile);
        }
        m_pFile=0;
        m_Stack.clear();
    }
    
    void XML(char* tag,yarp::os::impl::String data)
    {
        Tabs();
        fprintf(m_pFile,"<%s>%s</%s>\n",tag,data.c_str(),tag);
    }
    void XMLopen(char *tag)
    {
        Tabs(1);
        m_Stack.push_back(yarp::os::impl::String(tag));
        fprintf(m_pFile,"<%s>\n",tag);
    }
    void XMLclose()
    {
        Tabs(-1);
        fprintf(m_pFile,"</%s>\n",m_Stack.back().c_str());
        m_Stack.pop_back();
    }

protected:
    FILE *m_pFile;
    int m_Tabs;
    std::vector<yarp::os::impl::String> m_Stack;

    void Tabs(int move=0)
    {
        if (move<0)
        {
            m_Tabs+=move;
        }

        for (int t=0; t<m_Tabs; ++t)
        {
            fprintf(m_pFile,"\t");
        }

        if (move>0) 
        {
            m_Tabs+=move;
        }
    }
};

#endif