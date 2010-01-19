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

#ifndef __ICUB_TEST_XML_PRINTER_01122009__
#define __ICUB_TEST_XML_PRINTER_01122009__

#include <vector>
#include <string>

class XMLPrinter
{
public:
    XMLPrinter(std::string& fileName)
    {
        m_pFile=fopen(fileName.c_str(),"wb");

        m_Tabs=0;
        m_Stack.clear();
    }
    
    ~XMLPrinter()
    {
        if (m_pFile)
        {
            fclose(m_pFile);
        }
        m_pFile=NULL;
        m_Stack.clear();
    }
    
    void xml(const char* tag,std::string data)
    {
        tabs();
        fprintf(m_pFile,"<%s>%s</%s>\n",tag,data.c_str(),tag);
    }
    void xmlOpen(const char *tag)
    {
        tabs(+1);
        m_Stack.push_back(std::string(tag));
        fprintf(m_pFile,"<%s>\n",tag);
    }
    void xmlClose()
    {
        tabs(-1);
        fprintf(m_pFile,"</%s>\n",m_Stack.back().c_str());
        m_Stack.pop_back();
    }

protected:
    FILE *m_pFile;
    int m_Tabs;
    std::vector<std::string> m_Stack;

    void tabs(int move=0)
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
