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

#include "TestRoie.h"

iCubTestRoie::iCubTestRoie(yarp::os::Searchable& configuration) : iCubTest(configuration)
{
    m_aCycles=NULL;
    m_aMaxPos=NULL;
    m_aMinPos=NULL;
    m_aRefVel=NULL;
    m_aTolerance=NULL;

    m_Part=(iCubDriver::iCubPart)0;

    if (configuration.check("device"))
    {
        std::string device(configuration.find("device").asString());

        for (int p=0; p<iCubDriver::NUM_ICUB_PARTS; ++p)
        {
            if (device==iCubDriver::m_aiCubPartName[p])
            {
                m_Part=(iCubDriver::iCubPart)p;
                break;
            }
        }
    }

    m_NumJoints=iCubDriver::instance()->getNumOfJoints(m_Part);
    
    ///////////////////////////////////////////////////////////////
    /*
    if (m_nJoints<=0)
    {
    printf("\nERROR\n\n");
    return;
    }
    */
    ///////////////////////////////////////////////////////////////

    if (configuration.check("tolerance"))
    {
        yarp::os::Bottle bot=configuration.findGroup("tolerance").tail();

        int n=m_NumJoints<bot.size()?m_NumJoints:bot.size();

        m_aTolerance=new double[m_NumJoints];

        for (int i=0; i<n; ++i)
        {
            if (bot.get(i).isDouble()) m_aTolerance[i]=bot.get(i).asDouble();
        }
    }

    if (configuration.check("min"))
    {
        yarp::os::Bottle bot=configuration.findGroup("min").tail();

        int n=m_NumJoints<bot.size()?m_NumJoints:bot.size();

        m_aMinPos=new double[m_NumJoints];

        for (int i=0; i<n; ++i)
        {
            if (bot.get(i).isDouble()) m_aMinPos[i]=bot.get(i).asDouble();
        }
    }

    if (configuration.check("max"))
    {
        yarp::os::Bottle bot=configuration.findGroup("max").tail();

        int n=m_NumJoints<bot.size()?m_NumJoints:bot.size();

        m_aMaxPos=new double[m_NumJoints];

        for (int i=0; i<n; ++i)
        {
            if (bot.get(i).isDouble()) m_aMaxPos[i]=bot.get(i).asDouble();
        }
    }

    if (configuration.check("refvel"))
    {
        yarp::os::Bottle bot=configuration.findGroup("refvel").tail();

        int n=m_NumJoints<bot.size()?m_NumJoints:bot.size();

        m_aRefVel=new double[m_NumJoints];

        for (int i=0; i<n; ++i)
        {
            if (bot.get(i).isDouble()) m_aRefVel[i]=bot.get(i).asDouble();
        }
    }

    if (configuration.check("cycles"))
    {
        yarp::os::Bottle bot=configuration.findGroup("cycles").tail();

        int n=m_NumJoints<bot.size()?m_NumJoints:bot.size();

        m_aCycles=new int[m_NumJoints];

        for (int i=0; i<n; ++i)
        {
            if (bot.get(i).isInt()) m_aCycles[i]=bot.get(i).asInt();
        }
    }
}

iCubTestRoie::~iCubTestRoie()
{
    if (m_aTolerance) delete [] m_aTolerance;
    if (m_aMaxPos) delete [] m_aMaxPos;
    if (m_aMinPos) delete [] m_aMinPos;
    if (m_aRefVel) delete [] m_aRefVel;
    if (m_aCycles) delete [] m_aCycles;
}

iCubTestReport* iCubTestRoie::run()
{
    iCubTestReport* pTestReport=new iCubTestReport(m_Name,m_PartCode,m_Description);

    m_bSuccess=true;

    for (int joint=0; joint<m_NumJoints; ++joint)
    {
        iCubTestRoieReportEntry *pOutput=new iCubTestRoieReportEntry();

        char jointName[8];
        char tmpString[64];
        double current_pos = 0;
        double current_rotor = 0;
        double variation = 0;

        sprintf(jointName,"Joint %d",joint);
        pOutput->m_Name=jointName;
        sprintf(tmpString,"%f",m_aMinPos[joint]);
        pOutput->m_MinVal=tmpString;
        sprintf(tmpString,"%f",m_aMaxPos[joint]);
        pOutput->m_MaxVal=tmpString;
        sprintf(tmpString,"%f",m_aRefVel[joint]);
        pOutput->m_RefVel=tmpString;
        sprintf(tmpString,"%f",m_aTolerance[joint]);
        pOutput->m_Tolerance=tmpString;

        //sprintf(posString,"%f",m_aMinPos[joint]);
        //pOutput->m_MinVal=posString;

        bool bSuccess=false;
        for (int cyc = 0; cyc< m_aCycles[joint]; cyc++)
        {
            sprintf(tmpString,"%d",cyc);
            pOutput->m_Cycles=tmpString;

            iCubDriver::ResultCode result;
            
            //goto to min position
            {
                result=iCubDriver::instance()->setPosAndWait(m_Part,joint,m_aMinPos[joint],m_aRefVel[joint]);
                if (result!=iCubDriver::IPOS_POSMOVE_OK)
                {
                    pOutput->m_Result="FAILED: iCubDriver::instance()->setPosAndWait";
                    bSuccess=false; pTestReport->incFailures(); pTestReport->addEntry(pOutput);
                }
                else
                {
                    bSuccess=true;
                }
            }

            //goto to max position
            {
                result=iCubDriver::instance()->setPosAndWait(m_Part,joint,m_aMaxPos[joint],m_aRefVel[joint]);
                if (result!=iCubDriver::IPOS_POSMOVE_OK)
                {
                    pOutput->m_Result="FAILED: iCubDriver::instance()->setPosAndWait";
                    bSuccess=false; pTestReport->incFailures(); pTestReport->addEntry(pOutput);
                }
                else
                {
                    bSuccess=true;
                }
            }

            // encoders 
            {
                result=iCubDriver::instance()->getEncPos(m_Part,joint,current_pos);
                if (result!=iCubDriver::IPOS_POSMOVE_OK)
                {
                    pOutput->m_Result="FAILED: iCubDriver::instance()->getEncPos";
                    bSuccess=false; pTestReport->incFailures(); pTestReport->addEntry(pOutput);
                    break;
                }
                else
                {
                    bSuccess=true;
                }
            }

            // rotor 
            {
                result=iCubDriver::instance()->getRotorPos(m_Part,joint,current_rotor);
                if (result!=iCubDriver::IPOS_POSMOVE_OK)
                {
                    pOutput->m_Result="FAILED: iCubDriver::instance()->getRotorPos";
                    bSuccess=false; pTestReport->incFailures(); pTestReport->addEntry(pOutput);
                }
                else
                {
                    bSuccess=true;
                }
            }

            variation = (current_pos-current_rotor);

            // this condition stop the test if something is going terribly wrong (*10)
            if (fabs(variation) > m_aTolerance[joint]*10) break;

        } //end of cycles

        if (bSuccess==false) continue;
        sprintf(tmpString,"%f",variation);
        pOutput->m_Variation=tmpString;

        if (fabs(variation) < m_aTolerance[joint])
        {
            pOutput->m_Result="SUCCESS";
        }
        else
        {
            pTestReport->incFailures();
            pOutput->m_Result="FAILED: value out of range";
        }
        
        pTestReport->addEntry(pOutput);
    }

    return pTestReport;
}
