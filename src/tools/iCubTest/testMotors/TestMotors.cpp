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

#include "TestMotors.h"

iCubTestMotors::iCubTestMotors(yarp::os::Searchable& configuration) : iCubTest(configuration)
{
    m_aTargetVal=NULL;
    m_aMaxErr=NULL;
    m_aMinErr=NULL;
    m_aRefVel=NULL;
    m_aRefAcc=NULL;
    m_aTimeout=NULL;

    m_part=(iCubPart)0;

    if (configuration.check("robot"))
    {
        m_robot = std::string (configuration.find("robot").asString());
    }
    m_icubDriver.open(m_robot);

    if (configuration.check("device"))
    {
        std::string device(configuration.find("device").asString());
        m_part = iCubPart(device);
    }

    m_NumJoints=m_icubDriver.getNumOfJoints(m_part);
    
    ///////////////////////////////////////////////////////////////
    /*
    if (m_nJoints<=0)
    {
    printf("\nERROR\n\n");
    return;
    }
    */
    ///////////////////////////////////////////////////////////////

    if (configuration.check("target"))
    {
        yarp::os::Bottle bot=configuration.findGroup("target").tail();

        int n=m_NumJoints<bot.size()?m_NumJoints:bot.size();

        m_aTargetVal=new double[m_NumJoints];

        for (int i=0; i<n; ++i)
        {
            if (bot.get(i).isDouble()) m_aTargetVal[i]=bot.get(i).asDouble();
        }
    }

    if (configuration.check("min"))
    {
        yarp::os::Bottle bot=configuration.findGroup("min").tail();

        int n=m_NumJoints<bot.size()?m_NumJoints:bot.size();

        m_aMinErr=new double[m_NumJoints];

        for (int i=0; i<n; ++i)
        {
            if (bot.get(i).isDouble()) m_aMinErr[i]=bot.get(i).asDouble();
        }
    }

    if (configuration.check("max"))
    {
        yarp::os::Bottle bot=configuration.findGroup("max").tail();

        int n=m_NumJoints<bot.size()?m_NumJoints:bot.size();

        m_aMaxErr=new double[m_NumJoints];

        for (int i=0; i<n; ++i)
        {
            if (bot.get(i).isDouble()) m_aMaxErr[i]=bot.get(i).asDouble();
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

    if (configuration.check("refacc"))
    {
        yarp::os::Bottle bot=configuration.findGroup("refacc").tail();

        int n=m_NumJoints<bot.size()?m_NumJoints:bot.size();

        m_aRefAcc=new double[m_NumJoints];

        for (int i=0; i<n; ++i)
        {
            if (bot.get(i).isDouble()) m_aRefAcc[i]=bot.get(i).asDouble();
        }
    }

    if (configuration.check("timeout"))
    {
        yarp::os::Bottle bot=configuration.findGroup("timeout").tail();

        int n=m_NumJoints<bot.size()?m_NumJoints:bot.size();

        m_aTimeout=new double[m_NumJoints];

        for (int i=0; i<n; ++i)
        {
            if (bot.get(i).isDouble()) m_aTimeout[i]=bot.get(i).asDouble();
        }
    }
}

iCubTestMotors::~iCubTestMotors()
{		
    if (m_aTargetVal) delete [] m_aTargetVal;
    if (m_aMaxErr) delete [] m_aMaxErr;
    if (m_aMinErr) delete [] m_aMinErr;
    if (m_aRefVel) delete [] m_aRefVel;
    if (m_aRefAcc) delete [] m_aRefAcc;
    if (m_aTimeout) delete [] m_aTimeout;
}

iCubTestReport* iCubTestMotors::run()
{
    iCubTestReport* pTestReport=new iCubTestReport(m_Name,m_partCode,m_Description);

    m_bSuccess=true;

    for (int joint=0; joint<m_NumJoints; ++joint)
    {
        iCubTestMotorsReportEntry *pOutput=new iCubTestMotorsReportEntry();

        char jointName[8];
        char posString[64];

        pOutput->m_Name="Joint ";
        sprintf(jointName,"%d",joint);
        pOutput->m_Name+=jointName;
        pOutput->m_Name+=" position";
        sprintf(posString,"%f",m_aTargetVal[joint]);
        pOutput->m_Target=std::string(posString);

        pOutput->m_Value="N/A";

        sprintf(posString,"%f",m_aMinErr[joint]);
        pOutput->m_MinVal=posString;

        sprintf(posString,"%f",m_aMaxErr[joint]);
        pOutput->m_MaxVal=posString;

        iCubDriver::ResultCode result=m_icubDriver.setPos(m_part,joint,m_aTargetVal[joint],m_aRefVel?m_aRefVel[joint]:0.0,m_aRefAcc?m_aRefAcc[joint]:0.0);

        bool bSetPosSuccess=false;

        switch (result)
        {
        case iCubDriver::DRIVER_FAILED:
            pOutput->m_Result="FAILED: !PolyDriver";
            break;
        case iCubDriver::IPOS_FAILED:
            pOutput->m_Result="FAILED: !IPositionControl";
            break;
        case iCubDriver::IPOS_POSMOVE_FAILED:
            pOutput->m_Result="FAILED: IPositionControl->positionMove";
            break;
        case iCubDriver::IPOS_SETREFSPEED_FAILED:
            pOutput->m_Result="FAILED: IPositionControl->setRefSpeed";
            break;
        case iCubDriver::IPOS_SETREFACC_FAILED:
            pOutput->m_Result="FAILED: IPositionControl->setRefAcceleration";
            break;
        case iCubDriver::IPOS_POSMOVE_OK:
            bSetPosSuccess=true;
        }

        if (!bSetPosSuccess)
        {
            m_bSuccess=false;
            pTestReport->incFailures();
            pTestReport->addEntry(pOutput);
            continue;
        }

        // wait some time
        yarp::os::Time::delay(m_aTimeout[joint]);

        // read encoders 
        double pos;
        result=m_icubDriver.getEncPos(m_part,joint,pos);
        bool bGetEncPosSuccess=false;
        switch (result)
        {
        case iCubDriver::IENC_FAILED:
                pOutput->m_Result="FAILED: !IEncoders";
            break;
        case iCubDriver::IENC_GETPOS_FAILED:
                pOutput->m_Result="FAILED: IEncoders->getEncoder";
            break;
        case iCubDriver::IENC_GETPOS_OK:
            sprintf(posString,"%f",pos);
            pOutput->m_Value=posString;
            bGetEncPosSuccess=true;
        }

        if (!bGetEncPosSuccess)
        {
            m_bSuccess=false;
            pTestReport->incFailures();
            pTestReport->addEntry(pOutput);
            continue;
        }

        if (pos>=m_aTargetVal[joint]+m_aMinErr[joint] && pos<=m_aTargetVal[joint]+m_aMaxErr[joint])
        {
            pOutput->m_Result="SUCCESS";
        }
        else
        {
            m_bSuccess=false;
            pTestReport->incFailures();
            pOutput->m_Result="FAILED: value out of range";
        }

        pTestReport->addEntry(pOutput);
    }

    return pTestReport;
}
