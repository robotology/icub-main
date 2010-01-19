// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2009 Robotcub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Author Alessandro Scalzo alessandro@liralab.it
 */

#ifndef __ICUB_TEST_PART_01122009__
#define __ICUB_TEST_PART_01122009__

#include <vector>
#include <string>

#include <yarp/os/Searchable.h>
#include <yarp/os/Value.h>

#include "DriverInterface.h"
#include "Test.h"
#include "TestPartReportEntry.h"

class iCubTestPart : public iCubTest
{
public:
    iCubTestPart(yarp::os::Searchable& configuration) : iCubTest(configuration)
    {
        yarp::os::Value val;
        yarp::os::Bottle bot;

        m_aTargetVal=NULL;
        m_aMaxVal=NULL;
        m_aMinVal=NULL;
        m_aRefVel=NULL;
        m_aRefAcc=NULL;
        m_aTimeout=NULL;

        m_Part=(iCubDriver::iCubPart)0;

        val=configuration.find("device");

        if (!val.isNull())
        {
            std::string device(val.asString());

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

        bot=configuration.findGroup("target").tail();
        if (!bot.isNull())
        {
            m_aTargetVal=new double[m_NumJoints];
            int n=m_NumJoints<bot.size()?m_NumJoints:bot.size();
            
            for (int i=0; i<n; ++i)
            {
                if (bot.get(i).isDouble()) m_aTargetVal[i]=bot.get(i).asDouble();
            }
        }

        bot=configuration.findGroup("min");
        if (!bot.isNull())
        {
            m_aMinVal=new double[m_NumJoints];
            bot=bot.tail();
            int n=m_NumJoints<bot.size()?m_NumJoints:bot.size();
            
            for (int i=0; i<n; ++i)
            {
                if (bot.get(i).isDouble()) m_aMinVal[i]=bot.get(i).asDouble();
            }
        }


        bot=configuration.findGroup("max");
        if (!bot.isNull())
        {
            m_aMaxVal=new double[m_NumJoints];
            bot=bot.tail();
            int n=m_NumJoints<bot.size()?m_NumJoints:bot.size();
            
            for (int i=0; i<n; ++i)
            {
                if (bot.get(i).isDouble()) m_aMaxVal[i]=bot.get(i).asDouble();
            }
        }

        bot=configuration.findGroup("refvel");
        if (!bot.isNull())
        {
            m_aRefVel=new double[m_NumJoints];
            bot=bot.tail();
            int n=m_NumJoints<bot.size()?m_NumJoints:bot.size();
            
            for (int i=0; i<n; ++i)
            {
                if (bot.get(i).isDouble()) m_aRefVel[i]=bot.get(i).asDouble();
            }
        }

        bot=configuration.findGroup("refacc");
        if (!bot.isNull())
        {
            m_aRefAcc=new double[m_NumJoints];
            bot=bot.tail();
            int n=m_NumJoints<bot.size()?m_NumJoints:bot.size();
            
            for (int i=0; i<n; ++i)
            {
                if (bot.get(i).isDouble()) m_aRefAcc[i]=bot.get(i).asDouble();
            }
        }

        bot=configuration.findGroup("timeout");
        if (!bot.isNull())
        {
            m_aTimeout=new double[m_NumJoints];
            bot=bot.tail();
            int n=m_NumJoints<bot.size()?m_NumJoints:bot.size();
            
            for (int i=0; i<n; ++i)
            {
                if (bot.get(i).isDouble()) m_aTimeout[i]=bot.get(i).asDouble();
            }
        }
    }

    virtual ~iCubTestPart()
    {		
        if (m_aTargetVal)
        {
            delete [] m_aTargetVal;
        }
        if (m_aMaxVal)
        {
            delete [] m_aMaxVal;
        }
        if (m_aMinVal)
        {
            delete [] m_aMinVal;
        }
        if (m_aRefVel)
        {
            delete [] m_aRefVel;
        }
        if (m_aRefAcc)
        {
            delete [] m_aRefAcc;
        }
        if (m_aTimeout)
        {
            delete [] m_aTimeout;
        }
    }

    iCubTestReport run()
    {
        iCubTestReport testReport(m_Name,m_PartCode,m_Description);
            
        m_bSuccess=true;

        for (int joint=0; joint<iCubDriver::instance()->getNumOfJoints(iCubDriver::LEFT_ARM); ++joint)
        {
            iCubTestPartReportEntry *pOutput=new iCubTestPartReportEntry;
            char jointName[8];
            char posString[64];

            pOutput->m_Name="Joint ";
            sprintf(jointName,"%d",joint);
            pOutput->m_Name+=jointName;
            pOutput->m_Name+=" position";
            sprintf(posString,"%f",m_aTargetVal[joint]);
            pOutput->m_Target=std::string(posString);

            pOutput->m_Value="N/A";
            
            sprintf(posString,"%f",m_aMinVal[joint]);
            pOutput->m_MinVal=posString;
            
            sprintf(posString,"%f",m_aMaxVal[joint]);
            pOutput->m_MaxVal=posString;

            iCubDriver::ResultCode result=iCubDriver::instance()->setPos(m_Part,joint,m_aTargetVal[joint],m_aRefVel[joint],m_aRefAcc[joint]);

            bool bSuccess=false;

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
                bSuccess=true;
            }

            if (!bSuccess)
            {
                m_bSuccess=false;
                testReport.incFailures();
                testReport.addEntry(pOutput);
                continue;
            }

            // only if success

            result=iCubDriver::instance()->waitPos(m_Part,joint,m_aTimeout[joint]);

            bSuccess=false;

            switch (result)
            {
            case iCubDriver::IPOS_FAILED:
                pOutput->m_Result="FAILED: !IPositionControl";
                break;
            case iCubDriver::IPOS_CHECKMOTIONDONE_FAILED:
                pOutput->m_Result="FAILED: IPositionControl->checkMotionDone";
                break;
            case iCubDriver::IPOS_CHECKMOTIONDONE_TIMEOUT:
                pOutput->m_Result="FAILED: Timeout in IPositionControl->positionMove";
                break;
            case iCubDriver::IPOS_CHECKMOTIONDONE_OK:
                bSuccess=true;
            }

            if (!bSuccess)
            {
                m_bSuccess=false;
                testReport.incFailures();
                testReport.addEntry(pOutput);
                continue;
            }

            double pos;
            result=iCubDriver::instance()->getEncPos(m_Part,joint,pos);

            bSuccess=false;

            switch (result)
            {
            case iCubDriver::IENC_FAILED:
                pOutput->m_Result="FAILED: !IEncoders";
                break;
            case iCubDriver::IENC_GETPOS_FAILED:
                pOutput->m_Result="FAILED: IEncoders->getEncoder";
				break;
            case iCubDriver::IENC_GETPOS_OK:
                bSuccess=true;
            }

            if (!bSuccess)
            {
                m_bSuccess=false;
                testReport.incFailures();
                testReport.addEntry(pOutput);
                continue;
            }

            sprintf(posString,"%f",pos);
            pOutput->m_Value=posString;

            if (pos>=m_aMinVal[joint] && pos<=m_aMaxVal[joint])
            {
                pOutput->m_Result="SUCCESS";
            }
            else
            {
                m_bSuccess=false;
                testReport.incFailures();
                pOutput->m_Result="FAILED: value out of range";
            }

            testReport.addEntry(pOutput);
        }
        
        return testReport;
    }

protected:
    iCubDriver::iCubPart m_Part;
    int m_NumJoints;
    double *m_aTargetVal;
    double *m_aMaxVal;
    double *m_aMinVal;
    double *m_aRefVel;
    double *m_aRefAcc;
    double *m_aTimeout;
};

#endif
