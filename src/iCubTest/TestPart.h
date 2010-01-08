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

#include <yarp/os/impl/String.h>
#include <yarp/os/Searchable.h>
#include <yarp/os/Value.h>

#include "TestMotorInterface.h"
#include "Test.h"

class iCubTestPart : public iCubTest
{
public:
    iCubTestPart(yarp::os::Searchable& configuration) : iCubTest(configuration)
    {
        m_dTargetVal=NULL;
        m_dMaxVal=NULL;
        m_dMinVal=NULL;
        m_dRefVel=NULL;
        m_dRefAcc=NULL;
        m_dTimeout=NULL;

        yarp::os::Value val;
        yarp::os::Bottle bot;

        m_iPart=(iCubMotorDriver::iCubPart)0;

        val=configuration.find("device");

        if (!val.isNull())
        {
            yarp::os::impl::String device(val.asString());

            for (int p=0; p<iCubMotorDriver::NUM_ICUB_PARTS; ++p)
            {
                if (device==iCubMotorDriver::iCubPartName[p])
                {
                    m_iPart=(iCubMotorDriver::iCubPart)p;
                    break;
                }
            }
        }
        
        m_nJoints=iCubMotorDriver::Instance()->GetNumOfJoints(m_iPart);

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
            m_dTargetVal=new double[m_nJoints];
            int n=m_nJoints<bot.size()?m_nJoints:bot.size();
            
            for (int i=0; i<n; ++i)
            {
                if (bot.get(i).isDouble()) m_dTargetVal[i]=bot.get(i).asDouble();
            }
        }

        bot=configuration.findGroup("min");
        if (!bot.isNull())
        {
            m_dMinVal=new double[m_nJoints];
            bot=bot.tail();
            int n=m_nJoints<bot.size()?m_nJoints:bot.size();
            
            for (int i=0; i<n; ++i)
            {
                if (bot.get(i).isDouble()) m_dMinVal[i]=bot.get(i).asDouble();
            }
        }


        bot=configuration.findGroup("max");
        if (!bot.isNull())
        {
            m_dMaxVal=new double[m_nJoints];
            bot=bot.tail();
            int n=m_nJoints<bot.size()?m_nJoints:bot.size();
            
            for (int i=0; i<n; ++i)
            {
                if (bot.get(i).isDouble()) m_dMaxVal[i]=bot.get(i).asDouble();
            }
        }

        bot=configuration.findGroup("refvel");
        if (!bot.isNull())
        {
            m_dRefVel=new double[m_nJoints];
            bot=bot.tail();
            int n=m_nJoints<bot.size()?m_nJoints:bot.size();
            
            for (int i=0; i<n; ++i)
            {
                if (bot.get(i).isDouble()) m_dRefVel[i]=bot.get(i).asDouble();
            }
        }

        bot=configuration.findGroup("refacc");
        if (!bot.isNull())
        {
            m_dRefAcc=new double[m_nJoints];
            bot=bot.tail();
            int n=m_nJoints<bot.size()?m_nJoints:bot.size();
            
            for (int i=0; i<n; ++i)
            {
                if (bot.get(i).isDouble()) m_dRefAcc[i]=bot.get(i).asDouble();
            }
        }

        bot=configuration.findGroup("timeout");
        if (!bot.isNull())
        {
            m_dTimeout=new double[m_nJoints];
            bot=bot.tail();
            int n=m_nJoints<bot.size()?m_nJoints:bot.size();
            
            for (int i=0; i<n; ++i)
            {
                if (bot.get(i).isDouble()) m_dTimeout[i]=bot.get(i).asDouble();
            }
        }
    }

    virtual ~iCubTestPart()
    {		
        if (m_dTargetVal) delete [] m_dTargetVal;
        if (m_dMaxVal) delete [] m_dMaxVal;
        if (m_dMinVal) delete [] m_dMinVal;
        if (m_dRefVel) delete [] m_dRefVel;
        if (m_dRefAcc) delete [] m_dRefAcc;
        if (m_dTimeout) delete [] m_dTimeout;
    }

    bool run()
    {
        SetDateTime();
        
        m_bSuccess=true;

        for (int joint=0; joint<iCubMotorDriver::Instance()->GetNumOfJoints(iCubMotorDriver::LEFT_ARM); ++joint)
        {
            iCubTestOutput output;
            
            output.m_sName="Joint ";
            char sJoint[8],sPos[64];
            sprintf(sJoint,"%d",joint);
            output.m_sName+=sJoint;
            output.m_sName+=" position";
            sprintf(sPos,"%f",m_dTargetVal[joint]);
            output.m_sTarget=yarp::os::impl::String(sPos);

            output.m_sValue="N/A";
            
            sprintf(sPos,"%f",m_dMinVal[joint]);
            output.m_sMinVal=sPos;
            
            sprintf(sPos,"%f",m_dMaxVal[joint]);
            output.m_sMaxVal=sPos;

            iCubMotorDriver::ResultCode result=iCubMotorDriver::Instance()->SetPos(m_iPart,joint,m_dTargetVal[joint]);//,m_dRefVel[joint],m_dRefAcc[joint]);

            bool bSuccess=false;

            switch (result)
            {
            case iCubMotorDriver::DRIVER_FAILED:
                output.m_sResult="FAILED: !PolyDriver";
                break;
            case iCubMotorDriver::IPOS_FAILED:
                output.m_sResult="FAILED: !IPositionControl";
                break;
            case iCubMotorDriver::IPOS_POSMOVE_FAILED:
                output.m_sResult="FAILED: IPositionControl->positionMove";
                break;
            case iCubMotorDriver::IPOS_SETREFSPEED_FAILED:
                output.m_sResult="FAILED: IPositionControl->setRefSpeed";
                break;
            case iCubMotorDriver::IPOS_SETREFACC_FAILED:
                output.m_sResult="FAILED: IPositionControl->setRefAcceleration";
                break;
            case iCubMotorDriver::IPOS_POSMOVE_OK:
                bSuccess=true;
            }

            if (!bSuccess)
            {
                m_bSuccess=false;
                m_aOutput.push_back(output);
                continue;
            }

            // only if success

            result=iCubMotorDriver::Instance()->WaitPos(m_iPart,joint,m_dTimeout[joint]);

            bSuccess=false;

            switch (result)
            {
            case iCubMotorDriver::IPOS_FAILED:
                output.m_sResult="FAILED: !IPositionControl";
                break;
            case iCubMotorDriver::IPOS_CHECKMOTIONDONE_FAILED:
                output.m_sResult="FAILED: IPositionControl->checkMotionDone";
                break;
            case iCubMotorDriver::IPOS_CHECKMOTIONDONE_TIMEOUT:
                output.m_sResult="FAILED: Timeout in IPositionControl->positionMove";
                break;
            case iCubMotorDriver::IPOS_CHECKMOTIONDONE_OK:
                bSuccess=true;
            }

            if (!bSuccess)
            {
                m_bSuccess=false;
                m_aOutput.push_back(output);
                continue;
            }

            double pos;
            result=iCubMotorDriver::Instance()->GetEncPos(m_iPart,joint,pos);

            bSuccess=false;

            switch (result)
            {
            case iCubMotorDriver::IENC_FAILED:
                output.m_sResult="FAILED: !IEncoders";
                break;
            case iCubMotorDriver::IENC_GETPOS_FAILED:
                output.m_sResult="FAILED: IEncoders->getEncoder";
				break;
            case iCubMotorDriver::IENC_GETPOS_OK:
                bSuccess=true;
            }

            if (!bSuccess)
            {
                m_bSuccess=false;
                m_aOutput.push_back(output);
                continue;
            }

            sprintf(sPos,"%f",pos);
            output.m_sValue=sPos;

            if (pos>=m_dMinVal[joint] && pos<=m_dMaxVal[joint])
            {
                output.m_sResult="SUCCESS";
            }
            else
            {
                m_bSuccess=false;
                output.m_sResult="FAILED: value out of range";
            }

            m_aOutput.push_back(output);
        }
        
        return m_bSuccess;
    }

protected:
    iCubMotorDriver::iCubPart m_iPart;
    int m_nJoints;
    double *m_dTargetVal,*m_dMaxVal,*m_dMinVal;
    double *m_dRefVel,*m_dRefAcc,*m_dTimeout;
};

#endif
