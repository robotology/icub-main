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

#include <yarp/os/Time.h>

#include "DriverInterface.h"

const char* iCubDriver::m_aiCubPartName[NUM_ICUB_PARTS]={"torso","head","left_arm","right_arm","left_leg","right_leg"};

std::string iCubDriver::m_RobotName;

iCubDriver::iCubDriver()
{
    for (int part=TORSO; part<NUM_ICUB_PARTS; ++part)
    {
        m_aiCubPartNumJoints[part]=0;

        m_apDriver[part]=openDriver(m_aiCubPartName[part]);

        if (m_apDriver[part])
        {
            m_apDriver[part]->view(m_apEnc[part]);
            m_apDriver[part]->view(m_apPid[part]);
            m_apDriver[part]->view(m_apAmp[part]);
            m_apDriver[part]->view(m_apPos[part]);
            m_apDriver[part]->view(m_apVel[part]);

            if (m_apEnc[part])
            {
                m_apEnc[part]->getAxes(&m_aiCubPartNumJoints[part]);
            }

            for (int j=0; j<m_aiCubPartNumJoints[part]; ++j)
            {
                if (m_apPid[part])
                {
                    m_apPid[part]->enablePid(j);
                }

                if (m_apAmp[part])
                {
                    m_apAmp[part]->enableAmp(j);
                }
            }
        }
    }
}

void iCubDriver::close()
{
    for (int part=TORSO; part<NUM_ICUB_PARTS; ++part)
    {
        for (int j=0; j<m_aiCubPartNumJoints[part]; ++j)
        {
            m_apPid[part]->disablePid(j);
            m_apAmp[part]->disableAmp(j);
        }
    }
}

yarp::dev::PolyDriver* iCubDriver::openDriver(std::string part)
{
    std::string robot("/icubSim");
    yarp::dev::PolyDriver *pDriver=NULL;

    yarp::os::Property options;
    options.put("robot",m_RobotName.c_str());
    options.put("device","remote_controlboard");
    options.put("local",(std::string("/iCubTest/")+part).c_str());
    options.put("remote",(m_RobotName+"/"+part).c_str());

    pDriver=new yarp::dev::PolyDriver(options);

    if (!pDriver->isValid()) 
    {
        pDriver->close();
        delete pDriver;
        pDriver=NULL; 
    }

    return pDriver;
}

iCubDriver::ResultCode iCubDriver::setPos(int part,int joint,double position,double speed/*=0.0*/,double acc/*=0.0*/)
{
    if (!m_apDriver[part])        
    {
        return DRIVER_FAILED;
    }

    if (!m_apPos[part])
    {
        return IPOS_FAILED;
    }

    if (acc!=0.0)
    {
        if (!m_apPos[part]->setRefAcceleration(joint,acc))
        {
            return IPOS_SETREFACC_FAILED;
        }
    }

    if (speed!=0.0)
    {
        if (!m_apPos[part]->setRefSpeed(joint,speed))
        {
            return IPOS_SETREFSPEED_FAILED;
        }
    }

    if (m_apPos[part]->positionMove(joint,position))
    {
        return IPOS_POSMOVE_OK;
    }

    return IPOS_POSMOVE_FAILED;
}

iCubDriver::ResultCode iCubDriver::waitPos(int part,int joint,double timeout)
{
    static const double dTimeStep=0.1;
    bool bFlag;

    if (!m_apPos[part])
    {
        return IPOS_FAILED;
    }

    for (double dTimeElaps=0.0; dTimeElaps<timeout; dTimeElaps+=dTimeStep)
    {
        bool bSuccess=m_apPos[part]->checkMotionDone(joint,&bFlag);

        if (!bSuccess)
        {
            return IPOS_CHECKMOTIONDONE_FAILED;
        }

        if (bFlag)
        {
            return IPOS_CHECKMOTIONDONE_OK;
        }

        yarp::os::Time::delay(dTimeStep);
    }

    return IPOS_CHECKMOTIONDONE_TIMEOUT;
}

iCubDriver::ResultCode iCubDriver::getPIDError(int part,int joint,double &err)
{
    if (m_apPid[part]->getError(joint,&err))
    {
        return IPID_GETERROR_OK;
    }

    return IPID_GETERROR_FAILED;
}

iCubDriver::ResultCode iCubDriver::getEncPos(int part,int joint,double &pos)
{
    if (!m_apEnc[part])
    {
        return IENC_FAILED;
    }

    if (m_apEnc[part]->getEncoder(joint,&pos))
    {
        return IENC_GETPOS_OK;
    }

    return IENC_GETPOS_FAILED;
}

iCubDriver::ResultCode iCubDriver::getEncSpeed(int part,int joint,double &spd)
{
    if (!m_apEnc[part])
    {
        return IENC_FAILED;
    }

    if (m_apEnc[part]->getEncoderSpeed(joint,&spd))
    {
        return IENC_GETSPEED_OK;
    }

    return IENC_GETSPEED_FAILED;
}

iCubDriver::ResultCode iCubDriver::getEncAcc(int part,int joint,double &acc)
{
    if (!m_apEnc[part])
    {
        return IENC_FAILED;
    }

    if (m_apEnc[part]->getEncoderAcceleration(joint,&acc))
    {
        return IENC_GETACC_OK;
    }

    return IENC_GETACC_FAILED;
}