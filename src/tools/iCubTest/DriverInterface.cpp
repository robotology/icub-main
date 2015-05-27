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

void iCubDriver::open (std::string robotName)
{
    m_RobotName = robotName;
    for (int part=0; part<NUM_ICUB_PARTS; ++part)
    {
        m_apDriver[part]=openDriver(iCubPart(part));
        m_apDbgDriver[part]=openDebugDriver(iCubPart(part));

        bool b = true;
        
        if (m_apDbgDriver[part])
        {
            b &= m_apDbgDriver[part]->view(m_apDbg[part]); if (b==false)  fprintf(stderr,"m_apDbgDriver[part]->view(m_apDbg) failed\n");
        }
        if (m_apDriver[part])
        {
            b &= m_apDriver[part]->view(m_apEnc[part]);    if (b==false)  fprintf(stderr,"m_apDriver[part]->view(m_apEnc) failed\n");
            b &= m_apDriver[part]->view(m_apPid[part]);    if (b==false)  fprintf(stderr,"m_apDriver[part]->view(m_apPid) failed\n");
            b &= m_apDriver[part]->view(m_apAmp[part]);    if (b==false)  fprintf(stderr,"m_apDriver[part]->view(m_apAmp) failed\n");
            b &= m_apDriver[part]->view(m_apPos[part]);    if (b==false)  fprintf(stderr,"m_apDriver[part]->view(m_apPos) failed\n");
            b &= m_apDriver[part]->view(m_apVel[part]);    if (b==false)  fprintf(stderr,"m_apDriver[part]->view(m_apPos) failed\n");
            b &= m_apDriver[part]->view(m_apTrq[part]);    if (b==false)  fprintf(stderr,"m_apDriver[part]->view(m_apTrq) failed\n");
            b &= m_apDriver[part]->view(m_apImp[part]);    if (b==false)  fprintf(stderr,"m_apDriver[part]->view(m_apImp) failed\n");
            b &= m_apDriver[part]->view(m_apOpl[part]);    if (b==false)  fprintf(stderr,"m_apDriver[part]->view(m_apOpl) failed\n");
            b &= m_apDriver[part]->view(m_apCtl[part]);    if (b==false)  fprintf(stderr,"m_apDriver[part]->view(m_apCtl) failed\n");
            b &= m_apDriver[part]->view(m_apLim[part]);    if (b==false)  fprintf(stderr,"m_apDriver[part]->view(m_apLim) failed\n");
            
            if (m_apEnc[part])
            {
                m_apEnc[part]->getAxes(&m_aiCubPartNumJoints[part]);
            }
        }
    }
}

iCubDriver::iCubDriver()
{
    for (int part=0; part<NUM_ICUB_PARTS; ++part)
    {
        m_aiCubPartNumJoints[part]=0;
        m_apEnc[part]=0;
        m_apPid[part]=0;
        m_apAmp[part]=0;
        m_apPos[part]=0;
        m_apVel[part]=0;
        m_apTrq[part]=0;
        m_apImp[part]=0;
        m_apOpl[part]=0;
        m_apCtl[part]=0;
        m_apLim[part]=0;
        m_apDbg[part]=0;
    }
}

void iCubDriver::close()
{
    for (int part=0; part<NUM_ICUB_PARTS; ++part)
    {
        if (m_apDriver[part] && m_apDriver[part]->isValid())
        {
            m_apDriver[part]->close();
            delete m_apDriver[part];
            m_apDriver[part]=NULL; 
        }
        if (m_apDbgDriver[part] && m_apDbgDriver[part]->isValid())
        {
            m_apDbgDriver[part]->close();
            delete m_apDbgDriver[part];
            m_apDbgDriver[part]=NULL; 
        }
    }
}

yarp::dev::PolyDriver* iCubDriver::openDebugDriver(std::string part)
{
    yarp::dev::PolyDriver *pDriver=NULL;

    yarp::os::Property options;
    options.put("robot",m_RobotName.c_str());
    options.put("device","debugInterfaceClient");
    options.put("local",(std::string("/iCubTestDebug/")+part).c_str());
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

iCubDriver::ResultCode iCubDriver::startOpenloopCmd(int part,int joint,double pwm)
{
    if (!m_apDriver[part])        
    {
        return DRIVER_FAILED;
    }

    if (!m_apOpl[part] || !m_apCtl[part])
    {
        return IOPL_OPLSTART_FAILED;
    }
    
    if (!m_apCtl[part]->setOpenLoopMode(joint))
    {
        return IOPL_OPLSTART_FAILED;
    }

//    if (!m_apOpl[part]->setOutput(joint,pwm))
    {
        return IOPL_OPLSTART_FAILED;
    }

    return IOPL_OPLSTART_OK;
}

iCubDriver::ResultCode iCubDriver::stopOpenloopCmd(int part,int joint)
{
    if (!m_apDriver[part])
    {
        return DRIVER_FAILED;
    }

    if (!m_apOpl[part] || !m_apCtl[part])
    {
        return IOPL_OPLSTOP_FAILED;
    }

    if (!m_apOpl[part]->setRefOutput(joint,0.0))
    {
        return IOPL_OPLSTOP_FAILED;
    }

    if (!m_apCtl[part]->setPositionMode(joint))
    {
        return IOPL_OPLSTOP_FAILED;
    }

    return IOPL_OPLSTOP_OK;
}

iCubDriver::ResultCode iCubDriver::getPosPidSign(int part,int joint,double &posPidSign)
{
    posPidSign = 1.0;

    if (!m_apDriver[part])
    {
        return DRIVER_FAILED;
    }

    if (!m_apPid[part])
    {
        return IPID_GETPOSPID_FAILED;
    }

    yarp::dev::Pid pid;
    if (!m_apPid[part]->getPid(joint,&pid))
    {
        return IPID_GETPOSPID_FAILED;
    }

    if (pid.kp<0) posPidSign = -1.0;

    return IPID_GETPOSPID_OK;
}

iCubDriver::ResultCode iCubDriver::getJointLimits(int part,int joint, double& min, double& max)
{
    if (!m_apDriver[part])
    {
        return DRIVER_FAILED;
    }

    if (!m_apLim[part])
    {
        return ILIM_GETLIM_FAILED;
    }

    if (!m_apLim[part]->getLimits(joint, &min, &max))
    {
         return ILIM_GETLIM_FAILED;
    }

    return ILIM_GETLIM_OK;
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

iCubDriver::ResultCode iCubDriver::setPosAndWait(int part,int joint,double position,double speed,double acc, double tolerance, double timeout)
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

    if (!m_apPos[part]->positionMove(joint,position))
    {
        return IPOS_POSMOVE_FAILED;
    }

    int count=0;
    double current_pos=0.0;
    do
    {
        ResultCode r = getEncPos(part,joint,current_pos);
        if (r<0) return IPOS_POSMOVE_NOT_REACHED;
        yarp::os::Time::delay(0.020);
    }
    while (fabs(current_pos-position)>tolerance && (count++)<(timeout/0.020));

    if (fabs(current_pos-position)>tolerance)
        return IPOS_POSMOVE_NOT_REACHED;

    return IPOS_POSMOVE_OK;
}

iCubDriver::ResultCode iCubDriver::checkMotionDone(int part,int joint,double timeout)
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

    //return IENC_GETPOS_OK;
    printf("Encoder err\n");
    return IENC_GETPOS_FAILED;
}

iCubDriver::ResultCode iCubDriver::getRotorPos(int part,int joint,double &pos)
{
    if (!m_apDbg[part])
    {
        return IDBG_FAILED;
    }

    if (m_apDbg[part]->getRotorPosition(joint,&pos))
    {
        return IDBG_GETROTPOS_OK;
    }

    return IDBG_GETROTPOS_FAILED;
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
