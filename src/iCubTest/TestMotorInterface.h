// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2009 Robotcub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Author Alessandro Scalzo alessandro@liralab.it
 */

#ifndef __ICUB_MOTOR_DRIVER_01122009__
#define __ICUB_MOTOR_DRIVER_01122009__

#include <vector>

#include <yarp/os/impl/String.h>
#include <yarp/os/Time.h>
#include <yarp/os/Searchable.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>

class iCubMotorDriver
{
public:
    enum iCubPart {TORSO,HEAD,LEFT_ARM,RIGHT_ARM,LEFT_LEG,RIGHT_LEG,NUM_ICUB_PARTS};

    enum ResultCode 
    { 
        DRIVER_FAILED               = -1,

        IPOS_FAILED                 = -2,
        IPOS_SETREFACC_FAILED       = -3,
        IPOS_SETREFSPEED_FAILED     = -4,
        IPOS_POSMOVE_OK             = +5,
        IPOS_POSMOVE_FAILED         = -6,
        IPOS_CHECKMOTIONDONE_FAILED = -7,
        IPOS_CHECKMOTIONDONE_OK     = +8,
        IPOS_CHECKMOTIONDONE_TIMEOUT= -9,
        
        IPID_GETERROR_OK            =+10,
        IPID_GETERROR_FAILED        =-11,
        
        IENC_FAILED                 =-12,
        IENC_GETPOS_OK              =+13,
        IENC_GETPOS_FAILED          =-14,
        IENC_GETSPEED_OK            =+15,
        IENC_GETSPEED_FAILED        =-16,
        IENC_GETACC_OK              =+17,
        IENC_GETACC_FAILED          =-18
    };

    static iCubMotorDriver* Instance()
    {
        static iCubMotorDriver singleton;
        return &singleton; 

    }

    static void SetRobot(yarp::os::ConstString robot)
    {
        m_sRobot=robot;
    }

    ResultCode SetPos(int part,int joint,double position,double speed=0.0,double acc=0.0)
    {
        if (!apDriver[part])
        {
            return DRIVER_FAILED;
        }
        
        if (!apPos[part])
        {
            return IPOS_FAILED;
        }
        
        if (acc!=0.0)
        {
            if (!apPos[part]->setRefAcceleration(joint,acc))
            {
                return IPOS_SETREFACC_FAILED;
            }
        }
        
        if (speed!=0.0)
        {
            if (!apPos[part]->setRefSpeed(joint,speed))
            {
                return IPOS_SETREFSPEED_FAILED;
            }
        }
        
        if (apPos[part]->positionMove(joint,position))
        {
            return IPOS_POSMOVE_OK;
        }

        return IPOS_POSMOVE_FAILED;
    }

    ResultCode WaitPos(int part,int joint,double timeout)
    {
        static const double dTimeStep=0.1;
        bool bFlag;

        if (!apPos[part])
        {
            return IPOS_FAILED;
        }

        for (double dTimeElaps=0.0; dTimeElaps<timeout; dTimeElaps+=dTimeStep)
        {
            bool bSuccess=apPos[part]->checkMotionDone(joint,&bFlag);
            
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

    ResultCode GetPIDError(int part,int joint,double &err)
    {
        if (apPid[part]->getError(joint,&err))
        {
            return IPID_GETERROR_OK;
        }

        return IPID_GETERROR_FAILED;
    }

    ResultCode GetEncPos(int part,int joint,double &pos)
    {
        if (!apEnc[part])
        {
            return IENC_FAILED;
        }
        
        if (apEnc[part]->getEncoder(joint,&pos))
        {
            return IENC_GETPOS_OK;
        }

        return IENC_GETPOS_FAILED;
    }

    ResultCode GetEncSpeed(int part,int joint,double &spd)
    {
        if (!apEnc[part])
        {
            return IENC_FAILED;
        }

        if (apEnc[part]->getEncoderSpeed(joint,&spd))
        {
            return IENC_GETSPEED_OK;
        }

        return IENC_GETSPEED_FAILED;
    }

    ResultCode GetEncAcc(int part,int joint,double &acc)
    {
        if (!apEnc[part])
        {
            return IENC_FAILED;
        }

        if (apEnc[part]->getEncoderAcceleration(joint,&acc))
        {
            return IENC_GETACC_OK;
        }

        return IENC_GETACC_FAILED;
    }

    int GetNumOfJoints(int part)
    {
        return iCubPartNumJoints[part];
    }

    static const char *iCubPartName[NUM_ICUB_PARTS];

protected:

    int iCubPartNumJoints[NUM_ICUB_PARTS];
    
    iCubMotorDriver()
    {
        for (int part=TORSO; part<NUM_ICUB_PARTS; ++part)
        {
            iCubPartNumJoints[part]=0;

            apDriver[part]=OpenDriver(iCubPartName[part]);

            if (apDriver[part])
            {
                apDriver[part]->view(apEnc[part]);
                apDriver[part]->view(apPid[part]);
                apDriver[part]->view(apAmp[part]);
                apDriver[part]->view(apPos[part]);
                apDriver[part]->view(apVel[part]);

                if (apEnc[part])
                {
                    apEnc[part]->getAxes(&iCubPartNumJoints[part]);
                }

                for (int j=0; j<iCubPartNumJoints[part]; ++j)
                {
                    if (apPid[part])
                    {
                        apPid[part]->enablePid(j);
                    }

                    if (apAmp[part])
                    {
                        apAmp[part]->enableAmp(j);
                    }
                }
            }
        }
    }

    ~iCubMotorDriver()
    { 
        Close(); 
    }

    void Close()
    {
        for (int part=TORSO; part<NUM_ICUB_PARTS; ++part)
        {
            for (int j=0; j<iCubPartNumJoints[part]; ++j)
            {
                apPid[part]->disablePid(j);
                apAmp[part]->disableAmp(j);
            }
        }
    }

    yarp::dev::PolyDriver* OpenDriver(yarp::os::impl::String part)
    {
        yarp::os::impl::String robot("/icubSim");

        yarp::os::Property options;
        options.put("robot",m_sRobot.c_str());
        options.put("device","remote_controlboard");
        options.put("local",(yarp::os::impl::String("/iCubTest/")+part).c_str());
        options.put("remote",(m_sRobot+"/"+part).c_str());
   
        yarp::dev::PolyDriver *pDriver=new yarp::dev::PolyDriver(options);
   
        if (!pDriver->isValid()) 
        {
            pDriver->close();
            delete pDriver;
            pDriver=0; 
        }
   
        return pDriver;
    }

    iCubMotorDriver(const iCubMotorDriver&);
    iCubMotorDriver& operator=(const iCubMotorDriver&);

    yarp::dev::PolyDriver *apDriver[NUM_ICUB_PARTS];

    yarp::dev::IEncoders         *apEnc[NUM_ICUB_PARTS];
    yarp::dev::IPidControl       *apPid[NUM_ICUB_PARTS];
    yarp::dev::IAmplifierControl *apAmp[NUM_ICUB_PARTS];
    yarp::dev::IPositionControl  *apPos[NUM_ICUB_PARTS];
    yarp::dev::IVelocityControl  *apVel[NUM_ICUB_PARTS];

    static yarp::os::impl::String m_sRobot;
};

#endif
