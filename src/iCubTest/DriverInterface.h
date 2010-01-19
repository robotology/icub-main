// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2009 Robotcub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Author Alessandro Scalzo alessandro@liralab.it
 */

#ifndef __ICUB_DRIVER_01122009__
#define __ICUB_DRIVER_01122009__

#include <vector>
#include <string>

#include <yarp/os/Time.h>
#include <yarp/os/Searchable.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>

class iCubDriver
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

    static iCubDriver* instance()
    {
        static iCubDriver singleton;
        return &singleton;
    }

    static void setRobot(yarp::os::ConstString robotName)
    {
        m_RobotName=robotName;
    }

    ResultCode setPos(int part,int joint,double position,double speed=0.0,double acc=0.0)
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
            printf("************ ACC != 0.0 ***************\n");
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

    ResultCode waitPos(int part,int joint,double timeout)
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

    ResultCode getPIDError(int part,int joint,double &err)
    {
        if (m_apPid[part]->getError(joint,&err))
        {
            return IPID_GETERROR_OK;
        }

        return IPID_GETERROR_FAILED;
    }

    ResultCode getEncPos(int part,int joint,double &pos)
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

    ResultCode getEncSpeed(int part,int joint,double &spd)
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

    ResultCode getEncAcc(int part,int joint,double &acc)
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

    int getNumOfJoints(int part)
    {
        return m_aiCubPartNumJoints[part];
    }

    static const char *m_aiCubPartName[NUM_ICUB_PARTS];

protected:
    int m_aiCubPartNumJoints[NUM_ICUB_PARTS];
    
    iCubDriver()
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

    ~iCubDriver()
    { 
        close(); 
    }

    void close()
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

    yarp::dev::PolyDriver* openDriver(std::string part)
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

    iCubDriver(const iCubDriver&);
    iCubDriver& operator=(const iCubDriver&);

    yarp::dev::PolyDriver *m_apDriver[NUM_ICUB_PARTS];

    yarp::dev::IEncoders         *m_apEnc[NUM_ICUB_PARTS];
    yarp::dev::IPidControl       *m_apPid[NUM_ICUB_PARTS];
    yarp::dev::IAmplifierControl *m_apAmp[NUM_ICUB_PARTS];
    yarp::dev::IPositionControl  *m_apPos[NUM_ICUB_PARTS];
    yarp::dev::IVelocityControl  *m_apVel[NUM_ICUB_PARTS];

    static std::string m_RobotName;
};

#endif
