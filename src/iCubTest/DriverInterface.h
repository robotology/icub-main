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

#ifndef __ICUB_DRIVER_01122009__
#define __ICUB_DRIVER_01122009__

#include <vector>
#include <string>

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

    ResultCode setPos(int part,int joint,double position,double speed=0.0,double acc=0.0);
    ResultCode waitPos(int part,int joint,double timeout);
    ResultCode getPIDError(int part,int joint,double &err);
    ResultCode getEncPos(int part,int joint,double &pos);
    ResultCode getEncSpeed(int part,int joint,double &spd);
    ResultCode getEncAcc(int part,int joint,double &acc);

    int getNumOfJoints(int part)
    {
        return m_aiCubPartNumJoints[part];
    }

    static const char *m_aiCubPartName[NUM_ICUB_PARTS];

protected:
    iCubDriver();
    
    ~iCubDriver()
    { 
        close(); 
    }

    void close();

    yarp::dev::PolyDriver* openDriver(std::string part);

    iCubDriver(const iCubDriver&);
    iCubDriver& operator=(const iCubDriver&);

    yarp::dev::PolyDriver *m_apDriver[NUM_ICUB_PARTS];
    
    int m_aiCubPartNumJoints[NUM_ICUB_PARTS];

    yarp::dev::IEncoders         *m_apEnc[NUM_ICUB_PARTS];
    yarp::dev::IPidControl       *m_apPid[NUM_ICUB_PARTS];
    yarp::dev::IAmplifierControl *m_apAmp[NUM_ICUB_PARTS];
    yarp::dev::IPositionControl  *m_apPos[NUM_ICUB_PARTS];
    yarp::dev::IVelocityControl  *m_apVel[NUM_ICUB_PARTS];

    static std::string m_RobotName;
};

#endif
