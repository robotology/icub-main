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

/**
* This class (implemented as a singleton) provide easy access to iCub devices.
*/
class iCubDriver
{
public:
    /**
    * iCub parts enumerator.
    */
    enum iCubPart {TORSO,HEAD,LEFT_ARM,RIGHT_ARM,LEFT_LEG,RIGHT_LEG,NUM_ICUB_PARTS};

    /**
    * iCubDriver functions result codes enumerator.
    */
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

    /**
    * Returns the unique instance of the iCubDriver class.
    * @return a pointer to the singleton.
    */
    static iCubDriver* instance()
    {
        static iCubDriver singleton;
        return &singleton;
    }

    /**
    * Set the name of the target robot.
    * @param robotName the robot yarp name.
    * @return void.
    */
    static void setRobot(yarp::os::ConstString robotName)
    {
        m_RobotName=robotName;
    }

    /**
    * Set joint position.
    * @param part the robot part (head, torso, ...).
    * @param joint joint number in the kinematic structure.
    * @param position joint angular position [deg].
    * @param speed reference angular speed [deg/s].
    * @param acc reference angular acceleration [deg/s^2].
    * @return error code.
    */
    ResultCode setPos(int part,int joint,double position,double speed=0.0,double acc=0.0);

    /**
    * Wait for setPos completion (blocking).
    * @param part the robot part (head, torso, ...).
    * @param joint joint number in the kinematic structure.
    * @param timeout function returns failure after this time period [s].
    * @return error code.
    */
    ResultCode waitPos(int part,int joint,double timeout);
    
    /**
    * Pid error.
    * @param part the robot part (head, torso, ...).
    * @param joint joint number in the kinematic structure.
    * @param err returns the pid error.
    * @return error code.
    */
    ResultCode getPIDError(int part,int joint,double &err);

    /**
    * Encoder position.
    * @param part the robot part (head, torso, ...).
    * @param joint joint number in the kinematic structure.
    * @param pos returns position [deg] measured by encoder.
    * @return error code.
    */
    ResultCode getEncPos(int part,int joint,double &pos);

    /**
    * Encoder position.
    * @param part the robot part (head, torso, ...).
    * @param joint joint number in the kinematic structure.
    * @param spd returns speed [deg/s] measured by encoder.
    * @return error code.
    */
    ResultCode getEncSpeed(int part,int joint,double &spd);
    
    /**
    * Encoder position.
    * @param part the robot part (head, torso, ...).
    * @param joint joint number in the kinematic structure.
    * @param acc returns acceleration [deg/s^2] measured by encoder.
    * @return error code.
    */
    ResultCode getEncAcc(int part,int joint,double &acc);

    /**
    * Encoder position.
    * @param part the robot part (head, torso, ...).
    * @param joint joint number in the kinematic structure.
    * @param pos returns the encoder position.
    * @return error code.
    */
    int getNumOfJoints(int part)
    {
        return m_aiCubPartNumJoints[part];
    }

    /**
    * iCub part names.
    */
    static const char *m_aiCubPartName[NUM_ICUB_PARTS];

protected:
    /**
    * Protected default constructor (singleton implementation).
    */
    iCubDriver();
    
    /**
    * Deafault destructor.
    */
    ~iCubDriver()
    { 
        close(); 
    }

    /**
    * Disable motor amplifiers.
    */
    void close();

    /**
    * Open device drivers and enable motors.
    * @param part the iCub part (head, torso, ...).
    * return a pointer to the iCub part driver interface.
    */
    yarp::dev::PolyDriver* openDriver(std::string part);

    iCubDriver(const iCubDriver&);
    iCubDriver& operator=(const iCubDriver&);

    /// Array of iCub part drivers.
    yarp::dev::PolyDriver *m_apDriver[NUM_ICUB_PARTS];
    
    /// Array of iCub part joint numbers.
    int m_aiCubPartNumJoints[NUM_ICUB_PARTS];

    /// Interfaces to the iCub encoder devices.
    yarp::dev::IEncoders         *m_apEnc[NUM_ICUB_PARTS];
    /// Interfaces to the iCub PID controllers.
    yarp::dev::IPidControl       *m_apPid[NUM_ICUB_PARTS];
    /// Interfaces to the iCub amplifier devices.
    yarp::dev::IAmplifierControl *m_apAmp[NUM_ICUB_PARTS];
    /// Interfaces to the iCub position control devices.
    yarp::dev::IPositionControl  *m_apPos[NUM_ICUB_PARTS];
    /// Interfaces to the iCub velocity control devices.
    yarp::dev::IVelocityControl  *m_apVel[NUM_ICUB_PARTS];

    /// Target robot name.
    static std::string m_RobotName;
};

#endif
