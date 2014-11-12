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
#include <iCub/DebugInterfaces.h>

/**
* iCub parts class.
*/
static const int NUM_ICUB_PARTS = 6;
class iCubPart
{
private:
    std::string part_names[NUM_ICUB_PARTS];
    int num_part;
    void init()
    {
        int cnt=0;
        part_names[cnt++]="torso";
        part_names[cnt++]="head";
        part_names[cnt++]="left_arm";
        part_names[cnt++]="right_arm";
        part_names[cnt++]="left_leg";
        part_names[cnt++]="right_leg";
    }

public:
    iCubPart ()
    {
        init();
        num_part = 0; //by default the part is torso
    }

    iCubPart (const std::string &part)
    {
        init();
        for (int i=0; i<6; i++)
            if (part==part_names[i])
                num_part=i;
    }

    iCubPart (int p)
    {
        init();
        num_part = p;
    }
    operator int ()
    {
        return num_part;
    }
    operator std::string ()
    {
        return part_names[num_part];
    }
};

//enum iCubPart {TORSO,HEAD,LEFT_ARM,RIGHT_ARM,LEFT_LEG,RIGHT_LEG,NUM_ICUB_PARTS};

/**
* This class (implemented as a singleton) provide easy access to iCub devices.
*/
class iCubDriver
{
public:
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
        IPOS_POSMOVE_NOT_REACHED    = -61,
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
        IENC_GETACC_FAILED          =-18,

        IOPL_OPLSTART_OK             =+19,
        IOPL_OPLSTART_FAILED         =-20,
        IOPL_OPLSTOP_OK              =+21,
        IOPL_OPLSTOP_FAILED          =-22,

        ILIM_GETLIM_OK               =+23,
        ILIM_GETLIM_FAILED           =-24,

        IPID_GETPOSPID_OK            =+25,
        IPID_GETPOSPID_FAILED        =-26,

        IDBG_FAILED                  =-27,
        IDBG_GETROTPOS_OK            = 28,
        IDBG_GETROTPOS_FAILED        = -29
    };

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
    * Set joint position.
    * @param part the robot part (head, torso, ...).
    * @param joint joint number in the kinematic structure.
    * @param position joint angular position [deg].
    * @param speed reference angular speed [deg/s].
    * @param acc reference angular acceleration [deg/s^2].
    * @return error code.
    */
    ResultCode  setPosAndWait(int part,int joint,double position,double speed=0.0 ,double acc=0.0, double tolerance=0.5, double timeout=20.0);

    /**
    * Set a joint in openloop control and move it with the desired pwm.
    * @param part the robot part (head, torso, ...).
    * @param joint joint number in the kinematic structure.
    * @param pwm the pwm command.
    * @return error code.
    */
    ResultCode startOpenloopCmd(int part,int joint,double pwm);

    /**
    * Set a joint in position mode after an openloop command.
    * @param part the robot part (head, torso, ...).
    * @param joint joint number in the kinematic structure.
    * @return error code.
    */
    ResultCode stopOpenloopCmd(int part,int joint);

    /**
    * Returns the angular limits for the specified joint.
    * @param part the robot part (head, torso, ...).
    * @param joint joint number in the kinematic structure.
    * @param min the returned lower limit.
    * @param max the returned upper limit.
    * @return error code.
    */
    ResultCode getJointLimits(int part,int joint, double& min, double& max);

    /**
    * Wait for trajectory generation completion (blocking).
    * @param part the robot part (head, torso, ...).
    * @param joint joint number in the kinematic structure.
    * @param timeout function returns failure after this time period [s].
    * @return error code.
    */
    ResultCode checkMotionDone(int part,int joint,double timeout);
    
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
    * @param pos returns position [deg] measured by encoder.
    * @return error code.
    */
    ResultCode getRotorPos(int part,int joint,double &pos);

    /**
    * Gets the sign of the position PID.
    * @param part the robot part (head, torso, ...).
    * @param joint joint number in the kinematic structure.
    * @param posPidSign returns 1 or -1 depending on the sign of Kp.
    * @return error code.
    */
    ResultCode getPosPidSign(int part,int joint,double &posPidSign);

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
    * Default destructor.
    */
    ~iCubDriver()
    { 
        close(); 
    }

    /**
    *  default constructor.
    */
    iCubDriver();

    //start stop the driver
    void open (std::string robotName);
    void close();

protected:
    /**
    * Open device drivers and enable motors.
    * @param part the iCub part (head, torso, ...).
    * return a pointer to the iCub part driver interface.
    */
    yarp::dev::PolyDriver* openDriver(std::string part);
    yarp::dev::PolyDriver* openDebugDriver(std::string part);

    iCubDriver(const iCubDriver&);
    iCubDriver& operator=(const iCubDriver&);

    /// Array of iCub part drivers.
    yarp::dev::PolyDriver *m_apDriver[NUM_ICUB_PARTS];
    yarp::dev::PolyDriver *m_apDbgDriver[NUM_ICUB_PARTS];
    
    /// Array of iCub part joint numbers.
    int m_aiCubPartNumJoints[NUM_ICUB_PARTS];

    /// Interfaces to the iCub encoder devices.
    yarp::dev::IEncoders          *m_apEnc[NUM_ICUB_PARTS];
    /// Interfaces to the iCub PID controllers.
    yarp::dev::IPidControl        *m_apPid[NUM_ICUB_PARTS];
    /// Interfaces to the iCub amplifier devices.
    yarp::dev::IAmplifierControl  *m_apAmp[NUM_ICUB_PARTS];
    /// Interfaces to the iCub position control devices.
    yarp::dev::IPositionControl   *m_apPos[NUM_ICUB_PARTS];
    /// Interfaces to the iCub velocity control devices.
    yarp::dev::IVelocityControl   *m_apVel[NUM_ICUB_PARTS];
    /// Interfaces to the iCub torque control devices.
    yarp::dev::ITorqueControl     *m_apTrq[NUM_ICUB_PARTS];
    /// Interfaces to the iCub impedance control devices.
    yarp::dev::IImpedanceControl  *m_apImp[NUM_ICUB_PARTS];
    /// Interfaces to the iCub OpenLoop control devices.
    yarp::dev::IOpenLoopControl   *m_apOpl[NUM_ICUB_PARTS];
    /// Interfaces to the iCub OpenLoop control devices.
    yarp::dev::IControlMode       *m_apCtl[NUM_ICUB_PARTS];
    /// Interfaces to the iCub OpenLoop control devices.
    yarp::dev::IControlLimits     *m_apLim[NUM_ICUB_PARTS];
    /// Interfaces to the iCub encoder devices.
    yarp::dev::IDebugInterface    *m_apDbg[NUM_ICUB_PARTS];

    /// Target robot name.
    std::string m_RobotName;
};

#endif
