/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Marco Randazzo, Matteo Fumagalli
 * email:  marco.randazzo@iit.it
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

#ifndef GRAVITY_THREAD
#define GRAVITY_THREAD

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace iCub::ctrl;
using namespace iCub::iDyn;

#define MAX_JN 12
#define MAX_FILTER_ORDER 6
enum thread_status_enum {STATUS_OK=0, STATUS_DISCONNECTED}; 
enum{GRAVITY_COMPENSATION_OFF = 0, GRAVITY_COMPENSATION_ON = 1};
enum{EXTERNAL_TRQ_OFF = 0, EXTERNAL_TRQ_ON = 1};
enum{TORQUE_INTERFACE = 0, IMPEDANCE_POSITION = 1, IMPEDANCE_VELOCITY = 2};

class gravityCompensatorThread: public yarp::os::RateThread
{
private:

    std::string           wholeBodyName;
    BufferedPort<Vector> *port_inertial;
    BufferedPort<Vector> *la_additional_offset;
    BufferedPort<Vector> *ra_additional_offset;
    BufferedPort<Vector> *ll_additional_offset;
    BufferedPort<Vector> *rl_additional_offset;
    BufferedPort<Vector> *to_additional_offset;

    BufferedPort<Vector> *left_arm_exec_torques;
    BufferedPort<Vector> *right_arm_exec_torques;
    BufferedPort<Vector> *left_leg_exec_torques;
    BufferedPort<Vector> *right_leg_exec_torques;
    BufferedPort<Vector> *torso_exec_torques;

    BufferedPort<Vector> *left_arm_gravity_torques;
    BufferedPort<Vector> *right_arm_gravity_torques;
    BufferedPort<Vector> *left_leg_gravity_torques;
    BufferedPort<Vector> *right_leg_gravity_torques;
    BufferedPort<Vector> *torso_gravity_torques;

    PolyDriver   *ddLA;
    PolyDriver   *ddRA;
    PolyDriver   *ddH;
    IEncoders    *iencs_arm_left;
    IEncoders    *iencs_arm_right;
    IEncoders    *iencs_head;
    IControlMode      *iCtrlMode_arm_left;
    IControlMode      *iCtrlMode_arm_right;
    IControlMode      *iCtrlMode_torso;
    IInteractionMode  *iIntMode_arm_left;
    IInteractionMode  *iIntMode_arm_right;
    IInteractionMode  *iIntMode_torso;
    IImpedanceControl *iImp_arm_left;
    ITorqueControl    *iTqs_arm_left;
    IImpedanceControl *iImp_arm_right;
    ITorqueControl    *iTqs_arm_right;
    IImpedanceControl *iImp_torso;
    ITorqueControl    *iTqs_torso;
    thread_status_enum thread_status;

    PolyDriver   *ddLL;
    PolyDriver   *ddRL;
    PolyDriver   *ddT;
    IEncoders    *iencs_leg_left;
    IEncoders    *iencs_leg_right;
    IEncoders    *iencs_torso;
    IControlMode      *iCtrlMode_leg_left;
    IControlMode      *iCtrlMode_leg_right;
    IInteractionMode  *iIntMode_leg_left;
    IInteractionMode  *iIntMode_leg_right;
    IImpedanceControl *iImp_leg_left;
    ITorqueControl    *iTqs_leg_left;
    IImpedanceControl *iImp_leg_right;
    ITorqueControl    *iTqs_leg_right;

    Vector encoders_arm_left;
    Vector encoders_arm_right;
    Vector encoders_head;

    Vector encoders_leg_left;
    Vector encoders_leg_right;
    Vector encoders_torso;

    Vector *inertial;

    AWLinEstimator  *linEstUp;
    AWQuadEstimator *quadEstUp;
    AWLinEstimator  *linEstLow;
    AWQuadEstimator *quadEstLow;

    int left_arm_ctrlJnt;
    int right_arm_ctrlJnt;
    int left_leg_ctrlJnt;
    int right_leg_ctrlJnt;
    int torso_ctrlJnt;
    int allJnt;

    iCubWholeBody* icub;

    Vector q_head, dq_head, d2q_head;
    Vector q_larm, dq_larm, d2q_larm;
    Vector q_rarm, dq_rarm, d2q_rarm;
    Vector all_q_up, all_dq_up, all_d2q_up;

    Vector q_torso, dq_torso, d2q_torso;
    Vector q_lleg, dq_lleg, d2q_lleg;
    Vector q_rleg, dq_rleg, d2q_rleg;
    Vector all_q_low, all_dq_low, all_d2q_low;

    Vector w0,dw0,d2p0,Fend,Muend;
    Vector F_LArm, F_RArm, F_iDyn_LArm, F_iDyn_RArm, Offset_LArm, Offset_RArm;
    Vector F_LLeg, F_RLeg, F_iDyn_LLeg, F_iDyn_RLeg, Offset_LLeg, Offset_RLeg;
    Matrix F_ext_up, F_ext_low;
    Vector inertial_measurements;

    std::string side;
    std::string part;

    Vector exec_torques_LA,exec_torques_RA,exec_torques_LL,exec_torques_RL,exec_torques_TO;
    Vector gravity_torques_LA,gravity_torques_RA,gravity_torques_LL,gravity_torques_RL,gravity_torques_TO;
    Vector externalcmd_torques_LA,externalcmd_torques_RA,externalcmd_torques_LL,externalcmd_torques_RL, externalcmd_torques_TO;
    Vector ampli_LA, ampli_RA, ampli_LL, ampli_RL, ampli_TO;
    bool isCalibrated;
    bool inertial_enabled;
    
    Vector evalVelUp(const Vector &x);
    Vector evalVelLow(const Vector &x);
    Vector evalAccUp(const Vector &x);
    Vector evalAccLow(const Vector &x);

    void init_upper();
    void init_lower();
    void setLowerMeasure();
    void setUpperMeasure();

public:
    
    int gravity_mode;
    int external_mode;

    gravityCompensatorThread(std::string _wholeBodyName, int _rate, PolyDriver *_ddLA, PolyDriver *_ddRA, PolyDriver *_ddH, PolyDriver *_ddLL, PolyDriver *_ddRL, PolyDriver *_ddT, version_tag icub_type, bool _inertial_enabled);

    void setZeroJntAngVelAcc();
    bool readAndUpdate(bool waitMeasure=false);
    bool getLowerEncodersSpeedAndAcceleration();
    bool getUpperEncodersSpeedAndAcceleration();
    bool threadInit();
    void feedFwdGravityControl(int part_ctrlJnt, std::string s_part, IControlMode *iCtrlMode, ITorqueControl *iTqs, IImpedanceControl *iImp, IInteractionMode *iIntMode, const Vector &command, bool releasing=false);
    void run();
    void threadRelease();
    void closePort(Contactable *_port);

    inline thread_status_enum getThreadStatus() 
    {
        return thread_status;
    }

};

#endif
