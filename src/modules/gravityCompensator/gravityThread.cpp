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

#include "gravityThread.h"

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <iCub/ctrl/math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>
#include <iCub/skinDynLib/common.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::ctrl;
using namespace iCub::iDyn;
using namespace iCub::skinDynLib;
using namespace std;

Vector gravityCompensatorThread::evalVelUp(const Vector &x)
{
    AWPolyElement el;
    el.data=x;
    el.time=Time::now();

    return linEstUp->estimate(el);
}

Vector gravityCompensatorThread::evalVelLow(const Vector &x)
{
    AWPolyElement el;
    el.data=x;
    el.time=Time::now();

    return linEstLow->estimate(el);
}

Vector gravityCompensatorThread::evalAccUp(const Vector &x)
{
    AWPolyElement el;
    el.data=x;
    el.time=Time::now();

    return quadEstUp->estimate(el);
}

Vector gravityCompensatorThread::evalAccLow(const Vector &x)
{
    AWPolyElement el;
    el.data=x;
    el.time=Time::now();

    return quadEstLow->estimate(el);
}

void gravityCompensatorThread::init_upper()
{
    //---------------------PARTS-------------------------//
    // Left_arm variables
    allJnt = 0;
    int jnt=7;
    encoders_arm_left.resize(jnt,0.0);
    F_LArm.resize(6,0.0);
    F_iDyn_LArm.resize(6,0.0);
    Offset_LArm.resize(6,0.0);
    q_larm.resize(7,0.0);
    dq_larm.resize(7,0.0);
    d2q_larm.resize(7,0.0);
    allJnt+=jnt;

    // Right_arm variables
    jnt = 7;
    encoders_arm_right.resize(jnt,0.0);
    F_RArm.resize(6,0.0);
    F_iDyn_RArm.resize(6,0.0);
    Offset_RArm.resize(6,0.0);
    q_rarm.resize(7,0.0);
    dq_rarm.resize(7,0.0);
    d2q_rarm.resize(7,0.0);
    allJnt+=jnt;

    // Head variables
    jnt = 3;
    encoders_head.resize(jnt,0.0);
    q_head.resize(3,0.0);
    dq_head.resize(3,0.0);
    d2q_head.resize(3,0.0);
    allJnt+=jnt;

    all_q_up.resize(allJnt,0.0);
    all_dq_up.resize(allJnt,0.0);
    all_d2q_up.resize(allJnt,0.0); 
    gravity_torques_LA.resize(7); gravity_torques_LA.zero();
    gravity_torques_RA.resize(7); gravity_torques_RA.zero();
    exec_torques_LA.resize(7); exec_torques_LA.zero();
    exec_torques_RA.resize(7); exec_torques_RA.zero();
    ampli_LA.resize(7);ampli_LA=1.0;
    ampli_RA.resize(7);ampli_RA=1.0;
    externalcmd_torques_LA.resize(7);externalcmd_torques_LA.zero();
    externalcmd_torques_RA.resize(7);externalcmd_torques_RA.zero();
}

void gravityCompensatorThread::init_lower()
{
    //---------------------PARTS-------------------------//
    // Left_leg variables
    allJnt = 0;
    int jnt = 6; 
    encoders_leg_left.resize(jnt,0.0);
    q_lleg.resize(6,0.0);
    dq_lleg.resize(6,0.0);
    d2q_lleg.resize(6,0.0);
    allJnt+=jnt;

    // Right_leg variables
    jnt = 6;
    encoders_leg_right.resize(jnt,0.0);
    q_rleg.resize(6,0.0);
    dq_rleg.resize(6,0.0);
    d2q_rleg.resize(6,0.0);
    allJnt+=jnt;

    // Torso variables
    jnt = 3;
    encoders_torso.resize(jnt,0.0);
    q_torso.resize(3,0.0);
    dq_torso.resize(3,0.0);
    d2q_torso.resize(3,0.0);
    allJnt+=jnt;

    all_q_low.resize(allJnt,0.0);
    all_dq_low.resize(allJnt,0.0);
    all_d2q_low.resize(allJnt,0.0);
    gravity_torques_TO.resize(3); gravity_torques_TO.zero();
    gravity_torques_LL.resize(6); gravity_torques_LL.zero();
    gravity_torques_RL.resize(6); gravity_torques_RL.zero();
    exec_torques_TO.resize(3); exec_torques_TO.zero();
    exec_torques_LL.resize(6); exec_torques_LL.zero();
    exec_torques_RL.resize(6); exec_torques_RL.zero();
    ampli_TO.resize(3);ampli_TO=1.0;
    ampli_LL.resize(6);ampli_LL=1.0;
    ampli_RL.resize(6);ampli_RL=1.0;
    externalcmd_torques_TO.resize(3);externalcmd_torques_TO.zero();
    externalcmd_torques_LL.resize(6);externalcmd_torques_LL.zero();
    externalcmd_torques_RL.resize(6);externalcmd_torques_RL.zero();

}

void  gravityCompensatorThread::setLowerMeasure()
{
    icub->lowerTorso->setAng("torso",CTRL_DEG2RAD * q_torso);
    icub->lowerTorso->setDAng("torso",CTRL_DEG2RAD * dq_torso);
    icub->lowerTorso->setD2Ang("torso",CTRL_DEG2RAD * d2q_torso);

    icub->lowerTorso->setAng("left_leg",CTRL_DEG2RAD * q_lleg);
    icub->lowerTorso->setDAng("left_leg",CTRL_DEG2RAD * dq_lleg);
    icub->lowerTorso->setD2Ang("left_leg",CTRL_DEG2RAD * d2q_lleg);

    icub->lowerTorso->setAng("right_leg",CTRL_DEG2RAD * q_rleg);
    icub->lowerTorso->setDAng("right_leg",CTRL_DEG2RAD * dq_rleg);
    icub->lowerTorso->setD2Ang("right_leg",CTRL_DEG2RAD * d2q_rleg);
}

void  gravityCompensatorThread::setUpperMeasure()
{
    icub->upperTorso->setAng("head",CTRL_DEG2RAD * q_head);
    icub->upperTorso->setAng("left_arm",CTRL_DEG2RAD * q_larm);
    icub->upperTorso->setAng("right_arm",CTRL_DEG2RAD * q_rarm);
    icub->upperTorso->setDAng("head",CTRL_DEG2RAD * dq_head);
    icub->upperTorso->setDAng("left_arm",CTRL_DEG2RAD * dq_larm);
    icub->upperTorso->setDAng("right_arm",CTRL_DEG2RAD * dq_rarm);
    icub->upperTorso->setD2Ang("head",CTRL_DEG2RAD * d2q_head);
    icub->upperTorso->setD2Ang("left_arm",CTRL_DEG2RAD * d2q_larm);
    icub->upperTorso->setD2Ang("right_arm",CTRL_DEG2RAD * d2q_rarm);
    icub->upperTorso->setInertialMeasure(w0,dw0,d2p0);
}

gravityCompensatorThread::gravityCompensatorThread(string _wholeBodyName, int _rate, PolyDriver *_ddLA, PolyDriver *_ddRA, PolyDriver *_ddH, PolyDriver *_ddLL, PolyDriver *_ddRL, PolyDriver *_ddT, version_tag icub_type, bool _inertial_enabled) : RateThread(_rate), ddLA(_ddLA), ddRA(_ddRA), ddLL(_ddLL), ddRL(_ddRL), ddH(_ddH), ddT(_ddT)
{   
    gravity_mode = GRAVITY_COMPENSATION_ON;
    external_mode = EXTERNAL_TRQ_ON;
    wholeBodyName = _wholeBodyName;

    //--------------INTERFACE INITIALIZATION-------------//
    iencs_arm_left        = 0;
    iencs_arm_right       = 0;
    iencs_head            = 0;
    iCtrlMode_arm_left    = 0;
    iCtrlMode_arm_right   = 0;
    iCtrlMode_torso       = 0;
    iIntMode_arm_left     = 0;
    iIntMode_arm_right    = 0;
    iIntMode_torso        = 0;
    iImp_torso            = 0;
    iTqs_torso            = 0;
    iImp_arm_left         = 0;
    iTqs_arm_left         = 0;
    iImp_arm_right        = 0;
    iTqs_arm_right        = 0;
    iencs_leg_left        = 0;
    iencs_leg_right       = 0;
    iencs_torso           = 0;
    iCtrlMode_leg_left    = 0;
    iCtrlMode_leg_right   = 0;
    iIntMode_leg_left     = 0;
    iIntMode_leg_right    = 0;
    iImp_leg_left         = 0;
    iTqs_leg_left         = 0;
    iImp_leg_right        = 0;
    iTqs_leg_right        = 0;
    isCalibrated = false;
    inertial_enabled=_inertial_enabled;
    icub = new iCubWholeBody (icub_type,DYNAMIC, VERBOSE);
    thread_status=STATUS_DISCONNECTED;
    port_inertial        = 0;
    la_additional_offset = 0;
    ra_additional_offset = 0;
    ll_additional_offset = 0;
    rl_additional_offset = 0;
    to_additional_offset = 0;
    left_arm_exec_torques     = 0;
    right_arm_exec_torques    = 0;
    left_leg_exec_torques     = 0;
    right_leg_exec_torques    = 0;
    torso_exec_torques        = 0;
    left_arm_gravity_torques  = 0;
    right_arm_gravity_torques = 0;
    left_leg_gravity_torques  = 0;
    right_leg_gravity_torques = 0;
    torso_gravity_torques     = 0;

}
    
void gravityCompensatorThread::setZeroJntAngVelAcc()
{
    dq_head = 0.0;
    d2q_head = 0.0;
    dq_larm = 0.0;
    d2q_larm = 0.0;
    dq_rarm = 0.0;
    d2q_rarm = 0.0;
    
    dq_rleg=0.0;
    d2q_rleg=0.0;
    dq_lleg=0.0;
    d2q_lleg=0.0;
    dq_torso=0.0;
    d2q_torso=0.0;
}

bool gravityCompensatorThread::readAndUpdate(bool waitMeasure)
{
    int sz = 0;
    bool b = true;

    /*
    //offset currently not implemented
    Vector *offset_input = additional_offset->read(false);
    if(offset_input!=0)
    {
        sz = offset_input->length();
        if(sz!=ctrlJnt)
        {
            yWarning"warning...controlled joint < of offsets size!!!");
        }
        else
        {
            Vector o=*offset_input;
            torque_offset = 0.0;
            for(int i=0;i<ctrlJnt;i++)
                torque_offset[i] = o[i];
        }
    }
    */

    if (inertial_enabled)
    {
        inertial = port_inertial->read(waitMeasure);
        if(inertial!=0)
        {
            sz = inertial->length();
            inertial_measurements.resize(sz) ;
            inertial_measurements= *inertial;
            d2p0[0] = inertial_measurements[0];
            d2p0[1] = inertial_measurements[1];
            d2p0[2] = inertial_measurements[2];
            w0 [0] = 0;
            w0 [1] = 0;
            w0 [2] = 0;
            dw0 [0] = 0;
            dw0 [1] = 0;
            dw0 [2] = 0;
        }
        else
        {
            b = false;
        }
    }
    else
    {
        d2p0[0] = 0;
        d2p0[1] = 0;
        d2p0[2] = 9.81;
        w0 [0] = 0;
        w0 [1] = 0;
        w0 [2] = 0;
        dw0 [0] = 0;
        dw0 [1] = 0;
        dw0 [2] = 0;
    }
    
    b &= getUpperEncodersSpeedAndAcceleration();
    setUpperMeasure();
    b &= getLowerEncodersSpeedAndAcceleration();
    setLowerMeasure();

    return b;
}

bool gravityCompensatorThread::getLowerEncodersSpeedAndAcceleration()
{
    bool b = true;
    if (iencs_leg_left)  
    {b &= iencs_leg_left->getEncoders(encoders_leg_left.data());}
    else 
    {encoders_leg_left.zero();}

    if (iencs_leg_right) 
    {b &= iencs_leg_right->getEncoders(encoders_leg_right.data());}
    else 
    {encoders_leg_right.zero();}

    if (iencs_torso) 
    {b &= iencs_torso->getEncoders(encoders_torso.data());}
    else 
    {encoders_torso.zero();}

    for (size_t i=0;i<q_torso.length();i++)
    {
        q_torso(i) = encoders_torso(2-i);
        all_q_low(i) = q_torso(i);
    }
    for (size_t i=0;i<q_lleg.length();i++)
    {
        q_lleg(i) = encoders_leg_left(i);
        all_q_low(q_torso.length()+i) = q_lleg(i);
    }
    for (size_t i=0;i<q_rleg.length();i++)
    {
        q_rleg(i) = encoders_leg_right(i);
        all_q_low(q_torso.length()+q_lleg.length()+i) = q_rleg(i);
    }

    setZeroJntAngVelAcc();

    /*
    all_dq_low = evalVelLow(all_q_low);
    all_d2q_low = evalAccLow(all_q_low);
    
    for (int i=0;i<q_torso.length();i++)
    {
        dq_torso(i) = all_dq_low(i);
        d2q_torso(i) = all_d2q_low(i);
    }
    for (int i=0;i<q_lleg.length();i++)
    {
        dq_lleg(i) = all_dq_low(i+q_torso.length());
        d2q_lleg(i) = all_d2q_low(i+q_torso.length());
    }
    for (int i=0;i<q_rleg.length();i++)
    {
        dq_rleg(i) = all_dq_low(i+q_torso.length()+q_lleg.length());
        d2q_rleg(i) = all_d2q_low(i+q_torso.length()+q_lleg.length());
    }*/

    return b;
}


bool gravityCompensatorThread::getUpperEncodersSpeedAndAcceleration()
{
    bool b = true;
    if (iencs_arm_left) 
    {b &= iencs_arm_left->getEncoders(encoders_arm_left.data());}
    else 
    {encoders_arm_left.zero();}

    if (iencs_arm_right) 
    {b &= iencs_arm_right->getEncoders(encoders_arm_right.data());}
    else 
    {encoders_arm_right.zero();}

    if (iencs_head) 
    {b &= iencs_head->getEncoders(encoders_head.data());}
    else 
    {encoders_head.zero();}

    for (size_t i=0;i<q_head.length();i++)
    {
        q_head(i) = encoders_head(i);
        all_q_up(i) = q_head(i);
    }
    for (size_t i=0;i<q_larm.length();i++)
    {
        q_larm(i) = encoders_arm_left(i);
        all_q_up(q_head.length()+i) = q_larm(i);
    }
    for (size_t i=0;i<q_rarm.length();i++)
    {
        q_rarm(i) = encoders_arm_right(i);
        all_q_up(q_head.length()+q_larm.length()+i) = q_rarm(i);
    }

    setZeroJntAngVelAcc();
    
    /*
    all_dq_up = evalVelUp(all_q_up);
    all_d2q_up = evalAccUp(all_q_up);
    
    for (int i=0;i<q_head.length();i++)
    {
        dq_head(i) = all_dq_up(i);
        d2q_head(i) = all_d2q_up(i);
    }
    for (int i=0;i<q_larm.length();i++)
    {
        dq_larm(i) = all_dq_up(i+q_head.length());
        d2q_larm(i) = all_d2q_up(i+q_head.length());
    }
    for (int i=0;i<q_rarm.length();i++)
    {
        dq_rarm(i) = all_dq_up(i+q_head.length()+q_larm.length());
        d2q_rarm(i) = all_d2q_up(i+q_head.length()+q_larm.length());
    }*/

    return b;
}

bool gravityCompensatorThread::threadInit()
{
    //---------------------PORTS-------------------------//
    port_inertial=new BufferedPort<Vector>;
    if (!port_inertial->open("/gravityCompensator/inertial:i"))  {yError("Another gravityCompensator module is running? quitting"); return false;}

    la_additional_offset=new BufferedPort<Vector>;
    if (!la_additional_offset->open("/gravityCompensator/left_arm/ctrl_offset:i")) {yError("Another gravityCompensator module is running? quitting"); return false;}
    ra_additional_offset=new BufferedPort<Vector>;
    if (!ra_additional_offset->open("/gravityCompensator/right_arm/ctrl_offset:i")) {yError("Another gravityCompensator module is running? quitting"); return false;}
    ll_additional_offset=new BufferedPort<Vector>;
    if (!ll_additional_offset->open("/gravityCompensator/left_leg/ctrl_offset:i")) {yError("Another gravityCompensator module is running? quitting"); return false;}
    rl_additional_offset=new BufferedPort<Vector>;
    if (!rl_additional_offset->open("/gravityCompensator/right_leg/ctrl_offset:i")) {yError("Another gravityCompensator module is running? quitting"); return false;}
    to_additional_offset=new BufferedPort<Vector>;
    if (!to_additional_offset->open("/gravityCompensator/torso/ctrl_offset:i")) {yError("Another gravityCompensator module is running? quitting"); return false;}

    left_arm_exec_torques = new BufferedPort<Vector>;
    if (!left_arm_exec_torques->open("/gravityCompensator/left_arm/exec_torques:o")) {yError("Another gravityCompensator module is running? quitting"); return false;}
    right_arm_exec_torques = new BufferedPort<Vector>;
    if (!right_arm_exec_torques->open("/gravityCompensator/right_arm/exec_torques:o")) {yError("Another gravityCompensator module is running? quitting"); return false;}
    left_leg_exec_torques = new BufferedPort<Vector>;
    if (!left_leg_exec_torques->open("/gravityCompensator/left_leg/exec_torques:o")) {yError("Another gravityCompensator module is running? quitting"); return false;}
    right_leg_exec_torques = new BufferedPort<Vector>;
    if (!right_leg_exec_torques->open("/gravityCompensator/right_leg/exec_torques:o")) {yError("Another gravityCompensator module is running? quitting"); return false;}
    torso_exec_torques = new BufferedPort<Vector>;
    if (!torso_exec_torques->open("/gravityCompensator/torso/exec_torques:o")) {yError("Another gravityCompensator module is running? quitting"); return false;}

    left_arm_gravity_torques = new BufferedPort<Vector>;
    if (!left_arm_gravity_torques->open("/gravityCompensator/left_arm/gravity_torques:o")) {yError("Another gravityCompensator module is running? quitting"); return false;}
    right_arm_gravity_torques = new BufferedPort<Vector>;
    if (!right_arm_gravity_torques->open("/gravityCompensator/right_arm/gravity_torques:o")) {yError("Another gravityCompensator module is running? quitting"); return false;}
    left_leg_gravity_torques = new BufferedPort<Vector>;
    if (!left_leg_gravity_torques->open("/gravityCompensator/left_leg/gravity_torques:o")) {yError("Another gravityCompensator module is running? quitting"); return false;}
    right_leg_gravity_torques = new BufferedPort<Vector>;
    if (!right_leg_gravity_torques->open("/gravityCompensator/right_leg/gravity_torques:o")) {yError("Another gravityCompensator module is running? quitting"); return false;}
    torso_gravity_torques = new BufferedPort<Vector>;
    if (!torso_gravity_torques->open("/gravityCompensator/torso/gravity_torques:o")) {yError("Another gravityCompensator module is running? quitting"); return false;}

    //---------------------DEVICES--------------------------//
    if (ddLA) ddLA->view(iencs_arm_left);
    if (ddRA) ddRA->view(iencs_arm_right);
    if (ddH)  ddH->view(iencs_head);
    if (ddLL) ddLL->view(iencs_leg_left);
    if (ddRL) ddRL->view(iencs_leg_right);
    if (ddT)  ddT->view(iencs_torso);
    
    if (ddLA) ddLA->view(iCtrlMode_arm_left);
    if (ddRA) ddRA->view(iCtrlMode_arm_right);
    if (ddLA) ddLA->view(iIntMode_arm_left);
    if (ddRA) ddRA->view(iIntMode_arm_right);
    if (ddLA) ddLA->view(iImp_arm_left);
    if (ddLA) ddLA->view(iTqs_arm_left);
    if (ddRA) ddRA->view(iImp_arm_right);
    if (ddRA) ddRA->view(iTqs_arm_right);

    if (ddT)  ddT->view(iCtrlMode_torso);
    if (ddT)  ddT->view(iIntMode_torso);
    if (ddT)  ddT->view(iImp_torso);
    if (ddT)  ddT->view(iTqs_torso);

    if (ddLL) ddLL->view(iCtrlMode_leg_left);
    if (ddRL) ddRL->view(iCtrlMode_leg_right);
    if (ddLL) ddLL->view(iIntMode_leg_left);
    if (ddRL) ddRL->view(iIntMode_leg_right);
    if (ddLL) ddLL->view(iImp_leg_left);
    if (ddLL) ddLL->view(iTqs_leg_left);
    if (ddRL) ddRL->view(iImp_leg_right);
    if (ddRL) ddRL->view(iTqs_leg_right);    
    
    linEstUp =new AWLinEstimator(16,1.0);
    quadEstUp=new AWQuadEstimator(25,1.0);
    linEstLow =new AWLinEstimator(16,1.0);
    quadEstLow=new AWQuadEstimator(25,1.0);
    
    //-----------parts INIT VARIABLES----------------//
    init_upper();
    init_lower();

    //-----------CARTESIAN INIT VARIABLES----------------//
    left_arm_ctrlJnt  = 5;
    right_arm_ctrlJnt = 5;
    left_leg_ctrlJnt  = 4;
    right_leg_ctrlJnt = 4;
    torso_ctrlJnt     = 3;
    w0.resize(3,0.0);
    dw0.resize(3,0.0);
    d2p0.resize(3,0.0);
    Fend.resize(3,0.0);
    Muend.resize(3,0.0);
    F_ext_up.resize(6,3);
    F_ext_up = 0.0;
    F_ext_low.resize(6,3);
    F_ext_low = 0.0;
    inertial_measurements.resize(12);
    inertial_measurements.zero();

    int ctrl_mode = 0;
    
    switch(gravity_mode)
    {
        case GRAVITY_COMPENSATION_OFF:        yInfo("GRAVITY_COMPENSATION_OFF     \n");    break;
        case GRAVITY_COMPENSATION_ON:         yInfo("GRAVITY_COMPENSATION_ON      \n");    break;
        default:
        case VOCAB_CM_UNKNOWN:                yError("UNKNOWN  \n");    break;
    }

    thread_status = STATUS_OK;

    return true;
}


void gravityCompensatorThread::feedFwdGravityControl(int part_ctrlJnt, string s_part, IControlMode *iCtrlMode, ITorqueControl *iTqs, IImpedanceControl *iImp, IInteractionMode *iIntMode, const Vector &command, bool releasing)
{
    //check if interfaces are still up (icubinterface running)  
    if (iCtrlMode == 0) 
        {yError("ControlMode interface already closed, unable to reset compensation offset.\n");    return;}
    if (iTqs == 0)
        {yError("TorqueControl interface already closed, unable to reset compensation offset.\n");  return;}
    if (iIntMode == 0)
        {yError("InteractionMode interface already closed, unable to reset compensation offset.\n");  return;}
    if (iImp == 0)
        {yError("Impedance interface already closed, unable to reset compensation offset.\n");      return;}

    //set to zero all the offsets if the module is closing
    if(releasing)
    {
        for(int i=0;i<part_ctrlJnt;i++)
        {
            iImp->setImpedanceOffset(i,0.0);
            iTqs->setRefTorque(i,0.0);
        }
        return;
    }

    //set the appropriate feedforward term (normal operation)
    for(int i=0;i<part_ctrlJnt;i++)
    {
        int ctrl_mode=0;
        yarp::dev::InteractionModeEnum int_mode;
        iCtrlMode->getControlMode(i,&ctrl_mode);
        iIntMode->getInteractionMode(i,&int_mode);
        switch(ctrl_mode)
        {
            //for all this control modes do nothing
            case VOCAB_CM_CURRENT:
            case VOCAB_CM_PWM:
            case VOCAB_CM_IDLE:
            case VOCAB_CM_UNKNOWN:
            case VOCAB_CM_HW_FAULT:
                break;

            case VOCAB_CM_TORQUE:
                iTqs->setRefTorque(i,command[i]);
                break;

            case VOCAB_CM_POSITION:
            case VOCAB_CM_POSITION_DIRECT:
            case VOCAB_CM_MIXED:
            case VOCAB_CM_VELOCITY:
                 if (int_mode == VOCAB_IM_COMPLIANT)
                 {
                     iImp->setImpedanceOffset(i,command[i]);
                 }
                 else
                 {
                     //stiff or unknown mode, nothing to do
                 }
                 break;

            case VOCAB_CM_IMPEDANCE_POS:
            case VOCAB_CM_IMPEDANCE_VEL:
                    iImp->setImpedanceOffset(i,command[i]);
                break;

            default:
                if (s_part=="torso" && i==3)
                {
                    // do nothing, because joint 3 of the torso is only virtual
                }
                else
                {
                    yError("Unknown control mode (part: %s jnt:%d).\n",s_part.c_str(), i);
                }
                break;
        }
    }
}

void gravityCompensatorThread::run()
{  
    thread_status = STATUS_OK;
    static int delay_check=0;
    if(isCalibrated==true)
    {
        if (readAndUpdate(false) == false)
        {
            delay_check++;
            yWarning ("network delays detected (%d/10)\n", delay_check);
            if (delay_check>=10)
            {
                yError ("gravityCompensatorThread lost connection with wholeBodyDynamics.\n");
                thread_status = STATUS_DISCONNECTED;
            }
        }
        else
        {
            delay_check = 0;
        }

        Vector F_up(6,0.0);
        icub->upperTorso->setInertialMeasure(w0,dw0,d2p0);
        icub->upperTorso->solveKinematics();
        icub->upperTorso->solveWrench();

        //compute the arm torques
        Matrix F_sens_up = icub->upperTorso->estimateSensorsWrench(F_ext_up,false);
        gravity_torques_LA = icub->upperTorso->left->getTorques();
        gravity_torques_RA = icub->upperTorso->right->getTorques();

        //compute the torso torques
        icub->attachLowerTorso(F_up,F_up);
        icub->lowerTorso->solveKinematics();
        icub->lowerTorso->solveWrench();
        Vector tmp; tmp.resize(3);
        tmp = icub->lowerTorso->getTorques("torso");
        gravity_torques_TO[0] = tmp [2];
        gravity_torques_TO[1] = tmp [1];
        gravity_torques_TO[2] = tmp [0];

        //compute the leg torques
        Matrix F_sens_low = icub->lowerTorso->estimateSensorsWrench(F_ext_low,false);
        gravity_torques_LL = icub->lowerTorso->getTorques("left_leg");
        gravity_torques_RL = icub->lowerTorso->getTorques("right_leg");  
        
//#define DEBUG_TORQUES
#ifdef  DEBUG_TORQUES
    yDebug ("TORQUES:     %s ***  \n\n", torques_TO.toString().c_str());
    yDebug ("LL TORQUES:  %s ***  \n\n", torques_LL.toString().c_str());
    yDebug ("RL TORQUES:  %s ***  \n\n", torques_RL.toString().c_str());
#endif

        //read the external command ports
        Vector *offset_input_la = la_additional_offset->read(false);
        if(offset_input_la!=0)
        {
            int size = (offset_input_la->size() < 7) ? offset_input_la->size():7;
            for (int i=0; i<size; i++)
                {externalcmd_torques_LA[i]=(*offset_input_la)[i];}
        }
        Vector *offset_input_ra = ra_additional_offset->read(false);
        if(offset_input_ra!=0)
        {
            int size = (offset_input_ra->size() < 7) ? offset_input_ra->size():7;
            for (int i=0; i<size; i++)
                {externalcmd_torques_RA[i]=(*offset_input_ra)[i];}
        }
        Vector *offset_input_ll = ll_additional_offset->read(false);
        if(offset_input_ll!=0)
        {
            int size = (offset_input_ll->size() < 6) ? offset_input_ll->size():6;
            for (int i=0; i<size; i++)
                {externalcmd_torques_LL[i]=(*offset_input_ll)[i];}
        }
        Vector *offset_input_rl = rl_additional_offset->read(false);
        if(offset_input_rl!=0)
        {
            int size = (offset_input_rl->size() < 6) ? offset_input_rl->size():6;
            for (int i=0; i<size; i++)
                {externalcmd_torques_RL[i]=(*offset_input_rl)[i];}
        }
        Vector *offset_input_to = to_additional_offset->read(false);
        if(offset_input_to!=0)
        {
            int size = (offset_input_to->size() < 3) ? offset_input_to->size():3;
            for (int i=0; i<size; i++)
                {externalcmd_torques_TO[i]=(*offset_input_to)[i];}
        }

        //compute the command to be given to the joint
        if (gravity_mode==GRAVITY_COMPENSATION_ON)
        {
            if (external_mode==EXTERNAL_TRQ_ON)
            {
                exec_torques_LA = ampli_LA*gravity_torques_LA + externalcmd_torques_LA;
                exec_torques_RA = ampli_RA*gravity_torques_RA + externalcmd_torques_RA;
                exec_torques_LL = ampli_LL*gravity_torques_LL + externalcmd_torques_LL;
                exec_torques_RL = ampli_RL*gravity_torques_RL + externalcmd_torques_RL;
                exec_torques_TO = ampli_TO*gravity_torques_TO + externalcmd_torques_TO;
            }
            else
            {
                exec_torques_LA = ampli_LA*gravity_torques_LA;
                exec_torques_RA = ampli_RA*gravity_torques_RA;
                exec_torques_LL = ampli_LL*gravity_torques_LL;
                exec_torques_RL = ampli_RL*gravity_torques_RL;
                exec_torques_TO = ampli_TO*gravity_torques_TO;
            }
        }
        else
        {
            if (external_mode==EXTERNAL_TRQ_ON)
            {
                exec_torques_LA = externalcmd_torques_LA;
                exec_torques_RA = externalcmd_torques_RA;
                exec_torques_LL = externalcmd_torques_LL;
                exec_torques_RL = externalcmd_torques_RL;
                exec_torques_TO = externalcmd_torques_TO;
            }
            else
            {
                externalcmd_torques_LA.zero();
                externalcmd_torques_RA.zero();
                externalcmd_torques_LL.zero();
                externalcmd_torques_RL.zero();
                externalcmd_torques_TO.zero();
            }
        }

        //execute the commands
        static yarp::os::Stamp timestamp;
        timestamp.update();
        if (iCtrlMode_arm_left)
        {
            feedFwdGravityControl(left_arm_ctrlJnt, "left_arm", iCtrlMode_arm_left,iTqs_arm_left,iImp_arm_left,iIntMode_arm_left,exec_torques_LA);
            if (left_arm_exec_torques && left_arm_exec_torques->getOutputCount()>0)
            {
                left_arm_exec_torques->prepare()  =  exec_torques_LA;
                left_arm_exec_torques->setEnvelope(timestamp);
                left_arm_exec_torques->write();
            }
            if (left_arm_gravity_torques && left_arm_gravity_torques->getOutputCount()>0)
            {
                left_arm_gravity_torques->prepare()  =  gravity_torques_LA;
                left_arm_gravity_torques->setEnvelope(timestamp);
                left_arm_gravity_torques->write();
            }
        }
        if (iCtrlMode_arm_right)
        {
            feedFwdGravityControl(right_arm_ctrlJnt, "right_arm", iCtrlMode_arm_right,iTqs_arm_right,iImp_arm_right,iIntMode_arm_right,exec_torques_RA);
            if (right_arm_exec_torques && right_arm_exec_torques->getOutputCount()>0)
            {
                right_arm_exec_torques->prepare() =  exec_torques_RA;
                right_arm_exec_torques->setEnvelope(timestamp);
                right_arm_exec_torques->write();
            }
            if (right_arm_gravity_torques && right_arm_gravity_torques->getOutputCount()>0)
            {
                right_arm_gravity_torques->prepare()  =  gravity_torques_RA;
                right_arm_gravity_torques->setEnvelope(timestamp);
                right_arm_gravity_torques->write();
            }
        }
        if (iCtrlMode_torso)  
        {
            feedFwdGravityControl(torso_ctrlJnt, "torso", iCtrlMode_torso,iTqs_torso,iImp_torso,iIntMode_torso,exec_torques_TO);
            if (torso_exec_torques && torso_exec_torques->getOutputCount()>0)
            {
                torso_exec_torques->prepare()  =  exec_torques_TO;
                torso_exec_torques->setEnvelope(timestamp);
                torso_exec_torques->write();
            }
            if (torso_gravity_torques && torso_gravity_torques->getOutputCount()>0)
            {
                torso_gravity_torques->prepare()  =  gravity_torques_TO;
                torso_gravity_torques->setEnvelope(timestamp);
                torso_gravity_torques->write();
            }
        }
        if (iCtrlMode_leg_left)    
        {
            feedFwdGravityControl(left_leg_ctrlJnt, "left_leg", iCtrlMode_leg_left,iTqs_leg_left,iImp_leg_left,iIntMode_leg_left,exec_torques_LL);
            if (left_leg_exec_torques && left_leg_exec_torques->getOutputCount()>0)
            {
                left_leg_exec_torques->prepare() =  exec_torques_LL;
                left_leg_exec_torques->setEnvelope(timestamp);
                left_leg_exec_torques->write();
            }
            if (left_leg_gravity_torques && left_leg_gravity_torques->getOutputCount()>0)
            {
                left_leg_gravity_torques->prepare() =  gravity_torques_LL;
                left_leg_gravity_torques->setEnvelope(timestamp);
                left_leg_gravity_torques->write();
            }
        }
        if (iCtrlMode_leg_right)
        {
            feedFwdGravityControl(right_leg_ctrlJnt, "right_leg", iCtrlMode_leg_right,iTqs_leg_right,iImp_leg_right,iIntMode_leg_right,exec_torques_RL);
            if (right_leg_exec_torques && right_leg_exec_torques->getOutputCount()>0)
            {
                right_leg_exec_torques->prepare()  =  exec_torques_RL;
                right_leg_exec_torques->setEnvelope(timestamp);
                right_leg_exec_torques->write();
            }
            if (right_leg_gravity_torques && right_leg_gravity_torques->getOutputCount()>0)
            {
                right_leg_gravity_torques->prepare()  =  gravity_torques_RL;
                right_leg_gravity_torques->setEnvelope(timestamp);
                right_leg_gravity_torques->write();
            }
        }
    }
    else
    {
        if(Network::exists(string("/"+wholeBodyName+"/filtered/inertial:o").c_str()))
        {
            yInfo("connection exists! starting calibration...\n");
            //the following delay is required because even if the filtered port exists, may be the 
            //low pass filtered values have not reached yet the correct value. 
            Time::delay(1.0); 

            isCalibrated = true;
            Network::connect(string("/"+wholeBodyName+"/filtered/inertial:o").c_str(),"/gravityCompensator/inertial:i");
            setZeroJntAngVelAcc();
            setUpperMeasure();
            setLowerMeasure();

            readAndUpdate(true);

            icub->upperTorso->setInertialMeasure(w0,dw0,d2p0);
            Matrix F_sens_up = icub->upperTorso->estimateSensorsWrench(F_ext_up,false);
            icub->lowerTorso->setInertialMeasure(icub->upperTorso->getTorsoAngVel(),
                                                icub->upperTorso->getTorsoAngAcc(),
                                                icub->upperTorso->getTorsoLinAcc());
            Matrix F_sens_low = icub->lowerTorso->estimateSensorsWrench(F_ext_low,false);
            gravity_torques_LA = icub->upperTorso->getTorques("left_arm");
            gravity_torques_RA = icub->upperTorso->getTorques("right_arm");
            gravity_torques_LL = icub->lowerTorso->getTorques("left_leg");
            gravity_torques_RL = icub->lowerTorso->getTorques("right_leg");  
            Vector LATorques = icub->upperTorso->getTorques("left_arm");
            yDebug("encoders:  %.1lf, %.1lf, %.1lf, %.1lf, %.1lf, %.1lf, %.1lf\n", encoders_arm_left(0), encoders_arm_left(1), encoders_arm_left(2), encoders_arm_left(3), encoders_arm_left(4), encoders_arm_left(5), encoders_arm_left(6)); 
            yDebug("left  arm: %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf\n", gravity_torques_LA(0), gravity_torques_LA(1), gravity_torques_LA(2), gravity_torques_LA(3), gravity_torques_LA(4), gravity_torques_LA(5), gravity_torques_LA(6)); 
            yDebug("right arm: %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf\n", gravity_torques_RA(0), gravity_torques_RA(1), gravity_torques_RA(2), gravity_torques_RA(3), gravity_torques_RA(4), gravity_torques_RA(5), gravity_torques_RA(6)); 
            yDebug("left  leg: %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf\n",        gravity_torques_LL(0), gravity_torques_LL(1), gravity_torques_LL(2), gravity_torques_LL(3), gravity_torques_LL(4), gravity_torques_LL(5)); 
            yDebug("right leg: %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf\n",        gravity_torques_RL(0), gravity_torques_RL(1), gravity_torques_RL(2), gravity_torques_RL(3), gravity_torques_RL(4), gravity_torques_RL(5)); 
            yDebug("inertial:  %.1lf, %.1lf, %.1lf, %.1lf, %.1lf, %.1lf, %.1lf, %.1lf, %.1lf\n", d2p0(0), d2p0(1), d2p0(2), w0(0), w0(1), w0(2), dw0(0), dw0(1), dw0(2)); 
        }
        else
        {
            yInfo("waiting for connections from wholeBodyDynamics (port: %s)...\n", wholeBodyName.c_str());
            Time::delay(1.0);
        }
    }
}
    
void gravityCompensatorThread::threadRelease()
{
    externalcmd_torques_LA.zero();
    externalcmd_torques_RA.zero();
    externalcmd_torques_LL.zero();
    externalcmd_torques_RL.zero();
    externalcmd_torques_TO.zero();
    gravity_torques_LA.zero();
    gravity_torques_RA.zero();
    gravity_torques_LL.zero();
    gravity_torques_RL.zero();
    gravity_torques_TO.zero();
    exec_torques_LA.zero();
    exec_torques_RA.zero();
    exec_torques_LL.zero();
    exec_torques_RL.zero();
    exec_torques_TO.zero();

    if (iCtrlMode_arm_left)
    {
        yInfo("Setting gravity compensation offset to zero, left arm\n");
        feedFwdGravityControl(left_arm_ctrlJnt, "left_arm",iCtrlMode_arm_left,iTqs_arm_left,iImp_arm_left,iIntMode_arm_left,exec_torques_LA,true);
    }
    if (iCtrlMode_arm_right)    
    {
        yInfo("Setting gravity compensation offset to zero, right arm\n");
        feedFwdGravityControl(right_arm_ctrlJnt, "right_arm",iCtrlMode_arm_right,iTqs_arm_right,iImp_arm_right,iIntMode_arm_right,exec_torques_RA,true);
    }
    if (iCtrlMode_leg_left)     
    {
        yInfo("Setting gravity compensation offset to zero, left leg\n");
        feedFwdGravityControl(left_leg_ctrlJnt, "left_leg", iCtrlMode_leg_left,iTqs_leg_left,iImp_leg_left,iIntMode_leg_left,exec_torques_LL,true);
    }
    if (iCtrlMode_leg_right)
    {
        yInfo("Setting gravity compensation offset to zero, right leg\n");
        feedFwdGravityControl(right_leg_ctrlJnt, "right_leg",iCtrlMode_leg_right,iTqs_leg_right,iImp_leg_right,iIntMode_leg_right,exec_torques_RL,true);
    }
    if (iCtrlMode_torso)
    {
        yInfo("Setting gravity compensation offset to zero, torso\n");
        feedFwdGravityControl(torso_ctrlJnt, "torso",iCtrlMode_torso,iTqs_torso,iImp_torso,iIntMode_torso,exec_torques_TO,true);
    }
    Time::delay(0.5);

    left_arm_exec_torques->interrupt();
    right_arm_exec_torques->interrupt();
    left_leg_exec_torques->interrupt();
    right_leg_exec_torques->interrupt();
    torso_exec_torques->interrupt();
    left_arm_exec_torques->close();
    right_arm_exec_torques->close();
    left_leg_exec_torques->close();
    right_leg_exec_torques->close();
    torso_exec_torques->close();

    left_arm_gravity_torques->interrupt();
    right_arm_gravity_torques->interrupt();
    left_leg_gravity_torques->interrupt();
    right_leg_gravity_torques->interrupt();
    torso_gravity_torques->interrupt();
    left_arm_gravity_torques->close();
    right_arm_gravity_torques->close();
    left_leg_gravity_torques->close();
    right_leg_gravity_torques->close();
    torso_gravity_torques->close();

    if (left_arm_exec_torques)  {delete left_arm_exec_torques;  left_arm_exec_torques = 0;}
    if (right_arm_exec_torques) {delete right_arm_exec_torques; right_arm_exec_torques = 0;}
    if (left_leg_exec_torques)  {delete left_leg_exec_torques;  left_leg_exec_torques = 0;}
    if (right_leg_exec_torques) {delete right_leg_exec_torques; right_leg_exec_torques = 0;}
    if (torso_exec_torques)     {delete torso_exec_torques;     torso_exec_torques = 0;}

    if (left_arm_gravity_torques)  {delete left_arm_gravity_torques;  left_arm_gravity_torques = 0;}
    if (right_arm_gravity_torques) {delete right_arm_gravity_torques; right_arm_gravity_torques = 0;}
    if (left_leg_gravity_torques)  {delete left_leg_gravity_torques;  left_leg_gravity_torques = 0;}
    if (right_leg_gravity_torques) {delete right_leg_gravity_torques; right_leg_gravity_torques = 0;}
    if (torso_gravity_torques)     {delete torso_gravity_torques;     torso_gravity_torques = 0;}

    if (linEstUp)          {delete linEstUp;   linEstUp = 0;}
    if (quadEstUp)         {delete quadEstUp;  quadEstUp = 0;}
    if (linEstLow)         {delete linEstLow;  linEstLow = 0;}
    if (quadEstLow)        {delete quadEstLow; quadEstLow = 0;}

    //closing ports
    port_inertial->interrupt();
    port_inertial->close();
    la_additional_offset->interrupt();
    la_additional_offset->close();
    ra_additional_offset->interrupt();
    ra_additional_offset->close();
    ll_additional_offset->interrupt();
    ll_additional_offset->close();
    rl_additional_offset->interrupt();
    rl_additional_offset->close();
    to_additional_offset->interrupt();
    to_additional_offset->close();
    if (icub)      {delete icub; icub=0;}
}

void gravityCompensatorThread::closePort(Contactable *_port)
{
}

