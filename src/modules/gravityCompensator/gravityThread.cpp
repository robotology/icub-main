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
    torques_LA.resize(7);torques_RA.resize(7);
    ampli_larm.resize(7);ampli_larm=1.0;
    ampli_rarm.resize(7);ampli_rarm=1.0;
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
    torques_TO.resize(3);torques_LL.resize(6);torques_RL.resize(6);
    ampli_lleg.resize(6);ampli_lleg=1.0;
    ampli_rleg.resize(6);ampli_rleg=1.0;
    ampli_torso.resize(6);ampli_torso=1.0;

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

gravityCompensatorThread::gravityCompensatorThread(int _rate, PolyDriver *_ddLA, PolyDriver *_ddRA, PolyDriver *_ddH, PolyDriver *_ddLL, PolyDriver *_ddRL, PolyDriver *_ddT, version_tag icub_type) : RateThread(_rate), ddLA(_ddLA), ddRA(_ddRA), ddLL(_ddLL), ddRL(_ddRL), ddH(_ddH), ddT(_ddT)
{   
    gravity_mode = GRAVITY_COMPENSATION_ON;
    wholeBodyName = "wholeBodyDynamics";

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
    icub = new iCubWholeBody (icub_type,DYNAMIC, VERBOSE);

    //---------------------PORTS-------------------------//
    port_inertial=new BufferedPort<Vector>;
    port_inertial->open("/gravityCompensator/inertial:i");

    left_arm_additional_offset=new BufferedPort<Vector>;
    left_arm_additional_offset->open("/gravityCompensator/left_arm_ctrlOffset:i");
    right_arm_additional_offset=new BufferedPort<Vector>;
    right_arm_additional_offset->open("/gravityCompensator/right_arm_ctrlOffset:i");
    left_leg_additional_offset=new BufferedPort<Vector>;
    left_leg_additional_offset->open("/gravityCompensator/left_leg_ctrlOffset:i");
    right_leg_additional_offset=new BufferedPort<Vector>;
    right_leg_additional_offset->open("/gravityCompensator/right_leg_ctrlOffset:i");
    torso_additional_offset=new BufferedPort<Vector>;
    torso_additional_offset->open("/gravityCompensator/torso_ctrlOffset:i");

    left_arm_torques = new BufferedPort<Vector>;
    left_arm_torques->open("/gravityCompensator/left_arm_torques:o");
    right_arm_torques = new BufferedPort<Vector>;
    right_arm_torques->open("/gravityCompensator/right_arm_torques:o");
    left_leg_torques = new BufferedPort<Vector>;
    left_leg_torques->open("/gravityCompensator/left_leg_torques:o");
    right_leg_torques = new BufferedPort<Vector>;
    right_leg_torques->open("/gravityCompensator/right_leg_torques:o");
    torso_torques = new BufferedPort<Vector>;
    torso_torques->open("/gravityCompensator/torso_torques:o");

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
        case GRAVITY_COMPENSATION_OFF:        printf("GRAVITY_COMPENSATION_OFF     \n");    break;
        case GRAVITY_COMPENSATION_ON:         printf("GRAVITY_COMPENSATION_ON      \n");    break;
        default:
        case VOCAB_CM_UNKNOWN:                printf("UNKNOWN  \n");    break;
    }
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
            fprintf(stderr,"warning...controlled joint < of offsets size!!!");
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
    thread_status = STATUS_OK;
    return true;
}


void gravityCompensatorThread::feedFwdGravityControl(int part_ctrlJnt, string s_part, IControlMode2 *iCtrlMode, ITorqueControl *iTqs, IImpedanceControl *iImp, IInteractionMode *iIntMode, const Vector &G, const Vector &ampli, bool releasing)
{
    //check if interfaces are still up (icubinterface running)  
    if (iCtrlMode == 0) 
        {fprintf(stderr,"ControlMode interface already closed, unable to reset compensation offset.\n");    return;}
    if (iTqs == 0)
        {fprintf(stderr,"TorqueControl interface already closed, unable to reset compensation offset.\n");  return;}
    if (iIntMode == 0)
        {fprintf(stderr,"InteractionMode interface already closed, unable to reset compensation offset.\n");  return;}
    if (iImp == 0)
        {fprintf(stderr,"Impedance interface already closed, unable to reset compensation offset.\n");      return;}

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
            case VOCAB_CM_OPENLOOP:
            case VOCAB_CM_IDLE:
            case VOCAB_CM_UNKNOWN:
            case VOCAB_CM_HW_FAULT:
                break;

            case VOCAB_CM_TORQUE:    
                if(gravity_mode == GRAVITY_COMPENSATION_ON)
                {
                    //iTqs->setRefTorque(i,ampli[i]*G[i]+torque_offset[i]);
                    iTqs->setRefTorque(i,ampli[i]*G[i]);
                }
                else
                {
                    //iTqs->setRefTorque(i,torque_offset[i]);
                }
                break;
            case VOCAB_CM_POSITION:
            case VOCAB_CM_POSITION_DIRECT:
            case VOCAB_CM_MIXED:
            case VOCAB_CM_VELOCITY:
                 if (int_mode == VOCAB_IM_COMPLIANT)
                 {
                    if(gravity_mode == GRAVITY_COMPENSATION_ON)
                    {
                        //fprintf(stderr,"compensating gravity\n");
                        //double off = ampli[i]*G[i]+torque_offset[i];
                        double off = ampli[i]*G[i];
                        iImp->setImpedanceOffset(i,off);
                    }
                    else
                    {
                        //iImp->setImpedanceOffset(i,torque_offset[i]);
                    }
                 }
                 else
                 {
                     //stiff or unknown mode, nothing to do
                 }
                 break;
            case VOCAB_CM_IMPEDANCE_POS:
            case VOCAB_CM_IMPEDANCE_VEL:
                if(gravity_mode == GRAVITY_COMPENSATION_ON)
                {
                    //fprintf(stderr,"compensating gravity\n");
                    //double off = ampli[i]*G[i]+torque_offset[i];
                    double off = ampli[i]*G[i];
                    iImp->setImpedanceOffset(i,off);
                }
                else
                {
                    //iImp->setImpedanceOffset(i,torque_offset[i]);
                }
                break;
            default:
                if (s_part=="torso" && i==3)
                {
                    // do nothing, because joint 3 of the torso is only virtual
                }
                else
                {
                    fprintf(stderr,"Unknown control mode (part: %s jnt:%d).\n",s_part.c_str(), i);
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
            printf ("network delays detected (%d/10)\n", delay_check);
            if (delay_check>=10)
            {
                printf ("gravityCompensatorThread lost connection with wholeBodyDynamics.\n");
                thread_status = STATUS_DISCONNECTED;
            }
        }
        else
        {
            delay_check = 0;
        }

        Vector F_up(6);
        F_up=0.0;
        icub->upperTorso->setInertialMeasure(w0,dw0,d2p0);
        icub->upperTorso->solveKinematics();
        icub->upperTorso->solveWrench();

        //compute the arm torques
        Matrix F_sens_up = icub->upperTorso->estimateSensorsWrench(F_ext_up,false);
        torques_LA = icub->upperTorso->left->getTorques();
        torques_RA = icub->upperTorso->right->getTorques();

        //compute the torso torques
        icub->attachLowerTorso(F_up,F_up);
        icub->lowerTorso->solveKinematics();
        icub->lowerTorso->solveWrench();
        Vector tmp; tmp.resize(3);
        tmp = icub->lowerTorso->getTorques("torso");
        torques_TO[0] = tmp [2];
        torques_TO[1] = tmp [1];
        torques_TO[2] = tmp [0];

        //compute the leg torques
        Matrix F_sens_low = icub->lowerTorso->estimateSensorsWrench(F_ext_low,false);
        torques_LL = icub->lowerTorso->getTorques("left_leg");
        torques_RL = icub->lowerTorso->getTorques("right_leg");  
        
//#define DEBUG_TORQUES
#ifdef  DEBUG_TORQUES
    fprintf (stderr,"TORQUES:     %s ***  \n\n", torques_TO.toString().c_str());
    fprintf (stderr,"LL TORQUES:  %s ***  \n\n", torques_LL.toString().c_str());
    fprintf (stderr,"RL TORQUES:  %s ***  \n\n", torques_RL.toString().c_str());
#endif

        if (iCtrlMode_arm_left)  
        {
            feedFwdGravityControl(left_arm_ctrlJnt, "left_arm", iCtrlMode_arm_left,iTqs_arm_left,iImp_arm_left,iIntMode_arm_left,torques_LA,ampli_larm);
            if (left_arm_torques->getOutputCount()>0)
            {
                left_arm_torques->prepare()  =  torques_LA;
                left_arm_torques->write();
            }
        }
        if (iCtrlMode_arm_right)
        {
            feedFwdGravityControl(right_arm_ctrlJnt, "right_arm", iCtrlMode_arm_right,iTqs_arm_right,iImp_arm_right,iIntMode_arm_right,torques_RA,ampli_rarm);
            if (right_arm_torques->getOutputCount()>0)
            {
                right_arm_torques->prepare() =  torques_RA;
                right_arm_torques->write();
            }
        }
        if (iCtrlMode_torso)  
        {
            feedFwdGravityControl(torso_ctrlJnt, "torso", iCtrlMode_torso,iTqs_torso,iImp_torso,iIntMode_torso,torques_TO,ampli_torso);
            if (torso_torques->getOutputCount()>0)
            {
                torso_torques->prepare()  =  torques_TO;
                torso_torques->write();
            }
        }
        if (iCtrlMode_leg_left)    
        {
            feedFwdGravityControl(left_leg_ctrlJnt, "left_leg", iCtrlMode_leg_left,iTqs_leg_left,iImp_leg_left,iIntMode_leg_left,torques_LL,ampli_lleg);
            if (left_leg_torques->getOutputCount()>0)
            {
                left_leg_torques->prepare() =  torques_LL;
                left_leg_torques->write();
            }
        }
        if (iCtrlMode_leg_right)
        {
            feedFwdGravityControl(right_leg_ctrlJnt, "right_leg", iCtrlMode_leg_right,iTqs_leg_right,iImp_leg_right,iIntMode_leg_right,torques_RL,ampli_rleg);
            if (right_leg_torques->getOutputCount()>0)
            {
                right_leg_torques->prepare()  =  torques_RL;
                right_leg_torques->write();
            }
        }
    }
    else
    {
        if(Network::exists(string("/"+wholeBodyName+"/filtered/inertial:o").c_str()))
        {
            fprintf(stderr,"connection exists! starting calibration...\n");
            //the following delay is required because even if the filtered port exists, may be the 
            //low pass filtered values have not reached yet the correct value. 
            Time::delay(1.0); 

            isCalibrated = true;
            Network::connect(string("/"+wholeBodyName+"/filtered/inertial:o").c_str(),"/gravityCompensator/inertial:i");
            setZeroJntAngVelAcc();
            setUpperMeasure();
            setLowerMeasure();

            readAndUpdate(true);

            Vector F_up(6);
            F_up=0.0;
            icub->upperTorso->setInertialMeasure(w0,dw0,d2p0);
            Matrix F_sens_up = icub->upperTorso->estimateSensorsWrench(F_ext_up,false);
            icub->lowerTorso->setInertialMeasure(icub->upperTorso->getTorsoAngVel(),
                                                icub->upperTorso->getTorsoAngAcc(),
                                                icub->upperTorso->getTorsoLinAcc());
            Matrix F_sens_low = icub->lowerTorso->estimateSensorsWrench(F_ext_low,false);
            torques_LA = icub->upperTorso->getTorques("left_arm");
            torques_RA = icub->upperTorso->getTorques("right_arm");
            torques_LL = icub->lowerTorso->getTorques("left_leg");
            torques_RL = icub->lowerTorso->getTorques("right_leg");  
            Vector LATorques = icub->upperTorso->getTorques("left_arm");
            printf("encoders:  %.1lf, %.1lf, %.1lf, %.1lf, %.1lf, %.1lf, %.1lf\n", encoders_arm_left(0), encoders_arm_left(1), encoders_arm_left(2), encoders_arm_left(3), encoders_arm_left(4), encoders_arm_left(5), encoders_arm_left(6)); 
            printf("left  arm: %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf\n", torques_LA(0), torques_LA(1), torques_LA(2), torques_LA(3), torques_LA(4), torques_LA(5), torques_LA(6)); 
            printf("right arm: %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf\n", torques_RA(0), torques_RA(1), torques_RA(2), torques_RA(3), torques_RA(4), torques_RA(5), torques_RA(6)); 
            printf("left  leg: %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf\n",        torques_LL(0), torques_LL(1), torques_LL(2), torques_LL(3), torques_LL(4), torques_LL(5)); 
            printf("right leg: %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf\n",        torques_RL(0), torques_RL(1), torques_RL(2), torques_RL(3), torques_RL(4), torques_RL(5)); 
            printf("inertial:  %.1lf, %.1lf, %.1lf, %.1lf, %.1lf, %.1lf, %.1lf, %.1lf, %.1lf\n", d2p0(0), d2p0(1), d2p0(2), w0(0), w0(1), w0(2), dw0(0), dw0(1), dw0(2)); 
        }
        else
        {
            fprintf(stderr,"waiting for connections from wholeBodyDynamics (port: %s)...\n", wholeBodyName.c_str());
            Time::delay(1.0);
        }
    }
}
    
void gravityCompensatorThread::threadRelease()
{
    Vector Z(10);Z=0.0;
    
    if (iCtrlMode_arm_left)
    {
        fprintf(stderr,"Setting gravity compensation offset to zero, left arm\n");
        feedFwdGravityControl(left_arm_ctrlJnt, "left_arm",iCtrlMode_arm_left,iTqs_arm_left,iImp_arm_left,iIntMode_arm_left,Z,ampli_larm,true);
    }
    if (iCtrlMode_arm_right)    
    {
        fprintf(stderr,"Setting gravity compensation offset to zero, right arm\n");
        feedFwdGravityControl(right_arm_ctrlJnt, "right_arm",iCtrlMode_arm_right,iTqs_arm_right,iImp_arm_right,iIntMode_arm_right,Z,ampli_rarm,true);
    }
    if (iCtrlMode_leg_left)     
    {
        fprintf(stderr,"Setting gravity compensation offset to zero, left leg\n");
        feedFwdGravityControl(left_leg_ctrlJnt, "left_leg", iCtrlMode_leg_left,iTqs_leg_left,iImp_leg_left,iIntMode_leg_left,Z,ampli_lleg,true);
    }
    if (iCtrlMode_leg_right)
    {
        fprintf(stderr,"Setting gravity compensation offset to zero, right leg\n");
        feedFwdGravityControl(right_leg_ctrlJnt, "right_leg",iCtrlMode_leg_right,iTqs_leg_right,iImp_leg_right,iIntMode_leg_right,Z,ampli_rleg,true);
    }
    if (iCtrlMode_torso)
    {
        fprintf(stderr,"Setting gravity compensation offset to zero, torso\n");
        feedFwdGravityControl(torso_ctrlJnt, "torso",iCtrlMode_torso,iTqs_torso,iImp_torso,iIntMode_torso,Z,ampli_torso,true);
    }

    Time::delay(0.5);

/*    left_arm_torques->interrupt();
    right_arm_torques->interrupt();
    left_leg_torques->interrupt();
    right_leg_torques->interrupt();*/
/*    left_arm_torques->close();
    right_arm_torques->close();
    left_leg_torques->close();
    right_leg_torques->close();*/

    if (left_arm_torques)  {delete left_arm_torques;  left_arm_torques = 0;}
    if (right_arm_torques) {delete right_arm_torques; right_arm_torques = 0;}
    if (left_leg_torques)  {delete left_leg_torques;  left_leg_torques = 0;}
    if (right_leg_torques) {delete right_leg_torques; right_leg_torques = 0;}
    if (torso_torques)     {delete torso_torques;     torso_torques = 0;}        
    
    if (linEstUp)          {delete linEstUp;   linEstUp = 0;}
    if (quadEstUp)         {delete quadEstUp;  quadEstUp = 0;}
    if (linEstLow)         {delete linEstLow;  linEstLow = 0;}
    if (quadEstLow)        {delete quadEstLow; quadEstLow = 0;}

    //closing ports
    port_inertial->interrupt();
    port_inertial->close();
    left_arm_additional_offset->interrupt();
    left_arm_additional_offset->close();
    right_arm_additional_offset->interrupt();
    right_arm_additional_offset->close();
    left_leg_additional_offset->interrupt();
    left_leg_additional_offset->close();
    right_leg_additional_offset->interrupt();
    right_leg_additional_offset->close();
    torso_additional_offset->interrupt();
    torso_additional_offset->close();
    if (icub)      {delete icub; icub=0;}
}   

void gravityCompensatorThread::closePort(Contactable *_port)
{
}

