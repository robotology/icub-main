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

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <iCub/ctrl/math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>
#include <iCub/skinDynLib/skinContact.h>

#include <iostream>
#include <iomanip>
#include <string.h>
#include "observerThread.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::ctrl;
using namespace iCub::iDyn;
using namespace iCub::skinDynLib;
using namespace std;

#define TEST_LEG_SENSOR
#define MEASURE_FROM_FOOT
double lpf_ord1_3hz(double input, int j)
{ 
    if (j<0 || j>= MAX_JN)
    {
        cout<<"Received an invalid joint index to filter"<<endl;
        return 0;
    }

    static double xv[MAX_FILTER_ORDER][MAX_JN];
    static double yv[MAX_FILTER_ORDER][MAX_JN];
    xv[0][j] = xv[1][j] ; 
    xv[1][j] = input / 1.157889499e+01;
    yv[0][j] = yv[1][j] ; 
    yv[1][j] =   (xv[0][j]  + xv[1][j] ) + (  0.8272719460 * yv[0][j] );
    return (yv[1][j]);
}

Vector inverseDynamics::evalVelUp(const Vector &x)
{
    AWPolyElement el;
    el.data=x;
    el.time=Time::now();

    return linEstUp->estimate(el);
}

Vector inverseDynamics::evalVelLow(const Vector &x)
{
    AWPolyElement el;
    el.data=x;
    el.time=Time::now();

    return linEstLow->estimate(el);
}

Vector inverseDynamics::eval_domega(const Vector &x)
{
    AWPolyElement el;
    el.data=x;
    el.time=Time::now();

    return InertialEst->estimate(el);
}

Vector inverseDynamics::evalAccUp(const Vector &x)
{
    AWPolyElement el;
    el.data=x;
    el.time=Time::now();

    return quadEstUp->estimate(el);
}

Vector inverseDynamics::evalAccLow(const Vector &x)
{
    AWPolyElement el;
    el.data=x;
    el.time=Time::now();

    return quadEstLow->estimate(el);
}

void inverseDynamics::init_upper()
{
    //---------------------PARTS-------------------------//
    // Left_arm variables
    allJnt = 0;
    int jnt=7;
    encoders_arm_left.resize(jnt,0.0);
    F_LArm.resize(6,0.0);
    F_iDyn_LArm.resize(6,0.0);
    Offset_LArm.resize(6,0.0);
    allJnt+=jnt;

    // Right_arm variables
    jnt = 7;
    encoders_arm_right.resize(jnt,0.0);
    F_RArm.resize(6,0.0);
    F_iDyn_RArm.resize(6,0.0);
    Offset_RArm.resize(6,0.0);
    allJnt+=jnt;

    // Head variables
    jnt = 3;
    encoders_head.resize(jnt,0.0);
    allJnt+=jnt;

    current_status.all_q_up.resize(allJnt,0.0);
    current_status.all_dq_up.resize(allJnt,0.0);
    current_status.all_d2q_up.resize(allJnt,0.0);
    F_sens_up.zero(); 
    FM_sens_up.resize(6,2); FM_sens_up.zero();
}

void inverseDynamics::init_lower()
{
    //---------------------PARTS-------------------------//
    // Left_leg variables
    allJnt = 0;
    int jnt=6;
    encoders_leg_left.resize(jnt,0.0);
    F_LLeg.resize(6,0.0);
    F_iDyn_LLeg.resize(6,0.0);
    Offset_LLeg.resize(6,0.0);
    F_LFoot.resize(6,0.0);
    F_iDyn_LFoot.resize(6,0.0);
    Offset_LFoot.resize(6,0.0);
    allJnt+=jnt;

    // Right_leg variables
    jnt = 6;
    encoders_leg_right.resize(jnt,0.0);
    F_RLeg.resize(6,0.0);
    F_iDyn_RLeg.resize(6,0.0);
    Offset_RLeg.resize(6,0.0);
    F_RFoot.resize(6,0.0);
    F_iDyn_RFoot.resize(6,0.0);
    Offset_RFoot.resize(6,0.0);
    allJnt+=jnt;

    // Torso variables
    jnt = 3;
    encoders_torso.resize(jnt,0.0);
    allJnt+=jnt;

    current_status.all_q_low.resize(allJnt,0.0);
    current_status.all_dq_low.resize(allJnt,0.0);
    current_status.all_d2q_low.resize(allJnt,0.0);

    F_sens_low.zero();
    FM_sens_low.resize(6,2); FM_sens_low.zero();
}

void inverseDynamics::setStiffMode()
{
     if (iint_arm_left)
     {
         for (int i=0; i<7; i++)
            iint_arm_left->setInteractionMode(i,VOCAB_IM_STIFF);
     }
     if (iint_arm_right)
     {
         for (int i=0; i<7; i++)
            iint_arm_right->setInteractionMode(i,VOCAB_IM_STIFF);
     }
     if (iint_leg_left)
     {
         for (int i=0; i<6; i++)
            iint_leg_left->setInteractionMode(i,VOCAB_IM_STIFF);
     }
     if (iint_leg_right)
     {
         for (int i=0; i<6; i++)
             iint_leg_right->setInteractionMode(i,VOCAB_IM_STIFF);
     }
     if (iint_torso)
     {
         for (int i=0; i<3; i++)
            iint_torso->setInteractionMode(i,VOCAB_IM_STIFF);
     }

     if (icmd_arm_left)
     {
         for (int i=0; i<7; i++)
             {
                 int mode =0; icmd_arm_left->getControlMode(i, &mode);
                 if (mode==VOCAB_CM_TORQUE)  icmd_arm_left->setControlMode(i, VOCAB_CM_POSITION);
             }
     }

     if (icmd_arm_right)
     {
         for (int i=0; i<7; i++)
             {
                 int mode =0; icmd_arm_right->getControlMode(i, &mode);
                 if (mode==VOCAB_CM_TORQUE)  icmd_arm_right->setControlMode(i, VOCAB_CM_POSITION);
             }
     }

     if (icmd_leg_left)
     {
         for (int i=0; i<6; i++)
             {
                 int mode =0; icmd_leg_left->getControlMode(i, &mode);
                 if (mode==VOCAB_CM_TORQUE)  icmd_leg_left->setControlMode(i, VOCAB_CM_POSITION);
             }
     }

     if (icmd_leg_right)
     {
         for (int i=0; i<6; i++)
             {
                 int mode =0; icmd_leg_right->getControlMode(i, &mode);
                 if (mode==VOCAB_CM_TORQUE)  icmd_leg_right->setControlMode(i, VOCAB_CM_POSITION);
             }
     }

     if (icmd_torso)
     {
         for (int i=0; i<3; i++)
             {
                 int mode =0; icmd_torso->getControlMode(i, &mode);
                 if (mode==VOCAB_CM_TORQUE)  icmd_torso->setControlMode(i, VOCAB_CM_POSITION);
             }
     }
}

inverseDynamics::inverseDynamics(int _rate, PolyDriver *_ddAL, PolyDriver *_ddAR, PolyDriver *_ddH, PolyDriver *_ddLL, PolyDriver *_ddLR, PolyDriver *_ddT, string _robot_name, string _local_name, version_tag _icub_type, bool _autoconnect) : RateThread(_rate), ddAL(_ddAL), ddAR(_ddAR), ddH(_ddH), ddLL(_ddLL), ddLR(_ddLR), ddT(_ddT), robot_name(_robot_name), icub_type(_icub_type), local_name(_local_name), zero_sens_tolerance (1e-12)
{
    status_queue_size = 10;
    autoconnect = _autoconnect;
    com_enabled = true;
    com_vel_enabled = false;
    dummy_ft    = false;
    w0_dw0_enabled   = false;
    dumpvel_enabled = false;
    auto_drift_comp = false;
    add_legs_once = false;

    icub      = new iCubWholeBody(icub_type, DYNAMIC, VERBOSE);
    icub_sens = new iCubWholeBody(icub_type, DYNAMIC, VERBOSE);
    first = true;
    skinContactsTimestamp = 0.0;

    //--------------INTERFACE INITIALIZATION-------------//
    iencs_arm_left = 0;
    iencs_arm_right= 0;
    iencs_head     = 0;
    iencs_leg_left = 0;
    iencs_leg_right= 0;
    iencs_torso    = 0;
    iint_arm_left  = 0;
    iint_arm_right = 0;
    iint_head      = 0;
    iint_leg_left  = 0;
    iint_leg_right = 0;
    iint_torso     = 0;
    icmd_arm_left  = 0;
    icmd_arm_right = 0;
    icmd_head      = 0;
    icmd_leg_left  = 0;
    icmd_leg_right = 0;
    icmd_torso     = 0;

    //---------------------PORT--------------------------//

    port_inertial_thread=new BufferedPort<Vector>;
    port_ft_arm_left=new BufferedPort<Vector>;
    port_ft_arm_right=new BufferedPort<Vector>;
    port_ft_leg_left=new BufferedPort<Vector>;
    port_ft_leg_right=new BufferedPort<Vector>;
    port_ft_foot_left=new BufferedPort<Vector>;
    port_ft_foot_right=new BufferedPort<Vector>;
    port_RATorques = new BufferedPort<Bottle>;
    port_LATorques = new BufferedPort<Bottle>;
    port_RLTorques = new BufferedPort<Bottle>;
    port_LLTorques = new BufferedPort<Bottle>;
    port_RWTorques = new BufferedPort<Bottle>;
    port_LWTorques = new BufferedPort<Bottle>;
    port_TOTorques = new BufferedPort<Bottle>;
    port_HDTorques = new BufferedPort<Bottle>;
    port_external_wrench_RA = new BufferedPort<Vector>;
    port_external_wrench_LA = new BufferedPort<Vector>;
    port_external_wrench_RL = new BufferedPort<Vector>;
    port_external_wrench_LL = new BufferedPort<Vector>;
    port_external_wrench_RF = new BufferedPort<Vector>;
    port_external_wrench_LF = new BufferedPort<Vector>;
#ifdef TEST_LEG_SENSOR
    port_sensor_wrench_RL = new BufferedPort<Vector>;
    port_sensor_wrench_LL = new BufferedPort<Vector>;
    port_model_wrench_RL = new BufferedPort<Vector>;
    port_model_wrench_LL = new BufferedPort<Vector>;
#endif
    port_external_wrench_TO = new BufferedPort<Vector>;
    port_external_cartesian_wrench_RA = new BufferedPort<Vector>;
    port_external_cartesian_wrench_LA = new BufferedPort<Vector>;
    port_external_cartesian_wrench_RL = new BufferedPort<Vector>;
    port_external_cartesian_wrench_LL = new BufferedPort<Vector>;
	port_external_cartesian_wrench_RF = new BufferedPort<Vector>;
	port_external_cartesian_wrench_LF = new BufferedPort<Vector>;
    port_skin_contacts = new BufferedPort<skinContactList>;
    port_com_all      = new BufferedPort<Vector>;
    port_com_lb       = new BufferedPort<Vector>;
    port_com_ub       = new BufferedPort<Vector>;
    port_com_la       = new BufferedPort<Vector>;
    port_com_ra       = new BufferedPort<Vector>;
    port_com_ll       = new BufferedPort<Vector>;
    port_com_rl       = new BufferedPort<Vector>;
    port_com_to       = new BufferedPort<Vector>;
    port_com_hd       = new BufferedPort<Vector>;
    port_com_all_foot = new BufferedPort<Vector>;
    port_monitor = new BufferedPort<Vector>;
    port_contacts = new BufferedPort<skinContactList>;
    port_dumpvel = new BufferedPort<Vector>;
    port_external_ft_arm_left = new BufferedPort<Vector>;
    port_external_ft_arm_right = new BufferedPort<Vector>;
    port_external_ft_leg_left = new BufferedPort<Vector>;
    port_external_ft_leg_right = new BufferedPort<Vector>;
    port_COM_vel = new BufferedPort<Vector>;
    port_COM_Jacobian = new BufferedPort<Matrix>;
    port_all_velocities = new BufferedPort<Vector>;
    port_all_positions = new BufferedPort<Vector>;
    port_root_position_mat = new BufferedPort<Matrix>;
    port_root_position_vec = new BufferedPort<Vector>;

    port_inertial_thread->open(string("/"+local_name+"/inertial:i").c_str());
    port_ft_arm_left->open(string("/"+local_name+"/left_arm/FT:i").c_str());
    port_ft_arm_right->open(string("/"+local_name+"/right_arm/FT:i").c_str());
    port_ft_leg_left->open(string("/"+local_name+"/left_leg/FT:i").c_str());
    port_ft_leg_right->open(string("/"+local_name+"/right_leg/FT:i").c_str());
    port_ft_foot_left->open(string("/"+local_name+"/left_foot/FT:i").c_str());
    port_ft_foot_right->open(string("/"+local_name+"/right_foot/FT:i").c_str());
    port_RATorques->open(string("/"+local_name+"/right_arm/Torques:o").c_str());
    port_LATorques->open(string("/"+local_name+"/left_arm/Torques:o").c_str());
    port_RLTorques->open(string("/"+local_name+"/right_leg/Torques:o").c_str());
    port_LLTorques->open(string("/"+local_name+"/left_leg/Torques:o").c_str());
    port_RWTorques->open(string("/"+local_name+"/right_wrist/Torques:o").c_str());
    port_LWTorques->open(string("/"+local_name+"/left_wrist/Torques:o").c_str());
    port_TOTorques->open(string("/"+local_name+"/torso/Torques:o").c_str());
    port_HDTorques->open(string("/"+local_name+"/head/Torques:o").c_str());
    port_external_wrench_RA->open(string("/"+local_name+"/right_arm/endEffectorWrench:o").c_str()); 
    port_external_wrench_LA->open(string("/"+local_name+"/left_arm/endEffectorWrench:o").c_str()); 
    port_external_wrench_RL->open(string("/"+local_name+"/right_leg/endEffectorWrench:o").c_str()); 
    port_external_wrench_LL->open(string("/"+local_name+"/left_leg/endEffectorWrench:o").c_str()); 
    port_external_wrench_RF->open(string("/"+local_name+"/right_foot/endEffectorWrench:o").c_str()); 
    port_external_wrench_LF->open(string("/"+local_name+"/left_foot/endEffectorWrench:o").c_str()); 
#ifdef TEST_LEG_SENSOR
    port_sensor_wrench_RL->open(string("/"+local_name+"/right_leg/sensorWrench:o").c_str()); 
    port_sensor_wrench_LL->open(string("/"+local_name+"/left_leg/sensorWrench:o").c_str()); 
    port_model_wrench_RL->open(string("/"+local_name+"/right_leg/modelWrench:o").c_str()); 
    port_model_wrench_LL->open(string("/"+local_name+"/left_leg/modelWrench:o").c_str()); 
#endif
    port_external_cartesian_wrench_RA->open(string("/"+local_name+"/right_arm/cartesianEndEffectorWrench:o").c_str()); 
    port_external_cartesian_wrench_LA->open(string("/"+local_name+"/left_arm/cartesianEndEffectorWrench:o").c_str()); 
    port_external_cartesian_wrench_RL->open(string("/"+local_name+"/right_leg/cartesianEndEffectorWrench:o").c_str()); 
    port_external_cartesian_wrench_LL->open(string("/"+local_name+"/left_leg/cartesianEndEffectorWrench:o").c_str()); 
    port_external_cartesian_wrench_RF->open(string("/"+local_name+"/right_foot/cartesianEndEffectorWrench:o").c_str()); 
    port_external_cartesian_wrench_LF->open(string("/"+local_name+"/left_foot/cartesianEndEffectorWrench:o").c_str()); 
    port_external_wrench_TO->open(string("/"+local_name+"/torso/Wrench:o").c_str());
    port_com_all->open(string("/"+local_name+"/com:o").c_str());
    port_com_lb->open (string("/"+local_name+"/lower_body/com:o").c_str());
    port_com_ub->open (string("/"+local_name+"/upper_body/com:o").c_str());
    port_com_la ->open(string("/"+local_name+"/left_arm/com:o").c_str());
    port_com_ra ->open(string("/"+local_name+"/right_arm/com:o").c_str());
    port_com_ll ->open(string("/"+local_name+"/left_leg/com:o").c_str());
    port_com_rl ->open(string("/"+local_name+"/right_leg/com:o").c_str());
    port_com_hd ->open(string("/"+local_name+"/head/com:o").c_str());
    port_com_to ->open(string("/"+local_name+"/torso/com:o").c_str());
    port_com_all_foot->open(string("/"+local_name+"/com_foot:o").c_str());
    port_skin_contacts->open(string("/"+local_name+"/skin_contacts:i").c_str());
    port_monitor->open(string("/"+local_name+"/monitor:o").c_str());
    port_contacts->open(string("/"+local_name+"/contacts:o").c_str());
    port_dumpvel->open(string("/"+local_name+"/va:o").c_str());
    port_external_ft_arm_left->open(string("/"+local_name+"/left_arm/ext_ft_sens:o").c_str());
    port_external_ft_arm_right->open(string("/"+local_name+"/right_arm/ext_ft_sens:o").c_str());
    port_external_ft_leg_left->open(string("/"+local_name+"/left_leg/ext_ft_sens:o").c_str());
    port_external_ft_leg_right->open(string("/"+local_name+"/right_leg/ext_ft_sens:o").c_str());
    port_COM_vel->open(string("/"+local_name+"/com_vel:o").c_str());
    port_COM_Jacobian->open(string("/"+local_name+"/com_jacobian:o").c_str());
    port_all_velocities->open(string("/"+local_name+"/all_velocities:o").c_str());
    port_all_positions->open(string("/"+local_name+"/all_positions:o").c_str());
    port_root_position_mat->open(string("/"+local_name+"/root_position_mat:o").c_str());
    port_root_position_vec->open(string("/"+local_name+"/root_position_vec:o").c_str());

    if (autoconnect)
    {
        //from iCub to wholeBodyDynamics
        Network::connect(string("/"+local_name+"/filtered/inertial:o").c_str(),string("/"+local_name+"/inertial:i").c_str(),"tcp",false);			
        Network::connect(string("/"+robot_name+"/inertial").c_str(),           string("/"+local_name+"/unfiltered/inertial:i").c_str(),"tcp",false);
        Network::connect(string("/"+robot_name+"/left_arm/analog:o").c_str(),  string("/"+local_name+"/left_arm/FT:i").c_str(),"tcp",false);
        Network::connect(string("/"+robot_name+"/right_arm/analog:o").c_str(), string("/"+local_name+"/right_arm/FT:i").c_str(),"tcp",false);
        Network::connect(string("/"+robot_name+"/left_leg/analog:o").c_str(),  string("/"+local_name+"/left_leg/FT:i").c_str(),"tcp",false);
        Network::connect(string("/"+robot_name+"/right_leg/analog:o").c_str(), string("/"+local_name+"/right_leg/FT:i").c_str(),"tcp",false);
        Network::connect(string("/"+robot_name+"/left_foot/analog:o").c_str(),  string("/"+local_name+"/left_foot/FT:i").c_str(),"tcp",false);
        Network::connect(string("/"+robot_name+"/right_foot/analog:o").c_str(), string("/"+local_name+"/right_foot/FT:i").c_str(),"tcp",false);  
        //from wholeBodyDynamics to iCub (mandatory)
        Network::connect(string("/"+local_name+"/left_arm/Torques:o").c_str(), string("/"+robot_name+"/joint_vsens/left_arm:i").c_str(),"tcp",false);
        Network::connect(string("/"+local_name+"/right_arm/Torques:o").c_str(),string("/"+robot_name+"/joint_vsens/right_arm:i").c_str(),"tcp",false);
        Network::connect(string("/"+local_name+"/left_leg/Torques:o").c_str(), string("/"+robot_name+"/joint_vsens/left_leg:i").c_str(),"tcp",false);
        Network::connect(string("/"+local_name+"/right_leg/Torques:o").c_str(),string("/"+robot_name+"/joint_vsens/right_leg:i").c_str(),"tcp",false);
        Network::connect(string("/"+local_name+"/torso/Torques:o").c_str(),    string("/"+robot_name+"/joint_vsens/torso:i").c_str(),"tcp",false);
        //from wholeBodyDynamics to iCub (optional)
        if (Network::exists(string("/"+robot_name+"/joint_vsens/left_wrist:i").c_str()))
        Network::connect(string("/"+local_name+"/left_wrist/Torques:o").c_str(), string("/"+robot_name+"/joint_vsens/left_wrist:i").c_str(),"tcp",false);
        if (Network::exists(string("/"+robot_name+"/joint_vsens/right_wrist:i").c_str()))
        Network::connect(string("/"+local_name+"/right_wrist/Torques:o").c_str(),string("/"+robot_name+"/joint_vsens/right_wrist:i").c_str(),"tcp",false);
    }

    //---------------------DEVICES--------------------------//
    if (ddAL) {ddAL->view(iencs_arm_left);  ddAL->view(iint_arm_left);  ddAL->view(icmd_arm_left);}
    if (ddAR) {ddAR->view(iencs_arm_right); ddAR->view(iint_arm_right); ddAR->view(icmd_arm_right);}
    if (ddH)  {ddH->view(iencs_head);       ddH ->view(iint_head);       ddH ->view(icmd_head);}
    if (ddLL) {ddLL->view(iencs_leg_left);  ddLL->view(iint_leg_left);  ddLL->view(icmd_leg_left);}
    if (ddLR) {ddLR->view(iencs_leg_right); ddLR->view(iint_leg_right); ddLR->view(icmd_leg_right);}
    if (ddT)  {ddT->view(iencs_torso);      ddT ->view(iint_torso);      ddT ->view(icmd_torso);}

    linEstUp =new AWLinEstimator(16,1.0);
    quadEstUp=new AWQuadEstimator(25,1.0);
    linEstLow =new AWLinEstimator(16,1.0);
    quadEstLow=new AWQuadEstimator(25,1.0);
    InertialEst = new AWLinEstimator(16,1.0);

    //-----------parts INIT VARIABLES----------------//
    init_upper();
    init_lower();

    //-----------CARTESIAN INIT VARIABLES----------------//
    Fend.resize(3,0.0);
    Muend.resize(3,0.0);
    F_ext_up.resize(6,3);
    F_ext_up = 0.0;
    F_ext_low.resize(6,3);
    F_ext_low = 0.0;
    F_ext_left_arm.resize(6,0.0);
    F_ext_right_arm.resize(6,0.0);
    F_ext_cartesian_left_arm.resize(6,0.0);
    F_ext_cartesian_right_arm.resize(6,0.0);
    F_ext_left_leg.resize(6,0.0);
    F_ext_right_leg.resize(6,0.0); 
    F_ext_cartesian_left_leg.resize(6,0.0);
    F_ext_cartesian_right_leg.resize(6,0.0);
    F_ext_left_foot.resize(6,0.0);
    F_ext_right_foot.resize(6,0.0); 
    F_ext_cartesian_left_foot.resize(6,0.0);
    F_ext_cartesian_right_foot.resize(6,0.0);
    com_jac.resize(6,32);

}

bool inverseDynamics::threadInit()
{
    fprintf(stderr,"threadInit: waiting for port connections... \n\n");
    if (!dummy_ft)
    {
        Vector *dummy = port_inertial_thread->read(true); //blocking call: waits for ports connection
    }

    // N trials to get a more accurate estimation
    for(size_t i=0; i<status_queue_size; i++)
    {
        //read joints and ft sensor
        bool ret = readAndUpdate(true,true);
        if (ret == false)
        {
            printf("A problem occured during the initial readAndUpdate(), stopping... \n");
            thread_status = STATUS_DISCONNECTED;
            return false;
        }
    }

    // the queue previous_status now contains status_queue_size elements, and we can calibrate
    calibrateOffset();

    thread_status = STATUS_OK;
    return true;
}

bool iCubStatus::checkIcubNotMoving()
{
    bool ret = true;
    for (size_t i=0; i<this->all_dq_low.size(); i++)
        if (fabs(all_dq_low[i])>0.7)
            {
                ret = false;
                //fprintf(stderr,"%d ",i);
            }
    //fprintf(stderr,"\n");

    for (size_t i=0; i<this->all_dq_up.size(); i++)
        if (fabs(all_dq_up[i])>0.7)
            {
                ret = false;
                //fprintf(stderr,"%d ",i);
            }
    //fprintf(stderr,"\n");

    return ret;
}

void iCubStatus::dump (FILE* f)
{
    fprintf (f, "%f   ", timestamp);
    fprintf (f, "%s   ", all_q_up.toString().c_str());
    fprintf (f, "%s   ", all_dq_up.toString().c_str());
    fprintf (f, "%s   ", all_d2q_up.toString().c_str());
    fprintf (f, "%s   ", all_q_low.toString().c_str());
    fprintf (f, "%s   ", all_dq_low.toString().c_str());
    fprintf (f, "%s   ", all_d2q_low.toString().c_str());
    fprintf (f, "%s   ", ft_arm_left.toString().c_str());
    fprintf (f, "%s   ", ft_arm_right.toString().c_str());
    fprintf (f, "%s   ", ft_leg_left.toString().c_str());
    fprintf (f, "%s   ", ft_leg_right.toString().c_str());
    fprintf (f, "%s   ", inertial_w0.toString().c_str());
    fprintf (f, "%s   ", inertial_dw0.toString().c_str());
    fprintf (f, "%s \n", inertial_d2p0.toString().c_str());
}

void inverseDynamics::run()
{
    timestamp.update();

    thread_status = STATUS_OK;
    static int delay_check=0;
    if(readAndUpdate(false) == false)
    {
        delay_check++;
        printf ("network delays detected (%d/10)\n", delay_check);
        if (delay_check>=10)
        {
            printf ("inverseDynamics thread lost connection with iCubInterface.\n");
            thread_status = STATUS_DISCONNECTED;
        }
    }
    else
    {
        delay_check = 0;
    }

    //remove the offset from the FT sensors measurements
    F_LArm  = -1.0 * (current_status.ft_arm_left-Offset_LArm);
    F_RArm  = -1.0 * (current_status.ft_arm_right-Offset_RArm);
    F_LLeg  = -1.0 * (current_status.ft_leg_left-Offset_LLeg);
    F_RLeg  = -1.0 * (current_status.ft_leg_right-Offset_RLeg);
    F_LFoot = -1.0 * (current_status.ft_foot_left-Offset_LFoot);
    F_RFoot = -1.0 * (current_status.ft_foot_right-Offset_RFoot);
   //F_LFoot = 1.0 * (current_status.ft_foot_left);
   //F_RFoot = 1.0 * (current_status.ft_foot_right);

    //check if iCub is currently moving. If not, put the current iCub positions in the status queue
    current_status.iCub_not_moving = current_status.checkIcubNotMoving();
    if (current_status.iCub_not_moving == true)
    {
        not_moving_status.push_front(current_status);
        if (not_moving_status.size()>status_queue_size) 
            {
                not_moving_status.pop_back();
                if (auto_drift_comp) fprintf (stderr,"drift_comp: buffer full\n"); //@@@DEBUG
            }
    }
    else
    {
        //efficient way to clear the queue
        not_moving_status.clear();
        if (auto_drift_comp) fprintf (stderr,"drift_comp: clearing buffer\n");  //@@@DEBUG
    }

    // THIS BLOCK SHOULD BE NOW DEPRECATED
    if (w0_dw0_enabled == false)
    {
        //if w0 and dw0 are too noisy, you can disable them using 'no_w0_dw0' option
        current_status.inertial_w0.zero();
        current_status.inertial_dw0.zero();
    }

    Vector F_up(6, 0.0);
    icub->upperTorso->setInertialMeasure(current_status.inertial_w0,current_status.inertial_dw0,current_status.inertial_d2p0);
    icub->upperTorso->setSensorMeasurement(F_RArm,F_LArm,F_up);

//#define DEBUG_PERFORMANCE
#ifdef DEBUG_PERFORMANCE
    static double meanTime = 0.0;
    static double startTime = 0;
    startTime = Time::now();
#endif
    icub->upperTorso->solveKinematics();
    addSkinContacts();
    icub->upperTorso->solveWrench();
#ifdef DEBUG_PERFORMANCE
    meanTime += Time::now()-startTime;
    printf("Mean uppertorso NE time: %.4f\n", meanTime/getIterations());
#endif

//#define DEBUG_KINEMATICS
#ifdef DEBUG_KINEMATICS
    // DEBUG ONLY
    fprintf (stderr,"\nHEAD: %s \n", d2p0.toString().c_str());

    fprintf (stderr,"UPTORSO: %s \n", icub->upperTorso->getTorsoLinAcc().toString().c_str());
#endif

    icub->attachLowerTorso(F_RLeg,F_LLeg);
    icub->lowerTorso->solveKinematics();
    icub->lowerTorso->solveWrench();

//#define DEBUG_KINEMATICS
#ifdef DEBUG_KINEMATICS
    //DEBUG ONLY
    fprintf (stderr,"LOWTORSO->UP: %s *** %s *** %s\n",
            icub->lowerTorso->up->getLinAcc(0).toString().c_str(),
            icub->lowerTorso->up->getLinAcc(1).toString().c_str(),
            cub->lowerTorso->up->getLinAcc(2).toString().c_str());

    fprintf (stderr,"LOWTORSO: %s \n", icub->lowerTorso->getLinAcc().toString().c_str());

    fprintf (stderr,"LOWTORSO->RI: %s *** %s *** %s *** %s\n",
            icub->lowerTorso->right->getLinAcc(0).toString().c_str(),
            icub->lowerTorso->right->getLinAcc(1).toString().c_str(),
            icub->lowerTorso->right->getLinAcc(2).toString().c_str(),
            icub->lowerTorso->right->getLinAcc(3).toString().c_str());

    fprintf (stderr,"LOWTORSO->LE: %s *** %s *** %s *** %s\n",
            icub->lowerTorso->left->getLinAcc(0).toString().c_str(),
            icub->lowerTorso->left->getLinAcc(1).toString().c_str(),
            icub->lowerTorso->left->getLinAcc(2).toString().c_str(),
            icub->lowerTorso->left->getLinAcc(3).toString().c_str());
#endif

//#define DEBUG_WRENCH
#ifdef  DEBUG_WRENCH

    fprintf (stderr,"LOWTORSO->UP: %s *** %s \n",
            icub->lowerTorso->up->getForce(0).toString().c_str(),
            icub->lowerTorso->up->getMoment(0).toString().c_str());

    fprintf (stderr,"LOWTORSO->RO: %s *** %s \n",
            icub->lowerTorso->up->getForce(2).toString().c_str(),
            icub->lowerTorso->up->getMoment(2).toString().c_str());
#endif

    Vector LATorques = icub->upperTorso->getTorques("left_arm");
    Vector RATorques = icub->upperTorso->getTorques("right_arm");
    Vector HDtmp     = icub->upperTorso->getTorques("head");

    Vector LLTorques = icub->lowerTorso->getTorques("left_leg");
    Vector RLTorques = icub->lowerTorso->getTorques("right_leg");
    Vector TOtmp     = icub->lowerTorso->getTorques("torso");
    Vector TOTorques(3);
    Vector HDTorques(3);

    //head torques
    HDTorques[0] = HDtmp [0];
    HDTorques[1] = HDtmp [1];
    HDTorques[2] = HDtmp [2];
    //torso torques
    TOTorques[0] = TOtmp [2];
    TOTorques[1] = TOtmp [1];
    TOTorques[2] = TOtmp [0];

//#define DEBUG_TORQUES
#ifdef  DEBUG_TORQUES
    fprintf (stderr,"TORQUES:     %s ***  \n\n", TOTorques.toString().c_str());
#endif

    writeTorque(RATorques, 1, port_RATorques); //arm
    writeTorque(LATorques, 1, port_LATorques); //arm
    writeTorque(TOTorques, 4, port_TOTorques); //torso
    writeTorque(HDTorques, 0, port_HDTorques); //head
//  fprintf (stderr,"TORSO: %s \n",TOTorques.toString().c_str());
/*  fprintf (stderr,"TORSO: %s %s %s \n",TOTorques.toString().c_str(),
        LLTorques.toString().c_str(),
         RLTorques.toString().c_str());
*/

    if (ddLR) writeTorque(RLTorques, 2, port_RLTorques); //leg
    if (ddLL) writeTorque(LLTorques, 2, port_LLTorques); //leg
    writeTorque(RATorques, 3, port_RWTorques); //wrist
    writeTorque(LATorques, 3, port_LWTorques); //wrist

    Vector com_all(7), com_ll(7), com_rl(7), com_la(7),com_ra(7), com_hd(7), com_to(7), com_lb(7), com_ub(7);
    double mass_all  , mass_ll  , mass_rl  , mass_la  ,mass_ra  , mass_hd,   mass_to, mass_lb, mass_ub;
    Vector com_v; com_v.resize(3); com_v.zero();
    Vector all_dq; all_dq.resize(32,1); all_dq.zero();
    Vector all_q;  all_q.resize(32,1);  all_q.zero();

    // For balancing purposes
    yarp::sig::Vector com_all_foot; com_all_foot.resize(3); com_all_foot.zero();
    yarp::sig::Matrix rTf; rTf.resize(4,4); rTf.zero();
    yarp::sig::Matrix fTr; fTr.resize(4,4); fTr.zero();
    yarp::sig::Matrix lastRotTrans; lastRotTrans.resize(4,4); lastRotTrans.zero();
    lastRotTrans(2,0)=lastRotTrans(3,3)=lastRotTrans(0,2)=1;
    lastRotTrans(1,1)=-1;

    rTf = icub->lowerTorso->right->getH();

    rTf = (icub->lowerTorso->getHRight()) * rTf*lastRotTrans;           //Until the world reference frame
    fTr.setSubmatrix(rTf.submatrix(0,2,0,2).transposed(), 0, 0);         //CHECKED
    fTr.setSubcol(-1*fTr.submatrix(0,2,0,2)*rTf.subcol(0,3,3), 0, 3);    //CHECKED    
    fTr(3,3) = 1;

    //filling distance from root projection onto the floor to the right foot. 
    lastRotTrans(1,3)=-rTf(1,3);
    lastRotTrans(2,3)= -rTf(0,3); 


    if (com_enabled)
    {
        icub->computeCOM();

        if (com_vel_enabled)
        {
            icub->EXPERIMENTAL_computeCOMjacobian();
            icub->EXPERIMENTAL_getCOMjacobian(BODY_PART_ALL,com_jac);
            icub->EXPERIMENTAL_getCOMvelocity(BODY_PART_ALL,com_v,all_dq);
            icub->getAllPositions(all_q);
        }

        icub->getCOM(BODY_PART_ALL,     com_all, mass_all);
        icub->getCOM(LOWER_BODY_PARTS,  com_lb,  mass_lb);
        icub->getCOM(UPPER_BODY_PARTS,  com_ub,  mass_ub);
        icub->getCOM(LEFT_LEG,          com_ll,  mass_ll);
        icub->getCOM(RIGHT_LEG,         com_rl,  mass_rl);
        icub->getCOM(LEFT_ARM,          com_la,  mass_la);
        icub->getCOM(RIGHT_ARM,         com_ra,  mass_ra);
        icub->getCOM(HEAD,              com_hd,  mass_hd);
        icub->getCOM(TORSO,             com_to,  mass_to);
        com_lb.push_back(mass_lb);
        com_ub.push_back(mass_ub);
        com_all.push_back(mass_all);
        com_ll.push_back (mass_ll);
        com_rl.push_back (mass_rl);
        com_la.push_back (mass_la);
        com_ra.push_back (mass_ra);
        com_hd.push_back (mass_hd);
        com_to.push_back (mass_to);

        if (com_vel_enabled)
        {
            com_all.push_back(com_v[0]);
            com_all.push_back(com_v[1]);
            com_all.push_back(com_v[2]);
        }
        else
        {
            com_all.push_back(0);
            com_all.push_back(0);
            com_all.push_back(0);
        }

        #ifdef MEASURE_FROM_FOOT
        com_all_foot.setSubvector(0,com_all.subVector(0,2));
        com_all_foot.push_back(1);
        com_all_foot = fTr*com_all_foot;
        #endif

    }
    else
    {
        mass_all=mass_ll=mass_rl=mass_la=mass_ra=mass_hd=mass_to=0.0;
        com_all.zero(); com_ll.zero(); com_rl.zero(); com_la.zero(); com_ra.zero(); com_hd.zero(); com_to.zero();
    }

    // DYN/SKIN CONTACTS
    dynContacts = icub->upperTorso->leftSensor->getContactList();
    const dynContactList& contactListR = icub->upperTorso->rightSensor->getContactList();
    dynContacts.insert(dynContacts.begin(), contactListR.begin(), contactListR.end());
    // for each dynContact find the related skinContact (if any) and set the wrench in it
    unsigned long cId;
    bool contactFound=false;
    for(unsigned int i=0; i<dynContacts.size(); i++)
    {
        cId = dynContacts[i].getId();
        for(unsigned int j=0; j<skinContacts.size(); j++)
        {
            if(cId == skinContacts[j].getId())
            {
                skinContacts[j].setForceMoment( dynContacts[i].getForceMoment() );
                contactFound = true;
                j = skinContacts.size();    // break from the inside for loop
            }
        }
        // if there is no associated skin contact, create one
        if(!contactFound)
            skinContacts.push_back(skinContact(dynContacts[i]));
        contactFound = false;
    }

	//*********************************************** add the legs contacts JUST TEMP FIX!! *******************
	skinContact left_leg_contact;
	left_leg_contact.setLinkNumber(5);
	left_leg_contact.setBodyPart(iCub::skinDynLib::LEFT_LEG);
	left_leg_contact.setForceMoment(F_ext_left_leg);
	skinContact right_leg_contact;
	right_leg_contact.setLinkNumber(5);
	right_leg_contact.setBodyPart(iCub::skinDynLib::RIGHT_LEG);
	right_leg_contact.setForceMoment(F_ext_right_leg);
	
	if (!add_legs_once)
	{
		skinContacts.push_back(left_leg_contact);
		skinContacts.push_back(right_leg_contact);
		add_legs_once = true;
	}
	
	bool skin_lleg_found = false;
	bool skin_rleg_found = false;
	for(unsigned int j=0; j<skinContacts.size(); j++)
	{
		if (skinContacts[j].getBodyPart()==iCub::skinDynLib::LEFT_LEG)
		{
			skinContacts[j].setForceMoment(F_ext_left_leg);
			skin_lleg_found = true;
		}
		if (skinContacts[j].getBodyPart()==iCub::skinDynLib::RIGHT_LEG)
		{
			skinContacts[j].setForceMoment(F_ext_right_leg);
			skin_rleg_found = true;
		}
	}
	if (!skin_rleg_found) {skinContacts.push_back(right_leg_contact);} 
    if (!skin_lleg_found) {skinContacts.push_back(left_leg_contact);} 
    
	//*********************************************** add the legs contacts JUST TEMP FIX!! *******************

    F_ext_cartesian_left_arm = F_ext_cartesian_right_arm = zeros(6);
    F_ext_cartesian_left_leg = F_ext_cartesian_right_leg = zeros(6);
    F_ext_left_arm  = icub->upperTorso->leftSensor->getForceMomentEndEff();
    F_ext_right_arm = icub->upperTorso->rightSensor->getForceMomentEndEff();
    F_ext_left_leg  = icub->lowerTorso->leftSensor->getForceMomentEndEff();
    F_ext_right_leg = icub->lowerTorso->rightSensor->getForceMomentEndEff();

    // EXTERNAL DYNAMICS AT THE F/T SENSORS
    Matrix F_sensor_up = icub_sens->upperTorso->estimateSensorsWrench(zeros(6,3));
    F_ext_sens_right_arm = F_RArm - F_sensor_up.getCol(0);  // measured wrench - internal wrench = external wrench
    F_ext_sens_left_arm = F_LArm - F_sensor_up.getCol(1);   // measured wrench - internal wrench = external wrench

#ifdef TEST_LEG_SENSOR
    setUpperMeasure(true);
    setLowerMeasure(true);
#endif

    Matrix F_sensor_low = icub_sens->lowerTorso->estimateSensorsWrench(F_ext_low,false);
    F_ext_sens_right_leg = F_RLeg - F_sensor_low.getCol(0); // measured wrench - internal wrench = external wrench
    F_ext_sens_left_leg = F_LLeg - F_sensor_low.getCol(1);  // measured wrench - internal wrench = external wrench

#ifdef TEST_LEG_SENSOR
    F_mdl_right_leg = F_sensor_low.getCol(0);
    F_mdl_left_leg  = F_sensor_low.getCol(1);
    F_sns_right_leg = F_RLeg;
    F_sns_left_leg  = F_LLeg;
#endif

    yarp::sig::Matrix ht   = icub->upperTorso->getHUp()    * icub->upperTorso->up->getH();
    yarp::sig::Matrix ahl  = ht * icub->upperTorso->getHLeft()  * icub->upperTorso->left->getH();
    yarp::sig::Matrix ahr  = ht * icub->upperTorso->getHRight() * icub->upperTorso->right->getH();
    yarp::sig::Matrix lhl  = icub->lowerTorso->getHLeft()  * icub->lowerTorso->left->getH();
    yarp::sig::Matrix lhr  = icub->lowerTorso->getHRight() * icub->lowerTorso->right->getH();

    yarp::sig::Vector tmp1,tmp2;
    tmp1 = F_ext_left_arm.subVector(0,2); tmp1.push_back(0.0); tmp1 = ahl * tmp1;
    tmp2 = F_ext_left_arm.subVector(3,5); tmp2.push_back(0.0); tmp2 = ahl * tmp2;
    for (int i=0; i<3; i++) F_ext_cartesian_left_arm[i] = tmp1[i];
    for (int i=3; i<6; i++) F_ext_cartesian_left_arm[i] = tmp2[i-3];
    double n1=norm(F_ext_cartesian_left_arm.subVector(0,2));
    double n2=norm(F_ext_cartesian_left_arm.subVector(3,5));
    F_ext_cartesian_left_arm.push_back(n1);
    F_ext_cartesian_left_arm.push_back(n2);

    tmp1 = F_ext_right_arm.subVector(0,2); tmp1.push_back(0.0); tmp1 = ahr * tmp1;
    tmp2 = F_ext_right_arm.subVector(3,5); tmp2.push_back(0.0); tmp2 = ahr * tmp2;
    for (int i=0; i<3; i++) F_ext_cartesian_right_arm[i] = tmp1[i];
    for (int i=3; i<6; i++) F_ext_cartesian_right_arm[i] = tmp2[i-3];
    n1=norm(F_ext_cartesian_right_arm.subVector(0,2));
    n2=norm(F_ext_cartesian_right_arm.subVector(3,5));
    F_ext_cartesian_right_arm.push_back(n1);
    F_ext_cartesian_right_arm.push_back(n2);

    tmp1 = F_ext_left_leg.subVector(0,2); tmp1.push_back(0.0); tmp1 = lhl * tmp1;
    tmp2 = F_ext_left_leg.subVector(3,5); tmp2.push_back(0.0); tmp2 = lhl * tmp2;
    for (int i=0; i<3; i++) F_ext_cartesian_left_leg[i] = tmp1[i];
    for (int i=3; i<6; i++) F_ext_cartesian_left_leg[i] = tmp2[i-3];

    tmp1 = F_ext_right_leg.subVector(0,2); tmp1.push_back(0.0); tmp1 = lhr * tmp1;
    tmp2 = F_ext_right_leg.subVector(3,5); tmp2.push_back(0.0); tmp2 = lhr * tmp2;
    for (int i=0; i<3; i++) F_ext_cartesian_right_leg[i] = tmp1[i];
    for (int i=3; i<6; i++) F_ext_cartesian_right_leg[i] = tmp2[i-3];

    //computation of the root
    yarp::sig::Vector angles (3);
    angles[0] = 0;
    angles[1] = -90/180.0*M_PI;
    angles[2] = 0;
    yarp::sig::Matrix r1 = iCub::ctrl::euler2dcm(angles);
    yarp::sig::Matrix ilhl = iCub::ctrl::SE3inv(lhl);
    yarp::sig::Matrix foot_root_mat = r1*ilhl;
    
    yarp::sig::Vector foot_tmp = iCub::ctrl::dcm2rpy(foot_root_mat);
    //printf ("before\n %s\n", foot_root_mat.toString().c_str());
    yarp::sig::Vector foot_root_vec (6,0.0); 
    foot_root_vec[3] = foot_root_mat[0][3]*1000;
    foot_root_vec[4] = foot_root_mat[1][3]*1000;
    foot_root_vec[5] = foot_root_mat[2][3]*1000;
    foot_root_vec[0] = foot_tmp[0]*180.0/M_PI;
    foot_root_vec[1] = foot_tmp[1]*180.0/M_PI;
    foot_root_vec[2] = foot_tmp[2]*180.0/M_PI;
    //yarp::sig::Matrix test = iCub::ctrl::rpy2dcm(foot_tmp);
    //printf ("afer\n %s\n", test.toString().c_str());
    //printf ("angles %+.2f %+.2f %+.2f\n", foot_tmp[0]*180.0/M_PI, foot_tmp[1]*180.0/M_PI, foot_tmp[2]*180.0/M_PI);

    //computation for the foot
    Matrix foot_hn(4,4); foot_hn.zero();
    foot_hn(0,2)=1;foot_hn(0,3)=-7.75;
    foot_hn(1,1)=-1;
    foot_hn(2,0)=-1;
    foot_hn(3,3)=1;

    tmp1 = F_LFoot.subVector(0,2); tmp1.push_back(0.0); tmp1 = foot_hn * tmp1;
    tmp2 = F_LFoot.subVector(3,5); tmp2.push_back(0.0); tmp2 = foot_hn * tmp2;
    for (int i=0; i<3; i++) F_ext_left_foot[i] = tmp1[i];
    for (int i=3; i<6; i++) F_ext_left_foot[i] = tmp2[i-3];

    tmp1 = F_RFoot.subVector(0,2); tmp1.push_back(0.0); tmp1 = foot_hn * tmp1;
    tmp2 = F_RFoot.subVector(3,5); tmp2.push_back(0.0); tmp2 = foot_hn * tmp2;
    for (int i=0; i<3; i++) F_ext_right_foot[i] = tmp1[i];
    for (int i=3; i<6; i++) F_ext_right_foot[i] = tmp2[i-3];

    tmp1 = F_ext_left_foot.subVector(0,2); tmp1.push_back(0.0); tmp1 = lhl * tmp1;
    tmp2 = F_ext_left_foot.subVector(3,5); tmp2.push_back(0.0); tmp2 = lhl * tmp2;
    for (int i=0; i<3; i++) F_ext_cartesian_left_foot[i] = tmp1[i];
    for (int i=3; i<6; i++) F_ext_cartesian_left_foot[i] = tmp2[i-3];

    tmp1 = F_ext_right_foot.subVector(0,2); tmp1.push_back(0.0); tmp1 = lhr * tmp1;
    tmp2 = F_ext_right_foot.subVector(3,5); tmp2.push_back(0.0); tmp2 = lhr * tmp2;
    for (int i=0; i<3; i++) F_ext_cartesian_right_foot[i] = tmp1[i];
    for (int i=3; i<6; i++) F_ext_cartesian_right_foot[i] = tmp2[i-3];

    // *** MONITOR DATA ***
    //sendMonitorData();

    // *** DUMP VEL DATA ***
    //sendVelAccData();

    if (com_vel_enabled)
    {
        // com_jac = M_PI/180.0 * (com_jac);
        // all_dq  = 180.0/M_PI * all_dq;
        // all_q  = 180.0/M_PI * all_q;
        broadcastData<Vector> (com_v, port_COM_vel);
        broadcastData<Vector> (all_dq, port_all_velocities);
        broadcastData<Vector> (all_q,  port_all_positions);
        broadcastData<Matrix> (com_jac, port_COM_Jacobian);
    }
    broadcastData<Vector> (com_all, port_com_all);
    broadcastData<Vector> (com_lb,  port_com_lb);
    broadcastData<Vector> (com_ub,  port_com_ub);
    broadcastData<Vector> (com_ll,  port_com_ll);
    broadcastData<Vector> (com_rl,  port_com_rl);
    broadcastData<Vector> (com_la,  port_com_la);
    broadcastData<Vector> (com_ra,  port_com_ra);
    broadcastData<Vector> (com_hd,  port_com_hd);
    broadcastData<Vector> (com_to,  port_com_to);
    broadcastData<Vector> (com_all_foot, port_com_all_foot);

    broadcastData<Vector> (F_up,                                    port_external_wrench_TO);
    broadcastData<Vector> (F_ext_right_arm,                         port_external_wrench_RA);
    broadcastData<Vector> (F_ext_left_arm,                          port_external_wrench_LA);
    broadcastData<Vector> (F_ext_cartesian_right_arm,               port_external_cartesian_wrench_RA);
    broadcastData<Vector> (F_ext_cartesian_left_arm,                port_external_cartesian_wrench_LA);
    broadcastData<Vector> (F_ext_right_leg,                         port_external_wrench_RL);
    broadcastData<Vector> (F_ext_left_leg,                          port_external_wrench_LL);
    broadcastData<Vector> (F_ext_right_foot,                        port_external_wrench_RF);
    broadcastData<Vector> (F_ext_left_foot,                         port_external_wrench_LF);
#ifdef TEST_LEG_SENSOR
    broadcastData<Vector> (F_sns_right_leg,                         port_sensor_wrench_RL);
    broadcastData<Vector> (F_sns_left_leg,                          port_sensor_wrench_LL);
    broadcastData<Vector> (F_mdl_right_leg,                         port_model_wrench_RL);
    broadcastData<Vector> (F_mdl_left_leg,                          port_model_wrench_LL);
#endif
    broadcastData<Vector> (F_ext_cartesian_right_leg,               port_external_cartesian_wrench_RL);
    broadcastData<Vector> (F_ext_cartesian_left_leg,                port_external_cartesian_wrench_LL);
    broadcastData<Vector> (F_ext_cartesian_right_foot,              port_external_cartesian_wrench_RF);
    broadcastData<Vector> (F_ext_cartesian_left_foot,               port_external_cartesian_wrench_LF);
    broadcastData<skinContactList>( skinContacts,                   port_contacts);
    broadcastData<Vector> (F_ext_sens_right_arm,                    port_external_ft_arm_right);
    broadcastData<Vector> (F_ext_sens_left_arm,                     port_external_ft_arm_left);
    broadcastData<Vector> (F_ext_sens_right_leg,                    port_external_ft_leg_right);
    broadcastData<Vector> (F_ext_sens_left_leg,                     port_external_ft_leg_left);

    broadcastData<Matrix> (foot_root_mat,                           port_root_position_mat);
    broadcastData<Vector> (foot_root_vec,                           port_root_position_vec);
}

void inverseDynamics::threadRelease()
{
    fprintf(stderr, "Closing the linear estimator\n");
    if(linEstUp)
    {
        delete linEstUp;
        linEstUp = 0;
    }
    if(linEstLow)
    {
        delete linEstLow;
        linEstLow = 0;
    }
    fprintf(stderr, "Closing the quadratic estimator\n");
    if(quadEstUp)
    {
        delete quadEstUp;
        quadEstUp = 0;
    }
    if(quadEstLow)
    {
        delete quadEstLow;
        quadEstLow = 0;
    }
    fprintf(stderr, "Closing the inertial estimator\n");
    if(InertialEst)
    {
        delete InertialEst;
        InertialEst = 0;
    }

    fprintf(stderr, "Closing RATorques port\n");
    closePort(port_RATorques);
    fprintf(stderr, "Closing LATorques port\n");
    closePort(port_LATorques);
    fprintf(stderr, "Closing RLTorques port\n");
    closePort(port_RLTorques);
    fprintf(stderr, "Closing LLTorques port\n");
    closePort(port_LLTorques);
    fprintf(stderr, "Closing RWTorques port\n");
    closePort(port_RWTorques);
    fprintf(stderr, "Closing LWTorques port\n");
    closePort(port_LWTorques);
    fprintf(stderr, "Closing TOTorques port\n");
    closePort(port_TOTorques);
    fprintf(stderr, "Closing HDTorques port\n");
    closePort(port_HDTorques);
    fprintf(stderr, "Closing external_wrench_RA port\n");
    closePort(port_external_wrench_RA);
    fprintf(stderr, "Closing external_wrench_LA port\n");
    closePort(port_external_wrench_LA);
#ifdef TEST_LEG_SENSOR
    closePort(port_sensor_wrench_RL);
    closePort(port_sensor_wrench_LL);
    closePort(port_model_wrench_RL);
    closePort(port_model_wrench_LL);
#endif
    fprintf(stderr, "Closing cartesian_external_wrench_RA port\n");
    closePort(port_external_cartesian_wrench_RA);
    fprintf(stderr, "Closing cartesian_external_wrench_LA port\n");
    closePort(port_external_cartesian_wrench_LA);
    fprintf(stderr, "Closing external_wrench_RL port\n");
    closePort(port_external_wrench_RL);
    fprintf(stderr, "Closing external_wrench_LL port\n");	
    closePort(port_external_wrench_LL);
    fprintf(stderr, "Closing cartesian_external_wrench_RL port\n");
    closePort(port_external_cartesian_wrench_RL);
    fprintf(stderr, "Closing cartesian_external_wrench_LL port\n");	
    closePort(port_external_cartesian_wrench_LL);
    fprintf(stderr, "Closing external_wrench_RF port\n");
    closePort(port_external_wrench_RF);
    fprintf(stderr, "Closing external_wrench_LF port\n");	
    closePort(port_external_wrench_LF);
    fprintf(stderr, "Closing cartesian_external_wrench_RF port\n");
    closePort(port_external_cartesian_wrench_RF);
    fprintf(stderr, "Closing cartesian_external_wrench_LF port\n");	
    closePort(port_external_cartesian_wrench_LF);
    fprintf(stderr, "Closing external_wrench_TO port\n");	
    closePort(port_external_wrench_TO);
    fprintf(stderr, "Closing COM ports\n");	
    closePort(port_com_all);
    closePort(port_com_lb);
    closePort(port_com_ub);
    closePort(port_com_ra);
    closePort(port_com_rl);
    closePort(port_com_la);
    closePort(port_com_ll);
    closePort(port_com_hd);
    closePort(port_com_to);
    closePort(port_com_all_foot);

    fprintf(stderr, "Closing inertial port\n");
    closePort(port_inertial_thread);
    fprintf(stderr, "Closing ft_arm_right port\n");
    closePort(port_ft_arm_right);
    fprintf(stderr, "Closing ft_arm_left port\n");
    closePort(port_ft_arm_left);
    fprintf(stderr, "Closing ft_leg_right port\n");
    closePort(port_ft_leg_right);
    fprintf(stderr, "Closing ft_leg_left port\n");
    closePort(port_ft_leg_left);
    fprintf(stderr, "Closing ft_foot_right port\n");
    closePort(port_ft_foot_right);
    fprintf(stderr, "Closing ft_foot_left port\n");
    closePort(port_ft_foot_left);
    fprintf(stderr, "Closing skin_contacts port\n");
    closePort(port_skin_contacts);
    fprintf(stderr, "Closing monitor port\n");
    closePort(port_monitor);
    fprintf(stderr, "Closing dump port\n");
    closePort(port_dumpvel);
    fprintf(stderr, "Closing contacts port\n");
    closePort(port_contacts);
    fprintf(stderr, "Closing external_ft_arm_left port\n");
    closePort(port_external_ft_arm_left);
    fprintf(stderr, "Closing external_ft_arm_right port\n");
    closePort(port_external_ft_arm_right);
    fprintf(stderr, "Closing external_ft_leg_left port\n");
    closePort(port_external_ft_leg_left);
    fprintf(stderr, "Closing external_ft_leg_right port\n");
    closePort(port_external_ft_leg_right);
    fprintf(stderr, "Close COM velocity port\n");
    closePort(port_COM_vel);
    fprintf(stderr, "Closing COM Jacobian port\n");
    closePort(port_COM_Jacobian);
    fprintf(stderr, "Closing All Velocities port\n");
    closePort(port_all_velocities);
    fprintf(stderr, "Closing All Positions port\n");
    closePort(port_all_positions);
    fprintf(stderr, "Closing Foot/Root port\n");
    closePort(port_root_position_mat);
    closePort(port_root_position_vec);

    if (icub)      {delete icub; icub=0;}
    if (icub_sens) {delete icub_sens; icub=0;}
}   

void inverseDynamics::closePort(Contactable *_port)
{
    if (_port)
    {
        _port->interrupt();
        _port->close();

        delete _port;
        _port = 0;
    }
}

template <class T> void inverseDynamics::broadcastData(T& _values, BufferedPort<T> *_port)
{
    if (_port && _port->getOutputCount()>0)
    {
        _port->setEnvelope(this->timestamp);
        _port->prepare()  = _values ;
        _port->write();
    }
}

void inverseDynamics::writeTorque(Vector _values, int _address, BufferedPort<Bottle> *_port)
{
    Bottle a;
    a.addInt(_address);
    for(size_t i=0;i<_values.length();i++)
        a.addDouble(_values(i));
    _port->prepare() = a;
    _port->write();
}

void inverseDynamics::calibrateOffset(calib_enum calib_code)
{
    fprintf(stderr,"calibrateOffset: starting calibration... \n");

    Offset_LArm.zero();
    Offset_RArm.zero();
    Offset_LLeg.zero();
    Offset_RLeg.zero();
    Offset_LFoot.zero();
    Offset_RFoot.zero();
    F_ext_up.zero();
    F_ext_low.zero();
    setUpperMeasure(true);
    setLowerMeasure(true);

    size_t Ntrials = previous_status.size();
    list<iCubStatus>::iterator it=previous_status.begin();

    icub_sens->upperTorso->setInertialMeasure(it->inertial_w0,it->inertial_dw0,it->inertial_d2p0);
    Matrix F_sensor_up = icub_sens->upperTorso->estimateSensorsWrench(F_ext_up,false);
    icub_sens->lowerTorso->setInertialMeasure(icub_sens->upperTorso->getTorsoAngVel(),icub_sens->upperTorso->getTorsoAngAcc(),icub_sens->upperTorso->getTorsoLinAcc());
    Matrix F_sensor_low = icub_sens->lowerTorso->estimateSensorsWrench(F_ext_low,false);

    for (it=previous_status.begin() ; it != previous_status.end(); it++ )
    {
        /*
        // TO BE VERIFIED IF USEFUL
        setZeroJntAngVelAcc();
        */

        F_iDyn_LArm  = -1.0 * F_sensor_up.getCol(1);
        F_iDyn_RArm = -1.0 * F_sensor_up.getCol(0);
        F_iDyn_LLeg  = -1.0 * F_sensor_low.getCol(1);
        F_iDyn_RLeg = -1.0 * F_sensor_low.getCol(0);
        F_iDyn_LFoot = Vector(6,0.0); //@@@ TO BE COMPLETED
        F_iDyn_RFoot = Vector(6,0.0); //@@@ TO BE COMPLETED

        Offset_LArm = Offset_LArm + (it->ft_arm_left-F_iDyn_LArm);
        Offset_RArm = Offset_RArm + (it->ft_arm_right-F_iDyn_RArm);
        Offset_LLeg = Offset_LLeg + (it->ft_leg_left-F_iDyn_LLeg);
        Offset_RLeg = Offset_RLeg + (it->ft_leg_right-F_iDyn_RLeg);
        Offset_LFoot = Offset_LFoot + (it->ft_foot_left-F_iDyn_LFoot);
        Offset_RFoot = Offset_RFoot + (it->ft_foot_right-F_iDyn_RFoot);
    }

    Offset_LArm  = 1.0/(double)Ntrials * Offset_LArm;
    Offset_RArm  = 1.0/(double)Ntrials * Offset_RArm;
    Offset_LLeg  = 1.0/(double)Ntrials * Offset_LLeg;
    Offset_RLeg  = 1.0/(double)Ntrials * Offset_RLeg;
    Offset_LFoot = 1.0/(double)Ntrials * Offset_LFoot;
    Offset_RFoot = 1.0/(double)Ntrials * Offset_RFoot;

    //printVector(Offset_LArm, "Offset left arm:");
    //printVector(Offset_RArm, "Offset right arm:");
    //printVector(Offset_LLeg, "Offset left leg:");
    //printVector(Offset_RLeg, "Offset right leg:");
    
    it=previous_status.begin();
    fprintf(stderr,"\n");
    fprintf(stderr, "Ntrials: %d\n", (int)Ntrials);
    fprintf(stderr, "F_LArm:      %s\n", it->ft_arm_left.toString().c_str());
    fprintf(stderr, "F_idyn_LArm: %s\n", F_iDyn_LArm.toString().c_str());
    fprintf(stderr, "F_RArm:      %s\n", it->ft_arm_right.toString().c_str());
    fprintf(stderr, "F_idyn_RArm: %s\n", F_iDyn_RArm.toString().c_str());
    fprintf(stderr, "F_LLeg:      %s\n", it->ft_leg_left.toString().c_str());
    fprintf(stderr, "F_idyn_LLeg: %s\n", F_iDyn_LLeg.toString().c_str());
    fprintf(stderr, "F_RLeg:      %s\n", it->ft_leg_right.toString().c_str());
    fprintf(stderr, "F_idyn_RLeg: %s\n", F_iDyn_RLeg.toString().c_str());
    fprintf(stderr, "\n");
    fprintf(stderr, "Left Arm:    %s\n", Offset_LArm.toString().c_str());
    fprintf(stderr, "Right Arm:   %s\n", Offset_RArm.toString().c_str());
    fprintf(stderr, "Left Leg:    %s\n", Offset_LLeg.toString().c_str());
    fprintf(stderr, "Right Leg:   %s\n", Offset_RLeg.toString().c_str());
    fprintf(stderr, "Left Foot:   %s\n", Offset_LFoot.toString().c_str());
    fprintf(stderr, "Right Foot:  %s\n", Offset_RFoot.toString().c_str());
    fprintf(stderr, "\n");
}

bool inverseDynamics::readAndUpdate(bool waitMeasure, bool _init)
{
    bool b = true;
    
    // arms
    if (ddAL)
    {
        Vector* tmp= 0;
        if (waitMeasure) fprintf(stderr,"Trying to connect to left arm sensor...");
        if (!dummy_ft)
        {
            tmp = port_ft_arm_left->read(waitMeasure);
            if (tmp != 0)
            {
                current_status.ft_arm_left  = *tmp;
            }
        }
        else
        {
            current_status.ft_arm_left.zero();
        }
        if (waitMeasure) fprintf(stderr,"done. \n");
    }

    if (ddAR)
    {
        Vector* tmp= 0;
        if (waitMeasure) fprintf(stderr,"Trying to connect to right arm sensor...");
        if (!dummy_ft)   
        {
            tmp = port_ft_arm_right->read(waitMeasure);
            if (tmp != 0)
            {
                current_status.ft_arm_right = *tmp;
            }
        }
        else
        {
            current_status.ft_arm_right.zero();
        }
        if (waitMeasure) fprintf(stderr,"done. \n");
    }
    b &= getUpperEncodersSpeedAndAcceleration();
    setUpperMeasure(_init);

    // legs
    if (ddLL)
    {
        Vector* tmp= 0;
        if (waitMeasure) fprintf(stderr,"Trying to connect to left leg sensor...");
        if (!dummy_ft)
        {
            tmp = port_ft_leg_left->read(waitMeasure);
            if (tmp != 0)
            {
                current_status.ft_leg_left  = *tmp;
            }
        }
        else
        {
            current_status.ft_leg_left.zero();
        }
        if (waitMeasure) fprintf(stderr,"done. \n");
    }
    if (ddLR)
    {
        Vector* tmp= 0;
        if (waitMeasure) fprintf(stderr,"Trying to connect to right leg sensor...");
        if (!dummy_ft)
        {
            tmp = port_ft_leg_right->read(waitMeasure);
            if (tmp != 0)
            {
                current_status.ft_leg_right = *tmp;
            }
        }
        else
        {
            current_status.ft_leg_right.zero();
        }
        if (waitMeasure) fprintf(stderr,"done. \n");
    }

    // feet
    if (ddLL)
    {
        Vector* tmp= 0;
        if (waitMeasure) fprintf(stderr,"Trying to connect to left foot sensor...");
        if (!dummy_ft)
        {
            tmp = port_ft_foot_left->read(false); //not all the robot versions have the FT sensors installed in the feet
            if (tmp != 0)
            {
                current_status.ft_foot_left  = *tmp;
            }
        }
        else
        {
            current_status.ft_foot_left.zero();
        }
        if (waitMeasure) fprintf(stderr,"done. \n");
    }
    if (ddLR)
    {
        Vector* tmp= 0;
        if (waitMeasure) fprintf(stderr,"Trying to connect to right foot sensor...");
        if (!dummy_ft)
        {
            tmp = port_ft_foot_right->read(false); //not all the robot versions have the FT sensors installed in the feet
            if (tmp != 0)
            {
                current_status.ft_foot_right = *tmp;
            }
        }
        else
        {
            current_status.ft_foot_right.zero();
        }
        if (waitMeasure) fprintf(stderr,"done. \n");
    }

    b &= getLowerEncodersSpeedAndAcceleration();
    setLowerMeasure(_init);

    //inertial sensor
    if (waitMeasure) fprintf(stderr,"Trying to connect to inertial sensor...");
    Vector *inertial = port_inertial_thread->read(waitMeasure);
    if (waitMeasure) fprintf(stderr,"done. \n");

    int sz = 0;
    if(inertial!=0)
    {
//#define DEBUG_FIXED_INERTIAL
#ifdef DEBUG_FIXED_INERTIAL
         (*inertial)[0] = 0;
         (*inertial)[1] = 0;
         (*inertial)[2] = 9.81;
         (*inertial)[3] = 0;
         (*inertial)[4] = 0;
         (*inertial)[5] = 0;
#endif
        current_status.inertial_d2p0[0] = (*inertial)[0];
        current_status.inertial_d2p0[1] = (*inertial)[1];
        current_status.inertial_d2p0[2] = (*inertial)[2];
        current_status.inertial_w0 [0] =  (*inertial)[3]*CTRL_DEG2RAD;
        current_status.inertial_w0 [1] =  (*inertial)[4]*CTRL_DEG2RAD;
        current_status.inertial_w0 [2] =  (*inertial)[5]*CTRL_DEG2RAD;
        current_status.inertial_dw0 = this->eval_domega(current_status.inertial_w0);
        //printf ("%3.3f, %3.3f, %3.3f \n",current_status.inertial_d2p0[0],current_status.inertial_d2p0[1],current_status.inertial_d2p0[2]);
#ifdef DEBUG_PRINT_INERTIAL
        printf ("meas_w  (rad/s):  %3.3f, %3.3f, %3.3f \n", w0[0],   w0[1],   w0[2]);
        printf ("meas_dwo(rad/s):  %3.3f, %3.3f, %3.3f \n", dw0[0],  dw0[1],  dw0[2]);
#endif
    }

    //update the status memory
    current_status.timestamp=Time::now();
    previous_status.push_front(current_status);
    if (previous_status.size()>status_queue_size) previous_status.pop_back();

    return b;
}

bool inverseDynamics::getLowerEncodersSpeedAndAcceleration()
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

    for (size_t i=0;i<3;i++)
    {
        current_status.all_q_low(i) = encoders_torso(2-i);
    }
    for (size_t i=0;i<6;i++)
    {
        current_status.all_q_low(3+i) = encoders_leg_left(i);
    }
    for (size_t i=0;i<6;i++)
    {
        current_status.all_q_low(3+6+i) = encoders_leg_right(i);
    }
    current_status.all_dq_low = evalVelLow(current_status.all_q_low);
    current_status.all_d2q_low = evalAccLow(current_status.all_q_low);

    return b;
}


bool inverseDynamics::getUpperEncodersSpeedAndAcceleration()
{
    bool b = true;
    if (iencs_arm_left) b &= iencs_arm_left->getEncoders(encoders_arm_left.data());
    else encoders_arm_left.zero();
    if (iencs_arm_right) b &= iencs_arm_right->getEncoders(encoders_arm_right.data());
    else encoders_arm_right.zero();
    if (iencs_head) b &= iencs_head->getEncoders(encoders_head.data());
    else encoders_head.zero();

    for (size_t i=0;i<3;i++)
    {
        current_status.all_q_up(i) = encoders_head(i);
    }
    for (size_t i=0;i<7;i++)
    {
        current_status.all_q_up(3+i) = encoders_arm_left(i);
    }
    for (size_t i=0;i<7;i++)
    {
        current_status.all_q_up(3+7+i) = encoders_arm_right(i);
    }
    current_status.all_dq_up = evalVelUp(current_status.all_q_up);
    current_status.all_d2q_up = evalAccUp(current_status.all_q_up);

    return b;
}

void inverseDynamics::setLowerMeasure(bool _init)
{
    if(!_init)
    {
        icub->lowerTorso->setAng("torso",CTRL_DEG2RAD * current_status.get_q_torso());
        icub->lowerTorso->setDAng("torso",CTRL_DEG2RAD * current_status.get_dq_torso());
        icub->lowerTorso->setD2Ang("torso",CTRL_DEG2RAD * current_status.get_d2q_torso());

        icub->lowerTorso->setAng("left_leg",CTRL_DEG2RAD * current_status.get_q_lleg());
        icub->lowerTorso->setDAng("left_leg",CTRL_DEG2RAD * current_status.get_dq_lleg());
        icub->lowerTorso->setD2Ang("left_leg",CTRL_DEG2RAD * current_status.get_d2q_lleg());

        icub->lowerTorso->setAng("right_leg",CTRL_DEG2RAD * current_status.get_q_rleg());
        icub->lowerTorso->setDAng("right_leg",CTRL_DEG2RAD * current_status.get_dq_rleg());
        icub->lowerTorso->setD2Ang("right_leg",CTRL_DEG2RAD * current_status.get_d2q_rleg());
    }
    else
    {
        icub_sens->lowerTorso->setAng("torso",CTRL_DEG2RAD * current_status.get_q_torso());
        icub_sens->lowerTorso->setDAng("torso",CTRL_DEG2RAD * current_status.get_dq_torso());
        icub_sens->lowerTorso->setD2Ang("torso",CTRL_DEG2RAD * current_status.get_d2q_torso());

        icub_sens->lowerTorso->setAng("left_leg",CTRL_DEG2RAD * current_status.get_q_lleg());
        icub_sens->lowerTorso->setDAng("left_leg",CTRL_DEG2RAD * current_status.get_dq_lleg());
        icub_sens->lowerTorso->setD2Ang("left_leg",CTRL_DEG2RAD * current_status.get_d2q_lleg());

        icub_sens->lowerTorso->setAng("right_leg",CTRL_DEG2RAD * current_status.get_q_rleg());
        icub_sens->lowerTorso->setDAng("right_leg",CTRL_DEG2RAD * current_status.get_dq_rleg());
        icub_sens->lowerTorso->setD2Ang("right_leg",CTRL_DEG2RAD * current_status.get_d2q_rleg());
    }
}

void inverseDynamics::setUpperMeasure(bool _init)
{
    if(!_init)
    {
        icub->upperTorso->setAng("head",CTRL_DEG2RAD * current_status.get_q_head());
        icub->upperTorso->setAng("left_arm",CTRL_DEG2RAD * current_status.get_q_larm());
        icub->upperTorso->setAng("right_arm",CTRL_DEG2RAD * current_status.get_q_rarm());
        icub->upperTorso->setDAng("head",CTRL_DEG2RAD * current_status.get_dq_head());
        icub->upperTorso->setDAng("left_arm",CTRL_DEG2RAD * current_status.get_dq_larm());
        icub->upperTorso->setDAng("right_arm",CTRL_DEG2RAD * current_status.get_dq_rarm());
        icub->upperTorso->setD2Ang("head",CTRL_DEG2RAD * current_status.get_d2q_head());
        icub->upperTorso->setD2Ang("left_arm",CTRL_DEG2RAD * current_status.get_d2q_larm());
        icub->upperTorso->setD2Ang("right_arm",CTRL_DEG2RAD * current_status.get_d2q_rarm());
        icub->upperTorso->setInertialMeasure(current_status.inertial_w0,current_status.inertial_dw0,current_status.inertial_d2p0);
    }
    else
    {
        icub_sens->upperTorso->setAng("head",CTRL_DEG2RAD * current_status.get_q_head());
        icub_sens->upperTorso->setAng("left_arm",CTRL_DEG2RAD * current_status.get_q_larm());
        icub_sens->upperTorso->setAng("right_arm",CTRL_DEG2RAD * current_status.get_q_rarm());
        icub_sens->upperTorso->setDAng("head",CTRL_DEG2RAD * current_status.get_dq_head());
        icub_sens->upperTorso->setDAng("left_arm",CTRL_DEG2RAD * current_status.get_dq_larm());
        icub_sens->upperTorso->setDAng("right_arm",CTRL_DEG2RAD * current_status.get_dq_rarm());
        icub_sens->upperTorso->setD2Ang("head",CTRL_DEG2RAD * current_status.get_d2q_head());
        icub_sens->upperTorso->setD2Ang("left_arm",CTRL_DEG2RAD * current_status.get_d2q_larm());
        icub_sens->upperTorso->setD2Ang("right_arm",CTRL_DEG2RAD *current_status.get_d2q_rarm());
        icub_sens->upperTorso->setInertialMeasure(current_status.inertial_w0,current_status.inertial_dw0,current_status.inertial_d2p0);
    }
}

void inverseDynamics::setZeroJntAngVelAcc()
{
    current_status.all_dq_up.zero();
    current_status.all_dq_low.zero();
    current_status.all_d2q_up.zero();
    current_status.all_d2q_low.zero();
}

void inverseDynamics::addSkinContacts()
{
    skinContactList *scl = port_skin_contacts->read(false);
    if(scl)
    {
        skinContactsTimestamp = Time::now();
        if(scl->empty() && !default_ee_cont)   // if no skin contacts => leave the old contacts but reset pressure and contact list
        {
            for(skinContactList::iterator it=skinContacts.begin(); it!=skinContacts.end(); it++)
            {
                it->setPressure(0.0);
                it->setActiveTaxels(0);
            }
            return;
        }
        
        map<BodyPart, skinContactList> contactsPerBp = scl->splitPerBodyPart();
        // if there are more than 1 contact and less than 10 taxels are active then suppose zero moment
        for(map<BodyPart,skinContactList>::iterator it=contactsPerBp.begin(); it!=contactsPerBp.end(); it++)
            if(it->second.size()>1)
                for(skinContactList::iterator c=it->second.begin(); c!=it->second.end(); c++)
                    if(c->getActiveTaxels()<10)
                        c->fixMoment();
        
        icub->upperTorso->clearContactList();
        icub->upperTorso->leftSensor->addContacts(contactsPerBp[LEFT_ARM].toDynContactList());
        icub->upperTorso->rightSensor->addContacts(contactsPerBp[RIGHT_ARM].toDynContactList());
        skinContacts = contactsPerBp[LEFT_ARM];
        skinContacts.insert(skinContacts.end(), contactsPerBp[RIGHT_ARM].begin(), contactsPerBp[RIGHT_ARM].end());
    }
    else if(Time::now()-skinContactsTimestamp>SKIN_EVENTS_TIMEOUT && skinContactsTimestamp!=0.0)
    {
        // if time is up, remove all the contacts
        icub->upperTorso->clearContactList();
        skinContacts.clear();
        add_legs_once = true;
    }
}

void inverseDynamics::sendMonitorData()
{
    Vector monitorData(0);
    monitorData = current_status.inertial_w0 * CTRL_RAD2DEG;        // w inertial sensor
    Vector temp = current_status.inertial_dw0 * CTRL_RAD2DEG;       // dw inertial sensor
    for(int i=0;i<3;i++) monitorData.push_back(temp(i));
    temp = icub->upperTorso->getAngVel() * CTRL_RAD2DEG;            // w upper node
    for(int i=0;i<3;i++) monitorData.push_back(temp(i));
    temp = icub->upperTorso->getAngAcc() * CTRL_RAD2DEG;            // dw upper node
    for(int i=0;i<3;i++) monitorData.push_back(temp(i));    
    monitorData.push_back(norm(icub->upperTorso->getLinAcc()));     // lin acc norm upper node
    monitorData.push_back(norm(icub->upperTorso->getTorsoForce())); // force norm upper node
    monitorData.push_back(norm(icub->upperTorso->getTorsoMoment()));// moment norm upper node    
    for(size_t i=0;i<3;i++){                                           // w torso
        temp = icub->lowerTorso->up->getAngVel(i) * CTRL_RAD2DEG;
        for(size_t j=0;j<temp.size();j++) monitorData.push_back(temp[j]);
    }
    for(size_t i=0;i<3;i++){                                           // dw torso
        temp = icub->lowerTorso->up->getAngAcc(i) * CTRL_RAD2DEG;
        for(size_t j=0;j<temp.size();j++) monitorData.push_back(temp[j]);
    }
    //for(int j=0;j<TOTorques.size();j++)                             // torso torques
    //    monitorData.push_back(TOTorques[j]);
    for(size_t i=0;i<3;i++)                                            // norm of COM ddp torso
        monitorData.push_back(norm(icub->lowerTorso->up->getLinAccCOM(i)));
    for(size_t i=0;i<3;i++)                                            // norm of forces torso
        monitorData.push_back(norm(icub->lowerTorso->up->getForce(i)));
    for(size_t i=0;i<3;i++){                                           // moments torso
        temp = icub->lowerTorso->up->getMoment(i);
        for(size_t j=0;j<temp.size();j++)
            monitorData.push_back(temp(j));
    }
    for(int i=0;i<3;i++)                                            // norm of moments head
        monitorData.push_back(norm(icub->upperTorso->up->getMoment(i)));    
    port_monitor->prepare()             = monitorData;
    port_monitor->write();
}

void inverseDynamics::sendVelAccData()
{
    Vector dump(0);
    size_t i;
    if(dumpvel_enabled)
    {
        for(i=0; i< current_status.all_q_up.size(); i++)
            dump.push_back(current_status.all_q_up[i]);
        for(i=0; i< current_status.all_dq_up.size(); i++)
            dump.push_back(current_status.all_dq_up[i]);
        for(i=0; i< current_status.all_d2q_up.size(); i++)
            dump.push_back(current_status.all_d2q_up[i]);
        for(i=0; i< current_status.all_q_low.size(); i++)
            dump.push_back(current_status.all_q_low[i]);
        for(i=0; i< current_status.all_dq_low.size(); i++)
            dump.push_back(current_status.all_dq_low[i]);
        for(i=0; i< current_status.all_d2q_low.size(); i++)
            dump.push_back(current_status.all_d2q_low[i]);

        port_dumpvel->prepare() = dump;
        port_dumpvel->write();
    }
}

