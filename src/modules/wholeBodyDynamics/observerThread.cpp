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
    // Left_arm variables
    allJnt = 0;
    int jnt=6;
    encoders_leg_left.resize(jnt,0.0);
    F_LLeg.resize(6,0.0);
    F_iDyn_LLeg.resize(6,0.0);
    Offset_LLeg.resize(6,0.0);
    allJnt+=jnt;

    // Right_leg variables
    jnt = 6;
    encoders_leg_right.resize(jnt,0.0);
    F_RLeg.resize(6,0.0);
    F_iDyn_RLeg.resize(6,0.0);
    Offset_RLeg.resize(6,0.0);
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

inverseDynamics::inverseDynamics(int _rate, PolyDriver *_ddAL, PolyDriver *_ddAR, PolyDriver *_ddH, PolyDriver *_ddLL, PolyDriver *_ddLR, PolyDriver *_ddT, string _robot_name, string _local_name, string icub_type, bool _autoconnect) : RateThread(_rate), ddAL(_ddAL), ddAR(_ddAR), ddH(_ddH), ddLL(_ddLL), ddLR(_ddLR), ddT(_ddT), robot_name(_robot_name), local_name(_local_name) 
{
    autoconnect = _autoconnect;
    com_enabled = true;
    com_vel_enabled = false;
    dummy_ft    = false;
    w0_dw0_enabled   = false;
    dumpvel_enabled = false;
    auto_drift_comp = false;

    icub      = new iCubWholeBody(DYNAMIC, VERBOSE, icub_type);
    icub_sens = new iCubWholeBody(DYNAMIC, VERBOSE, icub_type);
    first = true;
    skinContactsTimestamp = 0.0;

    //--------------INTERFACE INITIALIZATION-------------//

    iencs_arm_left = 0;
    iencs_arm_right= 0;
    iencs_head     = 0;
    iencs_leg_left = 0;
    iencs_leg_right= 0;
    iencs_torso    = 0;

    //---------------------PORT--------------------------//

    port_inertial_thread=new BufferedPort<Vector>;
    port_ft_arm_left=new BufferedPort<Vector>;
    port_ft_arm_right=new BufferedPort<Vector>;
    port_ft_leg_left=new BufferedPort<Vector>;
    port_ft_leg_right=new BufferedPort<Vector>;
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
    port_external_wrench_TO = new BufferedPort<Vector>; 
    port_external_cartesian_wrench_RA = new BufferedPort<Vector>;  
    port_external_cartesian_wrench_LA = new BufferedPort<Vector>;  
    port_skin_contacts = new BufferedPort<skinContactList>;
    port_com_all = new BufferedPort<Vector>;
    port_com_la  = new BufferedPort<Vector>;
    port_com_ra  = new BufferedPort<Vector>;
    port_com_ll  = new BufferedPort<Vector>;
    port_com_rl  = new BufferedPort<Vector>;
    port_com_to  = new BufferedPort<Vector>;
    port_com_hd  = new BufferedPort<Vector>;
    port_monitor = new BufferedPort<Vector>;
    port_dyn_contacts = new BufferedPort<dynContactList>;
    port_dumpvel = new BufferedPort<Vector>;

    port_inertial_thread->open(string("/"+local_name+"/inertial:i").c_str());
    port_ft_arm_left->open(string("/"+local_name+"/left_arm/FT:i").c_str());
    port_ft_arm_right->open(string("/"+local_name+"/right_arm/FT:i").c_str());
    port_ft_leg_left->open(string("/"+local_name+"/left_leg/FT:i").c_str());
    port_ft_leg_right->open(string("/"+local_name+"/right_leg/FT:i").c_str());
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
    port_external_cartesian_wrench_RA->open(string("/"+local_name+"/right_arm/cartesianEndEffectorWrench:o").c_str()); 
    port_external_cartesian_wrench_LA->open(string("/"+local_name+"/left_arm/cartesianEndEffectorWrench:o").c_str()); 
    port_external_wrench_TO->open(string("/"+local_name+"/torso/Wrench:o").c_str());
    port_com_all->open(string("/"+local_name+"/com:o").c_str());
    port_com_la ->open(string("/"+local_name+"/left_arm/com:o").c_str());
    port_com_ra ->open(string("/"+local_name+"/right_arm/com:o").c_str());
    port_com_ll ->open(string("/"+local_name+"/left_leg/com:o").c_str());
    port_com_rl ->open(string("/"+local_name+"/right_leg/com:o").c_str());
    port_com_hd ->open(string("/"+local_name+"/head/com:o").c_str());
    port_com_to ->open(string("/"+local_name+"/torso/com:o").c_str());
    port_skin_contacts->open(string("/"+local_name+"/skin_contacts:i").c_str());
    port_monitor->open(string("/"+local_name+"/monitor:o").c_str());
    port_dyn_contacts->open(string("/"+local_name+"/dyn_contacts:o").c_str());
    port_dumpvel->open(string("/"+local_name+"/va:o").c_str());

    if (autoconnect)
    {
        //from iCub to wholeBodyDynamics
        Network::connect(string("/"+local_name+"/filtered/inertial:o").c_str(),string("/"+local_name+"/inertial:i").c_str(),"tcp",false);			
        Network::connect(string("/"+robot_name+"/inertial").c_str(),           string("/"+local_name+"/unfiltered/inertial:i").c_str(),"tcp",false);
        Network::connect(string("/"+robot_name+"/left_arm/analog:o").c_str(),  string("/"+local_name+"/left_arm/FT:i").c_str(),"tcp",false);
        Network::connect(string("/"+robot_name+"/right_arm/analog:o").c_str(), string("/"+local_name+"/right_arm/FT:i").c_str(),"tcp",false);
        Network::connect(string("/"+robot_name+"/left_leg/analog:o").c_str(),  string("/"+local_name+"/left_leg/FT:i").c_str(),"tcp",false);
        Network::connect(string("/"+robot_name+"/right_leg/analog:o").c_str(), string("/"+local_name+"/right_leg/FT:i").c_str(),"tcp",false);
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
    if (ddAL) ddAL->view(iencs_arm_left);
    if (ddAR) ddAR->view(iencs_arm_right);
    if (ddH)  ddH->view(iencs_head);
    if (ddLL) ddLL->view(iencs_leg_left);
    if (ddLR) ddLR->view(iencs_leg_right);
    if (ddT)  ddT->view(iencs_torso);

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
}

bool inverseDynamics::threadInit()
{
    calibrateOffset(10);
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

void inverseDynamics::run()
{
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

    //get the FT sensors measurments
    if(current_status.ft_arm_left!=0)  F_LArm = -1.0 * (*current_status.ft_arm_left-Offset_LArm);
    if(current_status.ft_arm_right!=0) F_RArm = -1.0 * (*current_status.ft_arm_right-Offset_RArm);
    if(current_status.ft_leg_left!=0)  F_LLeg = -1.0 * (*current_status.ft_leg_left-Offset_LLeg);
    if(current_status.ft_leg_right!=0) F_RLeg = -1.0 * (*current_status.ft_leg_right-Offset_RLeg);

    //check if iCub is currently moving. If not, put the current iCub positions in the status queue
    current_status.iCub_not_moving = current_status.checkIcubNotMoving();
    if (current_status.iCub_not_moving == true)
    {
        not_moving_status.push(current_status);
        if (not_moving_status.size()>10) 
            {
                not_moving_status.pop();
                if (auto_drift_comp) fprintf (stderr,"drift_comp: buffer full\n"); //@@@DEBUG
            }
    }
    else
    {
        //efficient way to clear the queue
        std::queue<iCubStatus> empty;
        std::swap( not_moving_status, empty );
        if (auto_drift_comp) fprintf (stderr,"drift_comp: clearing buffer\n");  //@@@DEBUG
    }

    // THIS BLOCK SHOULD BE NOW DEPRECATED
    if (w0_dw0_enabled == false)
    {
        //if w0 and dw0 are too noisy, you can disable them using 'no_w0_dw0' option
        current_status.inertial_w0.zero();
        current_status.inertial_dw0.zero();
    }

    Vector F_up(6);
    F_up=0.0;
    icub->upperTorso->setInertialMeasure(current_status.inertial_w0,current_status.inertial_dw0,current_status.inertial_d2p0);
    icub->upperTorso->setSensorMeasurement(F_RArm,F_LArm,F_up);
    icub->upperTorso->solveKinematics();
    addSkinContacts();
    icub->upperTorso->solveWrench();

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

    Vector com_all(7), com_ll(7), com_rl(7), com_la(7),com_ra(7), com_hd(7), com_to(7); 
    double mass_all  , mass_ll  , mass_rl  , mass_la  ,mass_ra  , mass_hd,   mass_to;

    if (com_enabled)
    {
        icub->computeCOM();
        icub->getCOM(ALL_BODY_PARTS,    com_all, mass_all);
        icub->getCOM(LEFT_LEG,          com_ll,  mass_ll);
        icub->getCOM(RIGHT_LEG,         com_rl,  mass_rl);
        icub->getCOM(LEFT_ARM,          com_la,  mass_la);
        icub->getCOM(RIGHT_ARM,         com_ra,  mass_ra);
        icub->getCOM(HEAD,              com_hd,  mass_hd);
        icub->getCOM(TORSO,             com_to,  mass_to);
        com_all.push_back(mass_all);
        com_ll.push_back (mass_ll);
        com_rl.push_back (mass_rl);
        com_la.push_back (mass_la);
        com_ra.push_back (mass_ra);
        com_hd.push_back (mass_hd);
        com_to.push_back (mass_to);

        if (com_vel_enabled)
        {
            //experimental
            Vector com_v;
            icub->EXPERIMENTAL_computeCOMjacobian();
            icub->EXPERIMENTAL_getCOMvelocity(com_v);
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
    }
    else
    {
        mass_all=mass_ll=mass_rl=mass_la=mass_ra=mass_hd=mass_to=0.0;
        com_all.zero(); com_ll.zero(); com_rl.zero(); com_la.zero(); com_ra.zero(); com_hd.zero(); com_to.zero();
    }

    //F_ext_left_arm=icub->upperTorso->left->getForceMomentEndEff();//-icub_sens->upperTorso->left->getForceMomentEndEff();
    //F_ext_right_arm=icub->upperTorso->right->getForceMomentEndEff();//-icub_sens->upperTorso->right->getForceMomentEndEff();

    // DYN CONTACTS
    dynContactList contactList = icub->upperTorso->leftSensor->getContactList();
    dynContactList contactListR = icub->upperTorso->rightSensor->getContactList();
    contactList.insert(contactList.begin(), contactListR.begin(), contactListR.end());
    //printf("%s\n", contactList.toString().c_str());

    F_ext_left_arm = F_ext_right_arm = F_ext_cartesian_left_arm = F_ext_cartesian_right_arm = zeros(6);
    int eeInd = icub->upperTorso->left->getN()-1;
    for(dynContactList::const_iterator it=contactList.begin(); it!=contactList.end();it++){
        if(it->getLinkNumber() == eeInd){
            if(it->getBodyPart()==LEFT_ARM)
                F_ext_left_arm = it->getForceMoment();
            else if(it->getBodyPart()==RIGHT_ARM)
                F_ext_right_arm = it->getForceMoment();
        }
    }
    yarp::sig::Matrix ht  = icub->upperTorso->getHUp()    * icub->upperTorso->up->getH();
    yarp::sig::Matrix hl  = ht * icub->upperTorso->getHLeft()  * icub->upperTorso->left->getH();
    yarp::sig::Matrix hr  = ht * icub->upperTorso->getHRight() * icub->upperTorso->right->getH();


    yarp::sig::Vector tmp1 = F_ext_left_arm.subVector(0,2); tmp1.push_back(0.0); tmp1 = hl * tmp1;
    yarp::sig::Vector tmp2 = F_ext_left_arm.subVector(3,5); tmp2.push_back(0.0); tmp2 = hl * tmp2;
    for (int i=0; i<3; i++)	F_ext_cartesian_left_arm[i] = tmp1[i];
    for (int i=3; i<6; i++)	F_ext_cartesian_left_arm[i] = tmp2[i-3];
    double n1=norm(F_ext_cartesian_left_arm.subVector(0,2));
    double n2=norm(F_ext_cartesian_left_arm.subVector(3,5));
    F_ext_cartesian_left_arm.push_back(n1);
    F_ext_cartesian_left_arm.push_back(n2);
    
    
    tmp1 = F_ext_right_arm.subVector(0,2); tmp1.push_back(0.0); tmp1 = hr * tmp1;
    tmp2 = F_ext_right_arm.subVector(3,5); tmp2.push_back(0.0); tmp2 = hr * tmp2;
    for (int i=0; i<3; i++)	F_ext_cartesian_right_arm[i] = tmp1[i];
    for (int i=3; i<6; i++)	F_ext_cartesian_right_arm[i] = tmp2[i-3];
    n1=norm(F_ext_cartesian_right_arm.subVector(0,2));
    n2=norm(F_ext_cartesian_right_arm.subVector(3,5));
    F_ext_cartesian_right_arm.push_back(n1);
    F_ext_cartesian_right_arm.push_back(n2);
    

    // *** MONITOR DATA ***
    //sendMonitorData();

    // *** DUMP VEL DATA ***
    //sendVelAccData();

    broadcastData<Vector> (com_all, port_com_all);
    broadcastData<Vector> (com_ll,  port_com_ll);
    broadcastData<Vector> (com_rl,  port_com_rl);
    broadcastData<Vector> (com_la,  port_com_la);
    broadcastData<Vector> (com_ra,  port_com_ra);
    broadcastData<Vector> (com_hd,  port_com_hd);
    broadcastData<Vector> (com_to,  port_com_to);

    broadcastData<Vector> (F_up,                                    port_external_wrench_TO);
    broadcastData<Vector> (F_ext_right_arm,                         port_external_wrench_RA);
    broadcastData<Vector> (F_ext_left_arm,                          port_external_wrench_LA);
    broadcastData<Vector> (F_ext_cartesian_right_arm,               port_external_cartesian_wrench_RA);
    broadcastData<Vector> (F_ext_cartesian_left_arm,                port_external_cartesian_wrench_LA);
    broadcastData<iCub::skinDynLib::dynContactList>( contactList,   port_dyn_contacts);

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
    fprintf(stderr, "Closing cartesian_external_wrench_RA port\n");
    closePort(port_external_cartesian_wrench_RA);
    fprintf(stderr, "Closing cartesian_external_wrench_LA port\n");	
    closePort(port_external_cartesian_wrench_LA);
    fprintf(stderr, "Closing external_wrench_TO port\n");	
    closePort(port_external_wrench_TO);
    fprintf(stderr, "Closing COM ports\n");	
    closePort(port_com_all);
    closePort(port_com_ra);
    closePort(port_com_rl);
    closePort(port_com_la);
    closePort(port_com_ll);
    closePort(port_com_hd);
    closePort(port_com_to);

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
    fprintf(stderr, "Closing skin_contacts port\n");
    closePort(port_skin_contacts);
    fprintf(stderr, "Closing monitor port\n");
    closePort(port_monitor);
    fprintf(stderr, "Closing dump port\n");
    closePort(port_dumpvel);
    fprintf(stderr, "Closing dyn_contacts port\n");
    closePort(port_dyn_contacts);

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

void inverseDynamics::calibrateOffset(const unsigned int Ntrials)
{
    fprintf(stderr,"SensToTorques: starting sensor offset calibration, waiting for port connections... \n\n");
    if (!dummy_ft)
    {
        Vector *dummy = port_inertial_thread->read(true); //blocking call: waits for ports connection
    }

    Offset_LArm.zero();
    Offset_RArm.zero();
    Offset_LLeg.zero();
    Offset_RLeg.zero();

    // N trials to get a more accurate estimation
    for(unsigned int i=0; i<Ntrials; i++)
    {
        //read joints and ft sensor
        readAndUpdate(true,true);

        /*
        // TO BE VERIEFIED IF USEFUL
        setZeroJntAngVelAcc();
        setUpperMeasure(true);
        setLowerMeasure(true);
        */

        icub_sens->upperTorso->setInertialMeasure(current_status.inertial_w0,current_status.inertial_dw0,current_status.inertial_d2p0);
        Matrix F_sensor_up = icub_sens->upperTorso->estimateSensorsWrench(F_ext_up,false);
        icub_sens->lowerTorso->setInertialMeasure(icub_sens->upperTorso->getTorsoAngVel(),icub_sens->upperTorso->getTorsoAngAcc(),icub_sens->upperTorso->getTorsoLinAcc());
        Matrix F_sensor_low = icub_sens->lowerTorso->estimateSensorsWrench(F_ext_low,false);

        F_iDyn_LArm  = -1.0 * F_sensor_up.getCol(1);
        F_iDyn_RArm = -1.0 * F_sensor_up.getCol(0);
        F_iDyn_LLeg  = -1.0 * F_sensor_low.getCol(1);
        F_iDyn_RLeg = -1.0 * F_sensor_low.getCol(0);

        if (current_status.ft_arm_right) {F_RArm = *current_status.ft_arm_right;}
        else {F_RArm.zero();}
        if (current_status.ft_arm_left) {F_LArm = *current_status.ft_arm_left;}
        else {F_LArm.zero();}
        if (current_status.ft_leg_right) {F_RLeg = *current_status.ft_leg_right;}
        else {F_RLeg.zero();}
        if (current_status.ft_leg_left) {F_LLeg = *current_status.ft_leg_left;}
        else {F_LLeg.zero();}

        Offset_LArm = Offset_LArm + (F_LArm-F_iDyn_LArm);
        Offset_RArm = Offset_RArm + (F_RArm-F_iDyn_RArm);
        Offset_LLeg = Offset_LLeg + (F_LLeg-F_iDyn_LLeg);
        Offset_RLeg = Offset_RLeg + (F_RLeg-F_iDyn_RLeg);
    }

    Offset_LArm = 1.0/(double)Ntrials * Offset_LArm;
    Offset_RArm = 1.0/(double)Ntrials * Offset_RArm;
    Offset_LLeg = 1.0/(double)Ntrials * Offset_LLeg;
    Offset_RLeg = 1.0/(double)Ntrials * Offset_RLeg;

    //printVector(Offset_LArm, "Offset left arm:");
    //printVector(Offset_RArm, "Offset right arm:");
    //printVector(Offset_LLeg, "Offset left leg:");
    //printVector(Offset_RLeg, "Offset right leg:");
    
    fprintf(stderr,"\n");
    fprintf(stderr, "Ntrials: %d\n", Ntrials);
    fprintf(stderr, "F_LArm:      %s\n", F_LArm.toString().c_str());
    fprintf(stderr, "F_idyn_LArm: %s\n", F_iDyn_LArm.toString().c_str());
    fprintf(stderr, "F_RArm:      %s\n", F_RArm.toString().c_str());
    fprintf(stderr, "F_idyn_RArm: %s\n", F_iDyn_RArm.toString().c_str());
    fprintf(stderr, "F_LLeg:      %s\n", F_LLeg.toString().c_str());
    fprintf(stderr, "F_idyn_LLeg: %s\n", F_iDyn_LLeg.toString().c_str());
    fprintf(stderr, "F_RLeg:      %s\n", F_RLeg.toString().c_str());
    fprintf(stderr, "F_idyn_RLeg: %s\n", F_iDyn_RLeg.toString().c_str());
    fprintf(stderr, "\n");
    fprintf(stderr, "Left Arm:	  %s\n", Offset_LArm.toString().c_str());
    fprintf(stderr, "Right Arm:	  %s\n", Offset_RArm.toString().c_str());
    fprintf(stderr, "Left Leg:	  %s\n", Offset_LLeg.toString().c_str());
    fprintf(stderr, "Right Leg:	  %s\n", Offset_RLeg.toString().c_str());
    fprintf(stderr, "\n");
}

bool inverseDynamics::readAndUpdate(bool waitMeasure, bool _init)
{
    bool b = true;

    // arms
    if (ddAL)
    {
        if (waitMeasure) fprintf(stderr,"Trying to connect to left arm sensor...");
        if (!dummy_ft)   current_status.ft_arm_left  = port_ft_arm_left->read(waitMeasure);
        else             {if (!current_status.ft_arm_left) {current_status.ft_arm_left = new yarp::sig::Vector(6);} current_status.ft_arm_left->zero();}
        if (waitMeasure) fprintf(stderr,"done. \n");
    }

    if (ddAR)
    {
        if (waitMeasure) fprintf(stderr,"Trying to connect to right arm sensor...");
        if (!dummy_ft)   current_status.ft_arm_right = port_ft_arm_right->read(waitMeasure);
        else             {if (!current_status.ft_arm_right) {current_status.ft_arm_right = new yarp::sig::Vector(6);} current_status.ft_arm_right->zero();}
        if (waitMeasure) fprintf(stderr,"done. \n");
    }
    b &= getUpperEncodersSpeedAndAcceleration();
    setUpperMeasure(_init);

    // legs
    if (ddLL)
    {
        if (waitMeasure) fprintf(stderr,"Trying to connect to left leg sensor...");
        if (!dummy_ft)   current_status.ft_leg_left  = port_ft_leg_left->read(waitMeasure);
        else             {if (!current_status.ft_leg_left) {current_status.ft_leg_left = new yarp::sig::Vector(6);} current_status.ft_leg_left->zero();}
        if (waitMeasure) fprintf(stderr,"done. \n");
    }
    if (ddLR)
    {
        if (waitMeasure) fprintf(stderr,"Trying to connect to right leg sensor...");
        if (!dummy_ft)   current_status.ft_leg_right = port_ft_leg_right->read(waitMeasure);
        else             {if (!current_status.ft_leg_right) {current_status.ft_leg_right = new yarp::sig::Vector(6);} current_status.ft_leg_right->zero();}
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
#ifdef DEBUG_PRINT_INERTIAL
        printf ("meas_w  (rad/s):  %3.3f, %3.3f, %3.3f \n", w0[0],   w0[1],   w0[2]);
        printf ("meas_dwo(rad/s):  %3.3f, %3.3f, %3.3f \n", dw0[0],  dw0[1],  dw0[2]);
#endif
    }

    //update the status memory
    current_status.timestamp=Time::now();
    previous_status.push(current_status);
    if (previous_status.size()>10) previous_status.pop();

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

void inverseDynamics::addSkinContacts(){
    skinContactList *skinContacts = port_skin_contacts->read(false);
    if(skinContacts){
        skinContactsTimestamp = Time::now();
        icub->upperTorso->leftSensor->clearContactList();
        icub->upperTorso->rightSensor->clearContactList();
        // if there are more than 1 contact
        if(skinContacts->size()>1){
            for(skinContactList::iterator it=skinContacts->begin(); it!=skinContacts->end(); it++){
                //suppose zero moment if less than 10 taxels are active
                if(it->getActiveTaxels()<10)
                    it->fixMoment();
            }
        }
        dynContactList cl = skinContacts->toDynContactList();
        for(dynContactList::iterator it=cl.begin(); it!=cl.end(); it++){
            if(it->getBodyPart()==LEFT_ARM){
                icub->upperTorso->leftSensor->addContact((*it));
            }else if(it->getBodyPart() == RIGHT_ARM){
                icub->upperTorso->rightSensor->addContact((*it));
            }
        }
    }else if(skinContactsTimestamp!=0.0 && Time::now()-skinContactsTimestamp > SKIN_EVENTS_TIMEOUT){   
        // if time is up, remove all the contacts
        //fprintf(stderr, "Skin event timeout (%3.3f sec)\n", SKIN_EVENTS_TIMEOUT);
        icub->upperTorso->leftSensor->clearContactList();
        icub->upperTorso->rightSensor->clearContactList();
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

