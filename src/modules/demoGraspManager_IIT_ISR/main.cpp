/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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

/** 
\defgroup demoGraspManager_IIT_ISR demoGraspManager_IIT_ISR
 
@ingroup icub_module  
 
The manager module for the Joint Grasping Demo developed by IIT 
and ISR. 

Copyright (C) 2010 RobotCub Consortium
 
Author: Ugo Pattacini 

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
This module collects the 3-d object positions estimated by the 
particle filter and sends data to the head and arm controllers 
in order to gaze at the target, reach for it and eventually 
grasp it. 
It relies on the YARP ICartesianControl interface to control 
both arms and on the YARP IGazeControl interface to control the 
gaze. 
 
Furthermore, there exists a second modality that enables to 
estimate the 3-d object position using stereo vision that needs 
to be calibrated in advance relying on a feed-forward neural 
network. 
 
\section lib_sec Libraries 
- ctrlLib. 
- iKin.  
- YARP libraries. 

\section parameters_sec Parameters
None. 
 
\section portsa_sec Ports Accessed
Assumes that \ref icub_iCubInterface (with ICartesianControl 
interface implemented) and \ref iKinGazeCtrl are running. 
 
\section portsc_sec Ports Created 
 
- \e /demoGraspManager_IIT_ISR/trackTarget:i receives the 3-d 
  position to track.
 
- \e /demoGraspManager_IIT_ISR/imdTargetLeft:i receives the 
  blobs list as produced by the \ref motionCUT module for the
  left eye.
 
- \e /demoGraspManager_IIT_ISR/imdTargetRight:i receives the 
  blobs list as produced by the \ref motionCUT module for the
  right eye.
 
- \e /demoGraspManager_IIT_ISR/cmdFace:o sends out commands to 
  the face expression high level interface in order to give an
  emotional representation of the current robot state.
 
- \e /demoGraspManager_IIT_ISR/rpc remote procedure 
    call. Recognized remote commands:
    -'quit' quit the module
 
\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files 
None. 
 
\section conf_file_sec Configuration Files
The configuration file passed through the option \e --from
should look like as follows:
 
\code 
[general]
// the robot name to connect to 
robot	        icub
// the thread period [ms] 
thread_period   30
// left arm switch 
left_arm        on 
// right arm switch 
right_arm       on 
// arm trajectory execution time [s]
traj_time       2.0 
// homes limbs if target detection timeout expires [s]
idle_tmo        5.0 
// enable the use of stereo vision calibrated by NN 
use_network off 
// NN configuration file 
network         network.ini 

[torso] 
// joint switch (min **) (max **) [deg]; 'min', 'max' optional 
pitch on  (max 30.0) 
roll off 
yaw on

[left_arm]
// the offset [m] to be added to the desired position  
reach_offset	    0.0 -0.15 -0.05
// the offset [m] for grasping 
grasp_offset	    0.0 0.0 -0.05
// perturbation given as standard deviation [m] 
grasp_sigma 0.01 0.01 0.01 
// hand orientation to be kept [axis-angle rep.] 
hand_orientation 0.064485 0.707066 0.704201 3.140572 
// enable impedance velocity mode 
impedance_velocity_mode off 
impedance_stiffness 0.5 0.5 0.5 0.2 0.1 
impedance_damping 60.0 60.0 60.0 20.0 0.0 

[right_arm]
reach_offset	    0.0 0.15 -0.05
grasp_offset	    0.0 0.0 -0.05
grasp_sigma	        0.01 0.01 0.01
hand_orientation    -0.012968 -0.721210 0.692595 2.917075
impedance_velocity_mode off 
impedance_stiffness 0.5 0.5 0.5 0.2 0.1 
impedance_damping 60.0 60.0 60.0 20.0 0.0 
 
[home_arm]
// home position [deg] 
poss    -30.0 30.0 0.0  45.0 0.0  0.0  0.0  0.0
// velocities to reach home positions [deg/s] 
vels    10.0  10.0 10.0 10.0 10.0 10.0 10.0 10.0

[arm_selection]
// hysteresis range added around plane y=0 [m]
hysteresis_thres 0.1

[grasp]
// ball radius [m] for still target detection 
sphere_radius   0.05 
// timeout [s] for still target detection 
sphere_tmo      3.0 
// timeout [s] to open hand after closure 
release_tmo     3.0 
// open hand positions [deg] 
open_hand       0.0 0.0 0.0   0.0   0.0 0.0 0.0   0.0   0.0 
// close hand positions [deg] 
close_hand      0.0 80.0 12.0 18.0 27.0 50.0 20.0  50.0 135.0 
// velocities to reach hand positions [deg/s] 
vels_hand       10.0 10.0  10.0 10.0 10.0 10.0 10.0 10.0  10.0 
\endcode 

\section tested_os_sec Tested OS
Windows, Linux

\author Ugo Pattacini
*/ 

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Random.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/GazeControl.h>

#include <gsl/gsl_math.h>
#include <iCub/ctrl/math.h>
#include <iCub/ctrl/neuralNetworks.h>
#include <iCub/iKin/iKinFwd.h>

#include <string>

#define DEFAULT_THR_PER     20

#define NOARM               0
#define LEFTARM             1
#define RIGHTARM            2
#define USEDARM             3

#define OPENHAND            0
#define CLOSEHAND           1

#define FACE_HAPPY          ("hap")
#define FACE_SAD            ("sad")
#define FACE_ANGRY          ("ang")
#define FACE_SHY            ("shy")
#define FACE_EVIL           ("evi")
#define FACE_CUNNING        ("cun")
#define FACE_SURPRISED      ("sur")

#define STATE_IDLE              0
#define STATE_REACH             1
#define STATE_CHECKMOTIONDONE   2
#define STATE_RELEASE           3
#define STATE_WAIT              4

YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;


class Predictor
{
protected:
    ff2LayNN_tansig_purelin net;
    iCubEye *eye;

public:
    Predictor() : eye(NULL) { }

    bool configure(Property &options)
    {
        eye=new iCubEye("left");
        eye->releaseLink(0);
        eye->releaseLink(1);
        eye->releaseLink(2);

        if (net.configure(options))
        {
            net.printStructure();
            return true;
        }
        else
            return false;
    }

    Vector predict(const Vector &torso, const Vector &head,
                   Bottle *imdLeft, Bottle *imdRight)
    {
        Bottle *firstBlobLeft=imdLeft->get(0).asList();
        Bottle *firstBlobRight=imdRight->get(0).asList();

        Vector in(7);
        in[0]=head[3];                              // tilt
        in[1]=head[4];                              // pan
        in[2]=head[5];                              // ver
        in[3]=firstBlobLeft->get(0).asDouble();     // ul
        in[4]=firstBlobLeft->get(1).asDouble();     // vl
        in[5]=firstBlobRight->get(0).asDouble();    // ur
        in[6]=firstBlobRight->get(1).asDouble();    // vr

        Vector out=net.predict(in);
        Vector homOut(4);
        homOut[0]=out[0];
        homOut[1]=out[1];
        homOut[2]=out[2];
        homOut[3]=1.0;

        Vector dof(eye->getDOF());
        dof[0]=CTRL_DEG2RAD*torso[2];
        dof[1]=CTRL_DEG2RAD*torso[1];
        dof[2]=CTRL_DEG2RAD*torso[0];
        dof[3]=CTRL_DEG2RAD*head[0];
        dof[4]=CTRL_DEG2RAD*head[1];
        dof[5]=CTRL_DEG2RAD*head[2];
        dof[6]=CTRL_DEG2RAD*head[3];
        dof[7]=CTRL_DEG2RAD*(head[4]+head[5]/2.0);

        homOut=eye->getH(dof)*homOut;
        out[0]=homOut[0];
        out[1]=homOut[1];
        out[2]=homOut[2];

        return out;
    }

    ~Predictor()
    {
        if (eye!=NULL)
            delete eye;
    }
};


class managerThread : public RateThread
{
protected:
    ResourceFinder &rf;

    string name;
    string robot;

    bool useLeftArm;
    bool useRightArm;
    int  armSel;

    PolyDriver *drvTorso, *drvHead, *drvLeftArm, *drvRightArm;
    PolyDriver *drvCartLeftArm, *drvCartRightArm;
    PolyDriver *drvGazeCtrl;

    IEncoders         *encTorso;
    IEncoders         *encHead;
    IPositionControl  *posTorso;
    IEncoders         *encArm;
    IPositionControl  *posArm;
    ICartesianControl *cartArm;
    IGazeControl      *gazeCtrl;    

    BufferedPort<Vector> inportTrackTarget;
    BufferedPort<Bottle> inportIMDTargetLeft;
    BufferedPort<Bottle> inportIMDTargetRight;
    Port outportCmdFace;

    Vector leftArmReachOffs;
    Vector leftArmGraspOffs;
    Vector leftArmGraspSigma;
    Vector leftArmHandOrien;
    Vector leftArmJointsStiffness;
    Vector leftArmJointsDamping;

    Vector rightArmReachOffs;
    Vector rightArmGraspOffs;
    Vector rightArmGraspSigma;
    Vector rightArmHandOrien;
    Vector rightArmJointsStiffness;
    Vector rightArmJointsDamping;

    Vector *armReachOffs;
    Vector *armGraspOffs;
    Vector *armGraspSigma;
    Vector *armHandOrien;

    Vector homePoss, homeVels;

    Predictor pred;
    bool useNetwork;
    bool wentHome;
    bool leftArmImpVelMode;
    bool rightArmImpVelMode;

    double trajTime;
    double reachTol;
    double idleTimer, idleTmo;
    double hystThres;
    double sphereRadius, sphereTmo;
    double releaseTmo;

    double latchTimer;
    Vector sphereCenter;

    Vector openHandPoss, closeHandPoss;
    Vector handVels;

    Vector targetPos;
    Vector torso;
    Vector head;

    Matrix R,Rx,Ry,Rz;

    int state;
    int startup_context_id_left;
    int startup_context_id_right;
    int startup_context_id_gaze;

    void getTorsoOptions(Bottle &b, const char *type, const int i, Vector &sw, Matrix &lim)
    {
        if (b.check(type))
        {
            Bottle &grp=b.findGroup(type);
            sw[i]=grp.get(1).asString()=="on"?1.0:0.0;

            if (grp.check("min","Getting minimum value"))
            {
                lim(i,0)=1.0;
                lim(i,1)=grp.find("min").asDouble();
            }

            if (grp.check("max","Getting maximum value"))
            {
                lim(i,2)=1.0;
                lim(i,3)=grp.find("max").asDouble();
            }
        }
    }

    void getArmOptions(Bottle &b, Vector &reachOffs, Vector &graspOffs,
                       Vector &graspSigma, Vector &orien, bool &impVelMode,
                       Vector &impStiff, Vector &impDamp)
    {
        if (b.check("reach_offset","Getting reaching offset"))
        {
            Bottle &grp=b.findGroup("reach_offset");
            int sz=grp.size()-1;
            int len=sz>3?3:sz;

            for (int i=0; i<len; i++)
                reachOffs[i]=grp.get(1+i).asDouble();
        }

        if (b.check("grasp_offset","Getting grasping offset"))
        {
            Bottle &grp=b.findGroup("grasp_offset");
            int sz=grp.size()-1;
            int len=sz>3?3:sz;

            for (int i=0; i<len; i++)
                graspOffs[i]=grp.get(1+i).asDouble();
        }

        if (b.check("grasp_sigma","Getting grasping sigma"))
        {
            Bottle &grp=b.findGroup("grasp_sigma");
            int sz=grp.size()-1;
            int len=sz>3?3:sz;

            for (int i=0; i<len; i++)
                graspSigma[i]=grp.get(1+i).asDouble();
        }

        if (b.check("hand_orientation","Getting hand orientation"))
        {
            Bottle &grp=b.findGroup("hand_orientation");
            int sz=grp.size()-1;
            int len=sz>4?4:sz;

            for (int i=0; i<len; i++)
                orien[i]=grp.get(1+i).asDouble();
        }

        impVelMode=b.check("impedance_velocity_mode",Value("off"),"Getting arm impedance-velocity-mode").asString()=="on"?true:false;

        if (b.check("impedance_stiffness","Getting joints stiffness"))
        {
            Bottle &grp=b.findGroup("impedance_stiffness");
            int sz=grp.size()-1;
            int len=sz>impStiff.length()?impStiff.length():sz;

            for (int i=0; i<len; i++)
                impStiff[i]=grp.get(1+i).asDouble();
        }

        if (b.check("impedance_damping","Getting joints damping"))
        {
            Bottle &grp=b.findGroup("impedance_damping");
            int sz=grp.size()-1;
            int len=sz>impDamp.length()?impDamp.length():sz;

            for (int i=0; i<len; i++)
                impDamp[i]=grp.get(1+i).asDouble();
        }
    }

    void getHomeOptions(Bottle &b, Vector &poss, Vector &vels)
    {
        if (b.check("poss","Getting home poss"))
        {
            Bottle &grp=b.findGroup("poss");
            int sz=grp.size()-1;
            int len=sz>7?7:sz;

            for (int i=0; i<len; i++)
                poss[i]=grp.get(1+i).asDouble();
        }

        if (b.check("vels","Getting home vels"))
        {
            Bottle &grp=b.findGroup("vels");
            int sz=grp.size()-1;
            int len=sz>7?7:sz;

            for (int i=0; i<len; i++)
                vels[i]=grp.get(1+i).asDouble();
        }
    }

    void getGraspOptions(Bottle &b, Vector &openPoss, Vector &closePoss, Vector &vels)
    {
        if (b.check("open_hand","Getting openHand poss"))
        {
            Bottle &grp=b.findGroup("open_hand");
            int sz=grp.size()-1;
            int len=sz>9?9:sz;

            for (int i=0; i<len; i++)
                openPoss[i]=grp.get(1+i).asDouble();
        }

        if (b.check("close_hand","Getting closeHand poss"))
        {
            Bottle &grp=b.findGroup("close_hand");
            int sz=grp.size()-1;
            int len=sz>9?9:sz;

            for (int i=0; i<len; i++)
                closePoss[i]=grp.get(1+i).asDouble();
        }

        if (b.check("vels_hand","Getting hand vels"))
        {
            Bottle &grp=b.findGroup("vels_hand");
            int sz=grp.size()-1;
            int len=sz>9?9:sz;

            for (int i=0; i<len; i++)
                vels[i]=grp.get(1+i).asDouble();
        }
    }

    void initCartesianCtrl(Vector &sw, Matrix &lim, const int sel=USEDARM)
    {
        ICartesianControl *icart=cartArm;
        Vector dof;
        string type;

        if (sel==LEFTARM)
        {
            if (useLeftArm)
            {
                drvCartLeftArm->view(icart);
                icart->storeContext(&startup_context_id_left);
            }
            else
                return;

            type="left_arm";
        }
        else if (sel==RIGHTARM)
        {
            if (useRightArm)
            {
                drvCartRightArm->view(icart);
                icart->storeContext(&startup_context_id_right);
            }
            else
                return;

            type="right_arm";
        }
        else if (armSel!=NOARM)
            type=armSel==LEFTARM?"left_arm":"right_arm";
        else
            return;

        fprintf(stdout,"*** Initializing %s controller ...\n",type.c_str());

        icart->setTrackingMode(false);
        icart->setTrajTime(trajTime);
        icart->setInTargetTol(reachTol);
        icart->getDOF(dof);

        for (int j=0; j<sw.length(); j++)
        {
            dof[j]=sw[j];

            if (sw[j] && (lim(j,0) || lim(j,2)))
            {
                double min, max;
                icart->getLimits(j,&min,&max);

                if (lim(j,0))
                    min=lim(j,1);

                if (lim(j,2))
                    max=lim(j,3);

                icart->setLimits(j,min,max);
                fprintf(stdout,"jnt #%d in [%g, %g] deg\n",j,min,max);
            }
        }

        icart->setDOF(dof,dof);

        fprintf(stdout,"DOF's=( ");
        for (int i=0; i<dof.length(); i++)
            fprintf(stdout,"%s ",dof[i]>0.0?"on":"off");
        fprintf(stdout,")\n");
    }

    void getSensorData()
    {
        bool newTarget=false;

        if (encTorso->getEncoders(torso.data()))
            R=rotx(torso[1])*roty(-torso[2])*rotz(-torso[0]);

        encHead->getEncoders(head.data());

        if (useNetwork)
        {            
            Bottle *imdTargetLeft=inportIMDTargetLeft.read(false);
            Bottle *imdTargetRight=inportIMDTargetRight.read(false);

            if ((imdTargetLeft!=NULL) && (imdTargetRight!=NULL))
            {
                targetPos=pred.predict(torso,head,imdTargetLeft,imdTargetRight);
                newTarget=true;
            }
        }
        else if (Vector *targetPosNew=inportTrackTarget.read(false))
        {
            targetPos=*targetPosNew;
            newTarget=true;
        }

        if (newTarget)
        {    
            idleTimer=Time::now();

            if (state==STATE_IDLE)
            {
                resetTargetBall();

                fprintf(stdout,"--- Got target => REACHING\n");
                
                wentHome=false;
                state=STATE_REACH;
            }
        }
        else if (((state==STATE_IDLE) || (state==STATE_REACH)) && 
                 ((Time::now()-idleTimer)>idleTmo) && !wentHome)
        {    
            fprintf(stdout,"--- Target timeout => IDLE\n");

            steerHeadToHome();
            stopControl();
            steerTorsoToHome();
            steerArmToHome(LEFTARM);
            steerArmToHome(RIGHTARM);

            wentHome=true;
            state=STATE_IDLE;
        }
    }

    void doIdle()
    {
    }

    void commandHead()
    {
        if (state!=STATE_IDLE)
            gazeCtrl->lookAtFixationPoint(targetPos);
    }

    void steerHeadToHome()
    {
        Vector homeHead(3);

        homeHead[0]=-1.0;
        homeHead[1]=0.0;
        homeHead[2]=0.3;

        fprintf(stdout,"*** Homing head\n");

        gazeCtrl->lookAtFixationPoint(homeHead);
    }

    void steerTorsoToHome()
    {
        Vector homeTorso(3);
        homeTorso.zero();

        Vector velTorso(3);
        velTorso=10.0;

        fprintf(stdout,"*** Homing torso\n");

        posTorso->setRefSpeeds(velTorso.data());
        posTorso->positionMove(homeTorso.data());
    }

    void steerArmToHome(const int sel=USEDARM)
    {
        IPositionControl *ipos=posArm;
        string type;

        if (sel==LEFTARM)
        {
            if (useLeftArm)
                drvLeftArm->view(ipos);
            else
                return;

            type="left_arm";
        }
        else if (sel==RIGHTARM)
        {
            if (useRightArm)
                drvRightArm->view(ipos);
            else
                return;

            type="right_arm";
        }
        else if (armSel!=NOARM)
            type=armSel==LEFTARM?"left_arm":"right_arm";
        else
            return;

        fprintf(stdout,"*** Homing %s\n",type.c_str());

        for (int j=0; j<homeVels.length(); j++)
        {
            ipos->setRefSpeed(j,homeVels[j]);
            ipos->positionMove(j,homePoss[j]);
        }

        openHand(sel);
    }

    void stopArmJoints(const int sel=USEDARM)
    {
        IEncoders        *ienc=encArm;
        IPositionControl *ipos=posArm;
        string type;

        if (sel==LEFTARM)
        {
            if (useLeftArm)
            {
                drvLeftArm->view(ienc);
                drvLeftArm->view(ipos);
            }
            else
                return;

            type="left_arm";
        }
        else if (sel==RIGHTARM)
        {
            if (useRightArm)
            {
                drvRightArm->view(ienc);
                drvRightArm->view(ipos);
            }
            else
                return;

            type="right_arm";
        }
        else if (armSel!=NOARM)
            type=armSel==LEFTARM?"left_arm":"right_arm";
        else
            return;

        fprintf(stdout,"*** Stopping %s joints\n",type.c_str());

        for (int j=0; j<homeVels.length(); j++)
        {
            double fb;

            ienc->getEncoder(j,&fb);
            ipos->positionMove(j,fb);
        }
    }

    void moveHand(const int action, const int sel=USEDARM)
    {
        IPositionControl *ipos=posArm;
        Vector *poss=NULL;       
        string actionStr, type;

        switch (action)
        {
        case OPENHAND:
                poss=&openHandPoss;
                actionStr="Opening";
                break;

        case CLOSEHAND:
                poss=&closeHandPoss;
                actionStr="Closing";
                break;

        default:
            return;
        }

        if (sel==LEFTARM)
        {    
            drvLeftArm->view(ipos);
            type="left_hand";
        }
        else if (sel==RIGHTARM)
        {    
            drvRightArm->view(ipos);
            type="right_hand";
        }
        else
            type=armSel==LEFTARM?"left_hand":"right_hand";

        fprintf(stdout,"*** %s %s\n",actionStr.c_str(),type.c_str());

        for (int j=0; j<handVels.length(); j++)
        {
            int k=homeVels.length()+j;

            ipos->setRefSpeed(k,handVels[j]);
            ipos->positionMove(k,(*poss)[j]);
        }
    }

    void openHand(const int sel=USEDARM)
    {
        moveHand(OPENHAND,sel);
    }

    void closeHand(const int sel=USEDARM)
    {        
        moveHand(CLOSEHAND,sel);
    }

    void selectArm()
    {
        if (useLeftArm && useRightArm)
        {
            if (state==STATE_REACH)
            {    
                // handle the hysteresis thresholds
                if ((armSel==LEFTARM) && (targetPos[1]>hystThres) ||
                    (armSel==RIGHTARM) && (targetPos[1]<-hystThres))
                {
                    fprintf(stdout,"*** Change arm event triggered\n");
                    state=STATE_CHECKMOTIONDONE;
                }
            }
            else if (state==STATE_CHECKMOTIONDONE)
            {
                bool done=false;
                cartArm->checkMotionDone(&done);

                if (done)
                {
                    stopControl();
                    steerArmToHome();

                    // swap interfaces
                    if (armSel==RIGHTARM)
                    {
                        armSel=LEFTARM;

                        drvLeftArm->view(encArm);
                        drvLeftArm->view(posArm);
                        drvCartLeftArm->view(cartArm);
                        armReachOffs=&leftArmReachOffs;
                        armGraspOffs=&leftArmGraspOffs;
                        armGraspSigma=&leftArmGraspSigma;
                        armHandOrien=&leftArmHandOrien;
                    }
                    else
                    {
                        armSel=RIGHTARM;

                        drvRightArm->view(encArm);
                        drvRightArm->view(posArm);
                        drvCartRightArm->view(cartArm);
                        armReachOffs=&rightArmReachOffs;
                        armGraspOffs=&rightArmGraspOffs;
                        armGraspSigma=&rightArmGraspSigma;
                        armHandOrien=&rightArmHandOrien;
                    }

                    fprintf(stdout,"*** Using %s\n",armSel==LEFTARM?"left_arm":"right_arm");
                    stopArmJoints();
                    state=STATE_REACH;
                }
            }
        }
    }

    void doReach()
    {
        if (useLeftArm || useRightArm)
        {
            if (state==STATE_REACH)
            {
                Vector x=R.transposed()*(targetPos+*armReachOffs);
                limitRange(x);
                x=R*x;
    
                cartArm->goToPose(x,*armHandOrien);
            }
        }
    }

    void doGrasp()
    {
        if (useLeftArm || useRightArm)
        {
            if (state==STATE_REACH)
            {
                if (checkTargetForGrasp() && checkArmForGrasp())
                {    
                    Vector graspOffs(3);
                    for (int i=0; i<graspOffs.length(); i++)
                        graspOffs[i]=Random::normal((*armGraspOffs)[i],(*armGraspSigma)[i]);

                    Vector x=R.transposed()*(targetPos+graspOffs);
                    limitRange(x);
                    x=R*x;

                    fprintf(stdout,"--- Hand in position AND Target still => GRASPING\n");
                    fprintf(stdout,"*** Grasping x=%s\n",x.toString().c_str());

                    cartArm->goToPoseSync(x,*armHandOrien);
                    closeHand();

                    latchTimer=Time::now();
                    state=STATE_RELEASE;
                }
            }
        }
    }

    void doRelease()
    {
        if (useLeftArm || useRightArm)
        {
            if (state==STATE_RELEASE)
            {
                if ((Time::now()-latchTimer)>releaseTmo)
                {
                    fprintf(stdout,"--- Timeout elapsed => RELEASING\n");

                    openHand();

                    latchTimer=Time::now();
                    state=STATE_WAIT;
                }
            }
        }
    }

    void doWait()
    {
        if (useLeftArm || useRightArm)
        {
            if (state==STATE_WAIT)
            {
                if ((Time::now()-latchTimer)>idleTmo)
                {
                    fprintf(stdout,"--- Timeout elapsed => IDLING\n");
                    state=STATE_IDLE;
                }
            }
        }
    }

    void commandFace()
    {
        if (state==STATE_IDLE)
            setFace(FACE_SHY);
        else if (state==STATE_REACH)
        {
            if (useLeftArm || useRightArm)
            {
                if (checkArmForGrasp())
                    setFace(FACE_EVIL);
                else
                    setFace(FACE_ANGRY);
            }
            else
                setFace(FACE_EVIL);
        }
        else if (state==STATE_WAIT)
            setFace(FACE_HAPPY);
    }

    bool checkArmForGrasp()
    {
        Vector x,o;
        cartArm->getPose(x,o);

        // true if arm has reached the position 
        if (norm(targetPos+*armReachOffs-x)<sphereRadius)
            return true;
        else
            return false;
    }

    bool checkTargetForGrasp()
    {
        const double t=Time::now();

        // false if target is considered to be still moving
        if (norm(targetPos-sphereCenter)>sphereRadius)
        {
            resetTargetBall();
            return false;
        }
        else if (t-latchTimer<sphereTmo || t-idleTimer>1.0)
            return false;
        else
            return true;
    }

    void resetTargetBall()
    {
        latchTimer=Time::now();
        sphereCenter=targetPos;
    }

    void stopControl()
    {
        if (useLeftArm || useRightArm)
            cartArm->stopControl();
    }

    void setFace(const string &type)
    {
        Bottle in, out;

        out.addVocab(Vocab::encode("set"));
        out.addVocab(Vocab::encode("mou"));
        out.addVocab(Vocab::encode(type.c_str()));
        outportCmdFace.write(out,in);

        out.clear();

        out.addVocab(Vocab::encode("set"));
        out.addVocab(Vocab::encode("leb"));
        out.addVocab(Vocab::encode(type.c_str()));
        outportCmdFace.write(out,in);

        out.clear();

        out.addVocab(Vocab::encode("set"));
        out.addVocab(Vocab::encode("reb"));
        out.addVocab(Vocab::encode(type.c_str()));
        outportCmdFace.write(out,in);
    }

    void limitRange(Vector &x)
    {               
        x[0]=x[0]>-0.1 ? -0.1 : x[0];       
    }

    Matrix &rotx(const double theta)
    {
        double t=CTRL_DEG2RAD*theta;
        double c=cos(t);
        double s=sin(t);

        Rx(1,1)=Rx(2,2)=c;
        Rx(1,2)=-s;
        Rx(2,1)=s;

        return Rx;
    }

    Matrix &roty(const double theta)
    {
        double t=CTRL_DEG2RAD*theta;
        double c=cos(t);
        double s=sin(t);

        Ry(0,0)=Ry(2,2)=c;
        Ry(0,2)=s;
        Ry(2,0)=-s;

        return Ry;
    }

    Matrix &rotz(const double theta)
    {
        double t=CTRL_DEG2RAD*theta;
        double c=cos(t);
        double s=sin(t);

        Rz(0,0)=Rz(1,1)=c;
        Rz(0,1)=-s;
        Rz(1,0)=s;

        return Rz;
    }

    void close()
    {
        if (drvTorso)
            delete drvTorso;

        if (drvHead)
            delete drvHead;

        if (drvLeftArm)
            delete drvLeftArm;

        if (drvRightArm)
            delete drvRightArm;

        if (drvCartLeftArm)
            delete drvCartLeftArm;

        if (drvCartRightArm)
            delete drvCartRightArm;

        if (drvGazeCtrl)
            delete drvGazeCtrl;

        inportTrackTarget.interrupt();
        inportTrackTarget.close();

        inportIMDTargetLeft.interrupt();
        inportIMDTargetLeft.close();

        inportIMDTargetRight.interrupt();
        inportIMDTargetRight.close();

        outportCmdFace.interrupt();
        outportCmdFace.close();
    }

public:
    managerThread(const string &_name, ResourceFinder &_rf) : 
                  RateThread(DEFAULT_THR_PER), name(_name), rf(_rf)
    {        
        drvTorso=drvHead=drvLeftArm=drvRightArm=NULL;
        drvCartLeftArm=drvCartRightArm=NULL;
        drvGazeCtrl=NULL;        
    }

    virtual bool threadInit()
    {
        // general part
        Bottle &bGeneral=rf.findGroup("general");
        bGeneral.setMonitor(rf.getMonitor());
        robot=bGeneral.check("robot",Value("icub"),"Getting robot name").asString().c_str();
        useLeftArm=bGeneral.check("left_arm",Value("on"),"Getting left arm use flag").asString()=="on"?true:false;
        useRightArm=bGeneral.check("right_arm",Value("on"),"Getting right arm use flag").asString()=="on"?true:false;
        useNetwork=bGeneral.check("use_network",Value("off"),"Getting network enable").asString()=="on"?true:false;
        trajTime=bGeneral.check("traj_time",Value(2.0),"Getting trajectory time").asDouble();
        reachTol=bGeneral.check("reach_tol",Value(0.01),"Getting reaching tolerance").asDouble();
        idleTmo=bGeneral.check("idle_tmo",Value(1e10),"Getting idle timeout").asDouble();        
        setRate(bGeneral.check("thread_period",Value(DEFAULT_THR_PER),"Getting thread period [ms]").asInt());

        // torso part
        Bottle &bTorso=rf.findGroup("torso");
        bTorso.setMonitor(rf.getMonitor());

        Vector torsoSwitch(3);   torsoSwitch.zero();
        Matrix torsoLimits(3,4); torsoLimits.zero();

        getTorsoOptions(bTorso,"pitch",0,torsoSwitch,torsoLimits);
        getTorsoOptions(bTorso,"roll",1,torsoSwitch,torsoLimits);
        getTorsoOptions(bTorso,"yaw",2,torsoSwitch,torsoLimits);

        // arm parts
        Bottle &bLeftArm=rf.findGroup("left_arm");
        Bottle &bRightArm=rf.findGroup("right_arm");
        bLeftArm.setMonitor(rf.getMonitor());
        bRightArm.setMonitor(rf.getMonitor());

        leftArmReachOffs.resize(3,0.0);
        leftArmGraspOffs.resize(3,0.0);
        leftArmGraspSigma.resize(3,0.0);
        leftArmHandOrien.resize(4,0.0);
        leftArmJointsStiffness.resize(5,0.0);
        leftArmJointsDamping.resize(5,0.0);
        rightArmReachOffs.resize(3,0.0);
        rightArmGraspOffs.resize(3,0.0);
        rightArmGraspSigma.resize(3,0.0);
        rightArmHandOrien.resize(4,0.0);
        rightArmJointsStiffness.resize(5,0.0);
        rightArmJointsDamping.resize(5,0.0);

        getArmOptions(bLeftArm,leftArmReachOffs,leftArmGraspOffs,
                      leftArmGraspSigma,leftArmHandOrien,leftArmImpVelMode,
                      leftArmJointsStiffness,leftArmJointsDamping);
        getArmOptions(bRightArm,rightArmReachOffs,rightArmGraspOffs,
                      rightArmGraspSigma,rightArmHandOrien,rightArmImpVelMode,
                      rightArmJointsStiffness,rightArmJointsDamping);

        // home part
        Bottle &bHome=rf.findGroup("home_arm");
        bHome.setMonitor(rf.getMonitor());
        homePoss.resize(7,0.0); homeVels.resize(7,0.0);
        getHomeOptions(bHome,homePoss,homeVels);

        // arm_selection part
        Bottle &bArmSel=rf.findGroup("arm_selection");
        bArmSel.setMonitor(rf.getMonitor());
        hystThres=bArmSel.check("hysteresis_thres",Value(0.0),"Getting hysteresis threshold").asDouble();

        // grasp part
        Bottle &bGrasp=rf.findGroup("grasp");
        bGrasp.setMonitor(rf.getMonitor());
        sphereRadius=bGrasp.check("sphere_radius",Value(0.0),"Getting sphere radius").asDouble();
        sphereTmo=bGrasp.check("sphere_tmo",Value(0.0),"Getting sphere timeout").asDouble();
        releaseTmo=bGrasp.check("release_tmo",Value(0.0),"Getting release timeout").asDouble();

        openHandPoss.resize(9,0.0); closeHandPoss.resize(9,0.0);
        handVels.resize(9,0.0);

        getGraspOptions(bGrasp,openHandPoss,closeHandPoss,handVels);

        // init network
        if (useNetwork)
        {
            Property options;
            options.fromConfigFile(rf.findFile(bGeneral.check("network",Value("network.ini"),
                                                              "Getting network data").asString().c_str()));

            if (!pred.configure(options))
                return false;            
        }

        // open ports
        inportTrackTarget.open((name+"/trackTarget:i").c_str());
        inportIMDTargetLeft.open((name+"/imdTargetLeft:i").c_str());
        inportIMDTargetRight.open((name+"/imdTargetRight:i").c_str());
        outportCmdFace.open((name+"/cmdFace:rpc").c_str());

        string fwslash="/";

        // open remote_controlboard drivers
        Property optTorso("(device remote_controlboard)");
        Property optHead("(device remote_controlboard)");
        Property optLeftArm("(device remote_controlboard)");
        Property optRightArm("(device remote_controlboard)");

        optTorso.put("remote",(fwslash+robot+"/torso").c_str());
        optTorso.put("local",(name+"/torso").c_str());

        optHead.put("remote",(fwslash+robot+"/head").c_str());
        optHead.put("local",(name+"/head").c_str());

        optLeftArm.put("remote",(fwslash+robot+"/left_arm").c_str());
        optLeftArm.put("local",(name+"/left_arm").c_str());

        optRightArm.put("remote",(fwslash+robot+"/right_arm").c_str());
        optRightArm.put("local",(name+"/right_arm").c_str());

        drvTorso=new PolyDriver;
        if (!drvTorso->open(optTorso))
        {
            close();
            return false;
        }

        drvHead=new PolyDriver;
        if (!drvHead->open(optHead))
        {
            close();
            return false;
        }

        if (useLeftArm)
        {
            drvLeftArm=new PolyDriver;
            if (!drvLeftArm->open(optLeftArm))
            {
                close();
                return false;
            }
        }

        if (useRightArm)
        {
            drvRightArm=new PolyDriver;
            if (!drvRightArm->open(optRightArm))
            {
                close();
                return false;
            }
        }

        // open cartesiancontrollerclient and gazecontrollerclient drivers
        Property optCartLeftArm("(device cartesiancontrollerclient)");
        Property optCartRightArm("(device cartesiancontrollerclient)");
        Property optGazeCtrl("(device gazecontrollerclient)");

        optCartLeftArm.put("remote",(fwslash+robot+"/cartesianController/left_arm").c_str());
        optCartLeftArm.put("local",(name+"/left_arm/cartesian").c_str());
    
        optCartRightArm.put("remote",(fwslash+robot+"/cartesianController/right_arm").c_str());
        optCartRightArm.put("local",(name+"/right_arm/cartesian").c_str());

        optGazeCtrl.put("remote","/iKinGazeCtrl");
        optGazeCtrl.put("local",(name+"/gaze").c_str());

        if (useLeftArm)
        {
            drvCartLeftArm=new PolyDriver;
            if (!drvCartLeftArm->open(optCartLeftArm))
            {
                close();
                return false;
            }

            if (leftArmImpVelMode)
            {
                IControlMode      *imode;
                IImpedanceControl *iimp;

                drvLeftArm->view(imode);
                drvLeftArm->view(iimp);

                int len=leftArmJointsStiffness.length()<leftArmJointsDamping.length()?
                        leftArmJointsStiffness.length():leftArmJointsDamping.length();

                for (int j=0; j<len; j++)
                {
                    imode->setImpedanceVelocityMode(j);
                    iimp->setImpedance(j,leftArmJointsStiffness[j],leftArmJointsDamping[j]);
                }
            }
        }

        if (useRightArm)
        {
            drvCartRightArm=new PolyDriver;
            if (!drvCartRightArm->open(optCartRightArm))
            {
                close();
                return false;
            }

            if (rightArmImpVelMode)
            {
                IControlMode      *imode;
                IImpedanceControl *iimp;

                drvRightArm->view(imode);
                drvRightArm->view(iimp);

                int len=rightArmJointsStiffness.length()<rightArmJointsDamping.length()?
                        rightArmJointsStiffness.length():rightArmJointsDamping.length();

                for (int j=0; j<len; j++)
                {
                    imode->setImpedanceVelocityMode(j);
                    iimp->setImpedance(j,rightArmJointsStiffness[j],rightArmJointsDamping[j]);
                }
            }
        }

        drvGazeCtrl=new PolyDriver;
        if (!drvGazeCtrl->open(optGazeCtrl))
        {
            close();
            return false;
        }

        // open views
        drvTorso->view(encTorso);
        drvTorso->view(posTorso);
        drvHead->view(encHead);
        drvGazeCtrl->view(gazeCtrl);

        gazeCtrl->storeContext(&startup_context_id_gaze);
        gazeCtrl->blockNeckRoll(0.0);

        if (useLeftArm)
        {
            drvLeftArm->view(encArm);
            drvLeftArm->view(posArm);
            drvCartLeftArm->view(cartArm);
            armReachOffs=&leftArmReachOffs;
            armGraspOffs=&leftArmGraspOffs;
            armGraspSigma=&leftArmGraspSigma;
            armHandOrien=&leftArmHandOrien;
            armSel=LEFTARM;
        }
        else if (useRightArm)
        {
            drvRightArm->view(encArm);
            drvRightArm->view(posArm);
            drvCartRightArm->view(cartArm);
            armReachOffs=&rightArmReachOffs;
            armGraspOffs=&rightArmGraspOffs;
            armGraspSigma=&rightArmGraspSigma;
            armHandOrien=&rightArmHandOrien;
            armSel=RIGHTARM;
        }
        else
        {
            encArm=NULL;
            posArm=NULL;
            cartArm=NULL;
            armReachOffs=NULL;
            armGraspOffs=NULL;
            armGraspSigma=NULL;
            armHandOrien=NULL;
            armSel=NOARM;
        }

        // init
        int torsoAxes;
        encTorso->getAxes(&torsoAxes);
        torso.resize(torsoAxes,0.0);

        int headAxes;
        encHead->getAxes(&headAxes);
        head.resize(headAxes,0.0);

        targetPos.resize(3,0.0);
        R=Rx=Ry=Rz=eye(3,3);

        initCartesianCtrl(torsoSwitch,torsoLimits,LEFTARM);
        initCartesianCtrl(torsoSwitch,torsoLimits,RIGHTARM);

        // steer the robot to the initial configuration
        steerHeadToHome();
        stopControl();
        steerTorsoToHome();
        steerArmToHome(LEFTARM);
        steerArmToHome(RIGHTARM);

        idleTimer=Time::now();
        Random::seed((int)idleTimer);

        wentHome=false;
        state=STATE_IDLE;

        return true;
    }

    virtual void run()
    {
        getSensorData();
        doIdle();
        commandHead();
        selectArm();
        doReach();
        doGrasp();
        doRelease();
        doWait();
        commandFace();
    }

    virtual void threadRelease()
    {
        steerHeadToHome();
        stopControl();
        steerTorsoToHome();
        steerArmToHome(LEFTARM);
        steerArmToHome(RIGHTARM);

        if (useLeftArm)
        {
            ICartesianControl *icart;
            drvCartLeftArm->view(icart);
            icart->restoreContext(startup_context_id_left);

            if (leftArmImpVelMode)
            {
                IControlMode *imode;
                drvLeftArm->view(imode);

                int len=leftArmJointsStiffness.length()<leftArmJointsDamping.length()?
                        leftArmJointsStiffness.length():leftArmJointsDamping.length();

                for (int j=0; j<len; j++)
                    imode->setVelocityMode(j);
            }
        }

        if (useRightArm)
        {
            ICartesianControl *icart;
            drvCartRightArm->view(icart);
            icart->restoreContext(startup_context_id_right);

            if (rightArmImpVelMode)
            {
                IControlMode *imode;
                drvRightArm->view(imode);

                int len=rightArmJointsStiffness.length()<rightArmJointsDamping.length()?
                        rightArmJointsStiffness.length():rightArmJointsDamping.length();

                for (int j=0; j<len; j++)
                    imode->setVelocityMode(j);
            }
        }

        gazeCtrl->restoreContext(startup_context_id_gaze);

        close();
    }
};


class managerModule: public RFModule
{
protected:
    managerThread *thr;    
    Port           rpcPort;

public:
    managerModule() { }

    virtual bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();            

        thr=new managerThread(getName().c_str(),rf);
        if (!thr->start())
        {
            delete thr;    
            return false;
        }

        rpcPort.open(getName("/rpc"));
        attach(rpcPort);

        return true;
    }

    virtual bool close()
    {
        rpcPort.interrupt();
        rpcPort.close();

        thr->stop();
        delete thr;

        return true;
    }

    virtual double getPeriod()    { return 1.0;  }
    virtual bool   updateModule() { return true; }
};


class myReport : public SearchMonitor
{
protected:
    Property comment, fallback, present, actual, reported;
    Bottle order;

public:
    virtual void report(const SearchReport& report, const char *context)
    {
        string ctx=context;
        string key=report.key.c_str();
        string prefix="";

        prefix=ctx;
        prefix+=".";

        key=prefix+key;
        if (key.substr(0,1)==".")
            key = key.substr(1,key.length());

        if (!present.check(key.c_str()))
        {
            present.put(key.c_str(),"present");
            order.addString(key.c_str());
        }

        if (report.isFound)
            actual.put(key.c_str(),report.value);

        if (report.isComment==true)
        {
            comment.put(key.c_str(),report.value);
            return;
        }

        if (report.isDefault==true)
        {
            fallback.put(key.c_str(),report.value);
            return;
        }

        if (comment.check(key.c_str()))
        {
            if (!reported.check(key.c_str()))
            {
                if (report.isFound)
                {
                    string hasValue=report.value.c_str();
                    if (hasValue.length()>35)
                        hasValue=hasValue.substr(0,30)+" ...";

                    fprintf(stdout,"Checking \"%s\": = %s (%s)\n",key.c_str(),
                            hasValue.c_str(),comment.check(key.c_str(),Value("")).toString().c_str());
                }
                else
                {
                    reported.put(key.c_str(),1);
                    bool hasDefault=fallback.check(key.c_str());
                    string defString="";

                    if (hasDefault)
                    {
                        defString+=" ";
                        defString+="(default ";
                        string theDefault=fallback.find(key.c_str()).toString().c_str();

                        if (theDefault=="")
                            defString+="is blank";
                        else
                            defString+=theDefault;

                        defString+=")";
                    }

                    fprintf(stdout,"Checking \"%s\": %s%s\n",key.c_str(),
                            comment.check(key.c_str(),Value("")).toString().c_str(),defString.c_str());
                }
            }
        }
    }
};


int main(int argc, char *argv[])
{
    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    YARP_REGISTER_DEVICES(icubmod)
    
    myReport rep;

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setMonitor(&rep);
    rf.setDefaultContext("demoGrasp_IIT_ISR/conf");
    rf.setDefaultConfigFile("config.ini");
    rf.configure("ICUB_ROOT",argc,argv);

    managerModule mod;
    mod.setName("/demoGraspManager_IIT_ISR");

    return mod.runModule(rf);
}



