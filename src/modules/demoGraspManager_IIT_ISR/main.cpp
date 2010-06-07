/** 
\defgroup demoGraspManager_IIT_ISR demoGraspManager_IIT_ISR
 
@ingroup icub_module  
 
The manager module for the Joint Grasping Demo developed by IIT 
and ISR. 

Copyright (C) 2009 RobotCub Consortium
 
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
 
\section lib_sec Libraries 
- YARP libraries. 

\section parameters_sec Parameters
None. 
 
\section portsa_sec Ports Accessed
Assumes that \ref icub_iCubInterface (with ICartesianControl 
interface implemented) and \ref iKinGazeCtrl are running. 
 
\section portsc_sec Ports Created 
 
- \e /demoGraspManager_IIT_ISR/trackTarget:i receives the 3-d 
  position to track.
 
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

[right_arm]
reach_offset	    0.0 0.15 -0.05
grasp_offset	    0.0 0.0 -0.05
grasp_sigma	        0.01 0.01 0.01
hand_orientation    -0.012968 -0.721210 0.692595 2.917075

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
#include <iCub/ctrl/ctrlMath.h>

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

#define STATE_IDLE          0
#define STATE_REACH         1
#define STATE_RELEASE       2
#define STATE_WAIT          3

YARP_DECLARE_DEVICES(icubmod)
                                                             
using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace ctrl;


class managerThread : public RateThread
{
protected:
    ResourceFinder &rf;

    string name;
    string robot;

    bool useLeftArm;
    bool useRightArm;
    int  armSel;

    PolyDriver *drvTorso, *drvLeftArm, *drvRightArm;
    PolyDriver *drvCartLeftArm, *drvCartRightArm;
    PolyDriver *drvGazeCtrl;

    IEncoders         *encTorso;
    IPositionControl  *posTorso;
    IEncoders         *encArm;
    IPositionControl  *posArm;
    ICartesianControl *cartArm;
    IGazeControl      *gazeCtrl;

    BufferedPort<Vector> *inportTrackTarget;
    Port *outportCmdFace;

    Vector leftArmReachOffs;
    Vector leftArmGraspOffs;
    Vector leftArmGraspSigma;
    Vector leftArmHandOrien;

    Vector rightArmReachOffs;
    Vector rightArmGraspOffs;
    Vector rightArmGraspSigma;
    Vector rightArmHandOrien;

    Vector *armReachOffs;
    Vector *armGraspOffs;
    Vector *armGraspSigma;
    Vector *armHandOrien;

    Vector homePoss, homeVels;

    double trajTime;
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

    Matrix R,Rx,Ry,Rz;

    int state;

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
                       Vector &graspSigma, Vector &orien)
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
                drvCartLeftArm->view(icart);
            else
                return;

            type="left_arm";
        }
        else if (sel==RIGHTARM)
        {
            if (useRightArm)
                drvCartRightArm->view(icart);
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
        if (encTorso->getEncoders(torso.data()))
            R=rotx(torso[1])*roty(-torso[2])*rotz(-torso[0]);

        if (Vector *targetPosNew=inportTrackTarget->read(false))
        {    
            targetPos=*targetPosNew;
            idleTimer=Time::now();

            if (state==STATE_IDLE)
            {
                resetTargetBall();

                fprintf(stdout,"--- Got target => REACHING\n");
                state=STATE_REACH;
            }
        }
        else if (state==STATE_REACH && Time::now()-idleTimer>idleTmo)
        {    
            fprintf(stdout,"--- Target timeout => IDLE\n");

            steerHeadToHome();
            stopControl();
            steerTorsoToHome();
            steerArmToHome(LEFTARM);
            steerArmToHome(RIGHTARM);

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
        // handle the hysteresis thresholds
        if (useLeftArm && useRightArm && (state==STATE_REACH))
        {    
            if ((armSel==LEFTARM) && (targetPos[1]>hystThres) ||
                (armSel==RIGHTARM) && (targetPos[1]<-hystThres))
            {
                stopControl();
                Time::delay(0.5);
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
                if (Time::now()-latchTimer>releaseTmo)
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
                if (Time::now()-latchTimer>idleTmo)
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
            if (checkArmForGrasp())
                setFace(FACE_EVIL);
            else
                setFace(FACE_ANGRY);
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
        outportCmdFace->write(out,in);

        out.clear();

        out.addVocab(Vocab::encode("set"));
        out.addVocab(Vocab::encode("leb"));
        out.addVocab(Vocab::encode(type.c_str()));
        outportCmdFace->write(out,in);

        out.clear();

        out.addVocab(Vocab::encode("set"));
        out.addVocab(Vocab::encode("reb"));
        out.addVocab(Vocab::encode(type.c_str()));
        outportCmdFace->write(out,in);
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

        if (inportTrackTarget)
        {
            inportTrackTarget->interrupt();
            inportTrackTarget->close();
            delete inportTrackTarget;
        }

        if (outportCmdFace)
        {
            outportCmdFace->interrupt();
            outportCmdFace->close();
            delete outportCmdFace;
        }
    }

public:
    managerThread(const string &_name, ResourceFinder &_rf) : 
                  RateThread(DEFAULT_THR_PER), name(_name), rf(_rf)
    {        
        drvTorso=drvLeftArm=drvRightArm=NULL;
        drvCartLeftArm=drvCartRightArm=NULL;
        drvGazeCtrl=NULL;

        inportTrackTarget=NULL;
        outportCmdFace=NULL;
    }

    virtual bool threadInit()
    {
        // general part
        Bottle &bGeneral=rf.findGroup("general");
        bGeneral.setMonitor(rf.getMonitor());
        robot=bGeneral.check("robot",Value("icub"),"Getting robot name").asString().c_str();
        useLeftArm=bGeneral.check("left_arm",Value("on"),"Getting left arm use flag").asString()=="on"?true:false;
        useRightArm=bGeneral.check("right_arm",Value("on"),"Getting right arm use flag").asString()=="on"?true:false;
        trajTime=bGeneral.check("traj_time",Value(2.0),"Getting trajectory time").asDouble();
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
        rightArmReachOffs.resize(3,0.0);
        rightArmGraspOffs.resize(3,0.0);
        rightArmGraspSigma.resize(3,0.0);
        rightArmHandOrien.resize(4,0.0);

        getArmOptions(bLeftArm,leftArmReachOffs,leftArmGraspOffs,
                      leftArmGraspSigma,leftArmHandOrien);
        getArmOptions(bRightArm,rightArmReachOffs,rightArmGraspOffs,
                      rightArmGraspSigma,rightArmHandOrien);

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

        string fwslash="/";

        // open remote_controlboard drivers
        Property optTorso("(device remote_controlboard)");
        Property optLeftArm("(device remote_controlboard)");
        Property optRightArm("(device remote_controlboard)");

        optTorso.put("remote",(fwslash+robot+"/torso").c_str());
        optTorso.put("local",(name+"/torso").c_str());

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
        }

        if (useRightArm)
        {
            drvCartRightArm=new PolyDriver;
            if (!drvCartRightArm->open(optCartRightArm))
            {
                close();
                return false;
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
        drvGazeCtrl->view(gazeCtrl);

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

        // open ports
        inportTrackTarget=new BufferedPort<Vector>;
        outportCmdFace   =new Port;

        inportTrackTarget->open((name+"/trackTarget:i").c_str());
        outportCmdFace->open((name+"/cmdFace:rpc").c_str());

        // init
        int torsoAxes;
        encTorso->getAxes(&torsoAxes);
        torso.resize(torsoAxes,0.0);

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
    YARP_REGISTER_DEVICES(icubmod)

    Network yarp;

    if (!yarp.checkNetwork())
        return -1;
    
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



