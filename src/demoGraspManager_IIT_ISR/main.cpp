
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Random.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/CartesianControl.h>

#include <gsl/gsl_math.h>
#include <iCub/ctrlMath.h>

#ifdef USE_ICUB_MOD
    #include "drivers.h"
#endif

#include <string>
#include <set>
#include <map>

#define DEFAULT_THR_PER     20

#define LEFTARM             0
#define RIGHTARM            1
#define USEDARM             -1

#define OPENHAND            0
#define CLOSEHAND           1

#define FACE_HAPPY          0
#define FACE_SAD            1
#define FACE_ANGRY          2
#define FACE_SHY            3
#define FACE_EVIL           4
#define FACE_CUNNING        5
#define FACE_SURPRISED      6

#define STATE_IDLE          0
#define STATE_REACH         1
#define STATE_GRASP         2
#define STATE_RELEASE       3
                                                             
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

    IEncoders         *encTorso;
    IPositionControl  *posTorso;
    IPositionControl  *posArm;
    ICartesianControl *cartArm;

    BufferedPort<Vector> *inportTrackTarget;
    BufferedPort<Bottle> *inportDetectGraspLeft;
    BufferedPort<Bottle> *inportDetectGraspRight;
    BufferedPort<Vector> *outportCmdHead;
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
    Vector *detectGrasp;

    Vector homePoss, homeVels;

    double trajTime;
    double idleTimer, idleTmo;
    double hystThres;
    double sphereRadius, sphereTmo;
    double releaseTmo;
    double graspModelDistThres;

    double latchTimer;
    Vector sphereCenter;

    Vector openHandPoss, closeHandPoss;
    Vector handVels;

    set<int> fingersSet;
    set<int> fingersMovingSet;
    multimap<int,int> jnts2FingersMap;

    Vector targetPos;
    Vector leftDetectGrasp;
    Vector rightDetectGrasp;
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
        else
            type=armSel==LEFTARM?"left_arm":"right_arm";

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
        else if (state!=STATE_IDLE && Time::now()-idleTimer>idleTmo)
        {    
            fprintf(stdout,"--- Target timeout => IDLE\n");

            steerHeadToHome();
            cartArm->stopControl();
            steerTorsoToHome();
            steerArmToHome(LEFTARM);
            steerArmToHome(RIGHTARM);

            state=STATE_IDLE;
        }

        if (Bottle *detectGraspNew=inportDetectGraspLeft->read(false))
        {
            int sz=detectGraspNew->size();
            int len=sz>leftDetectGrasp.length()?leftDetectGrasp.length():sz;
            for (int j=0; j<len; j++)
                leftDetectGrasp[j]=detectGraspNew->get(j).asDouble();
        }

        if (Bottle *detectGraspNew=inportDetectGraspRight->read(false))
        {
            int sz=detectGraspNew->size();
            int len=sz>rightDetectGrasp.length()?rightDetectGrasp.length():sz;
            for (int j=0; j<len; j++)
                rightDetectGrasp[j]=detectGraspNew->get(j).asDouble();
        }
    }

    void doIdle()
    {
    }

    void commandHead()
    {
        if (state!=STATE_IDLE)
        {
            outportCmdHead->prepare()=targetPos;
            outportCmdHead->write();
        }        
    }

    void steerHeadToHome()
    {
        Vector homeHead(3);

        homeHead[0]=-1.0;
        homeHead[0]=0.0;
        homeHead[0]=0.0;

        fprintf(stdout,"*** Homing head\n");

        outportCmdHead->prepare()=homeHead;
        outportCmdHead->write();
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
        else
            type=armSel==LEFTARM?"left_arm":"right_arm";

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

        fingersMovingSet=fingersSet;
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
                cartArm->stopControl();
                steerArmToHome();
            
                // swap interfaces
                armSel=!armSel;
            
                if (armSel==LEFTARM)
                {
                    drvLeftArm->view(posArm);
                    drvCartLeftArm->view(cartArm);
                    armReachOffs=&leftArmReachOffs;
                    armGraspOffs=&leftArmGraspOffs;
                    armGraspSigma=&leftArmGraspSigma;
                    armHandOrien=&leftArmHandOrien;
                    detectGrasp=&leftDetectGrasp;
                }
                else
                {
                    drvRightArm->view(posArm);
                    drvCartRightArm->view(cartArm);
                    armReachOffs=&rightArmReachOffs;
                    armGraspOffs=&rightArmGraspOffs;
                    armGraspSigma=&rightArmGraspSigma;
                    armHandOrien=&rightArmHandOrien;
                    detectGrasp=&rightDetectGrasp;
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

                    state=STATE_GRASP;
                }
            }
            else if (state==STATE_GRASP)
            {
                // check for hand closure
                if (isGraspEnded())
                {                    
                    fprintf(stdout,"--- Grasp done OR Hand closure complete => WAITING\n");

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
        else if (state==STATE_GRASP)
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
        // false if target is considered to be still moving
        if (norm(targetPos-sphereCenter)>sphereRadius)
        {
            resetTargetBall();
            return false;
        }
        else if (Time::now()-latchTimer<sphereTmo)
            return false;
        else
            return true;
    }

    bool isGraspEnded()
    {
        set<int> tmpSet=fingersMovingSet;  // latch the current moving fingers set

        for (set<int>::iterator i=fingersMovingSet.begin(); i!=fingersMovingSet.end(); ++i)
        {
            bool flag;
            posArm->checkMotionDone(*i,&flag);

            if (flag)
                tmpSet.erase(*i);
            else    // stop and remove all joints belonging to the finger
            {
                pair<multimap<int,int>::iterator,multimap<int,int>::iterator> fng=jnts2FingersMap.equal_range(*i);
                for (multimap<int,int>::iterator j=fng.first; j!=fng.second; ++j)
                {
                    if ((*detectGrasp)[j->second]>graspModelDistThres)
                    {                        
                        posArm->stop(j->first);
                        tmpSet.erase(j->first);
                    }
                }
            }
        }

        fingersMovingSet=tmpSet;    // update the moving fingers set

        if (fingersMovingSet.size())
            return false;
        else
            return true;
    }

    void resetTargetBall()
    {
        latchTimer=Time::now();
        sphereCenter=targetPos;
    }

    void setFace(const int type)
    {
        Bottle in, out;

        out.addString("set all ");

        switch (type)
        {
        case FACE_HAPPY:
            out.addString("hap");
            break;

        case FACE_SAD:
            out.addString("sad");
            break;

        case FACE_ANGRY:
            out.addString("ang");
            break;

        case FACE_SHY:
            out.addString("shy");
            break;

        case FACE_EVIL:
            out.addString("evi");
            break;

        case FACE_CUNNING:
            out.addString("cun");
            break;

        case FACE_SURPRISED:
            out.addString("sur");
            break;

        default:
            return;
        }

        outportCmdFace->write(out,in);
    }

    void limitRange(Vector &x)
    {               
        x[0]=x[0]>-0.1 ? -0.1 : x[0];       
    }

    Matrix &rotx(const double theta)
    {
        double t=(M_PI/180.0)*theta;
        double c=cos(t);
        double s=sin(t);

        Rx(1,1)=Rx(2,2)=c;
        Rx(1,2)=-s;
        Rx(2,1)=s;

        return Rx;
    }

    Matrix &roty(const double theta)
    {
        double t=(M_PI/180.0)*theta;
        double c=cos(t);
        double s=sin(t);

        Ry(0,0)=Ry(2,2)=c;
        Ry(0,2)=s;
        Ry(2,0)=-s;

        return Ry;
    }

    Matrix &rotz(const double theta)
    {
        double t=(M_PI/180.0)*theta;
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

        if (inportTrackTarget)
        {
            inportTrackTarget->interrupt();
            inportTrackTarget->close();
            delete inportTrackTarget;
        }

        if (inportDetectGraspLeft)
        {
            inportDetectGraspLeft->interrupt();
            inportDetectGraspLeft->close();
            delete inportDetectGraspLeft;
        }

        if (inportDetectGraspRight)
        {
            inportDetectGraspRight->interrupt();
            inportDetectGraspRight->close();
            delete inportDetectGraspRight;
        }

        if (outportCmdHead)
        {
            outportCmdHead->interrupt();
            outportCmdHead->close();
            delete outportCmdHead;
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

        inportTrackTarget=NULL;
        inportDetectGraspLeft=NULL;
        inportDetectGraspRight=NULL;
        outportCmdHead=NULL;
        outportCmdFace=NULL;
    }

    virtual bool threadInit()
    {
        // general part
        Bottle &bGeneral=rf.findGroup("general");
        bGeneral.setMonitor(rf.getMonitor());
        robot=bGeneral.check("robot",Value("icub"),"Getting robot name").asString();
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
        graspModelDistThres=bGrasp.check("grasp_thres",Value(0.0),"Getting grasp threshold").asDouble();

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

        // open cartesiancontrollerclient drivers
        Property optCartLeftArm("(device cartesiancontrollerclient)");
        Property optCartRightArm("(device cartesiancontrollerclient)");

        optCartLeftArm.put("remote",(fwslash+robot+"/cartesianController/left_arm").c_str());
        optCartLeftArm.put("local",(name+"/left_arm/cartesian").c_str());
    
        optCartRightArm.put("remote",(fwslash+robot+"/cartesianController/right_arm").c_str());
        optCartRightArm.put("local",(name+"/right_arm/cartesian").c_str());

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

        // open views
        drvTorso->view(encTorso);
        drvTorso->view(posTorso);

        if (useLeftArm)
        {
            drvLeftArm->view(posArm);
            drvCartLeftArm->view(cartArm);
            armReachOffs=&leftArmReachOffs;
            armGraspOffs=&leftArmGraspOffs;
            armGraspSigma=&leftArmGraspSigma;
            armHandOrien=&leftArmHandOrien;
            detectGrasp=&leftDetectGrasp;
            armSel=LEFTARM;
        }
        else
        {
            drvRightArm->view(posArm);
            drvCartRightArm->view(cartArm);
            armReachOffs=&rightArmReachOffs;
            armGraspOffs=&rightArmGraspOffs;
            armGraspSigma=&rightArmGraspSigma;
            armHandOrien=&rightArmHandOrien;
            detectGrasp=&rightDetectGrasp;
            armSel=RIGHTARM;
        }

        // open ports
        inportTrackTarget     =new BufferedPort<Vector>;
        inportDetectGraspLeft =new BufferedPort<Bottle>;
        inportDetectGraspRight=new BufferedPort<Bottle>;
        outportCmdHead        =new BufferedPort<Vector>;
        outportCmdFace        =new Port;

        inportTrackTarget->open((name+"/trackTarget:i").c_str());
        inportDetectGraspLeft->open((name+"/leftDetectGrasp:i").c_str());
        inportDetectGraspRight->open((name+"/rightDetectGrasp:i").c_str());
        outportCmdHead->open((name+"/cmdHead:o").c_str());
        outportCmdFace->open((name+"/cmdFace:rpc").c_str());

        // init
        int torsoAxes;
        encTorso->getAxes(&torsoAxes);
        torso.resize(torsoAxes,0.0);

        // hand joints set
        for (int i=7; i<16; i++)
            fingersSet.insert(i);

        // map from hand joints to fingers
        jnts2FingersMap.insert(pair<int,int>(8,0));
        jnts2FingersMap.insert(pair<int,int>(9,0));
        jnts2FingersMap.insert(pair<int,int>(10,0));
        jnts2FingersMap.insert(pair<int,int>(11,1));
        jnts2FingersMap.insert(pair<int,int>(12,1));
        jnts2FingersMap.insert(pair<int,int>(13,2));
        jnts2FingersMap.insert(pair<int,int>(14,2));
        jnts2FingersMap.insert(pair<int,int>(15,3));
        jnts2FingersMap.insert(pair<int,int>(15,4));

        targetPos.resize(3,0.0);
        leftDetectGrasp.resize(5,0.0);
        rightDetectGrasp.resize(5,0.0);
        R=Rx=Ry=Rz=eye(3,3);

        initCartesianCtrl(torsoSwitch,torsoLimits,LEFTARM);
        initCartesianCtrl(torsoSwitch,torsoLimits,RIGHTARM);

        // steer the robot to the initial configuration
        steerHeadToHome();
        cartArm->stopControl();
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
        commandFace();
    }

    virtual void threadRelease()
    {
        steerHeadToHome();
        cartArm->stopControl();
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

#ifdef USE_ICUB_MOD
    DriverCollection dev;
#endif

    managerModule mod;
    mod.setName("/demoGraspManager_IIT_ISR");

    return mod.runModule(rf);
}



