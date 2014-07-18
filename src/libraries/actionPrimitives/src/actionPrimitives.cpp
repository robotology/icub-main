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

#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>

#include <gsl/gsl_math.h>

#include <iCub/ctrl/math.h>
#include <iCub/perception/springyFingers.h>
#include <iCub/perception/tactileFingers.h>
#include <iCub/action/actionPrimitives.h>

#include <stdio.h>
#include <stdarg.h>
#include <sstream>
#include <algorithm>

#define RES_WAVER(x)                                (dynamic_cast<ArmWavingMonitor*>(x))
                                                    
#define ACTIONPRIM_DEFAULT_PER                      50      // [ms]
#define ACTIONPRIM_DEFAULT_EXECTIME                 2.0     // [s]
#define ACTIONPRIM_DEFAULT_REACHTOL                 0.005   // [m]
#define ACTIONPRIM_DUMP_PERIOD                      1.0     // [s]
#define ACTIONPRIM_DEFAULT_EXT_FORCE_THRES          1e9     // [N, Nm]
#define ACTIONPRIM_DEFAULT_PART                     "right_arm"
#define ACTIONPRIM_DEFAULT_TRACKINGMODE             "off"
#define ACTIONPRIM_DEFAULT_VERBOSITY                "off"
#define ACTIONPRIM_DEFAULT_WBDYN_STEMNAME           "wholeBodyDynamics"
#define ACTIONPRIM_DEFAULT_WBDYN_PORTNAME           "cartesianEndEffectorWrench:o"

// defines for balancing the arm when in home position
#define ACTIONPRIM_BALANCEARM_PERIOD                2.0     // [s]
#define ACTIONPRIM_BALANCEARM_LENGTH                0.04    // [m]

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::perception;
using namespace iCub::action;


namespace iCub
{

namespace action
{

// This class handles the arm way points
/************************************************************************/
class ArmWayPoints : public RateThread
{
    deque<ActionPrimitivesWayPoint> wayPoints;
    ActionPrimitives  *action;
    ICartesianControl *cartCtrl;
    double default_exec_time;
    bool firstRun;
    double t0;
    Vector x0;
    size_t i;

    /************************************************************************/
    double checkTime(const double time) const
    {
        return std::max(time,0.01);
    }

    /************************************************************************/
    double checkDefaultTime(const double time) const
    {
        return (time>0.0?time:default_exec_time);
    }

    /************************************************************************/
    void printWayPoint()
    {
        if (wayPoints[i].oEnabled)
            action->printMessage("reaching waypoint(%d): x=[%s]; o=[%s]\n",i,
                                 wayPoints[i].x.toString(3,3).c_str(),
                                 wayPoints[i].o.toString(3,3).c_str());
        else
            action->printMessage("reaching waypoint(%d): x=[%s]\n",i,
                                 wayPoints[i].x.toString(3,3).c_str());
    }

    /************************************************************************/
    void execCallback()
    {
        if (wayPoints[i].callback!=NULL)
        {
            action->printMessage("executing waypoint(%d)-end callback ...\n",i);
            wayPoints[i].callback->exec();
            action->printMessage("... waypoint(%d)-end callback executed\n",i);
        }
    }

public:
    /************************************************************************/
    ArmWayPoints(ActionPrimitives *_action, const deque<ActionPrimitivesWayPoint> &_wayPoints) :
                 RateThread(ACTIONPRIM_DEFAULT_PER)
    {
        action=_action;
        action->getCartesianIF(cartCtrl);
        wayPoints=_wayPoints;
        default_exec_time=ACTIONPRIM_DEFAULT_EXECTIME;
        firstRun=true;
    }

    /************************************************************************/
    void set_default_exec_time(const double exec_time)
    {
        default_exec_time=exec_time;
    }

    /************************************************************************/
    bool threadInit()
    {
        if ((cartCtrl!=NULL) && (wayPoints.size()>0))
        {
            Vector o;
            cartCtrl->getPose(x0,o);
            setRate((int)(1000.0*checkTime(wayPoints[0].granularity)));
            i=0;

            return true;
        }
        else
            return false;
    }

    /************************************************************************/
    void run()
    {
        if (firstRun)
        {
            printWayPoint();
            firstRun=false;
            t0=Time::now();
        }

        double t=Time::now()-t0;
        double trajTime=checkDefaultTime(wayPoints[i].trajTime);
        double r=t/checkTime(checkDefaultTime(wayPoints[i].duration));

        Vector x=(r<1.0)?(x0+r*(wayPoints[i].x-x0)):wayPoints[i].x;
        if (wayPoints[i].oEnabled)
            cartCtrl->goToPose(x,wayPoints[i].o,trajTime);
        else
            cartCtrl->goToPosition(x,trajTime);

        // waypoint attained
        if (r>=1.0)
        {
            execCallback();
            if (i<wayPoints.size()-1)
            {
                x0=wayPoints[i].x;
                i++;
                printWayPoint();
                setRate((int)(1000.0*checkTime(wayPoints[i].granularity)));
                t0=Time::now();
            }
            else
                askToStop();
        }
    }

    /************************************************************************/
    virtual ~ArmWayPoints()
    {
        if (isRunning())
            stop();
    }
};


// This class handles the automatic arm-waving
/************************************************************************/
class ArmWavingMonitor : public RateThread
{
    ICartesianControl *cartCtrl;
    Vector restPos;
    Vector q0,w0;
    double L;

public:
    /************************************************************************/
    ArmWavingMonitor(ICartesianControl *_cartCtrl) :
                     RateThread((int)(1000.0*ACTIONPRIM_BALANCEARM_PERIOD))
    {
        cartCtrl=_cartCtrl;
        L=ACTIONPRIM_BALANCEARM_LENGTH;

        restPos.resize(1,0.0);

        Rand::init();
    }

    /************************************************************************/
    void setRestPosition(const Vector &_restPos)
    {
        restPos=_restPos;
    }

    /************************************************************************/
    void afterStart(bool success)
    {
        cartCtrl->getRestPos(q0);
        cartCtrl->getRestWeights(w0);

        // start in suspended mode
        disable();
    }

    /************************************************************************/
    bool enable()
    {
        if (isSuspended())
        {
            cartCtrl->getRestPos(q0);
            cartCtrl->getRestWeights(w0);

            // impose further constraints as third task
            // since we reach only in position and not in orientation:
            // keep the wrist aligned with the forearm
            Vector q=q0;
            Vector w=w0;
            q[3+5]=0.0;
            w[3+5]=1.0;

            cartCtrl->setRestPos(q,q);
            cartCtrl->setRestWeights(w,w);

            resume();

            return true;
        }
        else
            return false;
    }

    /************************************************************************/
    bool disable()
    {
        if (!isSuspended())
        {
            cartCtrl->setRestPos(q0,q0);
            cartCtrl->setRestWeights(w0,w0);

            suspend();

            return true;
        }
        else
            return false;
    }

    /************************************************************************/
    void run()
    {
        int len=restPos.length();
        if ((cartCtrl!=NULL) && (len>=3))
        {
            Vector halves(len,0.5);
            Vector randOffs=L*(Rand::vector(len)-halves);
    
            cartCtrl->goToPosition(restPos+randOffs);
        }
    }

    /************************************************************************/
    virtual ~ArmWavingMonitor()
    {
        // safety check: make sure to stop
        // the interface when closing
        cartCtrl->stopControl();
    }
};

}

}


/************************************************************************/
ActionPrimitivesWayPoint::ActionPrimitivesWayPoint()
{
    x.resize(3,0.0);
    o.resize(4,0.0);
    oEnabled=false;
    duration=ACTIONPRIM_DISABLE_EXECTIME;
    trajTime=ACTIONPRIM_DISABLE_EXECTIME;
    granularity=ACTIONPRIM_DEFAULT_PER/1000.0;
    callback=NULL;
}


/************************************************************************/
void ActionPrimitives::ActionsQueue::clear()
{
    for (size_t i=0; i<this->size(); i++)
    {
        Action &action=(*this)[i];
        if (action.execWayPoints)
            delete action.wayPointsThr;
    }

    deque<Action>::clear();
}


/************************************************************************/
ActionPrimitives::ActionPrimitives() :
                  RateThread(ACTIONPRIM_DEFAULT_PER)
{
    init();
}


/************************************************************************/
ActionPrimitives::ActionPrimitives(Property &opt) :
                  RateThread(ACTIONPRIM_DEFAULT_PER)
{
    init();
    open(opt);
}


/************************************************************************/
void ActionPrimitives::init()
{
    armWaver=NULL;
    actionClb=NULL;
    actionWP=NULL;
    graspModel=NULL;

    armMoveDone =latchArmMoveDone =true;
    handMoveDone=latchHandMoveDone=true;
    configured=closed=false;
    checkEnabled=true;
    torsoActive=true;
    handSeqTerminator=false;
    fingersInPosition=true;
    reachTmoEnabled=false;
    locked=false;

    latchTimerWait=waitTmo=0.0;
    latchTimerReach=reachTmo=0.0;
    latchTimerHand=curHandTmo=0.0;
    latchTimerReachLog=0.0;
}


/************************************************************************/
bool ActionPrimitives::isValid() const
{
    return configured;
}


/************************************************************************/
int ActionPrimitives::printMessage(const char *format, ...)
{
    if (verbose)
    {
        fprintf(stdout,"*** %s: ",(local+"/"+part).c_str());
    
        va_list ap;
        va_start(ap,format);    
        int ret=vfprintf(stdout,format,ap);
        va_end(ap);
        
        return ret;
    }
    else
        return -1;
}


/************************************************************************/
bool ActionPrimitives::handleTorsoDOF(Property &opt, const string &key, const int j)
{
    if (opt.check(key.c_str()))
    {
        bool sw=opt.find(key.c_str()).asString()=="on"?true:false;

        Vector newDof(3,2.0), dummyRet;
        newDof[j]=sw?1.0:0.0;

        printMessage("%s %s\n",key.c_str(),sw?"enabled":"disabled");
        cartCtrl->setDOF(newDof,dummyRet);

        if (sw)
        {
            string minKey=key+"_min";
            string maxKey=key+"_max";
            double min, max;

            cartCtrl->getLimits(j,&min,&max);

            min=opt.check(minKey.c_str(),Value(min)).asDouble();
            max=opt.check(maxKey.c_str(),Value(max)).asDouble();

            cartCtrl->setLimits(j,min,max);
            cartCtrl->getLimits(j,&min,&max);

            Vector weights;
            if (cartCtrl->getRestWeights(weights))
            {
                // enforce the torso rest position
                weights[j]=4.0;
                cartCtrl->setRestWeights(weights,weights);
            }
            else
            {
                printMessage("failed to get weights from cartesian controller\n");
                return false;
            }

            printMessage("%s limits: [%g,%g] deg; weight=%g\n",key.c_str(),min,max,weights[j]);
        }

        return true;
    }

    return false;
}


/************************************************************************/
bool ActionPrimitives::configHandSeq(Property &opt)
{
    if (opt.check("hand_sequences_file"))
    {
        string handSeqFile=opt.find("hand_sequences_file").asString().c_str();        

        printMessage("Processing %s file\n",handSeqFile.c_str());
        Property handSeqProp; handSeqProp.fromConfigFile(handSeqFile.c_str());
    
        // GENERAL group
        Bottle &bGeneral=handSeqProp.findGroup("GENERAL");
        if (bGeneral.isNull())
        {
            printMessage("WARNING: \"GENERAL\" group is missing\n");    
            return false;
        }

        if (!bGeneral.check("numSequences"))
        {
            printMessage("WARNING: \"numSequences\" option is missing\n");    
            return false;
        }

        int numSequences=bGeneral.find("numSequences").asInt();

        // SEQUENCE groups
        for (int i=0; i<numSequences; i++)
        {
            ostringstream seq;
            seq<<"SEQ_"<<i;

            Bottle &bSeq=handSeqProp.findGroup(seq.str().c_str());
            if (bSeq.isNull())
            {
                printMessage("WARNING: \"%s\" group is missing\n",seq.str().c_str());
                return false;
            }

            if (!bSeq.check("key"))
            {
                printMessage("WARNING: \"key\" option is missing\n");
                return false;
            }

            string key=bSeq.find("key").asString().c_str();

            if (isValidHandSeq(key))
            {
                printMessage("WARNING: the sequence \"%s\" is already defined: skipping ...\n",
                             key.c_str());
                continue;
            }

            addHandSequence(key,bSeq);
        }
    
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitives::configGraspModel(Property &opt)
{
    bool ret=false;
    if (opt.check("grasp_model_type"))
    {
        string modelType=opt.find("grasp_model_type").asString().c_str();
        if (modelType!="none")
        {
            if (opt.check("grasp_model_file"))
            {
                if (modelType=="springy")
                    graspModel=new SpringyFingersModel;
                else if (modelType=="tactile")
                    graspModel=new TactileFingersModel;
                else
                {
                    printMessage("WARNING: unknown grasp model type %s!\n",modelType.c_str());
                    return false;
                }

                string modelFile=opt.find("grasp_model_file").asString().c_str();                
                printMessage("Retrieving grasp model data from %s file\n",modelFile.c_str());
                Property modelProp; modelProp.fromConfigFile(modelFile.c_str());

                // consistency check between the model and the part
                if (modelProp.check("type"))
                {
                    string type=modelProp.find("type").asString().c_str();
                    type+="_arm";
                    if (type!=part)
                    {
                        printMessage("WARNING: attempt to instantiate a grasp model of type %s while part is %s\n",
                                     type.c_str(),part.c_str());
                        return false;
                    }
                }

                // override some information
                modelProp.put("robot",robot.c_str());
                return graspModel->fromProperty(modelProp);
            }
            else
                printMessage("WARNING: unable to find \"grasp_model_file\" option!\n");
        }
        else
            ret=true;
    }

    return ret;
}


/************************************************************************/
bool ActionPrimitives::open(Property &opt)
{
    if (configured)
    {
        printMessage("WARNING: already configured\n");
        return true;
    }

    if (!opt.check("local"))
    {
        printMessage("ERROR: option \"local\" is missing\n");
        return false;
    }

    robot=opt.check("robot",Value("icub")).asString().c_str();
    local=opt.find("local").asString().c_str();
    part=opt.check("part",Value(ACTIONPRIM_DEFAULT_PART)).asString().c_str();
    default_exec_time=opt.check("default_exec_time",Value(ACTIONPRIM_DEFAULT_EXECTIME)).asDouble();
    tracking_mode=opt.check("tracking_mode",Value(ACTIONPRIM_DEFAULT_TRACKINGMODE)).asString()=="on"?true:false;
    verbose=opt.check("verbosity",Value(ACTIONPRIM_DEFAULT_VERBOSITY)).asString()=="on"?true:false;    

    int period=opt.check("thread_period",Value(ACTIONPRIM_DEFAULT_PER)).asInt();    
    double reach_tol=opt.check("reach_tol",Value(ACTIONPRIM_DEFAULT_REACHTOL)).asDouble();

    // create the model for grasp detection (if any)
    if (!configGraspModel(opt))
    {
        close();
        return false;
    }

    // get hand sequence motions (if any)
    configHandSeq(opt);

    // open the position client
    Property optPolyHand("(device remote_controlboard)");
    optPolyHand.put("remote",("/"+robot+"/"+part).c_str());
    optPolyHand.put("local",("/"+local+"/"+part+"/position").c_str());
    if (!polyHand.open(optPolyHand))
    {
        close();
        return false;
    }

    // open the cartesian client
    Property optPolyCart("(device cartesiancontrollerclient)");
    optPolyCart.put("remote",("/"+robot+"/cartesianController/"+part).c_str());
    optPolyCart.put("local",("/"+local+"/"+part+"/cartesian").c_str());
    if (!polyCart.open(optPolyCart))
    {
        close();
        return false;
    }

    // open views
    polyHand.view(modCtrl);
    polyHand.view(encCtrl);    
    polyHand.view(posCtrl);
    polyCart.view(cartCtrl);

    // latch the controller context
    cartCtrl->storeContext(&startup_context_id);

    // set tolerance
    cartCtrl->setInTargetTol(reach_tol);

    // set tracking mode
    setTrackingMode(tracking_mode);

    // handle torso DOF's
    if (!handleTorsoDOF(opt,"torso_pitch",0))
        return false;
    if (!handleTorsoDOF(opt,"torso_roll",1))
        return false;
    if (!handleTorsoDOF(opt,"torso_yaw",2))
        return false;

    Vector curDof;
    cartCtrl->getDOF(curDof);

    enableTorsoSw.resize(3,0.0);
    disableTorsoSw.resize(3,0.0);
    for (int i=0; i<3; i++)
        enableTorsoSw[i]=curDof[i];

    // start with torso disabled
    disableTorsoDof();    

    jHandMin=7;                     // hand first joint
    posCtrl->getAxes(&jHandMax);    // hand last joint

    // hand joints set
    for (int j=jHandMin; j<jHandMax; j++)
    {
        fingersJnts.push_back(j);
        fingersJntsSet.insert(j);
    }

    // map from hand joints to fingers
    fingers2JntsMap.insert(pair<int,int>(0,8));
    fingers2JntsMap.insert(pair<int,int>(0,9));
    fingers2JntsMap.insert(pair<int,int>(0,10));
    fingers2JntsMap.insert(pair<int,int>(1,11));
    fingers2JntsMap.insert(pair<int,int>(1,12));
    fingers2JntsMap.insert(pair<int,int>(2,13));
    fingers2JntsMap.insert(pair<int,int>(2,14));
    fingers2JntsMap.insert(pair<int,int>(3,15));
    fingers2JntsMap.insert(pair<int,int>(4,15));

    // start the thread with the specified period
    Time::turboBoost();
    setRate(period);
    start();

    // start the balancer thread
    armWaver=new ArmWavingMonitor(cartCtrl);
    armWaver->start();

    return configured=true;
}


/************************************************************************/
void ActionPrimitives::close()
{
    if (closed)
        return;

    if (armWaver!=NULL)
    {
        printMessage("stopping balancer thread ...\n");
        armWaver->stop();

        delete armWaver;
        armWaver=NULL;
    }

    if (isRunning())
    {
        printMessage("stopping main thread ...\n");
        stop();
    }

    if (polyHand.isValid() && polyCart.isValid())
        stopControl();

    if (polyHand.isValid())
    {
        printMessage("closing hand driver ...\n");
        polyHand.close();
    }

    if (polyCart.isValid())
    {
        printMessage("closing cartesian driver ...\n");
        cartCtrl->restoreContext(startup_context_id);
        polyCart.close();
    }

    delete graspModel;
    graspModel=NULL;

    actionsQueue.clear();

    closed=true;
}


/************************************************************************/
bool ActionPrimitives::isHandSeqEnded()
{
    // latch the current moving fingers set
    set<int> tmpSet=fingersMovingJntsSet;

    for (set<int>::iterator i=fingersMovingJntsSet.begin(); i!=fingersMovingJntsSet.end(); ++i)
    {
        if (handCheckMotionDone(*i))
            tmpSet.erase(*i);
    }

    // get data from the grasp model
    if (graspModel!=NULL)
    {
        Value out; graspModel->getOutput(out);
        Bottle *pB=out.asList();

        // span over fingers
        for (int fng=0; fng<5; fng++)
        {
            double val=pB->get(fng).asDouble();
            double thres=curGraspDetectionThres[fng];

            // detect contact on the finger
            if (val>thres)
            {
                fingersInPosition=false;

                // take joints belonging to the finger
                pair<multimap<int,int>::iterator,multimap<int,int>::iterator> i=fingers2JntsMap.equal_range(fng);
                
                for (multimap<int,int>::iterator j=i.first; j!=i.second; ++j)
                {
                    int jnt=j->second;

                    // stop and remove if not done yet
                    if (tmpSet.find(jnt)!=tmpSet.end())
                    {
                        printMessage("contact detected on finger %d: (%g>%g) => stopping joint %d\n",
                                     fng,val,thres,jnt);

                        posCtrl->stop(jnt);
                        tmpSet.erase(jnt);
                    }
                }                
            }
        }
    }

    // handle hand timeout
    if ((Time::now()-latchTimerHand)>curHandTmo)
    {
        printMessage("timeout (%g) expired on hand WP\n",curHandTmo);

        for (set<int>::iterator i=fingersMovingJntsSet.begin(); i!=fingersMovingJntsSet.end(); ++i)
        {
            posCtrl->stop(*i);
            tmpSet.erase(*i);
        }
    }

    // update the moving fingers set
    fingersMovingJntsSet=tmpSet;

    return (fingersMovingJntsSet.size()==0);
}


/************************************************************************/
void ActionPrimitives::postReachCallback()
{
    latchArmMoveDone=armMoveDone=false;
}


/************************************************************************/
bool ActionPrimitives::clearActionsQueue()
{
    if (configured)
    {
        mutex.lock();
        actionsQueue.clear();
        mutex.unlock();

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitives::lockActions()
{
    if (configured)
    {
        locked=true;
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitives::unlockActions()
{
    if (configured)
    {
        locked=false;
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitives::getActionsLockStatus() const
{
    return locked;
}


/************************************************************************/
bool ActionPrimitives::_pushAction(const bool execArm, const Vector &x, const Vector &o,
                                   const double execTime, const bool oEnabled, const bool execHand,
                                   const HandWayPoint &handWP, const bool handSeqTerminator,
                                   ActionPrimitivesCallback *clb)
{
    if (configured && !locked)
    {
        mutex.lock();
        Action action;
    
        action.waitState=false;
        action.execArm=execArm;
        action.x=x;
        action.o=o;
        action.execTime=execTime;
        action.oEnabled=oEnabled;
        action.execHand=execHand;
        action.handWP=handWP;
        action.handSeqTerminator=handSeqTerminator;
        action.execWayPoints=false;
        action.clb=clb;

        actionsQueue.push_back(action);
        mutex.unlock();

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitives::_pushAction(const Vector &x, const Vector &o,
                                   const string &handSeqKey, const double execTime,
                                   ActionPrimitivesCallback *clb, const bool oEnabled)
{
    if (configured)
    {
        map<string,deque<HandWayPoint> >::iterator itr=handSeqMap.find(handSeqKey);
        if (itr!=handSeqMap.end())
        {
            deque<HandWayPoint> &q=itr->second;
            if (q.size()>0)
            {
                Vector vectDummy(1);

                // combined action
                _pushAction(true,x,o,execTime,oEnabled,true,q[0],q.size()==1,q.size()==1?clb:NULL);

                if (q.size()>1)
                {
                    size_t i;

                    // decompose hand action in sum of fingers sequences
                    for (i=1; i<q.size()-1; i++)
                        _pushAction(false,vectDummy,vectDummy,ACTIONPRIM_DISABLE_EXECTIME,false,true,q[i],false,NULL);
    
                    // reserve the callback whenever the last hand WP is achieved
                    if (i<q.size())
                        _pushAction(false,vectDummy,vectDummy,ACTIONPRIM_DISABLE_EXECTIME,false,true,q[i],true,clb);
                }
            }

            return true;
        }
        else
        {
            printMessage("WARNING: \"%s\" hand sequence key not found\n",
                         handSeqKey.c_str());    

            return false;
        }
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitives::pushAction(const Vector &x, const Vector &o,
                                  const string &handSeqKey, const double execTime,
                                  ActionPrimitivesCallback *clb)
{
    return _pushAction(x,o,handSeqKey,execTime,clb,true);
}


/************************************************************************/
bool ActionPrimitives::pushAction(const Vector &x, const string &handSeqKey,
                                  const double execTime, ActionPrimitivesCallback *clb)
{
    Vector vectDummy(1);
    return _pushAction(x,vectDummy,handSeqKey,execTime,clb,false);
}


/************************************************************************/
bool ActionPrimitives::pushAction(const Vector &x, const Vector &o,
                                  const double execTime,
                                  ActionPrimitivesCallback *clb)
{
    if (configured)
    {
        HandWayPoint handDummy;

        _pushAction(true,x,o,execTime,true,false,handDummy,false,clb);

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitives::pushAction(const Vector &x, const double execTime,
                                  ActionPrimitivesCallback *clb)
{
    if (configured)
    {
        HandWayPoint handDummy;
        Vector vectDummy(1);

        _pushAction(true,x,vectDummy,execTime,false,false,handDummy,false,clb);

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitives::pushAction(const string &handSeqKey,
                                  ActionPrimitivesCallback *clb)
{
    if (configured)
    {
        map<string,deque<HandWayPoint> >::iterator itr=handSeqMap.find(handSeqKey);
        if (itr!=handSeqMap.end())
        {
            deque<HandWayPoint> &q=itr->second;
            Vector vectDummy(1);
            unsigned int i;

            // decompose hand action in sum of fingers sequences
            for (i=0; i<q.size()-1; i++)
                _pushAction(false,vectDummy,vectDummy,ACTIONPRIM_DISABLE_EXECTIME,false,true,q[i],false,NULL);

            // reserve the callback whenever the last hand WP is achieved
            if (i<q.size())
                _pushAction(false,vectDummy,vectDummy,ACTIONPRIM_DISABLE_EXECTIME,false,true,q[i],true,clb);

            return true;
        }
        else
        {
            printMessage("WARNING: \"%s\" hand sequence key not found\n",
                         handSeqKey.c_str());    

            return false;
        }
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitives::pushAction(const deque<ActionPrimitivesWayPoint> &wayPoints,
                                  ActionPrimitivesCallback *clb)
{
    if (configured && !locked)
    {
        mutex.lock();
        Action action;
        ArmWayPoints *thr=new ArmWayPoints(this,wayPoints);
        thr->set_default_exec_time(default_exec_time);

        action.waitState=false;
        action.execArm=false;
        action.execHand=false;
        action.handSeqTerminator=false;
        action.execWayPoints=true;
        action.wayPointsThr=thr;
        action.clb=clb;

        actionsQueue.push_back(action);
        mutex.unlock();

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitives::pushAction(const deque<ActionPrimitivesWayPoint> &wayPoints,
                                  const string &handSeqKey, ActionPrimitivesCallback *clb)
{
    if (configured && !locked)
    {
        mutex.lock();
        map<string,deque<HandWayPoint> >::iterator itr=handSeqMap.find(handSeqKey);
        if (itr!=handSeqMap.end())
        {
            deque<HandWayPoint> &q=itr->second;
            if (q.size()>0)
            {
                // combined action
                Action action;
                ArmWayPoints *thr=new ArmWayPoints(this,wayPoints);
                thr->set_default_exec_time(default_exec_time);

                action.waitState=false;
                action.execArm=false;
                action.execHand=true;
                action.handWP=q[0];
                action.handSeqTerminator=(q.size()==1);
                action.execWayPoints=true;
                action.wayPointsThr=thr;
                action.clb=(q.size()==1?clb:NULL);

                actionsQueue.push_back(action);                

                if (q.size()>1)
                {
                    Vector vectDummy(1);
                    size_t i;

                    // decompose hand action in sum of fingers sequences
                    for (i=1; i<q.size()-1; i++)
                        _pushAction(false,vectDummy,vectDummy,ACTIONPRIM_DISABLE_EXECTIME,false,true,q[i],false,NULL);
    
                    // reserve the callback whenever the last hand WP is achieved
                    if (i<q.size())
                        _pushAction(false,vectDummy,vectDummy,ACTIONPRIM_DISABLE_EXECTIME,false,true,q[i],true,clb);
                }
            }

            mutex.unlock();
            return true;
        }
        else
        {
            printMessage("WARNING: \"%s\" hand sequence key not found\n",
                         handSeqKey.c_str());    

            mutex.unlock();
            return false;
        }
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitives::pushWaitState(const double tmo, ActionPrimitivesCallback *clb)
{
    if (configured && !locked)
    {
        mutex.lock();
        Action action;

        action.waitState=true;
        action.tmo=tmo;
        action.execArm=false;
        action.execHand=false;
        action.handSeqTerminator=false;
        action.execWayPoints=false;        
        action.clb=clb;

        actionsQueue.push_back(action);
        mutex.unlock();

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitives::reachPose(const Vector &x, const Vector &o,
                                 const double execTime)
{
    if (configured && !locked)
    {
        disableArmWaving();

        const double t=execTime>0.0?execTime:default_exec_time;

        enableTorsoDof();

        cartCtrl->goToPose(x,o,t);

        printMessage("reach at %g [s] for [%s], [%s]\n",t,
                     x.toString(3,3).c_str(),
                     o.toString(3,3).c_str());

        postReachCallback();

        latchTimerReachLog=latchTimerReach=Time::now();

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitives::reachPosition(const Vector &x, const double execTime)
{
    if (configured && !locked)
    {
        disableArmWaving();

        const double t=execTime>0.0?execTime:default_exec_time;

        enableTorsoDof();

        cartCtrl->goToPosition(x,t);


        printMessage("reach at %g [s] for [%s]\n",t,
                     x.toString(3,3).c_str());

        postReachCallback();

        latchTimerReachLog=latchTimerReach=Time::now();

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitives::execQueuedAction()
{
    bool exec=false;
    Action action;

    mutex.lock();
    if (actionsQueue.size()>0)
    {
        action=actionsQueue.front();
        actionsQueue.pop_front();
        exec=true;
    }
    mutex.unlock();

    if (exec)
    {
        if (action.waitState)
            wait(action);

        if (action.execArm)
            cmdArm(action);

        if (action.execWayPoints)
        {
            actionWP=action.wayPointsThr;
            cmdArmWP(action);
        }
        else
            actionWP=NULL;

        if (action.execHand)
            cmdHand(action);

        actionClb=action.clb;
    }

    return exec;
}


/************************************************************************/
bool ActionPrimitives::execPendingHandSequences()
{
    bool exec=false;
    Action action;

    mutex.lock();
    if (actionsQueue.size()>0)
    {
        // polling on the first action in the queue
        action=actionsQueue.front();

        // if it is an hand-action then execute and update queue
        if (action.execHand && !action.execArm && !action.waitState)
        {    
            actionsQueue.pop_front();
            cmdHand(action);
            exec=true;
        }
    }
    mutex.unlock();

    return exec;
}


/************************************************************************/
void ActionPrimitives::run()
{
    const double t=Time::now();

    if (!armMoveDone)
    {
        Vector x,o,xdhat,odhat,qdhat;
        cartCtrl->getPose(x,o);
        cartCtrl->getDesired(xdhat,odhat,qdhat);

        if ((t-latchTimerReachLog)>ACTIONPRIM_DUMP_PERIOD)
        {
            printMessage("reaching... xdhat=[%s] |e|=%.3f [m]; odhat=[%s] |e|=%.3f\n",
                         xdhat.toString(3,3).c_str(),norm(xdhat-x),
                         odhat.toString(3,3).c_str(),norm(odhat-o));

            latchTimerReachLog=t;
        }

        if (actionWP!=NULL)
        {
            if (!actionWP->isRunning())
            {
                cartCtrl->checkMotionDone(&armMoveDone);
                delete actionWP;
                actionWP=NULL;
            }
        }
        else
            cartCtrl->checkMotionDone(&armMoveDone);

        // check if timeout has expired
        if (reachTmoEnabled && !armMoveDone)
        {
            if ((t-latchTimerReach)>reachTmo)
            {
                printMessage("timeout (%g) expired while reaching\n",reachTmo);
                armMoveDone=true;
            }
        }

        if (armMoveDone)
        {    
            printMessage("reaching complete\n");
            disableTorsoDof();
        }
    }

    if (!handMoveDone)
    {
        // check whether all the remaining active joints have come
        // to a complete stop
        handMoveDone=isHandSeqEnded();
        if (handMoveDone)
        {    
            printMessage("hand WP reached\n");

            if (!handSeqTerminator)
                if (execPendingHandSequences())     // here handMoveDone may switch false again
                    motionStartEvent.signal();
        }
    }

    latchArmMoveDone=armMoveDone;
    latchHandMoveDone=handMoveDone;

    if (latchArmMoveDone && latchHandMoveDone && (t-latchTimerWait>waitTmo))
    {    
        // execute action-end callback
        if (actionClb!=NULL)
        {
            printMessage("executing action-end callback ...\n");
            actionClb->exec();            
            printMessage("... action-end callback executed\n");

            actionClb=NULL;
        }

        if (execQueuedAction())
            motionStartEvent.signal();
        else
            motionDoneEvent.signal();
    }
}


/************************************************************************/
ActionPrimitives::~ActionPrimitives()
{
    close();
}


/************************************************************************/
bool ActionPrimitives::handCheckMotionDone(const int jnt)
{
    double fb;
    if (encCtrl->getEncoder(jnt,&fb))
        return (fabs(curHandFinalPoss[jnt-jHandMin]-fb)<curHandTols[jnt-jHandMin]);
    else
        return false;
}


/************************************************************************/
void ActionPrimitives::enableTorsoDof()
{
    // enable torso joints, if any
    if (!torsoActive && (norm(enableTorsoSw)>0.0))
    {
        Vector dummyRet;
        cartCtrl->setDOF(enableTorsoSw,dummyRet);
        torsoActive=true;
    }
}


/************************************************************************/
void ActionPrimitives::disableTorsoDof()
{
    // disable torso joints, if any
    if (torsoActive && (norm(enableTorsoSw)>0.0))
    {
        Vector dummyRet;
        cartCtrl->setDOF(disableTorsoSw,dummyRet);
        torsoActive=false;
    }
}


/************************************************************************/
bool ActionPrimitives::wait(const Action &action)
{
    if (configured)
    {        
        printMessage("wait for %g seconds\n",action.tmo);
        waitTmo=action.tmo;
        latchTimerWait=Time::now();

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitives::cmdArm(const Action &action)
{
    if (configured)
    {
        disableArmWaving();

        const Vector &x=action.x;
        const Vector &o=action.o;
        const bool    oEnabled=action.oEnabled;
        const double  t=action.execTime>0.0?action.execTime:default_exec_time;

        enableTorsoDof();

        if (oEnabled)
        {
            if (!cartCtrl->goToPoseSync(x,o,t))
            {
                printMessage("reach error\n");
                return false;
            }
            
            printMessage("reach at %g [s] for [%s], [%s]\n",t,
                         x.toString(3,3).c_str(),
                         o.toString(3,3).c_str());
        }
        else
        {
            if (!cartCtrl->goToPositionSync(x,t))
            {
                printMessage("reach error\n");
                return false;
            }

            printMessage("reach at %g [s] for [%s]\n",t,
                         x.toString(3,3).c_str());
        }

        postReachCallback();

        latchTimerReachLog=latchTimerReach=Time::now();

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitives::cmdArmWP(const Action &action)
{
    if (configured && action.execWayPoints)
    {
        disableArmWaving();
        enableTorsoDof();
        action.wayPointsThr->start();
        postReachCallback();
        latchTimerReachLog=latchTimerReach=Time::now();

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitives::cmdHand(const Action &action)
{
    if (configured)
    {
        disableArmWaving();

        const string &tag=action.handWP.tag;
        const Vector &poss=action.handWP.poss;
        const Vector &vels=action.handWP.vels;
        const Vector &tols=action.handWP.tols;
        const Vector &thres=action.handWP.thres;
        const double &tmo=action.handWP.tmo;
        
        fingersMovingJntsSet=fingersJntsSet;
        curHandFinalPoss=poss;
        curHandTols=tols;
        curGraspDetectionThres=thres;
        curHandTmo=tmo;

        size_t sz=std::min(fingersJnts.size(),std::min(poss.length(),vels.length()));
        for (size_t i=0; i<sz; i++)
        {
            size_t j=fingersJnts[i];
            modCtrl->setControlMode(j,VOCAB_CM_POSITION);
            posCtrl->setRefSpeed(j,vels[j-jHandMin]);
        }
        
        posCtrl->positionMove(sz,fingersJnts.getFirst(),poss.data());

        latchHandMoveDone=handMoveDone=false;
        handSeqTerminator=action.handSeqTerminator;
        fingersInPosition=true;
        printMessage("\"%s\" WP: [%s] (thres = [%s]) (tmo = %g)\n",
                     tag.c_str(),poss.toString(3,3).c_str(),
                     thres.toString(3,3).c_str(),tmo);

        latchTimerHand=Time::now();

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitives::addHandSeqWP(const string &handSeqKey, const Vector &poss,
                                    const Vector &vels, const Vector &tols, const Vector &thres,
                                    const double tmo)
{
    if ((poss.length()==9) && (vels.length()==9) && (tols.length()==9) && (thres.length()==5))
    {
        HandWayPoint handWP;

        handWP.tag=handSeqKey;
        handWP.poss=poss;
        handWP.vels=vels;
        handWP.tols=tols;
        handWP.thres=thres;
        handWP.tmo=tmo;
    
        handSeqMap[handSeqKey].push_back(handWP);

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitives::addHandSequence(const string &handSeqKey, const Bottle &sequence)
{
    Bottle &bSeq=const_cast<Bottle&>(sequence);

    if (!bSeq.check("numWayPoints"))
    {
        printMessage("WARNING: \"numWayPoints\" option is missing\n");
        return false;
    }

    int numWayPoints=bSeq.find("numWayPoints").asInt();
    bool ret=false;

    for (int j=0; j<numWayPoints; j++)
    {
        ostringstream wp;
        wp<<"wp_"<<j;

        Bottle &bWP=bSeq.findGroup(wp.str().c_str());
        if (bWP.isNull())
        {
            printMessage("WARNING: \"%s\" entry is missing\n",wp.str().c_str());
            return false;
        }

        if (!bWP.check("poss"))
        {
            printMessage("WARNING: \"poss\" option is missing\n");
            return false;
        }

        if (!bWP.check("vels"))
        {
            printMessage("WARNING: \"vels\" option is missing\n");
            return false;
        }

        if (!bWP.check("tols"))
        {
            printMessage("WARNING: \"tols\" option is missing\n");
            return false;
        }

        if (!bWP.check("thres"))
        {
            printMessage("WARNING: \"thres\" option is missing\n");
            return false;
        }

        if (!bWP.check("tmo"))
        {
            printMessage("WARNING: \"tmo\" option is missing\n");
            return false;
        }

        Bottle *bPoss=bWP.find("poss").asList();
        Vector poss(bPoss->size());

        for (size_t k=0; k<poss.length(); k++)
            poss[k]=bPoss->get(k).asDouble();

        Bottle *bVels=bWP.find("vels").asList();
        Vector vels(bVels->size());

        for (size_t k=0; k<vels.length(); k++)
            vels[k]=bVels->get(k).asDouble();

        Bottle *bTols=bWP.find("tols").asList();
        Vector tols(bTols->size());

        for (size_t k=0; k<tols.length(); k++)
            tols[k]=bTols->get(k).asDouble();

        Bottle *bThres=bWP.find("thres").asList();
        Vector thres(bThres->size());

        for (size_t k=0; k<thres.length(); k++)
            thres[k]=bThres->get(k).asDouble();

        double tmo=bWP.find("tmo").asDouble();

        if (addHandSeqWP(handSeqKey,poss,vels,tols,thres,tmo))
            ret=true;   // at least one WP has been added
        else
            printMessage("WARNING: \"%s\" entry is invalid, not added to \"%s\"\n",
                         wp.str().c_str(),handSeqKey.c_str());
    }

    return ret;
}


/************************************************************************/
bool ActionPrimitives::isValidHandSeq(const string &handSeqKey)
{
    map<string,deque<HandWayPoint> >::iterator itr=handSeqMap.find(handSeqKey);

    if (itr!=handSeqMap.end())
        return true;
    else
        return false;
}


/************************************************************************/
bool ActionPrimitives::removeHandSeq(const string &handSeqKey)
{
    map<string,deque<HandWayPoint> >::iterator itr=handSeqMap.find(handSeqKey);

    if (itr!=handSeqMap.end())
    {    
        handSeqMap[handSeqKey].clear();
        return true;
    }
    else
        return false;
}


/************************************************************************/
deque<string> ActionPrimitives::getHandSeqList()
{
    map<string,deque<HandWayPoint> >::iterator itr;
    deque<string> q;

    for (itr=handSeqMap.begin(); itr!=handSeqMap.end(); ++itr)
        q.push_back(itr->first);

    return q;
}


/************************************************************************/
bool ActionPrimitives::getHandSequence(const string &handSeqKey, Bottle &sequence)
{
    if (isValidHandSeq(handSeqKey))
    {
        deque<HandWayPoint> &handWP=handSeqMap[handSeqKey];
        sequence.clear();

        // numWayPoints part
        Bottle &bNum=sequence.addList();
        bNum.addString("numWayPoints");
        bNum.addInt(handWP.size());
        
        // wayPoints parts
        for (unsigned int i=0; i<handWP.size(); i++)
        {
            ostringstream wp;
            wp<<"wp_"<<i;

            Bottle &bWP=sequence.addList();
            bWP.addString(wp.str().c_str());

            // poss part
            Bottle &bPoss=bWP.addList();
            bPoss.addString("poss");
            Bottle &bPossVects=bPoss.addList();
            for (size_t j=0; j<handWP[i].poss.length(); j++)
                bPossVects.addDouble(handWP[i].poss[j]);

            // vels part
            Bottle &bVels=bWP.addList();
            bVels.addString("vels");
            Bottle &bVelsVects=bVels.addList();
            for (size_t j=0; j<handWP[i].vels.length(); j++)
                bVelsVects.addDouble(handWP[i].vels[j]);

            // tols part
            Bottle &bTols=bWP.addList();
            bTols.addString("tols");
            Bottle &bTolsVects=bTols.addList();
            for (size_t j=0; j<handWP[i].tols.length(); j++)
                bTolsVects.addDouble(handWP[i].tols[j]);

            // thres part
            Bottle &bThres=bWP.addList();
            bThres.addString("thres");
            Bottle &bThresVects=bThres.addList();
            for (size_t j=0; j<handWP[i].thres.length(); j++)
                bThresVects.addDouble(handWP[i].thres[j]);

            // tmo part
            Bottle &bTmo=bWP.addList();
            bTmo.addString("tmo");
            bTmo.addDouble(handWP[i].tmo);
        }       

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitives::areFingersMoving(bool &f) const
{
    if (configured)
    {
        f=latchHandMoveDone;
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitives::areFingersInPosition(bool &f) const
{
    if (configured)
    {
        f=fingersInPosition;
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitives::getGraspModel(Model *&model) const
{
    if (configured)
    {
        model=static_cast<Model*>(graspModel);
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitives::getCartesianIF(ICartesianControl *&ctrl) const
{
    if (configured)
    {
        ctrl=cartCtrl;
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitives::getTorsoJoints(Vector &torso)
{
    if (configured)
    {
        torso=enableTorsoSw;
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitives::setTorsoJoints(const Vector &torso)
{
    if (configured)
    {
        size_t len=std::min(torso.length(),size_t(3));
        for (size_t i=0; i<len; i++)
            enableTorsoSw[i]=torso[i];

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitives::getPose(Vector &x, Vector &o) const
{
    if (configured)
    {
        cartCtrl->getPose(x,o);
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitives::stopControl()
{
    if (configured)
    {
        suspend();

        if (actionWP!=NULL)
        {
            if (actionWP->isRunning())
                actionWP->stop();

            delete actionWP;
            actionWP=NULL;
        }

        clearActionsQueue();

        cartCtrl->stopControl();
        posCtrl->stop(fingersJnts.size(),fingersJnts.getFirst());

        armMoveDone =latchArmMoveDone =true;
        handMoveDone=latchHandMoveDone=true;

        resume();

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitives::setTrackingMode(const bool f)
{
    if (configured)
    {
        if (cartCtrl->setTrackingMode(f))
        {
            tracking_mode=f;
            return true;
        }
        else
            return false;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitives::enableArmWaving(const Vector &restPos)
{
    if (configured)
    {
        RES_WAVER(armWaver)->setRestPosition(restPos);
        printMessage("setting waving position to %s\n",
                     restPos.toString(3,3).c_str());

        if (RES_WAVER(armWaver)->enable())
            printMessage("arm waving enabled\n");

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitives::disableArmWaving()
{
    if (configured)
    {
        if (RES_WAVER(armWaver)->disable())
            printMessage("arm waving disabled\n");

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitives::enableReachingTimeout(const double tmo)
{
    if (configured)
    {
        reachTmo=tmo;
        reachTmoEnabled=true;

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitives::disableReachingTimeout()
{
    if (configured)
    {
        reachTmoEnabled=false;
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitives::getTrackingMode() const
{
    return tracking_mode;
}


/************************************************************************/
bool ActionPrimitives::checkActionsDone(bool &f, const bool sync)
{
    if (configured)
    {
        if (sync && checkEnabled)
        {
            motionDoneEvent.reset();
            motionDoneEvent.wait();
        }

        f=latchArmMoveDone && latchHandMoveDone;

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitives::checkActionOnGoing(bool &f, const bool sync)
{
    if (configured)
    {
        if (sync && checkEnabled)
        {
            motionStartEvent.reset();
            motionStartEvent.wait();
        }

        f=!latchArmMoveDone || !latchHandMoveDone;

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitives::syncCheckInterrupt(const bool disable)
{
    if (configured)
    {
        motionDoneEvent.signal();

        if (disable)
            checkEnabled=false;

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitives::syncCheckReinstate()
{
    if (configured)
    {
        checkEnabled=true;
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitivesLayer1::grasp(const Vector &x, const Vector &o, const Vector &d)
{
    if (configured)
    {
        printMessage("start grasping\n");

        pushAction(x+d,o,"open_hand");
        pushAction(x,o);
        pushAction("close_hand");

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitivesLayer1::touch(const Vector &x, const Vector &o, const Vector &d)
{
    if (configured)
    {
        printMessage("start touching\n");

        pushAction(x+d,o,"karate_hand");
        pushAction(x,o);

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitivesLayer1::tap(const Vector &x1, const Vector &o1,
                                 const Vector &x2, const Vector &o2,
                                 const double execTime)
{
    if (configured)
    {
        printMessage("start tapping\n");

        pushAction(x1,o1,"karate_hand");
        pushAction(x2,o2,execTime);
        pushAction(x1,o1);

        return true;
    }
    else
        return false;
}


/************************************************************************/
void liftAndGraspCallback::exec()
{
    // lift up the hand iff contact detected
    if (action->contactDetected)
    {
        Vector x,o;
        action->cartCtrl->getPose(x,o);
        action->printMessage("logged 3-d pos: [%s]\n",
                             x.toString(3,3).c_str());
    
        action->pushAction(x+action->grasp_d2,action->grasp_o);
    }

    action->disableContactDetection();
    action->pushAction("close_hand");
}


/************************************************************************/
void touchCallback::exec()
{
    // just disable the contact detection
    action->disableContactDetection();
}


/************************************************************************/
ActionPrimitivesLayer2::ActionPrimitivesLayer2()
{
    init();
}


/************************************************************************/
ActionPrimitivesLayer2::ActionPrimitivesLayer2(Property &opt) :
                        ActionPrimitivesLayer1(opt)
{
    init();
    skipFatherPart=true;
    open(opt);
}


/************************************************************************/
void ActionPrimitivesLayer2::init()
{    
    skipFatherPart=false;
    configuredLayer2=false;
    contactDetectionOn=false;
    contactDetected=false;

    execLiftAndGrasp=NULL;
    execTouch=NULL;
}


/************************************************************************/
void ActionPrimitivesLayer2::postReachCallback()
{
    // init the contact variable
    contactDetected=false;

    // call the main postReachCallback()
    ActionPrimitivesLayer1::postReachCallback();
}


/************************************************************************/
void ActionPrimitivesLayer2::run()
{
    // skip until this layer is configured
    if (!configuredLayer2)
        return;    

    // get the input from WBDYN
    if (Vector *wbdynWrench=wbdynPortIn.read(false))
    {
        size_t len=std::min(wbdynWrench->length(),wrenchExternal.length());
        for (size_t i=0; i<len; i++)
            wrenchExternal[i]=(*wbdynWrench)[i];
    }

    Vector forceExternal=wrenchExternal.subVector(0,2);
    const double forceExternalAbs=norm(forceExternal);

    // stop the arm iff contact detected while reaching
    if (!armMoveDone && contactDetectionOn && (forceExternalAbs>ext_force_thres))
    {
        if (actionWP!=NULL)
        {
            if (actionWP->isRunning())
                actionWP->stop();

            delete actionWP;
            actionWP=NULL;
        }

        cartCtrl->stopControl();

        printMessage("contact detected on arm: external force [%s], (%g>%g) => stopping arm\n",
                     forceExternal.toString(3,3).c_str(),forceExternalAbs,ext_force_thres);

        disableTorsoDof();

        armMoveDone=true;
        contactDetected=true;
    }

    // call the main run()
    // the order does matter
    ActionPrimitivesLayer1::run();
}


/************************************************************************/
bool ActionPrimitivesLayer2::open(Property &opt)
{
    if (!skipFatherPart)
        ActionPrimitivesLayer1::open(opt);

    if (configuredLayer2)
    {
        printMessage("WARNING: already configured\n");
        return true;
    }

    if (configured)
    {
        ext_force_thres=opt.check("ext_force_thres",Value(ACTIONPRIM_DEFAULT_EXT_FORCE_THRES)).asDouble();
        string wbdynStemName=opt.check("wbdyn_stem_name",Value(ACTIONPRIM_DEFAULT_WBDYN_STEMNAME)).asString().c_str();
        string wbdynPortName=opt.check("wbdyn_port_name",Value(ACTIONPRIM_DEFAULT_WBDYN_PORTNAME)).asString().c_str();

        // connect automatically to WTBO
        string wbdynServerName="/"+wbdynStemName+"/"+part+"/"+wbdynPortName;
        wbdynPortIn.open(("/"+local+"/"+part+"/wbdyn:i").c_str());
        if (!Network::connect(wbdynServerName.c_str(),wbdynPortIn.getName().c_str(),"udp"))
        {
            printMessage("ERROR: unable to connect to port %s\n",wbdynServerName.c_str());

            close();
            return false;
        }

        // create callbacks
        execLiftAndGrasp=new liftAndGraspCallback(this);
        execTouch=new touchCallback(this);

        wrenchExternal.resize(6,0.0);

        return configuredLayer2=true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitivesLayer2::isValid() const
{
    return (ActionPrimitivesLayer1::isValid() && configuredLayer2);
}


/************************************************************************/
void ActionPrimitivesLayer2::close()
{
    if (closed)
        return;

    // call the main close()
    // the order does matter
    ActionPrimitivesLayer1::close();

    if (!wbdynPortIn.isClosed())
    {
        wbdynPortIn.interrupt();
        wbdynPortIn.close();
    }

    if (execLiftAndGrasp!=NULL)
    {
        delete execLiftAndGrasp;
        execLiftAndGrasp=NULL;
    }

    if (execTouch!=NULL)
    {
        delete execTouch;
        execTouch=NULL;
    }
}


/************************************************************************/
bool ActionPrimitivesLayer2::grasp(const Vector &x, const Vector &o,
                                   const Vector &d1, const Vector &d2)
{
    if (configured)
    {
        printMessage("start grasping\n");

        enableContactDetection();

        pushAction(x+d1,o,"open_hand");
        pushAction(x,o,ACTIONPRIM_DISABLE_EXECTIME,execLiftAndGrasp);
        // the remaining part is done in the callback

        // save data
        grasp_d2=d2;
        grasp_o=o;

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitivesLayer2::grasp(const Vector &x, const Vector &o,
                                   const Vector &d)
{
    return ActionPrimitivesLayer1::grasp(x,o,d);
}


/************************************************************************/
bool ActionPrimitivesLayer2::touch(const Vector &x, const Vector &o, const Vector &d)
{
    if (configured)
    {
        printMessage("start touching\n");

        enableContactDetection();

        pushAction(x+d,o,"karate_hand");
        pushAction(x,o,ACTIONPRIM_DISABLE_EXECTIME,execTouch);

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitivesLayer2::getExtWrench(Vector &wrench) const
{
    if (configured)
    {
        wrench=wrenchExternal;
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitivesLayer2::getExtForceThres(double &thres) const
{
    if (configured)
    {
        thres=ext_force_thres;
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitivesLayer2::setExtForceThres(const double thres)
{
    if (configured)
    {
        ext_force_thres=thres;
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitivesLayer2::enableContactDetection()
{
    if (configured)
    {
        contactDetectionOn=true;
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitivesLayer2::disableContactDetection()
{
    if (configured)
    {
        contactDetectionOn=false;
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitivesLayer2::isContactDetectionEnabled(bool &f) const
{
    if (configured)
    {
        f=contactDetectionOn;
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ActionPrimitivesLayer2::checkContact(bool &f) const
{
    if (configured)
    {
        f=contactDetected;
        return true;
    }
    else
        return false;
}


/************************************************************************/
ActionPrimitivesLayer2::~ActionPrimitivesLayer2()
{
    close();
}



