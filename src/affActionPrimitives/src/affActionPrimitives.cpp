
#include <ace/Auto_Event.h>
#include <yarp/math/Math.h>
#include <yarp/os/Time.h>

#include <gsl/gsl_math.h>

#include <iCub/common.h>
#include <iCub/affActionPrimitives.h>

#include <stdio.h>
#include <string>

#define RES_EVENT(x)                    (static_cast<ACE_Auto_Event*>(x))

#define ACTIONPRIM_DEFAULT_PER          50      // [ms]
#define ACTIONPRIM_DEFAULT_TRAJTIME     1.5     // [s]
#define ACTIONPRIM_DEFAULT_REACHTOL     0.02    // [m]
#define ACTIONPRIM_DUMP_PERIOD          2.0     // [s]

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;


/************************************************************************/
affActionPrimitives::affActionPrimitives() :
                     RateThread(ACTIONPRIM_DEFAULT_PER)
{
    init();
}


/************************************************************************/
affActionPrimitives::affActionPrimitives(Property &opt) :
                     RateThread(ACTIONPRIM_DEFAULT_PER)
{
    init();
    open(opt);
}


/************************************************************************/
void affActionPrimitives::init()
{
    polyHand=polyCart=NULL;

    handMetrics=NULL;
    fs=NULL;

    mutex=NULL;
    motionDoneEvent=NULL;

    armMoveDone =latchArmMoveDone =true;
    handMoveDone=latchHandMoveDone=true;
    configured=closed=false;
    checkEnabled=true;

    latchTimer=waitTmo=0.0;
}


/************************************************************************/
bool affActionPrimitives::isValid()
{
    return configured;
}


/************************************************************************/
bool affActionPrimitives::handleTorsoDOF(Property &opt, const string &key,
                                         const int j)
{
    if (opt.check(key.c_str()))
    {
        bool sw=opt.find(key.c_str()).asString()=="on"?true:false;

        Vector newDof, dummyRet;
        newDof.resize(3,2);
        newDof[j]=sw?1:0;

        fprintf(stdout,"%s %s\n",key.c_str(),sw?"enabled":"disabled");
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

            fprintf(stdout,"%s limits: [%g,%g] deg\n",key.c_str(),min,max);
            
        }

        return true;
    }

    return false;
}


/************************************************************************/
bool affActionPrimitives::configHandSeq(Property &opt)
{
    if (opt.check("hand_sequences_file"))
    {
        string handSeqFile=opt.find("hand_sequences_file").asString().c_str();
        Property handSeqProp;

        fprintf(stdout,"Processing %s file\n",handSeqFile.c_str());
        handSeqProp.fromConfigFile(handSeqFile.c_str());
    
        // GENERAL group
        Bottle &bGeneral=handSeqProp.findGroup("GENERAL");
        if (bGeneral.isNull())
        {
            fprintf(stdout,"WARNING: \"GENERAL\" group is missing\n");    
            return false;
        }

        if (!bGeneral.check("numSequences"))
        {
            fprintf(stdout,"WARNING: \"numSequences\" option is missing\n");    
            return false;
        }

        int numSequences=bGeneral.find("numSequences").asInt();

        // SEQUENCE groups
        for (int i=0; i<numSequences; i++)
        {
            char seq[255];
            sprintf(seq,"SEQ_%d",i);

            Bottle bSeq=handSeqProp.findGroup(seq);
            if (bSeq.isNull())
            {
                fprintf(stdout,"WARNING: \"%s\" group is missing\n",seq);    
                return false;
            }

            if (!bSeq.check("key"))
            {
                fprintf(stdout,"WARNING: \"key\" option is missing\n");    
                return false;
            }

            string key=bSeq.find("key").asString().c_str();

            if (!bSeq.check("numWayPoints"))
            {
                fprintf(stdout,"WARNING: \"numWayPoints\" option is missing\n");    
                return false;
            }

            int numWayPoints=bSeq.find("numWayPoints").asInt();

            for (int j=0; j<numWayPoints; j++)
            {
                char wp[255];
                sprintf(wp,"wp_%d",j);

                Bottle bWP=bSeq.findGroup(wp);
                if (bWP.isNull())
                {
                    fprintf(stdout,"WARNING: \"%s\" entry is missing\n",wp);    
                    return false;
                }

                if (!bWP.check("poss"))
                {
                    fprintf(stdout,"WARNING: \"poss\" option is missing\n");    
                    return false;
                }

                if (!bWP.check("vels"))
                {
                    fprintf(stdout,"WARNING: \"vels\" option is missing\n");    
                    return false;
                }

                Bottle *bPoss=bWP.find("poss").asList();
                Vector poss(bPoss->size());

                for (int k=0; k<poss.length(); k++)
                    poss[k]=bPoss->get(k).asDouble();

                Bottle *bVels=bWP.find("vels").asList();
                Vector vels(bVels->size());

                for (int k=0; k<vels.length(); k++)
                    vels[k]=bVels->get(k).asDouble();

                addHandSeqWP(key,poss,vels);
            }
        }
    
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool affActionPrimitives::open(Property &opt)
{
    if (!opt.check("local"))
    {
        fprintf(stdout,"ERROR: option \"local\" is missing\n");
        return false;
    }

    if (!opt.check("hand_calibration_file"))
    {
        fprintf(stdout,"ERROR: option \"hand_calibration_file\" is missing\n");
        return false;
    }

    string robot=opt.check("robot",Value("icub")).asString().c_str();
    string part=opt.check("part",Value("right_arm")).asString().c_str();
    int period=opt.check("thread_period",Value(ACTIONPRIM_DEFAULT_PER)).asInt();
    double traj_time=opt.check("traj_time",Value(ACTIONPRIM_DEFAULT_TRAJTIME)).asDouble();
    double reach_tol=opt.check("reach_tol",Value(ACTIONPRIM_DEFAULT_REACHTOL)).asDouble();
    string local=opt.find("local").asString().c_str();
    string sensingCalibFile=opt.find("hand_calibration_file").asString().c_str();
    string fwslash="/";

    // get params from config file
    Property sensingCalibProp;
    sensingCalibProp.fromConfigFile(sensingCalibFile.c_str());
    Bottle &sensingCalib=sensingCalibProp.findGroup(part.c_str());

    if (sensingCalib.isNull())
    {
        close();
        return false;
    }

    // get hand sequence motions (if any)
    configHandSeq(opt);

    // open the position client
    Property optPolyHand("(device remote_controlboard)");
    optPolyHand.put("remote",(fwslash+robot+fwslash+part).c_str());
    optPolyHand.put("local",(fwslash+local+fwslash+part+fwslash+"position").c_str());

    polyHand=new PolyDriver;
    if (!polyHand->open(optPolyHand))
    {
        close();    
        return false;
    }

    // open the cartesian client
    Property optPolyCart("(device cartesiancontrollerclient)");
    optPolyCart.put("remote",(fwslash+robot+fwslash+"cartesianController"+fwslash+part).c_str());
    optPolyCart.put("local",(fwslash+local+fwslash+part+fwslash+"cartesian").c_str());

    polyCart=new PolyDriver;
    if (!polyCart->open(optPolyCart))
    {
        close();    
        return false;
    }

    // open views
    polyHand->view(encCtrl);
    polyHand->view(pidCtrl);
    polyHand->view(ampCtrl);
    polyHand->view(posCtrl);
    polyCart->view(cartCtrl);

    jHandMin=7;                     // hand first joint
    posCtrl->getAxes(&jHandMax);    // hand last joint

    for (int j=jHandMin; j<jHandMax; j++)
        enabledJoints.insert(j);

    // set trajectory time
    cartCtrl->setTrajTime(traj_time);

    // set tolerance
    cartCtrl->setInTargetTol(reach_tol);

    // set one shot mode
    cartCtrl->setTrackingMode(false);

    // handle torso DOF's
    handleTorsoDOF(opt,"torso_pitch",0);
    handleTorsoDOF(opt,"torso_roll",1);
    handleTorsoDOF(opt,"torso_yaw",2);

    // create hand metrix
    readMatrices(sensingCalib,sensingConstants);
    thresholds=sensingConstants["thresholds"].getRow(0);

    fprintf(stdout,"creating hand metrics...\n");
    handMetrics=new HandMetrics(encCtrl,pidCtrl,ampCtrl,sensingConstants);

    fprintf(stdout,"creating hand smoother...\n");
    fs=new FunctionSmoother(thresholds);

    mutex=new Semaphore(1);
    motionDoneEvent=new ACE_Auto_Event;

    // start the thread with the specified period
    Time::turboBoost();
    setRate(period);
    start();

    return configured=true;
}


/************************************************************************/
void affActionPrimitives::close()
{
    if (closed)
        return;

    if (polyHand!=NULL && polyCart!=NULL)
        stopControl();

    if (isRunning())
    {
        fprintf(stdout,"stopping thread ...\n");
        stop();
    }

    if (fs!=NULL)
    {    
        fprintf(stdout,"disposing hand smoother ...\n");
        delete fs;
    }

    if (handMetrics!=NULL)
    {
        fprintf(stdout,"disposing hand metrics ...\n");
        delete handMetrics;
    }

    if (polyHand!=NULL)
    {
        fprintf(stdout,"closing hand driver ...\n");
        delete polyHand;
    }

    if (polyCart!=NULL)
    {
        fprintf(stdout,"closing cartesian driver ...\n"); 
        delete polyCart;
    }

    if (mutex!=NULL)
        delete mutex;

    if (motionDoneEvent!=NULL)
        delete RES_EVENT(motionDoneEvent);

    closed=true;

    fprintf(stdout,"closing complete!\n");
}


/************************************************************************/
void affActionPrimitives::stopBlockedJoints(set<int> &activeJoints)
{
	Vector smoothedError;	
	fs->smooth(handMetrics->getError(),smoothedError,handMetrics->getTimeInterval());

	for (set<int>::const_iterator itr=enabledJoints.begin(); itr!=enabledJoints.end(); itr++)
    {
		int i=*itr;

		if (i>=thresholds.length() || i<0)
            fprintf(stdout,"WARNING: No thresholds for joint #%d specified.\n",i);
        else 
        {
			bool isOpening=thresholds[i]>0;

			if ((isOpening && smoothedError[i]>thresholds[i]) || (!isOpening && smoothedError[i]<thresholds[i])) 
            {
				posCtrl->stop(i);
				activeJoints.erase(i);
                fprintf(stdout,"joint #%d blocked\n",i);
			}
		}
	}
}


/************************************************************************/
bool affActionPrimitives::handMotionDone(const set<int> &joints)
{
	Vector v=handMetrics->getVelocity();

	for (set<int>::const_iterator itr=joints.begin(); itr!=joints.end(); itr++)
    {
		bool b;
		posCtrl->checkMotionDone(*itr,&b);

        if (fabs(v[*itr])>0.01 && !b)
            return false;
	}

	return true;
}


/************************************************************************/
bool affActionPrimitives::clearActionsQueue()
{
    if (configured)
    {
        mutex->wait();
        actionsQueue.clear();
        mutex->post();

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool affActionPrimitives::pushAction(const bool execArm, const Vector &x, const Vector &o,
                                     const bool execHand, const HandWayPoint &handWP)
{
    if (configured)
    {
        mutex->wait();
        Action action;
    
        action.waitState=false;
        action.execArm=execArm;
        action.x=x;
        action.o=o;
        action.execHand=execHand;
        action.handWP=handWP;
    
        actionsQueue.push_back(action);
        mutex->post();

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool affActionPrimitives::pushAction(const Vector &x, const Vector &o,
                                     const string &handSeqKey)
{
    if (configured)
    {
        map<string,deque<HandWayPoint> >::iterator itr=handSeqMap.find(handSeqKey);
        if (itr!=handSeqMap.end())
        {
            deque<HandWayPoint> &q=itr->second;

            for (size_t i=0; i<q.size(); i++)
                pushAction(true,x,o,true,q[i]);

            return true;
        }
        else
        {
            fprintf(stdout,"WARNING: \"%s\" hand sequence key not found\n",
                    handSeqKey.c_str());    

            return false;
        }
    }
    else
        return false;
}


/************************************************************************/
bool affActionPrimitives::pushAction(const Vector &x, const Vector &o)
{
    if (configured)
    {
        HandWayPoint dummy;
        pushAction(true,x,o,false,dummy);

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool affActionPrimitives::pushAction(const string &handSeqKey)
{
    if (configured)
    {
        map<string,deque<HandWayPoint> >::iterator itr=handSeqMap.find(handSeqKey);
        if (itr!=handSeqMap.end())
        {
            deque<HandWayPoint> &q=itr->second;
            Vector dummy(1);

            for (size_t i=0; i<q.size(); i++)
                pushAction(false,dummy,dummy,true,q[i]);

            return true;
        }
        else
        {
            fprintf(stdout,"WARNING: \"%s\" hand sequence key not found\n",
                    handSeqKey.c_str());    

            return false;
        }
    }
    else
        return false;
}


/************************************************************************/
bool affActionPrimitives::pushWaitState(const double tmo)
{
    if (configured)
    {
        mutex->wait();
        Action action;
        Vector dummy(1);

        action.waitState=true;
        action.tmo=tmo;
        action.execArm=false;
        action.execHand=false;

        actionsQueue.push_back(action);
        mutex->post();

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool affActionPrimitives::execQueuedAction()
{
    bool exec=false;
    Action action;

    mutex->wait();
    if (actionsQueue.size())
    {
        action=actionsQueue.front();
        actionsQueue.pop_front();
        exec=true;
    }
    mutex->post();

    if (exec)
    {
        if (action.waitState)
            wait(action.tmo);

        if (action.execArm)
            cmdArm(action.x,action.o);

        if (action.execHand)
            cmdHand(action.handWP);
    }

    return exec;
}


/************************************************************************/
void affActionPrimitives::run()
{
    const double t=Time::now();

    if (!armMoveDone)
    {
        Vector x,o,xdcap,odcap,qdcap;
        cartCtrl->getPose(x,o);
        cartCtrl->getDesired(xdcap,odcap,qdcap);

        if (t-t0>ACTIONPRIM_DUMP_PERIOD)
        {
            fprintf(stdout,"reaching... xdcap=%s |e|=%.3f [m]\n",
                    xdcap.toString().c_str(),norm(xdcap-x));

            t0=t;
        }

        cartCtrl->checkMotionDone(&armMoveDone);

        if (armMoveDone)
            fprintf(stdout,"reaching complete\n");            
    }

    if (!handMoveDone)
    {
        handMetrics->snapshot();
        stopBlockedJoints(activeJoints);

        // check whether all the remaining active joints have come
        // to a complete stop
        handMoveDone=handMotionDone(activeJoints);

        if (handMoveDone)
            fprintf(stdout,"hand WP reached\n");
    }

    latchArmMoveDone=armMoveDone;
    latchHandMoveDone=handMoveDone;

    if (latchArmMoveDone && latchHandMoveDone && (t-latchTimer>waitTmo))
        if (!execQueuedAction())
            RES_EVENT(motionDoneEvent)->signal();
}


/************************************************************************/
affActionPrimitives::~affActionPrimitives()
{
    close();
}


/************************************************************************/
bool affActionPrimitives::wait(const double tmo)
{
    if (configured)
    {        
        fprintf(stdout,"wait for %g seconds\n",tmo);
        waitTmo=tmo;
        latchTimer=Time::now();

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool affActionPrimitives::cmdArm(const Vector &x, const Vector &o)
{
    if (configured)
    {
        if (!cartCtrl->goToPoseSync(x,o))
        {
            fprintf(stdout,"reach error\n");
            return false;
        }
    
        latchArmMoveDone=armMoveDone=false;
        fprintf(stdout,"reach for [%s], [%s]\n",const_cast<Vector&>(x).toString().c_str(),
                                                const_cast<Vector&>(o).toString().c_str());
        t0=Time::now();
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool affActionPrimitives::cmdHand(const HandWayPoint &handWP)
{
    if (configured)
    {        
        activeJoints=enabledJoints;
        for (set<int>::const_iterator itr=enabledJoints.begin(); itr!=enabledJoints.end(); itr++)
        {   
            int j=*itr-jHandMin;

            if (j>=handWP.poss.length() || j>=handWP.vels.length())
                break;

            posCtrl->setRefSpeed(*itr,handWP.vels[j]);
            posCtrl->positionMove(*itr,handWP.poss[j]);
        }

        latchHandMoveDone=handMoveDone=false;
        fprintf(stdout,"moving hand to WP: [%s]\n",const_cast<Vector&>(handWP.poss).toString().c_str());

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool affActionPrimitives::addHandSeqWP(const string &handSeqKey,
                                       const Vector &poss, const Vector vels)
{
    HandWayPoint handWP;
    handWP.poss=poss;
    handWP.vels=vels;

    handSeqMap[handSeqKey].push_back(handWP);

    return true;
}


/************************************************************************/
bool affActionPrimitives::isValidHandSeq(const string &handSeqKey)
{
    map<string,deque<HandWayPoint> >::iterator itr=handSeqMap.find(handSeqKey);

    if (itr!=handSeqMap.end())
        return true;
    else
        return false;
}


/************************************************************************/
bool affActionPrimitives::removeHandSeq(const string &handSeqKey)
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
deque<string> affActionPrimitives::getHandSeqList()
{
    map<string,deque<HandWayPoint> >::iterator itr;
    deque<string> q;

    for (itr=handSeqMap.begin(); itr!=handSeqMap.end(); ++itr)
        q.push_back(itr->first);

    return q;
}


/************************************************************************/
bool affActionPrimitives::reach(const Vector &x, const Vector &o, const bool sync)
{
    if (configured)
    {        
        fprintf(stdout,"\"reach\" requested\n");
        if (!pushAction(x,o))
            return false;

        bool ret=true;

        if (sync)
        {
            bool f=false;
            ret=checkActionsDone(f,true);
        }

        return ret;
    }
    else
        return false;
}


/************************************************************************/
bool affActionPrimitives::moveHand(const string &handSeqKey, const bool sync)
{
    if (configured)
    {        
        fprintf(stdout,"\"%s\" hand sequence requested\n",handSeqKey.c_str());
        if (!pushAction(handSeqKey))
            return false;

        bool ret=true;

        if (sync)
        {
            bool f=false;
            ret=checkActionsDone(f,true);
        }

        return ret;
    }
    else
        return false;
}


/************************************************************************/
bool affActionPrimitives::getPose(Vector &x, Vector &o)
{
    if (configured)
        return cartCtrl->getPose(x,o);
    else
        return false;
}


/************************************************************************/
bool affActionPrimitives::stopControl()
{
    if (configured)
    {
        suspend();

        clearActionsQueue();

        cartCtrl->stopControl();

        for (set<int>::const_iterator itr=enabledJoints.begin(); itr!=enabledJoints.end(); itr++)
            posCtrl->stop(*itr);

        resume();

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool affActionPrimitives::checkActionsDone(bool &f, const bool sync)
{
    if (configured)
    {
        if (sync && checkEnabled)
        {
            RES_EVENT(motionDoneEvent)->reset();
            RES_EVENT(motionDoneEvent)->wait();
        }

        f=latchArmMoveDone && latchHandMoveDone;

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool affActionPrimitives::syncCheckInterrupt()
{
    if (configured)
    {
        checkEnabled=false;
        RES_EVENT(motionDoneEvent)->signal();

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool affActionPrimitives::syncCheckReinstate()
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
bool affActionPrimitivesLayer1::grasp(const Vector &x, const Vector &o, const Vector &d,
                                      const bool sync)
{
    if (configured)
    {
        fprintf(stdout,"start grasping\n");
        pushAction(x+d,o,"open");
        pushAction(x,o,"close");

        bool ret=true;

        if (sync)
        {
            bool f=false;
            ret=checkActionsDone(f,true);
        }

        return ret;
    }
    else
        return false;
}


/************************************************************************/
bool affActionPrimitivesLayer1::touch(const Vector &x, const Vector &o, const Vector &d,
                                      const bool sync)
{
    if (configured)
    {
        fprintf(stdout,"start touching\n");
        pushAction(x+d,o,"open");
        pushAction(x,o);

        bool ret=true;

        if (sync)
        {
            bool f=false;
            ret=checkActionsDone(f,true);
        }

        return ret;
    }
    else
        return false;
}


/************************************************************************/
bool affActionPrimitivesLayer1::tap(const Vector &x, const Vector &o, const Vector &d,
                                    const bool sync)
{
    if (configured)
    {
        fprintf(stdout,"start tapping\n");
        pushAction(x,o,"open");
        pushAction(x+d,o);
        pushAction(x,o);

        bool ret=true;

        if (sync)
        {
            bool f=false;
            ret=checkActionsDone(f,true);
        }

        return ret;
    }
    else
        return false;
}



