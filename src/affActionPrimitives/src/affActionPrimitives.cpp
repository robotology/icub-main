
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
#define ACTIONPRIM_DEFAULT_FINGERSVEL   20.0    // [deg/s]


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

            fprintf(stdout,"%s limits: [%d,%d] deg\n",key.c_str(),min,max);
            cartCtrl->setLimits(j,min,max);
        }

        return true;
    }

    return false;
}


/************************************************************************/
bool affActionPrimitives::getVector(Property &opt, const string &key, Vector &v,
                                    const int offs)
{
    if (opt.check(key.c_str()))
    {
        Bottle *b=opt.find(key.c_str()).asList();

        int l1=b->size();
        int l2=v.length();
        int l=l1<l2?l1:l2;

        for (int i=0; i<l; i++)
            v[offs+i]=b->get(i).asDouble();

        return true;
    }

    return false;
}


/************************************************************************/
bool affActionPrimitives::open(Property &opt)
{
    if (!opt.check("local"))
    {
        fprintf(stdout,"Error: option \"local\" is missing\n");
        return false;
    }

    if (!opt.check("hand_calibration_file"))
    {
        fprintf(stdout,"Error: option \"hand_calibration_file\" is missing\n");
        return false;
    }

    string robot=opt.check("robot",Value("icub")).asString().c_str();
    string part=opt.check("part",Value("right_arm")).asString().c_str();
    int period=opt.check("thread_period",Value(ACTIONPRIM_DEFAULT_PER)).asInt();
    double trajTime=opt.check("traj_time",Value(ACTIONPRIM_DEFAULT_TRAJTIME)).asDouble();
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

    iMin=7;                     // hand first joint
    posCtrl->getAxes(&iMax);    // hand last joint

    double fingersVel=opt.check("fingers_vel",Value(ACTIONPRIM_DEFAULT_FINGERSVEL)).asDouble();
    for (int i=iMin; i<iMax; i++)
    {    
        enabledJoints.insert(i);
        posCtrl->setRefSpeed(i,fingersVel);
    }

    // set trajectory time
    cartCtrl->setTrajTime(trajTime);

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

    fingerOpenPos.resize(iMax,0.0);
    fingerClosePos.resize(iMax,0.0);

    getVector(opt,"fingers_open_poss",fingerOpenPos,iMin);
    getVector(opt,"fingers_close_poss",fingerClosePos,iMin);

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
            fprintf(stdout,"Warning: No thresholds for joint #%d specified.\n",i);
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
bool affActionPrimitives::pushAction(const Vector &x, const Vector &o,
                                     bool (affActionPrimitives::*handAction)(const bool))
{
    if (configured)
    {
        mutex->wait();
        Action action;
    
        action.waitState=false;
        action.tmo=0.0;
        action.execReach=true;
        action.x=x;
        action.o=o;
        action.handAction=handAction;
    
        actionsQueue.push_back(action);
        mutex->post();

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool affActionPrimitives::pushAction(const Vector &x, const Vector &o)
{
    return pushAction(x,o,&affActionPrimitives::nopHand);
}


/************************************************************************/
bool affActionPrimitives::pushAction(bool (affActionPrimitives::*handAction)(const bool))
{
    if (configured)
    {        
        mutex->wait();
        Action action;
        Vector dummy(1);
    
        action.waitState=false;
        action.tmo=0.0;
        action.execReach=false;
        action.x=dummy;
        action.o=dummy;
        action.handAction=handAction;
    
        actionsQueue.push_back(action);
        mutex->post();
    
        return true;
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
        action.execReach=false;
        action.x=dummy;
        action.o=dummy;
        action.handAction=&affActionPrimitives::nopHand;

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
        action=actionsQueue[0];
        actionsQueue.pop_front();

        exec=true;
    }
    mutex->post();

    if (exec)
    {
        if (action.waitState)
            wait(action.tmo);

        if (action.execReach)
            reach(action.x,action.o);

        if (action.handAction!=NULL)
            (this->*action.handAction)(false);
    }

    return exec;
}


/************************************************************************/
void affActionPrimitives::run()
{
    if (!armMoveDone)
    {
        Vector x,o,xdcap,odcap,qdcap;
        cartCtrl->getPose(x,o);
        cartCtrl->getDesired(xdcap,odcap,qdcap);

        fprintf(stdout,"reaching... xdcap=%s |e|=%.3f [m]\n",
                xdcap.toString().c_str(),norm(xdcap-x));

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
            fprintf(stdout,"hand motion complete\n");
    }

    latchArmMoveDone=armMoveDone;
    latchHandMoveDone=handMoveDone;

    if (latchArmMoveDone && latchHandMoveDone)
        if (!execQueuedAction() && Time::now()-latchTimer>waitTmo)
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
        waitTmo=tmo;
        latchTimer=Time::now();

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool affActionPrimitives::reach(const Vector &x, const Vector &o, const bool sync)
{
    if (configured)
    {
        clearActionsQueue();
    
        xd=x;
        od=o;
        
        bool ret=cartCtrl->goToPoseSync(xd,od);

        if (!ret)
        {
            fprintf(stdout,"reach error\n");
            return false;
        }
    
        latchArmMoveDone=armMoveDone=false;
        fprintf(stdout,"reach for [%s], [%s]\n",xd.toString().c_str(),od.toString().c_str());
    
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
bool affActionPrimitives::grasp(const Vector &x, const Vector &o, const Vector &d,
                                const bool sync)
{
    if (configured)
    {
        fprintf(stdout,"start grasping\n");
        if (!reach(x+d,o))
            return false;

        pushAction(x,o,&affActionPrimitives::closeHand);

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
bool affActionPrimitives::touch(const Vector &x, const Vector &o, const Vector &d,
                                const bool sync)
{
    if (configured)
    {
        fprintf(stdout,"start touching\n");
        if (!reach(x+d,o))
            return false;

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
bool affActionPrimitives::tap(const Vector &x, const Vector &o, const Vector &d,
                              const bool sync)
{
    if (configured)
    {
        fprintf(stdout,"start tapping\n");
        if (!reach(x,o))
            return false;

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


/************************************************************************/
bool affActionPrimitives::moveHand(const Vector &fingerPos, const bool sync)
{
    if (configured)
    {        
        activeJoints=enabledJoints;
        for (set<int>::const_iterator itr=enabledJoints.begin(); itr!=enabledJoints.end(); itr++)
            posCtrl->positionMove(*itr,fingerPos[*itr]);

        latchHandMoveDone=handMoveDone=false;
        fprintf(stdout,"start moving hand\n");

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
bool affActionPrimitives::nopHand(const bool sync)
{
    return configured;
}


/************************************************************************/
bool affActionPrimitives::openHand(const bool sync)
{
    return moveHand(fingerOpenPos,sync);
}


/************************************************************************/
bool affActionPrimitives::closeHand(const bool sync)
{
    return moveHand(fingerClosePos,sync);
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



