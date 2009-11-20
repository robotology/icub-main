
#include <ace/Auto_Event.h>
#include <yarp/math/Math.h>
#include <yarp/os/Time.h>

#include <gsl/gsl_math.h>

#include <iCub/common.h>
#include <iCub/affActionPrimitives.h>

#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <string>

#define RES_EVENT(x)                (static_cast<ACE_Auto_Event*>(x))

#define MOTORIF_NOPHAND             0x00
#define MOTORIF_OPENHAND            0x01
#define MOTORIF_CLOSEHAND           0x02
#define MOTORIF_ALIGNHAND           0x03

#define MOTORIF_DEFAULT_PER         250     // [ms]
#define MOTORIF_DEFAULT_TRAJTIME    1.5     // [s]


using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;


affActionPrimitives::affActionPrimitives() : RateThread(MOTORIF_DEFAULT_PER)
{
    polyHand=polyCart=NULL;

    handMetrics=NULL;
    fs=NULL;

    mutex=NULL;
    motionDoneEvent=NULL;

    executeHand[MOTORIF_NOPHAND]  =&affActionPrimitives::nopHand;
    executeHand[MOTORIF_OPENHAND] =&affActionPrimitives::openHand;
    executeHand[MOTORIF_CLOSEHAND]=&affActionPrimitives::closeHand;
    executeHand[MOTORIF_ALIGNHAND]=&affActionPrimitives::alignHand;

    armMoveDone =latchArmMoveDone =true;
    handMoveDone=latchHandMoveDone=true;
    configured=closed=false;
    checkEnabled=true;
}


bool affActionPrimitives::open(Property &opt)
{
    if (!opt.check("local"))
    {
        fprintf(stdout,"Error: option \"local\" is missing\n");
        return false;
    }

    if (!opt.check("calibFile"))
    {
        fprintf(stdout,"Error: option \"calibFile\" is missing\n");
        return false;
    }

    string robot=opt.check("robot",Value("icub")).asString().c_str();
    string part=opt.check("part",Value("right_arm")).asString().c_str();
    int period=opt.check("period",Value(MOTORIF_DEFAULT_PER)).asInt();
    double trajTime=opt.check("trajTime",Value(MOTORIF_DEFAULT_TRAJTIME)).asDouble();
    string local=opt.find("local").asString().c_str();
    string sensingCalibFile=opt.find("calibFile").asString().c_str();
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

    iMin=7;                     // first hand joint
    posCtrl->getAxes(&iMax);    // last hand joint

    for (int i=iMin; i<iMax; i++)
    {    
        enabledJoints.insert(i);    // set ref joint speed
        posCtrl->setRefSpeed(i,20.0);
    }

//  // remove thumb joints
//  enabledJoints.erase(8);
//  enabledJoints.erase(9);
//  enabledJoints.erase(10);

    // set trajectory time
    cartCtrl->setTrajTime(trajTime);

    // set shot mode
    cartCtrl->setTrackingMode(false);

    // enable torso joints
    if (opt.check("torso"))
    {
        if (Bottle *joints=opt.find("torso").asList())
        {
            Vector curDof;
            cartCtrl->getDOF(curDof);
            Vector newDof=curDof;

            int l=joints->size()>3 ? 3 : joints->size();

            fprintf(stdout,"enabling torso joints... ");

            for (int i=0; i<l; i++)
                newDof[i]=joints->get(i).asInt();

            cartCtrl->setDOF(newDof,curDof);
            fprintf(stdout,"%s\n",curDof.toString().c_str());
        }
    }

    // create hand metrix
    readMatrices(sensingCalib,sensingConstants);
    thresholds=sensingConstants["thresholds"].getRow(0);

    fprintf(stdout,"creating hand metrics...\n");
    handMetrics=new HandMetrics(encCtrl,pidCtrl,ampCtrl,sensingConstants);

    fprintf(stdout,"creating hand smoother...\n");
    fs=new FunctionSmoother(thresholds);

    fingerOpenPos.resize(9,0.0);
    fingerClosePos.resize(9,0.0);
    fingerAlignPos.resize(9,0.0);

    fingerClosePos[0]=10.0;
    fingerClosePos[1]=40.0;
    fingerClosePos[2]=90.0;
    fingerClosePos[3]=10.0;
    fingerClosePos[4]=70.0;
    fingerClosePos[5]=70.0;
    fingerClosePos[6]=70.0;
    fingerClosePos[7]=70.0;
    fingerClosePos[8]=110.0;

    mutex=new Semaphore(1);
    motionDoneEvent=new ACE_Auto_Event;

    // start the thread with the specified period
    Time::turboBoost();
    setRate(period);
    start();

    return configured=true;
}


void affActionPrimitives::close()
{
    if (closed)
        return;    

    if (isRunning())
    {
        fprintf(stdout,"stopping thread...\n");
        stop();
    }

    if (fs!=NULL)
    {    
        fprintf(stdout,"disposing hand smoother...\n");
        delete fs;
    }

    if (handMetrics!=NULL)
    {
        fprintf(stdout,"disposing hand metrics...\n");
        delete handMetrics;
    }

    if (polyHand!=NULL)
    {
        fprintf(stdout,"closing hand driver...\n");
        delete polyHand;
    }

    if (polyCart!=NULL)
    {
        fprintf(stdout,"closing cartesian driver...\n"); 
        delete polyCart;
    }

    if (mutex!=NULL)
        delete mutex;

    if (motionDoneEvent!=NULL)
        delete RES_EVENT(motionDoneEvent);

    closed=true;

    fprintf(stdout,"closing complete!\n");
}


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


void affActionPrimitives::queue_clear()
{
    mutex->wait();
    q.clear();
    mutex->post();
}


void affActionPrimitives::queue_push(const Vector &x, const Vector &o, const int handId)
{
    mutex->wait();
    MotorIFQueue el;

    el.x=x;
    el.o=o;
    el.handId=handId;

    q.push_back(el);
    mutex->post();
}


void affActionPrimitives::queue_exec()
{
    bool exec=false;
    MotorIFQueue el;

    mutex->wait();
    if (q.size())
    {
        el=q[0];
        q.pop_front();

        exec=true;
    }
    mutex->post();

    if (exec)
    {
        if (el.x.length())
            reach(el.x,el.o,false);

        (this->*executeHand[el.handId])(false);
    }
}


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
        {
            fprintf(stdout,"reaching complete\n");
            queue_exec();
        }            
    }

    if (!handMoveDone)
    {
        handMetrics->snapshot();
        stopBlockedJoints(activeJoints);

        // check whether all the remaining active joints have come
        // to a complete stop
        handMoveDone=handMotionDone(activeJoints);

        if (handMoveDone)
        {    
            fprintf(stdout,"hand motion complete\n");
            queue_exec();
        }
    }

    latchArmMoveDone=armMoveDone;
    latchHandMoveDone=handMoveDone;

    if (latchArmMoveDone && latchHandMoveDone)
        RES_EVENT(motionDoneEvent)->signal();
}


affActionPrimitives::~affActionPrimitives()
{
    close();
}


void affActionPrimitives::setGraspOrien(const Vector &o)
{
    oGrasp=o;
}


void affActionPrimitives::setTapOrien(const Vector &o)
{
    oTap=o;
}

void affActionPrimitives::setHome(const Vector &x, const Vector &o)
{
    xHome=x;
    oHome=o;
}


void affActionPrimitives::setGraspDisplacement(const Vector &h)
{
    hGrasp=h;
}


void affActionPrimitives::setTapDisplacement(const yarp::sig::Vector &disp)
{
    dTap=disp;
}


bool affActionPrimitives::reach(const Vector &x, const Vector &o, const bool sync)
{
    if (configured)
    {
        queue_clear();

        xd=x;
        od=o;
        
        bool ret=cartCtrl->goToPoseSync(xd,od);

        latchArmMoveDone=armMoveDone=false;
        fprintf(stdout,"reach for [%s], [%s]\n",xd.toString().c_str(),od.toString().c_str());
    
        if (sync)
        {
            bool f=false;
            ret=check(f,true);
        }

        return ret;
    }
    else
        return false;
}


bool affActionPrimitives::grasp(const Vector &x, const bool sync)
{
    if (configured)
    {
        fprintf(stdout,"start grasping\n");
        if (!reach(x+hGrasp,oGrasp,false))
            return false;

        queue_push(x,oGrasp,MOTORIF_CLOSEHAND);

        bool ret=true;

        if (sync)
        {
            bool f=false;
            ret=check(f,true);
        }

        return ret;
    }
    else
        return false;
}


bool affActionPrimitives::touch(const Vector &x, const bool sync)
{
    return true;
}


bool affActionPrimitives::tap(const Vector &x, const bool sync)
{
    if (configured)
    {
        fprintf(stdout,"start tapping\n");
        if (!reach(x,oTap,false))
            return false;

        queue_push(x+dTap,oTap,MOTORIF_NOPHAND);

        bool ret=true;

        if (sync)
        {
            bool f=false;
            ret=check(f,true);
        }

        return ret;
    }
    else
        return false;
}


bool affActionPrimitives::moveHand(const Vector &fingerPos, const bool sync)
{
    if (configured)
    {        
        activeJoints=enabledJoints;
        for (set<int>::const_iterator itr=enabledJoints.begin(); itr!=enabledJoints.end(); itr++)
            posCtrl->positionMove(*itr,fingerPos[*itr-iMin]);

        latchHandMoveDone=handMoveDone=false;
        fprintf(stdout,"start moving hand\n");

        bool ret=true;

        if (sync)
        {
            bool f=false;
            ret=check(f,true);
        }

        return ret;
    }
    else
        return false;
}


bool affActionPrimitives::openHand(const bool sync)
{
    return moveHand(fingerOpenPos,sync);
}


bool affActionPrimitives::closeHand(const bool sync)
{
    return moveHand(fingerClosePos,sync);
}


bool affActionPrimitives::alignHand(const bool sync)
{
    return moveHand(fingerAlignPos,sync);
}


bool affActionPrimitives::getPose(Vector &x, Vector &o)
{
    if (configured)
        return cartCtrl->getPose(x,o);
    else
        return false;
}


bool affActionPrimitives::home(const bool sync)
{
    return reach(xHome,oHome,sync);
}


bool affActionPrimitives::check(bool &f, const bool sync)
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


void affActionPrimitives::syncCheckInterrupt()
{
    checkEnabled=false;
    RES_EVENT(motionDoneEvent)->signal();
}


void affActionPrimitives::syncCheckReinstate()
{
    checkEnabled=true;
}

