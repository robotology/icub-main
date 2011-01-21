//
// A tutorial on how to control a generic serial kinematic chain
// relying only on yarp ports
// 
// Open ports:
// 
// -) /ctrl/q:i    receive the joints angles feedback [deg] from the robot
// -) /ctrl/xd:i   receive the target pose in axis-angle format ([x y z ax ay az theta]) from the user
// -) /ctrl/qd:i   output the joints configuration where to move (as result of the inverse kinematics)
// -) /ctrl/v:o    output the velocity profiles that steer the joints to the final configuration [deg/s] (to be connected to the robot)
// -) /ctrl/x:o    output the current end-effector position in axis-angle format
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>

#include <yarp/os/Network.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include <iCub/iKin/iKinFwd.h>
#include <iCub/iKin/iKinInv.h>
#include <iCub/iKin/iKinIpOpt.h>

#include <string>
#include <stdio.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;


// This inherited class handles the incoming
// target limb pose (xyz + axis/angle) and
// the joints feedback
/*****************************************************************/
class inPort : public BufferedPort<Bottle>
{
protected:
    Semaphore mutex;
    Vector vect;

    /*****************************************************************/
    virtual void onRead(Bottle &b)
    {
        mutex.wait();

        if (vect.length()!=b.size())
            vect.resize(b.size());

        for (int i=0; i<vect.length(); i++)
            vect[i]=b.get(i).asDouble();

        mutex.post();
    }

public:
    /*****************************************************************/
    Vector get_vect()
    {
        mutex.wait();
        Vector _vect=vect;
        mutex.post();

        return _vect;
    }

    /*****************************************************************/
    void set_vect(const Vector &_vect)
    {
        mutex.wait();
        vect=_vect;
        mutex.post();
    }
};


// This class handles the data exchange
// between Solver and Controller
/*****************************************************************/
class exchangeData
{
protected:
    Semaphore mutex;

    Vector xd;
    Vector qd;

public:
    /*****************************************************************/
    void setDesired(const Vector &_xd, const Vector &_qd)
    {
        mutex.wait();
        xd=_xd;
        qd=_qd;
        mutex.post();
    }

    /*****************************************************************/
    void getDesired(Vector &_xd, Vector &_qd)
    {
        mutex.wait();
        _xd=xd;
        _qd=qd;
        mutex.post();
    }
};


// The thread launched by the application which is
// in charge of inverting the limb kinematic relying
// on IpOpt computation.
/*****************************************************************/
class Solver : public RateThread
{
protected:
    ResourceFinder &rf;
    iKinLimb       *limb;
    iKinChain      *chain;
    iKinIpOptMin   *slv;
    exchangeData   *commData;

    inPort         *port_q;
    inPort          port_xd;
    Port            port_qd;

    Vector xd_old;

public:
    /*****************************************************************/
    Solver(ResourceFinder &_rf, inPort *_port_q, exchangeData *_commData, unsigned int period) :
           RateThread(period), rf(_rf), port_q(_port_q), commData(_commData)
    {
        limb=NULL;
        chain=NULL;
        slv=NULL;
    }

    /*****************************************************************/
    virtual bool threadInit()
    {
        fprintf(stdout,"Starting Solver at %g ms\n",getRate());

        string name=rf.find("name").asString().c_str();
        unsigned int ctrlPose=rf.check("onlyXYZ")?IKINCTRL_POSE_XYZ:IKINCTRL_POSE_FULL;

        Property linksOptions;
        linksOptions.fromConfigFile(rf.findFile("config").c_str());

        // instantiate the limb
        limb=new iKinLimb(linksOptions);

        if (!limb->isValid())
        {
            fprintf(stdout,"Error: invalid links parameters!\n");
            delete limb;

            return false;
        }

        // get the chain object attached to the limb
        chain=limb->asChain();

        // pose initialization with the current joints position.
        // Remind that the representation used is the axis/angle,
        // the default one.
        xd_old=chain->EndEffPose();
        commData->setDesired(xd_old,chain->getAng());

        // instantiate the optimizer with the passed chain, the ctrlPose control
        // mode, the tolerance and a maximum number of iteration
        slv=new iKinIpOptMin(*chain,ctrlPose,1e-3,200);

        // we have a dedicated tolerance for the translational part
        // which is by default equal to 1e-6;
        // note that the tolerance is applied to the squared norm
        slv->setTranslationalTol(1e-8);

        // in order to speed up the process, a scaling for the problem 
        // is usually required (a good scaling holds each element of the jacobian
        // of constraints and the hessian of lagrangian in norm between 0.1 and 10.0)
        slv->setUserScaling(true,100.0,100.0,100.0);

        port_xd.open(("/"+name+"/xd:i").c_str());
        port_xd.useCallback();
        port_xd.set_vect(xd_old);

        port_qd.open(("/"+name+"/qd:o").c_str());        

        return true;
    }

    /*****************************************************************/
    virtual void afterStart(bool s)
    {
        if (s)
            fprintf(stdout,"Solver started successfully\n");
        else
            fprintf(stdout,"Solver did not start\n");
    }

    /*****************************************************************/
    virtual void run()
    {
        // get the target pose
        Vector xd=port_xd.get_vect();

        // if new target is received
        if (!(xd==xd_old))
        {
            // get the feedback and update the chain
            chain->setAng(CTRL_DEG2RAD*port_q->get_vect());

            // minimize also against the current joints position
            Vector q0=chain->getAng();
            Vector w_3rd(chain->getDOF()); w_3rd=1.0;

            // call the solver and start the convergence from the current point
            Vector dummyVect(1);
            Vector qdhat=slv->solve(q0,xd,0.0,dummyVect,dummyVect,0.01,q0,w_3rd);

            // qdhat is an estimation of the real qd, so that xdhat is the actual achieved pose
            Vector xdhat=chain->EndEffPose(qdhat);

            // update the exchange structure straightaway
            commData->setDesired(xdhat,qdhat);

            // send qdhat over yarp
            port_qd.write(CTRL_RAD2DEG*qdhat);

            // latch the current target
            xd_old=xd;
        }
    }

    /*****************************************************************/
    virtual void threadRelease()
    {
        port_xd.interrupt();
        port_qd.interrupt();
        port_xd.close();
        port_qd.close();

        delete slv;
        delete limb;
    }
};


// The thread launched by the application which is
// in charge of computing the velocities profile
/*****************************************************************/
class Controller : public RateThread
{
protected:
    ResourceFinder      &rf;
    iKinLimb            *limb;
    iKinChain           *chain;
    MultiRefMinJerkCtrl *ctrl;
    exchangeData        *commData;

    inPort              *port_q;
    Port                 port_v;
    Port                 port_x;

public:
    /*****************************************************************/
    Controller(ResourceFinder &_rf, inPort *_port_q, exchangeData *_commData, unsigned int period) :
               RateThread(period), rf(_rf), port_q(_port_q), commData(_commData)
    {
        limb=NULL;
        chain=NULL;
        ctrl=NULL;
    }

    /*****************************************************************/
    virtual bool threadInit()
    {
        fprintf(stdout,"Starting Controller at %g ms\n",getRate());

        string name=rf.find("name").asString().c_str();
        unsigned int ctrlPose=rf.check("onlyXYZ")?IKINCTRL_POSE_XYZ:IKINCTRL_POSE_FULL;

        Property linksOptions;
        linksOptions.fromConfigFile(rf.findFile("config").c_str());

        // instantiate the limb
        limb=new iKinLimb(linksOptions);

        if (!limb->isValid())
        {
            fprintf(stdout,"Error: invalid links parameters!\n");
            delete limb;

            return false;
        }

        // get the chain object attached to the limb
        chain=limb->asChain();

        // instantiate controller
        ctrl=new MultiRefMinJerkCtrl(*chain,ctrlPose,getRate()/1000.0);

        // set the task execution time
        ctrl->set_execTime(rf.check("T",Value(2.0)).asDouble(),true);

        port_v.open(("/"+name+"/v:o").c_str());
        port_x.open(("/"+name+"/x:o").c_str());

        return true;
    }

    /*****************************************************************/
    virtual void afterStart(bool s)
    {
        if (s)
            fprintf(stdout,"Controller started successfully\n");
        else
            fprintf(stdout,"Controller did not start\n");
    }

    /*****************************************************************/
    virtual void run()
    {
        Vector xd,qd;

        // get the current target pose (both xd and qd are required)
        commData->getDesired(xd,qd);

        // get the feedback
        ctrl->set_q(CTRL_DEG2RAD*port_q->get_vect());

        // control the limb and dump all available information at rate of 1/100th
        ctrl->iterate(xd,qd,0x0064ffff);

        // send v and x through YARP ports
        port_v.write(CTRL_RAD2DEG*ctrl->get_qdot());
        port_x.write(ctrl->get_x());
    }

    /*****************************************************************/
    virtual void threadRelease()
    {
        // make sure that the limb is stopped before closing
        Vector v_zero(chain->getDOF()); v_zero=0.0;
        port_v.write(v_zero);

        port_v.interrupt();
        port_x.interrupt();
        port_v.close();
        port_x.close();

        delete ctrl;
        delete limb;
    }
};


/*****************************************************************/
class CtrlModule: public RFModule
{
protected:
    Solver       *slv;
    Controller   *ctrl;
    inPort        port_q;
    exchangeData  commData;

public:
    /*****************************************************************/
    virtual bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();

        string name=rf.find("name").asString().c_str();

        // Note that Solver and Controller operate on
        // different limb objects (instantiated internally
        // and separately) in order to avoid any interaction.
        slv=new Solver(rf,&port_q,&commData,30);
        ctrl=new Controller(rf,&port_q,&commData,10);

        if (!slv->start())
        {
            delete slv;
            return false;
        }
        
        if (!ctrl->start())
        {
            delete slv;
            delete ctrl;
            return false;
        }

        // open the feedback port
        port_q.open(("/"+name+"/q:i").c_str());
        port_q.useCallback();

        return true;
    }

    /*****************************************************************/
    virtual bool close()
    {
        ctrl->stop();
        slv->stop();

        delete ctrl;
        delete slv;

        port_q.interrupt();
        port_q.close();

        return true;
    }

    /*****************************************************************/
    virtual double getPeriod()
    {
        return 1.0;
    }

    /*****************************************************************/
    virtual bool updateModule()
    {
        return true;
    }
};


/*****************************************************************/
int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefault("name","ctrl");
    rf.setDefault("config","config.ini");
    rf.configure(NULL,argc,argv);

    if (rf.check("help"))
    {
        fprintf(stdout,"Options:\n\n");
        fprintf(stdout,"\t--name    name: controller name (default: \"ctrl\")\n");
        fprintf(stdout,"\t--config  file: specify the file containing the DH parameters of the links (default: \"config.ini\")\n");
        fprintf(stdout,"\t--T       time: specify the task execution time in seconds (default: 2.0)\n");
        fprintf(stdout,"\t--onlyXYZ     : disable orientation control\n");

        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    CtrlModule mod;

    return mod.runModule(rf);
}



