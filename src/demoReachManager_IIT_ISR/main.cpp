
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <gsl/gsl_math.h>

#include <iostream>
#include <iomanip>
#include <string>

#define TARGET_X_OFFSET         0.000
#define TARGET_Y_OFFSET         0.030
#define TARGET_Z_OFFSET         0.000

#define ORIENT_LEFT_X           -0.064485
#define ORIENT_LEFT_Y           0.707066
#define ORIENT_LEFT_Z           -0.704201 
#define ORIENT_LEFT_R           3.140572

#define ORIENT_RIGHT_X          -0.012968
#define ORIENT_RIGHT_Y          -0.721210
#define ORIENT_RIGHT_Z          0.692595
#define ORIENT_RIGHT_R          2.917075
                                                             
using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;


class managerThread : public RateThread
{
protected:
    string name;
    string part;

    BufferedPort<Vector> *inportTrackTarget;
    BufferedPort<Vector> *outportCmdHead;
    BufferedPort<Vector> *outportCmdHand;

    IEncoders *encs;

    Vector targetPos;
    Vector targetOffset;
    Vector handOrient;
    Vector torso;

    Matrix R,Rx,Ry,Rz;

    bool targetNew;
    bool goHand;

    double Ts;

    int slot;

public:
    managerThread(const string &_name, unsigned int period, PolyDriver *drv, const string &_part) : 
                  RateThread(period), name(_name), part(_part)
    {
        Ts=getRate()/1000.0;
        slot=0;

        targetPos.resize(3);
        targetOffset.resize(3);
        handOrient.resize(4);
        torso.resize(3);

        targetPos=0.0;

        targetOffset[0]=TARGET_X_OFFSET;
        targetOffset[2]=TARGET_Z_OFFSET;

        if (part=="left_arm")
        {    
            targetOffset[1]=-TARGET_Y_OFFSET;
            handOrient[0]=ORIENT_LEFT_X;
            handOrient[1]=ORIENT_LEFT_Y;
            handOrient[2]=ORIENT_LEFT_Z;
            handOrient[3]=ORIENT_LEFT_R;
        }
        else
        {    
            targetOffset[1]=TARGET_Y_OFFSET;
            handOrient[0]=ORIENT_RIGHT_X;
            handOrient[1]=ORIENT_RIGHT_Y;
            handOrient[2]=ORIENT_RIGHT_Z;
            handOrient[3]=ORIENT_RIGHT_R;
        }

        R=Rx=Ry=Rz=eye(3,3);

        targetNew=false;
        goHand=false;

        drv->view(encs);
    }

    virtual bool threadInit()
    {
        inportTrackTarget=new BufferedPort<Vector>;
        outportCmdHead   =new BufferedPort<Vector>;
        outportCmdHand   =new BufferedPort<Vector>;

        inportTrackTarget->open((name+"/trackTarget:i").c_str());
        outportCmdHead->open((name+"/cmdHead:o").c_str());
        outportCmdHand->open((name+"/cmdHand:o").c_str());

        return true;
    }

    /******************************************************/
    virtual void run()
    {
        getSensorData();

        commandHead();

        doReach();

        targetNew=false;

        if (++slot>=10)
            slot=0;
    }

    virtual void threadRelease()
    {
        inportTrackTarget->interrupt();
        outportCmdHead->interrupt();
        outportCmdHand->interrupt();

        inportTrackTarget->close();
        outportCmdHead->close();
        outportCmdHand->close();

        delete inportTrackTarget;
        delete outportCmdHead;
        delete outportCmdHand;
    }

    void getSensorData()
    {
        if (encs->getEncoders(torso.data()))
            R=rotx(torso[1])*roty(-torso[2])*rotz(-torso[0]);

        if (Vector *targetPosNew=inportTrackTarget->read(false))
        {    
            targetPos=*targetPosNew;
            targetNew=true;
            goHand=true;
        }
    }

    void commandHead()
    {
        if (targetNew)
        {
            outportCmdHead->prepare()=targetPos;
            outportCmdHead->write();
        }        
    }

    void doReach()
    {
        if (slot==0 && goHand)
        {
            Vector y=R.transposed()*(targetPos+targetOffset);
            limitRange(y);
            y=R*y;

            Vector &x=outportCmdHand->prepare();
            x.resize(7);

            for (int i=0; i<3; i++)
                x[i]=y[i];

            for (int i=0; i<4; i++)
                x[3+i]=handOrient[i];

            outportCmdHand->write();
        }
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
};


class managerModule: public RFModule
{
protected:
    managerThread *thr;
    PolyDriver    *drv;
    Port           rpcPort;

public:
    managerModule() { }

    virtual bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();

        Property opt;
        opt.put("device","remote_controlboard");
        opt.put("remote","/icub/torso");
        opt.put("local",getName("torso"));

        drv=new PolyDriver(opt);

        if (!drv->isValid())
            return false;

        thr=new managerThread(getName().c_str(),10,drv,
                              rf.check("part",Value("left_arm")).asString().c_str());

        thr->start();

        rpcPort.open(getName("rpc"));
        attach(rpcPort);

        return true;
    }

    virtual bool close()
    {
        rpcPort.interrupt();
        rpcPort.close();

        thr->stop();
        delete thr;

        delete drv;

        return true;
    }

    virtual double getPeriod()    { return 1.0;  }
    virtual bool   updateModule() { return true; }
};


int main(int argc, char *argv[])
{
    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure("ICUB_ROOT",argc,argv);

    managerModule mod;
    mod.setName("/demoReachManager_IIT_ISR");

    return mod.runModule(rf);
}




