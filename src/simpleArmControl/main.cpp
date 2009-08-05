// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <ace/config.h>
#include <ace/OS.h>
#include <ace/Log_Msg.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Vocab.h>
#include <yarp/String.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <yarp/sig/Vector.h>

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp;

const double ARM_VELOCITY[]={5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5};
const double START_POSITION[]={0, 40, 0, -10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const double ZERO_POSITION[]={0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

const double MAX_POSITION[]={0, 80, 90, -10, 10, 70, 30};
const double MIN_POSITION[]={-120, 40, 0, -90, -100, 0, -70};

const int CONTROLLED_JOINTS=7;
const int ARM_JOINTS=15;
const int SAMPLER_RATE=100;
const double MAX_TIME=60; //after 4 seconds park arm

void waitMotion(IPositionControl *ipos)
{
    bool done=false;
    //Time::delay(15);
    int c=0;
    Time::delay(1);
    while(!done)
    {
        if (ipos->checkMotionDone(&done))
        {
            Time::delay(0.5);
            fprintf(stderr, ".");
        }
        else
        {
            fprintf(stderr, "CheckMotionDone returned false\n");
        }

        c++;
        if (c>30)
            done=true;
    }
    fprintf(stderr, "\n");
}

class ArmKeeper: public RateThread
{
private:
    IPositionControl *pos;
    double lastCommand;

public:
    ArmKeeper(IPositionControl *p): RateThread(100),
        pos(0)
    {
        lastCommand=Time::now();
        pos=p;
    }

    void notify()
    {
        lastCommand=Time::now();
    }

    bool threadInit()
    {
        notify();
        return true;
    }

    void run()
    {
        double currentTime=Time::now();

        if ((currentTime-lastCommand)>MAX_TIME)
        {
            notify();
            printf("ArmKeeper timed out, re-positioning the arm to the start position\n");
            if (pos!=0)
                pos->positionMove(START_POSITION);
        }
    }
};

// should avoid buffering
class MsgHandler:public BufferedPort<Vector>
{
private:
    int controlledJoints;
    double *cmdPositions;
    double *cmdVelocity;
    double *tmp;
    PolyDriver *dd;
    IPositionControl *ipos;
    ArmKeeper *keeper;
public:
    MsgHandler(PolyDriver *d):
      controlledJoints(CONTROLLED_JOINTS),
      cmdPositions(0),
      ipos(0),
      keeper(0)
    {
        printf("Controlling only %d joint\n", controlledJoints);
        useCallback();
        BufferedPort<Vector>::open("/james/armcontrol/i");

        cmdPositions=new double [ARM_JOINTS];
        memset(cmdPositions, 0, sizeof(double)*ARM_JOINTS);

        cmdVelocity= new double [ARM_JOINTS];
        memcpy(cmdVelocity, ARM_VELOCITY, sizeof(double)*ARM_JOINTS);

        tmp=new double [CONTROLLED_JOINTS];
        
        dd=d;
        bool ok=dd->view(ipos);

        ipos->setRefSpeeds(cmdVelocity);
    }

    ~MsgHandler()
    {
        if (cmdPositions!=0)
            delete [] cmdPositions;
        if (cmdVelocity!=0)
            delete [] cmdVelocity;
        if (tmp!=0)
            delete [] tmp;
    }

    void setNew(double *d)
    {
        int k=0;
        for(k=0; k<controlledJoints; k++)
        {
            if (d[k]>MAX_POSITION[k])
                d[k]=MAX_POSITION[k];
            if (d[k]<MIN_POSITION[k])
                d[k]=MIN_POSITION[k];

            cmdPositions[k]=d[k];
        }
    }

    void setKeeper(ArmKeeper *kp)
    {
        keeper=kp;
    }

    virtual void onRead(Vector &datum)
    {
        printf("Handling message\n");

        int axes=datum.size();

        if (axes!=controlledJoints)
        {
            printf("Wrong number of parametes, check number of joints\n");
            return;
        }

        int k;
        for(k=0; k<CONTROLLED_JOINTS; k++)
            tmp[k]=datum[k];

        setNew(tmp);
        
        //// debug
        printf("Moving arm to:\n");
        for(k=0; k<ARM_JOINTS; k++)
            printf("%.2lf\t", cmdPositions[k]);
        printf("\n");

        
        bool done=false;
        ipos->checkMotionDone(&done);

        if (done)
        {
            ipos->positionMove(cmdPositions);
            if (keeper!=0)
                keeper->notify();
        }
   }
};

class ArmSampler: public RateThread
{
private:
    IEncoders *iencs;
    IAmplifierControl *iamps;
    PolyDriver *dd;
    double *encoders;
    BufferedPort<Vector> portEncs;
    BufferedPort<Vector> portCurrs;
    bool inhibited;
    double *currents;
public:
    ArmSampler(PolyDriver *d, int rate): RateThread(rate),
        currents(0)
    { 
        dd=d;
        dd->view(iencs);
        dd->view(iamps);

        encoders=new double [ARM_JOINTS];
        currents=new double [ARM_JOINTS];

        portEncs.open("/james/armcontrol/local/encoders");
        portCurrs.open("/james/armcontrol/local/currents");
        inhibited=true;
    }

    ~ArmSampler()
    {
        portEncs.close();
        portCurrs.close();
        delete [] encoders;
        delete [] currents;
    }

    void inhibit()
    {
        inhibited=true;
    }

    void enable()
    {
        inhibited=false;
    }
  
    void run()
    {
        iencs->getEncoders(encoders);
        iamps->getCurrents(currents);

        Vector &vEncs=portEncs.prepare();
        Vector &vCurr=portCurrs.prepare();

        vEncs.size(CONTROLLED_JOINTS);
        vCurr.size(CONTROLLED_JOINTS);
        for(int k=0; k<CONTROLLED_JOINTS; k++)
        {
            (vEncs)[k]=encoders[k];
            (vCurr)[k]=currents[k];
        }

        portEncs.write();
        portCurrs.write();
    }

    void threadRelease(){}
};

//
int main(int argc, char *argv[]) 
{
    Property armOptions;
    armOptions.put("robot", "james");
    armOptions.put("part", "arm");
    armOptions.put("device", "remote_controlboard");
    armOptions.put("local", "/james/armcontrol/client");   //local port names
    armOptions.put("remote", "/james/arm");         //where we connect to

    Network::init();
	Time::turboBoost();

       // create a device
    PolyDriver armdd(armOptions);
    if (!armdd.isValid()) {
        ACE_OS::printf("Device not available.  Here are the known devices:\n");
        ACE_OS::printf("%s", Drivers::factory().toString().c_str());
        Network::fini();
        return 0;
    }

    IPidControl *pid;
    IAmplifierControl *amp;
    IPositionControl *pos;

    bool ok;
    ok = armdd.view(pid);
    ok &= armdd.view(amp);
    ok &= armdd.view(pos);

    if (!ok) {
        ACE_OS::printf("Problems acquiring interfaces\n");
        Network::fini();
        return 0;
    }

    //// enable amps the robot
    int jnts;
    pos->getAxes(&jnts);
    int i;
	for (i = 0; i < jnts; i++) {
		amp->enableAmp(i);
		pid->enablePid(i);
	}

    MsgHandler *msgHandler=new MsgHandler(&armdd);
    ArmSampler *sampler = new ArmSampler(&armdd, SAMPLER_RATE);
    ArmKeeper *keeper = new ArmKeeper(pos);
    sampler->start();
    keeper->start();
    msgHandler->setKeeper(keeper);

    // moving to start position
    pos->setRefSpeeds(ARM_VELOCITY);
    pos->positionMove(START_POSITION);
    waitMotion(pos);

    char cmd[80];
    bool quit=false;
    while (!quit) 
    {
        ACE_OS::printf("Type 'quit+enter' to exit the program\n");
        scanf("%s", cmd);
        if (strcmp(cmd, "quit")==0)
            quit=true;
    }

    keeper->stop();
    sampler->stop();

    pos->setRefSpeeds(ARM_VELOCITY);
    pos->positionMove(START_POSITION);
    waitMotion(pos);

    pos->positionMove(ZERO_POSITION);
    waitMotion(pos);

	for (i = 0; i < jnts; i++) {
		amp->disableAmp(i);
		pid->disablePid(i);
	}


    delete sampler;
    delete msgHandler;
    delete keeper;

    //finally close the dd
    armdd.close();

    Network::fini();
    return 0;
}
