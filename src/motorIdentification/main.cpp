// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <ace/config.h>
#include <ace/OS.h>
#include <ace/Log_Msg.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Vocab.h>
#include <yarp/String.h>
#include <math.h>
#include <yarp/os/Terminator.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <yarp/sig/Vector.h>
#include <vector>

#include <yarp/os/PortReaderBuffer.h>
#include <yarp/os/Random.h>

//#include <OnlineSVR.h>
//#include "YARPRndUtils.h"

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp;
using namespace yarp::sig;
//using namespace onlinesvr;


const int CONTROLLED_JOINT	= 0;
const int N=5;
const int SAMPLER_RATE		= 2000;

class ArmMover: public RateThread
{
private:
    IEncoders *iencs;
    IAmplifierControl *iamps;
	IPositionControl *ipos;
	IPidControl *ipids;
    PolyDriver *Arm_dd;
public:
    ArmMover(PolyDriver *Arm_d, int rate): RateThread(rate)
    { 
        Arm_dd=Arm_d;
        Arm_dd->view(iencs);
        Arm_dd->view(iamps);
		Arm_dd->view(ipos);
		Arm_dd->view(ipids);
    }

    ~ArmMover()
    {

    }

	bool threadInit()
	{
		ACE_OS::printf("Motor Identification --> running\n");
		return 1;
	}

    void run()
    {
		static int count = 2;
		double p,p1,p2,p3,p4;
		iencs->getEncoder(CONTROLLED_JOINT, &p);
		iencs->getEncoder(CONTROLLED_JOINT+1, &p1);
		iencs->getEncoder(CONTROLLED_JOINT+2, &p2);
                iencs->getEncoder(CONTROLLED_JOINT+3, &p3);
                iencs->getEncoder(CONTROLLED_JOINT+4, &p4);

		//fprintf(stderr, "Received a %.3f", p);

		if (count%2==0)
		{
			fixedTimeMove(CONTROLLED_JOINT,-20, p, 1);
        		fixedTimeMove(CONTROLLED_JOINT+1,-20, p1, 1);
			fixedTimeMove(CONTROLLED_JOINT+2,-20, p2, 1);
                        fixedTimeMove(CONTROLLED_JOINT+3,-20, p3, 1);
                        fixedTimeMove(CONTROLLED_JOINT+4,-20, p4, 1);

		}
		else
		{		
			fixedTimeMove(CONTROLLED_JOINT,20, p, 1);
			fixedTimeMove(CONTROLLED_JOINT+1,20, p1, 1);
			fixedTimeMove(CONTROLLED_JOINT+2, 20, p2, 1);
                        fixedTimeMove(CONTROLLED_JOINT+3,20, p3, 1);
                        fixedTimeMove(CONTROLLED_JOINT+4,20, p4, 1);


		}
		count ++;
	}

	void fixedTimeMove(int joint, const double cmdPosition, const double startPosition, double cmdTime)
	{

		double cmdVelocity = 0;
		
		cmdVelocity = fabs(startPosition - cmdPosition)/cmdTime;

		ipos->setRefSpeed(joint, cmdVelocity);
 		ipos->positionMove(joint, cmdPosition);
	}
};
//
int main(int argc, char *argv[]) 
{
	// these must go first
    fprintf(stderr, "Initializing net\n");
	Network::init();
	Time::turboBoost();
	fprintf(stderr, "Network initialized\n");

    Property armOptions;
    armOptions.put("robot", "icub");
    armOptions.put("part", "head");
    armOptions.put("device", "remote_controlboard");
    armOptions.put("local", "/icub/armcontrol/client");   //local port names
    armOptions.put("remote", "/icub/head");         //where we connect to

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
	IEncoders *enc;

    bool ok;
    ok = armdd.view(pid);
    ok &= armdd.view(amp);
    ok &= armdd.view(pos);
	ok &= armdd.view(enc);
	
    if (!ok) {
        ACE_OS::printf("Problems acquiring interfaces\n");
        Network::fini();
        return 0;
    }

    //// enable amps the robot
    int jnts;
    pos->getAxes(&jnts);

    int i= CONTROLLED_JOINT;
	for(i=CONTROLLED_JOINT;i<(N+CONTROLLED_JOINT);i++)
	{
	   amp->enableAmp(i);
           pid->enablePid(i);	
	}

    ArmMover *arm_mover = new ArmMover(&armdd, SAMPLER_RATE);
	arm_mover->start();

	Terminee terminee("/icub/head/quit");
    if (!terminee.isOk()) { 
        fprintf(stderr, "armTracking --> Failed to create proper quit socket\n"); 
       return 1;
    }   

    fprintf(stderr, "armTracking --> Driver instantiated and running (silently)\n");
    while (!terminee.mustQuit()) 
	{
		Time::delay(1); 
    }
		
	fprintf(stderr, "armTracking --> Received a quit message\n");

    arm_mover->stop();

    delete arm_mover;
    //finally close the dd
    armdd.close();

    Network::fini();

    return 0;
}

