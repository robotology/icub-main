// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_module
 *
 * \defgroup icub_yet_another_headcontrol yaHeadControl
 *
 * This is YetAnotherHeadControl. Written for fun.
 * Ports opened:
 * /icub/right/target
 * /icub/left/target
 * Both accept a bottle of three numbers x,y,code, output of a 
 * tracker. The controller connects to iCubInterface. Uses 
 * monocular info form either one of the eyes or stereo from 
 * both if available.
 * 
 * Read configuration info from file
 *
 * Run as:
 * yaHeadControl --file filename
 *
 * E.g. yaHeadControl --file icub_yaheadcontrol.ini
 *
 * 
 * \author Lorenzo Natale
 *
 */

#include <ace/config.h>
#include <ace/OS.h>
#include <ace/Log_Msg.h>

#include <yarp/os/PortablePair.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Vocab.h>
#include <yarp/os/Terminator.h>
#include <yarp/String.h>
#include <yarp/os/RateThread.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <yarp/sig/Vector.h>

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp;

const double TIMEOUT=0.1;
const double STOP_TIME=3;
const int THREAD_RATE=30;

const double START_POSITION[]={0,0,0,0,0,10,0,0};
const double START_SPEED[]={10,10,10,10,10,10,0,0};

BufferedPort<yarp::sig::Vector> *targetLeft=0; 
BufferedPort<yarp::sig::Vector> *targetRight=0;
Port *trackingPort=0;
BufferedPort<yarp::sig::Vector> *inertial=0;

class TrackerThread: public RateThread
{
public:
    double vorx;
    double vory;
    double vorz;
    double Kvergence;
    double Kpan;
    double Ktilt;
    double Kn_pan;
    double Kn_tilt;
    PolyDriver dd;
    bool init;
    Property options;
    IEncoders *iencs;
    IVelocityControl *ivel;
    IPositionControl *ipos;
    int joints;

    double *encoders;
    double *command;
    double *accelerations;
    double *prev;

    double timeStampL;
    double timeStampR;
    double timeStampLprev;
    double timeStampRprev;
    double timeStampTO;

    Vector targetL;
    Vector targetR;

    bool enableVOR;
    bool enableTracking;

public:
    TrackerThread(Property &op):RateThread(THREAD_RATE)
    {
        options=op;
        joints=0;

        enableVOR=true;
        enableTracking=true;
        
    }
    ~TrackerThread(){}

    bool threadInit()
    {
        fprintf(stderr, "Starting thread\n");
        dd.open(options);
        if (!dd.isValid()) 
            {
                printf("Device not available.  Here are the known devices:\n");
                printf("%s", Drivers::factory().toString().c_str());
                return false;
            }

        bool ok;
        ok = dd.view(ivel);
        ok = ok&&dd.view(ipos);        
        ok = ok&&dd.view(iencs);        

        if (!ok)
            {
                fprintf(stderr, "Problems acquiring interfaces\n");
                return false;
            }

        ivel->getAxes(&joints);
        printf("Working with %d axes\n", joints);
        
        encoders = new double [joints];
        command = new double [joints];
        accelerations = new double [joints];
        prev = new double [joints];
        
        int k=0;
        for (k=0; k<joints; k++)
            {
                accelerations[k]=100;
                command[k]=0;
                prev[k]=command[k];
            }

        ivel->setRefAccelerations(accelerations);
        Time::delay(0.3);
        ivel->velocityMove(command);
        Time::delay(1);

        ipos->setRefSpeeds(START_SPEED);
        ipos->positionMove(START_POSITION);

        bool done=false;
        fprintf(stderr, "Going to start position\n");
        while(!done)
            {
                Time::delay(0.3);
                ipos->checkMotionDone(&done);
                fprintf(stderr, ".");
            }
        fprintf(stderr, "done!\n");

        targetR.resize(3);
        targetR=0;
        targetL.resize(3);
        targetL=0;

        timeStampL=0;
        timeStampR=0;
        timeStampLprev=timeStampL;
        timeStampRprev=timeStampR;
        return true;
    }

    void run()
    {
        iencs->getEncoders(encoders);
        Vector *vr=targetRight->read(0);
        Vector *vl=targetLeft->read(0);
        Vector *gyro=inertial->read(0);
        double timeNow=Time::now();
        double delayR=0.0;
        double delayL=0.0;

        for(int k=0;k<joints;k++)
            command[k]=0;

        if (vr!=0)
            {
                targetR=*vr;
                timeStampRprev=timeStampR;
                timeStampR=Time::now();
            }

        if (vl!=0)
            {
                targetL=*vl;
                timeStampLprev=timeStampL;
                timeStampL=Time::now();
            }

        delayR=timeNow-timeStampR;
        delayL=timeNow-timeStampL;

        if ( (delayR<TIMEOUT)&&(delayL<TIMEOUT))
            {                           
                //binocular
                double d=targetR(0)-targetL(0);
                command[4]=Kpan*(0.7*targetR(0)+0.3*targetL(0));
                command[5]=Kvergence*d;
                command[3]=Ktilt*0.5*(targetL(1)+targetR(1));
                command[0]=Kn_tilt*(encoders[3]);
                command[2]=Kn_pan*(encoders[4]);
                timeStampTO=timeNow;
                enableVOR=true;
            }
        else if (delayR<TIMEOUT)
            {
                //monocular
                command[4]=Kpan*targetR(0);
                command[5]=0.1*Kvergence*(encoders[5]-10);
                command[3]=Ktilt*targetR(1);
                command[0]=Kn_tilt*(encoders[3]);
                command[2]=Kn_pan*(encoders[4]);
                timeStampTO=timeNow;
                enableVOR=true;
            }
        else if (delayL<TIMEOUT)
            {
                command[4]=Kpan*targetL(0);
                command[5]=0.1*Kvergence*(encoders[5]-10);
                command[3]=Ktilt*targetL(1);
                command[0]=Kn_tilt*(encoders[3]);
                command[2]=Kn_pan*(encoders[4]);
                timeStampTO=timeNow;
                enableVOR=true;
            }
        else
            {
                if ((timeNow-timeStampTO)>STOP_TIME)
                    {
                        enableVOR=false;
                        command[4]=0;
                        command[5]=0;
                        command[3]=0;
                        command[0]=0;
                        command[2]=0;
                    }
                else
                    {
                        for(int k=0;k<joints;k++)
                            command[k]=2.2*prev[k]/STOP_TIME;
                    }
            }

        if ( (gyro != 0) && (enableVOR))
            {
                command[1] = vorx*(*gyro)[0]; //roll
                command[3] -= ((*gyro)[6] * 180.0 / 3.141528 * vory); //tilt
                command[4] -= ((*gyro)[7] * 180.0 / 3.141528 * vorz); //pan
            }
        else
            {
                command[1]=0.0; //roll
            }
            
        bool ok=ivel->velocityMove(command);

        for(int k=0;k<joints;k++)
            prev[k]=command[k];
    }

    void threadRelease()
    {
        fprintf(stderr, "Releasing thread\n");
        int k=0;
        for(k=0;k<joints;k++)
            command[k]=0;
        ivel->velocityMove(command);
        Time::delay(0.5);

        bool done=false;
        fprintf(stderr, "Going back to start position\n");
        ipos->setRefSpeeds(START_SPEED);
        ipos->positionMove(START_POSITION);
        while(!done)
            {
                Time::delay(0.3);
                ipos->checkMotionDone(&done);
                fprintf(stderr, ".");
            }
        fprintf(stderr, "done!\n");

        delete [] command;
        delete [] encoders;
        delete [] accelerations;
    }
};

//
int main(int argc, char *argv[]) 
{
    Network::init();
	Time::turboBoost();

    Terminee terminee("/yaHeadControl/quit");
    if (!terminee.isOk()) {
        printf("Failed to create proper quit socket\n");
        return 1;
    }

    // just list the devices if no argument given
    if (argc <= 2) {
        printf("You can call %s like this:\n", argv[0]);
        printf("   %s --file FILENAME\n", argv[0]);
        printf("For example:\n");
        printf("   %s --file icub_yaheadcontrol.ini\n", argv[0]);
        
        return 0;
    }

    // get command line options
    Property options;
    options.fromCommand(argc, argv);
    if (!options.check("file")) {
        ACE_OS::printf("Missing the --file option\n");
        return 0;
    }

	if (!options.check("device")) {
		ACE_OS::printf("Adding device remote_controlboard\n");
		options.put("device", "remote_controlboard");
	}

    Value& file = options.find("file");

	Property p;
	printf("Head controller working with config file %s\n", file.asString().c_str());
	p.fromConfigFile(file.asString().c_str());
    fprintf(stderr, "%s", p.toString().c_str());

    Value robotname = p.findGroup("GENERAL").find("Robot");

	yarp::String s("/");
	s += robotname.asString().c_str();
	s += "/head/control";
	options.put("local", s.c_str());

	s.clear();
	s += "/";
	s += robotname.asString().c_str();
	s += "/head";
	options.put("remote", s.c_str());

    bool verbose = options.check("verbose") || p.findGroup("GENERAL").find("Verbose").asInt() != 0;

    // receive the reference value for the PID controller.

	yarp::String name;
	name.clear();
	name += "/";
	name += robotname.asString().c_str();
	name += "/right/target";
    targetRight=new BufferedPort<yarp::sig::Vector>;
    targetLeft=new BufferedPort<yarp::sig::Vector>;
    trackingPort=new Port;

    targetRight->open(name.c_str());
	name.clear();
	name += "/";
	name += robotname.asString().c_str();
	name += "/left/target";
    targetLeft->open(name.c_str());

    inertial = new BufferedPort<yarp::sig::Vector>;
	name.clear();
	name += "/";
	name += robotname.asString().c_str();
	name += "/inertial_data";
    inertial->open(name.c_str());

	yarp::String name_src("/");
	name_src += robotname.asString().c_str();
	name_src += "/inertial";

	Network::connect(name_src.c_str(), name.c_str(), "udp");
    
    TrackerThread *thread=new TrackerThread(options);

	Bottle tmp_b = p.findGroup("PIDS").findGroup("Pan");
	printf("%s\n", tmp_b.toString().c_str());
    thread->Kpan=tmp_b.get(1).asDouble();
    	
	tmp_b = p.findGroup("PIDS").findGroup("Tilt");
	printf("%s\n", tmp_b.toString().c_str());
    thread->Ktilt=tmp_b.get(1).asDouble();

	tmp_b = p.findGroup("PIDS").findGroup("Vergence");
	printf("%s\n", tmp_b.toString().c_str());
    thread->Kvergence=tmp_b.get(1).asDouble();
	
	tmp_b = p.findGroup("PIDS").findGroup("NeckPan");
	printf("%s\n", tmp_b.toString().c_str());
    thread->Kn_pan=tmp_b.get(1).asDouble();

	tmp_b = p.findGroup("PIDS").findGroup("NeckTilt");
    thread->Kn_tilt=tmp_b.get(1).asDouble();

	tmp_b = p.findGroup("PIDS").findGroup("VOR");
	printf("%s\n", tmp_b.toString().c_str());
    thread->vorx = tmp_b.get(1).asDouble();
	thread->vory = tmp_b.get(2).asDouble();
	thread->vorz = tmp_b.get(3).asDouble();

    if(!thread->start())
        {
            fprintf(stderr, "Error opening interfaces\n");
            Network::fini();
            return -1;
        }

    while (!terminee.mustQuit()) 
        {
        }

    thread->stop();

    Network::fini();
    return 0;
}
