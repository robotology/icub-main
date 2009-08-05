// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_module
 *
 * \defgroup Reaching armMover
 *
 * Random exploration. This module starts moving the arm
 * by picking a random position from a list. The list of positions
 * is passed as a parameter. After each movement, it moves the 
 * arm by perturbing the joints of (small) random amounts.
 *
 * Useful for learning arm forward map and Jacobian.
 *
 * Options:
 * --list file.txt (mandatory: a file containing the list of positions to explore)
 * 
 * in $ICUB_ROOT/src/armMover/main.cpp.
 *
 *
 * \author Lorenzo Natale
 */ 

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Vocab.h>
#include <yarp/math/Rand.h>

#include <yarp/os/Terminator.h>

#include "readPositions.h"

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <yarp/sig/Vector.h>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp;
using namespace yarp::sig;
using namespace yarp::math;

const int CONTROLLED_JOINTS=7;
const int ARM_JOINTS=16; //used only below to define constants
const int SAMPLER_RATE=100;
const double SLEEP_TIME		   = 0.1;
const double TIME_FOR_FIXATING = 20.0;
const int NMOVEMENTS=10;
const double MOVE_TIME2 = 1;

const double ARM_VELOCITY[ARM_JOINTS]={4.0,   4.0,     4.0,    4.0,
                            10.0,   5.0,  10.0,
                            100.0,  100.0,  100.0,  100.0,  100.0, 100.0, 100.0,   100.0,  100.0};

const double START_POSITION[ARM_JOINTS]={-25.8,  20.0,  0.0,  50,  
                                0.0,   0.0,   0.0, 
                                2465.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

const double STOP_POSITION[ARM_JOINTS]={-25.8,  20.0,  0.0,  50,  
                                0.0,   0.0,   0.0, 
                                2465.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

///// jacobian exploration constants
const double MAX_EXPLORATION[CONTROLLED_JOINTS]={3, 3, 3, 3, 3, 3, 3};
const double MIN_EXPLORATION[CONTROLLED_JOINTS]={-3, -3, -3, -3, -3, -3, -3};
////////////////////////////

BufferedPort<Bottle> *trackerWaitPort;
BufferedPort<Bottle> *dumperPort;

class ExplorerThread: public Thread
{
public:
	Semaphore mutex;
    Semaphore wait;
	IPositionControl *ipos;
	IEncoders *iencs;
    int nJoints;
    bool abortF;
	Vector command;
	Vector startPosition;
	Vector encoders;

	ExplorerThread(): ipos(0), iencs(0), mutex(0), nJoints(0), wait(0) {}

	~ExplorerThread(){}
	
	void setInterfaces(IPositionControl *ip, IEncoders *ie)
	{ ipos=ip; iencs=ie; }

	bool threadInit()
	{
		if (ipos==0)
		{
			fprintf(stderr, "ExplorerThread --> PositionControl interface not set, aborting\n"); 
			return false;
		}
		
        Rand::init();

        ipos->getAxes(&nJoints);
        command.resize(nJoints);
        startPosition.resize(nJoints);
        encoders.resize(nJoints);
        abortF=false;
	}

    void abort()
    {
        abortF=true;
        kick();
    }
	
    void kick()
    {
        //wake up thread
        fprintf(stderr, "---> Waking up expl thread\n");
        wait.post();
    }

    void run()
    {
        int k=0;
        int count=NMOVEMENTS;

        // init
        while(!isStopping())
            {
                //wait for kick
                wait.wait();
                if (abortF)
                    return;

                iencs->getEncoders(encoders.data());
                startPosition=encoders;
                command=startPosition;
                fprintf(stderr, "ExplorerThread: starting from:\n");
                int k;
                for(k=0;k<CONTROLLED_JOINTS;k++)
                    {
                        fprintf(stderr, "%.1lf\t",command[k]);
                    }
                fprintf(stderr, "\n");

                k=0;
                count=NMOVEMENTS;
                fprintf(stderr, "---> Controlling the arm to perform a rnd movement%d\n", count);
                count--;
                while(count>0)
                    {
                        double now=Time::now();

                        fprintf(stderr, "Waiting %d...", count);
                        waitMotion();
                        fprintf(stderr, "done!\n");

                        /// before we initiate a new command send the previous one
                        if (count>1)  //count>1 means all movements but the last one
                            {
                                Bottle &dumpBot=dumperPort->prepare();
                                dumpBot.clear();
                                dumpBot.addInt(1);

                                dumpBot.addDouble(startPosition[0]);
                                dumpBot.addDouble(startPosition[1]);
                                dumpBot.addDouble(startPosition[2]);
                                dumpBot.addDouble(startPosition[3]);

                                dumpBot.addDouble(command[0]);
                                dumpBot.addDouble(command[1]);
                                dumpBot.addDouble(command[2]);
                                dumpBot.addDouble(command[3]);
                                dumperPort->write();
                            }

                        /// initiate a new command
                        if (count>2)
                            {
                                fprintf(stderr, "Controlling the arm to perform a rnd movement%d\n", count);
                                command=startPosition;
                                for(k=0;k<CONTROLLED_JOINTS;k++)
                                    command[k]=startPosition[k]+Rand::scalar(MIN_EXPLORATION[k], MAX_EXPLORATION[k]);
                                ipos->positionMove(command.data());
                            }
                        if (count==2)
                            {
                                fprintf(stderr, "Going back%d\n", count);
                                //last movement, go back to the initial position
                                ipos->positionMove(startPosition.data());
                            }
                        
                        /////////////////////
                        
                        count--;
                        
                        Bottle &bot=trackerWaitPort->prepare();
                        bot.clear();
                        bot.addInt(0);
                        trackerWaitPort->write(false);
                        
                        Time::delay(0.1);
                    }

                fprintf(stderr, "---> ExplorerThread done loop\n"); 
                mutex.post();
            }
    }
	
	void synch()
	{
		fprintf(stderr, "Synch wait\n"); 
		mutex.wait();
		fprintf(stderr, "Synch done\n"); 
	}

    void waitMotion()
    {
        bool done=false;
        while(!done)
        {
            fprintf(stderr, "going to call check motion done...\n");
            ipos->checkMotionDone(&done);
            Time::delay(0.1);
            fprintf(stderr, ".");
        }
    }
};

class ArmMover: public RateThread
{
private:
    IEncoders *iencs;
    IAmplifierControl *iamps;
	IPositionControl *ipos;
	IPidControl *ipids;
    PolyDriver *Arm_dd;
	ExplorerThread *explorer;

	PositionList cube;

    Vector encoders;
    Vector command;
    Vector startPosition;
    Vector stopPosition;

    bool motion_done;
	int fixation;
    int nJoints;

	int count;
public:
    ArmMover(PolyDriver *Arm_d, int rate): RateThread(rate)
    { 
        Arm_dd=Arm_d;
        Arm_dd->view(iencs);
        Arm_dd->view(iamps);
		Arm_dd->view(ipos);
		Arm_dd->view(ipids);
        ipos->getAxes(&nJoints);

        encoders.resize(nJoints);
        command.resize(nJoints);
        
        iencs->getEncoders(encoders.data());
        startPosition=encoders;
        stopPosition=encoders;

        for(int k=0;k<CONTROLLED_JOINTS;k++)
        {
            startPosition[k]=START_POSITION[k];
            stopPosition[k]=STOP_POSITION[k];
        }

        motion_done=false;
		explorer=0;
		cube.clear();
		fixation=-1;
    }

    ~ArmMover()
    {
    }
	
	void setFixation(int v)
	{
		fixation=v;
	}

	void waitFixation();

	void setCube(PositionList &c)
	{
		cube=c;
	}

	void setExplorer(ExplorerThread *expl)
	{
		explorer=expl;
	}

	bool threadInit()
	{
        Rand::init();

		count=0; 
		fixation=-1;

		ipos->positionMove(startPosition.data());
        fprintf(stderr, "Moving to start position...");
        waitMotion();
        fprintf(stderr, "done!\n");

		return true;
	}

    void run()
    {
		//print position and current
		fprintf(stderr, "--> Looping ArmMover Thread");
		iencs->getEncoders(encoders.data());

        //move to a new predefined position
        bool done=false;
        fprintf(stderr, " going to check motion");
        ipos->checkMotionDone(&done);
        fprintf(stderr, "returned %d\n", done);
        if (done)
        {
            if (explorer!=0)
            {
                explorer->kick();
                explorer->synch();
            }
             
            chooseNewPosition(command);
            ipos->positionMove(command.data());
        }

//		if (fixation!=-1)
//		{
//			fprintf(stderr, "--> Pick new position and wait fixation\n");
//			pickNewPosition();
//			waitFixation();
//		}
//		else
//		{
//			fprintf(stderr, "--> Resetting arm\n");
//			resetArm();
//			fprintf(stderr, "--> Warning running with debug code\n");
//			//remove thi:
//			fixation=0;
//		}

//		Bottle &bot=trackerWaitPort->prepare();
//		bot.clear();
//		bot.addInt(1);
//		trackerWaitPort->write(false);
	}

	void suspend()
	{
		fprintf(stderr, "Calling suspend...\n");
		RateThread::suspend();

        fprintf(stderr, "Waiting for motion.");
        waitMotion();
        fprintf(stderr, "done\n");

        ipos->positionMove(startPosition.data());

        fprintf(stderr, "Parking arm.");
        waitMotion();     
        fprintf(stderr, "done\n");
	}

	void resume()
	{
		fprintf(stderr, "Calling resume...");
		RateThread::resume();
		fprintf(stderr, "done!\n");
	}

    void threadRelease()
	{
		/*fclose(dataFile);
		fclose(errorFile);*/
        ipos->positionMove(stopPosition.data());
        fprintf(stderr, "Parking...");
        waitMotion();
        fprintf(stderr, "done!\n");
	}

    void chooseNewPosition(Vector &nP)
    {
        int index;
        if (cube.size()==0)
        {
            printf("Empty cube, skipping");
            return;
        }
        bool valid[CONTROLLED_JOINTS];
        bool all_valid=false;
        int m;
        for(m=0; m<CONTROLLED_JOINTS;m++)
            valid[m]=false;

        Vector candidatePos(nJoints);

        index=int((cube.size()-1)*Rand::scalar()+0.5);
        //        static int localIndex=0;
        //        index=localIndex++;

        int k=0;
        for(k=0;k<CONTROLLED_JOINTS;k++)
            candidatePos(k)=cube[index][k];

        fprintf(stderr, "New position:\n");
        for(k=0;k<CONTROLLED_JOINTS;k++)
        {
            fprintf(stderr, "%.1lf\t", candidatePos(k));
        }
        fprintf(stderr, "\n");
        nP=candidatePos;
    }

    void waitMotion()
    {
        bool done=false;
        while(!done)
        {
            fprintf(stderr, "going to call check motion done...\n");
            ipos->checkMotionDone(&done);
            Time::delay(0.1);
            fprintf(stderr, ".");
        }
    }
};

class FixationPort: public BufferedPort<Bottle>
{
private:
	ArmMover *slave;
public:
	FixationPort(): slave(0){}
	void onRead(Bottle &bot)
	{
		if (slave!=0)
		{
			slave->setFixation(bot.get(0).asInt());
		}
	}

	void setSlave(ArmMover *s)
	{
		slave=s;
	}
};

///// Code from the robot james, need to be adapted to icub
int main(int argc, char *argv[]) 
{
	// these must go first
	Network::init();
	Time::turboBoost();

    Property param;
    param.fromCommand(argc, argv);

	trackerWaitPort = new BufferedPort<Bottle>;
//	trackerWaitPort->open("/james/arm/wait/o");

	dumperPort = new BufferedPort<Bottle>;
//	dumperPort->open("/james/arm/dumper/o");

	/// code
	FixationPort fixation;
//	fixation.useCallback();

    Property parameters;
    parameters.fromCommand(argc, argv);
    
    if (!parameters.check("positions"))
        {
            fprintf(stderr, "--positions file is a mandatory option\n");
            Network::fini();
            return -1;
        }
    Value& inifile= parameters.find("positions");
    fprintf(stderr, "Reading cube from %s\n", inifile.asString().c_str());

	PositionList list;
    bool ok=readCube(inifile.asString().c_str(), list);
    if (!ok)
    {
        fprintf(stderr, "\nError reading file of exploration positions\n");
        Network::fini();
        return -1;
    }

    fprintf(stderr, "ok!\n");

    //fprintf(stderr, "read %d positions", list.size());
    //    for(int kk=0;kk<list.size();kk++)
    //        {
    //            fprintf(stderr, "%lf\n", list[kk][0]);
    //        }

    Property armOptions;
    // remote stuff:
    //    armOptions.put("device", "remote_controlboard");
    //    armOptions.put("local", "/icub/armmover/client");   //local port names
    //    armOptions.put("remote", "/icub/right_arm");         //where we connect to
    ///// remote stuff end here

    //begin local stuff, see also below, search calibration
    Property armCalibOptions;
    if (!armOptions.fromConfigFile("/home/icub/Code/iCub/conf/icub_arm_right.ini"))
         {
            fprintf(stderr, "Check file\n");
             Network::fini();
             return 0;
         }
     armOptions.put("device", "esd2");
     armOptions.put("name", "icub/right_arm");
     armCalibOptions.fromString(armOptions.toString());
     armCalibOptions.unput("device");
     armCalibOptions.unput("subdevice");
     armCalibOptions.put("device","icubarmcalibrator");
     PolyDriver calib(armCalibOptions);

     if (!calib.isValid())
         {
             printf("Calibrator not available, Here are the known devices:\n");
             printf("%s", Drivers::factory().toString().c_str());
             Network::fini();
             return 0;
         }
//     ///// local stuff end here
    
    // create a device
    PolyDriver armdd(armOptions);
    if (!armdd.isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        Network::fini();
        return 0;
    }

    IPidControl *pid;
    IAmplifierControl *amp;
    IPositionControl *pos;
	IEncoders *enc;

    ICalibrator *ical;
    IControlCalibration *arm_cal;

    ok = armdd.view(pid);
    ok &= armdd.view(amp);
    ok &= armdd.view(pos);
	ok &= armdd.view(enc);

    // only local:
    ok &= calib.view(ical);
    ok &= armdd.view(arm_cal);

    arm_cal->setCalibrator(ical);
    arm_cal->calibrate();
    /////////////////////////////////////////

    if (!ok) {
        printf("Problems acquiring interfaces\n");
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

	ExplorerThread *explorer= new ExplorerThread();
	explorer->setInterfaces(pos, enc);
    ArmMover *arm_mover = new ArmMover(&armdd, SAMPLER_RATE);
    arm_mover->setCube(list);
	arm_mover->start();

    //thread will wait for a kick
    explorer->start();
	arm_mover->setExplorer(explorer);

	// init fixation callback
//    fixation.setSlave(arm_mover);
//	fixation.open("/james/arm/fixation");

	//char cmd[80];
    //bool quit=false;
    //while (!quit) 
    //{
        //ACE_OS::printf("Type 'quit+enter' to exit the program\n");
		//ACE_OS::printf("or 'pause/resume +enter' to pause/resume the arm\n");
    //    scanf("%s", cmd);
    //    if (strcmp(cmd, "quit")==0)
    //        quit=true;
	//	if (strcmp(cmd, "pause")==0)
	//	{
	//		arm_mover->suspend();
	//	}
	//	if (strcmp(cmd, "resume")==0)
	//	{
	//		arm_mover->resume();
    //    }
    //}

	Terminee terminee("/icub/armmover/quit");
    if (!terminee.isOk()) { 
        fprintf(stderr, "armMover --> Failed to create proper quit socket\n"); 
       return 1;
    }   

    fprintf(stderr, "armMover --> Driver instantiated and running (silently)\n");
    while (!terminee.mustQuit()) 
	{
		Time::delay(1); 
    }
		
	fprintf(stderr, "armMover --> Received a quit message\n");

    explorer->abort();
	explorer->stop();
    arm_mover->stop();

    arm_cal->park();

	fixation.close(); //close port so it does not write to a null arm_mover
    delete arm_mover;
	delete explorer;
	delete trackerWaitPort;
	delete dumperPort;

    //finally close the dd
    armdd.close();

    Network::fini();

    return 0;
}


#if 0
void ArmMover::waitFixation()
{
	//now wait for fixation
	int count = 0;
	// DEBUG, remove this
	fprintf(stderr, "Warning original code removed for debugging purpose\n");
	fprintf(stderr, "Please check code for this message and uncomment missing code\n");
	// removed from here
	/*
	while((fixation==0) && (count*SLEEP_TIME < TIME_FOR_FIXATING))
	{
		Time::delay(SLEEP_TIME);
		count ++;
		fprintf(stderr, "w");

		Bottle &bot=trackerWaitPort->prepare();
		bot.clear();
		bot.addInt(1);
		trackerWaitPort->write(false);
	}*/
	// until here

	if (count*SLEEP_TIME == TIME_FOR_FIXATING)
		{
            fprintf(stderr, "a");
            fixation = -1;
        }
	if (fixation==1)
        {
            fprintf(stderr, "f");
            //wait some extra time to give the dumper chance to save good data
            Time::delay(0.5);

			//start exploration
			if (explorer!=0)
			{
				explorer->start();
				explorer->synch(); //synch on the explorer
				explorer->stop();
			}
		}
	if (fixation==-1)
		fprintf(stderr, "#");
}
#endif
