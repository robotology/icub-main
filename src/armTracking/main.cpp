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

//#include <OnlineSVR.h>
#include "YARPRndUtils.h"

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp;
//using namespace onlinesvr;

const double ARM_VELOCITY[]={5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5};
const double START_POSITION[]={-12, 87, 35,  -80, 0, 0, 0, 0, 0, 0, 0, 0, 60, -60, 0};
const double STOP_POSITION[]={-30,  0,  0,  -10, 0, 0, 0, 0, 0, 0, 0, 0, 60, -60, 0};

//const double MAX_POSITION[]={   0, 100,90,  -80, 0, 0, 0, 0, 140, 60, 0, 0, 180, -120, 0};
const double MAX_POSITION[]={   0, 100, 100,  0, 0, 0, 0, 0, 0, 0, 0, 0, 60, -60, 0};
const double MIN_POSITION[]={ -198,  -99,-40, -90, 0, 0, 0, 0, 0, 0, 0, 0, 60, -60, 0};

///// jacoian exploration constants
const double MAX_EXPLORATION[]={3, 3, 3, 3};
const double MIN_EXPLORATION[]={-3, -3, -3, -3};
const int NMOVEMENTS=10;
const double MOVE_TIME2 = 1;
////////////////////////////

const int CONTROLLED_JOINTS=4;
const int ARM_JOINTS=15;
const int SAMPLER_RATE=6000;
const double MOVE_TIME = 6;

const double SLEEP_TIME		   = 0.1;
const double TIME_FOR_FIXATING = 20.0;

typedef std::vector<double *> PositionList;
typedef std::vector<double *>::iterator PositionListIt;
typedef std::vector<double *>::const_iterator PositionListConstIt;

BufferedPort<Bottle> *trackerWaitPort;
BufferedPort<Bottle> *dumperPort;

bool readCube(const char *filename, PositionList &list)
{
	list.clear();

	FILE *fp=fopen(filename, "rt");

	if (fp==0)
	{
		fprintf(stderr, "Error: could not open %s\n", filename);
		return false;
	}

	char line[80];
	char c;
	int k=0;
	printf("File is:\n", line);
	for (k=0;k<7;k++)
	{
		c=fscanf(fp, "%s\n", line);
		printf("%s ", line);
	}
	printf("\n");

	int elem=0;
    while(c!=EOF)
	{
		k=0;
        double *tmp=new double [7];
		for(k=0;k<7;k++)
            c=fscanf(fp, "%lf", &tmp[k]);
        
		elem++;
        list.push_back(tmp);
	}

	printf("Ok, read %d elements\n", elem);
	return true;
}

void waitMotion(IPositionControl *ipos)
{
	fprintf(stderr, "Checking if motion has been completed: ");

    bool done[ARM_JOINTS];
		
	for (int i = 0; i<ARM_JOINTS; i++)
		done[ARM_JOINTS]=false;

	bool done_all =false;

    int c=0;
    while(!done_all)
    {
		ipos->checkMotionDone(&done_all);
		//fprintf(stderr, ".");

		//Check all joints in position?
		for (int ii = 0; ii<ARM_JOINTS; ii++)
		{
			if (ipos->checkMotionDone(ii, &done[ii]))
			{
				printf("%d ", done[ii]);
			}
			else
			{
				fprintf(stderr, "CheckMotionDone returned false\n");
			}
		}
		printf("\n");
		c++;
		if (c>2000)
			done_all=true;
    }

	if (c>2000)
		fprintf(stderr, " not done!");
	else
		fprintf(stderr, " done!");

    fprintf(stderr, "\n");
}

class ExplorerThread: public Thread
{
public:
	Semaphore mutex;
	IPositionControl *ipos;
	IEncoders *iencs;
	YARPRndVector randomVector;

	ExplorerThread(): ipos(0), iencs(0), mutex(0) {}

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
		
		fprintf(stderr, "ExplorerThread --> Starting thread\n"); 
	//	mutex.post();

		randomVector.init();
		randomVector.resize(4, MAX_EXPLORATION, MIN_EXPLORATION);

		Bottle &bot=trackerWaitPort->prepare();
		bot.clear();
		bot.addInt(0);
		trackerWaitPort->write(false);

		Time::delay(0.1);

		return true;
	}
	
	void run()
	{
		// do something
		int count=NMOVEMENTS+1;
		int k=0;

		double last=Time::now();
		fprintf(stderr, "Controlling the arm to perform a rnd movement%d\n", count);

		YVector command(ARM_JOINTS);
		YVector startPosition(ARM_JOINTS);
		YVector encoders(ARM_JOINTS);
		iencs->getEncoders(encoders.data());
		startPosition=encoders;

		fprintf(stderr, "Starting from:\n");
		for(k=0;k<CONTROLLED_JOINTS;k++)
		{
			fprintf(stderr, "%.1lf\t",startPosition[k]);
		}
		fprintf(stderr, "\n");

		const YVector &tmp=randomVector.getVector();
		command=startPosition;
		for(k=0;k<CONTROLLED_JOINTS;k++)
			command[k]=tmp[k]+startPosition[k];

		double time=fixedTimeMove(command.data(), encoders.data(), MOVE_TIME2);
		count--;
		while(count>0)
		{
			double now=Time::now();

			if ((now-last)>time)
			{
				//wait just to be safe
				waitMotion(ipos);
				
				/// before we initiate a new command send the previous one
				if (count>1)  //count>1 means all movements but the last one
				{
					Bottle &dumpBot=dumperPort->prepare();
					dumpBot.clear();
					dumpBot.addInt(1);

					dumpBot.addDouble(startPosition(1));
					dumpBot.addDouble(startPosition(2));
					dumpBot.addDouble(startPosition(3));
					dumpBot.addDouble(startPosition(4));

					dumpBot.addDouble(command(1));
					dumpBot.addDouble(command(2));
					dumpBot.addDouble(command(3));
					dumpBot.addDouble(command(4));
					dumperPort->write();
				}

				/// initiate a new command
				const YVector &tmp=randomVector.getVector();

				command=startPosition;
				for(k=0;k<CONTROLLED_JOINTS;k++)
					command[k]=tmp[k]+startPosition[k];

				iencs->getEncoders(encoders.data());
				if (count>2)
				{
					fprintf(stderr, "Controlling the arm to perform a rnd movement%d\n", count);
					time=fixedTimeMove(command.data(), encoders.data(), MOVE_TIME2);
				}
				if (count==2)
				{
					fprintf(stderr, "Going back%d\n", count);
					//last movement, go back to the initial position
					time=fixedTimeMove(startPosition.data(), encoders.data(), MOVE_TIME2);
				}
				/////////////////////

				count--;
				last=Time::now();
			}

			Bottle &bot=trackerWaitPort->prepare();
			bot.clear();
			bot.addInt(0);
			trackerWaitPort->write(false);

			Time::delay(0.1);
		}

		fprintf(stderr, "ExplorerThread --> Thread quit\n"); 
		mutex.post();
	}
	
	void synch()
	{
		fprintf(stderr, "Synch wait\n"); 
		mutex.wait();
		fprintf(stderr, "Synch done\n"); 
	}

	double fixedTimeMove(const double *cmdPositions, const double *startPositions, double cmdTime)
	{
		int k;
		double cmdVelocities[ARM_JOINTS];
		for(k=0; k<ARM_JOINTS; k++)
		{
			cmdVelocities[k] = 0;
		
			if (fabs(startPositions[k] - cmdPositions[k]) > 0.01)
				cmdVelocities[k] = fabs(startPositions[k] - cmdPositions[k])/cmdTime;
			else
				cmdVelocities[k] = 1.0;
		}

		ipos->setRefSpeeds(cmdVelocities);
 		ipos->positionMove(cmdPositions);	

		printf("ExplorerThread-> Moving arm to:\n");
		for(int j=0; j<ARM_JOINTS; j++)
			printf("%.2lf\t", cmdPositions[j]);
		printf("\n");
		printf("ExplorerThread-> with velocity:\n");
		for(int ii=0; ii<ARM_JOINTS; ii++)
		 printf("%.2lf\t", cmdVelocities[ii]);
		printf("\n");

		return cmdTime+1;
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

	//OnlineSVR gravityModel[3];

    double *encoders;
	double *currents;
	double *voltages;
	double *cmdVelocities;
    bool motion_done;
	int fixation;

	int count;
public:
    ArmMover(PolyDriver *Arm_d, int rate): RateThread(rate),
        currents(0)
    { 
        Arm_dd=Arm_d;
        Arm_dd->view(iencs);
        Arm_dd->view(iamps);
		Arm_dd->view(ipos);
		Arm_dd->view(ipids);

        encoders=new double [ARM_JOINTS];
        currents=new double [ARM_JOINTS];
		voltages=new double [ARM_JOINTS];
        cmdVelocities=new double [ARM_JOINTS];

	    for(int k=0; k<ARM_JOINTS; k++)
		{
		    encoders[k] = START_POSITION[k];
			cmdVelocities[k] = ARM_VELOCITY[k];
		}


        motion_done=false;
		explorer=0;
		cube.clear();
		fixation=-1;
    }

    ~ArmMover()
    {
        delete [] encoders;
        delete [] currents;
		delete [] voltages;
        delete [] cmdVelocities;

    }
	
	void setFixation(int v)
	{
		fixation=v;
	}

	void fixedTimeMove(double *startPositions, double  *cmdPositions, double cmdTime);
	void waitFixation();
	void resetArm();

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
		srand( (unsigned)time( NULL ) );
		
		count=0; 
		fixation=-1;

		resetArm();

		return true;
	}

    void run()
    {
		//print position and current
		iencs->getEncoders(encoders);
		iamps->getCurrents(currents);
		ipids->getOutputs(voltages);

		//move to a new predifined position

		// defineNewPosition();
		if (fixation!=-1)
		{
			pickNewPosition();
			waitFixation();
		}
		else
			resetArm();

		Bottle &bot=trackerWaitPort->prepare();
		bot.clear();
		bot.addInt(1);
		trackerWaitPort->write(false);
	}

	void suspend()
	{
		fprintf(stderr, "Calling suspend...");
		RateThread::suspend();

		waitMotion(ipos);

		iencs->getEncoders(encoders);
		
		double nextPos[ARM_JOINTS];

		for (int k=0; k<CONTROLLED_JOINTS; k++)
		{
			nextPos[k] = START_POSITION[k];
		}

		for (int j=0 ; j<ARM_JOINTS-CONTROLLED_JOINTS; j++)
		{
			nextPos[j+CONTROLLED_JOINTS] = encoders[j+CONTROLLED_JOINTS];
		}

		fixedTimeMove(encoders, nextPos, MOVE_TIME);

		fprintf(stderr, "done!\n");
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

		waitMotion(ipos);

		iencs->getEncoders(encoders);
		
		double nextPos[ARM_JOINTS];

		for (int k=0; k<ARM_JOINTS; k++)
		{
			nextPos[k] = STOP_POSITION[k];
		}

		fixedTimeMove(encoders, nextPos, MOVE_TIME);
	}

	void pickNewPosition()
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
		
		double candidatePos[ARM_JOINTS];
		memcpy(candidatePos, encoders, sizeof(double)*ARM_JOINTS);

		while(!all_valid)
		{
			for(m=0; m<CONTROLLED_JOINTS;m++)
				valid[m]=false;

			index=int((cube.size()-1)*double (rand())/(RAND_MAX));
			
			//index is now from 0..cube.length()-1
			iencs->getEncoders(encoders);
			
			//copy encoedrs -> candidatePos
			memcpy(candidatePos, encoders, sizeof(double)*ARM_JOINTS);
			
			// joints in the cube have their own weird order
            candidatePos[1]=cube[index][0]*180/3.14;
            candidatePos[0]=cube[index][1]*180/3.14;
            candidatePos[2]=cube[index][2]*180/3.14;
            candidatePos[3]=cube[index][3]*180/3.14;
			
			fprintf(stderr, "Candidate position:\n");
			for(m=0;m<4;m++)
			{
				fprintf(stderr, "%.1lf\t", candidatePos[m]);
			}
			fprintf(stderr, "\n");

			// now check limits
			int k;
			for (k=0; k<CONTROLLED_JOINTS;k++)
			{
				if ( (MIN_POSITION[k]<=0) && (MAX_POSITION[k]<=0))
				{
					candidatePos[k]-=360;
					
					if (candidatePos[k]>MIN_POSITION[k])
						if (candidatePos[k]<MAX_POSITION[k])
							valid[k]=true;
				}
				if ( (MIN_POSITION[k]>=0) && (MAX_POSITION[k]>=0))
				{
					if (candidatePos[k]>MIN_POSITION[k])
						if (candidatePos[k]<MAX_POSITION[k])
							valid[k]=true;	
				}
				if ( (MIN_POSITION[k]<0) && (MAX_POSITION[k]>0))
				{
					if (candidatePos[k]<MAX_POSITION[k])
					{	
						if (candidatePos[k]>0)
							valid[k]=true;
					}
					else if (candidatePos[k]>MIN_POSITION[k]+360)
					{
						if (candidatePos[k]<360)
						{
							valid[k]=true;
							candidatePos[k]-=360;
						}
					}
				}
			}

			all_valid=true;
			for(m=0; m<CONTROLLED_JOINTS;m++)
				all_valid=all_valid&&valid[m];

		}

        fixedTimeMove(encoders, candidatePos, MOVE_TIME);
	}

	void defineNewPosition()
	{
		double candidatePos[ARM_JOINTS];
		iencs->getEncoders(encoders);

		for (int k=0; k<CONTROLLED_JOINTS; k++)
		{
			candidatePos[k] = (MAX_POSITION[k] - MIN_POSITION[k]) * ((double) rand())/RAND_MAX +  MIN_POSITION[k];
			if (candidatePos[k]>MAX_POSITION[k] || candidatePos[k]<MIN_POSITION[k])
			{
				printf("Error: requested position is out of range.\n");
				candidatePos[k] = encoders[k];
			}
		}

		for (int j=0 ; j<ARM_JOINTS-CONTROLLED_JOINTS; j++)
		{
			candidatePos[j+CONTROLLED_JOINTS] = encoders[j+CONTROLLED_JOINTS];
		}

		fixedTimeMove(encoders, candidatePos, MOVE_TIME);
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

//
int main(int argc, char *argv[]) 
{
	// these must go first
    fprintf(stderr, "Initializing net\n");
	Network::init();
	Time::turboBoost();
	fprintf(stderr, "Network initialized\n");

    fprintf(stderr, "Opening ports\n");
	trackerWaitPort = new BufferedPort<Bottle>;
	trackerWaitPort->open("/james/arm/wait/o");

	dumperPort = new BufferedPort<Bottle>;
	dumperPort->open("/james/arm/dumper/o");
    fprintf(stderr, "Port opened\n");

	/// code
	FixationPort fixation;
	fixation.useCallback();

    fprintf(stderr, "Reading cube");
	PositionList list;
	readCube(argv[1], list);
    fprintf(stderr, "Cube read");

    Property armOptions;
    armOptions.put("robot", "james");
    armOptions.put("part", "left_arm");
    armOptions.put("device", "remote_controlboard");
    armOptions.put("local", "/james/armcontrol/client");   //local port names
    armOptions.put("remote", "/james/laft_arm");         //where we connect to

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
	arm_mover->setExplorer(explorer);

	// init fixation callback
    fixation.setSlave(arm_mover);
	fixation.open("/james/arm/fixation");

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

	Terminee terminee("/james/arm/quit");
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

	explorer->stop();
    arm_mover->stop();

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


//// some functions
//
void ArmMover::fixedTimeMove(double* startPositions, double* cmdPositions, double cmdTime)
{
	double cmdVelocities[ARM_JOINTS];

	for(int k=0; k<ARM_JOINTS; k++)
	{
		cmdVelocities[k] = 0;
		
		if (fabs(startPositions[k] - cmdPositions[k]) > 0.01)
			cmdVelocities[k] = fabs(startPositions[k] - cmdPositions[k])/cmdTime;
		else
			cmdVelocities[k] = 1.0;
	}

	ipos->setRefSpeeds(cmdVelocities);
 	ipos->positionMove(cmdPositions);	

    printf("ArmMover-> Moving arm to:\n");

    for(int j=0; j<ARM_JOINTS; j++)
	    printf("%.2lf\t", cmdPositions[j]);
	printf("\n");
//    printf("Moving arm with velocity:\n");
//   for(int ii=0; ii<ARM_JOINTS; ii++)
//	    printf("%.2lf\t", cmdVelocities[ii]);
//	printf("\n");

	//wait enough time to have the
	//movement finished
	Time::delay(cmdTime+1);
	waitMotion(ipos);
}

void ArmMover::resetArm()
{
	double start_pos[ARM_JOINTS];
	double next_pos[ARM_JOINTS];

	iencs->getEncoders(start_pos);

	for(int i=0;i<ARM_JOINTS; i++)
		next_pos[i] = START_POSITION[i];

	fixedTimeMove(start_pos, next_pos, MOVE_TIME);
}

void ArmMover::waitFixation()
{
	//now wait for fixation
	int count = 0;
	while((fixation==0) && (count*SLEEP_TIME < TIME_FOR_FIXATING))
	{
		Time::delay(SLEEP_TIME);
		count ++;
		fprintf(stderr, "w");

		Bottle &bot=trackerWaitPort->prepare();
		bot.clear();
		bot.addInt(1);
		trackerWaitPort->write(false);
	}

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
