// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Vocab.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <yarp/sig/Vector.h>

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;

const double HEAD_VELOCITY[]={      10,  10,  10,  10,  10,  10};
const double START_POSITION_HEAD[]={ 0,   0,   0,   0,   0,   0};
const double MAX_POSITION_HEAD[]={   0,  30,  45,  15,  52,  15};
const double MIN_POSITION_HEAD[]={ -40, -40, -45, -35, -50,   0};

const int    SAMPLER_RATE          = 1000;

const double MOVE_TIME_HEAD        = 0.75;
const int	 CONTROLLED_JOINTS_HEAD = 6;
const int    HEAD_JOINTS            = 6;


FILE *OUTPUT_FILE = fopen("Data.txt","a");

  
void initFile()
{
	fprintf(OUTPUT_FILE, "theta0[deg] theta1[deg] theta2[deg] theta3[deg] theta4[deg] theta5[deg] theta6[deg] ");
	fprintf(OUTPUT_FILE, "i0[mA] i1[mA] i2[mA] i3[mA] i4[mA] i5[mA] i6[mA] ");
	fprintf(OUTPUT_FILE, "v0[V] v1[V] v2[V] v3[V] v4[V] v5[V] v6[V]");
	fprintf(OUTPUT_FILE, "\n");
}

void writeFile(double* encs, double *curr, double *volts)
{
}


void waitMotion(IPositionControl *ipos)
{
    bool done=false;
	bool did = false;
    // Time::delay(0.1);
	
	int n = 0; 
	ipos->getAxes(&n);

    int c=0;
    
    fprintf(stderr, "\n");
}


class BodyMover: public RateThread
{
private:
   

    IEncoders *iencsHead;
    IAmplifierControl *iampsHead;
	IPositionControl *iposHead;
	IPidControl *ipidsHead;
    PolyDriver *Head_dd;
    double *encodersHead;
	double *currentsHead;
	double *voltagesHead;
	double *cmdPositionsHead;
	double *cmdVelocitiesHead;

public:
    BodyMover( PolyDriver *Head_d, int rate): RateThread(rate)
    { 
      
        Head_dd=Head_d;
        Head_dd->view(iencsHead);
        Head_dd->view(iampsHead);
		Head_dd->view(iposHead);
		Head_dd->view(ipidsHead);


        encodersHead=new double [HEAD_JOINTS];
        currentsHead=new double [HEAD_JOINTS];
		voltagesHead=new double [HEAD_JOINTS];
        cmdPositionsHead=new double [HEAD_JOINTS];
        cmdVelocitiesHead=new double [HEAD_JOINTS];


		int k;
	    

	    for(k=0; k<HEAD_JOINTS; k++)
		{
		    cmdPositionsHead[k] = START_POSITION_HEAD[k];
			cmdVelocitiesHead[k] = HEAD_VELOCITY[k];
		}

    }

    ~BodyMover()
    {
      
        delete [] encodersHead;
        delete [] currentsHead;
		delete [] voltagesHead;
		delete [] cmdPositionsHead;
        delete [] cmdVelocitiesHead;
    }

	bool threadInit()
	{
		//change the random seed
		srand( (unsigned)time( NULL ) );
		printf("Initializing seed \n");
		return true;
	}



	void fixedTimeMoveHead(double* startPositionsHead,  double* cmdPositionsHeadHead, double cmdTime)
	{
		//arm movement
		double cmdVelocitiesHeadHead[HEAD_JOINTS];

		int k = 0;
		int j = 0;
		int ii = 0;

		for(k=0; k<HEAD_JOINTS; k++)
		{
			cmdVelocitiesHeadHead[k] = 0;
			
			if (fabs(startPositionsHead[k] - cmdPositionsHeadHead[k]) > 0.1)
				cmdVelocitiesHeadHead[k] = fabs(startPositionsHead[k] - cmdPositionsHeadHead[k])/cmdTime;
			else
				cmdVelocitiesHeadHead[k] = 1.0;
		}
		
		
 		iposHead->setRefSpeeds(cmdVelocitiesHeadHead);
		iposHead->positionMove(cmdPositionsHeadHead);	

		printf("Moving head to:\n");
		for(j=0; j<HEAD_JOINTS; j++)
			printf("%.2lf\t", cmdPositionsHeadHead[j]);
		printf("\n");
		printf("Moving arm with velocity:\n");
		for(ii=0; ii<HEAD_JOINTS; ii++)
			printf("%.2lf\t", cmdVelocitiesHeadHead[ii]);
		printf("\n");

		//Time::delay(cmdTime);
	}

    void run()
    {

		//checkMotionDone(iposRightArm);
		//wait settling time
		//Time::delay(10);
		
		//if (motion_done)
		{


			//print position and current: HEAD 
			iencsHead->getEncoders(encodersHead);
			iampsHead->getCurrents(currentsHead);
			ipidsHead->getOutputs(voltagesHead);

			////////////
			defineNewPositionHead();
			fixedTimeMoveHead(encodersHead, cmdPositionsHead, MOVE_TIME_HEAD);

			waitMotion(iposHead);
		}
    }

    void threadRelease()
	{
	
		iposHead->setRefSpeeds(HEAD_VELOCITY);
		iposHead->positionMove(START_POSITION_HEAD);
	}

	

	void defineNewPositionHead()
	{

		double candidatePos[CONTROLLED_JOINTS_HEAD];    

		//redefine seed of the random generator
        int k;
		for (k=0; k<CONTROLLED_JOINTS_HEAD; k++)
		{
			candidatePos[k] = (MAX_POSITION_HEAD[k] - MIN_POSITION_HEAD[k]) * ((double) rand())/RAND_MAX +  MIN_POSITION_HEAD[k];
		}

		for(k=0; k<CONTROLLED_JOINTS_HEAD; k++)
        {
            if (candidatePos[k]>MAX_POSITION_HEAD[k] || candidatePos[k]<MIN_POSITION_HEAD[k])
                printf("Error: requested position is out of range.\n");
			else
			{
				//cmdVelocitiesHead[k] = fabs(candidatePos[k] - cmdPositionsHead[k])/8;
	            cmdPositionsHead[k]=candidatePos[k];
			}
        }
	}
};

//
int main(int argc, char *argv[]) 
{
    Property armOptions;
   
    Property headOptions;
    headOptions.put("robot", "icub");
    headOptions.put("part", "head");
    headOptions.put("device", "remote_controlboard");
    headOptions.put("local", "/icub/headcontrol/client");    //local port names
    headOptions.put("remote", "/icub/head");					//where we connect to

    Network::init();
	Time::turboBoost();
	initFile();

    // create a device
	PolyDriver headdd(headOptions);

    if (!headdd.isValid()) {
        printf("Device HEAD not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        Network::fini();
        return 0;
    }

    IPidControl *pid;
    IAmplifierControl *amp;
    IPositionControl *pos;

    //// enable amps the robot
    int jnts;

    int i;
	bool ok;

    ok = headdd.view(pid);
    ok &= headdd.view(amp);
    ok &= headdd.view(pos);

    if (!ok) {
        printf("Problems acquiring HEAD interfaces\n");
        Network::fini();
        return 0;
    }

    //// enable amps the robot
    pos->getAxes(&jnts);
	for (i = 0; i < jnts; i++) {
		amp->enableAmp(i);
		pid->enablePid(i);
	}

    pos->setRefSpeeds(HEAD_VELOCITY);
    pos->positionMove(START_POSITION_HEAD);
	waitMotion(pos);
	
    BodyMover *body_mover = new BodyMover( &headdd, SAMPLER_RATE);
    body_mover->start();

    char cmd[80];
    bool quit=false;
    size_t ret;
    while (!quit) 
    {
        printf("Type 'quit+enter' to exit the program\n");
        ret=scanf("%s", cmd);
        if (ret!=1)
            quit=true;

        if (strcmp(cmd, "quit")==0)
            quit=true;
    }

    body_mover->stop();

    delete body_mover;

    //finally close the dd

	headdd.close();

    Network::fini();   

	fclose(OUTPUT_FILE);
    return 0;
}
