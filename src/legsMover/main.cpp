// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <ace/config.h>
#include <ace/OS.h>
#include <ace/Log_Msg.h>

#include <math.h>
#include <time.h>

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

const double LEGS_VELOCITY[12]={10,10,10,10,10,10,10,10,10,10,10,10};
const double START_POSITION_LEGS[12]={0,0,0,0,0,0,0,0,0,0,0,0};
const double MAX_POSITION_LEGS[12]={  10,   4,80, 30, 25, 40, 10,   4,80, 30, 25, 40};
const double MIN_POSITION_LEGS[12]={ -50, -90, 0,-10,-25,-20,-50, -90, 0,-10,-25,-20};

const double MOVE_TIME_LEGS        = 1.25;
const int	 CONTROLLED_JOINTS_LEGS = 12;
const int    LEGS_JOINTS            = 12;

const int    SAMPLER_RATE          = 500;



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
    int k;
	for (k=0; k<CONTROLLED_JOINTS_LEGS; k++)
	{
		fprintf(OUTPUT_FILE, "%.2f ", encs[k]);
	}

	for (k=0; k<CONTROLLED_JOINTS_LEGS; k++)
	{
		fprintf(OUTPUT_FILE, "%.2f ", curr[k]);
	}

	for (k=0; k<CONTROLLED_JOINTS_LEGS; k++)
	{
		fprintf(OUTPUT_FILE, "%.2f ", volts[k]*12/1333);
	}
	fprintf(OUTPUT_FILE, "\n");
}


void waitMotion(IPositionControl *ipos)
{
    bool done=false;
	bool did = false;
    Time::delay(MOVE_TIME_LEGS);
	
	int n = 0; 
	ipos->getAxes(&n);
#if 0                             
    int c=0;
    while(!done)
    {
        if (ipos->checkMotionDone(&done))   
        {
            Time::delay(0.05);
            fprintf(stderr, ".");
        }
        else
        {
			Time::delay(0.05);
            fprintf(stderr, "CheckMotionDone returned false\n");
        }

		if (!done)
		{
			for (int i = 0; i < n; i++)
			{
				ipos->checkMotionDone(i, &did);
				if (!did)
					fprintf(stderr, "Joint %d not done", i);
			}
		}

        c++;
        if (c>4)
            done=true;
    }
    fprintf(stderr, "\n");
#endif
}


class BodyMover: public RateThread
{
private:
    IEncoders *iencsLegs;
    IAmplifierControl *iampsLegs;
	IPositionControl *iposLegs;
	IPidControl *ipidsLegs;
    PolyDriver *Legs_dd;
    double *encodersLegs;
	double *currentsLegs;
	double *voltagesLegs;
	double *cmdPositionsLegs;
	double *cmdVelocitiesLegs;


public:
    BodyMover(PolyDriver *Legs_d, int rate): RateThread(rate)
    { 
        Legs_dd=Legs_d;
        Legs_dd->view(iencsLegs);
        Legs_dd->view(iampsLegs);
		Legs_dd->view(iposLegs);
		Legs_dd->view(ipidsLegs);

        encodersLegs=new double [LEGS_JOINTS];
        currentsLegs=new double [LEGS_JOINTS];
		voltagesLegs=new double [LEGS_JOINTS];
        cmdPositionsLegs=new double [LEGS_JOINTS];
        cmdVelocitiesLegs=new double [LEGS_JOINTS];

 

		int k;
	    for(k=0; k<LEGS_JOINTS; k++)
		{
		    cmdPositionsLegs[k] = START_POSITION_LEGS[k];
			cmdVelocitiesLegs[k] = LEGS_VELOCITY[k];
		}

    }

    ~BodyMover()
    {
        delete [] encodersLegs;
        delete [] currentsLegs;
		delete [] voltagesLegs;
		delete [] cmdPositionsLegs;
        delete [] cmdVelocitiesLegs;

    }

	bool threadInit()
	{
		//change the random seed
		srand( (unsigned)time( NULL ) );
		printf("Initializing seed \n");
		return true;
	}

	void fixedTimeMoveLegs(double* startPositionsLegs,  double* cmdPositionsLegsLegs, double cmdTime)
	{
		//legs movement
		double cmdVelocitiesLegsLegs[LEGS_JOINTS];

		int k = 0;
		int j = 0;
		int ii = 0;

		for(k=0; k<LEGS_JOINTS; k++)
		{
			cmdVelocitiesLegsLegs[k] = 0;
			
			if (fabs(startPositionsLegs[k] - cmdPositionsLegsLegs[k]) > 0.1)
				cmdVelocitiesLegsLegs[k] = fabs(startPositionsLegs[k] - cmdPositionsLegsLegs[k])/cmdTime;
			else
				cmdVelocitiesLegsLegs[k] = 1.0;
		}
		
		//TMP
 		iposLegs->setRefSpeeds(cmdVelocitiesLegsLegs);
		iposLegs->positionMove(cmdPositionsLegsLegs);	

		printf("Moving legs to:\n");
		for(j=0; j<LEGS_JOINTS; j++)
			printf("%.2lf\t", cmdPositionsLegsLegs[j]);
		printf("\n");
		printf("Moving legs with velocity:\n");
		for(ii=0; ii<LEGS_JOINTS; ii++)
			printf("%.2lf\t", cmdVelocitiesLegsLegs[ii]);
		printf("\n");
	}

    void run()
    {

		//checkMotionDone(iposLegs);
		//wait settling time
		//Time::delay(10);
		
		//if (motion_done)
		{
			//print position and current:  legs
			iencsLegs->getEncoders(encodersLegs);
			iampsLegs->getCurrents(currentsLegs);
			ipidsLegs->getOutputs(voltagesLegs);


			////////////
			writeFile(encodersLegs, currentsLegs, voltagesLegs);
			defineNewPositionLegs();
			fixedTimeMoveLegs(encodersLegs, cmdPositionsLegs, MOVE_TIME_LEGS);
			waitMotion(iposLegs);
		}
    }

    void threadRelease()
	{
		iposLegs->setRefSpeeds(LEGS_VELOCITY);
		iposLegs->positionMove(START_POSITION_LEGS);

	}

	void defineNewPositionLegs()
	{

		static int count = 0;
		double candidatePos[CONTROLLED_JOINTS_LEGS];    

		//redefine seed of the random generator
        int k;
		for (k=0; k<CONTROLLED_JOINTS_LEGS; k++)
		{
			candidatePos[k] = (MAX_POSITION_LEGS[k] - MIN_POSITION_LEGS[k]) * ((double) rand())/RAND_MAX +  MIN_POSITION_LEGS[k];
		}

		for(k=0; k<CONTROLLED_JOINTS_LEGS; k++)
        {
            if (candidatePos[k]>MAX_POSITION_LEGS[k] || candidatePos[k]<MIN_POSITION_LEGS[k])
                printf("Error: requested position is out of range.\n");
			else
			{
				//cmdVelocitiesLegs[k] = fabs(candidatePos[k] - cmdPositionsLegs[k])/8;
	            cmdPositionsLegs[k]=candidatePos[k];
			}
        }
/*
		if (count%2==0)
		{
			for(k=CONTROLLED_JOINTS_LEGS+1; k<LEGS_JOINTS; k++)
			{
				cmdPositionsLegs[k] = 0.0;
			}
		}
		else
		{
			cmdPositionsLegs[CONTROLLED_JOINTS_LEGS+1] = 45.0;
			cmdPositionsLegs[CONTROLLED_JOINTS_LEGS+2] = 45.0;
			for(k=CONTROLLED_JOINTS_LEGS+3; k<LEGS_JOINTS-1; k++)
			{
				cmdPositionsLegs[k] = 45.0;
			}
			cmdPositionsLegs[LEGS_JOINTS-1] = 65.0;
		}
*/
		count ++;
	}
};

//
int main(int argc, char *argv[]) 
{
    Property legsOptions;
    legsOptions.put("robot", "icub");
    legsOptions.put("part", "LEGS");
    legsOptions.put("device", "remote_controlboard");
    legsOptions.put("local", "/icub/legscontrol/client");    //local port names
	legsOptions.put("remote", "/icub/legs");					//where we connect to

    Network::init();
	Time::turboBoost();
	
	initFile();

    // create a device
    PolyDriver legsdd(legsOptions);

    if (!legsdd.isValid()) {
        ACE_OS::printf("Device legs not available.  Here are the known devices:\n");
        ACE_OS::printf("%s", Drivers::factory().toString().c_str());
        Network::fini();
        return 0;
    }

    IPidControl *pid;
    IAmplifierControl *amp;
    IPositionControl *pos;

    bool ok;
    ok = legsdd.view(pid);
    ok &= legsdd.view(amp);
    ok &= legsdd.view(pos);

    if (!ok) {
        ACE_OS::printf("Problems acquiring legs interfaces\n");
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

    pos->setRefSpeeds(LEGS_VELOCITY);
    pos->positionMove(START_POSITION_LEGS);
	waitMotion(pos);


    //// enable amps the robot
    pos->getAxes(&jnts);
	for (i = 0; i < jnts; i++) {
		amp->enableAmp(i);
		pid->enablePid(i);
	}

    BodyMover *body_mover = new BodyMover(&legsdd, SAMPLER_RATE);
    body_mover->start();

    char cmd[80];
    bool quit=false;
    while (!quit) 
    {
        ACE_OS::printf("Type 'quit+enter' to exit the program\n");
        scanf("%s", cmd);
        if (strcmp(cmd, "quit")==0)
            quit=true;
    }

    body_mover->stop();

    delete body_mover;

    //finally close the dd
    legsdd.close();

    Network::fini();   

	fclose(OUTPUT_FILE);
    return 0;
}
