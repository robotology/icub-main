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

const double RIGHT_ARM_VELOCITY[]={    10.0,  10.0, 10.0, 10.0, 10.0, 5.0, 10.0,  100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0};
const double START_POSITION_RIGHT_ARM[]={-25.8, 20.0,   0,  50,  0.0, 0.0,  0.0, 2500.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0};
const double MAX_POSITION_RIGHT_ARM[]={ -20.8, 90.0,  10,  70,  20,   0,  20, 2500,   15, 90,   0,   0, 90, 90,   0,   0};
const double MIN_POSITION_RIGHT_ARM[]={ -30.5, 20.0, -10,  50, -20, -65, -20, 2500, -105,  0, -90, -90,  0,  0, -90, -65};

const double HEAD_WAIST_VELOCITY[]={      10,  10,  10,  10,  10,  10,  10,    10,  10,  10, 10, 10};
const double START_POSITION_HEAD_WAIST[]={ 0,   0,   0,   0,   0,   0,   0,  4095,   0,   0,  0,  0};
const double MAX_POSITION_HEAD_WAIST[]={   0,  30,  55,  15,  52,  15,  20,  4095,  20,  20,  0,  0};
const double MIN_POSITION_HEAD_WAIST[]={ -40, -40, -55, -35, -50,   0, -20,  4095, -20,  10,  0,  0};


const double MOVE_TIME_RIGHT_ARM         = 1.25;
const int	 CONTROLLED_JOINTS_RIGHT_ARM = 7;
const int    RIGHT_ARM_JOINTS            = 16;

const int    SAMPLER_RATE          = 1000;

const double MOVE_TIME_HEAD_WAIST         = 1.25;
const int	 CONTROLLED_JOINTS_HEAD_WAIST = 10;
const int    HEAD_WAIST_JOINTS            = 12;


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
	for (k=0; k<CONTROLLED_JOINTS_RIGHT_ARM; k++)
	{
		fprintf(OUTPUT_FILE, "%.2f ", encs[k]);
	}

	for (k=0; k<CONTROLLED_JOINTS_RIGHT_ARM; k++)
	{
		fprintf(OUTPUT_FILE, "%.2f ", curr[k]);
	}

	for (k=0; k<CONTROLLED_JOINTS_RIGHT_ARM; k++)
	{
		fprintf(OUTPUT_FILE, "%.2f ", volts[k]*12/1333);
	}
	fprintf(OUTPUT_FILE, "\n");
}


void waitMotion(IPositionControl *ipos)
{
    bool done=false;
	bool did = false;
	
	int n = 0; 
	ipos->getAxes(&n);

    int c=0;
#if 0
    while(!done)
    {
        if (ipos->checkMotionDone(&done))   
        {
            Time::delay(0.05);
            fprintf(stderr, "_");
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
					fprintf(stderr, "Joint %d not done!!!", i);
			}
		}

        c++;
        if (c>5)
            done=true;

            }
    fprintf(stderr, "\n");
#endif
}


class BodyMover: public RateThread
{
private:
    IEncoders *iencsRightArm;
    IAmplifierControl *iampsRightArm;
	IPositionControl *iposRightArm;
	IPidControl *ipidsRightArm;
    PolyDriver *RightArm_dd;
    double *encodersRightArm;
	double *currentsRightArm;
	double *voltagesRightArm;
	double *cmdPositionsRightArm;
	double *cmdVelocitiesRightArm;

    IEncoders *iencsHeadWaist;
    IAmplifierControl *iampsHeadWaist;
	IPositionControl *iposHeadWaist;
	IPidControl *ipidsHeadWaist;
    PolyDriver *HeadWaist_dd;
    double *encodersHeadWaist;
	double *currentsHeadWaist;
	double *voltagesHeadWaist;
	double *cmdPositionsHeadWaist;
	double *cmdVelocitiesHeadWaist;

public:
    BodyMover(PolyDriver *RightArm_d, PolyDriver *HeadWaist_d, int rate): RateThread(rate)
    { 
        RightArm_dd=RightArm_d;
        RightArm_dd->view(iencsRightArm);
        RightArm_dd->view(iampsRightArm);
		RightArm_dd->view(iposRightArm);
		RightArm_dd->view(ipidsRightArm);

        HeadWaist_dd=HeadWaist_d;
        HeadWaist_dd->view(iencsHeadWaist);
        HeadWaist_dd->view(iampsHeadWaist);
		HeadWaist_dd->view(iposHeadWaist);
		HeadWaist_dd->view(ipidsHeadWaist);

        encodersRightArm=new double [RIGHT_ARM_JOINTS];
        currentsRightArm=new double [RIGHT_ARM_JOINTS];
		voltagesRightArm=new double [RIGHT_ARM_JOINTS];
        cmdPositionsRightArm=new double [RIGHT_ARM_JOINTS];
        cmdVelocitiesRightArm=new double [RIGHT_ARM_JOINTS];

        encodersHeadWaist=new double [HEAD_WAIST_JOINTS];
        currentsHeadWaist=new double [HEAD_WAIST_JOINTS];
		voltagesHeadWaist=new double [HEAD_WAIST_JOINTS];
        cmdPositionsHeadWaist=new double [HEAD_WAIST_JOINTS];
        cmdVelocitiesHeadWaist=new double [HEAD_WAIST_JOINTS];


		int k;
	    for(k=0; k<RIGHT_ARM_JOINTS; k++)
		{
		    cmdPositionsRightArm[k] = START_POSITION_RIGHT_ARM[k];
			cmdVelocitiesRightArm[k] = RIGHT_ARM_VELOCITY[k];
		}

	    for(k=0; k<HEAD_WAIST_JOINTS; k++)
		{
		    cmdPositionsHeadWaist[k] = START_POSITION_HEAD_WAIST[k];
			cmdVelocitiesHeadWaist[k] = HEAD_WAIST_VELOCITY[k];
		}

    }

    ~BodyMover()
    {
        delete [] encodersRightArm;
        delete [] currentsRightArm;
		delete [] voltagesRightArm;
		delete [] cmdPositionsRightArm;
        delete [] cmdVelocitiesRightArm;

        delete [] encodersHeadWaist;
        delete [] currentsHeadWaist;
		delete [] voltagesHeadWaist;
		delete [] cmdPositionsHeadWaist;
        delete [] cmdVelocitiesHeadWaist;
    }

	bool threadInit()
	{
		//change the random seed
		srand( (unsigned)time( NULL ) );
		printf("Initializing seed \n");
		return true;
	}

	void fixedTimeMoveRightArm(double* startPositionsRightArm,  double* cmdPositionsRightArmRightArm, double cmdTime)
	{
		//arm movement
		double cmdVelocitiesRightArmRightArm[RIGHT_ARM_JOINTS];

		int k = 0;
		int j = 0;
		int ii = 0;

		for(k=0; k<RIGHT_ARM_JOINTS; k++)
		{
			cmdVelocitiesRightArmRightArm[k] = 0;
			
			if (fabs(startPositionsRightArm[k] - cmdPositionsRightArmRightArm[k]) > 0.1)
				cmdVelocitiesRightArmRightArm[k] = fabs(startPositionsRightArm[k] - cmdPositionsRightArmRightArm[k])/cmdTime;
			else
				cmdVelocitiesRightArmRightArm[k] = 1.0;
		}
		
		//TMP
 		iposRightArm->setRefSpeeds(cmdVelocitiesRightArmRightArm);
		iposRightArm->positionMove(cmdPositionsRightArmRightArm);	

		printf("Moving arm to:\n");
		for(j=0; j<RIGHT_ARM_JOINTS; j++)
			printf("%.2lf\t", cmdPositionsRightArmRightArm[j]);
		printf("\n");
		printf("Moving arm with velocity:\n");
		for(ii=0; ii<RIGHT_ARM_JOINTS; ii++)
			printf("%.2lf\t", cmdVelocitiesRightArmRightArm[ii]);
		printf("\n");
	}

	void fixedTimeMoveHeadWaist(double* startPositionsHeadWaist,  double* cmdPositionsHeadWaistHeadWaist, double cmdTime)
	{
		//arm movement
		double cmdVelocitiesHeadWaistHeadWaist[HEAD_WAIST_JOINTS];

		int k = 0;
		int j = 0;
		int ii = 0;

		for(k=0; k<HEAD_WAIST_JOINTS; k++)
		{
			cmdVelocitiesHeadWaistHeadWaist[k] = 0;
			
			if (fabs(startPositionsHeadWaist[k] - cmdPositionsHeadWaistHeadWaist[k]) > 0.1)
				cmdVelocitiesHeadWaistHeadWaist[k] = fabs(startPositionsHeadWaist[k] - cmdPositionsHeadWaistHeadWaist[k])/cmdTime;
			else
				cmdVelocitiesHeadWaistHeadWaist[k] = 1.0;
		}
		
		
 		iposHeadWaist->setRefSpeeds(cmdVelocitiesHeadWaistHeadWaist);
		iposHeadWaist->positionMove(cmdPositionsHeadWaistHeadWaist);	

		printf("Moving head to:\n");
		for(j=0; j<HEAD_WAIST_JOINTS; j++)
			printf("%.2lf\t", cmdPositionsHeadWaistHeadWaist[j]);
		printf("\n");
		printf("Moving arm with velocity:\n");
		for(ii=0; ii<HEAD_WAIST_JOINTS; ii++)
			printf("%.2lf\t", cmdVelocitiesHeadWaistHeadWaist[ii]);
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
			//print position and current: RIGHT ARM
			iencsRightArm->getEncoders(encodersRightArm);
			iampsRightArm->getCurrents(currentsRightArm);
			ipidsRightArm->getOutputs(voltagesRightArm);


			//print position and current: HEAD WAIST
			iencsHeadWaist->getEncoders(encodersHeadWaist);
			iampsHeadWaist->getCurrents(currentsHeadWaist);
			ipidsHeadWaist->getOutputs(voltagesHeadWaist);

			////////////
			writeFile(encodersRightArm, currentsRightArm, voltagesRightArm);
			defineNewPositionRightArm();
			fixedTimeMoveRightArm(encodersRightArm, cmdPositionsRightArm, MOVE_TIME_RIGHT_ARM);

			defineNewPositionHeadWaist();
			fixedTimeMoveHeadWaist(encodersHeadWaist, cmdPositionsHeadWaist, MOVE_TIME_RIGHT_ARM);

            Time::delay(MOVE_TIME_RIGHT_ARM);
			waitMotion(iposRightArm);
			waitMotion(iposHeadWaist);
		}
    }

    void threadRelease()
	{
		iposRightArm->setRefSpeeds(RIGHT_ARM_VELOCITY);
		iposRightArm->positionMove(START_POSITION_RIGHT_ARM);

		iposHeadWaist->setRefSpeeds(HEAD_WAIST_VELOCITY);
		iposHeadWaist->positionMove(START_POSITION_HEAD_WAIST);
	}

	void defineNewPositionRightArm()
	{

		static int count = 0;
		double candidatePos[CONTROLLED_JOINTS_RIGHT_ARM];    

		//redefine seed of the random generator
        int k;
		for (k=0; k<CONTROLLED_JOINTS_RIGHT_ARM; k++)
		{
			candidatePos[k] = (MAX_POSITION_RIGHT_ARM[k] - MIN_POSITION_RIGHT_ARM[k]) * ((double) rand())/RAND_MAX +  MIN_POSITION_RIGHT_ARM[k];
		}

		for(k=0; k<CONTROLLED_JOINTS_RIGHT_ARM; k++)
        {
            if (candidatePos[k]>MAX_POSITION_RIGHT_ARM[k] || candidatePos[k]<MIN_POSITION_RIGHT_ARM[k])
                printf("Error: requested position is out of range.\n");
			else
			{
				//cmdVelocitiesRightArm[k] = fabs(candidatePos[k] - cmdPositionsRightArm[k])/8;
	            cmdPositionsRightArm[k]=candidatePos[k];
			}
        }
		if (count%2==0)
		{
			for(k=CONTROLLED_JOINTS_RIGHT_ARM+1; k<RIGHT_ARM_JOINTS; k++)
			{
				cmdPositionsRightArm[k] = 0.0;
			}
		}
		else
		{
			cmdPositionsRightArm[CONTROLLED_JOINTS_RIGHT_ARM+1] = 25.0;
			cmdPositionsRightArm[CONTROLLED_JOINTS_RIGHT_ARM+2] = 25.0;
			for(k=CONTROLLED_JOINTS_RIGHT_ARM+3; k<RIGHT_ARM_JOINTS-1; k++)
			{
				cmdPositionsRightArm[k] = 25.0;
			}
			cmdPositionsRightArm[RIGHT_ARM_JOINTS-1] = 45.0;
		}
		count ++;  
	}

	void defineNewPositionHeadWaist()
	{

		double candidatePos[CONTROLLED_JOINTS_HEAD_WAIST];    

		//redefine seed of the random generator
        int k;
		for (k=0; k<CONTROLLED_JOINTS_HEAD_WAIST; k++)
		{
			candidatePos[k] = (MAX_POSITION_HEAD_WAIST[k] - MIN_POSITION_HEAD_WAIST[k]) * ((double) rand())/RAND_MAX +  MIN_POSITION_HEAD_WAIST[k];
		}

		for(k=0; k<CONTROLLED_JOINTS_HEAD_WAIST; k++)
        {
            if (candidatePos[k]>MAX_POSITION_HEAD_WAIST[k] || candidatePos[k]<MIN_POSITION_HEAD_WAIST[k])
                printf("Error: requested position is out of range.\n");
			else
			{
				//cmdVelocitiesHeadWaist[k] = fabs(candidatePos[k] - cmdPositionsHeadWaist[k])/8;
	            cmdPositionsHeadWaist[k]=candidatePos[k];
			}
        }
	}
};

//
int main(int argc, char *argv[]) 
{
    Property armOptions;
    armOptions.put("robot", "icub");
    armOptions.put("part", "right_arm");
    armOptions.put("device", "remote_controlboard");
    armOptions.put("local", "/icub/armcontrol/client");    //local port names
    armOptions.put("remote", "/icub/right_arm");					//where we connect to

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
    PolyDriver armdd(armOptions);
	PolyDriver headdd(headOptions);
    if (!armdd.isValid()) {
        ACE_OS::printf("Device ARM not available.  Here are the known devices:\n");
        ACE_OS::printf("%s", Drivers::factory().toString().c_str());
        Network::fini();
        return 0;
    }

    if (!headdd.isValid()) {
        ACE_OS::printf("Device HEAD not available.  Here are the known devices:\n");
        ACE_OS::printf("%s", Drivers::factory().toString().c_str());
        Network::fini();
        return 0;
    }

    Network::connect("/icub/armcontrol/client/rpc:o", "/icub/left_arm/rpc:i");
    Network::connect("/icub/armcontrol/client/command:o", "/icub/left_arm/command:i");

    IPidControl *pid;
    IAmplifierControl *amp;
    IPositionControl *pos;

    bool ok;
    ok = armdd.view(pid);
    ok &= armdd.view(amp);
    ok &= armdd.view(pos);

    if (!ok) {
        ACE_OS::printf("Problems acquiring ARM interfaces\n");
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

    pos->setRefSpeeds(RIGHT_ARM_VELOCITY);
    pos->positionMove(START_POSITION_RIGHT_ARM);
	waitMotion(pos);

    ok = headdd.view(pid);
    ok &= headdd.view(amp);
    ok &= headdd.view(pos);

    if (!ok) {
        ACE_OS::printf("Problems acquiring HEAD interfaces\n");
        Network::fini();
        return 0;
    }

    //// enable amps the robot
    pos->getAxes(&jnts);
	for (i = 0; i < jnts; i++) {
		amp->enableAmp(i);
		pid->enablePid(i);
	}

    pos->setRefSpeeds(HEAD_WAIST_VELOCITY);
    pos->positionMove(START_POSITION_HEAD_WAIST);
	waitMotion(pos);
	
    BodyMover *body_mover = new BodyMover(&armdd, &headdd, SAMPLER_RATE);
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
    armdd.close();
	headdd.close();

    Network::fini();   

	fclose(OUTPUT_FILE);
    return 0;
}
