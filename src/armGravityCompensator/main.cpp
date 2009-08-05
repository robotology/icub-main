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

#include <OnlineSVR.h>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp;
using namespace onlinesvr;

const double ARM_VELOCITY[]={5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5};
const double START_POSITION[]={-15, 90, 35,  -60, 0, 0, 0, 0, 0, 0, 0, 0, 60, -60, 0};
const double STOP_POSITION[]={-30,  0,  0,  -10, 0, 0, 0, 0, 0, 0, 0, 0, 60, -60, 0};

//const double MAX_POSITION[]={   0, 100,90,  -80, 0, 0, 0, 0, 140, 60, 0, 0, 180, -120, 0};
const double MAX_POSITION[]={   0, 100,90,  -80, 0, 0, 0, 0, 0, 0, 0, 0, 60, -60, 0};
const double MIN_POSITION[]={ -40,  70, 0, -100, 0, 0, 0, 0, 0, 0, 0, 0, 60, -60, 0};

const int CONTROLLED_JOINTS=4;
const int ARM_JOINTS=15;
const int SAMPLER_RATE=12000;
const double MOVE_TIME = 5;


void writeFile(FILE *dataFile, double* encs, double *curr, double *volts)
{
	fprintf(dataFile, "Angular positions: ");
    int k;
	for (k=0; k<CONTROLLED_JOINTS; k++)
	{
		fprintf(dataFile, "%.2f; ", encs[k]);
	}
	fprintf(dataFile, "\n");

	fprintf(dataFile, "Motor currents:   ");
	for (k=0; k<CONTROLLED_JOINTS; k++)
	{
		fprintf(dataFile, "%.2f; ", curr[k]);
	}
	fprintf(dataFile, "\n");

	fprintf(dataFile, "Applied voltages:  ");
	for (k=0; k<CONTROLLED_JOINTS; k++)
	{
		fprintf(dataFile, "%.2f; ", volts[k]);
	}
	fprintf(dataFile, "\n");
}

void waitMotion(IPositionControl *ipos)
{
	fprintf(stderr, "Checking if motion has been completed: ");

    bool done[ARM_JOINTS];
		
    int i;
	for (i = 0; i<ARM_JOINTS; i++)
		done[ARM_JOINTS]=false;

	bool done_all =false;

    int c=0;
    while(!done_all)
    {
		ipos->checkMotionDone(&done_all);
		fprintf(stderr, ".");

		//Check all joints in position?
		for (i = 0; i<ARM_JOINTS; i++)
		{
			if (ipos->checkMotionDone(i, &done[i]))
			{
				printf("%d ", done[i]);
			}
			else
			{
				fprintf(stderr, "CheckMotionDone returned false\n");
			}
		}

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

void fixedTimeMove(double* startPositions, double* cmdPositions, double cmdTime, IPositionControl *ipos)
{
	double cmdVelocities[ARM_JOINTS];

    int k;
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

    printf("Moving arm to:\n");
    for(k=0; k<ARM_JOINTS; k++)
	    printf("%.2lf\t", cmdPositions[k]);
	printf("\n");
    printf("Moving arm with velocity:\n");
    for(k=0; k<ARM_JOINTS; k++)
	    printf("%.2lf\t", cmdVelocities[k]);
	printf("\n");

	//wait enough time to have the
	//movement finished
	Time::delay(cmdTime+1);
	waitMotion(ipos);
}

class ArmMover: public RateThread
{
private:
    IEncoders *iencs;
    IAmplifierControl *iamps;
	IPositionControl *ipos;
	IPidControl *ipids;
    PolyDriver *Arm_dd;

	OnlineSVR gravityModel[3];

    double *encoders;
	double *currents;
	double *voltages;
	double *cmdVelocities;
    BufferedPort<yarp::sig::Vector> portEncs;
    BufferedPort<yarp::sig::Vector> portCurrs;
	BufferedPort<yarp::sig::Vector> portVolts;
    bool motion_done;

	FILE *errorFile;
	FILE *dataFile;
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


        portEncs.open("/james/arm_gravity_collector/local/encoders");
        portCurrs.open("/james/arm_gravity_collector/local/currents");
		portVolts.open("/james/arm_gravity_collector/local/voltages");
        motion_done=false;
		errorFile=0;
    }

    ~ArmMover()
    {
        portEncs.close();
        portCurrs.close();
		portVolts.close();
        delete [] encoders;
        delete [] currents;
		delete [] voltages;
        delete [] cmdVelocities;

    }
  
	bool threadInit()
	{

		srand( (unsigned)time( NULL ) );

		gravityModel[0].SetC(1);	
		gravityModel[0].SetEpsilon(0.1);
		gravityModel[0].SetKernelType(OnlineSVR::KERNEL_RBF);
		gravityModel[0].SetKernelParam(1);
		gravityModel[0].SetVerbosity(OnlineSVR::VERBOSITY_NO_MESSAGES);
		gravityModel[0].SetSaveKernelMatrix(true);

		gravityModel[1].SetC(1);
		gravityModel[1].SetEpsilon(0.1);
		gravityModel[1].SetKernelType(OnlineSVR::KERNEL_RBF);
		gravityModel[1].SetKernelParam(1);
		gravityModel[1].SetVerbosity(OnlineSVR::VERBOSITY_NO_MESSAGES);
		gravityModel[1].SetSaveKernelMatrix(true);


		gravityModel[2].SetC(1);
		gravityModel[2].SetEpsilon(0.1);
		gravityModel[2].SetKernelType(OnlineSVR::KERNEL_RBF);
		gravityModel[2].SetKernelParam(1);
		gravityModel[2].SetVerbosity(OnlineSVR::VERBOSITY_NO_MESSAGES);
		gravityModel[2].SetSaveKernelMatrix(true);


		//Load the SVR
		//gravityModel[0].LoadOnlineSVR("gravity1.svr");
		//gravityModel[1].LoadOnlineSVR("gravity2.svr");
		//gravityModel[2].LoadOnlineSVR("gravity3.svr");

		errorFile=fopen("error.log", "w");
		dataFile=fopen("data.txt", "w");
		count=0; 

        return true;
	}

    void run()
    {

		//print position and current
		iencs->getEncoders(encoders);
		iamps->getCurrents(currents);
		ipids->getOutputs(voltages);

        int k;
		for(k=0; k<CONTROLLED_JOINTS; k++)
		{
			voltages[k]*=12.0/1333.0;
		}


		yarp::sig::Vector &vEncs=portEncs.prepare();
		yarp::sig::Vector &vCurr=portCurrs.prepare();
		yarp::sig::Vector &vVolts=portVolts.prepare();

		vEncs.size(CONTROLLED_JOINTS);
		vCurr.size(CONTROLLED_JOINTS);
		vVolts.size(CONTROLLED_JOINTS);
		for(k=0; k<CONTROLLED_JOINTS; k++)
		{
			(vEncs)[k]=encoders[k];
			(vCurr)[k]=currents[k];
			(vVolts)[k]=voltages[k];
		}

		writeFile(dataFile, encoders, currents, voltages);
			double errPrev[3];
			//double errPost[3];

			double* angularPositions;
			angularPositions = new double[3];
			angularPositions[0] = (encoders[0]-MIN_POSITION[0])/(MAX_POSITION[0]-MIN_POSITION[0]);
			angularPositions[1] = (encoders[1]-MIN_POSITION[1])/(MAX_POSITION[1]-MIN_POSITION[1]);
			angularPositions[2] = (encoders[2]-MIN_POSITION[2])/(MAX_POSITION[2]-MIN_POSITION[2]);

			errPrev[0]=gravityModel[0].Margin(angularPositions, voltages[0], 3);
			errPrev[1]=gravityModel[1].Margin(angularPositions, voltages[1], 3);
			errPrev[2]=gravityModel[2].Margin(angularPositions, voltages[2], 3);
			
			gravityModel[0].Train(&angularPositions, voltages, 1, 3);
			gravityModel[1].Train(&angularPositions, voltages+1, 1, 3);
			gravityModel[2].Train(&angularPositions, voltages+2, 1, 3);

			//gravityModel[0].ShowDetails();
			count++; 

			double errPost=gravityModel[0].Margin(encoders, voltages[0], 3);
			printf("[%d] Point: %.4lf %.4lf %.4lf\n", count, voltages[0], voltages[1], voltages[2]);
			printf("[%d] Errors: %.4lf %.4lf %.4lf\n", count, errPrev[0], errPrev[1], errPrev[2]);
			fprintf(errorFile, "%d %.4lf %.4lf %.4lf\n", count, errPrev[0], errPrev[1], errPrev[2]);

			char temp[80];
			sprintf(temp, "gravity%d.svr", count);
			if (count%10==0)
			{
				fclose(errorFile);
				errorFile=fopen("error.log", "a");

				gravityModel[0].SaveOnlineSVR("gravity1.svr");
				gravityModel[1].SaveOnlineSVR("gravity2.svr");
				gravityModel[2].SaveOnlineSVR("gravity3.svr");
				
				fclose(dataFile);
				dataFile=fopen("data.txt", "a");
			}

		portEncs.write();
		portCurrs.write();
		portVolts.write();

		//move to a new predifined position

		defineNewPosition();
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

		fixedTimeMove(encoders, nextPos, MOVE_TIME, ipos);

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
		fclose(dataFile);
		fclose(errorFile);

		waitMotion(ipos);

		iencs->getEncoders(encoders);
		
		double nextPos[ARM_JOINTS];

		for (int k=0; k<ARM_JOINTS; k++)
		{
			nextPos[k] = STOP_POSITION[k];
		}

		fixedTimeMove(encoders, nextPos, MOVE_TIME, ipos);
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


		fixedTimeMove(encoders, candidatePos, MOVE_TIME, ipos);
	}
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
	double start_pos[ARM_JOINTS];
	double next_pos[ARM_JOINTS];

	enc->getEncoders(start_pos);

    int i;
	for (i = 0; i < jnts; i++) {
		amp->enableAmp(i);
		pid->enablePid(i);
		next_pos[i] = START_POSITION[i];
	}

	fixedTimeMove(start_pos, next_pos, MOVE_TIME, pos);

    ArmMover *Arm_mover = new ArmMover(&armdd, SAMPLER_RATE);
    Arm_mover->start();

    char cmd[80];
    bool quit=false;
    while (!quit) 
    {
        ACE_OS::printf("Type 'quit+enter' to exit the program\n");
		ACE_OS::printf("or 'pause/resume +enter' to pause/resume the arm\n");
        scanf("%s", cmd);
        if (strcmp(cmd, "quit")==0)
            quit=true;
		if (strcmp(cmd, "pause")==0)
		{
			Arm_mover->suspend();
		}
		if (strcmp(cmd, "resume")==0)
		{
			Arm_mover->resume();
		}

    }

    Arm_mover->stop();


    delete Arm_mover;

    //finally close the dd
    armdd.close();

    Network::fini();

    return 0;
}
