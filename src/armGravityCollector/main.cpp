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

int MOVEMENT_TIME;

FILE *OUTPUT_FILE = fopen("Data.txt","a");

void getNumberOfJoints(const char *initFile, int &n)
{
    Property config;
    config.fromConfigFile(initFile);
    n=(config.findGroup("DIMENSIONS").find("numberOfJoints")).asInt();
    fprintf(stderr, "The loaded file uses %d joints\n", n);
}

void getMovementTime(const char *initFile, int &n)
{
    Property config;
    config.fromConfigFile(initFile);
    Bottle& xtmp = config.findGroup("GENERAL").findGroup("movementTime");
    if (xtmp.size() == 2)
    {
        n = xtmp.get(1).asInt();
        fprintf(stderr, "Setting movementTime to %d \n", n);
    }
    else
    {
        fprintf(stderr, "The loaded file does not have a correct entry movementTime %d\n", xtmp.size());
        n = -1;
    }
}

void getMaxPositions(const char *initFile, double *maxPos, int n)
{
    Property config;
    config.fromConfigFile(initFile);
    Bottle& xtmp = config.findGroup("LIMITS").findGroup("Max");
    if (xtmp.size() == n+1)
        for(int i = 0; i < n; i++) maxPos[i] = xtmp.get(i+1).asDouble();
    else
        fprintf(stderr, "The loaded file has a wrong number of [max] entries\n");
}

void getMinPositions(const char *initFile, double *minPos, int n)
{
    Property config;
    config.fromConfigFile(initFile);
    Bottle& xtmp = config.findGroup("LIMITS").findGroup("Min");
    if (xtmp.size() == n+1)
        for(int i = 0; i < n; i++) minPos[i] = xtmp.get(i+1).asDouble();
    else
        fprintf(stderr, "The loaded file has a wrong number of [min] entries\n");
}

void initFile()
{
	fprintf(OUTPUT_FILE, "theta0[deg] theta1[deg] theta2[deg] theta3[deg] ");
	fprintf(OUTPUT_FILE, "i0[mA] i1[mA] i2[mA] i3[mA] ");
	fprintf(OUTPUT_FILE, "v0[V] v1[V] v2[V] v3[V] ");
	fprintf(OUTPUT_FILE, "\n");
}

void writeFile(double* encs, double *curr, double *volts, int n)
{
    int k;
	for (k=0; k<n; k++)
	{
		fprintf(OUTPUT_FILE, "%.2f ", encs[k]);
	}

	for (k=0; k<n; k++)
	{
		fprintf(OUTPUT_FILE, "%.2f ", curr[k]);
	}

	for (k=0; k<n; k++)
	{
		fprintf(OUTPUT_FILE, "%.2f ", volts[k]*12/1333);
	}
	fprintf(OUTPUT_FILE, "\n");
}


void waitMotion(IPositionControl *ipos)
{
    fprintf(stderr, "CheckMotionDone will mow check the position\n");
    bool done=false;
    //Time::delay(15);
    int c=0;
    Time::delay(0.1);
    while(!done)
    {
        if (ipos->checkMotionDone(&done))
        {
            Time::delay(0.1);
            fprintf(stderr, ".");
        }
        else
        {
            fprintf(stderr, "CheckMotionDone returned false\n");
        }

        c++;
        if (c>100)
            done=true;
    }
    fprintf(stderr, "\n");
}


class ArmMover: public RateThread
{
private:
    IEncoders *iencs;
    IAmplifierControl *iamps;
	IPositionControl *ipos;
	IPidControl *ipids;
    PolyDriver *Arm_dd;
    double *encoders;
	double *currents;
	double *voltages;
	double *cmdPositions;
	double *cmdVelocities;
	double *maxPositions;
	double *minPositions;
    BufferedPort<Vector> portEncs;
    BufferedPort<Vector> portCurrs;
	BufferedPort<Vector> portVolts;
    bool motion_done;
    int numberCmdJoints;
public:
    ArmMover(PolyDriver *Arm_d, 
             int rate,
             int nCmdJoints,
             double *maxPos, 
             double *minPos): RateThread(rate),
        currents(0)
    { 
        Arm_dd=Arm_d;
        Arm_dd->view(iencs);
        Arm_dd->view(iamps);
		Arm_dd->view(ipos);
		Arm_dd->view(ipids);    
        
        numberCmdJoints = nCmdJoints;

        encoders     =new double [numberCmdJoints];
        currents     =new double [numberCmdJoints];
		voltages     =new double [numberCmdJoints];
        cmdPositions =new double [numberCmdJoints];
        cmdVelocities=new double [numberCmdJoints];
        maxPositions =maxPos;
        minPositions =minPos;


        portEncs.open("/icub/arm_gravity_collector/local/encoders");
        portCurrs.open("/icub/arm_gravity_collector/local/currents");
		portVolts.open("/icub/arm_gravity_collector/local/voltages");
        motion_done=false;
    }

    ~ArmMover()
    {
        fprintf(stderr, "Closing ports\n");
        portEncs.close();
        portCurrs.close();
		portVolts.close();
        fprintf(stderr, "Deleting vectors\n");
        delete [] encoders;
        delete [] currents;
		delete [] voltages;
		delete [] cmdPositions;
        delete [] cmdVelocities;
        fprintf(stderr, "Vectors deleted\n");
    }

	void checkMotionDone(IPositionControl *ipos)
	{
		ipos->checkMotionDone(&motion_done);
	}

	bool threadInit()
	{
		//change the random seed
		srand( (unsigned)time( NULL ) );
		printf("Initializing seed \n");
		return true;
	}

	void fixedTimeMove(double *nextPositions, double *startPositions, double cmdTime)
	{
		int k;
		for(k=0; k<numberCmdJoints; k++)
		{
			cmdVelocities[k] = 0;
		
			if (fabs(startPositions[k] - nextPositions[k]) > 0.01)
				cmdVelocities[k] = fabs(startPositions[k] - nextPositions[k])/cmdTime;
			else
				cmdVelocities[k] = 1.0;
		}
		return;
	}
  
    void run()
    {
        fprintf(stderr, "Checking if motion was done\n");
		checkMotionDone(ipos);
        fprintf(stderr, "Motion was checked\n");
		//wait settling time
		//Time::delay(10);
        //double tmpEncs;

		if (motion_done)
		{
			//print position and current
            int k;
            fprintf(stderr, "Getting data\n");
            for (k = 0; k < numberCmdJoints; k++)
            {
                //tmpEncs = encoders + k * sizeof(double);
                //iencs->getEncoder(k, &tmpEncs);
                iencs->getEncoder(k, encoders + k );
                iamps->getCurrent(k, currents + k );
                ipids->getOutput (k, voltages + k );
            }
            fprintf(stderr, "Data Received\n");

			Vector &vEncs=portEncs.prepare();
			Vector &vCurr=portCurrs.prepare();
			Vector &vVolts=portVolts.prepare();

            fprintf(stderr, "Preparing vectors\n");
			vEncs.size(numberCmdJoints);
			vCurr.size(numberCmdJoints);
			vVolts.size(numberCmdJoints);
			for(k=0; k<numberCmdJoints; k++)
			{
				(vEncs)[k]=encoders[k];
				(vCurr)[k]=currents[k];
				(vVolts)[k]=voltages[k];
			}
			writeFile(encoders, currents, voltages, numberCmdJoints);

			portEncs.write();
			portCurrs.write();
			portVolts.write();
            fprintf(stderr, "Vectors written\n");

			//move to a new predifined position

            fprintf(stderr, "Preparing new positions\n");
			defineNewPosition();
            fixedTimeMove(cmdPositions, encoders, MOVEMENT_TIME);
	        printf("Moving arm to:\n");
		    for(k=0; k<numberCmdJoints; k++)
			    printf("%.2lf\t", cmdPositions[k]);
			printf("\n");
	        printf("Moving arm with velocity:\n");
		    for(k=0; k<numberCmdJoints; k++)
			    printf("%.2lf\t", cmdVelocities[k]);
			printf("\n");

			//set new positions
            for (k = 0; k < numberCmdJoints; k++)
                ipos->setRefSpeed(k, cmdVelocities[k]);
			//set new positions
            for (k = 0; k < numberCmdJoints; k++)
			    ipos->positionMove(k, cmdPositions[k]);
            //fprintf(stderr, "New positions sent \n");
            Time::delay(MOVEMENT_TIME);
		}
        else
            fprintf(stderr, "Thread is ciclying without doing nothing\n");
    }

    void threadRelease()
	{
		waitMotion(ipos);
	}

	void defineNewPosition()
	{
		double *candidatePos=new double[numberCmdJoints];

		//redefine seed of the random generator
        int k;
		for (k=0; k<numberCmdJoints; k++)
		{
			candidatePos[k] = (maxPositions[k] - minPositions[k]) * ((double) rand())/RAND_MAX +  minPositions[k];
		}

		for(k=0; k<numberCmdJoints; k++)
        {
            if (candidatePos[k]>maxPositions[k] || candidatePos[k]<minPositions[k])
                printf("Error: requested position is out of range.\n");
			else
			{
				//cmdVelocities[k] = fabs(candidatePos[k] - cmdPositions[k])/8;
	            cmdPositions[k]=candidatePos[k];
			}
        }

        delete[] candidatePos;
	}

};

//
int main(int argc, char *argv[]) 
{

    Property parameters;
    parameters.fromCommand(argc, argv);
    if (!parameters.check("config"))
    {
        fprintf(stderr, "--config file is a mandatory option\n");
        return -1;
    }  
    ConstString inifile= parameters.find("config").asString();
    fprintf(stderr, "Reading positions file from %s\n", inifile.c_str());
    Property config;
    config.fromConfigFile(inifile.c_str());
    
    Property armOptions;
    char robotName[80];
    char partName[80];
    char clientName[80];
    char serverName[80];
    strcpy(robotName, config.findGroup("GENERAL").find("robot").asString().c_str());
    strcpy(partName, config.findGroup("GENERAL").find("part").asString().c_str());
    strcpy(clientName, "/");
    strcat(clientName, robotName);
    strcat(clientName, "/");
    strcat(clientName, partName);
    strcat(clientName, "/");
    strcat(clientName, "client");

    strcpy(serverName, "/");
    strcat(serverName, robotName);
    strcat(serverName, "/");
    strcat(serverName, partName);

    armOptions.put("device", "remote_controlboard");
    armOptions.put("local", clientName);    //local port names
    armOptions.put("remote",serverName);	//where we connect to

    Network::init();
	Time::turboBoost();
	//initFile();

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

    //get number of current part joints
    int nJnts;
    pos->getAxes(&nJnts);
    //get number of joints in the file
    int nCmdJnts;
    getNumberOfJoints(inifile.c_str(), nCmdJnts);
    if (nCmdJnts > nJnts)
    {
        fprintf(stderr, "Current part uses more parts than available\n");
        return -1;
    }          

    double *maxPos = new double[nCmdJnts];
    getMaxPositions(inifile.c_str(), maxPos, nCmdJnts);
    double *minPos = new double[nCmdJnts];
    getMinPositions(inifile.c_str(), minPos, nCmdJnts);

    getMovementTime(inifile.c_str(), MOVEMENT_TIME);
    if (MOVEMENT_TIME <= 1)
    {
        fprintf(stderr, "You have setted a movementTime <=1. Increase it. \n");
        return -1;
    }


    ArmMover *Arm_mover = new ArmMover(&armdd, 
                                       MOVEMENT_TIME*1000, 
                                       nCmdJnts,
                                       maxPos,
                                       minPos);

    Time::delay(1);
    Arm_mover->start();

    char cmd[80];
    bool quit=false;
    while (!quit) 
    {
        ACE_OS::printf("Type 'quit+enter' to exit the program\n");
        scanf("%s", cmd);
        if (strcmp(cmd, "quit")==0)
            quit=true;
    }

    Arm_mover->stop();

    delete Arm_mover;
    fprintf(stderr, "Deleting [max] and [min] vectors\n");
    delete [] maxPos;
    delete [] minPos;
    fprintf(stderr, "[max] and [min] vectors have been deleted\n");

    //finally close the dd
    fprintf(stderr, "Closing the device driver\n");
    armdd.close();
    fprintf(stderr, "Device driver closed\n");

    Network::fini();

    fprintf(stderr, "Closing file\n");
	fclose(OUTPUT_FILE);
    fprintf(stderr, "File closed\n");
    return 0;
}
