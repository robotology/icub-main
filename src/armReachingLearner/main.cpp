#include <stdio.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Time.h>
#include <RFNet.h>
#include <iostream>
#include <yarp/os/Terminator.h>
#undef main

using namespace yarp::os;
using namespace yarp;

const int LEARN1_INPUT_SIZE=4;
const int LEARN1_OUTPUT_SIZE=3;

const double LEARN1_INPUT_NORM[]={360, 360, 360, 360};
const double LEARN1_OUTPUT_NORM[]={90,90,90};

const int DUMP_STEPS=1;

// THINGS TO DO
// -CHECK NETWORK INIT PARAMETERS
// -CHECK NETWORK NORMALIZATION VALUES
// -DATA is appended to the existing file, so the file is never deleted

class CollectorPortHandler:public Thread
{
private:
	Semaphore mutex;
    BufferedPort<Bottle> port;
	Bottle Datum;
	double prev;
	double now;
	bool spoke;
	double spokeTime;
    bool timedOut;
public:
	CollectorPortHandler(): prev(0), now(0), spokeTime(0), spoke(false)
	{
		Datum.addInt(0);
		Datum.addInt(0);
		Datum.addInt(0);
	}

    void open(const char *name)
    {
        port.open(name);
    }
    
    void close()
    {
        port.close();
    }

	virtual void run()
	{
        prev=Time::now();
        now=Time::now();
        spoke=false;
        timedOut=true;
        while(!Thread::isStopping())
            {
                Bottle *v=port.read(false);
                if (v!=0)
                    {
                        mutex.wait();
                        Datum=*v;
                        timedOut=false;
                        mutex.post();
                        prev=Time::now();
                    }

                now=Time::now();

                if( (now-prev)>3 && (!spoke))
                    {
                        fprintf(stderr, "Collector --> warning: port timed out\n");
                        timedOut=true;
                        spoke=true;
                        spokeTime=now;
                    }

                if (now-spokeTime>10)
                    spoke=false;

				Time::delay(0.1);
            }
    }
	
    bool getTimedOut()
    {
        bool ret;
        mutex.wait();
        ret=timedOut;
        mutex.post();
        return ret;
    }
	Bottle get()
	{
		Bottle ret;
		mutex.wait();
		ret=Datum;
		mutex.post();
		return ret;
	}
};

Port *trackerPort;
Port *armMoverPort;
CollectorPortHandler *ballLeftPort;
CollectorPortHandler *ballRightPort; 
dev::IEncoders *iArmEncs;
dev::IPidControl *iArmPids;
dev::IEncoders *iHeadEncs;

class Collector: public RateThread
{
public:
    Collector(): RateThread(0)
    {
        // open ports
     }
    ~Collector()
    {    }

    bool threadInit();
    void threadRelease();
    void run();
	void writeDump();

private:
	double *armEncoders;
	double *headEncoders;
	double *armTensions;

	double *learnDataInput1;
	double *learnDataOutput1;
	double *learnTestData;

	Bottle leftB;
	Bottle rightB;

	RFNet net1;

	int armAxes;
	int headAxes;

	FILE *fp;
	int holdCounter;
	int fixationWas;
    int nSamples;
};

class CollectorJacobian: public RateThread
{
public:
    CollectorJacobian(): RateThread(0)
    {
        // open ports
     }
    ~CollectorJacobian()
    {    }

    bool threadInit();
    void threadRelease();
    void run();
	void writeDump();

private:
	double *armEncoders;
	double *headEncoders;
	double *armTensions;
	double *command;
	double *startPos;

	Bottle leftB;
	Bottle rightB;

	int armAxes;
	int headAxes;

	FILE *fp;
	int holdCounter;
	int fixationWas;
    int nSamples;
};

int main(int argc, const char **argv)
{
	printf("yoh! my name is %s and I like to learn the arm/head map\n", argv[0]);
    Network::init();

	Property armOptions;
    armOptions.put("robot", "james");
    armOptions.put("part", "arm");
    armOptions.put("device", "remote_controlboard");
    armOptions.put("local", "/collector/armcontrol/client");   //local port names
    armOptions.put("remote", "/james/arm");         //where we connect to

	Property headOptions;
    headOptions.put("robot", "james");
    headOptions.put("part", "head");
    headOptions.put("device", "remote_controlboard");
	headOptions.put("subdevice", "jameshead");
    headOptions.put("local", "/collector/headcontrol/client");   //local port names
    headOptions.put("remote", "/james/head");         //where we connect to

    // create arm device
	dev::PolyDriver arm;
	dev::PolyDriver head;
	arm.open(armOptions);
	head.open(headOptions);

	iArmEncs=0;
	iHeadEncs=0;
	bool ret;
	ret=arm.view(iArmEncs);
	ret=ret&&arm.view(iArmPids);
	if (!ret)
		fprintf(stderr, "Problem acquiring arm interfaces, thread will not start\n");
		
	if (!ret)
		return -1;

	ret=ret&&head.view(iHeadEncs);
	if (!ret)
		fprintf(stderr, "Problem acquiring head encoder interface, thread will not start\n");
	
	if (!ret)
		return -1;

    trackerPort=new Port;
	armMoverPort=new Port;
	ballLeftPort=new CollectorPortHandler;
	ballRightPort=new CollectorPortHandler;

	trackerPort->open("/collector/tracker/i");
    ballLeftPort->open("/collector/ball/left/i");
    ballRightPort->open("/collector/ball/right/i");
	armMoverPort->open("/collector/armmover/i");

	ballLeftPort->start();
    ballRightPort->start();

    Collector *collect;
	CollectorJacobian *collectJac;

	collect=new Collector;
	collectJac=new CollectorJacobian;

	if (!collect->start())
		return -1;
	if (!collectJac->start())
		return -1;
    
	Terminee terminee("/collector/quit");
    if (!terminee.isOk()) { 
        fprintf(stderr, "Failed to create proper quit socket\n"); 
       return 1;
    }   


    fprintf(stderr, "Driver instantiated and running (silently)\n");
    while (!terminee.mustQuit()) 
	{
		Time::delay(1); 
    }
		
	fprintf(stderr, "Received a quit message\n");

	ballLeftPort->stop();
	ballRightPort->stop();
	collect->stop();
	collectJac->stop();

	delete collect; 
	delete collectJac;

	delete trackerPort;
	delete ballLeftPort;
	delete ballRightPort;

	arm.close();
	head.close();
	Network::fini();
	return 0;
}

bool Collector::threadInit()
{
	iHeadEncs->getAxes(&headAxes);
	iArmEncs->getAxes(&armAxes);

	armEncoders=new double [armAxes];
	headEncoders=new double [headAxes];
	armTensions=new double [armAxes];
	
	// TODO: CHECK INIT PARAMETERS, these might be wrong
	net1.Init(4,3,0,0,0,0.0000001,50,YVector(LEARN1_INPUT_SIZE, LEARN1_INPUT_NORM),
									  YVector(LEARN1_OUTPUT_SIZE, LEARN1_OUTPUT_NORM));
	
	learnDataInput1=new double [LEARN1_INPUT_SIZE];
	learnDataOutput1=new double [LEARN1_OUTPUT_SIZE];
	learnTestData=new double [LEARN1_OUTPUT_SIZE];

	fp=fopen("arm-learner.txt", "wt");

	fixationWas=0;
	holdCounter=0;
    
    nSamples=0;
	return true;
}

void Collector::threadRelease()
{
  	delete [] armEncoders;
	delete [] headEncoders;
	delete [] armTensions;
	delete [] learnDataInput1;
	delete [] learnDataOutput1;
	delete [] learnTestData;

	fclose(fp);
}

void Collector::run()
{
    //read from the tracker
	Bottle v;
    trackerPort->read(v); //block

	int k=0;
	int fixation=v.get(0).asInt();

	if ( (fixation==1) && (fixationWas!=1) && (holdCounter==0))
	{
		holdCounter=DUMP_STEPS; //hold
	}
	fixationWas=fixation; //save fixation for next loop

    //fprintf(stderr, "%d %d %d\n", fixation, fixationWas, holdCounter);
    if (holdCounter>0)
	{
        holdCounter--;
		iArmEncs->getEncoders(armEncoders);
		iHeadEncs->getEncoders(headEncoders);
		iArmPids->getOutputs(armTensions);

		leftB=ballLeftPort->get();
		rightB=ballRightPort->get();

		for(k=0; k<4; k++)
		{
			learnDataInput1[k]=armEncoders[k];
		}

		learnDataOutput1[0]=headEncoders[1];
		learnDataOutput1[1]=headEncoders[4];
		learnDataOutput1[2]=headEncoders[5];
		
		YVector tmp(3);		
		net1.Simulate(YVector(4, learnDataInput1), 0.01, tmp);
		learnTestData[0]=tmp(1);
		learnTestData[1]=tmp(2);
		learnTestData[2]=tmp(3);

		net1.Train(YVector(4, learnDataInput1), YVector(3, learnDataOutput1));

		double err=0;
		for(int m=0; m<3; m++)
		{
			err+=(learnTestData[m]-learnDataOutput1[m])*(learnTestData[m]-learnDataOutput1[m]);
		}
		fprintf(stderr, "\nError:%lf", err);

		// ok, dump
		writeDump();

	//	printf("\ndump!\n");
    }
	else
	{
		printf(".");
	}
}

void Collector::writeDump()
{
    if ( (ballRightPort->getTimedOut()) && (ballLeftPort->getTimedOut()))
        {
            fprintf(stderr, "left or right ball timed out, skipping dumping\n");
            return;
        }

	int k=0;
	for(k=0;k<4;k++)
		fprintf(fp, "%lf\t", armEncoders[k]);
	for(k=0;k<4;k++)
		fprintf(fp, "%lf\t", armTensions[k]);
	for(k=0;k<headAxes;k++)
		fprintf(fp, "%lf\t", headEncoders[k]);

	fprintf(fp, "%lf\t", rightB.get(0).asDouble());
	fprintf(fp, "%lf\t", rightB.get(1).asDouble());
	fprintf(fp, "%lf\t", rightB.get(2).asDouble());

	fprintf(fp, "%lf\t", leftB.get(0).asDouble());
	fprintf(fp, "%lf\t", leftB.get(1).asDouble());
	fprintf(fp, "%lf\n", leftB.get(2).asDouble());

    nSamples++;
    fprintf(stderr, "\nOpen-loop --> dumped %d samples", nSamples);

    if (nSamples%10==0)
        {
            fprintf(stderr, " Flushing file\n");
            fclose(fp);
            fp=fopen("arm-learner.txt","at");
        }

	//fprintf(fp, "\n");
}


bool CollectorJacobian::threadInit()
{
	iHeadEncs->getAxes(&headAxes);
	iArmEncs->getAxes(&armAxes);

	armEncoders=new double [armAxes];
	headEncoders=new double [headAxes];
	armTensions=new double [armAxes];
	command=new double[4];
	startPos=new double[4];
	fp=fopen("arm-jacobian.txt", "wt");

	fixationWas=0;
    
    nSamples=0;
	return true;
}

void CollectorJacobian::threadRelease()
{
  	delete [] armEncoders;
	delete [] headEncoders;
	delete [] armTensions;
	delete [] command;
	delete [] startPos;

	fclose(fp);
}

void CollectorJacobian::run()
{
    //read from the tracker
	Bottle v;
    armMoverPort->read(v); //block

	int k=0;
	int fixation=v.get(0).asInt();

	if (fixation==1)
	{
		startPos[0]=v.get(1).asDouble();
		startPos[1]=v.get(2).asDouble();
		startPos[2]=v.get(3).asDouble();
		startPos[3]=v.get(4).asDouble();

		command[0]=v.get(5).asDouble();
		command[1]=v.get(6).asDouble();
		command[2]=v.get(7).asDouble();
		command[3]=v.get(8).asDouble();

		// iArmEncs->getEncoders(armEncoders);
		//	iHeadEncs->getEncoders(headEncoders);
		//	iArmPids->getOutputs(armTensions);

		leftB=ballLeftPort->get();
		rightB=ballRightPort->get();

		// ok, dump
		writeDump();
    }
	else
	{
		printf(".");
	}
}

void CollectorJacobian::writeDump()
{
    if ( (ballRightPort->getTimedOut()) && (ballLeftPort->getTimedOut()))
        {
            fprintf(stderr, "left or right ball timed out, skipping dumping\n");
            return;
        }

	int k=0;
	for(k=0;k<4;k++)
		fprintf(fp, "%lf\t", startPos[k]);

	for(k=0;k<4;k++)
		fprintf(fp, "%lf\t", command[k]);

	fprintf(fp, "%lf\t", rightB.get(0).asDouble());
	fprintf(fp, "%lf\t", rightB.get(1).asDouble());
	fprintf(fp, "%lf\t", rightB.get(2).asDouble());

	fprintf(fp, "%lf\t", leftB.get(0).asDouble());
	fprintf(fp, "%lf\t", leftB.get(1).asDouble());
	fprintf(fp, "%lf\n", leftB.get(2).asDouble());

    nSamples++;
    fprintf(stderr, "\nJacobian--> dumped %d samples", nSamples);

    if (nSamples%10==0)
        {
            fprintf(stderr, " Flushing file\n");
            fclose(fp);
            fp=fopen("arm-jacobian.txt","at");
        }

	//fprintf(fp, "\n");
}
