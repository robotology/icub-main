// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "nr.h"
#include <ace/config.h>
#include <ace/OS.h>
#include <ace/Log_Msg.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Vocab.h>
#include <yarp/String.h>
#include <yarp/os/Semaphore.h>
#include "YARPRndUtils.h"
#include <math.h>
#include <yarp/os/Terminator.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <yarp/sig/Vector.h>
#include <vector>

#include <yarp/os/PortReaderBuffer.h>

#include <RFNet.h>
RFNet armNet;
RFNet armClosedLoop;

YARPRndVector randomVector;
//PositionList exploredPositions;

//#include <OnlineSVR.h>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp;
//using namespace onlinesvr;

const double ARM_VELOCITY[]={5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5};
const double START_POSITION_ARM[]={-30, 90, 0,  -90, 0, 0, 0, 0, 0, 0, 0, 0, 60, -60, 0};
const double STOP_POSITION_ARM[]={-30,  0,  0,  -10, 0, 0, 0, 0, 0, 0, 0, 0, 60, -60, 0};

const double HEAD_VELOCITY[]		={5, 5, 5, 5, 5, 5, 5};
const double START_POSITION_HEAD[]	={0, 0, 0, 0, 90, 0, 0};
const double STOP_POSITION_HEAD[]	={0, 0, 0, 0, 90, 0, 0};

const double MAX_POSITION_ARM[]={   0, 100, 100,  0, 0, 0, 0, 0, 0, 0, 0, 0, 60, -60, 0};
const double MIN_POSITION_ARM[]={ -198,  -99,-40, -90, 0, 0, 0, 0, 0, 0, 0, 0, 60, -60, 0};

const double ARM_INITIAL_GUESS[]={-15, 90, 35,  -60, 0, 0, 0, 0, 0, 0, 0, 0, 60, -60, 0};
const double ARM_INITIAL_GUESS_MIN[]={-40, 72, -14, -90};
const double ARM_INITIAL_GUESS_MAX[]={-1, 100, 85, -31};

const int CONTROLLED_JOINTS_ARM=4;
const int CONTROLLED_JOINTS_HEAD=7;
const int ARM_JOINTS=15;
const int HEAD_JOINTS=7;
const int REACHING_TRIALS=1;

double WEIGHT_FACTOR[]={0,0,1,0};
double Q_CONSTRAINED[]={-100, 0, 35, -45};

const double SLEEP_TIME		   = 0.1;
const double TIME_FOR_FIXATING = 20.0;
const double REST_TIME		   = 1.0;
const double MOVE_TIME		   = 8.0;
const int    SAMPLER_RATE	   = 6000;
const double PI				   = 3.14159265358979;

const double RADIUS_FACTOR = 4.5/4;

const int COLLECTOR_TIME_OUT=1;

//// closed loop parameters
const int CLOSED_LOOP_RATE = 30;
const double CLOSED_LOOP_THRESHOLD = 10;
const double CL_GAIN=0.6;
const double CL_MAX_SPEED=10;
const int CL_TIMEOUT=30;

typedef std::vector<double *> PositionList;
typedef std::vector<double *>::iterator PositionListIt;
typedef std::vector<double *>::const_iterator PositionListConstIt;

//Inverse kinematic functions
void inverse_kinematics(Vector& , Vector& , Vector& );
DP evaluate_error (Vec_I_DP& );

typedef std::vector<double *> PositionList;
typedef std::vector<double *>::iterator PositionListIt;
typedef std::vector<double *>::const_iterator PositionListConstIt;

bool readPositions(const char *filename, PositionList &list)
{
	list.clear();

	FILE *fp=fopen(filename, "rt");

	if (fp==0)
	{
		fprintf(stderr, "Error: could not open %s\n", filename);
		return false;
	}

	int elem=0;
	int k;
	char c=2;
    while(c!=EOF)
	{
		k=0;
        double *tmp=new double [7];
		for(k=0;k<4;k++)
            c=fscanf(fp, "%lf", &tmp[k]);

        list.push_back(tmp);

		for(k=0;k<4;k++)
            c=fscanf(fp, "%lf", &tmp[k]);
    
		for(k=0;k<6;k++)
            c=fscanf(fp, "%lf", &tmp[k]);
        
		elem++;
    }

	printf("Ok, read %d elements\n", elem);
	return true;
}

void waitMotionArm(IPositionControl *iposArm)
{
	//TMP
	//return;
	fprintf(stderr, "Checking if arm motion has been completed: ");

    bool done[ARM_JOINTS];
		
	for (int i = 0; i<ARM_JOINTS; i++)
		done[ARM_JOINTS]=false;

	bool done_all =false;

    int c=0;
    while(!done_all)
    {
		iposArm->checkMotionDone(&done_all);
		fprintf(stderr, ". \n");

		//Check all joints in position?
		for (int ii = 0; ii<ARM_JOINTS; ii++)
		{
			if (iposArm->checkMotionDone(ii, &done[ii]))
			{
				printf("%d ", done[ii]);
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

void waitMotionHead(IPositionControl *iposHead)
{
	//TMP
	return;
	fprintf(stderr, "Checking if head motion has been completed: ");

    bool done[HEAD_JOINTS];
		
	for (int i = 0; i<HEAD_JOINTS; i++)
		done[HEAD_JOINTS]=false;

	bool done_all =false;

    int c=0;
    while(!done_all)
    {
		iposHead->checkMotionDone(&done_all);
		fprintf(stderr, ".");

		//Check all joints in position?
		for (int ii = 0; ii<HEAD_JOINTS; ii++)
		{
			if (iposHead->checkMotionDone(ii, &done[ii]))
			{
				printf("%d ", done[ii]);
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
		Datum.addInt(128);
		Datum.addInt(128);
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

                if( (now-prev)>COLLECTOR_TIME_OUT && (!spoke))
                    {
                        fprintf(stderr, "Warning: port timed out\n");
						mutex.wait();
                        timedOut=true;
						mutex.post();
                        spoke=true;
                        spokeTime=now;
                    }

                if (now-spokeTime>2)
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

//CollectorPortHandler *targetPort;
CollectorPortHandler *handPortLeft;
CollectorPortHandler *handPortRight;

class ClosedLoopThread: public RateThread
{
public:
	Semaphore mutex;
	IPositionControl *ipos;
	IEncoders *iencs;
	IVelocityControl *velArm;
	int clState;
	
	ClosedLoopThread(): RateThread(CLOSED_LOOP_RATE), ipos(0), iencs(0), 
velArm(0) 
	{
		clState=0;
	}

	~ClosedLoopThread(){}
	
	void setInterfaces(IPositionControl *ip, IEncoders *ie, IVelocityControl *vA)
	{ ipos=ip; iencs=ie; velArm=vA;}

	bool threadInit()
	{
		if (ipos==0)
		{
			fprintf(stderr, "ClosedLoopThread --> PositionControl interface not set, aborting\n"); 
			return false;
		}
		
		fprintf(stderr, "ClosedLoopThread --> Starting thread\n"); 

		mutex.wait();
		clState=0;
		mutex.post();

		for (int i = 0; i<15; i++)
			velArm->setRefAcceleration(i,1);

		return true;
	}

	int done()
	{
		int ret;

		mutex.wait();
		ret=clState;
		mutex.post();

		return ret;
	}
	
	void run()
	{
		//double t1 = Time::now();
		double vels[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		//repeat untill error<threshold
		//set isDone when done..
		YVector armPosition(ARM_JOINTS);
		YVector dQ(4);
		YVector Q(4);
		YVector jacobianVector(12);
		YMatrix jacobian(4,3);
		YVector X(3);

		iencs->getEncoders(armPosition.data());

		Q(1)=armPosition(1);
		Q(2)=armPosition(2);
		Q(3)=armPosition(3);
		Q(4)=armPosition(4);
	
		Bottle botLeft=handPortLeft->get();
		Bottle botRight=handPortRight->get();

		X(1)=botRight.get(0).asDouble()+100;
		X(2)=botRight.get(1).asDouble();
		X(3)=botLeft.get(0).asDouble()+100;
		
		
		armClosedLoop.Simulate(Q, 0.001, jacobianVector);

		
		///////// prepare jacobian from vector
		int k=1;
		for (int c=1;c<=3; c++)
		{
			for(int r=1;r<=4;r++)
			{
				jacobian(r,c)=jacobianVector(k); 
				k++;
			}
		}
		///////////////////////////////////////////

		
		double err=X.norm2();
		if (err<CLOSED_LOOP_THRESHOLD)
		{
			fprintf(stderr, "CL->done\n");
			mutex.wait();
			clState=1;
			mutex.post();

			Time::delay(.5);
			velArm->velocityMove(0,0);
			velArm->velocityMove(1,0);
			velArm->velocityMove(2,0);
			velArm->velocityMove(3,0);

		}

		dQ=jacobian*X;
		dQ=-CL_GAIN*dQ;

		/// check time out
		bool leftHandNotVisib = handPortLeft->getTimedOut();
		bool rightHandNotVisib = handPortRight->getTimedOut();
		
		if (rightHandNotVisib || leftHandNotVisib)
		{
			mutex.wait();
			clState=-1;
			mutex.post();
			fprintf(stderr, "CL can't see the hand\n");
			Time::delay(.5);
			velArm->velocityMove(0,0);
			velArm->velocityMove(1,0);
			velArm->velocityMove(2,0);
			velArm->velocityMove(3,0);
			Time::delay(.5);
		}
		else ///////////////// control loop
		{
			fprintf(stderr, "CL error: %lf\n", err);
			clState=0;
			for (int i = 0; i<4; i++)
				vels[i] = dQ(i+1);

			velArm->velocityMove(vels);
		}
				//double t2 = Time::now();
		//fprintf(stderr, "TimeLoopThread --> %.3lf\n", t2-t1);
	}

	void threadRelease()
	{		
		fprintf(stderr, "ClosedLoopThread --> Stopping thread\n"); 
		//velArm->velocityMove(0,0);
		//velArm->velocityMove(1,0);
		//velArm->velocityMove(2,0);
		//velArm->velocityMove(3,0);
		fprintf(stderr, "ClosedLoopThread --> Thread stopped\n"); 
	}

};
ClosedLoopThread *closedLoop;

class feedfwdArmMover: public RateThread
{
private:
	Vector X;		//desired point to be reached

    IEncoders *iencsArm;
    IAmplifierControl *iampsArm;
	IPositionControl *iposArm;
	IPidControl *ipidsArm;

	IEncoders *iencsHead;
    IAmplifierControl *iampsHead;
	IPositionControl *iposHead;
	IPidControl *ipidsHead;

    PolyDriver *Arm_dd;
	PolyDriver *Head_dd;

    double *encodersArm;
	double *voltagesArm;
	double *cmdVelocitiesArm;

    double *encodersHead;
	double *cmdVelocitiesHead;

	bool motion_done;
	int fixation;

	int count;

public:
    feedfwdArmMover(PolyDriver *Arm_d, PolyDriver *Head_d, int rate): RateThread(rate),
		X(3)
    { 
        Arm_dd=Arm_d;
		Head_dd=Head_d;

        Arm_dd->view(iencsArm);
		Head_dd->view(iencsHead);

        Arm_dd->view(iampsArm);
		Arm_dd->view(iposArm);
		Arm_dd->view(ipidsArm);

        Head_dd->view(iampsHead);
		Head_dd->view(iposHead);
		Head_dd->view(ipidsHead);

        encodersArm=new double [ARM_JOINTS];
		voltagesArm=new double [ARM_JOINTS];
        cmdVelocitiesArm=new double [ARM_JOINTS];

        encodersHead=new double [HEAD_JOINTS];
        cmdVelocitiesHead=new double [HEAD_JOINTS];


		int k = 0;

	    for(k=0; k<ARM_JOINTS; k++)
		{
		    encodersArm[k] = START_POSITION_ARM[k];
			cmdVelocitiesArm[k] = ARM_VELOCITY[k];
		}

	    for(k=0; k<HEAD_JOINTS; k++)
		{
		    encodersHead[k] = START_POSITION_HEAD[k];
			cmdVelocitiesHead[k] = HEAD_VELOCITY[k];
		}

		//initializing point to be reached
		//X[0] = 0;	X[1] = 90;	X[2] = 0;
		X[0] =  89;
		X[1] = 4;
		X[2] = 32;
 
        motion_done=false;

		fixation=0;
    }

    ~feedfwdArmMover()
    {
        delete [] encodersArm;
		delete [] voltagesArm;
        delete [] cmdVelocitiesArm;

        delete [] encodersHead;
        delete [] cmdVelocitiesHead;
    }
	
	void setFixation(int v)
	{
		fixation=v;
	}

	void fixedTimeMove(double *startPositionsArm, double  *cmdPositionsArm,
		double *startPositionsHead, double  *cmdPositionsHead, double cmdTime);
	bool waitFixation();
	void resetArm();

	bool threadInit()
	{
		srand( (unsigned)time( NULL ) );
		
		count=0; 
		fixation=0;

		resetArm();

		randomVector.init();
		randomVector.resize(CONTROLLED_JOINTS_ARM,
							ARM_INITIAL_GUESS_MAX,
							ARM_INITIAL_GUESS_MIN);

		return true;
	}

    void run()
    {
		//print position and current
		//TMP
		iencsArm->getEncoders(encodersArm);
		//ipidsArm->getOutputs(voltagesArm);

		for(int k=0; k<CONTROLLED_JOINTS_ARM; k++)
		{
			voltagesArm[k]*=12.0/1333.0;
		}

		//move to a new predifined position

		if (waitFixation())
		{
			int ret=0;

			int trials=0;
			while(ret!=1)
			{
				closedLoop->start();
				pickNewPosition();
			
				ret=0;
				double startT=Time::now();
				double dT=0;
				while(ret==0)
				{
					ret=closedLoop->done();
					if (ret==1)
					{	
						fprintf(stderr, "CL -> received a success \n");
					}
					if (ret==-1)
					{	
						fprintf(stderr, "CL -> received a hand not visible \n");
					}

					Time::delay(0.2);
					
					dT=Time::now()-startT;

					if (dT>CL_TIMEOUT)
					{	
						fprintf(stderr, "CL -> quitting for timeout \n");
						ret=1;  //we go home
					}
				}
				closedLoop->stop();
				Time::delay(0.5);
				trials++;

				if (trials>=REACHING_TRIALS)
					ret=1; //we go home anyway
			}

			Time::delay(1.5);

			resetArm();
		}
	}

	void suspend()
	{
		fprintf(stderr, "Calling suspend...");
		RateThread::suspend();
		resetArm();
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
		fprintf(stderr, "Releasing feedfwdArmMover \n");
		
		int k = 0;
		waitMotionArm(iposArm);
		waitMotionHead(iposHead);

		//arm
		//TMP
		iencsArm->getEncoders(encodersArm);		
		double nextPosArm[ARM_JOINTS];
		for (k=0; k<ARM_JOINTS; k++)
		{
			nextPosArm[k] = STOP_POSITION_ARM[k];
		}

		double nextPosHead[HEAD_JOINTS];
		for (k=0; k<HEAD_JOINTS; k++)
		{
			nextPosHead[k] = STOP_POSITION_HEAD[k];
		}

		fixedTimeMove(encodersArm, nextPosArm, encodersHead, nextPosHead, MOVE_TIME);
	}

	bool checkLimitsArm(double *candidatePosArm)
	{
		bool ret=true;

		for (int k=0;k<CONTROLLED_JOINTS_ARM;k++)
		{
			ret=ret&&(candidatePosArm[k]<MAX_POSITION_ARM[k]);
			ret=ret&&(candidatePosArm[k]>MIN_POSITION_ARM[k]);
		}

		return ret;
	}

	double computeCandidatePositionArm(double *candidatePosArm)
	{
		int m = 0;
		//candidate position is given by the 
		//inverse kinematic function
		Vector Q(4);
		Vector Q0(4);
		Vec_DP Q_NR(4);
		
		YVector tmp=randomVector.getVector();
 
		//double ranIndex=rand()/(double)(RAND_MAX) * expolredPositions.size();
		//double *tmp=exploredPositions[ranIndex];
		//for(m=0;m<CONTROLLED_JOINTS_ARM;m++)
		//	Q0[m] = ARM_INITIAL_GUESS[m];
		
		for(m=0;m<CONTROLLED_JOINTS_ARM;m++)
			Q0[m] = tmp[m];
		
		inverse_kinematics(X, Q, Q0);
		Q_NR[0] = Q[0];
		Q_NR[1] = Q[1]; 
		Q_NR[2] = Q[2];
		Q_NR[3] = Q[3];

		double err = evaluate_error (Q_NR);

		fprintf(stderr, "Distance from desired configuration: %.3lf \n", err);
		
	
		fprintf(stderr, "-->Starting position:\n");
		for(m=0;m<CONTROLLED_JOINTS_ARM;m++)
			fprintf(stderr, "%.1lf\t", Q0[m]);
		fprintf(stderr, "=======\n");
	
		fprintf(stderr, "-->Candidate position:\n");
		for(m=0;m<CONTROLLED_JOINTS_ARM;m++)
		{
			candidatePosArm[m] = Q_NR[m];
			fprintf(stderr, "%.1lf\t", candidatePosArm[m]);
		}
		fprintf(stderr, "=======\n");
		return err;
	}

	void pickNewPosition()
	{		
		double candidatePosArm[ARM_JOINTS];
		double candidatePosHead[HEAD_JOINTS];
		memcpy(candidatePosArm, encodersArm, sizeof(double)*ARM_JOINTS);
		memcpy(candidatePosHead, encodersHead, sizeof(double)*HEAD_JOINTS);

		bool valid=false;

		int count=0;
		double saved;
		saved=Q_CONSTRAINED[2];
		while(!valid)
		{
			//index is now from 0..cube.length()-1
			
			//TMP
			iencsArm->getEncoders(encodersArm);
			
			//copy encoders -> candidatePosArm
			memcpy(candidatePosArm, encodersArm, sizeof(double)*ARM_JOINTS);
			

			//use the inverse kinemantic to 
			//compute a candidate position
			double err=computeCandidatePositionArm(candidatePosArm);


			// now check limits
			valid = checkLimitsArm(candidatePosArm);
			
			if (err>1)
				valid=false;
			Q_CONSTRAINED[2]=Q_CONSTRAINED[2]+5;
			if (!valid)
			{
				
				if (Q_CONSTRAINED[2]>MAX_POSITION_ARM[2])
					Q_CONSTRAINED[2]=MIN_POSITION_ARM[2];
			}
			count++;
			if (count==360/5)
				break;
		}
		//Q_CONSTRAINED[2]=saved;

		for (int ii = 0; ii<HEAD_JOINTS; ii++)
			candidatePosHead[ii] = 0;

		candidatePosHead[1] = X[0];
		candidatePosHead[4] = X[1];
		candidatePosHead[5] = X[2];

		if(valid)
			fixedTimeMove(encodersArm, candidatePosArm, encodersHead, candidatePosHead, MOVE_TIME);
		else
			fprintf(stderr, "Arm command was out of limits, aborting\n");
	}

};

class FixationPort: public BufferedPort<Bottle>
{
private:
	feedfwdArmMover *slave;
public:
	FixationPort(): slave(0){}
	void onRead(Bottle &bot)
	{
		if (slave!=0)
		{
			slave->setFixation(bot.get(0).asInt());
		}
	}

	void setSlave(feedfwdArmMover *s)
	{
		slave=s;
	}
};

//
int main(int argc, char *argv[]) 
{
	// these must go first
	Network::init();
	Time::turboBoost();

	//load the network first
	armNet.LoadNet("openloop.net");
	armClosedLoop.LoadNet("jacobian.net");

	//readPositions(argv[1], exploredPositions);

//	targetPort=new CollectorPortHandler;
//	targetPort->open("/james/armfb/target/i");
//	targetPort->start();

	handPortLeft = new CollectorPortHandler;
	handPortLeft->open("/james/armfb/hand/left/i");
	handPortLeft->start();
	
	handPortRight = new CollectorPortHandler;
	handPortRight->open("/james/armfb/hand/right/i");
	handPortRight->start();

	/// code
	FixationPort fixation;
	fixation.open("/james/armfb/fixation/i");
	fixation.useCallback();

	// arm
    Property armOptions;
    armOptions.put("robot", "james");
    armOptions.put("part", "arm");
    armOptions.put("device", "remote_controlboard");
    armOptions.put("local", "/james/armfb/client");   //local port names
    armOptions.put("remote", "/james/arm");				   //where we connect to

    // create a device
    PolyDriver armdd(armOptions);
    if (!armdd.isValid()) {
        ACE_OS::printf("Device not available.  Here are the known devices:\n");
        ACE_OS::printf("%s", Drivers::factory().toString().c_str());
        Network::fini();
        return 0;
    }

	// head
	Property options;
	options.put("device", "remote_controlboard");
	
	char name1[255];
	char name2[255];
    ACE_OS::sprintf(name1, "/%s/%s/headcontrol_fwd", "james", "head");
    options.put("local", name1);
    ACE_OS::sprintf(name2, "/%s/%s", "james", "head");
	options.put("remote", name2);
	
	fprintf(stderr, "%s", options.toString().c_str());

	// create a device 
    PolyDriver headdd(options);
    if (!headdd.isValid()) {
        ACE_OS::printf("Device not available.  Here are the known devices:\n");
        ACE_OS::printf("%s", Drivers::factory().toString().c_str());
        Network::fini();
        return 1;
    }

    IPidControl *pid;
    IAmplifierControl *amp;
    IPositionControl *pos;
	IEncoders *encArm;
	IVelocityControl *velArm;
	IEncoders *encHead;

    bool ok;
    ok = armdd.view(pid);
    ok &= armdd.view(amp);
    ok &= armdd.view(pos);
	ok &= armdd.view(encArm);
	ok &= armdd.view(velArm);
	ok &= headdd.view(encHead);
	
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

    feedfwdArmMover *arm_mover = new feedfwdArmMover(&armdd, &headdd, SAMPLER_RATE);
	closedLoop = new ClosedLoopThread;

	closedLoop->setInterfaces(pos, encArm, velArm);

	// init fixation callback
	fixation.setSlave(arm_mover);

	fprintf(stderr, "Starting arm thread...");
	arm_mover->start();
	fprintf(stderr, "done!\n");


	//char cmd[80];
    //bool quit=false;
    //while (!quit) 
    //{
    //    ACE_OS::printf("Type 'quit+enter' to exit the program\n");
	//	ACE_OS::printf("or 'pause/resume +enter' to pause/resume the arm\n");
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
	//	}
    //}

	Terminee terminee("/james/armfb/quit");
    if (!terminee.isOk()) { 
        fprintf(stderr, "armFb --> Failed to create proper quit socket\n"); 
       return 1;
    }   

    fprintf(stderr, "armFb --> Module instantiated and running\n");
    while (!terminee.mustQuit()) 
	{
		Time::delay(1); 
    }
		
	fprintf(stderr, "armFb --> Received a quit message\n");

	closedLoop->stop();
    arm_mover->stop();

	fixation.close(); //close port so it does not write to a null arm_mover
    delete arm_mover;
	delete closedLoop;

//	delete targetPort;
	delete handPortLeft;
	delete handPortRight;

    //finally close the dd
    armdd.close();

    Network::fini();

    return 0;
}

///// some functions
//
void feedfwdArmMover::fixedTimeMove(double* startPositionsArm,  double* cmdPositionsArm, 
									double* startPositionsHead, double* cmdPositionsHead, double cmdTime)
{
	//arm movement
	double cmdVelocitiesArm[ARM_JOINTS];

	int k = 0;
	int j = 0;
	int ii = 0;

	for(k=0; k<ARM_JOINTS; k++)
	{
		cmdVelocitiesArm[k] = 0;
		
		if (fabs(startPositionsArm[k] - cmdPositionsArm[k]) > 0.1)
			cmdVelocitiesArm[k] = fabs(startPositionsArm[k] - cmdPositionsArm[k])/cmdTime;
		else
			cmdVelocitiesArm[k] = 1.0;
	}
	
	//TMP
 	iposArm->setRefSpeeds(cmdVelocitiesArm);
	iposArm->positionMove(cmdPositionsArm);	

    printf("Moving arm to:\n");
    for(j=0; j<ARM_JOINTS; j++)
	    printf("%.2lf\t", cmdPositionsArm[j]);
	printf("\n");
    printf("Moving arm with velocity:\n");
    for(ii=0; ii<ARM_JOINTS; ii++)
	    printf("%.2lf\t", cmdVelocitiesArm[ii]);
	printf("\n");

	//head movement
	double cmdVelocitiesHead[HEAD_JOINTS];

	for(k=0; k<HEAD_JOINTS; k++)
	{
		cmdVelocitiesHead[k] = 0;
		
		if (fabs(startPositionsHead[k] - cmdPositionsHead[k]) > 0.1)
			cmdVelocitiesHead[k] = fabs(startPositionsHead[k] - cmdPositionsHead[k])/cmdTime;
		else
			cmdVelocitiesHead[k] = 1.0;
	}

	//TMP
 	// iposHead->setRefSpeeds(cmdVelocitiesHead);
	// iposHead->positionMove(cmdPositionsHead);	

    //wait enough time to have the
	//movement finished
	Time::delay(cmdTime+1);
	//waitMotionArm(iposArm);
	//waitMotionHead(iposHead);
}

void feedfwdArmMover::resetArm()
{
	fprintf(stderr, "Resetting arm! ");
	int i = 0;
	//arm
	double start_pos_arm[ARM_JOINTS];
	double next_pos_arm[ARM_JOINTS];
	//TMP
	iencsArm->getEncoders(start_pos_arm);
	for(i=0;i<ARM_JOINTS; i++)
	{
		next_pos_arm[i] = START_POSITION_ARM[i];
		//TMP
		//start_pos_arm[i]=next_pos_arm[i];
	}

	//head
	double start_pos_head[HEAD_JOINTS];
	double next_pos_head[HEAD_JOINTS];
	//TMP
	//iencsHead->getEncoders(start_pos_head);
	for(i=0;i<HEAD_JOINTS; i++)
	{
		//TMP
		start_pos_head[i] = START_POSITION_HEAD[i];
		next_pos_head[i] = START_POSITION_HEAD[i];
	}

	fixedTimeMove(start_pos_arm, next_pos_arm, start_pos_head, next_pos_head, MOVE_TIME);
}

bool feedfwdArmMover::waitFixation()
{
	///// DEBUG
//	iencsHead->getEncoders(encodersHead);


//	X[0] = encodersHead[4];
//	X[1] = encodersHead[5];
//	X[2] = 0;//bot.get(2).asDouble()*RADIUS_FACTOR;

//	return true;
	//////////////////

	//now wait for fixation
	int count = 0;
	while((fixation!=1) && (count*SLEEP_TIME < TIME_FOR_FIXATING))
	{
		Time::delay(SLEEP_TIME);
		count ++;
		fprintf(stderr, "w"); 
	}

	if (fixation==1)
	{
		fprintf(stderr, "f\n");
		//TMP
		iencsHead->getEncoders(encodersHead);

		//Bottle bot=targetPort->get();

		X[0] = encodersHead[4];
		X[1] = encodersHead[5];
		X[2] = encodersHead[1];//bot.get(2).asDouble()*RADIUS_FACTOR;

		return true;
	}
	else
	{
		fprintf(stderr, "a\n");
		return false;
	}
}

// -----------------------------------
// inverse kinematics computation
// -----------------------------------

// inverse kinematics desired configuration
Vector DESIRED_CFG(3);

void forward_kinematics(Vector& Q, Vector &X)
{
	YVector tmpOutput(3);
	YVector tmpInput(4);
	tmpInput(1)=Q[0];
	tmpInput(2)=Q[1];
	tmpInput(3)=Q[2];
	tmpInput(4)=Q[3];

	armNet.Simulate(tmpInput, 0.01, tmpOutput);

	X[0]=tmpOutput(1);
	X[1]=tmpOutput(2);
	X[2]=tmpOutput(3);
}

DP evaluate_error (Vec_I_DP& Q_NR)
{

	// error of the forward kinematics w.r.t. a given desired position

	// translate from NR's vector representation to YARP's
	Vector X(3), Q(4);
	Q[0] = Q_NR[0]; Q[1] = Q_NR[1]; Q[2] = Q_NR[2];	 Q[3] = Q_NR[3];

	// evaluate forward kinematics
	forward_kinematics(Q,X);

	double err = (X[0] - DESIRED_CFG[0])*(X[0] - DESIRED_CFG[0]) + 
		(X[1] - DESIRED_CFG[1])*(X[1] - DESIRED_CFG[1]) + 
		(X[2] - DESIRED_CFG[2])*(X[2] - DESIRED_CFG[2]) + 
		WEIGHT_FACTOR[0]*(Q[0]-Q_CONSTRAINED[0])*(Q[0]-Q_CONSTRAINED[0])+
		WEIGHT_FACTOR[1]*(Q[1]-Q_CONSTRAINED[1])*(Q[1]-Q_CONSTRAINED[1])+
		WEIGHT_FACTOR[2]*(Q[2]-Q_CONSTRAINED[2])*(Q[2]-Q_CONSTRAINED[2])+
		WEIGHT_FACTOR[3]*(Q[3]-Q_CONSTRAINED[3])*(Q[3]-Q_CONSTRAINED[3]);

	// return norm of distance to desired position; add a punishment term
	// which avoids bending the elbow in a totally innatural way
	return err;
}

inline void get_psum(Mat_I_DP& p, Vec_O_DP& psum)
{
    
	int i,j;
    DP sum;
    int mpts = p.nrows();
    int ndim = p.ncols();
    
    for (j=0;j<ndim;j++) {
		for (sum=0.0,i=0;i<mpts;i++) {
			sum += p[i][j];
		}
		psum[j] = sum;
    }

}

DP amotry(Mat_IO_DP& p, Vec_O_DP& y, Vec_IO_DP& psum, DP funk(Vec_I_DP&), const int ihi, const DP fac)
{
    
    int j;
    DP fac1, fac2, ytry;
    int ndim = p.ncols();
    Vec_DP ptry(ndim);
    
    fac1 = (1.0-fac) / ndim;
    fac2 = fac1 - fac;
    
    for (j=0;j<ndim;j++) {
      ptry[j] = psum[j]*fac1 - p[ihi][j]*fac2;
    }
    
    ytry = funk(ptry);
    
    if (ytry < y[ihi]) {

      y[ihi] = ytry;

      for (j=0;j<ndim;j++) {
        psum[j] += ptry[j] - p[ihi][j];
        p[ihi][j] = ptry[j];
      }

    }

    return ytry;
    
}
  
void amoeba(Mat_IO_DP& p, Vec_IO_DP& y, const DP ftol, DP funk(Vec_I_DP&), int& nfunk)
{
    const int NMAX=5000;
    const DP TINY = 1.0e-10;
    int i, ihi, ilo, inhi, j;
    DP rtol, ysave, ytry;
    int mpts = p.nrows();
    int ndim = p.ncols();
    Vec_DP psum(ndim);

    nfunk = 0;

    get_psum( p, psum );
    
    for (;;) {

      ilo = 0;
      ihi = (y[0]>y[1]) ? (inhi=1,0) : (inhi=0,1);

      for (i=0;i<mpts;i++) {

        if (y[i] <= y[ilo]) {
          ilo = i;
        }

        if (y[i] > y[ihi]) {
          inhi = ihi;
          ihi = i;
        } else if (y[i] > y[inhi] && i != ihi) {
          inhi = i;
        }

      }
      
      rtol = 2.0*fabs(y[ihi]-y[ilo]) / (fabs(y[ihi])+fabs(y[ilo])+TINY);
      
      if (rtol < ftol) {
        SWAP( y[0], y[ilo] );
        for (i=0;i<ndim;i++) {
          SWAP( p[0][i], p[ilo][i] );
        }
        break;
      }
      
      if (nfunk >= NMAX) {
        //        nrerror("NMAX exceeded");
        return;
      }
      
      nfunk += 2;
      ytry = amotry( p, y, psum, funk, ihi, -1.0 );
      
      if (ytry <= y[ilo]) {

        ytry = amotry( p, y, psum, funk, ihi, 2.0 );

      } else if (ytry >= y[inhi]) {

        ysave = y[ihi];
        ytry = amotry( p, y, psum, funk, ihi, 0.5 );
        if (ytry >= ysave) {
          for (i=0;i<mpts;i++) {
            if (i != ilo) {
              for (j=0;j<ndim;j++) {
                p[i][j] = psum[j] = 0.5*(p[i][j]+p[ilo][j]);
              }
              y[i] = funk(psum);
            }
          }
          nfunk += ndim;
          get_psum( p, psum );
        }

      } else {

        --nfunk;

      }

    }
    
}

  
void inverse_kinematics(Vector& X, Vector& Q, Vector& init)
{

	// here we are. you get me the desired Cartesian position 
	// of the e.e., X, and a starting point, init, and I am going
	// to get you the appropriate Qs (IN DEGREES)

	// set desired cfg
	DESIRED_CFG[0] = X[0]; DESIRED_CFG[1] = X[1]; DESIRED_CFG[2] = X[2];

	// four points for the simplex, three are the arguments,
	// tolerance is 1e-6, strting simplex is much bigger
	const int MP = 5,NP = 4;
	const DP FTOL = 1.0e-6;
	const DP simplex_dim = .1;

	// declare x, y, p
	Vec_DP x(NP),y(MP);
	Mat_DP p(MP,NP);
  
	// initialise y and p. p's rows contain the initial simplex coordinates;
	// given the the point init, we build a simplex around it. y is the evaluated
	// in the vertices of the simplex
	int i, nfunc, j;
	for ( i=0; i<MP; i++ ) {
		for ( j=0; j<NP; j++ ) {
			p[i][j] = ( i==(j+1) ? init[j]+simplex_dim : init[j] );
			x[j] = p[i][j];
	    }
		y[i] = evaluate_error(x);
	}
	// go for the gold!
	amoeba(p,y,FTOL,evaluate_error,nfunc);

	// ok, show me what you got!
	Q[0] = p[0][0]; Q[1] = p[0][1]; Q[2] = p[0][2]; Q[3] = p[0][3];

}
