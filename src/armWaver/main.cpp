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
#include <yarp/os/Semaphore.h>

#include <yarp/sig/Image.h>
#include <yarp/sig/ImageFile.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <yarp/sig/Vector.h>
#include <math.h>

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp;

//const double ARM_VELOCITY[]={10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10};
const double ARM_VELOCITY[]={10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10};
//const double START_POSITION[]={-15, 90, 35, -60, 0, 0, 80, 0, 0, 0, 0, 0, 0, 0, 0};
//const double START_POSITION[]={0, 80, 25, -60, 0, 65, 60, 0, 0, 0, 0, 0, 0, 0, 0};
const double START_POSITION[]={-30, 80, 70, -70, 0, 0, 60, 0, 0, 0, 0, 0, 0, 0, 0}; 

// ----------------------------
// evaluation handlocalization
// ----------------------------
bool RANDOM_MOV = false;
// periodic movement
double POSITION1[]={-30, 50, 45, -40, -10, 0, 80, 0, 0, 0, 0, 0, 0, 0, 0};
double POSITION2[]={0, 85, 10, -80, 0, 20, 80, 0, 0, 0, 0, 0, 0, 0, 0};
// evaluation for:
// random movement
const double EVAL_RAND_MIN = 10;
const double EVAL_RAND_MAX = 35;
double p1 = EVAL_RAND_MAX;
double p2 = EVAL_RAND_MIN;

//const double ZERO_POSITION[]={-15, 90, 35, -60, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const double ZERO_POSITION[]={-30, 80, 70, -70, -20, 20, 60, 0, 0, 0, 0, 0, 0, 0, 0};

const int ITERATIONS= 0; //19; //5;

const int ARM_THREAD_RATE=100;
const int DUMPER_RATE=50;  //continuous

bool moving;
double startTime; 
int length;
Semaphore mutex;

/////////////////////////
// smart check
bool motionDone()
{
    mutex.wait();

    bool ret;
    
    if (moving)
    {
        double t=Time::now();

        if ((t-startTime)>(length+0.5))
        {
            moving=false;

        }
    }
    ret=!moving;

    mutex.post();
    return ret;
}
///////////////////////////////////////////

void waitMotion()
{
    fprintf(stderr, "Entering wait\n");
    while(!motionDone())
    {
       Time::delay(0.5);
       fprintf(stderr, ".");
    }
    fprintf(stderr, "done\n");
}

class ArmDumper: public RateThread
{
private:
    IEncoders *iencs;
    PolyDriver *dd;
    double *encoders;
    int nj;

    int count;
    int frame;
    int sequence;
    double beginTime;
    double estPeriod;

    BufferedPort< ImageOf<PixelRgb> > imgPort;

    Semaphore mutex;

    FILE *fp;
public:
    ArmDumper(PolyDriver *d, int rate): RateThread(rate),
        iencs(0),
        encoders(0),
        fp(0),
        sequence(0)
    { 
        dd=d;
        dd->view(iencs);

        iencs->getAxes(&nj);
        encoders=new double [nj];
    }
        

    ~ArmDumper()
    {
        delete [] encoders;
    }

    void startDump()
    {
        mutex.wait();

        sequence++;
        fprintf(stderr, "--> Starting dump of sequence %d\n", sequence);

        char tmp[80];

        if(fp!=0)
           fclose(fp);

        sprintf(tmp, "arm%d.log", sequence);
        
        fp=fopen(tmp,"w");
        frame=0;

        mutex.post();
    }

    void stopDump()
    {
        mutex.wait();

        fprintf(stderr, "--> Stopping dump of sequence %d\n", sequence);

        if (fp!=0)
            fclose(fp);

        fp=0;

        mutex.post();
    }

    void dump(const ImageOf<PixelRgb> *img, const double *encs, int seq, int frame)
    {
        char tmp[80];
        sprintf(tmp, "frame%.2d_%.4d.ppm", seq, frame);

        sig::file::write(*img, tmp);

        for(int k=0;k<nj;k++)
            fprintf(fp, "%.2lf\t", encs[k]);
        
        fprintf(fp, "\n");
    }

    bool threadInit()
    {
        fprintf(stderr, "Starting sampler thread\n");
        imgPort.open("/armWaver/image");
        count=1;
        estPeriod=0.0;
        beginTime=0.0;
        return true;
    }

    void run()
    {
        ImageOf<PixelRgb> *img=imgPort.read(0); //do not block
        
        double prevTime=beginTime;
        beginTime=Time::now();
        estPeriod+=(beginTime-prevTime);

        mutex.wait();
        iencs->getEncoders(encoders);

        if ((fp!=0) && (img!=0))
        {
            dump(img, encoders, sequence, frame);
            frame++;
        }

        mutex.post();

        //if (img!=0)
        count++;
        
        if (count%50==0)
        {
            fprintf(stderr, "Loop %d est time %.3lf[ms]\n", count, 1000*(estPeriod/50));
            estPeriod=0;
        }
    }

    void threadRelease()
    {
        imgPort.close();
        fprintf(stderr, "Closing sampler thread\n");
    } 

    void getPosition(double *p)
    {
        mutex.wait();
        
        memcpy(p, encoders, sizeof(double)*nj);

        mutex.post();
    }

    int axes()
        {return nj;}
};

// should avoid buffering
class ArmThread:public RateThread
{
private:
    ArmDumper *dumper;
    PolyDriver *dd;
    IPositionControl *ipos;
    IAmplifierControl *iamp;
    IPidControl *ipid;
    double *cmdVelocity;
    double *cmdPositions;
    double *lastCommand;
    int nj;

    bool done;
    int iterations;
public:
    ArmThread(PolyDriver *d, int rate): RateThread(rate),
      ipos(0),
      iamp(0),
      ipid(0),
      cmdVelocity(0),
      cmdPositions(0),
      lastCommand(0),
      dumper(0)
    {
        dd=d;
        bool ok=dd->view(ipos);
        ok=ok&&dd->view(ipid);
        ok=ok&&dd->view(iamp);

        ipos->getAxes(&nj);

        cmdVelocity=new double [nj];
        cmdPositions=new double [nj];
        lastCommand=new double [nj];

        for(int i=0;i<nj; i++)
        {
            cmdVelocity[i]=0.0;
            cmdPositions[i]=0.0;
            lastCommand[i]=0.0;
        }

        done=false;
    }

    ~ArmThread()
    {
        if (cmdPositions!=0)
            delete [] cmdPositions;
        if (cmdVelocity!=0)
            delete [] cmdVelocity;
    }

    void setNew(const double *d, int time=8)
    {
        mutex.wait();
            int k;
            length=time;
            for(k=0;k<nj;k++)
            {
                cmdVelocity[k]=fabs(lastCommand[k]-d[k])/time;
                if(cmdVelocity[k]<0.1)
                   cmdVelocity[k]=0.1;

                lastCommand[k]=d[k];
            }
    
           /* 
            for(k=0;k<nj;k++)
                fprintf(stderr, "%lf\t", d[k]);

            fprintf(stderr, "\n");

            for(k=0;k<nj;k++)
                fprintf(stderr, "%lf\t", cmdVelocity[k]);

            fprintf(stderr, "\n");*/

           ipos->setRefSpeeds(cmdVelocity);
           ipos->positionMove(d);

            moving=true;
            startTime=Time::now();
        mutex.post();
    }

    bool threadInit()
    {
        //// enable amps
        int i;
	    for (i = 0; i < nj; i++) {
		    iamp->enableAmp(i);
		    ipid->enablePid(i);
	    }

        //moving to ready position
        setNew(START_POSITION, 8);
        
        waitMotion();

        done=false;
        iterations=0;

        if (dumper!=0)
            dumper->startDump();

		return true;
    }

    void run()
    {
        if (!done)
        {
            if (iterations==ITERATIONS)
            {
                if (motionDone())
                {
                   fprintf(stderr, "Sequence done\n");
                   done=true;
                }
            }
            else if (motionDone())
            {
                iterations++;
        
                fprintf(stderr, "[%d] setting new position\n", iterations);

                if (iterations%2==0)
                {
					// random movement
					if (RANDOM_MOV) {
						p2 = rand();
						while ((p2>EVAL_RAND_MAX) | (p2<EVAL_RAND_MIN) | (abs(p2-p1)<10)) {
							p2=rand();
						}
						POSITION2[2] = p2;
					}
                    // periodic movement
					setNew(POSITION2,4);
                }
                else
                {
					// random movement
					if (RANDOM_MOV) {
						p1 = rand();
						while ((p1>EVAL_RAND_MAX) | (p1<EVAL_RAND_MIN) | (abs(p2-p1)<10)) {
							p1=rand();
						}
						POSITION1[2] = p1;
					}
                    // periodic movement
                    setNew(POSITION1,4);
                }
             }
        }
    }

    void start(double *current)
    {
        done=false;

        for(int k=0; k<nj; k++)
            lastCommand[k]=current[k];
        
        RateThread::start();
    }

    bool isDone()
    {
        return done;
    }

    void threadRelease()
    {
        fprintf(stderr, "Entering ArmThread doRelease\n");

        if (dumper!=0)
            dumper->stopDump();

        fprintf(stderr, "Setting START_POSITION\n");
        setNew(START_POSITION,8);
        waitMotion();

        fprintf(stderr, "Setting ZERO_POSITION\n");
        setNew(ZERO_POSITION,8);
        waitMotion();

        int i;
	    for (i = 0; i < nj; i++) {
		    //iamp->disableAmp(i);
            // ipid->disablePid(i);
	    }
    }

    void setDumper(ArmDumper *d)
    {
        if (d!=0)
            dumper=d;
    }
};

//
int main(int argc, char *argv[]) 
{
    Property armOptions;
    armOptions.put("robot", "james");
    armOptions.put("part", "arm");
    armOptions.put("device", "remote_controlboard");
    armOptions.put("local", "/james/arm-waver/client");    //local port names
    armOptions.put("remote", "/james/arm");                //where we connect to

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

    IEncoders *iencs;
    armdd.view(iencs);

    ArmThread *armThread=new ArmThread(&armdd, ARM_THREAD_RATE);
    ArmDumper *dumper = new ArmDumper(&armdd, DUMPER_RATE);
    dumper->start();

    armThread->setDumper(dumper);

    double *p=new double[dumper->axes()];

    char cmd[80];
    bool quit=false;
    while (!quit) 
    {
        ACE_OS::printf("Type 'quit+enter' to exit the program, or 'go' to begin\n");
        scanf("%s", cmd);
        if (strcmp(cmd, "quit")==0)
            quit=true;

        if (strcmp(cmd, "go")==0)
        {
            ACE_OS::printf("Starting sequence\n");

            dumper->getPosition(p);

            // dumper->startDump();

            // Time::delay(5);

            // dumper->stopDump();

            armThread->start(p);

            fprintf(stderr, "Waiting for sequence to be done\n");

            while(!armThread->isDone())
            {
                fprintf(stderr, ".");
                Time::delay(0.4);
            }
            
            fprintf(stderr, "Stopping sequence\n");

            armThread->stop();
            ACE_OS::printf("End of loop\n");
        }
    } 

    dumper->stop();

    delete armThread;
    delete dumper;

    delete [] p;
    
    //finally close the dd
    armdd.close();

    Network::fini();
    return 0;
}
