// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
*
@ingroup icub_module
\defgroup icubdemoy3 iCubDemoY3

The year 3 demo application.

\section intro_sec Description
This modules move the robot into a series of positions. Repeat forever or until quit
(whichever comes first).

\section lib_sec Libraries
YARP libraries.

\section parameters_sec Parameters
--positions filename.txt

--robot robotname

Optionally:
--verbose: print sequence of positions and speeds

\section portsa_sec Ports Accessed
Access iCubInterface ports.

\section portsc_sec Ports Created
Creates remote_controlboard for each of the robot parts.
 
\section conf_file_sec Configuration Files
The module requires a sequence of positions:
--positions.

The file consists in a few sections:
\code
time 2
\endcode
specifies the time between two movements.

Follows a file for each robot parts. The file contains the list of positions
to cycle.
\code
RIGHTARM armRight1
LEFTARM armLeft1
RIGHTLEG rightLegRemap
LEFTLEG leftLegRemap
HEAD headRemap
TORSO torsoRemap
\endcode

\section tested_os_sec Tested OS
Linux and Windows.

\section example_sec Example Instantiation of the Module
iCubDemoY3 --positions $ICUB_ROOT/app/demoy3/fullBody.txt

\author Lorenzo Natale

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at \in src/iCubDemoY3/main.cpp.
**/

#include <list>
#include <map>
#include <vector>
#include <string>
#include <iostream>

#include <math.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Property.h>
#include <yarp/os/Vocab.h>
#include <yarp/os/Terminator.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/ResourceFinder.h>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp;
using namespace yarp::sig;
using namespace std;

const int FIXED_TIME_MOVE=5;
const int SAMPLER_RATE=100;
////////////////////////////

struct LimbInterfaces
{
    PolyDriver *dd;
    IPositionControl *ipos;
    IEncoders *ienc;
    Vector encoders;
    Vector speed;
    Vector cmd;
    bool *md;
    int nj;
    LimbInterfaces(): dd(0), ipos(0), ienc(0), md(0)
    {}

    ~LimbInterfaces()
    {
        if (!md)
            delete [] md;
    }

    void resize(int n)
    {
        nj=n;
        cmd.resize(nj);
        encoders.resize(nj);
        speed.resize(nj);
        md=new bool[nj];
    }

};

typedef std::list<Vector> PositionList;
typedef PositionList::iterator PositionListIterator;
typedef std::vector<PositionList> RobotSequence;
typedef std::vector<LimbInterfaces> RobotInterfaces;
typedef RobotInterfaces::iterator RobotInterfacesIterator;

enum Index{
    RIGHTARM=0,
    LEFTARM=1,
    HEAD=2,
    TORSO=3,
    RIGHTLEG=4,
    LEFTLEG=5,
    LIMBS=6};

class DemoSequences
{
public:
    std::vector<std::string> tags;
    RobotSequence sequences;
    
    DemoSequences(const std::vector<std::string> &t)
    {
        tags=t;
        sequences.resize(LIMBS);
    }

    bool fromFile(const char *filename)
    {
        Property config;
        config.fromConfigFile(filename);
        bool ret=true;

        Value tmp;
        for(int k=0;k<LIMBS;k++)
            {
                Bottle seqFile=config.findGroup(tags[k].c_str());
                int length=(seqFile.findGroup("DIMENSIONS").find("numberOfPoses")).asInt();
                int nj=seqFile.findGroup("DIMENSIONS").find("numberOfJoints").asInt();
                for(int ii=0;ii<length;ii++)
                    {
                        char tmp[80];
                        if (seqFile.findGroup("REORDER")==0)
                            sprintf(tmp, "POSITION%d", ii);
                        else
                            sprintf(tmp, "POSITION%d", seqFile.findGroup("REORDER").findGroup("order").get(ii+1).asInt());
                        Bottle &xtmp=seqFile.findGroup(tmp).findGroup("jointPositions");
                        Vector vect;
                        vect.resize(nj);
                        for(int l=0;l<xtmp.size();l++)
                            vect[l]=xtmp.get(l+1).asDouble();
                        sequences[k].push_back(vect);
                    }
            }

        return ret;
    }

    void dump()
    {
        for(int l=0;l<LIMBS;l++)
            {
                cout<<"Sequence for "<< tags[l]<<":\n";

                int s=sequences[l].size();
                PositionListIterator it=sequences[l].begin();
                while(it!=sequences[l].end())
                    {
                        Vector &v=(*it);
                        for(int e=0;e<v.length();e++)
                            cout<<v[e]<< " ";
                        cout<<endl;

                        it++;
                    }
                cout<<"-----";
            }
    }
};

class Robot
{
public:

    std::vector<std::string> tags;
    std::vector<Property> options;
    RobotInterfaces interfaces;
    
    Robot()
    {
        tags.resize(LIMBS);
        options.resize(LIMBS);
        interfaces.resize(LIMBS);
    }

    ~Robot()
    {
        RobotInterfacesIterator it;
        for(it=interfaces.begin(); it!=interfaces.end(); it++)
            {
                if((*it).dd!=0)
                    {
                        (*it).dd->close();
                        delete (*it).dd;
                    }
            }
    }

    bool createDevices()
    {
        bool success=true;
        for(int k=0; k<LIMBS;k++)
            {
                LimbInterfaces tmp;
                tmp.dd=new PolyDriver;
                tmp.dd->open(options[k]);
                if (!tmp.dd->isValid())
                    {
                        std::cout<<"Error opening " << tags[k] << endl;
                        return false;
                    }
                
                bool ret=true;
                ret=ret&&tmp.dd->view(tmp.ipos);
                ret=ret&&tmp.dd->view(tmp.ienc);

                interfaces[k]=tmp;
                success=success&&ret;
            }
        return success;
    }
};

class RobotMover: public RateThread
{
private:
	RobotSequence sequences;
    bool sequencesF;

    bool motion_done;
    int nJoints;

	int count;
    Robot *robot;
    bool done;

    bool verbose;

    PositionListIterator *its;
    PositionListIterator *itends;
    double motionTime;
    bool spoke;
public:
    RobotMover(Robot *r, int rate):RateThread(rate)
    { 
        sequencesF=false;
        robot=r;
        verbose=false;
        its=new PositionListIterator [LIMBS];
        itends=new PositionListIterator [LIMBS];
        motionTime=FIXED_TIME_MOVE;
        spoke=false;
    }

    ~RobotMover()
    {
        delete [] its;
        delete [] itends;
    }

    void setVerbose(bool f)
    { verbose=f; }

    void waitMotion(double dT, bool checkDone=false)
    {
        bool done=false;
        Time::delay(dT);
        if (checkDone)
            {
                double acc=0;
                while(!done)
                    {
                        done=true;
                        for(int k=0;k<LIMBS;k++)
                            {
                                bool d=true;
                                bool *tmp=robot->interfaces[k].md;
                                for(int j=0;j<robot->interfaces[k].nj;j++)
                                    {
                                        //robot->interfaces[k].ipos->checkMotionDone(j, tmp);
                                        *tmp=true;
                                        done=done&&(*tmp);
                                        tmp++;
                                    }

                                Time::delay(0.05);
                                acc+=0.05;
                            }
                        Time::delay(0.1);
                        acc+=0.1;

                        if(acc>2)
                            done=true;
                    }

                for(int k=0;k<LIMBS;k++)
                    {
                        bool d=true;
                        cout << robot->tags[k] << " dumping MotionDone:";
                        for(int j=0;j<robot->interfaces[k].nj;j++)
                            cout<<robot->interfaces[k].md[j];
                        
                        cout<<endl;
                    }
            }
    }

    void computeSpeed(double dT)
    {
        if (dT<=0)
            return;

        for(int l=0;l<LIMBS;l++)
            {
                int nj=robot->interfaces[l].nj;
                Vector &encs=robot->interfaces[l].encoders;
                Vector &sp=robot->interfaces[l].speed;
                Vector &cmd=robot->interfaces[l].cmd;
                
                for(int j=0;j<nj;j++)
                    {
                        double dp=fabs(encs[j]-cmd[j]);
                        double tmp=dp/dT;
                        if (tmp<0.1)
                            tmp=0.1;

                        sp[j]=tmp;
                    }
            }
    }

    void resetSequence()
    {
        for(int l=0;l<LIMBS;l++)
            {
                its[l]=sequences[l].begin();
                itends[l]=sequences[l].end();
            }
    }
	
	bool threadInit()
	{
        cout<<"Starting controller..."<<endl;
        if(!sequencesF)
            {
                cout<<"sequences not set, returning false"<<endl;
                return false;
            }

        resetSequence();
        for(int l=0;l<LIMBS;l++)
            {
                int nj;
                robot->interfaces[l].ipos->getAxes(&nj);
                robot->interfaces[l].resize(nj);
                robot->interfaces[l].speed=0;
                robot->interfaces[l].encoders=0;
                robot->interfaces[l].cmd=0;
            }

        done=false;
		return true;
	}

    void run()
    {
        cout << "Running..." << endl;
        spoke=false;
        if(!done)
            {
                for(int l=0;l<LIMBS;l++)
                    {
                        Vector &p=*(its[l]);
                        while (!robot->interfaces[l].ienc->getEncoders(robot->interfaces[l].encoders.data()))
                            {
                                if (!spoke)
                                    {
                                        fprintf(stderr, "Waiting for encoders!\n");
                                        spoke=true;
                                    }
                                
                            }
                        robot->interfaces[l].cmd=p;
                        its[l]++;
                    }

                computeSpeed(motionTime);

                for(int l=0; l<LIMBS;l++)
                    {
                        robot->interfaces[l].ipos->setRefSpeeds(robot->interfaces[l].speed.data());
                        robot->interfaces[l].ipos->positionMove(robot->interfaces[l].cmd.data());

                        if (verbose)
                            {
                                cout<<robot->tags[l];
                                cout<<" going to: "<<robot->interfaces[l].cmd.toString()<<endl;
                                cout<<" speed: "<<robot->interfaces[l].speed.toString()<<endl;
                                cout<<"-------"<<endl;
                            }
                    }

                //wait a bit less, adjustment
                waitMotion(motionTime);
            }

        if (its[0]==itends[0])
            {
                static int count=1;
                resetSequence();
                cout<<"Sequence " << count << " completed\n";
                count++;
                //done=true;
            }
	}

    void setTime(double dT)
    {
        motionTime=dT;
    }

    void waitDone()
    {
        while(!done)
            Time::delay(0.5);
    }

	void suspend()
	{

	}

	void resume()
	{

	}

    void threadRelease()
    {
        cout<<"Stopping controller..."<<endl;
	}

    void setSequences(const RobotSequence &seq)
    {
        sequences=seq;
        sequencesF=true;
    }
};

#include <fstream>
using namespace std;
int main(int argc, char *argv[]) 
{
	// these must go first
	Network yarp;
	Time::turboBoost();

    // pick up name of main configuration file

    // find configuration file via ResourceFinder if needed
    // This is trivial if full path has already been provided,
    // but less trivial if just a filename has been given.
    // There is also the possibility of user overrides in 
    // standardized forms not anticipated in this program.
    ResourceFinder finder;
    finder.setDefaultContext("demoy3");
    finder.configure("ICUB_ROOT",argc,argv);
    finder.setVerbose(true);
    finder.setDefault("positions", "fullBody.txt");
    finder.setDefault("robot", "icub");
    std::string inifile = finder.findFile("positions").c_str();
    std::string robotName=finder.find("robot").asString().c_str();

    bool verbose=finder.check("verbose");
    bool diagnostic=finder.check("diagnostic");

    if (inifile=="") {
        fprintf(stderr, "No position file specified\n");
        return 0;
    } 
    else {
        fprintf(stderr, "Reading positions file from %s\n", inifile.c_str());
    }

    std::string prefix=string("/")+robotName+"/";

    Robot icub;
    
    icub.options[0].put("device", "remote_controlboard");
    icub.options[0].put("local", (prefix+"demo/right_arm/client").c_str()); //local port names
    icub.options[0].put("remote", (prefix+"right_arm").c_str());         //where we connect to
    icub.tags[0]="RIGHTARM";

    icub.options[1].put("device", "remote_controlboard");
    icub.options[1].put("local", (prefix+"demo/left_arm/client").c_str());   //local port names
    icub.options[1].put("remote", (prefix+"left_arm").c_str());         //where we connect to
    icub.tags[1]="LEFTARM";

    icub.options[2].put("device", "remote_controlboard");
    icub.options[2].put("local", (prefix+"demo/head/client").c_str());   //local port names
    icub.options[2].put("remote", (prefix+"head").c_str());         //where we connect to
    icub.tags[2]="HEAD";

    icub.options[3].put("device", "remote_controlboard");
    icub.options[3].put("local", (prefix+"demo/torso/client").c_str());  //local port names
    icub.options[3].put("remote", (prefix+"torso").c_str());         //where we connect to
    icub.tags[3]="TORSO";

    icub.options[4].put("device", "remote_controlboard");
    icub.options[4].put("local", (prefix+"demo/right_leg/client").c_str());   //local port names
    icub.options[4].put("remote", (prefix+"right_leg").c_str());         //where we connect to
    icub.tags[4]="RIGHTLEG";

    icub.options[5].put("device", "remote_controlboard");
    icub.options[5].put("local", (prefix+"demo/left_leg/client").c_str());   //local port names
    icub.options[5].put("remote", (prefix+"left_leg").c_str());         //where we connect to
    icub.tags[5]="LEFTLEG";

    if (diagnostic)
        {
            for (int k=0;k<6; k++)
                icub.options[k].put("diagnostic", "");
        }

    DemoSequences *sequences=new DemoSequences(icub.tags);

    sequences->fromFile(inifile.c_str());
    if (verbose)
        sequences->dump();

    Property op;
    op.fromConfigFile(inifile.c_str());
    double dT=-1;
    if (op.check("time"))
        dT=op.find("time").asDouble();

    if(!icub.createDevices())
    {
        fprintf(stderr, "Error creating devices, check parameters\n");
        return -1;
    }

    RobotMover *robot_mover = new RobotMover(&icub, SAMPLER_RATE);
    robot_mover->setVerbose(verbose);
    robot_mover->setSequences(sequences->sequences);
    if (dT>0)
        {
            cout<<"Setting time to " << dT <<endl;
            robot_mover->setTime(dT);
        }

    robot_mover->start();

    std::string reply;
    while(reply!="quit")
        {
            std::cin>>reply;
        }

	fprintf(stderr, "icub demo --> Received a quit message\n");
    robot_mover->stop();

    delete robot_mover;
    delete sequences;

    return 0;
}
