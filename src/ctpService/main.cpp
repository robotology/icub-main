/** 
\defgroup ctpService Constant Time Position Service
 
@ingroup icub_module 
 
Hidden beyond a sophisticated name lies a simple but useful 
service that will allow you to send constant time position
commands to any of the robot ports.

\section intro_sec Description

Sit silently and waits for incoming commands. Receive position
commands requests and sends position commands, but it computes
the reference speeds s.t. commands are all executed in the same
time irrespectively of the current position (the service clearly
reads the current encoders).

\section ports_sec Ports and Messages

- Opens robot position interface for a given part.
- Opens incoming port, for receiving commands.

Incoming commands are the following:

[ctpc] [time] TIME(seconds) [off] j [pos] list

Example:
This requires 1 second movement of joints 5,6,7 to 10 10 10 respectively:
[ctpc] [time] 1 [off] 5 [pos] (10 10 10)

\section parameters_sec Parameters
Run as:

ctpService --robot robotname --part part

robot: name of robot (e.g. icub)
part: prefix for part name (e.g. right_arm)

\section lib_sec Libraries 
- YARP libraries. 
 
\section tested_os_sec Tested OS
Windows, Linux

Copyright (C) 2010 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.
 
Author: Lorenzo Natale 
*/ 

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <iostream>
#include <iomanip>
#include <string>
#include <deque>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;

#define VCTP_TIME VOCAB4('t','i','m','e')
#define VCTP_OFFSET VOCAB3('o','f','f')
#define VCTP_CMD VOCAB4('c','t','p','c')
#define VCTP_POSITION VOCAB3('p','o','s')

// Class for position control
class ActionItem
{
private:
    int off;
    Vector cmd;
    double time;
public:
    const int &getOffset() const { return off; }
    int &getOffset() { return off; }

    const double &getTime() const { return time; }
    double &getTime() {return time; }

    const Vector &getCmd() const { return cmd; }

    Vector &getCmd() { return cmd; }
};

class Actions
{
    ActionItem data; //this might become a list
public:
    Actions(){}
    ~Actions(){}

    int size()
    {
        return 1;
    }

    ActionItem pop()
    {
        return data;
    }

    void push_back(const ActionItem &v)
    {
        data=(v);
    }
};

class scriptPosPort
{
protected:
    bool verbose;
    bool connected;
    Property          drvOptions;
    PolyDriver       *drv;
    IPositionControl *pos;
    IEncoders        *enc;
    Actions actions;
 
    void send(const ActionItem &x)
    {
        if (!connected)
        {
            cerr<<"Error: not connected to control board skipping"<<endl;
            return;
        }

        int size=x.getCmd().size();
        int offset=x.getOffset();
        double time=x.getTime();
        int nJoints=0;

        enc->getAxes(&nJoints);
        if ((offset+size)>nJoints)
        {
            cerr<<"Error: detected possible overflow, skipping"<<endl;
            cerr<<"For debug --> joints: "<<nJoints<< " off: "<<offset<<" cmd lenght: "<<size<<endl;
        }

        Vector disp(size);

        if (time==0)
        {
            return;
        }

        for (int i=0; i<disp.length(); i++)
        {
            double q;

           
            if (!enc->getEncoder(offset+i,&q))
            {
                 cerr<<"Error: encoders timed out, cannot rely on encoder feedback, aborted"<<endl;
                 return;
            }
                
            disp[i]=x.getCmd()[i]-q;

            if (disp[i]<0.0)
                disp[i]=-disp[i];
        }

        for (int i=0; i<disp.length(); i++)
        {
            pos->setRefSpeed(offset+i,disp[i]/time);
            pos->positionMove(offset+i,x.getCmd()[i]);
        }

        cout << "Script port: " << const_cast<Vector &>(x.getCmd()).toString() << endl;
    }

public:
    scriptPosPort()
    {
        drvOptions.clear();
        drv=0;
        verbose=1;
        connected=false;
    }

    bool configure(const Property &copt)
    {
        bool ret=true;
        Property &options=const_cast<Property &> (copt);

        if (options.check("device"))
            drvOptions.put("device", options.find("device").asString());
        else
            drvOptions.put("device","remote_controlboard");

        std::string name="/localct/";

        std::string remote=std::string("/")+std::string(options.find("robot").asString());
        remote+=name+std::string(options.find("part").asString());

        std::string local=std::string("/")+std::string(options.find("robot").asString());
        local+=std::string("/")+std::string(options.find("part").asString());
   
        drvOptions.put("remote",local.c_str());
        drvOptions.put("local",remote.c_str());


        if (verbose)
        {
            std::cout << "Driver options:\n" << drvOptions.toString().c_str();
        }

        return ret;
     }

    void push_back(const ActionItem &item)
    {
        actions.push_back(item);

        //for now send it immediatly
        send(actions.pop());
    }

    bool connect()
    {
        drv=new PolyDriver(drvOptions);

        if (drv->isValid())
            connected=drv->view(pos)&&drv->view(enc);
        else
            connected=false;

        if (!connected)
        {    
            delete drv;
            drv=0;
        }

        return connected;
    }

    ~scriptPosPort()
    {
        if (drv)
            delete drv;
    }
};


class scriptModule: public RFModule
{
protected:
    Port          rpcPort;
    std::string   name;
    bool          verbose;
    scriptPosPort posPort;

public:
    scriptModule() 
    {
        verbose=true;
    }

    bool handlectpm(const Bottle &cmd, Bottle &reply)
    {
        const int expectedMsgSize=7;
        bool parsed=false;
        if (cmd.size()!=expectedMsgSize)
        {
            return false;
        }

        if (cmd.get(1).asVocab()==VCTP_TIME)
        {
            double time=cmd.get(2).asDouble();
            int offset=0;
            Bottle *posCmd=0;
            if (cmd.get(3).asVocab()==VCTP_OFFSET)
            {
                offset=cmd.get(4).asInt();
                if (cmd.get(5).asVocab()==VCTP_POSITION)
                {
                    posCmd=cmd.get(6).asList();
                    //check posCmd!=0
                    parsed=true;
                    if (verbose)
                    {
                        cout<<"Received command:"<<endl;
                        cout<<"Time: " << time << "offset: " << offset <<" Cmd: " << posCmd->toString()<<endl;
                    }
                    ActionItem tmp;
                    tmp.getCmd().resize(posCmd->size());
                    for(int k=0;k<posCmd->size();k++)
                        tmp.getCmd()[k]=posCmd->get(k).asDouble();

                    tmp.getOffset()=offset;
                    tmp.getTime()=time;
                    posPort.push_back(tmp);
                    return true;
                }
            }
        }
        return false;
    }

    virtual bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();

        if (rf.check("name"))
            name=std::string("/")+rf.find("name").asString().c_str();
        else
            name="/ctpservice";
        name+=std::string("/")+rf.find("part").asString().c_str()+"/rpc";

        rpcPort.open(name.c_str());
        attach(rpcPort);

        rf.find("part").asString();
        rf.find("robot").asString();

        Property portProp;
        portProp.put("part", rf.find("part"));
        portProp.put("robot", rf.find("robot"));

        if (!posPort.configure(portProp))
        {
            cerr<<"Error configuring position controller, check parameters"<<endl;
            return false;
        }
        
        if (!posPort.connect())
        {
            cerr<<"Error cannot conenct to remote ports"<<endl;
            return false;
        }

        cout << "***** connect to rpc port and type \"help\" for commands list" << endl;

        return true;
    }

    virtual bool respond(const Bottle &command, Bottle &reply)
    {
        bool ret;
        if (command.size()!=0)
        {
            switch (command.get(0).asVocab())
            {
                case VOCAB4('h','e','l','p'):
                {                    
                    cout << "Available commands:"          << endl;
                    reply.addVocab(Vocab::encode("ack"));
                    return true;
                }
                case VCTP_CMD:
                    {
                        ret=handlectpm(command, reply);
                        return ret;
                    }
                default:
                    return RFModule::respond(command,reply);
            }
        }
        else
        {
            reply.addVocab(Vocab::encode("nack"));
            return false;
        }
    }

    virtual bool close()
    {
        rpcPort.interrupt();
        rpcPort.close();

        return true;
    }

    virtual double getPeriod()    { return 1.0;  }
    virtual bool   updateModule() { return true; }
};


int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure("ICUB_ROOT",argc,argv);

    if (rf.check("help"))
    {
        cout << "Options:" << endl << endl;
        cout << "\t--name moduleName: set new module name"      << endl;
        cout << "\t--robot robotname: robot name"      << endl;
        cout << "\t--part partname: robot part name" << endl;
        cout << "\t--from   fileName: input configuration file" << endl;
        cout << "\t--context dirName: resource finder context"  << endl;

        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    scriptModule mod;
 
    return mod.runModule(rf);
}



