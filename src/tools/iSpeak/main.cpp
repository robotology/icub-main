/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

/**
@ingroup icub_tools

\defgroup iSpeak iSpeak
 
Acquire sentences over a yarp port and then let the robot
pronounce them, also controlling the facial expressions. 

\section intro_sec Description

The behavior is pretty intuitive and does not need any further
detail.\n 
This module has been tested only on Linux since it requires the 
<b>festival</b> package: <i>sudo apt-get install festival</i>.

\section lib_sec Libraries 
- YARP libraries. 
- Festival package for speech synthesis. 

\section parameters_sec Parameters
--name \e name 
- The parameter \e name identifies the unique stem-name used to 
  open all relevant ports.
 
--robot \e robot 
- The parameter \e robot specifies the robot to connect to. 
 
\section portsa_sec Ports Accessed
At startup an attempt is made to connect to 
/<robot>/face/emotions port. 

\section portsc_sec Ports Created 
- \e /<name>: this port receives the request for speech 
  synthesis as a property-like bottle. Each bottle must
  contain the "phrase" and "time" information. For example the
  bottle (phrase "down with the service robotics") (time 10.0)
  will let the robot speak the phrase "down with the service
  robotics" and at the same time the facial expression will be
  changed for 10 seconds to mimic the proper mouth movement.
  Obvioulsy, 10 seconds are really too much for that particular
  phrase: this was meant to underline that the time tuning is
  left to you :) .
 
- \e /<name>/emotions: this port serves to command the facial 
  expressions. At startup an attempt to connect to the proper
  robot port is automatically made.
 
- \e /<name>/rpc: a remote procedure call port useful to query
  whether the robot is still speaking or not: the query command
  is the vocab [stat], whereas the response will be a string:
  either "speaking" or "quiet".

\section tested_os_sec Tested OS
Linux. 

\author Ugo Pattacini
*/ 

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Port.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <deque>

using namespace std;
using namespace yarp::os;


class iSpeak : public BufferedPort<Property>,
               public RateThread
{
    string name;
    string robot;
    Port emotions;

    deque<Property> buffer;
    Semaphore mutex;

    double t0;
    double dt;
    string state;
    bool speaking;

public:
    iSpeak() : RateThread(200)
    {
        speaking=false;
        state="sur";
    }

    void configure(ResourceFinder &rf)
    {
        name=rf.find("name").asString().c_str();
        robot=rf.find("robot").asString().c_str();
    }

    bool threadInit()
    {
        open(("/"+name).c_str());
        useCallback();

        emotions.open(("/"+name+"/emotions").c_str());
        Network::connect(emotions.getName().c_str(),("/"+robot+"/face/emotions/in").c_str());

        return true;
    }

    void threadRelease()
    {
        interrupt();
        close();

        emotions.interrupt();
        emotions.close();
    }

    void onRead(Property &request)
    {
        mutex.wait();
        buffer.push_back(request);
        mutex.post();
    }

    void speak(Property &request)
    {
        string phrase=request.check("phrase",Value("I have received nothing to say")).asString().c_str();
        double time=request.check("time",Value(1.0)).asDouble();

        system(("echo \""+phrase+"\" | festival --batch --tts &").c_str());

        t0=Time::now();
        dt=time;
        speaking=true;
    }

    void run()
    {
        mutex.wait();
        if (buffer.size()>0)
        {
            speak(buffer.front());
            buffer.pop_front();
        }
        mutex.post();

        if (speaking)
        {
            if (state=="sur")
                state="hap";
            else
                state="sur";

            if (Time::now()-t0>dt)
            {
                state="hap";
                speaking=false;
            }

            Bottle cmd, reply;
            cmd.addVocab(Vocab::encode("set"));
            cmd.addVocab(Vocab::encode("mou"));
            cmd.addVocab(Vocab::encode(state.c_str()));
            emotions.write(cmd,reply);
        }        
    }
    
    bool isSpeaking() const
    {
        return speaking;
    }    
};


class Launcher: public RFModule
{
protected:
    iSpeak speaker;
    Port   rpc;

public:
    bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();

        speaker.configure(rf);
        if (!speaker.start())
            return false;

        string name=rf.find("name").asString().c_str();
        rpc.open(("/"+name+"/rpc").c_str());
        attach(rpc);

        return true;
    }

    bool close()
    {
        rpc.interrupt();
        rpc.close();

        speaker.stop();

        return true;
    }

    bool respond(const Bottle &command, Bottle &reply)
    {
        if (command.get(0).asVocab()==VOCAB4('s','t','a','t'))
        {
            reply.addString(speaker.isSpeaking()?"speaking":"quiet");
            return true;
        }
        else
            return RFModule::respond(command,reply);
    }

    double getPeriod()
    {
        return 1.0;
    }

    bool updateModule()
    {
        return true;
    }
};


int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
        return -1;

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefault("name","iSpeak");
    rf.setDefault("robot","icub");
    rf.configure("ICUB_ROOT",argc,argv);

    Launcher launcher;
    return launcher.runModule(rf);
}


