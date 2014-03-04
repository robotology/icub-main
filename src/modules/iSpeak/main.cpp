/* 
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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
@ingroup icub_module

\defgroup iSpeak iSpeak
 
Acquire sentences over a yarp port and then let the robot
utter them, also controlling the facial expressions. 

\section intro_sec Description

The behavior is pretty intuitive and does not need any further
detail.\n 
 
\section lib_sec Libraries 
- YARP libraries. 
- Packages for speech synthesis (e.g. festival, espeak, ...).

\section parameters_sec Parameters
--name \e name 
- The parameter \e name identifies the unique stem-name used to 
  open all relevant ports.
 
--robot \e robot 
- The parameter \e robot specifies the robot to connect to. 
 
--period \e T 
- The period given in [ms] for controlling the mouth. 
 
--package \e pck 
- The parameter \e pck specifies the package used for utterance; 
  e.g. "festival", "espeak", ...
 
--package_options \e opt 
- The parameter \e opt is a string specifying further 
  command-line options to be used with the chosen package. Refer
  to the package documentation for the available options.
 
\section portsa_sec Ports Accessed
At startup an attempt is made to connect to 
/<robot>/face/emotions/in port. 

\section portsc_sec Ports Created 
- \e /<name>: this port receives the string for speech
  synthesis. In case a double is received in place of a string,
  then the mouth will be controlled without actually uttering
  any word; that double accounts for the uttering time. \n
  Optionally, as second parameter available in both modalities,
  an integer can be provided that overrides the default period
  used to control the mouth, expressed in [ms]. Negative values
  are not processed and serve as placeholders. \n Finally,
  available only in string mode, a third double can be provided
  that establishes the uttering duration in seconds,
  irrespective of the words actually spoken.
 
- \e /<name>/emotions:o: this port serves to command the facial
  expressions.
 
- \e /<name>/rpc: a remote procedure call port used for the 
  following run-time querie: \n
  - [stat]: returns "speaking" or "quiet".
  - [set] [opt] "package_options": set new package dependent
   command-line options.
  - [get] [opt]: returns a string containing the current
   package dependent command-line options.

\section tested_os_sec Tested OS
Linux and Windows.

\author Ugo Pattacini
*/ 

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <deque>

#include <yarp/os/all.h>

using namespace std;
using namespace yarp::os;


/************************************************************************/
class MouthHandler : public RateThread
{
    string state;
    RpcClient emotions;
    Mutex mutex;
    double t0, duration;

    /************************************************************************/
    void send()
    {
        if (emotions.getOutputCount()>0)
        {
            Bottle cmd, reply;
            cmd.addVocab(Vocab::encode("set"));
            cmd.addVocab(Vocab::encode("mou"));
            cmd.addVocab(Vocab::encode(state.c_str()));
            emotions.write(cmd,reply);
        }
    }

    /************************************************************************/
    void run()
    {
        mutex.lock();

        if (state=="sur")
            state="hap";
        else
            state="sur";

        send();

        mutex.unlock();

        if (duration>=0.0)
            if (Time::now()-t0>=duration)
                suspend();
    }

    /************************************************************************/
    bool threadInit()
    {
        t0=Time::now();
        return true;
    }

    /************************************************************************/
    void threadRelease()
    {
        emotions.interrupt();
        emotions.close();
    }

public:
    /************************************************************************/
    MouthHandler() : RateThread(1000), duration(-1.0) { }

    /************************************************************************/
    void configure(ResourceFinder &rf)
    {
        string name=rf.find("name").asString().c_str();
        string robot=rf.find("robot").asString().c_str();
        emotions.open(("/"+name+"/emotions:o").c_str());

        state="sur";
        setRate(rf.check("period",Value(200)).asInt());
    }

    /************************************************************************/
    void setAutoSuspend(const double duration)
    {
        this->duration=duration;
    }

    /************************************************************************/
    void resume()
    {
        t0=Time::now();
        RateThread::resume();        
    }

    /************************************************************************/
    void suspend()
    {
        if (isSuspended())
            return;

        RateThread::suspend();

        mutex.lock();
        state="hap";
        send();
        mutex.unlock();
    }
};


/************************************************************************/
class iSpeak : protected BufferedPort<Bottle>,
               public    RateThread
{
    string name;
    string package;
    string package_options;
    deque<Bottle> buffer;
    Mutex mutex;
    
    bool speaking;
    MouthHandler mouth;

    /************************************************************************/
    void onRead(Bottle &request)
    {
        mutex.lock();
        buffer.push_back(request);
        mutex.unlock();
    }

    /************************************************************************/
    bool threadInit()
    {
        open(("/"+name).c_str());
        useCallback();
        return true;
    }

    /************************************************************************/
    void threadRelease()
    {
        mouth.stop();
        interrupt();
        close();
    }

    /************************************************************************/
    void speak(const string &phrase)
    {
        string command("echo \"");
        command+=phrase;
        command+="\" | ";
        command+=package;
        command+=" ";

        if (package=="festival")
            command+="--tts ";

        command+=package_options;
        int ret=system(command.c_str());
    }

    /************************************************************************/
    void run()
    {
        string phrase;
        double time;
        bool onlyMouth=false;
        int rate=(int)mouth.getRate();
        bool resetRate=false;
        double duration=-1.0;

        mutex.lock();
        if (buffer.size()>0)    // protect also the access to the size() method
        {
            Bottle request=buffer.front();
            buffer.pop_front();

            if (request.size()>0)
            {
                if (request.get(0).isString())
                {
                    phrase=request.get(0).asString().c_str();
                    speaking=true;
                }
                else if (request.get(0).isDouble() || request.get(0).isInt())
                {
                    time=request.get(0).asDouble();
                    speaking=true;
                    onlyMouth=true;
                }

                if (request.size()>1)
                {
                    if (request.get(1).isInt())
                    {
                        int newRate=request.get(1).asInt();
                        if (newRate>0)
                        {
                            mouth.setRate(newRate);
                            resetRate=true;
                        }
                    }

                    if ((request.size()>2) && request.get(0).isString())
                        if (request.get(2).isDouble() || request.get(2).isInt())
                            duration=request.get(2).asDouble();
                }
            }
        }
        mutex.unlock();

        if (speaking)
        {
            mouth.setAutoSuspend(duration);
            if (mouth.isSuspended())
                mouth.resume();
            else
                mouth.start();

            if (onlyMouth)
                Time::delay(time);
            else
                speak(phrase);

            mouth.suspend();
            if (resetRate)
                mouth.setRate(rate);

            speaking=false;
        }
    }

public:
    /************************************************************************/
    iSpeak() : RateThread(200)
    {
        speaking=false;
    }

    /************************************************************************/
    void configure(ResourceFinder &rf)
    {
        name=rf.find("name").asString().c_str();
        package=rf.find("package").asString().c_str();
        package_options=rf.find("package_options").asString().c_str();

        mouth.configure(rf);

        printf("iSpeak wraps around \"%s\" speech synthesizer\n",package.c_str());
        printf("starting command-line options: \"%s\"\n",package_options.c_str());
    }
    
    /************************************************************************/
    bool isSpeaking() const
    {
        return speaking;
    }

    /************************************************************************/
    string get_package_options() const
    {
        return package_options;
    }

    /************************************************************************/
    void set_package_options(const string &package_options)
    {
        this->package_options=package_options;
    }
};


/************************************************************************/
class Launcher: public RFModule
{
protected:
    iSpeak speaker;
    RpcServer rpc;

public:
    /************************************************************************/
    bool configure(ResourceFinder &rf)
    {
        // request high resolution scheduling
        Time::turboBoost();

        speaker.configure(rf);
        if (!speaker.start())
            return false;

        string name=rf.find("name").asString().c_str();
        rpc.open(("/"+name+"/rpc").c_str());
        attach(rpc);

        return true;
    }

    /************************************************************************/
    bool close()
    {
        rpc.interrupt();
        rpc.close();

        speaker.stop();

        return true;
    }

    /************************************************************************/
    bool respond(const Bottle &command, Bottle &reply)
    {
        int cmd0=command.get(0).asVocab();
        if (cmd0==Vocab::encode("stat"))
        {
            reply.addString(speaker.isSpeaking()?"speaking":"quiet");
            return true;
        }

        if (command.size()>1)
        {
            int cmd1=command.get(1).asVocab();
            if (cmd1==Vocab::encode("opt"))
            {
                if (cmd0==Vocab::encode("get"))
                {
                    reply.addString(speaker.get_package_options().c_str());
                    return true;
                }

                if (cmd0==Vocab::encode("set"))
                {
                    string cmd2=command.get(2).asString().c_str();
                    speaker.set_package_options(cmd2);
                    reply.addString("ack");
                    return true;
                }
            }
        }

        return RFModule::respond(command,reply);
    }

    /************************************************************************/
    double getPeriod()
    {
        return 1.0;
    }

    /************************************************************************/
    bool updateModule()
    {
        return true;
    }
};


/************************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        printf("YARP server not available!\n");
        return -1;
    }

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefault("name","iSpeak");
    rf.setDefault("robot","icub");
    rf.setDefault("package","festival");
    rf.setDefault("package_options","");
    rf.configure(argc,argv);

    Launcher launcher;
    return launcher.runModule(rf);
}


