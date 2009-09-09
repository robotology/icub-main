// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
 
/** 
@ingroup icub_module

\defgroup icub_iCubInterface iCubInterface2

Interface application to the robot.

\section intro_sec Description

Interface to the robot icub. This process instantiates most of the 
devices 
controlling the robot. This include:
CAN bus devices for all CAN networks (head-torso, legs, rightarm and leftarm) and
the inertial sensor device.

The module instantiates network wrappers which create the YARP ports that allow
remote control of the robot and access to sensory reading. For historical reasons
(and preference to favor modularity) framegrabbers are not
instantiated here.

The names of the ports instantiated by iCubInterface follow the standard defined
for the robot (see the Manual), basically for each "part" there are three ports:
- /robotname/part/rpc:i
- /robotname/part/command:i
- /robotname/part/state:o

Where \e robotname is the name of the robot (icub), and \e part is the logical group 
of joints referred to by the port. Name of the robot and parts are specified in the
configuration file (see below).

In iCub these are defined as:

head, torso, right_leg, left_leg, right_arm, left_arm

- The "rpc:i" port provides access to all methods defined in the control board
interfaces (see YARP documentation).
- The "command:i" port, receives a stream of vectors which contains velocity 
commands. In general this is useful for closed loop, servoing control.
- The "state:o" port streams out a vector containing the motor encoder positions
for all joints.

\section lib_sec Libraries
YARP libraries.

\section parameters_sec Parameters
--file icub.ini (robot config file, search in $ICUB_ROOT/conf)
 
--config icub.ini (requires full path to the file)

\section portsa_sec Ports Accessed
None.

\section portsc_sec Ports Created

Ports follow the pattern:
- /icub/partname/rpc:i
- /icub/partname/command:i
- /icub/partname/state:o

where \e partname can be head, torso left_arm, right_arm, left_leg and right_leg.

/icub/partname/rpc:i
is an rpc port that gives access to the control board interfaces as defined in YARP.

/icub/partname/command:i
receive a yarp::sig::vector which contains the desired velocity of all joints. The size
of the vector must match the number of joints of the part.

/icub/partname/state:o
streams out a yarp::sig::vector which contains the current encoder feedback. The size 
of the vector matches the number of joints of the part. Values are updated with the 
period specified by the parameter \e threadrate see below.

\section in_files_sec Input Data Files
None

\section out_data_sec Output Data Files
None
 
\section conf_file_sec Configuration Files

 The module requires a description of the robot. This file descriptor is specified with the 
 parameter --config <CONFIG_FILE>.

 Since September 2009 iCubInterface uses the ResourceFinder to locate the file descriptor. 
 Please make sure you understand how this works. 

 In short you have the following options:

  - Run as iCubInterface --config <CONFIG_FILE>: loads parameters from CONFIG_FILE. CONFIG_FILE is 
  searched following the ResourceFinder policy, app/$ICUB_ROBOTNAME/conf (if the enviornment 
  variable $ICUB_ROBOTNAME exists)and app/default/conf
  
  - Run without parameters: search for a file iCubInterface.ini in app/$ICUB_ROBOTNAME/app 
  (if the enviornment variable $ICUB_ROBOTNAME exists) or app/default/conf. Where iCubInterface.ini
  is a file that contains the line "config CONFIG_FILE"

  You can prevent default behaviors in the following way:
  - Run as iCubInterface --config <CONFIG_FILE_WITHFULLPATH>: if CONFIG_FILE_WITHFULLPATH has
  full path to a valid file (as in ./icubSafe.ini), this is used in place of the defaults
  - Run as iCubInterface --from <OTHER_CONFIG>: prevents the RF from using iCubInterface.ini in 
  app/$ICUB_ROBOTNAME/conf or app/default/conf

The file consists in a few sections:
\code
[GENERAL]
name        icub
networks    (headtorso legs leftarm rightarm)
\endcode

The parameter \e name specifies the name of the robot. This will be used as a prefix
for all the port names (e.g. /\e icub /head... /\e icub /right_arm and so on...).

\e networks is a list of can networks of the robot. iCub has 4 can networks:
 
- headtorso: grouping dsp boards attached to head and torso
- legs: grouping dsp boards for the two legs
- leftarm and rightarm, grouping dsp boards controlling the left and right arm respectively

iCubInterface instantiates a can bus device to control each network separately. To do this
it needs to know the device driver to use and the initialization file. These are specified
in separate sections, that have the same name of the network. For example this is the section
for the \e headtorso network:
\code
[headtorso]
file            icub_head_waist.ini
device          canmotioncontrol
canbusdevice    pcan
calibrator      icubheadcalibrator
\endcode

This section specifies that we want to access the \e headtorso network through a 
\e canmotioncontrol and a \e pcan device. Here you need to specify a device 
implementing the motor control interfaces defined in YARP, for a can bus device.
In icub this is done by the \e canmotioncontrol  and the \e pcan  devices. The
former is a generic device converting messages to/from the control boards into can bus
messages. The latter is a lower level wrapper around the specific canbus device driver
on the robot. Another suitabile wrapper for this purpose is the \e ecan  device
(skipping a lot of non useful details).
The canbus devices will be initialized using information in \e icub_head_waist.ini . 
We also specify the calibrator object to use for calibration and parking of the robot. 
Parameters to the calibrator are in the same 
ini file. A detailed description of the ini file is probably not useful for most users
and is reported elsewhere. 
 
As you might have noticed for practical reasons some can networks group together dsp
boards that belong to different robot parts (e.g. \e headtorso  and 
\e legs ). iCubInterface allows splitting these networks in \e parts . Each one
of these \e parts will have a set of joints from 0 to \e naxis-1, where \e naxis  is
the total number of joints available in the network.
\verbatim
[headtorso]
file              icub_head_waist.ini
device            canmotioncontrol
canbusdevice      pcan
calibrator        icubheadcalibrator
parts             (head torso)
\endverbatim

The entry \e parts  specifies that we want to split the \e headtorso network
in two logical groups. Details on how to do this are specified in separate sections:
\verbatim
[part head]
threadrate 20
start 0
end 5
\endverbatim
 
\verbatim
[part torso]
threadrate 20
start 6
end 8
\endverbatim

Here we have specified that the part \e head maps joints 0-5 of \e headtorso whereas
the part torso maps joints 6-8 of the same network.

Three ports will be created for each parts, namely:

- /icub/head/rpc:i, /icub/head/command:i and /icub/head/state:o, 
 each controlling 6 joints numbered from 0 to 5
- /icub/torso/rpc:i, /icub/torso/command:i and /icub/torso/state:o,  
 each controlling 3 joints from 0 to 2.

The parameter \e threadrate specifies the periodiciy (ms) of the thread
serving the network ports.

By default if no \e start  or \e end  parameters are specified
the whole network is exported as a single part. For example the 
\e right_arm  and \e left_arm parts are just a one to one mapping
of their respective network devices.

\verbatim
[leftarm]
file            icub_left_arm_safe.ini
device          canmotioncontrol
canbusdevice    pcan
calibrator      icubarmcalibrator
parts           (left_arm)
\endverbatim

\verbatim
[part left_arm]
threadrate 20
\endverbatim
 
which creates the following ports:

/icub/left_arm/rpc:i, /icub/left_arm/command:i, /icub/left_arm/state:o
mapping 16 joints from 0 to 15.
Finally you can specify the inertial sensor to use:
\verbatim
[INERTIAL]
file            icub_inertial.ini
device          inertial
subdevice       xsensmtx
\endverbatim

and the name of the port to which to send the terminate command
\verbatim
[TERMINATEPORT]
Name /icub/quit
\endverbatim

\section tested_os_sec Tested OS
Linux and Windows.

\section example_sec Example Instantiation of the Module
iCubInterface --config $ICUB_ROOT/app/default/conf/icub.ini

\author Lorenzo Natale

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.


This file can be edited at src/iCubInterface2/main.cpp.
*/ 
    
#include <ace/OS.h>
#include <ace/Log_Msg.h>

#include <yarp/os/Network.h>
#include <yarp/os/Port.h> 
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h> 
#include <yarp/os/Terminator.h>
#include <yarp/os/ResourceFinder.h>

#ifdef USE_ICUB_MOD
#include "drivers.h"
#endif

#include "RobotInterface.h" 
#include "RobotInterfaceRemap.h"

using namespace yarp::os;
using namespace yarp::dev;  
using namespace yarp;

static bool terminated = false;
static bool askAbort=false;
IRobotInterface *ri=0;

static void handler (int) {
    static int ct = 0;
    ct++;    

    fprintf(stderr, "Asking to shut down \n");
    terminated = true;
    if (ct==3)
    {
        fprintf(stderr, "Aborting parking...\n");
        if(ri!=0)
        {
            ri->abort();
        }
    }

    if (ct>3)
        fprintf(stderr, "iCubInterface is already shutting down, this might take a while. If the application does not close you might have to kill it manually.\n");

}

int main(int argc, char *argv[]) 
{
    Network yarp; //initialize network, this goes before everything

    if (!yarp.checkNetwork())
    {
        fprintf(stderr, "Sorry YARP network does not seem to be available, is the yarp server available?\n");
        return -1;
    }

    #ifdef USE_ICUB_MOD
    yarp::dev::DriverCollection dev;
    #endif
    
 //   Property p;

 //   p.fromCommand(argc, argv);   

    ACE_OS::signal(SIGINT, (ACE_SignalHandler) handler);
    ACE_OS::signal(SIGTERM, (ACE_SignalHandler) handler);

 //   if (!p.check("robot") && !p.check("file") && !p.check("config"))
 //   {
  //      ACE_OS::printf("Mandatory parameters:\n");
  //      ACE_OS::printf("--file <CONFIG_FILE>  read robot config file CONFIG_FILE\n");
   //     ACE_OS::printf("Note: this files are searched in $ICUB_ROOT/conf (obsolete)\n");
   //     ACE_OS::printf("--config <CONFIG_FILE> full path to robot config file\n");
    //    ACE_OS::printf("-Examples:\n");
    //    ACE_OS::printf("%s --file icub.ini (obsolete, not standard)\n", argv[0]);
   //     ACE_OS::printf("%s --config $ICUB_ROOT/app/default/conf/icub.ini\n)", argv[0]);
    //    return -1;
   // }
    
  	Time::turboBoost();  

    //for compatibility with old usage of iCubInterface, the use of the ResourceFinder
    //here is merely functional and should NOT be taken as an example
    ResourceFinder rf;
    rf.setVerbose();
    rf.setDefaultConfigFile("iCubInterface.ini");
    rf.configure("ICUB_ROOT", argc, argv);
    ConstString configFile=rf.findFile("config");

    std::string filename;
    bool remap=false;
    if (configFile!="")
    {
        configFile=rf.findFile("config");
        filename=configFile.c_str();
        remap=true;
    }
    else
    {
        configFile=rf.find("file").asString();
        if (configFile!="")
        {
            const char *conf = ACE_OS::getenv("ICUB_ROOT");
            filename+=conf;
            filename+="/conf/";
            filename+=configFile.c_str();
            printf("Read robot description from %s\n", filename.c_str());
        }
        else
        {
            printf("\n");
            printf("Error: iCubInterface was not able to find a valid configuration file ");
            printf("(since September 2009 we changed a bit how iCubInterface locates configuratrion files).\n");
            printf("== Old possibilities:\n");
            printf("--config <CONFIG_FILE> read config file CONFIG_FILE.\n iCubInterface now ");
            printf("uses the ResourceFinder class to search for CONFIG_FILE. ");
            printf("Make sure you understand how the ResourceFinder works. In particular ");
            printf("you most likely need to set the ICUB_ROBOTNAME environment variable ");
            printf("to tell the RF to add app/$ICUB_ROBOTNAME/conf to the search path.\n");
            printf("--file <CONFIG_FILE> read config file from $ICUB_ROOT/conf: old style ");
            printf("initialization method, obsolete. Still here for compatibility reasons.\n");
            printf("== New possibilities:\n");
            printf("Place a file called iCubInterface.ini in app/$ICUB_ROBOTNAME/conf that contains ");
            printf("the line \"config icubSafe.ini\" (or anything of your choice), and run iCubInterface ");
            printf("without parameters\n.");
            printf("== Preventing default behaviors:\n");
            printf("Use full path in <CONFIG_FILE> (e.g. --config ./icubSafe.ini).\n");
            printf("Use --from: change config file (e.g. --from iCubInterfaceCustom.ini).\n");
            return -1;
        }
    }
  //      ACE_OS::printf("--file <CONFIG_FILE>  read robot config file CONFIG_FILE\n");
   //     ACE_OS::printf("Note: this files are searched in $ICUB_ROOT/conf (obsolete)\n");
   //     ACE_OS::printf("--config <CONFIG_FILE> full path to robot config file\n");
    //    ACE_OS::printf("-Examples:\n");
    //    ACE_OS::printf("%s --file icub.ini (obsolete, not standard)\n", argv[0]);
   //     ACE_OS::printf("%s --config $ICUB_ROOT/app/default/conf/icub.ini\n)", argv[0]);
    //    return -1;
        

    Property robotOptions;
    bool ok=robotOptions.fromConfigFile(filename.c_str());
    if (!ok) 
    {
        fprintf(stderr, "Sorry could not open %s\n", filename.c_str());
        return -1;
    }

    printf("Config file loaded successfully\n");

    String quitPortName;
    if (robotOptions.check("TERMINATEPORT"))
        {
            Value &v=robotOptions.findGroup("TERMINATEPORT").find("Name");
            quitPortName=String(v.toString().c_str());
        }
    else
        quitPortName=String("/iCubInterface/quit");

    fprintf(stderr, "Quit will listen on %s\n", quitPortName.c_str());

    // LATER: must use a specific robot name to make the port unique.
    Terminee terminee(quitPortName.c_str());
  
    //    Terminee terminee("/iCubInterface/quit");
    if (!terminee.isOk()) { 
        ACE_OS::printf("Failed to create proper quit socket\n"); 
       return 1;
    }   
  
    IRobotInterface *i;
    if (remap)
    {
        i=new RobotInterfaceRemap;
    }
    else
    {
        i=new RobotInterface;
    }
    ri=i; //set pointer to RobotInterface object (used in the handlers above)
     
    ok = i->initialize(filename); 
    if (!ok)
        return 0;

    char c = 0;    
    ACE_OS::printf("Driver instantiated and running (silently)\n");
    while (!terminee.mustQuit()&&!terminated) 
    {
        Time::delay(2); 
    }

	ACE_OS::printf("Received a quit message\n");
    
    i->park(); //default behavior is blocking (safer)
    i->finalize();  
    ri=0;  //tell signal handler interface is not longer valid (do this before you destroy i ;)
    delete i; 
    i=0;
    return 0;  
}


