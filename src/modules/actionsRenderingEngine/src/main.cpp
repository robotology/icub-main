/* 
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Carlo Ciliberto, Vadim Tikhanoff
* email:   carlo.ciliberto@iit.it vadim.tikhanoff@iit.it
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
@ingroup icub_module
\defgroup actionsRenderingEngine actionsRenderingEngine

A module combining multiple libraries and modules from the iCub repository
that allows to exectute some basic and complex actions on objects placed on
a table in front of the robot.

\section intro_sec Description 
This module makes use of the \ref ActionPrimitives library in order to interact
with the objects placed on a table. The module can independently receive
visual inputs from a stereo/mono tracker or from an object detector.
The module will ask the system to perform the translation from stereo to cartesian
information according to the current "stereo to cartesian" modality chosen:

-- homography: which assumes that the target point lies on a table and exploits
homography to obtain the relative 3D cartesian coordinates.

-- disparity: which uses the \ref StereoDisparity module to obtain the 3D cartesian
estimate of the 2D target.

-- network: which uses a previously trained neural network structure to predict the
3D cartesian coordinate from the stereo input.

The commands sent as bottles to the module port /<modName>/cmd:io
are described in the following. Note that all commands can be supplied 
with the optional parameter "left" or "right" to select the arm in use
for the action. 

Some commands require to specify a visual target for 
the action required. In these cases the parameter [target] can be expressed as follows:

(notation: [.] identifies a vocab, "." identifies a string)

--[motion] when using motion cues to detect the visual target.

--[track] when using the output of the visual tracker.

--[fixation] when the target is assumed to be in both cameras image centers.

--[raw] when the target is provided as a raw couple of camera plane coordinates to one of the two 'raw:i' ports.

--(<camera> u v) when the target is a 2D couple of camera plane coordinates. If not specified, the camera used is assumed to be the left one.

--("cartesian" x y z) or (x y z) when the target is a 3D cartesian position wrt the robot's reference frame.

--"object_name" when using a visual classifier to localize objects. This option represents a label to access online information stored in the
  databased provided by \ref objectsPropertiesCollector module. The label itself is used to access the corresponding object through the "name"
  property, whereas the relevant 3D information are available through the "position_3d" property.

The commands:

<b>GET</b> 
format: [get] <request>
action: the system returns the requested element. <request> can be of the form:

--[s2c] [target]: returns the current cartesian coordinates of a visual 
target wrt to the robot root reference frame.

<b>IDLE</b> 
format: [idle]
action: the gaze controller is reset to the original context stored at
start and head/eye control is interrupted.
 
<b>HOME</b> 
format: [home] "param1" "param2"
action: the arms are sent to home position. the optional parameters
"param1" and "param2" can be independently set to "gaze" or
"head" in order to bring the gaze controller in home position
and "fingers" or "hands" to open the robot hands. Alternatively
the parameter "all" can be supplied to perform both optional actions.
 
<b>OBSERVE</b> 
format: [observe]
action: if the robot is holding an object it brings it in the FoV of
its cameras with the final purpose of visually explore it.

<b>DROP</b> 
format: [drop] "param1" "param2" or [drop] "over" [target] "param1" "param2"
action: if the robot is holding an object it brings it over the table and drops it
on a random position approximatively in front of it.
Optional parameter "side" or "above" can be provided in order to select the 
hand orientation that the robot should try to mantain during the drop action.
Optional parameter "away" can be provided in orded to have the robot leave the object
outside the cameras FoV (on the right or left side of the robot depending on the arm in use).
Optional parameter "over" can be supplied together with the action [target] in order
to have the robot drop the object on top of the visual target. Also in this case
the optional parameter "side" or "above" can be specified. Furthermore in this case an optional
parameters "gently" can be specified in order for the robot to gently deploy the hand-held
object over the target.

<b>TAKE</b> 
format: [take] [target] "param1"
action: the robot tries to reach the specified [target] and grasp it.
Optional parameter "side" or "above" can be supplied to choose the orientation the robot
should try to mantain while performing the action (default: "above").

<b>TOUCH</b> 
format: [touch] [target] "param1" "param2"
action: the robot tries to reach the specified [target] and then brings the arm back to home position.
Optional parameter "side" or "above" can be supplied to choose the orientation the robot
should try to mantain while performing the action (default: "above").
Optional parameter "still" can be provided to avoid the robot bring its arm back to home position after
reaching has been achieved.

<b>PUSH</b> 
format: [push] [target] "param1"
action: the robot tries to reach the specified [target] from one side and then push it laterally.
Optional parameter "away" can be supplied in order to have the robot push the object away from its
root reference frame.

<b>POINT</b> 
format: [point] [target]
action: the robot tries to point the specified [target] with its index finger.

<b>LOOK</b> 
format: [look] [target]
action: the robot looks at the specified [target].
 
<b>TRACK</b> 
format: [track] [target]
action: the specified [target] visual position is supplied to the tracker module and the gaze controller is 
updated in order to keep the object constantly inside the robot fovea.

<b>TEACH ACTION</b> 
format: [teach] "action_name" [start/stop] "param1"
action: If parameter [start] is specified and the action "action_name" has not been registered yet, the
robot arm is set to torque  control mode. The cartesian position (wrt the starting point) and orientation
of the robot arm is stored until the command [teach] "action_name" [stop] is received. The learned action
is recorded to the file "actions/<arm>/<action_name>.txt" and can be repeated autonomously 
using the command <b>IMITATE ACTION</b> ([imitate] "action_name").

<b>IMITATE ACTION</b> 
format: [imitate] "action_name"
action:  the system loads the file "actions/<arm>/<action_name>.txt" and controls the arm in order to 
repeat the trajectory previously registered.


<b>CALIBRATION</b>
different types of calibrations can be requested by issuing commands of the form: [calib] [calibration_type] "param1"
In the following a short description of the possible values of [calibration_type]:

--[table] the robot will try to find out the table height 
exploiting the contact detection based on force control. If the 
contact is detected then the new table height is sent to the 
homography module that is in charge of extracting the 3d 
coordinates of the object from its 2d projection on the image 
plane. The file containing the table information is also updated 
accordingly to avoid calibrating always at start-up.

--[fingers] the robot will calibrate the fingers in order to detect
whether correct grasp has been achieved.

--[kinematics] these option requires a parameter [start/stop]. when started, the system is set to cartesian
admittance control and the robot arm can be moved around by the human user. When the [calib] [kinematics] [stop]
command is received, the system is set to velocity mode and the offset between the initial and the current
position is stored.


\section lib_sec Libraries 
- YARP libraries. 
- \ref ActionPrimitives library. 

\section portsa_sec Ports Accessed
Assumes that \ref icub_iCubInterface (with ICartesianControl interface implemented) is running together with the
cartesian controllers \ref icub_cartesianController.
 
\section portsc_sec Ports Created 
Aside from the internal ports created by \ref ActionPrimitives 
library, we also have: 
 
- \e /<modName>/cmd:io receives a bottle containing commands 
  whose formats are specified in the previous section. The port 
  replies as soon as the current action has been completed.
 
- \e /<modName>/rpc remote procedure call. 
    Recognized remote commands:
    -[help]: returns the list of available rpc commands.
    -[get]: get requests
        * [status]:returns the bottle (gaze <status>) (left_arm <status>) (right_arm <status>) where
                   <status> can be equal to "idle", "busy" or "unavailable".
    -[impedance] [on]/[off]: enable/disable (if available) impedance velocity control.
    -[mode] [homography]/[disparity]/[network]: sets the desired stereo to cartesian mode.
    -[interrupt] : interrupts any action deleting also the action queue for both arms.
    -[reinstate] : if the module was interrupted reinstate it.
 
\section parameters_sec Parameters 
The following are the options that are not usually contained 
within the configuration file. 
 
--name \e name
- specify the module name, which is \e actionsRenderingEngine by 
  default.

--from \e file 
- specify the configuration file (use \e --context option to 
  select the current context)

\section in_files_sec Input/Output Data Files
-- table.ini contains the table height and is updated on-line as 
   result of an exploration phase.
-- network.ini contains the information needed by the system to 
   load the neural network that computes the stereo2cartesian mapping.
-- hands_sequences.ini contains the list of hand sequences for the 
   action primitives.
-- exploration_poses.ini contains a list of position used to explore
   multiple points of view with the robot torso.
-- grasp_model_<hand>.ini contains the information needed by the
   PerceptiveModel layer. 
-- actions/<arm>/<action_name>.actions each action "learned by imitation"
   is stored in a .txt file with the same name of the action itself.


\section tested_os_sec Tested OS
Windows, Linux

\author Carlo Ciliberto, Vadim Tikhanoff
*/ 

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <string>

#include <iCub/utils.h>
#include <iCub/MotorThread.h>
#include <iCub/VisuoThread.h>

#define RPC_HELP                    VOCAB4('h','e','l','p')
#define RPC_GET                     VOCAB3('g','e','t')
#define RPC_GET_STATUS              VOCAB4('s','t','a','t')
#define RPC_IMPEDANCE               VOCAB4('i','m','p','e')
#define RPC_S2C_MODE                VOCAB4('m','o','d','e')
#define RPC_INTERRUPT               VOCAB4('i','n','t','e')
#define RPC_REINSTATE               VOCAB4('r','e','i','n')

#define CMD_IDLE                    VOCAB4('i','d','l','e')
#define CMD_HOME                    VOCAB4('h','o','m','e')
#define CMD_CALIBRATE               VOCAB4('c','a','l','i')
#define CMD_OBSERVE                 VOCAB4('o','b','s','e')
#define CMD_DROP                    VOCAB4('d','r','o','p')

#define CMD_HOLD                    VOCAB4('h','o','l','d')

#define CMD_LEARN_MIL               VOCAB4('l','e','a','r')

#define CMD_GET                     VOCAB3('g','e','t')
#define CMD_GET_S2C                 VOCAB3('s','2','c')
#define CMD_TAKE                    VOCAB4('t','a','k','e')
#define CMD_TOUCH                   VOCAB4('t','o','u','c')
#define CMD_PICK                    VOCAB4('p','i','c','k')
#define CMD_PUSH                    VOCAB4('p','u','s','h')
#define CMD_POINT                   VOCAB4('p','o','i','n')
#define CMD_LOOK                    VOCAB4('l','o','o','k')
#define CMD_TRACK                   VOCAB4('t','r','a','c')


#define CMD_ACTION_TEACH            VOCAB4('t','e','a','c')
#define CMD_ACTION_IMITATE          VOCAB4('i','m','i','t')


#define CALIB_TABLE                 VOCAB4('t','a','b','l')
#define CALIB_FINGERS               VOCAB4('f','i','n','g')
#define CALIB_KIN_OFFSET            VOCAB4('k','i','n','e')




#ifdef WIN32
    #pragma warning(disable:4996)
#endif

YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::action;

class ActionsRenderingEngineModule: public RFModule
{
protected:
    MotorThread                 *motorThr;
    VisuoThread                 *visuoThr;

    Initializer                 *initializer;

    Port                        cmdPort;
    Port                        rpcPort;

    bool                        interrupted;
    volatile bool               closing;


    bool check(Bottle &bot, const string &name)
    {
        bool found=false;
        for(int i=0; i<bot.size(); i++)
        {
            if(bot.get(i).asString()==name.c_str())
            {
                found=true;
                break;
            }
        }

        return found;
    }


public:
    ActionsRenderingEngineModule()
    {}

    virtual bool configure(ResourceFinder &rf)
    {
        Bottle bManager=rf.findGroup("manager");

        string name=rf.check("name",Value("actionsRenderingEngine")).asString().c_str();
        setName(name.c_str());

        cmdPort.open(("/"+name+"/cmd:io").c_str());

        rpcPort.open(("/"+name+"/rpc").c_str());
        attach(rpcPort);

        initializer=new Initializer(rf);

        motorThr=new MotorThread(rf,initializer);
        visuoThr=new VisuoThread(rf,initializer);

        if(!motorThr->start() || !visuoThr->start())
        {
            close();
            return false;
        }

        interrupted=false;
        closing=false;

        return true;
    }

    virtual bool interruptModule()
    {
    
        closing=true;

        cmdPort.close();
        rpcPort.close();
        //cmdPort.interrupt();
        //rpcPort.interrupt();

        initializer->interrupt();

        if(visuoThr!=NULL)
            visuoThr->interrupt();

         if(motorThr!=NULL)
            motorThr->interrupt();
            
        return true;
    }

    virtual bool close()
    {

        cmdPort.close();
        rpcPort.close();
        
        if(motorThr!=NULL)
        {
            motorThr->reinstate();
            motorThr->stop();
            delete motorThr;
        }


        if(visuoThr!=NULL)
        {
            visuoThr->reinstate();
            visuoThr->stop();
            delete visuoThr;
        }


        initializer->close();
        
        delete initializer;

        return true;
    }

    virtual double getPeriod()
    {
        return 0.1;
    }

    virtual bool updateModule()
    {
        if(closing)
            return true;

        motorThr->update();


        Bottle command,reply;
        command.clear();
        reply.clear();

        //wait for a command. will provide reply
        if(!isStopping())
            cmdPort.read(command,true);

        if(command.size()==0)
            return true;

        if(interrupted)
        {
            reply.addString("Module currently interrupted. Reinstate for action.");
            cmdPort.reply(reply);
            return true;
        }

        switch(command.get(0).asVocab())
        {
            case CMD_IDLE:
            {
                motorThr->setGazeIdle();
                reply.addString("idle");
                break;
            }

            case CMD_HOME:
            {
                motorThr->goHome(command);
                reply.addString("home");

                break;
            }

            case CMD_GET:
            {
                if(command.size()>1)
                {
                    switch(command.get(1).asVocab())
                    {
                        case CMD_GET_S2C:
                        {
                            if(command.size()>2)
                            {
                                Vector xd;
                                visuoThr->getTarget(command.get(2),command);
                                if(motorThr->targetToCartesian(command.find("target").asList(),xd))
                                {
                                    reply.clear();
                                    for(size_t i=0; i<xd.size(); i++)
                                        reply.addDouble(xd[i]);
                                }
                                else
                                    reply.addString("Error");
                            }

                            break;
                        }
                    }

                    break;
                }

                break;
            }

            case CMD_CALIBRATE:
            {
                switch(command.get(1).asVocab())
                {
                    case CALIB_TABLE:
                    {
                        if(motorThr->calibTable(command))
                            reply.addString("table height found");
                        else
                            reply.addString("table height not found");

                        break;
                    }

                     case CALIB_FINGERS:
                    {
                        if(motorThr->calibFingers(command))
                            reply.addString("fingers calibrated");
                        else
                            reply.addString("could not calibrate fingers");

                        break;
                    }

                     case CALIB_KIN_OFFSET:
                    {
                        if(command.get(2).asString()=="start")
                        {
                            motorThr->startLearningModeKinOffset(command);
                            reply.addString("learn kinematic offset mode: on");
                        }
                        else if(command.get(2).asString()=="stop")
                        {
                            motorThr->suspendLearningModeKinOffset(command);
                            reply.addString("learn kinematic offset mode: off");
                        }

                        break;
                    }
                }

                break;
            }

            //------ action "learning" ---------//
            case CMD_ACTION_TEACH:
            {
                if(check(command,"start"))
                {
                    string action_name=command.get(1).asString().c_str();

                    Bottle &action=command.addList();
                    action.addString("action_name");
                    action.addString(action_name.c_str());

                    if(!motorThr->startLearningModeAction(command))
                        reply.addString(("action "+action_name+" already known").c_str());
                    else
                    {
                        motorThr->setGazeIdle();
                        motorThr->lookAtHand();
                        reply.addString("start teaching");
                    }
                }

                if(check(command,"stop"))
                {
                    bool ok=motorThr->suspendLearningModeAction(command);
                    
                    fprintf(stdout,"stopped %s\n",ok?"ok":"bad");
                    motorThr->setGazeIdle();
                    reply.addString("stop teaching");
                }

                break;
            }

            case CMD_ACTION_IMITATE:
            {
                string action_name=command.get(1).asString().c_str();

                Bottle &action=command.addList();
                action.addString("action_name");
                action.addString(action_name.c_str());

                motorThr->lookAtHand();

                if(!motorThr->imitateAction(command))
                    reply.addString(("action "+action_name+" unkown").c_str());
                else
                    reply.addString(("action "+action_name+" done").c_str());

                motorThr->setGazeIdle();

                break;
            }
            //-----------------------------------//


            case CMD_LEARN_MIL:
            {
                string obj_name=command.get(1).asString().c_str();
                if(motorThr->isHolding(command)) // 
                {
                    fprintf(stdout,"Deploying %s.\n",obj_name.c_str());
                    motorThr->deploy(command);
                }

                //if it is not currently tracking anything, start learning what it has in fixation
                Vector stereo;
                if(!visuoThr->isTracking())
                {
                    Value v("fix");
                            visuoThr->getTarget(v,command);
                }
                visuoThr->startLearningMIL(obj_name.c_str());

                fprintf(stdout,"Looking at %s.\n",obj_name.c_str());
                motorThr->exploreTorso(10.0);

                visuoThr->trainMIL();

                reply.addString((obj_name + " learned").c_str());
                fprintf(stdout,"'%s' learned.\n",obj_name.c_str());

                return true;
            }

            case CMD_OBSERVE:
            {
                if(!motorThr->isHolding(command))
                {
                    reply.addString("Nothing to drop. Not holding anything");
                    motorThr->release(command);
                    motorThr->goHome(command);
                    break;
                }

                motorThr->lookAtHand();
                motorThr->drawNear(command);
                motorThr->setGazeIdle();

                reply.addString("observing");

                break;
            }

            case CMD_DROP:
            {
                if(!motorThr->isHolding(command))
                {
                    reply.addString("Nothing to drop. Not holding anything");
                    motorThr->release(command);
                    motorThr->goHome(command);
                    break;
                }

                if(check(command,"over") && command.size()>2)
                    visuoThr->getTarget(command.get(2),command);

                motorThr->setGazeIdle();

                motorThr->deploy(command);

                motorThr->keepFixation();

                motorThr->goHome(command);
                motorThr->setGazeIdle();

                reply.addString("dropped");
                break;
            }

            case CMD_TAKE:
            {
                if(command.size()<2)
                {
                    reply.addString("Error");
                    break;
                }

                visuoThr->getTarget(command.get(1),command);

                motorThr->lookAtObject();

                if(!motorThr->reach(command))
                {
                    reply.addString("failed. Please specify the target");
                    break;
                }

                motorThr->lookAtHand();
                motorThr->grasp(command);

                if(motorThr->isHolding(command))
                {
                    if(check(command,"near"))
                    {
                        motorThr->drawNear(command);
                        motorThr->setGazeIdle();
                    }
                    else
                    {
                        motorThr->setGazeIdle();
                        Bottle b;
                        b.addString("head");
                        b.addString("arms");
                        motorThr->goHome(b);
                    }

                    reply.addString("holding");
                }
                else
                {
                   motorThr->setGazeIdle();
                   motorThr->release(command);
                   motorThr->goHome(command);
                   reply.addString("failed");
                }

                break;
            }

            case CMD_TOUCH:
            {
                 if(command.size()<2)
                {
                    reply.addString("Error");
                    break;
                }

                visuoThr->getTarget(command.get(1),command);

                motorThr->lookAtObject();

                if(!motorThr->reach(command))
                {
                    reply.addString("failed. Please specify the target");
                    break;
                }

                if(!check(command,"still"))
                {
                    Time::delay(2.0);
                    motorThr->goHome(command);
                }

                reply.addString("touched");
                break;
            } 

            case CMD_PUSH:
            {
                if(command.size()<2)
                {
                    reply.addString("Error");
                    break;
                }

                visuoThr->getTarget(command.get(1),command);

                motorThr->lookAtObject();

                if(!motorThr->push(command))
                {
                    reply.addString("failed. Please specify the target");
                    break;
                }

                motorThr->goHome(command);
                motorThr->setGazeIdle();

                reply.addString("pushed");
                break;
            }

            case CMD_POINT:
            {
                if(command.size()<2)
                {
                    reply.addString("Error");
                    break;
                }

                visuoThr->getTarget(command.get(1),command);

                motorThr->lookAtObject();

                if(!motorThr->point(command))
                {
                    reply.addString("failed. Please specify the target");
                    break;
                }

                if(!check(command,"still"))
                {
                    Time::delay(2.0);
                    motorThr->goHome(command);
                }

                reply.addString("pointed");
                break;
            }

            case CMD_LOOK:
            {
                if(command.size()<2)
                {
                    reply.addString("Error");
                    break;
                }

                visuoThr->getTarget(command.get(1),command);

                motorThr->lookAtObject();

                if(!motorThr->look(command))
                {
                    reply.addString("failed. Please specify the target");
                    break;
                }

                reply.addString("looked");
                break;
            }

            case CMD_TRACK:
            {
                //motion tracking is handled differently (temporary cheat)
                if(command.get(1).asVocab()!=Vocab::encode("motion"))
                {
                    if(command.size()<2)
                    {
                        reply.addString("Error");
                        break;
                    }

                    visuoThr->getTarget(command.get(1),command);
                }
                else
                    visuoThr->trackMotion();

                motorThr->lookAtObject();

                reply.addString("tracking");

                break;
            }

            default:
            {
                reply.addString("failed");
                break;
            }
        }

        cmdPort.reply(reply);
        return true;
    }

    virtual bool respond(const Bottle &command, Bottle &reply)
    {
        if(command.size()==0)
        {
            reply.addString("No command received.");
            return true;
        }

        switch(command.get(0).asVocab())
        {
            case RPC_HELP:
            {
                string rep="Commands:\n";
                rep+="\"interrupt\"\t\t\tinterrupts all the actions untill a 'reinstate' command is issued\n";
                rep+="\"reinstate\"\t\t\treinstates the system (usually called after an 'interrupt' command\n";
                rep+="\"impedance on/off\"\\off\t\t\tset the impedance control on or off\n";
                rep+="\"mode homography\"\\disparity\\network\t\t\tset the 'stereo to cartesian' mode";
                rep+="\"status\"\t\t\tget the module current status\n";
                reply.addString(rep.c_str());
                break;
            }

            case RPC_GET:
            {
                if(command.size()>1)
                {
                    switch(command.get(1).asVocab())
                    {
                        case RPC_GET_STATUS:
                        {
                            motorThr->getStatus(reply);
                            break;
                        }
                    }
                }
                break;
            }

            //set impedance on or off
            case RPC_IMPEDANCE:
            {
                if(command.get(1).asString()=="on")
                {
                    bool done=motorThr->setImpedance(true);
                    done?reply.addString("Impedance turned on"):reply.addString("Unable to set impedance on");
                }
                else if(command.get(1).asString()=="off")
                {
                    motorThr->setImpedance(false);
                    reply.addString("impedance turned off");
                }
                else
                {
                    reply.addString("What do you want? Impedance on or off?");
                }

                break;
            }

            //set the stereo2cartesian mode
            case RPC_S2C_MODE:
            {
                motorThr->setStereoToCartesianMode(command.get(1).asVocab(),reply);

                break;
            }

            case RPC_INTERRUPT:
            {
                interrupted=true;
                motorThr->interrupt();
                visuoThr->interrupt();
                reply.addString("interrupted");
                break;
            }

            case RPC_REINSTATE:
            {
                interrupted=false;
                motorThr->reinstate();
                visuoThr->reinstate();
                reply.addString("reinstated");
                break;
            }

            default:
            {
                reply.addString("command not recognized");
                break;
            }
        }
        return true;
    }
};


int main(int argc, char *argv[])
{
    Network yarp;   

    if (!yarp.checkNetwork())
        return -1;

    YARP_REGISTER_DEVICES(icubmod)

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("actionsRenderingEngine/conf");
    rf.setDefaultConfigFile("config.ini");
    rf.setDefault("name","actionsRenderingEngine");
    rf.configure("ICUB_ROOT",argc,argv);

    ActionsRenderingEngineModule mod;

    return mod.runModule(rf);
}


