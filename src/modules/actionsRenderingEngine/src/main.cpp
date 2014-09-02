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
that allows to execute some basic and complex actions on objects placed on
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

\section cmd_port Issuing commands

The commands sent as bottles to the module port /<modName>/cmd:io
are described in the following. The response to any command    
consists in the vocab [ack]/[nack] in case of success/failure. 

Some commands require to specify a visual target for 
the action required. In these cases the parameter [target] can be expressed as follows:

(notation: [.] identifies a vocab, "." identifies a string)

--[motion] when using motion cues to detect the visual target.

--[track] when using the output of the visual tracker.

--[fixation] when the target is assumed to be in both cameras image centers.

--[raw] when the target is provided as a raw couple of camera plane coordinates to one of the two 'raw:i' ports.

--("left"|"right" u v) when the target is a 2D couple of camera plane coordinates. If not specified, camera is assumed to be "left".

--("cartesian" x y z) or (x y z) when the target is a 3D cartesian position wrt the robot's reference frame.    
    
--("cartesian" x y z ax ay az theta) or (x y z ax ay az theta): occasionally also the orientation can be provided.

--"object_name" when using a visual classifier to localize objects. This option represents a label to access online information stored in the
  databased provided by \ref objectsPropertiesCollector module. The label itself is used to access the corresponding object through the "name"
  property, whereas the relevant 3D information are available through the "position_3d" property.

General Parameters:

The majority of commands can usually be provided with general optional parameters:

--<arm>: the user can specify the arm required for the action ("left"/"right"). Otherwise the default arm will be used.

--"no_head"/"no_gaze": the action does not affect the gaze of the robot (this parameter does not influece commands that explicitly require
  to move the head like LOOK).

--"no_sacc": disables the saccadic movements of the gaze controller during the action.    
    
--"still": prevents the robot from bringing back home the arm after having accomplished the action.
    
The commands:    

<b>IDLE</b> \n
format: [idle] \n
action: the gaze controller is reset to the original context stored at
start and head/eye control is interrupted.
 
<b>HOME</b> \n
format: [home] "param1" "param2" \n
action: the arms are sent to home position. the optional parameters
"param1" and "param2" can be independently set to "gaze" or
"head" in order to bring the gaze controller in home position
and "fingers" or "hands" to open the robot hands. Alternatively
the parameter "all" can be supplied to perform both optional actions.
 
<b>OBSERVE</b> \n
format: [observe] \n
action: if the robot is holding an object it brings it in the FoV of
its cameras with the final purpose of visually explore it.

<b>DROP</b> \n
format: [drop] "param1" "param2" or [drop] "over" [target] "param1" "param2" \n
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

<b>TAKE</b> \n
format: [take] [target] "param1" \n
action: the robot tries to reach the specified [target] and grasp it.
Optional parameter "side" or "above" can be supplied to choose the orientation the robot
should try to mantain while performing the action (default: "above").
    
<b>CLOSE</b> \n
format: [close] "param1" \n
action: close the hand ("left"/"right" option can be given).
    
<b>TAKE_TOOL</b> \n
format: [tato] "param1" \n    
action: the robot will reach a specified position to take the 
tool from a user. Optional parameter "left" or "right" can be    
supplied to choose the orientation the robot    

<b>CLOSE_TOOL</b> \n
format: [clto] "param1" \n
action: close the hand (predefined or specified) for grabbing    
the tool.    
    
<b>GRASP</b> \n    
format: [grasp] [target] \n    
action: the robot tries to reach the specified [target] and 
performs a power grasp. The target must be specified both in    
cartesian position and orientation. \n    
As further parameter user may specify the way the robot will 
approach the target by providing the options ("approach" (dx dy    
dz wrist_pitch)), where dx/dy/dz account for offset displacement    
in meters wrt to the grasping reference frame and wrist_pitch is    
the apporaching pitch of the wrist given in degrees.    

<b>TOUCH</b> \n
format: [touch] [target] "param1" "param2" \n
action: the robot tries to reach the specified [target] and then brings the arm back to home position.
Optional parameter "side" or "above" can be supplied to choose the orientation the robot
should try to mantain while performing the action (default: "above").

<b>PUSH</b> \n
format: [push] [target] "param1" \n
action: the robot tries to reach the specified [target] from one side and then push it laterally.
Optional parameter "away" can be supplied in order to have the robot push the object away from its
root reference frame.

<b>POINT</b> \n
format: [point] [target] \n
action: the robot tries to point the specified [target] with its index finger.

<b>LOOK</b> \n
format: [look] [target] "param1" (block_eyes ver) \n
action: the robot looks at the specified [target]. "param1" can be set equal to "fixate" in order to
keep the gaze fixating the requested target also when other    
commands are issued to the torso. \n    
If provided, the option (block_eyes ver) serves to block the    
eyes at the specified vergence while gazing. \n    
Note: the special target [hand] (with optional parameter 
"left"/"right") can be provided to have the robot look at its   
own hand. The robot will keep looking at its own hand until an    
idle command.    

<b>EXPECT</b> \n
format: [expect] \n
action: the robot puts one arm forward with the palm of the hand facing up and waiting for an object
to be put on it.

<b>GIVE</b> \n
format: [give] \n
action: the robot puts one arm forward with the palm of the hand facing up and opens the fingers so that
the object held in the hand is free to be taken.

<b>TRACK</b> \n
format: [track] [target] "param1" \n
action: the specified [target] visual position is supplied to the tracker module and the gaze controller is 
updated in order to keep the object constantly inside the robot fovea. The optional "param1" can be put equal to
"no_sacc" in order to disable the saccadic movements of the gaze controller during the tracking process.

<b>TEACH ACTION</b> \n
format: [teach] "action_name" [start/stop] "param1" \n
action: If parameter [start] is specified and the action "action_name" has not been registered yet, the
robot arm is set to torque  control mode. The cartesian position (wrt the starting point) and orientation
of the robot arm is stored until the command [teach] "action_name" [stop] is received. The learned action
is recorded to the file "actions/<arm>/<action_name>.txt" and can be repeated autonomously 
using the command <b>IMITATE ACTION</b> ([imitate] "action_name").

<b>IMITATE ACTION</b> \n
format: [imitate] "action_name" \n
action:  the system loads the file "actions/<arm>/<action_name>.txt" and controls the arm in order to 
repeat the trajectory previously registered.

<b>CALIBRATION</b>\n
different types of calibrations can be requested by issuing commands of the form: [calib] [calibration_type] "param1" "param2"
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
torque control and the robot arm can be moved around by the human user. When the [calib] [kinematics] [stop]
command is received, the system is set to velocity mode and the offset between the initial and the current
position is stored. It is also possible to associate a specific kinematic offset to a single object. To do 
so, it is required to issue the command [calib] [kinematics] [stop] <object_name>. Then, whenever the system
will be asked to preform an action over such object, the system will use the learnt offset.
 
<b>EXPLORATION</b> \n
different types of explorations can be requested by issuing commands of the form: [explore] [exploration_type]
In the following a short description of the possible values of [exploration_type]:
 
--[torso] the robot will start moving its torso exploring different positions (specified in the file exploration_poses.ini).
then it will go back to the initial position.
 
--[hand] the robot will start moving its hand (typically while holding an object) and at the same time look at it. For safety
issues, by default the robot moves the other hand back to home position in order to avoid collision. This behaviour can be modified
by providing the optional parameter "keep_other_hand_still".

 
\section get_port The "Get" Port
 
The actionsRenderingEngine can be queried via the port /<modName>/get:io in order to obtain 
the following data:
 
 
<b>GET</b> 
format: [get] <request>
action: the system returns the requested element.    
    
<request> can be of the form:
 
--[s2c] [target_1] ... [target_n]: returns the current cartesian    
coordinates of the n visual targets wrt to the robot root    
reference frame. The number of targets is not predefined.    
 
--[table]: returns the current height of the table in front of    
  the robot  
 
--[holding]: returns true if the robot is holding an object in  
its active hand.    
    
--[hand] [image]: returns a vector of the form (u_left v_left    
u_right v_right hand-head-distance) that reports the projection    
of the end-effector of the active arm in both the robot cameras    
together with the distance (in meters) between the robot    
forehead and its hand.    
 
\section lib_sec Libraries 
- YARP libraries. 
- \ref ActionPrimitives library. 
 
\section portsa_sec Ports Accessed
Assume that the robot interface is operative and the
\ref wholeBodyDynamics is running.
 
\section portsc_sec Ports Created 
Aside from the internal ports created by \ref ActionPrimitives 
library, we also have: 
 
- \e /<modName>/cmd:io receives a bottle containing commands 
  whose formats are specified in the previous section. The port 
  replies as soon as the current action has been completed.
 
- \e /<modName>/rpc remote procedure call. 
    Recognized remote commands:\n
    -[help]: returns the list of available rpc commands.\n
    -[get]: get requests\n
    -[status]: returns the bottle (gaze <status>)
     (left_arm <status>) (right_arm <status>) where 
     <status> can be equal to "idle", "busy" or "unavailable".\n
    -[impedance] [on]/[off]: enable/disable (if available) impedance velocity control.\n
    -[waveing] [on]/[off]: enable/disable the iCub arm(s) waving.\n
    -[mode] [homography]/[disparity]/[network]: sets the desired stereo to cartesian mode.\n
    -[interrupt]: interrupts any action deleting also the action queue for both arms.\n
    -[reinstate]: if the module was interrupted reinstate it.\n
    -[elbow] "left"|"right"|"both" <height> <weight>: to change
     elbow parameters.
 
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
-- kinematic_offsets.ini contains the table height as well as
   the arms kinematic offsets and is updated on-line as result 
   of an exploration phase. 
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
#include <yarp/os/RpcServer.h>
#include <yarp/os/PortReader.h>
#include <yarp/os/Time.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/Vector.h>

#include <string>

#include <iCub/utils.h>
#include <iCub/MotorThread.h>
#include <iCub/VisuoThread.h>


#define ACK                         VOCAB3('a','c','k')
#define NACK                        VOCAB4('n','a','c','k')

#define RPC_HELP                    VOCAB4('h','e','l','p')
#define RPC_GET                     VOCAB3('g','e','t')
#define RPC_GET_STATUS              VOCAB4('s','t','a','t')
#define RPC_IMPEDANCE               VOCAB4('i','m','p','e')
#define RPC_S2C_MODE                VOCAB4('m','o','d','e')
#define RPC_INTERRUPT               VOCAB4('i','n','t','e')
#define RPC_REINSTATE               VOCAB4('r','e','i','n')
#define RPC_WAVEING                 VOCAB4('w','a','v','e')
#define RPC_ELBOW                   VOCAB4('e','l','b','o')

#define CMD_IDLE                    VOCAB4('i','d','l','e')
#define CMD_HOME                    VOCAB4('h','o','m','e')
#define CMD_CALIBRATE               VOCAB4('c','a','l','i')
#define CMD_EXPLORE                 VOCAB4('e','x','p','l')

#define CMD_OBSERVE                 VOCAB4('o','b','s','e')
#define CMD_DROP                    VOCAB4('d','r','o','p')

#define CMD_HOLD                    VOCAB4('h','o','l','d')

#define CMD_LEARN_MIL               VOCAB4('l','e','a','r')

#define CMD_GET                     VOCAB3('g','e','t')
#define CMD_TAKE                    VOCAB4('t','a','k','e')
#define CMD_GRASP                   VOCAB4('g','r','a','s')
#define CMD_TOUCH                   VOCAB4('t','o','u','c')
#define CMD_PICK                    VOCAB4('p','i','c','k')
#define CMD_PUSH                    VOCAB4('p','u','s','h')
#define CMD_POINT                   VOCAB4('p','o','i','n')
#define CMD_LOOK                    VOCAB4('l','o','o','k')
#define CMD_TRACK                   VOCAB4('t','r','a','c')
#define CMD_EXPECT                  VOCAB4('e','x','p','e')
#define CMD_GIVE                    VOCAB4('g','i','v','e')
#define CMD_CLOSE                   VOCAB4('c','l','o','s')
#define CMD_GAZE                    VOCAB4('r','e','l','e')

//commands for tool
#define CMD_TAKE_TOOL               VOCAB4('t','a','t','o')
#define CMD_CLOSE_TOOL              VOCAB4('c','l','t','o')

#define CMD_ACTION_TEACH            VOCAB4('t','e','a','c')
#define CMD_ACTION_IMITATE          VOCAB4('i','m','i','t')


//sub commands: get
#define GET_S2C                     VOCAB3('s','2','c')
#define GET_TABLE                   VOCAB4('t','a','b','l')
#define GET_HOLDING                 VOCAB4('h','o','l','d')
#define GET_HAND                    VOCAB4('h','a','n','d')
#define GET_IMAGE                   VOCAB4('i','m','a','g')
#define GET_IDLE                    VOCAB4('i','d','l','e')

//sub commands: calib
#define CALIB_TABLE                 VOCAB4('t','a','b','l')
#define CALIB_FINGERS               VOCAB4('f','i','n','g')
#define CALIB_KIN_OFFSET            VOCAB4('k','i','n','e')

//sub commands: explore
#define EXPLORE_TORSO               VOCAB4('t','o','r','s')
#define EXPLORE_HAND                VOCAB4('h','a','n','d')




#define PORT_TAG_CMD                0
#define PORT_TAG_GET                1




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

class ActionsRenderingEngine
{
protected:
    MotorThread                 *motorThr;
    VisuoThread                 *visuoThr;

    Initializer                 *initializer;

    bool                        interrupted;
    volatile bool               closing;
    bool                        idle;


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
    ActionsRenderingEngine()
    {}

    bool initialize(ResourceFinder &rf)
    {
        string name=rf.check("name",Value("actionsRenderingEngine")).asString().c_str();

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
        idle=true;

        return true;
    }


    bool close()
    {
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

    bool interrupt()
    {
        closing=true;

        initializer->interrupt();

        if(visuoThr!=NULL)
            visuoThr->interrupt();

         if(motorThr!=NULL)
            motorThr->interrupt();

         return true;
    }

    bool respond(const Bottle &command, Bottle &reply)
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
                    reply.addString("Impedance turned off");
                }
                else
                {
                    reply.addString("What do you want? Impedance on or off?");
                }

                break;
            }

            case RPC_WAVEING:
            {
                if(command.get(1).asString()=="on")
                {
                    motorThr->setWaveing(true);
                    reply.addString("Waveing turned on");
                }
                else if(command.get(1).asString()=="off")
                {
                    motorThr->setWaveing(false);
                    reply.addString("Waveing turned off");
                }
                else
                {
                    reply.addString("What do you want? Waveing on or off?");
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

            case RPC_ELBOW:
            {
                if (command.size()>=4)
                {
                    string arm=command.get(1).asString().c_str();
                    double height=command.get(2).asDouble();
                    double weight=command.get(3).asDouble();

                    if ((arm=="both") || (arm=="left"))
                        motorThr->changeElbowHeight(LEFT,height,weight);

                    if ((arm=="both") || (arm=="right"))
                        motorThr->changeElbowHeight(RIGHT,height,weight);

                    reply.addString("elbow parameters updated");
                }
                else
                    reply.addString("missing elbow parameters");

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

    bool process(int &port_tag, Bottle &command, Bottle &reply)
    {
        if(closing)
        {
            reply.addVocab(NACK);
            reply.addString("Sorry. Module closing.");
            return true;
        }

        if(command.size()==0)
        {
            reply.addVocab(NACK);
            reply.addString("No command received.");
        }
        else if(interrupted)
        {
            reply.addVocab(NACK);
            reply.addString("Module currently interrupted. Reinstate for action.");
        }
        else
        {
            if(port_tag==PORT_TAG_GET)
            {
                if(command.get(0).asVocab()!=CMD_GET)
                {
                    reply.addVocab(NACK);
                    reply.addString("This port can only process [get] commands");
                }
                else if(command.size()>1)
                {
                    switch(command.get(1).asVocab())
                    {
                        case GET_S2C:
                        {
                            if(command.size()>2)
                            {
                                for(int target_idx=2; target_idx<command.size(); target_idx++)
                                {
                                    Vector xd;
                                    Bottle tmp_command;
                                    visuoThr->getTarget(command.get(target_idx),tmp_command);

                                    Bottle &tmp_reply=reply.addList();
                                    if(motorThr->targetToCartesian(tmp_command.find("target").asList(),xd))
                                    {
                                        for(size_t i=0; i<xd.size(); i++)
                                            tmp_reply.addDouble(xd[i]);
                                    }
                                    else
                                        tmp_reply.addVocab(NACK);
                                }
                            }

                            break;
                        }
                        case GET_IDLE:
                        {
                            reply.clear();
                            if(idle)
                                reply.addVocab(ACK);
                            else
                                reply.addVocab(NACK);

                            break;
                        }

                        case GET_TABLE:
                        {
                            reply.clear();
                            double table_height;
                            if(motorThr->getTableHeight(&table_height))
                            {
                                Bottle &tmp_reply=reply.addList();
                                tmp_reply.addString("table_height");
                                tmp_reply.addDouble(table_height);
                            }
                            else
                                reply.addVocab(NACK);

                            break;
                        }

                        case GET_HOLDING:
                        {
                            reply.clear();
                            if(motorThr->isHolding(command))
                                reply.addVocab(ACK);
                            else
                                reply.addVocab(NACK);

                            break;
                        }
                        
                        case GET_HAND:
                        {
                            if(command.get(2).asVocab()==GET_IMAGE)
                            {
                                reply.clear();
                                if(!motorThr->getHandImagePosition(reply))
                                    reply.addVocab(NACK);
                            }
                            else
                                reply.addVocab(NACK);

                            break;
                        }


                        default:
                        {
                            reply.addVocab(NACK);
                            break;
                        }
                    }
                }
                else
                    reply.addVocab(NACK);
            }
            else if(port_tag==PORT_TAG_CMD)
            {
                switch(command.get(0).asVocab())
                {
                    case CMD_IDLE:
                    {
                        motorThr->setGazeIdle();
                        reply.addVocab(ACK);
                        break;
                    }

                    case CMD_HOME:
                    {
                        motorThr->goHome(command);
                        reply.addVocab(ACK);

                        break;
                    }
                    case CMD_CLOSE:
                    {
                        motorThr->grasp(command);
                        reply.addVocab(ACK);

                        break;
                    }
                    case CMD_CLOSE_TOOL:
                    {
                        motorThr->grasp_tool(command);
                        reply.addVocab(ACK);

                        break;
                    }
                    case CMD_GAZE:
                    {
                        motorThr->clearIt(command);
                        reply.addVocab(ACK);

                        break;
                    }

                    //----------- for retro_compatibility -----------------
                    case CMD_GET:
                    {
                        if(command.size()>1)
                        {
                            switch(command.get(1).asVocab())
                            {
                                case GET_S2C:
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
                                            reply.addVocab(NACK);
                                    }

                                    break;
                                }
                            }

                            break;
                        }

                        break;
                    }
                    //-----------------------------------------------------

                    case CMD_EXPLORE:
                    {
                        if(command.size()>1)
                        {
                            switch(command.get(1).asVocab())
                            {
                                case EXPLORE_TORSO:
                                {
                                    if(motorThr->exploreTorso(command))
                                        reply.addVocab(ACK);
                                    else
                                        reply.addVocab(NACK);

                                    break;
                                }

                                case EXPLORE_HAND:
                                {
                                    if(motorThr->exploreHand(command))
                                        reply.addVocab(ACK);
                                    else
                                        reply.addVocab(NACK);

                                    break;
                                }

                                default:
                                {
                                    string rep=command.get(1).asString().c_str();
                                    reply.addVocab(NACK);
                                    reply.addString(("parameter '"+rep+"' not supported by 'explore' command.").c_str());
                                    break;
                                }
                            }
                        }
                        else
                        {
                            reply.addVocab(NACK);
                            reply.addString("explore command needs further parameter (e.g. 'torso')");
                        }
                        break;
                    }
                    
                    case CMD_CALIBRATE:
                    {
                        switch(command.get(1).asVocab())
                        {
                            case CALIB_TABLE:
                            {
                                idle = false;
                                if(motorThr->calibTable(command))
                                    reply.addVocab(ACK);
                                else
                                    reply.addVocab(NACK);
                                idle = true;
                                break;
                            }

                             case CALIB_FINGERS:
                            {
                                if(motorThr->calibFingers(command))
                                    reply.addVocab(ACK);
                                else
                                    reply.addVocab(NACK);

                                break;
                            }

                             case CALIB_KIN_OFFSET:
                            {
                                if(command.get(2).asString()=="start")
                                {
                                    if(motorThr->startLearningModeKinOffset(command))
                                    {
                                        reply.addVocab(ACK);
                                        reply.addString("learn kinematic offset mode: on");
                                    }
                                    else
                                        reply.addVocab(NACK);
                                }
                                else if(command.get(2).asString()=="stop")
                                {
                                    if(motorThr->suspendLearningModeKinOffset(command))
                                    {
                                        reply.addVocab(ACK);
                                        reply.addString("learn kinematic offset mode: off");
                                    }
                                    else
                                        reply.addVocab(NACK);
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
                            {
                                reply.addVocab(NACK);
                                reply.addString(("action "+action_name+" already known").c_str());
                            }
                            else
                            {
                                motorThr->setGazeIdle();
                                motorThr->lookAtHand(command);
                                reply.addVocab(ACK);
                                reply.addString("start teaching");
                            }
                        }

                        if(check(command,"stop"))
                        {
                            if(motorThr->suspendLearningModeAction(command))
                            {
                                motorThr->setGazeIdle();
                                reply.addVocab(ACK);
                                reply.addString("stop teaching");
                            }
                            else
                                reply.addVocab(NACK);
                        }

                        break;
                    }

                    case CMD_ACTION_IMITATE:
                    {
                        string action_name=command.get(1).asString().c_str();

                        Bottle &action=command.addList();
                        action.addString("action_name");
                        action.addString(action_name.c_str());

                        motorThr->lookAtHand(command);

                        if(!motorThr->imitateAction(command))
                        {
                            reply.addVocab(NACK);
                            reply.addString(("action "+action_name+" unknown").c_str());
                        }
                        else
                        {
                            reply.addVocab(ACK);
                            reply.addString(("action "+action_name+" done").c_str());
                        }
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
                        motorThr->exploreTorso(command);

                        visuoThr->trainMIL();

                        reply.addString((obj_name + " learned").c_str());
                        fprintf(stdout,"'%s' learned.\n",obj_name.c_str());

                        break;
                    }

                    case CMD_OBSERVE:
                    {
                        if(!motorThr->isHolding(command))
                        {
                            reply.addVocab(NACK);
                            reply.addString("Nothing to observe. Not holding anything");
                            motorThr->release(command);
                            motorThr->goHome(command);
                            break;
                        }

                        motorThr->lookAtHand(command);
                        motorThr->drawNear(command);
                        motorThr->setGazeIdle();

                        reply.addVocab(ACK);

                        break;
                    }

                    case CMD_EXPECT:
                    {
                        if(!motorThr->expect(command))
                        {
                            motorThr->setGazeIdle();
                            motorThr->release(command);
                            motorThr->goHome(command);
                            reply.addVocab(NACK);
                            break;
                        }

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

                            reply.addVocab(ACK);
                        }
                        else
                        {
                           motorThr->setGazeIdle();
                           motorThr->release(command);
                           motorThr->goHome(command);
                           reply.addVocab(NACK);
                        }

                        break;
                    }

                    case CMD_GIVE:
                    {
                        if(!motorThr->isHolding(command))
                        {
                            reply.addVocab(NACK);
                            reply.addString("Nothing to give. Not holding anything");
                            motorThr->release(command);
                            motorThr->goHome(command);
                            break;
                        }

                        motorThr->give(command);

                       motorThr->setGazeIdle();
                       motorThr->release(command);
                       motorThr->goHome(command);
                       reply.addVocab(NACK);

                        break;
                    }


                    case CMD_DROP:
                    {
                        //to be fixed by config ini...choice of either check for holding or not
                        //if(!motorThr->isHolding(command))
                        //{
                        //    reply.addVocab(NACK);
                        //    reply.addString("Nothing to drop. Not holding anything");
                        //    motorThr->release(command);
                        //    motorThr->goHome(command);
                        //    break;
                       // }
                        idle = false;
                        if(check(command,"over") && command.size()>2)
                            visuoThr->getTarget(command.get(2),command);

                        motorThr->setGazeIdle();

                        motorThr->deploy(command);
                        motorThr->keepFixation(command);
                        motorThr->goUp(command,0.1);

                        motorThr->goHome(command);
                        motorThr->setGazeIdle();

                        reply.addVocab(ACK);
                        idle = true;
                        break;
                    }

                    case CMD_TAKE:
                    {
                        idle = false;
                        if(command.size()<2)
                        {
                            reply.addVocab(NACK);
                            break;
                        }

                        visuoThr->getTarget(command.get(1),command);
                        motorThr->preTakeHand(command);

                        if(!motorThr->reach(command))
                        {
                            reply.addVocab(NACK);
                            break;
                        }

                        motorThr->lookAtHand(command);
                        motorThr->shiftAndGrasp(command);

                        if (motorThr->isHolding(command))
                        {
                            if (check(command,"near"))
                            {
                                motorThr->drawNear(command);
                                motorThr->setGazeIdle();
                            }
                            else
                            {
                                motorThr->goUp(command,0.1);
                                motorThr->setGazeIdle();

                                if (!check(command,"still"))
                                {
                                    Bottle b;
                                    b.addString("head");
                                    b.addString("arms");
                                    motorThr->goHome(b);
                                }
                            }

                            reply.addVocab(ACK);
                        }
                        else
                        {
                           motorThr->setGazeIdle();
                           motorThr->release(command);

                           if (!check(command,"still"))
                               motorThr->goHome(command);

                           reply.addVocab(NACK);
                        }
                        idle = true;

                        break;
                    }

                    case CMD_GRASP:
                    {
                        idle = false;
                        if (command.size()<2)
                        {
                            reply.addVocab(NACK);
                            break;
                        }

                        visuoThr->getTarget(command.get(1),command);
                        if (motorThr->powerGrasp(command))
                        {
                            motorThr->goUp(command,0.1);
                            motorThr->setGazeIdle();

                            if (!check(command,"still"))
                            {
                                Bottle b;
                                b.addString("head");
                                b.addString("arms");
                                motorThr->goHome(b);
                            }

                            reply.addVocab(ACK);
                        }
                        else
                        {
                            motorThr->setGazeIdle();
                            motorThr->release(command);
                            if (!check(command,"still"))
                                motorThr->goHome(command);

                            reply.addVocab(NACK);
                        }

                        idle = true;
                        break;
                    }

                    case CMD_TOUCH:
                    {
                         if(command.size()<2)
                        {
                            reply.addVocab(NACK);
                            break;
                        }

                        visuoThr->getTarget(command.get(1),command);

                        if(!motorThr->reach(command))
                        {
                            reply.addVocab(NACK);
                            break;
                        }

                        if(!check(command,"still"))
                        {
                            Time::delay(2.0);
                            motorThr->goHome(command);
                        }

                        reply.addVocab(ACK);
                        break;
                    } 

                    case CMD_PUSH:
                    {
                        if(command.size()<2)
                        {
                            reply.addVocab(NACK);
                            break;
                        }

                        visuoThr->getTarget(command.get(1),command);

                        if (!motorThr->push(command))
                        {
                            reply.addVocab(NACK);
                            break;
                        }

                        if (!check(command,"still"))
                        {
                            motorThr->setGazeIdle();
                            motorThr->goHome(command);
                        }

                        reply.addVocab(ACK);
                        break;
                    }

                    case CMD_POINT:
                    {
                        if(command.size()<2)
                        {
                            reply.addVocab(NACK);
                            break;
                        }

                        visuoThr->getTarget(command.get(1),command);

                        if(!motorThr->point(command))
                        {
                            reply.addVocab(NACK);
                            break;
                        }

                        if(!check(command,"still"))
                        {
                            Time::delay(2.0);
                            motorThr->goHome(command);
                        }

                        reply.addVocab(ACK);
                        break;
                    }

                    case CMD_LOOK:
                    {
                        if(command.size()<2)
                        {
                            reply.addVocab(NACK);
                            break;
                        }
                        
                        if(!check(command,"hand"))
                            visuoThr->getTarget(command.get(1),command);

                        if(!motorThr->look(command))
                        {
                            reply.addVocab(NACK);
                            break;
                        }

                        reply.addVocab(ACK);
                        break;
                    }

                    case CMD_TRACK:
                    {
                        //motion tracking is handled differently (temporary cheat)
                        if(command.get(1).asVocab()!=Vocab::encode("motion"))
                            visuoThr->getTarget(command.get(1),command);
                        else
                            visuoThr->trackMotion();

                        motorThr->trackTemplate(command);

                        reply.addVocab(ACK);

                        break;
                    }
                    case CMD_TAKE_TOOL:
                    {
                        if (command.size()<2)
                        {
                            reply.addVocab(NACK);
                            break;
                        }

                        motorThr->takeTool(command);
                        reply.addVocab(ACK);

                        break;
                    }
                    default:
                    {
                        reply.addVocab(NACK);
                        break;
                    }
                }
            }
            
        }

        if(reply.isNull() || reply.size()==0)
        {
            reply.addVocab(NACK);
            reply.addString("Random Error");
        }

        return true;
    }


};





class ARE_PortReader: public PortReader
{
protected:
    ActionsRenderingEngine                      *are;

    int                                         port_tag;


    bool read(ConnectionReader &connection)
    {
        Bottle command, reply;
        if (!command.read(connection))
            return false;

        //do stuff with the ARE here
        are->process(port_tag,command,reply);
        //-------------

        if (ConnectionWriter *writer=connection.getWriter())
            reply.write(*writer);

        return true;
    }

public:
    ARE_PortReader(ActionsRenderingEngine *_are, int _port_tag)
        :are(_are),port_tag(_port_tag)
    {}

};



class ActionsRenderingEngineModule: public RFModule
{
protected:
    ActionsRenderingEngine      *are;

    ARE_PortReader              *port_reader_cmd;
    ARE_PortReader              *port_reader_get;

    RpcServer                   port_cmd;
    RpcServer                   port_get;
    RpcServer                   port_rpc;


public:
    ActionsRenderingEngineModule()
    {}

    virtual bool configure(ResourceFinder &rf)
    {
        string name=rf.check("name",Value("actionsRenderingEngine")).asString().c_str();
        setName(name.c_str());

        are=new ActionsRenderingEngine();

        if(!are->initialize(rf))
        {
            fprintf(stdout,"[ARE][Error] ARE could not initialize\n");
            return false;
        }

        port_reader_cmd=new ARE_PortReader(are,PORT_TAG_CMD);
        port_reader_get=new ARE_PortReader(are,PORT_TAG_GET);

        port_cmd.setReader(*port_reader_cmd);
        port_get.setReader(*port_reader_get);

        port_cmd.open(("/"+name+"/cmd:io").c_str());
        port_get.open(("/"+name+"/get:io").c_str());
        port_rpc.open(("/"+name+"/rpc").c_str());

        attach(port_rpc);

        return true;
    }

    virtual bool interruptModule()
    {
        port_cmd.interrupt();
        port_get.interrupt();
        port_rpc.interrupt();

        are->interrupt();

        return true;
    }

    virtual bool close()
    {
        port_cmd.close();
        port_get.close();
        port_rpc.close();

        are->close();

        delete port_reader_cmd;
        delete port_reader_get;

        delete are;

        return true;
    }

    virtual double getPeriod()
    {
        return 0.1;
    }

    virtual bool updateModule()
    {
        return true;
    }

    virtual bool respond(const Bottle &command, Bottle &reply)
    {
        if(are->respond(command,reply))
            return true;

        return RFModule::respond(command,reply);
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
    rf.setDefaultContext("actionsRenderingEngine");
    rf.setDefaultConfigFile("config.ini");
    rf.setDefault("name","actionsRenderingEngine");
    rf.configure(argc,argv);

    ActionsRenderingEngineModule mod;

    return mod.runModule(rf);
}


