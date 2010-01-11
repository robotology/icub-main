// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
// vim:expandtab:tabstop=4:shiftwidth=4:softtabstop=4:

/*
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Assif Mirza, modified by Frank Broz
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
 * @addtogroup icub_ihaNew_IcubControl
 *

\section intro_sec Description
Icub Controller for IHA 2.0

This module sends action sequence messages to the robot to move it.
Changes from IHA 1.0 include removing code to control the facial
expressions (which is now handled by the motivation dynamics module)
and accessing the sequences through the action definition file
rather than specifying the sequence dir on the command line.

\section lib_sec Libraries
- YARP libraries.
- IHA Debug Library

\section parameters_sec Parameters
\verbatim
--dbg [INT]   : debug printing level
--name [STR]  : process name for ports
--file [STR]  : config file

--robot [STR]                : prefix of robot ports e.g. icubSim
--action_defs [STR]          : file defining actions
--motor_motion_timeout [INT] : time (ms) to wait for a motor to move
--sensor_output_rate [INT]   : sensor rate
--speed_multiplier [FLT]     : movement speed multiplier
--always_set_speed [STR]     : set speed on motor even if it hasnt changed
\endverbatim

\section portsa_sec Ports Accessed

\section portsc_sec Ports Created
- /iha/controller/action:cmd  for sending action commands
- /iha/controller/encoders:out for processes to recieve encoder values
 
- /iha/sm/quit  - module quit port

\section conf_file_sec Configuration Files
conf/ihaIcubControl.ini

Sample INI file:
\verbatim
name ihacontrol

robot icub

############################################################
# Which robot parts are active
head 1
left_arm 1
right_arm 1
torso 0
left_leg 0
right_leg 0
############################################################

############################################################
# Actions
#
action_defs conf/iha_actiondefs.ini
#
############################################################

# time before we give up on a motor motion
motor_motion_timeout 5000

# Output rate of encoders data Hz
sensor_output_rate 10

speed_multiplier 1.0
always_set_speed TRUE
\endverbatim

\section tested_os_sec Tested OS
Linux

\section example_sec Example Instantiation of the Module
ihaIcubControl --name /iha/controller --robot iCub --file conf/ihaIcubControl.ini --action_defs conf/iha_actiondefs.ini --speed_multiplier 1

See script $ICUB_ROOT/app/ihaNew/controller.sh

\see iCub::iha::ActionSelect
\see iCub::iha::Actions
\see iCub::contrib::IcubControlModule
\see iCub::iha::EncodersOutputThread

\author Assif Mirza, modified by Frank Broz

Copyright (C) 2009 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at \in src/interactionHistoryNew/icub_control/src/IcubControlModule.cpp.
*/

#include <yarp/String.h>
#include <yarp/os/all.h>

// Constants
//const int numParts = 6;
#define numParts 6
yarp::os::ConstString partName[numParts] = {"Head","LArm","RArm","Torso","LLeg","RLeg"};
yarp::os::ConstString partConfigName[numParts] = {"head","left_arm","right_arm","torso","left_leg","right_leg"};

#include <iCub/ihaNew/IcubControlModule.h>
#include <iCub/ihaNew/iha_utils.h>
using namespace iCub::ihaNew;

// MODULE Implementation ================================================================

bool IcubControlModule::open(Searchable& config){
   
	if (config.check("dbg")) { IhaDebug::setLevel(config.find("dbg").asInt()); }
 	ACE_OS::fprintf(stderr, "Debug level : %d\n",IhaDebug::getLevel());

    if (config.check("help","if present, display usage message")) {
		cerr << "Usage : " << "\n"
		<< "------------------------------------------" << "\n"
		<< "  --dbg [INT]   : debug printing level" << "\n"
		<< "  --name [STR]  : process name for ports" << "\n"
		<< "  --file [STR]  : config file" << "\n"
		<< "---------------------------------------------------------------------------" << "\n"
		<< "  --robot [STR]                : prefix of robot ports e.g. icubSim" << "\n"
        << "  --action_defs [STR]          : file defining actions" << "\n"
        << "  --motor_motion_timeout [INT] : time (ms) to wait for a motor to move" << "\n"
        << "  --sensor_output_rate [INT]   : sensor rate" << "\n"
        << "  --speed_multiplier [FLT]     : movement speed multiplier" << "\n"
        << "  --always_set_speed [STR]     : set speed on motor even if it hasnt changed" << "\n"
		<< "---------------------------------------------------------------------------" << "\n"
		<< "\n";
        return false;
    }

    bool ok = true;

    //------------------------------------------------------
	// get all the configured actions
	ConstString action_defs_file = config.check("action_defs",Value("conf/action_defs.ini")).asString();

	// create the action defs object and read the actions
	// from the config file
	Property actiondefs_props;
	actiondefs_props.fromConfigFile(action_defs_file.c_str()); 

	if (!iCubActions.open(actiondefs_props)) {
		fprintf(stderr,"Error in action definitions\n");
		exit(-1);
	}

	action_commands = iCubActions.getFullActionCommandList();
	NUM_ACTIONS = action_commands.size();
	IhaDebug::pmesg(DBGL_STATUS1,(char *)"NUM_ACTIONS defined : %d\n",NUM_ACTIONS);
	
    currentAction=0;

    //------------------------------------------------------
    // create names of ports
    ConstString commandPortName = getName("action:cmd");
    ConstString encodersPortName = getName("encoders:out");

	// open the ports
	commandPort.open(commandPortName.c_str());
	encodersPort.open(encodersPortName.c_str());

	//------------------------------------------------------
    // Read parameters
	// Motor motion timeout
	motor_motion_timeout = config.check("motor_motion_timeout",Value(5000)).asInt();
    IhaDebug::pmesg(DBGL_INFO,(char *)"motor_motion_timeout:%d\n",motor_motion_timeout);
	sensor_output_rate = config.check("sensor_output_rate",Value(20)).asInt();
    IhaDebug::pmesg(DBGL_INFO,(char *)"sensor_output_rate:%d\n",sensor_output_rate);
	speed_multiplier = config.check("speed_multiplier",Value(1.0)).asDouble();
    IhaDebug::pmesg(DBGL_INFO,(char *)"speed_multiplier:%f\n",speed_multiplier);
    always_set_speed = boolStringTest(config.check("always_set_speed",Value("FALSE")).asString());
    IhaDebug::pmesg(DBGL_INFO,(char *)"always_set_speed %s\n",always_set_speed?"TRUE":"FALSE");


	//------------------------------------------------------
	// Get robot configuration
	Value robot = config.check("robot",Value("icubSim"));
    IhaDebug::pmesg(DBGL_INFO,(char *)"robot:%s\n",robot.asString().c_str());

	// there will be a line in the config for each configured part e.g. 
	// head 1
	// left_arm 1
	// right_arm 0

	IhaDebug::pmesg(DBGL_INFO,(char *)"Active parts:\n");
	for (int part=0;part<numParts;part++) {
		active[part] = config.check(partConfigName[part].c_str(),Value(1)).asInt()==1;
		IhaDebug::pmesg(DBGL_INFO,(char *)"  %s %s\n",partConfigName[part].c_str(),active[part]?"TRUE":"FALSE");
	}

	PolyDriver* dd[numParts];

	//------------------------------------------------------
	// initialise connect to robot
	
	char tempname[100];
	Property partconfig;

	numJointsTotal=0;
	for (int part=0;part<numParts;part++) {
		if (active[part]) {
			partconfig.clear();
			partconfig.put("device","remote_controlboard");
			partconfig.put("robot",robot);

			partconfig.put("part",partConfigName[part].c_str());
			ACE_OS::sprintf(tempname,"/%s/%s",robot.asString().c_str(), partConfigName[part].c_str());
			partconfig.put("remote",tempname);
			ACE_OS::sprintf(tempname,"/%s/%s/client",robot.asString().c_str(), partConfigName[part].c_str());
			partconfig.put("local",tempname);

			fprintf(stderr,"%s",partconfig.toString().c_str());
			fprintf(stderr,"\n");

			fprintf(stderr,"Create device %s\n", partConfigName[part].c_str());
			dd[part] = new PolyDriver(partconfig);
			if (!dd[part]->isValid()) {
				fprintf(stderr,"Cannot create driver %s\n", partConfigName[part].c_str());
				exit(1);
			}

			fprintf(stderr,"View %s as IPositionControl\n", partConfigName[part].c_str());
			dd[part]->view(pos[part]);
			if (pos[part]==NULL) {
				fprintf(stderr,"Cannot view part %s as IPositionControl\n", partConfigName[part].c_str());
				exit(1);
			}

			fprintf(stderr,"View head as IEncoders\n", partConfigName[part].c_str());
			dd[part]->view(enc[part]);
			if (enc[part]==NULL) {
				fprintf(stderr,"Cannot view part %s as Iencoders\n", partConfigName[part].c_str());
				exit(1);
			}

			if (!pos[part]->getAxes(&numJoints[part])) {
				fprintf(stderr,"Cannot get number of axes for %s\n", partConfigName[part].c_str());
				exit(1);
			}
			IhaDebug::pmesg(DBGL_STATUS1,(char *)"%s returns %d axes\n", partConfigName[part].c_str(),numJoints[part]);

			numJointsTotal+=numJoints[part];
		}
	}

    IhaDebug::pmesg(DBGL_DEBUG2,(char *)"Allocate arrays. numParts=%d\n",numParts);
	// allocate the motor level arrays
	for (int part=0;part<numParts;part++) {
		if (active[part]) {
            refs[part] = new double[numJoints[part]];
            speeds[part] = new double[numJoints[part]];
            encs[part] = new double[numJoints[part]];
            accls[part] = new double[numJoints[part]];

            motorMoving[part] = new bool[numJoints[part]];
            motorFault[part] = new bool[numJoints[part]];

            partMoving[part]=false;

            for (int j=0;j<numJoints[part];j++) {
                refs[part][j] = 0.0;
                speeds[part][j] = 0.0;
                encs[part][j] = 0.0;
                //reference acceleration used to generate the velocity profile
                //set to 50 degrees/sec^2 (taken from arm control tutorial)
                accls[part][j] = 50.0;
                motorMoving[part][j]=false;
                motorFault[part][j]=false;
            }
            
            //set the reference acceleration for the part
            pos[part]->setRefAccelerations(accls[part]);
        }
	}


    //debugPrintArrays();

    // for the action  command reply
	response.addString("ACK");

    //---------------------------------------------------------
    // Start the encoders output thread
	IhaDebug::pmesg(DBGL_STATUS1, (char *)"Starting Encoders Output Thread\n");
    encodersOutputThread = new EncodersOutputThread((int)(1000.0/sensor_output_rate), this, encodersPort);
	encodersOutputThread->start();

    //---------------------------------------------------------
    // Standard Quit port
    ok &= quitPort.open(getName("quit"));
    attach(quitPort, true);

    return ok;
}

bool IcubControlModule::close(){
    commandPort.close();
    encodersPort.close();
    encodersOutputThread->stop();
    
	for (int part=0;part<numParts;part++) {
		if (active[part]) {
            delete [] refs[part];
            delete [] speeds[part];
            delete [] encs[part];
            delete [] accls[part];

            delete [] motorMoving[part];
            delete [] motorFault[part];
        }
	}

    return true;
}

bool IcubControlModule::interruptModule(){
    commandPort.interrupt();
    encodersPort.interrupt();
    return true;
}

bool IcubControlModule::updateModule(){
    IhaDebug::pmesg(DBGL_DEBUG2,(char *)"IcubControlModule::updateModule() called()\n");
    if (commandPort.isWriting()) {
        IhaDebug::pmesg(DBGL_DEBUG2,(char *)"Port Still writing.\n");
        Time::delay(0.1);
        return true;
    }

    Bottle cmd;
    if (commandPort.read(cmd,true)) // true means that this code will reply
    {
        IhaDebug::pmesg(DBGL_DEBUG2,(char *)"Got command %s\n",cmd.toString().c_str());

        // the current action
        //IhaDebug::pmesg(DBGL_DEBUG2,(char *)"Current Action : %d\n",currentAction);
        IhaDebug::pmesg(DBGL_DEBUG2,(char *)"Current Action : %d %s\n", \
                        currentAction,iCubActions.getActionName(currentAction).c_str());
        double st=Time::now();

        if (cmd.size() > 0) {
            currentAction = cmd.get(0).asInt();
            if (!sendAction(currentAction)) {
                ACE_OS::fprintf(stderr,"sendAction failed\n");
                //return false;
            } else {
                IhaDebug::pmesg(DBGL_DEBUG2,(char *)"Action %d took %f seconds\n",currentAction,Time::now()-st);
            }
            IhaDebug::pmesg(DBGL_DEBUG2,(char *)"Sending ACK\n");
            commandPort.reply(response);
        }
    } else {
        IhaDebug::pmesg(DBGL_DEBUG2,(char *)"No command\n");
        Time::delay(0.1);
    }

    return true;
}

bool IcubControlModule::respond(const Bottle &command,Bottle &reply){
        
    return false;
} 	


// iCub Interface Implementation =============================================================

/** 
 * Single motor version of doPositionMove
 * joint number within part moved
 */
bool IcubControlModule::doPositionMove(IPositionControl *pos, int joint, int ref, int speed) {
	if (ref==JOINTREF_UNDEF) return true;
	int sp = getMultipledSpeed(speed);
	IhaDebug::pmesg(DBGL_DEBUG2,(char *)"setRefSpeed(%d,%d)\n",joint,sp);
	if (!pos->setRefSpeed(joint,sp)) {
		ACE_OS::fprintf(stderr,"Error setting ref speed\n");
		return false;
	}
	IhaDebug::pmesg(DBGL_DEBUG3,(char *)"Joint %d Ref %d Speed %d\n",joint,ref,speed);
	if (!pos->positionMove(joint,ref)) {
		ACE_OS::fprintf(stderr,"Error in positionMove\n");
		return false;
	}
	return true;
}
/** 
 * Single motor version of doPositionMove
 * joint number within part moved
 * no speed change
 */
bool IcubControlModule::doPositionMove(IPositionControl *pos, int joint, int ref) {
	if (ref==JOINTREF_UNDEF) return true;
	IhaDebug::pmesg(DBGL_DEBUG3,(char *)"Joint %d Ref %d \n",joint,ref);
	if (!pos->positionMove(joint,ref)) {
		ACE_OS::fprintf(stderr,"Error in positionMove\n");
		return false;
	}
	return true;
}
/** 
 * Single motor version of doPositionMove
 * joint number if 0-52
 */
bool IcubControlModule::doPositionMove(int joint, int ref, int speed) {
	if (ref==JOINTREF_UNDEF) return true; // handle undefined joint in a multi

	int index=0;
	for (int part=0; part<numParts; part++) {
		if (active[part] && joint<index+numJoints[part]) {
			IhaDebug::pmesg(DBGL_DEBUG2,(char *)"%s : Move joint %d to %d Speed %d \n", partName[part].c_str(), joint-index, ref, speed);
			return doPositionMove(pos[part], joint-index, ref, speed);
		}
		index += numJoints[part];
	}

	ACE_OS::fprintf(stderr,"Joint %d out of range \n",joint);
	return false;
}

/** 
 * Single motor version of doSetSpeed
 * joint number within part moved
 */
bool IcubControlModule::doSetSpeed(IPositionControl *pos, int joint, int speed) {
	int sp = getMultipledSpeed(speed);
	IhaDebug::pmesg(DBGL_DEBUG2,(char *)"setRefSpeed(%d,%d)\n",joint,sp);
	if (!pos->setRefSpeed(joint,sp)) {
		ACE_OS::fprintf(stderr,"Error setting ref speed\n");
		return false;
	}
	return true;
}
/** 
 * Single motor version of doSetSpeed
 * joint number if 0-52
 */
bool IcubControlModule::doSetSpeed(int joint, int speed) {
	int index=0;
	for (int part=0; part<numParts; part++) {
		if (active[part] && joint<index+numJoints[part]) {
			IhaDebug::pmesg(DBGL_DEBUG2,(char *)"%s : Set speed joint %d to %d \n", partName[part].c_str(), joint-index, speed);
			return doSetSpeed(pos[part], joint-index, speed);
		}
		index += numJoints[part];
	}

	ACE_OS::fprintf(stderr,"Joint %d out of range \n",joint);
	return false;
}

/** 
 * Multi motor version of doPositionMove
 * Does not set speed
 */
bool IcubControlModule::doMultiPositionMove(IPositionControl *posc, const double *refs) {
	if (!posc->positionMove(refs)) {
		ACE_OS::fprintf(stderr,"Error in multi positionMove\n");
		return false;
	}
	return true;
}


/** 
 * Wait for a motion to be complete
 */
bool IcubControlModule::waitForCompletion() {
	bool complete=false; // keep running till this is true
		
	double st=Time::now(); // we may timeout if some motor/encoder pairs are faulty

	IhaDebug::pmesg(DBGL_DEBUG1,(char *)"motors to move (M=move, F=fault):\n             ");
	for (int part=0;part<numParts;part++) {
		if (active[part]) {
			IhaDebug::pmesg(DBGL_DEBUG1,(char *)" %s:",partName[part].c_str());
			for (int j=0;j<numJoints[part];j++) {
				IhaDebug::pmesg(DBGL_DEBUG1,(char *)"%s",motorFault[part][j]?"F":(motorMoving[part][j]?"M":partMoving[part]?".":" "));
			}
		}
	}
	IhaDebug::pmesg(DBGL_DEBUG1,(char *)"\n");


	while (!complete) {
		complete=true;
		for (int part=0;part<numParts;part++) {
			// only check active parts that are set to move
			if (active[part] && partMoving[part]) {
				partMoving[part]=false; // will be set true again if something is still moving
				// only check motors we put in motion ourselves that are not faulty
				for (int j=0;j<numJoints[part];j++) {
					if (motorMoving[part][j] && !motorFault[part][j]) {
						bool flag=false;
						if (!pos[part]->checkMotionDone(j,&flag)) {
							ACE_OS::fprintf(stderr,"Error calling checkMotionDone\n");
							//return false;
						}
						motorMoving[part][j]=!flag;
						// just one moving motor (that is not faulty) should negate complete
						complete &= !motorMoving[part][j];
						if (motorMoving[part][j]) partMoving[part]=true;
					}
				} // joint
			} // part
		}
		
		IhaDebug::pmesg(DBGL_DEBUG1,"motorsMoving:");
		for (int part=0;part<numParts;part++) {
			if (active[part]) {
				IhaDebug::pmesg(DBGL_DEBUG1,(char *)" %s:",partName[part].c_str());
				for (int j=0;j<numJoints[part];j++) {
					IhaDebug::pmesg(DBGL_DEBUG1,(char *)"%s",partMoving[part]==false?" ":motorMoving[part][j]?"T":".");
				}
			}
		}
		IhaDebug::pmesg(DBGL_DEBUG1,(char *)"\n");

		// Check timeout. If any motors are still in motion, mark them as faulty
		// and break out of here
		if (Time::now()-st > motor_motion_timeout/1000) {
			IhaDebug::pmesg(DBGL_STATUS1,(char *)"Motor Motion Timout\n");
			for (int part=0;part<numParts;part++) {
				if (active[part] ) {
					for (int j=0;j<numJoints[part];j++) {
						motorFault[part][j]=motorMoving[part][j];
						if (motorFault[part][j]) IhaDebug::pmesg(DBGL_STATUS1,(char *)"%s:%d faulty.\n",partName[part].c_str(),j);
					}
				}
			}
			return true;
		}
		if (!complete) {
			Time::delay(0.05);
			if (IhaDebug::getLevel()<DBGL_DEBUG1) IhaDebug::pmesg(DBGL_STATUS1,(char *)"*");
		}
	}
	return true;
}


bool IcubControlModule::multiMove() 
{
    /**
	 * allDef flag will tell if all joints in a group are being moved
	 * if so, then we can do a proper multiPosition move, otherwise
	 * we will have to do individual position moves for each motot
	 * This is necessary only in implementations which can operate
	 * optimized for all motors moving
     */
	bool allDef = true;
 
 	// loop at part level
	for (int part=0;part<numParts;part++) {
		// only for active parts
		if (active[part] && partMoving[part]) {
/*			allDef=true;
			for (int j=0;j<numJoints[part];j++) { 
				// if any joint does not have a reference set
				// then individual moves are necessary
				if (refs[part][j]==JOINTREF_UNDEF) { allDef=false; break; } 
			}

			if (allDef) {
				pmesg(DBGL_STATUS2,"Calling doMultiPositionMove(posHead, refs[%d])\n",part);
				if (!doMultiPositionMove(pos[part], refs[part])) {
					fprintf(stderr,"doMultiPositionMove - %s failed\n", partName[part].c_str());
					return false;
				}
			}
			else */ {
				IhaDebug::pmesg(DBGL_STATUS2,"Calling individual doPositionMoves for %s\n", partName[part].c_str());
				for (int j=0;j<numJoints[part];j++) {
					if (motorMoving[part][j]) {
						if (!doPositionMove(pos[part], j, (int)refs[part][j])) {
							ACE_OS::fprintf(stderr,"Err calling doPositionMove\n"); 
							return false;
						}
					}
				}
			}
		}
	}
						
	return true;
}

// runs a sequence string
// this is the parallel version
bool IcubControlModule::runSequenceParallel(string seq) {
    IhaDebug::pmesg(DBGL_DEBUG2,(char *)"runSequenceParallel: %s\n",seq.c_str());
	bool running = true;
	string str;

	// reset refs to be undefined
	// and motors not moving
	for (int part=0;part<numParts;part++) {
        if (active[part]) {
            partMoving[part]=false;
            for (int j=0;j<numJoints[part];j++) { 
                refs[part][j]=JOINTREF_UNDEF; 
                motorMoving[part][j]=false; 
            }
        }
	}
	bool multiMovePending = false;

	// view the sequence line as a string stream
	istringstream ss(seq.c_str());

	while (ss >> str && running) {
		if (str=="POS") 
		{
			// parse the string stream
			int joint, ref, speed;
			ss >> joint;	// note this is the absolute joint number (not the number within the part)
			ss >> ref;
			ss >> speed;
			IhaDebug::pmesg(DBGL_DEBUG3,(char *)"pos request  %d %d %d\n",joint, ref, speed);

			int movePart = 0;  // to hold part of requested joint move
			int moveJoint = 0; // and joint number 

			int index=0;
			for (int part=0;part<numParts;part++) {
				if (joint < index+numJoints[part]) {
					// set the joint/part number
					movePart=part;
					moveJoint=joint-index;
					// set the reference in the refs array
					refs[part][joint-index]=ref;
					break;
				}
				index += numJoints[part];
			}
			IhaDebug::pmesg(DBGL_DEBUG1,(char *)"pos request %s:%d ref:%d spd:%d\n", partName[movePart].c_str(), moveJoint, ref, speed);

			// set the speed of the joint if it has changed
			if (always_set_speed || speeds[movePart][moveJoint]!=speed) {
				IhaDebug::pmesg(DBGL_DEBUG2,(char *)"set speed for %s:%d to %d\n",partName[movePart].c_str(), moveJoint, speed);
				if (!doSetSpeed(pos[movePart],moveJoint,speed)) {
					fprintf(stderr,"Error setting ref speed\n");
					running=false;
				}
				speeds[movePart][moveJoint]=speed;
			}
			
			// Note that this joint is moving
			motorMoving[movePart][moveJoint]=true;
			// Note that this part is moving
			partMoving[movePart]=true;

			multiMovePending=true;
		} 
		else if (str=="BLK") 
		{

			// we have been accumulating a multi move
			// so now send it
			if (multiMovePending) {
				IhaDebug::pmesg(DBGL_DEBUG1, (char *)"Sending multi pos move\n");

				double st=Time::now();
				if (!multiMove()) {
					fprintf(stderr,"multiMove failed\n");
					//running=false;
					return false;
				}
				IhaDebug::pmesg(DBGL_DEBUG2,(char *)"Multi took %f seconds\n",Time::now()-st);

				multiMovePending=false;
			}

			IhaDebug::pmesg(DBGL_DEBUG1,(char *)"Blocking\n");
			double st=Time::now();
			if (!waitForCompletion()) {
				fprintf(stderr,"waitForCompletion failed\n");
				//running=false;
				return false;
			}
			IhaDebug::pmesg(DBGL_DEBUG1,(char *)"BLK took %f seconds\n",Time::now()-st);

		} 
		else 
		{
			IhaDebug::pmesg(DBGL_INFO,(char *)"Incorrect string: %s\n",seq.c_str());
			running=false;
		}
	}
	if (running && multiMovePending) {
		IhaDebug::pmesg(DBGL_DEBUG1, (char *)"Sending multi pos move\n");

		double st=Time::now();
		if (!multiMove()) {
			fprintf(stderr,(char *)"multiMove failed\n");
			running=false;
		}
		IhaDebug::pmesg(DBGL_DEBUG2,(char *)"Multi took %f seconds\n",Time::now()-st);

		IhaDebug::pmesg(DBGL_DEBUG1,(char *)"Blocking\n");

		st=Time::now();
		if (!waitForCompletion()) {
			fprintf(stderr,"waitForCompletion failed\n");
			running=false;
			return false;
		}
		IhaDebug::pmesg(DBGL_DEBUG1,(char *)"BLK took %f seconds\n",Time::now()-st);
	}

	return running;
}


bool IcubControlModule::sendAction(int act) {
	bool retval=true;
    //if (act==1 || act==2 || act==16) { // this needs to change to use BSet5
    //    IhaDebug::pmesg(DBGL_STATUS2,(char *)"sendExpression %d started.\n",act);
    //    retval = sendExpression(act);
    //    IhaDebug::pmesg(DBGL_STATUS2,(char *)"sendExpression %d complete. %s\n",act, retval?"":"FAILED");
    //}
    if (act < 0 || act > iCubActions.getNumActions()) {
        IhaDebug::pmesg(DBGL_STATUS2,(char *)"Action %d not configured.\n",act);
        return false;
    }
        
	IhaDebug::pmesg(DBGL_STATUS2,(char *)"sendAction %d started.\n",act);
	retval = runSequenceParallel ( iCubActions.getSequenceString(act) );
	IhaDebug::pmesg(DBGL_STATUS2,(char *)"sendAction %d complete. %s\n",act,retval?"":"FAILED");
	return retval;
}

double** IcubControlModule::getEncoderReadings() {
    for (int part=0;part<numParts;part++) {
        if (active[part]) {
            if (!enc[part]->getEncoders(encs[part])) {
                ACE_OS::fprintf(stderr,"Error reading %s encoders values\n",partName[part].c_str());
                //debugPrintArrays();
            } else {
                IhaDebug::pmesg(DBGL_DEBUG2,(char *)"Got part=%d=%s encoders.\n",part,partName[part].c_str());
            }
        }
    }
    return encs;
}

void iCub::iha::EncodersOutputThread::run() {
    double start = Time::now();

    //ACE_OS::fprintf(stderr,"EncodersOutputThread::run() %f ms.\n",1000*(Time::now()-progstart));

    double** _encs = controller->getEncoderReadings();
    //IhaDebug::pmesg(DBGL_DEBUG2,"getEncoders took %f ms.\n",1000*(Time::now() - start));

    //for (int part=0;part<numParts;part++) {
    //    if (controller->isActive(part)) {
    //        for (int j=0;j<controller->getNumJoints(part);j++) {
    //            ACE_OS::fprintf(stderr,"_encs[%d][%d]=%f\n",part,j,_encs[part][j]);
    //        }
    //    }
    //}
    //controller->debugPrintArrays();

    Bottle bot;
    // timestamp as first item
    int ms = (int) ( 1000*(Time::now() - progstart) );
    bot.addInt(ms);

    // Sensor readings
    for (int part=0;part<numParts;part++) {
        if (controller->isActive(part)) {
            for (int j=0;j<controller->getNumJoints(part);j++) {
                bot.addDouble(_encs[part][j]);
            }
        }
    }
    
    // current action value
    int ca = controller->getCurrentAction();
    bot.addDouble((double)ca);

    // write to the output port
    outPort->write(bot);

    if (IhaDebug::getLevel()==DBGL_STATUSLINE) {
        IhaDebug::pmesg(DBGL_STATUSLINE,(char *)"TS: %d Action: %f\r",ms,ca);
    } else {
        IhaDebug::pmesg(DBGL_STATUS1,(char *)"TS: %d Action: %f\n",ms,ca);
    }
        
    IhaDebug::pmesg(DBGL_DEBUG2,(char *)"EncodersOutputThread::run() took %f ms.\n",1000*(Time::now()-start));
}

void IcubControlModule::debugPrintArrays() {
    ACE_OS::fprintf(stderr,"Arrays:\n");
	for (int part=0;part<numParts;part++) {
		if (active[part]) {
            ACE_OS::fprintf(stderr,(char *)"Part %d. Joints %d : \n",part,numJoints[part]);
             for (int j=0;j<numJoints[part];j++) {
                ACE_OS::fprintf(stderr,"  Joint %d:  ",j);
                ACE_OS::fprintf(stderr,"r=%f ",refs[part][j]);
                ACE_OS::fprintf(stderr,"s=%f ",speeds[part][j]);
                ACE_OS::fprintf(stderr,"e=%f ",encs[part][j]);
                ACE_OS::fprintf(stderr,"m=%s ",motorMoving[part][j]?"T":"F");
                ACE_OS::fprintf(stderr,"f=%s\n",motorFault[part][j]?"T":"F");
             }
        }
     }
}
    

IcubControlModule::IcubControlModule(){
}

IcubControlModule::~IcubControlModule(){ 
}


