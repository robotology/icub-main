// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
// vim:expandtab:tabstop=4:shiftwidth=4:softtabstop=4:

/*
 * Copyright (C) 2008 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Assif Mirza
 * email:   assif.mirza@robotcub.org
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


#include <iCub/iha/ActionSelectionModule.h>
#include <iCub/iha/ActionSelect.h>
#include <iCub/iha/iha_utils.h>

using namespace iCub::iha;

/**
 * @addtogroup icub_iha_ActionSelection
 *

\section icub_iha_as_intro_sec Action Selection
A simple mechanism is adopted for action selection whereby the robot can execute one of a number of ``atomic'' actions (or no action) at any timestep. This is seen as a tractable first-step, and a more sophisticated action or behaviour generation capability would allow for more open-ended development.

The actual action selected will either be a random selection of one of the atomic actions, or will be an action that was previously executed after an experience in the history that is \e near to the current episode. An advantage of this approach is that behaviour can be bootstrapped from early random activity, and later behaviour built on previous experience.

The process of action selection is as follows: 

\li up to \f$K\f$ \e candidate \e experiences from the experience space within a given information distance \e radius \f$r_0\f$ of the current experience \f$E_{current}\f$ are initially selected;
\li these \f$K\f$ experiences are ranked as \f$E_1,\ldots ,E_K\f$ according to how close they are to \f$E_{current}\f$;
\li then, experience \f$E_i\f$ is chosen using the \b Roulette-Wheel \b Selection process described below;
\li if an experience is chosen from the candidate list, then the particular action that was executed following the chosen experience is then chosen as the action to be executed next, otherwise a random action is chosen. 

The selection also ensures that, with small probability, the robot may still choose a random action as this may potentially help to discover new, more salient experiences. This is also controlled by the \b Temperature parameter.
This has the advantage of emulating body-babbling, \e i.e. apparently random body movements that have the (hypothesized) purpose of learning the capabilities of the body in an environment. 
Early in development, there are fewer, more widely spread experiences in the space, so random actions would be chosen more often. Later in development, it is more likely that an the action selected will come from past experience.  

\subsection icub_iha_as_roulette Roulette-Wheel Action Selection
The chance of random action selection is represented as a probability.
The probabilities are calculated using a "gravitational model" where each experience is represented as a point mass a particular distance from the \f$E_{current}\f$. The probability of selecting an experience \f$E_i\f$ from \f$E_1,\ldots ,E_K\f$ is:
\f[
 p_i = C_h \frac{m_i q_i}{{D(E_{current},E_i)}^2}
\f]
where \f$q_i\f$ is the \e {quality value} of \f$E_i\f$, \f$m_i\f$ is the mass (ie how many experiences have been merged into this experience) and \f$D(E_{current},E_i)\f$ is the experience distance.  \f$C_h\f$ is an optional quantity that is used to adjust for ``horizon effect'' when considering experiences of different horizon length together, and is given by 
\f[
 C_h = \frac{\sqrt{h}}{\sqrt{H_{max}}}
\f]
The chance of random is added to the list as:
\f[
 p_0 = \frac{\sum_{i=1}^K {p_i} } { {(r_{max}/\tau)}^2 }
\f]
where \f$r_{max}\f$ is the radius of the ball that includes the ranked experiences and \f$\tau\f$ is a \e {temperature} factor, that controls the chance of random action selection.

Then the weighting on the ``roulette wheel'' is given by:
\f[
 w_i = \frac{p_i}{\sum_{i=0}^K p_i}
\f]


\section lib_sec Libraries
- YARP libraries.
- IHA Debug Library

\section parameters_sec Parameters
\verbatim
--dbg <INT>                : debug printing level
--name <STR>               : process name for ports
--file <STR>               : configuration from given file

--connect_to_coords <STR>  : connect to specified port for face
\endverbatim

\section portsa_sec Ports Accessed
- /iha/controller/action:cmd
- /iha/status/monitor:in - status information collection port
- /iha/ds/XX/currdist:out - neighbour list from datastore XX

\section portsc_sec Ports Created
- /iha/ac/action:out - connect to the action command port of the controller
- /iha/ac/currdist:in:XX - where XX is the datastore number (see configuration file)

- /iha/ac/status:out - to write status info for monitor process
- /iha/ac/quit  - module quit port
 
\section conf_file_sec Configuration Files
conf/ihaActionSelection.ini

Sample INI file:
\verbatim
############################################################
# Actions
#
action_defs conf/iha_actiondefs.ini
sequence_dir ./conf/sequences
num_actions 21
#
############################################################


############################################################
# Main configuration settings for experience creation
HORIZONS 20
#
############################################################

############################################################
# This setting gives a max radius for the neighbour list for
# the purposes of working out relative probablities in the
# experience selection
neighbour_radius 3.0
#
############################################################

############################################################
# TEMPERATURE
# start temp
temperature 3.0
# cooling (use instead of adapt_temp)
# 0 = no cooling
temp_dec 0.01
#
############################################################

max_repeat_zero 3
\endverbatim

\section tested_os_sec Tested OS
Linux

\section example_sec Example Instantiation of the Module
ihaActionSelection --name /iha/as --file conf/ihaActionSelection.ini --connect_to_action /iha/controller/action:cmd --connect_to_dist /iha/ds/currdist:out --dbg 50

See also the script $ICUB_ROOT/app/iha_manual/iha_actionsel.sh

\see iCub::iha::ActionSelect
\see iCub::iha::Actions
\see iCub::contrib::ActionSelectionModule

\author Assif Mirza

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at \in src/interactionHistory/action_selection/src/ActionSelectionModule.cpp.
 */

ActionSelectionModule::ActionSelectionModule(){
}

ActionSelectionModule::~ActionSelectionModule(){ 
}


bool ActionSelectionModule::open(Searchable& config){
   
	if (config.check("dbg")) { IhaDebug::setLevel(config.find("dbg").asInt()); }
 	ACE_OS::fprintf(stderr, "Debug level : %d\n",IhaDebug::getLevel());

    if (config.check("help","if present, display usage message")) {
		cerr << "Usage : " << "\n"
		<< "------------------------------------------" << "\n"
		<< "  --dbg [INT]   : debug printing level" << "\n"
		<< "  --name [STR]  : process name for ports" << "\n"
		<< "  --file [STR]  : config file" << "\n"
		<< "---------------------------------------------------------------------------" << "\n"
		<< "  --action_defs [STR]       : action definitions file" << "\n"
		<< "  --sequence_dir [STR]      : directory containing sequence files" << "\n"
        << "  --connect_to_action [STR] : connect to specified port for sending actions" << "\n"
        << "  --connect_to_dist [STR]   : connect to all neighbourhood ports [STR]:hor" << "\n"
        << "  --max_repeat_zero [INT]   : maximum times to repeat action zero: 0=indefinitely" << "\n"
        << "  --test                    : test mode - send random actions" << "\n"
        << "  --temperature [FLT]       : starting temp for anealing" << "\n"
        << "  --temp_dec [FLT]          : rate at which temp reduces" << "\n"
        << "  --neighbour_radius [FLT]  : set a max distance of exps of neighbours in space" << "\n"
		<< "---------------------------------------------------------------------------" << "\n"
		<< "  [HORIZONS] <list of horizons>" << "\n"
		<< "---------------------------------------------------------------------------" << "\n"
		<< "\n";
        return false;
    }

    bool ok = true;

    test_mode=false;
	if (config.check("test")) { test_mode=true; }

	// -------------------------------------------------------------------------
	// ACTIONS
    // parameters
	max_repeat_zero = config.check("max_repeat_zero",Value(5)).asInt();
	IhaDebug::pmesg(DBGL_INFO,"max_repeat_zero %d\n",max_repeat_zero);


	// open the output port where we write action advice
    ConstString actionOutPortName = getName("action:out");
	IhaDebug::pmesg(DBGL_INFO,"Writing actions to port %s\n",actionOutPortName.c_str());
	actionOutPort.open(actionOutPortName.c_str());

	// if required we can connect 
	if (config.check("connect_to_action")) {
		// reverse connection
		if (connectToParamReverse(config,"connect_to_action",actionOutPortName.c_str(), 0.25, this)) {
            IhaDebug::pmesg(DBGL_INFO,"Connected to Action\n");
        } else {
            ok = false;
        }
	}

    // Initialize the action selector
    actionSelector.init(config);
	// get all the configured actions
	ConstString action_defs_file = config.check("action_defs",Value("action_defs.txt")).asString();

	// create the action defs object and read the actions
	// from the config file
	//Property actiondefs_props;
	//actiondefs_props.fromConfigFile(action_defs_file.c_str()); 

	//ConstString sequence_directory = config.check("equenceDir",Value(".")).asString();
	actionSelector.getActions(action_defs_file);



	// -------------------------------------------------------------------------
				

	// -------------------------------------------------------------------------
    // Distance ports : One per horizon
	Bottle& horizonsConfig =  config.findGroup("HORIZONS");
	
    horizons = new int[horizonsConfig.size()-1];
	int h=0;
	for (int i=1;i<horizonsConfig.size();i++) {
		horizons[h] = horizonsConfig.get(i).asInt();
		//------------------------------------------------------
		// create a port for reading current distances 
        char suffix[100];
		sprintf(suffix,"currdist:in:%d",horizons[h]);
        ConstString currDistPortName = getName(suffix);
		IhaDebug::pmesg(DBGL_INFO,"Reading current distances (h=%d) using port %s\n",horizons[h],currDistPortName.c_str());
		currdistPorts[horizons[h]] = new BufferedPort<Bottle>;
		currdistPorts[horizons[h]]->open(currDistPortName.c_str());
		currdistPorts[horizons[h]]->setStrict(false);

        // Also connect if required
        if (config.check("connect_to_dist")) {
            sprintf(suffix,":%d",horizons[h]);
            if (connectToParamWithSuffix(config,"connect_to_dist", suffix, currDistPortName.c_str(), 0.25, this)) {
                IhaDebug::pmesg(DBGL_INFO,"Connected to Distances h=%d\n", horizons[h]);
            } else {
                ok = false;
            }
        }

		h++;
	}
	numHorizons=h;
    IhaDebug::pmesg(DBGL_INFO,"%d Horizons configured:\n",numHorizons);
    for (h=0;h<numHorizons;h++) {
        IhaDebug::pmesg(DBGL_INFO,"Horizon #%d = %d\n",h,horizons[h]);
    }

    // declare the other dynamic arrays based on horizon length
    data_read = new bool [numHorizons];
    neighbourlist = new Bottle* [numHorizons];
	
	IhaDebug::pmesg(DBGL_STATUS1,"action_control: Sending default action\n");
    robot_busy=false;
	sendAction(0); // will also set the next behaviour set
	
	//------------------------------------------------------
    // Create a port to write status to
    ConstString statusPortName = getName("status:out");
    statusPort.open(statusPortName.c_str());

	IhaDebug::pmesg(DBGL_STATUS1,"action_control: Starting mainLoop\n");
	
    ok &= quitPort.open(getName("quit"));
    attach(quitPort, true);
    return ok;
}

bool ActionSelectionModule::close()
{
    actionOutPort.close();
	for (int h=0;h<numHorizons;h++) {
		currdistPorts[horizons[h]]->close();
    }
    delete [] horizons;
    delete [] data_read;
    delete [] neighbourlist;
    return true;
}

bool ActionSelectionModule::interruptModule()
{
    actionOutPort.interrupt();
	for (int h=0;h<numHorizons;h++) {
		currdistPorts[horizons[h]]->interrupt();
    }
    return true;
}

bool ActionSelectionModule::respond(const Bottle &command,Bottle &reply){
        
    return false;
} 	


/**
 * To write an action to the action port to be executed by icub_control
 *
 * Uses the Actions class
 */
bool ActionSelectionModule::sendAction(int act) {
	if (robot_busy) {
		ACE_OS::fprintf(stderr,"Error: sendAction called while robot_busy\n");
		return false;
	}
	bool retval=true;
	double st=Time::now();
	IhaDebug::pmesg(DBGL_STATUS2,"sendAction %d started.\n",act);
	robot_busy=true;
    // set the current and previous actions
    prev_act=current_action;
	current_action=act;
    if (act==0) {
        zero_action_count++;
    }
    else {
        zero_action_count=0;
    }

    writeStatus(statusPort,"ActSelect","Action",act);

	//IhaDebug::pmesg(DBGL_DEBUG2,"Sending Action %d=%s\n",act,action_commands[act].c_str());

	IhaDebug::pmesg(DBGL_DEBUG2,"Sending Action %d, %s\n",act,actionSelector.getActionName(act).c_str());
	Bottle bot,reply;
	bot.addInt(act);
	IhaDebug::pmesg(DBGL_DEBUG3,"Write: %s\n",bot.toString().c_str());
	if (actionOutPort.write(bot,reply) && reply=="ACK") {

		IhaDebug::pmesg(DBGL_DEBUG2,"Got ACK took %f seconds\n",Time::now()-st);

        actionSelector.updateNextBehaviourSet(act);
	} else {
		IhaDebug::pmesg(DBGL_DEBUG2,"Write complete no ACK took %f seconds\n",Time::now()-st);
		IhaDebug::pmesg(DBGL_DEBUG2,"reply = %s\n",reply==NULL?"NULL":reply.toString().c_str());
		retval=true;
	}
	//current_action=0;
	robot_busy=false;
	IhaDebug::pmesg(DBGL_STATUS2,"sendAction %d complete. %s\n",act,retval?"":"FAILED");
	//return retval;
	return true;
}


/**
 * To read one set of exp distances from the ports
 */
void ActionSelectionModule::readDistances() 
{
    IhaDebug::pmesg(DBGL_DEBUG3,"ReadDistances from %d horizons:\n",numHorizons);
    for (int h=0;h<numHorizons;h++) {
        IhaDebug::pmesg(DBGL_DEBUG3,"Horizon #%d = %d\n",h,horizons[h]);
    }
	int check_again=0; // makes sure we check all ports again if we have
					   // data on at least one port
	int got_data=0;
					   
	// read the distance data from the ports 
	while (got_data<numHorizons && check_again < 2 && !isStopping()) 
	{
		// for each horizon (seperate data stores in seperate threads)
		for (int hor=0; hor < numHorizons && !isStopping(); hor++) 
		{
			IhaDebug::pmesg(DBGL_DEBUG3,"Reading from horizon %d port\n",horizons[hor]);

			Bottle *b = currdistPorts[horizons[hor]]->read(false); // non-blocking read

			if (neighbourlist[hor]==NULL)   // we haven't received any thing for this hor yet
			{
				if (b!=NULL) {       // and we got something this time around
					neighbourlist[hor]=b;   // so save the data
					data_read[hor] = true;
					got_data++;
					IhaDebug::pmesg(DBGL_DEBUG2,"Got data hor %d exp %d: got_data=%d, check_again=%d\n",horizons[hor],b->get(1).asInt(),got_data,check_again);
				}
			} 
			else 					// we've already got something for this hor
			{
				if (b!=NULL) {		// and we have something again ??!?
					neighbourlist[hor] = b; // just overwrite with new value
					data_read[hor] = true;
					got_data++;
					IhaDebug::pmesg(DBGL_DEBUG2,"Got data hor %d (again) exp %d: got_data=%d, check_again=%d\n",horizons[hor],b->get(1).asInt(),got_data,check_again);
				}
			}

			// check for quit signal
			if (b!=NULL) {
				if (b->get(0).asInt() == -1) {
					IhaDebug::pmesg(DBGL_INFO,"Got quit\n");
					interruptModule();
                    close();
				}
            } else {			 // nothing received
                Time::delay(0.01);
			}
		}
		if (got_data>0) check_again++;
	}
    writeStatus(statusPort,"ActSelect","ExpID",neighbourlist[0]->get(1).asInt());

}

bool ActionSelectionModule::updateModule(){

	//---------------------------------------------------------------------
	// test mode - send random actions
	if (test_mode) {

		if (isStopping()) return false;

		int chosenAction;
		// choose a random action from full action set
		chosenAction = actionSelector.chooseRandom(); 
		IhaDebug::pmesg(DBGL_STATUS1,"Choosing random action %d\n",(int)chosenAction);

		//---------------------------------------------------------
		// finally, send the action to the robot
		if (!sendAction(chosenAction)) {
            fprintf(stderr,"sendAction failed INTERRUPT\n");
            interruptModule();
            close();
        }

        Time::delay(0.5);
        return true;
	}
	//---------------------------------------------------------------------

	//---------------------------------------------------------------------
    // all the time the robot is busy, just keep consuming data
    if (robot_busy) 
    {
        Time::delay(0.01);
		if (isStopping()) return false;
        IhaDebug::pmesg(DBGL_DEBUG2,"Robot busy, read again...\n");
        readDistances();
        return true;
    }
	//---------------------------------------------------------------------

    // get the neighbourlist from the ports
    readDistances();
    if (isStopping()) return false;
    // read the neighbourlist
    actionSelector.parseNeighbourList(neighbourlist);
    if (isStopping()) return false;
    // select an action
    int act = actionSelector.selectAction();
    //if (act==0 && prev_act==0) {
    IhaDebug::pmesg(DBGL_DEBUG1,"Zero count = %d\n",zero_action_count);
    if (max_repeat_zero>0 && zero_action_count > max_repeat_zero) {
        IhaDebug::pmesg(DBGL_STATUS1, "action(0) repeated %d times already - get random instead\n", max_repeat_zero);
        act = actionSelector.selectRandomAction(actionSelector.getNextBehaviourSet(0));
        zero_action_count=0;
    }
    if (isStopping()) return false;
    // and send it
    sendAction(act);

    if (isStopping()) return false;

    return true;
}

