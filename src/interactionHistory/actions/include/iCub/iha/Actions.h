#ifndef _IHA__ACTION_DEFS__H__
#define _IHA__ACTION_DEFS__H__

#include <string>
#include <map>
#include <vector>
#include <yarp/os/all.h>
#include <ace/OS.h>

#include <iCub/iha/Sequence.h>
#include <iCub/iha/Expression.h>

/** \class Actions
 */

namespace iCub {
	namespace iha {
		class Actions;
	}
}

/**

\brief Actions class library (part of the Interaction History Architecture)

Provides an interface to the action description files.  These files are stored as Sequence s of motor positions in *.seq files.  The format was inherited from the Kaspar control sequence files (University of Hertfordshire) and adapted for use with the iCub.

This class provides methods to read such files into a vector of Actions classes and methods to access them.

Also provided is a mechanism to group actions into behaviour sets.  Actions can then have a "next behaviour set" defined which can be used to restrict the next possible actions following any given action.

Additionally, a new type of action the "Expression" (see the Expressions class) can be defined in a *.exp file which is used to drive the serial board expression package on the iCub hardware.

Configuration is done in an INI file pointed to from the calling module.  A path to the directory containing the sequence files also needs to be defined.

\author Assif Mirza

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at \in src/interactionHistory/actions/include/iCub/iha/Actions.h
 */

class iCub::iha::Actions 
{
public:
	Actions() : current_sequence(-1) {}
	~Actions();

	bool open(yarp::os::Property& config, yarp::os::ConstString seq_dir);

	/**
	 * Get the command list for the default behaviour set
	 */
	std::vector<std::string> getActionCommandList();
	/**
	 * Get the command list for the given behaviour set
	 */
	std::vector<std::string> getActionCommandList(int behaviour_set);
	/**
	 * Get the full command list for all actions
	 */
	std::vector<std::string> getFullActionCommandList();


	/**
	 * Play sequence by absolute (action) number
	 */
	bool playSequence(int act, int actid);
	/**
	 * Play sequence by Command 
	 */
	bool playSequence(string act, int actid);

	std::string getSequenceString(int act);
	std::string getExpressionString(int act);

	/**
	 * Get behaviour set (vector of actions) by number
	 */
	std::vector<int> getBehaviourSet(int bsetno);

	/**
	 * Get the next behaviour set number for a given action
	 */
	int getActionNextBehaviourSetNo(int act);
 
	/**
	 * Get the next behaviour set (vector of actions) for
	 * a given action.
	 */
	std::vector<int> getActionNextBehaviourSet(int act);

	void debugPrintBehaviourSet(int dbgl, int bsetno);

	std::vector< Expression > expressions; /**< Vector of expressions */

	int getNumActions() { return num_actions; };
private:

	int num_actions;

	
	std::string* action_names;/**< action names for info on screen etc.*/

	std::string* action_commands;/**< action commands max 4 chars */

	std::map<std::string, int> action_command_map;/**< A map to join action names and commands */

	std::string* ucmd;/**< URBI command for each action */

	int num_joints;
	std::string* joint_names;
	int* joint_motors;
	std::map<std::string,int> joint_name_motor_map;

	std::string* seq_fname;

	std::vector< Sequence > sequences; /**< Vector of sequences */

	int current_sequence;

	int num_behavioursets;
	int bset_default;
	std::vector< vector< int > > bsets;
	
	int* nextBSet;/**< array indexed by action of next behaviour set for any action */
};

#endif

