#ifndef _IHA_SEQUENCE__H__
#define _IHA_SEQUENCE__H__

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <map>
#include <vector>
#include <fstream>
#include <sstream>

#include <ace/OS.h>

#include <iCub/iha/debug.h>

/*! \def SEQTYPE_JOINT_POSITION
 * A motor position type for a SequenceLine */
#define SEQTYPE_JOINT_POSITION 0
/*! \def SEQTYPE_BLOCK
 * A BLOCK type for a SequenceLine */
#define SEQTYPE_BLOCK 1

namespace iCub {
	namespace iha {
		class SequenceLine;
		class Sequence;
	}
}

using namespace std;

/** 
 *
 * \brief A single line of a Sequence describing a single Actions
 *
 * A sequence line can be a motor position command or a BLOCK statement.
\author Assif Mirza

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at \in src/interactionHistory/actions/include/iCub/iha/Actions.h
 */

class iCub::iha::SequenceLine {
	public:
		int type; /**< The type can be SEQTYPE_JOINT_POSITION or SEQTYPE_BLOCK */
		std::string joint_name; /**< Joint name defined in the Actions ini file (default name iha_actiondefs.ini) */
		int value; /**< position */
		int speed; /**< speed to move to position. A reasonable value would be between 5 and 30 for the iCub*/
		SequenceLine(){}
		/**
		 * Costructor taking type, joint name, position and speed
		 */
		SequenceLine(int t, std::string jn, int v, int s) : type(t), joint_name(jn), value(v), speed(s){}
		void print() {
			if (type==SEQTYPE_BLOCK) {
				IhaDebug::pmesg(DBGL_DEBUG2,"BLOCK\n");
			} else {
				IhaDebug::pmesg(DBGL_DEBUG2,"%s %d %d\n",joint_name.c_str(),value,speed);
			}
		}
};
	
/** 
 * \brief A sequence of motor positions
 *
 * This class contins methods to store sequences of motor positions that are subsequently used as Actions in the Interaction History Architecture.
 * 
 * Each Sequence consits of a SequenceLine which holds either a position command or a BLOCK command.  Consecutive position commands up to any BLOVK commandare executed in parallel. The sequence should wait at any BLOCK command till the previous positions are reached by the robot before sending the next consecutive position commands.
 *
\author Assif Mirza

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at \in src/interactionHistory/actions/include/iCub/iha/Actions.h
 */

class iCub::iha::Sequence {
public:
	std::string name; /**< The name of this sequence */
	std::vector < SequenceLine > sequence; /**< This sequence as a series of SequenceLine objects. */

	/**
	 * read sequence from file
	 */
	bool read(string fname) {
		ifstream ifs(fname.c_str());
		std::string line;
		istringstream iss;
		std::string _joint_name;
		int _value;
		int _speed;
		std::string _speedstr;

		// first line contains only name and key associations
		if (!getline(ifs,line)) {
			ACE_OS::fprintf(stderr,"Error: empty file\n");
			return false;
		}

		// now sequence starts
		while( getline(ifs,line) ) {
			char* token = NULL;
			token = strtok((char*)line.c_str(),",");

			// skip empty lines
			if (token==NULL) continue;
			// and comments
			if (strncmp(token,"%",1)==0) continue;
			
			if (strncmp(token,"BLOCK",5)==0) {
				IhaDebug::pmesg(DBGL_DEBUG2,"sequence item: BLOCK\n");
				SequenceLine ksl;
				ksl.type=SEQTYPE_BLOCK;
				sequence.push_back(ksl);
				continue;
			}
			_joint_name = string(token);
			token = strtok(NULL,",");
			if (token==NULL) {
				IhaDebug::pmesg(DBGL_INFO,"Error in file\n");
				return false;
			}
			_value = atoi(token);
			token = strtok(NULL,",");
			if (token==NULL) {
				IhaDebug::pmesg(DBGL_INFO,"Error in file\n");
				return false;
			}
			if (strcmp(token,"defaultSpeed")==0) {
				_speed=10;
			} else {
				_speed=atoi(token);
			}

			IhaDebug::pmesg(DBGL_DEBUG2,"read: %s %d %d\n",_joint_name.c_str(),_value,_speed); 

			SequenceLine ksl(SEQTYPE_JOINT_POSITION,_joint_name,_value,_speed);
			sequence.push_back(ksl);
		}
		
	}
	void print() {
		for (vector<SequenceLine>::iterator it=sequence.begin();it!=sequence.end();it++) {
			it->print();
		}
	}
};


#endif
