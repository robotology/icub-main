#ifndef _IHA_EXPRESSION__H__
#define _IHA_EXPRESSION__H__

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <map>
#include <vector>
#include <fstream>
#include <sstream>

#include <ace/OS.h>

#include <iCub/iha/debug.h>

/** \class  Expression
 */

namespace iCub {
	namespace iha {
		class Expression;
	}
}

using namespace std;

/**
 * \brief Class to hold the description of a single expression.
 * Hold a single expression consisting of values for the Left/Rught Eyebrows, Mouth and Eyelid positions.
 *
 * Specific to the iCub serial interface expression hardware.
 *
 * For more information on how to set the expression values see the VIKI page \link http://eris.liralab.it/wiki/VVV08/face_expressions Face Expressions \endlink
\author Assif Mirza

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at \in src/interactionHistory/actions/include/iCub/iha/Expressions.h
 */
class iCub::iha::Expression {
	public:
		std::string part1; /**< Left Eyebrow */
		std::string part2; /**< Right Eyebrow */
		std::string part3; /**< Mouth */
		std::string part4; /**< Eyelids */
		Expression(){}
		void print() {
			IhaDebug::pmesg(DBGL_DEBUG2,"%s %s %s\n",part1.c_str(),part2.c_str(),part3.c_str(),part4.c_str());
		}
	
	/**
	 * Read expression from file
	 *
	 * Example of a file (Frown.exp)
	 * \code
	 * L04 R04 M0B S3B
	 * \endcode
	 *
	 * \param fname filename
	 * \return success
	 */
	bool read(string fname) {
		ifstream ifs(fname.c_str());
		std::string line;
		istringstream iss;

		// Only expecting one line

		char* token = NULL;
		while( getline(ifs,line) ) {
			token = strtok((char*)line.c_str()," ");

			// skip empty lines
			if (token==NULL) continue;
			// and comments
			if (strncmp(token,"%",1)==0) continue;
			break;
		}
			
		part1 = string(token);
		if (token==NULL) {
			IhaDebug::pmesg(DBGL_INFO,"Error in file\n");
			return false;
		}
		fprintf(stderr,"%s\n",part1.c_str());

		token = strtok(NULL," ");
		part2 = string(token);
		if (token==NULL) {
			IhaDebug::pmesg(DBGL_INFO,"Error in file\n");
			return false;
		}
		fprintf(stderr,"%s\n",part2.c_str());

		token = strtok(NULL," ");
		part3 = string(token);
		if (token==NULL) {
			IhaDebug::pmesg(DBGL_INFO,"Error in file\n");
			return false;
		}
		fprintf(stderr,"%s\n",part3.c_str());

		token = strtok(NULL," ");
		part4 = string(token);
		if (token==NULL) {
			IhaDebug::pmesg(DBGL_INFO,"Error in file\n");
			return false;
		}
		fprintf(stderr,"%s\n",part4.c_str());

		IhaDebug::pmesg(DBGL_DEBUG2,"read: %s %s %s %s\n",part1.c_str(),part2.c_str(),part3.c_str(),part4.c_str()); 
		
	}
};


#endif
