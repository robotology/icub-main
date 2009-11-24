// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/**

\defgroup icub_iha_port_reader Port Reader / File Writer (IHA)

\brief A utility process that can attach to a port and display the output and/or write that output to a log file (part of the Interaction History Architecture)

\section intro_sec Description
A utility process that can attach to a port and display the output and/or write that output to a log file.

\section lib_sec Libraries
- YARP libraries.
- IHA Debug Library
- IHA File utilities

\section parameters_sec Parameters
\verbatim
  --port PORTNAME    :  Input port (required)
  [--filename FILE]  :  Output filename. If given the output will be written to it.
  [--dir PATH     ]  :  Output path. If given. Will be created if it doesnt exist.
  [--noecho       ]  :  If specified, output will not be echoed to stderr
\endverbatim

\section portsa_sec Ports Accessed

As specified in parameters

\section portsc_sec Ports Created

\verbatim
PORTNAME:portfw
\endverbatim
 
\section conf_file_sec Configuration Files
conf/ihaTemplate.ini

\section tested_os_sec Tested OS
Linux

\section example_sec Example Instantiation of the Module
\verbatim
ihaPortReader --port /iha/module/stdout 
\endverbatim

\verbatim
ihaPortReader --port /iha/module/stdout --filename /tmp/logfile --noecho
\endverbatim

\author Assif Mirza

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at \in src/interactionHistory/rw_utils/src/PortWriter.cpp.
 */

#include <stdio.h>
#include <yarp/os/all.h>
#include <iCub/iha/file_utils.h>
#include <sstream>
#include <string>

using namespace yarp::os;
using namespace iCub::iha;

BufferedPort<Bottle> dataPortIn;

bool running = true;
// simple signal handler stops out program
void term_sighandler(int sig) {
	ACE_OS::fprintf(stderr,"Got term signal\n");
	running = false;
	dataPortIn.interrupt();
}


void printUsage() {
	ACE_OS::fprintf(stdout,"An all purpose reader/filewriter for the yarp run output.\n");
	ACE_OS::fprintf(stdout,"  --port PORTNAME    :  Input port (required)\n");
	ACE_OS::fprintf(stdout,"  [--filename FILE]  :  Output filename. If given the output will be written to it.\n");
	ACE_OS::fprintf(stdout,"  [--dir PATH     ]  :  Output path. If given. Will be created if it doesnt exist.\n");
	ACE_OS::fprintf(stdout,"  [--noecho       ]  :  If specified, output will not be echoed to stderr\n");
}

std::string bottleToString(Bottle& bot) {
	std::ostringstream oss;

    //this doesn't seem to be working properly in terms of 
    //printing/recording time accurately
    //possible problems: 
    // -calc of time from gettimeofday
    // -precision of double rep in Bottle
    // -precision of double in printing

	typedef std::numeric_limits< double > dbl;
	oss.precision(dbl::digits10);
    for (int i=0; i<bot.size(); i++) {
        Value& element = bot.get(i);
        switch (element.getCode()) {
        case BOTTLE_TAG_INT:
            oss << element.asInt();
            break;
        case BOTTLE_TAG_DOUBLE:
            oss << element.asDouble();
            break;
        default:
            oss << element.asString();
            break;
		}
        oss << " ";
    }
	//oss << std::endl;

	return oss.str();
}


int main(int argc, char *argv[]) {
	signal(SIGTERM, term_sighandler); // register a SIGTERM handler

	Network::init();
	
	Property cmdLine;
	cmdLine.fromCommand(argc,argv);

	if (cmdLine.check("help") || !cmdLine.check("port")) {
		printUsage();
		exit(1);
	}
	ConstString portname = cmdLine.find("port").asString();

	bool echo = !cmdLine.check("noecho");
	
	bool filewriter=false;
	if (cmdLine.check("filename")) {
		ACE_OS::fprintf(stderr,"Writing to a file\n");
		filewriter=true;
	}

	ConstString filename = cmdLine.check("filename", Value("")).asString();
	ConstString dirpath = cmdLine.check("dir", Value(".")).asString();

	// config file read: filenames and directories
	// get the output directory basename
		
	if (filewriter) {
		if (dirpath!="." && !dirExists(dirpath)) {
		// create the directory for data output
		if (ACE_OS::mkdir(dirpath)<0) {
			ACE_OS::fprintf(stderr,"Error creating new data directory\n");
			return false;
		}
		ACE_OS::fprintf(stderr,"Created new data directory %s\n",dirpath.c_str());
		}

		ConstString fullpath_filename = ConstString( dirpath + ConstString("/") + filename );
	
		ACE_OS::fprintf(stderr,"Writing data to %s\n",fullpath_filename.c_str());
		// check if the file already exists
		if (fileExists(fullpath_filename.c_str())) {
			ACE_OS::fprintf(stderr,"Error: file %s exists\n",fullpath_filename.c_str());
			exit(1);
		}
			
		openOutputFile(fullpath_filename);
	}

	//------------------------------------------------------
	// create the input port
	char inportstr[30];
	ACE_OS::sprintf(inportstr,"%s:portfw",portname.c_str());
	// open the port
	dataPortIn.open(inportstr);
	
	// connect
	if (!Network::connect(portname,inportstr)) {
		ACE_OS::fprintf(stderr,"Error: Cannot connect %s to %s\n",inportstr, portname.c_str());
		exit(1);
	}

	Bottle *b;
	while (running) {
		b = dataPortIn.read(true);
		if (b==NULL) {running=false;continue;}

		//std::string str = bottleToString(*b).c_str();
        std::string str = bottleToString(*b);
		if (filewriter) writeToOuputFile( str.c_str() );
		if (echo) ACE_OS::fprintf(stderr,"%s",str.c_str());
	}
	
	Network::fini();
	
	if (filewriter) closeOutputFile();
	
	return 0;
}
