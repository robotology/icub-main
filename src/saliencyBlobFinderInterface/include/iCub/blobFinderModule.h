// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#ifndef _BLOBFINDERMODULE_H_
#define _BLOBFINDERMODULE_H_

#include <ace/config.h>

//within project includes
#include <iCub/WatershedModule.h>

//YARP include
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;

/**
*
@ingroup icub_module
\defgroup icub_saliencyBlobFinderInterface saliencyBlobFinderInterface

Module graphical interface for saliencyBlobFinder

\section intro_sec Description
This module is purely an interface towards the module saliencyBlobFinder. It sends commands via /command:o port as bottles.
The protocol respects the directives that the saliencyBlobFinder requires.
In addition this interface is able to draw an image. This useful functionality allows the user to visualise straigh away important information 
and result of his/her interaction

The module does:
-   stream the command to the saliencyBlobFinder module
-   visualise an image in the graphics interface


\image html saliencyBlobFinder.png

\section lib_sec Libraries
YARP
GTK

\section parameters_sec Parameters
--name : name of the module and relative port name

 
\section portsa_sec Ports Accessed
/blobFinder/cmd


\section portsc_sec Ports Created
Input ports:
none
Outports
- /watershed/command:o

\section in_files_sec Input Data Files
none

\section out_data_sec Output Data Files
none
 
\section conf_file_sec Configuration Files
none


\section tested_os_sec Tested OS
Linux and Windows.

\section example_sec Example Instantiation of the Module
saliencyBlobFinderInterface 


\author Francesco Rea

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

**/

class blobFinderModule : public Module {
private:
    /**
	* port where the commands are sent
	*/
    Port cmdPort;
	/**
	* options of the connection
	*/
	Property options;	//

public:
    //------------------ PUBLIC METHODS -----------------------------------

	/**
	* default constructor
	*/
	blobFinderModule();
    /**
	* default destructor
	*/
    ~blobFinderModule();
	/**
	* open and initialise the module
	*/
	bool open(Searchable& config); //
	/**
	* try to interrupt any communications or resource usage
	*/
    bool interruptModule(); // 
	/**
	* closes all the ports 
	*/
	bool close(); //
	/**
	* active control of the Module
	*/
	bool updateModule();
    
    /**
	* set the attribute options of class Property
	*/
	void setOptions(Property options); //
    
    ///----------- public attributes -------------
    WatershedModule* gui;

};

#endif //_BLOBFINDERMODULE_H_


//----- end-of-file --- ( next line intentionally left blank ) ------------------
