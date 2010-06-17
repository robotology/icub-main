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
@ingroup icub_guis
\defgroup saliencyBlobFinderInterface saliencyBlobFinderInterface

Module graphical interface for saliencyBlobFinder

\section intro_sec Description
This module is purely an interface towards the module saliencyBlobFinder. It sends commands via /command:o port.
The protocol respects the communication directives that the saliencyBlobFinder requires.
In addition this interface is able to draw an image. This useful functionality allows the user to visualise straigh away important information 
and result of his/her interaction

The module does:
-   stream commands to the saliencyBlobFinder module
-   visualise an image in the graphics interface

\image html saliencyBlobFinder.png

In the GUI there are some sliding controls on the left hand side which are in sequence:
<ul>
    <li>coefficients for the algorithm that defines the saliency of the blob </li>
    <li>min and max dimension of the blobs which will be analysed</li>
    <li>controls for the selection of the colour which the system has to look for</li>
</ul>

On the right side of the GUI there is a series of checkbox which select the typology of the output sent on the /image:o port.
The last checkbox that toggle from OFF to ON selects the output. It can be choosen between this possible alternatives:
<ul>
<li> tagged: the image composed by all the blobs filled with a gray scale equal to their list position</li>
<li> watershed: the result of the watershed operation</li>
<li> foveaBlob: the representation of the blob foveated in that moment</li>
<li> colourVQ: image composed by all the blobs filled with quantization colour</li>
<li> meanColour: the composition of all the colour filled with the mean colour of all the pixel which belong to the blob</li>
<li> maxSaliencyBlob: the representation of the max saliency blob</li>
</ul>



\section lib_sec Libraries
YARP
GTK

\section parameters_sec Parameters
--name : name of the module and relative port name

 
\section portsa_sec Ports Accessed
/blobFinder/cmd


\section portsc_sec Ports Created
Input ports:
- <name>/cmd
Outports:
- <name>/command:o

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
    BufferedPort<Bottle > cmdPort;
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
