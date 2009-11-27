// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-




/**
*
@ingroup icub_module
\defgroup icub_openCVLogPolar openCVLogPolar

This module is a wrapper of the opencv logpolar transform; it extracts the log polar image 
from an input image.
It can produce a fake output image composed of coloured circles

\section intro_sec Description

The module does:
-	creates a logpolar image starting from the input image
-	regenerate the original image from the logPolar image
-	creates a fake image composed of coloured circles

\section lib_sec Libraries
YARP 
OPENCV

\section parameters_sec Parameters
Here is a  comprehensive list of the parameters you can pass to the module.

--name (string) name of the module. The name of all the ports will be istantiated starting from this name 
--mode (string) defines the modality with which the module will produce the output: 
			[SIMULATION: produce a fake image, FORWARD: from image to logPolar, REVERSE: from logPolar to image]
 
\section portsa_sec Ports Accessed
none

\section portsc_sec Ports Created
<name>/image:i
<name>/image:o
<name>/simulation:o
<name>/inverse:o

Output ports:
- <name>/image:o: streams out a yarp::sig::ImageOf<PixelRgb> which is the result of the processing (either FORWARD or REVERSE)
- <name>/inverse:o:  streams out a yarp::sig::Image<PixelRgb> which is the reverse logPolar of the image in out (only FORWARD mode)
- <name>/simulation:o:  streams out a yarp::sig::Image<PixelRgb> which is composed of coloured circles

Input ports:
- <name>/image:i: input ports which takes as input a yarp::sig::ImageOf<PixelRgb>

\section in_files_sec Input Data Files
none

\section out_data_sec Output Data Files
none
 
\section conf_file_sec Configuration Files
none

\section tested_os_sec Tested OS
Linux and Windows.

\section example_sec Example Instantiation of the Module
openCVLogPolar --name /LogPolar/ --mode FORWARD

\author Francesco Rea

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

**/

#include <yarp/os/all.h>
#include <iCub/LogPolarModule.h>

int main(int argc, char *argv[]) {
	
	
    //initialise Yarp Network
	Network yarp;
	// Create and run our module
	LogPolarModule module;

	// Get command line options
	Property options;
	if(argc<2){
		//there are no parameters
		// litte help on how to use
		printf("______ HELP ________ \n");
		printf(" \n");
		printf("USER COMMANDS: \n");
		printf("--mode (SIMULATION,FORWARD,INVERSE): selects what this module operates \n");
		printf("--name (XXXX): defines the name of this module \n");
		printf(" \n");
		printf(" \n");
		//start of the default mode
		printf("No commands, starting of the default mode ................... \n");
		options.put("mode","FORWARD");
		options.put("name","/rea/logPolar");
	}
	else{
		//estracts the command from command line
		options.fromCommand(argc,argv);
	}
	module.setOptions(options);

    return module.runModule(argc,argv);

    //MyModule module;
    //module.setName("/default/name");
    //return module.runModule(argc,argv);
}
