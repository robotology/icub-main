/*
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 */


/* YARP */
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>

/* iCub */
#include <iCub/BlobDescriptorModule.h>

int main(int argc, char *argv[])
{
	/* initialize YARP network */
	Network yarp;
	if(! yarp.checkNetwork() )
		return -1; // EXIT_FAILURE
	
	/* create module */
	BlobDescriptorModule bdm;
	
	/* prepare and configure Resource Finder */
	ResourceFinder rf;
	rf.setVerbose(true);
	rf.setDefaultConfigFile("blobDescriptor.ini");  // overridden by --from parameter
	rf.setDefaultContext("blobDescriptor/conf");    // overridden by --context parameter
	rf.configure("ICUB_ROOT", argc, argv);
	
	/* runModule calls configure first and, if successful, it then runs the module */
	bdm.runModule(rf);
	return 0; // EXIT_SUCCESS
}
