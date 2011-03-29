/*
 * A bottom-up approach for generating particles for the "pf3dTracker" module
 *
 * Copyright (C) 2010 RobotCub Consortium
 *
 * Author: Martim Brandao
 * Note: Should you use or reference my work on your own research, please let me know (mbrandao _AT_ isr.ist.utl.pt)
 *
 * Image sequence as input, N particles (3D position of balls) as output.
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

// yarp
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>

// iCub
#include <iCub/pf3dBottomup.hpp>

using namespace std;
using namespace yarp::os;

int main(int argc, char *argv[])
{
	Network yarp;
	pf3dBottomup bottomup;
	bottomup.setName("/pf3dBottomup");
	return bottomup.runModule(argc,argv);
}

