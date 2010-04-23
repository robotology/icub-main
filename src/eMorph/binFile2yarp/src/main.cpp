// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * \defgroup 
 * @ingroup emorph_lib
 *
 * Interface for the event-based camera of the emorph project.
 *
 * Author: Charles Clercq, Giorgio Metta
 * Copyright (C) 2010 RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#include <yarp/os/all.h>
#include <binFile2yarp.h>
#include <cstdlib>

using namespace yarp::os;
using namespace std;

/*
 * LATER: standardize according to iCub specs.
 * parameter handling is crappy!
 */
int main(int argc, char* argv[])
{
	Network yarp;
	std::string in;
	bool end = false;
	string fileName;
    if(argc > 1)
        fileName = argv[1];
    else
        fileName = "./raw_events.bin";
	binfile2yarp bF2Y(fileName);
	bF2Y.start();
    while(!end)
    {
        std::cin >> in;
        if (in=="quit")
            end=true;
    }
	bF2Y.stop();
    return 0;
}
