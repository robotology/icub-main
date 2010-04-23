// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * \defgroup 
 * @ingroup emorph_lib
 *
 * Interface for the event-based camera of the emorph project.
 *
 * Author: Giorgio Metta
 * Copyright (C) 2010 RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#include <yarp/os/all.h>

using namespace yarp::os;

int main(void)
{
	Network yarp;
    return 0;
}

#if 0

#include <yarp/os/all.h>
#include <buffer2image.h>
#include <cstdlib>

using namespace yarp::os;
using namespace std;

/*
 * LATER: standardize according to iCub specs.
 * parameter handling is crappy!
 */
int main(void)
{
	Network yarp;
	std::string in;



#ifdef _DEBUG
	cout << "Create a C_yarpViewer instance" << endl;
#endif
	C_cframeConverter cFC;
#ifdef _DEBUG
	cout << "Register the instance to the callback" << endl;
#endif
	cFC.useCallback();
#ifdef _DEBUG
	cout << "Open the port for listening" << endl;
#endif
	cFC.open("/converter/in");
#ifdef _DEBUG
	cout << "Connect the 'read' and 'write port'" << endl;
#endif
	//Network::connect("/DV128/out","/converter/in");
    bool end = false;
	//cvNamedWindow("Events", 0);
    while(!end)
    {
        std::cin >> in;
        if (in=="quit")
            end=true;
    }
#ifdef _DEBUG
	cout << "Close the listening port" << endl;
#endif
	cFC.close();
#ifdef _DEBUG
	cout << "Unregister form the callback" << endl;
#endif
    cFC.disableCallback();
    return 0;
}

#endif
