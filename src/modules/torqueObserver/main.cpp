/* 
 * Copyright (C) <2010> RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author Serena Ivaldi
 * email:   serena.ivaldi@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <iostream>
#include "torqueObserver.h"

using namespace std;
using namespace yarp::os;

int main(int argc, char * argv[])
{
    //initialize yarp network
    Network yarp;
	
    //create your module
    TorqueObserver tauObs;

    // prepare and configure the resource finder
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure("ICUB_ROOT", argc, argv);

	if( tauObs.configure(rf) )
		cout<<"TorqueObserver configured correctly!"<<endl;
	else
	{
		cout<<"TorqueObserver is not properly configured: exiting.."<<endl;
		return 0;
	}

	cout<<"Start module TorqueObserver..."<<endl;
	
	//running module: if configure() succeeds, module runs
	tauObs.runModule();
	
    cout<<"TorqueObserver has been closed!"<<endl;
    return 1;
}

	
	

		
