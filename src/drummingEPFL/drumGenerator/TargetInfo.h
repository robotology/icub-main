#ifndef TARGETINFO_H
#define TARGETINFO_H

#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/os/RateThread.h>

#include <stdio.h>
#include <string>
#include <vector>

using namespace yarp::os;
using namespace yarp::dev;
using namespace std;

// This class handles the information concerning the target angles
// needed to reach the different target drums 

class TargetInfo
{
	public:
	
		TargetInfo(int dofs, int size); //size is the number of drums
		~TargetInfo();
			
		bool Initialize(Property fromFile); //get the initial values from file
		bool UpdateInfo(); //get the new values from the IK module
				
		int nbStates; // number of drums + idle state
		int nbDOFs;
		BufferedPort<Bottle> ik_port;
		
		vector < vector <double> > drumsPos; //corresponding target angles 
		vector<int> drumIDs; //id of the markers used by the IK
		vector<int> muOn, muOff;
};

#endif
