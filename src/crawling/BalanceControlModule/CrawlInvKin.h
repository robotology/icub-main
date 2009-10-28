#ifndef CRAWL_INV_KIN__H
#define CRAWL_INV_KIN__H

#include <ace/OS.h>

#include <yarp/os/Network.h>
#include <yarp/os/Module.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <iostream>
#include <iomanip>
#include <string>

#include <iCub/iKinIpOpt.h>
#include <iCub/iKinInv.h>

#define SHOULDER_MAXABDUCTION   (100.0*(M_PI/180.0))


using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace iKin;

// FROM UGO's code: work with crawling?
// This inherited class handles the constraints to be 
// enforced on iCub shoulders joints to protect cables.
class iCubShoulderConstr : public iKinLinIneqConstr
{
public:

    iCubShoulderConstr(unsigned int dofs, double lower_bound_inf,
                       double upper_bound_inf);
};


//class for the IK (that will be FK for now)
class IKManager
{
    private:
    const string partName;

    
    protected:
    iCubArm *leftArm, *rightArm;
    iCubLeg *leftLeg, *rightLeg;   
    iKinChain *chain;
        
    
    public: 

    double rate;
    
    vector <vector <double> > mvt_parameters;
    vector<double> param_distance;
         
    IKManager();
    ~IKManager();

    double dShoulder, dHip, DShoulder, DHip, legLength;
    
    double getArmAmplitude(double* positions, double leg_amplitude); 
    double getTurnParams(double turn_angle, double amplitude, int side, int limb);   
};


#endif
