#include <iCub/iKinSlv.h>
using namespace iKin;

#include <yarp/os/all.h>
using namespace yarp::os;

#include <string>
using namespace std;

class ArmDrumStickSolver : public ArmCartesianSolver
{
protected:
    // we're required just to instantiate the iCubArmDrumStick object
    virtual PartDescriptor *getPartDesc(Searchable &options);
    
public:
    ArmDrumStickSolver(const string &_slvName="ArmDrumStickSolver") : ArmCartesianSolver(_slvName) { }
};
