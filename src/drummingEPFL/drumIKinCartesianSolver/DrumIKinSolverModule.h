#include "ArmDrumStickSolver.h"

#include <yarp/os/RFModule.h>
using namespace yarp;

class DrumIKinSolverModule: public RFModule
{
protected:
    ArmDrumStickSolver *slv;

public:
    DrumIKinSolverModule();

    virtual bool configure(ResourceFinder &rf);

    virtual bool close();

    virtual double getPeriod();
    virtual bool   updateModule();
};
