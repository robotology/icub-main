
#include <iCub/iKinFwd.h>
using namespace iKin;
//using namespace yarp;
//using namespace yarp::os;
//using namespace yarp::dev;
//using namespace yarp::sig;
//using namespace iKin;
using namespace std;


// First you need to specify the new object Arm + Stick
// by inheriting from iKinLimb and give the DH parameters
// for the links
class ICubArmDrumStick : public iKinLimb
{
protected:
	virtual void allocate(const string &_type, double drumStickLength);
public:
    // redefine constructors just in case
    ICubArmDrumStick();
    ICubArmDrumStick(const string &_type, double drumStickLength);
    ICubArmDrumStick(const ICubArmDrumStick &armDrumStick);
};