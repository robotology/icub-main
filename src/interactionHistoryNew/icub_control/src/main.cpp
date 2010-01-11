#include <yarp/os/Network.h>
#include <yarp/os/Module.h>

#include <iCub/iha/debug.h>

#include <iCub/iha/IcubControlModule.h>

using namespace std;
using namespace yarp::os;
using namespace iCub::contrib;

/**
 * @ingroup icub_iha2_module
 *
 * \defgroup icub_iha2_IcubControl iCub Control Module (IHA2)
 *
 * \brief Process that controls the robot using the Position interface and outputs encoder values
 *
 *
 */

int main(int argc, char *argv[]) {

    IhaDebug::setLevel(DBGL_STATUS1);
    IhaDebug::pmesg(DBGL_INFO,"Icub Control Module\n");
    Network yarp;
    IcubControlModule module;
    module.setName("/my"); // set default name of module
    return module.runModule(argc,argv);
}
