#include <yarp/os/Network.h>
#include <yarp/os/Module.h>

#include <iCub/iha/debug.h>

#include <iCub/iha/MotivationDynamicsModule.h>

using namespace std;
using namespace yarp::os;
using namespace iCub::contrib;

/**
 * @ingroup icub_iha2_module
 *
 * \defgroup icub_iha2_Dynamics Motivation Dynamics (IHA2)
 *
 * \brief Motivation Dynamics for the IHA providing simple reward mapping from environment.
 *
 *
 */

int main(int argc, char *argv[]) {

    IhaDebug::setLevel(DBGL_STATUS1);
    IhaDebug::pmesg(DBGL_INFO,"Motivation Dynamics Module\n");
    Network yarp;
    MotivationDynamicsModule module;
    module.setName("/dynamics"); // set default name of module
    return module.runModule(argc,argv);
}
