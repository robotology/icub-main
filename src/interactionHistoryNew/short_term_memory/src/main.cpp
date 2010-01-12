#include <yarp/os/Network.h>
#include <yarp/os/Module.h>

#include <iCub/iha/debug.h>

#include <iCub/iha/ShortTermMemoryModule.h>

using namespace std;
using namespace yarp::os;
using namespace iCub::contrib;
using namespace iCub::iha;

/**
@ingroup icub_iha2_module

\defgroup icub_iha2_Memory Short Term Memory (IHA2)

\brief Takes current sensor data, stores it for a user specified window of recent history, 
and computes task-specific scores (used to assign reward) based on this history

 */

int main(int argc, char *argv[]) {

    IhaDebug::setLevel(DBGL_STATUS1);
    IhaDebug::pmesg(DBGL_INFO,"Short Term Memory Module\n");
    Network yarp;
    ShortTermMemoryModule module;
    module.setName("/short_term_memory"); // set default name of module
    return module.runModule(argc,argv);
}
