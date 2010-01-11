#include <yarp/os/Network.h>
#include <yarp/os/Module.h>

#include <iCub/iha/debug.h>

#include <iCub/iha/AudioAnalyserModule.h>

using namespace std;
using namespace yarp::os;

/**
 * @ingroup icub_iha2_module
 *
 * \defgroup icub_iha2_AudioAnalyser Audio Analyser Module (IHA2)
 *
 * \brief Process that analyses sound input and reports drumbeats at a fixed rate
 *
 *
 */


int main(int argc, char *argv[]) {

    IhaDebug::setLevel(DBGL_STATUS1);
    IhaDebug::pmesg(DBGL_INFO,"Audio Analyser Module\n");
    Network yarp;
    AudioAnalyserModule module;
    module.setName("/audio_analyser"); // set default name of module
    return module.runModule(argc,argv);
}
