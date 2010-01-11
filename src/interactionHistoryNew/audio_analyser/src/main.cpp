#include <yarp/os/Network.h>
#include <yarp/os/Module.h>

#include <iCub/iha/debug.h>

#include <iCub/iha2/AudioAnalyserModule.h>

using namespace std;
using namespace yarp::os;


int main(int argc, char *argv[]) {

    IhaDebug::setLevel(DBGL_STATUS1);
    IhaDebug::pmesg(DBGL_INFO,"Audio Analyser Module\n");
    Network yarp;
    AudioAnalyserModule module;
    module.setName("/audio_analyser"); // set default name of module
    return module.runModule(argc,argv);
}
