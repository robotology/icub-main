#include <yarp/os/Network.h>
#include <yarp/os/Module.h>

#include <iCub/iha/debug.h>

#include <iCub/iha/MobileEyeModule.h>

using namespace std;
using namespace yarp::os;
using namespace iCub::contrib;
using namespace iCub::iha;

/**
 * @ingroup icub_iha2_module
 *
 * \defgroup icub_iha2_MobileEye Gaze Tracking and Visual Attention (IHA2)
 *
 * \brief Combines gaze direction input from a gaze tracker and face detection on the gaze tracker's scene image.
 *
 */

int main(int argc, char *argv[]) {

    IhaDebug::setLevel(DBGL_STATUS1);
    IhaDebug::pmesg(DBGL_INFO,"MobileEye Module\n");
    Network yarp;
    MobileEyeModule module;
    module.setName("/mobileeye"); // set default name of module
    int retval = module.runModule(argc,argv);
    fprintf(stderr,"MobileEye Module Ended\n");
    yarp::os::Time::delay(3);
    return retval;
}
