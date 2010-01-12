#include <yarp/os/Network.h>
#include <yarp/os/Module.h>

#include <iCub/iha/debug.h>

#include <iCub/iha/SensorMotorInterfaceModule.h>

using namespace std;
using namespace yarp::os;
using namespace iCub::contrib;
using namespace iCub::iha;

/**
 * @ingroup icub_iha2_module
 *
 * \defgroup icub_iha2_SensorMotorInterface Sensor Motor Interface (IHA2)
 *
 * \brief Provides a single sensor feed for the interaction history by collecting encoder values, images, sound, face detection, gaze detection, and action from various processes. 
 *
 */

int main(int argc, char *argv[]) {

    IhaDebug::setLevel(DBGL_STATUS1);
    IhaDebug::pmesg(DBGL_INFO,"Sensor Motor Interface Module\n");
    Network yarp;
    SensorMotorInterfaceModule module;
    module.setName("/sensor_motor_interface"); // set default name of module
    int retval = module.runModule(argc,argv);
    fprintf(stderr,"SensorMotorInterfaceModule Module Ended\n");
    yarp::os::Time::delay(3);
    fprintf(stderr,"Ciao!\n");
    return retval;
}
