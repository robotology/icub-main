
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>
#include <iCub/disparityModule.h>

using namespace yarp::os;

int main(int argc, char *argv[]) 
{
    Network::init();
    printf("Network initialization \n");
    disparityModule module;
    return module.runModule(argc,argv);
    
    Network::fini();
    
}
