
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <iCub/disparityModule.h>

#include <string.h>

using namespace yarp::os;
using namespace std;

int main(int argc, char *argv[]) 
{
    /* initialize yarp network */ 
   Network yarp;

   /* create your module */

   disparityModule module; 

   /* prepare and configure the resource finder */

   ResourceFinder rf;
   rf.setVerbose(true);
   rf.setDefaultConfigFile("demo.ini"); //overridden by --from parameter
   rf.setDefaultContext("demo/conf");   //overridden by --context parameter
   rf.configure("ICUB_ROOT", argc, argv);
 
   /* run the module: runModule() calls configure first and, if successful, it then runs */

   module.runModule(rf);

   return 0;}
