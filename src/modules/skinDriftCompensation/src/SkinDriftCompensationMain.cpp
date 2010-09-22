#include "iCub/SkinDriftCompensation.h" 

int main(int argc, char * argv[])
{
   /* initialize yarp network */ 
   Network yarp;

   /* create your module */
   SkinDriftCompensation module; 

   /* prepare and configure the resource finder */
   ResourceFinder rf;
   rf.setVerbose(true);
   rf.setDefaultConfigFile("SkinDriftCompensationLeft.ini");		//overridden by --from parameter
   rf.setDefaultContext("graspingDemo/conf");						//overridden by --context parameter
   rf.configure("ICUB_ROOT", argc, argv);
 
   /* run the module: runModule() calls configure first and, if successful, it then runs */
   module.runModule(rf);

   return 0;
}
