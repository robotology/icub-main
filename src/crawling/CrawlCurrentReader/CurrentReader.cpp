#include <iostream>
#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>

#include <math.h>

using namespace std;
using namespace yarp::os;

#include <fstream>
using namespace std;

#define MANAGER_PORT "/manager/in"
#define CURRENT_PORT "/icub/controlBoardDumper/torso/getCurrents"
#define MAX_CURRENT 20000

#define CURRENT_FILE_NAME "currents.dat"

#define INIT_POS_COMMAND 1


class CurrentReader:public RFModule
{
    BufferedPort<Bottle> currentPort; //a port to handle messages
    BufferedPort<Bottle> managerPort; //a port to handle messages
	ofstream currentFile;

public:

    double getPeriod()
    {
          //module periodicity (seconds), called implicitly by the module.
        return 0.2;
    }

    // This is our main function. Will be called periodically every getPeriod() seconds.
    bool updateModule()
    {
		Bottle *currentBottle = currentPort.read();
		bool exceeded = false;
		cout << currentBottle->toString() << endl;
		for(int i=0; i<currentBottle->size(); ++i)
		{
			double current = fabs(currentBottle->get(i).asDouble());
			currentFile << current << "\t";
			if(current > MAX_CURRENT)
			{
				exceeded = true;
			}
		}
		currentFile << endl;
		if(exceeded)
		{
			Bottle cmd;
			cmd.addInt(INIT_POS_COMMAND);
			managerPort.write(INIT_POS_COMMAND);
			cout << "EXCEEDED !!!" << endl;
		}
        return true;
    }

       

    
    // Configure function. Receive a previously initialized
    // resource finder object. Use it to configure your module.
    // If you are migrating from the old module, this is the function
    // equivalent to the "open" method.
    bool configure(yarp::os::ResourceFinder &rf)
    {
        currentPort.open("/CrawCurrentReader/in");  
		managerPort.open("/CrawCurrentReader/out");

		Network::connect(CURRENT_PORT, "/CrawCurrentReader/in");
		Network::connect( "/CrawCurrentReader/in", MANAGER_PORT);

		currentFile.open(CURRENT_FILE_NAME);
        return true;
    }

    // Interrupt function.
    bool interruptModule()
    {
        currentPort.close();
		managerPort.close();
        return true;
    }

    //
    // Close function, to perform cleanup.
    bool close()
    {
        //optional, close port explicitly
        currentPort.close();
		managerPort.close();
        return true;
    }
};


int main(int argc, char * argv[])
{
    //initialize yarp network
    Network yarp;

    //create your module
    CurrentReader currentReader; 

    // prepare and configure the resource finder
    ResourceFinder rf;
    rf.configure("ICUB_ROOT", argc, argv);
    rf.setVerbose(true);
 
    currentReader.configure(rf);
    currentReader.runModule();

    return 0;
}