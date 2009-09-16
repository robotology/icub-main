// Audit Trail 

// Changed the context path from "demoAAM" to "demoAAM/conf" as you need to specify the full sub-path to .ini file 
// David Vernon 13/8/09
//
// Changed to new convention for handling port names and module name
// (module name has no / prefix and port names _always_ have / prefix)
// David Vernon 26/08/09  

 

#include <yarp/os/all.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/all.h>
#include <iostream>
#include <string>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;

bool replaceDoubleSlash(string &str) {
   string::size_type loc;
   if ((loc=str.find("//",0))!=string::npos) {
      str.erase(loc+1,1);
      return true;
   }
   return false;
} 

class Sender : public RateThread
{
private:
	BufferedPort<Bottle> *port_out;
	ConstString portName;

	unsigned int period;
	double threshold;

public:

	Sender(const ConstString& name, int _period, double _threshold) : portName(name), RateThread(_period), period(_period), threshold(_threshold)
	{ }

	virtual bool threadInit()
	{
		port_out= new BufferedPort<Bottle>();
		port_out->open(portName.c_str());
		cout << "Starting Driver at " << period << " ms" << endl;
		return true;
	}

	virtual void afterStart(bool s)
	{
		if (s)
			cout << "Driver started successfully" << endl;
		else
			cout << "Driver did not start" << endl;
	}

	virtual void run()
	{
		Bottle &b = port_out->prepare();
		b.clear();
		b.addDouble(threshold);
		port_out->write();
	}

	virtual void threadRelease()
	{
		port_out->close();
	}
};


class AAMDemoModule: public Module
{
private:
	Sender *portOut;
	ConstString portOutName;

public:
	AAMDemoModule() { }

	virtual bool open(Searchable &config)
	{
	    string tempPortOutName;

		// read options from command line
		Bottle initialBottle(config.toString().c_str());

		// if autoAssociativeMemory.ini exists, then load it
		ConstString initializationFile = initialBottle.check("from",
								  Value("demoAAM.ini"),
								  "Initialization file (string)").asString();

		ConstString context = initialBottle.check("context",
						    Value("demoAAM/conf"),           // need to specify the full sub-path to ini file DV 13/8/09
						    "Context (string)").asString();

		// create and initialize resource finder
		ResourceFinder rf;
		rf.setDefaultContext(context.c_str());
		rf.setDefaultConfigFile(initializationFile.c_str());
		rf.configure("ICUB_ROOT", 0, NULL);

		// pass configuration over to bottle
		Bottle botConfig(rf.toString().c_str());

		// parse parameters or assign default values (append to getName=="/aam")
		tempPortOutName = "/";
        tempPortOutName += botConfig.check("portThresholdOut",
		                              Value(getName("/threshold:o")),
		                              "Output threshold port (string)").asString();
        replaceDoubleSlash(tempPortOutName);     // temporary fix until behaviour of getName() is changed
        portOutName = tempPortOutName.c_str();

		int period = botConfig.check("period",
                                     Value(1000),
                                     "period (milliseconds)").asInt();
        double threshold = botConfig.check("threshold",
                                           Value(0.6),
                                           "threshold (double)").asDouble();

		portOut = new Sender((ConstString)portOutName, period, threshold);
		portOut->start();

		return true;
	}

	virtual bool close()
	{
		delete portOut;
		return true;
	}

	virtual bool   updateModule() { return true; }
};


int main(int argc, char *argv[])
{
	Network yarp;

	if (!yarp.checkNetwork())
		return false;

	AAMDemoModule mod;
	
	mod.setName("aamdriver");

	return mod.runModule(argc,argv);
}

