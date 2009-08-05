#include <yarp/os/all.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/all.h>
#include <iostream>
#include <string>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;

class Sender : public RateThread
{
private:
	BufferedPort<Bottle> *port_out;
	ConstString portName;

	unsigned int period;
	double threshold;

public:

	Sender(ConstString& name, int _period, double _threshold) : portName(name), RateThread(_period), period(_period), threshold(_threshold)
	{ }

	virtual bool threadInit()
	{
		port_out= new BufferedPort<Bottle>();
		port_out->open(portName);
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
	
		// read options from command line
		Bottle initialBottle(config.toString().c_str());

		// if autoAssociativeMemory.ini exists, then load it
		ConstString initializationFile = initialBottle.check("from",
								  Value("demoAAM.ini"),
								  "Initialization file (string)").asString();

		ConstString context = initialBottle.check("context",
						    Value("demoAAM"),
						    "Context (string)").asString();

		// create and initialize resource finder
		ResourceFinder rf;
		rf.setDefaultContext(context.c_str());
		rf.setDefaultConfigFile(initializationFile.c_str());
		rf.configure("ICUB_ROOT", 0, NULL);

		// pass configuration over to bottle
		Bottle botConfig(rf.toString().c_str());

		// parse parameters or assign default values (append to getName=="/aam")
		portOutName = botConfig.check("portThresholdOut",
		                              Value(getName("threshold:o")),
		                              "Output threshold port (string)").asString();        int period = botConfig.check("period",
                                     Value(1000),
                                     "period (milliseconds)").asInt();
        double threshold = botConfig.check("threshold",
                                           Value(0.6),
                                           "threshold (double)").asDouble();

		portOut = new Sender(portOutName, period, threshold);
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
	
	mod.setName("/aamdriver");

	return mod.runModule(argc,argv);
}
