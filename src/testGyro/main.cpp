#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Property.h>
#include <yarp/os/Time.h>

#include <yarp/dev/GenericSensorInterfaces.h>

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

#include <yarp/os/Module.h>

class MyModule: public Module
{
    PolyDriver *dd;
    IGenericSensor *inertial;
    int nch;

public:
    MyModule():Module()
    {
        dd=0;
        inertial=0;
    }

    ~MyModule()
    {
        if (dd!=0)
            delete dd;
    }
    
    virtual double getPeriod()
    {
        return 0.01;
    }

    virtual int runModule(int argc, char *argv[])
    {
        dd=new PolyDriver();
        Property options;
    
        options.fromCommand(argc, argv);
        // this is a test for icub so I assume you 
        // want to open the xsensmtx inertial device
        if (!options.check("device"))
            options.put("device", "xsensmtx");
        bool ret=dd->open(options);

        if (!ret)
            {
                fprintf(stderr, "Failed to open device\n");
                return 0;
            }

        dd->view(inertial);

        nch=10;
        inertial->getChannels(&nch);

        fprintf(stderr, "Number of channels %d\n", nch);

        return Module::runModule();
    }

    virtual bool updateModule()
    {
        Vector data(nch);

        data=0.0;

        inertial->read(data);
	static int c=0;
	c++;

	if (c%100==0)
            fprintf(stderr, "(%s)\n", data.toString().c_str());

        return true;
    }


};

int main(int argc, char *argv[])
{
    MyModule mod;
    if (argc<2)
        {
            printf("Usage: %s --device DEVICENAME --serial SERIALPORT\n", argv[0]);
            printf("DEVICENAME: name of the device (default: 'xsensmtx')\n");
            printf("SERIALPORT: serial port name or number\n");
            printf("For example:\n%s --serial /dev/ttyS0\n", argv[0]);
            printf("On windows SERIALPORT is an integer, which identifies the COM port\n");
            return 0;
        }

    mod.attachTerminal();
    return mod.runModule(argc, argv);
}
