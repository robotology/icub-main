#ifndef DRIVER_H
#define DRIVER_H

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CanBusInterface.h>
#include <yarp/os/Searchable.h>

#define MAX_READ_MSG 64

class cDriver
{
private:
    yarp::dev::PolyDriver dd;
    yarp::dev::ICanBus *iCanBus;
    yarp::dev::ICanBufferFactory *iFactory;
public:
    cDriver ();
    ~cDriver () {}
    int init(yarp::os::Searchable &config);
    int uninit();
    int receive_message(yarp::dev::CanBuffer &messages, int howMany = MAX_READ_MSG, double TIMEOUT = 1);
    int send_message(yarp::dev::CanBuffer &message, int n);

    yarp::dev::CanBuffer createBuffer(int m);
    void destroyBuffer(yarp::dev::CanBuffer &buff);
};

#endif
