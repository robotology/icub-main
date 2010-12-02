// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef ICUBSIMULATION_SIMULATORMODULE_INC
#define ICUBSIMULATION_SIMULATORMODULE_INC

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

#include "SimConfig.h"

// wouldn't be better to have this in conditional compilation?
#include <iCub/RC_DIST_FB_logpolar_mapper.h>

// forward declaration
class Simulation;

class SimulatorModule : public yarp::os::PortReader {
public:
    SimulatorModule();

    std::string moduleName;
    iCub::logpolar::logpolarTransform trsf;
    bool firstpass;

    yarp::sig::ImageOf<yarp::sig::PixelRgb> buffer;

    // wrapper to logpolarTransform, taking into account initialization
    bool cartToLogPolar(yarp::sig::ImageOf<yarp::sig::PixelRgb> &lp, 
                        const yarp::sig::ImageOf<yarp::sig::PixelRgb> &cart);
    bool subsampleFovea(yarp::sig::ImageOf<yarp::sig::PixelRgb>& dst, 
                        const yarp::sig::ImageOf<yarp::sig::PixelRgb>& src);

    bool open();
    bool runModule();

	bool configure(int argc, char **argv);

    void sendTouch(yarp::os::Bottle& report);
    bool shouldSendTouch();

    void sendInertial(yarp::os::Bottle& report);
    bool shouldSendInertial();

    bool closeModule();
    bool interruptModule();

    void displayStep(int pause);

	bool read(yarp::os::ConnectionReader& connection);

    bool respond(const yarp::os::Bottle &command, 
                 yarp::os::Bottle &reply);

    void getImage();
    void sendImage(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >& port);
    void sendImageFov(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >& port);
    void sendImageLog(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >& port);


private:
    yarp::dev::PolyDriver *iCubLArm, *iCubRArm, *iCubHead, *iCubLLeg ,*iCubRLeg, *iCubTorso;
	yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > portLeft, portRight, portWide, portLeftFov, portLeftLog, portRightFov, portRightLog ;
    yarp::os::Port cmdPort;
	yarp::os::BufferedPort<yarp::os::Bottle> tactilePort;
    yarp::os::BufferedPort<yarp::os::Bottle> inertialPort;

    int _argc;
    char **_argv;
    int w, h;
    bool stopped;
    bool extractImages;
    Simulation *sim;
    void (*wrappedStep) (int pause);
    yarp::os::Semaphore mutex, pulse, ack;
    double sloth;

    void init();
    void initImagePorts();
	void initIcubParts();
	SimConfig finder;
};

#endif
