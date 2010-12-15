// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Vadim Tikhanoff, Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */


#ifndef ICUBSIMULATION_SIMULATORMODULE_INC
#define ICUBSIMULATION_SIMULATORMODULE_INC

#include <yarp/os/Bottle.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/Image.h>
#include <yarp/dev/PolyDriver.h>

#include "RobotStreamer.h"
#include "RobotConfig.h"
#include "WorldManager.h"
#include "Simulation.h"

// wouldn't be better to have this in conditional compilation?
#include <iCub/RC_DIST_FB_logpolar_mapper.h>

class SimulatorModule : public yarp::os::PortReader, public RobotStreamer
{
public:
    SimulatorModule(RobotConfig& config, Simulation *sim);

    bool open();
    bool runModule();

    bool closeModule();
    bool interruptModule();

	bool read(yarp::os::ConnectionReader& connection);

    bool respond(const yarp::os::Bottle &command, 
                 yarp::os::Bottle &reply);

    // RobotStreamer interface

    virtual void sendVision();

    virtual void sendTouchLeft(yarp::os::Bottle& report);
    virtual void sendTouchRight(yarp::os::Bottle& report);

    virtual bool shouldSendTouchLeft();
    virtual bool shouldSendTouchRight();

    virtual void sendInertial(yarp::os::Bottle& report);
    virtual bool shouldSendInertial();

private:
    // wrapper to logpolarTransform, taking into account initialization
    bool cartToLogPolar(yarp::sig::ImageOf<yarp::sig::PixelRgb> &lp, 
                        const yarp::sig::ImageOf<yarp::sig::PixelRgb> &cart);
    bool subsampleFovea(yarp::sig::ImageOf<yarp::sig::PixelRgb>& dst, 
                        const yarp::sig::ImageOf<yarp::sig::PixelRgb>& src);


    void displayStep(int pause);

    void getImage();
    void sendImage(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >& port);
    void sendImageFov(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >& port);
    void sendImageLog(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >& port);

    std::string moduleName;
    iCub::logpolar::logpolarTransform trsf;
    bool firstpass;

    yarp::sig::ImageOf<yarp::sig::PixelRgb> buffer;

    yarp::dev::PolyDriver *iCubLArm, *iCubRArm, *iCubHead, *iCubLLeg ,*iCubRLeg, *iCubTorso;
	yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > portLeft, portRight, portWide, portLeftFov, portLeftLog, portRightFov, portRightLog ;
    yarp::os::Port cmdPort;
	yarp::os::BufferedPort<yarp::os::Bottle> tactileLeftPort;
    yarp::os::BufferedPort<yarp::os::Bottle> tactileRightPort;
    yarp::os::BufferedPort<yarp::os::Bottle> tactilePort;
    yarp::os::BufferedPort<yarp::os::Bottle> inertialPort;

    int _argc;
    char **_argv;
    int w, h;
    bool stopped;
    bool extractImages;
    void (*wrappedStep) (int pause);
    yarp::os::Semaphore mutex, pulse, ack;
    double sloth;

    void init();
    void initImagePorts();
	void initIcubParts();

    yarp::dev::PolyDriver *createPart(const char *name);

    WorldManager world_manager;
    RobotConfig& robot_config;
    RobotFlags& robot_flags;
    yarp::os::ResourceFinder& finder;
    Simulation *sim;
};

#endif
