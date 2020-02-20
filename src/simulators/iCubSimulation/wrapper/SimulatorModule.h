// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Vadim Tikhanoff, Paul Fitzpatrick
* email:   vadim.tikhanoff@iit.it, paulfitz@alum.mit.edu
* website: www.robotcub.org
* Permission is granted to copy, distribute, and/or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
* A copy of the license can be found at
* http://www.robotcub.org/icub/license/gpl.txt
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
* Public License for more details
*/


#ifndef ICUBSIMULATION_SIMULATORMODULE_INC
#define ICUBSIMULATION_SIMULATORMODULE_INC

#include <yarp/os/Bottle.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Image.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IRobotDescription.h>
#include <yarp/dev/IMultipleWrapper.h>

#include "RobotStreamer.h"
#include "RobotConfig.h"
#include "WorldManager.h"
#include "Simulation.h"

#include <mutex>
#include <string>

#ifndef OMIT_LOGPOLAR
// wouldn't be better to have this in conditional compilation?
#include <iCub/logpolar/RC_DIST_FB_logpolar_mapper.h>
#endif
#include "iCub/skinDynLib/skinContactList.h"

class SimulatorModule : public yarp::os::PortReader, public RobotStreamer
{
public:
    SimulatorModule(WorldManager& world, RobotConfig& config, Simulation *sim);

    bool open();
    bool runModule();

    bool closeModule();
    bool interruptModule();

    bool read(yarp::os::ConnectionReader& connection);

    bool respond(const yarp::os::Bottle &command,
                 yarp::os::Bottle &reply);

    // RobotStreamer interface

    virtual void sendVision();

    virtual void sendTouchLeftHand(yarp::os::Bottle& report);
    virtual void sendTouchRightHand(yarp::os::Bottle& report);

    virtual bool shouldSendTouchLeftHand();
    virtual bool shouldSendTouchRightHand();

    virtual void sendInertial(yarp::os::Bottle& report);
    virtual bool shouldSendInertial();

    // whole_body_skin_emul
    virtual void sendSkinEvents(iCub::skinDynLib::skinContactList& skinContactListReport);
    virtual bool shouldSendSkinEvents();

    virtual void sendTouchLeftArm(yarp::os::Bottle& report);
    virtual void sendTouchRightArm(yarp::os::Bottle& report);
    virtual bool shouldSendTouchLeftArm();
    virtual bool shouldSendTouchRightArm();

    virtual void sendTouchLeftForearm(yarp::os::Bottle& report);
    virtual void sendTouchRightForearm(yarp::os::Bottle& report);
    virtual bool shouldSendTouchLeftForearm();
    virtual bool shouldSendTouchRightForearm();

    virtual void sendTouchTorso(yarp::os::Bottle& report);
    virtual bool shouldSendTouchTorso();

private:

#ifndef OMIT_LOGPOLAR
    // wrapper to logpolarTransform, taking into account initialization
    bool cartToLogPolar(yarp::sig::ImageOf<yarp::sig::PixelRgb> &lp,
                        const yarp::sig::ImageOf<yarp::sig::PixelRgb> &cart);
    bool subsampleFovea(yarp::sig::ImageOf<yarp::sig::PixelRgb>& dst,
                        const yarp::sig::ImageOf<yarp::sig::PixelRgb>& src);

    void sendImageFov(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >& port);
    void sendImageLog(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >& port);
#endif

    void displayStep(int pause);

    virtual void checkTorques();
    void getTorques( yarp::os::BufferedPort<yarp::os::Bottle>& Port );

    void getImage();
    void sendImage(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >& port);

    std::string moduleName;
    yarp::dev::IRobotDescription* idesc;
    yarp::dev::PolyDriver *dd_descSrv;
    yarp::dev::PolyDriver *dd_descClnt;

#ifndef OMIT_LOGPOLAR
    iCub::logpolar::logpolarTransform trsf;
#endif
    bool firstpass;

    yarp::os::Stamp  camerasStamp;
    yarp::os::Stamp generalStamp;
    yarp::sig::ImageOf<yarp::sig::PixelRgb> buffer;

    yarp::dev::PolyDriver *iCubLArm, *iCubRArm, *iCubHead, *iCubLLeg ,*iCubRLeg, *iCubTorso;
    yarp::dev::PolyDriver masserver, simImu;;
    yarp::dev::PolyDriverList polyList;
    yarp::dev::IMultipleWrapper* iMWrapper{nullptr};
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > portLeft, portRight, portWide;

#ifndef OMIT_LOGPOLAR
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > portLeftFov, portLeftLog, portRightFov, portRightLog;
#endif
    yarp::os::Port cmdPort;
    yarp::os::BufferedPort<yarp::os::Bottle> tactileLeftHandPort, tactileRightHandPort, inertialPort; //Matej - renamed tactileLeftPort to tactileLeftHandPort, same for right, deleted tactilePort
    yarp::os::BufferedPort<yarp::os::Bottle> trqLeftLegPort, trqRightLegPort, trqLeftArmPort, trqRightArmPort, trqTorsoPort;
    yarp::os::Port tactileLeftHandPortrpc, tactileRightHandPortrpc;
    //whole_body_skin_emul
    yarp::os::BufferedPort<iCub::skinDynLib::skinContactList> skinEventsPort;
    yarp::os::BufferedPort<yarp::os::Bottle> tactileLeftArmPort, tactileRightArmPort, tactileLeftForearmPort, tactileRightForearmPort, tactileTorsoPort;

    int _argc;
    char **_argv;
    int w, h;
    bool stopped;
    bool extractImages;
    void (*wrappedStep) (int pause);
    std::mutex mtx;
    double sloth;

    bool initSimulatorModule();
    void initImagePorts();
    void initIcubParts();

    yarp::dev::PolyDriver *createPart(const char *name);

    WorldManager& world_manager;
    RobotConfig& robot_config;
    RobotFlags& robot_flags;
    yarp::os::ResourceFinder& finder;
    Simulation *sim;
    bool failureToLaunch;
};

#endif
