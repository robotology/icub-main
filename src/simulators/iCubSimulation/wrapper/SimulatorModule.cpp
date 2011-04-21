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
#include "SimulatorModule.h"

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Property.h>
#include <yarp/os/Network.h>
#include <yarp/os/Os.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>

#include <stdio.h>
#include <string.h>

#include <string>

#ifdef WIN32
#include <windows.h>
#endif

using namespace yarp::os;
using namespace yarp::sig;
#ifndef OMIT_LOGPOLAR
using namespace iCub::logpolar;
#endif
using namespace std;

bool viewParam1 = false, viewParam2 = false;
bool noCam;

const int ifovea = 128;
const int baseWidth = 320;
const int baseHeight = 240;

SimulatorModule::SimulatorModule(WorldManager& world, RobotConfig& config, 
                                 Simulation *sim) : 
    moduleName(config.getModuleName()), mutex(1), pulse(0), ack(0), 
    world_manager(world),
    robot_config(config),
    robot_flags(config.getFlags()),
    finder(config.getFinder()),
    sim(sim) {
    Property options;       
    wrappedStep = NULL;
    stopped = false;
    extractImages = false;
    viewParam1 = false;
    viewParam2 = false;	
    sim = NULL;
    sloth = 0;
    iCubLArm = NULL;
    iCubRArm = NULL;
    iCubHead = NULL; 
    iCubLLeg = NULL;
    iCubRLeg = NULL;
    iCubTorso = NULL;
    failureToLaunch = false;
}

void SimulatorModule::sendTouchLeft(Bottle& report){
    tactileLeftPort.prepare() = report;
    tactileLeftPort.write();
}

void SimulatorModule::sendTouchRight(Bottle& report){
    tactileRightPort.prepare() = report;
    tactileRightPort.write();
}
	
bool SimulatorModule::shouldSendTouchLeft() {
    return tactileLeftPort.getOutputCount()>0;
}

bool SimulatorModule::shouldSendTouchRight() {
    return tactileRightPort.getOutputCount()>0;
}

void SimulatorModule::sendInertial(Bottle& report){
    inertialPort.prepare() = report;
    inertialPort.write();
}

bool SimulatorModule::shouldSendInertial(){
    return inertialPort.getOutputCount()>0;
}

void SimulatorModule::sendVision() {
    displayStep(0);
}

bool SimulatorModule::closeModule() {
    printf("Closing module...\n");
    interruptModule();
    
    if (sim!=NULL)
        delete sim;
    sim=NULL;

    portLeft.interrupt();        
    portRight.interrupt();
#ifndef OMIT_LOGPOLAR
    portLeftFov.interrupt();
    portLeftLog.interrupt();
    portRightFov.interrupt();
    portRightLog.interrupt();
#endif    
    portWide.interrupt();
    
    portLeft.close();
    
    portRight.close();

#ifndef OMIT_LOGPOLAR
    portLeftFov.close();
    portLeftLog.close();
    portRightFov.close();
    portRightLog.close();
#endif
    
    portWide.close();
    
    tactileLeftPort.close();
    tactileRightPort.close();
    inertialPort.close();
    cmdPort.close();
    
    fprintf(stderr, "Successfully terminated...bye...\n");
    return true;
}

// this should become a close
bool SimulatorModule::interruptModule() {
    stopped = true;
    
    if (iCubLArm!=NULL) {
        delete iCubLArm;
        iCubLArm = NULL;
    }
    if (iCubRArm!=NULL) {
        delete iCubRArm;
        iCubRArm = NULL;
    }
    if (iCubHead!=NULL) {
        delete iCubHead;
        iCubHead = NULL;
    }
    if (iCubLLeg!=NULL) {
        delete iCubLLeg;
        iCubLLeg = NULL;
    }
    if (iCubRLeg!=NULL) {
        delete iCubRLeg;
        iCubRLeg = NULL;
    }
    if (iCubTorso!=NULL) {
        delete iCubTorso;
        iCubTorso = NULL;
    }
    world_manager.clear();
  
    return true;
}

bool SimulatorModule::read(ConnectionReader& connection){
    //printf("in read...\n");
    Bottle cmd, reply;
    cmd.read(connection);
    respond(cmd,reply);
    if (connection.getWriter()!=NULL){
        reply.write(*connection.getWriter());
    }
    return true;
}

bool SimulatorModule::respond(const Bottle &command, Bottle &reply) {
    ConstString cmd = command.get(0).asString();
    bool ok = true;
    bool done = false;
    if (cmd=="help") {
        printf("Commands available:\n");
        printf("\tleft\n");
        printf("\tright\n");
        printf("\twide\n");
        printf("\tworld\n");
        reply.fromString("world etc");
        done = true;
    } else if (cmd=="left") {
        viewParam1 = true;
        viewParam2 = false;
        reply.fromString("ok");
        done = true;
    } else if (cmd=="right") {
        viewParam1 = false;
        viewParam2 = true;
        reply.fromString("ok");
        done = true;
    } else if (cmd=="wide") {
        viewParam1 = false;
        viewParam2 = false;
        //Simulation::start(); // sets viewpoint
        reply.fromString("ok");
        done = true;
    } else if (cmd=="world"){
        return world_manager.respond(command,reply);
    }
    return ok;
}

yarp::dev::PolyDriver *SimulatorModule::createPart(const char *name) {
    printf("Creating interface for body part %s\n", name);
    Property options;
    ConstString part_file = finder.findFile(name);
    options.fromConfigFile(part_file.c_str());
    ConstString part_port = options.check("name",Value(1),"what did the user select?").asString();
    string full_name = moduleName + part_port.c_str();
    options.put("name", full_name.c_str() );
    options.put("joint_device", "robot_joints");
    yarp::dev::PolyDriver *driver = new yarp::dev::PolyDriver(options);

    if (!driver->isValid()){
        printf("Device not available. Here are the known devices:\n");
        printf("%s", yarp::dev::Drivers::factory().toString().c_str());
        failureToLaunch = true;
        return NULL;
    }
    return driver;
}

void SimulatorModule::init()
{
    if (!robot_flags.valid) {
        printf("Robot flags are not set when creating SimulatorModule\n");
        failureToLaunch = true;
        return;
    }

    Property options;
    if (robot_flags.actLArm || robot_flags.actLHand) {
        //start left arm device driver
        iCubLArm = createPart("left_arm");
    }

    if (robot_flags.actRArm || robot_flags.actRHand) {
      	//start right arm device driver
      	iCubRArm = createPart("right_arm");
    }

    if (robot_flags.actHead) {
        //start head device driver
        iCubHead = createPart("head");
    }

    if (robot_flags.actLegs) {
        //start left leg device driver
        iCubLLeg = createPart("left_leg");
    }

    if (robot_flags.actLegs) {
        //start right leg device driver
        iCubRLeg = createPart("right_leg");
    }
    if (robot_flags.actTorso) {
        //start torso device driver
        iCubTorso = createPart("torso");
    }
}

void SimulatorModule::initImagePorts() {
    Property options;

    ConstString cameras = finder.findFile("cameras");
    options.fromConfigFile(cameras.c_str());
    
    ConstString nameExternal = 
        options.check("name_wide",
                      Value("/cam"),
                      "Name of external view camera port").asString();
    
    ConstString nameLeft = 
        options.check("name_left",
                      Value("/cam/left"),
                      "Name of left camera port").asString();
    
    ConstString nameRight = 
        options.check("name_right",
                      Value("/cam/right"),
                      "Name of right camera port").asString();

#ifndef OMIT_LOGPOLAR
    ConstString nameLeftFov = 
        options.check("name_leftFov",
                      Value("/cam/left/fov"),
                      "Name of left camera fovea port").asString();
    ConstString nameRightFov = 
        options.check("name_rightFov",
                      Value("/cam/right/fov"),
                      "Name of right camera fovea port ").asString();

    ConstString nameLeftLog = 
        options.check("name_leftLog",
                      Value("/cam/left/logpolar"),
                      "Name of left camera logpolar port").asString();
    ConstString nameRightLog = 
        options.check("name_rightLog",
                      Value("/cam/right/logpolar"),
                      "Name of right camera logpolar port").asString();
#endif

    string leftCam = moduleName + nameLeft.c_str();
    string rightCam = moduleName + nameRight.c_str();
    string mainCam = moduleName + nameExternal.c_str();

#ifndef OMIT_LOGPOLAR    
    string leftFov = moduleName + nameLeftFov.c_str();
    string rightFov = moduleName + nameRightFov.c_str();
    string leftLog = moduleName + nameLeftLog.c_str();
    string rightLog = moduleName + nameRightLog.c_str();
#endif

    portLeft.open( leftCam.c_str());
    portRight.open( rightCam.c_str() );
    portWide.open( mainCam.c_str() );

#ifndef OMIT_LOGPOLAR    
    portLeftFov.open( leftFov.c_str() );
    portRightFov.open( rightFov.c_str() );

    portLeftLog.open( leftLog.c_str() );
    portRightLog.open( rightLog.c_str() );
#endif
}


bool SimulatorModule::open() {
    cmdPort.setReader(*this);
    string world = moduleName + "/world";
    
    string tactileLeft = moduleName + "/skin/left_hand";
    string tactileRight = moduleName + "/skin/right_hand";

    string inertial = moduleName + "/inertial";
    cmdPort.open( world.c_str() );
    tactileLeftPort.open( tactileLeft.c_str() );
    tactileRightPort.open( tactileRight.c_str() );
    inertialPort.open( inertial.c_str() );

    if (robot_flags.actVision) {
        initImagePorts();
    }
    init();
    if (failureToLaunch) return false;

    sim->init(this,&robot_config);

    w = 480;
    h = 640;

    // the main image buffer.
    buffer.resize( w, h);

    firstpass = true;

    return true;
}

bool SimulatorModule::runModule() {

    Time::delay(1); // there's some clash over keyboard on Windows
    sim->simLoop(h,w);
    return true;
}



void SimulatorModule::displayStep(int pause) {
    if (sim->checkSync()){
        bool needLeft = (portLeft.getOutputCount()>0);
        bool needRight = (portRight.getOutputCount()>0);
        bool needWide = (portWide.getOutputCount()>0);
#ifndef OMIT_LOGPOLAR
        bool needLeftFov = (portLeftFov.getOutputCount()>0);
        bool needLeftLog = (portLeftLog.getOutputCount()>0);
        bool needRightFov = (portRightFov.getOutputCount()>0);
        bool needRightLog = (portRightLog.getOutputCount()>0); 
#endif

        mutex.wait();
        //stopping
        bool done = stopped;
        mutex.post();


#ifndef WIN32_DYNAMIC_LINK 
        const char *order = "lrw";
        if (viewParam1) {
            order = "rwl";
        }
        if (viewParam2) {
            order = "lwr";
        }

        for (int i=0; i<3; i++) {
            char ch = order[i];
            switch (ch) {
            case 'l':
                if (needLeft) {
                    sim->drawView(true,false,false);
                    getImage();
                    sendImage(portLeft);
                    sim->clearBuffer();
                }
#ifndef OMIT_LOGPOLAR    
                if (needLeftFov) {
                    sim->drawView(true,false,false);
                    getImage();
                    sendImageFov(portLeftFov);
                    sim->clearBuffer();
                }
                if (needLeftLog) {
                    sim->drawView(true,false,false);
                    getImage();
                    sendImageLog(portLeftLog);
                    sim->clearBuffer();
                }
#endif
                break;
            case 'r':
                if (needRight) {
                    sim->drawView(false,true,false);
                    getImage();
                    sendImage(portRight);
                    sim->clearBuffer();
                }
#ifndef OMIT_LOGPOLAR    
                if (needRightFov) {
                    sim->drawView(false,true,false);
                    getImage();
                    sendImageFov(portRightFov);
                    sim->clearBuffer();
                }
                if (needRightLog) {
                    sim->drawView(false,true,false);
                    getImage();
                    sendImageLog(portRightLog);
                    sim->clearBuffer();
                }
#endif
                break;
            case 'w':
                if (needWide) {
                    sim->drawView(false,false,true);
                    getImage();
                    sendImage(portWide);
                    sim->clearBuffer();
                }
                break;
            }
        }
#else
        // per-image operations can be done here
        if (wrappedStep!=NULL) {
            wrappedStep(pause);
        }
#endif
        /**/
        double now = Time::now();
        static double first = now;
        static double target = 0;
        now -= first;
        target += sloth;
        if (target>now) {
            Time::delay(target-now);
        }
        sim->checkSync(true);
    }
}

void SimulatorModule::getImage(){
    sim->getImage(buffer);
}

void SimulatorModule::sendImage(BufferedPort<ImageOf<PixelRgb> >& port) {
    ImageOf<PixelRgb>& normal = port.prepare();
    normal.copy( buffer );
    port.write();
}

#ifndef OMIT_LOGPOLAR    

void SimulatorModule::sendImageFov(BufferedPort<ImageOf<PixelRgb> >& portFov) {
    static ImageOf<PixelRgb> fov;
    ImageOf<PixelRgb>& targetFov = portFov.prepare();
    subsampleFovea( fov, buffer );
    targetFov.copy( fov );
    portFov.write();
}

void SimulatorModule::sendImageLog(BufferedPort<ImageOf<PixelRgb> >& portLog) {
    static ImageOf<PixelRgb> lp;
    lp.resize (252, 152);
    lp.zero();
    ImageOf<PixelRgb>& targetLog = portLog.prepare();
    cartToLogPolar( lp , buffer );
    targetLog.copy( lp );
    portLog.write();
}


bool SimulatorModule::cartToLogPolar(ImageOf<PixelRgb>& lp, const ImageOf<PixelRgb>& cart) {
    //  
    if (firstpass) {
        if (!trsf.allocated())
            trsf.allocLookupTables(C2L, lp.height(), lp.width(), cart.width(), cart.height(), 1.);

        firstpass = false;
    }

    return trsf.cartToLogpolar(lp, cart);
}

bool SimulatorModule::subsampleFovea(yarp::sig::ImageOf<yarp::sig::PixelRgb>& dst, 
                                     const yarp::sig::ImageOf<yarp::sig::PixelRgb>& src) {
    //
    dst.resize (ifovea, ifovea);
    return iCub::logpolar::subsampleFovea(dst, src);
}

#endif
