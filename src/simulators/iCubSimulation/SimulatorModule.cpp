// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Vadim Tikhanoff, Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#include "SimulatorModule.h"
#include "iCubSimulationControl.h"

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Property.h>
#include <yarp/os/Network.h>
#include <yarp/os/Os.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>

#include <stdio.h>
#include <string.h>

#ifdef WIN32
#include <windows.h>
#endif

using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::logpolar;

#include "iCub_Sim.h"


bool viewParam1 = false, viewParam2 = false;
bool noCam;

const int ifovea = 128;
const int baseWidth = 320;
const int baseHeight = 240;

SimulatorModule::SimulatorModule(RobotConfig& config) : 
    moduleName(config.getModuleName()), mutex(1), pulse(0), ack(0), 
    robot_config(config),
    finder(config.getFinder()) {
	Property options;       
    wrappedStep = NULL;
    stopped = false;
    extractImages = false;
    viewParam1 = false;
    viewParam2 = false;	
    sim = NULL;
    sloth = 0;
}
    

void SimulatorModule::sendTouch(Bottle& report){
    tactilePort.prepare() = report;
    tactilePort.write();
}
	
bool SimulatorModule::shouldSendTouch() {
    return tactilePort.getOutputCount()>0;
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
    portLeftFov.interrupt();
    portLeftLog.interrupt();
    portRight.interrupt();
    portRightFov.interrupt();
    portRightLog.interrupt();
    
    portWide.interrupt();
    
    portLeft.close();
    portLeftFov.close();
    portLeftLog.close();
    
    portRight.close();
    portRightFov.close();
    portRightLog.close();
    
    portWide.close();
    
    tactilePort.close();
    inertialPort.close();
    cmdPort.close();
    
    fprintf(stderr, "Successfully terminated...bye...\n");
    return true;
}

// this should become a close
bool SimulatorModule::interruptModule() {
    OdeInit& odeinit = OdeInit::get();
    stopped = true;//odeinit.stop = true;
    
    if (iCubLArm!=NULL && odeinit._iCub->actLArm == "on"|| odeinit._iCub->actRHand == "on") {
        delete iCubLArm;
        iCubLArm = NULL;
    }
    if (iCubRArm!=NULL && odeinit._iCub->actRArm == "on" || odeinit._iCub->actRHand == "on") {
        delete iCubRArm;
        iCubRArm = NULL;
    }
    if (iCubHead!=NULL && odeinit._iCub->actHead == "on") {
        delete iCubHead;
        iCubHead = NULL;
    }
    if (iCubLLeg!=NULL && odeinit._iCub->actLegs == "on") {
        delete iCubLLeg;
        iCubLLeg = NULL;
    }
    if (iCubRLeg!=NULL && odeinit._iCub->actLegs == "on") {
        delete iCubRLeg;
        iCubRLeg = NULL;
    }
    if (iCubTorso!=NULL && odeinit._iCub->actTorso == "on") {
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

void SimulatorModule::init()
{
    OdeInit& odeinit = OdeInit::get();
	Property options;
    if (odeinit._iCub->actLArm == "on" || odeinit._iCub->actLHand == "on"){
      	//start left arm device driver
	    ConstString left_arm = finder.findFile("left_arm");
	    options.fromConfigFile(left_arm.c_str());
        ConstString leftArmPort = options.check("name",Value(1),"what did the user select?").asString();
        string leftArm = moduleName + leftArmPort.c_str();
        options.put("name", leftArm.c_str() );
      	iCubLArm = new yarp::dev::PolyDriver(options);

        if (!iCubLArm->isValid()){
            printf("Device not available. Here are the known devices:\n");
	        printf("%s", yarp::dev::Drivers::factory().toString().c_str());
            Network::fini();
            yarp::os::exit(1);
        }
    }

    if (odeinit._iCub->actRArm == "on" || odeinit._iCub->actRHand == "on"){
      	//start right arm device driver
      	ConstString right_arm = finder.findFile("right_arm");
	    options.fromConfigFile(right_arm.c_str());
        ConstString rightArmPort = options.check("name",Value(1),"what did the user select?").asString();
        string rightArm = moduleName + rightArmPort.c_str();
        options.put("name", rightArm.c_str() );
      	iCubRArm = new yarp::dev::PolyDriver(options);

        if (!iCubRArm->isValid()){
            printf("Device not available. Here are the known devices:\n");
	        printf("%s", yarp::dev::Drivers::factory().toString().c_str());
            Network::fini();
            yarp::os::exit(1);
        }
    }

    if (odeinit._iCub->actHead == "on"){
      	//start head device driver
      	ConstString head = finder.findFile("head");
	    options.fromConfigFile(head.c_str());
        ConstString headPort = options.check("name",Value(1),"what did the user select?").asString();
        string headStr = moduleName + headPort.c_str();
        options.put("name", headStr.c_str() );
      	iCubHead = new yarp::dev::PolyDriver(options);

        if (!iCubHead->isValid()){
            printf("Device not available. Here are the known devices:\n");
	        printf("%s", yarp::dev::Drivers::factory().toString().c_str());
            Network::fini();
            yarp::os::exit(1);
        }
    }

    if (odeinit._iCub->actLegs == "on"){
      	//start left leg device driver
     	ConstString left_leg = finder.findFile("left_leg");
	    options.fromConfigFile(left_leg.c_str());
        ConstString leftLegPort = options.check("name",Value(1),"what did the user select?").asString();
        string leftLeg = moduleName + leftLegPort.c_str();
        options.put("name", leftLeg.c_str() );
      	iCubLLeg = new yarp::dev::PolyDriver(options);

        if (!iCubLLeg->isValid()){
            printf("Device not available. Here are the known devices:\n");
	        printf("%s", yarp::dev::Drivers::factory().toString().c_str());
            Network::fini();
            yarp::os::exit(1);
        }
    }

    if (odeinit._iCub->actLegs == "on"){
       	//start right leg device driver
     	ConstString right_leg = finder.findFile("right_leg");
	    options.fromConfigFile(right_leg.c_str());
        ConstString rightLegPort = options.check("name",Value(1),"what did the user select?").asString();
        string rightLeg = moduleName + rightLegPort.c_str();
        options.put("name", rightLeg.c_str() );
      	iCubRLeg = new yarp::dev::PolyDriver(options);

        if (!iCubRLeg->isValid()){
            printf("Device not available. Here are the known devices:\n");
	        printf("%s", yarp::dev::Drivers::factory().toString().c_str());
            Network::fini();
            yarp::os::exit(1);
        }
    }
    if (odeinit._iCub->actTorso == "on"){
        //start torso device driver
      	ConstString torso = finder.findFile("torso");
	    options.fromConfigFile(torso.c_str());
        ConstString torsoPort = options.check("name",Value(1),"what did the user select?").asString();
        string torsoStr = moduleName + torsoPort.c_str();
        options.put("name", torsoStr.c_str() );
      	iCubTorso = new yarp::dev::PolyDriver(options);
        
        if (!iCubTorso->isValid()){
            printf("Device not available. Here are the known devices:\n");
	        printf("%s", yarp::dev::Drivers::factory().toString().c_str());
            Network::fini();
            yarp::os::exit(1);
        }
    }

	odeinit._wrld->model_DIR = finder.findPath("model_path_default");
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


    string leftCam = moduleName + nameLeft.c_str();
    string rightCam = moduleName + nameRight.c_str();
    string mainCam = moduleName + nameExternal.c_str();
    
    string leftFov = moduleName + nameLeftFov.c_str();
    string rightFov = moduleName + nameRightFov.c_str();
    string leftLog = moduleName + nameLeftLog.c_str();
    string rightLog = moduleName + nameRightLog.c_str();

    portLeft.open( leftCam.c_str());
    portRight.open( rightCam.c_str() );
    portWide.open( mainCam.c_str() );

    portLeftFov.open( leftFov.c_str() );
    portRightFov.open( rightFov.c_str() );

    portLeftLog.open( leftLog.c_str() );
    portRightLog.open( rightLog.c_str() );
}


bool SimulatorModule::open() {
    OdeInit& odeinit = OdeInit::get();
    
	cmdPort.setReader(*this);
    string world = moduleName + "/world";
    string tactile = moduleName + "/touch";
    string inertial = moduleName + "/inertial";
	cmdPort.open( world.c_str() );
	tactilePort.open( tactile.c_str() );
    inertialPort.open( inertial.c_str() );

    if (odeinit._iCub->actVision=="on") {
        initImagePorts();
    }
    init();
    sim = new Simulation(this,&robot_config);
	 
    w = 480;
    h = 640;

    // the main image buffer.
    buffer.resize( w, h);

    firstpass = true;

    return true;
}

bool SimulatorModule::runModule() {

    Time::delay(1); // there's some clash over keyboard on Windows
	Simulation::simLoop(h,w);
    return true;
}



void SimulatorModule::displayStep(int pause) {
    OdeInit& odeinit = OdeInit::get();
    if (odeinit.sync){
        bool needLeft = (portLeft.getOutputCount()>0);
        bool needRight = (portRight.getOutputCount()>0);
        bool needWide = (portWide.getOutputCount()>0);
        bool needLeftFov = (portLeftFov.getOutputCount()>0);
        bool needLeftLog = (portLeftLog.getOutputCount()>0);
        bool needRightFov = (portRightFov.getOutputCount()>0);
        bool needRightLog = (portRightLog.getOutputCount()>0); 

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
                break;
            case 'r':
			    if (needRight) {
				    sim->drawView(false,true,false);
                    getImage();
				    sendImage(portRight);
				    sim->clearBuffer();
                }
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
        odeinit.sync = false;
    }
}

void SimulatorModule::getImage(){
    
    int w = 320;
    int h = 240;
    int p = 3;//320 240

	char buf[ 320 * 240 * 3 ];
    glReadPixels( 0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, buf);
    static ImageOf<PixelRgb> img;
    img.setQuantum(1);
	img.setExternal(buf,w,h);

    // inefficient flip!
    ImageOf<PixelRgb> target;
    target.resize(img);
    int ww = img.width();
    int hh = img.height();
    for (int x=0; x<ww; x++) {
        for (int y=0; y<hh; y++) {
            target(x,y) = img(x,hh-1-y);
        }
    }
    buffer.copy(target);
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
}

void SimulatorModule::sendImage(BufferedPort<ImageOf<PixelRgb> >& port) {
    ImageOf<PixelRgb>& normal = port.prepare();
    normal.copy( buffer );
    port.write();
}

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
