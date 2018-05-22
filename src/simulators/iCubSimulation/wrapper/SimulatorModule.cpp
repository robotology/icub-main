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
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
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
    idesc = NULL;
    dd_descSrv = NULL;
    dd_descClnt = NULL;
}

void SimulatorModule::sendTouchLeftHand(Bottle& report){
    tactileLeftHandPort.prepare() = report;
    generalStamp.update(Time::now());
    tactileLeftHandPort.setEnvelope(generalStamp);
    tactileLeftHandPort.write();
}

void SimulatorModule::sendTouchRightHand(Bottle& report){
    tactileRightHandPort.prepare() = report;
    generalStamp.update(Time::now());
    tactileRightHandPort.setEnvelope(generalStamp);
    tactileRightHandPort.write();
}
	
bool SimulatorModule::shouldSendTouchLeftHand() {
    return tactileLeftHandPort.getOutputCount()>0;
}

bool SimulatorModule::shouldSendTouchRightHand() {
    return tactileRightHandPort.getOutputCount()>0;
}


//whole_body_skin_emul
void SimulatorModule::sendSkinEvents(iCub::skinDynLib::skinContactList& skinContactListReport){
    iCub::skinDynLib::skinContactList &skinEvents = skinEventsPort.prepare();
    skinEvents.clear();
    skinEvents.insert(skinEvents.end(), skinContactListReport.begin(), skinContactListReport.end()); 
    generalStamp.update(Time::now());
    skinEventsPort.setEnvelope(generalStamp);
    skinEventsPort.write();
}
    
bool SimulatorModule::shouldSendSkinEvents(){
    return skinEventsPort.getOutputCount()>0;
}

void SimulatorModule::sendTouchLeftArm(Bottle& report){
     tactileLeftArmPort.prepare() = report;
     generalStamp.update(Time::now());
     tactileLeftArmPort.setEnvelope(generalStamp);
     tactileLeftArmPort.write();
}

void SimulatorModule::sendTouchRightArm(Bottle& report){
     tactileRightArmPort.prepare() = report;
     generalStamp.update(Time::now());
     tactileRightArmPort.setEnvelope(generalStamp);
     tactileRightArmPort.write();
}
        
bool SimulatorModule::shouldSendTouchLeftArm() {
     return tactileLeftArmPort.getOutputCount()>0;
}

bool SimulatorModule::shouldSendTouchRightArm() {
     return tactileRightArmPort.getOutputCount()>0;
}

void SimulatorModule::sendTouchLeftForearm(Bottle& report){
    tactileLeftForearmPort.prepare() = report;
    generalStamp.update(Time::now());
    tactileLeftForearmPort.setEnvelope(generalStamp);
    tactileLeftForearmPort.write();
}

void SimulatorModule::sendTouchRightForearm(Bottle& report){
    tactileRightForearmPort.prepare() = report;
    generalStamp.update(Time::now());
    tactileRightForearmPort.setEnvelope(generalStamp);
    tactileRightForearmPort.write();
}
        
bool SimulatorModule::shouldSendTouchLeftForearm() {
    return tactileLeftForearmPort.getOutputCount()>0;
}

bool SimulatorModule::shouldSendTouchRightForearm() {
    return tactileRightForearmPort.getOutputCount()>0;
}
    
void SimulatorModule::sendTouchTorso(Bottle& report){
    tactileTorsoPort.prepare() = report;
    generalStamp.update(Time::now());
    tactileTorsoPort.setEnvelope(generalStamp);
    tactileTorsoPort.write();
}

bool SimulatorModule::shouldSendTouchTorso() {
    return tactileTorsoPort.getOutputCount()>0;
}
//end of whole_body_skin_emul methods

void SimulatorModule::sendInertial(Bottle& report){
    inertialPort.prepare() = report;
    generalStamp.update(Time::now());
    inertialPort.setEnvelope(generalStamp);
    inertialPort.write();
}

bool SimulatorModule::shouldSendInertial(){
    return inertialPort.getOutputCount()>0;
}

void SimulatorModule::sendVision() {
    displayStep(0);
}

bool SimulatorModule::closeModule() {
    yInfo("Closing module...\n");
    interruptModule();

    if (sim != NULL)
        delete sim;
    sim = NULL;

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

    tactileLeftHandPort.close();
    tactileRightHandPort.close();
    tactileLeftArmPort.close();
    tactileRightArmPort.close();
    tactileLeftForearmPort.close();
    tactileRightForearmPort.close();
    tactileTorsoPort.close();

    inertialPort.close();
    cmdPort.close();

    trqLeftLegPort.close();
    trqRightLegPort.close();
    trqTorsoPort.close();
    trqLeftArmPort.close();
    trqRightArmPort.close();

    if (dd_descClnt)
    {
        dd_descClnt->close();
        delete dd_descClnt;
        dd_descClnt = 0;
    }
    if (dd_descSrv)
    {
        dd_descSrv->close();
        delete dd_descSrv;
        dd_descSrv = 0;
    }


    yInfo("Successfully terminated...bye...\n");
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
    //yDebug("in read...\n");
    Bottle cmd, reply;
    cmd.read(connection);
    respond(cmd,reply);
    if (connection.getWriter()!=NULL){
        reply.write(*connection.getWriter());
    }
    return true;
}

bool SimulatorModule::respond(const Bottle &command, Bottle &reply) {
    string cmd = command.get(0).asString();
    bool ok = true;
    bool done = false;
    if (cmd=="help") {
        yInfo("Commands available:\n");
        yInfo("\tleft\n");
        yInfo("\tright\n");
        yInfo("\twide\n");
        yInfo("\tworld\n");
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
    yDebug("Creating interface for body part %s\n", name);
    Property options;
    string part_file = finder.findFile(name);
    options.fromConfigFile(part_file.c_str());
    string part_port = options.check("name",Value(1),"what did the user select?").asString();
    string full_name = moduleName + part_port.c_str();
    options.put("name", full_name.c_str() );
    options.put("joint_device", "robot_joints");
    yarp::dev::PolyDriver *driver = new yarp::dev::PolyDriver(options);

    if (!driver->isValid()){
        yError("Device not available. Here are the known devices:\n");
        yError("%s", yarp::dev::Drivers::factory().toString().c_str());
        failureToLaunch = true;
        return NULL;
    }
    return driver;
}

bool SimulatorModule::initSimulatorModule()
{
    if (!robot_flags.valid)
    {
        yDebug("Robot flags are not set when creating SimulatorModule\n");
        failureToLaunch = true;
        return false;
    }

    //first we try to open the robotDescriptionClient, because maybe an external server is already running
    bool b_client = false;
    Property clnt_opt;
    clnt_opt.put("device", "robotDescriptionClient");
    clnt_opt.put("local", "/icubSim/robotDescriptionClient");
    clnt_opt.put("remote", "/robotDescription");
    dd_descClnt = new yarp::dev::PolyDriver;
    if (yarp::os::Network::exists("/robotDescription/rpc"))
    {
        if (dd_descClnt->open(clnt_opt) && dd_descClnt->isValid())
        {
            yInfo() << "External robotDescriptionServer was found. Connection succesful.";
            b_client = true;
            dd_descClnt->view(idesc);
        }
        else
        {
            yInfo() << "External robotDescriptionServer was not found. Opening a new RobotDescriptionServer.";
        }
    }
    else
    {
        yInfo() << "External robotDescriptionServer was not found. Opening a new RobotDescriptionServer.";
    }

    //if the previous operation failed, then retry opening first a robotDescriptionServer and then a robotDescriptionClient
    if (b_client == false)
    {
        dd_descSrv = new yarp::dev::PolyDriver;
        Property srv_opt;
        srv_opt.put("device", "robotDescriptionServer");
        srv_opt.put("local", "/robotDescription");

        bool b_server = false;
        if (dd_descSrv->open(srv_opt) && dd_descSrv->isValid())
        {
            b_server = true;
        }
        else
        {
            //unable to open a robotDescriptionServer, this is a critical failure!
            delete dd_descSrv;
            dd_descSrv = 0;
            yError() << "Unable to open robotDescriptionServer";
            failureToLaunch = true;
            return false;
        }

        //robotDescriptionServer is running, so let's retry to open the robotDescriptionClient
        if (dd_descClnt->open(clnt_opt) && dd_descClnt->isValid())
        {
            b_client = true;
            dd_descClnt->view(idesc);
        }
        else
        {
            //unable to open the robotDescriptionClient, this is a critical failure!
            delete dd_descClnt;
            dd_descClnt = 0;
            yError() << "Unable to open robotDescriptionClient";
            failureToLaunch = true;
            return false;
        }
    }

    Property options;
    yarp::dev::DeviceDescription desc;
    if (robot_flags.actLArm || robot_flags.actLHand) {
        //start left arm device driver
        iCubLArm = createPart("left_arm");
        desc.device_name = moduleName+"/left_arm";
        desc.device_type = "controlboardwrapper2";
        if (idesc) idesc->registerDevice(desc);
    }

    if (robot_flags.actRArm || robot_flags.actRHand) {
        //start right arm device driver
        iCubRArm = createPart("right_arm");
        desc.device_name = moduleName + "/right_arm";
        desc.device_type = "controlboardwrapper2";
        if (idesc) idesc->registerDevice(desc);
    }

    if (robot_flags.actHead) {
        //start head device driver
        iCubHead = createPart("head");
        desc.device_name = moduleName + "/head";
        desc.device_type = "controlboardwrapper2";
        if (idesc) idesc->registerDevice(desc);
    }

    if (robot_flags.actLegs) {
        //start left leg device driver
        iCubLLeg = createPart("left_leg");
        desc.device_name = moduleName + "/left_leg";
        desc.device_type = "controlboardwrapper2";
        if (idesc) idesc->registerDevice(desc);
    }

    if (robot_flags.actLegs) {
        //start right leg device driver
        iCubRLeg = createPart("right_leg");
        desc.device_name = moduleName + "/right_leg";
        desc.device_type = "controlboardwrapper2";
        if (idesc) idesc->registerDevice(desc);
    }
    if (robot_flags.actTorso) {
        //start torso device driver
        iCubTorso = createPart("torso");
        desc.device_name = moduleName + "/torso";
        desc.device_type = "controlboardwrapper2";
        if (idesc) idesc->registerDevice(desc);
    }

    //dd_descClnt will be not used anymore.
    dd_descClnt->close();
    delete dd_descClnt;
    dd_descClnt = 0;

    return true;
}

void SimulatorModule::initImagePorts() {
    Property options;

    string cameras = finder.findFile("cameras");
    options.fromConfigFile(cameras.c_str());
    
    string nameExternal = 
        options.check("name_wide",
                      Value("/cam"),
                      "Name of external view camera port").asString();
    
    string nameLeft = 
        options.check("name_left",
                      Value("/cam/left"),
                      "Name of left camera port").asString();
    
    string nameRight = 
        options.check("name_right",
                      Value("/cam/right"),
                      "Name of right camera port").asString();

#ifndef OMIT_LOGPOLAR
    string nameLeftFov = 
        options.check("name_leftFov",
                      Value("/cam/left/fov"),
                      "Name of left camera fovea port").asString();
    string nameRightFov = 
        options.check("name_rightFov",
                      Value("/cam/right/fov"),
                      "Name of right camera fovea port ").asString();

    string nameLeftLog = 
        options.check("name_leftLog",
                      Value("/cam/left/logpolar"),
                      "Name of left camera logpolar port").asString();
    string nameRightLog = 
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
    
    string tactileLeft = moduleName + "/skin/left_hand_comp"; //Matej - changed the name to comp - this corresponds to the real iCub convention - higher values ~ higher pressure
    string tactileRight = moduleName + "/skin/right_hand_comp";
    string tactileLeftrpc = moduleName + "/skin/left_hand_comp/rpc:i";
    string tactileRightrpc = moduleName + "/skin/right_hand_comp/rpc:i";
    
    string torqueLeftLeg = moduleName +"/joint_vsens/left_leg:i";
    string torqueRightLeg = moduleName +"/joint_vsens/right_leg:i";
    string torqueTorso = moduleName +"/joint_vsens/torso:i";
    string torqueRightArm = moduleName +"/joint_vsens/left_arm:i";
    string torqueLeftArm = moduleName +"/joint_vsens/right_arm:i";
    
    string inertial = moduleName + "/inertial";
    cmdPort.open( world.c_str() );
    tactileLeftHandPort.open( tactileLeft.c_str() );
    tactileLeftHandPortrpc.open( tactileLeftrpc.c_str() );
    tactileRightHandPort.open( tactileRight.c_str() );
    tactileRightHandPortrpc.open( tactileRightrpc.c_str() );
    inertialPort.open( inertial.c_str() );

    trqLeftLegPort.open( torqueLeftLeg.c_str() );
    trqRightLegPort.open( torqueRightLeg.c_str() );
    trqTorsoPort.open( torqueTorso.c_str() );
    trqLeftArmPort.open( torqueLeftArm.c_str() );
    trqRightArmPort.open( torqueRightArm.c_str() );

    //whole_body_skin_emul
    string skinEventsPortString = moduleName + "/skinManager/skin_events:o";
    skinEventsPort.open(skinEventsPortString.c_str());
    string tactileLeftArmPortString = moduleName + "/skin/left_arm_comp";
    tactileLeftArmPort.open(tactileLeftArmPortString.c_str());
    string tactileRightArmPortString = moduleName + "/skin/right_arm_comp";
    tactileRightArmPort.open(tactileRightArmPortString.c_str());
    string tactileLeftForearmPortString = moduleName + "/skin/left_forearm_comp";
    tactileLeftForearmPort.open(tactileLeftForearmPortString.c_str());
    string tactileRightForearmPortString = moduleName + "/skin/right_forearm_comp";
    tactileRightForearmPort.open(tactileRightForearmPortString.c_str());
    string tactileTorsoPortString = moduleName + "/skin/torso_comp";
    tactileTorsoPort.open(tactileTorsoPortString.c_str());
    
    if (robot_flags.actVision) {
        initImagePorts();
    }
    initSimulatorModule();
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

void SimulatorModule::checkTorques()
{
    bool needLeftLeg = (trqLeftLegPort.getInputCount()>0);
    bool needRightLeg = (trqRightLegPort.getInputCount()>0);
    bool needLeftArm = (trqLeftArmPort.getInputCount()>0);
    bool needRightArm = (trqRightArmPort.getInputCount()>0);
    bool needTorso = (trqTorsoPort.getInputCount()>0);

    if (needLeftLeg)
        getTorques( trqLeftLegPort );
    if (needRightLeg)
        getTorques( trqRightLegPort );
    if (needLeftArm)
        getTorques( trqLeftArmPort );
    if (needRightArm)
        getTorques( trqRightArmPort );
    if (needTorso)
        getTorques( trqTorsoPort );
}

void SimulatorModule::getTorques( yarp::os::BufferedPort<yarp::os::Bottle>& buffPort )
{
    mutex.wait();
    Bottle *torques = NULL;
    torques = buffPort.read(false);
    if ( torques !=NULL )
    {
        sim->getTrqData( torques->tail() );
    }
    mutex.post();
}

void SimulatorModule::displayStep(int pause) {
    //if (sim->checkSync())
    {
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

        camerasStamp.update(Time::now());

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
        //sim->checkSync(true);
    }
}

void SimulatorModule::getImage(){
    sim->getImage(buffer);
}

void SimulatorModule::sendImage(BufferedPort<ImageOf<PixelRgb> >& port) {
    ImageOf<PixelRgb>& normal = port.prepare();
    normal.copy( buffer );
    port.setEnvelope(camerasStamp);
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
