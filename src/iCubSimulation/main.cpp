// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_module
 * 
 * \defgroup icub_Simulation iCubSimulation
 * 
 * This module is the physics simulator of the iCub robot. 
 *
 * \image html iCubSim.jpg
 * \image latex robotMotorGui.eps "A window of the iCub Simulator running on Linux" width=15cm
 * 
 * \section intro_sec Description
 * 
 *	The module does:
 * -	Has been designed to reproduce, as accurately as possible, the physics and the dynamics of the robot and its environment
 * -   It has been constructed collecting data directly from the robot design specifications in order to achieve an exact replication 
 * -	The software infrastructure and inter-process communication are as those used to control the physical robot
 *
 * \section lib_sec Libraries
 * YARP
 * ODE
 * SDL 
 *
 * \section parameters_sec Parameters
 * All the parameters are located in the conf/iCub_parts_activation.ini
 *
 *	[SETUP]
 * elevation on
 *
 *	[PARTS]
 *	legs on
 *	torso on 
 *	left_arm on
 *	left_hand on
 *	right_arm on 
 *	right_hand on
 *	head on
 *	fixed_hip on 
 *
 *	[VISION]
 *	cam off
 *
 *	[RENDER]
 *	objects off
 *	cover on
 * 
 * \section portsa_sec Ports Accessed
 * No ports are accessed nor needed by the iCub simulator
 *
 * \section portsc_sec Ports Created
 * list of ports created by the module:
 *
 * - /icubSim/left_leg/rpc:i : left_leg rpc port
 * - /icubSim/left_leg/command:i : left_leg command port
 * - /icubSim/left_leg/state:o : left_leg state port
 *
 * - /icubSim/right_leg/rpc:i : right_leg rpc port
 * - /icubSim/right_leg/command:i : right_leg command port
 * - /icubSim/right_leg/state:o : right_leg state port
 *
 * - /icubSim/torso/rpc:i : torso rpc port
 * - /icubSim/torso/command:i : torso command port
 * - /icubSim/torso/state:o : torso state port
 *
 * - /icubSim/left_arm/rpc:i : left arm rpc port
 * - /icubSim/left_arm/command:i : left arm command port
 * - /icubSim/left_arm/state:o : left arm state port
 *
 * - /icubSim/right_arm/rpc:i : right arm rpc port
 * - /icubSim/right_arm/command:i : right arm command port
 * - /icubSim/right_arm/state:o : right arm state port
 *
 * - /icubSim/head/rpc:i : head rpc port
 * - /icubSim/head/command:i : head command port
 * - /icubSim/head/state:o : head state port
 *
 * - /icubSim/face/eyelids : port to control the eyelids
 * - /icubSim/cam/left : streams out the data from the left camera
 * - /icubSim/cam/right : streams out the data from the right camera
 * - /icubSim/cam : streams out the data from the global view
 *
 * - /icubSim/world : port to manipulate the environment
 * - /icubSim/touch : streams out a sequence the touch sensors for both hands
 *
 * - /icubSim/texture : port to receive texture data to place on an object (e.g. data from a webcam etc...)
 *
 * \section in_files_sec Input Data Files
 * iCubSimulator expects the following configuration files:
 * - iCub_parts_activation.ini  : the runtime parameters
 * - Sim_camera.ini  : initialization file for the camera(s) of the iCub Simulation
 * - Sim_head.ini  : initialization file for the head of the iCub Simulation
 * - Sim_left_arm.ini  : initialization file for the left arm of the iCub Simulation
 * - Sim_left_leg.ini  : initialization file for the left leg of the iCub Simulation
 * - Sim_right_arm.ini  : initialization file for the right arm of the iCub Simulation
 * - Sim_right_leg.ini  : initialization file for the right leg of the iCub Simulation
 * - Sim_torso.ini  : initialization file for the torso of the iCub Simulation
 * - Video.ini  : initialization file for the texture port
 *
 * \section tested_os_sec Tested OS
 * Linux, Windows and Mac partially.
 *
 * \author Tikhanoff Vadim, Fitzpatrick Paul
 *
 * Copyright (C) 2007 - 2008 - 2009 RobotCub Consortium
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * This file can be edited at src/iCubSimulation/main.cpp.
 *
 **/

//\file main.cpp
//\brief This file is responsible for the initialisation the module and yarp ports. It deals with incoming world commands such as getting information or creating new objects 

#include "iCubSimulationControl.h"
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Property.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/ImageFile.h>
#include <yarp/sig/ImageDraw.h>


#include <stdio.h>
#include <string.h>

#include <ace/OS.h>
#undef main

#ifdef WIN32
#include <windows.h>
#endif
#include <iostream>
#include "SimConfig.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

#include "iCub_Sim.h"

VideoTexture *Simulation::video=0;

OdeInit *_odeinit = NULL;
 OdeInit& getOdeInit() {
  if (_odeinit==NULL) {
     _odeinit=new OdeInit;
  }
  return *_odeinit;
 }
#define odeinit (getOdeInit())

static int num=0;// number of objects in simulation
static int nextobj=0;// next object to recycle if num==NUM
size_t i;
int setBody = 0;
dReal sides[3];
#define DENSITY (1.0)		// density of all objects

bool viewParam1 = false, viewParam2 = false;
bool noCam;

class SimulatorModule : public yarp::os::PortReader {
private:
    PolyDriver *iCubLArm, *iCubRArm, *iCubHead, *iCubLLeg ,*iCubRLeg, *iCubTorso;
	BufferedPort<ImageOf<PixelRgb> > portLeft, portRight, portWide;
	Port cmdPort;
	BufferedPort<Bottle> tactilePort;

    int _argc;
    char **_argv;
    int w, h;
    bool stopped;
    bool extractImages;
    Simulation *sim;
    void (*wrappedStep) (int pause);
    Semaphore mutex, pulse, ack;
    double sloth;

    void init();
    void initImagePorts();
	void initIcubParts();
	SimConfig finder;

public:
    SimulatorModule() : mutex(1), pulse(0), ack(0) {
	Property options;       
		wrappedStep = NULL;
        stopped = false;
        extractImages = false;
		viewParam1 = false;
		viewParam2 = false;	
        sim = NULL;
        sloth = 0;
    }

    bool open();
    bool runModule();

	bool configure(int argc, char **argv){
		finder.configure(argc, argv);
		return true;	
	}

	void sendTouch(Bottle& report){
		tactilePort.prepare() = report;
		tactilePort.write();
	}
	
	bool shouldSendTouch(){
		return tactilePort.getOutputCount()>0;
	}

    bool closeModule() {
   		printf("Closing module...\n");
        interruptModule();
    
        if (sim!=NULL)
            delete sim;
        sim=NULL;

        portLeft.interrupt();
        portRight.interrupt();
        portWide.interrupt();

        portLeft.close();
        portRight.close();
        portWide.close();

        tactilePort.close();
        cmdPort.close();

        fprintf(stderr, "Successfully terminated...bye...\n");
        return true;
    }

    // this should become a close
    bool interruptModule() {
        stopped = true;//odeinit.stop = true;

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
        return true;
    }

    void displayStep(int pause);

	bool read (ConnectionReader& connection){
		//printf("in read...\n");
		Bottle cmd, reply;
		cmd.read(connection);
		respond(cmd,reply);
		if (connection.getWriter()!=NULL){
			reply.write(*connection.getWriter());
		}
		return true;
	}

    bool respond (const Bottle &command, Bottle &reply) {
       //fprintf(stderr, "Reading from world port...\n");
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
		
			ODE_access.wait();
			ConstString subcmd = command.get(1).asString();
			if (subcmd=="get"||subcmd=="set"||subcmd=="mk"||subcmd=="grab"||subcmd=="rot") {
				int id = command.get(2).asVocab();
				
				dBodyID bid = NULL;
				dGeomID bid2 = NULL;
				switch (id) {
					case VOCAB4('c','u','b','e'):
						bid = odeinit._wrld->Box;
						setBody = 0;
						break;
					case VOCAB4('b','a','l','l'):
						bid = odeinit._wrld->ballBody;
						setBody = 1;
						break;
					case VOCAB3('b','o','x'):
						bid = odeinit._iCub->body[1];//temporary then will be replaced...
						setBody = 2;
						break;
					case VOCAB4('s','b','o','x'):
						bid2 = odeinit._iCub->geom[1];//temporary then will be replaced...
						setBody = 5;
						break;
					case VOCAB3('c','y','l'):
						bid = odeinit._iCub->body[1];//temporary then will be replaced...
						setBody = 6;
						break;
					case VOCAB4('s','c','y','l'):
						bid2 = odeinit._iCub->geom[1];//temporary then will be replaced...
						setBody = 7;
						break;
					case VOCAB4('l','h','a','n'):
						if (odeinit._iCub->actLHand=="on"){bid = odeinit._iCub->body[10]; printf("Full left hand\n");}
						else {bid = odeinit._iCub->l_hand; printf("slim left hand\n");}
						setBody = 3;
						break;
					case VOCAB4('r','h','a','n'):
						if (odeinit._iCub->actRHand=="on"){bid = odeinit._iCub->body[11]; printf("Full left hand\n");}
						else {bid = odeinit._iCub->r_hand; printf("slim right hand\n");}
						setBody = 4;
						break;
				}
				reply.clear();
				if (bid!=NULL || bid2!=NULL) {
					if (subcmd=="get") {
						if (setBody == 0 || setBody == 1 || setBody == 3 || setBody == 4){
							const dReal *coords = dBodyGetPosition(bid);
							reply.addDouble(coords[0]);
							reply.addDouble(coords[1]);
							reply.addDouble(coords[2]);
						}
						if (setBody == 2){
							unsigned int N = command.get(3).asInt();
							if ((N >(i+1)) || (N < 1 )){reply.addString("object not known");}
							else{
								const dReal *coords = dBodyGetPosition(odeinit._wrld->obj[N-1].boxbody);
								reply.addDouble(coords[0]);
								reply.addDouble(coords[1]);
								reply.addDouble(coords[2]);
							}
						}
						if (setBody == 5){
							unsigned int N = command.get(3).asInt();
							if ((N >(i+1)) || (N < 1 )){reply.addString("object not known");}
							else{	const dReal *coords = dGeomGetPosition(odeinit._wrld->s_obj[N-1].geom[0]);
								reply.addDouble(coords[0]);
								reply.addDouble(coords[1]);
								reply.addDouble(coords[2]);
								}
						}
						if (setBody==6){
							unsigned int N = command.get(3).asInt();
							if ( (N >(i+1)) || (N < 1 )){reply.addString("object not known");}
							else{
								const dReal *coords = dBodyGetPosition(odeinit._wrld->cyl_obj[N-1].cylbody);
								reply.addDouble(coords[0]);
								reply.addDouble(coords[1]);
								reply.addDouble(coords[2]);
							}
						}
						if (setBody==7){
							unsigned int N = command.get(3).asInt();
							if ((N >(i+1)) || (N < 1 )){reply.addString("object not known");}
							else{
								const dReal *coords = dGeomGetPosition(odeinit._wrld->s_cyl_obj[N-1].cylgeom[0]);
								reply.addDouble(coords[0]);
								reply.addDouble(coords[1]);
								reply.addDouble(coords[2]);
							}
						}
						//reply.fromString("ok");
					} 
					if (subcmd=="set") {
						if (setBody == 0 || setBody == 1){
							double x = command.get(3).asDouble();
							double y = command.get(4).asDouble();
							double z = command.get(5).asDouble();
							dBodySetPosition(bid,x,y,z);
							dBodySetLinearVel (bid, 0.0, 0.0, 0.0);
							dBodySetAngularVel (bid, 0.0, 0.0, 0.0);
						}
						if (setBody==2){
							unsigned int N = command.get(3).asInt();
							if ( (N >(i+1)) || (N < 1 )){reply.addString("object not known");}
							else{
								double x = command.get(4).asDouble();
								double y = command.get(5).asDouble();
								double z = command.get(6).asDouble();
								dBodySetPosition(odeinit._wrld->obj[N-1].boxbody,x,y,z);
								dBodySetLinearVel (odeinit._wrld->obj[N-1].boxbody, 0.0, 0.0, 0.0);
								dBodySetAngularVel (odeinit._wrld->obj[N-1].boxbody, 0.0, 0.0, 0.0);
							}
						}
						if (setBody==5){
							unsigned int N = command.get(3).asInt();
							if ((N >(i+1)) || (N < 1 )){reply.addString("object not known");}
							else{
								double x = command.get(4).asDouble();
								double y = command.get(5).asDouble();
								double z = command.get(6).asDouble();
								dGeomSetPosition(odeinit._wrld->s_obj[N-1].geom[0],x,y,z);
							}
						}
						if (setBody==6){
							unsigned int N = command.get(3).asInt();
							if ( (N >(i+1)) || (N < 1 )){reply.addString("object not known");}
							else{
								double x = command.get(4).asDouble();
								double y = command.get(5).asDouble();
								double z = command.get(6).asDouble();
								dBodySetPosition(odeinit._wrld->cyl_obj[N-1].cylbody,x,y,z);
								dBodySetLinearVel (odeinit._wrld->cyl_obj[N-1].cylbody, 0.0, 0.0, 0.0);
								dBodySetAngularVel (odeinit._wrld->cyl_obj[N-1].cylbody, 0.0, 0.0, 0.0);
							}
						}
						if (setBody==7){
							unsigned int N = command.get(3).asInt();
							if ((N >(i+1)) || (N < 1 )){reply.addString("object not known");}
							else{
								double x = command.get(4).asDouble();
								double y = command.get(5).asDouble();
								double z = command.get(6).asDouble();
								dGeomSetPosition(odeinit._wrld->s_cyl_obj[N-1].cylgeom[0],x,y,z);
							}
						}
						reply.fromString("ok");
					}
					if (subcmd=="mk"){ //this allows the user to create some boxes around the world		

						if (setBody==2){
						
							if (num < MAXNUM){
							i = odeinit._wrld->OBJNUM;
							odeinit._wrld->OBJNUM++;
						}
						    odeinit._wrld->obj[i].size[0] = command.get(3).asDouble();
							odeinit._wrld->obj[i].size[1] = command.get(4).asDouble();
							odeinit._wrld->obj[i].size[2] = command.get(5).asDouble();   
							double x = command.get(6).asDouble(); // x position 
							double y = command.get(7).asDouble(); // y position 
							double z = command.get(8).asDouble(); // z position
							double R = command.get(9).asDouble(); // colour R
							double G = command.get(10).asDouble();// colour G
							double B = command.get(11).asDouble();// colour B
							
							odeinit.mutex.wait();
							
							dMass m;
							dMassSetZero(&m);
							odeinit._wrld->obj[i].boxbody = dBodyCreate (odeinit.world);
							dMassSetBoxTotal (&m,DENSITY,odeinit._wrld->obj[i].size[0],odeinit._wrld->obj[i].size[1],odeinit._wrld->obj[i].size[2]);
							dBodySetMass (odeinit._wrld->obj[i].boxbody,&m);
							odeinit._wrld->obj[i].geom[0] = dCreateBox (odeinit.space,odeinit._wrld->obj[i].size[0],odeinit._wrld->obj[i].size[1],odeinit._wrld->obj[i].size[2]);
							dGeomSetBody (odeinit._wrld->obj[i].geom[0],odeinit._wrld->obj[i].boxbody);
							dBodySetPosition(odeinit._wrld->obj[i].boxbody,x,y,z);
							odeinit._wrld->color[i][0] = R;
							odeinit._wrld->color[i][1] = G;
							odeinit._wrld->color[i][2] = B;
							
							odeinit.mutex.post();

						}
						if (setBody==5){

							if (num < MAXNUM){
							i = odeinit._wrld->S_OBJNUM;
							odeinit._wrld->S_OBJNUM++;
						}
							odeinit._wrld->s_obj[i].size[0] = command.get(3).asDouble();
							odeinit._wrld->s_obj[i].size[1] = command.get(4).asDouble();
							odeinit._wrld->s_obj[i].size[2] = command.get(5).asDouble();   
							double x = command.get(6).asDouble();
							double y = command.get(7).asDouble();
							double z = command.get(8).asDouble();
							double R = command.get(9).asDouble();
							double G = command.get(10).asDouble();
							double B = command.get(11).asDouble();
							odeinit._wrld->s_obj[i].geom[0] = dCreateBox (odeinit.space,odeinit._wrld->s_obj[i].size[0],odeinit._wrld->s_obj[i].size[1],odeinit._wrld->s_obj[i].size[2]);
							dGeomSetPosition(odeinit._wrld->s_obj[i].geom[0],x,y,z);
							odeinit._wrld->s_color[i][0] = R;
							odeinit._wrld->s_color[i][1] = G;
							odeinit._wrld->s_color[i][2] = B;
						}
						if (setBody==6){
							//cyl with gravity
							if (num < MAXNUM){
								i = odeinit._wrld->cylOBJNUM;
								odeinit._wrld->cylOBJNUM++;
							}
							odeinit._wrld->cyl_obj[i].radius = command.get(3).asDouble();//radius
							odeinit._wrld->cyl_obj[i].lenght = command.get(4).asDouble();//lenght
							double x = command.get(5).asDouble(); // x position 
							double y = command.get(6).asDouble(); // y position 
							double z = command.get(7).asDouble(); // z position
							double R = command.get(8).asDouble(); // colour R
							double G = command.get(9).asDouble();// colour G
							double B = command.get(10).asDouble();// colour B

							odeinit.mutex.wait();
							
							dMass m;
							dMassSetZero(&m);
							odeinit._wrld->cyl_obj[i].cylbody = dBodyCreate (odeinit.world);
							dMassSetCylinderTotal (&m,DENSITY,3,odeinit._wrld->cyl_obj[i].radius,odeinit._wrld->cyl_obj[i].lenght);
							dBodySetMass (odeinit._wrld->cyl_obj[i].cylbody,&m);
							odeinit._wrld->cyl_obj[i].cylgeom[0] = dCreateCylinder(odeinit.space,odeinit._wrld->cyl_obj[i].radius,odeinit._wrld->cyl_obj[i].lenght);
							dGeomSetBody (odeinit._wrld->cyl_obj[i].cylgeom[0],odeinit._wrld->cyl_obj[i].cylbody);
							dBodySetPosition(odeinit._wrld->cyl_obj[i].cylbody,x,y,z);
							odeinit._wrld->color1[i][0] = R;
							odeinit._wrld->color1[i][1] = G;
							odeinit._wrld->color1[i][2] = B;
							
							odeinit.mutex.post();
						}
						if (setBody==7){
							if (num < MAXNUM){
								i = odeinit._wrld->S_cylOBJNUM;
								odeinit._wrld->S_cylOBJNUM++;
							}
							odeinit._wrld->s_cyl_obj[i].radius = command.get(3).asDouble();//radius
							odeinit._wrld->s_cyl_obj[i].lenght = command.get(4).asDouble();//lenght
							double x = command.get(5).asDouble(); // x position 
							double y = command.get(6).asDouble(); // y position 
							double z = command.get(7).asDouble(); // z position
							double R = command.get(8).asDouble(); // colour R
							double G = command.get(9).asDouble();// colour G
							double B = command.get(10).asDouble();// colour B

							odeinit._wrld->s_cyl_obj[i].cylgeom[0] = dCreateCylinder (odeinit.space,odeinit._wrld->s_cyl_obj[i].radius,odeinit._wrld->s_cyl_obj[i].lenght);
							dGeomSetPosition(odeinit._wrld->s_cyl_obj[i].cylgeom[0],x,y,z);
							odeinit._wrld->s_color1[i][0] = R;
							odeinit._wrld->s_color1[i][1] = G;
							odeinit._wrld->s_color1[i][2] = B;
						}

							reply.fromString("ok");
					}
					if (subcmd=="rot"){
						if ( setBody == 0 || setBody == 1 ){
					
						}
						if ( setBody == 2 ){
							unsigned int N = command.get(3).asInt();
							if ( (N >(i+1)) || (N < 1 )){reply.addString("object not known");}
							else{
								dMatrix3 Rtx,Rty,Rtz,Rtmp1,Rtmp2;
								double rotx = (command.get(4).asDouble() * M_PI) / 180;
								double roty = (command.get(5).asDouble() * M_PI) / 180;
								double rotz = (command.get(6).asDouble() * M_PI) / 180;

								dRFromAxisAndAngle(Rtx,1,0,0,rotx);
								dRFromAxisAndAngle(Rty,0,1,0,roty);
								dRFromAxisAndAngle(Rtz,0,0,1,rotz);

								dMultiply0 (Rtmp1,Rty,Rtz,3,3,3);
								dMultiply0 (Rtmp2,Rtx,Rtmp1,3,3,3);
								dGeomSetRotation(odeinit._wrld->obj[N-1].geom[0],Rtmp2);
							}
						}

						if ( setBody == 5 ){
							unsigned int N = command.get(3).asInt();
							if ( (N >(i+1)) || (N < 1 )){reply.addString("object not known");}
							else{
								dMatrix3 Rtx,Rty,Rtz, Rtmp1,Rtmp2;

								double rotx = (command.get(4).asDouble() * M_PI) / 180;
								double roty = (command.get(5).asDouble() * M_PI) / 180;
								double rotz = (command.get(6).asDouble() * M_PI) / 180;

								dRFromAxisAndAngle(Rtx,1,0,0,rotx);
								dRFromAxisAndAngle(Rty,0,1,0,roty);
								dRFromAxisAndAngle(Rtz,0,0,1,rotz);

								dMultiply0 (Rtmp1,Rty,Rtz,3,3,3);
								dMultiply0 (Rtmp2,Rtx,Rtmp1,3,3,3);
								dGeomSetRotation(odeinit._wrld->s_obj[N-1].geom[0],Rtmp2);
							}
						}

						if ( setBody == 6 ){
							unsigned int N = command.get(3).asInt();
							if ( (N >(i+1)) || (N < 1 )){reply.addString("object not known");}
							else{
								dMatrix3 Rtx,Rty,Rtz,Rtmp1,Rtmp2;
								double rotx = (command.get(4).asDouble() * M_PI) / 180;
								double roty = (command.get(5).asDouble() * M_PI) / 180;
								double rotz = (command.get(6).asDouble() * M_PI) / 180;

								dRFromAxisAndAngle(Rtx,1,0,0,rotx);
								dRFromAxisAndAngle(Rty,0,1,0,roty);
								dRFromAxisAndAngle(Rtz,0,0,1,rotz);

								dMultiply0 (Rtmp1,Rty,Rtz,3,3,3);
								dMultiply0 (Rtmp2,Rtx,Rtmp1,3,3,3);
								dGeomSetRotation(odeinit._wrld->cyl_obj[N-1].cylgeom[0],Rtmp2);
							}
						}

						if ( setBody == 7 ){
							unsigned int N = command.get(3).asInt();
							if ( (N >(i+1)) || (N < 1 )){reply.addString("object not known");}
							else{
								dMatrix3 Rtx,Rty,Rtz, Rtmp1,Rtmp2;

								double rotx = (command.get(4).asDouble() * M_PI) / 180;
								double roty = (command.get(5).asDouble() * M_PI) / 180;
								double rotz = (command.get(6).asDouble() * M_PI) / 180;

								dRFromAxisAndAngle(Rtx,1,0,0,rotx);
								dRFromAxisAndAngle(Rty,0,1,0,roty);
								dRFromAxisAndAngle(Rtz,0,0,1,rotz);

								dMultiply0 (Rtmp1,Rty,Rtz,3,3,3);
								dMultiply0 (Rtmp2,Rtx,Rtmp1,3,3,3);
								dGeomSetRotation(odeinit._wrld->s_cyl_obj[N-1].cylgeom[0],Rtmp2);
							}
						}

						reply.fromString("ok");
					
					}
					if (subcmd=="grab"){
						if (odeinit._iCub->actLHand=="off" || odeinit._iCub->actRHand=="off"){
							if (setBody == 0 || setBody == 1){
								ConstString hand = command.get(3).asString();
								int T = command.get(4).asInt();
								if (T == 1){
									if (hand == "left" && odeinit._iCub->actLHand=="off"){
										odeinit._iCub->grab = dJointCreateFixed(odeinit.world,0);
										dJointAttach (odeinit._iCub->grab,odeinit._iCub->l_hand,bid );
										dJointSetFixed(odeinit._iCub->grab);
									}
									else if (hand == "right" && odeinit._iCub->actRHand=="off"){
										odeinit._iCub->grab1 = dJointCreateFixed(odeinit.world,0);
										dJointAttach (odeinit._iCub->grab1,odeinit._iCub->r_hand,bid );
										dJointSetFixed(odeinit._iCub->grab1);
									}
									else {
										reply.addString("Hand not known");
									}
								}
								if (T == 0 && hand == "left" && odeinit._iCub->actLHand=="off"){
									dJointDestroy(odeinit._iCub->grab);
								}	
								if (T == 0 && hand == "right"&& odeinit._iCub->actRHand=="off"){
									dJointDestroy(odeinit._iCub->grab1);
								}								
							}
							if (setBody == 2){
								unsigned int N = command.get(3).asInt();
								if (N >(i+1)){reply.addString("object not known");}
								ConstString hand = command.get(4).asString();
								int T = command.get(5).asInt();
								if (T == 1){
									if (hand == "left" && odeinit._iCub->actLHand=="off"){
										odeinit._iCub->grab = dJointCreateFixed(odeinit.world,0);
										dJointAttach (odeinit._iCub->grab,odeinit._iCub->l_hand,odeinit._wrld->obj[N-1].boxbody );
										dJointSetFixed(odeinit._iCub->grab);
									}
									else if (hand == "right" && odeinit._iCub->actRHand=="off"){
										odeinit._iCub->grab1 = dJointCreateFixed(odeinit.world,0);
										dJointAttach (odeinit._iCub->grab1,odeinit._iCub->r_hand,odeinit._wrld->obj[N-1].boxbody );
										dJointSetFixed(odeinit._iCub->grab1);
									}
									else {
										reply.addString("Hand not known or selected one with full finger functionality");
									}
								}
								if (T == 0 && hand == "left" && odeinit._iCub->actLHand=="off"){
									dJointDestroy(odeinit._iCub->grab);
								}	
								if (T == 0 && hand == "right" && odeinit._iCub->actRHand=="off"){
									dJointDestroy(odeinit._iCub->grab1);
								}	
							}
						}
						else{
							reply.addString("Feature not available with finger functionality");
						}
						reply.fromString("ok");
					}						
				} else {
					reply.addString("object not known");
					reply.addVocab(id);
				}
				done = true;
			}
			done = true;
			ODE_access.post();
		}
        if (done) { return ok; }
        return ok;
    }

    void sendImage(BufferedPort<ImageOf<PixelRgb> >& port);
};


static SimulatorModule *simulatorModule = NULL;

void sendTouch(Bottle& report) {
	simulatorModule->sendTouch(report);
}

bool shouldSendTouch() {
	return simulatorModule->shouldSendTouch();
}


void sendVision(){
simulatorModule->displayStep(0);
}

static void displayStep(int pause) {
    simulatorModule->displayStep(pause);
}

void SimulatorModule::init()
{
	Property options;
  	//start left arm device driver
	ConstString left_arm = finder.find("left_arm");
	options.fromConfigFile(left_arm.c_str());
  	iCubLArm = new PolyDriver(options);

  	//start right arm device driver
  	ConstString right_arm = finder.find("right_arm");
	options.fromConfigFile(right_arm.c_str());
  	iCubRArm = new PolyDriver(options);

  	//start head device driver
  	ConstString head = finder.find("head");
	options.fromConfigFile(head.c_str());
  	iCubHead = new PolyDriver(options);

  	//start left leg device driver
 	ConstString left_leg = finder.find("left_leg");
	options.fromConfigFile(left_leg.c_str());
  	iCubLLeg = new PolyDriver(options);

   	//start right leg device driver
 	ConstString right_leg = finder.find("right_leg");
	options.fromConfigFile(right_leg.c_str());
  	iCubRLeg = new PolyDriver(options);

    //start torso device driver
  	ConstString torso = finder.find("torso");
	options.fromConfigFile(torso.c_str());
  	iCubTorso = new PolyDriver(options);

  if(!iCubLArm->isValid() || !iCubRArm->isValid() || !iCubHead->isValid() || !iCubLLeg->isValid() || !iCubRLeg->isValid() || !iCubTorso->isValid())
    {
      ACE_OS::printf("Device not available. Here are the known devices:\n");
	  ACE_OS::printf("%s", Drivers::factory().toString().c_str());
      Network::fini();
      exit(1);
    }

}

void SimulatorModule::initImagePorts() {
    Property options;

	ConstString cameras = finder.find("cameras");
	options.fromConfigFile(cameras.c_str());
   	//readConfig(options,"conf/Sim_camera.ini");
    
    ConstString nameExternal = 
        options.check("name_wide",
                      Value("/icubSim/cam"),
                      "Name of external view camera port").asString();
    
    ConstString nameLeft = 
        options.check("name_left",
                      Value("/icubSim/cam/left"),
                      "Name of left camera port").asString();
    
    ConstString nameRight = 
        options.check("name_right",
                      Value("/icubSim/cam/right"),
                      "Name of right camera port").asString();

    portLeft.open(nameLeft);
    portRight.open(nameRight);
    portWide.open(nameExternal);
}


bool SimulatorModule::open() {
    
	cmdPort.setReader(*this);
	cmdPort.open("/icubSim/world");
	tactilePort.open("/icubSim/touch");

    if (odeinit._iCub->actVision=="on") {
        initImagePorts();
    }
	 init();
	 sim = new Simulation();
	 w = 480;
     h = 620;
    return true;
}

bool SimulatorModule::runModule() {

    Time::delay(1); // there's some clash over keyboard on Windows
	Simulation::simLoop(h,w);
    return true;
}

void SimulatorModule::displayStep(int pause) {
    bool needLeft = (portLeft.getOutputCount()>0);// || viewParam1;
    bool needRight = (portRight.getOutputCount()>0);// || viewParam2;
    bool needWide = (portWide.getOutputCount()>0);// || (!(viewParam1 || viewParam2));

    mutex.wait();
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
				sendImage(portLeft);
				sim->clearBuffer();
            }
            break;
        case 'r':
			if (needRight) {
				sim->drawView(false,true,false);
				sendImage(portRight);
				sim->clearBuffer();
            }
            break;
        case 'w':
            if (needWide) {
				sim->drawView(false,false,true);
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
}

void SimulatorModule::sendImage(BufferedPort<ImageOf<PixelRgb> >& port) {
    int w = 320;
    int h = 240;
    int p = 3;//320 240

	char buf[320*240*3];
    
    glReadPixels(0,0,w,h,GL_RGB,GL_UNSIGNED_BYTE,buf);
	static ImageOf<PixelRgb> img;
    img.setQuantum(1);
	img.setExternal(buf,w,h);
    
    // inefficient flip
    ImageOf<PixelRgb>& target = port.prepare();
    target.resize(img);
    IMGFOR(target,x,y) {
        target(x,y) = img(x,img.height()-1-y);
    }
    port.write();
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT); // refresh opengl
}

class MyNetwork
{
public:
    MyNetwork()
    {
        Network::init();
    }
        
        ~MyNetwork()
        {
            Network::fini();
            fprintf(stderr, "Closing network\n");
        }
};

int main( int argc, char** argv) 
{		
    MyNetwork yarp;
    SimConfig finder;
		
    finder.configure(argc, argv);

	odeinit._wrld->OBJNUM = 0;
	odeinit._wrld->waitOBJ = 0;
	odeinit._wrld->S_OBJNUM = 0;
		
	odeinit._wrld->cylOBJNUM = 0;
	odeinit._wrld->waitOBJ1 = 0;
	odeinit._wrld->S_cylOBJNUM = 0;

	dInitODE(); 

    Drivers::factory().add(new DriverCreatorOf<iCubSimulationControl>("simulationcontrol", 
        "controlboard",
        "iCubSimulationControl"));

    if (!Network::checkNetwork()) {
        printf("Please start a yarp name server first\n");
        exit(1);
    }

    SimulatorModule module;
    simulatorModule = &module;

    module.open();

    // this blocks until termination (through ctrl+c or a kill)
    module.runModule();

    module.closeModule();

    delete _odeinit;

    dCloseODE();

    return 0;
} 
