// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
* Copyright (C) 2007 Vadim Tikhanoff, Paul Fitzpatrick
* CopyPolicy: Released under the terms of the GNU GPL v2.0. 
*
*/
/**
 * \file iCub_Sim.h
 * \brief This class controls the simulation speed using dWorldstep for "exact" calculations, the collisions between objects/spaces and the rendering functions. It also deals with separating the physics calsulations from the rendering 
 * \author Vadim Tikhanoff, Paul Fitzpatrick
 * \date 2007
 * \note Release under GNU GPL v2.0
 **/
//#pragma once
#ifndef ICUB_SIMH
#define ICUB_SIMH

#include <yarp/os/Os.h>

#include "SDL_thread.h"
#include "SDL.h"
#include "SDL_timer.h"
#include "SDL_opengl.h"
#include "rendering.h"
#include <ode/ode.h>
#include <assert.h>
#include "OdeInit.h"
#include "iCub.h"
#include <stdio.h>
#include "world.h"
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>
#include <yarp/os/Time.h>
#include "pidfilter.h"
#include <time.h>
#include "SimConfig.h"
#include <signal.h>

#include "VideoTexture.h"

int stop = 0;
int v = 0;

static float xyz[3];
static float hpr[8];
static float rez[3];

//extern OdeInit odeinit;
extern OdeInit& getOdeInit();
#define odeinit (getOdeInit())

int contactPoint;
int mouseDiffx, mouseDiffy;
bool picking = false;
float cam_rx = 0.0, cam_ry = 0.0;

int width = 640;
int height = 480;

int mouse0_down_x, mouse1_down_x;
int mouse0_down_y, mouse1_down_y;
int mouse_ray_x;
int mouse_ray_y;
static float *VAD;
static float *VAD2;
const dReal *pos;
float angle_xref = 0.0f;
float angle_yref = 25.0f;
float ydistance = 10.0f;
float xdistance = 0.0f;
static float view_xyz[3];	// position x,y,z
static float view_hpr[3];	// heading, pitch, roll (degrees)
float view2_xyz[3];
float view2_hpr[3];
float zoom = 0;
//angle of rotation
float xpos = 0, ypos = 0, zpos = 0, xrot = 0, yrot = 0, angle=0.0;
float lastx, lasty;
float xrotrad, yrotrad;
clock_t startTime, finishTime;
double duration, frames, FPS,seconds, TimestepManager;
static float test[3];
SDL_TimerID id;
Uint32          colorkey;
SDL_Surface     *image;

bool extractImages = false;
extern bool viewParam1,viewParam2;
//bool viewParam2 = false;

extern void sendTouch(Bottle& report);
extern bool shouldSendTouch();
extern void sendInertial(Bottle& report);
extern bool shouldSendInertial();
static Semaphore ODE_access(1);
extern void sendVision();

static bool eyeCams;
const GLfloat light_position[] = { 0.0f, 5.0f, 5.0f, 0.0f };

class Simulation{
    static VideoTexture *video;

public:
        
	static void draw(){
		odeinit._iCub->draw();
		odeinit._wrld->draw();
	}

	static void setJointTorques(){
		odeinit._iCub->setJointTorques();
	}
	static void setJointSpeed(){
		odeinit._iCub->setJointSpeeds();

	}

	static void printStats(){

		finishTime = clock() ;
		duration += (double)(finishTime - startTime) / CLOCKS_PER_SEC ;
		frames ++ ;
		FPS = frames / duration ;
		startTime = clock() ;
		odeinit.SimTime = duration;
		//printf("duration: %.2lf\n",odeinit.SimTime);
		static double starting_time_stamp = 0;
		//		test[0] = dBodyGetPosition(odeinit._iCub->body_cube[0])[0];
		//		test[1] = dBodyGetPosition(odeinit._iCub->body_cube[0])[1];
		//		test[2] = dBodyGetPosition(odeinit._iCub->body_cube[0])[2];
		//		printf("test[0] %f  test[1] %f  test[2] %f\n",test[0],test[1],test[2]);
		if( duration - starting_time_stamp >= 1){
			//printf("Frames: %.2lf   Duration: %.2lf   fps: %3.1f \n",frames,duration,FPS);
			starting_time_stamp = duration;
		}
		//printf("%lf %lf %lf %lf %lf %lf\n", odeinit._iCub->ra_speed[0],odeinit._iCub->ra_speed[1],odeinit._iCub->ra_speed[2],odeinit._iCub->ra_speed[3],odeinit._iCub->ra_speed[4],odeinit._iCub->ra_speed[5]);
		//drawText(text, textPos);
	}

	static void quit(int code)
	{
		SDL_Quit();
		yarp::os::exit(code);
	}
	static void handle_key_down(SDL_keysym* keysym)
	{
		switch (keysym->sym)
		{
		case SDLK_e:
			break;
		case SDLK_r:
			break;
		case SDLK_t:
			break;
		case SDLK_y:
			break;
		default:
			break;
		}
	}
	static void handle_mouse_motion(SDL_MouseMotionEvent* mousemotion)
	{
		if (SDL_GetMouseState(NULL, NULL) & SDL_BUTTON(1)){// MOUSE LEFT BUTTON
			//if (!picking){
			//camera movement
			angle_xref += (float)mousemotion->xrel; // 10.0f;
			angle_yref += (float)mousemotion->yrel; // 10.0f;
			mouseMovement(angle_xref,angle_yref);

			if (v<1){
				//mouse_ray_x = mouse0_down_x; 
				//mouse_ray_y = mouse0_down_y;
			}
			/*mouseDiffx = mouse0_down_x - mouse_ray_x;
			mouseDiffy = mouse0_down_y - mouse_ray_y;
			mouse_ray_x = mouse0_down_x;
			mouse_ray_y = mouse0_down_y;*/

			//VAD = ScreenToWorld(mouse0_down_x,mouse0_down_y,0);
			//xpos = VAD[0];ypos = VAD[1];zpos = VAD[2];
			//VAD2 =ScreenToWorld(mouse0_down_x,mouse0_down_y,1);
			//xpos2 = VAD2[0];ypos2 = VAD2[1];zpos2 = VAD2[2];

			//if (i<1){ray = dCreateRay(space,100*100);}
			//Origin[0] = xpos;
			//Origin[1] = ypos;
			//Origin[2] = zpos;
			//Origin[3] = 0;
			//Direction[0] = xpos2;
			//Direction[1] = ypos2;
			//Direction[2] = zpos2;
			//Direction[3] = 0;
			//dGeomRaySet(ray, Origin[0], Origin[1], Origin[2], Direction[0], Direction[1], Direction[2]);
			//dGeomSetPosition(ray, xpos,ypos,zpos);
			//i++;
		}
		if (SDL_GetMouseState(NULL, NULL) & SDL_BUTTON(3)){// MOUSE RIGHT BUTTON

			//xdistance -= mousemotion->xrel / 10.0f;
			//ydistance -= mousemotion->yrel / 10.0f;	
		}
	}

	static void process_events(void){
		SDL_Event event;

		Uint8 * keystate = SDL_GetKeyState(NULL);
		if(keystate[SDLK_q]){xrot += 1 * 0.1f;if (xrot >360) xrot -= 360 * 0.1f;}
		if(keystate[SDLK_z]){xrot -= 1 * 0.1f;if (xrot < -360) xrot += 360 * 0.1f;}
		if(keystate[SDLK_w]){yrotrad = (yrot / 180 * 3.141592654f); xrotrad = (xrot / 180 * 3.141592654f); 
		xpos += float(sin(yrotrad))* 0.005f; ;zpos -= float(cos(yrotrad))* 0.005f; ypos -= float(sin(xrotrad))* 0.005f;}
		if(keystate[SDLK_s]){yrotrad = (yrot / 180 * 3.141592654f); xrotrad = (xrot / 180 * 3.141592654f); 
		xpos -= float(sin(yrotrad))* 0.005f;zpos += float(cos(yrotrad))* 0.005f; ;ypos += float(sin(xrotrad))* 0.005f;}
		if (keystate[SDLK_a]){yrotrad = (yrot / 180 * 3.141592654f);xpos -= float(cos(yrotrad)) * 0.008;zpos -= float(sin(yrotrad)) * 0.008; }
		if (keystate[SDLK_d]){yrotrad = (yrot / 180 * 3.141592654f);xpos += float(cos(yrotrad)) * 0.008;zpos += float(sin(yrotrad)) * 0.008;}

		if(keystate[SDLK_1]){initViewpoint();}
		
		if (keystate[SDLK_5]){

			if ((odeinit._iCub->eyeLidRot) < 0.55) odeinit._iCub->eyeLidRot += 0.01;
			cout<<odeinit._iCub->eyeLidRot<<endl;
		}
		if (keystate[SDLK_6]){
			if ((odeinit._iCub->eyeLidRot) > 0.01) odeinit._iCub->eyeLidRot -= 0.01;
			cout<<odeinit._iCub->eyeLidRot<<endl;
		}

		/* Grab all the events off the queue. */
		while (SDL_PollEvent(&event)){
			switch (event.type)
			{
			case SDL_VIDEORESIZE:
				width = event.resize.w;
				height = event.resize.h;
				SDL_SetVideoMode(width,height,16,SDL_OPENGL | SDL_RESIZABLE);
				setup_opengl();
				odeinit._iCub->reinitialized = true;
				//draw_screen( );
				break;
			case SDL_KEYDOWN:
				/* Handle key presses*/
				handle_key_down(&event.key.keysym);
				// SDL_GetKeyName(event.key.keysym.sym));
				break;
				break;
			case SDL_MOUSEMOTION:
				handle_mouse_motion(&event.motion);
				mouse0_down_x = event.button.x;
				mouse0_down_y = event.button.y;	
				break;
			case SDL_QUIT:
				/* Handle quit requests (like Ctrl-c). */
					odeinit.stop = true;
				break;

			case SDL_MOUSEBUTTONDOWN:
				handle_mouse_motion(&event.motion);
				switch (event.button.button)
				{
				case SDL_BUTTON_LEFT:
					//deleteRay = false;
					picking = false;
					//printf(" Down\n");
					break;
				case SDL_BUTTON_MIDDLE:
					break;
				case SDL_BUTTON_RIGHT:
					break;
				default:
					//this is not reached
					break;
				}
				break;
				break;
			case SDL_MOUSEBUTTONUP:
				switch (event.button.button)
				{
				case SDL_BUTTON_LEFT:
					//printf(" up\n");
					v=0;
					break;
				case SDL_BUTTON_MIDDLE:
					//nothing
					break;
				case SDL_BUTTON_RIGHT:
					//nothing
					break;
				default:
					//this is not reached either
					break;
				}
				break;
			}
		}
	}
static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{

assert(o1);
		assert(o2);
		if (dGeomIsSpace(o1) || dGeomIsSpace(o2)){
			// colliding a space with something
			dSpaceCollide2(o1,o2,data,&nearCallback);
			// Note we do not want to test intersections within a space,
			// only between spaces.
			return;
		}
  int i;
  // if (o1->body && o2->body) return;

  // exit without doing anything if the two bodies are connected by a joint
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if (b1 && b2 && dAreConnectedExcluding (b1,b2,dJointTypeContact)) return;

  dContact contact[MAX_CONTACTS];   // up to MAX_CONTACTS contacts per box-box
  for (i=0; i<MAX_CONTACTS; i++) {
    contact[i].surface.mode = dContactSlip1| dContactSlip2| dContactBounce | dContactSoftCFM;
    contact[i].surface.mu = dInfinity;
    contact[i].surface.mu2 = 0;
    contact[i].surface.bounce = 0.1;
    contact[i].surface.bounce_vel = 0.1;
    contact[i].surface.slip1 = (dReal)0.0006;
	contact[i].surface.slip2 = (dReal)0.0006;
	contact[i].surface.soft_cfm = 0.0001;
  }
  if (int numc = dCollide (o1,o2,MAX_CONTACTS,&contact[0].geom,
			   sizeof(dContact))) {
    dMatrix3 RI;
    dRSetIdentity (RI);
    for (i=0; i<numc; i++) {
      dJointID c = dJointCreateContact (odeinit.world,odeinit.contactgroup,contact+i);
      dJointAttach (c,b1,b2);
     // if (show_contacts) dsDrawBox (contact[i].geom.pos,RI,ss);
    }
  }
}
	/*static void nearCallback (void *data, dGeomID o1, dGeomID o2){
		assert(o1);
		assert(o2);
		if (dGeomIsSpace(o1) || dGeomIsSpace(o2)){
			// colliding a space with something
			dSpaceCollide2(o1,o2,data,&nearCallback);
			// Note we do not want to test intersections within a space,
			// only between spaces.
			return;
		}
		int i;
		// exit without doing anything if the two bodies are connected by a joint
		dBodyID b1 = dGeomGetBody(o1);
		dBodyID b2 = dGeomGetBody(o2);
		if (b1 && b2 && dAreConnectedExcluding (b1,b2,dJointTypeContact)) {printf("testing space %p %p\n", b1,b1); return;}

		const int N = 5;
		dContact contact[N];   // up to MAX_CONTACTS
		for (i=0; i<N; i++) {
			contact[i].surface.mode = 0;//dContactSlip1| dContactSlip2| dContactApprox1;
			contact[i].surface.mu = dInfinity;
			//contact[i].surface.mu2 = 0;
			//contact[i].surface.slip1 = (dReal)0.0001;
			//contact[i].surface.slip2 = (dReal)0.0001;
			//contact[i].surface.soft_cfm = 0.1;
		}
		if (int numc = dCollide (o1,o2,5,&contact[0].geom,
			sizeof(dContact))) {
				/*dMatrix3 RI;
				dRSetIdentity (RI);
				for (i=0; i<numc; i++) {
					dJointID c = dJointCreateContact (odeinit.world,odeinit.contactgroup,contact+i);
					dJointAttach (c,b1,b2);
					//dsSetColor (1.0,0.0,0.0); dsDrawBox (contact[i].geom.pos,RI,ss);
				}
		}
	}
*/
	static void inspectBodyTouch(Bottle& report){
		report.clear();
		if (odeinit._iCub->actLHand == "on" && odeinit._iCub->actRHand == "on" ){
			const char *names[] = {
				"lpam","rpam",
				"lind","lmid","lrng","llit","lthm",
				"rind","rmid","rrng","rlit","rthm",
				NULL
			};
			int nameIndex = 0;
			for (int x = 0; x<50; x++){
				//selected body parts for touch sensor left and right arm
				if (x == 10 || x == 11 || x == 30 || x == 24 || x == 25 || x == 26 || x == 27 || x == 49 || x == 43 || x == 44 || x == 45 || x ==  46){
					bool result = odeinit._iCub->checkTouchSensor(x);
					const char *name = names[nameIndex];
					nameIndex++;
					if (name==NULL) {printf("sensor list out of date!\n");yarp::os::exit(1);}
					report.addVocab(Vocab::encode(name));
					report.addInt(result?1:0);
				}
			}
		}
		else if (odeinit._iCub->actLHand == "on" && odeinit._iCub->actRHand == "off" ){
			const char *names[] = {
				"lpam",
				"lind","lmid","lrng","llit","lthm",
				"rhand",
				NULL
			};
			int nameIndex = 0;
			for (int i = 0; i<2; i++){
				if (i == 0){
					for (int x = 0; x<31; x++){
						if (x == 10 || x == 30 || x == 24 || x == 25 || x == 26 || x == 27){
							bool result = odeinit._iCub->checkTouchSensor(x);
							const char *name = names[nameIndex];
							nameIndex++;
							if (name==NULL) {
								printf("sensor list out of date!\n");
								yarp::os::exit(1);
							}
							report.addVocab(Vocab::encode(name));
							report.addInt(result?1:0);
						}
					}
				}else{
					bool result = odeinit._iCub->checkTouchSensor(odeinit._iCub->r_hand);
					const char *name = names[nameIndex];nameIndex++;
					if (name==NULL) {
						printf("sensor list out of date!\n");
						yarp::os::exit(1);}
					report.addVocab(Vocab::encode(name));
					report.addInt(result?1:0);
				}
			}
		}
		else if (odeinit._iCub->actRHand == "on" && odeinit._iCub->actLHand == "off"  ){
			const char *names[] = {
				"rpam",
				"rind","rmid","rrng","rlit","rthm",
				"lhand",
				NULL
			};
			int nameIndex = 0;
			for (int i = 0; i<2; i++){
				if (i == 0){
					for (int x = 0; x<50; x++){
						if (x == 11 || x == 49 || x == 43 || x == 44 || x == 45 || x ==  46){
							bool result = odeinit._iCub->checkTouchSensor(x);
							const char *name = names[nameIndex];
							nameIndex++;
							if (name==NULL) {
                                printf("sensor list out of date!\n");
                                yarp::os::exit(1);}
							report.addVocab(Vocab::encode(name));
							report.addInt(result?1:0);
						}
					}
				}else{
					bool result = odeinit._iCub->checkTouchSensor(odeinit._iCub->l_hand);
					const char *name = names[nameIndex];
					nameIndex++;
					if (name==NULL) {
                        printf("sensor list out of date!\n");
                        yarp::os::exit(1);
                    }
					report.addVocab(Vocab::encode(name));
					report.addInt(result?1:0);
				}
			}
		}
		else{ //both off
			const char *names[] = {
				"lhand","rhand",
				NULL
			};
			int nameIndex = 0;
			for (int i = 0; i<2; i++){
				if (i == 0){
					bool result = odeinit._iCub->checkTouchSensor(odeinit._iCub->l_hand);
					const char *name = names[nameIndex];nameIndex++;
					if (name==NULL) {
                        printf("sensor list out of date!\n");
                        yarp::os::exit(1);
                    }
					report.addVocab(Vocab::encode(name));
					report.addInt(result?1:0);
				}
				else{
					bool result = odeinit._iCub->checkTouchSensor(odeinit._iCub->r_hand);
					const char *name = names[nameIndex];nameIndex++;
					if (name==NULL) {
                        printf("sensor list out of date!\n");
                        yarp::os::exit(1);
                    }
					report.addVocab(Vocab::encode(name));
					report.addInt(result?1:0);
				}
			}
		}
	}

	static void getAngles(const dReal *m, float& z, float& y, float& x) {
		const dReal eps = 0.00001;

		y = -asin(m[2]);
		float c = cos(y);

		if (fabs(c)>eps) {
			x = atan2(-m[6]/c,m[10]/c);
			z = atan2(-m[1]/c,m[0]/c);
		} else {
			x = 0;
			z = -atan2(m[4],m[5]);
		}
		x *= -180/M_PI;
		y *= 180/M_PI;
		z *= 180/M_PI;
	}

	static void initViewpoint(){
		xpos = 0;
		ypos = 1;
		zpos = 1;
		xrot = 25;
		yrot = 0;
		//zrot = 0;
	}
	static void mouseMovement(float x, float y) {
		float diffx = x-lastx; //check the difference between the current x and the last x position
		float diffy = y-lasty; //check the difference between the current y and the last y position
		lastx =x; //set lastx to the current x position
		lasty =y; //set lasty to the current y position
		xrot += (float) diffy; //set the xrot to xrot with the addition of the difference in the y position
		yrot += (float) diffx;	//set the xrot to yrot with the addition of the difference in the x position
	}

	static void draw_screen(){
        
		static clock_t startTimeODE= clock(), finishTimeODE= clock();
		glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT); // refresh opengl

		if (extractImages || odeinit._iCub->actVision == "on"){
			sendVision();
		}
		
		glViewport(0,0,width,height);
		glMatrixMode (GL_PROJECTION);
		glLoadIdentity();
		gluPerspective( 75, width/height, 0.01, 100.0 );
		glMatrixMode (GL_MODELVIEW);
		glLoadIdentity();
		glLightfv(GL_LIGHT0, GL_POSITION, light_position);	
		glRotatef (xrot, 1,0,0);
		glRotatef (yrot, 0,1,0);
		glTranslated(-xpos,-ypos,-zpos);

        // set up any video textures
       
        if (video!=0)
            DrawVideo(video);

		//draw the ground
		glColor3d(0.5,0.5,1);
		glPushMatrix();
		glRotatef(90.0,1,0,0);
		glRotatef(180.0,0,1,0);
		DrawGround(false);     
		glPopMatrix();
		glDisable(GL_TEXTURE_2D);
		draw();
		glEnable(GL_TEXTURE_2D);
		drawSkyDome(0,0,0,50,50,50); // Draw the Skybox

		SDL_GL_SwapBuffers();// Swap Buffers
	}

	void drawView(bool left, bool right, bool wide){
		const dReal *pos;
		const dReal *rot;
		glViewport(0,0,320,240);
		glMatrixMode (GL_PROJECTION);
		glLoadIdentity();
		gluPerspective( 75, 320/240, 0.04, 100.0 );
	
		if (left){
			pos = dGeomGetPosition(odeinit._iCub->Leye1_geom);
			rot = dGeomGetRotation(odeinit._iCub->Leye1_geom);
			float zoomFactor = 51.0f;//51 with gluPerspective 0.1
			zoom = 0.023;
			glMatrixMode (GL_MODELVIEW);
			glLoadIdentity();
			glLightfv(GL_LIGHT0, GL_POSITION, light_position);
			gluLookAt(
				pos[0]  - rot[2]/ zoomFactor,
				pos[1]  - rot[6]/ zoomFactor,
				(pos[2]+zoom) - rot[10]/ zoomFactor,
				pos[0],
				pos[1],
				(pos[2]+zoom),
				-rot[4], 1, 0
				);
		}
		if (right){
			pos = dGeomGetPosition(odeinit._iCub->Reye1_geom);
			rot = dGeomGetRotation(odeinit._iCub->Reye1_geom);
			float zoomFactor = 51.0f;//51 with gluPerspective 0.1
			zoom = 0.023;
			glMatrixMode (GL_MODELVIEW);
			glLoadIdentity();
			glLightfv(GL_LIGHT0, GL_POSITION, light_position);
			gluLookAt(
				pos[0]  - rot[2]/ zoomFactor,
				pos[1]  - rot[6]/ zoomFactor,
				(pos[2]+zoom) - rot[10]/ zoomFactor,
				pos[0],
				pos[1],
				(pos[2]+zoom),
				-rot[4], 1, 0
				);
		}	
		if (wide){
			glMatrixMode (GL_MODELVIEW);
			glLoadIdentity();
			glLightfv(GL_LIGHT0, GL_POSITION, light_position);
			glRotatef (xrot, 1,0,0);
			glRotatef (yrot, 0,1,0);
			glTranslated(-xpos,-ypos,-zpos);
		}

		//draw the ground
		glColor3d(0.5,0.5,1);
		glEnable(GL_TEXTURE_2D);
		glPushMatrix();
		glRotatef(90.0,1,0,0);
		glRotatef(180.0,0,1,0);
		DrawGround(false);     
		glPopMatrix();
		glDisable(GL_TEXTURE_2D);
		draw();
		glEnable(GL_TEXTURE_2D);
		drawSkyDome(0,0,0,50,50,50); // Draw the Skybox	
	}

    void clearBuffer(){
		glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT); // refresh opengl
	}

    static void retreiveInertialData(Bottle& inertialReport){
        static dReal OldLinearVel[3], LinearVel[3], LinearAccel[3], roll, pitch, yaw;
        inertialReport.clear();
        //this prepares a bottle of 12 doubles to be sent on the /icubSim/inertial port
        
        roll = dBodyGetRotation(odeinit._iCub->head)[1] ;
        pitch = dBodyGetRotation(odeinit._iCub->head)[6] ;
        yaw = dBodyGetRotation(odeinit._iCub->head)[2] ;
        //Add Euler angles roll pitch yaw
        inertialReport.addDouble( roll * 180/M_PI); // roll 
        inertialReport.addDouble( pitch * 180/M_PI); // pitch
        inertialReport.addDouble( yaw * 180/M_PI); // yaw
        		
        //in order to calculate linear acceleration (make sure of body) Inertial Measurement Unit IMU
		LinearVel[0] = dBodyGetLinearVel(odeinit._iCub->head)[0];
		LinearVel[1] = dBodyGetLinearVel(odeinit._iCub->head)[1];
		LinearVel[2] = dBodyGetLinearVel(odeinit._iCub->head)[2];
		//// a = dv/dt = ( t - t_old ) / dt
		LinearAccel[0] = ( LinearVel[0] - OldLinearVel[0] ) / 0.02;
		LinearAccel[1] = ( LinearVel[1] - OldLinearVel[1] ) / 0.02;
		LinearAccel[2] = ( LinearVel[2] - OldLinearVel[2] ) / 0.02;
		OldLinearVel[0] = LinearVel[0];
		OldLinearVel[1] = LinearVel[1];
		OldLinearVel[2] = LinearVel[2];
        
		////Add linear acceleration
        inertialReport.addDouble( -(sin(pitch) * cos(roll)) * 9.8 );
        inertialReport.addDouble( (sin(roll)) * 9.8 );
        inertialReport.addDouble( (cos(pitch) * cos(roll)) * 9.8 );
		
        //Add angular velocity
        inertialReport.addDouble(-dBodyGetAngularVel(odeinit._iCub->head)[2] / 10);
        inertialReport.addDouble(-dBodyGetAngularVel(odeinit._iCub->head)[0] / 10);
        inertialReport.addDouble( dBodyGetAngularVel(odeinit._iCub->head)[1] / 10);
        
        //Add magnetic fields
        inertialReport.addDouble(0.0);
        inertialReport.addDouble(0.0);
        inertialReport.addDouble(0.0);

    }

	static Uint32 ODE_process(Uint32 interval, void *param){
		//static clock_t startTimeODE= clock(), finishTimeODE= clock();
		//startTimeODE = clock();
		
		odeinit.mutex.wait();
		dSpaceCollide(odeinit.space,0,&nearCallback);
		dWorldStep(odeinit.world, 0.01); // TIMESTEP
        odeinit.sync = true;
		odeinit.mutex.post();

		if (shouldSendTouch()) {
			Bottle report;
			inspectBodyTouch(report);
			sendTouch(report);
		}

		dJointGroupEmpty (odeinit.contactgroup);

        if (shouldSendInertial()) {
            Bottle inertialReport;
            retreiveInertialData(inertialReport);
            sendInertial(inertialReport);
        }
		setJointSpeed();
		//setJointTorques();
		//finishTimeODE = clock() ;
		//SPS();
		//printf("ODE=%lf\n",(double)(finishTimeODE - startTimeODE) / CLOCKS_PER_SEC);
		return(interval);
	}
	static int thread_func(void *unused)
	{
        // this needs to be kept synchronized with the timestep in
        // dWorldStep, in order to get correct world clock time
        //  --paulfitz
		int delay = 50;
		id = SDL_AddTimer( delay, &Simulation::ODE_process, (void*)1);

		return(0);
	}
	/*
	static void SPS()     
	{
	    static float sps           = 0.0f;      
	    static float previousTime  = 0.0f; 
	    static int currentsps; 
	    static char  strSPS[60]    = {0};

	    float currentTime = (GetTickCount() * 0.001f);    

	    ++sps; // Increment the SPS counter

	if( currentTime - previousTime > 1.0f ){
	    previousTime = currentTime;
	    currentsps = int(sps);
	    printf("current SPS: %d\n",currentsps);
	    sps = 0.0f;
	    }
	}
	*/

	static void sighandler(int sig){
		odeinit.stop = true;
		cout << "\nCAUGHT Ctrl-c" << endl;
	}	

	static void simLoop(int h,int w){        

		SDL_Init(SDL_INIT_TIMER | SDL_GL_ACCELERATED_VISUAL);
		SDL_SetVideoMode(h,w,32,SDL_OPENGL | SDL_RESIZABLE);// | SDL_SWSURFACE| SDL_ANYFORMAT); // on init 
		SimConfig finder;

		dAllocateODEDataForThread(dAllocateMaskAll);
		ConstString logo = finder.find("logo");

		image = SDL_LoadBMP(finder.find(logo.c_str()));
		SDL_WM_SetIcon(image,0);
		SDL_FreeSurface(image);
		SDL_WM_SetCaption("iCub Simulator", "image");

		SDL_Thread *thread;

		thread = SDL_CreateThread(thread_func, NULL);

		if ( thread == NULL ) {
			fprintf(stderr, "Unable to create thread: %s\n", SDL_GetError());
			return;
		}

		initViewpoint();
		setup_opengl();
		startTime = clock();
		odeinit.stop = false;
		
		yarp::os::signal(yarp::os::YARP_SIGINT, sighandler);
		yarp::os::signal(yarp::os::YARP_SIGTERM, sighandler);

		odeinit._wrld->WAITLOADING = false;
		odeinit._wrld->static_model = false;
		while(!odeinit.stop)
		{
			/* Process incoming events. */
			process_events();
			/* Draw the screen. */
			if ( !odeinit._wrld->WAITLOADING ){
				odeinit.mutexTexture.wait();
				draw_screen();  
				odeinit.mutexTexture.post();
			}
			else{
				glFinish();
				glFlush();
				//make sure it can also be done for static objects
				if (odeinit._wrld->static_model){
					odeinit._wrld->loadTexture(odeinit._wrld->texture, odeinit._wrld->s_modelTexture[odeinit._wrld->s_MODEL_NUM-1]);
				}else{
					odeinit._wrld->loadTexture(odeinit._wrld->texture, odeinit._wrld->modelTexture[odeinit._wrld->MODEL_NUM-1]);
				}
				odeinit._wrld->WAITLOADING = false;	
				odeinit._wrld->static_model = false;	
			}
		}     
        printf("\n\nStopping SDL and ODE threads...\n");
		//stop the timer
		SDL_RemoveTimer(id);
		//Stop the thread
		SDL_KillThread( thread );
	}
	
    Simulation(){
        video=new VideoTexture;
    
        Property options;
		
		SimConfig finder;
		ConstString videoconf = finder.find("video");
        
		options.fromConfigFile(videoconf.c_str());

        Bottle textures = options.findGroup("textures").tail();
        for (int i=0; i<textures.size(); i++) {
            ConstString name = textures.get(i).asString();
            printf("Adding video texture %s\n", name.c_str());
            video->add(options.findGroup(name.c_str()));
		}
  	}

    ~Simulation()
    {
        delete video;
    }
};

#endif

