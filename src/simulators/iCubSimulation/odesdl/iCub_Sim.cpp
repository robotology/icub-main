// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Paul Fitzpatrick, Vadim Tikhanoff, Martin Peniak
* email:   paulfitz@alum.mit.edu, vadim.tikhanoff@iit.it, martin.peniak@plymouth.ac.uk
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

#include "iCub_Sim.h"

#include "OdeInit.h"

using namespace yarp::sig;

// globals
Semaphore ODE_access(1);
#define CTRL_RAD2DEG    (180.0/M_PI)
#define CTRL_DEG2RAD    (M_PI/180.0)

// locals
// NOTE that we use (long) instead of (clock_t), because on MacOS, (clock_t) is unsigned, while we need negative numbers
static long gl_frame_length = 1000/30; // update opengl and vision stream at 30 Hz
static long ode_step_length = 10;      // target duration of the ODE step in CPU time (set to 0 to go as fast as possible, set to dstep*1000 to go realtime)
static double dstep = 10.0/1000.0;     // step size in ODE's dWorldStep in seconds

static bool glrun;  // draw gl
static bool simrun; // run simulator thread

static int stop = 0;
static int v = 0;

static float xyz[3];
static float hpr[8];
static float rez[3];

static int contactPoint;
static int mouseDiffx, mouseDiffy;
static bool picking = false;
static float cam_rx = 0.0, cam_ry = 0.0;

static int width = 640;
static int height = 480;

static int mouse0_down_x, mouse1_down_x;
static int mouse0_down_y, mouse1_down_y;
static int mouse_ray_x;
static int mouse_ray_y;
static float *VAD;
static float *VAD2;
const dReal *pos;
static float angle_xref = 0.0f;
static float angle_yref = 25.0f;
static float ydistance = 10.0f;
static float xdistance = 0.0f;
static float view_xyz[3];	// position x,y,z
static float view_hpr[3];	// heading, pitch, roll (degrees)
static float view2_xyz[3];
static float view2_hpr[3];
static float zoom = 0;
static float xpos = 0, ypos = 0, zpos = 0, xrot = 0, yrot = 0, zrot = 0, angle=0.0;
static float lastx, lasty;
static float xrotrad = 0, yrotrad = 0;
static long startTime, finishTime;
static double duration, frames, FPS,seconds, TimestepManager;
static float test[3];
//static SDL_TimerID id;
static Uint32          colorkey;
static SDL_Surface     *image;
static bool extractImages = false;
static VideoTexture *video = NULL;
static RobotStreamer *robot_streamer = NULL;        
static RobotConfig *robot_config = NULL;        
static bool eyeCams;
static const GLfloat light_position[] = { 0.0f, 5.0f, 5.0f, 0.0f };

//camera calibration parameters
static int width_left;
static int width_right;
static int height_left;
static int height_right;
static double fov_left;
static double fov_right;


static int cameraSizeWidth;
static int cameraSizeHeight;


// # of touch sensors
#define N_TOUCH_SENSORS 12
// allocate feedback structs as a static array. We don't allocate feedback structs 
// at every simulationstep since memory allocation at every step would degrade simulation performance.
static dJointFeedback touchSensorFeedbacks[MAX_CONTACTS * N_TOUCH_SENSORS];

static int nFeedbackStructs=0;

void OdeSdlSimulation::draw() {
    OdeInit& odeinit = OdeInit::get();
    odeinit._iCub->draw();
    odeinit._wrld->draw();
}

void OdeSdlSimulation::printStats() {
    OdeInit& odeinit = OdeInit::get();

    finishTime = (long) clock() ;
    duration += (double)(finishTime - startTime) / CLOCKS_PER_SEC ;
    frames ++ ;
    FPS = frames / duration ;
    startTime = (long) clock() ;
    odeinit.SimTime = duration;
    //printf("duration: %.2lf\n",odeinit.SimTime);
    static double starting_time_stamp = 0;
    //test[0] = dBodyGetPosition(odeinit._iCub->body_cube[0])[0];
    //test[1] = dBodyGetPosition(odeinit._iCub->body_cube[0])[1];
    //test[2] = dBodyGetPosition(odeinit._iCub->body_cube[0])[2];
    //printf("test[0] %f  test[1] %f  test[2] %f\n",test[0],test[1],test[2]);
    if( duration - starting_time_stamp >= 1){
        //printf("Frames: %.2lf   Duration: %.2lf   fps: %3.1f \n",frames,duration,FPS);
        starting_time_stamp = duration;
    }
    //printf("%lf %lf %lf %lf %lf %lf\n", odeinit._iCub->ra_speed[0],odeinit._iCub->ra_speed[1],odeinit._iCub->ra_speed[2],odeinit._iCub->ra_speed[3],odeinit._iCub->ra_speed[4],odeinit._iCub->ra_speed[5]);
    //drawText(text, textPos);
}

void OdeSdlSimulation::handle_key_down(SDL_keysym* keysym) {
    switch (keysym->sym)
    {
        case SDLK_e:
            break;
        case SDLK_r:
            initViewpoint();
            break;
        case SDLK_t:
            break;
        case SDLK_y:
            break;
        case SDLK_SPACE:
            printf("SPACEBAR pressed! Press spacebar again to disable/enable drawing.\n");
            glrun = !glrun;
            break;
        default:
            break;
    }
}

void OdeSdlSimulation::handle_mouse_motion(SDL_MouseMotionEvent* mousemotion) {
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

void OdeSdlSimulation::process_events(void) {
    OdeInit& odeinit = OdeInit::get();
    SDL_Event event;

    Uint8 * keystate = SDL_GetKeyState(NULL);
    if(keystate[SDLK_q]){xrot += 1 * 0.4f;if (xrot >360) xrot -= 360 * 0.1f;}
    if(keystate[SDLK_z]){xrot -= 1 * 0.4f;if (xrot < -360) xrot += 360 * 0.1f;}
    if(keystate[SDLK_w]){yrotrad = (yrot / 180 * 3.141592654f); xrotrad = (xrot / 180 * 3.141592654f); 
        xpos += float(sin(yrotrad))* 0.005f; ;zpos -= float(cos(yrotrad))* 0.005f; ypos -= float(sin(xrotrad))* 0.005f;}
    if(keystate[SDLK_s]){yrotrad = (yrot / 180 * 3.141592654f); xrotrad = (xrot / 180 * 3.141592654f); 
        xpos -= float(sin(yrotrad))* 0.005f;zpos += float(cos(yrotrad))* 0.005f; ;ypos += float(sin(xrotrad))* 0.005f;}
    if (keystate[SDLK_a]){yrotrad = (yrot / 180 * 3.141592654f);xpos -= float(cos(yrotrad)) * 0.008;zpos -= float(sin(yrotrad)) * 0.008; }
    if (keystate[SDLK_d]){yrotrad = (yrot / 180 * 3.141592654f);xpos += float(cos(yrotrad)) * 0.008;zpos += float(sin(yrotrad)) * 0.008;}
    if(keystate[SDLK_e]){zrot += 1 * 0.4f;if (zrot >360) zrot -= 360 * 0.4f;}
    if(keystate[SDLK_c]){zrot -= 1 * 0.4f;if (zrot < -360) zrot += 360 * 0.4f;}

    if (keystate[SDLK_f]){ypos +=1 *0.005f;}
    if (keystate[SDLK_v]){ypos -=1 *0.005f;}

    if(keystate[SDLK_1]){initViewpoint();}

    if (keystate[SDLK_5]){

        if ((odeinit._iCub->eyeLidRot) < 0.55) odeinit._iCub->eyeLidRot += 0.01;
        cout<<odeinit._iCub->eyeLidRot<<endl;
    }
    if (keystate[SDLK_6]){
        if ((odeinit._iCub->eyeLidRot) > 0.01) odeinit._iCub->eyeLidRot -= 0.01;
        cout<<odeinit._iCub->eyeLidRot<<endl;
    }
    if (keystate[SDLK_h])
    {
        odeinit.sendHomePos();
    }
    /* Grab all the events off the queue. */
    while (SDL_PollEvent(&event)){
        switch (event.type)
        {
            case SDL_VIDEORESIZE:
                width = event.resize.w;
                height = event.resize.h;
                SDL_SetVideoMode(width,height,16,SDL_OPENGL | SDL_RESIZABLE);
                {
                    bool ok = setup_opengl(robot_config->getFinder());
                    if (!ok) {
                        odeinit.stop = true;
                    }
                }
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

void OdeSdlSimulation::nearCallback (void *data, dGeomID o1, dGeomID o2) {
    OdeInit& odeinit = OdeInit::get();

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
        contact[i].surface.bounce = 0.01;
        contact[i].surface.bounce_vel = 0.01;
        contact[i].surface.slip1 = (dReal)0.000001;
        contact[i].surface.slip2 = (dReal)0.000001;
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
            // check if the bodies are touch sensitive.
            bool b1isTouchSensitive = isBodyTouchSensitive (b1);
            bool b2isTouchSensitive = isBodyTouchSensitive (b2); 
            // if any of the bodies are touch sensitive...
            if (b1isTouchSensitive || b2isTouchSensitive) {
                // ... add a feedback structure to the contact joint.
                dJointSetFeedback (c, &(touchSensorFeedbacks[nFeedbackStructs])); 
                nFeedbackStructs++;
            }
            //fprintf(stdout,"colllliiiissssiiiiooon: %d %d\n", dGeomGetClass (o1), dGeomGetClass (o2));
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

// returns true if the body with the bodyID is a touch-sensitive body, returns false otherwise.
bool OdeSdlSimulation::isBodyTouchSensitive (dBodyID bodyID) {
    OdeInit& odeinit = OdeInit::get();

    // check the smaller hand parts if the left hand is active.
    if (odeinit._iCub->actLHand == "on") {	
        if (bodyID == odeinit._iCub->body[10]) {
            return true;
        } else if (bodyID == odeinit._iCub->body[30]) {
            return true;
        } else if (bodyID == odeinit._iCub->body[24]) {
            return true;
        } else if (bodyID == odeinit._iCub->body[25]) {
            return true;
        } else if	(bodyID == odeinit._iCub->lhandfingers3) {
            return true;
        }
    } else { // check the whole left hand body if the hand is not active.
        if (bodyID == odeinit._iCub->l_hand) {
            return true;
        }
    }

    // check the smaller hand parts if the right hand is active.
    if (odeinit._iCub->actRHand == "on") {	
        if (bodyID == odeinit._iCub->body[11]) {
            return true;
        } else if (bodyID == odeinit._iCub->body[49]) {
            return true;
        } else if (bodyID == odeinit._iCub->body[43]) {
            return true;
        } else if	(bodyID == odeinit._iCub->body[44]) {
            return true;
        } else if	(bodyID == odeinit._iCub->rhandfingers3) {
            return true;
        }
    } else { // check the whole right hand body if the hand is not active.
        if (bodyID == odeinit._iCub->r_hand) {
            return true;
        }
    }

    return false;
}

// this is a function to mimic the sensor data from the physical icub fingetip/palm sensors
void OdeSdlSimulation::inspectBodyTouch_icubSensors(Bottle& reportLeft, Bottle& reportRight, bool boolean) { 
    OdeInit& odeinit = OdeInit::get();
    reportLeft.clear();
    reportRight.clear();
    int indicesLeft[6] = {24, 25, 26, 27, 30, 10};
    int indicesRight[6] = {43, 44, 45, 46, 49, 11};

    if (odeinit._iCub->actLHand == "on" && odeinit._iCub->actRHand == "on" ){
        double resultLeft=0, resultRight = 0;
        for (int x = 0; x < 6; x++){
            if (boolean){
                resultLeft = odeinit._iCub->checkTouchSensor( indicesLeft[x] );
                resultRight = odeinit._iCub->checkTouchSensor( indicesRight[x] );
            }
            else{
                resultLeft = odeinit._iCub->checkTouchSensor_continuousValued( indicesLeft[x] );
                resultRight = odeinit._iCub->checkTouchSensor_continuousValued( indicesRight[x] );
            }

            if (x < 5){
                for (int i = 0; i < 12; i++){
                    reportLeft.addDouble(resultLeft * 255);
                    reportRight.addDouble(resultRight * 255);
                }
            }
            if (x == 5){
                for (int y = 0; y<3; y++){            
                    for (int i = 0; i < 12; i++){
                        reportLeft.addDouble(0.0);
                        reportRight.addDouble(0.0);
                    }
                }
                for (int y = 0; y<4; y++){ 
                    for (int i = 0; i < 12; i++){
                        reportLeft.addDouble(resultLeft * 255);
                        reportRight.addDouble(resultRight * 255);
                    }
                }
                for (int y = 0; y<4; y++){ 
                    for (int i = 0; i < 12; i++){
                        reportLeft.addDouble(0.0);
                        reportRight.addDouble(0.0);
                    }
                }
            }
        }
    }//end lhand on rhand on
    else if (odeinit._iCub->actLHand == "on" && odeinit._iCub->actRHand == "off" ){ 
        double resultLeft=0, resultRight = 0;
        for (int x = 0; x < 6; x++){
             if (boolean){
                resultLeft = odeinit._iCub->checkTouchSensor( indicesLeft[x] );
                resultRight = odeinit._iCub->checkTouchSensor( odeinit._iCub->r_hand );
            }
            else{
                resultLeft = odeinit._iCub->checkTouchSensor_continuousValued( indicesLeft[x] );
                resultRight = odeinit._iCub->checkTouchSensor_continuousValued(odeinit._iCub->r_hand);
            }
            if (x < 5){
                for (int i = 0; i < 12; i++){
                    reportLeft.addDouble(resultLeft * 255);
                    reportRight.addDouble(resultRight * 255);
                }
            }
            if (x == 5){
                for (int y = 0; y<3; y++){            
                    for (int i = 0; i < 12; i++){
                        reportLeft.addDouble(0.0);
                        reportRight.addDouble(0.0);
                    }
                }
                for (int y = 0; y<4; y++){ 
                    for (int i = 0; i < 12; i++){
                        reportLeft.addDouble(resultLeft * 255);
                        reportRight.addDouble(resultRight * 255);
                    }
                }
                for (int y = 0; y<4; y++){ 
                    for (int i = 0; i < 12; i++){
                        reportLeft.addDouble(0.0);
                        reportRight.addDouble(0.0);
                    }
                }
            }
        }
    }//end lhand on rhand off
    else if (odeinit._iCub->actRHand == "on" && odeinit._iCub->actLHand == "off"  ){
        double resultLeft=0, resultRight = 0;
        for (int x = 0; x < 6; x++){
            if (boolean){
                resultLeft = odeinit._iCub->checkTouchSensor( odeinit._iCub->l_hand );
                resultRight = odeinit._iCub->checkTouchSensor( indicesRight[x] );
            }
            else{
                resultLeft = odeinit._iCub->checkTouchSensor_continuousValued( odeinit._iCub->l_hand );
                resultRight = odeinit._iCub->checkTouchSensor_continuousValued( indicesRight[x] );
            }
            
            if (x < 5){
                for (int i = 0; i < 12; i++){
                    reportLeft.addDouble(resultLeft * 255);
                    reportRight.addDouble(resultRight * 255);
                }
            }
            if (x == 5){
                for (int y = 0; y<3; y++){            
                    for (int i = 0; i < 12; i++){
                        reportLeft.addDouble(0.0);
                        reportRight.addDouble(0.0);
                    }
                }
                for (int y = 0; y<4; y++){ 
                    for (int i = 0; i < 12; i++){
                        reportLeft.addDouble(resultLeft * 255);
                        reportRight.addDouble(resultRight * 255);
                    }
                }
                for (int y = 0; y<4; y++){ 
                    for (int i = 0; i < 12; i++){
                        reportLeft.addDouble(0.0);
                        reportRight.addDouble(0.0);
                    }
                }
            }
        }   
    }//end lhand off rhand on
    else{//both off
        for (int x = 0; x < 6; x++){
            double resultLeft=0, resultRight = 0;
            if (boolean){
                resultLeft = odeinit._iCub->checkTouchSensor( odeinit._iCub->l_hand );
                resultRight = odeinit._iCub->checkTouchSensor( odeinit._iCub->r_hand );
            }
            else{
                resultLeft = odeinit._iCub->checkTouchSensor_continuousValued( odeinit._iCub->l_hand );
                resultRight = odeinit._iCub->checkTouchSensor_continuousValued(odeinit._iCub->r_hand);
            }
            
            if (x < 5){
                for (int i = 0; i < 12; i++){
                    reportLeft.addDouble(resultLeft * 255);
                    reportRight.addDouble(resultRight * 255);
                }
            }
            if (x == 5){
                for (int y = 0; y<3; y++){            
                    for (int i = 0; i < 12; i++){
                        reportLeft.addDouble(0.0);
                        reportRight.addDouble(0.0);
                    }
                }
                for (int y = 0; y<4; y++){ 
                    for (int i = 0; i < 12; i++){
                        reportLeft.addDouble(resultLeft * 255);
                        reportRight.addDouble(resultRight * 255);
                    }
                }
                for (int y = 0; y<4; y++){ 
                    for (int i = 0; i < 12; i++){
                        reportLeft.addDouble(0.0);
                        reportRight.addDouble(0.0);
                    }
                }
            }
        }
    }//end both off
}


void OdeSdlSimulation::getAngles(const dReal *m, float& z, float& y, float& x) {
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

void OdeSdlSimulation::initViewpoint() {
    xpos = 0;
    ypos = 1;
    zpos = 1;
    xrot = 25;
    yrot = 0;
    zrot = 0;
}

void OdeSdlSimulation::mouseMovement(float x, float y) {
    float diffx = x-lastx; //check the difference between the current x and the last x position
    float diffy = y-lasty; //check the difference between the current y and the last y position
    lastx =x; //set lastx to the current x position
    lasty =y; //set lasty to the current y position
    xrot += (float) diffy; //set the xrot to xrot with the addition of the difference in the y position
    yrot += (float) diffx;	//set the xrot to yrot with the addition of the difference in the x position
}

void OdeSdlSimulation::draw_screen() {
    OdeInit& odeinit = OdeInit::get();
        
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT); // refresh opengl

    if (extractImages || odeinit._iCub->actVision == "on"){
        robot_streamer->sendVision();
    }

    glViewport(0,0,width,height);
    glMatrixMode (GL_PROJECTION);
    glLoadIdentity();
    gluPerspective( 75, (float)width/height, 0.01, 100.0 );
    glMatrixMode (GL_MODELVIEW);
    glLoadIdentity();
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);	
    glRotatef (xrot, 1,0,0);
    glRotatef (yrot, 0,1,0);
    glRotatef (zrot, 0,0,1);
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



void OdeSdlSimulation::retreiveInertialData(Bottle& inertialReport) {
    OdeInit& odeinit = OdeInit::get();
    static dReal OldLinearVel[3], LinearVel[3], LinearAccel[3];
    inertialReport.clear();

    //get euler angles from quaternions
    dQuaternion angles;
    dGeomGetQuaternion( odeinit._iCub->inertialGeom, angles );
    dReal w, x, y, z;
    w = angles[0];
    x = angles[1];
    y = angles[2];
    z = angles[3];
    
    double sqw = w * w;    
    double sqx = x * x;    
    double sqy = y * y;    
    double sqz = z * z; 
    float roll, pitch, yaw;

    double unit = sqx + sqy + sqz + sqw; // if normalised is one, otherwise is correction factor
    double test = x*y + z*w;
    if (test > 0.499*unit) { // singularity at north pole
        roll = 2 * atan2(x,w);
        pitch = M_PI/2;
        yaw = 0;
        return;
    }
    if (test < -0.499*unit) { // singularity at south pole
        roll = -2 * atan2(x,w);
        pitch = -M_PI/2;
        yaw = 0;
    return;
    }
    roll =(float) ( atan2(2.0*y*w-2*x*z , sqx - sqy - sqz + sqw) ); //z
    pitch = (float) (atan2(2.0*x*w-2*y*z , -sqx + sqy - sqz + sqw) );//x
    yaw = asin(2*test/unit);//y

    //roll = dBodyGetRotation(odeinit._iCub->head)[4]; // was 1
    //pitch = dBodyGetRotation(odeinit._iCub->head)[6];
    //yaw = dBodyGetRotation(odeinit._iCub->head)[2];

    //Add Euler angles roll pitch yaw
    inertialReport.addDouble( -yaw * 180/M_PI);// yaw
    inertialReport.addDouble( -pitch * 180/M_PI);// pitch
    inertialReport.addDouble( roll * 180/M_PI);// roll 

    /*//in order to calculate linear acceleration (make sure of body) Inertial Measurement Unit IMU
    LinearVel[0] = dBodyGetLinearVel(odeinit._iCub->inertialBody)[0];
    LinearVel[1] = dBodyGetLinearVel(odeinit._iCub->inertialBody)[1];
    LinearVel[2] = dBodyGetLinearVel(odeinit._iCub->inertialBody)[2];
    //// a = dv/dt = ( t - t_old ) / dt
    LinearAccel[0] = ( LinearVel[0] - OldLinearVel[0] ) / 0.02;
    LinearAccel[1] = ( LinearVel[1] - OldLinearVel[1] ) / 0.02;
    LinearAccel[2] = ( LinearVel[2] - OldLinearVel[2] ) / 0.02;
    OldLinearVel[0] = LinearVel[0];
    OldLinearVel[1] = LinearVel[1];
    OldLinearVel[2] = LinearVel[2];*/
        
    ////Add linear acceleration
    Vector grav,grav1,grav2,grav3;
    grav.resize(3);
    grav1.resize(3);
    grav2.resize(3);
    grav3.resize(3);
    double  theta;

    grav[0]=0;
    grav[1]=0;
    grav[2]=9.81;

    theta = pitch;
    grav1[0]=grav[0]*cos(theta)+grav[2]*sin(theta);
    grav1[1]=grav[1];
    grav1[2]=grav[0]*(-sin(theta))+grav[2]*cos(theta);

    theta = yaw;
    grav2[0]=grav1[0];
    grav2[1]=grav1[1]*cos(theta)+grav1[2]*(-sin(theta));
    grav2[2]=grav1[1]*sin(theta)+grav1[2]*cos(theta);

    theta = roll;
    grav3[0]=grav2[0]*cos(theta)+grav2[1]*(-sin(theta));
    grav3[1]=grav2[0]*sin(theta)+grav2[1]*cos(theta);
    grav3[2]=grav2[2];

    inertialReport.addDouble( grav3[0] );
    inertialReport.addDouble( grav3[1] );
    inertialReport.addDouble( grav3[2] );

    //Add angular velocity
    inertialReport.addDouble(-dBodyGetAngularVel(odeinit._iCub->inertialBody)[2]*CTRL_RAD2DEG);
    inertialReport.addDouble(-dBodyGetAngularVel(odeinit._iCub->inertialBody)[0]*CTRL_RAD2DEG);
    inertialReport.addDouble( dBodyGetAngularVel(odeinit._iCub->inertialBody)[1]*CTRL_RAD2DEG);
        
    //Add magnetic fields
    inertialReport.addDouble(0.0);
    inertialReport.addDouble(0.0);
    inertialReport.addDouble(0.0);
}

int OdeSdlSimulation::thread_ode(void *unused) {
    //SLD_AddTimer freezes the system if delay is too short. Instead use a while loop that waits if there was time left after the computation of ODE_process
    double cpms = 1e3 / CLOCKS_PER_SEC;
    long lastOdeProcess = (long) (clock()*cpms);
    double avg_ode_step_length = 0.0;
    long count = 0;
    simrun = true;
    double timeCache = ode_step_length;
    long lastTimeCacheUpdate = (long) (clock()*cpms);
    double alpha = 0.99;
    // if realTime=true when delays occur the simulation tries to recover by running more steps in a row
    // if realTime=false the simulation executes the simulation steps with a fixed rate irregardless of delays
    bool realTime = true;
    long temp;

    while (simrun) {
        temp = (long) (clock()*cpms);
        timeCache += temp - lastTimeCacheUpdate;
        lastTimeCacheUpdate = temp;
        while(timeCache < ode_step_length){
            SDL_Delay((unsigned int)(ode_step_length-timeCache));
            temp = (long) (clock()*cpms);
            timeCache += temp - lastTimeCacheUpdate;
            lastTimeCacheUpdate = temp;
        }

        /*if(timeCache >= 2.0*ode_step_length) 
            printf("Simulation delay: running %d steps in a row to recover.\n", (int)(timeCache/ode_step_length));*/

        while(timeCache >= ode_step_length){
            count++;
            lastOdeProcess = (long) (clock()*cpms);
            ODE_process(1, (void*)1);
            avg_ode_step_length = alpha*avg_ode_step_length + (1.0-alpha)*((long) (clock()*cpms) -lastOdeProcess);
            
            if(realTime)
                timeCache -= ode_step_length;
            else
                timeCache = 0.0;

            // check if the desired timestep is achieved, if not, print a warning msg
            if(count % (10000/ode_step_length)==0){
                if(avg_ode_step_length >= ode_step_length+1)
                    printf("WARNING: the simulation is too slow to run in real-time, you should increase the timestep in ode_params.ini (current value: %ld, suggested value: %.0f)\n", 
                        ode_step_length, avg_ode_step_length);
                else if(avg_ode_step_length <= ode_step_length-1)
                    printf("INFO: you could get a more accurate dynamics simulation by decreasing the timestep in ode_params.ini (current value: %ld, suggested value: %.0f)\n", 
                        ode_step_length, avg_ode_step_length);
            }
        }
    }
    return(0);
}

Uint32 OdeSdlSimulation::ODE_process(Uint32 interval, void *param) {
    OdeInit& odeinit = OdeInit::get();
    //static clock_t startTimeODE= clock(), finishTimeODE= clock();
    //startTimeODE = clock();

    odeinit.mutex.wait();
    nFeedbackStructs=0;
    dSpaceCollide(odeinit.space,0,&nearCallback);
    dWorldStep(odeinit.world, dstep);
    // do 1 TIMESTEP in controllers (ok to run at same rate as ODE: 1 iteration takes about 300 times less computation time than dWorldStep)
    for (int ipart = 0; ipart<MAX_PART; ipart++) {
        if (odeinit._controls[ipart] != NULL) {
            odeinit._controls[ipart]->jointStep();
        }
    }
    odeinit.sync = true;
    odeinit.mutex.post();

    if ( robot_streamer->shouldSendTouchLeft() || robot_streamer->shouldSendTouchRight() ) {
        Bottle reportLeft;
        Bottle reportRight;
        bool boolean = true;
        if (odeinit._iCub->actPressure == "on")
            boolean = false;
            
        inspectBodyTouch_icubSensors(reportLeft, reportRight, boolean);//inspectBodyTouch_continuousValued(report);
        
        if ( robot_streamer->shouldSendTouchLeft() )
                robot_streamer->sendTouchLeft( reportLeft );

        if ( robot_streamer->shouldSendTouchRight() )
            robot_streamer->sendTouchRight( reportRight );
    }

    dJointGroupEmpty (odeinit.contactgroup);

    if (robot_streamer->shouldSendInertial()) {
        Bottle inertialReport;
        retreiveInertialData(inertialReport);
        robot_streamer->sendInertial(inertialReport);
    }

    //go and check if torques are needed
    robot_streamer->checkTorques();

    odeinit._iCub->setJointControlAction();
    
    //finishTimeODE = clock() ;
    //SPS();
    //printf("ODE=%lf\n",(double)(finishTimeODE - startTimeODE) / CLOCKS_PER_SEC);
    return(interval);
}


/*int OdeSdlSimulation::thread_func(void *unused) {
    // this needs to be kept synchronized with the timestep in
    // dWorldStep, in order to get correct world clock time
    //  --paulfitz
    int delay = 50;
    id = SDL_AddTimer( delay, &OdeSdlSimulation::ODE_process, (void*)1);

    return(0);
}
*/
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

void OdeSdlSimulation::sighandler(int sig) {
    OdeInit& odeinit = OdeInit::get();
    odeinit.stop = true;
    cout << "\nCAUGHT Ctrl-c" << endl;
}

void OdeSdlSimulation::simLoop(int h,int w) {
    OdeInit& odeinit = OdeInit::get();

    SDL_Init(SDL_INIT_TIMER | SDL_GL_ACCELERATED_VISUAL);
    SDL_SetVideoMode(h,w,32,SDL_OPENGL | SDL_RESIZABLE);// | SDL_SWSURFACE| SDL_ANYFORMAT); // on init 

    dAllocateODEDataForThread(dAllocateMaskAll);
    ConstString logo = robot_config->getFinder().findFile("logo");

    image = SDL_LoadBMP(robot_config->getFinder().findFile(logo.c_str()).c_str());
    SDL_WM_SetIcon(image,0);
    SDL_FreeSurface(image);
    SDL_WM_SetCaption("iCub Simulator", "image");

    //SDL_Thread *thread;
    SDL_Thread *ode_thread = SDL_CreateThread(thread_ode, NULL);
    //thread = SDL_CreateThread(thread_func, NULL);

    if ( ode_thread == NULL ) {
        fprintf(stderr, "Unable to create thread: %s\n", SDL_GetError());
        return;
    }

    initViewpoint();
    bool ok = setup_opengl(robot_config->getFinder());
    if (!ok) return;
    startTime = (long) clock();
    odeinit.stop = false;

    yarp::os::signal(yarp::os::YARP_SIGINT, sighandler);
    yarp::os::signal(yarp::os::YARP_SIGTERM, sighandler);

    glrun = true;
    odeinit._wrld->WAITLOADING = false;
    odeinit._wrld->static_model = false;
    long prevTime = (long) clock();
    long timeLeft;
    
    if (odeinit._iCub->actStartHomePos == "on")
        odeinit.sendHomePos();

    while(!odeinit.stop) {
        /* Process incoming events. */
        process_events();
        /* Draw the screen. */
        if ( !odeinit._wrld->WAITLOADING ){
            if (glrun) {
                odeinit.mutexTexture.wait();
                draw_screen();  
                odeinit.mutexTexture.post();
                // check for framerate
                timeLeft = (prevTime - (long) clock()) + gl_frame_length;
                //cout << "check for framerate " << timeLeft << endl;
                if (timeLeft > 0) 
                { // if there is still time left in this frame, just wait
                    SDL_Delay(timeLeft);
                }
                prevTime = (long) clock();
            } else {
                SDL_Delay(100);
            }
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
    //SDL_RemoveTimer(id);
    //Stop the thread
    //SDL_KillThread( thread );
    simrun = false;
    //SDL_WaitThread( thread, NULL );
    SDL_WaitThread( ode_thread, NULL );
    //SDL_Quit();
}

void OdeSdlSimulation::drawView(bool left, bool right, bool wide) {
    OdeInit& odeinit = OdeInit::get();
    const dReal *pos;
    const dReal *rot;
    glViewport(0,0,cameraSizeWidth,cameraSizeHeight);
    glMatrixMode (GL_PROJECTION);
    
    if (left){
        glLoadIdentity();
        gluPerspective( fov_left, (float) width_left/height_left, 0.04, 100.0 );
        pos = dGeomGetPosition(odeinit._iCub->Leye1_geom);
        rot = dGeomGetRotation(odeinit._iCub->Leye1_geom);
        glMatrixMode (GL_MODELVIEW);
        glLoadIdentity();
        glLightfv(GL_LIGHT0, GL_POSITION, light_position);
        gluLookAt(
                  pos[0],
                  pos[1],
                  pos[2],
                  pos[0]  + rot[2],
                  pos[1]  + rot[6],
                  pos[2] + rot[10],
                  -rot[4], 1, 0
                  );
    }
    if (right){
        glLoadIdentity();
        gluPerspective( fov_right, (float) width_right/height_right, 0.04, 100.0 );//55.8
        pos = dGeomGetPosition(odeinit._iCub->Reye1_geom);
        rot = dGeomGetRotation(odeinit._iCub->Reye1_geom);
        glMatrixMode (GL_MODELVIEW);
        glLoadIdentity();
        glLightfv(GL_LIGHT0, GL_POSITION, light_position);
        gluLookAt(
                  pos[0],
                  pos[1],
                  pos[2],
                  pos[0]  + rot[2],
                  pos[1]  + rot[6],
                  pos[2] + rot[10],
                  -rot[4], 1, 0
                  );
    }	
    if (wide){
        glLoadIdentity();
        gluPerspective( 55.8, (float) cameraSizeWidth/cameraSizeHeight, 0.04, 100.0 );//here nothing to do with cameras
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
    draw();//robot
    glEnable(GL_TEXTURE_2D);
    drawSkyDome(0,0,0,50,50,50); // Draw the Skybox	
}

void OdeSdlSimulation::clearBuffer() {
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT); // refresh opengl
}

OdeSdlSimulation::OdeSdlSimulation() {
}

void OdeSdlSimulation::init(RobotStreamer *streamer, 
                            RobotConfig *config) {
    OdeInit& odeinit = OdeInit::get();
    if (video!=NULL) {
        fprintf(stderr, "Only one Simulation object allowed\n");
        yarp::os::exit(1);
    }
    robot_streamer = streamer;
    robot_config = config;

    ode_step_length = config->getWorldTimestep();
    dstep = ode_step_length*1e-3;

    video = new VideoTexture;
    string moduleName = odeinit.getName();
    video->setName( moduleName ); 
    odeinit._iCub->eyeLidsPortName = moduleName;
    Property options;

    //get the camera calibration parameters
    string camcalib_context=robot_config->getFinder().check("camcalib_context",
                                                            Value("cameraCalibration")).asString().c_str();
    string camcalib_file=robot_config->getFinder().check("camcalib_file",
                                                         Value("icubSimEyes.ini")).asString().c_str();

    ResourceFinder rf_camcalib;
    rf_camcalib.setVerbose();
    rf_camcalib.setDefaultContext(camcalib_context.c_str());
    rf_camcalib.setDefaultConfigFile(camcalib_file.c_str());
    rf_camcalib.configure(0,NULL);

    //left
    Bottle &bCalibLeft=rf_camcalib.findGroup("CAMERA_CALIBRATION_LEFT");
    width_left=bCalibLeft.check("w",Value(320)).asInt();
    height_left=bCalibLeft.check("h",Value(240)).asInt();

    cameraSizeWidth=width_left;
    cameraSizeHeight=height_left;

    double focal_length_left=bCalibLeft.check("fy",Value(257.34)).asDouble();
    fov_left=2*atan2((double)height_left,2*focal_length_left)*180.0/M_PI;

    //right
    Bottle &bCalibRight=rf_camcalib.findGroup("CAMERA_CALIBRATION_RIGHT");
    width_right=bCalibRight.check("w",Value(320)).asInt();
    height_right=bCalibRight.check("h",Value(240)).asInt();

    double focal_length_right=bCalibRight.check("fy",Value(257.34)).asDouble();
    fov_right=2*atan2((double)height_right,2*focal_length_right)*180.0/M_PI;
    //--------------------------------------//


    ConstString videoconf = robot_config->getFinder().findFile("video");
    options.fromConfigFile(videoconf.c_str());

    Bottle textures = *options.find("textures").asList();
    for (int i=0; i<textures.size(); i++) {
        ConstString name = textures.get(i).asString();
        printf("Adding video texture %s\n", name.c_str());
        video->add(options.findGroup(name.c_str()));
    }
}

OdeSdlSimulation::~OdeSdlSimulation() {
    delete video;
}


bool OdeSdlSimulation::checkSync(bool reset) {
    OdeInit& odeinit = OdeInit::get();
    if (reset) {
        odeinit.sync = false;
    }
    return odeinit.sync;
}


bool OdeSdlSimulation::getTrqData(Bottle data) {
    OdeInit& odeinit = OdeInit::get();
    for (int s=0; s<data.size(); s++){
        odeinit._iCub->torqueData[s] = data.get(s).asDouble();
        //fprintf(stdout,"torques... %lf \n",odeinit._iCub->torqueData[s]);
    }
    return true;
}



bool OdeSdlSimulation::getImage(ImageOf<PixelRgb>& target) {
    int w = cameraSizeWidth;
    int h = cameraSizeHeight;
    int p = 3;

    char *buf=new char[w * h * p];
    glReadPixels( 0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, buf);
    ImageOf<PixelRgb> img;
    img.setQuantum(1);
    img.setExternal(buf,w,h);

    // inefficient flip!
    target.resize(img);
    int ww = img.width();
    int hh = img.height();
    for (int x=0; x<ww; x++) {
        for (int y=0; y<hh; y++) {
            target(x,y) = img(x,hh-1-y);
        }
    }
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    delete[] buf;
    return true;
}
