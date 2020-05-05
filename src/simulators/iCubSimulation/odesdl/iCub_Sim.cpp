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
#include <yarp/os/LogStream.h>
#include <mutex>
#include <cstdlib>
#include <csignal>
#include <set>

using namespace yarp::sig;

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

struct contactICubSkinEmul_t{
    bool coverTouched;
    bool indivTaxelResolution; 
    std::set<unsigned int> taxelsTouched;
};
    
static std::map<SkinPart,contactICubSkinEmul_t> contactICubSkinEmulMap;
    
/* For every collision detected by ODE, contact joints (up to MAX_CONTACTS per collison) are created and a feedback structs may be associated with them - that will carry information about the contact.
 * The number of collisions and contact joints may vary, but we allocate these as a static array for performance issues.
 * (Allocating feedback structs at every simulation step would degrade simulation performance).
 * If the MAX_DJOINT_FEEDBACKSTRUCTS was exceeded, contacts will still be saved for the purposes of whole_body_skin_emul,
 * but the forces send to skinEvents will not be available.
*/
#define MAX_DJOINT_FEEDBACKSTRUCTS 500

static dJointFeedback touchSensorFeedbacks[MAX_DJOINT_FEEDBACKSTRUCTS]; 
static int nFeedbackStructs=0;

static bool START_SELF_COLLISION_DETECTION = false; //we want to set this trigger on only after the robot is in in home pos -
 //it's initial configuration is with arms inside the thighs 
static const double EXTRA_MARGIN_FOR_TAXEL_POSITION_M = 0.03; //0.03 //for skin emulation we get the coordinates of the collision and contact with skin cover from ODE; 
//after transforming to local reference frame of respective skin part, we emulate which set of taxels would get activated at that position; 
//however, with errors in the position, we need an extra margin, so the contact falls onto some taxels
static const double MORE_EXTRA_MARGIN_FOR_TAXEL_POSITION_M = 0.01; //0.01;

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
    //yDebug("duration: %.2lf\n",odeinit.SimTime);
    static double starting_time_stamp = 0;
    //test[0] = dBodyGetPosition(odeinit._iCub->body_cube[0])[0];
    //test[1] = dBodyGetPosition(odeinit._iCub->body_cube[0])[1];
    //test[2] = dBodyGetPosition(odeinit._iCub->body_cube[0])[2];
    //yDebug("test[0] %f  test[1] %f  test[2] %f\n",test[0],test[1],test[2]);
    if( duration - starting_time_stamp >= 1){
        //yDebug("Frames: %.2lf   Duration: %.2lf   fps: %3.1f \n",frames,duration,FPS);
        starting_time_stamp = duration;
    }
    //yDebug("%lf %lf %lf %lf %lf %lf\n", odeinit._iCub->ra_speed[0],odeinit._iCub->ra_speed[1],odeinit._iCub->ra_speed[2],odeinit._iCub->ra_speed[3],odeinit._iCub->ra_speed[4],odeinit._iCub->ra_speed[5]);
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
            yInfo("SPACEBAR pressed! Press spacebar again to disable/enable drawing.\n");
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
        yDebug()<<odeinit._iCub->eyeLidRot;
    }
    if (keystate[SDLK_6]){
        if ((odeinit._iCub->eyeLidRot) > 0.01) odeinit._iCub->eyeLidRot -= 0.01;
        yDebug()<<odeinit._iCub->eyeLidRot;
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
                        //yDebug(" Down\n");
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
                        //yDebug(" up\n");
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

    const double CONTACT_HEIGHT_TRESHOLD_METERS = 0.1; //for debugging or skin emulation purposes, assuming the robot is in contact with a flat ground (typically standing), 
    //the contacts generated between the robot and the ground that are always present can be ignored
    
    OdeInit& odeinit = OdeInit::get();

    assert(o1);
    assert(o2);
     
    dSpaceID space1,space2;
    dSpaceID superSpace1,superSpace2;
    std::string geom1className("");
    std::string geom2ClassName("");
    std::string geom1name("");
    std::string geom2name("");
    bool geom1isiCubPart = false;
    bool geom2isiCubPart = false;
    bool geom1isTorsoOrArm = false;
    bool geom2isTorsoOrArm = false;
    int subLevel1;
    //determine the indentation level for the printouts based on the sublevel in the hiearchy of spaces
    string indentString("");
    std::map<dGeomID,string>::iterator geom1namesIt;
    std::map<dGeomID,string>::iterator geom2namesIt;
    
    if (dGeomIsSpace(o1)){
       space1 = (dSpaceID)o1;
    } else {
       space1 = dGeomGetSpace(o1);
       indentString = indentString + " --- "; //extra indentation level because it is a geom in that space
    }
    subLevel1 = dSpaceGetSublevel(space1);
    for (int i=1;i<=subLevel1;i++){ //start from i=1, for sublevel==0 we don't add any indentation
      indentString = indentString + " --- ";
    }
     
    if (odeinit.verbosity > 3) yDebug("%s nearCallback()\n",indentString.c_str());
   
    if (dGeomIsSpace(o1)){
        space1 = (dSpaceID)o1;
        if (odeinit.verbosity > 3){
          yDebug("%s Object nr. 1: %s, sublevel: %d, contained within: %s, nr. geoms: %d. \n",indentString.c_str(),odeinit._iCub->dSpaceNames[space1].c_str(),dSpaceGetSublevel(space1),odeinit._iCub->dSpaceNames[dGeomGetSpace(o1)].c_str(),dSpaceGetNumGeoms(space1));
        }
    }
    else{ //it's a geom
        getGeomClassName(dGeomGetClass(o1),geom1className);
        superSpace1 = dGeomGetSpace(o1);
        geom1namesIt = odeinit._iCub->dGeomNames.find(o1);
        if (geom1namesIt != odeinit._iCub->dGeomNames.end()){
           geom1name = geom1namesIt->second;   
           if (odeinit.verbosity > 3) yDebug("%s Object nr. 1: geom: %s, class: %s, contained within %s (sublevel %d).\n",indentString.c_str(),geom1name.c_str(),geom1className.c_str(),odeinit._iCub->dSpaceNames[superSpace1].c_str(),dSpaceGetSublevel(superSpace1));
        }
        else{
           if (odeinit.verbosity > 3) yDebug("%s Object nr. 1: A geom, ID: %p, class: %s, contained within %s (sublevel %d).\n",indentString.c_str(),o1,geom1className.c_str(),odeinit._iCub->dSpaceNames[superSpace1].c_str(),dSpaceGetSublevel(superSpace1));
        }
    }
 
    if (dGeomIsSpace(o2)){
        space2 = (dSpaceID)o2;
        if (odeinit.verbosity > 3){
               yDebug("%s Object nr. 2: %s, sublevel: %d, contained within: %s, nr. geoms: %d. \n",indentString.c_str(),odeinit._iCub->dSpaceNames[space2].c_str(),dSpaceGetSublevel(space2),odeinit._iCub->dSpaceNames[dGeomGetSpace(o2)].c_str(),dSpaceGetNumGeoms(space2));
        }
    } else {
        getGeomClassName(dGeomGetClass(o2),geom2ClassName);
        superSpace2 = dGeomGetSpace(o2);
        geom2namesIt = odeinit._iCub->dGeomNames.find(o2);
        if (geom2namesIt != odeinit._iCub->dGeomNames.end()){
           geom2name = geom2namesIt->second;
           if (odeinit.verbosity > 3) yDebug("%s Object nr. 2: geom: %s, class: %s, contained within %s (sublevel %d).\n",indentString.c_str(),geom2name.c_str(),geom2ClassName.c_str(),odeinit._iCub->dSpaceNames[superSpace2].c_str(),dSpaceGetSublevel(superSpace2));
        }
        else{
           if (odeinit.verbosity > 3) yDebug("%s Object nr. 2: A geom, ID: %p, class: %s, contained within %s (sublevel %d).\n",indentString.c_str(),o2,geom2ClassName.c_str(),odeinit._iCub->dSpaceNames[superSpace2].c_str(),dSpaceGetSublevel(superSpace2));
        }
    }
    
    // if at least one of the geoms is a space, we need to go deeper -> recursive calls 
    if (dGeomIsSpace(o1) || dGeomIsSpace(o2)){
      if (dGeomIsSpace(o1) && dGeomIsSpace(o2)){ //if both are spaces, we exclude special combinations from the checking
          if (((space1 == odeinit._iCub->iCubHeadSpace) && (space2 == odeinit._iCub->iCubTorsoSpace)) || ((space1 == odeinit._iCub->iCubTorsoSpace) && (space2 == odeinit._iCub->iCubHeadSpace))){
              if (odeinit.verbosity > 3) yDebug("%s Ignoring head vs. torso collision space checking.\n",indentString.c_str()); 
                //these are unnecessary geoms to check, moreover 2 of these were colliding while not connected by a joint
          }
          else if (((space1 == odeinit._iCub->iCubLegsSpace) && (space2 == odeinit._iCub->iCubTorsoSpace)) || ((space1 == odeinit._iCub->iCubTorsoSpace) && (space2 == odeinit._iCub->iCubLegsSpace))){
             if (odeinit.verbosity > 3) yDebug("%s Ignoring legs vs. torso collision space checking.\n",indentString.c_str()); 
            //these are unnecessary geoms to check - it always check collisions of geoms connected by a joint
          }
          else{
            dSpaceCollide2(o1,o2,data,&nearCallback);
          }
       }
       else{
           dSpaceCollide2(o1,o2,data,&nearCallback);
       }
       //}
      //if (dGeomIsSpace(o2)){
	//  dSpaceCollide2(o2,o1,data,&nearCallback); //start the recursion from the other end
      //}
      return;      
    }  
    /* Note we do not want to test intersections within a space,
    * only between spaces. Therefore, we do not call  dSpaceCollide ((dSpaceID)o1, data, &nearCallback) and the same for o2 */
	
    /* if we made it up to here, it means we have two geoms (not spaces) o1, o2 from two different spaces and we should handle their collision */
     
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    if (b1 && b2 && dAreConnectedExcluding (b1,b2,dJointTypeContact)){
      if (odeinit.verbosity > 3) yDebug("%s Collision ignored: the bodies of o1 and o2 are connected by a joint.\n",indentString.c_str());
      return;
    }
    // list of self-collisions to ignore
    if (selfCollisionOnIgnoreList(geom1name,geom2name)){
       if (odeinit.verbosity > 3){
           yDebug("%s geom: %s (class: %s, contained within %s) AND geom: %s (class: %s, contained within %s).\n",indentString.c_str(),geom1name.c_str(),geom1className.c_str(),odeinit._iCub->dSpaceNames[superSpace1].c_str(),geom2name.c_str(),geom2ClassName.c_str(),odeinit._iCub->dSpaceNames[superSpace2].c_str());
           yDebug("%s Collision ignored (ignore list).\n",indentString.c_str());
       }
       return;
    }
       
    if (odeinit.verbosity > 3) yDebug("%s Collision candidate. Preparing contact joints.\n",indentString.c_str());
    dContact contact[MAX_CONTACTS];   // up to MAX_CONTACTS contacts per box-box
    int i;
    for (i=0; i<MAX_CONTACTS; i++) {
        contact[i].surface.mode = dContactSlip1| dContactSlip2| dContactBounce | dContactSoftCFM;
        contact[i].surface.mu = odeinit.contactFrictionCoefficient;
        contact[i].surface.mu2 = odeinit.contactFrictionCoefficient;
        contact[i].surface.bounce = 0.01;
        contact[i].surface.bounce_vel = 0.01;
        contact[i].surface.slip1 = (dReal)0.000001;
        contact[i].surface.slip2 = (dReal)0.000001;
        contact[i].surface.soft_cfm = 0.0001;
    }
    int numc = dCollide (o1,o2,MAX_CONTACTS,&contact[0].geom,sizeof(dContact)); 
    if (numc > 0){          
        if (odeinit.verbosity > 3) yDebug("%s Collision suspect confirmed. There are %d contacts - creating joints.\n",indentString.c_str(),numc);
        dMatrix3 RI;
        dRSetIdentity (RI);
        if(contact[0].geom.pos[1]>CONTACT_HEIGHT_TRESHOLD_METERS){ //non-foot contact
            if (odeinit.verbosity > 2){
               yDebug("%s   ****** non-ground COLLISION, %d contacts - creating joints************************************************************\n",indentString.c_str(),numc);
               yDebug("%s geom: %s (%s, contained within %s) AND geom: %s (%s, contained within %s)\n",indentString.c_str(),geom1name.c_str(),geom1className.c_str(),odeinit._iCub->dSpaceNames[superSpace1].c_str(),geom2name.c_str(),geom2ClassName.c_str(),odeinit._iCub->dSpaceNames[superSpace2].c_str());
            }
        }
        for (i=0; i<numc; i++) {
            if (odeinit.verbosity > 4) yDebug("%s	Contact joint nr. %d (index:%d): at (%f,%f,%f), depth: %f \n",indentString.c_str(),i+1,i,contact[i].geom.pos[0],contact[i].geom.pos[1],contact[i].geom.pos[2],contact[i].geom.depth);
            dJointID c = dJointCreateContact (odeinit.world,odeinit.contactgroup,contact+i);
            dJointAttach (c,b1,b2);
            // if (show_contacts) dsDrawBox (contact[i].geom.pos,RI,ss);
            // check if the bodies are touch sensitive.
            if (odeinit._iCub->actSkinEmul == "off"){ //this is the old implementation - hands (fingers and palm) are checked for touch
                bool b1isTouchSensitive = isBodyTouchSensitive (b1);
                bool b2isTouchSensitive = isBodyTouchSensitive (b2); 
                // if any of the bodies are touch sensitive...
                if (b1isTouchSensitive || b2isTouchSensitive) {
                    // ... add a feedback structure to the contact joint.
                    if (odeinit.verbosity > 2) yDebug("%s	Adding tactile feedback for emulating finger/palm skin to this one (ODE joint feedback counter: %d).\n",indentString.c_str(),nFeedbackStructs);
                    dJointSetFeedback (c, &(touchSensorFeedbacks[nFeedbackStructs])); 
                    nFeedbackStructs++;
                }
            }
            else { //whole_body_skin_emul ~ actSkinEmul is on
            /* here we treat all bodies belonging to the icub as touch sensitive
            * we want to know if the geom is part of the iCub - that is its superSpace is one of the iCub subspaces*/ 

                if ((superSpace1 == odeinit._iCub->iCubHeadSpace) ||  (superSpace1 == odeinit._iCub->iCubLegsSpace)){ 
                    geom1isiCubPart = true;
                }
                else if ((superSpace1==odeinit._iCub->iCubTorsoSpace) || (superSpace1==odeinit._iCub->iCubLeftArmSpace) || (superSpace1== odeinit._iCub->iCubRightArmSpace)){
                    geom1isiCubPart = true;
                    geom1isTorsoOrArm = true;
                }
                // || (superSpace1 == iCub){ - this should never happen here - in the self-collision mode, the iCub space contains only subspaces - no geoms directly
                
                if ((superSpace2 == odeinit._iCub->iCubHeadSpace) ||  (superSpace2 == odeinit._iCub->iCubLegsSpace)){ 
                    geom2isiCubPart = true;
                }
                else if ((superSpace2==odeinit._iCub->iCubTorsoSpace) || (superSpace2==odeinit._iCub->iCubLeftArmSpace) || (superSpace2== odeinit._iCub->iCubRightArmSpace)){
                    geom2isiCubPart = true;
                    geom2isTorsoOrArm = true;
                }
        
                // if (geom1isiCubPart || geom2isiCubPart){ //we don't have the legs and head implemented yet - these don't have skin in the real robot - but legs will -> should do that
                if ( geom1isTorsoOrArm || geom2isTorsoOrArm){
                    if (odeinit.verbosity > 3) yDebug("%s	Adding tactile feedback for whole-body skinContact to this contact (ODE joint feedback counter: %d).\n",indentString.c_str(),nFeedbackStructs);
                    if (nFeedbackStructs >= MAX_DJOINT_FEEDBACKSTRUCTS){
                        yWarning("out of contact joint feedback structures for ODE (exceeded %d) - some contact joints will not have info about forces stored\n.",MAX_DJOINT_FEEDBACKSTRUCTS); 
                    }
                    else{
                        dJointSetFeedback (c, &(touchSensorFeedbacks[nFeedbackStructs])); 
                        nFeedbackStructs++;	
                    }
                    OdeInit::contactOnSkin_t contactOnSkin, contactOnSkin2;
                    if (geom1isiCubPart){
                        contactOnSkin.body_geom_space_id = superSpace1;
                        contactOnSkin.body_geom_id = o1; 
                        contactOnSkin.body_index = 1;
                        contactOnSkin.contact_geom = contact[i].geom;
                        contactOnSkin.contact_joint = c; 
                        odeinit.listOfSkinContactInfos.push_back(contactOnSkin);
                    }
                    if (geom2isiCubPart){
                        contactOnSkin2.body_geom_space_id = superSpace2;
                        contactOnSkin2.body_geom_id = o2; 
                        contactOnSkin2.body_index = 2;
                        contactOnSkin2.contact_geom = contact[i].geom;
                        contactOnSkin2.contact_joint = c; 
                        odeinit.listOfSkinContactInfos.push_back(contactOnSkin2);
                    }
                } 
                else {
                    if (odeinit.verbosity > 3) yDebug("%s Ignoring skin contact - so far only arms and torso are implemented.\n",indentString.c_str());
                }
            }   //whole_body_skin_emul ~ actSkinEmul is on
            if (odeinit.verbosity > 3) yDebug("\n");
        } // for numc - contacts
    } // if (numc > 0)
    else{
       if (odeinit.verbosity > 3) yDebug("%s Collision suspect NOT confirmed. There were %d contacts.\n",indentString.c_str(),numc);
    }
}


bool OdeSdlSimulation::selfCollisionOnIgnoreList(string geom1_string, string geom2_string)
{
  /** left arm vs. torso ********/  
 if ( ( (geom1_string.compare("upper left arm cover")==0)  &&  (geom2_string.compare("torsoGeom[4]")==0) )  || ( (geom2_string.compare("upper left arm cover")==0)  &&  (geom1_string.compare("torsoGeom[4]")==0) ) ){
      return true; 
  }
  if ( ( (geom1_string.compare("upper left arm cover")==0)  &&  (geom2_string.compare("torso cover")==0) )  || ( (geom2_string.compare("upper left arm cover")==0)  &&  (geom1_string.compare("torso cover")==0) ) ){
      return true; 
  }
  if ( ( (geom1_string.compare("geom[2]")==0)  &&  (geom2_string.compare("torso cover")==0) )  || ( (geom2_string.compare("geom[2]")==0)  &&  (geom1_string.compare("torso cover")==0) ) ){
      return true; 
  } //geom[2] is the cylinder in at shoulder joint (when it is "on" - part activated, it may collide ; when off (different geom name), it will not go into the torso, so no need to handle this)
  
  if ( ( (geom1_string.compare("geom[4]")==0)  &&  (geom2_string.compare("torso cover")==0) )  || ( (geom2_string.compare("geom[4]")==0)  &&  (geom1_string.compare("torso cover")==0) ) ){
      return true; 
  } //geom[4] is the cylinder in upper left arm (similarly, no need to test for the version with part off (ICubSim::initLeftArmOff))
  if ( ( (geom1_string.compare("geom[4]")==0)  &&  (geom2_string.compare("torsoGeom[5]")==0) )  || ( (geom2_string.compare("geom[4]")==0)  &&  (geom1_string.compare("torsoGeom[5]")==0) ) ){
      return true; 
  } //upper arm cylinder colliding with torso box
    
  /** right arm vs. torso ********/
  if ( ( (geom1_string.compare("upper right arm cover")==0)  &&  (geom2_string.compare("torsoGeom[5]")==0) )  || ( (geom2_string.compare("upper right arm cover")==0)  &&  (geom1_string.compare("torsoGeom[5]")==0) ) ){
      return true; 
  }
  if ( ( (geom1_string.compare("upper right arm cover")==0)  &&  (geom2_string.compare("torso cover")==0) )  || ( (geom2_string.compare("upper right arm cover")==0)  &&  (geom1_string.compare("torso cover")==0) ) ){
      return true; 
  }
  if ( ( (geom1_string.compare("geom[3]")==0)  &&  (geom2_string.compare("torso cover")==0) )  || ( (geom2_string.compare("geom[3]")==0)  &&  (geom1_string.compare("torso cover")==0) ) ){
      return true; 
  } //geom[3] is the cylinder in at shoulder joint (when it is "on" - part activated, it may collide ; when off (different geom name), it will not go into the torso, so no need to handle this)
  if ( ( (geom1_string.compare("geom[5]")==0)  &&  (geom2_string.compare("torso cover")==0) )  || ( (geom2_string.compare("geom[5]")==0)  &&  (geom1_string.compare("torso cover")==0) ) ){
      return true; 
  } //geom[5] is the cylinder in upper right arm (similarly, no need to test for the version with part off (ICubSim::initRightArmOff))
  if ( ( (geom1_string.compare("geom[5]")==0)  &&  (geom2_string.compare("torsoGeom[5]")==0) )  || ( (geom2_string.compare("geom[5]")==0)  &&  (geom1_string.compare("torsoGeom[5]")==0) ) ){
      return true; 
  } //upper arm cylinder colliding with torso box
  
   /** left arm vs. torso ********/  
 if ( ( (geom1_string.compare("upper left arm cover")==0)  &&  (geom2_string.compare("torsoGeom[4]")==0) )  || ( (geom2_string.compare("upper left arm cover")==0)  &&  (geom1_string.compare("torsoGeom[4]")==0) ) ){
      return true; 
  }
  if ( ( (geom1_string.compare("upper left arm cover")==0)  &&  (geom2_string.compare("torso cover")==0) )  || ( (geom2_string.compare("upper left arm cover")==0)  &&  (geom1_string.compare("torso cover")==0) ) ){
      return true; 
  }
  if ( ( (geom1_string.compare("geom[2]")==0)  &&  (geom2_string.compare("torso cover")==0) )  || ( (geom2_string.compare("geom[2]")==0)  &&  (geom1_string.compare("torso cover")==0) ) ){
      return true; 
  } //geom[2] is the cylinder in at shoulder joint (when it is "on" - part activated, it may collide ; when off (different geom name), it will not go into the torso, so no need to handle this)
  
  if ( ( (geom1_string.compare("geom[4]")==0)  &&  (geom2_string.compare("torso cover")==0) )  || ( (geom2_string.compare("geom[4]")==0)  &&  (geom1_string.compare("torso cover")==0) ) ){
      return true; 
  } //geom[4] is the cylinder in upper left arm (similarly, no need to test for the version with part off (ICubSim::initLeftArmOff))
  if ( ( (geom1_string.compare("geom[4]")==0)  &&  (geom2_string.compare("torsoGeom[5]")==0) )  || ( (geom2_string.compare("geom[4]")==0)  &&  (geom1_string.compare("torsoGeom[5]")==0) ) ){
      return true; 
  } //upper arm cylinder colliding with torso box

  
  return false;  
} 
 
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

// this is a function to mimic the sensor data from the physical icub fingertip/palm sensors
//but the palm cover is not being checked here 
void OdeSdlSimulation::inspectHandTouch_icubSensors(Bottle& reportLeft, Bottle& reportRight, bool boolean) { 
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

            if (x < 5){ //five fingers 
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
                //these are palm taxels
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
            yWarning("Simulation delay: running %d steps in a row to recover.\n", (int)(timeCache/ode_step_length));*/

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
                    yWarning("the simulation is too slow to run in real-time, you should increase the timestep in ode_params.ini (current value: %ld, suggested value: %.0f)\n", 
                        ode_step_length, avg_ode_step_length);
                else if(avg_ode_step_length <= ode_step_length-1)
                    yWarning("you could get a more accurate dynamics simulation by decreasing the timestep in ode_params.ini (current value: %ld, suggested value: %.0f)\n", 
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

    odeinit.mtx.lock();
    nFeedbackStructs=0;
    
    if (odeinit.verbosity > 3) yDebug("\n ***info code collision detection ***"); 
    if (odeinit.verbosity > 3) yDebug("OdeSdlSimulation::ODE_process: dSpaceCollide(odeinit.space,0,&nearCallback): will test iCub space against the rest of the world (e.g. ground).\n");
    dSpaceCollide(odeinit.space,0,&nearCallback); //determines which pairs of geoms in a space may potentially intersect, and calls a callback function with each candidate pair
    if (odeinit._iCub->actSelfCol == "on"){
           if (START_SELF_COLLISION_DETECTION){ 
                if (odeinit.verbosity > 3){
                    yDebug("OdeSdlSimulation::ODE_process: dSpaceCollide(odeinit._iCub->iCub,0,&nearCallback): will test iCub subspaces against each other.");
                }
                dSpaceCollide(odeinit._iCub->iCub,0,&nearCallback); //determines which pairs of geoms in a space may potentially intersect, and calls a callback function with each candidate pair
        }
    }
    if (odeinit.verbosity > 3) yDebug("***END OF info code collision detection\n ***"); 
    
    dWorldStep(odeinit.world, dstep);
    // do 1 TIMESTEP in controllers (ok to run at same rate as ODE: 1 iteration takes about 300 times less computation time than dWorldStep)
    for (int ipart = 0; ipart<MAX_PART; ipart++) {
        if (odeinit._controls[ipart] != NULL) {
            odeinit._controls[ipart]->jointStep();
        }
    }

    // UPDATE INERTIAL

    if(odeinit._imu) {
        Bottle inertialBot;
        retreiveInertialData(inertialBot);
        odeinit._imu->updateIMUData(inertialBot);
    }


    odeinit.sync = true;
    odeinit.mtx.unlock();

    if (odeinit._iCub->actSkinEmul == "off"){
        if ( robot_streamer->shouldSendTouchLeftHand() || robot_streamer->shouldSendTouchRightHand() ) {
            Bottle reportLeft;
            Bottle reportRight;
            bool boolean = true;
            if (odeinit._iCub->actPressure == "on"){
                boolean = false;
            }    
            inspectHandTouch_icubSensors(reportLeft, reportRight, boolean);//inspectBodyTouch_continuousValued(report);
            
            if ( robot_streamer->shouldSendTouchLeftHand() )
                    robot_streamer->sendTouchLeftHand( reportLeft );

            if ( robot_streamer->shouldSendTouchRightHand() )
                robot_streamer->sendTouchRightHand( reportRight );
        }
    }
    else{ // actSkinEmul == "on"
          if(robot_streamer->shouldSendSkinEvents() || (robot_streamer->shouldSendTouchLeftHand() || robot_streamer->shouldSendTouchRightHand() ||
            robot_streamer->shouldSendTouchLeftArm() || robot_streamer->shouldSendTouchLeftForearm() || 
            robot_streamer->shouldSendTouchRightArm() || robot_streamer->shouldSendTouchRightForearm() || 
            robot_streamer->shouldSendTouchTorso())){ 
               if (! odeinit.listOfSkinContactInfos.empty()){ //if someone is reading AND there are contacts to process 
                    if (odeinit.verbosity > 2) yDebug("OdeSdlSimulation::ODE_process():There were %lu iCub collisions to process.", odeinit.listOfSkinContactInfos.size());
                    inspectWholeBodyContactsAndSendTouch(); 
               }
               else{ //someone is reading but no contacts, we send empty lists
                   if(robot_streamer->shouldSendSkinEvents()){
                        skinContactList emptySkinContactList;
                        emptySkinContactList.clear();
                        robot_streamer->sendSkinEvents(emptySkinContactList); 
                   }
                   if(robot_streamer->shouldSendTouchLeftHand()){
                         Bottle bottleLeftHand = Bottle(odeinit._iCub->emptySkinActivationHand);    
                         robot_streamer->sendTouchLeftHand(bottleLeftHand);
                   }
                   if(robot_streamer->shouldSendTouchRightHand()){
                         Bottle bottleRightHand = Bottle(odeinit._iCub->emptySkinActivationHand);  
                         robot_streamer->sendTouchRightHand(bottleRightHand);
                   }
                   if(robot_streamer->shouldSendTouchLeftArm()){
                         Bottle bottleLeftArm = Bottle(odeinit._iCub->emptySkinActivationUpperArm); 
                         robot_streamer->sendTouchLeftArm(bottleLeftArm);
                   }
                   if(robot_streamer->shouldSendTouchLeftForearm()){
                         Bottle bottleLeftForearm = Bottle(odeinit._iCub->emptySkinActivationForearm);    
                         robot_streamer->sendTouchLeftForearm(bottleLeftForearm);
                   }
                   if(robot_streamer->shouldSendTouchRightArm()){
                         Bottle bottleRightArm = Bottle(odeinit._iCub->emptySkinActivationUpperArm);
                         robot_streamer->sendTouchRightArm(bottleRightArm);
                   }
                   if(robot_streamer->shouldSendTouchRightForearm()){
                         Bottle bottleRightForearm = Bottle(odeinit._iCub->emptySkinActivationForearm);
                         robot_streamer->sendTouchRightForearm(bottleRightForearm);
                   }
                   if(robot_streamer->shouldSendTouchTorso()){
                        Bottle bottleTorso = Bottle(odeinit._iCub->emptySkinActivationTorso);
                        robot_streamer->sendTouchTorso(bottleTorso);
                   }
              }
        }
        odeinit.listOfSkinContactInfos.clear();
        if(odeinit.verbosity > 4){
            yDebug("contactICubSkinEmulMap before resetting:");
            printContactICubSkinEmulMap();
        }    
        resetContactICubSkinEmulMap();
        if(odeinit.verbosity > 4){
            yDebug("contactICubSkinEmulMap after resetting:");
            printContactICubSkinEmulMap();
        } 
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
    //yDebug("ODE=%lf\n",(double)(finishTimeODE - startTimeODE) / CLOCKS_PER_SEC);
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
  yDebug("current SPS: %d\n",currentsps);
  sps = 0.0f;
  }
  }
*/

void OdeSdlSimulation::sighandler(int sig) {
    OdeInit& odeinit = OdeInit::get();
    odeinit.stop = true;
    yInfo() << "\nCAUGHT Ctrl-c";
}

void OdeSdlSimulation::simLoop(int h,int w) {
    yDebug("***** OdeSdlSimulation::simLoop \n");
    OdeInit& odeinit = OdeInit::get();

    SDL_Init(SDL_INIT_TIMER | SDL_GL_ACCELERATED_VISUAL);
    SDL_SetVideoMode(h,w,32,SDL_OPENGL | SDL_RESIZABLE);// | SDL_SWSURFACE| SDL_ANYFORMAT); // on init 

    dAllocateODEDataForThread(dAllocateMaskAll);
    string logo = robot_config->getFinder().findFile("logo");

    image = SDL_LoadBMP(robot_config->getFinder().findFile(logo.c_str()).c_str());
    SDL_WM_SetIcon(image,0);
    SDL_FreeSurface(image);
    SDL_WM_SetCaption("iCub Simulator", "image");

    //SDL_Thread *thread;
    SDL_Thread *ode_thread = SDL_CreateThread(thread_ode, NULL);
    //thread = SDL_CreateThread(thread_func, NULL);

    if ( ode_thread == NULL ) {
        yError("Unable to create thread: %s\n", SDL_GetError());
        return;
    }

    initViewpoint();
    bool ok = setup_opengl(robot_config->getFinder());
    if (!ok) return;
    startTime = (long) clock();
    odeinit.stop = false;

    std::signal(SIGINT, sighandler);
    std::signal(SIGTERM, sighandler);

    glrun = true;
    odeinit._wrld->WAITLOADING = false;
    odeinit._wrld->static_model = false;
    long prevTime = (long) clock();
    long timeLeft;
    
    if (odeinit._iCub->actStartHomePos == "on"){
        odeinit.sendHomePos();
    }
    if (odeinit._iCub->actSelfCol == "on") {
       if (odeinit._iCub->actStartHomePos == "on"){
           Time::delay(2.0); //we want to set this trigger on only after the robot is in home pos -
            //it's initial configuration is with arms inside the thighs - generating many self-collisions
           START_SELF_COLLISION_DETECTION = true;
       }
       else{
           yWarning("the robot is not starting from HomePos and self-collision mode is on. The initial posture is already self-colliding.\n");
           START_SELF_COLLISION_DETECTION = true;
       }
    }
    
    while(!odeinit.stop) {
        /* Process incoming events. */
        process_events();
        /* Draw the screen. */
        if ( !odeinit._wrld->WAITLOADING ){
            if (glrun) {
                odeinit.mtxTexture.lock();
                draw_screen();  
                odeinit.mtxTexture.unlock();
                // check for framerate
                timeLeft = (prevTime - (long) clock()) + gl_frame_length;
                //yDebug() << "check for framerate " << timeLeft;
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
    yInfo("Stopping SDL and ODE threads...");
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
        yError("Only one Simulation object allowed\n");
        std::exit(1);
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


    string videoconf = robot_config->getFinder().findFile("video");
    options.fromConfigFile(videoconf.c_str());

    Bottle textures = *options.find("textures").asList();
    for (int i=0; i<textures.size(); i++) {
        string name = textures.get(i).asString();
        yInfo("Adding video texture %s\n", name.c_str());
        video->add(options.findGroup(name.c_str()));
    }
    
    initContactICubSkinEmulMap();
   
};

void OdeSdlSimulation::initContactICubSkinEmulMap(void)
{

    contactICubSkinEmul_t skin_emul_struct;
    
    //SKIN_LEFT_HAND
    skin_emul_struct.coverTouched = false; //for the hand, this comprises also fingertips - they are treated like covers
    skin_emul_struct.indivTaxelResolution = true;
    contactICubSkinEmulMap[SKIN_LEFT_HAND]=skin_emul_struct;
    
    //SKIN_LEFT_FOREARM
    skin_emul_struct.coverTouched = false;
    skin_emul_struct.indivTaxelResolution = true;
    contactICubSkinEmulMap[SKIN_LEFT_FOREARM]=skin_emul_struct;
    
    //SKIN_LEFT_UPPER_ARM
    skin_emul_struct.coverTouched = false;
    skin_emul_struct.indivTaxelResolution = false;
    contactICubSkinEmulMap[SKIN_LEFT_UPPER_ARM]=skin_emul_struct;
    
    //SKIN_RIGHT_HAND
    skin_emul_struct.coverTouched = false; //for the hand, this comprises also fingertips - they are treated like covers
    skin_emul_struct.indivTaxelResolution = true;
    contactICubSkinEmulMap[SKIN_RIGHT_HAND]=skin_emul_struct;
    
    //SKIN_RIGHT_FOREARM
    skin_emul_struct.coverTouched = false;
    skin_emul_struct.indivTaxelResolution = true;
    contactICubSkinEmulMap[SKIN_RIGHT_FOREARM]=skin_emul_struct;
    
    //SKIN_RIGHT_UPPER_ARM
    skin_emul_struct.coverTouched = false;
    skin_emul_struct.indivTaxelResolution = false;
    contactICubSkinEmulMap[SKIN_RIGHT_UPPER_ARM]=skin_emul_struct;
 
    //SKIN_FRONT_TORSO
    skin_emul_struct.coverTouched = false;
    skin_emul_struct.indivTaxelResolution = false;
    contactICubSkinEmulMap[SKIN_FRONT_TORSO]=skin_emul_struct;
   
    //LEFT_LEG_UPPER
    skin_emul_struct.coverTouched = false;
    skin_emul_struct.indivTaxelResolution = false;
    contactICubSkinEmulMap[LEFT_LEG_UPPER]=skin_emul_struct;
    
     //LEFT_LEG_LOWER
    skin_emul_struct.coverTouched = false;
    skin_emul_struct.indivTaxelResolution = false;
    contactICubSkinEmulMap[LEFT_LEG_LOWER]=skin_emul_struct;
    
     //LEFT_FOOT
    skin_emul_struct.coverTouched = false;
    skin_emul_struct.indivTaxelResolution = false;
    contactICubSkinEmulMap[LEFT_FOOT]=skin_emul_struct;
    
     //RIGHT_LEG_UPPER
    skin_emul_struct.coverTouched = false;
    skin_emul_struct.indivTaxelResolution = false;
    contactICubSkinEmulMap[RIGHT_LEG_UPPER]=skin_emul_struct;
    
     //RIGHT_LEG_LOWER
    skin_emul_struct.coverTouched = false;
    skin_emul_struct.indivTaxelResolution = false;
    contactICubSkinEmulMap[RIGHT_LEG_LOWER]=skin_emul_struct;
    
     //RIGHT_FOOT
    skin_emul_struct.coverTouched = false;
    skin_emul_struct.indivTaxelResolution = false;
    contactICubSkinEmulMap[RIGHT_FOOT]=skin_emul_struct;
    

}

void OdeSdlSimulation::resetContactICubSkinEmulMap(void)
{

    //SKIN_LEFT_HAND
    contactICubSkinEmulMap[SKIN_LEFT_HAND].coverTouched=false;
    contactICubSkinEmulMap[SKIN_LEFT_HAND].taxelsTouched.clear();
    
    //SKIN_LEFT_FOREARM
    contactICubSkinEmulMap[SKIN_LEFT_FOREARM].coverTouched=false;
    contactICubSkinEmulMap[SKIN_LEFT_FOREARM].taxelsTouched.clear();
    
    //SKIN_LEFT_UPPER_ARM
    contactICubSkinEmulMap[SKIN_LEFT_UPPER_ARM].coverTouched=false;
    contactICubSkinEmulMap[SKIN_LEFT_UPPER_ARM].taxelsTouched.clear();
    
    //SKIN_RIGHT_HAND
    contactICubSkinEmulMap[SKIN_RIGHT_HAND].coverTouched=false;
    contactICubSkinEmulMap[SKIN_RIGHT_HAND].taxelsTouched.clear();
    
    //SKIN_RIGHT_FOREARM
    contactICubSkinEmulMap[SKIN_RIGHT_FOREARM].coverTouched=false;
    contactICubSkinEmulMap[SKIN_RIGHT_FOREARM].taxelsTouched.clear();
    
    //SKIN_RIGHT_UPPER_ARM
    contactICubSkinEmulMap[SKIN_RIGHT_UPPER_ARM].coverTouched=false;
    contactICubSkinEmulMap[SKIN_RIGHT_UPPER_ARM].taxelsTouched.clear();
    
    //SKIN_FRONT_TORSO
    contactICubSkinEmulMap[SKIN_FRONT_TORSO].coverTouched=false;
    contactICubSkinEmulMap[SKIN_FRONT_TORSO].taxelsTouched.clear();
    
    //LEFT_LEG_UPPER
    contactICubSkinEmulMap[LEFT_LEG_UPPER].coverTouched=false;
    contactICubSkinEmulMap[LEFT_LEG_UPPER].taxelsTouched.clear();
    
    //LEFT_LEG_LOWER
    contactICubSkinEmulMap[LEFT_LEG_LOWER].coverTouched=false;
    contactICubSkinEmulMap[LEFT_LEG_LOWER].taxelsTouched.clear();
    
     //LEFT_FOOT
    contactICubSkinEmulMap[LEFT_FOOT].coverTouched=false;
    contactICubSkinEmulMap[LEFT_FOOT].taxelsTouched.clear();
    
     //RIGHT_LEG_UPPER
    contactICubSkinEmulMap[RIGHT_LEG_UPPER].coverTouched=false;
    contactICubSkinEmulMap[RIGHT_LEG_UPPER].taxelsTouched.clear();
    
     //RIGHT_LEG_LOWER
    contactICubSkinEmulMap[RIGHT_LEG_LOWER].coverTouched=false;
    contactICubSkinEmulMap[RIGHT_LEG_LOWER].taxelsTouched.clear();
    
     //RIGHT_FOOT
    contactICubSkinEmulMap[RIGHT_FOOT].coverTouched=false;
    contactICubSkinEmulMap[RIGHT_FOOT].taxelsTouched.clear();
}

void OdeSdlSimulation::printContactICubSkinEmulMap(void)
{
     std::set<unsigned int> taxels_touched;
     yDebug("OdeSdlSimulation::printContactICubSkinEmulMap");
     for (std::map<SkinPart,contactICubSkinEmul_t>::const_iterator it=contactICubSkinEmulMap.begin(); it!=contactICubSkinEmulMap.end(); ++it){
         yDebug("key: %d, %s,cover touched: %d, indivTaxelResolution: %d, list of taxel IDs:", it->first,SkinPart_s[it->first].c_str(),it->second.coverTouched,it->second.indivTaxelResolution);
         taxels_touched = it->second.taxelsTouched;
         for (std::set<unsigned int>::const_iterator taxel_it = taxels_touched.begin(); taxel_it!=taxels_touched.end(); ++taxel_it){
                yDebug("%d ",*taxel_it);
         }
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
        //yDebug(stdout,"torques... %lf \n",odeinit._iCub->torqueData[s]);
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

void OdeSdlSimulation::inspectWholeBodyContactsAndSendTouch()
{
      //SkinDynLib enums
      SkinPart skinPart;    // id of the part of the skin (e.g. SKIN_LEFT_FOREARM; from skinDynLib/common.h)
      BodyPart bodyPart;    // id of the body part
      HandPart handPart;    // id of the hand part  - INDEX, MIDDLE, RING, LITTLE, THUMB, PALM, ALL_HAND_PARTS, HAND_PART_SIZE
     
      //coordinate transformations for skinEvents and for emulating ind. taxel groups per skin part
      Vector geoCenter_SIM_FoR_forHomo(4,0.0), normal_SIM_FoR_forHomo(4,0.0);
      Vector force_SIM_FoR_forHomo(4,0.0), moment_SIM_FoR_forHomo(4,0.0);
      Vector v1(4,0.0); //auxilliary vector
      Vector geoCenter_link_FoR(3,0.0), normal_link_FoR(3,0.0);
      Vector force_link_FoR(3,0.0), moment_link_FoR(3,0.0);
      double forceOnBody_magnitude; 
      double left_arm_encoders[16], right_arm_encoders[16], torso_encoders[3], head_encoders[6];
      Vector left_arm_for_iKin(10,0.0), right_arm_for_iKin(10,0.0), inertial_for_iKin(6,0.0);
      Matrix T_root_to_link = yarp::math::zeros(4,4);
      Matrix T_link_to_root = yarp::math::zeros(4,4);
      std::vector<unsigned int> taxel_list;  
      bool upper_body_transforms_available = false;
      
      bool skinCoverFlag = false;
      bool fingertipFlag = true;
      OdeInit& odeinit = OdeInit::get();
      skinContactList mySkinContactList;
      mySkinContactList.clear();
          
      if ((odeinit._iCub->actHead=="off") || (odeinit._iCub->actTorso=="off") || (odeinit._iCub->actLArm=="off") || (odeinit._iCub->actRArm=="off")){
        upper_body_transforms_available = false;
        yWarning("With self-collisions on but head/torso/left_arm/right_arm off, the upper body transforms are unavailable and skinContactList can't be created.");
      }
      else{
           upper_body_transforms_available = true;
          
           odeinit._controls[PART_ARM_LEFT]->getEncodersRaw(left_arm_encoders); //! do not use the BODY_PART enums from skinDynLib to index here - in the simulator the order of "controls[]" is different
           odeinit._controls[PART_ARM_RIGHT]->getEncodersRaw(right_arm_encoders);
           odeinit._controls[PART_TORSO]->getEncodersRaw(torso_encoders);
           odeinit._controls[PART_HEAD]->getEncodersRaw(head_encoders); //first three are probably neck joints, then the eyes
           for (int j=0;j<TORSO_DOF;j++){
               left_arm_for_iKin(j)=torso_encoders[j]; //first 3 joints - 0 to 2 - in iKin arm are torso joints
               right_arm_for_iKin(j)=torso_encoders[j];
               inertial_for_iKin(j)=torso_encoders[j];
           }
           for (int l=0;l<7;l++){
               left_arm_for_iKin(l+TORSO_DOF) = left_arm_encoders[l]; // then we put seven arm joints (we ignore the rest of the encoders up to 16 - these are fingers)
               right_arm_for_iKin(l+TORSO_DOF) = right_arm_encoders[l]; 
           }
           for (int m=0;m<3;m++){
               inertial_for_iKin(m+TORSO_DOF) = head_encoders[m]; //we put the second three - the neck joints and ignore the rest of head_encoders (the eyes)
           }
           odeinit._iCub->iKinLeftArm.setAng(left_arm_for_iKin);
           odeinit._iCub->iKinRightArm.setAng(right_arm_for_iKin);
           odeinit._iCub->iKinInertialSensor.setAng(inertial_for_iKin);
      }
      
      if (odeinit.verbosity > 4) yDebug("OdeSdlSimulation::inspectWholeBodyContactsAndSendTouch:There were %lu iCub collisions to process.", odeinit.listOfSkinContactInfos.size());
      //main loop through all the contacts
      for (list<OdeInit::contactOnSkin_t>::iterator it = odeinit.listOfSkinContactInfos.begin(); it!=odeinit.listOfSkinContactInfos.end(); it++){
          skinPart = SKIN_PART_UNKNOWN; bodyPart = BODY_PART_UNKNOWN;  handPart = ALL_HAND_PARTS; skinCoverFlag = false; fingertipFlag = false;
          taxel_list.clear();
          odeinit._iCub->getSkinAndBodyPartFromSpaceAndGeomID((*it).body_geom_space_id,(*it).body_geom_id,skinPart,bodyPart,handPart,skinCoverFlag,fingertipFlag);
          if(upper_body_transforms_available){
              geoCenter_SIM_FoR_forHomo.zero(); geoCenter_SIM_FoR_forHomo(3)=1.0; //setting the extra row to 1 - for multiplication by homogenous rototransl. matrix
              normal_SIM_FoR_forHomo.zero(); normal_SIM_FoR_forHomo(3)=1.0; 
              force_SIM_FoR_forHomo.zero(); force_SIM_FoR_forHomo(3)=1.0; 
              moment_SIM_FoR_forHomo.zero(); moment_SIM_FoR_forHomo(3)=1.0;
              geoCenter_link_FoR.zero();normal_link_FoR.zero();
              moment_link_FoR.zero();force_link_FoR.zero();
              forceOnBody_magnitude=0.0;
              T_root_to_link.zero(); T_link_to_root.zero();
              for (int i=0;i<3;i++){
                  geoCenter_SIM_FoR_forHomo(i)= (*it).contact_geom.pos[i]; //in global (i.e. simulator) coordinates
                  normal_SIM_FoR_forHomo(i) = (*it).contact_geom.normal[i];
              }
              dJointFeedback * fb = dJointGetFeedback ((*it).contact_joint);
              if (fb==NULL){
                  yDebug("Warning:OdeSdlSimulation::inspectWholeBodyContactsAndSendTouch: This joint (at %d skin part) has no feedback structure defined - contact force not available: setting to -1.",skinPart); 
                  forceOnBody_magnitude = -1;
              }
              else{
              //yDebug("OdeSdlSimulation::processWholeBodyCollisions: joint feedback structure:\n.");
              //yDebug("f1 force vector in simulator FoR: %f %f %f \n",fb->f1[0],fb->f1[1],fb->f1[2]); // assuming it is global ODE FoR ~ simulator FoR
              //yDebug("f2 force vector: %f %f %f \n",fb->f2[0],fb->f2[1],fb->f2[2]);
              //f2 force vector has same magnitude but opposite direction than f1
                  for(int k=0;k<3;k++){
                      if((*it).body_index == 1){
                          force_SIM_FoR_forHomo(k)=fb->f1[k];
                          moment_SIM_FoR_forHomo(k)=fb->t1[k];
                      }
                      else if((*it).body_index == 2){
                          force_SIM_FoR_forHomo(k)=fb->f2[k];
                          moment_SIM_FoR_forHomo(k)=fb->t2[k];
                      }
                      else{
                          yError("OdeSdlSimulation::inspectWholeBodyContactsAndSendTouch: unexpected body_index for colliding body: %d.\n",(*it).body_index);
                      }
                }
                  forceOnBody_magnitude=sqrt(force_SIM_FoR_forHomo(0)*force_SIM_FoR_forHomo(0) + force_SIM_FoR_forHomo(1)*force_SIM_FoR_forHomo(1) 
                      + force_SIM_FoR_forHomo(2)*force_SIM_FoR_forHomo(2)); 
              }
              //Let's do all the transformations
              //Assuming, dJointFeedback and contact_geom data from ODE are in global ODE frame; the contact_geom.pos is the position; the contact_geom.normal and the dJointFeedback
              // vectors (f1, m1) are originating from the global origin, i.e. they need to be translated to contact_geom.pos;
              //see the post in ode-users group "dJointFeedback and dContactGeom reference frame", 6.12.2013; local FoR of the contact point; 
                        
              switch(bodyPart){
                  case LEFT_ARM:
                      T_root_to_link = odeinit._iCub->iKinLeftArm.getH(SkinPart_2_LinkNum[skinPart].linkNum + TORSO_DOF);
                      //e.g. skinPart LEFT_UPPER_ARM gives link number 2, which means we ask iKin for getH(2+3), which gives us  FoR 6 - at the first elbow joint, which is the FoR for the upper arm 
                      break;
                  case RIGHT_ARM:
                      T_root_to_link = odeinit._iCub->iKinRightArm.getH(SkinPart_2_LinkNum[skinPart].linkNum + TORSO_DOF);
                      break;
                  case TORSO:
                      T_root_to_link = odeinit._iCub->iKinInertialSensor.getH(SkinPart_2_LinkNum[skinPart].linkNum); 
                      // SkinPart_2_LinkNum[SKIN_FRONT_TORSO].linkNum  is 2, this should give us the FoR 3 - the first neck joint which is the expected torso FoR
                      //- check " SKIN torso 2" in iCub/main/app/iCubGui/skeleton.ini
                      //- importantly, this needs to be the iKinInertialSensor, not the iKin Arm; 
                      break;
                  default:
                      if (odeinit.verbosity > 0) yDebug("OdeSdlSimulation::processWholeBodyCollisions: FoR transforms to BODY PART %d not implemented yet\n",bodyPart);
                          continue;
              }
              T_link_to_root = SE3inv(T_root_to_link);
                    
              v1.zero();      
              v1 = T_link_to_root * (odeinit._iCub->H_r2w) * geoCenter_SIM_FoR_forHomo; //first transform to robot coordinates, then transform to local FoR of respective body part
              geoCenter_link_FoR = v1.subVector(0,2); //strip the last one away
                  
              v1.zero();
              v1 = T_link_to_root * (odeinit._iCub->H_r2w) * normal_SIM_FoR_forHomo; 
              normal_link_FoR = v1.subVector(0,2);
              
              v1.zero();
              v1 = T_link_to_root * (odeinit._iCub->H_r2w) * force_SIM_FoR_forHomo;
              force_link_FoR = v1.subVector(0,2); 
                
              v1.zero();
              v1 = T_link_to_root * (odeinit._iCub->H_r2w) * moment_SIM_FoR_forHomo;
              moment_link_FoR = v1.subVector(0,2); 
                
              //Note that the normal, force, and moment are just carrying the orientation (and apart from the normal also magnitude) - they will still need to be translated to the 
              //appropariate CoP / geoCenter to make the arrow to the taxel
              //Note also the dJointFeedback force vector does not necessarily point along the normal at the contact point (which points into the colliding body) - as is a sum of the 
              //forces along the normal and frictional forces perpendicular to the normal
              //alternatively, I could just take the magnitude from the force and send the normal as the direction
                
              //yDebug("Contact coordinates in ODE / SIM FoR: %s\n",geoCenter_SIM_FoR_forHomo.subVector(0,2).toString().c_str());
              Vector temp_v4(4,0.0);
              temp_v4 =  (odeinit._iCub->H_r2w) * geoCenter_SIM_FoR_forHomo;
              //yDebug("Contact coordinates in robot root FoR: %s\n",temp_v4.subVector(0,2).toString().c_str());
              //yDebug("Left arm for iKin:\n %s \n",left_arm_for_iKin.toString().c_str());
              //yDebug("Rototranslation matrix root to link:\n %s\n",T_root_to_link.toString().c_str());
              //yDebug("Contact coordinates in link FoR: %s\n",geoCenter_link_FoR.toString().c_str());
              /*for (int l=0;l<2;l++){ geoCenter_link_FoR(l)=0.0;  force_link_FoR(l)=1.0;  normal_link_FoR(l)=1.0; moment_link_FoR(l)=1.0;
              } */
              //forceOnBody_magnitude=10.0;
              if (contactICubSkinEmulMap[skinPart].indivTaxelResolution && (skinCoverFlag || fingertipFlag)){ //indiv taxels get emulated only on covers - where the actual skin is 
                if(skinCoverFlag){
                    mapPositionIntoTaxelList(skinPart,geoCenter_link_FoR,taxel_list); 
                }
                else if(fingertipFlag){
                    mapFingertipIntoTaxelList(handPart,taxel_list);   
                }
              }
              else{    
                  taxel_list.push_back(FAKE_TAXEL_ID); // we will emulate one non-existent activated "taxel" per contact joint - say taxel "10000"
              }
              skinContact c(bodyPart, skinPart, getLinkNum(skinPart), geoCenter_link_FoR, geoCenter_link_FoR,taxel_list, forceOnBody_magnitude, normal_link_FoR,force_link_FoR,moment_link_FoR);       
              //we have only one source of information - the contact as detected by ODE - therefore, we take the coordinates and set them both to CoP 
              //(which is supposed to come from the dynamic estimation) and as geoCenter (from skin); Similarly, we derive the pressure directly from the force vector from ODE.
              if (odeinit.verbosity > 4) yDebug("Creating skin contact as follows: %s.\n",c.toString().c_str());
              mySkinContactList.push_back(c); 
          } //if(upper_body_transforms_available){
          // here we collect the info for emulating the skin ports (compensated tactile ports) 
          if(skinCoverFlag || fingertipFlag){ 
                //if it was a cover (including palm cover) or fingertip that was touched, we will append the taxels touched to respective contactICubSkinEmulMap
                contactICubSkinEmulMap[skinPart].coverTouched = true;
                if (contactICubSkinEmulMap[skinPart].indivTaxelResolution){
                    if (!taxel_list.empty()){
                        unsigned int first_taxel_in_list = taxel_list[0];
                        if (first_taxel_in_list != FAKE_TAXEL_ID){
                            for (std::vector<unsigned int>::const_iterator it = taxel_list.begin() ; it != taxel_list.end(); ++it){
                                contactICubSkinEmulMap[skinPart].taxelsTouched.insert(*it); //inserting the taxel IDs into the set
                            }                 
                        }
                    }
                }
          }
      } //cycle through odeinit.listOfSkinContactInfos
      
      //all contacts have been processed, now we produce the output
      
      if(robot_streamer->shouldSendSkinEvents()){ //note that these are generated here for any body parts - not only those that have tactile sensors in the real robot
        // the contacts can be visualized using the icubGui (not skinGui) 
          robot_streamer->sendSkinEvents(mySkinContactList); //we send even if empty
      }  
   
      //for hands, this is now done differently than in the original inspectTouch_icubSensors, where finger bodies were inspected, whether they have contact joints attached to them
      // the palm cover replaces sensing in the palm body
      //now all info about contacts has come from cycling through the odeinit.listOfSkinContactInfos above and it has beem filled into appropriate structs
      //the output of actual pressure values is discontinued; 
      int y=0;
      if(robot_streamer->shouldSendTouchLeftHand()){
            Bottle bottleLeftHand;
            if (contactICubSkinEmulMap[SKIN_LEFT_HAND].coverTouched){ 
                //prepare the bottle
                //first 60 are fingers
                if (contactICubSkinEmulMap[SKIN_LEFT_HAND].indivTaxelResolution){
                    for (y = 0; y<=59; y++){ 
                        if (contactICubSkinEmulMap[SKIN_LEFT_HAND].taxelsTouched.count(y)){ // if element (taxel ID) is in the set, count returns 1
                            bottleLeftHand.addDouble(255.0); 
                        }
                        else{
                            bottleLeftHand.addDouble(0.0); 
                        } 
                    }
                }
                else{ //we fill them all
                   for (y = 0; y<=59; y++){ 
                        bottleLeftHand.addDouble(255); //we ignore the thermal pad positions which should be 0s for now
                }
                    }
                //zero padding - the port output: 61-96 zeros; taxel IDs 60-95
                for (y = 60; y<=95; y++){           
                        bottleLeftHand.addDouble(0.0);
                }
                
                //pam - positions 97-144 palm taxels; taxel IDs have index by one lower (inside these, IDs 107, 119, 131, and 139 are thermal pads ~ 0s); 
                    if (contactICubSkinEmulMap[SKIN_LEFT_HAND].indivTaxelResolution){
                    for (y = 96; y<=143; y++){ 
                            if (contactICubSkinEmulMap[SKIN_LEFT_HAND].taxelsTouched.count(y)){ // if element (taxel ID) is in the set, count returns 1
                                bottleLeftHand.addDouble(255.0); 
                            }
                            else{
                                 bottleLeftHand.addDouble(0.0); 
                            }
                        }
                    }
                    else{ //we fill the whole palm
                        for (int y = 96; y<=143; y++){ 
                        bottleLeftHand.addDouble(255.0); //we ignore the thermal pad positions, which should be 0s, for now
                        }
                    }
                //filling the rest: 145-192 zeros. IDs: 144-191
                for (int y = 144; y<=191; y++){ 
                    bottleLeftHand.addDouble(0.0);
                }
            }
            else{
                bottleLeftHand = Bottle(odeinit._iCub->emptySkinActivationHand);
            }
            robot_streamer->sendTouchLeftHand(bottleLeftHand);
      }
        
      
      if(robot_streamer->shouldSendTouchRightHand()){
          Bottle bottleRightHand;
            if (contactICubSkinEmulMap[SKIN_RIGHT_HAND].coverTouched){ 
                //prepare the bottle
                //first 60 are fingers
                if (contactICubSkinEmulMap[SKIN_RIGHT_HAND].indivTaxelResolution){
                    for (y = 0; y<=59; y++){ 
                        if (contactICubSkinEmulMap[SKIN_RIGHT_HAND].taxelsTouched.count(y)){ // if element (taxel ID) is in the set, count returns 1
                            bottleRightHand.addDouble(255.0); 
                          }
                          else{
                            bottleRightHand.addDouble(0.0); 
                          } 
                          }
                  }
                else{ //we fill them all
                   for (y = 0; y<=59; y++){ 
                        bottleRightHand.addDouble(255); //we ignore the thermal pad positions which should be 0s for now
              }
                  }
                //zero padding - the port output: 61-96 zeros; taxel IDs 60-95
                for (y = 60; y<=95; y++){           
                      bottleRightHand.addDouble(0.0);
              }
                
                //pam - positions 97-144 palm taxels; taxel IDs have index by one lower (inside these, IDs 107, 119, 131, and 139 are thermal pads ~ 0s); 
                  if (contactICubSkinEmulMap[SKIN_RIGHT_HAND].indivTaxelResolution){
                    for (y = 96; y<=143; y++){ 
                          if (contactICubSkinEmulMap[SKIN_RIGHT_HAND].taxelsTouched.count(y)){ // if element (taxel ID) is in the set, count returns 1
                                bottleRightHand.addDouble(255.0); 
                            }
                            else{
                                 bottleRightHand.addDouble(0.0); 
                            }
                        }
                  }
                  else{ //we fill the whole palm
                        for (int y = 96; y<=143; y++){ 
                        bottleRightHand.addDouble(255.0); //we ignore the thermal pad positions, which should be 0s, for now
                        }
                  }                               
                  //filling the rest: 145-192 zeros. IDs: 144-191
                  for (int y = 144; y<=191; y++){ 
                    bottleRightHand.addDouble(0.0);
                  }
            }
        else{
            bottleRightHand = Bottle(odeinit._iCub->emptySkinActivationHand);
        }
        // yDebug("bottleRightHand: %s \n",bottleRightHand.toString().c_str());  
        // yDebug("bottleRightHand: %s \n",bottleRightHand.toString().c_str());  
        robot_streamer->sendTouchRightHand(bottleRightHand);
     }
        
        
     if(robot_streamer->shouldSendTouchLeftArm()){
         Bottle bottleLeftArm;
         if (contactICubSkinEmulMap[SKIN_LEFT_UPPER_ARM].coverTouched){
             if (contactICubSkinEmulMap[SKIN_LEFT_UPPER_ARM].indivTaxelResolution){
                for (int y = 0; y<=767; y++){ 
                    if (contactICubSkinEmulMap[SKIN_LEFT_UPPER_ARM].taxelsTouched.count(y)){ // if element (taxel ID) is in the set, count returns 1
                             bottleLeftArm.addDouble(255.0); 
                    }
                    else{
                             bottleLeftArm.addDouble(0.0); 
                    }
                }
             }
             else{ //we fill the whole upper arm 
                  bottleLeftArm = Bottle(odeinit._iCub->fullSkinActivationUpperArm);
             }     
         }
         else{
             bottleLeftArm = Bottle(odeinit._iCub->emptySkinActivationUpperArm);
         }
         robot_streamer->sendTouchLeftArm(bottleLeftArm);
     }
     if(robot_streamer->shouldSendTouchLeftForearm()){
         Bottle bottleLeftForearm;
          if (contactICubSkinEmulMap[SKIN_LEFT_FOREARM].coverTouched){
             if (contactICubSkinEmulMap[SKIN_LEFT_FOREARM].indivTaxelResolution){
                for (int y = 0; y<=383; y++){ 
                    if (contactICubSkinEmulMap[SKIN_LEFT_FOREARM].taxelsTouched.count(y)){ // if element (taxel ID) is in the set, count returns 1
                             bottleLeftForearm.addDouble(255.0); 
                    }
                    else{
                             bottleLeftForearm.addDouble(0.0); 
                    }
                }
             }
             else{ //we fill the whole forearm 
                  bottleLeftForearm = Bottle(odeinit._iCub->fullSkinActivationForearm);
             }     
         }
         else{
             bottleLeftForearm = Bottle(odeinit._iCub->emptySkinActivationForearm);
         }
         robot_streamer->sendTouchLeftForearm(bottleLeftForearm);
     }
     if(robot_streamer->shouldSendTouchRightArm()){
         Bottle bottleRightArm;
         if (contactICubSkinEmulMap[SKIN_RIGHT_UPPER_ARM].coverTouched){
             if (contactICubSkinEmulMap[SKIN_RIGHT_UPPER_ARM].indivTaxelResolution){
                for (int y = 0; y<=767; y++){ 
                    if (contactICubSkinEmulMap[SKIN_RIGHT_UPPER_ARM].taxelsTouched.count(y)){ // if element (taxel ID) is in the set, count returns 1
                        bottleRightArm.addDouble(255.0); 
                    }
                    else{
                        bottleRightArm.addDouble(0.0); 
                    }
                }
             }
             else{ //we fill the whole upper arm 
                  bottleRightArm = Bottle(odeinit._iCub->fullSkinActivationUpperArm);
             }     
         }
         else{
             bottleRightArm = Bottle(odeinit._iCub->emptySkinActivationUpperArm);
         }
         robot_streamer->sendTouchRightArm(bottleRightArm);
      }
      if(robot_streamer->shouldSendTouchRightForearm()){
         Bottle bottleRightForearm;
          if (contactICubSkinEmulMap[SKIN_RIGHT_FOREARM].coverTouched){
             if (contactICubSkinEmulMap[SKIN_RIGHT_FOREARM].indivTaxelResolution){
                for (int y = 0; y<=383; y++){ 
                    if (contactICubSkinEmulMap[SKIN_RIGHT_FOREARM].taxelsTouched.count(y)){ // if element (taxel ID) is in the set, count returns 1
                             bottleRightForearm.addDouble(255.0); 
                    }
                    else{
                             bottleRightForearm.addDouble(0.0); 
                    }
                }
             }
             else{ //we fill the whole forearm 
                  bottleRightForearm = Bottle(odeinit._iCub->fullSkinActivationForearm);
             }     
         }
         else{
             bottleRightForearm = Bottle(odeinit._iCub->emptySkinActivationForearm);
         }
         robot_streamer->sendTouchRightForearm(bottleRightForearm);
      }
      if(robot_streamer->shouldSendTouchTorso()){
         Bottle bottleTorso;
         if (contactICubSkinEmulMap[SKIN_FRONT_TORSO].coverTouched){
             if (contactICubSkinEmulMap[SKIN_FRONT_TORSO].indivTaxelResolution){
                for (int y = 0; y<=767; y++){ 
                    if (contactICubSkinEmulMap[SKIN_FRONT_TORSO].taxelsTouched.count(y)){ // if element (taxel ID) is in the set, count returns 1
                             bottleTorso.addDouble(255.0); 
                    }
                    else{
                             bottleTorso.addDouble(0.0); 
                    }
                }
             }
             else{ //we fill the whole torso 
                  bottleTorso = Bottle(odeinit._iCub->fullSkinActivationTorso);
             }     
         }
         else{
             bottleTorso = Bottle(odeinit._iCub->emptySkinActivationTorso);
         }
         robot_streamer->sendTouchTorso(bottleTorso);
     } 
}

     
void OdeSdlSimulation::mapPositionIntoTaxelList(const SkinPart skin_part,const Vector geo_center_link_FoR,std::vector<unsigned int>& list_of_taxels){
 
  
   // EXTRA_MARGIN_FOR_TAXEL_POSITION_M = 0.03; //for skin emulation we get the coordinates of the collision and contact with skin cover from ODE; 
   //after transforming to local reference frame of respective skin part, we emulate which set of taxels would get activated at that position; 
   //however, with errors in the position, we need an extra margin, so the contact falls onto some taxels
    switch (skin_part){
        case SKIN_LEFT_HAND:
            if ((geo_center_link_FoR[0]<=0.003+MORE_EXTRA_MARGIN_FOR_TAXEL_POSITION_M) && (geo_center_link_FoR[0]>=-0.014) && (geo_center_link_FoR[1]>=-0.026-EXTRA_MARGIN_FOR_TAXEL_POSITION_M-1.5*MORE_EXTRA_MARGIN_FOR_TAXEL_POSITION_M) && (geo_center_link_FoR[1]<=-0.0055)){
                list_of_taxels.push_back(121);list_of_taxels.push_back(122);list_of_taxels.push_back(123);
                list_of_taxels.push_back(124);list_of_taxels.push_back(125);list_of_taxels.push_back(126);
                list_of_taxels.push_back(127);list_of_taxels.push_back(128);
                //list_of_taxels.push_back();list_of_taxels.push_back();list_of_taxels.push_back();
            }
            else if ((geo_center_link_FoR[0]<=0.003+MORE_EXTRA_MARGIN_FOR_TAXEL_POSITION_M) && (geo_center_link_FoR[0]>=-0.014) && (geo_center_link_FoR[1]>=-0.0055) && (geo_center_link_FoR[1]<=0.01)){
                list_of_taxels.push_back(96);list_of_taxels.push_back(97);list_of_taxels.push_back(98); 
                list_of_taxels.push_back(99);list_of_taxels.push_back(102);list_of_taxels.push_back(103);
                list_of_taxels.push_back(120);list_of_taxels.push_back(129);list_of_taxels.push_back(130);
            }
            else if ((geo_center_link_FoR[0]<=0.003+MORE_EXTRA_MARGIN_FOR_TAXEL_POSITION_M) && (geo_center_link_FoR[0]>=-0.014) && (geo_center_link_FoR[1]>=0.01) && (geo_center_link_FoR[1]<=0.03+EXTRA_MARGIN_FOR_TAXEL_POSITION_M) ){
                    list_of_taxels.push_back(100);list_of_taxels.push_back(101);list_of_taxels.push_back(104);  
                    list_of_taxels.push_back(105);list_of_taxels.push_back(106);list_of_taxels.push_back(113);  
                    list_of_taxels.push_back(116);list_of_taxels.push_back(117);  
            }
            else if ((geo_center_link_FoR[0]<=-0.014) && (geo_center_link_FoR[0]>=-0.024) && (geo_center_link_FoR[1]>=0.0-EXTRA_MARGIN_FOR_TAXEL_POSITION_M-2*MORE_EXTRA_MARGIN_FOR_TAXEL_POSITION_M) && (geo_center_link_FoR[1]<=0.03+EXTRA_MARGIN_FOR_TAXEL_POSITION_M) ){
                    list_of_taxels.push_back(108);list_of_taxels.push_back(109);list_of_taxels.push_back(110);  
                    list_of_taxels.push_back(111);list_of_taxels.push_back(112);list_of_taxels.push_back(114);  
                    list_of_taxels.push_back(115);list_of_taxels.push_back(118); list_of_taxels.push_back(142); 
                    list_of_taxels.push_back(143);
            }   
            else if ((geo_center_link_FoR[0]<=-0.024) && (geo_center_link_FoR[0]>=-0.04-EXTRA_MARGIN_FOR_TAXEL_POSITION_M-2.0*MORE_EXTRA_MARGIN_FOR_TAXEL_POSITION_M) && (geo_center_link_FoR[1]>=0.0-EXTRA_MARGIN_FOR_TAXEL_POSITION_M-2*MORE_EXTRA_MARGIN_FOR_TAXEL_POSITION_M) && (geo_center_link_FoR[1]<=0.03+EXTRA_MARGIN_FOR_TAXEL_POSITION_M) ){
                    list_of_taxels.push_back(132);list_of_taxels.push_back(133);list_of_taxels.push_back(134);  
                    list_of_taxels.push_back(135);list_of_taxels.push_back(136);list_of_taxels.push_back(137);  
                    list_of_taxels.push_back(138);list_of_taxels.push_back(140); list_of_taxels.push_back(141);                   
            } 
            else{
                yWarning("OdeSdlSimulation::mapPositionIntoTaxelList: WARNING: contact at part: %d, coordinates: %f %f %f, but no taxels asigned to this position. \n",skin_part,geo_center_link_FoR[0],geo_center_link_FoR[1],geo_center_link_FoR[2]); 
            }
            break;
         case SKIN_RIGHT_HAND:
            if ((geo_center_link_FoR[0]<=0.003+MORE_EXTRA_MARGIN_FOR_TAXEL_POSITION_M) && (geo_center_link_FoR[0]>=-0.014) && (geo_center_link_FoR[1]>=-0.026-EXTRA_MARGIN_FOR_TAXEL_POSITION_M-1.5*MORE_EXTRA_MARGIN_FOR_TAXEL_POSITION_M) && (geo_center_link_FoR[1]<=-0.0055)){
                list_of_taxels.push_back(120);list_of_taxels.push_back(121);list_of_taxels.push_back(122);
                list_of_taxels.push_back(123);list_of_taxels.push_back(124);list_of_taxels.push_back(125);
                list_of_taxels.push_back(126);list_of_taxels.push_back(128);
                //list_of_taxels.push_back();list_of_taxels.push_back();list_of_taxels.push_back();
            }
            else if ((geo_center_link_FoR[0]<=0.003+MORE_EXTRA_MARGIN_FOR_TAXEL_POSITION_M) && (geo_center_link_FoR[0]>=-0.014) && (geo_center_link_FoR[1]>=-0.0055) && (geo_center_link_FoR[1]<=0.01)){
                list_of_taxels.push_back(99);list_of_taxels.push_back(102);list_of_taxels.push_back(103); 
                list_of_taxels.push_back(104);list_of_taxels.push_back(105);list_of_taxels.push_back(106);
                list_of_taxels.push_back(127);list_of_taxels.push_back(129);list_of_taxels.push_back(130);
            }
            else if ((geo_center_link_FoR[0]<=0.003+MORE_EXTRA_MARGIN_FOR_TAXEL_POSITION_M) && (geo_center_link_FoR[0]>=-0.014) && (geo_center_link_FoR[1]>=0.01) && (geo_center_link_FoR[1]<=0.03+EXTRA_MARGIN_FOR_TAXEL_POSITION_M) ){
                    list_of_taxels.push_back(96);list_of_taxels.push_back(97);list_of_taxels.push_back(98);  
                    list_of_taxels.push_back(100);list_of_taxels.push_back(101);list_of_taxels.push_back(110);  
                    list_of_taxels.push_back(111);list_of_taxels.push_back(112);  
            }
            else if ((geo_center_link_FoR[0]<=-0.014) && (geo_center_link_FoR[0]>=-0.024) && (geo_center_link_FoR[1]>=0.0-EXTRA_MARGIN_FOR_TAXEL_POSITION_M-2*MORE_EXTRA_MARGIN_FOR_TAXEL_POSITION_M) && (geo_center_link_FoR[1]<=0.03+EXTRA_MARGIN_FOR_TAXEL_POSITION_M) ){
                    list_of_taxels.push_back(108);list_of_taxels.push_back(109);list_of_taxels.push_back(113);  
                    list_of_taxels.push_back(114);list_of_taxels.push_back(115);list_of_taxels.push_back(116);  
                    list_of_taxels.push_back(117);list_of_taxels.push_back(118); list_of_taxels.push_back(142); 
                    list_of_taxels.push_back(143);
            }   
            else if ((geo_center_link_FoR[0]<=-0.024) && (geo_center_link_FoR[0]>=-0.040-EXTRA_MARGIN_FOR_TAXEL_POSITION_M-2.0*MORE_EXTRA_MARGIN_FOR_TAXEL_POSITION_M) && (geo_center_link_FoR[1]>=0.0-EXTRA_MARGIN_FOR_TAXEL_POSITION_M-2*MORE_EXTRA_MARGIN_FOR_TAXEL_POSITION_M) && (geo_center_link_FoR[1]<=0.03+EXTRA_MARGIN_FOR_TAXEL_POSITION_M) ){
                    list_of_taxels.push_back(132);list_of_taxels.push_back(133);list_of_taxels.push_back(134);  
                    list_of_taxels.push_back(135);list_of_taxels.push_back(136);list_of_taxels.push_back(137);  
                    list_of_taxels.push_back(138);list_of_taxels.push_back(140); list_of_taxels.push_back(141);                   
            } 
            else{
                yWarning("OdeSdlSimulation::mapPositionIntoTaxelList: WARNING: contact at part: %d, coordinates: %f %f %f, but no taxels asigned to this position. \n",skin_part,geo_center_link_FoR[0],geo_center_link_FoR[1],geo_center_link_FoR[2]); 
            }
            break;
          case SKIN_LEFT_FOREARM:
             //upper small patch (7 triangles in V1 skin)
             if((geo_center_link_FoR[0]>=-0.0326) && (geo_center_link_FoR[0]<=0.0326) && (geo_center_link_FoR[1]>=-0.0528) && (geo_center_link_FoR[1]<=0.0039) && (geo_center_link_FoR[2]>=-0.0538) && (geo_center_link_FoR[2]<=0.0)){
                //triangle taxel IDs 288-299
                pushTriangleToTaxelList(288,list_of_taxels); //pushes taxel IDs of whole triangle into list_of_taxels, starting from startingTaxelID and skipping 7th and 11th taxels (thermal pads)
                //triangle 300-311
                pushTriangleToTaxelList(300,list_of_taxels);
                //triangle 348-359 
                pushTriangleToTaxelList(348,list_of_taxels);
             }
             else if((geo_center_link_FoR[0]>=-0.0545) && (geo_center_link_FoR[0]<=0.0) && (geo_center_link_FoR[1]>=-0.1288) && (geo_center_link_FoR[1]<=-0.0528) && (geo_center_link_FoR[2]>=-0.0569) && (geo_center_link_FoR[2]<=0.0)){
                //triangle 204:215
                pushTriangleToTaxelList(204,list_of_taxels);
                //triangle 336:347
                pushTriangleToTaxelList(336,list_of_taxels);              
             }
             else if((geo_center_link_FoR[0]>=0.0) && (geo_center_link_FoR[0]<=0.0545) && (geo_center_link_FoR[1]>=-0.1288) && (geo_center_link_FoR[1]<=-0.0528) && (geo_center_link_FoR[2]>=-0.0569) && (geo_center_link_FoR[2]<=0.0)){
                //triangle 252:263
                pushTriangleToTaxelList(252,list_of_taxels);
                //triangle 312:323
                pushTriangleToTaxelList(312,list_of_taxels);
             }
             
              ///////////////////////////////////////////////////////////////////////
             
             //lower patch - big (16 triangles)
             else if((geo_center_link_FoR[0]>=-0.0375) && (geo_center_link_FoR[0]<=0.0) && (geo_center_link_FoR[1]>=-0.0716) && (geo_center_link_FoR[1]<=0.0) && (geo_center_link_FoR[2]>=0.0281) && (geo_center_link_FoR[2]<=0.0484)){
                //triangle nr. 12 in CAD, taxel IDs 132:143
                pushTriangleToTaxelList(132,list_of_taxels);
                //triangle 16 168:179
                pushTriangleToTaxelList(168,list_of_taxels);
             }
             else  if((geo_center_link_FoR[0]>=-0.0375) && (geo_center_link_FoR[0]<=0.0) && (geo_center_link_FoR[1]>=-0.1281) && (geo_center_link_FoR[1]<=-0.0716) && (geo_center_link_FoR[2]>=0.0343) && (geo_center_link_FoR[2]<=0.0526)){
                //triangle 3, 156:167
                pushTriangleToTaxelList(156,list_of_taxels);
                //triangle 8, 144:155
                pushTriangleToTaxelList(144,list_of_taxels);                
             }
             else if((geo_center_link_FoR[0]>=-0.0375) && (geo_center_link_FoR[0]<=0.0) && (geo_center_link_FoR[1]>=-0.1333) && (geo_center_link_FoR[1]<=-0.0716) && (geo_center_link_FoR[2]>=0.0) && (geo_center_link_FoR[2]<=0.0343)){
                //triangle 4, 24:35
                pushTriangleToTaxelList(24,list_of_taxels);
                //triangle 6, 12:23
                pushTriangleToTaxelList(12,list_of_taxels);             
             }    
             else if((geo_center_link_FoR[0]>=-0.0375) && (geo_center_link_FoR[0]<=0.0) && (geo_center_link_FoR[1]>=-0.0716) && (geo_center_link_FoR[1]<=0.0) && (geo_center_link_FoR[2]>=0.0) && (geo_center_link_FoR[2]<=0.0281)){
                //triangle 10, 0:11
                pushTriangleToTaxelList(0,list_of_taxels);
                //triangle 14, 180:191
                pushTriangleToTaxelList(180,list_of_taxels);
             }    
             else if((geo_center_link_FoR[0]>=0.0) && (geo_center_link_FoR[0]<=0.0375) && (geo_center_link_FoR[1]>=-0.0716) && (geo_center_link_FoR[1]<=0.0) && (geo_center_link_FoR[2]>=0.0281) && (geo_center_link_FoR[2]<=0.0484)){
                //triangle 11, 120:131
                pushTriangleToTaxelList(120,list_of_taxels);
                //triangle 15, 60:71
                pushTriangleToTaxelList(60,list_of_taxels);
             }
             else if((geo_center_link_FoR[0]>=0.0) && (geo_center_link_FoR[0]<=0.0375) && (geo_center_link_FoR[1]>=-0.1281) && (geo_center_link_FoR[1]<=-0.0716) && (geo_center_link_FoR[2]>=0.0343) && (geo_center_link_FoR[2]<=0.0526)){
                //triangle 2, 96:107
                pushTriangleToTaxelList(96,list_of_taxels);
                //triangle 7, 108:119
                pushTriangleToTaxelList(108,list_of_taxels);              
             }
             else if((geo_center_link_FoR[0]>=0.0) && (geo_center_link_FoR[0]<=0.0375) && (geo_center_link_FoR[1]>=-0.1333) && (geo_center_link_FoR[1]<=-0.0716) && (geo_center_link_FoR[2]>=0.0) && (geo_center_link_FoR[2]<=0.0343)){
                //triangle 1, 84:95
                pushTriangleToTaxelList(84,list_of_taxels);
                //triangle 5, 72:83
                pushTriangleToTaxelList(72,list_of_taxels);
             }
             else if((geo_center_link_FoR[0]>=0.0) && (geo_center_link_FoR[0]<=0.0375) && (geo_center_link_FoR[1]>=-0.0716) && (geo_center_link_FoR[1]<=0.0) && (geo_center_link_FoR[2]>=0.0) && (geo_center_link_FoR[2]<=0.0281)){
                //triangle 9, 36:47
                pushTriangleToTaxelList(36,list_of_taxels);
                //triangle 13, 48:59
                pushTriangleToTaxelList(48,list_of_taxels);
             }
             else{
                yWarning("OdeSdlSimulation::mapPositionIntoTaxelList: WARNING: contact at part: %d, coordinates: %f %f %f, but no taxels asigned to this position. \n",skin_part,geo_center_link_FoR[0],geo_center_link_FoR[1],geo_center_link_FoR[2]); 
             }
             break;
        case SKIN_RIGHT_FOREARM: //the y and z axes have opposite directions between left and right forearm FoR
             //upper small patch (7 triangles in V1 skin)
             if((geo_center_link_FoR[0]>=-0.0326) && (geo_center_link_FoR[0]<=0.0326) && (geo_center_link_FoR[1]>=-0.0039) && (geo_center_link_FoR[1]<=0.0528) && (geo_center_link_FoR[2]>=0.0) && (geo_center_link_FoR[2]<=0.0538)){
                //triangle taxel IDs 288-299
                pushTriangleToTaxelList(288,list_of_taxels); //pushes taxel IDs of whole triangle into list_of_taxels, starting from startingTaxelID and skipping 7th and 11th taxels (thermal pads)
                //triangle 300-311
                pushTriangleToTaxelList(300,list_of_taxels);
                //triangle 348-359 
                pushTriangleToTaxelList(348,list_of_taxels);
             }
             else if((geo_center_link_FoR[0]>=-0.0545) && (geo_center_link_FoR[0]<=0.0) && (geo_center_link_FoR[1]>=0.0528) && (geo_center_link_FoR[1]<=0.1288) && (geo_center_link_FoR[2]>=0.0) && (geo_center_link_FoR[2]<=0.0569)){
                //triangle 204:215
                pushTriangleToTaxelList(204,list_of_taxels);
                //triangle 336:347
                pushTriangleToTaxelList(336,list_of_taxels);              
             }
             else if((geo_center_link_FoR[0]>=0.0) && (geo_center_link_FoR[0]<=0.0545) && (geo_center_link_FoR[1]>=0.0528) && (geo_center_link_FoR[1]<=0.1288) && (geo_center_link_FoR[2]>=0.0) && (geo_center_link_FoR[2]<=0.0569)){
                //triangle 252:263
                pushTriangleToTaxelList(252,list_of_taxels);
                //triangle 312:323
                pushTriangleToTaxelList(312,list_of_taxels);
             }
             
              ///////////////////////////////////////////////////////////////////////
             
             //lower patch - big (16 triangles)
             else if((geo_center_link_FoR[0]>=-0.0375) && (geo_center_link_FoR[0]<=0.0) && (geo_center_link_FoR[1]>=0.0) && (geo_center_link_FoR[1]<=0.0716) && (geo_center_link_FoR[2]>=-0.0484) && (geo_center_link_FoR[2]<=-0.0281)){
                //triangle nr. 12 in CAD, taxel IDs 132:143
                pushTriangleToTaxelList(132,list_of_taxels);
                //triangle 16 168:179
                pushTriangleToTaxelList(168,list_of_taxels);
             }
             else  if((geo_center_link_FoR[0]>=-0.0375) && (geo_center_link_FoR[0]<=0.0) && (geo_center_link_FoR[1]>=0.0716) && (geo_center_link_FoR[1]<=0.1281) && (geo_center_link_FoR[2]>=-0.0526) && (geo_center_link_FoR[2]<=-0.0343)){
                //triangle 3, 156:167
                pushTriangleToTaxelList(156,list_of_taxels);
                //triangle 8, 144:155
                pushTriangleToTaxelList(144,list_of_taxels);                
             }
             else if((geo_center_link_FoR[0]>=-0.0375) && (geo_center_link_FoR[0]<=0.0) && (geo_center_link_FoR[1]>=0.0716) && (geo_center_link_FoR[1]<=0.1333) && (geo_center_link_FoR[2]>=-0.0343) && (geo_center_link_FoR[2]<=0.0)){
                //triangle 4, 24:35
                pushTriangleToTaxelList(24,list_of_taxels);
                //triangle 6, 12:23
                pushTriangleToTaxelList(12,list_of_taxels);             
             }    
             else if((geo_center_link_FoR[0]>=-0.0375) && (geo_center_link_FoR[0]<=0.0) && (geo_center_link_FoR[1]>=0.0) && (geo_center_link_FoR[1]<=0.0716) && (geo_center_link_FoR[2]>=-0.0281) && (geo_center_link_FoR[2]<=0.0)){
                //triangle 10, 0:11
                pushTriangleToTaxelList(0,list_of_taxels);
                //triangle 14, 180:191
                pushTriangleToTaxelList(180,list_of_taxels);
             }    
             else if((geo_center_link_FoR[0]>=0.0) && (geo_center_link_FoR[0]<=0.0375) && (geo_center_link_FoR[1]>=0.0) && (geo_center_link_FoR[1]<=0.0716) && (geo_center_link_FoR[2]>=-0.0484) && (geo_center_link_FoR[2]<=-0.0281)){
                //triangle 11, 120:131
                pushTriangleToTaxelList(120,list_of_taxels);
                //triangle 15, 60:71
                pushTriangleToTaxelList(60,list_of_taxels);
             }
             else if((geo_center_link_FoR[0]>=0.0) && (geo_center_link_FoR[0]<=0.0375) && (geo_center_link_FoR[1]>=0.0716) && (geo_center_link_FoR[1]<=0.1281) && (geo_center_link_FoR[2]>=-0.0526) && (geo_center_link_FoR[2]<=-0.0343)){
                //triangle 2, 96:107
                pushTriangleToTaxelList(96,list_of_taxels);
                //triangle 7, 108:119
                pushTriangleToTaxelList(108,list_of_taxels);              
             }
             else if((geo_center_link_FoR[0]>=0.0) && (geo_center_link_FoR[0]<=0.0375) && (geo_center_link_FoR[1]>=0.0716) && (geo_center_link_FoR[1]<=0.1333) && (geo_center_link_FoR[2]>=-0.0343) && (geo_center_link_FoR[2]<=0.0)){
                //triangle 1, 84:95
                pushTriangleToTaxelList(84,list_of_taxels);
                //triangle 5, 72:83
                pushTriangleToTaxelList(72,list_of_taxels);
             }
             else if((geo_center_link_FoR[0]>=0.0) && (geo_center_link_FoR[0]<=0.0375) && (geo_center_link_FoR[1]>=0.0) && (geo_center_link_FoR[1]<=0.0716) && (geo_center_link_FoR[2]>=-0.0281) && (geo_center_link_FoR[2]<=0.0)){
                //triangle 9, 36:47
                pushTriangleToTaxelList(36,list_of_taxels);
                //triangle 13, 48:59
                pushTriangleToTaxelList(48,list_of_taxels);
             }
             else{
                yWarning("OdeSdlSimulation::mapPositionIntoTaxelList: WARNING: contact at part: %d, coordinates: %f %f %f, but no taxels asigned to this position. \n",skin_part,geo_center_link_FoR[0],geo_center_link_FoR[1],geo_center_link_FoR[2]); 
             }
             break;     
             
          default:  
            yWarning("OdeSdlSimulation::mapPositionIntoTaxelList: WARNING: contact at part: %d, but no taxel resolution implemented for this skin part. \n",skin_part); 
    }
    
//      if (odeinit.verbosity > 2) {
//          yDebug("OdeSdlSimulation::mapPositionIntoTaxelList: contact at part: %d, coordinates: %f %f %f. \n",skin_part,geo_center_link_FoR[0],geo_center_link_FoR[1],geo_center_link_FoR[2]); 
//          yDebug("    Taxel list: ");
//          for (std::vector<unsigned int>::const_iterator it = list_of_taxels.begin() ; it != list_of_taxels.end(); ++it){
//                 yDebug("%d,",*it);
//          }
//          yDebug("\n");
//      }
     return;
}

//pushes taxel IDs of whole triangle into list_of_taxels, starting from startingTaxelID and skipping 7th and 11th taxels (thermal pads)
void OdeSdlSimulation::pushTriangleToTaxelList(const int startingTaxelID,std::vector<unsigned int>& list_of_taxels)
{
    int i = startingTaxelID;
    for (i=startingTaxelID;i<startingTaxelID+6;i++){
        list_of_taxels.push_back(i);
    }
    //skipping 7th and 11th taxel - thermal pads 
    for (i = startingTaxelID + 7; i < startingTaxelID + 10; i++){
        list_of_taxels.push_back(i);
    }
    list_of_taxels.push_back(startingTaxelID+11);
}

void OdeSdlSimulation::mapFingertipIntoTaxelList(const HandPart hand_part,std::vector<unsigned int>& list_of_taxels)
{
    int i=0;
    switch(hand_part)
    {
        case INDEX:
            for(i=0; i<=11; i++){
                list_of_taxels.push_back(i);
            }
            break;
        case MIDDLE:
            for(i=12; i<=23; i++){
                list_of_taxels.push_back(i);
            }
            break;
        case RING:
            for(i=24; i<=35; i++){
                list_of_taxels.push_back(i);
            }
            break;    
        case LITTLE:
            for(i=36; i<=47; i++){
                list_of_taxels.push_back(i);
            }
            break;
        case THUMB:
            for(i=48; i<=59; i++){
                list_of_taxels.push_back(i);
            }
            break;    
        default:    
            printf("Warning: OdeSdlSimulation::mapFingertipIntoTaxelList: unexpected HandPart: %d. Pushing fake taxel ID \n",hand_part);
            list_of_taxels.push_back(FAKE_TAXEL_ID);
        
    }
    
    
}

//Auxiliary function to print class of geom - according to section 9.5 of ODE manual
std::string OdeSdlSimulation::getGeomClassName(const int geom_class,std::string & s)
{
  switch(geom_class){
      case 0: 
        s = "sphere";
        break;
      case 1: 
        s = "box";
        break;
      case 2: 
        s = "capsule";
        break;
      case 3: 
        s = "cylinder";
        break;
      case 4: 
        s = "plane";
        break;  
      case 8: 
        s= "triangle mesh";
        break;  
      case 10:
      case 11:  
        s = "simple space";
        break;  
      case 12:  
        s="hash space";
        break;  
      default: 
        s ="unknown type";
        break;
    } 
  return s;
 
}
