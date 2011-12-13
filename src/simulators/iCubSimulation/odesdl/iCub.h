// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Vadim Tikhanoff
* email:  vadim.tikhanoff@iit.it
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

/**
 * \file iCub.h
 * \brief The iCub header file  
 * \author Vadim Tikhanoff
 * \date 2007
 * \note Released under GNU GPL v2.0
 **/
#ifndef ICUBSIMULATION_ICUB_INC
#define ICUBSIMULATION_ICUB_INC

#include <iostream>
#include <map>
#include "SDL.h"
#include "SDL_opengl.h"
#include "rendering.h"

#include <yarp/sig/Vector.h>

#include <ode/ode.h>
#include <string>
#include "RobotConfig.h"

#include "EyeLidsController.h"


#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif 
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

class ICubData{ 
public:	
    //some Object constants....
    #define MAX_CONTACTS 10// maximum number of contact points per body
    #define NUM 10          // max number of objects

    ConstString configPath;
    ICubData();
};

class ICubSim : public ICubData {
public:

    EyeLids *eyeLids;
    static const bool textured = true;
    ConstString actElevation, actStartHomePos, actLegs, actTorso, actLArm, actRArm, actLHand, actRHand, actHead, actfixedHip, actVision, actHeadCover, actWorld, actPressure, actScreen, actLegsCovers, actLeftArmCovers, actRightArmCovers, actTorsoCovers;
    double elev;

    dGeomID screenGeom;
    bool     reinitialized;
    float    eyeLidRot;
    string   eyeLidsPortName;

    dSpaceID    iCub;
    dGeomID     geom_cube[1];

    dBodyID     inertialBody;
    dGeomID     inertialGeom;
    dBodyID     iCubHead;
    dGeomID     iCubHeadGeom;
    /*----Lower Body parts----*/
    dBodyID     leftLeg[20];
    dGeomID     leftLegGeom[20];
    //for encapsulated objects
    dGeomID     leftLeg_2_1,leftLeg_2_2;
    dGeomID     leftLeg_3_1,leftLeg_3_2;
    dGeomID     leftLeg_4_1,leftLeg_4_2;

    dBodyID     rightLeg[20];
    dGeomID     rightLegGeom[20];
    dGeomID     rightLeg_2_1,rightLeg_2_2;
    dGeomID     rightLeg_3_1,rightLeg_3_2;
    dGeomID     rightLeg_4_1,rightLeg_4_2;

    dBodyID     torso[9];
    dGeomID     torsoGeom[9];

    dBodyID     body[50];
    dGeomID     geom[50];

    dBodyID     neck[2];
    dGeomID     neckgeom[2];

    dGeomID     lhandfings0_geom,lhandfings1_geom;
    dBodyID     lhandfingers0;

    dGeomID     lhandfings2_geom,lhandfings3_geom;
    dBodyID     lhandfingers1;

    dGeomID     lhandfings4_geom,lhandfings5_geom;
    dBodyID     lhandfingers2;

    dGeomID     lhandfings6_geom,lhandfings7_geom;
    dBodyID     lhandfingers3;

    dGeomID     rhandfings0_geom,rhandfings1_geom;
    dBodyID     rhandfingers0;

    dGeomID     rhandfings2_geom,rhandfings3_geom;
    dBodyID     rhandfingers1;

    dGeomID     rhandfings4_geom,rhandfings5_geom;
    dBodyID     rhandfingers2;

    dGeomID     rhandfings6_geom,rhandfings7_geom;
    dBodyID     rhandfingers3;

    //init. encapsulated object-the left+right leg
    dGeomID     l_leg0_geom,l_leg1_geom,l_leg2_geom,l_leg3_geom,l_leg4_geom,l_leg5_geom,l_leg6_geom,l_leg7_geom,l_leg8_geom;
    dGeomID     r_leg0_geom,r_leg1_geom,r_leg2_geom,r_leg3_geom,r_leg4_geom,r_leg5_geom,r_leg6_geom,r_leg7_geom,r_leg8_geom;
    dBodyID     legs;
    //init. encapsulated object-the torso
    dGeomID     torso0_geom,torso1_geom,torso1b_geom,torso2_geom,torso3_geom,torso4_geom,torso5_geom;
    dBodyID     body_torso;
    //init. encapsulated object-the left arm
    dGeomID     larm0_geom,larm1_geom,larm2_geom,larm3_geom,larm4_geom,larm5_geom;
    dBodyID     larm;
    //init. encapsulated object-the right arm
    dGeomID     rarm0_geom,rarm1_geom,rarm2_geom,rarm3_geom,rarm4_geom,rarm5_geom;
    dBodyID     rarm;
    //init. encapsulated object-the hand if user selects it
    dGeomID     l_hand0_geom, l_hand1_geom, l_hand2_geom, l_hand3_geom, l_hand4_geom, l_hand5_geom;
    dGeomID     r_hand0_geom, r_hand1_geom, r_hand2_geom, r_hand3_geom, r_hand4_geom, r_hand5_geom;
    dBodyID     l_hand, r_hand;

    //init. encapsulated object-the head
    dGeomID     head0_geom, head1_geom, head2_geom, head3_geom,head4_geom,head5_geom,head6_geom,head7_geom,head8_geom;
    dGeomID     neck0_geom, neck1_geom;
    dBodyID     head;

    //init encapsulated object_the EYE
    dGeomID     eye1, eye2,eye3,eye4,eye5;
    dGeomID     eye1_geom,eye2_geom,eye3_geom,eye4_geom,eye5_geom;
    dBodyID     eye;

    //init. encapsulated object - the left eye
    dGeomID     List1_L_E, List2_L_E;
    dGeomID     Leye1_geom, Leye2_geom;
    dBodyID     leye;

    //init. encapsulated object - the right eye
    dGeomID     List1_R_E,List2_R_E;
    dGeomID     Reye1_geom,Reye2_geom;
    dBodyID     reye;

    //init. eye lids
    dGeomID     topEyeLid_geom;
    dGeomID     bottomEyeLid_geom;
    dBodyID     topEyeLid;
    dBodyID     bottomEyeLid;

    /*----Lower Body joints----*/
    dJointID    elevJoint;

    dJointID    LLegjoints [20];
    dJointID    RLegjoints [20];

    dJointID    Torsojoints[8];
    dJointID    LAjoints [25];
    dJointID    RAjoints [25];

    dJointID    Hjoints [6];
    dJointID    inertialJoint;


    dJointID    grab;
    dJointID    grab1;

    dJointID    fixedHipJoint;

    /*----Lower Body joints speeds----*/
    dReal    LLeg_speed	[20];
    dReal    RLeg_speed	[20];

    dReal    Torso_speed[8];
    dReal    la_speed[25];
    dReal    la_speed1[25];
    dReal    ra_speed[25];
    dReal    ra_speed1[25];
    dReal    h_speed[25];

    map <string, ConstString> model;
    ConstString textureName[100];
    map <string, dTriMeshDataID> model_TriData;
    map <string, dTriMeshX> model_trimesh;

    int modelTexture[100];
    int modelTextureIndex;
    int numCovers;

    double torqueData[100];

    class iCubCovers {
    public:
        dBodyID body;
        dGeomID geom;
        virtual dBodyID getBody()const {return body;}
        virtual dGeomID getGeom()const {return geom;}

        void reloadTexture(ConstString texture, const int &modelTexture)
        {
            //ConstString tmptext = (char *) model_DIR.c_str();
            //ConstString texture = ("C:/DEV/iCub/app/simConfig/models/blueCovers.bmp");
            setupTexture( (char* ) texture.c_str(), modelTexture);
        }

    };
    map<string, iCubCovers> model_ThreeD_obj;

    /*------ Joints Positions------*/
    Vector          jP_torso[3];
    Vector          jP_leftArm[8];
    Vector          jP_rightArm[8];
    Vector          jP_head[4];
    Vector          jP_leftEye[2];
    Vector          jP_rightEye[2];
    Vector          jP_leftLeg[6];
    Vector          jP_rightLeg[6];
    Vector          jP_inertial;

    ICubSim(dWorldID world, dSpaceID space, dReal X, dReal Y, dReal Z,
            RobotConfig& config);

    ~ICubSim();

    void resetSpeeds();
    void setJointSpeeds();
    void setJointTorques();
    //void syncAngles();

    bool checkTouchSensor(int bodyToCheck);
    bool checkTouchSensor(dBodyID id);
    double checkTouchSensor_continuousValued(int bodyToCheck);
    double checkTouchSensor_continuousValued(dBodyID id);
    void draw();

    private:
    //int inc;
    bool loadJointPosition(const char *joints_path);
    void setPosition(dReal agentX, dReal agentY, dReal agentZ );
    void init( dWorldID world, dSpaceID space, dReal X, dReal Y, dReal Z,
               RobotConfig& config);
    void activateiCubParts(RobotConfig& config);
};


#endif
