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
#include "OdeInit.h"

#include "EyeLidsController.h"

//these were added for the self-collision and skin emulation 
#include "iCub/skinDynLib/skinContactList.h"
#include <iCub/ctrl/math.h>
#include <yarp/math/Math.h>
#include <map>  
#include <iCub/iKin/iKinFwd.h>
#include <yarp/sig/Matrix.h>

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif 
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

  using namespace yarp::math;
  using namespace iCub::skinDynLib;
  using namespace iCub::ctrl;

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
    ConstString actElevation, actStartHomePos, actLegs, actTorso, actLArm, actRArm, actLHand, actRHand, actHead, actfixedHip, actVision, actHeadCover, actWorld, actPressure, actScreen, actLegsCovers, actLeftArmCovers, actRightArmCovers, actTorsoCovers, actSelfCol, actCoversCol, actSkinEmul;
    double elev;

    dGeomID screenGeom;
    bool     reinitialized;
    float    eyeLidRot;
    string   eyeLidsPortName;

   dSpaceID    iCub;
   dSpaceID iCubHeadSpace, iCubTorsoSpace, iCubLeftArmSpace, iCubRightArmSpace, iCubLegsSpace; //these are needed for the iCub self-collisions; 
   //nevertheless, if actSelfCol == off, then they will all be set to iCub
   std::map<dSpaceID, string> dSpaceNames; //needed for development and testing of self-collisions
   std::map<dGeomID, string> dGeomNames; 
   
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

    /*---- joint speeds----*/
    dReal    LLeg_speed	[20];
    dReal    RLeg_speed	[20];
    dReal    Torso_speed[8];
    dReal    la_speed[25];
    dReal    la_speed1[25];
    dReal    ra_speed[25];
    dReal    ra_speed1[25];
    dReal    h_speed[25];

    /*---- joint torques----*/
    dReal    LLeg_torques[20];
    dReal    RLeg_torques[20];
    dReal    Torso_torques[8];
    dReal    la_torques[25];
    dReal    ra_torques[25];
    dReal    h_torques[25];

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
    /*------- Joint Axes ----------*/
    Vector          jA_torso[3];
    Vector          jA_leftArm[8];
    Vector          jA_rightArm[8];
    
     //We make these class variables, besides joint position initialization (loadJointPosition()), they will be used repeatedly in the self-collision mode in ODE_process
    iCub::iKin::iCubArm iKinLeftArm, iKinRightArm;
    iCub::iKin::iCubInertialSensor iKinInertialSensor; //needed to get FoR 3 in the kinematics - the first neck joint -  FoR for the skin of the torso
    
    // Preset bottles with empty or full activation of some skin parts that can be sent to a port
    Bottle emptySkinActivationHand;
    Bottle emptySkinActivationForearm;
    Bottle emptySkinActivationUpperArm;
    Bottle emptySkinActivationTorso;
    Bottle fullSkinActivationForearm;
    Bottle fullSkinActivationUpperArm;
    Bottle fullSkinActivationTorso;
    
    // rototranslation form robot root to simulation world reference frame and vice versa
    Matrix H_r2w, H_w2r;
    
    ICubSim(dWorldID world, dSpaceID space, dReal X, dReal Y, dReal Z,
            RobotConfig& config);

    ~ICubSim();

    void resetSpeeds();
    
    /**
    * Set the control action for all the joints, that can be either a velocity
    * command or a torque command, depending on the current control mode.
    */
    void setJointControlAction();
    //void syncAngles();

    bool checkTouchSensor(int bodyToCheck);
    bool checkTouchSensor(dBodyID id);
    double checkTouchSensor_continuousValued(int bodyToCheck);
    double checkTouchSensor_continuousValued(dBodyID id);
    void draw();
    
    void getSkinAndBodyPartFromSpaceAndGeomID(const dSpaceID geomSpaceID, const dGeomID geomID, SkinPart& skinPart,BodyPart& bodyPart,
                                              HandPart& handPart, bool& skinCoverFlag, bool& fingertipFlag,
                                              std::string& linkName, std::string& frameName);
    static void printPositionOfGeom(dGeomID geomID);
    static void printPositionOfBody(dBodyID bodyID);

    private:
    //int inc;
    bool loadJointPosition(const char *joints_path);
    void setPosition(dReal agentX, dReal agentY, dReal agentZ );
    void init( dWorldID world, dSpaceID space, dReal X, dReal Y, dReal Z,
               RobotConfig& config);
    void activateiCubParts(RobotConfig& config);

    void initLegsOff(dWorldID world, dSpaceID subspace);
    void initLegsOn(dWorldID world, dSpaceID subspace);
    void initTorsoOff(dWorldID world, dSpaceID subspace);
    void initTorsoOn(dWorldID world, dSpaceID subspace);
    void initLeftArmOff(dWorldID world, dSpaceID subspace);
    void initLeftArmOn(dWorldID world, dSpaceID subspace);
    void initRightArmOff(dWorldID world, dSpaceID subspace);
    void initRightArmOn(dWorldID world, dSpaceID subspace);
    void initLeftHandOff(dWorldID world, dSpaceID subspace);
    void initLeftHandOn(dWorldID world, dSpaceID subspace);
    void initRightHandOff(dWorldID world, dSpaceID subspace);
    void initRightHandOn(dWorldID world, dSpaceID subspace);
    void initHead(dWorldID world, dSpaceID subspace);
    void initEyes(dWorldID world, dSpaceID subspace);
    void initCovers(ResourceFinder& finder);

    void initLegJoints();
    void initTorsoJoints(OdeParams &p);
    void initLeftArmJoints(OdeParams &p);
    void initRightArmJoints(OdeParams &p);
    void initLeftHandJoints();
    void initRightHandJoints();
    void initHeadJoints();
    
    void init_iKin();
    void initSkinActivationBottles();
};


#endif
