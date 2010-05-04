// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Vadim Tikhanoff
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

/**
 * \file iCub.h
 * \brief The iCub header file  
 * \author Vadim Tikhanoff
 * \date 2007
 * \note Release under GNU GPL v2.0
 **/
#pragma once
#include "SDL.h"
#include "SDL_opengl.h"
#include "rendering.h" 
#include "MS3D.h"
#include "xloader.h"
#include <ode/ode.h>

#include "EyeLidsController.h"

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif 
using namespace yarp::os;

class ICubData{ 
public:	
	//some Object constants....
	#define MAX_CONTACTS 10		// maximum number of contact points per body
	#define NUM 10			// max number of objects

	ConstString configPath;
ICubData();
};

class ICubSim : public ICubData {
public:
    EyeLids *eyeLids;
    static const bool textured = true;
	ConstString actElevation, actLegs, actTorso, actLArm, actRArm, actLHand, actRHand, actHead, actfixedHip, actVision, actCover, actWorld;
    double elev;
	
//    dGeomID screenGeom;

	bool reinitialized;
	float eyeLidRot;

    int inc;

	dSpaceID iCub;
	//dBodyID body_cube[1000];
	dGeomID geom_cube[1];

	dBodyID iCubHead;
	dGeomID iCubHeadGeom;
	/*----Lower Body parts----*/
	dBodyID		leftLeg[20];
	dGeomID		leftLegGeom[20];
	//for encapsulated objects
	dGeomID     leftLeg_2_1,leftLeg_2_2;
	dGeomID  	leftLeg_3_1,leftLeg_3_2;
	dGeomID  	leftLeg_4_1,leftLeg_4_2;

	dBodyID		rightLeg[20];
	dGeomID		rightLegGeom[20];
	dGeomID     rightLeg_2_1,rightLeg_2_2;
	dGeomID  	rightLeg_3_1,rightLeg_3_2;
	dGeomID  	rightLeg_4_1,rightLeg_4_2;

	dBodyID		torso[8];
	dGeomID		torsoGeom[8];

	dBodyID		body[50];
	dGeomID		geom[50];

	dBodyID neck[2];
	dGeomID neckgeom[2];

	dGeomID lhandfings0_geom,lhandfings1_geom;
	dBodyID lhandfingers0;
	
	dGeomID lhandfings2_geom,lhandfings3_geom;
	dBodyID lhandfingers1;

	dGeomID lhandfings4_geom,lhandfings5_geom;
	dBodyID lhandfingers2;

	dGeomID lhandfings6_geom,lhandfings7_geom;
	dBodyID lhandfingers3;

	dGeomID rhandfings0_geom,rhandfings1_geom;
	dBodyID rhandfingers0;
	
	dGeomID rhandfings2_geom,rhandfings3_geom;
	dBodyID rhandfingers1;

	dGeomID rhandfings4_geom,rhandfings5_geom;
	dBodyID rhandfingers2;

	dGeomID rhandfings6_geom,rhandfings7_geom;
	dBodyID rhandfingers3;

	//init. encapsulated object-the left+right leg
	dGeomID l_leg0_geom,l_leg1_geom,l_leg2_geom,l_leg3_geom,l_leg4_geom,l_leg5_geom,l_leg6_geom,l_leg7_geom,l_leg8_geom;
	dGeomID r_leg0_geom,r_leg1_geom,r_leg2_geom,r_leg3_geom,r_leg4_geom,r_leg5_geom,r_leg6_geom,r_leg7_geom,r_leg8_geom;
	dBodyID legs;
	//init. encapsulated object-the torso
	dGeomID torso0_geom,torso1_geom,torso2_geom,torso3_geom,torso4_geom,torso5_geom;
	dBodyID body_torso;
	//init. encapsulated object-the left arm
	dGeomID larm0_geom,larm1_geom,larm2_geom,larm3_geom,larm4_geom,larm5_geom;
	dBodyID larm;
	//init. encapsulated object-the right arm
	dGeomID rarm0_geom,rarm1_geom,rarm2_geom,rarm3_geom,rarm4_geom,rarm5_geom;
	dBodyID rarm;
	//init. encapsulated object-the hand if user selects it
	dGeomID l_hand0_geom, l_hand1_geom, l_hand2_geom, l_hand3_geom, l_hand4_geom, l_hand5_geom;
	dGeomID r_hand0_geom, r_hand1_geom, r_hand2_geom, r_hand3_geom, r_hand4_geom, r_hand5_geom;
	dBodyID l_hand, r_hand;

	//init. encapsulated object-the head
	dGeomID head0_geom, head1_geom, head2_geom, head3_geom,head4_geom,head5_geom,head6_geom,head7_geom,head8_geom;
	dGeomID neck0_geom, neck1_geom;
	dBodyID head;
	
	//init encapsulated object_the EYE
	dGeomID eye1, eye2,eye3,eye4,eye5;
	dGeomID eye1_geom,eye2_geom,eye3_geom,eye4_geom,eye5_geom;
	dBodyID eye;

	//init. encapsulated object - the left eye
	dGeomID List1_L_E, List2_L_E;
	dGeomID Leye1_geom, Leye2_geom;
	dBodyID leye;

	//init. encapsulated object - the right eye
	dGeomID List1_R_E,List2_R_E;
	dGeomID Reye1_geom,Reye2_geom;
	dBodyID reye;

	//init. eye lids
	dGeomID topEyeLid_geom;
	dGeomID bottomEyeLid_geom;
	dBodyID topEyeLid;
	dBodyID bottomEyeLid;

	/*----Lower Body joints----*/
	dJointID elevJoint;

	dJointID LLegjoints [20];
	dJointID RLegjoints [20];

	dJointID Torsojoints[8];
	dJointID LAjoints [25];
	dJointID RAjoints [25];

	dJointID Hjoints [6];

	dJointID grab;
	dJointID grab1;

	dJointID fixedHipJoint;

	/*----Lower Body joints speeds----*/
	dReal	 LLeg_speed	[20];
	dReal	 RLeg_speed	[20];

	dReal    Torso_speed[8];
	dReal	 la_speed	[25];
	dReal	 la_speed1	[25];
	dReal	 ra_speed	[25];
	dReal	 ra_speed1	[25];
	dReal	 h_speed	[25];

struct Object {
  dBodyID body;			   // the body
  dGeomID geom;
};

dBodyID vad;

public:

	void resetSpeeds();
	void setJointSpeeds();
	void setJointTorques();
	//void syncAngles();
	bool checkTouchSensor(int bodyToCheck);
	bool checkTouchSensor(dBodyID id);
	void draw();
    void setPosition(dReal agentX, dReal agentY, dReal agentZ );
	void init( dWorldID world, dSpaceID space, dReal X, dReal Y, dReal Z);
	void activateiCubParts();
	
	~ICubSim();

	ICubSim(dWorldID world, dSpaceID space, dReal X, dReal Y, dReal Z);
};
