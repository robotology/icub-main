// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * \file world.cpp
 * \brief This creates and places all the objects that are in the environment.The object parameters and joint parameters are included in this file.
 * \author Vadim Tikhanoff
 * \date 2007
 * \note Release under GNU GPL v2.0
 **/
#include "world.h"
#include "SimConfig.h"
#include <stdio.h>
#include <yarp/os/ConstString.h>
#include <iostream>

static float xyz[3], hpr[3];
static dReal ballVel[3], ballDamp[3];

worldSimData::worldSimData() {
}

void worldSim::resetSpeeds() {
	int x;
	// joint speeds
	for(x = 0; x < numObjJoints; x++) {
		//speed[x] = 0.0;
	}
}

void worldSim::syncAngles() {
}

void worldSim::ballDamping()
{
	ballVel[0] = dBodyGetLinearVel (ballBody)[0];
	ballVel[1] = dBodyGetLinearVel (ballBody)[1];
	ballVel[2] = dBodyGetLinearVel (ballBody)[2];

	ballDamp[0] = ballVel[0] * -0.8;
	ballDamp[1] = ballVel[1] * -0.8;
	ballDamp[2] = ballVel[2] * -0.8;

	j  = dBodyGetJoint(ballBody, 0);

	if(j){
		dBodyAddForce(ballBody,ballDamp[0],ballDamp[1],ballDamp[2]);
	}
}
void worldSim::draw(){

	if (actWorld == "on"){
		////table geom
		glColor3d(0.6,0.6,0.0);
		glPushMatrix();LDEsetM(dGeomGetPosition(tableGeom[0]),dGeomGetRotation(tableGeom[0]));
		DrawBox(0.03,0.5,0.03,false,textured,2);glPopMatrix();

		glPushMatrix();LDEsetM(dGeomGetPosition(tableGeom[1]),dGeomGetRotation(tableGeom[1]));
		DrawBox(0.03,0.5,0.03,false,textured,2);glPopMatrix();

		glPushMatrix();LDEsetM(dGeomGetPosition(tableGeom[2]),dGeomGetRotation(tableGeom[2]));
		DrawBox(0.03,0.5,0.03,false,textured,2);glPopMatrix();

		glPushMatrix();LDEsetM(dGeomGetPosition(tableGeom[3]),dGeomGetRotation(tableGeom[3]));
		DrawBox(0.03,0.5,0.03,false,textured,2);glPopMatrix();

		glPushMatrix();LDEsetM(dGeomGetPosition(tableGeom[4]),dGeomGetRotation(tableGeom[4]));
		DrawBox(0.7,0.03,0.4,false,textured,2);glPopMatrix();

		glColor3d(0.8,0.0,0.0);
		glPushMatrix();LDEsetM(dGeomGetPosition(box_part[0]),dGeomGetRotation(box_part[0]));
		DrawBox(0.01,0.01,0.1,false,textured,2);glPopMatrix();

		glPushMatrix();LDEsetM(dGeomGetPosition(box_part[1]),dGeomGetRotation(box_part[1]));
		DrawBox(0.01,0.01,0.1,false,textured,2);glPopMatrix();

		glPushMatrix();LDEsetM(dGeomGetPosition(box_part[2]),dGeomGetRotation(box_part[2]));
		DrawBox(0.1,0.01,0.01,false,textured,2);glPopMatrix();

		glPushMatrix();LDEsetM(dGeomGetPosition(box_part[3]),dGeomGetRotation(box_part[3]));
		DrawBox(0.1,0.01,0.01,false,textured,2);glPopMatrix();

		glPushMatrix();LDEsetM(dGeomGetPosition(box_part[4]),dGeomGetRotation(box_part[4]));
		DrawBox(0.01,0.1,0.01,false,textured,2);glPopMatrix();

		glPushMatrix();LDEsetM(dGeomGetPosition(box_part[5]),dGeomGetRotation(box_part[5]));
		DrawBox(0.01,0.1,0.01,false,textured,2);glPopMatrix();

		glPushMatrix();LDEsetM(dGeomGetPosition(box_part[6]),dGeomGetRotation(box_part[6]));
		DrawBox(0.01,0.1,0.01,false,textured,2);glPopMatrix();

		glPushMatrix();LDEsetM(dGeomGetPosition(box_part[7]),dGeomGetRotation(box_part[7]));
		DrawBox(0.01,0.1,0.01,false,textured,2);glPopMatrix();

		glPushMatrix();LDEsetM(dGeomGetPosition(box_part[8]),dGeomGetRotation(box_part[8]));
		DrawBox(0.01,0.01,0.1,false,textured,2);glPopMatrix();

		glPushMatrix();LDEsetM(dGeomGetPosition(box_part[9]),dGeomGetRotation(box_part[9]));
		DrawBox(0.01,0.01,0.1,false,textured,2);glPopMatrix();

		glPushMatrix();LDEsetM(dGeomGetPosition(box_part[10]),dGeomGetRotation(box_part[10]));
		DrawBox(0.1,0.01,0.01,false,textured,2);glPopMatrix();

		glPushMatrix();LDEsetM(dGeomGetPosition(box_part[11]),dGeomGetRotation(box_part[11]));
		DrawBox(0.1,0.01,0.01,false,textured,2);glPopMatrix();

		glColor3d(0.0,0.0,0.8);
		glPushMatrix(); LDEsetM(dBodyGetPosition(ballBody),dBodyGetRotation(ballBody));
		DrawSphere(0.04,false,textured,2);glPopMatrix();	

	}
	
	if (OBJNUM != waitOBJ){
		Time::delay(0.1); waitOBJ++;}
	else{
		for (int i=0; i<OBJNUM; i++) {
			glColor3d(color[i][0],color[i][1],color[i][2]);
			glPushMatrix();LDEsetM(dBodyGetPosition(obj[i].boxbody),dBodyGetRotation(obj[i].boxbody));
			DrawBox(obj[i].size[0],obj[i].size[1],obj[i].size[2],false,textured,2);glPopMatrix();
		}
	}
	for (int i=0; i<S_OBJNUM; i++) {
		glColor3d(s_color[i][0],s_color[i][1],s_color[i][2]);
		glPushMatrix();LDEsetM(dGeomGetPosition(s_obj[i].geom[0]),dGeomGetRotation(s_obj[i].geom[0]));
		DrawBox(s_obj[i].size[0],s_obj[i].size[1],s_obj[i].size[2],false,textured,2);glPopMatrix();
	}
	
	if (cylOBJNUM != waitOBJ1){
		Time::delay(0.1); waitOBJ1++;}
	else{
		for (int i=0; i<cylOBJNUM; i++) {
			glColor3d(color1[i][0],color1[i][1],color1[i][2]);
			glPushMatrix();LDEsetM(dBodyGetPosition(cyl_obj[i].cylbody),dBodyGetRotation(cyl_obj[i].cylbody));
			DrawCylinder(cyl_obj[i].radius,cyl_obj[i].lenght,false,textured,2);glPopMatrix();
		}
	}
	for (int i=0; i<S_cylOBJNUM; i++) {
		glColor3d(s_color1[i][0],s_color1[i][1],s_color1[i][2]);
		glPushMatrix();LDEsetM(dGeomGetPosition(s_cyl_obj[i].cylgeom[0]),dGeomGetRotation(s_cyl_obj[i].cylgeom[0]));
		DrawCylinder(s_cyl_obj[i].radius,s_cyl_obj[i].lenght,false,textured,2);glPopMatrix();
	}
	
	if (MODEL_NUM != waitMOD){
		Time::delay(0.1); waitMOD++;}
	else{

		for (int i=0; i<MODEL_NUM; i++){
			glColor3d(1.0,1.0,1.0);
			glPushMatrix();LDEsetM(dGeomGetPosition(ThreeD_obj[i].geom),dGeomGetRotation(ThreeD_obj[i].geom));     //DRAW THE MODEL
	        DrawX( trimesh[i], modelTexture[i]);
			glPopMatrix();
		}
		for (int i=0; i<s_MODEL_NUM; i++){
			glColor3d(1.0,1.0,1.0);
			glPushMatrix();LDEsetM(dGeomGetPosition(s_ThreeD_obj[i].geom),dGeomGetRotation(s_ThreeD_obj[i].geom));     //DRAW THE MODEL
	        DrawX( s_trimesh[i], s_modelTexture[i]);
			glPopMatrix();
		}
	}
    if (SPHNUM != waitSPH){
		Time::delay(0.1); waitSPH++;}
	else{
		for (int i=0; i<SPHNUM; i++) {
			glColor3d(color2[i][0],color2[i][1],color2[i][2]);
			glPushMatrix();LDEsetM(dBodyGetPosition(sph[i].sphbody),dBodyGetRotation(sph[i].sphbody));
			DrawSphere(sph[i].radius,false,textured,2);glPopMatrix();
		}
	}
	    for (int i=0; i<S_SPHNUM; i++) {
			glColor3d(s_color2[i][0],s_color2[i][1],s_color2[i][2]);
			glPushMatrix();LDEsetM(dGeomGetPosition(s_sph[i].sphgeom[0]),dGeomGetRotation(s_sph[i].sphgeom[0]));
			DrawSphere(s_sph[i].radius,false,textured,2);glPopMatrix();
		}
}

void worldSim::loadTexture(ConstString texture, int numTexture){

	cout << " NUMBER TEXTURE " << numTexture << endl;
	ConstString tmptext = (char *) model_DIR.c_str();
	texture = tmptext + "/"+ texture;
    setupTexture( (char* ) texture.c_str(), numTexture );
}

void worldSim::setPosition(dReal agent1X, dReal agent1Y, dReal agent1Z ) {

	if (actWorld == "on"){
		dGeomSetPosition(tableGeom[0],0.3,0.25,0.3);
		dGeomSetPosition(tableGeom[1],-0.3,0.25,0.3);
		dGeomSetPosition(tableGeom[2],0.3,0.25,0.6);
		dGeomSetPosition(tableGeom[3],-0.3,0.25,0.6);
		dGeomSetPosition(tableGeom[4],0.0,0.5,0.45);

		dBodySetPosition(Box,0.05,0.55, 0.35);

		dBodySetPosition(ballBody,-0.15,0.65, 0.35);
	}
}

void worldSim::activateWorld() {

	Property options;
	SimConfig finder;
  	//start left arm device driver
	ConstString general = finder.find("general");
	options.fromConfigFile(general.c_str());

	//readConfig(options,"conf/iCub_parts_activation.ini");
	actWorld = options.findGroup("RENDER").check("objects",Value(1),"What did the user select?").asString();
}

void worldSim::init( dWorldID world, dSpaceID space, dReal X, dReal Y, dReal Z ) {
	activateWorld();
	/*------------iCub Space creation-------------*/
	/*
	* objects in the same space do not collide...see collision function in ICub_sim
	*/	
	boxObj = dSimpleSpaceCreate(space);
	dSpaceSetCleanup(boxObj,0);
	
	if (actWorld == "on"){
		dMass m, m2;

		float ms = 1;
		/*---------------Body creation-----------*/

		tableGeom[0] = dCreateBox (space,0.03,0.5,0.03);
		tableGeom[1] = dCreateBox (space,0.03,0.5,0.03);
		tableGeom[2] = dCreateBox (space,0.03,0.5,0.03);
		tableGeom[3] = dCreateBox (space,0.03,0.5,0.03);
		tableGeom[4] = dCreateBox (space,0.7,0.03,0.4);

		dMassSetZero(&m);
		dMassSetZero(&m2);

		Box = dBodyCreate (world);dMassSetZero(&m);dMassSetZero(&m2);
		box_part[0] = dCreateBox (boxObj,0.01,0.01,0.1); dMassSetBoxTotal(&m2,0.05, 0.1, 0.01,0.01);
		dGeomSetBody (box_part[0],Box);
		dGeomSetOffsetPosition(box_part[0],-0.05-m2.c[0], -m2.c[0], -m2.c[0]);
		dMassAdd (&m, &m2);

		box_part[1] = dCreateBox (boxObj,0.01,0.01,0.1); dMassSetBoxTotal(&m2,0.05, 0.01,0.01,0.1);
		dGeomSetBody (box_part[1],Box);
		dGeomSetOffsetPosition(box_part[1],0.05-m2.c[0], -m2.c[0], -m2.c[0]);
		dMassAdd (&m, &m2);

		box_part[2] = dCreateBox (boxObj,0.1,0.01,0.01); dMassSetBoxTotal(&m2,0.05, 0.1,0.01,0.01);
		dGeomSetBody (box_part[2],Box);
		dGeomSetOffsetPosition(box_part[2],-m2.c[0], -m2.c[0], 0.045-m2.c[0]);
		dMassAdd (&m, &m2);

		box_part[3] = dCreateBox (boxObj,0.1,0.01,0.01); dMassSetBoxTotal(&m2,0.05, 0.1,0.01,0.01);
		dGeomSetBody (box_part[3],Box);
		dGeomSetOffsetPosition(box_part[3],-m2.c[0], -m2.c[0], -0.045-m2.c[0]);
		dMassAdd (&m, &m2);

		box_part[4] = dCreateBox (boxObj, 0.01,0.1,0.01); dMassSetBoxTotal(&m2,0.05, 0.01,0.1,0.01);
		dGeomSetBody (box_part[4],Box);
		dGeomSetOffsetPosition(box_part[4],-0.05-m2.c[0], 0.045-m2.c[0], -0.045-m2.c[0]);
		dMassAdd (&m, &m2);

		box_part[5] = dCreateBox (boxObj, 0.01,0.1,0.01); dMassSetBoxTotal(&m2,0.05, 0.01,0.1,0.01);
		dGeomSetBody (box_part[5],Box);
		dGeomSetOffsetPosition(box_part[5],-0.05-m2.c[0], 0.045-m2.c[0], 0.045-m2.c[0]);
		dMassAdd (&m, &m2);

		box_part[6] = dCreateBox (boxObj, 0.01,0.1,0.01); dMassSetBoxTotal(&m2,0.05, 0.01,0.1, 0.01);
		dGeomSetBody (box_part[6],Box);
		dGeomSetOffsetPosition(box_part[6],0.05-m2.c[0], 0.045-m2.c[0], -0.045-m2.c[0]);
		dMassAdd (&m, &m2);

		box_part[7] = dCreateBox (boxObj, 0.01,0.1,0.01); dMassSetBoxTotal(&m2,0.05, 0.01,0.1, 0.01);
		dGeomSetBody (box_part[7],Box);
		dGeomSetOffsetPosition(box_part[7],0.05-m2.c[0], 0.045-m2.c[0], 0.045-m2.c[0]);
		dMassAdd (&m, &m2);

		box_part[8] = dCreateBox (boxObj,0.01,0.01,0.1); dMassSetBoxTotal(&m2,0.05, 0.1, 0.01,0.01);
		dGeomSetBody (box_part[8],Box);
		dGeomSetOffsetPosition(box_part[8],-0.05-m2.c[0], 0.1-m2.c[0], -m2.c[0]);
		dMassAdd (&m, &m2);

		box_part[9] = dCreateBox (boxObj,0.01,0.01,0.1); dMassSetBoxTotal(&m2,0.05, 0.01,0.01,0.1);
		dGeomSetBody (box_part[9],Box);
		dGeomSetOffsetPosition(box_part[9],0.05-m2.c[0], 0.1-m2.c[0], -m2.c[0]);
		dMassAdd (&m, &m2);

		box_part[10] = dCreateBox (boxObj,0.1,0.01,0.01); dMassSetBoxTotal(&m2,0.05, 0.1,0.01,0.01);
		dGeomSetBody (box_part[10],Box);
		dGeomSetOffsetPosition(box_part[10],-m2.c[0], 0.1-m2.c[0], 0.045-m2.c[0]);
		dMassAdd (&m, &m2);

		box_part[11] = dCreateBox (boxObj,0.1,0.01,0.01); dMassSetBoxTotal(&m2,0.05, 0.1,0.01,0.01);
		dGeomSetBody (box_part[11],Box);
		dGeomSetOffsetPosition(box_part[11],-m2.c[0], 0.1-m2.c[0], -0.045-m2.c[0]);
		dMassAdd (&m, &m2);

		dBodySetMass(Box,&m);
		/*-----------------Add independent objects to the space-------------*/

		/*---------------Independed encapsulated object position -------------*/

		dMassSetZero(&m);
		ballBody = dBodyCreate(world);
		dMassSetSphereTotal(&m,0.5, 0.04);
		ballGeom = dCreateSphere (space, 0.04);
		dGeomSetBody(ballGeom,ballBody);
		dBodySetMass(ballBody, &m);
	}
	setPosition( X, Y, Z );
}

worldSim::~worldSim() {

	if (actWorld == "on"){
		for (int i=0; i<5; i++){
			dGeomDestroy(tableGeom[i]);
		}
		for (int i=0; i<12; i++){
			dGeomDestroy(box_part[i]);
		}
		dGeomDestroy(ballGeom);
	}
	dSpaceDestroy (boxObj);
}

worldSim::worldSim(dWorldID world, dSpaceID space, dReal X, dReal Y, dReal Z)
: worldSimData() {
	resetSpeeds();
	init(world, space, X, Y, Z);
}
