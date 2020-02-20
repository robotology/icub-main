// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Vadim Tikhanoff
* email:   vadim.tikhanoff@iit.it
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
 * \file world.cpp
 * \brief This creates and places all the objects that are in the environment.The object parameters and joint parameters are included in this file.
 * \author Vadim Tikhanoff
 * \date 2007
 * \note Released under GNU GPL v2.0
 **/
#include "world.h"
#include <cstdio>
#include <yarp/os/LogStream.h>
#include <iostream>
#include "OdeInit.h"

using namespace std;
using namespace yarp::os;

static float xyz[3], hpr[3];
static dReal ballVel[3], ballDamp[3];

worldSimData::worldSimData() {
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

        /*glColor3d(0.8,0.0,0.0);
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
        */
        glColor3d(0.0,0.0,0.8);
        glPushMatrix(); LDEsetM(dBodyGetPosition(ballBody),dBodyGetRotation(ballBody));
        DrawSphere(0.04,false,textured,2);glPopMatrix();

    }

    for (int i=0; i<OBJNUM; i++) {
        glColor3d(color[i][0],color[i][1],color[i][2]);
        glPushMatrix();LDEsetM(dBodyGetPosition(obj[i].boxbody),dBodyGetRotation(obj[i].boxbody));
        DrawBox(obj[i].size[0],obj[i].size[1],obj[i].size[2],false,textured,2);glPopMatrix();
    }
    for (int i=0; i<S_OBJNUM; i++) {
        glColor3d(s_color[i][0],s_color[i][1],s_color[i][2]);
        glPushMatrix();LDEsetM(dGeomGetPosition(s_obj[i].geom[0]),dGeomGetRotation(s_obj[i].geom[0]));
        DrawBox(s_obj[i].size[0],s_obj[i].size[1],s_obj[i].size[2],false,textured,2);glPopMatrix();
    }

    for (int i=0; i<cylOBJNUM; i++) {
        glColor3d(color1[i][0],color1[i][1],color1[i][2]);
        glPushMatrix();LDEsetM(dBodyGetPosition(cyl_obj[i].cylbody),dBodyGetRotation(cyl_obj[i].cylbody));
        DrawCylinder(cyl_obj[i].radius,cyl_obj[i].length,false,textured,2);glPopMatrix();
    }

    for (int i=0; i<S_cylOBJNUM; i++) {
        glColor3d(s_color1[i][0],s_color1[i][1],s_color1[i][2]);
        glPushMatrix();LDEsetM(dGeomGetPosition(s_cyl_obj[i].cylgeom[0]),dGeomGetRotation(s_cyl_obj[i].cylgeom[0]));
        DrawCylinder(s_cyl_obj[i].radius,s_cyl_obj[i].length,false,textured,2);glPopMatrix();
    }

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
    for (int i=0; i<SPHNUM; i++) {
        glColor3d(color2[i][0],color2[i][1],color2[i][2]);
        glPushMatrix();LDEsetM(dBodyGetPosition(sph[i].sphbody),dBodyGetRotation(sph[i].sphbody));
        DrawSphere(sph[i].radius,false,textured,2);glPopMatrix();
    }
    for (int i=0; i<S_SPHNUM; i++) {
        glColor3d(s_color2[i][0],s_color2[i][1],s_color2[i][2]);
        glPushMatrix();LDEsetM(dGeomGetPosition(s_sph[i].sphgeom[0]),dGeomGetRotation(s_sph[i].sphgeom[0]));
        DrawSphere(s_sph[i].radius,false,textured,2);glPopMatrix();
    }
}

void worldSim::loadTexture(string texture, int numTexture){

   // yInfo() << " NUMBER TEXTURE " << numTexture;
    string tmptext = (char *) model_DIR.c_str();
    texture = tmptext + "/"+ texture;
    setupTexture( (char* ) texture.c_str(), numTexture );
}

void worldSim::setPosition(dReal agent1X, dReal agent1Y, dReal agent1Z ) {
        //dBodySetPosition(tempBody,-0.0,0.2, -100);
        //dGeomSetPosition(tempGeom[1],-0.4,0.2, -100);
    if (actWorld == "on"){
        dGeomSetPosition(tableGeom[0],0.3,0.25,0.3);
        dGeomSetPosition(tableGeom[1],-0.3,0.25,0.3);
        dGeomSetPosition(tableGeom[2],0.3,0.25,0.6);
        dGeomSetPosition(tableGeom[3],-0.3,0.25,0.6);
        dGeomSetPosition(tableGeom[4],0.0,0.5,0.45);

        //dBodySetPosition(Box,0.05,0.6, 0.55);

        dBodySetPosition(ballBody,-0.15,0.65, 0.35);
    }
}

void worldSim::activateWorld(RobotConfig& config) {
    ResourceFinder& finder = config.getFinder();

    Property options;
    //start left arm device driver
    string parts = finder.findFile("parts");
    options.fromConfigFile(parts.c_str());

    actWorld = options.findGroup("RENDER").check("objects",Value(1),"What did the user select?").asString();
}

void worldSim::init( dWorldID world, dSpaceID space, dReal X, dReal Y, dReal Z,
                     RobotConfig& config) {

    model_DIR = config.getFinder().findPath("model_path_default");

    activateWorld(config);
    /*------------iCub Space creation-------------*/
    /*
    * objects in the same space do not collide...see collision function in ICub_sim
    */
    boxObj = dSimpleSpaceCreate(space);
    dSpaceSetCleanup(boxObj,0);

    //tempBody = dBodyCreate (world);
    //tempGeom[0] = dCreateBox (space,0.1,0.1,0.1);
    //dGeomSetBody(tempGeom[0],tempBody);
    //tempGeom[1] = dCreateBox (space,0.1,0.1,0.1);

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

        /*Box = dBodyCreate (world);dMassSetZero(&m);dMassSetZero(&m2);
        box_part[0] = dCreateBox (boxObj,0.01,0.01,0.1); dMassSetBoxTotal(&m2,0.01, 0.1, 0.01,0.01);
        dGeomSetBody (box_part[0],Box);
        dGeomSetOffsetPosition(box_part[0],-0.05-m2.c[0], -m2.c[0], -m2.c[0]);
        dMassAdd (&m, &m2);

        box_part[1] = dCreateBox (boxObj,0.01,0.01,0.1); dMassSetBoxTotal(&m2,0.01, 0.01,0.01,0.1);
        dGeomSetBody (box_part[1],Box);
        dGeomSetOffsetPosition(box_part[1],0.05-m2.c[0], -m2.c[0], -m2.c[0]);
        dMassAdd (&m, &m2);

        box_part[2] = dCreateBox (boxObj,0.1,0.01,0.01); dMassSetBoxTotal(&m2,0.01, 0.1,0.01,0.01);
        dGeomSetBody (box_part[2],Box);
        dGeomSetOffsetPosition(box_part[2],-m2.c[0], -m2.c[0], 0.045-m2.c[0]);
        dMassAdd (&m, &m2);

        box_part[3] = dCreateBox (boxObj,0.1,0.01,0.01); dMassSetBoxTotal(&m2,0.01, 0.1,0.01,0.01);
        dGeomSetBody (box_part[3],Box);
        dGeomSetOffsetPosition(box_part[3],-m2.c[0], -m2.c[0], -0.045-m2.c[0]);
        dMassAdd (&m, &m2);

        box_part[4] = dCreateBox (boxObj, 0.01,0.1,0.01); dMassSetBoxTotal(&m2,0.01, 0.01,0.1,0.01);
        dGeomSetBody (box_part[4],Box);
        dGeomSetOffsetPosition(box_part[4],-0.05-m2.c[0], 0.045-m2.c[0], -0.045-m2.c[0]);
        dMassAdd (&m, &m2);

        box_part[5] = dCreateBox (boxObj, 0.01,0.1,0.01); dMassSetBoxTotal(&m2,0.01, 0.01,0.1,0.01);
        dGeomSetBody (box_part[5],Box);
        dGeomSetOffsetPosition(box_part[5],-0.05-m2.c[0], 0.045-m2.c[0], 0.045-m2.c[0]);
        dMassAdd (&m, &m2);

        box_part[6] = dCreateBox (boxObj, 0.01,0.1,0.01); dMassSetBoxTotal(&m2,0.01, 0.01,0.1, 0.01);
        dGeomSetBody (box_part[6],Box);
        dGeomSetOffsetPosition(box_part[6],0.05-m2.c[0], 0.045-m2.c[0], -0.045-m2.c[0]);
        dMassAdd (&m, &m2);

        box_part[7] = dCreateBox (boxObj, 0.01,0.1,0.01); dMassSetBoxTotal(&m2,0.01, 0.01,0.1, 0.01);
        dGeomSetBody (box_part[7],Box);
        dGeomSetOffsetPosition(box_part[7],0.05-m2.c[0], 0.045-m2.c[0], 0.045-m2.c[0]);
        dMassAdd (&m, &m2);

        box_part[8] = dCreateBox (boxObj,0.01,0.01,0.1); dMassSetBoxTotal(&m2,0.015, 0.1, 0.01,0.01);
        dGeomSetBody (box_part[8],Box);
        dGeomSetOffsetPosition(box_part[8],-0.05-m2.c[0], 0.1-m2.c[0], -m2.c[0]);
        dMassAdd (&m, &m2);

        box_part[9] = dCreateBox (boxObj,0.01,0.01,0.1); dMassSetBoxTotal(&m2,0.01, 0.01,0.01,0.1);
        dGeomSetBody (box_part[9],Box);
        dGeomSetOffsetPosition(box_part[9],0.05-m2.c[0], 0.1-m2.c[0], -m2.c[0]);
        dMassAdd (&m, &m2);

        box_part[10] = dCreateBox (boxObj,0.1,0.01,0.01); dMassSetBoxTotal(&m2,0.01, 0.1,0.01,0.01);
        dGeomSetBody (box_part[10],Box);
        dGeomSetOffsetPosition(box_part[10],-m2.c[0], 0.1-m2.c[0], 0.045-m2.c[0]);
        dMassAdd (&m, &m2);

        box_part[11] = dCreateBox (boxObj,0.1,0.01,0.01); dMassSetBoxTotal(&m2,0.01, 0.1,0.01,0.01);
        dGeomSetBody (box_part[11],Box);
        dGeomSetOffsetPosition(box_part[11],-m2.c[0], 0.1-m2.c[0], -0.045-m2.c[0]);
        dMassAdd (&m, &m2);

        dBodySetMass(Box,&m);
        */
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
        //for (int i=0; i<12; i++){
        //    dGeomDestroy(box_part[i]);
        //}
        dGeomDestroy(ballGeom);
    }
    dSpaceDestroy (boxObj);
}

worldSim::worldSim(dWorldID world, dSpaceID space, dReal X, dReal Y, dReal Z,
                   RobotConfig& config)
    : box_dynamic(obj,MAXNUM,OBJNUM,color),
      box_static(s_obj,MAXNUM,S_OBJNUM,s_color),
      cylinder_dynamic(cyl_obj,MAXNUM,cylOBJNUM,color1),
      cylinder_static(s_cyl_obj,MAXNUM,S_cylOBJNUM,s_color1),
      model_dynamic(ThreeD_obj,MAXNUM,MODEL_NUM,NULL),
      model_static(s_ThreeD_obj,MAXNUM,s_MODEL_NUM,NULL),
      sphere_dynamic(sph,MAXNUM,SPHNUM,color2),
      sphere_static(s_sph,MAXNUM,S_SPHNUM,s_color2)
{
    init(world, space, X, Y, Z, config);
}


// BOX
bool worldSim::MyObject::create(const WorldOp& op, WorldResult& result, int idx) {
    if (!op.size.isValid()) {
        result.setFail("size not set");
        return false;
    }
    size[0] = op.size.get(0);
    size[1] = op.size.get(1);
    size[2] = op.size.get(2);
    OdeInit& odeinit = OdeInit::get();
    if (op.dynamic.get()) {
        dMass m;
        dMassSetZero(&m);
        boxbody = dBodyCreate (odeinit.world);
        dMassSetBoxTotal (&m,DENSITY,size[0],size[1],size[2]);
        dBodySetMass (boxbody,&m);
    }
    if (op.collide.get())
        geom[0] = dCreateBox (odeinit.space,size[0],size[1],size[2]);
    else
        geom[0] = dCreateBox (odeinit._iCub->iCub,size[0],size[1],size[2]);

    if (op.dynamic.get()) {
        dGeomSetBody (geom[0],boxbody);
    }
    return true;
}

// CYLINDER
bool worldSim::MyObject1::create(const WorldOp& op, WorldResult& result, int idx) {
    if (!op.radius.isValid()) {
        result.setFail("radius not set");
        return false;
    }
    if (!op.length.isValid()) {
        result.setFail("length not set");
        return false;
    }
    radius = op.radius.get();
    length = op.length.get();
    OdeInit& odeinit = OdeInit::get();
    if (op.dynamic.get()) {
        dMass m;
        dMassSetZero(&m);
        cylbody = dBodyCreate (odeinit.world);
        dMassSetCylinderTotal (&m,DENSITY,3,radius,length);
        dBodySetMass (cylbody,&m);
    }
    if (op.collide.get())
        cylgeom[0] = dCreateCylinder(odeinit.space,radius,length);
    else
        cylgeom[0] = dCreateCylinder(odeinit._iCub->iCub,radius,length);

    if (op.dynamic.get()) {
        dGeomSetBody (cylgeom[0],cylbody);
    }
    return true;
}

// 3D model
bool worldSim::MyObject2::create(const WorldOp& op, WorldResult& result, int idx) {
    yDebug("Making a model\n");
    if (!op.modelName.isValid()) {
        result.setFail("model name not set");
        return false;
    }
    if (!op.modelTexture.isValid()) {
        result.setFail("model texture not set");
        return false;
    }

    string model = op.modelName.get();
    OdeInit& odeinit = OdeInit::get();
    odeinit._wrld->texture = op.modelTexture.get().c_str();

    bool dynamic = op.dynamic.get();
    dTriMeshDataID *tridata = dynamic?odeinit._wrld->TriData:odeinit._wrld->s_TriData;
    dTriMeshX *trimesh = dynamic?odeinit._wrld->trimesh:odeinit._wrld->s_trimesh;
    int *modelTexture = dynamic?odeinit._wrld->modelTexture:odeinit._wrld->s_modelTexture;

    tridata[idx] = dGeomTriMeshDataCreate();
    string tmp = (char *) odeinit._wrld->model_DIR.c_str();
    model = string(tmp.c_str()) + "/" + model.c_str();
    trimesh[idx] = dLoadMeshFromX(model.c_str());
    if (!trimesh[idx]){
        result.setFail("Check spelling and location of model file");
        return false;
    }

    dGeomTriMeshDataBuildSingle(tridata[idx],
                                trimesh[idx]->Vertices,
                                3 * sizeof(float),
                                trimesh[idx]->VertexCount,
                                trimesh[idx]->Indices,
                                trimesh[idx]->IndexCount,
                                3 * sizeof(int));
    if (dynamic) {
        body = dBodyCreate (odeinit.world);
    }
    if (op.collide.get())
        geom = dCreateTriMesh(odeinit.space, tridata[idx], 0, 0, 0);
    else
        geom = dCreateTriMesh(odeinit._iCub->iCub, tridata[idx], 0, 0, 0);

    //geom = dCreateTriMesh(odeinit.space, tridata[idx], 0, 0, 0);
    dGeomSetData(geom,tridata[idx]);
    if (dynamic) {
        dMass m;
        dMassSetZero(&m);
        dMassSetTrimesh(&m, 0.1, geom);
        dGeomSetBody(geom, body);
        dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
        dBodySetMass(body, &m);
    }

    // this all seems incredibly dodgy
    modelTexture[idx] = idx + 1 + (dynamic?19:49);
    odeinit.mtxTexture.lock();
    odeinit._wrld->static_model = !dynamic;
    odeinit._wrld->WAITLOADING = true;
    odeinit.mtxTexture.unlock();

    return true;
}


// SPHERE
bool worldSim::MyObject3::create(const WorldOp& op, WorldResult& result, int idx) {
    if (!op.radius.isValid()) {
        result.setFail("radius not set");
        return false;
    }
    radius = op.radius.get();
    OdeInit& odeinit = OdeInit::get();
    if (op.dynamic.get()) {
        dMass m;
        dMassSetZero(&m);
        sphbody = dBodyCreate(odeinit.world);
        dMassSetSphereTotal(&m,DENSITY, radius);
        dBodySetMass(sphbody, &m);
    }
    if (op.collide.get())
        sphgeom[0] = dCreateSphere (odeinit.space, radius);
    else
        sphgeom[0] = dCreateSphere (odeinit._iCub->iCub, radius);
    if (op.dynamic.get()) {
        dGeomSetBody(sphgeom[0], sphbody);
    }
    return true;
}

