// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Vadim Tikhanoff, Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#include "WorldManager.h"

#include "OdeInit.h"
#include "iCub_Sim.h"

static int a = 0, b = 0, c = 0;
static int num=0;// number of objects in simulation
static int nextobj=0;// next object to recycle if num==NUM
static size_t i;
static int setBody = 0;
static dReal sides[3];

#define DENSITY (1.0)		// density of all objects

using namespace yarp::os;

bool WorldManager::respond(const Bottle &command, Bottle &reply) {
    bool done = false;
    bool ok = true;

    ODE_access.wait();
    ConstString subcmd = command.get(1).asString();
    if (subcmd=="get"||subcmd=="set"||subcmd=="mk"||subcmd=="grab"||subcmd=="rot" || subcmd=="del") {
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
            bid = odeinit._wrld->tempBody;
            setBody = 2;
            break;
        case VOCAB4('s','b','o','x'):
            bid2 = odeinit._wrld->tempGeom[0];//temporary then will be replaced...
            setBody = 5;
            break;
        case VOCAB3('c','y','l'):
            bid = odeinit._wrld->tempBody;
            setBody = 6;
            break;
        case VOCAB4('s','c','y','l'):
            bid2 = odeinit._wrld->tempGeom[0];//temporary then will be replaced...
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
        case VOCAB4('m','o','d','e'):
            bid = odeinit._wrld->tempBody;
            setBody = 8;
            break;
        case VOCAB4('s','m','o','d'):
            bid2 = odeinit._wrld->tempGeom[0];
            setBody = 9;
            break;
        case VOCAB4('m','d','i','r'):
            bid2 = odeinit._wrld->tempGeom[0];
            setBody = 10;
            break;
        case VOCAB3('s','p','h'):
            bid = odeinit._wrld->tempBody;
            setBody = 11;
            break;
        case VOCAB4('s','s','p','h'):
            bid2 = odeinit._wrld->tempGeom[0];
            setBody = 12;
            break;
        case VOCAB3('a','l','l'):
            bid = odeinit._wrld->tempBody;
            setBody = 11;
            break;
        }
        reply.clear();
        if (bid!=NULL || bid2!=NULL) {
            if (subcmd=="del"){

                int b = odeinit._wrld->OBJNUM;//box
                int sb = odeinit._wrld->S_OBJNUM;
                int c = odeinit._wrld->cylOBJNUM;//box
                int sc = odeinit._wrld->S_cylOBJNUM;
                int s = odeinit._wrld->SPHNUM;//box
                int ss = odeinit._wrld->S_SPHNUM;
                     
                for (int x=0; x<b; x++){
                    odeinit.mutex.wait();
                    dGeomDestroy(odeinit._wrld->obj[x].geom[0]);
                    odeinit._wrld->OBJNUM = 0;
                    odeinit.mutex.post();
                }
                for (int x=0; x<sb; x++){
                    odeinit.mutex.wait();
                    dGeomDestroy(odeinit._wrld->s_obj[x].geom[0]);
                    odeinit._wrld->S_OBJNUM = 0;
                    odeinit.mutex.post();
                }
                for (int x=0; x<c; x++){
                    odeinit.mutex.wait();
                    dGeomDestroy(odeinit._wrld->cyl_obj[x].cylgeom[0]);
                    odeinit._wrld->cylOBJNUM = 0;
                    odeinit.mutex.post();
                }
                for (int x=0; x<sc; x++){
                    odeinit.mutex.wait();
                    dGeomDestroy(odeinit._wrld->s_cyl_obj[x].cylgeom[0]);
                    odeinit._wrld->S_cylOBJNUM = 0;
                    odeinit.mutex.post();
                }
                for (int x=0; x<s; x++){
                    odeinit.mutex.wait();
                    dGeomDestroy(odeinit._wrld->sph[x].sphgeom[0]);
                    odeinit._wrld->SPHNUM = 0;
                    odeinit.mutex.post();
                }
                for (int x=0; x<ss; x++){
                    odeinit.mutex.wait();
                    dGeomDestroy(odeinit._wrld->s_sph[x].sphgeom[0]);
                    odeinit._wrld->S_SPHNUM = 0;
                    odeinit.mutex.post();
                }
             
            }
            if (subcmd=="get") {
                if (setBody == 0 || setBody == 1 || setBody == 3 || setBody == 4){
                    const dReal *coords = dBodyGetPosition(bid);
                    reply.addDouble(coords[0]);
                    reply.addDouble(coords[1]);
                    reply.addDouble(coords[2]);
                }
                if (setBody == 2){
                    unsigned int N = command.get(3).asInt();
                    if ((N >(i+1)) || (N < 1) || odeinit._wrld->OBJNUM == 0){reply.addString("object not known");}
                    else{
                        const dReal *coords = dBodyGetPosition(odeinit._wrld->obj[N-1].boxbody);
                        reply.addDouble(coords[0]);
                        reply.addDouble(coords[1]);
                        reply.addDouble(coords[2]);
                    }
                }
                if (setBody == 5){
                    unsigned int N = command.get(3).asInt();
                    if ((N >(i+1)) || (N < 1) || odeinit._wrld->S_OBJNUM == 0 ){reply.addString("object not known");}
                    else{	const dReal *coords = dGeomGetPosition(odeinit._wrld->s_obj[N-1].geom[0]);
                        reply.addDouble(coords[0]);
                        reply.addDouble(coords[1]);
                        reply.addDouble(coords[2]);
                    }
                }
                if (setBody==6){
                    unsigned int N = command.get(3).asInt();
                    if ( (N >(i+1)) || (N < 1) || odeinit._wrld->cylOBJNUM == 0 ){reply.addString("object not known");}
                    else{
                        const dReal *coords = dBodyGetPosition(odeinit._wrld->cyl_obj[N-1].cylbody);
                        reply.addDouble(coords[0]);
                        reply.addDouble(coords[1]);
                        reply.addDouble(coords[2]);
                    }
                }
                if (setBody==7){
                    unsigned int N = command.get(3).asInt();
                    if ((N >(i+1)) || (N < 1) || odeinit._wrld->S_cylOBJNUM == 0 ){reply.addString("object not known");}
                    else{
                        const dReal *coords = dGeomGetPosition(odeinit._wrld->s_cyl_obj[N-1].cylgeom[0]);
                        reply.addDouble(coords[0]);
                        reply.addDouble(coords[1]);
                        reply.addDouble(coords[2]);
                    }
                }
                if (setBody==8){
                    unsigned int N = command.get(3).asInt();
                    if ((N >(i+1)) || (N < 1) ||  odeinit._wrld->MODEL_NUM == 0 ){reply.addString("object not known");}
                    else{
                        const dReal *coords = dGeomGetPosition(odeinit._wrld->ThreeD_obj[N-1].geom);
                        reply.addDouble(coords[0]);
                        reply.addDouble(coords[1]);
                        reply.addDouble(coords[2]);
                    }
                }
                if (setBody==9){
                    unsigned int N = command.get(3).asInt();
                    if ((N >(i+1)) || (N < 1) ||  odeinit._wrld->s_MODEL_NUM == 0 ){reply.addString("object not known");}
                    else{
                        const dReal *coords = dGeomGetPosition(odeinit._wrld->s_ThreeD_obj[N-1].geom);
                        reply.addDouble(coords[0]);
                        reply.addDouble(coords[1]);
                        reply.addDouble(coords[2]);
                    }
                }
                if (setBody==10){
                    // SET MODEL DIRECTORY................................................
                    reply.addString ( (char*) odeinit._wrld->model_DIR.c_str() );
							
                }
                if (setBody==11){
                    unsigned int N = command.get(3).asInt();
                    if ((N >(i+1)) || (N < 1) || odeinit._wrld->SPHNUM == 0 ){reply.addString("object not known");}
                    else{
                        const dReal *coords = dBodyGetPosition(odeinit._wrld->sph[N-1].sphbody);
                        reply.addDouble(coords[0]);
                        reply.addDouble(coords[1]);
                        reply.addDouble(coords[2]);
                    }
                }
                if (setBody==12){
                    unsigned int N = command.get(3).asInt();
                    if ((N >(i+1)) || (N < 1) || odeinit._wrld->S_SPHNUM == 0 ){reply.addString("object not known");}
                    else{
                        const dReal *coords = dGeomGetPosition(odeinit._wrld->s_sph[N-1].sphgeom[0]);
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
                    odeinit.mutex.wait();
                    printf("Preparing to set %ld %g %g %g\n", 
                           (long int)bid,
                           x, y, z);
                    dBodySetPosition(bid,x,y,z);
                    dBodySetLinearVel (bid, 0.0, 0.0, 0.0);
                    dBodySetAngularVel (bid, 0.0, 0.0, 0.0);
                    odeinit.mutex.post();
                }
                if (setBody==2){
                    unsigned int N = command.get(3).asInt();
                    if ( (N >(i+1)) || (N < 1) || odeinit._wrld->OBJNUM == 0 ){reply.addString("object not known");}
                    else{
                        double x = command.get(4).asDouble();
                        double y = command.get(5).asDouble();
                        double z = command.get(6).asDouble();
                        odeinit.mutex.wait();
                        dBodySetPosition(odeinit._wrld->obj[N-1].boxbody,x,y,z);
                        dBodySetLinearVel (odeinit._wrld->obj[N-1].boxbody, 0.0, 0.0, 0.0);
                        dBodySetAngularVel (odeinit._wrld->obj[N-1].boxbody, 0.0, 0.0, 0.0);
                        odeinit.mutex.post();
                    }
                }
                if (setBody==5){
                    unsigned int N = command.get(3).asInt();
                    if ((N >(i+1)) || (N < 1) || odeinit._wrld->S_OBJNUM == 0 ){reply.addString("object not known");}
                    else{
                        double x = command.get(4).asDouble();
                        double y = command.get(5).asDouble();
                        double z = command.get(6).asDouble();
                        odeinit.mutex.wait();
                        dGeomSetPosition(odeinit._wrld->s_obj[N-1].geom[0],x,y,z);
                        odeinit.mutex.post();
                    }
                }
                if (setBody==6){
                    unsigned int N = command.get(3).asInt();
                    if ( (N >(i+1)) || (N < 1) || odeinit._wrld->cylOBJNUM == 0 ){reply.addString("object not known");}
                    else{
                        double x = command.get(4).asDouble();
                        double y = command.get(5).asDouble();
                        double z = command.get(6).asDouble();
                        odeinit.mutex.wait();
                        dBodySetPosition(odeinit._wrld->cyl_obj[N-1].cylbody,x,y,z);
                        dBodySetLinearVel (odeinit._wrld->cyl_obj[N-1].cylbody, 0.0, 0.0, 0.0);
                        dBodySetAngularVel (odeinit._wrld->cyl_obj[N-1].cylbody, 0.0, 0.0, 0.0);
                        odeinit.mutex.post();
                    }
                }
                if (setBody==7){
                    unsigned int N = command.get(3).asInt();
                    if ((N >(i+1)) || (N < 1) || odeinit._wrld->S_cylOBJNUM == 0 ){reply.addString("object not known");}
                    else{
                        double x = command.get(4).asDouble();
                        double y = command.get(5).asDouble();
                        double z = command.get(6).asDouble();
                        odeinit.mutex.wait();
                        dGeomSetPosition(odeinit._wrld->s_cyl_obj[N-1].cylgeom[0],x,y,z);
                        odeinit.mutex.post();
                    }
                }
                if (setBody==8){
                    unsigned int N = command.get(3).asInt();
                    if ((N >(i+1)) || (N < 1) || odeinit._wrld->MODEL_NUM == 0 ){reply.addString("object not known");}
                    else{
                        double x = command.get(4).asDouble();
                        double y = command.get(5).asDouble();
                        double z = command.get(6).asDouble();
                        dBodySetPosition(odeinit._wrld->ThreeD_obj[N-1].body,x,y,z);
                        dBodySetLinearVel (odeinit._wrld->ThreeD_obj[N-1].body, 0.0, 0.0, 0.0);
                        dBodySetAngularVel (odeinit._wrld->ThreeD_obj[N-1].body, 0.0, 0.0, 0.0);
                    }
                }
                if (setBody==9){
                    unsigned int N = command.get(3).asInt();
                    if ((N >(i+1)) || (N < 1) || odeinit._wrld->s_MODEL_NUM == 0 ){reply.addString("object not known");}
                    else{
                        double x = command.get(4).asDouble();
                        double y = command.get(5).asDouble();
                        double z = command.get(6).asDouble();
                        dGeomSetPosition(odeinit._wrld->s_ThreeD_obj[N-1].geom,x,y,z);
                    }
                }
                if (setBody==10){
                    // SET MODEL DIRECTORY................................................
                    odeinit._wrld->model_DIR = command.get(3).asString();
                    cout << odeinit._wrld->model_DIR.c_str() << endl;
                }
                if (setBody==11){
                    unsigned int N = command.get(3).asInt();
                    if ((N >(i+1)) || (N < 1) || odeinit._wrld->SPHNUM == 0 ){reply.addString("object not known");}
                    else{
                        double x = command.get(4).asDouble();
                        double y = command.get(5).asDouble();
                        double z = command.get(6).asDouble();
                        odeinit.mutex.wait();
                        dBodySetPosition(odeinit._wrld->sph[N-1].sphbody,x,y,z);
                        odeinit.mutex.post();
                        reply.fromString("ok");
                    }
                }
                if (setBody==12){
                    unsigned int N = command.get(3).asInt();
                            
                    if ((N >(i+1)) || (N < 1) || odeinit._wrld->S_SPHNUM == 0 ){reply.addString("object not known");}
                    else{
                        double x = command.get(4).asDouble();
                        double y = command.get(5).asDouble();
                        double z = command.get(6).asDouble();
                        odeinit.mutex.wait();
                        dGeomSetPosition(odeinit._wrld->s_sph[N-1].sphgeom[0],x,y,z);
                        odeinit.mutex.post();
                        reply.fromString("ok");
                    }
                }
                //reply.fromString("ok");
            }
            if (subcmd=="mk"){ //this allows the user to create some objects around the world		

                if (setBody==2){
                    // box with gravity
                    if (num < MAXNUM){
                        i = odeinit._wrld->OBJNUM;								
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
                    odeinit._wrld->OBJNUM++;

                    odeinit.mutex.post();

                }
                if (setBody==5){
                    // static box (without gravity)
                    if (num < MAXNUM){
                        i = odeinit._wrld->S_OBJNUM;
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

                    odeinit.mutex.wait();

                    odeinit._wrld->s_obj[i].geom[0] = dCreateBox (odeinit.space,odeinit._wrld->s_obj[i].size[0],odeinit._wrld->s_obj[i].size[1],odeinit._wrld->s_obj[i].size[2]);
                    dGeomSetPosition(odeinit._wrld->s_obj[i].geom[0],x,y,z);
                    odeinit._wrld->s_color[i][0] = R;
                    odeinit._wrld->s_color[i][1] = G;
                    odeinit._wrld->s_color[i][2] = B;
                    odeinit._wrld->S_OBJNUM++;

                    odeinit.mutex.post();
                }
                if (setBody==6){
                    //cyl with gravity
                    if (num < MAXNUM){
                        i = odeinit._wrld->cylOBJNUM;
								
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
                    odeinit._wrld->cylOBJNUM++;

                    odeinit.mutex.post();
                }
                if (setBody==7){
                    // static cylinder (without gravity)
                    if (num < MAXNUM){
                        i = odeinit._wrld->S_cylOBJNUM;								
                    }
                    odeinit._wrld->s_cyl_obj[i].radius = command.get(3).asDouble();//radius
                    odeinit._wrld->s_cyl_obj[i].lenght = command.get(4).asDouble();//lenght
                    double x = command.get(5).asDouble(); // x position 
                    double y = command.get(6).asDouble(); // y position 
                    double z = command.get(7).asDouble(); // z position
                    double R = command.get(8).asDouble(); // colour R
                    double G = command.get(9).asDouble();// colour G
                    double B = command.get(10).asDouble();// colour B

                    odeinit.mutex.wait();

                    odeinit._wrld->s_cyl_obj[i].cylgeom[0] = dCreateCylinder (odeinit.space,odeinit._wrld->s_cyl_obj[i].radius,odeinit._wrld->s_cyl_obj[i].lenght);
                    dGeomSetPosition(odeinit._wrld->s_cyl_obj[i].cylgeom[0],x,y,z);
                    odeinit._wrld->s_color1[i][0] = R;
                    odeinit._wrld->s_color1[i][1] = G;
                    odeinit._wrld->s_color1[i][2] = B;
                    odeinit._wrld->S_cylOBJNUM++;

                    odeinit.mutex.post();
                }
                // 3D model
                if (setBody==8){

                    ConstString model = command.get(3).asString();
                    odeinit._wrld->texture = command.get(4).asString();
							                           
                    double x = command.get(5).asDouble(); // x position 
                    double y = command.get(6).asDouble(); // y position 
                    double z = command.get(7).asDouble(); // z position

                    cout << "\nAsking to create 3D Model.......\n" << endl;

                    odeinit.mutex.wait();
                    dMass m;
                    dMassSetZero(&m);
                    odeinit._wrld->TriData[a] = dGeomTriMeshDataCreate();
							
                    ConstString tmp = (char *) odeinit._wrld->model_DIR.c_str();
                    model = tmp + "/" + model;
                    odeinit._wrld->trimesh[a] = dLoadMeshFromX(model);
                    if (!odeinit._wrld->trimesh[a]){
                        cout << "Check spelling/location of file" << endl;
                        odeinit.mutex.post();
                    }else{
                        dGeomTriMeshDataBuildSingle(odeinit._wrld->TriData[a], odeinit._wrld->trimesh[a]->Vertices, 3 * sizeof(float), odeinit._wrld->trimesh[a]->VertexCount, odeinit._wrld->trimesh[a]->Indices, odeinit._wrld->trimesh[a]->IndexCount, 3 * sizeof(int));
                        odeinit._wrld->ThreeD_obj[a].body = dBodyCreate (odeinit.world);
                        odeinit._wrld->ThreeD_obj[a].geom = dCreateTriMesh(odeinit.space, odeinit._wrld->TriData[a], 0, 0, 0);
                        dGeomSetData(odeinit._wrld->ThreeD_obj[a].geom,odeinit._wrld->TriData[a]);
                        dMassSetTrimesh(&m,  0.1, odeinit._wrld->ThreeD_obj[a].geom);
                        //printf("mass at %f %f %f %f\n", m.mass, m.c[0], m.c[1], m.c[2]);
                        dGeomSetBody(odeinit._wrld->ThreeD_obj[a].geom, odeinit._wrld->ThreeD_obj[a].body);
                        dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
                        dGeomSetPosition(odeinit._wrld->ThreeD_obj[a].geom,x,y,z);
                        dBodySetMass(odeinit._wrld->ThreeD_obj[a].body, &m);
                        odeinit.mutex.post();

                        if (num < MAXNUM){
                            odeinit._wrld->MODEL_NUM++;
                            a = odeinit._wrld->MODEL_NUM;
                            odeinit._wrld->modelTexture[a-1] = odeinit._wrld->MODEL_NUM + 19;
                        }
                        odeinit.mutexTexture.wait();
                        odeinit._wrld->WAITLOADING = true;	
                        odeinit.mutexTexture.post();
                    }
                }
                if (setBody==9){

                    ConstString model = command.get(3).asString();
                    odeinit._wrld->texture = command.get(4).asString();
                            
                    double x = command.get(5).asDouble(); // x position 
                    double y = command.get(6).asDouble(); // y position 
                    double z = command.get(7).asDouble(); // z position

                    cout << "\nAsking to create static 3D Model.......\n" << endl;

                    odeinit.mutex.wait();
                    ConstString tmp = (char *) odeinit._wrld->model_DIR.c_str();
                    model = tmp + "/" + model;
                    odeinit._wrld->s_TriData[b] = dGeomTriMeshDataCreate();
                    odeinit._wrld->s_trimesh[b] = dLoadMeshFromX(model);
                    if (!odeinit._wrld->s_trimesh[b]){
                        cout << "Check spelling/location of file" << endl;
                        odeinit.mutex.post();
                    }else{
                        dGeomTriMeshDataBuildSingle(odeinit._wrld->s_TriData[b], odeinit._wrld->s_trimesh[b]->Vertices, 3 * sizeof(float), odeinit._wrld->s_trimesh[b]->VertexCount, odeinit._wrld->s_trimesh[b]->Indices, odeinit._wrld->s_trimesh[b]->IndexCount, 3 * sizeof(int));
                        odeinit._wrld->s_ThreeD_obj[b].geom = dCreateTriMesh(odeinit.space, odeinit._wrld->s_TriData[b], 0, 0, 0);
                        dGeomSetData(odeinit._wrld->s_ThreeD_obj[b].geom,odeinit._wrld->s_TriData[b]);
                        dGeomSetPosition(odeinit._wrld->s_ThreeD_obj[b].geom,x,y,z);
                        odeinit.mutex.post();

                        if (num < MAXNUM){
                            odeinit._wrld->s_MODEL_NUM++;
                            b = odeinit._wrld->s_MODEL_NUM;
                            odeinit._wrld->s_modelTexture[b-1] = odeinit._wrld->s_MODEL_NUM + 49;
                        }
                        odeinit.mutexTexture.wait();
                        odeinit._wrld->static_model = true;
                        odeinit._wrld->WAITLOADING = true;	
                        odeinit.mutexTexture.post();
                    }
                }
                if (setBody==11){
                    //sphere with gravity
                    if (num < MAXNUM){
                        i = odeinit._wrld->SPHNUM;							    
                    }
                    odeinit._wrld->sph[i].radius = command.get(3).asDouble();
                    double x = command.get(4).asDouble(); // x position 
                    double y = command.get(5).asDouble(); // y position 
                    double z = command.get(6).asDouble(); // z position
                    double R = command.get(7).asDouble(); // colour R
                    double G = command.get(8).asDouble();// colour G
                    double B = command.get(9).asDouble();// colour B
							
                    odeinit.mutex.wait();
							
                    dMass m;
                    dMassSetZero(&m);
                    odeinit._wrld->sph[i].sphbody = dBodyCreate(odeinit.world);
                    dMassSetSphereTotal(&m,DENSITY, odeinit._wrld->sph[i].radius);
                    dBodySetMass(odeinit._wrld->sph[i].sphbody, &m);
                    odeinit._wrld->sph[i].sphgeom[0] = dCreateSphere (odeinit.space, odeinit._wrld->sph[i].radius);
                    dGeomSetBody(odeinit._wrld->sph[i].sphgeom[0], odeinit._wrld->sph[i].sphbody);
                    dBodySetPosition(odeinit._wrld->sph[i].sphbody,x,y,z);
                    odeinit._wrld->color2[i][0] = R;
                    odeinit._wrld->color2[i][1] = G;
                    odeinit._wrld->color2[i][2] = B;
                    odeinit._wrld->SPHNUM++;
							
                    odeinit.mutex.post();
                }
                if (setBody==12){
                    //static sphere (without gravity)
                    if (num < MAXNUM){
                        i = odeinit._wrld->S_SPHNUM;							    
                    }
                    odeinit._wrld->s_sph[i].radius = command.get(3).asDouble();
                    double x = command.get(4).asDouble(); // x position 
                    double y = command.get(5).asDouble(); // y position 
                    double z = command.get(6).asDouble(); // z position
                    double R = command.get(7).asDouble(); // colour R
                    double G = command.get(8).asDouble();// colour G
                    double B = command.get(9).asDouble();// colour B
							
                    odeinit.mutex.wait();
							
                    odeinit._wrld->s_sph[i].sphgeom[0] = dCreateSphere (odeinit.space, odeinit._wrld->s_sph[i].radius);
                    dGeomSetPosition(odeinit._wrld->s_sph[i].sphgeom[0],x,y,z);
                    odeinit._wrld->s_color2[i][0] = R;
                    odeinit._wrld->s_color2[i][1] = G;
                    odeinit._wrld->s_color2[i][2] = B;
                    odeinit._wrld->S_SPHNUM++;
							
                    odeinit.mutex.post();
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
                if ( setBody == 8 ){
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
                        dGeomSetRotation(odeinit._wrld->ThreeD_obj[N-1].geom,Rtmp2);
                    }
                }

                if ( setBody == 9 ){
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
                        dGeomSetRotation(odeinit._wrld->s_ThreeD_obj[N-1].geom,Rtmp2);
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
                        odeinit.mutex.wait();
                        if (T == 1){
                               
                            if (hand == "left" && odeinit._iCub->actLHand=="off"){
                                odeinit._iCub->grab = dJointCreateFixed(odeinit.world,0);
                                dJointAttach (odeinit._iCub->grab,odeinit._iCub->l_hand,odeinit._wrld->obj[N-1].boxbody );
                                dJointSetFixed(odeinit._iCub->grab);
                            }
                            else if (hand == "left" && odeinit._iCub->actLHand=="on"){
                                odeinit._iCub->grab = dJointCreateFixed(odeinit.world,0);
                                dJointAttach (odeinit._iCub->grab, odeinit._iCub->body[10], odeinit._wrld->obj[N-1].boxbody );
                                dJointSetFixed(odeinit._iCub->grab);
                            }
                            else if (hand == "right" && odeinit._iCub->actRHand=="off"){
                                odeinit._iCub->grab1 = dJointCreateFixed(odeinit.world,0);
                                dJointAttach (odeinit._iCub->grab1,odeinit._iCub->r_hand,odeinit._wrld->obj[N-1].boxbody );
                                dJointSetFixed(odeinit._iCub->grab1);
                            }
                            else if (hand == "right" && odeinit._iCub->actRHand=="on"){
                                odeinit._iCub->grab1 = dJointCreateFixed(odeinit.world,0);
                                dJointAttach (odeinit._iCub->grab1, odeinit._iCub->body[11], odeinit._wrld->obj[N-1].boxbody );
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
                        odeinit.mutex.post();
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
	return ok;
}


void WorldManager::clear() {
    for (int x =0; x < a; x++)
        dTriMeshXDestroy(odeinit._wrld->trimesh[x]);
}


