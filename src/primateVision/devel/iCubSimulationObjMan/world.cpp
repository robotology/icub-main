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
#include <string.h>
#include <yarp/os/ConstString.h>
#include <string>
#include <iostream>


using std::string;
using std::cout;
using std::endl;

static float xyz[3], hpr[3];
static dReal ballVel[3], ballDamp[3];

//labels
Model *ballModel;
Model *bottleModel;
Model *canModel;
Model *duckModel;
Model *mugModel;
Model *fagsModel;
Model *unknownModel;

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
    if (reinitialized)
	{
        ballModel->reloadTextures();
        bottleModel->reloadTextures();
        canModel->reloadTextures();
        duckModel->reloadTextures();
        mugModel->reloadTextures();
		fagsModel->reloadTextures();
		unknownModel->reloadTextures();
	}
	if (actWorld == "on"){
		////table geom
		glColor3d(0.6,0.6,0.0);
		glPushMatrix();LDEsetM(dGeomGetPosition(tableGeom[0]),dGeomGetRotation(tableGeom[0]));
		DrawBox(0.03,0.5,0.03,false,textured,8);glPopMatrix();

		glPushMatrix();LDEsetM(dGeomGetPosition(tableGeom[1]),dGeomGetRotation(tableGeom[1]));
		DrawBox(0.03,0.5,0.03,false,textured,8);glPopMatrix();

		glPushMatrix();LDEsetM(dGeomGetPosition(tableGeom[2]),dGeomGetRotation(tableGeom[2]));
		DrawBox(0.03,0.5,0.03,false,textured,8);glPopMatrix();

		glPushMatrix();LDEsetM(dGeomGetPosition(tableGeom[3]),dGeomGetRotation(tableGeom[3]));
		DrawBox(0.03,0.5,0.03,false,textured,8);glPopMatrix();

		glPushMatrix();LDEsetM(dGeomGetPosition(tableGeom[4]),dGeomGetRotation(tableGeom[4]));
		DrawBox(0.7,0.03,0.4,false,textured,8);glPopMatrix();

		glColor3d(0.8,0.0,0.0);
		glPushMatrix();LDEsetM(dGeomGetPosition(box_part[0]),dGeomGetRotation(box_part[0]));
		DrawBox(0.01,0.01,0.1,false,textured,8);glPopMatrix();

		glPushMatrix();LDEsetM(dGeomGetPosition(box_part[1]),dGeomGetRotation(box_part[1]));
		DrawBox(0.01,0.01,0.1,false,textured,8);glPopMatrix();

		glPushMatrix();LDEsetM(dGeomGetPosition(box_part[2]),dGeomGetRotation(box_part[2]));
		DrawBox(0.1,0.01,0.01,false,textured,8);glPopMatrix();

		glPushMatrix();LDEsetM(dGeomGetPosition(box_part[3]),dGeomGetRotation(box_part[3]));
		DrawBox(0.1,0.01,0.01,false,textured,8);glPopMatrix();

		glPushMatrix();LDEsetM(dGeomGetPosition(box_part[4]),dGeomGetRotation(box_part[4]));
		DrawBox(0.01,0.1,0.01,false,textured,8);glPopMatrix();

		glPushMatrix();LDEsetM(dGeomGetPosition(box_part[5]),dGeomGetRotation(box_part[5]));
		DrawBox(0.01,0.1,0.01,false,textured,8);glPopMatrix();

		glPushMatrix();LDEsetM(dGeomGetPosition(box_part[6]),dGeomGetRotation(box_part[6]));
		DrawBox(0.01,0.1,0.01,false,textured,8);glPopMatrix();

		glPushMatrix();LDEsetM(dGeomGetPosition(box_part[7]),dGeomGetRotation(box_part[7]));
		DrawBox(0.01,0.1,0.01,false,textured,8);glPopMatrix();

		glPushMatrix();LDEsetM(dGeomGetPosition(box_part[8]),dGeomGetRotation(box_part[8]));
		DrawBox(0.01,0.01,0.1,false,textured,8);glPopMatrix();

		glPushMatrix();LDEsetM(dGeomGetPosition(box_part[9]),dGeomGetRotation(box_part[9]));
		DrawBox(0.01,0.01,0.1,false,textured,8);glPopMatrix();

		glPushMatrix();LDEsetM(dGeomGetPosition(box_part[10]),dGeomGetRotation(box_part[10]));
		DrawBox(0.1,0.01,0.01,false,textured,8);glPopMatrix();

		glPushMatrix();LDEsetM(dGeomGetPosition(box_part[11]),dGeomGetRotation(box_part[11]));
		DrawBox(0.1,0.01,0.01,false,textured,8);glPopMatrix();

		glColor3d(0.0,0.0,0.8);
		glPushMatrix(); LDEsetM(dBodyGetPosition(ballBody),dBodyGetRotation(ballBody));
		DrawSphere(0.04,false,textured,8);glPopMatrix();	

	}
	
	if (OBJNUM != waitOBJ){
		Time::delay(0.1); waitOBJ++;}
	else{
		for (int i=0; i<OBJNUM; i++) {
			glColor3d(color[i][0],color[i][1],color[i][2]);
			glPushMatrix();LDEsetM(dBodyGetPosition(obj[i].boxbody),dBodyGetRotation(obj[i].boxbody));
			DrawBox(obj[i].size[0],obj[i].size[1],obj[i].size[2],false,textured,8);glPopMatrix();
		}
	}
	for (int i=0; i<S_OBJNUM; i++) {
		glColor3d(s_color[i][0],s_color[i][1],s_color[i][2]);
		glPushMatrix();LDEsetM(dGeomGetPosition(s_obj[i].geom[0]),dGeomGetRotation(s_obj[i].geom[0]));
		DrawBox(s_obj[i].size[0],s_obj[i].size[1],s_obj[i].size[2],false,textured,8);glPopMatrix();
	}
	
	if (cylOBJNUM != waitOBJ1){
		Time::delay(0.1); waitOBJ1++;}
	else{
		for (int i=0; i<cylOBJNUM; i++) {
			glColor3d(color1[i][0],color1[i][1],color1[i][2]);
			glPushMatrix();LDEsetM(dBodyGetPosition(cyl_obj[i].cylbody),dBodyGetRotation(cyl_obj[i].cylbody));
			DrawCylinder(cyl_obj[i].radius,cyl_obj[i].lenght,false,textured,8);glPopMatrix();
		}
	}
	for (int i=0; i<S_cylOBJNUM; i++) {
		glColor3d(s_color1[i][0],s_color1[i][1],s_color1[i][2]);
		glPushMatrix();LDEsetM(dGeomGetPosition(s_cyl_obj[i].cylgeom[0]),dGeomGetRotation(s_cyl_obj[i].cylgeom[0]));
		DrawCylinder(s_cyl_obj[i].radius,s_cyl_obj[i].lenght,false,textured,8);glPopMatrix();
	}

	 //objManText = s_labl_obj[i].texture[i];
     
     for (int i=0; i < S_lablOBJNUM; i++) {
        glColor3d(1.0,1.0,1.0);
		
        rotation_x = rotation_x + rotation_x_increment;
        rotation_y = rotation_y + rotation_y_increment;
        rotation_z = rotation_z + rotation_z_increment;

        if (rotation_x > 359) rotation_x = 0;
        if (rotation_y > 359) rotation_y = 0;
        if (rotation_z > 359) rotation_z = 0;
       //glPushMatrix();
        //glRotatef(rotation_x,1.0,0.0,0.0); // Rotations of the object (the model matrix is multiplied by the rotation matrices)
        //glRotatef(rotation_y,0.0,1.0,0.0);
        
        /*glRotatef(90.0,1.0,0.0,0.0);glRotatef(rotation_z,0.0,0.0,1.0);*//*DrawSphere(s_labl_obj[i].radius,true,textured,8);*/
       
       
        if (s_labl_obj[i].label == 0){
            glPushMatrix();LDEsetM(dGeomGetPosition(s_labl_obj[i].labelgeom[0]),dGeomGetRotation(s_labl_obj[i].labelgeom[0])); glRotatef(180.0,0.0,0.0,1.0);glRotatef(-90.0,1.0,0.0,0.0);glScalef(0.05,0.05,0.05);glTranslatef(0,0.5,2);mugModel->draw(false,15); glPopMatrix();
            if (active[i]){
               // LDEsetM(dGeomGetPosition(s_labl_obj[i].labelgeom[0]),dGeomGetRotation(s_labl_obj[i].labelgeom[0]));
            }
        }
        if (s_labl_obj[i].label == 1){
            glPushMatrix();LDEsetM(dGeomGetPosition(s_labl_obj[i].labelgeom[0]),dGeomGetRotation(s_labl_obj[i].labelgeom[0])); glRotatef(180.0,0.0,0.0,1.0);glRotatef(-90.0,1.0,0.0,0.0);glScalef(0.05,0.05,0.05);glTranslatef(0,0.5,2);duckModel->draw(false,15); glPopMatrix();
            if (active[i]){
                //LDEsetM(dGeomGetPosition(s_labl_obj[i].labelgeom[0]),dGeomGetRotation(s_labl_obj[i].labelgeom[0]));
            }
        }
        if (s_labl_obj[i].label == 2){
            glPushMatrix();LDEsetM(dGeomGetPosition(s_labl_obj[i].labelgeom[0]),dGeomGetRotation(s_labl_obj[i].labelgeom[0])); glRotatef(180.0,0.0,0.0,1.0);glRotatef(-90.0,1.0,0.0,0.0);glScalef(0.05,0.05,0.05);glTranslatef(0,0.5,2);canModel->draw(false,15); glPopMatrix();
            //LDEsetM(dGeomGetPosition(s_labl_obj[i].labelgeom[0]),dGeomGetRotation(s_labl_obj[i].labelgeom[0]));
        }
        if (s_labl_obj[i].label == 3){
            glPushMatrix();LDEsetM(dGeomGetPosition(s_labl_obj[i].labelgeom[0]),dGeomGetRotation(s_labl_obj[i].labelgeom[0])); glRotatef(180.0,0.0,0.0,1.0);glRotatef(-90.0,1.0,0.0,0.0);glScalef(0.05,0.05,0.05);glTranslatef(0,0.5,2);ballModel->draw(false,15); glPopMatrix();
            //LDEsetM(dGeomGetPosition(s_labl_obj[i].labelgeom[0]),dGeomGetRotation(s_labl_obj[i].labelgeom[0]));
        }

        if (s_labl_obj[i].label == 4){
			if (!passed){
				DrawObjManTextures(objManText);    
			}

            glPushMatrix();LDEsetM(dGeomGetPosition(s_labl_obj[i].labelgeom[0]),dGeomGetRotation(s_labl_obj[i].labelgeom[0])); glRotatef(180.0,0.0,0.0,1.0);glRotatef(-90.0,1.0,0.0,0.0);glScalef(0.05,0.05,0.05);glTranslatef(0,0.5,2);bottleModel->draw(false,15); glPopMatrix();
            glPushMatrix();LDEsetM(dGeomGetPosition(s_labl_obj[i].labelgeom[0]),dGeomGetRotation(s_labl_obj[i].labelgeom[0]));DrawBox(0.08,0.08,0.001,false,textured,s_labl_obj[i].texture);glPopMatrix(); 
			if (!passed){
				
				passed = true;
			}
			
        }
		 if (s_labl_obj[i].label == 5){
           glDisable(GL_CULL_FACE); glPushMatrix();LDEsetM(dGeomGetPosition(s_labl_obj[i].labelgeom[0]),dGeomGetRotation(s_labl_obj[i].labelgeom[0])); glRotatef(180.0,0.0,0.0,1.0);glRotatef(-90.0,1.0,0.0,0.0);glScalef(0.05,0.05,0.05);glTranslatef(0,0.5,2);fagsModel->draw(false,15); glPopMatrix();glEnable(GL_CULL_FACE);
            //LDEsetM(dGeomGetPosition(s_labl_obj[i].labelgeom[0]),dGeomGetRotation(s_labl_obj[i].labelgeom[0]));
        }
		if (s_labl_obj[i].label == 6){
           glDisable(GL_CULL_FACE); glPushMatrix();LDEsetM(dGeomGetPosition(s_labl_obj[i].labelgeom[0]),dGeomGetRotation(s_labl_obj[i].labelgeom[0])); glRotatef(180.0,0.0,0.0,1.0);glRotatef(-90.0,1.0,0.0,0.0);glScalef(0.05,0.05,0.05);glTranslatef(0,0.5,2);unknownModel->draw(false,15); glPopMatrix();glEnable(GL_CULL_FACE);
            //LDEsetM(dGeomGetPosition(s_labl_obj[i].labelgeom[0]),dGeomGetRotation(s_labl_obj[i].labelgeom[0]));
        }
        
	}

	//get data from objManServer:
    objList = inPort_objList.read(); //blocking
 	glColor3d(1.0,1.0,1.0);

	if (objList->size()<=MAX_OBJS){
		//Set the positions for each object
		for (int i=0; i < objList->size(); i++) {
			objMutex.wait();
			dGeomSetPosition(s_labl_obj[i].labelgeom[0],objList->get(i)->x,objList->get(i)->y,objList->get(i)->z);
			objMutex.post();
		}
		
		//DRAW THEM ALL
		for (int i=0; i < objList->size(); i++) {
				
			if ( objList->get(i)->label == "bottle" ){

				int pin = objList->get(i)->tex.getRowSize();
				//get a pointer to the input data:
				unsigned char* imgDataIn = (unsigned char*)objList->get(i)->tex.getRawImage();
  				DrawObjManTexturesPort( i + 50, ObjManwidth, ObjManheight, imgDataIn, pin);//(unsigned char*)imgSized.getRawImage());
				
				glPushMatrix();LDEsetM(dGeomGetPosition(s_labl_obj[i].labelgeom[0]),dGeomGetRotation(s_labl_obj[i].labelgeom[0])); 
				glRotatef(180.0,0.0,0.0,1.0);glRotatef(-90.0,1.0,0.0,0.0);glScalef(0.05,0.05,0.05);glTranslatef(0,0.5,2);bottleModel->draw(false,15); 					glPopMatrix();
            	glPushMatrix();LDEsetM(dGeomGetPosition(s_labl_obj[i].labelgeom[0]),dGeomGetRotation(s_labl_obj[i].labelgeom[0]));
				glRotatef(-180.0,1.0,0.0,0.0);				
				DrawBox(0.08,0.08,0.001,false,textured, i + 50 );glPopMatrix(); 
			}

				if (objList->get(i)->label == "fags"){

				int pin = objList->get(i)->tex.getRowSize();
				//get a pointer to the input data:
				unsigned char* imgDataIn = (unsigned char*)objList->get(i)->tex.getRawImage();
				DrawObjManTexturesPort( i + 50, ObjManwidth, ObjManheight, imgDataIn, pin);

           		glDisable(GL_CULL_FACE); glPushMatrix();LDEsetM(dGeomGetPosition(s_labl_obj[i].labelgeom[0]),dGeomGetRotation(s_labl_obj[i].labelgeom[0])); 				glRotatef(180.0,0.0,0.0,1.0);glRotatef(-90.0,1.0,0.0,0.0);glScalef(0.05,0.05,0.05);glTranslatef(0,0.5,2);fagsModel->draw(false,15); 				glPopMatrix();
				glEnable(GL_CULL_FACE);
				glPushMatrix();LDEsetM(dGeomGetPosition(s_labl_obj[i].labelgeom[0]),dGeomGetRotation(s_labl_obj[i].labelgeom[0]));glRotatef(-180.0,1.0,0.0,0.0);	DrawBox(0.08,0.08,0.001,false,textured,i + 50);glPopMatrix(); 
        }
			if (objList->get(i)->label == "unknown"){
				int pin = objList->get(i)->tex.getRowSize();
				//get a pointer to the input data:
				unsigned char* imgDataIn = (unsigned char*)objList->get(i)->tex.getRawImage();
				DrawObjManTexturesPort( i + 50, ObjManwidth, ObjManheight, imgDataIn, pin);
           		glDisable(GL_CULL_FACE); glPushMatrix();LDEsetM(dGeomGetPosition(s_labl_obj[i].labelgeom[0]),dGeomGetRotation(s_labl_obj[i].labelgeom[0])); 				glRotatef(180.0,0.0,0.0,1.0);glRotatef(-90.0,1.0,0.0,0.0);glScalef(0.05,0.05,0.05);glTranslatef(0,0.5,2);unknownModel->draw(false,15); 					glPopMatrix();
				glEnable(GL_CULL_FACE);
            	glPushMatrix();LDEsetM(dGeomGetPosition(s_labl_obj[i].labelgeom[0]),dGeomGetRotation(s_labl_obj[i].labelgeom[0]));glRotatef(-180.0,1.0,0.0,0.0);	DrawBox(0.08,0.08,0.001,false,textured,i + 50);glPopMatrix(); 
        	}

		if (objList->get(i)->label == "coke"){
				int pin = objList->get(i)->tex.getRowSize();
				//get a pointer to the input data:
				unsigned char* imgDataIn = (unsigned char*)objList->get(i)->tex.getRawImage();
				DrawObjManTexturesPort( i + 50, ObjManwidth, ObjManheight, imgDataIn, pin);
           		glDisable(GL_CULL_FACE); glPushMatrix();LDEsetM(dGeomGetPosition(s_labl_obj[i].labelgeom[0]),dGeomGetRotation(s_labl_obj[i].labelgeom[0])); 				glRotatef(180.0,0.0,0.0,1.0);glRotatef(-90.0,1.0,0.0,0.0);glScalef(0.05,0.05,0.05);glTranslatef(0,0.5,2);canModel->draw(false,15); 					glPopMatrix();
				glEnable(GL_CULL_FACE);
            	glPushMatrix();LDEsetM(dGeomGetPosition(s_labl_obj[i].labelgeom[0]),dGeomGetRotation(s_labl_obj[i].labelgeom[0]));glRotatef(-180.0,1.0,0.0,0.0);DrawBox(0.08,0.08,0.001,false,textured,i + 50);glPopMatrix(); 
        	}
						
		}
	}
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
    // t3dInit();
  // _scale = computeScale(STRS);

	Property options;
	SimConfig finder;
  	//start left arm device driver
	ConstString general = finder.find("general");
	options.fromConfigFile(general.c_str());

	//readConfig(options,"conf/iCub_parts_activation.ini");
	actWorld = options.findGroup("RENDER").check("objects",Value(1),"What did the user select?").asString();

	//probing the port
  	inPort_s.open("/icubSim/input/serv_params");
  	Network::connect("/icubSim/input/serv_params","/objManServer/output/serv_params");
  	Network::connect("/objManServer/output/serv_params","/icubSim/input/serv_params");
  	
  	inPort_s.write(empty,server_response);
  	rsp = server_response.content();
  	std::cout << "ObjManServer Probe Response: " << rsp.toString() << std::endl;
  	ObjManwidth = rsp.width;
  	ObjManheight = rsp.height;
  	psb_in = rsp.psb;

	cout << "ObjManWidth "<< ObjManwidth << " ObjManHeight " << ObjManwidth << " Psb_in " << psb_in <<endl;

	inPort_objList.open("/icubSim/input/objList"); 
  	Network::connect("/objManServer/output/objList" , "/icubSim/input/objList");

	imgDataRGBA = (unsigned char*) malloc(4*ObjManwidth*ObjManheight*sizeof(unsigned char));
}

void worldSim::init( dWorldID world, dSpaceID space, dReal X, dReal Y, dReal Z ) {
    activateWorld();
     ballModel =  new Model();
     bottleModel =  new Model();
     canModel =  new Model();
     duckModel =  new Model();
     mugModel = new Model();	
	 fagsModel = new Model();
	 unknownModel= new Model();

    for (int x=0; x< 100; x++)
        active[x] = false;

    passed = false;

    SimConfig finder;
    ballModel->loadModelData(finder.find("data/model/Ms3d/Ball.ms3d").c_str()); 
    bottleModel->loadModelData(finder.find("data/model/Ms3d/Bottle.ms3d").c_str());  
    canModel->loadModelData(finder.find("data/model/Ms3d/Can.ms3d").c_str());  
    duckModel->loadModelData(finder.find("data/model/Ms3d/Duck.ms3d").c_str());  
    mugModel->loadModelData(finder.find("data/model/Ms3d/Mug.ms3d").c_str());  
	fagsModel->loadModelData(finder.find("data/model/Ms3d/fags.ms3d").c_str());  
    unknownModel->loadModelData(finder.find("data/model/Ms3d/unknown.ms3d").c_str());  

	/*------------iCub Space creation-------------*/
	/*
	* objects in the same space do not collide...see collision function in ICub_sim
	*/	
	boxObj = dSimpleSpaceCreate(space);
	dSpaceSetCleanup(boxObj,0);
    // Absolute rotation values (0-359 degrees) and rotation increments for each frame
    rotation_x=0; rotation_x_increment=0.1;
    rotation_y=0; rotation_y_increment=1;
    rotation_z=0; rotation_z_increment=1;
	
	int x, y ,z;
	for (int i=0; i<=MAX_OBJS; i++){
		x = -25 + i;
		y = 100;
		z = 50;
		s_labl_obj[i].labelgeom[0] = dCreateSphere (space,0.01);
		dGeomSetPosition(s_labl_obj[i].labelgeom[0],x,y,z); 
	}
	
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

        if (ballModel!=0)
        delete ballModel;
    if (bottleModel!=0)
        delete bottleModel;
    if (canModel!=0)
        delete canModel;
    if (duckModel!=0)
        delete duckModel;
    if (mugModel!=0)
        delete mugModel;
	if (fagsModel!=0)
        delete fagsModel;

	dSpaceDestroy (boxObj);
}

worldSim::worldSim(dWorldID world, dSpaceID space, dReal X, dReal Y, dReal Z)
: worldSimData() {
	resetSpeeds();
	init(world, space, X, Y, Z);
}
