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
 * \file iCub.cpp
 * \brief This cpp creates and places all the objects that are used to make the iCub simulator robot (all parts dimensions weight and joint configurations). It also deals with the sensors feedback and setting the joint speeds
 * \author Vadim Tikhanoff
 * \date 2007
 * \note Released under GNU GPL v2.0
 **/
#include <stdio.h>
#include <string.h>
#include <yarp/os/ConstString.h>
#include <yarp/os/Bottle.h>
#include <yarp/math/Math.h>
#include <iCub/iKin/iKinFwd.h>
#include <iCub/ctrl/math.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

#include "iCub.h"
#include "EyeLidsController.h"

#include "MS3D.h"
#include "xloader.h"

using std::string;
using std::cout;
using std::endl;

using namespace iCub::iKin;
using namespace iCub::ctrl;
using namespace yarp::math;

#ifndef CTRL_DEG2RAD 
#define CTRL_DEG2RAD M_PI/180.0
#endif
#ifndef CTRL_RAD2DEG
#define CTRL_RAD2DEG 180.0/M_PI
#endif

Model *iCubHeadModel;
Model *topEyeLidModel;
Model *bottomEyeLidModel;


ICubData::ICubData() {
}

// The maximum force that the simulated touch sensor can sense.
// Upper limit for the touch sensor is currently set as 100N. It can safely be changed.
#define TOUCH_SENSOR_MAX_FORCE 100.0

// This function returns the touch sensor value in the range 0-1.
// Input fb is the feedback structure of the contactJoint on a touch sensitive body. 
double getTouchSensorValue (dJointFeedback* fb) {

    double forceOnBody = sqrt(fb->f1[0]*fb->f1[0] + fb->f1[1]*fb->f1[1] + fb->f1[2]*fb->f1[2]);

    // If the force on the touch sensor is more than it can sense, set the force to the maximum value.
    if (forceOnBody > TOUCH_SENSOR_MAX_FORCE) {
        forceOnBody = TOUCH_SENSOR_MAX_FORCE;
    }
    // normalize the touch sensor output to the range 0-1.
    forceOnBody = forceOnBody / TOUCH_SENSOR_MAX_FORCE;

    return forceOnBody;
}

//function to check the activation of the sensor on the selected body...
double ICubSim::checkTouchSensor_continuousValued(dBodyID id){
    dJointID j;
    double result = 0;
    j  = dBodyGetJoint(id, 0); 

    if(dJointGetType(j) == dJointTypeContact) {
        // get the feedback from the joint.
        dJointFeedback * fb = dJointGetFeedback (j);
        // get the sensor value.
        result = getTouchSensorValue(fb);
    }

    return result;
}

double ICubSim::checkTouchSensor_continuousValued(int bodyToCheck) {
    dJointID j;
    double result = 0;

    if (bodyToCheck == 26 || bodyToCheck == 27 || bodyToCheck == 45 || bodyToCheck == 46)
    {
        if (bodyToCheck == 26 || bodyToCheck == 27){ 
            j  = dBodyGetJoint(lhandfingers3, 0);
        } else { 
            j  = dBodyGetJoint(rhandfingers3, 0); 
        }
    } else {
        j  = dBodyGetJoint(body[bodyToCheck], 0);
    }
    if(dJointGetType(j) == dJointTypeContact) {
        // get the feedback from the joint.
        dJointFeedback * fb = dJointGetFeedback (j);
        // get the sensor value.
        result = getTouchSensorValue(fb);
    }

    return result;
}

//function to check the activation of the sensor on the selected body...
bool ICubSim::checkTouchSensor(dBodyID id){
    dJointID j;
    bool result = false;
    j  = dBodyGetJoint(id, 0); 
    if(dJointGetType(j) == dJointTypeContact) {
        result = true;
    }
    return result;
}
bool ICubSim::checkTouchSensor(int bodyToCheck) {
    dJointID j;
    bool result = false;

    if (bodyToCheck == 26 || bodyToCheck == 27 || bodyToCheck == 45 || bodyToCheck == 46)
    {
        if (bodyToCheck == 26 || bodyToCheck == 27){ 
            j  = dBodyGetJoint(lhandfingers3, 0);
        }
        else{ 
            j  = dBodyGetJoint(rhandfingers3, 0); 
        }
    }

    else{
        j  = dBodyGetJoint(body[bodyToCheck], 0);
    }
    if(dJointGetType(j) == dJointTypeContact) {
        result = true;
    }

    return result;
}

void ICubSim::resetSpeeds() {
    int x;
    for(x = 0; x < 10; x++) {
        LLeg_speed[x] = 0.0;
        RLeg_speed[x] = 0.0;
        LLeg_torques[x] = 0.0;
        RLeg_torques[x] = 0.0;
    }

    for(x=0; x<7; x++){
        Torso_speed[x] = 0.0;
        Torso_torques[x] = 0.0;
    }

    for(x = 0; x < 25; x++) {
        la_speed[x] = 0.0;
        la_speed1[x] = 0.0;
        la_torques[x] = 0.0;
        ra_speed[x] = 0.0;
        ra_speed1[x] = 0.0;
        ra_torques[x] = 0.0;
        h_speed[x]  = 0.0;
        h_torques[x] = 0.0;
    }
}

void ICubSim::setJointControlAction() {
    OdeInit& odeInit = OdeInit::get();
    int cm[25];
    memset(cm, VOCAB_CM_VELOCITY, sizeof(int) * 25);
    /*for(int i=0;i<5;i++){
        printf("LAJoint %d angle: %.3f\t", i, 180.0/M_PI*dJointGetHingeAngle(LAjoints[i]));
        printf("vel_d: %.3f\t", 180.0/M_PI*dJointGetHingeParam(LAjoints[i], dParamVel));
        printf("fmax: %.1f\t", dJointGetHingeParam(LAjoints[i], dParamFMax));
        printf("torque_d: %.1f\n", la_torques[i]);
    }*/
    /*for(int i=0;i<5;i++){
        printf("RAJoint %d angle: %.1f\t", i, 180.0/M_PI*dJointGetHingeAngle(RAjoints[i]));
        printf("vel_d: %.1f\t", 180.0/M_PI*dJointGetHingeParam(RAjoints[i], dParamVel));
        printf("fmax: %.1f\t", dJointGetHingeParam(RAjoints[i], dParamFMax));
        printf("torque_d: %.1f\n", ra_torques[i]);
    }*/
    /*for(int i=0;i<3;i++){
        printf("TorsoJoint %d angle: %.1f\t", i, 180.0/M_PI*dJointGetHingeAngle(Torsojoints[i]));
        printf("vel_d: %.1f\t", 180.0/M_PI*dJointGetHingeParam(Torsojoints[i], dParamVel));
        printf("fmax: %.1f\t", dJointGetHingeParam(Torsojoints[i], dParamFMax));
        printf("torque_d: %.1f\n", Torso_torques[i]);
    }*/

    // BodyPart: 1 left arm, 2 right arm, 3 head, 4 left leg, 5 right leg, 6 torso
    // LEFT LEG
    int bodyPart = 4;
    if(odeInit._controls[bodyPart])
        odeInit._controls[bodyPart]->getControlModesRaw(cm);
    for (int x=0; x<6;x++){
        if(cm[x]==VOCAB_CM_TORQUE)
            dJointAddHingeTorque(LLegjoints[x], LLeg_torques[x]);
        else
            dJointSetHingeParam(LLegjoints[x], dParamVel, LLeg_speed[x]);
    }
    // RIGHT LEG
    bodyPart = 5;
    if(odeInit._controls[bodyPart])
        odeInit._controls[bodyPart]->getControlModesRaw(cm);
    else
        memset(cm, VOCAB_CM_VELOCITY, sizeof(int) * 25);
    for (int x=0; x<6;x++){
        if(cm[x]==VOCAB_CM_TORQUE)
            dJointAddHingeTorque(RLegjoints[x], RLeg_torques[x]);
        else
            dJointSetHingeParam(RLegjoints[x], dParamVel, RLeg_speed[x]);
    }
    // TORSO
    bodyPart = 6;
    if(odeInit._controls[bodyPart])
        odeInit._controls[bodyPart]->getControlModesRaw(cm);
    else
        memset(cm, VOCAB_CM_VELOCITY, sizeof(int) * 25);
    for (int x=0; x<5; x++){
        if(cm[x]==VOCAB_CM_TORQUE)
            dJointAddHingeTorque(Torsojoints[x], Torso_torques[x]);
        else
            dJointSetHingeParam(Torsojoints[x], dParamVel, Torso_speed[x]);
    }
    // LEFT ARM
    bodyPart = 1;
    if(odeInit._controls[bodyPart])
        odeInit._controls[bodyPart]->getControlModesRaw(cm);
    else
        memset(cm, VOCAB_CM_VELOCITY, sizeof(int) * 25);
    for (int x=0; x<5; x++){
        if(cm[x]==VOCAB_CM_TORQUE){
            dJointAddHingeTorque(LAjoints[x], la_torques[x]);
        }
        else
            dJointSetHingeParam(LAjoints[x], dParamVel, la_speed[x]);
    }

    // RIGHT ARM
    bodyPart = 2;
    if(odeInit._controls[bodyPart])
        odeInit._controls[bodyPart]->getControlModesRaw(cm);
    else
        memset(cm, VOCAB_CM_VELOCITY, sizeof(int) * 25);
    for (int x=0; x<5; x++){
        if(cm[x]==VOCAB_CM_TORQUE){
            dJointAddHingeTorque(RAjoints[x], ra_torques[x]);
        }
        else{
            dJointSetHingeParam(RAjoints[x], dParamVel, ra_speed[x]);
        }
    }

    for (int x=5; x<6;x++){//for the hand
        dJointSetUniversalParam(LAjoints[x],dParamVel, la_speed[x]);
        dJointSetUniversalParam(LAjoints[x], dParamVel2, la_speed1[x]);
        dJointSetUniversalParam(RAjoints[x],dParamVel,  ra_speed[x]);
        dJointSetUniversalParam(RAjoints[x], dParamVel2, ra_speed1[x]);
    }
    for (int x=6; x<25;x++){
        if (x!=9 && x!=13 && x!=17 && x!=21 && x!=22){
            dJointSetHingeParam(LAjoints[x], dParamVel, la_speed[x]);
            dJointSetHingeParam(RAjoints[x], dParamVel, ra_speed[x]);
        }
    }
    for (int x=22; x<23;x++){//for the hands
        dJointSetUniversalParam(LAjoints[x], dParamVel, la_speed[x]);
        dJointSetUniversalParam(LAjoints[x], dParamVel2, la_speed1[x]);
        dJointSetUniversalParam(RAjoints[x], dParamVel, ra_speed[x]);
        dJointSetUniversalParam(RAjoints[x], dParamVel2, ra_speed1[x]);
    }
        
    // HEAD
    bodyPart = 3;
    if(odeInit._controls[bodyPart])
        odeInit._controls[bodyPart]->getControlModesRaw(cm);
    else
        memset(cm, VOCAB_CM_VELOCITY, sizeof(int) * 25);
    for (int x=0; x<6; x++){
        if(cm[x]==VOCAB_CM_TORQUE)
            dJointAddHingeTorque(Hjoints[x], h_torques[x]);
        else
            dJointSetHingeParam(Hjoints[x], dParamVel, h_speed[x]);
    }
    
    /*dMatrix3 R;
    dRFromAxisAndAngle(R,1,0,0,eyeLids.eyeLidsRotation);
    dBodySetRotation(topEyeLid,R);
    dRFromAxisAndAngle(R,1,0,0,-eyeLids.eyeLidsRotation);
    dBodySetRotation(bottomEyeLid,R);*/
}

void ICubSim::draw(){
    if (reinitialized)
    {
        glFinish();
        glFlush();
        if(actHeadCover == "on")
        {
            iCubHeadModel->reloadTextures();
            topEyeLidModel->reloadTextures();
            bottomEyeLidModel->reloadTextures();
        }

        if (model_ThreeD_obj.size())
            model_ThreeD_obj[(*model_ThreeD_obj.begin()).first].reloadTexture(textureName[0], modelTexture[0]);

        reinitialized = false; 
    }

    if (actScreen == "on")
    {
        glColor3d(1.0,1.0,1.0);
        glPushMatrix();LDEsetM(dGeomGetPosition(screenGeom),dGeomGetRotation(screenGeom));
        DrawBox(1.0,1.0,0.001,false,textured,15);glPopMatrix();
    }

    //glColor3d(1.0,0.0,1.0);
    //glPushMatrix();LDEsetM(dGeomGetPosition(geom_cube[0]),dGeomGetRotation(geom_cube[0]));DrawBox(0.1,2.005,0.1,false,false,0);glPopMatrix();

    if (actLegs == "off"){
        glColor3d(1.0,1.0,1.0);
        if (actLegsCovers == "on")
        {
            glPushMatrix();
            LDEsetM(dGeomGetPosition(l_leg1_geom),dGeomGetRotation(l_leg1_geom));     //DRAW THE MODEL
            DrawX( model_trimesh["leftFoot"], modelTexture[0]);
            glPopMatrix();

            glPushMatrix();
            LDEsetM(dGeomGetPosition(l_leg3_geom),dGeomGetRotation(l_leg3_geom));     //DRAW THE MODEL
            DrawX( model_trimesh["lowerLeg"], modelTexture[0]);
            glPopMatrix();

            glPushMatrix();
            LDEsetM(dGeomGetPosition(l_leg6_geom),dGeomGetRotation(l_leg5_geom));     //DRAW THE MODEL
            DrawX( model_trimesh["upperLeftLeg"], modelTexture[0]);
            glPopMatrix();

            //right leg
            glPushMatrix();
            LDEsetM(dGeomGetPosition(r_leg1_geom),dGeomGetRotation(r_leg1_geom));     //DRAW THE MODEL
            DrawX( model_trimesh["rightFoot"], modelTexture[0]);
            glPopMatrix();

            glPushMatrix();
            LDEsetM(dGeomGetPosition(r_leg3_geom),dGeomGetRotation(r_leg3_geom));     //DRAW THE MODEL
            DrawX( model_trimesh["lowerLeg"], modelTexture[0]);
            glPopMatrix();

            glPushMatrix();
            LDEsetM(dGeomGetPosition(r_leg6_geom),dGeomGetRotation(r_leg5_geom));     //DRAW THE MODEL
            DrawX( model_trimesh["upperRightLeg"], modelTexture[0]);
            glPopMatrix();
        }
        if (actLegsCovers == "off")
        {
            glColor3d(0.9,0.9,0.9);
            glPushMatrix();LDEsetM(dGeomGetPosition(l_leg0_geom),dGeomGetRotation(l_leg0_geom));
            DrawBox(0.054,0.004,0.13,false,textured,2);glPopMatrix();//Taken from ODE use Y, Z, X
        }

        glColor3d(0.5,0.5,0.5);
        glPushMatrix(); LDEsetM(dGeomGetPosition(l_leg1_geom),dGeomGetRotation(l_leg1_geom));
        DrawCylinder(0.027,0.095,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dGeomGetPosition(l_leg2_geom),dGeomGetRotation(l_leg2_geom));
        DrawCylinder(0.0245,0.063,false,textured,2);glPopMatrix();

        glColor3d(1.0,1.0,1.0);
        glPushMatrix(); LDEsetM(dGeomGetPosition(l_leg3_geom),dGeomGetRotation(l_leg3_geom));
        DrawCylinder(0.0315,fabs(jP_leftLeg[2][2]-jP_leftLeg[1][2]),false,textured,2);glPopMatrix();	

        glColor3d(0.5,0.5,0.5);
        glPushMatrix(); LDEsetM(dGeomGetPosition(l_leg4_geom),dGeomGetRotation(l_leg4_geom));
        DrawCylinder(0.0315,0.077,false,textured,2);glPopMatrix();
        glColor3d(1.0,1.0,1.0);
        glPushMatrix(); LDEsetM(dGeomGetPosition(l_leg5_geom),dGeomGetRotation(l_leg5_geom));
        DrawCylinder(0.034,0.224,false,textured,2);glPopMatrix();

        glColor3d(0.5,0.5,0.5);
        glPushMatrix(); LDEsetM(dGeomGetPosition(l_leg6_geom),dGeomGetRotation(l_leg6_geom));
        DrawCylinder(0.031,0.075,false,textured,2);glPopMatrix();
        glColor3d(1.0,1.0,1.0);
        glPushMatrix(); LDEsetM(dGeomGetPosition(l_leg7_geom),dGeomGetRotation(l_leg7_geom));
        DrawCylinder(0.038,0.013,false,textured,2);glPopMatrix();

        //////////RIGHT LEG
        if (actLegsCovers == "off")
        {
            glColor3d(0.9,0.9,0.9);
            glPushMatrix();LDEsetM(dGeomGetPosition(r_leg0_geom),dGeomGetRotation(r_leg0_geom));
            DrawBox(0.054,0.004,0.13,false,textured,2);glPopMatrix();
        }
        glColor3d(0.5,0.5,0.5);
        glPushMatrix(); LDEsetM(dGeomGetPosition(r_leg1_geom),dGeomGetRotation(r_leg1_geom));
        DrawCylinder(0.027,0.095,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dGeomGetPosition(r_leg2_geom),dGeomGetRotation(r_leg2_geom));
        DrawCylinder(0.0245,0.063,false,textured,2);glPopMatrix();	

        glColor3d(1.0,1.0,1.0);
        glPushMatrix(); LDEsetM(dGeomGetPosition(r_leg3_geom),dGeomGetRotation(r_leg3_geom));
        DrawCylinder(0.0315,fabs(jP_rightLeg[2][2]-jP_rightLeg[1][2]),false,textured,2);glPopMatrix();

        glColor3d(0.5,0.5,0.5);
        glPushMatrix(); LDEsetM(dGeomGetPosition(r_leg4_geom),dGeomGetRotation(r_leg4_geom));
        DrawCylinder(0.0315,0.077,false,textured,2);glPopMatrix();	

        glColor3d(1.0,1.0,1.0);
        glPushMatrix(); LDEsetM(dGeomGetPosition(r_leg5_geom),dGeomGetRotation(r_leg5_geom));
        DrawCylinder(0.034,0.224,false,textured,2);glPopMatrix();

        glColor3d(0.5,0.5,0.5);
        glPushMatrix(); LDEsetM(dGeomGetPosition(r_leg6_geom),dGeomGetRotation(r_leg6_geom));
        DrawCylinder(0.031,0.075,false,textured,2);glPopMatrix();

        glColor3d(1.0,1.0,1.0);
        glPushMatrix(); LDEsetM(dGeomGetPosition(r_leg7_geom),dGeomGetRotation(r_leg7_geom));
        DrawCylinder(0.038,0.013,false,textured,2);glPopMatrix();

    }else{
        if (actLegsCovers == "off")
        {
            glColor3d(1.0,1.0,1.0);
            glPushMatrix();LDEsetM(dBodyGetPosition(leftLeg[0]),dBodyGetRotation(leftLeg[0]));
            DrawBox(0.054,0.004,0.13,false,textured,2);glPopMatrix();                                               //foot
        }
        if (actLegsCovers == "on")
        {   glColor3d(1.0,1.0,1.0);
            glPushMatrix();
            LDEsetM(dBodyGetPosition(leftLeg[1]),dBodyGetRotation(leftLeg[0]));     //DRAW THE MODEL
            DrawX( model_trimesh["leftFoot"], modelTexture[0]);
            glPopMatrix();

            glPushMatrix();
            LDEsetM(dGeomGetPosition(leftLeg_2_2),dGeomGetRotation(leftLeg_2_2));     //DRAW THE MODEL
            DrawX( model_trimesh["lowerLeg"], modelTexture[0]);
            glPopMatrix();

            glPushMatrix();
            LDEsetM(dGeomGetPosition(leftLeg_4_2),dGeomGetRotation(leftLeg_3_2));     //DRAW THE MODEL
            DrawX( model_trimesh["upperLeftLeg"], modelTexture[0]);
            glPopMatrix();
            //right leg
            glPushMatrix();
            LDEsetM(dBodyGetPosition(rightLeg[1]),dBodyGetRotation(rightLeg[0]));     //DRAW THE MODEL
            DrawX( model_trimesh["rightFoot"], modelTexture[0]);
            glPopMatrix();

            glPushMatrix();
            LDEsetM(dGeomGetPosition(rightLeg_2_2),dGeomGetRotation(rightLeg_2_2));     //DRAW THE MODEL
            //glTranslatef(0.0, 0.0,0.5*fabs(jP_leftArm[5][2] - jP_leftArm[4][2]));
            DrawX( model_trimesh["lowerLeg"], modelTexture[0]);
            glPopMatrix();

            glPushMatrix();
            LDEsetM(dGeomGetPosition(rightLeg_4_2),dGeomGetRotation(rightLeg_3_2));     //DRAW THE MODEL
            //glTranslatef(0.0, 0.0,0.5*fabs(jP_leftArm[5][2] - jP_leftArm[4][2]));
            DrawX( model_trimesh["upperRightLeg"], modelTexture[0]);
            glPopMatrix();
        }

        glColor3d(0.5,0.5,0.5);
        glPushMatrix(); LDEsetM(dBodyGetPosition(leftLeg[1]),dBodyGetRotation(leftLeg[1]));
        DrawCylinder(0.027,0.095,false,textured,2);glPopMatrix();                                               //foot joint
        
        glPushMatrix(); LDEsetM(dGeomGetPosition(leftLeg_2_1),dGeomGetRotation(leftLeg_2_1));
        DrawCylinder(0.0245,0.063,false,textured,2);glPopMatrix();                                              //foot joint
        glColor3d(1.0,1.0,1.0);
        glPushMatrix(); LDEsetM(dGeomGetPosition(leftLeg_2_2),dGeomGetRotation(leftLeg_2_2));
        DrawCylinder(0.0315,fabs(jP_leftLeg[2][2]-jP_leftLeg[1][2]),false,textured,2);glPopMatrix();            //tibia

        glColor3d(0.5,0.5,0.5);
        glPushMatrix(); LDEsetM(dGeomGetPosition(leftLeg_3_1),dGeomGetRotation(leftLeg_3_1));
        DrawCylinder(0.0315,0.077,false,textured,2);glPopMatrix();	                                            //knee
        glColor3d(1.0,1.0,1.0);
        glPushMatrix(); LDEsetM(dGeomGetPosition(leftLeg_3_2),dGeomGetRotation(leftLeg_3_2));
        DrawCylinder(0.034,fabs(jP_leftLeg[3][2]-jP_leftLeg[2][2]),false,textured,2);glPopMatrix();             //thigh

        //glPushMatrix(); LDEsetM(dGeomGetPosition(leftLeg_4_1),dGeomGetRotation(leftLeg_4_1));
        //DrawSphere(0.017,false,false);glPopMatrix();
        glColor3d(0.5,0.5,0.5);
        glPushMatrix(); LDEsetM(dGeomGetPosition(leftLeg_4_2),dGeomGetRotation(leftLeg_4_2));
        DrawCylinder(0.031,0.075,false,textured,2);glPopMatrix();

        glColor3d(1.0,1.0,1.0);
        glPushMatrix(); LDEsetM(dBodyGetPosition(leftLeg[5]),dBodyGetRotation(leftLeg[5]));
        DrawCylinder(0.038,0.013,false,textured,2);glPopMatrix();
        //////////RIGHT LEG
        if (actLegsCovers == "off")
        {
            glColor3d(0.9,0.9,0.9);
            glPushMatrix();LDEsetM(dBodyGetPosition(rightLeg[0]),dBodyGetRotation(rightLeg[0]));
            DrawBox(0.054,0.004,0.13,false,textured,2);glPopMatrix();
        }

        glColor3d(0.5,0.5,0.5);
        glPushMatrix(); LDEsetM(dBodyGetPosition(rightLeg[1]),dBodyGetRotation(rightLeg[1]));
        DrawCylinder(0.027,0.095,false,textured,2);glPopMatrix();
        glPushMatrix(); LDEsetM(dGeomGetPosition(rightLeg_2_1),dGeomGetRotation(rightLeg_2_1));
        DrawCylinder(0.0245,0.063,false,textured,2);glPopMatrix();
        glColor3d(1.0,1.0,1.0);
        glPushMatrix(); LDEsetM(dGeomGetPosition(rightLeg_2_2),dGeomGetRotation(rightLeg_2_2));
        DrawCylinder(0.0315,fabs(jP_rightLeg[2][2]-jP_rightLeg[1][2]),false,textured,2);glPopMatrix();

        glColor3d(0.5,0.5,0.5);
        glPushMatrix(); LDEsetM(dGeomGetPosition(rightLeg_3_1),dGeomGetRotation(rightLeg_3_1));
        DrawCylinder(0.0315,0.077,false,textured,2);glPopMatrix();
        glColor3d(1.0,1.0,1.0);
        glPushMatrix(); LDEsetM(dGeomGetPosition(rightLeg_3_2),dGeomGetRotation(rightLeg_3_2));
        DrawCylinder(0.034,fabs(jP_rightLeg[3][2]-jP_rightLeg[2][2]),false,textured,2);glPopMatrix();
        //glPushMatrix(); LDEsetM(dGeomGetPosition(rightLeg_4_1),dGeomGetRotation(rightLeg_4_1));
        //DrawSphere(0.017,false,false);glPopMatrix();

        glColor3d(0.5,0.5,0.5);
        glPushMatrix(); LDEsetM(dGeomGetPosition(rightLeg_4_2),dGeomGetRotation(rightLeg_4_2));
        DrawCylinder(0.031,0.075,false,textured,2);glPopMatrix();
        glColor3d(1.0,1.0,1.0);
        glPushMatrix(); LDEsetM(dBodyGetPosition(rightLeg[5]),dBodyGetRotation(rightLeg[5]));
        DrawCylinder(0.038,0.013,false,textured,2);glPopMatrix();
    }
    if (actTorso == "off"){
        glColor3d(1.0,1.0,1.0);
        if (actTorsoCovers == "on")
        {
            glPushMatrix();
            LDEsetM(dGeomGetPosition(torso1_geom),dGeomGetRotation(torso1_geom));     //DRAW THE MODEL
            DrawX( model_trimesh["waist"], modelTexture[0]);
            glPopMatrix();

            glPushMatrix();
            if (actHead == "on")
                LDEsetM(dBodyGetPosition(neck[0]),dGeomGetRotation(torso3_geom));     //DRAW THE MODEL
            else
                LDEsetM(dGeomGetPosition(neck0_geom),dGeomGetRotation(torso3_geom));

            DrawX( model_trimesh["torso"], modelTexture[0]);
            glPopMatrix();
        }
        glColor3d(1.0,1.0,1.0);
        glPushMatrix(); LDEsetM(dGeomGetPosition(torso0_geom),dGeomGetRotation(torso0_geom));
        DrawBox(0.0470,fabs((jP_torso[0][2]-0.031/*torso pitch cyl rad*/)-(jP_leftLeg[5][2]-0.031/*leg cyl rad*/)),0.064,false,textured,2);glPopMatrix();

        glPushMatrix();LDEsetM(dGeomGetPosition(torso1_geom),dGeomGetRotation(torso1_geom));
        //DrawBox(0.176,0.063,0.127,false,textured,2);glPopMatrix();
        DrawCylinder(0.031,fabs(jP_leftLeg[3][1]-jP_rightLeg[3][1]),false,textured,2);glPopMatrix();
        glColor3d(0.5,0.5,0.5);
        glPushMatrix(); LDEsetM(dGeomGetPosition(torso2_geom),dGeomGetRotation(torso2_geom));
        DrawCylinder(0.031,0.097,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dGeomGetPosition(torso3_geom),dGeomGetRotation(torso3_geom));
        DrawCylinder(0.04,0.0274,false,textured,2);glPopMatrix();
        glColor3d(1.0,1.0,1.0);

        if (actTorsoCovers == "off"){
            glPushMatrix();LDEsetM(dBodyGetPosition(torso[4]),dBodyGetRotation(torso[4]));
            DrawBox(fabs(jP_leftArm[1][1]-jP_torso[2][1])-0.011-0.5*0.059 /*clavicule height + half shoulder height*/,fabs((jP_head[0][2]-0.015/*head pitch cyl rad*/)-(jP_torso[2][2]+0.031+0.0274/*torso roll cyl rad + torso yaw cyl height*/))/*0.118*/,0.109,false,textured,2);glPopMatrix();

            glPushMatrix();LDEsetM(dBodyGetPosition(torso[5]),dBodyGetRotation(torso[5]));
            DrawBox(fabs(jP_rightArm[1][1]-jP_torso[2][1])-0.011-0.5*0.059 /*clavicule height + half shoulder height*/,fabs((jP_head[0][2]-0.015/*head pitch cyl rad*/)-(jP_torso[2][2]+0.031+0.0274/*torso roll cyl rad + torso yaw cyl height*/))/*0.118*/,0.109,false,textured,2);glPopMatrix();
        }
     }else{
        glColor3d(1.0,1.0,1.0);
        if (actTorsoCovers == "on")
        {
            glPushMatrix();
            LDEsetM(dBodyGetPosition(torso[1]),dBodyGetRotation(torso[0]));     //DRAW THE MODEL
            DrawX( model_trimesh["waist"], modelTexture[0]);
            glPopMatrix();

            glPushMatrix();
            if (actHead == "on")
                LDEsetM(dBodyGetPosition(neck[0]),dBodyGetRotation(torso[3]));     //DRAW THE MODEL
            else
                LDEsetM(dGeomGetPosition(neck0_geom),dBodyGetRotation(torso[3]));

            DrawX( model_trimesh["torso"], modelTexture[0]);
            glPopMatrix();
        }
        //////TORSO
        glColor3d(1.0,1.0,1.0);
        glPushMatrix();LDEsetM(dBodyGetPosition(torso[0]),dBodyGetRotation(torso[0]));
        DrawBox(0.0470,fabs((jP_torso[0][2]-0.031/*torso pitch cyl rad*/)-(jP_leftLeg[5][2]-0.031/*leg cyl rad*/)),0.064,false,textured,2);glPopMatrix();

        glPushMatrix();LDEsetM(dBodyGetPosition(torso[1]),dBodyGetRotation(torso[1]));
        //DrawBox(0.176,0.063,0.127,false,textured,2);glPopMatrix();
        DrawCylinder(0.031,fabs(jP_leftLeg[3][1]-jP_rightLeg[3][1]),false,textured,2);glPopMatrix();
        glColor3d(0.5,0.5,0.5);
        glPushMatrix(); LDEsetM(dBodyGetPosition(torso[2]),dBodyGetRotation(torso[2]));
        DrawCylinder(0.031,0.097,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dBodyGetPosition(torso[3]),dBodyGetRotation(torso[3]));
        DrawCylinder(0.04,0.0274,false,textured,2);glPopMatrix();
        glColor3d(1.0,1.0,1.0);

        if (actTorsoCovers == "off"){
            glPushMatrix();LDEsetM(dBodyGetPosition(torso[4]),dBodyGetRotation(torso[4]));
            DrawBox(fabs(jP_leftArm[1][1]-jP_torso[2][1])-0.011-0.5*0.059 /*clavicule height + half shoulder height*/,fabs((jP_head[0][2]-0.015/*head pitch cyl rad*/)-(jP_torso[2][2]+0.031+0.0274/*torso roll cyl rad + torso yaw cyl height*/))/*0.118*/,0.109,false,textured,2);
            glPopMatrix();

            glPushMatrix();LDEsetM(dBodyGetPosition(torso[5]),dBodyGetRotation(torso[5]));
            DrawBox(fabs(jP_rightArm[1][1]-jP_torso[2][1])-0.011-0.5*0.059/*clavicule height + half shoulder height*/,fabs((jP_head[0][2]-0.015/*head pitch cyl rad*/)-(jP_torso[2][2]+0.031+0.0274/*torso roll cyl rad + torso yaw cyl height*/))/*0.118*/,0.109,false,textured,2);
            glPopMatrix();
        }
    }
    if (actLArm == "off"){
        glColor3d(1.0,1.0,1.0);
        if (actLeftArmCovers == "on")
        {
            glPushMatrix();
            LDEsetM(dGeomGetPosition(larm2_geom),dGeomGetRotation(larm2_geom));     //DRAW THE MODEL
            glTranslatef(0.0, 0.0,0.5*fabs(jP_leftArm[4][2] - jP_leftArm[2][2]));
            DrawX( model_trimesh["upperLeftArm"], modelTexture[0]);
            glPopMatrix();

            //draw texture
            glPushMatrix();
            LDEsetM(dGeomGetPosition(larm3_geom),dGeomGetRotation(larm3_geom));     //DRAW THE MODEL
            glTranslatef(0.0, 0.0,0.5*fabs(jP_leftArm[5][2] - jP_leftArm[3][2]));
            DrawX( model_trimesh["lowerArm"], modelTexture[0]);
            glPopMatrix();
        }
        glColor3d(1.0,1.0,1.0);
        glPushMatrix(); LDEsetM(dGeomGetPosition(larm0_geom),dGeomGetRotation(larm0_geom));
        DrawCylinder(0.031,0.011,false,textured,2);glPopMatrix();
        glColor3d(0.5,0.5,0.5);
        glPushMatrix(); LDEsetM(dGeomGetPosition(larm1_geom),dGeomGetRotation(larm1_geom));
        DrawCylinder(0.03,0.059,false,textured,2);glPopMatrix();
        glColor3d(1.0,1.0,1.0);
        glPushMatrix(); LDEsetM(dGeomGetPosition(larm2_geom),dGeomGetRotation(larm2_geom));
        DrawCylinder(0.026 ,fabs(jP_leftArm[4][2]-jP_leftArm[2][2]),false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dGeomGetPosition(larm3_geom),dGeomGetRotation(larm3_geom));
        DrawCylinder(0.02 ,fabs(jP_leftArm[5][2]-jP_leftArm[3][2]),false,textured,2);glPopMatrix();

    }else{
        //LEFT ARM
        glColor3d(1.0,1.0,1.0);
        if (actLeftArmCovers == "on")
        {
            glPushMatrix();
            LDEsetM(dBodyGetPosition(body[6]),dBodyGetRotation(body[4]));     //DRAW THE MODEL
            DrawX( model_trimesh["upperLeftArm"], modelTexture[0]);
            glPopMatrix();

            //draw texture
            glPushMatrix();
            LDEsetM(dBodyGetPosition(body[8]),dBodyGetRotation(body[8]));     //DRAW THE MODEL
            glTranslatef(0.0, 0.0,0.5*fabs(jP_leftArm[5][2] - jP_leftArm[3][2]));
            DrawX( model_trimesh["lowerArm"], modelTexture[0]);
            glPopMatrix();
        }

        glColor3d(1.0,1.0,1.0);
        glPushMatrix(); LDEsetM(dBodyGetPosition(body[0]),dBodyGetRotation(body[0]));
        DrawCylinder(0.031,0.011,false,textured,2);glPopMatrix();
        glColor3d(0.5,0.5,0.5);
        glPushMatrix(); LDEsetM(dBodyGetPosition(body[2]),dBodyGetRotation(body[2]));
        DrawCylinder(0.03,0.059,false,textured,2);glPopMatrix();
        glColor3d(1.0,1.0,1.0); 
        glPushMatrix(); LDEsetM(dBodyGetPosition(body[4]),dBodyGetRotation(body[4]));
        DrawCylinder(0.026 ,fabs(jP_leftArm[4][2]-jP_leftArm[2][2]),false,textured,2);
        glPopMatrix();

        //glColor3d(1.0,1.0,0.0);
        //glPushMatrix(); LDEsetM(dBodyGetPosition(body[6]),dBodyGetRotation(body[6]));
        //DrawSphere(0.01,false,false,0);glPopMatrix();
        glColor4d(1.0,1.0,1.0,0.5);
        glPushMatrix(); LDEsetM(dBodyGetPosition(body[8]),dBodyGetRotation(body[8]));
        DrawCylinder(0.02 ,fabs(jP_leftArm[5][2]-jP_leftArm[3][2]),false,textured,2);
        glPopMatrix();

    }
    if (actRArm == "off"){
        glColor3d(1.0,1.0,1.0);
        if (actRightArmCovers == "on")
        {
            glPushMatrix();
            LDEsetM(dGeomGetPosition(rarm2_geom),dGeomGetRotation(rarm2_geom));     //DRAW THE MODEL
            glTranslatef(0.0, 0.0,0.5*fabs(jP_rightArm[4][2] - jP_rightArm[2][2]));
            DrawX( model_trimesh["upperRightArm"], modelTexture[0]);
            glPopMatrix();

            //draw texture
            glPushMatrix();
            LDEsetM(dGeomGetPosition(rarm3_geom),dGeomGetRotation(rarm3_geom));     //DRAW THE MODEL
            glTranslatef(0.0, 0.0,0.5*fabs(jP_leftArm[5][2] - jP_leftArm[3][2]));
            DrawX( model_trimesh["lowerArm"], modelTexture[0]);
            glPopMatrix();
        }
        glColor3d(1.0,1.0,1.0);
        glPushMatrix(); LDEsetM(dGeomGetPosition(rarm0_geom),dGeomGetRotation(rarm0_geom));
        DrawCylinder(0.031,0.011,false,textured,2);glPopMatrix();
        glColor3d(0.5,0.5,0.5);
        glPushMatrix(); LDEsetM(dGeomGetPosition(rarm1_geom),dGeomGetRotation(rarm1_geom));
        DrawCylinder(0.03,0.059,false,textured,2);glPopMatrix();
        glColor3d(1.0,1.0,1.0);
        glPushMatrix(); LDEsetM(dGeomGetPosition(rarm2_geom),dGeomGetRotation(rarm2_geom));
        DrawCylinder(0.026 ,fabs(jP_rightArm[4][2]-jP_rightArm[2][2]),false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dGeomGetPosition(rarm3_geom),dGeomGetRotation(rarm3_geom));
        DrawCylinder(0.02 ,fabs(jP_rightArm[5][2]-jP_rightArm[3][2]),false,textured,2);glPopMatrix();

    }else{
        //RIGHT ARM
        if (actRightArmCovers == "on")
        {
            glPushMatrix();
            LDEsetM(dBodyGetPosition(body[7]),dBodyGetRotation(body[5]));     //DRAW THE MODEL
            DrawX( model_trimesh["upperRightArm"], modelTexture[0]);
            glPopMatrix();

            glPushMatrix();
            LDEsetM(dBodyGetPosition(body[9]),dBodyGetRotation(body[9]));     //DRAW THE MODEL
            glTranslatef(0.0, 0.0,0.5*fabs(jP_rightArm[5][2] - jP_rightArm[3][2]));
            DrawX( model_trimesh["lowerArm"], modelTexture[0]);
            glPopMatrix();
        }
        glColor3d(1.0,1.0,1.0);
        glPushMatrix(); LDEsetM(dBodyGetPosition(body[1]),dBodyGetRotation(body[1]));
        DrawCylinder(0.031,0.011,false,textured,2);glPopMatrix();
        glColor3d(0.5,0.5,0.5);
        glPushMatrix(); LDEsetM(dBodyGetPosition(body[3]),dBodyGetRotation(body[3]));
        DrawCylinder(0.03,0.059,false,textured,2);glPopMatrix();

        glColor3d(1.0,1.0,1.0);
        glPushMatrix(); LDEsetM(dBodyGetPosition(body[5]),dBodyGetRotation(body[5]));
        DrawCylinder(0.026 ,fabs(jP_rightArm[4][2]-jP_rightArm[2][2]),false,textured,2);glPopMatrix();

        //glPushMatrix(); LDEsetM(dBodyGetPosition(body[7]),dBodyGetRotation(body[7]));
        //DrawSphere(0.01,false,false);glPopMatrix();

        glPushMatrix(); LDEsetM(dBodyGetPosition(body[9]),dBodyGetRotation(body[9]));
        DrawCylinder(0.02 ,fabs(jP_rightArm[5][2]-jP_rightArm[3][2]),false,textured,2);glPopMatrix();
        
    }
    if (actLHand == "off"){
        glColor3d(1.0,1.0,1.0);
        if (actLeftArmCovers == "off")
        {
            glPushMatrix();LDEsetM(dGeomGetPosition(l_hand0_geom),dGeomGetRotation(l_hand0_geom));
            DrawBox(0.022,0.069,0.065,false,textured,2);glPopMatrix();
        }else
        {
            glPushMatrix();
            LDEsetM(dGeomGetPosition(l_hand0_geom),dGeomGetRotation(l_hand0_geom));     //DRAW THE MODEL
            glTranslatef(0.0, 0.5*fabs(jP_rightArm[7][2] - jP_rightArm[6][2]), 0.0);
            DrawX( model_trimesh["leftPalm"], modelTexture[0]);
            glPopMatrix();

        }

        glPushMatrix(); LDEsetM(dGeomGetPosition(l_hand1_geom),dGeomGetRotation(l_hand1_geom));
        DrawCylinder(0.0065,0.08,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dGeomGetPosition(l_hand2_geom),dGeomGetRotation(l_hand2_geom));
        DrawCylinder(0.0065,0.084,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dGeomGetPosition(l_hand3_geom),dGeomGetRotation(l_hand3_geom));
        DrawCylinder(0.0065,0.08,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dGeomGetPosition(l_hand4_geom),dGeomGetRotation(l_hand4_geom));
        DrawCylinder(0.0065,0.073,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dGeomGetPosition(l_hand5_geom),dGeomGetRotation(l_hand5_geom));
        DrawCylinder(0.0065,0.064,false,textured,2);glPopMatrix();

    }else{

        if (actLeftArmCovers == "off")
        {
            glPushMatrix();LDEsetM(dBodyGetPosition(body[10]),dBodyGetRotation(body[10]));
            DrawBox(0.022,0.069,0.065,false,textured,2);glPopMatrix();//Taken from ODE use Y, Z, Xs
        }else
        {
            glPushMatrix();
            LDEsetM(dBodyGetPosition(body[10]),dBodyGetRotation(body[10]));     //DRAW THE MODEL
            glTranslatef(0.0, 0.5*fabs(jP_rightArm[7][2] - jP_rightArm[6][2]), 0.0);
            DrawX( model_trimesh["leftPalm"], modelTexture[0]);
            glPopMatrix();

        }

        //LEFT HAND + FINGERS
        glPushMatrix(); LDEsetM(dBodyGetPosition(body[12]),dBodyGetRotation(body[12]));
        DrawCylinder(0.0065,0.012,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dBodyGetPosition(body[13]),dBodyGetRotation(body[13]));
        DrawCylinder(0.0065,0.012,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dGeomGetPosition(lhandfings0_geom),dGeomGetRotation(lhandfings0_geom));
        DrawCylinder(0.0065,0.012,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dGeomGetPosition(lhandfings1_geom),dGeomGetRotation(lhandfings1_geom));
        DrawCylinder(0.0065,0.012,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dBodyGetPosition(body[16]),dBodyGetRotation(body[16]));
        DrawCylinder(0.0065,0.026,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dBodyGetPosition(body[17]),dBodyGetRotation(body[17]));
        DrawCylinder(0.0065,0.028,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dGeomGetPosition(lhandfings2_geom),dGeomGetRotation(lhandfings2_geom));
        DrawCylinder(0.0065,0.026,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dGeomGetPosition(lhandfings3_geom),dGeomGetRotation(lhandfings3_geom));
        DrawCylinder(0.0065,0.022,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dBodyGetPosition(body[20]),dBodyGetRotation(body[20]));
        DrawCylinder(0.0065,0.022,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dBodyGetPosition(body[21]),dBodyGetRotation(body[21]));
        DrawCylinder(0.0065,0.024,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dGeomGetPosition(lhandfings4_geom),dGeomGetRotation(lhandfings4_geom));
        DrawCylinder(0.0065,0.022,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dGeomGetPosition(lhandfings5_geom),dGeomGetRotation(lhandfings5_geom));
        DrawCylinder(0.0065,0.019,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dBodyGetPosition(body[24]),dBodyGetRotation(body[24]));
        DrawCylinder(0.0065,0.02,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dBodyGetPosition(body[25]),dBodyGetRotation(body[25]));
        DrawCylinder(0.0065,0.02,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dGeomGetPosition(lhandfings6_geom),dGeomGetRotation(lhandfings6_geom));
        DrawCylinder(0.0065,0.02,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dGeomGetPosition(lhandfings7_geom),dGeomGetRotation(lhandfings7_geom));
        DrawCylinder(0.0065,0.02,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dBodyGetPosition(body[28]),dBodyGetRotation(body[28]));
        DrawCylinder(0.0065,0.026,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dBodyGetPosition(body[29]),dBodyGetRotation(body[29]));
        DrawCylinder(0.0065,0.022,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dBodyGetPosition(body[30]),dBodyGetRotation(body[30]));
        DrawCylinder(0.0065,0.016,false,textured,2);glPopMatrix();
    }
    if (actRHand == "off"){
        glColor3d(1.0,1.0,1.0);
        if (actRightArmCovers == "off")
        {
            glPushMatrix();LDEsetM(dGeomGetPosition(r_hand0_geom),dGeomGetRotation(r_hand0_geom));
            DrawBox(0.022,0.069,0.065,false,textured,2);glPopMatrix();
        }else
        {
            glPushMatrix();
            LDEsetM(dGeomGetPosition(r_hand0_geom),dGeomGetRotation(r_hand0_geom));     //DRAW THE MODEL
            glTranslatef(0.0, 0.5*fabs(jP_rightArm[7][2] - jP_rightArm[6][2]), 0.0);
            DrawX( model_trimesh["rightPalm"], modelTexture[0]);
            glPopMatrix();

        }
        
        glColor3d(1.0,1.0,1.0);
        glPushMatrix(); LDEsetM(dGeomGetPosition(r_hand1_geom),dGeomGetRotation(r_hand1_geom));
        DrawCylinder(0.0065,0.08,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dGeomGetPosition(r_hand2_geom),dGeomGetRotation(r_hand2_geom));
        DrawCylinder(0.0065,0.084,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dGeomGetPosition(r_hand3_geom),dGeomGetRotation(r_hand3_geom));
        DrawCylinder(0.0065,0.08,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dGeomGetPosition(r_hand4_geom),dGeomGetRotation(r_hand4_geom));
        DrawCylinder(0.0065,0.073,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dGeomGetPosition(r_hand5_geom),dGeomGetRotation(r_hand5_geom));
        DrawCylinder(0.0065,0.064,false,textured,2);glPopMatrix();


    }else{
        if (actRightArmCovers == "off")
        {
            glPushMatrix();LDEsetM(dBodyGetPosition(body[11]),dBodyGetRotation(body[11]));
            DrawBox(0.022,0.069,0.065,false,textured,2);glPopMatrix();//Taken from ODE use Y, Z, Xs
        }else
        {
            glPushMatrix();
            LDEsetM(dBodyGetPosition(body[11]),dBodyGetRotation(body[11]));     //DRAW THE MODEL
            glTranslatef(0.0, 0.5*fabs(jP_rightArm[7][2] - jP_rightArm[6][2]), 0.0);
            DrawX( model_trimesh["rightPalm"], modelTexture[0]);
            glPopMatrix();

        }
        //RIGHT HAND FINGERS
        
        glPushMatrix(); LDEsetM(dBodyGetPosition(body[31]),dBodyGetRotation(body[31]));
        DrawCylinder(0.0065,0.012,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dBodyGetPosition(body[32]),dBodyGetRotation(body[32]));
        DrawCylinder(0.0065,0.012,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dGeomGetPosition(rhandfings0_geom),dGeomGetRotation(rhandfings0_geom));
        DrawCylinder(0.0065,0.012,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dGeomGetPosition(rhandfings1_geom),dGeomGetRotation(rhandfings1_geom));
        DrawCylinder(0.0065,0.012,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dBodyGetPosition(body[35]),dBodyGetRotation(body[35]));
        DrawCylinder(0.0065,0.026,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dBodyGetPosition(body[36]),dBodyGetRotation(body[36]));
        DrawCylinder(0.0065,0.028,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dGeomGetPosition(rhandfings2_geom),dGeomGetRotation(rhandfings2_geom));
        DrawCylinder(0.0065,0.026,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dGeomGetPosition(rhandfings3_geom),dGeomGetRotation(rhandfings3_geom));
        DrawCylinder(0.0065,0.022,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dBodyGetPosition(body[39]),dBodyGetRotation(body[39]));
        DrawCylinder(0.0065,0.022,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dBodyGetPosition(body[40]),dBodyGetRotation(body[40]));
        DrawCylinder(0.0065,0.024,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dGeomGetPosition(rhandfings4_geom),dGeomGetRotation(rhandfings4_geom));
        DrawCylinder(0.0065,0.022,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dGeomGetPosition(rhandfings5_geom),dGeomGetRotation(rhandfings5_geom));
        DrawCylinder(0.0065,0.019,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dBodyGetPosition(body[43]),dBodyGetRotation(body[43]));
        DrawCylinder(0.0065,0.02,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dBodyGetPosition(body[44]),dBodyGetRotation(body[44]));
        DrawCylinder(0.0065,0.02,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dGeomGetPosition(rhandfings6_geom),dGeomGetRotation(rhandfings6_geom));
        DrawCylinder(0.0065,0.02,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dGeomGetPosition(rhandfings7_geom),dGeomGetRotation(rhandfings7_geom));
        DrawCylinder(0.0065,0.02,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dBodyGetPosition(body[47]),dBodyGetRotation(body[47]));
        DrawCylinder(0.0065,0.026,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dBodyGetPosition(body[48]),dBodyGetRotation(body[48]));
        DrawCylinder(0.0065,0.022,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dBodyGetPosition(body[49]),dBodyGetRotation(body[49]));
        DrawCylinder(0.0065,0.016,false,textured,2);glPopMatrix();
    }
    if (actHead == "off"){
        glPushMatrix(); LDEsetM(dGeomGetPosition(neck0_geom),dGeomGetRotation(neck0_geom));
        DrawCylinder(0.015,0.077,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dGeomGetPosition(neck1_geom),dGeomGetRotation(neck1_geom));
        DrawCylinder(0.015,0.077,false,textured,2);glPopMatrix();
    }else{
        glPushMatrix(); LDEsetM(dBodyGetPosition(neck[0]),dBodyGetRotation(neck[0]));
        DrawCylinder(0.015,0.077,false,textured,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dBodyGetPosition(neck[1]),dBodyGetRotation(neck[1]));
        DrawCylinder(0.015,0.077,false,textured,2);glPopMatrix();
    }

    glPushMatrix(); LDEsetM(dGeomGetPosition(head0_geom),dGeomGetRotation(head0_geom));
    DrawCylinder(0.015,0.06,false,textured,2);glPopMatrix();

    if (actHeadCover == "on"){
        glPushMatrix(); LDEsetM(dGeomGetPosition(eye1_geom),dGeomGetRotation(head1_geom));
        glTranslatef(0.0,0.0,-fabs(jP_head[3][0]-jP_head[2][0])*1.20);
        glScalef(0.95,0.95,1);
        iCubHeadModel->draw(false,8);
        glPopMatrix();
    
        if (!eyeLids){
            eyeLids = new EyeLids;
            eyeLids->setName( eyeLidsPortName );
        } 
        glPushMatrix();LDEsetM(dGeomGetPosition(eye1_geom),dGeomGetRotation(head1_geom));
        eyeLids->checkPort();
        glRotatef(eyeLids->eyeLidsRotation,1,0,0);
        topEyeLidModel->draw(false,8); 
        glRotatef(-2*eyeLids->eyeLidsRotation,1,0,0);
        bottomEyeLidModel->draw(false,8); 
        glPopMatrix();

    }else{
        glPushMatrix(); LDEsetM(dGeomGetPosition(head1_geom),dGeomGetRotation(head1_geom));
        DrawBox(0.104, 0.002,0.052,false,textured,2);
        glPopMatrix();

        glColor3d(0.3,0.3,0.3);
        glPushMatrix(); LDEsetM(dGeomGetPosition(head2_geom),dGeomGetRotation(head2_geom));
        DrawBox(0.002, 0.093,0.052,false,false,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dGeomGetPosition(head3_geom),dGeomGetRotation(head3_geom));
        DrawBox(0.002, 0.093,0.052,false,false,2);glPopMatrix();
        //glPushMatrix(); LDEsetM(dGeomGetPosition(head4_geom),dGeomGetRotation(head4_geom));
        //DrawBox( 0.104, 0.002 ,0.032,false,false,2);glPopMatrix();

        //glPushMatrix(); LDEsetM(dGeomGetPosition(head5_geom),dGeomGetRotation(head5_geom));
        //DrawBox( 0.011, 0.026,0.025,false,false,2);glPopMatrix();
        glColor3d(0.3,0.3,0.3);
        glPushMatrix(); LDEsetM(dGeomGetPosition(head6_geom),dGeomGetRotation(head6_geom));
        DrawBox(  0.011, 0.051,0.012,false,false,2);glPopMatrix();

        glPushMatrix(); LDEsetM(dGeomGetPosition(head7_geom),dGeomGetRotation(head7_geom));
        DrawBox( 0.02, 0.022, 0.012,false,false,2);glPopMatrix();
    }
    glPushMatrix(); LDEsetM(dGeomGetPosition(eye1_geom),dGeomGetRotation(eye1_geom));
    DrawCylinder(0.002,0.068,false,true,1);glPopMatrix();

    glPushMatrix(); LDEsetM(dGeomGetPosition(eye2_geom),dGeomGetRotation(eye2_geom));
    DrawCylinder(0.006,0.030,false,true,1);glPopMatrix();

    glPushMatrix(); LDEsetM(dGeomGetPosition(eye3_geom),dGeomGetRotation(eye3_geom));
    DrawCylinder(0.006,0.05,false,true,1);glPopMatrix();

    glPushMatrix(); LDEsetM(dGeomGetPosition(eye3_geom),dGeomGetRotation(eye3_geom));
    DrawCylinder(0.006,0.05,false,true,1);glPopMatrix();

    glPushMatrix(); LDEsetM(dGeomGetPosition(eye4_geom),dGeomGetRotation(eye4_geom));
    DrawCylinder(0.006,0.030,false,true,1);glPopMatrix();

    glPushMatrix(); LDEsetM(dGeomGetPosition(eye5_geom),dGeomGetRotation(eye5_geom));
    DrawCylinder(0.006,0.05,false,true,1);glPopMatrix();

    glPushMatrix(); LDEsetM(dGeomGetPosition(Leye1_geom),dGeomGetRotation(Leye1_geom));
    glColor3d(0,0,0);
    DrawCylinder(0.006,0.04,false,false,1);
    glColor3d(1,1,1);
    DrawSphere(0.0185,false,false,0);
    glPopMatrix();

    glPushMatrix(); LDEsetM(dGeomGetPosition(Reye1_geom),dGeomGetRotation(Reye1_geom));
    glColor3d(0,0,0);
    DrawCylinder(0.006,0.04,false,false,1);
    glColor3d(1,1,1);
    DrawSphere(0.0185,false,false,0);
    glPopMatrix();

    glColor3d(1,0.49,0.14);
    glPushMatrix();LDEsetM(dBodyGetPosition(inertialBody),dBodyGetRotation(inertialBody));
    DrawBox( 0.03, 0.02, 0.05,false,false,2);
    glPopMatrix();

}
bool ICubSim::loadJointPosition(const char *joints_path){

    ifstream fin(joints_path);

    if(!fin.is_open())
    {
        fprintf(stdout,"Could not find joint.ini file!\n");
        return false;
    }

    stringstream sFin;
    sFin << fin.rdbuf();

    Bottle bJoints(sFin.str().c_str());

    /*---- legs ----*/
    Bottle &bLeftLeg=bJoints.findGroup("left_leg");
    Bottle &bRightLeg=bJoints.findGroup("right_leg");


    //the legs are inverse ordered
    for(int i=0; i<6; i++)
    {
        jP_leftLeg[5-i].resize(3);
        jP_rightLeg[5-i].resize(3);

        Bottle *leftJoint=bLeftLeg.get(i+1).asList();
        Bottle *rightJoint=bRightLeg.get(i+1).asList();

        for(int j=0; j<3; j++)
        {
            jP_leftLeg[5-i][j]=leftJoint->get(j).asDouble();
            jP_rightLeg[5-i][j]=rightJoint->get(j).asDouble();
        }
    }

    // Del Prete: read parameters from iKin (for arms and torso only)
    iCubArm iKinLeftArm("left");
    iCubArm iKinRightArm("right");
    iKinLeftArm.releaseLink(0); iKinLeftArm.releaseLink(1); iKinLeftArm.releaseLink(2);
    iKinRightArm.releaseLink(0); iKinRightArm.releaseLink(1); iKinRightArm.releaseLink(2);
    iKinLeftArm.setAllConstraints(false);   // disable joint limits
    iKinRightArm.setAllConstraints(false);
    iKinLeftArm.setAng(zeros(10));  // set all ang to zero
    iKinRightArm.setAng(zeros(10));
    Matrix Hl(4,4), Hr(4,4);  // rototranslation matrix
    Vector tl(3), tr(3), z(3);
    // rototranslation form robot root to simulation world reference frame
    Matrix H_r2w(4,4), H_w2r(4,4);
    H_r2w(0,2) = -1.0;  H_r2w(0,3) = -0.026;
    H_r2w(1,0) = -1.0;
    H_r2w(2,1) =  1.0;  H_r2w(2,3) = -0.5976; //-elev;
    H_r2w(3,3) = 1.0;
    H_w2r = SE3inv(H_r2w);
    
    /*---- torso ----*/
    for(int i=0; i<3; i++)
    {
        jP_torso[i].resize(3);
        if(i==0)
            Hl = H_w2r * iKinLeftArm.getH0();
        else
            Hl = H_w2r * iKinLeftArm.getH(i-1);
        jP_torso[i][0] = Hl(2,3);
        jP_torso[i][1] = Hl(0,3);
        jP_torso[i][2] = Hl(1,3);
        jA_torso[i] = Hl.subcol(0,2,3);
        if(i>0)
            jA_torso[i] = -1.0*jA_torso[i];
    }

    /*---- arms ----*/
    for(int i=0; i<8; i++)
    {
        jP_leftArm[i].resize(3);
        Hl = H_w2r*iKinLeftArm.getH(i+2);
        jP_leftArm[i][0] = Hl(2,3);
        jP_leftArm[i][1] = Hl(0,3);
        jP_leftArm[i][2] = Hl(1,3);
        jA_leftArm[i] = Hl.subcol(0,2,3);
        if(i>1)
            jA_leftArm[i] = -1.0*jA_leftArm[i];

        jP_rightArm[i].resize(3);
        Hr = H_w2r*iKinRightArm.getH(i+2);
        jP_rightArm[i][0] = Hr(2,3);
        jP_rightArm[i][1] = Hr(0,3);
        jP_rightArm[i][2] = Hr(1,3);
        jA_rightArm[i] = Hr.subcol(0,2,3);
        if(i==1 || i==3)
            jA_rightArm[i] = -1.0*jA_rightArm[i];
    }

    /*---- head ----*/
    Bottle &bHead=bJoints.findGroup("head");

    for(int i=0; i<4; i++)
    {
        jP_head[i].resize(3);

        Bottle *headJoint=bHead.get(i+1).asList();

        for(int j=0; j<3; j++)
            jP_head[i][j]=headJoint->get(j).asDouble();
    }

    /*---- eyes ----*/
    Bottle &bLeftEye=bJoints.findGroup("left_eye");
    Bottle &bRightEye=bJoints.findGroup("right_eye");

    for(int i=0; i<2; i++)
    {
        jP_leftEye[i].resize(3);
        jP_rightEye[i].resize(3);

        Bottle *leftJoint=bLeftEye.get(i+1).asList();
        Bottle *rightJoint=bRightEye.get(i+1).asList();

        for(int j=0; j<3; j++)
        {
            jP_leftEye[i][j]=leftJoint->get(j).asDouble();
            jP_rightEye[i][j]=rightJoint->get(j).asDouble();
        }
    }
     /*---- inertial ----*/
    Bottle &bInertial=bJoints.findGroup("inertial");
    Bottle *inertialJP = bInertial.get(1).asList();
    jP_inertial.resize(3);

    for(int j=0; j<3; j++)
    {
        jP_inertial[j] = inertialJP->get(j).asDouble();
    }
    return true;
}

void ICubSim::setPosition(dReal agentX, dReal agentY, dReal agentZ ) {
    //Coordinates X Y Z using the 3D Cartesian coordinate system
    //dGeomSetPosition(geom_cube[0],0.0,0.05, 2); //reference on the z 

    if (actScreen == "on")
        dGeomSetPosition(screenGeom,0.0,1.0,0.7);

    if (actLegs == "off"){
        //starting from the left foot
        dBodySetPosition (legs,   jP_leftLeg[0][1], 0.0, jP_leftLeg[0][0]);
    }else{
        //left lower body part
        dBodySetPosition(leftLeg[0], jP_leftLeg[0][1], elev + 0.0021, jP_leftLeg[0][0]); //dBodySetPosition(leftLeg[0], 0.068, elev + 0.0021, 0.0); //FROM ODE Y, Z, X    
        dBodySetPosition(leftLeg[1], jP_leftLeg[1][1], elev + jP_leftLeg[1][2], jP_leftLeg[1][0]);//dBodySetPosition(leftLeg[1], 0.068, elev + 0.031, -0.0235);
        dBodySetPosition(leftLeg[2], jP_leftLeg[1][1], elev + jP_leftLeg[1][2], jP_leftLeg[1][0]);//dBodySetPosition(leftLeg[2], 0.068, elev + 0.031, -0.034);
        dBodySetPosition(leftLeg[3], jP_leftLeg[2][1], elev + jP_leftLeg[2][2], jP_leftLeg[2][0]);//dBodySetPosition(leftLeg[3], 0.068, elev + 0.244, -0.034);
        dBodySetPosition(leftLeg[4], jP_leftLeg[4][1], elev + jP_leftLeg[4][2], jP_leftLeg[4][0]);//dBodySetPosition(leftLeg[4], 0.068, elev + 0.468, -0.034);
        dBodySetPosition(leftLeg[5],0.0295, elev + jP_leftLeg[4][2], jP_leftLeg[4][0]);//dBodySetPosition(leftLeg[5], 0.0295, elev + 0.468, -0.034);
        //right lower body part
        dBodySetPosition(rightLeg[0], jP_rightLeg[0][1], elev + 0.0021, jP_rightLeg[0][0]); //dBodySetPosition(rightLeg[0], -0.068, elev + 0.0021, 0.0); 
        dBodySetPosition(rightLeg[1], jP_rightLeg[1][1], elev + jP_rightLeg[1][2], jP_rightLeg[1][0]);//dBodySetPosition(rightLeg[1], -0.068, elev + 0.031, -0.0235);
        dBodySetPosition(rightLeg[2], jP_rightLeg[1][1], elev + jP_rightLeg[1][2], jP_rightLeg[1][0]);//dBodySetPosition(rightLeg[2], -0.068, elev + 0.031, -0.034);
        dBodySetPosition(rightLeg[3], jP_rightLeg[2][1], elev + jP_rightLeg[2][2], jP_rightLeg[2][0]);//dBodySetPosition(rightLeg[3], -0.068, elev + 0.244, -0.034);
        dBodySetPosition(rightLeg[4], jP_rightLeg[4][1], elev + jP_rightLeg[4][2], jP_rightLeg[4][0]);//dBodySetPosition(rightLeg[4], -0.068, elev + 0.468, -0.034);
        dBodySetPosition(rightLeg[5],-0.0295, elev + jP_leftLeg[4][2], jP_leftLeg[4][0]);//dBodySetPosition(rightLeg[5], -0.0295, elev + 0.468, -0.034);
    }
    if (actTorso == "off"){
        dBodySetPosition (body_torso,   jP_torso[0][1], elev + 0.5*fabs((jP_torso[0][2]-0.031/*torso pitch cyl rad*/)+(jP_leftLeg[5][2]-0.031/*leg cyl rad*/))/*0.4912*/, jP_torso[0][0]);
        dBodySetPosition (torso[4],   0.5*(jP_leftArm[1][1]+jP_torso[2][1]-0.011-0.5*0.059 /*clavicule height + half shoulder height*/)/*prev=0.038*/, elev +0.5*fabs((jP_head[0][2]-0.015/*head pitch cyl rad*/)+(jP_torso[2][2]+0.031+0.0274/*torso roll cyl rad + torso yaw cyl height*/))/*prev= 0.7414*/, jP_torso[2][0]);
        dBodySetPosition (torso[5],   0.5*(jP_rightArm[1][1]+jP_torso[2][1]+0.011+0.5*0.059/*clavicule height + half shoulder height*/)/*prev=-0.038*/, elev +0.5*fabs((jP_head[0][2]-0.015/*head pitch cyl rad*/)+(jP_torso[2][2]+0.031+0.0274/*torso roll cyl rad + torso yaw cyl height*/))/*prev= 0.7414*/, jP_torso[2][0]);
    }else{
        dBodySetPosition (torso[0],   jP_torso[0][1], elev + 0.5*fabs((jP_torso[0][2]-0.031/*torso pitch cyl rad*/)+(jP_leftLeg[5][2]-0.031/*leg cyl rad*/))/*0.4912*/, jP_torso[0][0]);
        dBodySetPosition (torso[1],   jP_torso[0][1], elev +jP_torso[0][2]/*0.557*/, jP_torso[0][0]);
        dBodySetPosition (torso[2],   jP_torso[1][1], elev +jP_torso[1][2]/*0.557*/, jP_torso[1][0]); 
        dBodySetPosition (torso[3],   jP_torso[2][1], elev +jP_torso[2][2]+0.031+0.0274*0.5/*torso roll cyl rad + torso yaw cyl half height*/, jP_torso[2][0]); //0.0, elev +0.6687, -0.026);
        dBodySetPosition (torso[4],   0.5*(jP_leftArm[1][1]+jP_torso[2][1]-0.011-0.5*0.059 /*clavicule height + half shoulder height*/)/*prev=0.038*/, elev +0.5*fabs((jP_head[0][2]-0.015/*head pitch cyl rad*/)+(jP_torso[2][2]+0.031+0.0274/*torso roll cyl rad + torso yaw cyl height*/))/*prev= 0.7414*/, jP_torso[2][0]);
        dBodySetPosition (torso[5],   0.5*(jP_rightArm[1][1]+jP_torso[2][1]+0.011+0.5*0.059/*clavicule height + half shoulder height*/)/*prev=-0.038*/, elev +0.5*fabs((jP_head[0][2]-0.015/*head pitch cyl rad*/)+(jP_torso[2][2]+0.031+0.0274/*torso roll cyl rad + torso yaw cyl height*/))/*prev= 0.7414*/, jP_torso[2][0]);
    }
    if (actLArm == "off"){
        dBodySetPosition (larm,   jP_leftArm[1][1]-0.5*(0.011+0.059) /*half clavicule height + half shoulder height*//*prev=0.0815*/, elev +jP_leftArm[1][2], jP_leftArm[1][0]);
    }else{
        //left arm
        dBodySetPosition (body[0],   jP_leftArm[1][1]-0.5*(0.011+0.059) /*half clavicule height + half shoulder height*//*prev=0.0815*/, elev +jP_leftArm[1][2], jP_leftArm[1][0]);
        dBodySetPosition (body[2],   jP_leftArm[2][1], elev + jP_leftArm[2][2], jP_leftArm[2][0]);
        dBodySetPosition (body[4],   jP_leftArm[4][1], elev + 0.5*(jP_leftArm[4][2]+jP_leftArm[2][2]), jP_leftArm[4][0]);
        dBodySetPosition (body[6],   jP_leftArm[3][1], elev + jP_leftArm[3][2], jP_leftArm[3][0]);
        dBodySetPosition (body[8],   jP_leftArm[5][1], elev + 0.5*(jP_leftArm[5][2]+jP_leftArm[3][2]), jP_leftArm[5][0]);
    }
    if (actRArm == "off"){
        dBodySetPosition (rarm,   jP_rightArm[1][1]+0.5*(0.011+0.059) /*half clavicule height + half shoulder height*//*prev=0.0815*/, elev +jP_rightArm[1][2], jP_rightArm[1][0]);
    }else{
        ////right arm
        dBodySetPosition (body[1],   jP_rightArm[1][1]+0.5*(0.011+0.059) /*half clavicule height + half shoulder height*//*prev=0.0815*/, elev +jP_rightArm[1][2], jP_rightArm[1][0]);
        dBodySetPosition (body[3],   jP_rightArm[2][1], elev + jP_rightArm[2][2], jP_rightArm[2][0]);
        dBodySetPosition (body[5],   jP_rightArm[4][1], elev + 0.5*(jP_rightArm[4][2]+jP_rightArm[2][2]), jP_rightArm[4][0]);
        dBodySetPosition (body[7],   jP_rightArm[3][1], elev + jP_rightArm[3][2], jP_rightArm[3][0]);
        dBodySetPosition (body[9],   jP_rightArm[5][1], elev + 0.5*(jP_rightArm[5][2]+jP_rightArm[3][2]), jP_rightArm[5][0]);
    }
    if (actLHand == "off"){
        dBodySetPosition(l_hand,   jP_leftArm[7][1], elev + jP_leftArm[6][2]-0.5*fabs(jP_leftArm[7][2]-jP_leftArm[6][2]),jP_leftArm[7][0]);
    }else{
        //left hand fingers
        dBodySetPosition (body[10],   jP_leftArm[7][1], elev + jP_leftArm[6][2]-0.5*fabs(jP_leftArm[7][2]-jP_leftArm[6][2]),jP_leftArm[7][0]);
        dBodySetPosition (body[12],   jP_leftArm[7][1], elev +jP_leftArm[6][2]-0.5*fabs(jP_leftArm[7][2]-jP_leftArm[6][2])-0.04, jP_leftArm[7][0]+0.025);
        dBodySetPosition (body[13],   jP_leftArm[7][1], elev +jP_leftArm[6][2]-0.5*fabs(jP_leftArm[7][2]-jP_leftArm[6][2])-0.04, jP_leftArm[7][0]+0.01);
        dBodySetPosition(lhandfingers0,jP_leftArm[7][1],elev +jP_leftArm[6][2]-0.5*fabs(jP_leftArm[7][2]-jP_leftArm[6][2])-0.04, jP_leftArm[7][0]-0.016125);
        dBodySetPosition (body[16],   jP_leftArm[7][1], elev +jP_leftArm[6][2]-0.5*fabs(jP_leftArm[7][2]-jP_leftArm[6][2])-0.059, jP_leftArm[7][0]+0.025);
        dBodySetPosition (body[17],   jP_leftArm[7][1], elev +jP_leftArm[6][2]-0.5*fabs(jP_leftArm[7][2]-jP_leftArm[6][2])-0.060, jP_leftArm[7][0]+0.01);
        dBodySetPosition(lhandfingers1,jP_leftArm[7][1],elev +jP_leftArm[6][2]-0.5*fabs(jP_leftArm[7][2]-jP_leftArm[6][2])-0.059, jP_leftArm[7][0]-0.016125);
        dBodySetPosition (body[20],   jP_leftArm[7][1], elev +jP_leftArm[6][2]-0.5*fabs(jP_leftArm[7][2]-jP_leftArm[6][2])-0.083, jP_leftArm[7][0]+0.025);
        dBodySetPosition (body[21],   jP_leftArm[7][1], elev +jP_leftArm[6][2]-0.5*fabs(jP_leftArm[7][2]-jP_leftArm[6][2])-0.086, jP_leftArm[7][0]+0.01);
        dBodySetPosition(lhandfingers2,jP_leftArm[7][1], elev +jP_leftArm[6][2]-0.5*fabs(jP_leftArm[7][2]-jP_leftArm[6][2])-0.083, jP_leftArm[7][0]-0.016125);
        dBodySetPosition (body[24],   jP_leftArm[7][1], elev +jP_leftArm[6][2]-0.5*fabs(jP_leftArm[7][2]-jP_leftArm[6][2])-0.104, jP_leftArm[7][0]+0.025);
        dBodySetPosition (body[25],   jP_leftArm[7][1], elev +jP_leftArm[6][2]-0.5*fabs(jP_leftArm[7][2]-jP_leftArm[6][2])-0.108, jP_leftArm[7][0]+0.01);
        dBodySetPosition(lhandfingers3,jP_leftArm[7][1], elev +jP_leftArm[6][2]-0.5*fabs(jP_leftArm[7][2]-jP_leftArm[6][2])-0.104, jP_leftArm[7][0]-0.016125);
        dBodySetPosition (body[28] , jP_leftArm[7][1], elev+ jP_leftArm[6][2]-0.045/*fabs(jP_leftArm[7][2]-jP_leftArm[6][2])+0.016*/,jP_leftArm[7][0]+0.045); //left thumb1 
        dBodySetPosition (body[29] , jP_leftArm[7][1], elev +jP_leftArm[6][2]-0.045/*fabs(jP_leftArm[7][2]-jP_leftArm[6][2])+0.016*/,jP_leftArm[7][0]+0.069); //left thumb2 
        dBodySetPosition (body[30] , jP_leftArm[7][1], elev +jP_leftArm[6][2]-0.045/*fabs(jP_leftArm[7][2]-jP_leftArm[6][2])+0.016*/,jP_leftArm[7][0]+0.088); //left thumb3
    }
    if (actRHand == "off"){
        dBodySetPosition(r_hand,   jP_rightArm[7][1], elev + jP_rightArm[6][2]-0.5*fabs(jP_rightArm[7][2]-jP_rightArm[6][2]),jP_rightArm[7][0]);
    }else{
        //right hand fingers
        dBodySetPosition (body[11],   jP_rightArm[7][1], elev + jP_rightArm[6][2]-0.5*fabs(jP_rightArm[7][2]-jP_rightArm[6][2]),jP_rightArm[7][0]);
        dBodySetPosition (body[31],   jP_rightArm[7][1], elev +jP_rightArm[6][2]-0.5*fabs(jP_rightArm[7][2]-jP_rightArm[6][2])-0.04, jP_rightArm[7][0]+0.025);
        dBodySetPosition (body[32],   jP_rightArm[7][1], elev +jP_rightArm[6][2]-0.5*fabs(jP_rightArm[7][2]-jP_rightArm[6][2])-0.04, jP_rightArm[7][0]+0.01);
        dBodySetPosition(rhandfingers0,jP_rightArm[7][1],elev +jP_rightArm[6][2]-0.5*fabs(jP_rightArm[7][2]-jP_rightArm[6][2])-0.04, jP_rightArm[7][0]-0.016125);
        dBodySetPosition (body[35],   jP_rightArm[7][1], elev +jP_rightArm[6][2]-0.5*fabs(jP_rightArm[7][2]-jP_rightArm[6][2])-0.059, jP_rightArm[7][0]+0.025);
        dBodySetPosition (body[36],   jP_rightArm[7][1], elev+jP_rightArm[6][2]-0.5*fabs(jP_rightArm[7][2]-jP_rightArm[6][2])-0.060, jP_rightArm[7][0]+0.01);
        dBodySetPosition(rhandfingers1,jP_rightArm[7][1],elev +jP_rightArm[6][2]-0.5*fabs(jP_rightArm[7][2]-jP_rightArm[6][2])-0.059, jP_rightArm[7][0]-0.016125);
        dBodySetPosition (body[39],   jP_rightArm[7][1], elev +jP_rightArm[6][2]-0.5*fabs(jP_rightArm[7][2]-jP_rightArm[6][2])-0.083, jP_rightArm[7][0]+0.025);
        dBodySetPosition (body[40],   jP_rightArm[7][1], elev +jP_rightArm[6][2]-0.5*fabs(jP_rightArm[7][2]-jP_rightArm[6][2])-0.086, jP_rightArm[7][0]+0.01);
        dBodySetPosition(rhandfingers2,jP_rightArm[7][1], elev +jP_rightArm[6][2]-0.5*fabs(jP_rightArm[7][2]-jP_rightArm[6][2])-0.083, jP_rightArm[7][0]-0.016125);
        dBodySetPosition (body[43],   jP_rightArm[7][1], elev +jP_rightArm[6][2]-0.5*fabs(jP_rightArm[7][2]-jP_rightArm[6][2])-0.104, jP_rightArm[7][0]+0.025);
        dBodySetPosition (body[44],   jP_rightArm[7][1], elev +jP_rightArm[6][2]-0.5*fabs(jP_rightArm[7][2]-jP_rightArm[6][2])-0.108, jP_rightArm[7][0]+0.01);
        dBodySetPosition(rhandfingers3,jP_rightArm[7][1], elev+ jP_rightArm[6][2]-0.5*fabs(jP_rightArm[7][2]-jP_rightArm[6][2])-0.104, jP_rightArm[7][0]-0.016125);
        dBodySetPosition (body[47] , jP_rightArm[7][1], elev +jP_rightArm[6][2]-0.045/*fabs(jP_rightArm[7][2]-jP_rightArm[6][2])+0.016*/,jP_rightArm[7][0]+0.045); //right thumb1 
        dBodySetPosition (body[48] , jP_rightArm[7][1], elev+ jP_rightArm[6][2]-0.045/*fabs(jP_rightArm[7][2]-jP_rightArm[6][2])+0.016*/,jP_rightArm[7][0]+0.069); //right thumb2 
        dBodySetPosition (body[49] , jP_rightArm[7][1], elev +jP_rightArm[6][2]-0.045/*fabs(jP_rightArm[7][2]-jP_rightArm[6][2])+0.016*/,jP_leftArm[7][0]+0.088); //right thumb3
    }
    if (actHead == "off"){
        dBodySetPosition(head, jP_head[0][1], elev +jP_head[2][2]+ 0.03, jP_head[0][0]);
    }else{
        dBodySetPosition (neck[0], jP_head[0][1], elev +jP_head[0][2], jP_head[0][0]);//dBodySetPosition (neck[0], -0.0, elev +0.815, -0.026);
        dBodySetPosition (neck[1], jP_head[1][1], elev +jP_head[1][2], jP_head[1][0]);//dBodySetPosition (neck[1], -0.0, elev +0.845, -0.026);
        dBodySetPosition (head,    jP_head[2][1], elev +jP_head[2][2] + 0.03/*half neck yaw*/, jP_head[2][0]);//dBodySetPosition (head, -0.0, elev +0.89, -0.026);
        dBodySetPosition (eye,     jP_head[3][1], elev +jP_head[3][2], jP_head[3][0]);//dBodySetPosition (eye, -0.0, elev +0.89,  -0.026);
        dBodySetPosition (leye,    jP_leftEye[1][1], elev + jP_leftEye[1][2], jP_leftEye[1][0]);//dBodySetPosition (leye, -0.0, elev +0.89, -0.026);
        dBodySetPosition (reye,    jP_rightEye[1][1], elev + jP_rightEye[1][2], jP_rightEye[1][0]);//dBodySetPosition (reye, -0.0, elev +0.89, -0.026);
    }
    // eyelids position
    dBodySetPosition (topEyeLid, 0.0, elev + jP_head[3][2], 0.035);
    dBodySetPosition (bottomEyeLid, 0.0, elev + jP_head[3][2], 0.035);

    dBodySetPosition (inertialBody, jP_inertial[1], elev + jP_inertial[2], jP_inertial[0]);
}

#define FLAGIFY(flags,name) name = flags.name?"on":"off"

void ICubSim::activateiCubParts(RobotConfig& config) {

    ConstString general = config.getFinder().findFile("general");

    Property options;
    options.fromConfigFile(general.c_str());

    RobotFlags& flags = config.getFlags();

    config.setFlags();
    FLAGIFY(flags,actScreen);
    FLAGIFY(flags,actElevation);
    FLAGIFY(flags,actLegs);
    FLAGIFY(flags,actTorso);
    FLAGIFY(flags,actLArm);
    FLAGIFY(flags,actRArm);
    FLAGIFY(flags,actLHand);
    FLAGIFY(flags,actRHand);
    FLAGIFY(flags,actHead);
    FLAGIFY(flags,actfixedHip);
    FLAGIFY(flags,actVision);
    FLAGIFY(flags,actPressure);
    FLAGIFY(flags,actWorld);
    FLAGIFY(flags,actHeadCover);
    FLAGIFY(flags,actLegsCovers);
    FLAGIFY(flags,actLeftArmCovers);
    FLAGIFY(flags,actRightArmCovers);
    FLAGIFY(flags,actTorsoCovers);
    FLAGIFY(flags, actStartHomePos);

    if (actElevation == "off"){
        elev = 0;
    }else {
        elev = 0.2; 
    }
    modelTextureIndex = 59;
}

void ICubSim::init( dWorldID world, dSpaceID space, dReal X, dReal Y, dReal Z, RobotConfig& config) 
{
    ResourceFinder& finder = config.getFinder();
    activateiCubParts(config);

    for (int x =0; x<100; x++)
        torqueData[x] = 0.0;

    //load joint positions from file and from iKin
    loadJointPosition(finder.findFile(finder.find("joints").asString().c_str()).c_str());

    // --- READ ODE CONFIGURATION PARAMETERS FROM CONFIG FILE ---
    OdeParams odeParameters = config.getOdeParameters();
        
    //init
    iCub = dSimpleSpaceCreate(space);
    dSpaceSetCleanup(iCub,0);

    initCovers(finder);

    if (actScreen == "on")
        screenGeom = dCreateBox(space,1.0,1.0,0.01); //this will create a thin screen of width 1m height 1m and depth 0.01m

    if (actLegs == "off"){
        initLegsOff(world); //here we create legs as one body
    }else{
        initLegsOn(world);
    }
    if (actTorso == "off"){
        initTorsoOff(world);
    }else{
        initTorsoOn(world);
    }
    if (actLArm == "off"){
        initLeftArmOff(world);
    }else{
        initLeftArmOn(world);
    }
    if (actRArm == "off"){
        initRightArmOff(world);
    }else{
        initRightArmOn(world);
    }
    if (actLHand == "off"){
        initLeftHandOff(world);
    }else{
        initLeftHandOn(world);
    }

    if (actRHand == "off"){
        initRightHandOff(world);
    }else{
        initRightHandOn(world);
    }

    initHead(world);

    initEyes(world);

    // inertial sensor box
    dMass m;
    inertialBody = dBodyCreate (world);dMassSetZero(&m);dMassSetBoxTotal(&m,0.0001,0.03,0.02,0.05);
    dBodySetMass(inertialBody,&m);
    inertialGeom = dCreateBox (iCub,0.03,0.02,0.05);dGeomSetBody (inertialGeom,inertialBody);

    setPosition( X, Y, Z);

    if (actElevation == "on")
    {
        elevJoint = dJointCreateFixed(world, 0);
        if (actTorso == "off"){
            dJointAttach (elevJoint,body_torso,0 );dJointSetFixed(elevJoint);
        }else{
            //dJointAttach (elevJoint,torso[0],0 );dJointSetFixed(elevJoint);
        }
    }
    //joints initialization
    for (int x=0; x<6; x++){
        LLegjoints[x] = dJointCreateHinge(world, 0);
        RLegjoints[x] = dJointCreateHinge(world, 0);
    }
    for (int x=0; x<5; x++){
        Torsojoints[x] = dJointCreateHinge(world,0);
    }
    for (int x=0; x<5; x++){
        LAjoints[x] = dJointCreateHinge(world,0);
        RAjoints[x] = dJointCreateHinge(world,0);
    }
    for(int x = 5; x < 6; x++) {
        LAjoints[x] = dJointCreateUniversal(world, 0);
        RAjoints[x] = dJointCreateUniversal(world, 0);
    }
    for(int x = 6; x < 22; x++) {
        LAjoints[x] = dJointCreateHinge(world, 0);
        RAjoints[x] = dJointCreateHinge(world, 0);
    }
    for(int x = 22; x < 23; x++) {
        LAjoints[x] = dJointCreateUniversal(world, 0);
        RAjoints[x] = dJointCreateUniversal(world, 0);
    }
    for(int x = 23; x < 25; x++) {
        LAjoints[x] = dJointCreateHinge(world, 0);
        RAjoints[x] = dJointCreateHinge(world, 0);
    }
    for(int x = 0; x <6; x++) {Hjoints[x] = dJointCreateHinge(world, 0);}

    initLegJoints();
    initTorsoJoints(odeParameters);

    initLeftArmJoints(odeParameters);
    initLeftHandJoints();

    initRightArmJoints(odeParameters);
    initRightHandJoints();
    
    initHeadJoints();
    
    //joint parameters
    for (int x=0; x<6; x++){
        //dJointSetHingeParam(LLegjoints[x], dParamVel, LLeg_speed[x]);// Desired motor velocity (this will be an angular or linear velocity).
        dJointSetHingeParam(LLegjoints[x], dParamFMax, odeParameters.motorMaxTorque);     //The maximum force or torque that the motor will use to achieve//the desired velocity.

        dJointSetHingeParam(RLegjoints[x], dParamVel, RLeg_speed[x]);// Desired motor velocity (this will be an angular or linear velocity).
        dJointSetHingeParam(RLegjoints[x], dParamFMax,odeParameters.motorMaxTorque);     //The maximum force or torque that the motor will use to achieve//the desired velocity.
    }
    for (int x=0; x<5; x++){
        dJointSetHingeParam(Torsojoints[x], dParamVel, Torso_speed[x]);
        dJointSetHingeParam(Torsojoints[x], dParamFMax,odeParameters.motorMaxTorque);
    }
    for (int x=0; x<5; x++){
        dJointSetHingeParam(LAjoints[x], dParamVel, la_speed[x]);
        dJointSetHingeParam(LAjoints[x], dParamFMax,odeParameters.motorMaxTorque);
        dJointSetHingeParam(RAjoints[x], dParamVel, ra_speed[x]);
        dJointSetHingeParam(RAjoints[x], dParamFMax,odeParameters.motorMaxTorque);
    }
    for (int x=5; x<6;x++){//for the hands
        dJointSetUniversalParam(LAjoints[x], dParamVel, la_speed[x]);
        dJointSetUniversalParam(LAjoints[x], dParamFMax,odeParameters.motorMaxTorque);
        dJointSetUniversalParam(LAjoints[x], dParamVel2, la_speed1[x]);
        dJointSetUniversalParam(LAjoints[x], dParamFMax2,odeParameters.motorMaxTorque);

        dJointSetUniversalParam(RAjoints[x], dParamVel, ra_speed[x]);
        dJointSetUniversalParam(RAjoints[x], dParamFMax,odeParameters.motorMaxTorque);
        dJointSetUniversalParam(RAjoints[x], dParamVel2, ra_speed1[x]);
        dJointSetUniversalParam(RAjoints[x], dParamFMax2,odeParameters.motorMaxTorque);
    }
    for (int x=6; x<25; x++){//22
        if (x!=9 && x!=13 && x!=17 && x!=21 && x!=22){
            dJointSetHingeParam(LAjoints[x], dParamVel, la_speed[x]);
            dJointSetHingeParam(LAjoints[x], dParamFMax,odeParameters.motorMaxTorque);
            dJointSetHingeParam(RAjoints[x], dParamVel, ra_speed[x]);
            dJointSetHingeParam(RAjoints[x], dParamFMax,odeParameters.motorMaxTorque);
        }
    }
    for (int x=22; x<23;x++){//for the hands
        dJointSetUniversalParam(LAjoints[x], dParamVel, la_speed[x]);
        dJointSetUniversalParam(LAjoints[x], dParamFMax, odeParameters.motorMaxTorque);
        dJointSetUniversalParam(LAjoints[x], dParamVel2, la_speed1[x]);
        dJointSetUniversalParam(LAjoints[x], dParamFMax2, odeParameters.motorMaxTorque);

        dJointSetUniversalParam(RAjoints[x], dParamVel, ra_speed[x]);
        dJointSetUniversalParam(RAjoints[x], dParamFMax, odeParameters.motorMaxTorque);
        dJointSetUniversalParam(RAjoints[x], dParamVel2, ra_speed1[x]);
        dJointSetUniversalParam(RAjoints[x], dParamFMax2, odeParameters.motorMaxTorque);
    }

    dJointSetHingeParam(Hjoints[0], dParamVel, h_speed[0]);
    dJointSetHingeParam(Hjoints[0], dParamFMax, odeParameters.motorMaxTorque);
    if (actHead == "on" ){
        /*-------------head parameters--------------*/
        for (int x=1; x<6; x++){//Joint parameters
            dJointSetHingeParam(Hjoints[x], dParamVel, h_speed[x]);
            dJointSetHingeParam(Hjoints[x], dParamFMax, odeParameters.motorMaxTorque);
        }
    }

    /* Create a fixed hip joint */
    if (actfixedHip == "on"){// && actElevation == "off") {
        fixedHipJoint = dJointCreateFixed(world, 0);
        if (actTorso == "off") {
            // attach to the lower torso
            dJointAttach (fixedHipJoint,body_torso,0);
            // move the torso up slightly (0.03 units)???
            //dBodySetPosition (body_torso,   0.0, 0.4912, -0.034);
        } else {
            // attach to the lower torso
            dJointAttach (fixedHipJoint,torso[0],0);
            // move the torso up slightly (0.03 units)???
            //dBodySetPosition (torso[0],   0.0, 0.4912, -0.034);
        }
        // this call fixes the joint to its current position in 3D space
        dJointSetFixed (fixedHipJoint);
    }
    inertialJoint = dJointCreateFixed(world, 0);
    dJointAttach (inertialJoint,inertialBody,head);
    dJointSetFixed (inertialJoint);
}

void ICubSim::initLegsOff(dWorldID world)
{
    dMass m, m2;
    dMatrix3 Rtx;

    legs = dBodyCreate (world);dMassSetZero(&m);dMassSetZero(&m2);
    l_leg0_geom = dCreateBox (iCub,0.054,0.004,0.13);dMassSetBoxTotal(&m2,0.08185,0.054,0.004,0.13);
    dGeomSetBody (l_leg0_geom,legs);
    dGeomSetOffsetPosition(l_leg0_geom,-m2.c[0], 0.0021-m2.c[0], -m2.c[0]);
    dMassAdd (&m, &m2);

    l_leg1_geom = dCreateCylinder (iCub,0.027,0.095);dMassSetCylinderTotal(&m2,0.59285,3,0.027,0.095);
    dGeomSetBody (l_leg1_geom,legs);
    dGeomSetOffsetPosition(l_leg1_geom,-m2.c[0], jP_leftLeg[1][2]-m2.c[0], -m2.c[0]);
    dMassAdd (&m, &m2);

    l_leg2_geom = dCreateCylinder (iCub,0.0245,0.063);dMassSetCylinderTotal(&m2,0.14801,3,0.0245,0.063);
    dRFromAxisAndAngle(Rtx,0,1,0,M_PI * 0.5);
    dGeomSetBody (l_leg2_geom,legs);
    dGeomSetOffsetRotation(l_leg2_geom,Rtx);
    dGeomSetOffsetPosition(l_leg2_geom,-m2.c[0], jP_leftLeg[1][2]-m2.c[0], -m2.c[0]);
    dMassAdd (&m, &m2);

    l_leg3_geom = dCreateCylinder (iCub,0.0315,0.213);dMassSetCylinderTotal(&m2,0.95262,3,0.0315,0.213);
    dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetBody (l_leg3_geom,legs);
    dGeomSetOffsetRotation(l_leg3_geom,Rtx);
    dGeomSetOffsetPosition(l_leg3_geom,-m2.c[0], 0.5*fabs(jP_leftLeg[2][2]+jP_leftLeg[1][2])-m2.c[0], -m2.c[0]);
    dMassAdd (&m, &m2);

    l_leg4_geom = dCreateCylinder (iCub,0.0315,0.077);dMassSetCylinderTotal(&m2,0.79206,3,0.0315,0.077);
    dRFromAxisAndAngle(Rtx,0,1,0,M_PI * 0.5);
    dGeomSetBody (l_leg4_geom,legs);
    dGeomSetOffsetRotation(l_leg4_geom,Rtx);
    dGeomSetOffsetPosition(l_leg4_geom,-m2.c[0], jP_leftLeg[2][2]-m2.c[0], -m2.c[0]);
    dMassAdd (&m, &m2);

    l_leg5_geom = dCreateCylinder (iCub,0.034,0.224);dMassSetCylinderTotal(&m2,1.5304,3,0.034,0.224);
    dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetBody (l_leg5_geom,legs);
    dGeomSetOffsetRotation(l_leg5_geom,Rtx);
    dGeomSetOffsetPosition(l_leg5_geom,-m2.c[0], 0.5*fabs(jP_leftLeg[3][2]+jP_leftLeg[2][2])-m2.c[0],-m2.c[0]);

    l_leg6_geom = dCreateCylinder (iCub,0.031,0.075);dMassSetCylinderTotal(&m2,1.5304,3,0.031,0.075);
    dGeomSetBody (l_leg6_geom,legs);
    dGeomSetOffsetPosition(l_leg6_geom,-m2.c[0], jP_leftLeg[4][2]-m2.c[0], -m2.c[0]);
    dMassAdd (&m, &m2);

    l_leg7_geom = dCreateCylinder (iCub,0.038,0.013);dMassSetCylinderTotal(&m2,0.32708,3,0.038,0.013);
    dRFromAxisAndAngle(Rtx,0,1,0,M_PI * 0.5);
    dGeomSetBody (l_leg7_geom,legs);
    dGeomSetOffsetRotation(l_leg7_geom,Rtx);
    dGeomSetOffsetPosition(l_leg7_geom,-fabs(jP_leftLeg[0][1]-0.0295)-m2.c[0], jP_leftLeg[4][2]-m2.c[0],-m2.c[0]);
    dMassAdd (&m, &m2);

    double r_foot_y=fabs(jP_leftLeg[0][1]-jP_rightLeg[0][1]);

    r_leg0_geom = dCreateBox (iCub,0.054,0.004,0.13);dMassSetBoxTotal(&m2,0.08185,0.054,0.004,0.13);
    dGeomSetBody (r_leg0_geom,legs);
    dGeomSetOffsetPosition(r_leg0_geom,-r_foot_y-m2.c[0], 0.0021-m2.c[0], -m2.c[0]);
    dMassAdd (&m, &m2);

    r_leg1_geom = dCreateCylinder (iCub,0.027,0.095);dMassSetCylinderTotal(&m2,0.59285,3,0.027,0.095);
    dGeomSetBody (r_leg1_geom,legs);
    dGeomSetOffsetPosition(r_leg1_geom,-r_foot_y-m2.c[0],jP_leftLeg[1][2]-m2.c[0], -m2.c[0]);
    dMassAdd (&m, &m2);

    r_leg2_geom = dCreateCylinder (iCub,0.0245,0.063);dMassSetCylinderTotal(&m2,0.14801,3,0.0245,0.063);
    dRFromAxisAndAngle(Rtx,0,1,0,M_PI * 0.5);
    dGeomSetBody (r_leg2_geom,legs);
    dGeomSetOffsetRotation(r_leg2_geom,Rtx);
    dGeomSetOffsetPosition(r_leg2_geom,-r_foot_y-m2.c[0], jP_leftLeg[1][2]-m2.c[0], -m2.c[0]);
    dMassAdd (&m, &m2);

    r_leg3_geom = dCreateCylinder (iCub,0.0315,0.213);dMassSetCylinderTotal(&m2,0.95262,3,0.0315,0.213);
    dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetBody (r_leg3_geom,legs);
    dGeomSetOffsetRotation(r_leg3_geom,Rtx);
    dGeomSetOffsetPosition(r_leg3_geom,-r_foot_y-m2.c[0], 0.5*fabs(jP_leftLeg[2][2]+jP_leftLeg[1][2])-m2.c[0], -m2.c[0]);
    dMassAdd (&m, &m2);

    r_leg4_geom = dCreateCylinder (iCub,0.0315,0.077);dMassSetCylinderTotal(&m2,0.79206,3,0.0315,0.077);
    dRFromAxisAndAngle(Rtx,0,1,0,M_PI * 0.5);
    dGeomSetBody (r_leg4_geom,legs);
    dGeomSetOffsetRotation(r_leg4_geom,Rtx);
    dGeomSetOffsetPosition(r_leg4_geom,-r_foot_y-m2.c[0], jP_rightLeg[2][2]-m2.c[0], -m2.c[0]);
    dMassAdd (&m, &m2);

    r_leg5_geom = dCreateCylinder (iCub,0.034,0.224);dMassSetCylinderTotal(&m2,1.5304,3,0.034,0.224);
    dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetBody (r_leg5_geom,legs);
    dGeomSetOffsetRotation(r_leg5_geom,Rtx);
    dGeomSetOffsetPosition(r_leg5_geom,-r_foot_y-m2.c[0],0.5*fabs(jP_leftLeg[3][2]+jP_leftLeg[2][2])-m2.c[0], -m2.c[0]);

    r_leg6_geom = dCreateCylinder (iCub,0.031,0.075);dMassSetCylinderTotal(&m2,1.5304,3,0.031,0.075);
    dGeomSetBody (r_leg6_geom,legs);
    dGeomSetOffsetPosition(r_leg6_geom,-r_foot_y-m2.c[0], jP_rightLeg[4][2]-m2.c[0], -m2.c[0]);
    dMassAdd (&m, &m2);

    r_leg7_geom = dCreateCylinder (iCub,0.038,0.013);dMassSetCylinderTotal(&m2,0.32708,3,0.038,0.013);
    dRFromAxisAndAngle(Rtx,0,1,0,M_PI * 0.5);
    dGeomSetBody (r_leg7_geom,legs);
    dGeomSetOffsetRotation(r_leg7_geom,Rtx);
    dGeomSetOffsetPosition(r_leg7_geom,fabs(jP_rightLeg[0][1]+0.0295)-r_foot_y-m2.c[0], jP_rightLeg[4][2]-m2.c[0], -m2.c[0]);
    dMassAdd (&m, &m2);

    dMassTranslate(&m,-m.c[0],-m.c[1],-m.c[2]);
    dBodySetMass(legs,&m);
}

void ICubSim::initLegsOn(dWorldID world)
{
    dMass m, m2;
    dMatrix3 Rtx;
    //rotation matrises
    dQuaternion q1;
    dQFromAxisAndAngle(q1,0,1,0,M_PI * 0.5);
    
    //left lower parts
    leftLeg[0] = dBodyCreate (world);dMassSetZero(&m);dMassSetBoxTotal(&m,0.08185,0.054,0.004,0.13); //Y, Z, X
    dBodySetMass(leftLeg[0],&m);
    leftLegGeom[0] = dCreateBox (iCub,0.054,0.004,0.13);dGeomSetBody (leftLegGeom[0],leftLeg[0]);

    leftLeg[1] = dBodyCreate (world);dMassSetZero(&m);dMassSetCylinderTotal(&m,0.59285,3,0.027,0.095);
    dBodySetMass(leftLeg[1],&m);
    leftLegGeom[1] = dCreateCylinder (iCub,0.027,0.095);dGeomSetBody(leftLegGeom[1],leftLeg[1]);

    //----------------------------------------------------------ankle encapsulated objects
    leftLeg[2] = dBodyCreate (world);dMassSetZero(&m);dMassSetZero(&m2);
    leftLeg_2_1 = dCreateCylinder (iCub,0.0245,0.063);dMassSetCylinderTotal(&m2,0.14801,3,0.0245,0.063);
    dRFromAxisAndAngle(Rtx,0,1,0,M_PI * 0.5);
    dGeomSetBody (leftLeg_2_1,leftLeg[2]);
    dGeomSetOffsetRotation(leftLeg_2_1,Rtx);//dGeomSetPosition (leftLeg_1 , 0.0, 1.0, 0.0);
    dGeomSetOffsetPosition(leftLeg_2_1,-m2.c[0], -m2.c[0], -m2.c[0]);//dMassRotate(&m2,Rtx);
    dMassAdd (&m, &m2);
    //second object
    leftLeg_2_2 = dCreateCylinder (iCub,0.0315,fabs(jP_leftLeg[2][2]-jP_leftLeg[1][2]));dMassSetCylinderTotal(&m2,0.95262,3,0.0315,0.213);
    dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetBody (leftLeg_2_2,leftLeg[2]);
    dGeomSetOffsetRotation(leftLeg_2_2,Rtx);
    dGeomSetOffsetPosition(leftLeg_2_2,-m2.c[0], 0.5*fabs(jP_leftLeg[2][2]-jP_leftLeg[1][2])-m2.c[0], -m2.c[0]);//dGeomSetPosition (leftLeg_2 , 0.0, 1, 0.0);
    //add mass accumulated
    dMassAdd (&m, &m2);
    //translate
    dMassTranslate(&m,-m.c[0],-m.c[1],-m.c[2]);
    //Set mass to the actual body
    dBodySetMass(leftLeg[2],&m);
    //----------------------------------
    leftLeg[3] = dBodyCreate (world);dMassSetZero(&m);dMassSetZero(&m2);
    leftLeg_3_1 = dCreateCylinder (iCub,0.0315,0.077);dMassSetCylinderTotal(&m2,0.79206,3,0.0315,0.077);
    dRFromAxisAndAngle(Rtx,0,1,0,M_PI * 0.5);
    dGeomSetBody (leftLeg_3_1,leftLeg[3]);
    dGeomSetOffsetRotation(leftLeg_3_1,Rtx);//dGeomSetPosition (leftLeg_1 , 0.0, 1.0, 0.0);
    dGeomSetOffsetPosition(leftLeg_3_1,-m2.c[0], -m2.c[0], -m2.c[0]);//dMassRotate(&m2,Rtx);
    dMassAdd (&m, &m2);
    //second object
    leftLeg_3_2 = dCreateCylinder (iCub,0.034,fabs(jP_leftLeg[3][2]-jP_leftLeg[2][2]));dMassSetCylinderTotal(&m2,1.5304,3,0.034,0.224);
    dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetRotation (leftLeg_3_2,Rtx);
    dGeomSetBody (leftLeg_3_2,leftLeg[3]);
    dGeomSetOffsetRotation(leftLeg_3_2,Rtx);
    dGeomSetOffsetPosition(leftLeg_3_2,-m2.c[0], 0.5*fabs(jP_leftLeg[3][2]-jP_leftLeg[2][2])/*0.1065*/-m2.c[0], -m2.c[0]);//dGeomSetPosition (leftLeg_2 , 0.0, 1, 0.0);
    //add mass accumulated
    dMassAdd (&m, &m2);
    //translate
    dMassTranslate(&m,-m.c[0],-m.c[1],-m.c[2]);
    //Set mass to the actual body
    dBodySetMass(leftLeg[3],&m);
    //---------------------------------------------------
    leftLeg[4] = dBodyCreate (world);dMassSetZero(&m);dMassSetZero(&m2);
    leftLeg_4_1 = dCreateSphere(iCub,0.017);dMassSetSphereTotal(&m,0.01,0.017);
    dGeomSetBody (leftLeg_4_1,leftLeg[4]);
    dGeomSetOffsetPosition(leftLeg_4_1,-m2.c[0], -m2.c[0], -m2.c[0]);//dMassRotate(&m2,Rtx);
    dMassAdd (&m, &m2);
    //second object
    leftLeg_4_2 = dCreateCylinder (iCub,0.031,0.075);dMassSetCylinderTotal(&m2,1.5304,3,0.031,0.075);
    dGeomSetBody (leftLeg_4_2,leftLeg[4]);
    dGeomSetOffsetPosition(leftLeg_4_2,-m2.c[0], -m2.c[0], -m2.c[0]);//dGeomSetPosition (leftLeg_2 , 0.0, 1, 0.0);
    //add mass accumulated
    dMassAdd (&m, &m2);
    //translate
    dMassTranslate(&m,-m.c[0],-m.c[1],-m.c[2]);
    //Set mass to the actual body
    dBodySetMass(leftLeg[4],&m);
    //------------------------------------------------
    leftLeg[5] = dBodyCreate (world);dMassSetZero(&m);dMassSetCylinderTotal(&m,0.32708,3,0.038,0.013);
    dBodySetMass(leftLeg[5],&m);
    dBodySetQuaternion(leftLeg[5],q1);
    leftLegGeom[5] = dCreateCylinder (iCub,0.038,0.013);dGeomSetBody(leftLegGeom[5],leftLeg[5]);
    //--------------------------------------------------------
    //RIGHT LEG
    //--------------------------------------------------------
    //right lower parts
    rightLeg[0] = dBodyCreate (world);dMassSetZero(&m);dMassSetBoxTotal(&m,0.08185,0.054,0.004,0.13); //Y, Z, X
    dBodySetMass(rightLeg[0],&m);
    rightLegGeom[0] = dCreateBox (iCub,0.054,0.004,0.13);dGeomSetBody (rightLegGeom[0],rightLeg[0]);

    rightLeg[1] = dBodyCreate (world);dMassSetZero(&m);dMassSetCylinderTotal(&m,0.59285,3,0.027,0.095);
    dBodySetMass(rightLeg[1],&m);
    rightLegGeom[1] = dCreateCylinder (iCub,0.027,0.095);dGeomSetBody(rightLegGeom[1],rightLeg[1]);
    //----------------------------------------------------------ankle encapsulated objects
    rightLeg[2] = dBodyCreate (world);dMassSetZero(&m);dMassSetZero(&m2);
    rightLeg_2_1 = dCreateCylinder (iCub,0.0245,0.063);dMassSetCylinderTotal(&m2,0.14801,3,0.0245,0.063);
    dRFromAxisAndAngle(Rtx,0,1,0,M_PI * 0.5);
    dGeomSetBody (rightLeg_2_1,rightLeg[2]);
    dGeomSetOffsetRotation(rightLeg_2_1,Rtx);//dGeomSetPosition (rightLeg_1 , 0.0, 1.0, 0.0);
    dGeomSetOffsetPosition(rightLeg_2_1,-m2.c[0], -m2.c[0], -m2.c[0]);//dMassRotate(&m2,Rtx);
    dMassAdd (&m, &m2);
    //second object
    rightLeg_2_2 = dCreateCylinder (iCub,0.0315,fabs(jP_rightLeg[2][2]-jP_rightLeg[1][2]));dMassSetCylinderTotal(&m2,0.95262,3,0.0315,0.213);
    dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetRotation (rightLeg_2_2,Rtx);
    dGeomSetBody (rightLeg_2_2,rightLeg[2]);
    dGeomSetOffsetRotation(rightLeg_2_2,Rtx);
    dGeomSetOffsetPosition(rightLeg_2_2,-m2.c[0], 0.5*fabs(jP_leftLeg[2][2]-jP_leftLeg[1][2])-m2.c[0], -m2.c[0]);//dGeomSetPosition (rightLeg_2 , 0.0, 1, 0.0);
    //add mass accumulated
    dMassAdd (&m, &m2);
    //translate
    dMassTranslate(&m,-m.c[0],-m.c[1],-m.c[2]);
    //Set mass to the actual body
    dBodySetMass(rightLeg[2],&m);
    //----------------------------------
    rightLeg[3] = dBodyCreate (world);dMassSetZero(&m);dMassSetZero(&m2);
    rightLeg_3_1 = dCreateCylinder (iCub,0.0315,0.077);dMassSetCylinderTotal(&m2,0.79206,3,0.0315,0.077);
    dRFromAxisAndAngle(Rtx,0,1,0,M_PI * 0.5);
    dGeomSetBody (rightLeg_3_1,rightLeg[3]);
    dGeomSetOffsetRotation(rightLeg_3_1,Rtx);//dGeomSetPosition (rightLeg_1 , 0.0, 1.0, 0.0);
    dGeomSetOffsetPosition(rightLeg_3_1,-m2.c[0], -m2.c[0], -m2.c[0]);//dMassRotate(&m2,Rtx);
    dMassAdd (&m, &m2);
    //second object
    rightLeg_3_2 = dCreateCylinder (iCub,0.034,fabs(jP_rightLeg[3][2]-jP_rightLeg[2][2]));dMassSetCylinderTotal(&m2,1.5304,3,0.034,0.224);
    dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetRotation (rightLeg_3_2,Rtx);
    dGeomSetBody (rightLeg_3_2,rightLeg[3]);
    dGeomSetOffsetRotation(rightLeg_3_2,Rtx);
    dGeomSetOffsetPosition(rightLeg_3_2,-m2.c[0],0.5*fabs(jP_rightLeg[3][2]-jP_rightLeg[2][2])-m2.c[0], -m2.c[0]);//dGeomSetPosition (rightLeg_2 , 0.0, 1, 0.0);
    //add mass accumulated
    dMassAdd (&m, &m2);
    //translate
    dMassTranslate(&m,-m.c[0],-m.c[1],-m.c[2]);
    //Set mass to the actual body
    dBodySetMass(rightLeg[3],&m);
    //---------------------------------------------------
    rightLeg[4] = dBodyCreate (world);dMassSetZero(&m);dMassSetZero(&m2);
    rightLeg_4_1 = dCreateSphere(iCub,0.017);dMassSetSphereTotal(&m,0.01,0.017);
    dGeomSetBody (rightLeg_4_1,rightLeg[4]);
    dGeomSetOffsetPosition(rightLeg_4_1,-m2.c[0], -m2.c[0], -m2.c[0]);//dMassRotate(&m2,Rtx);
    dMassAdd (&m, &m2);
    //second object
    rightLeg_4_2 = dCreateCylinder (iCub,0.031,0.075);dMassSetCylinderTotal(&m2,1.5304,3,0.031,0.075);
    dGeomSetBody (rightLeg_4_2,rightLeg[4]);
    dGeomSetOffsetPosition(rightLeg_4_2,-m2.c[0], -m2.c[0], -m2.c[0]);//dGeomSetPosition (rightLeg_2 , 0.0, 1, 0.0);
    //add mass accumulated
    dMassAdd (&m, &m2);
    //translate
    dMassTranslate(&m,-m.c[0],-m.c[1],-m.c[2]);
    //Set mass to the actual body
    dBodySetMass(rightLeg[4],&m);
    //------------------------------------------------
    rightLeg[5] = dBodyCreate (world);dMassSetZero(&m);dMassSetCylinderTotal(&m,0.32708,3,0.038,0.013);
    dBodySetMass(rightLeg[5],&m);
    dBodySetQuaternion(rightLeg[5],q1);
    rightLegGeom[5] = dCreateCylinder (iCub,0.038,0.013);dGeomSetBody(rightLegGeom[5],rightLeg[5]);
}

void ICubSim::initTorsoOff(dWorldID world)
{
    dMass m, m2;
    dMatrix3 Rtx;

    body_torso = dBodyCreate (world);dMassSetZero(&m);dMassSetZero(&m2);
    torso0_geom = dCreateBox (iCub,0.0470,fabs((jP_torso[0][2]-0.031/*torso pitch cyl rad*/)-(jP_leftLeg[5][2]-0.031/*leg cyl rad*/)),0.064);dMassSetBoxTotal(&m2,0.20297,0.004,0.13,0.054);
    dGeomSetBody (torso0_geom,body_torso);
    dGeomSetOffsetPosition(torso0_geom,-m2.c[0], -m2.c[0], -m2.c[0]);
    dMassAdd (&m, &m2);

    double z_offset=0.5*fabs((jP_torso[0][2]-0.031/*torso pitch cyl rad*/)+(jP_leftLeg[5][2]-0.031/*leg cyl rad*/));

    //torso1_geom = dCreateBox (iCub,0.176,0.063,0.127);dMassSetBoxTotal(&m2,3.623,0.176,0.063,0.127);
    torso1_geom = dCreateCylinder (iCub,0.031,fabs(jP_leftLeg[3][1]-jP_rightLeg[3][1]));dMassSetCylinderTotal(&m2,0.91179,3,0.031,fabs(jP_leftLeg[3][1]-jP_rightLeg[3][1]));
    //dRFromAxisAndAngle(Rtx,0,1,0,M_PI * 0.5);
    dGeomSetBody (torso1_geom,body_torso);
    //dGeomSetOffsetRotation(torso3_geom,Rtx);
    dGeomSetOffsetPosition(torso1_geom,-m2.c[0], jP_torso[0][2]-z_offset-m2.c[0], -0.006-m2.c[0]);
    dMassAdd (&m, &m2);

    torso2_geom = dCreateCylinder (iCub,0.031,0.097);dMassSetCylinderTotal(&m2,0.91179,3,0.031,0.097);
    //dRFromAxisAndAngle(Rtx,0,1,0,M_PI * 0.5);
    dGeomSetBody (torso2_geom,body_torso);
    //dGeomSetOffsetRotation(torso2_geom,Rtx);
    dGeomSetOffsetPosition(torso2_geom,jP_torso[1][1]-jP_torso[0][1]-m2.c[0], jP_torso[1][2]-z_offset-m2.c[0], -m2.c[0]);
    dMassAdd (&m, &m2);

    /*torso3_geom = dCreateCylinder (iCub,0.04,0.0274);dMassSetCylinderTotal(&m,0.45165,3,0.04,0.0274);
    dRFromAxisAndAngle(Rtx,0,1,0,M_PI * 0.5);
    dGeomSetBody (torso3_geom,body_torso);
    dGeomSetOffsetRotation(torso3_geom,Rtx);
    dGeomSetOffsetPosition(torso3_geom,-m2.c[0], 0.029-m2.c[0], -0.034-m2.c[0]);
    dMassAdd (&m, &m2);*/

    torso3_geom = dCreateCylinder (iCub,0.04,0.0274);dMassSetCylinderTotal(&m2,0.45165,3,0.04,0.0274);
    dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetBody (torso3_geom,body_torso);
    dGeomSetOffsetRotation(torso3_geom,Rtx);
    dGeomSetOffsetPosition(torso3_geom,jP_torso[2][1]-jP_torso[1][1]-m2.c[0], jP_torso[2][2]-z_offset-m2.c[0], jP_torso[1][0]-jP_torso[0][0]-m2.c[0]);
    dMassAdd (&m, &m2);

    dMassTranslate(&m,-m.c[0],-m.c[1],-m.c[2]);
    dBodySetMass(body_torso,&m);

    torso[4] = dBodyCreate (world);dMassSetZero(&m);dMassSetBoxTotal(&m,1.8388,0.076,0.118,0.109);
    dBodySetMass(torso[4],&m);
    torsoGeom[4] = dCreateBox (iCub,fabs(jP_leftArm[1][1]-jP_torso[2][1])-0.011-0.5*0.059/*clavicule height + half shoulder height*/,0.118,0.109);dGeomSetBody (torsoGeom[4],torso[4]);

    torso[5] = dBodyCreate (world);dMassSetZero(&m);dMassSetBoxTotal(&m,1.8388,0.076,0.118,0.109);
    dBodySetMass(torso[5],&m);
    torsoGeom[5] = dCreateBox (iCub,fabs(jP_leftArm[1][1]-jP_torso[2][1])-0.011-0.5*0.059/*clavicule height + half shoulder height*/,0.118,0.109);dGeomSetBody (torsoGeom[5],torso[5]);
}

void ICubSim::initTorsoOn(dWorldID world)
{
    dMass m;
    dQuaternion q, q1;
    dQFromAxisAndAngle(q,1,0,0, M_PI * 0.5);
    dQFromAxisAndAngle(q1,0,1,0,M_PI * 0.5);

    //TORSO CREATION
    torso[0] = dBodyCreate (world);dMassSetZero(&m);dMassSetBoxTotal(&m,0.20297,0.004,0.13,0.054); //Y, Z, X
    dBodySetMass(torso[0],&m);
    torsoGeom[0] = dCreateBox (iCub,0.0470,fabs((jP_torso[0][2]-0.031/*torso pitch cyl rad*/)-(jP_leftLeg[5][2]-0.031/*leg cyl rad*/)),0.064);dGeomSetBody (torsoGeom[0],torso[0]);

    torso[1] = dBodyCreate (world);dMassSetZero(&m);dMassSetCylinderTotal(&m,0.91179,3,0.031,fabs(jP_leftLeg[3][1]-jP_rightLeg[3][1]));//dMassSetBoxTotal(&m,3.623,0.176,0.063,0.127); //Y, Z, X
    dBodySetMass(torso[1],&m);
    dBodySetQuaternion(torso[1],q1);
    torsoGeom[1] = dCreateCylinder (iCub,0.031,fabs(jP_leftLeg[3][1]-jP_rightLeg[3][1]));dGeomSetBody (torsoGeom[1],torso[1]);//dCreateBox (iCub,0.176,0.063,0.127);

    torso[2] = dBodyCreate (world);dMassSetZero(&m);dMassSetCylinderTotal(&m,0.91179,3,0.031,0.097);
    dBodySetMass(torso[2],&m);
    torsoGeom[2] = dCreateCylinder (iCub,0.031,0.097);dGeomSetBody(torsoGeom[2],torso[2]);

    torso[3] = dBodyCreate (world);dMassSetZero(&m);dMassSetCylinderTotal(&m,0.45165,3,0.04,0.0274);
    dBodySetMass(torso[3],&m);
    dBodySetQuaternion(torso[3],q);
    torsoGeom[3] = dCreateCylinder (iCub,0.04,0.0274);dGeomSetBody(torsoGeom[3],torso[3]);

    torso[4] = dBodyCreate (world);dMassSetZero(&m);dMassSetBoxTotal(&m,1.8388,fabs(jP_leftArm[1][1]-jP_torso[2][1])-0.011-0.5*0.059/*clavicule height + half shoulder height*/,fabs((jP_head[0][2]-0.015/*head pitch cyl rad*/)-(jP_torso[2][2]+0.031+0.0274/*torso roll cyl rad + torso yaw cyl height*/)),0.109);
    dBodySetMass(torso[4],&m);
    torsoGeom[4] = dCreateBox (iCub,fabs(jP_leftArm[1][1]-jP_torso[2][1])-0.011-0.5*0.059/*clavicule height + half shoulder height*/,fabs((jP_head[0][2]-0.015/*head pitch cyl rad*/)-(jP_torso[2][2]+0.031+0.0274/*torso roll cyl rad + torso yaw cyl height*/)),0.109);dGeomSetBody (torsoGeom[4],torso[4]);

    torso[5] = dBodyCreate (world);dMassSetZero(&m);dMassSetBoxTotal(&m,1.8388,fabs(jP_leftArm[1][1]-jP_torso[2][1])-0.011-0.5*0.059/*clavicule height + half shoulder height*/,fabs((jP_head[0][2]-0.015/*head pitch cyl rad*/)-(jP_torso[2][2]+0.031+0.0274/*torso roll cyl rad + torso yaw cyl height*/)),0.109);
    dBodySetMass(torso[5],&m);
    torsoGeom[5] = dCreateBox (iCub,fabs(jP_leftArm[1][1]-jP_torso[2][1])-0.011-0.5*0.059/*clavicule height + half shoulder height*/,fabs((jP_head[0][2]-0.015/*head pitch cyl rad*/)-(jP_torso[2][2]+0.031+0.0274/*torso roll cyl rad + torso yaw cyl height*/)),0.109);dGeomSetBody (torsoGeom[5],torso[5]);
}

void ICubSim::initLeftArmOff(dWorldID world)
{
    dMass m, m2;
    dMatrix3 Rtx;

    double offset[3];
    offset[0]=jP_leftArm[1][0];
    offset[1]=jP_leftArm[1][1]-0.5*(0.011+0.059) /*half clavicule height + half shoulder height*/;
    offset[2]=jP_leftArm[1][2];

    larm = dBodyCreate (world);dMassSetZero(&m);dMassSetZero(&m2);
    larm0_geom = dCreateCylinder (iCub,0.031,0.011);dMassSetCylinderTotal(&m2,0.48278,3,0.031,0.011);
    dRFromAxisAndAngle(Rtx,0,1,0,M_PI * 0.5);
    dGeomSetBody (larm0_geom,larm);
    dGeomSetOffsetRotation(larm0_geom,Rtx);
    dGeomSetOffsetPosition(larm0_geom,-m2.c[0], -m2.c[0], -m2.c[0]);
    dMassAdd (&m, &m2);

    larm1_geom = dCreateCylinder (iCub,0.03,0.059);dMassSetCylinderTotal(&m2,0.20779,3,0.03,0.059);
    dGeomSetBody (larm1_geom,larm);
    dGeomSetOffsetPosition(larm1_geom,jP_leftArm[2][1]-offset[1]-m2.c[0], jP_leftArm[2][2]-offset[2]-m2.c[0], -m2.c[0]);
    dMassAdd (&m, &m2);

    larm2_geom = dCreateCylinder (iCub, 0.026 ,fabs(jP_leftArm[4][2]-jP_leftArm[2][2]));dMassSetCylinderTotal(&m2,1.1584,3,0.026,fabs(jP_leftArm[4][2]-jP_leftArm[2][2]));
    dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetBody (larm2_geom,larm);
    dGeomSetOffsetRotation(larm2_geom,Rtx);
    dGeomSetOffsetPosition(larm2_geom,jP_leftArm[4][1]-offset[1]-m2.c[0], 0.5*(jP_leftArm[4][2]+jP_leftArm[2][2])-offset[2]-m2.c[0], -m2.c[0]);
    dMassAdd (&m, &m2);

    larm3_geom = dCreateCylinder (iCub, 0.02 ,fabs(jP_leftArm[5][2]-jP_leftArm[3][2]));dMassSetCylinderTotal(&m2,0.48774,3,0.02,fabs(jP_leftArm[5][2]-jP_leftArm[3][2]));
    dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetBody (larm3_geom,larm);
    dGeomSetOffsetRotation(larm3_geom,Rtx);
    dGeomSetOffsetPosition(larm3_geom,jP_leftArm[5][1]-offset[1]-m2.c[0], 0.5*(jP_leftArm[5][2]+jP_leftArm[3][2])-offset[2]-m2.c[0], -m2.c[0]);
    dMassAdd (&m, &m2);

    dMassTranslate(&m,-m.c[0],-m.c[1],-m.c[2]);
    dBodySetMass(larm,&m);
}

void ICubSim::initLeftArmOn(dWorldID world)
{
    dMass m, m2;
    //rotation matrises
    dQuaternion q, q1;
    dQFromAxisAndAngle(q,1,0,0, M_PI * 0.5);
    dQFromAxisAndAngle(q1,0,1,0,M_PI * 0.5);

    ///LEFT ARM
    body[0] = dBodyCreate (world);dMassSetZero(&m);dMassSetCylinderTotal(&m,0.48278,3,0.031,0.011);
    dBodySetMass(body[0],&m);
    dBodySetQuaternion(body[0],q1);
    geom[0] = dCreateCylinder (iCub,0.031,0.011);dGeomSetBody(geom[0],body[0]);

    body[2] = dBodyCreate (world);dMassSetZero(&m);dMassSetCylinderTotal(&m,0.20779,3,0.03,0.059);
    dBodySetMass(body[2],&m);
    geom[2] = dCreateCylinder (iCub,0.03,0.059);dGeomSetBody(geom[2],body[2]);

    body[4] = dBodyCreate (world);dMassSetZero(&m);dMassSetCylinderTotal(&m,1.1584,3,0.026,fabs(jP_leftArm[4][2]-jP_leftArm[2][2]));
    dBodySetMass(body[4],&m);
    dBodySetQuaternion(body[4],q);
    geom[4] = dCreateCylinder(iCub, 0.026 ,fabs(jP_leftArm[4][2]-jP_leftArm[2][2]));dGeomSetBody(geom[4],body[4]);

    body[6] = dBodyCreate (world);dMassSetZero(&m);dMassSetSphereTotal(&m,0.050798,0.01);
    dBodySetMass(body[6],&m);
    geom[6] = dCreateSphere(iCub,0.01) ;dGeomSetBody(geom[6],body[6]);
    //left lower arm
    body[8] = dBodyCreate (world);dMassSetZero(&m);dMassSetCylinderTotal(&m,0.48774,3,0.02,fabs(jP_leftArm[5][2]-jP_leftArm[3][2]));
    dBodySetMass(body[8],&m);
    dBodySetQuaternion(body[8],q);
    geom[8] = dCreateCylinder(iCub, 0.02 ,fabs(jP_leftArm[5][2]-jP_leftArm[3][2]));dGeomSetBody(geom[8],body[8]);
}

void ICubSim::initRightArmOff(dWorldID world)
{
    dMass m, m2;
    dMatrix3 Rtx;

    double offset[3];
    offset[0]=jP_rightArm[1][0];
    offset[1]=jP_rightArm[1][1]+0.5*(0.011+0.059) /*half clavicule height + half shoulder height*/;
    offset[2]=jP_rightArm[1][2];

    rarm = dBodyCreate (world);dMassSetZero(&m);dMassSetZero(&m2);
    rarm0_geom = dCreateCylinder (iCub,0.031,0.011);dMassSetCylinderTotal(&m,0.48278,3,0.031,0.011);
    dRFromAxisAndAngle(Rtx,0,1,0,M_PI * 0.5);
    dGeomSetBody (rarm0_geom,rarm);
    dGeomSetOffsetRotation(rarm0_geom,Rtx);
    dGeomSetOffsetPosition(rarm0_geom,-m2.c[0], -m2.c[0], -m2.c[0]);
    dMassAdd (&m, &m2);

    rarm1_geom = dCreateCylinder (iCub,0.03,0.059);dMassSetCylinderTotal(&m,0.20779,3,0.03,0.059);
    dGeomSetBody (rarm1_geom,rarm);
    dGeomSetOffsetPosition(rarm1_geom,jP_rightArm[2][1]-offset[1]-m2.c[0], jP_rightArm[2][2]-offset[2]-m2.c[0], -m2.c[0]);
    dMassAdd (&m, &m2);

    rarm2_geom = dCreateCylinder (iCub, 0.026 ,fabs(jP_leftArm[4][2]-jP_leftArm[2][2]));dMassSetCylinderTotal(&m,1.1584,3,0.026,fabs(jP_rightArm[4][2]-jP_rightArm[2][2]));
    dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetBody (rarm2_geom,rarm);
    dGeomSetOffsetRotation(rarm2_geom,Rtx);
    dGeomSetOffsetPosition(rarm2_geom,jP_rightArm[4][1]-offset[1]-m2.c[0], 0.5*(jP_rightArm[4][2]+jP_rightArm[2][2])-offset[2]-m2.c[0], -m2.c[0]);
    dMassAdd (&m, &m2);

    rarm3_geom = dCreateCylinder (iCub, 0.02 ,fabs(jP_rightArm[5][2]-jP_rightArm[3][2]));dMassSetCylinderTotal(&m,0.48774,3,0.02,fabs(jP_rightArm[5][2]-jP_rightArm[3][2]));
    dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetBody (rarm3_geom,rarm);
    dGeomSetOffsetRotation(rarm3_geom,Rtx);
    dGeomSetOffsetPosition(rarm3_geom,jP_rightArm[5][1]-offset[1]-m2.c[0], 0.5*(jP_rightArm[5][2]+jP_rightArm[3][2])-offset[2]-m2.c[0], -m2.c[0]);
    dMassAdd (&m, &m2);

    dMassTranslate(&m,-m.c[0],-m.c[1],-m.c[2]);
    dBodySetMass(rarm,&m);
}

void ICubSim::initRightArmOn(dWorldID world)
{
    dMass m, m2;
    //rotation matrises
    dQuaternion q, q1;
    dQFromAxisAndAngle(q,1,0,0, M_PI * 0.5);
    dQFromAxisAndAngle(q1,0,1,0,M_PI * 0.5);

    ///RIGHT ARM
    body[1] = dBodyCreate (world);dMassSetZero(&m);dMassSetCylinderTotal(&m,0.48278,3,0.031,0.011);
    dBodySetMass(body[1],&m);
    dBodySetQuaternion(body[1],q1);
    geom[1] = dCreateCylinder (iCub,0.031,0.011);dGeomSetBody(geom[1],body[1]);

    body[3] = dBodyCreate (world);dMassSetZero(&m);dMassSetCylinderTotal(&m,0.20779,3,0.03,0.059);
    dBodySetMass(body[3],&m);
    geom[3] = dCreateCylinder (iCub,0.03,0.059);dGeomSetBody(geom[3],body[3]);

    body[5] = dBodyCreate (world);dMassSetZero(&m);dMassSetCylinderTotal(&m,1.1584,3,0.026,fabs(jP_rightArm[4][2]-jP_rightArm[2][2]));
    dBodySetMass(body[5],&m);
    dBodySetQuaternion(body[5],q);
    geom[5] = dCreateCylinder(iCub, 0.026 ,fabs(jP_rightArm[4][2]-jP_rightArm[2][2]));dGeomSetBody(geom[5],body[5]);

    body[7] = dBodyCreate (world);dMassSetZero(&m);dMassSetSphereTotal(&m,0.050798,0.01);
    dBodySetMass(body[7],&m);
    geom[7] = dCreateSphere(iCub,0.01) ;dGeomSetBody(geom[7],body[7]);
    //left lower arm
    body[9] = dBodyCreate (world);dMassSetZero(&m);dMassSetCylinderTotal(&m,0.48774,3,0.02,fabs(jP_rightArm[5][2]-jP_rightArm[3][2]));
    dBodySetMass(body[9],&m);
    dBodySetQuaternion(body[9],q);
    geom[9] = dCreateCylinder(iCub, 0.02 ,fabs(jP_rightArm[5][2]-jP_rightArm[3][2]));dGeomSetBody(geom[9],body[9]);
}

void ICubSim::initLeftHandOff(dWorldID world)
{
    dMass m, m2;
    dMatrix3 Rtx;

    l_hand = dBodyCreate (world);dMassSetZero(&m);dMassSetZero(&m2);
    l_hand0_geom = dCreateBox (iCub,0.022,0.069,0.065);dMassSetBoxTotal(&m,0.19099,0.024,0.069,0.065);
    dGeomSetBody (l_hand0_geom,l_hand);
    dGeomSetOffsetPosition(l_hand0_geom,-m2.c[0], -m2.c[0], -m2.c[0]);
    dMassAdd (&m, &m2);

    l_hand1_geom = dCreateCylinder (iCub,0.0065,0.08);dMassSetCylinderTotal(&m2,0.030947,3,0.0065,0.08);
    dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetBody(l_hand1_geom,l_hand);
    dGeomSetOffsetRotation(l_hand1_geom,Rtx);
    dGeomSetOffsetPosition(l_hand1_geom,-m2.c[0], -0.0745-m2.c[0], 0.02275-m2.c[0]);
    dMassAdd (&m, &m2);

    l_hand2_geom = dCreateCylinder (iCub,0.0065,0.084);dMassSetCylinderTotal(&m2,0.030947,3,0.0065,0.084);
    dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetBody(l_hand2_geom,l_hand);
    dGeomSetOffsetRotation(l_hand2_geom,Rtx);
    dGeomSetOffsetPosition(l_hand2_geom,-m2.c[0], -0.0745-m2.c[0], 0.0065-m2.c[0]);
    dMassAdd (&m, &m2);

    l_hand3_geom = dCreateCylinder (iCub,0.0065,0.08);dMassSetCylinderTotal(&m2,0.030947,3,0.0065,0.08);
    dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetBody(l_hand3_geom,l_hand);
    dGeomSetOffsetRotation(l_hand3_geom,Rtx);
    dGeomSetOffsetPosition(l_hand3_geom,-m2.c[0], -0.0745-m2.c[0], -0.00975-m2.c[0]);
    dMassAdd (&m, &m2);

    l_hand4_geom = dCreateCylinder (iCub,0.0065,0.073);dMassSetCylinderTotal(&m2,0.030947,3,0.0065,0.073);
    dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetBody(l_hand4_geom,l_hand);
    dGeomSetOffsetRotation(l_hand4_geom,Rtx);
    dGeomSetOffsetPosition(l_hand4_geom,-m2.c[0], -0.071-m2.c[0], -0.026-m2.c[0]);
    dMassAdd (&m, &m2);

    l_hand5_geom = dCreateCylinder (iCub,0.0065,0.064);dMassSetCylinderTotal(&m2,0.02341,3,0.0065,0.064);
    dGeomSetBody(l_hand5_geom,l_hand);
    dGeomSetOffsetPosition(l_hand5_geom,-m2.c[0], 0.016-m2.c[0], 0.0645-m2.c[0]);
    dMassAdd (&m, &m2);

    dMassTranslate(&m,-m.c[0],-m.c[1],-m.c[2]);
    dBodySetMass(l_hand,&m);
}

void ICubSim::initLeftHandOn(dWorldID world)
{
    dMass m, m2;
    dMatrix3 Rtx;
    //rotation matrises
    dQuaternion q;
    dQFromAxisAndAngle(q,1,0,0, M_PI * 0.5);

    //Create all left fingers
    body[10] = dBodyCreate (world);dMassSetZero(&m);dMassSetBoxTotal(&m,0.19099,0.024,0.069,0.065);
    dBodySetMass(body[10],&m);
    geom[10] = dCreateBox (iCub,0.022,0.069,0.065);dGeomSetBody (geom[10],body[10]);

    body[12] = dBodyCreate (world);dMassSetZero(&m);dMassSetCylinderTotal(&m,0.002,3,0.0065,0.012);
    dBodySetMass(body[12],&m);
    dBodySetQuaternion(body[12],q);
    geom[12] = dCreateCylinder (iCub,0.0065,0.012);dGeomSetBody(geom[12],body[12]);

    body[13] = dBodyCreate (world);dMassSetZero(&m);dMassSetCylinderTotal(&m,0.002,3,0.0065,0.012);
    dBodySetMass(body[13],&m);
    dBodySetQuaternion(body[13],q);
    geom[13] = dCreateCylinder (iCub,0.0065,0.012);dGeomSetBody(geom[13],body[13]);

    lhandfingers0 = dBodyCreate (world);dMassSetZero(&m);dMassSetZero(&m2);//CreateCylinder(0.0065,0.012);
    lhandfings0_geom = dCreateCylinder (iCub,0.0065,0.012);dMassSetCylinderTotal(&m2,0.002,3,0.0065,0.012);
    dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetBody (lhandfings0_geom,lhandfingers0);
    dGeomSetOffsetRotation(lhandfings0_geom,Rtx);//dGeomSetPosition (rightLeg_1 , 0.0, 1.0, 0.0);
    dGeomSetOffsetPosition(lhandfings0_geom,-m2.c[0], -m2.c[0], 0.008125-m2.c[0]);//dMassRotate(&m2,Rtx);
    dMassAdd (&m, &m2);
    //second object
    lhandfings1_geom = dCreateCylinder (iCub,0.0065,0.012);dMassSetCylinderTotal(&m2,0.002,3,0.0065,0.012);
    dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetRotation (lhandfings1_geom,Rtx);
    dGeomSetBody (lhandfings1_geom,lhandfingers0);
    dGeomSetOffsetRotation(lhandfings1_geom,Rtx);
    dGeomSetOffsetPosition(lhandfings1_geom,-m2.c[0], -m2.c[0], -0.008125-m2.c[0]);//dGeomSetPosition (rightLeg_2 , 0.0, 1, 0.0);
    //add mass accumulated
    dMassAdd (&m, &m2);
    //translate
    dMassTranslate(&m,-m.c[0],-m.c[1],-m.c[2]);
    //Set mass to the actual body
    dBodySetMass(lhandfingers0,&m);

    body[16] = dBodyCreate (world);dMassSetZero(&m);dMassSetCylinderTotal(&m,0.002,3,0.0065,0.026);
    dBodySetMass(body[16],&m);
    dBodySetQuaternion(body[16],q);
    geom[16] = dCreateCylinder (iCub,0.0065,0.026);dGeomSetBody(geom[16],body[16]);

    body[17] = dBodyCreate (world);dMassSetZero(&m);dMassSetCylinderTotal(&m,0.002,3,0.0065,0.028);
    dBodySetMass(body[17],&m);
    dBodySetQuaternion(body[17],q);
    geom[17] = dCreateCylinder (iCub,0.0065,0.028);dGeomSetBody(geom[17],body[17]);

    lhandfingers1 = dBodyCreate (world);dMassSetZero(&m);dMassSetZero(&m2);//CreateCylinder(0.0065,0.012);
    lhandfings2_geom = dCreateCylinder (iCub,0.0065,0.026);dMassSetCylinderTotal(&m2,0.002,3,0.0065,0.026);
    dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetBody (lhandfings2_geom,lhandfingers1);
    dGeomSetOffsetRotation(lhandfings2_geom,Rtx);//dGeomSetPosition (rightLeg_1 , 0.0, 1.0, 0.0);
    dGeomSetOffsetPosition(lhandfings2_geom,-m2.c[0], -m2.c[0], 0.008125-m2.c[0]);//dMassRotate(&m2,Rtx);
    dMassAdd (&m, &m2);
    //second object
    lhandfings3_geom = dCreateCylinder (iCub,0.0065,0.022);dMassSetCylinderTotal(&m2,0.002,3,0.0065,0.022);
    dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetRotation (lhandfings3_geom,Rtx);
    dGeomSetBody (lhandfings3_geom,lhandfingers1);
    dGeomSetOffsetRotation(lhandfings3_geom,Rtx);
    dGeomSetOffsetPosition(lhandfings3_geom,-m2.c[0], 0.002-m2.c[0], -0.008125-m2.c[0]);//dGeomSetPosition (rightLeg_2 , 0.0, 1, 0.0);
    dMassAdd (&m, &m2);
    dMassTranslate(&m,-m.c[0],-m.c[1],-m.c[2]);dBodySetMass(lhandfingers1,&m);

    body[20] = dBodyCreate (world);dMassSetZero(&m);dMassSetCylinderTotal(&m,0.002,3,0.0065,0.022);dBodySetMass(body[20],&m);
    dBodySetQuaternion(body[20],q);geom[20] = dCreateCylinder(iCub, 0.0065,0.022);dGeomSetBody(geom[20],body[20]);
    body[21] = dBodyCreate (world);dMassSetZero(&m);dMassSetCylinderTotal(&m,0.002,3,0.0065,0.024);dBodySetMass(body[21],&m);
    dBodySetQuaternion(body[21],q);geom[21] = dCreateCylinder(iCub, 0.0065,0.024);dGeomSetBody(geom[21],body[21]);

    lhandfingers2 = dBodyCreate (world);dMassSetZero(&m);dMassSetZero(&m2);//CreateCylinder(0.0065,0.012);
    lhandfings4_geom = dCreateCylinder (iCub,0.0065,0.022);dMassSetCylinderTotal(&m2,0.002,3,0.0065,0.022);
    dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetBody (lhandfings4_geom,lhandfingers2);
    dGeomSetOffsetRotation(lhandfings4_geom,Rtx);//dGeomSetPosition (rightLeg_1 , 0.0, 1.0, 0.0);
    dGeomSetOffsetPosition(lhandfings4_geom,-m2.c[0], -m2.c[0], 0.008125-m2.c[0]);
    dMassAdd (&m, &m2);
    //second object
    lhandfings5_geom = dCreateCylinder (iCub,0.0065,0.019);dMassSetCylinderTotal(&m2,0.002,3,0.0065,0.019);
    dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetRotation (lhandfings5_geom,Rtx);
    dGeomSetBody (lhandfings5_geom,lhandfingers2);
    dGeomSetOffsetRotation(lhandfings5_geom,Rtx);
    dGeomSetOffsetPosition(lhandfings5_geom,-m2.c[0], 0.0055-m2.c[0], -0.008125-m2.c[0]);//dGeomSetPosition (rightLeg_2 , 0.0, 1, 0.0);
    dMassAdd (&m, &m2);
    dMassTranslate(&m,-m.c[0],-m.c[1],-m.c[2]);dBodySetMass(lhandfingers2,&m);

    body[24] = dBodyCreate (world);dMassSetZero(&m);dMassSetCylinderTotal(&m,0.002,3,0.0065,0.02);dBodySetMass(body[24],&m);
    dBodySetQuaternion(body[24],q);geom[24] = dCreateCylinder(iCub, 0.0065,0.02);dGeomSetBody(geom[24],body[24]);
    body[25] = dBodyCreate (world);dMassSetZero(&m);dMassSetCylinderTotal(&m,0.002,3,0.0065,0.02);dBodySetMass(body[25],&m);
    dBodySetQuaternion(body[25],q);geom[25] = dCreateCylinder(iCub, 0.0065,0.02);dGeomSetBody(geom[25],body[25]);

    lhandfingers3 = dBodyCreate (world);dMassSetZero(&m);dMassSetZero(&m2);//CreateCylinder(0.0065,0.012);
    lhandfings6_geom = dCreateCylinder (iCub,0.0065,0.02);dMassSetCylinderTotal(&m2,0.002,3,0.0065,0.02);
    dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetBody (lhandfings6_geom,lhandfingers3);
    dGeomSetOffsetRotation(lhandfings6_geom,Rtx);//dGeomSetPosition (rightLeg_1 , 0.0, 1.0, 0.0);
    dGeomSetOffsetPosition(lhandfings6_geom,-m2.c[0], -m2.c[0], 0.008125-m2.c[0]);
    dMassAdd (&m, &m2);
    //second object
    lhandfings7_geom = dCreateCylinder (iCub,0.0065,0.02);dMassSetCylinderTotal(&m2,0.002,3,0.0065,0.02);
    dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetRotation (lhandfings7_geom,Rtx);
    dGeomSetBody (lhandfings7_geom,lhandfingers3);
    dGeomSetOffsetRotation(lhandfings7_geom,Rtx);
    dGeomSetOffsetPosition(lhandfings7_geom,-m2.c[0], 0.007-m2.c[0], -0.008125-m2.c[0]);//dGeomSetPosition (rightLeg_2 , 0.0, 1, 0.0);
    dMassAdd (&m, &m2);
    dMassTranslate(&m,-m.c[0],-m.c[1],-m.c[2]);dBodySetMass(lhandfingers3,&m);

    body[28] = dBodyCreate (world);dMassSetZero(&m);dMassSetCylinderTotal(&m,0.002,3,0.0065,0.026);
    dBodySetMass(body[28],&m);//dBodySetQuaternion(body[28],q1);
    geom[28] = dCreateCylinder(iCub,0.0065,0.026);dGeomSetBody(geom[28],body[28]);

    body[29] = dBodyCreate (world);dMassSetZero(&m);dMassSetCylinderTotal(&m,0.002,3,0.0065,0.022);
    dBodySetMass(body[29],&m);//dBodySetQuaternion(body[29],q1);
    geom[29] = dCreateCylinder(iCub,0.0065,0.022);dGeomSetBody(geom[29],body[29]);

    body[30] = dBodyCreate (world);dMassSetZero(&m);dMassSetCylinderTotal(&m,0.002,3,0.0065,0.016);
    dBodySetMass(body[30],&m);//dBodySetQuaternion(body[30],q1);
    geom[30] = dCreateCylinder(iCub,0.0065,0.016);dGeomSetBody(geom[30],body[30]);
}

void ICubSim::initRightHandOff(dWorldID world)
{
    dMass m, m2;
    dMatrix3 Rtx;

    r_hand = dBodyCreate (world);dMassSetZero(&m);dMassSetZero(&m2);
    r_hand0_geom = dCreateBox (iCub,0.022,0.069,0.065);dMassSetBoxTotal(&m,0.19099,0.024,0.069,0.065);
    dGeomSetBody (r_hand0_geom,r_hand);
    dGeomSetOffsetPosition(r_hand0_geom,-m2.c[0], -m2.c[0], -m2.c[0]);
    dMassAdd (&m, &m2);

    r_hand1_geom = dCreateCylinder (iCub,0.0065,0.08);dMassSetCylinderTotal(&m2,0.030947,3,0.0065,0.08);
    dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetBody(r_hand1_geom,r_hand);
    dGeomSetOffsetRotation(r_hand1_geom,Rtx);
    dGeomSetOffsetPosition(r_hand1_geom,-m2.c[0], -0.0745-m2.c[0], 0.02275-m2.c[0]);
    dMassAdd (&m, &m2);

    r_hand2_geom = dCreateCylinder (iCub,0.0065,0.084);dMassSetCylinderTotal(&m2,0.030947,3,0.0065,0.084);
    dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetBody(r_hand2_geom,r_hand);
    dGeomSetOffsetRotation(r_hand2_geom,Rtx);
    dGeomSetOffsetPosition(r_hand2_geom,-m2.c[0], -0.0745-m2.c[0], 0.0065-m2.c[0]);
    dMassAdd (&m, &m2);

    r_hand3_geom = dCreateCylinder (iCub,0.0065,0.08);dMassSetCylinderTotal(&m2,0.030947,3,0.0065,0.08);
    dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetBody(r_hand3_geom,r_hand);
    dGeomSetOffsetRotation(r_hand3_geom,Rtx);
    dGeomSetOffsetPosition(r_hand3_geom,-m2.c[0], -0.0745-m2.c[0], -0.00975-m2.c[0]);
    dMassAdd (&m, &m2);

    r_hand4_geom = dCreateCylinder (iCub,0.0065,0.073);dMassSetCylinderTotal(&m2,0.030947,3,0.0065,0.073);
    dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetBody(r_hand4_geom,r_hand);
    dGeomSetOffsetRotation(r_hand4_geom,Rtx);
    dGeomSetOffsetPosition(r_hand4_geom,-m2.c[0], -0.071-m2.c[0], -0.026-m2.c[0]);
    dMassAdd (&m, &m2);

    r_hand5_geom = dCreateCylinder (iCub,0.0065,0.064);dMassSetCylinderTotal(&m2,0.02341,3,0.0065,0.064);
    dGeomSetBody(r_hand5_geom,r_hand);
    dGeomSetOffsetPosition(r_hand5_geom,-m2.c[0], 0.016-m2.c[0], 0.0645-m2.c[0]);
    dMassAdd (&m, &m2);

    dMassTranslate(&m,-m.c[0],-m.c[1],-m.c[2]);
    dBodySetMass(r_hand,&m);
}

void ICubSim::initRightHandOn(dWorldID world)
{
    dMass m, m2;
    dMatrix3 Rtx;
    //rotation matrises
    dQuaternion q;
    dQFromAxisAndAngle(q,1,0,0, M_PI * 0.5);

    //Create all right fingers
    body[11] = dBodyCreate (world);dMassSetZero(&m);dMassSetBoxTotal(&m,0.19099,0.024,0.069,0.065);
    dBodySetMass(body[11],&m);
    geom[11] = dCreateBox (iCub,0.022,0.069,0.065);dGeomSetBody (geom[11],body[11]);

    body[31] = dBodyCreate (world);dMassSetZero(&m);dMassSetCylinderTotal(&m,0.002,3,0.0065,0.012);
    dBodySetMass(body[31],&m);
    dBodySetQuaternion(body[31],q);
    geom[31] = dCreateCylinder (iCub,0.0065,0.012);dGeomSetBody(geom[31],body[31]);

    body[32] = dBodyCreate (world);dMassSetZero(&m);dMassSetCylinderTotal(&m,0.002,3,0.0065,0.012);
    dBodySetMass(body[32],&m);
    dBodySetQuaternion(body[32],q);
    geom[32] = dCreateCylinder (iCub,0.0065,0.012);dGeomSetBody(geom[32],body[32]);

    rhandfingers0 = dBodyCreate (world);dMassSetZero(&m);dMassSetZero(&m2);//CreateCylinder(0.0065,0.012);
    rhandfings0_geom = dCreateCylinder (iCub,0.0065,0.012);dMassSetCylinderTotal(&m2,0.002,3,0.0065,0.012);
    dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetBody (rhandfings0_geom,rhandfingers0);
    dGeomSetOffsetRotation(rhandfings0_geom,Rtx);//dGeomSetPosition (rightLeg_1 , 0.0, 1.0, 0.0);
    dGeomSetOffsetPosition(rhandfings0_geom,-m2.c[0], -m2.c[0], 0.008125-m2.c[0]);//dMassRotate(&m2,Rtx);
    dMassAdd (&m, &m2);
    //second object
    rhandfings1_geom = dCreateCylinder (iCub,0.0065,0.012);dMassSetCylinderTotal(&m2,0.002,3,0.0065,0.012);
    dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetRotation (rhandfings1_geom,Rtx);
    dGeomSetBody (rhandfings1_geom,rhandfingers0);
    dGeomSetOffsetRotation(rhandfings1_geom,Rtx);
    dGeomSetOffsetPosition(rhandfings1_geom,-m2.c[0], -m2.c[0], -0.008125-m2.c[0]);//dGeomSetPosition (rightLeg_2 , 0.0, 1, 0.0);
    //add mass accumulated
    dMassAdd (&m, &m2);
    //translate
    dMassTranslate(&m,-m.c[0],-m.c[1],-m.c[2]);
    //Set mass to the actual body
    dBodySetMass(rhandfingers0,&m);

    body[35] = dBodyCreate (world);dMassSetZero(&m);dMassSetCylinderTotal(&m,0.002,3,0.0065,0.026);
    dBodySetMass(body[35],&m);
    dBodySetQuaternion(body[35],q);
    geom[35] = dCreateCylinder (iCub,0.0065,0.026);dGeomSetBody(geom[35],body[35]);

    body[36] = dBodyCreate (world);dMassSetZero(&m);dMassSetCylinderTotal(&m,0.002,3,0.0065,0.028);
    dBodySetMass(body[36],&m);
    dBodySetQuaternion(body[36],q);
    geom[36] = dCreateCylinder (iCub,0.0065,0.028);dGeomSetBody(geom[36],body[36]);

    rhandfingers1 = dBodyCreate (world);dMassSetZero(&m);dMassSetZero(&m2);//CreateCylinder(0.0065,0.012);
    rhandfings2_geom = dCreateCylinder (iCub,0.0065,0.026);dMassSetCylinderTotal(&m2,0.002,3,0.0065,0.026);
    dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetBody (rhandfings2_geom,rhandfingers1);
    dGeomSetOffsetRotation(rhandfings2_geom,Rtx);//dGeomSetPosition (rightLeg_1 , 0.0, 1.0, 0.0);
    dGeomSetOffsetPosition(rhandfings2_geom,-m2.c[0], -m2.c[0], 0.008125-m2.c[0]);//dMassRotate(&m2,Rtx);
    dMassAdd (&m, &m2);
    //second object
    rhandfings3_geom = dCreateCylinder (iCub,0.0065,0.022);dMassSetCylinderTotal(&m2,0.002,3,0.0065,0.022);
    dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetRotation (rhandfings3_geom,Rtx);
    dGeomSetBody (rhandfings3_geom,rhandfingers1);
    dGeomSetOffsetRotation(rhandfings3_geom,Rtx);
    dGeomSetOffsetPosition(rhandfings3_geom,-m2.c[0], 0.002-m2.c[0], -0.008125-m2.c[0]);//dGeomSetPosition (rightLeg_2 , 0.0, 1, 0.0);
    dMassAdd (&m, &m2);
    dMassTranslate(&m,-m.c[0],-m.c[1],-m.c[2]);dBodySetMass(rhandfingers1,&m);

    body[39] = dBodyCreate (world);dMassSetZero(&m);dMassSetCylinderTotal(&m,0.002,3,0.0065,0.022);dBodySetMass(body[39],&m);
    dBodySetQuaternion(body[39],q);geom[39] = dCreateCylinder(iCub, 0.0065,0.022);dGeomSetBody(geom[39],body[39]);
    body[40] = dBodyCreate (world);dMassSetZero(&m);dMassSetCylinderTotal(&m,0.002,3,0.0065,0.024);dBodySetMass(body[40],&m);
    dBodySetQuaternion(body[40],q);geom[40] = dCreateCylinder(iCub, 0.0065,0.024);dGeomSetBody(geom[40],body[40]);

    rhandfingers2 = dBodyCreate (world);dMassSetZero(&m);dMassSetZero(&m2);//CreateCylinder(0.0065,0.012);
    rhandfings4_geom = dCreateCylinder (iCub,0.0065,0.022);dMassSetCylinderTotal(&m2,0.002,3,0.0065,0.022);
    dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetBody (rhandfings4_geom,rhandfingers2);
    dGeomSetOffsetRotation(rhandfings4_geom,Rtx);//dGeomSetPosition (rightLeg_1 , 0.0, 1.0, 0.0);
    dGeomSetOffsetPosition(rhandfings4_geom,-m2.c[0], -m2.c[0], 0.008125-m2.c[0]);
    dMassAdd (&m, &m2);
    //second object
    rhandfings5_geom = dCreateCylinder (iCub,0.0065,0.019);dMassSetCylinderTotal(&m2,0.002,3,0.0065,0.019);
    dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetRotation (rhandfings5_geom,Rtx);
    dGeomSetBody (rhandfings5_geom,rhandfingers2);
    dGeomSetOffsetRotation(rhandfings5_geom,Rtx);
    dGeomSetOffsetPosition(rhandfings5_geom,-m2.c[0], 0.0055-m2.c[0], -0.008125-m2.c[0]);//dGeomSetPosition (rightLeg_2 , 0.0, 1, 0.0);
    dMassAdd (&m, &m2);
    dMassTranslate(&m,-m.c[0],-m.c[1],-m.c[2]);dBodySetMass(rhandfingers2,&m);

    body[43] = dBodyCreate (world);dMassSetZero(&m);dMassSetCylinderTotal(&m,0.002,3,0.0065,0.02);dBodySetMass(body[43],&m);
    dBodySetQuaternion(body[43],q);geom[43] = dCreateCylinder(iCub, 0.0065,0.02);dGeomSetBody(geom[43],body[43]);
    body[44] = dBodyCreate (world);dMassSetZero(&m);dMassSetCylinderTotal(&m,0.002,3,0.0065,0.02);dBodySetMass(body[44],&m);
    dBodySetQuaternion(body[44],q);geom[44] = dCreateCylinder(iCub, 0.0065,0.02);dGeomSetBody(geom[44],body[44]);

    rhandfingers3 = dBodyCreate (world);dMassSetZero(&m);dMassSetZero(&m2);//CreateCylinder(0.0065,0.012);
    rhandfings6_geom = dCreateCylinder (iCub,0.0065,0.02);dMassSetCylinderTotal(&m2,0.002,3,0.0065,0.02);
    dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetBody (rhandfings6_geom,rhandfingers3);
    dGeomSetOffsetRotation(rhandfings6_geom,Rtx);//dGeomSetPosition (rightLeg_1 , 0.0, 1.0, 0.0);
    dGeomSetOffsetPosition(rhandfings6_geom,-m2.c[0], -m2.c[0], 0.008125-m2.c[0]);
    dMassAdd (&m, &m2);
    //second object
    rhandfings7_geom = dCreateCylinder (iCub,0.0065,0.02);dMassSetCylinderTotal(&m2,0.002,3,0.0065,0.02);
    dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetRotation (rhandfings7_geom,Rtx);
    dGeomSetBody (rhandfings7_geom,rhandfingers3);
    dGeomSetOffsetRotation(rhandfings7_geom,Rtx);
    dGeomSetOffsetPosition(rhandfings7_geom,-m2.c[0], 0.007-m2.c[0], -0.008125-m2.c[0]);//dGeomSetPosition (rightLeg_2 , 0.0, 1, 0.0);
    dMassAdd (&m, &m2);
    dMassTranslate(&m,-m.c[0],-m.c[1],-m.c[2]);dBodySetMass(rhandfingers3,&m);

    body[47] = dBodyCreate (world);dMassSetZero(&m);dMassSetCylinderTotal(&m,0.002,3,0.0065,0.026);
    dBodySetMass(body[47],&m);//dBodySetQuaternion(body[28],q1);
    geom[47] = dCreateCylinder(iCub,0.0065,0.026);dGeomSetBody(geom[47],body[47]);

    body[48] = dBodyCreate (world);dMassSetZero(&m);dMassSetCylinderTotal(&m,0.002,3,0.0065,0.022);
    dBodySetMass(body[48],&m);//dBodySetQuaternion(body[29],q1);
    geom[48] = dCreateCylinder(iCub,0.0065,0.022);dGeomSetBody(geom[48],body[48]);

    body[49] = dBodyCreate (world);dMassSetZero(&m);dMassSetCylinderTotal(&m,0.002,3,0.0065,0.016);
    dBodySetMass(body[49],&m);//dBodySetQuaternion(body[30],q1);
    geom[49] = dCreateCylinder(iCub,0.0065,0.016);dGeomSetBody(geom[49],body[49]);
}

void ICubSim::initHead(dWorldID world)
{
    dMass m, m2;
    dMatrix3 Rtx;
    //rotation matrises
    dQuaternion q1;
    dQFromAxisAndAngle(q1,0,1,0,M_PI * 0.5);

    head = dBodyCreate (world);
    if (actHead == "off")
    {
        dMassSetZero(&m);dMassSetZero(&m2);
        neck0_geom = dCreateCylinder (iCub,0.015,0.077);dMassSetCylinderTotal(&m2,0.28252,3,0.015,0.077);
        dRFromAxisAndAngle(Rtx,0,1,0,M_PI * 0.5);
        dGeomSetBody (neck0_geom,head);
        dGeomSetOffsetRotation(neck0_geom,Rtx);//dGeomSetPosition (rightLeg_1 , 0.0, 1.0, 0.0);
        dGeomSetOffsetPosition(neck0_geom, -fabs(jP_head[2][1]-jP_head[0][1])-m2.c[0], -fabs(jP_head[2][2]-jP_head[0][2])-0.03-m2.c[0],  -fabs(jP_head[2][0]-jP_head[0][0])-m2.c[0]);
        dMassAdd (&m, &m2);

        neck1_geom = dCreateCylinder (iCub,0.015,0.077);dMassSetCylinderTotal(&m2,0.1,3,0.015,0.077);
        dRFromAxisAndAngle(Rtx,0,0,1,M_PI * 0.5);
        dGeomSetBody (neck1_geom,head);
        dGeomSetOffsetRotation(neck1_geom,Rtx);//dGeomSetPosition (rightLeg_1 , 0.0, 1.0, 0.0);
        dGeomSetOffsetPosition(neck1_geom, -fabs(jP_head[2][1]-jP_head[1][1])-m2.c[0], -fabs(jP_head[2][2]-jP_head[1][2])-0.03-m2.c[0], -fabs(jP_head[2][0]-jP_head[1][0])-m2.c[0]);
        dMassAdd (&m, &m2);
        dMassTranslate(&m,-m.c[0],-m.c[1],-m.c[2]);dBodySetMass(head,&m);

    }else{
        neck[0] = dBodyCreate (world);dMassSetZero(&m);dMassSetCylinderTotal(&m,0.28252,3,0.015,0.077);
        dBodySetMass(neck[0],&m);dBodySetQuaternion(neck[0],q1);
        neckgeom[0] = dCreateCylinder(iCub, 0.015,0.077);dGeomSetBody(neckgeom[0],neck[0]);

        neck[1] = dBodyCreate (world);dMassSetZero(&m);dMassSetCylinderTotal(&m,0.1,3,0.015,0.077);
        dBodySetMass(neck[1],&m);
        neckgeom[1] = dCreateCylinder(iCub, 0.015,0.077);dGeomSetBody(neckgeom[1],neck[1]);

        eye = dBodyCreate (world);//left eye
        leye = dBodyCreate (world);//left eye
        reye = dBodyCreate (world);//right eye
    }
    topEyeLid = dBodyCreate (world);//top eye lid
    bottomEyeLid = dBodyCreate (world);//bottom eye lid
    dBodySetGravityMode(topEyeLid,0);
    dBodySetGravityMode(bottomEyeLid,0);

    dMassSetZero(&m);dMassSetZero(&m2);
    //Head encapsulated object to save computation on the joints....

    //head yaw cyl
    head0_geom = dCreateCylinder (iCub,0.015,0.06);dMassSetCylinderTotal (&m2,0.13913,3,0.015,0.06);
    dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetBody (head0_geom,head);
    dGeomSetOffsetRotation(head0_geom,Rtx);//dGeomSetPosition (rightLeg_1 , 0.0, 1.0, 0.0);
    dGeomSetOffsetPosition(head0_geom,-m2.c[0], -m2.c[0], -m2.c[0]);
    dMassAdd (&m, &m2);

        
    //larger horizontal plate
    head1_geom = dCreateBox (iCub,0.104, 0.002,0.052);dMassSetBoxTotal (&m2,0.1562,0.104,0.002,0.052);
    dGeomSetBody (head1_geom,head);
    dGeomSetOffsetPosition(head1_geom,-m2.c[0], /*-0.011-*/0.03/*half neck yaw*/-m2.c[0], -0.0125-m2.c[0]);
    dMassAdd (&m, &m2);

    //left vertical plate
    head2_geom = dCreateBox (iCub,0.002, 0.093,0.052);dMassSetBoxTotal (&m2,0.1562,0.002, 0.093,0.052);
    dGeomSetBody (head2_geom,head);
    dGeomSetOffsetPosition(head2_geom ,0.052-m2.c[0], /*0.0355-*/0.03/*half neck yaw*/-m2.c[0], -0.0125-m2.c[0]);
    dMassAdd (&m, &m2);

    //right vertical plate
    head3_geom = dCreateBox (iCub,0.002, 0.093,0.052);dMassSetBoxTotal (&m2,0.1562,0.002, 0.093,0.032);
    dGeomSetBody (head3_geom,head);
    dGeomSetOffsetPosition(head3_geom,-0.052-m2.c[0], /*0.0355-*/0.03/*half neck yaw*/-m2.c[0], -0.0125-m2.c[0]);
    dMassAdd (&m, &m2);

    //smaller horizontal plate (hidden)
    head4_geom = dCreateBox (iCub, 0.104, 0.002,0.032);dMassSetBoxTotal (&m2,0.01,0.104, 0.002,0.032);
    dGeomSetBody (head4_geom,head);
    dGeomSetOffsetPosition(head4_geom,-m2.c[0], /*0.0355-*/0.03/*half neck yaw*/-m2.c[0], -0.0125-m2.c[0]);
    dMassAdd (&m, &m2);

    //nose initial box
    head5_geom = dCreateBox (iCub, 0.011, 0.026, 0.025);dMassSetBoxTotal (&m2,0.0278,0.011, 0.026,0.025);
    dGeomSetBody (head5_geom,head);
    dGeomSetOffsetPosition(head5_geom,-m2.c[0], /*-0.01-*/m2.c[0],  jP_head[3][0]-jP_head[2][0]-0.04-m2.c[0]);
    dMassAdd (&m, &m2);

    //nose diagonal box
    head6_geom = dCreateBox (iCub, 0.011, 0.051, 0.012);dMassSetBoxTotal (&m2,0.0278,0.011, 0.051,0.012);
    dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.2);
    dGeomSetBody (head6_geom,head);
    dGeomSetOffsetRotation(head6_geom,Rtx);
    dGeomSetOffsetPosition(head6_geom,-m2.c[0], /*0.001-*/0.03/*half neck yaw*/-m2.c[0], jP_head[3][0]-jP_head[2][0]-0.02-m2.c[0]);
    dMassAdd (&m, &m2);

    //nose end box
    head7_geom = dCreateBox (iCub, 0.02, 0.022, 0.012);dMassSetBox (&m2,0.0278,0.02, 0.022,0.012);
    dGeomSetBody (head7_geom,head);
    dGeomSetOffsetPosition(head7_geom,-m2.c[0], /*0.028-*/jP_head[3][2]-jP_head[2][2]-0.03/*half neck yaw*/-m2.c[0], jP_head[3][0]-jP_head[2][0]-m2.c[0]);
    dMassAdd (&m, &m2);

    //dMassTranslate(&m,-m.c[0],-m.c[1],-m.c[2]);dBodySetMass(head,&m);
}

void ICubSim::initEyes(dWorldID world)
{
    dBodyID select[3];
    double offset[3];
    if (actHead == "off")
    {
        select[0] = select[1] = select[2] = head;
        offset[0]=jP_head[3][0]-jP_head[2][0];
        offset[1]=jP_head[3][1]-jP_head[2][1];
        offset[2]=jP_head[3][2]-jP_head[2][2]-0.03;
    }
    else
    {
        select[0] = eye;
        select[1] = leye;
        select[2] = reye;
        offset[0]=offset[1]=offset[2]=0.0;
    }    

    dMass m, m2;
    dMatrix3 Rtx;

    //eyes pitch cylinder
    dMassSetZero(&m);dMassSetZero(&m2);
    eye1_geom = dCreateCylinder (iCub,0.002,0.068);dMassSetCylinderTotal (&m2,0.0059678,3,0.002,0.068);
    dRFromAxisAndAngle(Rtx,0,1,0,M_PI * 0.5);
    dGeomSetBody (eye1_geom,select[0]);
    dGeomSetOffsetRotation(eye1_geom,Rtx);
    dGeomSetOffsetPosition(eye1_geom,offset[1]-m2.c[0], /*0.037-*/+offset[2]-m2.c[0], +offset[0]-m2.c[0]);
    dMassAdd (&m, &m2);

    eye2_geom = dCreateCylinder (iCub,0.006,0.030);dMassSetCylinderTotal (&m2,0.11,3,0.006,0.030);
    //dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetBody (eye2_geom,select[0]);
    //dGeomSetOffsetRotation(eye2_geom,Rtx);
    dGeomSetOffsetPosition(eye2_geom,0.034+offset[1]-m2.c[0], /*0.037-*/+offset[2]-m2.c[0], -0.015+offset[0]-m2.c[0]);
    dMassAdd (&m, &m2);

    eye3_geom = dCreateCylinder (iCub,0.006,0.05);dMassSetCylinderTotal (&m2,0.0387,3,0.006,0.05);
    dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetBody (eye3_geom,select[0]);
    dGeomSetOffsetRotation(eye3_geom,Rtx);
    dGeomSetOffsetPosition(eye3_geom,0.034+offset[1]-m2.c[0], /*0.037-*/+offset[2]-m2.c[0], -0.03+offset[0]-m2.c[0]);
    dMassAdd (&m, &m2);

    eye4_geom = dCreateCylinder (iCub,0.006,0.030);dMassSetCylinderTotal (&m2,0.0234,3,0.006,0.030);
    //dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetBody (eye4_geom,select[0]);
    //dGeomSetOffsetRotation(eye4_geom,Rtx);
    dGeomSetOffsetPosition(eye4_geom,-0.034+offset[1]-m2.c[0], /*0.037-*/+offset[2]-m2.c[0], -0.015+offset[0]-m2.c[0]);
    dMassAdd (&m, &m2);

    eye5_geom = dCreateCylinder (iCub,0.006,0.05);dMassSetCylinderTotal (&m2,0.0387,3,0.006,0.05);
    dRFromAxisAndAngle(Rtx,1,0,0,M_PI * 0.5);
    dGeomSetBody (eye5_geom,select[0]);
    dGeomSetOffsetRotation(eye5_geom,Rtx);
    dGeomSetOffsetPosition(eye5_geom,-0.034+offset[1]-m2.c[0], /*0.037-*/+offset[2]-m2.c[0], -0.03+offset[0]-m2.c[0]);
    dMassAdd (&m, &m2);
    dMassTranslate(&m,-m.c[0],-m.c[1],-m.c[2]);

    //eyeLids
    topEyeLid_geom = dCreateCylinder (iCub,0.01,0.01);
    dGeomSetBody (topEyeLid_geom,topEyeLid);

    bottomEyeLid_geom = dCreateCylinder (iCub,0.01,0.01);
    dGeomSetBody (bottomEyeLid_geom,bottomEyeLid);

    if (actHead == "on"){
        dBodySetMass(eye,&m);
    }else {
        dBodySetMass(head,&m);
    }
    dMassSetZero(&m);dMassSetZero(&m2);

    if(actHead=="off")
    {
        offset[0]=jP_leftEye[1][0]-jP_head[2][0];
        offset[1]=jP_leftEye[1][1]-jP_head[2][1];
        offset[2]=jP_leftEye[1][2]-jP_head[2][2]-0.03;
    }
    else
    {
        offset[0]=0.0;
        offset[1]=0.0;
        offset[2]=0.0;
    }

    Leye1_geom = dCreateCylinder (iCub,0.006,0.011);dMassSetCylinderTotal (&m2,0.0234,3,0.006,0.011);
    dGeomSetBody (Leye1_geom,select[1]);
    dGeomSetOffsetPosition(Leye1_geom,offset[1]-m2.c[0], /*0.037*/+offset[2]-m2.c[0], +offset[0]-m2.c[0]);
    dMassAdd (&m, &m2);

    dMassTranslate(&m,-m.c[0],-m.c[1],-m.c[2]);
    if (actHead == "on"){
        dBodySetMass(leye,&m);
    }else {
        dBodySetMass(head,&m);
    }
    dMassSetZero(&m);dMassSetZero(&m2);

    if(actHead=="off")
    {
        offset[0]=jP_rightEye[1][0]-jP_head[2][0];
        offset[1]=jP_rightEye[1][1]-jP_head[2][1];
        offset[2]=jP_rightEye[1][2]-jP_head[2][2]-0.03;
    }
    else
    {
        offset[0]=0.0;
        offset[1]=0.0;
        offset[2]=0.0;
    }


    Reye1_geom = dCreateCylinder (iCub,0.006,0.011);dMassSetCylinderTotal (&m2,0.0234,3,0.006,0.011);
    dGeomSetBody (Reye1_geom,select[2]);
    dGeomSetOffsetPosition(Reye1_geom,offset[1]-m2.c[0], /*0.037*/+offset[2]-m2.c[0], +offset[0]-m2.c[0]);
    dMassAdd (&m, &m2);

    dMassTranslate(&m,-m.c[0],-m.c[1],-m.c[2]);
    if (actHead == "on"){
        dBodySetMass(reye,&m);
    }else {
        dBodySetMass(head,&m);
    }
}

void ICubSim::initCovers(ResourceFinder& finder)
{
    if (actHeadCover == "on")
    {
        iCubHeadModel =  new Model();
        topEyeLidModel =      new Model();
        bottomEyeLidModel =   new Model();
        iCubHeadModel->loadModelData(finder.findFile("data/model/iCub_Head.ms3d").c_str());
        topEyeLidModel->loadModelData(finder.findFile("data/model/topEyeLid.ms3d").c_str());
        bottomEyeLidModel->loadModelData(finder.findFile("data/model/bottomEyeLid.ms3d").c_str());
    }

    numCovers = 10;

    if (actLegsCovers == "on")
    {
        model["lowerLeg"] = finder.findFile("lowerLegCover");//also used for left
        model["upperRightLeg"] = finder.findFile("upperRightLegCover");
        model["upperLeftLeg"] = finder.findFile("upperLeftLegCover");
        model["rightFoot"] = finder.findFile("rightFootCover");
        model["leftFoot"] = finder.findFile("leftFootCover");
    }
    if(actLeftArmCovers == "on" || actRightArmCovers == "on")
        model["lowerArm"] = finder.findFile("lowerArmCover");//used for both arms

    if (actLeftArmCovers == "on")
    {
        model["upperLeftArm"] = finder.findFile("leftUpperArmCover");
        model["leftPalm"] = finder.findFile("leftPalm");
    }

    if (actRightArmCovers == "on")
    {
        model["upperRightArm"] = finder.findFile("rightUpperArmCover");
        model["rightPalm"] = finder.findFile("rightPalm");
    }

    if (actTorsoCovers == "on")
    {
        model["torso"] = finder.findFile("torsoCover");
        model["waist"] = finder.findFile("waistCover");
    }

    textureName[0] = finder.findFile("lowerArmTexture");//texture used for all covers

    cout << "\nCreating 3D Model of the icub.......\n" << endl;
    for (map<string,ConstString>::iterator itr=model.begin(); itr != model.end(); itr++)
    {
        model_TriData[(*itr).first] = dGeomTriMeshDataCreate();
        model_trimesh[(*itr).first] = dLoadMeshFromX(model[(*itr).first].c_str());
        if (!model_trimesh[(*itr).first])
        {
            cout << "Check spelling/location of file" << endl;
        }else
        {
            dGeomTriMeshDataBuildSingle(model_TriData[(*itr).first], model_trimesh[(*itr).first]->Vertices, 3 * sizeof(float), model_trimesh[(*itr).first]->VertexCount, model_trimesh[(*itr).first]->Indices, model_trimesh[(*itr).first]->IndexCount, 3 * sizeof(int));
            model_ThreeD_obj[(*itr).first].geom = dCreateTriMesh(iCub, model_TriData[(*itr).first], 0, 0, 0);
            dGeomSetData(model_ThreeD_obj[(*itr).first].geom,model_TriData[(*itr).first]);
            //dGeomSetPosition(model_ThreeD_obj[(*itr).first].geom,0.0,0.5,0.5);
        }
    }
    modelTextureIndex++;
    modelTexture[0] = modelTextureIndex;
    //the reference object
    //body_cube[0] = dBodyCreate(world); dMassSetZero(&m); dMassSetBoxTotal (&m,0.1,0.1,0.005,0.1);dBodySetMass(body_cube[0],&m);
    //geom_cube[0] = dCreateBox(space,0.1,0.005,0.1); //dGeomSetBody(geom_cube[0],body_cube[0]); 
}

void ICubSim::initLegJoints()
{
    if (actLegs == "off" && actTorso == "on")
    {
        dJointAttach (LLegjoints[0], legs, torso[0]);
        dJointSetHingeAnchor (LLegjoints[0], jP_leftLeg[5][1], elev + jP_leftLeg[5][2], jP_leftLeg[5][0]);//dJointSetHingeAnchor (LLegjoints[0], 0.0255, elev +0.468, -0.034);
        dJointSetHingeAxis (LLegjoints[0], 1.0, 0.0, 0.0);
        dJointSetHingeParam(LLegjoints[0],dParamLoStop, -2.7925);dJointSetHingeParam(LLegjoints[0],dParamHiStop, 2.7925);/**/

        dJointAttach (RLegjoints[0], legs, torso[0]);
        dJointSetHingeAnchor (RLegjoints[0], jP_rightLeg[5][1], elev + jP_rightLeg[5][2], jP_rightLeg[5][0]);//dJointSetHingeAnchor (RLegjoints[0], -0.0255, elev +0.468, -0.034);
        dJointSetHingeAxis (RLegjoints[0], 1.0, 0.0, 0.0);
        dJointSetHingeParam(RLegjoints[0],dParamLoStop, -0.0);dJointSetHingeParam(RLegjoints[0],dParamHiStop, 0.0);
    }
    else if (actLegs == "off" && actTorso == "off")
    {
        dJointAttach (Torsojoints[0], legs, body_torso);
        dJointSetHingeAnchor (Torsojoints[0], jP_rightLeg[5][1], elev + jP_rightLeg[5][2], 0.0);//dJointSetHingeAnchor (Torsojoints[0], -0.0255, elev +0.468, 0.0);
        dJointSetHingeAxis (Torsojoints[0], 0.0, 0.0, 1.0);
        dJointSetHingeParam(Torsojoints[0],dParamLoStop, -2.7925);dJointSetHingeParam(LLegjoints[0],dParamHiStop, 2.7925);
    }
    else
    {
        dJointAttach (LLegjoints[0], leftLeg[0], leftLeg[1]);
        dJointSetHingeAnchor (LLegjoints[0], jP_leftLeg[0][1], elev + jP_leftLeg[0][2], jP_leftLeg[0][0]);//dJointSetHingeAnchor (LLegjoints[0], 0.068, elev +0.031, -0.0235);
        dJointSetHingeAxis (LLegjoints[0], 0.0, 0.0, 1.0);
        dJointSetHingeParam(LLegjoints[0],dParamLoStop, -2.7925);dJointSetHingeParam(LLegjoints[0],dParamHiStop, 2.7925);

        dJointAttach (LLegjoints[1], leftLeg[1], leftLeg[2]);
        dJointSetHingeAnchor (LLegjoints[1], jP_leftLeg[1][1], elev + jP_leftLeg[1][2], jP_leftLeg[1][0]);//dJointSetHingeAnchor (LLegjoints[1], 0.068,elev +0.031, -0.034);
        dJointSetHingeAxis (LLegjoints[1], 1.0, 0.0, 0.0);
        dJointSetHingeParam(LLegjoints[1],dParamLoStop, -2.7925);dJointSetHingeParam(LLegjoints[1],dParamHiStop, 2.7925);

        dJointAttach (LLegjoints[2], leftLeg[2], leftLeg[3]);
        dJointSetHingeAnchor (LLegjoints[2], jP_leftLeg[2][1], elev + jP_leftLeg[2][2], jP_leftLeg[2][0]);//dJointSetHingeAnchor (LLegjoints[2], 0.068, elev +0.244, -0.034);
        dJointSetHingeAxis (LLegjoints[2], 1.0, 0.0, 0.0);
        dJointSetHingeParam(LLegjoints[2],dParamLoStop, -2.7925);dJointSetHingeParam(LLegjoints[2],dParamHiStop, 2.7925);

        dJointAttach (LLegjoints[3], leftLeg[3], leftLeg[4]);
        dJointSetHingeAnchor (LLegjoints[3], jP_leftLeg[3][1], elev + jP_leftLeg[3][2], jP_leftLeg[3][0]);//dJointSetHingeAnchor (LLegjoints[3], 0.068, elev +0.468, -0.034);
        dJointSetHingeAxis (LLegjoints[3], 0.0, 1.0, 0.0);
        dJointSetHingeParam(LLegjoints[3],dParamLoStop, -2.7925);dJointSetHingeParam(LLegjoints[3],dParamHiStop, 2.7925);

        dJointAttach (LLegjoints[4], leftLeg[4], leftLeg[5]);
        dJointSetHingeAnchor (LLegjoints[4], jP_leftLeg[4][1], elev + jP_leftLeg[4][2], jP_leftLeg[4][0]);//dJointSetHingeAnchor (LLegjoints[4], 0.068, elev +0.468, -0.034);
        dJointSetHingeAxis (LLegjoints[4], 0.0, 0.0, 1.0);
        dJointSetHingeParam(LLegjoints[4],dParamLoStop, -2.7925);dJointSetHingeParam(LLegjoints[4],dParamHiStop, 2.7925);

        if (actTorso == "off")
        {
            dJointAttach (LLegjoints[5], leftLeg[5], body_torso);
            dJointSetHingeAnchor (LLegjoints[5], jP_leftLeg[5][1], elev + jP_leftLeg[5][2], jP_leftLeg[5][0]);//dJointSetHingeAnchor (LLegjoints[5], 0.0255, elev +0.468, -0.034);
            dJointSetHingeAxis (LLegjoints[5], 1.0, 0.0, 0.0);
            dJointSetHingeParam(LLegjoints[5],dParamLoStop, -2.7925);dJointSetHingeParam(LLegjoints[5],dParamHiStop, 2.7925);
        }
        else
        {
            dJointAttach (LLegjoints[5], leftLeg[5], torso[0]);
            dJointSetHingeAnchor (LLegjoints[5], jP_leftLeg[5][1], elev + jP_leftLeg[5][2], jP_leftLeg[5][0]);//dJointSetHingeAnchor (LLegjoints[5], 0.0255, elev +0.468, -0.034);
            dJointSetHingeAxis (LLegjoints[5], 1.0, 0.0, 0.0);
            dJointSetHingeParam(LLegjoints[5],dParamLoStop, -2.7925);dJointSetHingeParam(LLegjoints[5],dParamHiStop, 2.7925);
        }
        //RIGHT LEG JOINTS
        dJointAttach (RLegjoints[0], rightLeg[0], rightLeg[1]);
        dJointSetHingeAnchor (RLegjoints[0], jP_rightLeg[0][1], elev + jP_rightLeg[0][2], jP_rightLeg[0][0]);//dJointSetHingeAnchor (RLegjoints[0], -0.068, elev +0.031, -0.0235);
        dJointSetHingeAxis (RLegjoints[0], 0.0, 0.0, 1.0);
        dJointSetHingeParam(RLegjoints[0],dParamLoStop, -2.7925);dJointSetHingeParam(RLegjoints[0],dParamHiStop, 2.7925);

        dJointAttach (RLegjoints[1], rightLeg[1], rightLeg[2]);
        dJointSetHingeAnchor (RLegjoints[1], jP_rightLeg[1][1], elev + jP_rightLeg[1][2], jP_rightLeg[1][0]);//dJointSetHingeAnchor (RLegjoints[1], -0.068,elev +0.031, -0.034);
        dJointSetHingeAxis (RLegjoints[1], 1.0, 0.0, 0.0);
        dJointSetHingeParam(RLegjoints[1],dParamLoStop, -2.7925);dJointSetHingeParam(RLegjoints[1],dParamHiStop, 2.7925);

        dJointAttach (RLegjoints[2], rightLeg[2], rightLeg[3]);
        dJointSetHingeAnchor (RLegjoints[2], jP_rightLeg[2][1], elev + jP_rightLeg[2][2], jP_rightLeg[2][0]);//dJointSetHingeAnchor (RLegjoints[2], -0.068, elev +0.244, -0.034);
        dJointSetHingeAxis (RLegjoints[2], 1.0, 0.0, 0.0);
        dJointSetHingeParam(RLegjoints[2],dParamLoStop, -2.7925);dJointSetHingeParam(RLegjoints[2],dParamHiStop, 2.7925);

        dJointAttach (RLegjoints[3], rightLeg[3], rightLeg[4]);
        dJointSetHingeAnchor (RLegjoints[3], jP_rightLeg[3][1], elev + jP_rightLeg[3][2], jP_rightLeg[3][0]);//dJointSetHingeAnchor (RLegjoints[3], -0.068, elev +0.468, -0.034);
        dJointSetHingeAxis (RLegjoints[3], 0.0, 1.0, 0.0);
        dJointSetHingeParam(RLegjoints[3],dParamLoStop, -2.7925);dJointSetHingeParam(RLegjoints[3],dParamHiStop, 2.7925);

        dJointAttach (RLegjoints[4], rightLeg[4], rightLeg[5]);
        dJointSetHingeAnchor (RLegjoints[4], jP_rightLeg[4][1], elev + jP_rightLeg[4][2], jP_rightLeg[4][0]);//dJointSetHingeAnchor (RLegjoints[4], -0.068, elev +0.468, -0.034);
        dJointSetHingeAxis (RLegjoints[4], 0.0, 0.0, 1.0);
        dJointSetHingeParam(RLegjoints[4],dParamLoStop, -2.7925);dJointSetHingeParam(RLegjoints[4],dParamHiStop, 2.7925);

        if (actTorso == "off")
        {
            dJointAttach (RLegjoints[5], rightLeg[5], body_torso);
            dJointSetHingeAnchor (RLegjoints[5], jP_rightLeg[5][1], elev + jP_rightLeg[5][2], jP_rightLeg[5][0]);//dJointSetHingeAnchor (RLegjoints[5], -0.0255, elev +0.468, -0.034);
            dJointSetHingeAxis (RLegjoints[5], 1.0, 0.0, 0.0);
            dJointSetHingeParam(RLegjoints[5],dParamLoStop, -2.7925);dJointSetHingeParam(RLegjoints[5],dParamHiStop, 2.7925);
        }
        else
        {
            dJointAttach (RLegjoints[5], rightLeg[5], torso[0]);
            dJointSetHingeAnchor (RLegjoints[5], jP_rightLeg[5][1], elev + jP_rightLeg[5][2], jP_rightLeg[5][0]);//dJointSetHingeAnchor (RLegjoints[5], -0.0255, elev +0.468, -0.034);
            dJointSetHingeAxis (RLegjoints[5], 1.0, 0.0, 0.0);
            dJointSetHingeParam(RLegjoints[5],dParamLoStop, -2.7925);dJointSetHingeParam(RLegjoints[5],dParamHiStop, 2.7925);
        }
    }
}

void ICubSim::initTorsoJoints(OdeParams &p)
{
    if (actTorso == "off")
    {
        dJointAttach (Torsojoints[3], body_torso, torso[4]);
        dJointSetHingeAnchor (Torsojoints[3], jP_torso[2][1], elev + jP_torso[2][2], jP_torso[2][0]);
        dJointSetHingeAxis (Torsojoints[3], 0.0, 1.0, 0.0);
        dJointSetHingeParam(Torsojoints[3],dParamLoStop, -2.7925);dJointSetHingeParam(Torsojoints[3],dParamHiStop, 2.7925);

        dJointAttach (Torsojoints[4], body_torso, torso[5]);
        dJointSetHingeAnchor (Torsojoints[4],  jP_torso[2][1], elev + jP_torso[2][2], jP_torso[2][0]);
        dJointSetHingeAxis (Torsojoints[4], 0.0, 1.0, 0.0);
        dJointSetHingeParam(Torsojoints[4],dParamLoStop, -2.7925);dJointSetHingeParam(Torsojoints[4],dParamHiStop, 2.7925);
        return;
    }

    dJointAttach (Torsojoints[0], torso[0], torso[1]);
    dJointAttach (Torsojoints[1], torso[1], torso[2]);
    dJointAttach (Torsojoints[2], torso[2], torso[3]);
    dJointAttach (Torsojoints[3], torso[3], torso[4]);
    dJointAttach (Torsojoints[4], torso[3], torso[5]);

    int i=0;
    for(int j=0; j<5; j++)
    {
        i = j>2 ? 2 : j;
        dJointSetHingeAnchor(Torsojoints[i], jP_torso[i][1], elev+jP_torso[i][2], jP_torso[i][0]);
        dJointSetHingeAxis(Torsojoints[i], jA_torso[i][0], jA_torso[i][1], jA_torso[i][2]);
    }

    /* Torso joints are in reserved order with respect to robotMotorGui
        Max  50  30  70
        Min -50 -30 -10
    */
    double safetyMargin = 0.1 * CTRL_DEG2RAD;   // margin added to the joint limits
    // this joint angle is the opposite of the real iCub joint angle so I had to invert the lower and upper bounds
    dJointSetHingeParam(Torsojoints[0],dParamLoStop, -70*CTRL_DEG2RAD-safetyMargin);
    dJointSetHingeParam(Torsojoints[0],dParamHiStop, 10*CTRL_DEG2RAD+safetyMargin);
    dJointSetHingeParam(Torsojoints[1],dParamLoStop, -30*CTRL_DEG2RAD-safetyMargin);
    dJointSetHingeParam(Torsojoints[2],dParamLoStop, -50*CTRL_DEG2RAD-safetyMargin);
    dJointSetHingeParam(Torsojoints[2],dParamHiStop, 50*CTRL_DEG2RAD+safetyMargin);
    dJointSetHingeParam(Torsojoints[3],dParamLoStop, -safetyMargin);
    dJointSetHingeParam(Torsojoints[3],dParamHiStop, safetyMargin);
    dJointSetHingeParam(Torsojoints[4],dParamLoStop, -safetyMargin);
    dJointSetHingeParam(Torsojoints[4],dParamHiStop, safetyMargin);

    for(int i=0;i<5;i++)
    {
        dJointSetHingeParam(Torsojoints[i], dParamFudgeFactor, p.fudgeFactor);
        dJointSetHingeParam(Torsojoints[i], dParamStopCFM, p.stopCFM);
        dJointSetHingeParam(Torsojoints[i], dParamStopERP, p.stopERP);
        dJointSetHingeParam(Torsojoints[i], dParamCFM, p.jointCFM);
        dJointSetHingeParam(Torsojoints[i], dParamBounce, p.jointStopBouncyness);
    }
}

void ICubSim::initLeftArmJoints(OdeParams &p)
{
    if (actLArm == "off")
    {
        dJointAttach (LAjoints[0], torso[4], larm);
        dJointSetHingeAnchor (LAjoints[0],   jP_leftArm[0][1], elev + jP_leftArm[0][2], jP_leftArm[0][0]);
        dJointSetHingeAxis (LAjoints[0], jA_leftArm[0][0], jA_leftArm[0][1], jA_leftArm[0][2]);
        //the angle has to be less than PI (180°) in order to be effective.....230° can not be reached 
        dJointSetHingeParam(LAjoints[0],dParamLoStop, -0.0);
        dJointSetHingeParam(LAjoints[0],dParamHiStop, 0.0);
        return;
    }

    dJointAttach (LAjoints[0], torso[4], body[0]);//joint left clavicule and left shoulder1
    dJointAttach (LAjoints[1], body[0], body[2]); //joint left shoulder1 and left shoulder2
    dJointAttach (LAjoints[2], body[2], body[4]); //joint left shoulder1 and left upper arm
    dJointAttach (LAjoints[3], body[4], body[6]); //joint left upper arm and left elbow mechanism
    dJointAttach (LAjoints[4], body[6], body[8]); //joint left elbow mechanism and left lower arm

    for(int i=0; i<5; i++)
    {
        dJointSetHingeAnchor(LAjoints[i], jP_leftArm[i][1], elev+jP_leftArm[i][2], jP_leftArm[i][0]);
        dJointSetHingeAxis(LAjoints[i], jA_leftArm[i][0], jA_leftArm[i][1], jA_leftArm[i][2]);
    }

    double safetyMargin = 0.1 * CTRL_DEG2RAD;   // margin added to the joint limits
    dJointSetHingeParam(LAjoints[0],dParamLoStop, -10.0*CTRL_DEG2RAD - safetyMargin);
    dJointSetHingeParam(LAjoints[0],dParamHiStop, 95.0*CTRL_DEG2RAD + safetyMargin);
    dJointSetHingeParam(LAjoints[1],dParamLoStop, -160.8*CTRL_DEG2RAD-safetyMargin);
    dJointSetHingeParam(LAjoints[1],dParamHiStop, 1.0+safetyMargin);
    dJointSetHingeParam(LAjoints[2],dParamLoStop,  -52.0*CTRL_DEG2RAD-safetyMargin);
    dJointSetHingeParam(LAjoints[2],dParamHiStop, 80.0*CTRL_DEG2RAD+safetyMargin);
    dJointSetHingeParam(LAjoints[3],dParamLoStop, -1.0*CTRL_DEG2RAD-safetyMargin);
    dJointSetHingeParam(LAjoints[3],dParamHiStop, 106.0*CTRL_DEG2RAD+safetyMargin);
    dJointSetHingeParam(LAjoints[4],dParamLoStop, -90.0*CTRL_DEG2RAD-safetyMargin);
    dJointSetHingeParam(LAjoints[4],dParamHiStop, 90.5*CTRL_DEG2RAD+safetyMargin);
   
    for(int i=0;i<5;i++)
    {
        dJointSetHingeParam(LAjoints[i], dParamFudgeFactor, p.fudgeFactor);
        dJointSetHingeParam(LAjoints[i], dParamStopCFM, p.stopCFM);
        dJointSetHingeParam(LAjoints[i], dParamStopERP, p.stopERP);
        dJointSetHingeParam(LAjoints[i], dParamCFM, p.jointCFM);
        dJointSetHingeParam(LAjoints[i], dParamBounce, p.jointStopBouncyness);
    }
}

void ICubSim::initLeftHandJoints()
{
    if (actLArm == "off" && actLHand == "off")
    {
        dJointAttach (LAjoints[5],larm,l_hand); //joint Universal left lower arm and left hand
        dJointSetUniversalAnchor (LAjoints[5],   jP_leftArm[5][1], elev + jP_leftArm[5][2], jP_leftArm[5][0]);//dJointSetUniversalAnchor (LAjoints[5],   0.117, elev +0.4735, -0.026);
        dJointSetUniversalAxis1 (LAjoints[5],0, 0, 1);dJointSetUniversalAxis2 (LAjoints[5], 1,0,0);
        dJointSetUniversalParam(LAjoints[5], dParamLoStop, -2.7925);dJointSetUniversalParam(LAjoints[5], dParamHiStop, 2.7925);
        dJointSetUniversalParam(LAjoints[5], dParamLoStop2, -2.7925);dJointSetUniversalParam(LAjoints[5], dParamHiStop2, 2.7925);
        return;
    }
    
    if(actLArm == "on" && actLHand == "off")
    {
        dJointAttach (LAjoints[5],body[8],l_hand); //joint Universal left lower arm and left hand
        dJointSetUniversalAnchor (LAjoints[5],   jP_leftArm[5][1], elev + jP_leftArm[5][2], jP_leftArm[5][0]);//dJointSetUniversalAnchor (LAjoints[5],   0.117, elev +0.4735, -0.026);
        dJointSetUniversalAxis1 (LAjoints[5],0,0, 1);dJointSetUniversalAxis2 (LAjoints[5], 1,0,0);
        dJointSetUniversalParam(LAjoints[5], dParamLoStop, -2.7925);dJointSetUniversalParam(LAjoints[5], dParamHiStop, 2.7925);
        dJointSetUniversalParam(LAjoints[5], dParamLoStop2, -2.7925);dJointSetUniversalParam(LAjoints[5], dParamHiStop2, 2.7925);
        return;
    }

    dBodyID temp;
    if (actLArm == "off")
        temp = larm;
    else
        temp = body[8];
    //LEFT HAND AND FINGER JOINTS
    dJointAttach (LAjoints[5],temp,body[10]); //joint Universal left lower arm and left hand
    dJointSetUniversalAnchor (LAjoints[5],   jP_leftArm[5][1], elev + jP_leftArm[5][2], jP_leftArm[5][0]);//0.117, elev +0.4735, -0.026);//dJointSetUniversalAnchor (LAjoints[5],   0.117, elev +0.4735, -0.026);
    dJointSetUniversalAxis1 (LAjoints[5],0,0, 1);dJointSetUniversalAxis2 (LAjoints[5], 1,0,0);
    dJointSetUniversalParam(LAjoints[5], dParamLoStop, -2.7925);dJointSetUniversalParam(LAjoints[5], dParamHiStop, 2.7925);
    dJointSetUniversalParam(LAjoints[5], dParamLoStop2, -2.7925);dJointSetUniversalParam(LAjoints[5], dParamHiStop2, 2.7925);

    dJointAttach (LAjoints[6], body[10],body[12]); //joint hand index finger1 
    dJointSetHingeAnchor (LAjoints[6],  jP_leftArm[7][1], elev +jP_leftArm[6][2]-0.5*fabs(jP_leftArm[7][2]-jP_leftArm[6][2])-0.034, jP_leftArm[7][0]+0.025);//dJointSetHingeAnchor (LAjoints[6],  0.117, elev +0.4055, -0.00325);
    dJointSetHingeAxis (LAjoints[6], 1.0, 0.0, 0.0);
    dJointSetHingeParam(LAjoints[6],dParamLoStop, -2.7925);dJointSetHingeParam(LAjoints[6],dParamHiStop, 2.7925);

    dJointAttach (LAjoints[7], body[10], body[13]); //joint hand middle finger1
    dJointSetHingeAnchor (LAjoints[7],  jP_leftArm[7][1], elev +jP_leftArm[6][2]-0.5*fabs(jP_leftArm[7][2]-jP_leftArm[6][2])-0.034, jP_leftArm[7][0]+0.01);//dJointSetHingeAnchor (LAjoints[7],  0.117, elev +0.4055, -0.0195);
    dJointSetHingeAxis (LAjoints[7], 1.0, 0.0, 0.0);
    dJointSetHingeParam(LAjoints[7],dParamLoStop, -2.7925);dJointSetHingeParam(LAjoints[7],dParamHiStop, 2.7925);

    dJointAttach (LAjoints[8], body[10],lhandfingers0); //joint hand ring finger1 lhandfingers0
    dJointSetHingeAnchor (LAjoints[8],  jP_leftArm[7][1],elev +jP_leftArm[6][2]-0.5*fabs(jP_leftArm[7][2]-jP_leftArm[6][2])-0.034, jP_leftArm[7][0]-0.016125);//dJointSetHingeAnchor (LAjoints[8],  0.117, elev +0.4055, -0.043875);
    dJointSetHingeAxis (LAjoints[8], 1.0, 0.0, 0.0);
    dJointSetHingeParam(LAjoints[8],dParamLoStop, -2.7925);dJointSetHingeParam(LAjoints[8],dParamHiStop, 2.7925);

    dJointAttach (LAjoints[10], body[12],body[16]); //index finger1 index finger2   
    dJointSetHingeAnchor (LAjoints[10],   jP_leftArm[7][1], elev +jP_leftArm[6][2]-0.5*fabs(jP_leftArm[7][2]-jP_leftArm[6][2])-0.046, jP_leftArm[7][0]+0.025);//dJointSetHingeAnchor (LAjoints[10],   0.117,elev + 0.393, -0.00325);
    dJointSetHingeAxis (LAjoints[10], 0.0, 0.0, 1.0);
    dJointSetHingeParam(LAjoints[10],dParamLoStop, -2.7925);dJointSetHingeParam(LAjoints[10],dParamHiStop, 2.7925);

    dJointAttach (LAjoints[11], body[13], body[17]); //middle finger1 middle finger2 
    dJointSetHingeAnchor (LAjoints[11],   jP_leftArm[7][1], elev +jP_leftArm[6][2]-0.5*fabs(jP_leftArm[7][2]-jP_leftArm[6][2])-0.046, jP_leftArm[7][0]+0.01);
    dJointSetHingeAxis (LAjoints[11], 0.0, 0.0, 1.0);
    dJointSetHingeParam(LAjoints[11],dParamLoStop, -2.7925);dJointSetHingeParam(LAjoints[11],dParamHiStop, 2.7925);

    dJointAttach (LAjoints[12], lhandfingers0,lhandfingers1); //ring finger1 ring finger2
    dJointSetHingeAnchor (LAjoints[12],   jP_leftArm[7][1],elev +jP_leftArm[6][2]-0.5*fabs(jP_leftArm[7][2]-jP_leftArm[6][2])-0.046, jP_leftArm[7][0]-0.016125);
    dJointSetHingeAxis (LAjoints[12], 0.0, 0.0, 1.0);
    dJointSetHingeParam(LAjoints[12],dParamLoStop, -2.7925);dJointSetHingeParam(LAjoints[12],dParamHiStop, 2.7925);

    dJointAttach (LAjoints[14], body[16],body[20]); //index finger2 index finger3   
    dJointSetHingeAnchor (LAjoints[14],   jP_leftArm[7][1], elev +jP_leftArm[6][2]-0.5*fabs(jP_leftArm[7][2]-jP_leftArm[6][2])-0.072, jP_leftArm[7][0]+0.025);
    dJointSetHingeAxis (LAjoints[14], 0.0, 0.0, 1.0);
    dJointSetHingeParam(LAjoints[14],dParamLoStop, -2.7925);dJointSetHingeParam(LAjoints[14],dParamHiStop, 2.7925);

    dJointAttach (LAjoints[15], body[17], body[21]); //middle finger2 middle finger3 
    dJointSetHingeAnchor (LAjoints[15],   jP_leftArm[7][1], elev +jP_leftArm[6][2]-0.5*fabs(jP_leftArm[7][2]-jP_leftArm[6][2])-0.074, jP_leftArm[7][0]+0.01);
    dJointSetHingeAxis (LAjoints[15], 0.0, 0.0, 1.0);
    dJointSetHingeParam(LAjoints[15],dParamLoStop, -2.7925);dJointSetHingeParam(LAjoints[15],dParamHiStop, 2.7925);

    dJointAttach (LAjoints[16], lhandfingers1,lhandfingers2); //ring finger2 ring finger3
    dJointSetHingeAnchor (LAjoints[16],   jP_leftArm[7][1],elev +jP_leftArm[6][2]-0.5*fabs(jP_leftArm[7][2]-jP_leftArm[6][2])-0.070, jP_leftArm[7][0]-0.016125);
    dJointSetHingeAxis (LAjoints[16], 0.0, 0.0, 1.0);
    dJointSetHingeParam(LAjoints[16],dParamLoStop, -2.7925);dJointSetHingeParam(LAjoints[16],dParamHiStop, 2.7925);

    dJointAttach (LAjoints[18], body[20],body[24]); //index finger3 index finger4   
    dJointSetHingeAnchor (LAjoints[18],   jP_leftArm[7][1], elev +jP_leftArm[6][2]-0.5*fabs(jP_leftArm[7][2]-jP_leftArm[6][2])-0.094, jP_leftArm[7][0]+0.025);
    dJointSetHingeAxis (LAjoints[18], 0.0, 0.0, 1.0);
    dJointSetHingeParam(LAjoints[18],dParamLoStop, -2.7925);dJointSetHingeParam(LAjoints[18],dParamHiStop, 2.7925);

    dJointAttach (LAjoints[19], body[21],body[25]); //middle finger3 middle finger4 
    dJointSetHingeAnchor (LAjoints[19],   jP_leftArm[7][1], elev +jP_leftArm[6][2]-0.5*fabs(jP_leftArm[7][2]-jP_leftArm[6][2])-0.098, jP_leftArm[7][0]+0.01);
    dJointSetHingeAxis (LAjoints[19], 0.0, 0.0, 1.0);
    dJointSetHingeParam(LAjoints[19],dParamLoStop, -2.7925);dJointSetHingeParam(LAjoints[19],dParamHiStop, 2.7925);

    dJointAttach (LAjoints[20], lhandfingers2,lhandfingers3); //ring finger3 ring finger4
    dJointSetHingeAnchor (LAjoints[20],   jP_leftArm[7][1],elev +jP_leftArm[6][2]-0.5*fabs(jP_leftArm[7][2]-jP_leftArm[6][2])-0.0905, jP_leftArm[7][0]-0.016125);
    dJointSetHingeAxis (LAjoints[20], 0.0, 0.0, 1.0);
    dJointSetHingeParam(LAjoints[20],dParamLoStop, -2.7925);dJointSetHingeParam(LAjoints[20],dParamHiStop, 2.7925);

    //thumb
    /*dJointAttach (LAjoints[22],body[10],body[28]);//left hand thumb1 
    dJointSetHingeAnchor(LAjoints[22], 0.117,elev + 0.455,0.006);
    dJointSetHingeAxis(LAjoints[22],0,1,-0.5); 
    dJointSetHingeParam(L
    Ajoints[22],  dParamLoStop, (dReal)-2.7925);dJointSetHingeParam(LAjoints[22],  dParamHiStop, (dReal) 2.7925);
    */
    dJointAttach (LAjoints[22],body[10],body[28]); //joint Universal left lower arm and left hand 
    dJointSetUniversalAnchor (LAjoints[22],  jP_leftArm[7][1], elev+ jP_leftArm[6][2]-0.045/*fabs(jP_leftArm[7][2]-jP_leftArm[6][2])+0.016*/,jP_leftArm[7][0]+0.045 - 0.013);
    dJointSetUniversalAxis1 (LAjoints[22],0,1.5,-0.5);dJointSetUniversalAxis2 (LAjoints[22], 1,0,0);
    dJointSetUniversalParam(LAjoints[22], dParamLoStop, -2.7925);dJointSetUniversalParam(LAjoints[22], dParamHiStop, 2.7925);
    dJointSetUniversalParam(LAjoints[22], dParamLoStop2, -2.7925);dJointSetUniversalParam(LAjoints[22], dParamHiStop2, 2.7925);

    dJointAttach (LAjoints[23],body[28],body[29]);//left thumb1 and thumb2
    dJointSetHingeAnchor(LAjoints[23], jP_leftArm[7][1], elev +jP_leftArm[6][2]-0.045/*fabs(jP_leftArm[7][2]-jP_leftArm[6][2])+0.016*/,jP_leftArm[7][0]+0.069 - 0.011);
    dJointSetHingeAxis(LAjoints[23],0.0,0.5,0); 
    dJointSetHingeParam(LAjoints[23],  dParamLoStop, (dReal)-2.7925);dJointSetHingeParam(LAjoints[23],  dParamHiStop, (dReal) 2.7925);

    dJointAttach (LAjoints[24],body[29],body[30]);//left thumb2 and thumb3
    dJointSetHingeAnchor(LAjoints[24],jP_leftArm[7][1], elev +jP_leftArm[6][2]-0.045/*fabs(jP_leftArm[7][2]-jP_leftArm[6][2])+0.016*/,jP_leftArm[7][0]+0.088 - 0.008);
    dJointSetHingeAxis(LAjoints[24],0.0,0.5,0); 
    dJointSetHingeParam(LAjoints[24],  dParamLoStop, (dReal)-2.7925);dJointSetHingeParam(LAjoints[24],  dParamHiStop, (dReal) 2.7925);
}

void ICubSim::initRightArmJoints(OdeParams &p)
{
    if (actRArm == "off")
    {
        dJointAttach (RAjoints[0], torso[5], rarm);
        dJointSetHingeAnchor (RAjoints[0],   jP_rightArm[0][1], elev + jP_rightArm[0][2], jP_rightArm[0][0]);
        dJointSetHingeAxis (RAjoints[0], jA_rightArm[0][0], jA_rightArm[0][1], jA_rightArm[0][2]);
        //the angle has to be less than PI (180°) in order to be effective.....230° can not be reached 
        dJointSetHingeParam(RAjoints[0],dParamLoStop, -0.0);
        dJointSetHingeParam(RAjoints[0],dParamHiStop, 0.0);
        return;
    }

    dJointAttach (RAjoints[0], torso[5], body[1]);//joint right clavicule and right shoulder1
    dJointAttach (RAjoints[1],  body[1], body[3]);//joint right shoulder1 and left shoulder2
    dJointAttach (RAjoints[2], body[3], body[5]); //joint right shoulder1 and right upper arm
    dJointAttach (RAjoints[3], body[5], body[7]); //joint right upper arm and right elbow mechanism
    dJointAttach (RAjoints[4], body[7], body[9]); //joint right elbow mechanism and right lower arm

    for(int i=0; i<5; i++)
    {
        dJointSetHingeAnchor(RAjoints[i], jP_rightArm[i][1], elev+jP_rightArm[i][2], jP_rightArm[i][0]);
        dJointSetHingeAxis(RAjoints[i], jA_rightArm[i][0], jA_rightArm[i][1], jA_rightArm[i][2]);
    }

    //Max     10   160.8  80     106      90       0     40      60    90      90      90       90       90     90      90   115
    //Min    -95   0     -37    15.5     -90     -90    -20      0     0       0       0        0        0      0       0     0      
    double safetyMargin = 0.1 * CTRL_DEG2RAD;   // margin added to the joint limits
    dJointSetHingeParam(RAjoints[0],dParamLoStop, -10.0*CTRL_DEG2RAD-safetyMargin);
    dJointSetHingeParam(RAjoints[0],dParamHiStop, 95.0*CTRL_DEG2RAD+safetyMargin);
    dJointSetHingeParam(RAjoints[1],dParamLoStop, -1.0*CTRL_DEG2RAD-safetyMargin); // the lower limit should be 0 but it gives problem
    dJointSetHingeParam(RAjoints[1],dParamHiStop, 160.8*CTRL_DEG2RAD+safetyMargin);//180° cannot be fully reached have to make with 179.4°
    dJointSetHingeParam(RAjoints[2],dParamLoStop,  -80.0*CTRL_DEG2RAD-safetyMargin);
    dJointSetHingeParam(RAjoints[2],dParamHiStop, 52.0*CTRL_DEG2RAD+safetyMargin);
    dJointSetHingeParam(RAjoints[3],dParamLoStop, -1.0*CTRL_DEG2RAD-safetyMargin); // lower limit should be 15° but it gives problem
    dJointSetHingeParam(RAjoints[3],dParamHiStop, 106.0*CTRL_DEG2RAD+safetyMargin);
    dJointSetHingeParam(RAjoints[4],dParamLoStop, -90.5*CTRL_DEG2RAD-safetyMargin);
    dJointSetHingeParam(RAjoints[4],dParamHiStop, 90.0*CTRL_DEG2RAD+safetyMargin);

    for(int i=0;i<5;i++)
    {
        dJointSetHingeParam(RAjoints[i], dParamFudgeFactor, p.fudgeFactor);
        dJointSetHingeParam(RAjoints[i], dParamStopCFM, p.stopCFM);
        dJointSetHingeParam(RAjoints[i], dParamStopERP, p.stopERP);
        dJointSetHingeParam(RAjoints[i], dParamCFM, p.jointCFM);
        dJointSetHingeParam(RAjoints[i], dParamBounce, p.jointStopBouncyness);
    }
}

void ICubSim::initRightHandJoints()
{
    if (actRArm == "off" && actRHand == "off")
    {
        dJointAttach (RAjoints[5],rarm,r_hand); //joint Universal left lower arm and left hand
        dJointSetUniversalAnchor (RAjoints[5],   jP_rightArm[5][1], elev + jP_rightArm[5][2], jP_rightArm[5][0]);//dJointSetUniversalAnchor (RAjoints[5],   -0.117,elev + 0.4735, -0.026);
        dJointSetUniversalAxis1 (RAjoints[5],0,0, 1);dJointSetUniversalAxis2 (RAjoints[5], 1,0,0);
        dJointSetUniversalParam(RAjoints[5], dParamLoStop, -2.7925);dJointSetUniversalParam(RAjoints[5], dParamHiStop, 2.7925);
        dJointSetUniversalParam(RAjoints[5], dParamLoStop2, -2.7925);dJointSetUniversalParam(RAjoints[5], dParamHiStop2, 2.7925);
        return;
    }
    if(actRArm == "on" && actRHand =="off")
    {
        dJointAttach (RAjoints[5],body[9],r_hand); //joint Universal left lower arm and left hand
        dJointSetUniversalAnchor (RAjoints[5],   jP_rightArm[5][1], elev + jP_rightArm[5][2], jP_rightArm[5][0]);//dJointSetUniversalAnchor (RAjoints[5],   -0.117,elev + 0.4735, -0.026);
        dJointSetUniversalAxis1 (RAjoints[5],0,0, 1);dJointSetUniversalAxis2 (RAjoints[5], 1,0,0);
        dJointSetUniversalParam(RAjoints[5], dParamLoStop, -2.7925);dJointSetUniversalParam(RAjoints[5], dParamHiStop, 2.7925);
        dJointSetUniversalParam(RAjoints[5], dParamLoStop2, -2.7925);dJointSetUniversalParam(RAjoints[5], dParamHiStop2, 2.7925);
        return;
    }

    dBodyID temp1;
    if (actRArm =="off"){
        temp1 = rarm;
    }else{
        temp1 = body[9];
    }
    //CREATE ALL RIGHT HAND + FINGER JOINTS
    dJointAttach (RAjoints[5],temp1,body[11]); //joint Universal left lower arm and left hand
    dJointSetUniversalAnchor (RAjoints[5],   jP_rightArm[5][1], elev + jP_rightArm[5][2], jP_rightArm[5][0]);//-0.117, elev +0.4735, -0.026);
    dJointSetUniversalAxis1 (RAjoints[5],0,0, 1);dJointSetUniversalAxis2 (RAjoints[5], 1,0,0);
    dJointSetUniversalParam(RAjoints[5], dParamLoStop, -2.7925);dJointSetUniversalParam(RAjoints[5], dParamHiStop, 2.7925);
    dJointSetUniversalParam(RAjoints[5], dParamLoStop2, -2.7925);dJointSetUniversalParam(RAjoints[5], dParamHiStop2, 2.7925);

    dJointAttach (RAjoints[6], body[11],body[31]); //joint hand index finger1 
    dJointSetHingeAnchor (RAjoints[6], jP_rightArm[7][1], elev +jP_rightArm[6][2]-0.5*fabs(jP_rightArm[7][2]-jP_rightArm[6][2])-0.034, jP_rightArm[7][0]+0.025);
    dJointSetHingeAxis (RAjoints[6], 1.0, 0.0, 0.0);
    dJointSetHingeParam(RAjoints[6],dParamLoStop, -2.7925);dJointSetHingeParam(RAjoints[6],dParamHiStop, 2.7925);

    dJointAttach (RAjoints[7], body[11], body[32]); //joint hand middle finger1
    dJointSetHingeAnchor (RAjoints[7],  jP_rightArm[7][1], elev +jP_rightArm[6][2]-0.5*fabs(jP_rightArm[7][2]-jP_rightArm[6][2])-0.034, jP_rightArm[7][0]+0.01);
    dJointSetHingeAxis (RAjoints[7], 1.0, 0.0, 0.0);
    dJointSetHingeParam(RAjoints[7],dParamLoStop, -2.7925);dJointSetHingeParam(RAjoints[7],dParamHiStop, 2.7925);

    dJointAttach (RAjoints[8], body[11],rhandfingers0); //joint hand ring finger1 lhandfingers0
    dJointSetHingeAnchor (RAjoints[8],  jP_rightArm[7][1],elev +jP_rightArm[6][2]-0.5*fabs(jP_rightArm[7][2]-jP_rightArm[6][2])-0.034, jP_rightArm[7][0]-0.016125);
    dJointSetHingeAxis (RAjoints[8], 1.0, 0.0, 0.0);
    dJointSetHingeParam(RAjoints[8],dParamLoStop, -2.7925);dJointSetHingeParam(RAjoints[8],dParamHiStop, 2.7925);

    dJointAttach (RAjoints[10], body[31],body[35]); //index finger1 index finger2   
    dJointSetHingeAnchor (RAjoints[10],   jP_rightArm[7][1], elev +jP_rightArm[6][2]-0.5*fabs(jP_rightArm[7][2]-jP_rightArm[6][2])-0.046, jP_rightArm[7][0]+0.025);
    dJointSetHingeAxis (RAjoints[10], 0.0, 0.0, 1.0);
    dJointSetHingeParam(RAjoints[10],dParamLoStop, -2.7925);dJointSetHingeParam(RAjoints[10],dParamHiStop, 2.7925);

    dJointAttach (RAjoints[11], body[32], body[36]); //middle finger1 middle finger2 
    dJointSetHingeAnchor (RAjoints[11],   jP_rightArm[7][1], elev +jP_rightArm[6][2]-0.5*fabs(jP_rightArm[7][2]-jP_rightArm[6][2])-0.046, jP_rightArm[7][0]+0.01);
    dJointSetHingeAxis (RAjoints[11], 0.0, 0.0, 1.0);
    dJointSetHingeParam(RAjoints[11],dParamLoStop, -2.7925);dJointSetHingeParam(RAjoints[11],dParamHiStop, 2.7925);

    dJointAttach (RAjoints[12], rhandfingers0,rhandfingers1); //ring finger1 ring finger2
    dJointSetHingeAnchor (RAjoints[12],   jP_rightArm[7][1],elev +jP_rightArm[6][2]-0.5*fabs(jP_rightArm[7][2]-jP_rightArm[6][2])-0.046, jP_rightArm[7][0]-0.016125);
    dJointSetHingeAxis (RAjoints[12], 0.0, 0.0, 1.0);
    dJointSetHingeParam(RAjoints[12],dParamLoStop, -2.7925);dJointSetHingeParam(RAjoints[12],dParamHiStop, 2.7925);

    dJointAttach (RAjoints[14], body[35],body[39]); //index finger2 index finger3   
    dJointSetHingeAnchor (RAjoints[14],   jP_rightArm[7][1], elev +jP_rightArm[6][2]-0.5*fabs(jP_rightArm[7][2]-jP_rightArm[6][2])-0.072, jP_rightArm[7][0]+0.025);
    dJointSetHingeAxis (RAjoints[14], 0.0, 0.0, 1.0);
    dJointSetHingeParam(RAjoints[14],dParamLoStop, -2.7925);dJointSetHingeParam(RAjoints[14],dParamHiStop, 2.7925);

    dJointAttach (RAjoints[15], body[36], body[40]); //middle finger2 middle finger3 
    dJointSetHingeAnchor (RAjoints[15],   jP_rightArm[7][1], elev +jP_rightArm[6][2]-0.5*fabs(jP_rightArm[7][2]-jP_rightArm[6][2])-0.074, jP_rightArm[7][0]+0.01);
    dJointSetHingeAxis (RAjoints[15], 0.0, 0.0, 1.0);
    dJointSetHingeParam(RAjoints[15],dParamLoStop, -2.7925);dJointSetHingeParam(RAjoints[15],dParamHiStop, 2.7925);

    dJointAttach (RAjoints[16], rhandfingers1,rhandfingers2); //ring finger2 ring finger3
    dJointSetHingeAnchor (RAjoints[16],   jP_rightArm[7][1],elev +jP_rightArm[6][2]-0.5*fabs(jP_rightArm[7][2]-jP_rightArm[6][2])-0.070, jP_rightArm[7][0]-0.016125);
    dJointSetHingeAxis (RAjoints[16], 0.0, 0.0, 1.0);
    dJointSetHingeParam(RAjoints[16],dParamLoStop, -2.7925);dJointSetHingeParam(RAjoints[16],dParamHiStop, 2.7925);

    dJointAttach (RAjoints[18], body[39],body[43]); //index finger3 index finger4   
    dJointSetHingeAnchor (RAjoints[18],   jP_rightArm[7][1], elev +jP_rightArm[6][2]-0.5*fabs(jP_rightArm[7][2]-jP_rightArm[6][2])-0.094, jP_rightArm[7][0]+0.025);
    dJointSetHingeAxis (RAjoints[18], 0.0, 0.0, 1.0);
    dJointSetHingeParam(RAjoints[18],dParamLoStop, -2.7925);dJointSetHingeParam(RAjoints[18],dParamHiStop, 2.7925);

    dJointAttach (RAjoints[19], body[40],body[44]); //middle finger3 middle finger4 
    dJointSetHingeAnchor (RAjoints[19],   jP_rightArm[7][1], elev +jP_rightArm[6][2]-0.5*fabs(jP_rightArm[7][2]-jP_rightArm[6][2])-0.098, jP_rightArm[7][0]+0.01);
    dJointSetHingeAxis (RAjoints[19], 0.0, 0.0, 1.0);
    dJointSetHingeParam(RAjoints[19],dParamLoStop, -2.7925);dJointSetHingeParam(RAjoints[19],dParamHiStop, 2.7925);

    dJointAttach (RAjoints[20], rhandfingers2,rhandfingers3); //ring finger3 ring finger4
    dJointSetHingeAnchor (RAjoints[20],   jP_rightArm[7][1],elev +jP_rightArm[6][2]-0.5*fabs(jP_rightArm[7][2]-jP_rightArm[6][2])-0.0905, jP_rightArm[7][0]-0.016125);
    dJointSetHingeAxis (RAjoints[20], 0.0, 0.0, 1.0);
    dJointSetHingeParam(RAjoints[20],dParamLoStop, -2.7925);dJointSetHingeParam(RAjoints[20],dParamHiStop, 2.7925);

    //thumb
    /*	dJointAttach (RAjoints[22],body[11],body[47]);//left hand thumb1 
    dJointSetHingeAnchor(RAjoints[22], -0.117,elev + 0.455,0.006);
    dJointSetHingeAxis(RAjoints[22],0,1,-0.5); 
    dJointSetHingeParam(RAjoints[22],  dParamLoStop, (dReal)-2.7925);dJointSetHingeParam(RAjoints[22],  dParamHiStop, (dReal) 2.7925);
    */

    dJointAttach (RAjoints[22],body[11],body[47]); //joint Universal left lower arm and left hand
    dJointSetUniversalAnchor (RAjoints[22],  jP_rightArm[7][1],elev + jP_rightArm[6][2]-0.045/*fabs(jP_rightArm[7][2]-jP_rightArm[6][2])+0.016*/, jP_rightArm[7][0]+0.045 -0.013);
    dJointSetUniversalAxis1 (RAjoints[22],0,1.5,-0.5);dJointSetUniversalAxis2 (RAjoints[22], 1,0,0);
    dJointSetUniversalParam(RAjoints[22], dParamLoStop, -2.7925);dJointSetUniversalParam(RAjoints[22], dParamHiStop, 2.7925);
    dJointSetUniversalParam(RAjoints[22], dParamLoStop2, -2.7925);dJointSetUniversalParam(RAjoints[22], dParamHiStop2, 2.7925);

    dJointAttach (RAjoints[23],body[47],body[48]);//left thumb1 and thumb2
    dJointSetHingeAnchor(RAjoints[23], jP_rightArm[7][1], elev + jP_rightArm[6][2]-0.045/*fabs(jP_rightArm[7][2]-jP_rightArm[6][2])+0.016*/, jP_rightArm[7][0]+0.069 -0.011);
    dJointSetHingeAxis(RAjoints[23],0,0.5,0); 
    dJointSetHingeParam(RAjoints[23],  dParamLoStop, (dReal)-2.7925);dJointSetHingeParam(RAjoints[23],  dParamHiStop, (dReal) 2.7925);

    dJointAttach (RAjoints[24],body[48],body[49]);//left thumb2 and thumb3
    dJointSetHingeAnchor(RAjoints[24], jP_rightArm[7][1], elev +jP_rightArm[6][2]-0.045/*fabs(jP_rightArm[7][2]-jP_rightArm[6][2])+0.016*/, jP_leftArm[7][0]+0.088 -0.008);
    dJointSetHingeAxis(RAjoints[24],0,0.5,0); 
    dJointSetHingeParam(RAjoints[24],  dParamLoStop, (dReal)-2.7925);dJointSetHingeParam(RAjoints[24],  dParamHiStop, (dReal) 2.7925);
}

void ICubSim::initHeadJoints()
{
    //HEAD JOINTS
    if (actTorso == "off" && actHead == "off"){
        dJointAttach (Hjoints[0], body_torso, head);
        dJointSetHingeAnchor(Hjoints[0], jP_head[0][1], elev + jP_head[0][2], jP_head[0][0]);//dJointSetHingeAnchor(Hjoints[0], -0.0,elev + 0.815, -0.026);
        dJointSetHingeAxis(Hjoints[0],  1, 0, 0); 
        dJointSetHingeParam(Hjoints[0],  dParamLoStop, -0.0); dJointSetHingeParam(Hjoints[0],  dParamHiStop, 0.0);
    }else if(actTorso == "off" && actHead == "on"){
        dJointAttach (Hjoints[0],body_torso,neck[0]);//base + neck1
        dJointSetHingeAnchor(Hjoints[0], jP_head[0][1], elev + jP_head[0][2], jP_head[0][0]);//dJointSetHingeAnchor(Hjoints[0], -0.0, elev +0.815, -0.026);
        dJointSetHingeAxis(Hjoints[0],  1, 0, 0); 
        dJointSetHingeParam(Hjoints[0],  dParamLoStop, -2.7925); dJointSetHingeParam(Hjoints[0],  dParamHiStop, 2.7925);
    }else if (actTorso == "on" && actHead == "off"){
        dJointAttach (Hjoints[0], torso[3], head);
        dJointSetHingeAnchor(Hjoints[0], jP_head[0][1], elev + jP_head[0][2], jP_head[0][0]);//dJointSetHingeAnchor(Hjoints[0], -0.0,elev + 0.815, -0.026);
        dJointSetHingeAxis(Hjoints[0],  1, 0, 0); 
        dJointSetHingeParam(Hjoints[0],  dParamLoStop, -0.0); dJointSetHingeParam(Hjoints[0],  dParamHiStop, 0.0);
    }else{
        dJointAttach (Hjoints[0],torso[3],neck[0]);//base + neck1
        dJointSetHingeAnchor(Hjoints[0], jP_head[0][1], elev + jP_head[0][2], jP_head[0][0]);//dJointSetHingeAnchor(Hjoints[0], -0.0,elev + 0.815, -0.026);
        dJointSetHingeAxis(Hjoints[0],  1, 0, 0); 
        dJointSetHingeParam(Hjoints[0],  dParamLoStop, -2.7925); dJointSetHingeParam(Hjoints[0],  dParamHiStop, 2.7925);
    }
    if (actHead == "on"){
        dJointAttach (Hjoints[1],neck[0],neck[1]);//neck1 + neck2
        dJointSetHingeAnchor(Hjoints[1], jP_head[1][1], elev + jP_head[1][2], jP_head[1][0]);//dJointSetHingeAnchor(Hjoints[1], -0.0,elev + 0.845,  -0.026);
        dJointSetHingeAxis(Hjoints[1],  0, 0, 1); 
        dJointSetHingeParam(Hjoints[1],  dParamLoStop, -2.7925); dJointSetHingeParam(Hjoints[1],  dParamHiStop, 2.7925);

        dJointAttach (Hjoints[2],neck[1],head);//neck2 and head
        dJointSetHingeAnchor(Hjoints[2], jP_head[2][1], elev + jP_head[2][2], jP_head[2][0]);//dJointSetHingeAnchor(Hjoints[2], 0.00,elev + 0.86,-0.026);
        dJointSetHingeAxis(Hjoints[2], 0,1,0); 
        dJointSetHingeParam(Hjoints[2],  dParamLoStop, -2.7925);dJointSetHingeParam(Hjoints[2],  dParamHiStop, 2.7925);

        dJointAttach (Hjoints[3],head, eye);
        dJointSetHingeAnchor(Hjoints[3], jP_head[3][1], elev + jP_head[3][2], jP_head[3][0]);//dJointSetHingeAnchor(Hjoints[3], 0.0 ,elev +0.89, 0.038);
        dJointSetHingeAxis(Hjoints[3],  1, 0, 0); 
        dJointSetHingeParam(Hjoints[3],  dParamLoStop, -2.7925); dJointSetHingeParam(Hjoints[3],  dParamHiStop, 2.7925);

        dJointAttach (Hjoints[4], eye, leye);
        dJointSetHingeAnchor(Hjoints[4], jP_leftEye[1][1], elev + jP_leftEye[1][2], jP_leftEye[1][0]);//dJointSetHingeAnchor(Hjoints[4], 0.034 ,elev + 0.89, 0.038);
        dJointSetHingeAxis(Hjoints[4],  0, 1, 0); 
        dJointSetHingeParam(Hjoints[4],  dParamLoStop, -2.7925); dJointSetHingeParam(Hjoints[4],  dParamHiStop, 2.7925);

        dJointAttach (Hjoints[5], eye, reye);
        dJointSetHingeAnchor(Hjoints[5], jP_rightEye[1][1], elev + jP_rightEye[1][2], jP_rightEye[1][0]);//dJointSetHingeAnchor(Hjoints[5], -0.034 ,elev +0.89, 0.038);
        dJointSetHingeAxis(Hjoints[5],  0, 1, 0); 
        dJointSetHingeParam(Hjoints[5],  dParamLoStop, -2.7925); dJointSetHingeParam(Hjoints[5],  dParamHiStop, 2.7925);

        /*dJointAttach (Hjoints[6], topEyeLid, head);
        dJointSetHingeAnchor(Hjoints[6],  0.0, elev + 0.928,	0.035);
        dJointSetHingeAxis(Hjoints[6],  0, 1, 0); 
        dJointSetHingeParam(Hjoints[6],  dParamLoStop,-2.7925); dJointSetHingeParam(Hjoints[6],  dParamHiStop,2.7925);

        dJointAttach (Hjoints[7], bottomEyeLid, head);
        dJointSetHingeAnchor(Hjoints[7], 0.0, elev + 0.928,	0.035);
        dJointSetHingeAxis(Hjoints[7],  0, 1, 0); 
        dJointSetHingeParam(Hjoints[7],  dParamLoStop,-2.7925); dJointSetHingeParam(Hjoints[7],  dParamHiStop,2.7925);*/

    }
}

ICubSim::~ICubSim() {

    //destroy all geoms 
    if (actLegs == "off"){
        dGeomDestroy(l_leg0_geom); dGeomDestroy(l_leg1_geom); dGeomDestroy(l_leg2_geom); dGeomDestroy(l_leg3_geom);
        dGeomDestroy(l_leg4_geom); dGeomDestroy(l_leg5_geom); dGeomDestroy(l_leg6_geom); dGeomDestroy(l_leg7_geom);
        dGeomDestroy(r_leg0_geom); dGeomDestroy(r_leg1_geom); dGeomDestroy(r_leg2_geom); dGeomDestroy(r_leg3_geom); 
        dGeomDestroy(r_leg4_geom); dGeomDestroy(r_leg5_geom); dGeomDestroy(r_leg6_geom); dGeomDestroy(r_leg7_geom);
    }else{
        dGeomDestroy(leftLegGeom[0]);dGeomDestroy(leftLegGeom[1]);dGeomDestroy(leftLeg_2_1);dGeomDestroy(leftLeg_2_2);
        dGeomDestroy(leftLeg_3_1);dGeomDestroy(leftLeg_3_2);dGeomDestroy(leftLeg_4_1);dGeomDestroy(leftLeg_4_2);
        dGeomDestroy(leftLegGeom[5]);
        dGeomDestroy(rightLegGeom[0]);dGeomDestroy(rightLegGeom[1]);dGeomDestroy(rightLeg_2_1);dGeomDestroy(rightLeg_2_2);
        dGeomDestroy(rightLeg_3_1);dGeomDestroy(rightLeg_3_2);dGeomDestroy(rightLeg_4_1);dGeomDestroy(rightLeg_4_2);
    dGeomDestroy(rightLegGeom[5]);
    }
    if (actTorso == "off"){
        dGeomDestroy(torso0_geom); dGeomDestroy(torso1_geom); dGeomDestroy(torso2_geom); dGeomDestroy(torso3_geom); 
        dGeomDestroy(torsoGeom[4]); dGeomDestroy(torsoGeom[5]); 
    }else{
        for (int i=0; i<6; i++){
            dGeomDestroy(torsoGeom[i]);
        }
    }

    if (actLArm == "off"){
        dGeomDestroy(larm0_geom); dGeomDestroy(larm1_geom); dGeomDestroy(larm2_geom); dGeomDestroy(larm3_geom);
    }else{
        dGeomDestroy(geom[0]); dGeomDestroy(geom[2]); dGeomDestroy(geom[4]); dGeomDestroy(geom[6]); dGeomDestroy(geom[8]);    
    }
    if (actRArm == "off"){
        dGeomDestroy(rarm0_geom); dGeomDestroy(rarm1_geom); dGeomDestroy(rarm2_geom); dGeomDestroy(rarm3_geom);
    }else{
        dGeomDestroy(geom[1]); dGeomDestroy(geom[3]); dGeomDestroy(geom[5]); dGeomDestroy(geom[7]); dGeomDestroy(geom[9]);    
    }
    if (actLHand == "off"){
        dGeomDestroy(l_hand0_geom); dGeomDestroy(l_hand1_geom); dGeomDestroy(l_hand2_geom); dGeomDestroy(l_hand3_geom); dGeomDestroy(l_hand4_geom); dGeomDestroy(l_hand5_geom);
    }else{
        dGeomDestroy(geom[10]);dGeomDestroy(geom[12]);dGeomDestroy(geom[13]);
        dGeomDestroy(lhandfings0_geom); dGeomDestroy(lhandfings1_geom); 	
        dGeomDestroy(geom[16]);dGeomDestroy(geom[17]);
        dGeomDestroy(lhandfings2_geom);dGeomDestroy(lhandfings3_geom);
        dGeomDestroy(geom[20]);dGeomDestroy(geom[21]);
        dGeomDestroy(lhandfings4_geom);dGeomDestroy(lhandfings5_geom);
        dGeomDestroy(geom[24]);dGeomDestroy(geom[25]);
        dGeomDestroy(lhandfings6_geom);dGeomDestroy(lhandfings7_geom);
        dGeomDestroy(geom[28]); dGeomDestroy(geom[29]); dGeomDestroy(geom[30]);
    }

    if (actRHand == "off"){
        dGeomDestroy(r_hand0_geom); dGeomDestroy(r_hand1_geom); dGeomDestroy(r_hand2_geom); dGeomDestroy(r_hand3_geom); dGeomDestroy(r_hand4_geom); dGeomDestroy(r_hand5_geom);
    }else{
        dGeomDestroy(geom[11]);dGeomDestroy(geom[31]);dGeomDestroy(geom[32]);
        dGeomDestroy(rhandfings0_geom);dGeomDestroy(rhandfings1_geom);  		
        dGeomDestroy(geom[35]);dGeomDestroy(geom[36]);
        dGeomDestroy(rhandfings2_geom);dGeomDestroy(rhandfings3_geom);
        dGeomDestroy(geom[39]);dGeomDestroy(geom[40]);
        dGeomDestroy(rhandfings4_geom);dGeomDestroy(rhandfings5_geom);
        dGeomDestroy(geom[43]);dGeomDestroy(geom[44]);
        dGeomDestroy(rhandfings6_geom);dGeomDestroy(rhandfings7_geom);
        dGeomDestroy(geom[47]); dGeomDestroy(geom[48]); dGeomDestroy(geom[49]);
    }

    if (actHead == "off"){
        dGeomDestroy(neck0_geom); dGeomDestroy(neck1_geom);
    }else{
        dGeomDestroy(neckgeom[0]); dGeomDestroy(neckgeom[1]);
    }
    dGeomDestroy(head0_geom); dGeomDestroy(head1_geom); dGeomDestroy(head2_geom); dGeomDestroy(head3_geom);
    dGeomDestroy(head4_geom); dGeomDestroy(head5_geom); dGeomDestroy(head6_geom); dGeomDestroy(head7_geom);

    dGeomDestroy(eye1_geom); dGeomDestroy(eye2_geom); dGeomDestroy(eye3_geom); dGeomDestroy(eye4_geom);
    dGeomDestroy(eye5_geom); dGeomDestroy(topEyeLid_geom); dGeomDestroy(bottomEyeLid_geom); dGeomDestroy(Leye1_geom); dGeomDestroy(Reye1_geom); 
    if (iCubHeadModel!=0)
        delete iCubHeadModel;

    if (topEyeLidModel!=0)
        delete topEyeLidModel;

    if (bottomEyeLidModel!=0)
        delete bottomEyeLidModel;

    if (eyeLids!=0)
        delete eyeLids;

    if (model_ThreeD_obj.size()){
        for (map<string,ConstString>::iterator itr=model.begin(); itr != model.end(); itr++)
        {
            dGeomDestroy(model_ThreeD_obj[(*itr).first].geom);
        }
    }
    dSpaceDestroy (iCub);
    
}

ICubSim::ICubSim(dWorldID world, dSpaceID space, dReal X, dReal Y, dReal Z,
                 RobotConfig& config)
: ICubData() {
    resetSpeeds();
    init(world, space, X, Y, Z, config);
    reinitialized = true;
    eyeLids = 0;
}
