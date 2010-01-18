// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2009 Eric Sauser, EPFL
 * RobotCub Consortium, European Commission FP6 Project IST-004370
 * email:   eric.sauser@a3.epfl.ch
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

#include "XSens2ICub.h"

double mTime;
double mPrevTime;


int Init(int argc, char* argv[]){
    
  
  bUseGUI     = true;
  bUseXSens   = false;
  bUseGloves  = false;
  bUseNetwork = false;
  
  port = 1207;
  sprintf(host,"127.0.0.1");

  strcpy(xsensPort,"/dev/ttyUSB1");
  strcpy(glovesPort,"/dev/ttyUSB0");

  bUseYarp= true;

  for(int i=1;i<argc;i++){
    if(strcmp(argv[i],"-h")==0){
      bUseNetwork = true;
      if(argc>i+1)
        strcpy(host,argv[i+1]);  
    }else if(strcmp(argv[i],"-p")==0){
      bUseNetwork = true;        
      if(argc>i+1)
        port = atoi(argv[i+1]);  
    }else if(strcmp(argv[i],"-x")==0){
      bUseXSens = true;        
      if(argc>i+1)
        strcpy(xsensPort,argv[i+1]);  
    }else if(strcmp(argv[i],"-g")==0){
      bUseGloves = true;        
      if(argc>i+1)
        strcpy(glovesPort,argv[i+1]);  
    }else if(strcmp(argv[i],"-nox")==0){
      bUseGUI = false;  
    }else if(strcmp(argv[i],"-noyarp")==0){
      bUseYarp = false;  
    }
  }



  if(bUseYarp){
    if (!(mYarp.checkNetwork()))
      bUseYarp = false;
    else
      bUseYarp = true;
  }

  if(bUseNetwork){
    printf("************************************************\n");
    printf("XSens2Hoap Server started on port <%d> and waiting for <%s>\n",port,host);
    printf("************************************************\n");
    hoap3Net.Init(host,port,true);
    hoap3Net.SetTimers(66,10000);
  }  
  if(bUseYarp){
    mGlovePorts[0].open("/MotionSensors/left_glove");  
    mGlovePorts[1].open("/MotionSensors/right_glove");  
    mArmPorts[0].open("/MotionSensors/left_arm");  
    mArmPorts[1].open("/MotionSensors/right_arm");  
  }

  if(bUseGloves){
    if(gloves.Init(glovesPort)){
      bUseGloves = true;    
    }else{
      bUseGloves = false;
    }
  }
  if(bUseXSens){
    if(xsens.Init(xsensPort)){
      xsens.Start();
      bUseXSens = true;          
    }else{
      bUseXSens = false;      
    }
  }
  
  
  refreshPeriod = 66;
  refreshTimer.Start(refreshPeriod);
  
  mTime = Time::now();

  for(int i=0;i<BID_SIZE;i++){
    mSensorBody.mBodyMatrices[i].Identity();
    mSensorBody.mBodyMatricesTransform[i].Identity();
  }
  //mSensorBody.mBodyMatricesTransform[BID_LEFTWRIST].RotationZ(DEG2RAD(-180.0f));

  mSensorBody.mBodyMatricesZero[BID_TORSO         ].Identity();
  mSensorBody.mBodyMatricesZero[BID_HEAD          ].RotationX(DEG2RAD(180.0f));
  mSensorBody.mBodyMatricesZero[BID_LEFTSHOULDER  ].RotationYXZ(DEG2RAD( 90.0f),DEG2RAD(180.0f),0.0f);
  mSensorBody.mBodyMatricesZero[BID_LEFTELBOW     ].RotationYXZ(DEG2RAD( 90.0f),DEG2RAD(180.0f),0.0f);
  mSensorBody.mBodyMatricesZero[BID_LEFTWRIST     ].RotationYXZ(DEG2RAD( 90.0f),DEG2RAD(180.0f),0.0f);
  mSensorBody.mBodyMatricesZero[BID_RIGHTSHOULDER ].RotationYXZ(DEG2RAD(-90.0f),DEG2RAD(180.0f),0.0f);
  mSensorBody.mBodyMatricesZero[BID_RIGHTELBOW    ].RotationYXZ(DEG2RAD(-90.0f),DEG2RAD(180.0f),0.0f);
  mSensorBody.mBodyMatricesZero[BID_RIGHTWRIST    ].RotationYXZ(DEG2RAD(-90.0f),DEG2RAD(180.0f),0.0f);

  mSensorBody.baseTruncHeight = 0.3f;
  mSensorBody.truncHeight     = 0.3f;
  mSensorBody.truncWidth      = 0.4f;  
  mSensorBody.headLength      = 0.2f;
  mSensorBody.neckLength      = 0.1f;
  mSensorBody.upperArmLength  = 0.35f;
  mSensorBody.forearmLength   = 0.2f;
  mSensorBody.handLength      = 0.08f;
  mSensorBody.finger0Length   = 0.05f;
  mSensorBody.finger1Length   = 0.05f;
  
  /*
  mSensorBody.torsoIndex            = 7372;
  mSensorBody.headIndex             = 7391;
  mSensorBody.leftShoulderIndex     = 7414;
  mSensorBody.rightShoulderIndex    = 7392;
  mSensorBody.leftElbowIndex        = 7357;
  mSensorBody.rightElbowIndex       = 7371;
  */
  /*
  mSensorBody.mBodyMatricesIndex[BID_TORSO]            = 1;
  mSensorBody.mBodyMatricesIndex[BID_HEAD]             = 2;
  mSensorBody.mBodyMatricesIndex[BID_LEFTSHOULDER]     = 3;
  mSensorBody.mBodyMatricesIndex[BID_LEFTELBOW]        = 4;
  mSensorBody.mBodyMatricesIndex[BID_LEFTWRIST]        = 5;
  mSensorBody.mBodyMatricesIndex[BID_RIGHTSHOULDER]    = 7372;
  mSensorBody.mBodyMatricesIndex[BID_RIGHTELBOW]       = 7392;
  mSensorBody.mBodyMatricesIndex[BID_RIGHTWRIST]       = 7371;
  */
  
  mSensorBody.mBodyMatricesIndex[BID_TORSO]            = 1;
  mSensorBody.mBodyMatricesIndex[BID_HEAD]             = 2;
    
mSensorBody.mBodyMatricesIndex[BID_LEFTSHOULDER]     = 7372;
  mSensorBody.mBodyMatricesIndex[BID_LEFTELBOW]        = 7392;
  mSensorBody.mBodyMatricesIndex[BID_LEFTWRIST]        = 7357;
  mSensorBody.mBodyMatricesIndex[BID_RIGHTSHOULDER]    = 7415;
  mSensorBody.mBodyMatricesIndex[BID_RIGHTELBOW]       = 7391;
  mSensorBody.mBodyMatricesIndex[BID_RIGHTWRIST]       = 7035;
  /*
  mSensorBody.mBodyMatricesIndex[BID_RIGHTSHOULDER]     = 7372;
  mSensorBody.mBodyMatricesIndex[BID_RIGHTELBOW]        = 7392;
  mSensorBody.mBodyMatricesIndex[BID_RIGHTWRIST]        = 7371;
  mSensorBody.mBodyMatricesIndex[BID_LEFTSHOULDER]      = 7415;
  mSensorBody.mBodyMatricesIndex[BID_LEFTELBOW]         = 7391;
  mSensorBody.mBodyMatricesIndex[BID_LEFTWRIST]         = 0;
  */
  Matrix3 asd;
  asd.RotationZ(DEG2RAD(180));
  xsens.SetPreMatrix(mSensorBody.mBodyMatricesIndex[BID_LEFTWRIST],asd);
  xsens.SetPreMatrix(mSensorBody.mBodyMatricesIndex[BID_RIGHTWRIST],asd);


  for(int i=0;i<14;i++){
    mSensorBody.mHandLeft.mSensorsArray[i]    = R_ONE;
    mSensorBody.mHandRight.mSensorsArray[i]   = R_ZERO;    
    mSensorBody.mHandOrigin.mSensorsArray[i]  = R_ZERO;
  }
  mSensorBody.mHandScale.mSensors.mThumb0      = DEG2RAD(45);
  mSensorBody.mHandScale.mSensors.mThumb1      = DEG2RAD(45);
  
  mSensorBody.mHandScale.mSensors.mIndex0      = DEG2RAD(75);
  mSensorBody.mHandScale.mSensors.mIndex1      = DEG2RAD(115);

  mSensorBody.mHandScale.mSensors.mMiddle0     = DEG2RAD(75);
  mSensorBody.mHandScale.mSensors.mMiddle1     = DEG2RAD(115);

  mSensorBody.mHandScale.mSensors.mRing0       = DEG2RAD(75);
  mSensorBody.mHandScale.mSensors.mRing1       = DEG2RAD(115);

  mSensorBody.mHandScale.mSensors.mLittle0     = DEG2RAD(75);
  mSensorBody.mHandScale.mSensors.mLittle1     = DEG2RAD(115);

  mSensorBody.mHandScale.mSensors.mThumbIndex  = DEG2RAD(45);
  mSensorBody.mHandScale.mSensors.mIndexMiddle = DEG2RAD(30);
  mSensorBody.mHandScale.mSensors.mMiddleRing  = DEG2RAD(20);
  mSensorBody.mHandScale.mSensors.mRingLittle  = DEG2RAD(20);
  


  hoap3min.torso0 = DEG2RAD( -90); hoap3max.torso0 = DEG2RAD(  90);
  hoap3min.torso1 = DEG2RAD( -90); hoap3max.torso1 = DEG2RAD(  90);
  hoap3min.torso2 = DEG2RAD( -90); hoap3max.torso2 = DEG2RAD(  90);

  hoap3min.hpan   = DEG2RAD( -60); hoap3max.hpan   = DEG2RAD(  60);
  hoap3min.hflx   = DEG2RAD( -45); hoap3max.hflx   = DEG2RAD(  15);
  

  hoap3min.rsfe   = DEG2RAD( -90); hoap3max.rsfe   = DEG2RAD(  10);
  hoap3min.rsaa   = DEG2RAD(   0); hoap3max.rsaa   = DEG2RAD( 160);
  hoap3min.rshr   = DEG2RAD( -35); hoap3max.rshr   = DEG2RAD(  80);
  hoap3min.reb    = DEG2RAD(   5); hoap3max.reb    = DEG2RAD( 105);
  hoap3min.rwr    = DEG2RAD( -90); hoap3max.rwr    = DEG2RAD(  90);
  hoap3min.rwp    = DEG2RAD( -90); hoap3max.rwp    = DEG2RAD(   0);
  hoap3min.rwy    = DEG2RAD( -20); hoap3max.rwy    = DEG2RAD(  40);


  hoap3min.lsfe   = DEG2RAD(- 90); hoap3max.lsfe   = DEG2RAD(  10);
  hoap3min.lsaa   = DEG2RAD(   0); hoap3max.lsaa   = DEG2RAD( 160);
  hoap3min.lshr   = DEG2RAD( -35); hoap3max.lshr   = DEG2RAD(  80);
  hoap3min.leb    = DEG2RAD(   5); hoap3max.leb    = DEG2RAD( 105);
  hoap3min.lwr    = DEG2RAD( -90); hoap3max.lwr    = DEG2RAD(  90);
  hoap3min.lwp    = DEG2RAD( -90); hoap3max.lwp    = DEG2RAD(   0);
  hoap3min.lwy    = DEG2RAD( -20); hoap3max.lwy    = DEG2RAD(  40);

  memset(&hoap3anglesPrev,0,sizeof(BasicUpperBodyJointAngles));
  memset(&hoap3angles    ,0,sizeof(BasicUpperBodyJointAngles));



  for(int i=0;i<BID_SIZE;i++){
    posesMatrices[0][i].Identity();
  }
  for(int i=0;i<BID_SIZE;i++){
    posesMatrices[1][i].Identity();
  }
  posesMatrices[1][BID_LEFTSHOULDER].RotationY(DEG2RAD(-90.0));
  posesMatrices[1][BID_LEFTELBOW].RotationY(DEG2RAD(-90.0));
  posesMatrices[1][BID_LEFTWRIST].RotationY(DEG2RAD(-90.0));

  for(int i=0;i<BID_SIZE;i++){
    posesMatrices[2][i].Identity();
  }
  posesMatrices[2][BID_LEFTSHOULDER].RotationYXZ(0,DEG2RAD(-90.0),DEG2RAD(-90.0));
  posesMatrices[2][BID_LEFTELBOW].RotationYXZ(0,DEG2RAD(-90.0),DEG2RAD(-90.0));
  posesMatrices[2][BID_LEFTWRIST].RotationYXZ(0,DEG2RAD(-90.0),DEG2RAD(-90.0));

  poseCount = 2;


  bSavingData   = false;
  dataString[0] = 0;
  bkgColor      = 0.0f;
  saveFileCount = 0;

  calibTimeout  = 3000;
  bCalibRequest = false;

  
  return TRUE;
}






void InitGL(){
  glClearColor(0.1,0.1,0.1,1.0);
  glEnable(GL_DEPTH_TEST);  
  
  float lightAmbient0[4] = {0.0f, 0.0f, 0.0f, 1.0f};
  float lightAmbient1[4] = {0.0f, 0.0f, 0.0f, 1.0f};
  float lightDiffuse0[4] = {0.5f, 0.5f, 0.5f, 1.0f};
  float lightDiffuse1[4] = {0.85f, 0.85f, 0.84f, 1.0f};

  glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient0);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse0);
  glLightfv(GL_LIGHT1, GL_AMBIENT, lightAmbient1);
  glLightfv(GL_LIGHT1, GL_DIFFUSE, lightDiffuse1);
   
  //Enable the first light and the lighting mode
  glEnable(GL_LIGHT0);
  glEnable(GL_LIGHT1);
  glEnable(GL_LIGHTING);
    
  glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
  glEnable(GL_COLOR_MATERIAL);
  
  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK); 
  glEnable(GL_DEPTH_TEST);
  glDepthMask(GL_TRUE);
  
  glEnable(GL_LINE_SMOOTH);
  glHint (GL_LINE_SMOOTH_HINT, GL_NICEST);
  glEnable(GL_POLYGON_SMOOTH);
  glHint (GL_POLYGON_SMOOTH_HINT, GL_NICEST);

  glLineWidth(1.5);

  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_BLEND);
  glShadeModel(GL_SMOOTH);

  glEnable(GL_TEXTURE_2D);
  glEnable(GL_NORMALIZE);
   
  currBtn   = -1;
  currX     = 0;
  currY     = 0; 
  
  camera.SetPosition(Vector3(0,0,2));
}






void  DrawSkel(Matrix3 *bodyMatrices)
{
  float refSize = 0.1;
  float segSize = 0.01;
  Vector3 pos,pos2;
  float length;
  float angleOffset;
  float angle0;
  float angle1;

  glPushMatrix();
    GLT::DrawRef(refSize);
    GLT::DrawSegment(Vector3(mSensorBody.baseTruncHeight,0.0f,0.0f),segSize);  
  glPopMatrix();

  //torso  
  pos = Vector3(mSensorBody.baseTruncHeight,0.0f,0.0f);
  glPushMatrix();
    glTranslatef(pos(0),pos(1),pos(2));
    glMultMatrixf(Matrix4(bodyMatrices[BID_TORSO]).RowOrderForceFloat());
    GLT::DrawRef(refSize);
    GLT::DrawSegment(Vector3(mSensorBody.truncHeight+mSensorBody.neckLength,0.0f,0.0f),segSize);
    glTranslatef(mSensorBody.truncHeight,-mSensorBody.truncWidth/2.0f,0.0f);
    GLT::DrawSegment(Vector3(0.0f,mSensorBody.truncWidth,0.0f),segSize);
    glTranslatef(0.0f,mSensorBody.truncWidth/2.0f,0.0f);
  glPopMatrix();
    
  // head
  pos += bodyMatrices[BID_TORSO]*Vector3(mSensorBody.truncHeight+mSensorBody.neckLength,0.0f,0.0f);  
  glPushMatrix();
    glTranslatef(pos(0),pos(1),pos(2));
    glMultMatrixf(Matrix4(bodyMatrices[BID_HEAD]).RowOrderForceFloat());
    GLT::DrawRef(refSize);
    GLT::DrawSegment(Vector3(mSensorBody.headLength,0.0f,0.0f),segSize);       
  glPopMatrix();
  
  
  // right upper arm  
  pos = Vector3(mSensorBody.baseTruncHeight,0.0f,0.0f);
  pos += bodyMatrices[BID_TORSO]*Vector3(mSensorBody.truncHeight,mSensorBody.truncWidth/2.0f,0.0f);  
  glPushMatrix();
    glTranslatef(pos(0),pos(1),pos(2));
    glMultMatrixf(Matrix4(bodyMatrices[BID_RIGHTSHOULDER]).RowOrderForceFloat());
    GLT::DrawRef(refSize);
    GLT::DrawSegment(Vector3(mSensorBody.upperArmLength,0.0f,0.0f),segSize);       
  glPopMatrix();


  pos += bodyMatrices[BID_RIGHTSHOULDER]*Vector3(mSensorBody.upperArmLength,0.0f,0.0f);  
  glPushMatrix();
    glTranslatef(pos(0),pos(1),pos(2));
    glMultMatrixf(Matrix4(bodyMatrices[BID_RIGHTELBOW]).RowOrderForceFloat());
    GLT::DrawRef(refSize);
    GLT::DrawSegment(Vector3(mSensorBody.forearmLength,0.0f,0.0f),segSize);       
  glPopMatrix();
  
  pos += bodyMatrices[BID_RIGHTELBOW]*Vector3(mSensorBody.forearmLength,0.0f,0.0f);
  
  glPushMatrix();
    glTranslatef(pos(0),pos(1),pos(2));
    glMultMatrixf(Matrix4(bodyMatrices[BID_RIGHTWRIST]).RowOrderForceFloat());
    
    pos += bodyMatrices[BID_RIGHTWRIST]*Vector3(mSensorBody.handLength,0.0f,0.0f);
    rightHandHMatrix.Transformation(bodyMatrices[BID_RIGHTWRIST],pos);
    
    GLT::DrawRef(refSize);
    
    angleOffset = -mSensorBody.mHandRight.mSensors.mIndexMiddle * mSensorBody.mHandScale.mSensors.mIndexMiddle;
    for(int i=-1;i<4;i++){
      glPushMatrix();
      glRotatef(-(float(i)-1.0)*10,0.0,0.0,1.0);        
      length = mSensorBody.handLength;
      if(i<0){
        glRotatef(10,0.0,1.0,0.0);  
        glRotatef(RAD2DEG(mSensorBody.mHandRight.mSensors.mThumbIndex * mSensorBody.mHandScale.mSensors.mThumbIndex),0.0,1.0,1.0);  
        length*=0.8;
      }
      
      switch(i){
      case -1: 
        angle0 = mSensorBody.mHandRight.mSensors.mThumb0 * mSensorBody.mHandScale.mSensors.mThumb0; 
        angle1 = mSensorBody.mHandRight.mSensors.mThumb1 * mSensorBody.mHandScale.mSensors.mThumb1; 
        break;
      case 0:          
        angle0 = mSensorBody.mHandRight.mSensors.mIndex0 * mSensorBody.mHandScale.mSensors.mIndex0; 
        angle1 = mSensorBody.mHandRight.mSensors.mIndex1 * mSensorBody.mHandScale.mSensors.mIndex1; 
        break;
      case 1:          
        angle0 = mSensorBody.mHandRight.mSensors.mMiddle0 * mSensorBody.mHandScale.mSensors.mMiddle0; 
        angle1 = mSensorBody.mHandRight.mSensors.mMiddle1 * mSensorBody.mHandScale.mSensors.mMiddle1; 
        break;
      case 2:
        angle0 = mSensorBody.mHandRight.mSensors.mRing0 * mSensorBody.mHandScale.mSensors.mRing0; 
        angle1 = mSensorBody.mHandRight.mSensors.mRing1 * mSensorBody.mHandScale.mSensors.mRing1; 
        break;
      case 3:  
        angle0 = mSensorBody.mHandRight.mSensors.mLittle0 * mSensorBody.mHandScale.mSensors.mLittle0; 
        angle1 = mSensorBody.mHandRight.mSensors.mLittle1 * mSensorBody.mHandScale.mSensors.mLittle1; 
        break;
      }
       
      
      GLT::DrawSegment(Vector3(length,0.0f,0.0f),segSize/2);    
        glPushMatrix();
          glTranslatef(length,0.0f,0.0f);
          if(i<0){
            glRotatef(-70,1.0,0.0,0.0);  
          }else{
            glRotatef((float(i)-1.0)*10,0.0,0.0,1.0);
            glRotatef(RAD2DEG(angleOffset),0.0,0.0,-1.0);
            switch(i){
              case 0: angleOffset += mSensorBody.mHandRight.mSensors.mIndexMiddle * mSensorBody.mHandScale.mSensors.mIndexMiddle; break;
              case 1: angleOffset += mSensorBody.mHandRight.mSensors.mMiddleRing  * mSensorBody.mHandScale.mSensors.mMiddleRing;  break;
              case 2: angleOffset += mSensorBody.mHandRight.mSensors.mRingLittle  * mSensorBody.mHandScale.mSensors.mRingLittle;  break;
            }             
          }        
          glRotatef(RAD2DEG(angle0),0,1,0);
          length = mSensorBody.finger0Length * (1.0-fabs(float(i)-1.0)*0.15);
          GLT::DrawSegment(Vector3(length,0.0f,0.0f),segSize/2);    
          glTranslatef(length,0.0f,0.0f);
          glRotatef(RAD2DEG(angle1),0,1,0);
          length = mSensorBody.finger1Length * (1.0-fabs(float(i)-1.0)*0.15);
          GLT::DrawSegment(Vector3(length,0.0f,0.0f),segSize/2);    
        glPopMatrix();
      glPopMatrix();
    }
    /*
    for(int i=-1;i<4;i++){
      glPushMatrix();
      glRotatef(-(float(i)-1.0)*10,0.0,0.0,1.0);        
      length = mSensorBody.handLength;
      if(i<0){
        glRotatef(10,0.0,1.0,0.0);  
        length*=0.8;
      }
      
      GLT::DrawSegment(Vector3(length,0.0f,0.0f),segSize/2);    
        glPushMatrix();
          glTranslatef(length,0.0f,0.0f);
          if(i<0){
            glRotatef(-80,1.0,0.0,0.0);  
          }else{
            glRotatef((float(i)-1.0)*10,0.0,0.0,1.0);
          }        
          glRotatef(10,0,1,0);
          length = mSensorBody.finger0Length * (1.0-fabs(float(i)-1.0)*0.15);
          GLT::DrawSegment(Vector3(length,0.0f,0.0f),segSize/2);    
          glTranslatef(length,0.0f,0.0f);
          glRotatef(10,0,1,0);
          length = mSensorBody.finger1Length * (1.0-fabs(float(i)-1.0)*0.15);
          GLT::DrawSegment(Vector3(length,0.0f,0.0f),segSize/2);    
        glPopMatrix();
      glPopMatrix();
    }
    */
  glPopMatrix();

  // left upper arm
  pos = Vector3(mSensorBody.baseTruncHeight,0.0f,0.0f);
  pos += bodyMatrices[BID_TORSO]*Vector3(mSensorBody.truncHeight,-mSensorBody.truncWidth/2.0f,0.0f);  
  glPushMatrix();
    glTranslatef(pos(0),pos(1),pos(2));
    glMultMatrixf(Matrix4(bodyMatrices[BID_LEFTSHOULDER]).RowOrderForceFloat());
    GLT::DrawRef(refSize);
    GLT::DrawSegment(Vector3(mSensorBody.upperArmLength,0.0f,0.0f),segSize);       
  glPopMatrix();

  pos += bodyMatrices[BID_LEFTSHOULDER]*Vector3(mSensorBody.upperArmLength,0.0f,0.0f);  
  glPushMatrix();
    glTranslatef(pos(0),pos(1),pos(2));
    glMultMatrixf(Matrix4(bodyMatrices[BID_LEFTELBOW]).RowOrderForceFloat());
    GLT::DrawRef(refSize);
    GLT::DrawSegment(Vector3(mSensorBody.forearmLength,0.0f,0.0f),segSize);       
  glPopMatrix();
  
  pos += bodyMatrices[BID_LEFTELBOW]*Vector3(mSensorBody.forearmLength,0.0f,0.0f);
  glPushMatrix();
    glTranslatef(pos(0),pos(1),pos(2));
    glMultMatrixf(Matrix4(bodyMatrices[BID_LEFTWRIST]).RowOrderForceFloat());

    pos += bodyMatrices[BID_LEFTWRIST]*Vector3(mSensorBody.handLength,0.0f,0.0f);
    leftHandHMatrix.Transformation(bodyMatrices[BID_LEFTWRIST],pos);

    GLT::DrawRef(refSize);

    angleOffset = -mSensorBody.mHandLeft.mSensors.mIndexMiddle * mSensorBody.mHandScale.mSensors.mIndexMiddle;
    for(int i=-1;i<4;i++){
      glPushMatrix();
      glRotatef((float(i)-1.0)*10,0.0,0.0,1.0);        
      length = mSensorBody.handLength;
      if(i<0){
        glRotatef(10,0.0,1.0,0.0);  
        glRotatef(RAD2DEG(mSensorBody.mHandLeft.mSensors.mThumbIndex * mSensorBody.mHandScale.mSensors.mThumbIndex),0.0,1.0,-1.0);  
        length*=0.8;
      }
      
      switch(i){
      case -1: 
        angle0 = mSensorBody.mHandLeft.mSensors.mThumb0 * mSensorBody.mHandScale.mSensors.mThumb0; 
        angle1 = mSensorBody.mHandLeft.mSensors.mThumb1 * mSensorBody.mHandScale.mSensors.mThumb1; 
        break;
      case 0:          
        angle0 = mSensorBody.mHandLeft.mSensors.mIndex0 * mSensorBody.mHandScale.mSensors.mIndex0; 
        angle1 = mSensorBody.mHandLeft.mSensors.mIndex1 * mSensorBody.mHandScale.mSensors.mIndex1; 
        break;
      case 1:          
        angle0 = mSensorBody.mHandLeft.mSensors.mMiddle0 * mSensorBody.mHandScale.mSensors.mMiddle0; 
        angle1 = mSensorBody.mHandLeft.mSensors.mMiddle1 * mSensorBody.mHandScale.mSensors.mMiddle1; 
        break;
      case 2:
        angle0 = mSensorBody.mHandLeft.mSensors.mRing0 * mSensorBody.mHandScale.mSensors.mRing0; 
        angle1 = mSensorBody.mHandLeft.mSensors.mRing1 * mSensorBody.mHandScale.mSensors.mRing1; 
        break;
      case 3:  
        angle0 = mSensorBody.mHandLeft.mSensors.mLittle0 * mSensorBody.mHandScale.mSensors.mLittle0; 
        angle1 = mSensorBody.mHandLeft.mSensors.mLittle1 * mSensorBody.mHandScale.mSensors.mLittle1; 
        break;
      }
       
      
      GLT::DrawSegment(Vector3(length,0.0f,0.0f),segSize/2);    
        glPushMatrix();
          glTranslatef(length,0.0f,0.0f);
          if(i<0){
            glRotatef(70,1.0,0.0,0.0);  
          }else{
            glRotatef(-(float(i)-1.0)*10,0.0,0.0,1.0);
            glRotatef(RAD2DEG(angleOffset),0.0,0.0,1.0);
            switch(i){
              case 0: angleOffset += mSensorBody.mHandLeft.mSensors.mIndexMiddle * mSensorBody.mHandScale.mSensors.mIndexMiddle; break;
              case 1: angleOffset += mSensorBody.mHandLeft.mSensors.mMiddleRing  * mSensorBody.mHandScale.mSensors.mMiddleRing;  break;
              case 2: angleOffset += mSensorBody.mHandLeft.mSensors.mRingLittle  * mSensorBody.mHandScale.mSensors.mRingLittle;  break;
            }             
          }        
          glRotatef(RAD2DEG(angle0),0,1,0);
          length = mSensorBody.finger0Length * (1.0-fabs(float(i)-1.0)*0.15);
          GLT::DrawSegment(Vector3(length,0.0f,0.0f),segSize/2);    
          glTranslatef(length,0.0f,0.0f);
          glRotatef(RAD2DEG(angle1),0,1,0);
          length = mSensorBody.finger1Length * (1.0-fabs(float(i)-1.0)*0.15);
          GLT::DrawSegment(Vector3(length,0.0f,0.0f),segSize/2);    
        glPopMatrix();
      glPopMatrix();
    }

  glPopMatrix();
  
  /*
  glPushMatrix();
  glMultMatrixf(leftHandHMatrix.RowOrderForceFloat());
  GLT::DrawRef(0.2);
  glPopMatrix();

  glPushMatrix();
  glMultMatrixf(rightHandHMatrix.RowOrderForceFloat());
  GLT::DrawRef(0.2);
  glPopMatrix();
  */
  
  
}




void Display(){
  if(bSavingData){
    if(bHandsOn){ 
      glClearColor(0.8,0.4,0.4,1.0);
    }else{
      glClearColor(0.5,0.1,0.1,1.0);
    }
  }else{
    if(bHandsOn){ 
      glClearColor(0.4,0.4,0.4,1.0);
    }else{
      glClearColor(0.1,0.1,0.1,1.0);
      //glClearColor(0.9,0.9,0.9,1.0);
    }
  }
  
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glPushMatrix();
    
  camera.Apply();

  float refSize = 0.1;
  float segSize = 0.01;

  Vector3 pos,pos2;

  glRotatef(90.0f,0.0f,0.0f,1.0f);
  glTranslatef(-0.3f,0.0f,0.0f);
  //GLT::DrawRef(refSize);
  GLT::SetColor(1.0f,1.0f,0.0f,0.7f);
  GLT::SetColor(0.6f,0.7f,1.0f,0.7f);
  
 // glTranslatef(0.0f,0.5f,0.0f);
  DrawSkel(mSensorBody.mBodyMatrices);
#if 1
  glPushMatrix();

  glTranslatef(0.0f,-0.00f,0.05f);
  GLT::SetColor(1.0f,0.0f,0.0f,0.7f);
  DrawSkel(mSensorBody.mBodyMatricesReconstructed);

  Matrix3 R;

  GLT::SetColor(0.0f,1.0f,0.0f,0.7f);
  glTranslatef(0.0f,-0.0f,0.05f);
    
  
  glPushMatrix();
    // R = mSensorBody.mBodyMatricesZero[BID_LEFTSHOULDER].Transpose() * mSensorBody.mBodyMatricesZero[BID_TORSO] * mSensorBody.mBodyMatrices[BID_TORSO].Transpose() * mSensorBody.mBodyMatrices[BID_LEFTSHOULDER];
    R = mSensorBody.mBodyMatricesZero[BID_TORSO]; glMultMatrixf(Matrix4(R).RowOrderForceFloat());
    GLT::DrawRef(refSize);
    GLT::DrawSegment(Vector3(mSensorBody.baseTruncHeight,0.0f,0.0f),segSize);  
        
    glPushMatrix();
      glTranslatef(mSensorBody.baseTruncHeight,0.0f,0.0f);
      R.RotationY(hoap3angles.torso0); glMultMatrixf(Matrix4(R).RowOrderForceFloat());
      R.RotationZ(hoap3angles.torso1); glMultMatrixf(Matrix4(R).RowOrderForceFloat());
      R.RotationX(hoap3angles.torso2); glMultMatrixf(Matrix4(R).RowOrderForceFloat());
      
      GLT::DrawRef(refSize);
      GLT::DrawSegment(Vector3(mSensorBody.truncHeight+mSensorBody.neckLength,0.0f,0.0f),segSize);
      glTranslatef(mSensorBody.truncHeight,-mSensorBody.truncWidth/2.0f,0.0f);
      GLT::DrawSegment(Vector3(0.0f,mSensorBody.truncWidth,0.0f),segSize);
      glTranslatef(0.0f,mSensorBody.truncWidth/2.0f,0.0f);

        
        /*
        glPushMatrix();
          glTranslatef(mSensorBody.neckLength,0.0f,0.0f);
          R.RotationX(hoap3angles.hpan);
          glMultMatrixf(Matrix4(R).RowOrderForceFloat());
          R.RotationY(hoap3angles.hflx);
          glMultMatrixf(Matrix4(R).RowOrderForceFloat());        
          GLT::DrawRef(refSize);
          GLT::DrawSegment(Vector3(mSensorBody.headLength,0.0f,0.0f),segSize);       
        glPopMatrix();
  */
        
      /*
      glPushMatrix();
        glTranslatef(0.0f,mSensorBody.truncWidth/2.0f,0.0f);
        R.RotationY(hoap3angles.rsfe);
        glMultMatrixf(Matrix4(R).RowOrderForceFloat());
        R.RotationZ(-hoap3angles.rsaa);
        glMultMatrixf(Matrix4(R).RowOrderForceFloat());        
        R.RotationX(hoap3angles.rshr);
        glMultMatrixf(Matrix4(R).RowOrderForceFloat());        
        GLT::DrawRef(refSize);
        GLT::DrawSegment(Vector3(mSensorBody.upperArmLength,0.0f,0.0f),segSize);
        glPushMatrix();
          glTranslatef(mSensorBody.upperArmLength,0.0f,0.0f);
          R.RotationY(-hoap3angles.reb);
          glMultMatrixf(Matrix4(R).RowOrderForceFloat());
          GLT::DrawRef(refSize);
          GLT::DrawSegment(Vector3(mSensorBody.forearmLength,0.0f,0.0f),segSize);
          glPushMatrix();
            glTranslatef(mSensorBody.forearmLength,0.0f,0.0f);
            R.RotationX(hoap3angles.rwr);
            glMultMatrixf(Matrix4(R).RowOrderForceFloat());
            R.RotationY(hoap3angles.rwp);
            glMultMatrixf(Matrix4(R).RowOrderForceFloat());
            R.RotationZ(hoap3angles.rwy);
            glMultMatrixf(Matrix4(R).RowOrderForceFloat());
            GLT::DrawRef(refSize);
            GLT::DrawSegment(Vector3(mSensorBody.handLength,0.0f,0.0f),segSize);
            glPushMatrix();
              glTranslatef(mSensorBody.handLength,0.0f,0.0f);
              GLT::DrawRef(refSize);
            glPopMatrix();
          glPopMatrix();
        glPopMatrix();
      glPopMatrix();
      */

      glPushMatrix();
        glTranslatef(0.0f,-mSensorBody.truncWidth/2.0f,0.0f);

        R = mSensorBody.mBodyMatricesZero[BID_TORSO].Transpose() * mSensorBody.mBodyMatricesZero[BID_LEFTSHOULDER]; glMultMatrixf(Matrix4(R).RowOrderForceFloat());
        R.RotationZ(hoap3angles.lsfe); glMultMatrixf(Matrix4(R).RowOrderForceFloat());
        R.RotationY(hoap3angles.lsaa); glMultMatrixf(Matrix4(R).RowOrderForceFloat());
        R.RotationX(hoap3angles.lshr); glMultMatrixf(Matrix4(R).RowOrderForceFloat());

        GLT::DrawRef(refSize);
        GLT::DrawSegment(Vector3(mSensorBody.upperArmLength,0.0f,0.0f),segSize);

        glPushMatrix();
          glTranslatef(mSensorBody.upperArmLength,0.0f,0.0f);

          R = mSensorBody.mBodyMatricesZero[BID_LEFTSHOULDER].Transpose() * mSensorBody.mBodyMatricesZero[BID_LEFTELBOW]; glMultMatrixf(Matrix4(R).RowOrderForceFloat());
          
          R.RotationZ(hoap3angles.leb);  glMultMatrixf(Matrix4(R).RowOrderForceFloat());
          
          GLT::DrawRef(refSize);
          GLT::DrawSegment(Vector3(mSensorBody.forearmLength,0.0f,0.0f),segSize);
          
          glPushMatrix();
            glTranslatef(mSensorBody.forearmLength,0.0f,0.0f);
            
            R = mSensorBody.mBodyMatricesZero[BID_LEFTELBOW].Transpose() * mSensorBody.mBodyMatricesZero[BID_LEFTWRIST]; glMultMatrixf(Matrix4(R).RowOrderForceFloat());
            
            R.RotationX(hoap3angles.lwr);  glMultMatrixf(Matrix4(R).RowOrderForceFloat());
            R.RotationY(hoap3angles.lwp);  glMultMatrixf(Matrix4(R).RowOrderForceFloat());
            R.RotationZ(hoap3angles.lwy);  glMultMatrixf(Matrix4(R).RowOrderForceFloat());

            GLT::DrawRef(refSize);
            GLT::DrawSegment(Vector3(mSensorBody.handLength,0.0f,0.0f),segSize);

            glPushMatrix();
              glTranslatef(mSensorBody.handLength,0.0f,0.0f);
              GLT::DrawRef(refSize);
            glPopMatrix();
          glPopMatrix();
        glPopMatrix();
      glPopMatrix();
        

      glPushMatrix();
        glTranslatef(0.0f,mSensorBody.truncWidth/2.0f,0.0f);

        R = mSensorBody.mBodyMatricesZero[BID_TORSO].Transpose() * mSensorBody.mBodyMatricesZero[BID_RIGHTSHOULDER]; glMultMatrixf(Matrix4(R).RowOrderForceFloat());
        R.RotationZ(hoap3angles.rsfe); glMultMatrixf(Matrix4(R).RowOrderForceFloat());
        R.RotationY(hoap3angles.rsaa); glMultMatrixf(Matrix4(R).RowOrderForceFloat());
        R.RotationX(hoap3angles.rshr); glMultMatrixf(Matrix4(R).RowOrderForceFloat());

        GLT::DrawRef(refSize);
        GLT::DrawSegment(Vector3(mSensorBody.upperArmLength,0.0f,0.0f),segSize);

        glPushMatrix();
          glTranslatef(mSensorBody.upperArmLength,0.0f,0.0f);

          R = mSensorBody.mBodyMatricesZero[BID_RIGHTSHOULDER].Transpose() * mSensorBody.mBodyMatricesZero[BID_RIGHTELBOW]; glMultMatrixf(Matrix4(R).RowOrderForceFloat());
          
          R.RotationZ(hoap3angles.reb);  glMultMatrixf(Matrix4(R).RowOrderForceFloat());
          
          GLT::DrawRef(refSize);
          GLT::DrawSegment(Vector3(mSensorBody.forearmLength,0.0f,0.0f),segSize);
          
          glPushMatrix();
            glTranslatef(mSensorBody.forearmLength,0.0f,0.0f);
            
            R = mSensorBody.mBodyMatricesZero[BID_RIGHTELBOW].Transpose() * mSensorBody.mBodyMatricesZero[BID_RIGHTWRIST]; glMultMatrixf(Matrix4(R).RowOrderForceFloat());
            
            R.RotationX(hoap3angles.rwr);  glMultMatrixf(Matrix4(R).RowOrderForceFloat());
            R.RotationY(hoap3angles.rwp);  glMultMatrixf(Matrix4(R).RowOrderForceFloat());
            R.RotationZ(hoap3angles.rwy);  glMultMatrixf(Matrix4(R).RowOrderForceFloat());

            GLT::DrawRef(refSize);
            GLT::DrawSegment(Vector3(mSensorBody.handLength,0.0f,0.0f),segSize);

            glPushMatrix();
              glTranslatef(mSensorBody.handLength,0.0f,0.0f);
              GLT::DrawRef(refSize);
            glPopMatrix();
          glPopMatrix();
        glPopMatrix();
      glPopMatrix();

    glPopMatrix();  
  glPopMatrix();
  
  
  for(int i=0;i<4;i++){
    glPushMatrix();
    glTranslatef(pts[i](0),pts[i](1),pts[i](2));
    GLT::DrawRef(0.1);
    glPopMatrix();  
  }
  
  
  glPopMatrix();
      
  
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  gluOrtho2D(-100, 100, -100, 100);
  glScalef(1.0f, -1.0f, 1.0f);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_LIGHTING);
  glDisable(GL_CULL_FACE);
  
  
  
  GLT::DisplayText(50,50,"1. asdgffghgfhf",5);  
  GLT::DisplayText(50,55,"2. sdgfdgsagsdg",5);  
  GLT::DisplayText(50,60,"3. asdadgreeasd",5);  
  
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);
  glEnable(GL_CULL_FACE);

  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();

  glMatrixMode(GL_PROJECTION);
#endif
  glPopMatrix();
  
  glutSwapBuffers();
} 

void OnReshape(int width, int height){
  camera.SetViewport(width,height);
  camera.Apply();  
}

int   defMotionId=0;
float masterTime = 0.0;
float localTime = 0.0;


void OnKeyPress(unsigned char key, int x, int y){
  if(key=='c'){
    calibTimer.Start(calibTimeout);
    bCalibRequest = true;    
  }
  if(key=='w'){
    gloves.StartCalibration();
  }
  
  if(key=='e'){
    gloves.StopCalibration();
  }

  if(key=='d'){
    xsens.MultiPoseCalibrationReset();
    poseCount = 0;
  }
  if(key=='f'){
    for(int i=0;i<BID_SIZE;i++){
      xsens.MultiPoseCalibrationSetPose(mSensorBody.mBodyMatricesIndex[i],posesMatrices[poseCount][i]);
    }
    xsens.MultiPoseCalibrationFetch();
    poseCount++;
  }
  if(key=='g'){
    xsens.MultiPoseCalibrate();
  }
  
  if(key=='h'){
    bShowPose = !bShowPose; 
  }
  
  if(key=='r'){
    xsens.StartSequencer(true);
  }
  if(key=='t'){
    xsens.StopSequencer();
  }

  if(key==' '){
    bHandsOn = !bHandsOn;
  }
  static int poseL=0;
  static int poseR=0;
  if(key=='b'){
    if(poseR==0){
      cout <<"Getting pose left "<<poseL<<endl;
      gloves.GetCalibrationPose(FD_HAND_LEFT,poseL++);
      if(poseL>=gloves.GetNbCalibrationPose()){
	gloves.PosesCalibration(FD_HAND_LEFT);
	poseL = 0;
	cout <<"Calibration left"<<endl;
      }  
    }else{
      cout << "please finish other right hand calib"<<endl;
    }
  }
  if(key=='n'){
    if(poseL==0){
      cout <<"Getting pose right "<<poseR<<endl;
      gloves.GetCalibrationPose(FD_HAND_RIGHT,poseR++);
      if(poseR>=gloves.GetNbCalibrationPose()){
	gloves.PosesCalibration(FD_HAND_RIGHT);
	poseR = 0;
	cout <<"Calibration right"<<endl;
      }  
    }else{
      cout << "please finish other left hand calib"<<endl;
    }
  }
  
  if(key=='l'){
    bReadFile = true;
    static int cnt = 0;
    char txt[256];
    if(jfile.is_open()) jfile.close();
    sprintf(txt,"/home/esauser/Documents/MATLAB/GraspNPlace/data/source/data%03d.txt",cnt);
    cnt+=3;
    jfile.open(txt);  
  }
    
  if((key>='1')&&(key<='8')){
    defMotionId = key - '1';
    masterTime = 0;
    localTime = 0;
    //int j = key-'0';
    //rightHandHMatrix.GetTranslation(pts[j]);
  }
    
  if(key=='s'){
    bHandsOn = false;
    if(!bSavingData){
      char txt[256];
      sprintf(txt,"./data/data%03d.txt",saveFileCount);
      string str(txt);
      StartSavingData(str);
      saveFileCount ++;
    }else{
      StopSavingData();
    }
  }
}

void OnSpecialKeyPress(int key, int x, int y){
}

void OnMouseButton(int button, int state, int x, int y){
  if(state==GLUT_DOWN){
    currBtn = button;
    currX   = x;
    currY   = y;        
  }else{
    currBtn = -1;
  }
}

void OnMouseMotion(int x, int y){
  if(currBtn ==  GLUT_LEFT_BUTTON){
    camera.Move((float)(x-currX),-(float)(y-currY),0);
    currX = x;
    currY = y;
  }
  if(currBtn ==  GLUT_RIGHT_BUTTON){
    camera.Move(0,0,(float)(y-currY)/8.0f);
    currX = x;
    currY = y;
  }
}

void OnIdle(){
  usleep(20000);
  if(bCalibRequest){
    if(calibTimer.IsDone()){
      bCalibRequest = false;    
      xsens.Calibrate();
    }
  }

  bool rtDone=refreshTimer.IsDone();
  if(rtDone){    
    refreshTimer.Start(refreshPeriod);
    if(bUseXSens){
      xsens.GetData();
    }
    if(bUseGloves){
      gloves.Update();
      mSensorBody.mHandLeft  = gloves.GetSensorValue(FD_HAND_LEFT);
      mSensorBody.mHandRight = gloves.GetSensorValue(FD_HAND_RIGHT);
    }else{
      for(int i=0;i<14;i++){
        mSensorBody.mHandLeft.mSensorsArray[i]    = R_ZERO;
        mSensorBody.mHandRight.mSensorsArray[i]   = R_ZERO;
      }
    }
    
    //CalculateHoap3Angles();
    UpdateSkel();
    ComputeAngles();
    //PrintHoap3Angles();


    
    GlovesToICub(&mSensorBody.mHandLeft.mSensors,&mICubHandRight);
    if(bUseYarp)    
      SendDataToYarp();
    
  }
  if(bUseNetwork){
    hoap3Net.SendMessage((char*)&hoap3angles,sizeof(BasicUpperBodyJointAngles));
    hoap3Net.Step();        
  }
  if(rtDone){    
    if(bReadFile){
      float f;
      jfile >> f;
      if(jfile.eof()) {
        bReadFile = false;
        jfile.close();  
      }
          
      for(int i=0;i<sizeof(BasicUpperBodyJointAngles)/sizeof(float);i++){
        jfile >> f;
        ((float*)&hoap3angles)[i] = f;
      }  
      for(int i=0;i<32+28+1;i++){
        jfile >> f;
      }  
    }
  }
  if(bUseGUI){
    Display();  
  }
  if(rtDone){
    if(bSavingData){
      UpdateSavingData();
    }
  }
}


void  UpdateSkel(){
  if(!bShowPose){
    if(bUseXSens){
      Matrix3 tmp;
      for(int i=0;i<BID_SIZE;i++){
        mSensorBody.mBodyMatricesTransform[i].Mult(xsens.GetOutput(mSensorBody.mBodyMatricesIndex[i]),tmp);      
        mSensorBody.mBodyMatricesZero[i].Mult(tmp,mSensorBody.mBodyMatrices[i]);
      }
    }else{
      
      for(int i=0;i<BID_SIZE;i++){
        mSensorBody.mBodyMatrices[i] = mSensorBody.mBodyMatricesZero[i];
      }

      static Vector3 psv,pev,pwv;
      
      //cout << masterTime<<" "<<localTime<<endl;
      masterTime += 0.05;
      if(masterTime>2)
        localTime  += 0.05;
      
//          static float time = 0.0;
//          time += 0.05;
          
      Matrix3 shoulder;
      Matrix3 elbow;
      Matrix3 wrist;

      double val = sin(localTime); ;

      double sfe = -(hoap3min.lsfe +(hoap3max.lsfe-hoap3min.lsfe)/2 + val* (hoap3max.lsfe-hoap3min.lsfe)/2);
      double saa = -(hoap3min.lsaa +(hoap3max.lsaa-hoap3min.lsaa)/2 + val* (hoap3max.lsaa-hoap3min.lsaa)/2);
      double shr = -(hoap3min.lshr +(hoap3max.lshr-hoap3min.lshr)/2 + val* (hoap3max.lshr-hoap3min.lshr)/2);
      double eb  =  (hoap3min.leb  +(hoap3max.leb -hoap3min.leb )/2 + val* (hoap3max.leb -hoap3min.leb )/2);
      double wr  = -(hoap3min.lwr  +(hoap3max.lwr -hoap3min.lwr )/2 + val* (hoap3max.lwr -hoap3min.lwr )/2);
      double wp  =  (hoap3min.lwp  +(hoap3max.lwp -hoap3min.lwp )/2 + val* (hoap3max.lwp -hoap3min.lwp )/2);
      double wy  = -(hoap3min.lwy  +(hoap3max.lwy -hoap3min.lwy )/2 + val* (hoap3max.lwy -hoap3min.lwy )/2);


      Vector3 sv,ev,wv;
      switch(defMotionId){
      case 0:
        sv.Set(DEG2RAD(30),DEG2RAD(-20),0);
        ev.Set(DEG2RAD(90),0,0);
        wv.Set(0,0,0);
        break;
      case 1:
        sv.Set(sfe,DEG2RAD(-20),0);
        ev.Set(DEG2RAD(90),0,0);
        wv.Set(0,0,0);
        break;
      case 2:
        sv.Set(DEG2RAD(30),saa,0);
        ev.Set(DEG2RAD(90),0,0);
        wv.Set(0,0,0);
        break;
      case 3:
        sv.Set(DEG2RAD(30),DEG2RAD(-20),shr);
        ev.Set(DEG2RAD(90),0,0);
        wv.Set(0,0,0);
        break;
      case 4:
        sv.Set(DEG2RAD(30),DEG2RAD(-20),0);
        ev.Set(eb,0,0);
        wv.Set(0,0,0);
        break;
      case 5:
        sv.Set(DEG2RAD(30),DEG2RAD(-20),0);
        ev.Set(DEG2RAD(90),0,0);
        wv.Set(wr ,0,0);
        break;
      case 6:
        sv.Set(DEG2RAD(30),DEG2RAD(-20),0);
        ev.Set(DEG2RAD(90),0,0);
        wv.Set(0,wp ,0);
        break;
      case 7:
        sv.Set(DEG2RAD(30),DEG2RAD(-20),0);
        ev.Set(DEG2RAD(90),0,0);
        wv.Set(0,0,wy );
        break;
      }      


      if(masterTime>2){
        psv = sv;
        pev = ev;
        pwv = wv;        
      }else{
        double val = masterTime/2;
        sv = sv*val + psv*(1.0-val);   
        ev = ev*val + pev*(1.0-val);   
        wv = wv*val + pwv*(1.0-val);   
      }

      shoulder.EulerRotation(2,1,0,sv);
      elbow.EulerRotation   (2,1,0,ev);
      wrist.EulerRotation   (0,1,2,wv);

      Matrix3 tmp,tmp2;

      // SHoulder
      mSensorBody.mBodyMatricesTransform[BID_RIGHTSHOULDER].Mult(shoulder,tmp);      
      mSensorBody.mBodyMatricesZero[BID_RIGHTSHOULDER].Mult(tmp,mSensorBody.mBodyMatrices[BID_RIGHTSHOULDER]);
        
      //Elbow
      mSensorBody.mBodyMatricesTransform[BID_RIGHTELBOW].Mult(shoulder,tmp);      
      tmp.Mult(elbow,tmp2);      
      mSensorBody.mBodyMatricesZero[BID_RIGHTELBOW].Mult(tmp2,mSensorBody.mBodyMatrices[BID_RIGHTELBOW]);

      //Wrist
      mSensorBody.mBodyMatricesTransform[BID_RIGHTWRIST].Mult(shoulder,tmp);      
      tmp.Mult(elbow,tmp2);      
      tmp2.Mult(wrist,tmp);      
      mSensorBody.mBodyMatricesZero[BID_RIGHTWRIST].Mult(tmp,mSensorBody.mBodyMatrices[BID_RIGHTWRIST]);

      sv(0) = -sv(0);
      sv(2) = -sv(2);
      ev(0) = -ev(0);
      wv(0) = -wv(0);
      wv(2) = -wv(2);

      shoulder.EulerRotation(2,1,0,sv);
      elbow.EulerRotation   (2,1,0,ev);
      wrist.EulerRotation   (0,1,2,wv);

      // SHoulder
      mSensorBody.mBodyMatricesTransform[BID_LEFTSHOULDER].Mult(shoulder,tmp);      
      mSensorBody.mBodyMatricesZero[BID_LEFTSHOULDER].Mult(tmp,mSensorBody.mBodyMatrices[BID_LEFTSHOULDER]);
        
      //Elbow
      mSensorBody.mBodyMatricesTransform[BID_LEFTELBOW].Mult(shoulder,tmp);      
      tmp.Mult(elbow,tmp2);      
      mSensorBody.mBodyMatricesZero[BID_LEFTELBOW].Mult(tmp2,mSensorBody.mBodyMatrices[BID_LEFTELBOW]);

      //Wrist
      mSensorBody.mBodyMatricesTransform[BID_LEFTWRIST].Mult(shoulder,tmp);      
      tmp.Mult(elbow,tmp2);      
      tmp2.Mult(wrist,tmp);      
      mSensorBody.mBodyMatricesZero[BID_LEFTWRIST].Mult(tmp,mSensorBody.mBodyMatrices[BID_LEFTWRIST]);

    }
  }else{
    Matrix3 tmp;
    for(int i=0;i<BID_SIZE;i++){
      mSensorBody.mBodyMatricesTransform[i].Mult(posesMatrices[poseCount][i],tmp);      
      mSensorBody.mBodyMatricesZero[i].Mult(tmp,mSensorBody.mBodyMatrices[i]);
    }    
  }
}



void  ArmJointsToICub();
void  UpdateDataString(){
  //ArmJointsToICub();

  mICubArmRight.mSFE = RAD2DEG(1) * -hoap3angles.rsfe;
  mICubArmRight.mSAA = RAD2DEG(1) * -hoap3angles.rsaa;
  mICubArmRight.mSHR = RAD2DEG(1) * -hoap3angles.rshr;
  mICubArmRight.mEB  = RAD2DEG(1) *  hoap3angles.reb ;
  mICubArmRight.mWR  = RAD2DEG(1) * -hoap3angles.rwr ;
  mICubArmRight.mWP  = RAD2DEG(1) *  hoap3angles.rwp ;
  mICubArmRight.mWY  = RAD2DEG(1) * -hoap3angles.rwy ;


  dataString[0] = 0; 
  char txt[1024];
  int cnt = sizeof(BasicUpperBodyJointAngles)/sizeof(float);
  
  float time = float(mTimeChrono.ElapsedTimeMs())*0.001;
  sprintf(dataString,"%f",time);

  strcpy(txt,dataString);
  sprintf(dataString,"%s %f %f %f %f %f %f %f",txt,
	  mICubArmRight.mSFE,
	  mICubArmRight.mSAA,
	  mICubArmRight.mSHR,
	  mICubArmRight.mEB,
	  mICubArmRight.mWR,
	  mICubArmRight.mWP,
	  mICubArmRight.mWY);

  for(int i=0;i<14;i++){
   strcpy(txt,dataString);
   sprintf(dataString,"%s %f",txt,mSensorBody.mHandLeft.mSensorsArray[i]);
  }
  return;
  /*
  float *data = (float*)(&hoap3angles);
  dataString[0] = 0; 
  char txt[1024];
  int cnt = sizeof(BasicUpperBodyJointAngles)/sizeof(float);
  
  float time = float(mTimeChrono.ElapsedTimeMs())*0.001;
  sprintf(dataString,"%f",time);
  
  for(int i=0;i<cnt ;i++){
    strcpy(txt,dataString);
    sprintf(dataString,"%s %f",txt,*(data++));
  }  
  for(int i=0;i<16;i++){
    double d = *(rightHandHMatrix.GetArray()+i);
    float f = float(d);  
    strcpy(txt,dataString);
    sprintf(dataString,"%s %f",txt,f);
  }
  for(int i=0;i<16;i++){
    double d = *(leftHandHMatrix.GetArray()+i);
    float f = float(d);  
    strcpy(txt,dataString);
    sprintf(dataString,"%s %f",txt,f);
  }
  for(int i=0;i<14;i++){
    float f = float(mSensorBody.mHandRight.mSensorsArray[i]);  
    strcpy(txt,dataString);
    sprintf(dataString,"%s %f",txt,f);
  }
  for(int i=0;i<14;i++){
    float f = float(mSensorBody.mHandLeft.mSensorsArray[i]);  
    strcpy(txt,dataString);
    sprintf(dataString,"%s %f",txt,f);
  }
  
  
  strcpy(txt,dataString);
  sprintf(dataString,"%s %f",txt,(bHandsOn?1.0f:0.0f));
  
  */  
}

void  StartSavingData(string filename){
  if(dataFile.is_open()){
    dataFile.close();
  }
  dataFile.open(filename.c_str());
  mTimeChrono.Start();
  bSavingData = true;
}

void  UpdateSavingData(){
  UpdateDataString();
  if(dataFile.is_open()){
    dataFile << dataString << endl;
  }
}
void  StopSavingData(){
  if(dataFile.is_open()){
    dataFile.close();
  }
  bSavingData = false;
}


char* CleanPrintFloat(float f, int nup,int ndown){
  static char txt[256];
  txt[0] = ' ';//
  int cnt = 1;
  float tf  = fabs(f);
  bool bZero=false;
  for(int i=nup-1;i>=0;i--){
    float exp = powf(10.0f,float(i)); 
    float mf = floorf(tf / exp);
    int   df = (mf<9.5?int(mf):-1);
    tf = tf - mf*exp;
    if((df>0)&&(!bZero)){
      bZero = true;
      txt[cnt-1]=(f<0.0f?'-':' ');
    }
    if(df==0){
      if(bZero)
        txt[cnt] = '0';
      else
        txt[cnt] = ' ';
    }else if(df>0){
      txt[cnt] = '0'+df;
    }else
      txt[cnt] = 'x';
    cnt++;        
  }   
  if(!bZero){
    txt[cnt-1] = '0';  
  }
  if(ndown>0){
    txt[cnt++] = '.';
    for(int i=1;i<=ndown;i++){
      float exp = powf(10.0f,float(-i)); 
      float mf = floorf(tf / exp);
      int   df = (mf<9.5?int(mf):-1);
      tf = tf - mf*exp;
      if(df>=0)
        txt[cnt] = '0'+df;
      else
        txt[cnt] = 'x';
      cnt++;        
    }       
  }
  txt[cnt]=0;
  return txt;
}

void  PrintHoap3Angles(){
  float *data = (float*)(&hoap3angles);
  int cnt = sizeof(BasicUpperBodyJointAngles)/sizeof(float);
  for(int i=0;i<cnt ;i++){
    printf("%s ",CleanPrintFloat(-RAD2DEG(*(data)),3,2));
    data++;
  }  
  printf("\n");  
}







////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
bool bFirst = true;

void ComputeAngles(){
  
  if(bFirst){
    memset(&hoap3anglesPrev,    0,           sizeof(BasicUpperBodyJointAngles));
    bFirst = false; 
  }
  memcpy(&hoap3anglesPrev,&hoap3angles,sizeof(BasicUpperBodyJointAngles));
  memset(&hoap3angles,    0,           sizeof(BasicUpperBodyJointAngles));
  for(int i=0;i<BID_SIZE;i++){
    mSensorBody.mBodyMatricesReconstructed[i] = mSensorBody.mBodyMatricesZero[i];
  }  
  
  const REALTYPE SING_RADIUS_MAX = DEG2RAD(10.0);
  const REALTYPE SING_RADIUS_MIN = DEG2RAD(5.0);
  
  Matrix3 Rtmp,Rhr,Rd;
  Matrix3 R,RE,RT,R0,R1,R2,R3,Rx,Ry,Rz;
  Vector3 V0,V1,V2,VC;
  Vector3 p,v;
  Vector3 eulAng[2];
  Vector3 currV;
  Vector curr(7);
  float ang;
  int   currId  = 0;
  float currVal = 1000000.0;
  
  int choiceCnt = 8;
  Vector *vChoice, *aChoice;
  vChoice = new Vector[choiceCnt];
  aChoice = new Vector[choiceCnt];
  for(int i=0;i<choiceCnt;i++){
    vChoice[i].Resize(9); 
    aChoice[i].Resize(7); 
  }
  
  int   selId=0;
  
  Matrix3 RFwd;
  
  float f,dot,delta;
  
  Vector3 ex(1.0f,0.0f,0.0f);
  Vector3 ey(0.0f,1.0f,0.0f);
  Vector3 ez(0.0f,0.0f,1.0f);
  
  
  // TORSO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  R = mSensorBody.mBodyMatricesZero[BID_TORSO].Transpose() * mSensorBody.mBodyMatrices[BID_TORSO];
  
  R.GetEulerAnglesYZX(true, eulAng[0], 0);
  R.GetEulerAnglesYZX(true, eulAng[1], 1);
  
  // Forward Kinematics
  RE.EulerRotation(1,2,0,eulAng[0]);
  
  mSensorBody.mBodyMatricesReconstructed[BID_TORSO] = mSensorBody.mBodyMatricesZero[BID_TORSO] * RE; 
  

  currV.Set(hoap3anglesPrev.torso0,hoap3anglesPrev.torso1,hoap3anglesPrev.torso2);
  V0 = eulAng[0];
  V1 = eulAng[1];
  if((currV-V0).Norm() < (currV-V1).Norm())
    selId = 0;


  hoap3angles.torso0 = eulAng[selId].cx();
  hoap3angles.torso1 = eulAng[selId].cy();
  hoap3angles.torso2 = eulAng[selId].cz();
  
  
  
  // LEFTSHOULDER %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  R = mSensorBody.mBodyMatricesZero[BID_LEFTSHOULDER].Transpose() * mSensorBody.mBodyMatricesZero[BID_TORSO] * mSensorBody.mBodyMatrices[BID_TORSO].Transpose() * mSensorBody.mBodyMatrices[BID_LEFTSHOULDER];

  R.GetEulerAnglesZYX(true, eulAng[0], 0);
  R.GetEulerAnglesZYX(true, eulAng[1], 1);

  for(int i=0;i<choiceCnt/2;i++){  int i1 = i; int i2 = i+choiceCnt/2;
    for(int j=0;j<3;j++){
      vChoice[i1][j] = eulAng[0][j];  vChoice[i2][j] = eulAng[1][j];}}

  // Forward Kinematics
  RE.EulerRotation(2,1,0,eulAng[selId]);

  mSensorBody.mBodyMatricesReconstructed[BID_LEFTSHOULDER]  = mSensorBody.mBodyMatricesReconstructed[BID_TORSO] *  mSensorBody.mBodyMatricesZero[BID_TORSO].Transpose() * mSensorBody.mBodyMatricesZero[BID_LEFTSHOULDER];  
  mSensorBody.mBodyMatricesReconstructed[BID_LEFTSHOULDER] *= RE; 


  // LEFTELBOW %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  R = mSensorBody.mBodyMatricesZero[BID_LEFTELBOW].Transpose() * mSensorBody.mBodyMatricesZero[BID_LEFTSHOULDER] * mSensorBody.mBodyMatrices[BID_LEFTSHOULDER].Transpose() * mSensorBody.mBodyMatrices[BID_LEFTELBOW];

  R.GetEulerAnglesXZX(true, eulAng[0], 0);
  R.GetEulerAnglesXZX(true, eulAng[1], 1);

  for(int i=0;i<choiceCnt/2;i++){  int i1 = (i<2?i:i+2); int i2 = i1+2;
    for(int j=0;j<3;j++){
      vChoice[i1][3+j] = eulAng[0][j];  vChoice[i2][3+j] = eulAng[1][j];}}

  // Forward Kinematics
  RE.EulerRotation(0,2,0,eulAng[selId]);

  mSensorBody.mBodyMatricesReconstructed[BID_LEFTELBOW]  = mSensorBody.mBodyMatricesReconstructed[BID_LEFTSHOULDER] *  mSensorBody.mBodyMatricesZero[BID_LEFTSHOULDER].Transpose() * mSensorBody.mBodyMatricesZero[BID_LEFTELBOW];  
  mSensorBody.mBodyMatricesReconstructed[BID_LEFTELBOW] *= RE; 
  

  // LEFTWRIST %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  R = mSensorBody.mBodyMatricesZero[BID_LEFTWRIST].Transpose() * mSensorBody.mBodyMatricesZero[BID_LEFTELBOW] * mSensorBody.mBodyMatrices[BID_LEFTELBOW].Transpose() * mSensorBody.mBodyMatrices[BID_LEFTWRIST];

  R.GetEulerAnglesXYZ(true, eulAng[0], 0);
  R.GetEulerAnglesXYZ(true, eulAng[1], 1);

  for(int i=0;i<choiceCnt/2;i++){  int i1 = i*2; int i2 = i*2+1;
    for(int j=0;j<3;j++){
      vChoice[i1][6+j] = eulAng[0][j];  vChoice[i2][6+j] = eulAng[1][j];}}

  // Forward Kinematics
  RE.EulerRotation(0,1,2,eulAng[selId]);

  mSensorBody.mBodyMatricesReconstructed[BID_LEFTWRIST]  = mSensorBody.mBodyMatricesReconstructed[BID_LEFTELBOW] *  mSensorBody.mBodyMatricesZero[BID_LEFTELBOW].Transpose() * mSensorBody.mBodyMatricesZero[BID_LEFTWRIST];  
  mSensorBody.mBodyMatricesReconstructed[BID_LEFTWRIST] *= RE; 


  // LEFT CONSTRAINTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


  for(int i=0;i<choiceCnt;i++){
    aChoice[i][0] = vChoice[i][0]; aChoice[i][1] = vChoice[i][1]; aChoice[i][3] = vChoice[i][4]; 
    aChoice[i][5] = vChoice[i][7]; aChoice[i][6] = vChoice[i][8];

    REALTYPE s = vChoice[i][2] + vChoice[i][3] + vChoice[i][5] + vChoice[i][6];    PTRUNC(s);
    
    if(fabs(vChoice[i][4])>SING_RADIUS_MAX){
      aChoice[i][2] = vChoice[i][2] + vChoice[i][3];
      aChoice[i][4] = vChoice[i][5] + vChoice[i][6];
    }else if(fabs(vChoice[i][4])>SING_RADIUS_MIN){
      REALTYPE alpha = (fabs(vChoice[i][4])-SING_RADIUS_MIN)/(SING_RADIUS_MAX-SING_RADIUS_MIN); 
      aChoice[i][2] = alpha * (vChoice[i][2] + vChoice[i][3]) + (R_ONE-alpha)* s/2.0;
      aChoice[i][4] = alpha * (vChoice[i][5] + vChoice[i][6]) + (R_ONE-alpha)* s/2.0;
    }else{
      aChoice[i][2] = s/2;
      aChoice[i][4] = s/2;      
    }

    REALTYPE sgn0 = vChoice[i][2] + vChoice[i][3];  PTRUNC(sgn0);
    REALTYPE sgn1 = aChoice[i][2];                  PTRUNC(sgn1);
    if(cos(sgn0)*cos(sgn1) + sin(sgn0)*sin(sgn1) <0){
      aChoice[i][3] = -aChoice[i][3];
    }
 
    PTRUNC(aChoice[i][2]);
    PTRUNC(aChoice[i][4]);
  }
  

  // LEFT SELECTION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  curr[0] = hoap3anglesPrev.lsfe;  curr[1] = hoap3anglesPrev.lsaa;  curr[2] = hoap3anglesPrev.lshr;
  curr[3] = hoap3anglesPrev.leb;
  curr[4] = hoap3anglesPrev.lwr;   curr[5] = hoap3anglesPrev.lwp;   curr[6] = hoap3anglesPrev.lwy;
  
  currId  = 0;
  currVal = 1000000.0;
  for(int i=0;i<choiceCnt;i++){
    float nextVal = (curr-aChoice[i]).Norm();
    if(nextVal < currVal){
      currId  = i;
      currVal = nextVal;
    }
  }

  hoap3angles.lsfe = aChoice[currId][0];  hoap3angles.lsaa = aChoice[currId][1];  hoap3angles.lshr = aChoice[currId][2];
  hoap3angles.leb  = aChoice[currId][3];
  hoap3angles.lwr  = aChoice[currId][4];  hoap3angles.lwp  = aChoice[currId][5];  hoap3angles.lwy  = aChoice[currId][6];

  







  // RIGHTSHOULDER %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  R = mSensorBody.mBodyMatricesZero[BID_RIGHTSHOULDER].Transpose() * mSensorBody.mBodyMatricesZero[BID_TORSO] * mSensorBody.mBodyMatrices[BID_TORSO].Transpose() * mSensorBody.mBodyMatrices[BID_RIGHTSHOULDER];

  R.GetEulerAnglesZYX(true, eulAng[0], 0);
  R.GetEulerAnglesZYX(true, eulAng[1], 1);

  for(int i=0;i<choiceCnt/2;i++){  int i1 = i; int i2 = i+choiceCnt/2;
    for(int j=0;j<3;j++){
      vChoice[i1][j] = eulAng[0][j];  vChoice[i2][j] = eulAng[1][j];}}

  // Forward Kinematics
  RE.EulerRotation(2,1,0,eulAng[selId]);

  mSensorBody.mBodyMatricesReconstructed[BID_RIGHTSHOULDER]  = mSensorBody.mBodyMatricesReconstructed[BID_TORSO] *  mSensorBody.mBodyMatricesZero[BID_TORSO].Transpose() * mSensorBody.mBodyMatricesZero[BID_RIGHTSHOULDER];  
  mSensorBody.mBodyMatricesReconstructed[BID_RIGHTSHOULDER] *= RE; 


  // RIGHTELBOW %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  R = mSensorBody.mBodyMatricesZero[BID_RIGHTELBOW].Transpose() * mSensorBody.mBodyMatricesZero[BID_RIGHTSHOULDER] * mSensorBody.mBodyMatrices[BID_RIGHTSHOULDER].Transpose() * mSensorBody.mBodyMatrices[BID_RIGHTELBOW];

  R.GetEulerAnglesXZX(true, eulAng[0], 0);
  R.GetEulerAnglesXZX(true, eulAng[1], 1);

  for(int i=0;i<choiceCnt/2;i++){  int i1 = (i<2?i:i+2); int i2 = i1+2;
    for(int j=0;j<3;j++){
      vChoice[i1][3+j] = eulAng[0][j];  vChoice[i2][3+j] = eulAng[1][j];}}

  // Forward Kinematics
  RE.EulerRotation(0,2,0,eulAng[selId]);

  mSensorBody.mBodyMatricesReconstructed[BID_RIGHTELBOW]  = mSensorBody.mBodyMatricesReconstructed[BID_RIGHTSHOULDER] *  mSensorBody.mBodyMatricesZero[BID_RIGHTSHOULDER].Transpose() * mSensorBody.mBodyMatricesZero[BID_RIGHTELBOW];  
  mSensorBody.mBodyMatricesReconstructed[BID_RIGHTELBOW] *= RE; 
  

  // RIGHTWRIST %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  R = mSensorBody.mBodyMatricesZero[BID_RIGHTWRIST].Transpose() * mSensorBody.mBodyMatricesZero[BID_RIGHTELBOW] * mSensorBody.mBodyMatrices[BID_RIGHTELBOW].Transpose() * mSensorBody.mBodyMatrices[BID_RIGHTWRIST];

  R.GetEulerAnglesXYZ(true, eulAng[0], 0);
  R.GetEulerAnglesXYZ(true, eulAng[1], 1);

  for(int i=0;i<choiceCnt/2;i++){  int i1 = i*2; int i2 = i*2+1;
    for(int j=0;j<3;j++){
      vChoice[i1][6+j] = eulAng[0][j];  vChoice[i2][6+j] = eulAng[1][j];}}

  // Forward Kinematics
  RE.EulerRotation(0,1,2,eulAng[selId]);

  mSensorBody.mBodyMatricesReconstructed[BID_RIGHTWRIST]  = mSensorBody.mBodyMatricesReconstructed[BID_RIGHTELBOW] *  mSensorBody.mBodyMatricesZero[BID_RIGHTELBOW].Transpose() * mSensorBody.mBodyMatricesZero[BID_RIGHTWRIST];  
  mSensorBody.mBodyMatricesReconstructed[BID_RIGHTWRIST] *= RE; 


  // RIGHT CONSTRAINTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


  for(int i=0;i<choiceCnt;i++){
    aChoice[i][0] = vChoice[i][0]; aChoice[i][1] = vChoice[i][1]; aChoice[i][3] = vChoice[i][4]; 
    aChoice[i][5] = vChoice[i][7]; aChoice[i][6] = vChoice[i][8];

    REALTYPE s = vChoice[i][2] + vChoice[i][3] + vChoice[i][5] + vChoice[i][6];    PTRUNC(s);
    
    if(fabs(vChoice[i][4])>SING_RADIUS_MAX){
      aChoice[i][2] = vChoice[i][2] + vChoice[i][3];
      aChoice[i][4] = vChoice[i][5] + vChoice[i][6];
    }else if(fabs(vChoice[i][4])>SING_RADIUS_MIN){
      REALTYPE alpha = (fabs(vChoice[i][4])-SING_RADIUS_MIN)/(SING_RADIUS_MAX-SING_RADIUS_MIN); 
      aChoice[i][2] = alpha * (vChoice[i][2] + vChoice[i][3]) + (R_ONE-alpha)* s/2.0;
      aChoice[i][4] = alpha * (vChoice[i][5] + vChoice[i][6]) + (R_ONE-alpha)* s/2.0;
    }else{
      aChoice[i][2] = s/2;
      aChoice[i][4] = s/2;      
    }

    REALTYPE sgn0 = vChoice[i][2] + vChoice[i][3];  PTRUNC(sgn0);
    REALTYPE sgn1 = aChoice[i][2];                  PTRUNC(sgn1);
    if(cos(sgn0)*cos(sgn1) + sin(sgn0)*sin(sgn1) <0){
      aChoice[i][3] = -aChoice[i][3];
    }
 
    PTRUNC(aChoice[i][2]);
    PTRUNC(aChoice[i][4]);
  }
  

  // RIGHT SELECTION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  curr[0] = hoap3anglesPrev.rsfe;  curr[1] = hoap3anglesPrev.rsaa;  curr[2] = hoap3anglesPrev.rshr;
  curr[3] = hoap3anglesPrev.reb;
  curr[4] = hoap3anglesPrev.rwr;   curr[5] = hoap3anglesPrev.rwp;   curr[6] = hoap3anglesPrev.rwy;
  
  currId  = 0;
  currVal = 1000000.0;
  for(int i=0;i<choiceCnt;i++){
    float nextVal = (curr-aChoice[i]).Norm();
    if(nextVal < currVal){
      currId  = i;
      currVal = nextVal;
    }
  }

  hoap3angles.rsfe = aChoice[currId][0];  hoap3angles.rsaa = aChoice[currId][1];  hoap3angles.rshr = aChoice[currId][2];
  hoap3angles.reb  = aChoice[currId][3];
  hoap3angles.rwr  = aChoice[currId][4];  hoap3angles.rwp  = aChoice[currId][5];  hoap3angles.rwy  = aChoice[currId][6];




  /*
  hoap3angles.lsfe = TRUNC(hoap3angles.lsfe   , hoap3min.lsfe   , hoap3max.lsfe);
  
  hoap3angles.rsfe = TRUNC(hoap3angles.rsfe   , hoap3min.rsfe   , hoap3max.rsfe);
  hoap3angles.rsaa = TRUNC(hoap3angles.rsaa   , hoap3min.rsaa   , hoap3max.rsaa);
  hoap3angles.rshr = TRUNC(hoap3angles.rshr   , hoap3min.rshr   , hoap3max.rshr);
  hoap3angles.reb  = TRUNC(hoap3angles.reb    , hoap3min.reb    , hoap3max.reb );
*/

  /*
  cout << hoap3angles.torso0 << " "<< hoap3angles.torso1 << " "<< hoap3angles.torso2 << " ";
  cout << hoap3angles.lsfe << " "<< hoap3angles.lsaa << " "<< hoap3angles.lshr << " ";
  cout << hoap3angles.leb << " ";
  cout << hoap3angles.lwr << " "<< hoap3angles.lwp << " "<< hoap3angles.lwy << " ";
  cout << hoap3angles.rsfe << " "<< hoap3angles.rsaa << " "<< hoap3angles.rshr << " ";
  cout << hoap3angles.reb << " ";
  cout << hoap3angles.rwr << " "<< hoap3angles.rwp << " "<< hoap3angles.rwy << " ";
  cout << endl;
*/

  delete [] vChoice;
  delete [] aChoice;

}

////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////


void ComputeAnglesASD(){
  
  memcpy(&hoap3anglesPrev,&hoap3angles,sizeof(BasicUpperBodyJointAngles));
  
  const REALTYPE SEC_RADIUS = DEG2RAD(15.0);
  const REALTYPE MAX_VEL    = DEG2RAD(30.0);  
  
  Matrix3 R,R0,Rtmp,Rhr,Rd,Rx,Ry,Rz;
  Vector3 p,v;
  Vector3 ang;
  
  Matrix3 RFwd;
  
  float f,delta;
  
  Vector3 ex(1.0f,0.0f,0.0f);
  Vector3 ey(0.0f,1.0f,0.0f);
  Vector3 ez(0.0f,0.0f,1.0f);

  //cout << "C0"<<endl;

  // HEAD
  R0    = mSensorBody.mBodyMatrices[BID_TORSO];
  R     = mSensorBody.mBodyMatrices[BID_HEAD];
  Rtmp  = R;
  R0.Transpose().Mult(Rtmp,R);
  p = -R.GetColumn(2);
  hoap3angles.hpan = -atan2(p(1),p(2));
  hoap3angles.hflx = asin(p(0));
  hoap3angles.htlt = 0.0f;
  hoap3angles.hflx = TRUNC(hoap3angles.hflx,hoap3min.hflx,hoap3max.hflx);
  hoap3angles.hpan = TRUNC(hoap3angles.hpan,hoap3min.hpan,hoap3max.hpan);
  
  
  
  
  // TORSO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  R = mSensorBody.mBodyMatrices[BID_TORSO];
  p = R.GetColumn(2);
  f = p(0);
  p(0) = 0.0f;
  if(p.Norm2()>EPSILON){
    p.Normalize();
    if(-f<sin(hoap3max.torso))
      hoap3angles.torso = asin(R.GetColumn(0).Dot(p));
    else
      hoap3angles.torso = hoap3max.torso;
    if(R.GetColumn(0)[0]<0.0f)
      hoap3angles.torso = hoap3max.torso; 
  }else{
    hoap3angles.torso = hoap3anglesPrev.torso;// PIf/2.0f;
  }
  if(f>0.0f){
    hoap3angles.torso = 0.0f;    
  }
  
  //printf("%f\n",RAD2DEG(hoap3angles.torso));


  // Forward Kinematics
  mSensorBody.mBodyMatricesReconstructed[BID_TORSO] = mSensorBody.mBodyMatrices[BID_TORSO];
  
  
  
  
  R0    = mSensorBody.mBodyMatrices[BID_TORSO];
  Rtmp  = mSensorBody.mBodyMatrices[BID_LEFTSHOULDER];
  R0.Transpose().Mult(Rtmp,R);
  
  R.Print();
  R.GetEulerAnglesZYX(true, ang, 1);
  
  
  // LSFE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  R0    = mSensorBody.mBodyMatrices[BID_TORSO];
  Rtmp  = mSensorBody.mBodyMatrices[BID_LEFTSHOULDER];
  R0.Transpose().Mult(Rtmp,R);

  if(fabs(R(1,0))<0.95f)
    hoap3angles.lsfe = -atan2(R(0,0),R(2,0));
  else
    hoap3angles.lsfe = hoap3angles.lsfe;  
    
    //cout << hoap3angles.lsfe<<endl;
  // Forward Kinematics
  mSensorBody.mBodyMatricesReconstructed[BID_TORSO].Mult(mSensorBody.mBodyMatricesZero[BID_LEFTSHOULDER  ],mSensorBody.mBodyMatricesReconstructed[BID_LEFTSHOULDER]);

  Ry.RotationZ(-PIf/2.0f+hoap3angles.lsfe);
  Rtmp = mSensorBody.mBodyMatricesReconstructed[BID_LEFTSHOULDER];
  Rtmp.Mult(Ry,mSensorBody.mBodyMatricesReconstructed[BID_LEFTSHOULDER]);

  //mSensorBody.mBodyMatricesReconstructed[BID_TORSO].Mult(Ry,mSensorBody.mBodyMatricesReconstructed[BID_LEFTSHOULDER]);
    
  // LSAA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  hoap3angles.lsaa = -asinf(R(1,0));


  // Forward Kinematics
  Rz.RotationY(-hoap3angles.lsaa);
  Rtmp = mSensorBody.mBodyMatricesReconstructed[BID_LEFTSHOULDER];
  Rtmp.Mult(Rz,mSensorBody.mBodyMatricesReconstructed[BID_LEFTSHOULDER]);

#ifdef TRUNC_ANGLES
  // Trunc sfe n saa
  hoap3angles.lsfe = TRUNC(hoap3angles.lsfe,hoap3min.lsfe,hoap3max.lsfe);
  hoap3angles.lsaa = TRUNC(hoap3angles.lsaa,hoap3min.lsaa,hoap3max.lsaa);
  
  // Smooth sfe at limits
  if(hoap3angles.lsaa>hoap3max.lsaa-SEC_RADIUS){
     float alpha = 1.0f- (hoap3angles.lsaa-hoap3max.lsaa+SEC_RADIUS)/SEC_RADIUS;
     hoap3angles.lsfe  = alpha*hoap3angles.lsfe + (1.0f-alpha)*hoap3anglesPrev.lsfe;
  }
  // trunc velocity
  delta = hoap3angles.lsfe-hoap3anglesPrev.lsfe;
  if(fabs(delta)>MAX_VEL)  hoap3angles.lsfe = hoap3anglesPrev.lsfe+MAX_VEL*SIGN(delta);
  delta = hoap3angles.lsaa-hoap3anglesPrev.lsaa;
  if(fabs(delta)>MAX_VEL)  hoap3angles.lsaa = hoap3anglesPrev.lsaa+MAX_VEL*SIGN(delta);
#endif


  // LEB %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  R0    = mSensorBody.mBodyMatrices[BID_LEFTSHOULDER];
  R     = mSensorBody.mBodyMatrices[BID_LEFTELBOW];
  Rtmp  = R;
  R0.Transpose().Mult(Rtmp,R);
  hoap3angles.leb = atan2(R(1,0),R(0,0));

  // Forward Kinematics
  Ry.RotationY(-hoap3angles.leb);
  mSensorBody.mBodyMatricesReconstructed[BID_LEFTSHOULDER].Mult(Ry,mSensorBody.mBodyMatricesReconstructed[BID_LEFTELBOW]);


#ifdef TRUNC_ANGLES
  // Trunc eb
  hoap3angles.leb = TRUNC(hoap3angles.leb,hoap3min.leb,hoap3max.leb);
  // trunc velocity
  delta = hoap3angles.leb-hoap3anglesPrev.leb;
  if(fabs(delta)>MAX_VEL)  hoap3angles.leb = hoap3anglesPrev.leb+MAX_VEL*SIGN(delta);
#endif

  // LSHR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  R0    = mSensorBody.mBodyMatrices[BID_TORSO];
  R     = mSensorBody.mBodyMatrices[BID_LEFTELBOW];
  R0.Transpose().Mult(R,Rhr);
  Ry.RotationY(-PIf/2.0f-hoap3angles.lsfe);
  Rz.RotationZ(-hoap3angles.lsaa);
  Ry.Mult(Rz,Rd);
  Rd.Transpose().Mult(Rhr.GetColumn(0),p);
  hoap3angles.lshr = atan2(p(1),-p(2));

  //cout << hoap3angles.leb<<endl;
  if(hoap3angles.leb>0.1){
    hoap3angles.lshr = atan2(-p(1),p(2));  
  }else{
    hoap3angles.lshr = 0;
  }
/*
  if(hoap3angles.leb>hoap3max.leb-SEC_RADIUS){
    float alpha = -hoap3angles.leb/(SEC_RADIUS);
    hoap3angles.lshr = alpha *hoap3angles.lshr + (1-alpha)*hoap3angles.lshr;
  }
*/
  
 // if(fabs(hoap3angles.lshr)>PI/2.0)
//    hoap3angles.lshr = atan2(-p(1),p(2));
  //  hoap3angles.lshr = PI/2.0-(hoap3angles.lshr-PI/2.0);
  //if(hoap3angles.lshr<-PI/2.0)
   // hoap3angles.lshr = -PI-hoap3angles.lshr;
     
  //cout << hoap3angles.leb<<" " << hoap3angles.lshr<<endl;
#ifdef TRUNC_ANGLES
  // Trunc shr
  hoap3angles.lshr = TRUNC(hoap3angles.lshr,hoap3min.lshr,hoap3max.lshr);
  // Smooth shr at limits
  if(hoap3angles.leb>hoap3max.leb-SEC_RADIUS){
    float alpha = -hoap3angles.leb/(SEC_RADIUS);
    hoap3angles.lshr = alpha *hoap3angles.lshr; //+(1-alpha)*myJoints.rshr;
  }
  // trunc velocity
  delta = hoap3angles.lshr-hoap3anglesPrev.lshr;
  if(fabs(delta)>MAX_VEL)  hoap3angles.lshr = hoap3anglesPrev.lshr+MAX_VEL*SIGN(delta);
#endif

  //hoap3angles.lshr = 0;

  // Forward Kinematics
  Rx.RotationX(hoap3angles.lshr);
  Rtmp = mSensorBody.mBodyMatricesReconstructed[BID_LEFTSHOULDER];
  Rtmp.Mult(Rx,mSensorBody.mBodyMatricesReconstructed[BID_LEFTSHOULDER]);

  // Forward Kinematics
  Ry.RotationZ(hoap3angles.leb);
  mSensorBody.mBodyMatricesReconstructed[BID_LEFTSHOULDER].Mult(Ry,mSensorBody.mBodyMatricesReconstructed[BID_LEFTELBOW]);


  // Forward Kinematics
  mSensorBody.mBodyMatricesReconstructed[BID_TORSO].Mult(mSensorBody.mBodyMatricesZero[BID_LEFTSHOULDER  ],mSensorBody.mBodyMatricesReconstructed[BID_LEFTSHOULDER]);

  ang.Print();
  cout << hoap3angles.lsfe << " "<<hoap3angles.lsaa<< " " << hoap3angles.lshr<<endl;
  
  Rz.RotationZ(ang(0));
  Rtmp = mSensorBody.mBodyMatricesReconstructed[BID_LEFTSHOULDER];
  Rtmp.Mult(Rz,mSensorBody.mBodyMatricesReconstructed[BID_LEFTSHOULDER]);
  
  Ry.RotationY(ang(1));
  Rtmp = mSensorBody.mBodyMatricesReconstructed[BID_LEFTSHOULDER];
  Rtmp.Mult(Ry,mSensorBody.mBodyMatricesReconstructed[BID_LEFTSHOULDER]);

  Rx.RotationY(ang(2));
  Rtmp = mSensorBody.mBodyMatricesReconstructed[BID_LEFTSHOULDER];
  Rtmp.Mult(Rx,mSensorBody.mBodyMatricesReconstructed[BID_LEFTSHOULDER]);


  // Forward Kinematics
  Ry.RotationZ(hoap3angles.leb);
  mSensorBody.mBodyMatricesReconstructed[BID_LEFTSHOULDER].Mult(Ry,mSensorBody.mBodyMatricesReconstructed[BID_LEFTELBOW]);



  // LWR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  Rx.RotationX(hoap3angles.lshr);
  Ry.RotationY(-hoap3angles.leb);
  Rd.Mult(Rx,Rtmp);
  Rtmp.Mult(Ry,Rd);
  v = Rhr.GetColumn(1) - Rhr.GetColumn(1)*(Rhr.GetColumn(1).Dot(Rd.GetColumn(0)));
  p = Rd.Transpose()*v;
  hoap3angles.lwr = atan2(p(2),p(1))-PIf/2.0f;
  if(hoap3angles.lwr-hoap3anglesPrev.lwr<-PIf)
    hoap3angles.lwr += 2.0f*PIf;
  else if(hoap3angles.lwr-hoap3anglesPrev.lwr>PIf)
    hoap3angles.lwr -= 2.0f*PIf;
#ifdef TRUNC_ANGLES    
  // Trunc wr
  hoap3angles.lwr = -TRUNC(hoap3angles.lwr,hoap3min.lwr,hoap3max.lwr);
  // trunc velocity
  delta = hoap3angles.lwr-hoap3anglesPrev.lwr;
  if(fabs(delta)>MAX_VEL)  hoap3angles.lwr = hoap3anglesPrev.lwr+MAX_VEL*SIGN(delta);
#endif   
   
  REALTYPE jr,jp,jy;
  R     = mSensorBody.mBodyMatrices[BID_LEFTWRIST];
  Rd.Transpose().Mult(R,R0);

  R0    = mSensorBody.mBodyMatrices[BID_LEFTWRIST];
  R     = mSensorBody.mBodyMatrices[BID_LEFTELBOW];
  Rtmp  = R;
  R0.Transpose().Mult(Rtmp,R);

  
  //R.GetZYXEulerAngles(&jr,&jp,&jy);
  hoap3angles.lwr = -jy;
  hoap3angles.lwp = -jp;
  hoap3angles.lwy = jr;

  hoap3angles.lwr = PI;
  hoap3angles.lwp = 0;
  hoap3angles.lwy = 0;

  Ry.RotationYXZ(hoap3angles.lwr,hoap3angles.lwp,hoap3angles.lwy);
  mSensorBody.mBodyMatricesReconstructed[BID_LEFTELBOW].Mult(Ry,mSensorBody.mBodyMatricesReconstructed[BID_LEFTWRIST]);


  //cout << "C1"<<endl;
  
}







#if 0
//#define TRUNC_ANGLES

void CalculateHoap3Angles(){
  
  memcpy(&hoap3anglesPrev,&hoap3angles,sizeof(BasicUpperBodyJointAngles));
  
  const float SEC_RADIUS = DEG2RAD(15.0f);
  const float MAX_VEL    = DEG2RAD(30.0f);  
  
  Matrix3 R,R0,Rtmp,Rhr,Rd,Rx,Ry,Rz;
  Vector3 p,v;
  
  float f,delta;
  
  Vector3 ex(1.0f,0.0f,0.0f);
  Vector3 ey(0.0f,1.0f,0.0f);
  Vector3 ez(0.0f,0.0f,1.0f);

  // HEAD
  //hoap3angles.hflx = h.hn.angle[0];
  //hoap3angles.htlt = h.hr.angle[0];
  //hoap3angles.hpan = h.ht.angle[0];
    
  //R = Matrix3(h.Mt);   R.Print();
  //R = Matrix3(h.Mt_d); R.Print();
  //R = Matrix3(h.Mt_0); R.Print();
  R0    = mSensorBody.torso;
  R     = mSensorBody.head;
  Rtmp  = R;
  R0.Transpose().Mult(Rtmp,R);
  p = -R.GetColumn(2);
  hoap3angles.hpan = -atan2(p(1),p(2));
  hoap3angles.hflx = asin(p(0));
  hoap3angles.htlt = 0.0f;
  hoap3angles.hflx = TRUNC(hoap3angles.hflx,hoap3min.hflx,hoap3max.hflx);
  hoap3angles.hpan = TRUNC(hoap3angles.hpan,hoap3min.hpan,hoap3max.hpan);
  
  
  
  
  // TORSO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  R = mSensorBody.torso;
  /*
  if(R(2,0)!=0.0f){
    p = R.GetColumn(0) - ez*(R.GetColumn(0).Dot(ez));
    f = p.Dot(R.GetColumn(2));
    hoap3angles.torso = atan2(f,R(2,0));
  }else{
    hoap3angles.torso = 0.0f; 
  }
  hoap3angles.torso = 0.0f;
  */
  p = R.GetColumn(2);// - ex*(R.GetColumn(2).Dot(ex));
  f = p(0);
  p(0) = 0.0f;
  if(p.Norm2()>EPSILON){
    p.Normalize();
    if(-f<sin(hoap3max.torso))
      hoap3angles.torso = asin(R.GetColumn(0).Dot(p));
    else
      hoap3angles.torso = hoap3max.torso;
    if(R.GetColumn(0)[0]<0.0f)
      hoap3angles.torso = hoap3max.torso; 
  }else{
    hoap3angles.torso = hoap3anglesPrev.torso;// PIf/2.0f;
  }
  if(f>0.0f){
    hoap3angles.torso = 0.0f;    
  }
  
  //printf("%f\n",RAD2DEG(hoap3angles.torso));


  // RSFE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  R0    = mSensorBody.torso;
  R     = mSensorBody.rightShoulder;
  Rtmp  = R;
  R0.Transpose().Mult(Rtmp,R);
  if(R(1,0)!=1.0f)
    hoap3angles.rsfe = -atan2(R(0,0),R(2,0));
  else
    hoap3angles.rsfe = hoap3angles.rsfe;  
  // RSAA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  hoap3angles.rsaa = -asinf(R(1,0));
  
  // Trunc sfe n saa
  hoap3angles.rsfe = TRUNC(hoap3angles.rsfe,hoap3min.rsfe,hoap3max.rsfe);
  hoap3angles.rsaa = TRUNC(hoap3angles.rsaa,hoap3min.rsaa,hoap3max.rsaa);
  // Smooth sfe at limits
  if(hoap3angles.rsaa<hoap3min.rsaa+SEC_RADIUS){
     float alpha = 1.0f- ((hoap3min.rsaa)+SEC_RADIUS-hoap3angles.rsaa)/SEC_RADIUS;
     hoap3angles.rsfe  = alpha*hoap3angles.rsfe + (1.0f-alpha)*hoap3anglesPrev.rsfe;
  }
  // trunc velocity
  delta = hoap3angles.rsfe-hoap3anglesPrev.rsfe;
  if(fabs(delta)>MAX_VEL)  hoap3angles.rsfe = hoap3anglesPrev.rsfe+MAX_VEL*SIGN(delta);
  delta = hoap3angles.rsaa-hoap3anglesPrev.rsaa;
  if(fabs(delta)>MAX_VEL)  hoap3angles.rsaa = hoap3anglesPrev.rsaa+MAX_VEL*SIGN(delta);


  // REB %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  R0    = mSensorBody.rightShoulder;
  R     = mSensorBody.rightElbow;
  Rtmp  = R;
  R0.Transpose().Mult(Rtmp,R);
  hoap3angles.reb = -atan2(R(1,0),R(0,0));

  // Trunc eb
  hoap3angles.reb = TRUNC(hoap3angles.reb,hoap3min.reb,hoap3max.reb);
  // trunc velocity
  delta = hoap3angles.reb-hoap3anglesPrev.reb;
  if(fabs(delta)>MAX_VEL)  hoap3angles.reb = hoap3anglesPrev.reb+MAX_VEL*SIGN(delta);


  // RSHR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  R0    = mSensorBody.torso;
  R     = mSensorBody.rightElbow;
  R0.Transpose().Mult(R,Rhr);
  Ry.RotationY(-PIf/2.0f-hoap3angles.rsfe);
  Rz.RotationZ(-hoap3angles.rsaa);
  Ry.Mult(Rz,Rd);
  Rd.Transpose().Mult(Rhr.GetColumn(0),p);
  hoap3angles.rshr = atan2(p(1),-p(2));

  // Trunc shr
  hoap3angles.rshr = TRUNC(hoap3angles.rshr,hoap3min.rshr,hoap3max.rshr);
  // Smooth shr at limits
  if(hoap3angles.reb>hoap3max.reb-SEC_RADIUS){
    float alpha = -hoap3angles.reb/(SEC_RADIUS);
    hoap3angles.rshr = alpha *hoap3angles.rshr; //+(1-alpha)*myJoints.rshr;
  }
  // trunc velocity
  delta = hoap3angles.rshr-hoap3anglesPrev.rshr;
  if(fabs(delta)>MAX_VEL)  hoap3angles.rshr = hoap3anglesPrev.rshr+MAX_VEL*SIGN(delta);


  // RWR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  Rx.RotationX(hoap3angles.rshr);
  Ry.RotationY(-hoap3angles.reb);
  Rd.Mult(Rx,Rtmp);
  Rtmp.Mult(Ry,Rd);
  v = Rhr.GetColumn(1) - Rhr.GetColumn(1)*(Rhr.GetColumn(1).Dot(Rd.GetColumn(0)));
  p = Rd.Transpose()*v;
  hoap3angles.rwr = atan2(p(2),p(1))+PIf/2.0f;
  if(hoap3angles.rwr-hoap3anglesPrev.rwr<-PIf)
    hoap3angles.rwr += 2.0f*PIf;
  else if(hoap3angles.rwr-hoap3anglesPrev.rwr>PIf)
    hoap3angles.rwr -= 2.0f*PIf;
  // Trunc wr
  hoap3angles.rwr = -TRUNC(hoap3angles.rwr,hoap3min.rwr,hoap3max.rwr);
  // trunc velocity
  delta = hoap3angles.rwr-hoap3anglesPrev.rwr;
  if(fabs(delta)>MAX_VEL)  hoap3angles.rwr = hoap3anglesPrev.rwr+MAX_VEL*SIGN(delta);

  
  // LSFE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  R0    = mSensorBody.torso;
  R     = mSensorBody.leftShoulder;
  Rtmp  = R;
  R0.Transpose().Mult(Rtmp,R);
  if(R(1,0)!=1.0f)
    hoap3angles.lsfe = -atan2(R(0,0),R(2,0));
  else
    hoap3angles.lsfe = hoap3angles.lsfe;  
  // LSAA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  hoap3angles.lsaa = -asinf(R(1,0));


#ifdef TRUNC_ANGLES
  // Trunc sfe n saa
  hoap3angles.lsfe = TRUNC(hoap3angles.lsfe,hoap3min.lsfe,hoap3max.lsfe);
  hoap3angles.lsaa = TRUNC(hoap3angles.lsaa,hoap3min.lsaa,hoap3max.lsaa);
  
  // Smooth sfe at limits
  if(hoap3angles.lsaa>hoap3max.lsaa-SEC_RADIUS){
     float alpha = 1.0f- (hoap3angles.lsaa-hoap3max.lsaa+SEC_RADIUS)/SEC_RADIUS;
     hoap3angles.lsfe  = alpha*hoap3angles.lsfe + (1.0f-alpha)*hoap3anglesPrev.lsfe;
  }
  // trunc velocity
  delta = hoap3angles.lsfe-hoap3anglesPrev.lsfe;
  if(fabs(delta)>MAX_VEL)  hoap3angles.lsfe = hoap3anglesPrev.lsfe+MAX_VEL*SIGN(delta);
  delta = hoap3angles.lsaa-hoap3anglesPrev.lsaa;
  if(fabs(delta)>MAX_VEL)  hoap3angles.lsaa = hoap3anglesPrev.lsaa+MAX_VEL*SIGN(delta);
#endif


  // LEB %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  R0    = mSensorBody.leftShoulder;
  R     = mSensorBody.leftElbow;
  Rtmp  = R;
  R0.Transpose().Mult(Rtmp,R);
  hoap3angles.leb = atan2(R(1,0),R(0,0));

#ifdef TRUNC_ANGLES
  // Trunc eb
  hoap3angles.leb = TRUNC(hoap3angles.leb,hoap3min.leb,hoap3max.leb);
  // trunc velocity
  delta = hoap3angles.leb-hoap3anglesPrev.leb;
  if(fabs(delta)>MAX_VEL)  hoap3angles.leb = hoap3anglesPrev.leb+MAX_VEL*SIGN(delta);
#endif

  // LSHR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  R0    = mSensorBody.torso;
  R     = mSensorBody.leftElbow;
  R0.Transpose().Mult(R,Rhr);
  Ry.RotationY(-PIf/2.0f-hoap3angles.lsfe);
  Rz.RotationZ(-hoap3angles.lsaa);
  Ry.Mult(Rz,Rd);
  Rd.Transpose().Mult(Rhr.GetColumn(0),p);
  hoap3angles.lshr = atan2(p(1),-p(2));

#ifdef TRUNC_ANGLES
  // Trunc shr
  hoap3angles.lshr = TRUNC(hoap3angles.lshr,hoap3min.lshr,hoap3max.lshr);
  // Smooth shr at limits
  if(hoap3angles.leb>hoap3max.leb-SEC_RADIUS){
    float alpha = -hoap3angles.leb/(SEC_RADIUS);
    hoap3angles.lshr = alpha *hoap3angles.lshr; //+(1-alpha)*myJoints.rshr;
  }
  // trunc velocity
  delta = hoap3angles.lshr-hoap3anglesPrev.lshr;
  if(fabs(delta)>MAX_VEL)  hoap3angles.lshr = hoap3anglesPrev.lshr+MAX_VEL*SIGN(delta);
#endif

  // LWR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  Rx.RotationX(hoap3angles.lshr);
  Ry.RotationY(-hoap3angles.leb);
  Rd.Mult(Rx,Rtmp);
  Rtmp.Mult(Ry,Rd);
  v = Rhr.GetColumn(1) - Rhr.GetColumn(1)*(Rhr.GetColumn(1).Dot(Rd.GetColumn(0)));
  p = Rd.Transpose()*v;
  hoap3angles.lwr = atan2(p(2),p(1))-PIf/2.0f;
  if(hoap3angles.lwr-hoap3anglesPrev.lwr<-PIf)
    hoap3angles.lwr += 2.0f*PIf;
  else if(hoap3angles.lwr-hoap3anglesPrev.lwr>PIf)
    hoap3angles.lwr -= 2.0f*PIf;
#ifdef TRUNC_ANGLES    
  // Trunc wr
  hoap3angles.lwr = -TRUNC(hoap3angles.lwr,hoap3min.lwr,hoap3max.lwr);
  // trunc velocity
  delta = hoap3angles.lwr-hoap3anglesPrev.lwr;
  if(fabs(delta)>MAX_VEL)  hoap3angles.lwr = hoap3anglesPrev.lwr+MAX_VEL*SIGN(delta);
#endif   
   
  REALTYPE jr,jp,jy;
  R     = mSensorBody.leftWrist;
  Rd.Transpose().Mult(R,R0);

  R0    = mSensorBody.leftWrist;
  R     = mSensorBody.leftElbow;
  Rtmp  = R;
  R0.Transpose().Mult(Rtmp,R);

  
  R.GetZYXEulerAngles(&jr,&jp,&jy);
  hoap3angles.lwr = 0;-jy;
  hoap3angles.lwp = -jp;
  hoap3angles.lwy = jr;
  
  //printf("%f %f %f\n",jr,jp,jy);
  
  //printf("%f\n",mSensorBody.torso);
  //printf("%f %f %f %f %f\n",mSensorBody.torso,mSensorBody.rsfe,mSensorBody.rsaa,mSensorBody.rshr,mSensorBody.reb,mSensorBody.rwr);
}

#endif

void  ArmJointsToICub(){

  mICubArmLeft.mSFE  = RAD2DEG(1) * TRUNC( hoap3angles.lsfe   , hoap3min.lsfe   , hoap3max.lsfe);
  mICubArmLeft.mSAA  = RAD2DEG(1) * TRUNC(-hoap3angles.lsaa   , hoap3min.lsaa   , hoap3max.lsaa);
  mICubArmLeft.mSHR  = RAD2DEG(1) * TRUNC( hoap3angles.lshr   , hoap3min.lshr   , hoap3max.lshr);
  mICubArmLeft.mEB   = RAD2DEG(1) * TRUNC(-hoap3angles.leb    , hoap3min.leb    , hoap3max.leb );
  mICubArmLeft.mWR   = RAD2DEG(1) * TRUNC( hoap3angles.lwr    , hoap3min.lwr    , hoap3max.lwr );
  mICubArmLeft.mWP   = RAD2DEG(1) * TRUNC( hoap3angles.lwp    , hoap3min.lwp    , hoap3max.lwp );
  mICubArmLeft.mWY   = RAD2DEG(1) * TRUNC( hoap3angles.lwy    , hoap3min.lwy    , hoap3max.lwy );

  mICubArmRight.mSFE = RAD2DEG(1) * TRUNC(-hoap3angles.rsfe   , hoap3min.rsfe   , hoap3max.rsfe);
  mICubArmRight.mSAA = RAD2DEG(1) * TRUNC(-hoap3angles.rsaa   , hoap3min.rsaa   , hoap3max.rsaa);
  mICubArmRight.mSHR = RAD2DEG(1) * TRUNC(-hoap3angles.rshr   , hoap3min.rshr   , hoap3max.rshr);
  mICubArmRight.mEB  = RAD2DEG(1) * TRUNC( hoap3angles.reb    , hoap3min.reb    , hoap3max.reb );
  mICubArmRight.mWR  = RAD2DEG(1) * TRUNC(-hoap3angles.rwr    , hoap3min.rwr    , hoap3max.rwr );
  mICubArmRight.mWP  = RAD2DEG(1) * TRUNC( hoap3angles.rwp    , hoap3min.rwp    , hoap3max.rwp );
  mICubArmRight.mWY  = RAD2DEG(1) * TRUNC(-hoap3angles.rwy    , hoap3min.rwy    , hoap3max.rwy );
  
  //cout << hoap3angles.rsfe <<" "<< mICubArmRight.mSFE << endl;
  //cout << hoap3angles.rsaa <<" "<< mICubArmRight.mSAA << endl;
  //cout << hoap3angles.reb <<" "<< mICubArmRight.mEB << endl;
  //cout << hoap3angles.rwr <<" "<< mICubArmRight.mWR << endl;
  //cout << hoap3angles.rwp <<" "<< mICubArmRight.mWP << endl;
  
}

void  GlovesToICub(DataGloveHandSensors *ghand, iCubHand *hand){

  /*
  hand->mPalmAperture     = 1.0-0.33*(ghand->mIndexMiddle + ghand->mMiddleRing + ghand->mRingLittle);
  
  hand->mThumbOpposition  = ghand->mThumbIndex;

  hand->mThumb0     = ghand->mThumb0; 
  hand->mThumb1     = ghand->mThumb1; 
  hand->mIndex0     = ghand->mIndex0; 
  hand->mIndex1     = ghand->mIndex1; 
  hand->mMiddle0    = ghand->mMiddle0; 
  hand->mMiddle1    = ghand->mMiddle1; 
  hand->mRingLittle = 0.25*(ghand->mRing0 + ghand->mRing1 + ghand->mLittle0 + ghand->mLittle1);
  */
  Vector pose;
  if(hand == &mICubHandLeft)
    gloves.GetCalibratedPose(FD_HAND_LEFT,&pose);
  else
    gloves.GetCalibratedPose(FD_HAND_RIGHT,&pose);

  hand->mPalmAperture     = pose[0];  
  hand->mThumbOpposition  = pose[1];
  hand->mThumb0           = pose[2];
  hand->mThumb1           = pose[3];
  hand->mIndex0           = pose[4];
  hand->mIndex1           = pose[5];
  hand->mMiddle0          = pose[6];
  hand->mMiddle1          = pose[7];
  hand->mRingLittle       = pose[8];
  
}


void SendDataToYarp(){
  //defMotionId = 5;
  
  ArmJointsToICub();
  
  for(int i=0;i<2;i++){
    iCubHand              *hand;
    iCubArm               *arm;
    DataGloveHandSensors  *ghand;


    if(i==0){
      hand  = &mICubHandLeft;
      arm   = &mICubArmLeft;
      ghand = &mSensorBody.mHandLeft.mSensors;
    }else{
      hand  = &mICubHandRight;
      arm   = &mICubArmRight;
      ghand = &mSensorBody.mHandRight.mSensors;
    }

    GlovesToICub(ghand, hand);
  
  
    yarp::sig::Vector &vg= mGlovePorts[i].prepare();
    vg.resize(9);
    vg[0] =   hand->mPalmAperture;  
    vg[1] =   hand->mThumbOpposition;
    vg[2] =   hand->mThumb0    ; 
    vg[3] =   hand->mThumb1    ; 
    vg[4] =   hand->mIndex0    ; 
    vg[5] =   hand->mIndex1    ; 
    vg[6] =   hand->mMiddle0   ; 
    vg[7] =   hand->mMiddle1   ; 
    vg[8] =   hand->mRingLittle;
    //cout << v.toString()<<endl;
    mGlovePorts[i].write();
  

    yarp::sig::Vector &va= mArmPorts[i].prepare();
    va.resize(7);
    va[0] =   arm->mSFE;  
    va[1] =   arm->mSAA;  
    va[2] =   arm->mSHR;   
    va[3] =   arm->mEB;   
    va[4] =   arm->mWR;   
    va[5] =   arm->mWP;   
    va[6] =   arm->mWY;   
    mArmPorts[i].write();



  }
  
  
  
  
}

int main(int argc, char** argv)
{  
  
  
  if(!Init(argc,argv)){
    return 0;  
  }
  
  if(bUseGUI){
  
    glutInit(&argc, argv);
    glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);
    glutInitDisplayMode( GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    glutCreateWindow ("XSens2Hoap");
    
    glutDisplayFunc(Display); 
    glutReshapeFunc(OnReshape);
    glutKeyboardFunc(OnKeyPress);
    glutSpecialFunc(OnSpecialKeyPress);
    glutMouseFunc(OnMouseButton);
    glutMotionFunc(OnMouseMotion);
    glutIdleFunc(OnIdle);
  
    InitGL();
  
    glutMainLoop(); 
  }else{
    while(1){
      OnIdle();
    }
  }
  
  return(0);
}


