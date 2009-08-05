// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2008 Micha Hersch, EPFL
 * RobotCub Consortium, European Commission FP6 Project IST-004370
 * email:   micha.hersch@robotcub.org
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
#include "KinematicSimulator.h"

//#define COLLISION_TESTING

int cartesian_dim = 3;
int joint_angle_dim = 0;


KinematicSimulator::KinematicSimulator(){}


KinematicSimulator::KinematicSimulator(int argc, char **argv):
  GLUTBaseApplication(argc, argv){
  strcpy(tree_file,argv[1]);
  strcpy(shape_file,argv[2]);
  m_bProprioActive = false;
  proprio_arm.open("/simulator/arm:i");
  proprio_head.open("/simulator/head:i");
  targetIn.open("/simulator/target:i");
  targetOut.open("/simulator/target:o");
  cmdPort.open("/simulator/cmd:o");
  bodySchema.open("/simulator/body_schema:i");
  target_info=1;
  body=NULL;
  angles=NULL;
  arm = NULL;
  head = NULL;
  head_size=0;
  arm_size = 0;
  mapping = 0;

#ifdef TWO_BODIES
  body_data = NULL;
#endif

}

KinematicSimulator::~KinematicSimulator(){
    if(arm) delete arm;
    if(head) delete head;
    if(body)    delete body;
    if(angles) delete[] angles;
#ifdef TWO_BODIES
    if(arm) delete arm2;
    if(head) delete head2;
    if(body)    delete body2;
    if(body_data) delete[] body_data;
#endif
}



void  KinematicSimulator::InitOpenGL(){
  GetCamera()->Load("Camera.cfg");

  glEnable(GL_DEPTH_TEST);

  GLfloat LightAmbient[]= { 0.3f, 0.3f, 0.3f, 0.0f }; 	
  GLfloat LightDiffuse[]= { 0.4f, 0.8f, 0.3f, 1.0f };	
  GLfloat LightPosition[]= { -10.0f, 0.0f, 10.0f, 0.0f };	
  glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient);
  glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse);	
  glLightfv(GL_LIGHT1, GL_POSITION,LightPosition);
  glEnable(GL_LIGHT1);
  //glEnable(GL_LIGHTING);

  glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
  glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT);
  glEnable(GL_COLOR_MATERIAL);
  
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


  glEnable(GL_LINE_SMOOTH);
  glHint (GL_LINE_SMOOTH_HINT, GL_NICEST);
  glShadeModel (GL_SMOOTH);

  glClearColor(0.2,0.4,0.0,1.0);


  m_Camera[m_CurrentCamera]->Move(-200,-400,0,0,-20);
}


void KinematicSimulator::ClosePorts(){
  proprio_arm.close();
  proprio_head.close();
  targetIn.close();
  targetOut.close();
  cmdPort.close();
  bodySchema.close();
}

void KinematicSimulator::InputSpecialKey(int key, int x, int y){
    cart_vec_t dtar;
    switch(key){
    case GLUT_KEY_UP:
        dtar.GetArray()[2] = -20;
        break;
    case GLUT_KEY_DOWN:
        dtar.GetArray()[2] = 20;
        break;
    case GLUT_KEY_LEFT:
        dtar.GetArray()[0] = -20;
        break;
    case GLUT_KEY_RIGHT:
        dtar.GetArray()[0] = 20;
        break;
    case GLUT_KEY_PAGE_DOWN:
        dtar.GetArray()[1] = -20;
        break;
    case GLUT_KEY_PAGE_UP:
        dtar.GetArray()[1] = 20;
        break;
    }
    tarpos+=dtar;
    tarpos.Print();
}


void KinematicSimulator::InputNormalKey(unsigned char key,int x, int y){
    switch(key){
  case 27:
      cmdPort.SendQuit();
      ClosePorts();  
      Exit();
      break;
  case 'p':
      //      ListenToRobot();
      for(int i=0;i<body_size;i++){
          cout<<angles[i]*rad2deg<<" ";
      }
      cout<<endl;
      break;
  case 'r':
      body->GetArticulatedTree()->RandomAngle();
      cout<<"----- collisions ---------"<<endl;
      body->CheckAllCollisions(NULL);
      break;
  case 's':
      tarpos.Print();
      targetOut.SendPosition(tarpos.GetArray());
      break;
  default:
      cout<<"key not understood"<<endl;
  }
}
  
void KinematicSimulator::InitBody(){
  body = new EmbodiedKinematicTree(tree_file,shape_file);
  body_size = body->GetArticulatedTree()->GetSize();
  if(body->LoadChain("r_sfe","r_hand",0)){
      cout<<"found arm chain"<<endl;
      arm = body->GetChain(0);
      joint_angle_dim=arm_size = arm->GetNbJoints();// no last link
  }
  else{
      return;
  }
  if(body->LoadChain("neck_tilt","eyes",1)>0){
      cout<<"found head chain "<< endl;
      head = body->GetChain(1);
      head_size = head->GetNbJoints();
      cout<<"head size "<<head->GetNbJoints()<<endl;
  }

#ifdef TWO_BODIES
  //  display_trans.Zero();
  display_trans.GetArray()[0] = 800;
  body2 = new EmbodiedKinematicTree(tree_file,shape_file);
  body2->GetArticulatedTree()->GetJoint()->SetTranslation(display_trans.GetArray());
  if(body2->LoadChain("r_sfe","r_hand",0)){
      cout<<"found arm chain"<<endl;
      arm2 = body2->GetChain(0);
  }
  if(body2->LoadChain("neck_tilt","eyes",1)>0){
      cout<<"found head chain "<< endl;
      head2 = body2->GetChain(1);
  }
  body_data_size = body->GetTreeSize()*6;
  body_data = new float[body_data_size];
#endif



  cout<<"size "<<body_size<<endl; 
  angles = new float[body_size];
  arm_angles = angles;
  if(head){
       head_angles = angles+arm_size;
  }
}

void KinematicSimulator::Init(){
   InitOpenGL();
   InitBody();
   //   body->GetArticulatedTree()->PrintList();
}


void KinematicSimulator::RenderTarget(){
 
  switch(target_info){
  case 1:{ 
    glPushMatrix();
    glTranslatef(tarpos.GetArray()[0],tarpos.GetArray()[1],tarpos.GetArray()[2]);
    //glTranslatef(tarpos[0],tarpos[1],tarpos[2]);
   //  glTranslatef(-200,0,0);
    glColor3f(1.0,0,1.0);
    glutSolidSphere(40,15,15);
    glPopMatrix();
    break;
  }
  case 2:{
    Parallelipiped par;
    par.SetPosition(tarpos.GetArray());
    par.SetSize(5,-50,30);
    par.SetOrientation(tarpos.GetArray()+3);
    par.Render();
    break;
  }
  }
}

void KinematicSimulator::Render(){
 RenderTarget(); 
 body->UpdateShape();
 body->Render();
#ifdef TWO_BODIES
 tarpos += display_trans;
 RenderTarget();
 tarpos -= display_trans;
 body2->UpdateShape();
 body2->Render();
#endif

 glutPostRedisplay();
}


void KinematicSimulator::ListenToRobot(){
  proprio_arm.open();
  proprio_head.open();
}

void KinematicSimulator::Rearrange(float *a){
  float b[29];
  memcpy(b,a,29*sizeof(float));
  a[0] = 0; 
  a[1] = b[21]; 
  a[2] = b[22]; 
  a[3] = b[23]; 
  a[4] = b[6]; 
  a[5] = b[7]; 
  a[6] = b[8]; 
  a[7] = b[9]; 
  a[8] = 0; 
  a[9] = b[16]; 
  a[10] = b[17]; 
  a[11] = b[18]; 
  a[12] = b[19];
  a[13] = 0; 
  a[14] = b[20]; 
  a[15] = b[0]; 
  a[16] = b[1]; 
  a[17] = b[2]; 
  a[18] = b[3]; 
  a[19] = b[4]; 
  a[20] = b[5]; 
  a[21] = 0; 
  a[22] = b[10]; 
  a[23] = b[11]; 
  a[24] = b[12]; 
  a[25] = b[13]; 
  a[26] = b[14]; 
  a[27] = b[15];
  a[28] = 0;
}


void KinematicSimulator::OnIdle(){

  int ok = 0;
//   if(proprio.IsOpen()){
//     if(!m_bProprioActive){
//       //      targetIn.ConnectFrom("/target/out");
//       m_bProprioActive =(proprio.getInputCount>0)
// 	proprio.ConnectFrom("/proprio") && targetIn.ConnectFrom("/target/out") ;
//     }
//   }
#ifdef TWO_BODIES
  if(bodySchema.ReadBodySchema(body_data,body_data_size)){
      cout<<"got schema"<<endl;
      body2->GetArticulatedTree()->Deserialize(body_data,body_data_size);
      body2->GetArticulatedTree()->GetJoint()->SetTranslation(display_trans.GetArray());
  }

#endif
  m_bProprioActive = proprio_arm.getInputCount()>0 || proprio_head.getInputCount()>0 ;
  if(m_bProprioActive){
      //     cout<<"active"<<endl;
      ok = targetIn.ReadPositionAndOrientation(tarpos.GetArray(),tarpos.GetArray()+3);
      if(ok){
          target_info = ok>4?2:1;
      }
      if(ok)cout<<tarpos<<endl;
      if(mapping){
          ok = proprio_arm.ReadWithMapping(arm_angles,arm_size);
      }
      else{
          ok = proprio_arm.ReadPosition(arm_angles,arm_size);
      }
           //   cout<<angles[0]<<" "<<angles[1]<<" "<<angles[2]<<" "<<endl;
      //    Rearrange(angles);
      //     cout<<"ok "<<ok<<endl;
      if(ok){
          for(int i=0;i<arm_size;i++){
              arm_angles[i] *= deg2rad;
          }
          arm->SetAngles(arm_angles);
#ifdef TWO_BODIES
          arm2->SetAngles(arm_angles);
#endif
      }
      else{
          if(!proprio_arm.getInputCount()){
              m_bProprioActive=false;
          }
      }
      if(head){
          ok = proprio_head.ReadPosition(head_angles,head_size);
          if(ok){
              for(int i=0;i<head_size;i++){
                  //             cout<<" "<<head_angles[i];
                  head_angles[i] *= deg2rad;
              }
              //              cout<<endl;
              head->SetAngles(head_angles);
#ifdef TWO_BODIES
              head2->SetAngles(head_angles);
#endif

          }
      }
    }

      //Exit();      
}



#define SIMULATOR_APP_MAIN

#ifdef SIMULATOR_APP_MAIN

void usage(){
  cout<<"usage: simulator <struct file> <shape file> [--nomapping]"<<endl;
}

int main(int argc, char *argv[]){

  if(argc != 3 && argc != 4 ){
    cout<<argc<<endl;
    usage();
    return 0;
  }
  KinematicSimulator app(argc,argv);
  app.Init();
  app.Run();
  return 0;
}

#endif
