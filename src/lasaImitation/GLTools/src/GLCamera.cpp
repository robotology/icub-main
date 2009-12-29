#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
using namespace std;


#include "GLCamera.h"

GLCamera::GLCamera(){
  Clear();
  m_ref.SetOrigin(Vector3(0.0,0.0,-150.0));
  m_ref.Update();
}
void GLCamera::Clear(){
  m_position.Zero();
  m_orient.Identity();
  m_ref.Identity();    
  m_hold       = false;
  m_X          = 0;
  m_Y          = 0;
  m_Width      = 100;
  m_Height     = 100;
  m_Near       = 0.01f;
  m_Far        = 100.0f;
  m_ViewAngle  = 60.0f;
  m_ImWidth    = 100;
  m_ImHeight   = 100;
  m_PrincipalX = 0;
  m_PrincipalY = 0;
  m_FocalX     = 1;
  m_FocalY     = 1;
  m_Mode       = Centered;
}

void GLCamera::Apply(bool setIdentity){
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();  

  glViewport(m_X, m_Y, m_Width, m_Height);

  if(m_Height == 0)
    m_Height = 1;

  float ratio = float(m_Width) / float(m_Height);
  
  if(m_ViewAngle>0.0)
    gluPerspective(m_ViewAngle/ratio,ratio,m_Near,m_Far);
  else{
    glFrustum(-m_Near *  m_PrincipalX               / m_FocalX,
               m_Near * (m_ImWidth - m_PrincipalX)  / m_FocalX,
               m_Near * (m_PrincipalY - m_ImHeight) / m_FocalY,
               m_Near *  m_PrincipalY               / m_FocalY,
               m_Near,  m_Far);
  }

  glMatrixMode(GL_MODELVIEW);
  if(setIdentity)
    glLoadIdentity();
  
  
  m_ref.SetOrigin(m_position);
  m_ref.SetOrient(m_orient);
  m_ref.Update();
  glMultMatrixf(m_ref.GetInverse().RowOrderForceFloat());
}


void GLCamera::SetViewport(int w, int h){
  m_Width   = w;
  m_Height  = h;
}

void GLCamera::SetViewport(int x, int y, int w, int h){
  m_X       = x;
  m_Y       = y;
  m_Width   = w;
  m_Height  = h;
}

void GLCamera::SetProjection(float viewAngle, float mnear, float mfar){
  m_ViewAngle = viewAngle;
  m_Near      = mnear;
  m_Far       = mfar;
}

void GLCamera::SetProjection(float im_width,
                             float im_height,
                             float principal_x,
                             float principal_y,
                             float focal_x,
                             float focal_y,
                             float mnear, 
                             float mfar){
  m_PrincipalX = principal_x;
  m_PrincipalY = principal_y;
  m_FocalX     = focal_x;
  m_FocalY     = focal_y;
  m_Near       = mnear;
  m_Far        = mfar;
  m_ImWidth    = im_width;
  m_ImHeight   = im_height;

  m_ViewAngle  = 0;
}

void GLCamera::SetOrientation(const Matrix3 & orient){
  m_orient = orient;
}

void GLCamera::SetPosition(const Vector3 & pos){
  m_position = pos;
}

void GLCamera::Hold(){
  m_hold      = true;
  m_holdRef=m_ref;
}

void GLCamera::Accept(){
  m_hold = false;    
}

void GLCamera::Move(float dx, float dy, float dz){
  //if(m_hold)
  //  m_ref = m_holdRef;

  switch(m_Mode){
  case FreeMove: 
    printf("Free\n");   
    break;
  case Centered:
    float norm = m_position.Norm();
    Vector3 shift(dx,dy,0);
    shift *=norm /100.0f;    
    Vector3 absShift;
    m_orient.Mult(shift,absShift);
    m_position += absShift;
    norm += dz/norm;
    norm = MAX(1.0f,fabs(norm));
    m_position *= norm/m_position.Norm();    
    m_orient.SetColumn(m_position,2);
    m_orient.Normalize();
    break;
  }
}

/*
void GLCamera::Move(float dx, float dy, float dz, float ay, float ax,float az) {
  if(m_hold)
    m_ref.Copy(&m_holdRef);

  CVector3_t tmpSrc,tmpDst;
  CMatrix3_t tmpInv;
  m_inverse(m_ref.m_orient,tmpInv);

  v_set(dx,dy,dz,tmpSrc);    
  v_transform(tmpSrc,tmpInv,tmpDst);
  v_add(m_ref.m_origin,tmpSrc,m_ref.m_origin);

  CMatrix3_t tmp1,tmp2;

  m_rotation_x(-ax,tmp1);
  m_copy(m_ref.m_orient,tmp2);
  m_multiply(tmp1,tmp2,m_ref.m_orient);

  m_rotation_y(-ay,tmp1);
  m_copy(m_ref.m_orient,tmp2);
  m_multiply(tmp2,tmp1,m_ref.m_orient);

  m_rotation_z(-az,tmp1);
  m_copy(m_ref.m_orient,tmp2);
  m_multiply(tmp2,tmp1,m_ref.m_orient);

  m_ref.Update();
  glLoadIdentity();
  glMultMatrixf(*m_ref.GetRef());
}




void GLCamera::Save(string fname){
  ofstream f;
  f.open(fname.c_str());
  if(f.is_open()){
    f << m_ref.m_origin[0] << " "<< m_ref.m_origin[1] << " "<< m_ref.m_origin[2] << endl;
    f << m_ref.m_orient[ 0] << " "<< m_ref.m_orient[ 1] << " "<< m_ref.m_orient[ 2] << " "<<m_ref.m_orient[ 3] << endl;
    f << m_ref.m_orient[ 4] << " "<< m_ref.m_orient[ 5] << " "<< m_ref.m_orient[ 6] << " "<<m_ref.m_orient[ 7] << endl;
    f << m_ref.m_orient[ 8] << " "<< m_ref.m_orient[ 9] << " "<< m_ref.m_orient[10] << " "<<m_ref.m_orient[11] << endl;
    f << m_ref.m_orient[12] << " "<< m_ref.m_orient[13] << " "<< m_ref.m_orient[14] << " "<<m_ref.m_orient[15] << endl;
    f.close();
  }
}

void GLCamera::Load(string fname){
  ifstream f;
  f.open(fname.c_str());
  if(f.is_open()){
    m_ref.Identity();
    f >> m_ref.m_origin[0] >>  m_ref.m_origin[1] >>  m_ref.m_origin[2] ;
    f >> m_ref.m_orient[ 0] >>  m_ref.m_orient[ 1] >>  m_ref.m_orient[ 2] >> m_ref.m_orient[ 3] ;
    f >> m_ref.m_orient[ 4] >>  m_ref.m_orient[ 5] >>  m_ref.m_orient[ 6] >> m_ref.m_orient[ 7] ;
    f >> m_ref.m_orient[ 8] >>  m_ref.m_orient[ 9] >>  m_ref.m_orient[10] >> m_ref.m_orient[11] ;
    f >> m_ref.m_orient[12] >>  m_ref.m_orient[13] >>  m_ref.m_orient[14] >> m_ref.m_orient[15] ;
    f.close();
    m_ref.Update();
  }
}
*/
