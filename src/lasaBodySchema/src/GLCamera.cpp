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
#include <GL/glut.h>
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
  m_ref.Identity();    
  v_set(0.0,-150.0,-1500.0,m_ref.m_origin);
  m_hold    = false;
  m_Width   = 100;
  m_Height  = 100;
}

void GLCamera::Apply(){
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();  

  glViewport(0, 0, m_Width, m_Height);

  if(m_Height == 0)
    m_Height = 1;

  float ratio = 1.0f * m_Width / m_Height;
  
  gluPerspective(45,ratio,0.1,10000.0);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  m_ref.Update();
  glMultMatrixf(*m_ref.GetRef());
}


void GLCamera::Init(int w, int h){
  m_Width   = w;
  m_Height  = h;
}

void GLCamera::Move(float dx, float dy, float dz, float ay, float ax) {
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


  m_ref.Update();
  glLoadIdentity();
  glMultMatrixf(*m_ref.GetRef());
}

void GLCamera::Hold(){
  m_hold = true;
  m_holdRef.Copy(&m_ref);
}

void GLCamera::Accept(){
  m_hold = false;    
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
