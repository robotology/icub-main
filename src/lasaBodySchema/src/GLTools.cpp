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
using namespace std;

#include "GLTools.h"


float  GLTools::m_red     = 1.0;
float  GLTools::m_green   = 1.0;
float  GLTools::m_blue    = 1.0;

bool   GLTools::m_outline = true;
bool   GLTools::m_solid   = true;

void GLTools::SetColor(float r, float g, float b){
  m_red = r;  m_green = g;   m_blue = b;
}

void GLTools::DrawOutline(bool state){
  m_outline = state;
}
void GLTools::DrawSolid(bool state){
  m_solid = state;
}

void GLTools::DrawArc(float min, float max, CMatrix4_t *ref){
  if((!m_outline) && (!m_solid))
    return;
  
  glPushMatrix();
  if(ref!=NULL)
    glMultMatrixf(*ref);

  float stepSize  = pi/32.0;
  int   step      = (int)ceil((max - min)*deg2rad / stepSize);
  int   i;
  float currAngle = min*deg2rad;

  if(m_solid){
    glColor3f(m_red, m_green, m_blue);
    glBegin(GL_TRIANGLE_FAN);
  		glVertex3f( 0.0f, 0.0f,  0.0f);
      for(i=0;i<step;i++){
        glVertex3f(cos(currAngle), sin(currAngle), 0.0f);
        currAngle += stepSize;
      }
      glVertex3f(cos(max*deg2rad),  sin(max*deg2rad), 0.0f);
	  glEnd();
  }

  if(m_outline){
    glScalef(1.001,1.001,1.001);
    currAngle = min*deg2rad;
    glColor3f(0.0,0.0,0.0);
    glBegin(GL_LINE_LOOP);
  		glVertex3f( 0.0f, 0.0f,  0.0f);
      for(i=0;i<step;i++){
        glVertex3f(cos(currAngle),  sin(currAngle), 0.0f);
        currAngle += stepSize;
      }
      glVertex3f(cos(max*deg2rad),  sin(max*deg2rad), 0.0f);
	  glEnd();  
  }

  glPopMatrix();
}


void GLTools::Draw3DArc(CVector3_List_t * vl, CMatrix4_t *ref){
  if((!m_outline) && (!m_solid))
    return;
  
  glPushMatrix();
  if(ref!=NULL)
    glMultMatrixf(*ref);


  int   step      = (int)(*vl).size();
  int   i;

  if(m_solid){
    glColor3f(m_red, m_green, m_blue);
    glBegin(GL_TRIANGLE_FAN);
  		glVertex3f( 0.0f, 0.0f,  0.0f);
      for(i=0;i<step;i++){ 
        glVertex3f(((*vl)[i]).m_Vector[0],((*vl)[i]).m_Vector[1],((*vl)[i]).m_Vector[2]);
      }
	  glEnd();
  }

  if(m_outline){
    glScalef(1.001,1.001,1.001);
    glColor3f(0.0,0.0,0.0);
    glBegin(GL_LINE_STRIP);
  		//glVertex3f( 0.0f, 0.0f,  0.0f);
      for(i=0;i<step;i++){
        glVertex3f(((*vl)[i]).m_Vector[0],((*vl)[i]).m_Vector[1],((*vl)[i]).m_Vector[2]);
      }
	  glEnd();  
  }

  glPopMatrix();
}

void GLTools::DrawVector(CVector3_t *v, CMatrix4_t *ref,float width){
  if((!m_outline) && (!m_solid))
    return;

  CVector3_t vec;
  v_normalize((*v),vec);
  float norm = sqrt(v_dot((*v),(*v)));

  float orie = atan2(vec[0],vec[2]);
  float elev = asin(vec[1]);

  glPushMatrix();
  if(ref!=NULL)
    glMultMatrixf(*ref);
 	
  glRotatef(orie*rad2deg,0.0,1.0,0.0);
  glRotatef(-elev*rad2deg,1.0,0.0,0.0);

  glScalef(width,width,norm);

  glPushMatrix();
  glTranslatef(0.0,0.0,0.5);//0.5 0.25
  glScalef(0.1,0.1,1.0);
  if(m_solid){
    glColor3f(m_red, m_green, m_blue);
    glutSolidCube(1.0);//1.0 0.5
  }
  if(m_outline){
    glColor3f(0.0,0.0,0.0);
    glScalef(1.001,1.001,1.001);
    glutWireCube(1.0);//1.0 0.5
  }
  glPopMatrix();
  //cone
#ifdef WITH_CONE
  glTranslatef(0.0,0.0,0.5);
  glScalef(0.1,0.1,1.0);
  if(m_solid){
    glColor3f(m_red, m_green, m_blue);
    glutSolidCone(1.0,0.5,4,1);
  }
  if(m_outline){
    glColor3f(0.0,0.0,0.0);
    glScalef(1.001,1.001,1.001);
    glutWireCone(1.0,0.5,4,1);
  }  
#endif
  glPopMatrix();
}

void GLTools::DrawPlane(CVector3_t *v, CMatrix4_t *ref){
  if((!m_outline) && (!m_solid))
    return;
  
  CVector3_t vec;
  v_normalize((*v),vec);
  float norm = sqrt(v_dot((*v),(*v)));

  float orie = atan2(vec[0],vec[2]);
  float elev = asin(vec[1]);

  glPushMatrix();
  if(ref!=NULL)
    glMultMatrixf(*ref);
 	
  glRotatef(orie*rad2deg,0.0,1.0,0.0);
  glRotatef(-elev*rad2deg,1.0,0.0,0.0);


  glScalef(norm,norm,1.0);
  if(m_solid){
    glColor3f(m_red, m_green, m_blue);
    glBegin(GL_QUADS);
		  glVertex3f(-1.0f,-1.0f,  0.0f);
		  glVertex3f(-1.0f, 1.0f,  0.0f);
		  glVertex3f( 1.0f, 1.0f,  0.0f);
		  glVertex3f( 1.0f,-1.0f,  0.0f);
	  glEnd();
  }
  if(m_outline){
	  glScalef(1.001,1.001,1.0);
	  glColor3f(0.0f, 0.0f, 0.0f);
	  glBegin(GL_LINE_STRIP);
  		glVertex3f(-1.0f,-1.0f,  0.0f);
		  glVertex3f(-1.0f, 1.0f,  0.0f);
		  glVertex3f( 1.0f, 1.0f,  0.0f);
		  glVertex3f( 1.0f,-1.0f,  0.0f);
		  glVertex3f(-1.0f,-1.0f,  0.0f);
	  glEnd();
  }
  glPopMatrix();
}

void GLTools::DisplayText(int x, int y, const char * text){
  int index = 0;
  int pos   = x;
  glColor3f(1.0f,1.0f,1.0f);
  while(text[index]!=0){
    glRasterPos2f(pos,y);
    glutBitmapCharacter   (GLUT_BITMAP_8_BY_13,text[index]);
    pos += 8;//glutBitmapWidth(GLUT_BITMAP_8_BY_13,text[index]);
    index++;
  }
}
