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
#pragma warning( disable : 4786)

#include <GL/glut.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <string>
using namespace std;


#include "GLColorPlot.h"
#include "GLTools.h"

GLColorPlot::GLColorPlot(pGLSubWindow parentWindow)
: GLSubWindow(parentWindow){
  m_Buffer    = NULL;
  Clear();
}

GLColorPlot::~GLColorPlot(){
  Clear();
}

void GLColorPlot::Clear(){
  m_Values    = NULL;
  m_SizeX     = 0;
  m_SizeY     = 0;
  m_Offset    = 0;
  SetColorMap(1);

  m_AxesAuto  = true;

  m_MinC      = -1;  
  m_MaxC      =  1;


  if(m_Buffer!=NULL)
    delete [] m_Buffer;
  m_Buffer    = NULL;
}

void GLColorPlot::SetColorMap(int no){
  int i;
  switch(no){
  case 1:
    for(i=0*COLORMAP_SIZE/8;i<1*COLORMAP_SIZE/8;i++){
      m_Colormap[i].r = 0.0;
      m_Colormap[i].g = 0.0;
      m_Colormap[i].b =       ((float)(i + 1*COLORMAP_SIZE/8))/((float)COLORMAP_SIZE/4);
    }
    for(i=1*COLORMAP_SIZE/8;i<3*COLORMAP_SIZE/8;i++){
      m_Colormap[i].r = 0.0;
      m_Colormap[i].g =       ((float)(i - 1*COLORMAP_SIZE/8))/((float)COLORMAP_SIZE/4);
      m_Colormap[i].b = 1.0;
    }
    for(i=3*COLORMAP_SIZE/8;i<5*COLORMAP_SIZE/8;i++){
      m_Colormap[i].r =       ((float)(i - 3*COLORMAP_SIZE/8))/((float)COLORMAP_SIZE/4);
      m_Colormap[i].g = 1.0;
      m_Colormap[i].b = 1.0 - ((float)(i - 3*COLORMAP_SIZE/8))/((float)COLORMAP_SIZE/4);
    }
    for(i=5*COLORMAP_SIZE/8;i<7*COLORMAP_SIZE/8;i++){
      m_Colormap[i].r = 1.0;
      m_Colormap[i].g = 1.0 - ((float)(i - 5*COLORMAP_SIZE/8))/((float)COLORMAP_SIZE/4);
      m_Colormap[i].b = 0.0;
    }
    for(i=7*COLORMAP_SIZE/8;i<8*COLORMAP_SIZE/8;i++){
      m_Colormap[i].r = 1.0 - ((float)(i - 7*COLORMAP_SIZE/8))/((float)COLORMAP_SIZE/4);
      m_Colormap[i].g = 0.0;
      m_Colormap[i].b = 0.0;
    }

    break;

  case 0:
  default:
    for(i=0;i<COLORMAP_SIZE;i++){
      m_Colormap[i].r = ((float)i)/((float)COLORMAP_SIZE-1);
      m_Colormap[i].g = m_Colormap[i].r;
      m_Colormap[i].b = m_Colormap[i].r;
    }
    break;
  }
}

void GLColorPlot::SetColorPlot(float* Val, int sizeX, int sizeY){
  if((sizeX<=0)||(sizeY<=0))
    return;

  if(Val==NULL)
    return;

  if(m_Buffer!=NULL)
    delete [] m_Buffer;

  m_Values  = Val;
  m_SizeX   = sizeX;
  m_SizeY   = sizeY;
  m_Buffer  = new float[sizeX*sizeY*4];
}

void GLColorPlot::Render(){

  int i;

  // Finding axes limits;
  float minC,maxC;

  if(m_AxesAuto){
    if(m_Values==NULL)
      return;

    minC = m_Values[0];
    maxC = m_Values[0];
  
    float *val = m_Values;
    const int valSize = m_SizeX*m_SizeY;
    for(i=0;i<valSize;i++){
      if((*val)<minC) minC = (*val);
      if((*val)>maxC) maxC = (*val);
      val++;
    }
  }else{
    minC = m_MinC; maxC = m_MaxC;
  }
  //cout <<m_Title<<": "<< minC << " " << maxC<<endl;
  float *val = m_Values;
  float *buf = m_Buffer;
  const int valSize = m_SizeX*m_SizeY;
  float minMax = maxC-minC;
  float nVal;
  int   nIndex;
  const int offSize = m_Offset*m_SizeX;
  val += offSize;
  for(i=0;i<valSize-offSize;i++){
    nVal = ((*(val++)) - minC) / (minMax);
    if(nVal>1.0) nVal = 1.0;
    else if(nVal<0.0) nVal = 0.0;
    
    nIndex = (int)(floor(nVal * (COLORMAP_SIZE-1)));
     *(buf++) = m_Colormap[nIndex].r;
     *(buf++) = m_Colormap[nIndex].g;
     *(buf++) = m_Colormap[nIndex].b;
     *(buf++) = 0.6;
  }
  val = m_Values;
  for(i=0;i<offSize;i++){
    nVal = ((*(val++)) - minC) / (minMax);
    if(nVal>1.0) nVal = 1.0;
    else if(nVal<0.0) nVal = 0.0;
    
    nIndex = (int)(floor(nVal * (COLORMAP_SIZE-1)));
     *(buf++) = m_Colormap[nIndex].r;
     *(buf++) = m_Colormap[nIndex].g;
     *(buf++) = m_Colormap[nIndex].b;
     *(buf++) = 0.6;
  }

	glDisable(GL_DEPTH_TEST);

  glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
  glRasterPos2i(2,m_ClientRect.m_Height-2);
  glPixelZoom(((float)m_ClientRect.m_Width-4)/m_SizeX,((float)m_ClientRect.m_Height-2)/m_SizeY);
  glDrawPixels(m_SizeX,m_SizeY,GL_RGBA,GL_FLOAT,m_Buffer);
  
  glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	glEnable(GL_DEPTH_TEST);

}
void GLColorPlot::SetOffset(int off){
  if((off <= m_SizeY)&(off>=0))
    m_Offset = off;
}


void  GLColorPlot::SetAxes(float minC, float maxC){
  m_MinC = minC; m_MaxC = maxC;
  m_AxesAuto = false;  
}

void GLColorPlot::ClearAxes(){
  m_AxesAuto = true;
}

void  GLColorPlot::OnNormalKey(char key){  
}

void  GLColorPlot::OnSpecialKey(int key){
}

void  GLColorPlot::Resize(int w, int h){
}
