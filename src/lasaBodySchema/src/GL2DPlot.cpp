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


#include "GL2DPlot.h"
#include "GLTools.h"

GL2DPlot::GL2DPlot(pGLSubWindow parentWindow)
: GLSubWindow(parentWindow){
  Clear();
}

GL2DPlot::~GL2DPlot(){
  Clear();
}

void GL2DPlot::Clear(){
  m_AxesAuto = true;
  m_NbPlots = 0;
  m_PlotSize.clear();
  m_XValues.clear();
  m_YValues.clear();
  m_Color.clear();
  m_LineStyle.clear();
  m_MinX = -1;  m_MaxX =  1;
  m_MinY = -1;  m_MaxY =  1;
  m_Offset = 0;
}

void GL2DPlot::SetColor(float r, float g, float b){
  if(m_NbPlots>0){
    m_Color[m_NbPlots-1].r  = r;
    m_Color[m_NbPlots-1].g  = g;
    m_Color[m_NbPlots-1].b  = b;
  }
}

void GL2DPlot::SetColor(int no, float r, float g, float b){
  if((no<m_NbPlots)&&(no>=0)){
    m_Color[no].r  = r;
    m_Color[no].g  = g;
    m_Color[no].b  = b;
  }
}
void GL2DPlot::AddPlot(float* YVal, int size){
  AddPlot(NULL,YVal,size);
}
void GL2DPlot::AddPlot(float* XVal, float* YVal, int size){
  if((size<=0)||(YVal==NULL))
    return;

  m_PlotSize.push_back(size);
  m_XValues.push_back(XVal);
  m_YValues.push_back(YVal);

  RGBColor c;
  c.r = 1.0; c.g = 1.0; c.b = 1.0;
  m_Color.push_back(c);


  m_NbPlots++;
}
void GL2DPlot::SetOffset(int off){
  if((off>=0))
    m_Offset = off;
}
void GL2DPlot::Render(){

  int i,j;

  // Finding axes limits
  float minX,maxX,minY,maxY;

//  float *xval, *yval;

  if(m_AxesAuto){
    if(m_NbPlots>0){
      if(m_XValues[0]!=NULL){
        minX = m_XValues[0][0];
        maxX = m_XValues[0][0];
      }else{
        minX = 0;
        maxX = m_PlotSize[0]-1;
      }
      minY = m_YValues[0][0];
      maxY = m_YValues[0][0];
    }else{
      return;
    }
  
    for(i=0;i<m_NbPlots;i++){
      for(j=0;j<m_PlotSize[i];j++){
        minY = (m_YValues[i][j]<minY?m_YValues[i][j]:minY);
        maxY = (m_YValues[i][j]>maxY?m_YValues[i][j]:maxY);
      }
    }
    for(i=0;i<m_NbPlots;i++){
      if(m_XValues[i]!=NULL){
        for(j=0;j<m_PlotSize[i];j++){
          minX = (m_XValues[i][j]<minX?m_XValues[i][j]:minX);
          maxX = (m_XValues[i][j]>maxX?m_XValues[i][j]:maxX);
        }
      }else{
        minX = (0            <minX ? 0            :minX);
        maxX = (m_PlotSize[i]>maxX ? m_PlotSize[i]:maxX);
      }
    }

  }else{
    minX = m_MinX; maxX = m_MaxX;
    minY = m_MinY; maxY = m_MaxY;
  }

	glDisable(GL_DEPTH_TEST);

  glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
  glScalef(((float)m_ClientRect.m_Width)/2.0,((float)m_ClientRect.m_Height)/2.0,1.0);
  glTranslatef(1,1,0);
  glScalef(2.0/(maxX-minX),-2.0/(maxY-minY),1.0);
  glTranslatef(-(maxX+minX)/2.0,-(maxY+minY)/2.0,0.0);
  

  for(i=0;i<m_NbPlots;i++){
    if(m_XValues[i]!=NULL){
  	  glColor3fv((float*)&(m_Color[i]));
      glBegin(GL_LINE_STRIP);
      int j;
      const int pSize = m_PlotSize[i];
      float* xVal = m_XValues[i];
      float* yVal = m_YValues[i];
      for(j=0;j<pSize;j++){
        glVertex2f(*(xVal++),*(yVal++));
      }
      glEnd();
    }else{
  	  glColor3fv((float*)&(m_Color[i]));
      glBegin(GL_LINE_STRIP);
      int j;
      int off;
      if(m_Offset>m_PlotSize[i])
        off = 0;
      else
        off = m_Offset;

      const int pSize = m_PlotSize[i];      
      float* yVal = m_YValues[i]+off;
      for(j=0;j<pSize-off;j++){
        glVertex2f(j,*(yVal++));
      }
      yVal = m_YValues[i];
      for(j=pSize-off;j<pSize;j++){
        glVertex2f(j,*(yVal++));
      }
      glEnd();
    }
  }
  glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	glEnable(GL_DEPTH_TEST);

}


void  GL2DPlot::SetAxes(float minX, float minY, float maxX, float maxY){
  m_MinX = minX; m_MaxX = maxX;
  m_MinY = minY; m_MaxY = maxY;
  m_AxesAuto = false;  
}

void GL2DPlot::ClearAxes(){
  m_AxesAuto = true;
}

void  GL2DPlot::OnNormalKey(char key){  
}

void  GL2DPlot::OnSpecialKey(int key){
}

void  GL2DPlot::Resize(int w, int h){
}
