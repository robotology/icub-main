#ifdef WIN32
#pragma warning( disable : 4786)
#endif


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
  m_LineWidth.clear();
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

void  GL2DPlot::SetDataArray(int no, float *YVal, int size){
  if((no<m_NbPlots)&&(no>=0)){
    m_YValues[no] = YVal;
    m_PlotSize[no] = size;
  }  
}
void  GL2DPlot::SetLineWidth(int no, float w){
  if((no<m_NbPlots)&&(no>=0)){
    m_LineWidth[no] = w;
  }
}
void  GL2DPlot::Add2DSurfPlot(float* YVal0, float* YVal1, int size){
  Add2DSurfPlot(NULL,YVal0,YVal1,size);
}
void  GL2DPlot::Add2DSurfPlot(float* XVal, float* YVal0, float* YVal1, int size){
  if((size<=0)||(YVal0==NULL)||(YVal1==NULL))
    return;
    
  m_PlotSize.push_back(size);  m_PlotSize.push_back(size);
  m_XValues.push_back(XVal);   m_XValues.push_back(XVal);
  m_YValues.push_back(YVal0);  m_YValues.push_back(YVal1);
  m_PlotType.push_back(1);     m_PlotType.push_back(1);
  m_LineWidth.push_back(1.0f); m_LineWidth.push_back(1.0f);
  RGBColor c;  c.r = 1.0; c.g = 1.0; c.b = 1.0;
  m_Color.push_back(c);  m_Color.push_back(c);
  m_NbPlots+=2;  
}

void  GL2DPlot::AddVarSurfPlot(float* YVal0, float* YVal1, int size){
  AddVarSurfPlot(NULL,YVal0,YVal1,size);
}
void  GL2DPlot::AddVarSurfPlot(float* XVal, float* YVal0, float* YVal1, int size){
  if((size<=0)||(YVal0==NULL)||(YVal1==NULL))
    return;
    
  m_PlotSize.push_back(size);  m_PlotSize.push_back(size);
  m_XValues.push_back(XVal);   m_XValues.push_back(XVal);
  m_YValues.push_back(YVal0);  m_YValues.push_back(YVal1);
  m_PlotType.push_back(2);     m_PlotType.push_back(2);
  m_LineWidth.push_back(1.0f); m_LineWidth.push_back(1.0f);
  RGBColor c;  c.r = 1.0; c.g = 1.0; c.b = 1.0;
  m_Color.push_back(c);  m_Color.push_back(c);
  m_NbPlots+=2;  
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
  m_PlotType.push_back(0);
  m_LineWidth.push_back(1.0f);
  RGBColor c;
  c.r = 1.0; c.g = 1.0; c.b = 1.0;
  m_Color.push_back(c);


  m_NbPlots++;
}
void GL2DPlot::SetOffset(int off){
  if((off>=0))
    m_Offset = off;
}

void GL2DPlot::GetDataBoundaries(float *rminX,float *rminY,float*rmaxX,float*rmaxY){
  float minX,maxX,minY,maxY;

  int i,j;

  if(m_NbPlots>0){
    if(m_XValues[0]!=NULL){
      minX = m_XValues[0][0];
      maxX = m_XValues[0][0];
    }else{
      minX = 0;
      maxX = m_PlotSize[0]-1;
    }
    if(m_YValues.size()>0){
      if(m_YValues[0]!=NULL){
        minY = m_YValues[0][0];
        maxY = m_YValues[0][0];
      }
    }else{
      minY = 0;
      maxY = 0;      
    }
  }else{
    return;
  }
  minY = 0;
  maxY = 0;

  if(m_YValues.size()>0){
    for(i=0;i<m_NbPlots;i++){
      if(m_YValues[i]!=NULL){
        for(j=0;j<m_PlotSize[i];j++){
          if(m_PlotType[i]!=2){
          minY = (m_YValues[i][j]<minY?m_YValues[i][j]:minY);
          maxY = (m_YValues[i][j]>maxY?m_YValues[i][j]:maxY);
          }
        }
      }
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
  
  *rminX = minX;
  *rminY = minY;
  *rmaxX = maxX;
  *rmaxY = maxY;
  
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
    minY = 0;
    maxY = 0;
  
    for(i=0;i<m_NbPlots;i++){
      for(j=0;j<m_PlotSize[i];j++){
        if(m_PlotType[i]!=2){
        minY = (m_YValues[i][j]<minY?m_YValues[i][j]:minY);
        maxY = (m_YValues[i][j]>maxY?m_YValues[i][j]:maxY);
        }
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
  m_MinX= minX; m_MaxX=maxX;
  m_MinY= minY; m_MaxY=maxY;

	glDisable(GL_DEPTH_TEST);

  glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
  glScalef(((float)m_ClientRect.m_Width)/2.0,((float)m_ClientRect.m_Height)/2.0,1.0);
  glTranslatef(1,1,0);
  glScalef(2.0/(maxX-minX),-2.0/(maxY-minY),1.0);
  glTranslatef(-(maxX+minX)/2.0,-(maxY+minY)/2.0,0.0);
  
  if(m_NbPlots>0){
    glColor3f(1.0f,1.0f,1.0f);
    if(m_XValues[0]!=NULL){
      glBegin(GL_LINE_STRIP);
      glVertex2f(*(m_XValues[0]),0.0f);
      glVertex2f(*(m_XValues[0]+m_PlotSize[0]-1),0.0f);
      glEnd();
    }else{
      glBegin(GL_LINE_STRIP);
      glVertex2f(0.0f,0.0f);
      glVertex2f(m_PlotSize[0]-1.0f,0.0f);
      glEnd();
    }
  }
  for(i=0;i<m_NbPlots;i++){
    if(m_XValues[i]!=NULL){
  	  glColor3fv((float*)&(m_Color[i]));
      if(m_PlotType[i]==0){
        if(m_LineWidth[i]!=1.0f) glLineWidth(m_LineWidth[i]);
        glBegin(GL_LINE_STRIP);
        int j;
        const int pSize = m_PlotSize[i];
        float* xVal = m_XValues[i];
        float* yVal = m_YValues[i];
        for(j=0;j<pSize;j++){
          glVertex2f(*(xVal++),*(yVal++));
        }
        glEnd();
        if(m_LineWidth[i]!=1.0f) glLineWidth(1.0f);
      }else if(m_PlotType[i]==1){
        glBegin(GL_TRIANGLE_STRIP);
        int j;
        const int pSize = m_PlotSize[i];
        float* xVal = m_XValues[i];
        float* yVal0 = m_YValues[i];
        float* yVal1 = m_YValues[i+1];
        for(j=0;j<pSize;j++){
          glVertex2f(*(xVal),  *(yVal0++));
          glVertex2f(*(xVal++),*(yVal1++));
        }
        glEnd();
        glColor3f(m_Color[i].r*0.8f,m_Color[i].g*0.8f,m_Color[i].b*0.8f);
        glBegin(GL_LINE_STRIP);
        yVal0 = m_YValues[i]; for(j=0;j<pSize;j++)     glVertex2f(j,*(yVal0++));
        glEnd();
        glBegin(GL_LINE_STRIP);
        yVal0 = m_YValues[i+1]; for(j=0;j<pSize;j++)     glVertex2f(j,*(yVal0++));
        glEnd();
        i++;
      }
    }else{
  	  glColor3fv((float*)&(m_Color[i]));
      if(m_PlotType[i]==0){
        if(m_LineWidth[i]!=1.0f) glLineWidth(m_LineWidth[i]);
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
        if(m_LineWidth[i]!=1.0f) glLineWidth(1.0f);
      }else if(m_PlotType[i]==1){
        int j, off;
        if(m_Offset>m_PlotSize[i]) off = 0;
        else off = m_Offset;

        const int pSize = m_PlotSize[i];
        glBegin(GL_TRIANGLE_STRIP);
        float* yVal0 = m_YValues[i]+off;
        float* yVal1 = m_YValues[i+1]+off;
        for(j=0;j<pSize-off;j++){
          glVertex2f(j,*(yVal0++));
          glVertex2f(j,*(yVal1++));
        }
        yVal0 = m_YValues[i];
        yVal1 = m_YValues[i+1];
        for(j=pSize-off;j<pSize;j++){
          glVertex2f(j,*(yVal0++));
          glVertex2f(j,*(yVal1++));
        }
        glEnd();
        glColor3f(m_Color[i].r*0.8f,m_Color[i].g*0.8f,m_Color[i].b*0.8f);
        glBegin(GL_LINE_STRIP);
        glVertex2f(0,*(m_YValues[i+1]+off));
        yVal0 = m_YValues[i]+off; for(j=0;j<pSize-off;j++)     glVertex2f(j,*(yVal0++));
        yVal0 = m_YValues[i];     for(j=pSize-off;j<pSize;j++) glVertex2f(j,*(yVal0++));
        glEnd();
        glBegin(GL_LINE_STRIP);
        yVal0 = m_YValues[i+1]+off; for(j=0;j<pSize-off;j++)     glVertex2f(j,*(yVal0++));
        yVal0 = m_YValues[i+1];     for(j=pSize-off;j<pSize;j++) glVertex2f(j,*(yVal0++));
        glVertex2f(pSize-1,*(m_YValues[i]+pSize-off-1));
        glEnd();
        
        i++;
      }else if(m_PlotType[i]==2){
        int j, off;
        if(m_Offset>m_PlotSize[i]) off = 0;
        else off = m_Offset;

        const int pSize = m_PlotSize[i];
        glBegin(GL_TRIANGLE_STRIP);
        float* yVal0 = m_YValues[i]+off;
        float* yVal1 = m_YValues[i+1]+off;
        for(j=0;j<pSize-off;j++){
          glVertex2f(j,*(yVal0)+*(yVal1));
          glVertex2f(j,*(yVal0++)-*(yVal1++));
        }
        yVal0 = m_YValues[i];
        yVal1 = m_YValues[i+1];
        for(j=pSize-off;j<pSize;j++){
          glVertex2f(j,*(yVal0)+*(yVal1));
          glVertex2f(j,*(yVal0++)-*(yVal1++));
        }
        glEnd();
        
        glColor3f(m_Color[i].r*0.8f,m_Color[i].g*0.8f,m_Color[i].b*0.8f);
        glBegin(GL_LINE_STRIP);
        glVertex2f(0,*(m_YValues[i]+off)-*(m_YValues[i+1]+off));
        yVal0 = m_YValues[i]+off; yVal1 = m_YValues[i+1]+off; for(j=0;j<pSize-off;j++)     glVertex2f(j,*(yVal0++)+*(yVal1++));
        yVal0 = m_YValues[i];     yVal1 = m_YValues[i+1];     for(j=pSize-off;j<pSize;j++) glVertex2f(j,*(yVal0++)+*(yVal1++));
        glEnd();
        glBegin(GL_LINE_STRIP);
        yVal0 = m_YValues[i]+off; yVal1 = m_YValues[i+1]+off; for(j=0;j<pSize-off;j++)     glVertex2f(j,*(yVal0++)-*(yVal1++));
        yVal0 = m_YValues[i];     yVal1 = m_YValues[i+1];     for(j=pSize-off;j<pSize;j++) glVertex2f(j,*(yVal0++)-*(yVal1++));
        glVertex2f(pSize-1,*(m_YValues[i]+pSize-off-1)+*(m_YValues[i+1]+pSize-off-1));
        glVertex2f(pSize-1,*(m_YValues[i]+pSize-off-1));
        glEnd();
        
        i++;
      }
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
void  GL2DPlot::GetAxes(float *minX, float *minY, float *maxX, float *maxY){
  *minX=m_MinX; *maxX=m_MaxX;
  *minY=m_MinY; *maxY=m_MaxY;    
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
