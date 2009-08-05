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
#ifndef __GL2DPLOT_H__
#define __GL2DPLOT_H__


#include <GL/glut.h>
#include <vector>
using namespace std;

#include "GLSubWindow.h"

typedef struct{
  float r;
  float g;
  float b;  
} RGBColor, *pRGBColor;


class GL2DPlot : public GLSubWindow
{
protected:
  int               m_NbPlots;
  vector<int>       m_PlotSize;
  vector<float*>    m_XValues;
  vector<float*>    m_YValues;
  vector<RGBColor>  m_Color;
  vector<int>       m_LineStyle;

  float             m_MinX, m_MaxX;
  float             m_MinY, m_MaxY;

  bool              m_AxesAuto;

  int               m_Offset;
public:
  GL2DPlot(pGLSubWindow parentWindow = NULL);
  ~GL2DPlot();

          void  Clear();
          void  AddPlot(float* YVal, int size);
          void  AddPlot(float* XVal, float* YVal, int size);
          void  SetColor(float r, float g, float b);
          void  SetColor(int no, float r, float g, float b);

          void  SetAxes(float minX, float minY, float maxX, float maxY);
          void  ClearAxes();
  virtual void  Render();

  virtual void  Resize(int w, int h);
  virtual void  OnNormalKey(char key);
  virtual void  OnSpecialKey(int key);

          void  SetOffset(int off);
};
typedef GL2DPlot *pGL2DPlot;

#endif
