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
#ifndef __GLCOLORPLOT_H__
#define __GLCOLORPLOT_H__


#include <GL/glut.h>
#include <vector>
using namespace std;

#include "GLSubWindow.h"
#include "GL2DPlot.h"

#define COLORMAP_SIZE   64

class GLColorPlot : public GLSubWindow
{
protected:
  float*            m_Values;
  int               m_SizeX;
  int               m_SizeY;

  RGBColor          m_Colormap[COLORMAP_SIZE];

  float             m_MinC, m_MaxC;

  bool              m_AxesAuto;

  float *           m_Buffer;

  int               m_Offset;
public:
  GLColorPlot(pGLSubWindow parentWindow = NULL);
  ~GLColorPlot();

          void  Clear();
          void  SetColorPlot(float* Val, int sizeX, int sizeY);
          void  SetColorMap(int no);
          void  SetAxes(float minC, float maxC);
          void  ClearAxes();
          void  SetOffset(int off);
  virtual void  Render();

  virtual void  Resize(int w, int h);
  virtual void  OnNormalKey(char key);
  virtual void  OnSpecialKey(int key);

};
typedef GLColorPlot *pGLColorPlot;

#endif
