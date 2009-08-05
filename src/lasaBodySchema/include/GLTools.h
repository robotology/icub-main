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
#ifndef __GLTOOLS_H__
#define __GLTOOLS_H__

#include <GL/glut.h>

#include "Geometry.h"

class GLTools
{
protected:
  static float  m_red;
  static float  m_green;
  static float  m_blue;

  static bool   m_outline;
  static bool   m_solid;

public:
  static void SetColor(float r, float g, float b);

  static void DrawOutline (bool state);
  static void DrawSolid   (bool state);

  static void DrawArc   (float min, float max, CMatrix4_t *ref);  
  static void Draw3DArc (CVector3_List_t * vl, CMatrix4_t *ref);  

  static void DrawVector(CVector3_t *v, CMatrix4_t *ref,float width=100);
  static void DrawPlane (CVector3_t *v, CMatrix4_t *ref);
  static void DisplayText(int x, int y, const char * text);

};

#define GLT GLTools
#endif
