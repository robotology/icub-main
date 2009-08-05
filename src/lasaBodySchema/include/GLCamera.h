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
#ifndef __GLCAMERA_H__
#define __GLCAMERA_H__

#include <GL/glut.h>
#include <vector>
#include <string>
using namespace std;

#include "Geometry.h"

class GLCamera;
typedef GLCamera *pGLCamera;
typedef vector<pGLCamera> GLCamera_List;

class GLCamera
{
public:
  CRef_t  m_ref;
  CRef_t  m_holdRef;

  bool    m_hold;

  int     m_Width;
  int     m_Height;

public:
  GLCamera();
  
  void Init(int w, int h);
  void Apply();

  void Hold();
  void Move(float dx, float dy, float dz, float ay, float ax);
  void Accept();

  void Save(string fname);
  
  void Load(string fname);
};

#endif
