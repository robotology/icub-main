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
#ifndef __GLCONSOLE_H__
#define __GLCONSOLE_H__


#include <GL/glut.h>


#include "GLSubWindow.h"
#include "Console.h"
#include "Timer.h"


class GLConsole : public Console, public GLSubWindow
{
protected:
  int           m_FontHeight;
  int           m_CursorStatus;
  Timer         m_CursorTimer;

  //int   m_Width;
  //int   m_Height;
public:
  GLConsole(pGLSubWindow parentWindow = NULL);
  ~GLConsole();

  virtual   void  Render();
//            void  Resize(int width, int height);
  virtual   int   AutoCompletion();

  virtual void  Resize(int w, int h);
  virtual void  OnNormalKey(char key);
  virtual void  OnSpecialKey(int key);

};
typedef GLConsole *pGLConsole;

#endif
