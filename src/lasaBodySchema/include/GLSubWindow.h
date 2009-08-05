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
#ifndef __GLSUBWINDOW_H__
#define __GLSUBWINDOW_H__

#include <GL/glut.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
using namespace std;

#define GLSW_TITLE_HEIGHT   14


class Rect;
typedef Rect *pRect;
class Rect
{
public:
  int m_X,
      m_Y,
      m_Width,
      m_Height;
public:
  Rect();
  Rect(int x, int y, int w, int h);
  void Set(int x, int y, int w, int h);
  void Copy(pRect rect);
  bool IsInside(int px, int py);
};

class GLSubWindow;
typedef GLSubWindow *pGLSubWindow;
typedef vector<pGLSubWindow> GLSubWindow_List;

class GLSubWindow
{
public:
  pGLSubWindow      m_ParentWindow;
  GLSubWindow_List  m_ChildWindows;

  Rect              m_WindowRect;
  Rect              m_ClientRect;

  float             m_BackgroundColor[3];
  float             m_BackgroundAlpha;

  bool              m_Miminized;

  bool              m_HasFocus;

  string            m_Title;

  bool              m_Move;
  int               m_MoveStartX;
  int               m_MoveStartY;
  int               m_MouseButton;

  bool              m_Resize;
  int               m_ResizeStartX;
  int               m_ResizeStartY;


public:
          GLSubWindow(pGLSubWindow parentWindow = NULL);
	  virtual ~GLSubWindow();

          void  GetAbsoluteRect(pRect rect);

          void  SetSize(int w, int h);
          void  SetPos (int x, int y);
          void  Minimize(bool state);

          void  RenderWindow();

          void  SetTitle(string title);

  virtual void  Render();

  virtual void  Resize(int w, int h);

  virtual void  OnNormalKey(char key);
  virtual void  OnSpecialKey(int key);
  virtual int   OnClick(int button, int state, int x, int y);
  virtual int   OnDoubleClick(int button, int x, int y);
  virtual int   OnMove(int x, int y);

          void  OnWindowNormalKey(char key);
          void  OnWindowSpecialKey(int key);
          int   OnWindowClick(int button, int state, int x, int y);
          int   OnWindowDoubleClick(int button, int x, int y);
          int   OnWindowMove(int x, int y);

          int   IsParentMinimized();
};

#endif
