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
#ifndef __GLUTAPPLICATION_H__
#define __GLUTAPPLICATION_H__

#include <GL/glut.h>

#include "GLConsole.h"
#include "GLCamera.h"
#include "GLSnapshot.h"
#include "GL2DPlot.h"
#include "GLColorPlot.h"
#include "GLSubWindow.h"
#include "Timer.h"

class GLUTBaseApplication;
typedef GLUTBaseApplication *pGLUTBaseApplication;

class GLUTBaseApplication
{
protected:
  // Window ID
  int               m_WindowID;

  // Cameras
  GLCamera_List     m_Camera;
  int               m_CurrentCamera;

  // Mouse tracking
  int               m_CurrentMouseButton;
  int               m_CurrentMouseX;
  int               m_CurrentMouseY;


  // Console
  //GLConsole         m_Console;

  
  // Double click Timer
  Timer             m_DblClickTimer;
  int               m_LastPressedButton;

 protected:
  // Snapshots
  GLSnapshot        m_Snapshot;
  bool              m_Snap;

public:
  GLUTBaseApplication();
  GLUTBaseApplication(int argc, char **argv);
//  GLUTBaseApplication(int argc, char **argv, pTree config);
  virtual ~GLUTBaseApplication();

  virtual void  Init();
  virtual void  Render();
  virtual void  Free();
  virtual void  OnIdle();
  virtual void  Resize          (int width, int height);
  virtual void  InputMouseButton(int button, int state, int x, int y);
  virtual void  InputMouseMotion(int x, int y);
  virtual void  InputNormalKey  (unsigned char key, int x, int y);
  virtual void  InputSpecialKey (int key, int x, int y);

          void  SetCurrent();

          void  Run();
          void  Exit();
	  void  StartSnapshot(const char *filename,GLSnapshotMode mode);

          void  NextCamera();
          void  SetCamera(int num);
          GLCamera *GetCamera();
private:
          void  BaseFree();
          void  BaseRender();
          void  BaseResize          (int width, int height);
          void  BaseInputNormalKey  (unsigned char key, int x, int y);
          void  BaseInputSpecialKey (int key, int x, int y);
          void  BaseInputMouseButton(int button, int state, int x, int y);
          void  BaseInputMouseMotion(int x, int y);

protected:
    GLSubWindow_List  m_SubWindows;


private:
  static  void  GLUTApp_Render();
  static  void  GLUTApp_OnIdle();
  static  void  GLUTApp_Resize(int width, int height);
  static  void  GLUTApp_InputMouseButton(int button, int state, int x, int y);
  static  void  GLUTApp_InputMouseMotion(int x, int y);
  static  void  GLUTApp_InputNormalKey(unsigned char key, int x, int y);
  static  void  GLUTApp_InputSpecialKey(int key, int x, int y);

  static  pGLUTBaseApplication    m_GLUTApp;
};





/*
class GLUTApplication : public GLUTBaseApplication
{
public:
  GLUTApplication(int argc, char **argv);
//  GLUTApplication(int argc, char **argv, pTree config);
  ~GLUTApplication();

  virtual void  Init();
  virtual void  Render();
  virtual void  Free();
  virtual void  OnIdle();
  virtual void  Resize          (int width, int height);
  virtual void  InputMouseButton(int button, int state, int x, int y);
  virtual void  InputMouseMotion(int x, int y);
  virtual void  InputNormalKey  (unsigned char key, int x, int y);
  virtual void  InputSpecialKey (int key, int x, int y);
};
*/
#endif
