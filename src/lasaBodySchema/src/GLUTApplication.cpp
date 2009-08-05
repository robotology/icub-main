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
#include "GLUTApplication.h"


GLUTBaseApplication::GLUTBaseApplication(){}

GLUTBaseApplication::GLUTBaseApplication(int argc, char **argv){
  glutInit(&argc, argv);
	glutInitWindowPosition(0,0);
	glutInitWindowSize(600,500);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	m_WindowID = glutCreateWindow("Simulator");

  SetCurrent();

  glutDisplayFunc(GLUTBaseApplication::GLUTApp_Render);
	glutReshapeFunc(GLUTBaseApplication::GLUTApp_Resize);
	glutIdleFunc(GLUTBaseApplication::GLUTApp_OnIdle);
	glutIgnoreKeyRepeat(1);
	glutKeyboardFunc(GLUTBaseApplication::GLUTApp_InputNormalKey);
	glutSpecialFunc(GLUTBaseApplication::GLUTApp_InputSpecialKey);
	glutMouseFunc(GLUTBaseApplication::GLUTApp_InputMouseButton);
	glutMotionFunc(GLUTBaseApplication::GLUTApp_InputMouseMotion);
	//	glutFullScreen();

  m_Snap                = false;
  m_CurrentCamera       = 0;
  m_CurrentMouseButton  = -1;
  m_CurrentMouseX       = 0;
  m_CurrentMouseY       = 0;

  m_Camera.push_back(new GLCamera());
  


  m_LastPressedButton = -1;
}

//GLUTBaseApplication::GLUTBaseApplication(int argc, char **argv, pTree config){}
GLUTBaseApplication::~GLUTBaseApplication(){
  BaseFree();
}

void  GLUTBaseApplication::BaseFree(){
  unsigned int i;
  
  // Freeing Cameras
  for(i=0;i<m_Camera.size();i++)
    delete m_Camera[i];
  m_Camera.clear();

  for(i=0;i<m_SubWindows.size();i++){
    delete m_SubWindows[i];
  }
  m_SubWindows.clear();

  Free();
}

void  GLUTBaseApplication::BaseRender(){
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  if(m_Snap) m_Snapshot.Begin();
  

  m_Camera[m_CurrentCamera]->Apply();
  Render();

  //m_Console.Render();

  int i;
  for(i=0;i<m_SubWindows.size();i++)
    m_SubWindows[i]->RenderWindow();

  if(m_Snap) {
    m_Snapshot.Finish();
    m_Snap = false;
  }

  glFlush();
	glutSwapBuffers();
}

void  GLUTBaseApplication::BaseResize(int width, int height){
  int i;
  
  // Updating Camera(s)
  for(i=0;i<m_Camera.size();i++)
    m_Camera[i]->Init(width,height);
  m_Camera[m_CurrentCamera]->Apply();

  // Updating Snapshot
  m_Snapshot.SetViewport(width,height);

  // Updating Console
  //m_Console.Resize(width,height);

  Resize(width,height);
}

void  GLUTBaseApplication::BaseInputNormalKey(unsigned char key, int x, int y){
  int i;
  for(i=0;i<m_SubWindows.size();i++)
    m_SubWindows[i]->OnWindowNormalKey(key);


  InputNormalKey(key,x,y);
}

void  GLUTBaseApplication::BaseInputSpecialKey(int key, int x, int y){
  int i;
  for(i=0;i<m_SubWindows.size();i++)
    m_SubWindows[i]->OnWindowSpecialKey(key);
  
  InputSpecialKey(key,x,y);
}
void  GLUTBaseApplication::BaseInputMouseButton(int button, int state, int x, int y){
  int i;  
  if(state == GLUT_DOWN){
    if(m_DblClickTimer.IsDone())
      m_LastPressedButton = -1;
    if(m_LastPressedButton!=button){
      m_LastPressedButton = button;
      m_DblClickTimer.Start(500);
    }else{
      if(!m_DblClickTimer.IsDone()){
        for(i=0;i<m_SubWindows.size();i++)
          m_SubWindows[i]->OnWindowDoubleClick(button,x,y);

        m_LastPressedButton = -1;
      }
    }
  }

  int cFlag = 0;
  for(i=0;i<m_SubWindows.size();i++)
    cFlag |= m_SubWindows[i]->OnWindowClick(button,state,x,y);  

  if(cFlag)
    return;
  
  // Camera motion
  if((state ==  GLUT_UP) && (m_CurrentMouseButton == button)){
    m_CurrentMouseButton = -1;
    m_Camera[m_CurrentCamera]->Accept();
  }
  else if((state == GLUT_DOWN)&&(m_CurrentMouseButton == -1)){
    m_Camera[m_CurrentCamera]->Hold();
    m_CurrentMouseButton = button;
    m_CurrentMouseX      = x;
    m_CurrentMouseY      = y;
  }

  InputMouseButton(button,state,x,y);
}

void  GLUTBaseApplication::BaseInputMouseMotion(int x, int y){
  int i;
  for(i=0;i<m_SubWindows.size();i++)
    m_SubWindows[i]->OnWindowMove(x,y);


  if(m_CurrentMouseButton == GLUT_RIGHT_BUTTON)
    m_Camera[m_CurrentCamera]->Move(
              ((float)(x-m_CurrentMouseX)),
             -((float)(y-m_CurrentMouseY)),
              0.0,
              0.0,
              0.0);
  if(m_CurrentMouseButton == GLUT_LEFT_BUTTON){
    m_Camera[m_CurrentCamera]->Move(
              0.0,
              0.0,
              ((float)(y-m_CurrentMouseY)),
             -((float)(x-m_CurrentMouseX))/2.0,
              0.0);
  }
  if(m_CurrentMouseButton == GLUT_MIDDLE_BUTTON){
    m_Camera[m_CurrentCamera]->Move(
              0.0,
              0.0,
              0.0,
              -((float)(x-m_CurrentMouseX))/2.0,
             -((float)(y-m_CurrentMouseY))/2.0);
  }

  InputMouseMotion(x,y);
}
void  GLUTBaseApplication::NextCamera(){
  m_CurrentCamera++;
  if(m_CurrentCamera >= m_Camera.size())
    m_CurrentCamera = 0;
  m_Camera[m_CurrentCamera]->Apply();
}
void  GLUTBaseApplication::SetCamera(int num){
  if((num>=0) && (num < m_Camera.size())){
    m_CurrentCamera = num;
    m_Camera[m_CurrentCamera]->Apply();
  }
}
GLCamera *GLUTBaseApplication::GetCamera(){
  return  (m_Camera[m_CurrentCamera]);
}

void  GLUTBaseApplication::Run(){
	glutMainLoop();
}
void  GLUTBaseApplication::SetCurrent(){
  m_GLUTApp = this;
}
void  GLUTBaseApplication::Exit(){
	BaseFree();
  exit(0);
}


/**************************************************
 *
 * method StartSnapshot
 *
 *
 ****************************************************/

void GLUTBaseApplication::StartSnapshot(const char *filename, GLSnapshotMode mode=GLSnap_RGB){
  GLCamera *ccam = GetCamera();
  m_Snapshot.SetViewport(ccam->m_Width,ccam->m_Height);
  //m_Snapshot.SetViewport(200,100);
  m_Snapshot.SetMode(mode);
  m_Snapshot.SetFilename(filename);
  if(!m_Snapshot.Begin()){
    cout << "cannot save image"<<endl;
  }
  m_Snap=1;
}


// Virtual Callback functions
void  GLUTBaseApplication::Init(){}
void  GLUTBaseApplication::Render(){}
void  GLUTBaseApplication::Free(){}
void  GLUTBaseApplication::OnIdle(){}
void  GLUTBaseApplication::Resize(int width, int height){}
void  GLUTBaseApplication::InputMouseButton(int button, int state, int x, int y){}
void  GLUTBaseApplication::InputMouseMotion(int x, int y){}
void  GLUTBaseApplication::InputNormalKey(unsigned char key, int x, int y){}
void  GLUTBaseApplication::InputSpecialKey(int key, int x, int y){}


// Static Callback functions
void  GLUTBaseApplication::GLUTApp_Render(){
  if(m_GLUTApp!=NULL)
    m_GLUTApp->BaseRender();
}
void  GLUTBaseApplication::GLUTApp_OnIdle(){
  if(m_GLUTApp!=NULL)
    m_GLUTApp->OnIdle();
}
void  GLUTBaseApplication::GLUTApp_Resize(int width, int height){
  if(m_GLUTApp!=NULL)
    m_GLUTApp->BaseResize(width,height);
}
void  GLUTBaseApplication::GLUTApp_InputMouseButton(int button, int state, int x, int y){
  if(m_GLUTApp!=NULL)
    m_GLUTApp->BaseInputMouseButton(button,state,x,y);
}
void  GLUTBaseApplication::GLUTApp_InputMouseMotion(int x, int y){
  if(m_GLUTApp!=NULL)
    m_GLUTApp->BaseInputMouseMotion(x,y);
}
void  GLUTBaseApplication::GLUTApp_InputNormalKey(unsigned char key, int x, int y){
  if(m_GLUTApp!=NULL)
    m_GLUTApp->BaseInputNormalKey(key,x,y);
}
void  GLUTBaseApplication::GLUTApp_InputSpecialKey(int key, int x, int y){
  if(m_GLUTApp!=NULL)
    m_GLUTApp->BaseInputSpecialKey(key,x,y);
}
pGLUTBaseApplication    GLUTBaseApplication::m_GLUTApp = NULL;



/*--------------------------------------------------------------*/
/*--------------------------------------------------------------*/
/*--------------------------------------------------------------*/
/*--------------------------------------------------------------*/
/*--------------------------------------------------------------*/
/*--------------------------------------------------------------*/

/*

GLUTApplication::GLUTApplication(int argc, char **argv)
:GLUTBaseApplication(argc, argv){}

//GLUTApplication::GLUTApplication(int argc, char **argv, pTree config)
//:GLUTBaseApplication(argc, argv, config){}

GLUTApplication::~GLUTApplication(){}


void  GLUTApplication::Init(){
  glEnable(GL_DEPTH_TEST);
  GLfloat LightAmbient[]= { 0.0f, 0.0f, 0.0f, 0.0f }; 	
  GLfloat LightDiffuse[]= { 1.0f, 1.0f, 1.0f, 1.0f };	
  GLfloat LightPosition[]= { 1.0f, 0.1f, 1.0f, 0.0f };	
  glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient);
  glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse);	
  glLightfv(GL_LIGHT1, GL_POSITION,LightPosition);
  glEnable(GL_LIGHT1);

  glEnable(GL_COLOR_MATERIAL);
  glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT);
  glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);

//  glCullFace(GL_BACK);
  //glEnable(GL_CULL_FACE);
  //glEnable(GL_LIGHTING);

  
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  glEnable(GL_LINE_SMOOTH);
  glHint (GL_LINE_SMOOTH_HINT, GL_NICEST);
  glShadeModel (GL_SMOOTH);

  glClearColor(0.2,0.4,0.0,1.0);

}
void  GLUTApplication::Render(){
	glPushMatrix();
	glScalef(2.0,1.0,2.0);

	// Draw ground
	glColor3f(0.2f, 0.7f, 0.0f);
	glBegin(GL_QUADS);
		glVertex3f(-1.0f, 0.0f, -1.0f);
		glVertex3f(-1.0f, 0.0f,  1.0f);
		glVertex3f( 1.0f, 0.0f,  1.0f);
		glVertex3f( 1.0f, 0.0f, -1.0f);
	glEnd();
	glScalef(1.001,1.0,1.001);
	glColor3f(0.0f, 0.0f, 0.0f);
	glBegin(GL_LINE_STRIP);
		glVertex3f(-1.0f, 0.0f, -1.0f);
		glVertex3f(-1.0f, 0.0f,  1.0f);
		glVertex3f( 1.0f, 0.0f,  1.0f);
		glVertex3f( 1.0f, 0.0f, -1.0f);
		glVertex3f(-1.0f, 0.0f, -1.0f);
	glEnd();

	glPopMatrix();
}
void  GLUTApplication::Free(){
}
void  GLUTApplication::OnIdle(){
  glutPostRedisplay();
}
void  GLUTApplication::Resize(int width, int height){
}
void  GLUTApplication::InputMouseButton(int button, int state, int x, int y){

}
void  GLUTApplication::InputMouseMotion(int x, int y){
}
void  GLUTApplication::InputNormalKey(unsigned char key, int x, int y){
  if (key == 27) 
    Exit();
}
void  GLUTApplication::InputSpecialKey(int key, int x, int y){
}

*/




/*
 ofstream outFile("some_file_name.dat");//create output file object
streambuf * cout_backup=cout.rdbuf();//create backup of standard out
cout.rdbuf(outFile.rdbuf());//assign cout stream to outFile stream
cout<<"hello!"<<endl;//call any function that calls cout
cout.rdbuf(cout_backup);//restore the standard stream
outFile.close();//best to be neat and tidy about these things
*/
