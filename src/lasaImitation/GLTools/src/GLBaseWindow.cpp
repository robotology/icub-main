#include "GLBaseWindow.h"

int   GLBaseWindow::m_WindowWidth  = 1280;
int   GLBaseWindow::m_WindowHeight = 900;


GLBaseWindow::GLBaseWindow(){
  SetCurrent();
  /*
    glutInit(&argc, argv);
    glutInitWindowPosition(0,0);
    glutInitWindowSize(m_WindowWidth,m_WindowHeight);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
    m_WindowID = glutCreateWindow("-");

    glutDisplayFunc(GLBaseWindow::GLUTApp_Render);
    glutReshapeFunc(GLBaseWindow::GLUTApp_Resize);
    glutIdleFunc(GLBaseWindow::GLUTApp_OnIdle);
    glutIgnoreKeyRepeat(0);
    glutKeyboardFunc(GLBaseWindow::GLUTApp_InputNormalKey);
    glutSpecialFunc(GLBaseWindow::GLUTApp_InputSpecialKey);
    glutMouseFunc(GLBaseWindow::GLUTApp_InputMouseButton);
    glutMotionFunc(GLBaseWindow::GLUTApp_InputMouseMotion);
    //glutFullScreen();
*/
  //m_Snap                = false;
  
  m_CurrentMouseButton  = -1;
  m_CurrentMouseX       = 0;
  m_CurrentMouseY       = 0;
  
  m_CurrSubWindow     = NULL;

  m_LastPressedButton = -1;

  m_GridSize          = 20;

  m_FrameRateCounter  = 0;
  m_FrameRate         = 0;
  m_FrameRateTimer.Start(1000);

}


GLBaseWindow::~GLBaseWindow(){
  BaseFree();
}


void  GLBaseWindow::BaseFree(){
  // Freeing Cameras

  for(unsigned int i=0;i<m_SubWindows.size();i++){
    delete m_SubWindows[i];
  }
  m_SubWindows.clear();

  Free();
}


int   GLBaseWindow::GetFrameRate(){
  return m_FrameRate;
}

void  GLBaseWindow::BaseRender(){

  //if(m_Snap) m_Snapshot.Begin();
  glDisable(GL_CULL_FACE);
  glDisable(GL_LIGHTING);
  
  Render();

  unsigned int i;
  for(i=0;i<m_SubWindows.size();i++)
    m_SubWindows[i]->RenderWindow();


  /*if(m_Snap) {
    m_Snapshot.Finish();
    m_Snap = false;
  }*/
  glEnable(GL_CULL_FACE);
  glEnable(GL_LIGHTING);
}

void  GLBaseWindow::BaseResize(int width, int height){
  unsigned int i;
  
  m_WindowWidth  = width;
  m_WindowHeight = height;
  
  for(i=0;i<m_SubWindows.size();i++)
    m_SubWindows[i]->RefreshWindow();

  // Updating Snapshot
  //m_Snapshot.SetViewport(width,height);

  // Updating Console
  //m_Console.Resize(width,height);

  Resize(width,height);
}

void  GLBaseWindow::BaseInputNormalKey(unsigned char key, int x, int y){
  unsigned int i;
  for(i=0;i<m_SubWindows.size();i++)
    m_SubWindows[i]->OnWindowNormalKey(key);


  InputNormalKey(key,x,y);
}

void  GLBaseWindow::BaseInputSpecialKey(int key, int x, int y){
  unsigned int i;
  for(i=0;i<m_SubWindows.size();i++)
    m_SubWindows[i]->OnWindowSpecialKey(key);
  
  InputSpecialKey(key,x,y);
}

int  GLBaseWindow::BaseInputMouseButton(int button, int state, int x, int y){
  unsigned int i;  
  if(state == GLBW_BTNDOWN){
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

  GLSubWindow *ActiveSubWin = NULL;
  for(i=0;i<m_SubWindows.size();i++){
    int focus = m_SubWindows[i]->OnWindowClick(button,state,x,y);
    if(focus)
      ActiveSubWin = m_SubWindows[i];

    cFlag |= focus;  
  }
  for(i=0;i<m_SubWindows.size();i++){
    if(m_SubWindows[i]!=ActiveSubWin)
      m_SubWindows[i]->SetFocus(false);
  }  

  if(cFlag)
    return TRUE;
  
  InputMouseButton(button,state,x,y);
  
  return FALSE;
}

void  GLBaseWindow::BaseInputMouseMotion(int x, int y){
  unsigned int i;
  for(i=0;i<m_SubWindows.size();i++)
    m_SubWindows[i]->OnWindowMove(x,y);

  InputMouseMotion(x,y);
}

void  GLBaseWindow::SetCurrent(){
  m_GLBWin = this;
}

void  GLBaseWindow::AddSubWindows(pGLSubWindow win){
  if(win!=NULL)
    m_SubWindows.push_back(win);
}

/*
void  GLBaseWindow::Snap(char *filename, int x, int y,int w, int h, GLSnapshotMode mode){
  if(x>=0){
    m_Snapshot.SetViewport(x,y,w,h);
  }
  m_Snapshot.SetFilename(filename);
  m_Snapshot.SetMode(mode);
  m_Snap = true;
}

void  GLBaseWindow::Snap(char *filename, GLSnapshotMode mode){
  m_Snapshot.SetViewport(0,0,m_WindowWidth,m_WindowHeight);
  m_Snapshot.SetFilename(filename);
  m_Snapshot.SetMode(mode);
  m_Snap = true;
}
*/

// Virtual Callback functions
void  GLBaseWindow::Init(){}
void  GLBaseWindow::Render(){}
void  GLBaseWindow::Free(){}
void  GLBaseWindow::OnIdle(){}
void  GLBaseWindow::Resize(int width, int height){}
void  GLBaseWindow::InputMouseButton(int button, int state, int x, int y){}
void  GLBaseWindow::InputMouseMotion(int x, int y){}
void  GLBaseWindow::InputNormalKey(unsigned char key, int x, int y){}
void  GLBaseWindow::InputSpecialKey(int key, int x, int y){}


// Static Callback functions
void  GLBaseWindow::GLBWin_Render(){
  if(m_GLBWin!=NULL)
    m_GLBWin->BaseRender();
}
void  GLBaseWindow::GLBWin_Resize(int width, int height){
  if(m_GLBWin!=NULL)
    m_GLBWin->BaseResize(width,height);
}
void  GLBaseWindow::GLBWin_InputMouseButton(int button, int state, int x, int y){
  if(m_GLBWin!=NULL)
    m_GLBWin->BaseInputMouseButton(button,state,x,y);
}
void  GLBaseWindow::GLBWin_InputMouseMotion(int x, int y){
  if(m_GLBWin!=NULL)
    m_GLBWin->BaseInputMouseMotion(x,y);
}
void  GLBaseWindow::GLBWin_InputNormalKey(unsigned char key, int x, int y){
  if(m_GLBWin!=NULL)
    m_GLBWin->BaseInputNormalKey(key,x,y);
}
void  GLBaseWindow::GLBWin_InputSpecialKey(int key, int x, int y){
  if(m_GLBWin!=NULL)
    m_GLBWin->BaseInputSpecialKey(key,x,y);
}
pGLBaseWindow    GLBaseWindow::m_GLBWin = NULL;



