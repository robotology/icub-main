/*
 * animationview.cpp
 */

/*
#ifdef __APPLE__
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#include <QApplication.h>
#else
#if defined(WIN32) || defined(WIN64)
#include <windows.h>
#endif
#include <GL/glu.h>
#include <GL/glut.h>
#endif
*/

#include <qevent.h>
#include <qcursor.h>

#include "animationview.h"
#include "bvh.h"

#include "settings.h"

#define KEY_SHIFT 1
#define KEY_CTRL  2
#define KEY_ALT   4

AnimationView::AnimationView(QWidget* parent,yarp::os::ResourceFinder& config) : QGLWidget(parent)
{
    m_bInitialized=false;

    // fake glut initialization
    int args=1;
    // make sure the "iCubimator" string is not const char*
    char arg0[]="iCubimator";
    char* arg[]={arg0};
    glutInit(&args,arg);

    // init
    dragX=0;
    dragY=0;
    changeX=0;
    changeY=0;
    changeZ=0;
    xSelect=false;
    ySelect=false;
    zSelect=false;

    leftMouseButton=false;
    modifier=0;
    
    setMouseTracking(true);
    setFocusPolicy(QWidget::StrongFocus);

    pBVH=new BVH(config);

    connect(&mTimer,SIGNAL(timeout()),this,SLOT(timerTimeout()));
}

AnimationView::~AnimationView()
{
    if (pBVH) delete pBVH;
}

void AnimationView::drawFloor()
{
  float alpha=(100-Settings::floorTranslucency())/100.0;

  //glEnable(GL_DEPTH_TEST);
  glBegin(GL_QUADS);
  for(int i=-10;i<10;i++)
  {
    for(int j=-10;j<10;j++)
    {
      if((i+j) % 2)
      {
          glColor4f(0.1,0.1,0.1,alpha);
      }
      else
      {
          glColor4f(0.6,0.6,0.6,alpha);
      }

      glVertex3f(i*40,0,j*40);
      glVertex3f(i*40,0,(j+1)*40);
      glVertex3f((i+1)*40,0,(j+1)*40);
      glVertex3f((i+1)*40,0,j*40);
    }
  }
  glEnd();
}


void AnimationView::setProjection()
{
  gluPerspective(60.0,((float)width())/height(),1,2000);
}

void AnimationView::paintGL()
{
  draw();
}

void AnimationView::paintOverlayGL()
{
  draw();
}

void AnimationView::initializeGL()
{
    GLfloat position0[]={0.0,80.0,100.0,1.0};
    GLfloat ambient0[]={0.2,0.2,0.2,1.0};
    //GLfloat diffuse0[]={0.5,0.5,0.5,0.2};
    //GLfloat specular0[]={0.5,0.5,0.2,0.5};

    GLfloat position1[]={0.0,80.0,-100.0,1.0};
    GLfloat ambient1[]={0.2,0.2,0.2,1.0};
    GLfloat diffuse1[]={0.5,0.5,0.5,1.0};
    GLfloat specular1[]={1.0,1.0,1.0,1.0};

    glViewport(0,0,width(),height());

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

    glLightfv(GL_LIGHT0,GL_AMBIENT,ambient0);
    //glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse0);
    //glLightfv(GL_LIGHT0, GL_SPECULAR, specular0);

    glLightfv(GL_LIGHT1,GL_AMBIENT,ambient1);
    glLightfv(GL_LIGHT1,GL_DIFFUSE,diffuse1);
    glLightfv(GL_LIGHT1,GL_SPECULAR,specular1);

    glEnable(GL_NORMALIZE);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    setProjection();

    glLightfv(GL_LIGHT0,GL_POSITION,position0);
    glLightfv(GL_LIGHT1,GL_POSITION,position1);
    
    m_bInitialized=true;
}

void AnimationView::draw()
{
    if(!isValid()) initializeGL();

    if(Settings::fog())
    {
        glEnable(GL_FOG);
        {
            GLfloat fogColor[4]={0.5,0.5,0.5,0.3};
            int fogMode=GL_EXP; // GL_EXP2, GL_LINEAR
            glFogi(GL_FOG_MODE,fogMode);
            glFogfv(GL_FOG_COLOR,fogColor);
            glFogf(GL_FOG_DENSITY,0.005);
            glHint(GL_FOG_HINT,GL_DONT_CARE);
            glFogf(GL_FOG_START,200.0);
            glFogf(GL_FOG_END,2000.0);
        }
    }
    else
        glDisable(GL_FOG);

    glClearColor(0.5,0.5,0.5,0.3); /* fog color */
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDisable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_COLOR_MATERIAL);
    glShadeModel(GL_FLAT);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    camera.setModelView();
  
    pBVH->draw();
  
    drawFloor();
}

void AnimationView::mouseMoveEvent(QMouseEvent* event)
{
    if(leftMouseButton)
    {
        QPoint dragPos=QCursor::pos();

        // since MacOSX doesn't like the old "drag and snap back" solution, we're going for a
        // more elaborate solution

        // calculate drag distance from last dragging point
        dragX=dragPos.x()-oldDragX;
        dragY=dragPos.y()-oldDragY;

        // remember the current position as last dragging point
        oldDragX=dragPos.x();
        oldDragY=dragPos.y();

        // if mouse has moved sufficiently far enough from first clicking position ...
        if(abs(clickPos.x()-dragPos.x())>100 || abs(clickPos.y()-dragPos.y())>100)
        {
            // set remembered drag position back to first click position
            oldDragX=clickPos.x();
            oldDragY=clickPos.y();
            // set mouse cursor back to first click position
            QCursor::setPos(clickPos);
        }

        if(modifier & KEY_SHIFT)
            camera.pan(dragX/2,dragY/2,0);
        else if(modifier & KEY_ALT)
        {
            camera.pan(0,0,dragY);
            camera.rotate(0,dragX);
        }
        else
            camera.rotate(dragY,dragX);

        repaint();
     }
}

void AnimationView::mousePressEvent(QMouseEvent* event)
{
    if (event->button()==Qt::LeftButton)
    { 
        clickPos=QCursor::pos();
        // put in position for distance calculation
        oldDragX=clickPos.x();
        oldDragY=clickPos.y();
        leftMouseButton=true;
    }
}

void AnimationView::mouseReleaseEvent(QMouseEvent* event)
{
    if(event->button()==Qt::LeftButton)
    {
        leftMouseButton=false;
    }
  /*
  if(event->button()==Qt::LeftButton)
  {
    // move mouse cursor back to the beginning of the dragging process
    //// causes problems on leopard
    ////QCursor::setPos(returnPos);
    // show mouse cursor again
    setCursor(Qt::ArrowCursor);
    leftMouseButton=false;
    propDragging=0;
  }
  */
}

void AnimationView::mouseDoubleClickEvent(QMouseEvent* event)
{
}

void AnimationView::wheelEvent(QWheelEvent* event)
{
    camera.pan(0,0,-event->delta()/12);
    repaint();
}

void AnimationView::keyPressEvent(QKeyEvent* event)
{
    switch(event->key())
    {
    case Qt::Key_PageUp:
        camera.pan(0,0,-5);
        repaint();
        break;
    case Qt::Key_PageDown:
        camera.pan(0,0,5);
        repaint();
        break;
    case Qt::Key_Shift:
        modifier|=KEY_SHIFT;
        xSelect = true;
        repaint();
        break;
    case Qt::Key_Alt:
        modifier|=KEY_ALT;
        ySelect = true;
        repaint();
        break;
    case Qt::Key_Control:
        modifier|=KEY_CTRL;
        zSelect = true;
        repaint();
        break;
    }
    event->ignore();
}

void AnimationView::keyReleaseEvent(QKeyEvent* event)
{
    switch(event->key())
    {
    case Qt::Key_Shift:
        modifier&=!KEY_SHIFT;
        xSelect = false;
        repaint();
        break;
    case Qt::Key_Alt:
        modifier&=!KEY_ALT;
        ySelect = false;
        repaint();
        break;
    case Qt::Key_Control:
        modifier&=!KEY_CTRL;
        zSelect = false;
        repaint();
        break;
    }
    event->ignore();
}

/*
void AnimationView::drawCircle(int axis,float radius,int width)
{
  GLint circle_points=100;
  float angle;

  glLineWidth(width);
  switch(axis)
  {
    case 0: glColor4f(1,0,0,1); break;
    case 1: glColor4f(0,1,0,1); break;
    case 2: glColor4f(0,0,1,1); break;
  }
  glBegin(GL_LINE_LOOP);
  for(int i=0;i<circle_points;i++)
  {
    angle=2*M_PI*i/circle_points;
    switch(axis)
    {
      case 0: glVertex3f(0,radius*cos(angle),radius*sin(angle)); break;
      case 1: glVertex3f(radius*cos(angle),0,radius*sin(angle)); break;
      case 2: glVertex3f(radius*cos(angle),radius*sin(angle),0); break;
    }
  }
  glEnd();
  glBegin(GL_LINES);
  switch(axis)
  {
    case 0: glVertex3f(-10,0,0); glVertex3f(10,0,0); break;
    case 1: glVertex3f(0,-10,0); glVertex3f(0,10,0); break;
    case 2: glVertex3f(0,0,-10); glVertex3f(0,0,10); break;
  }
  glEnd();

}
*/

void AnimationView::resetCamera()
{
  camera.reset();
  repaint();
}

// handle widget resizes
void AnimationView::resizeEvent(QResizeEvent* newSize)
{
    if (!m_bInitialized) return;

    int w=newSize->size().width();
    int h=newSize->size().height();
    
    // reset coordinates
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    // resize GL viewport
    glViewport(0,0,w,h);

    // set up aspect ratio
    setProjection();

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

