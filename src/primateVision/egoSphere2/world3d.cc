/*
 * Copyright (C) 2007 Andrew Dankers
 *  
 */


#include "world3d.h"
#include <qimage.h>


//float Light_Ambient[]=  { 0.1f, 0.1f, 0.1f, 1.0f };  
//float Light_Diffuse[]=  { 1.2f, 1.2f, 1.2f, 1.0f }; 
//float Light_Position[]= { 2.0f, 2.0f, 0.0f, 1.0f };

float Light_Ambient[]=  { 1.0f, 1.0f, 1.0f, 1.0f };  
float Light_Diffuse[]=  { 1.0f, 1.0f, 1.0f, 1.0f }; 
float Light_Position[]= { 1.0f, 1.0f, 1.0f, 1.0f };

static  GLuint texName[1]; // parameter is the number of textures in program


#define PI    3.141592654
#define TWOPI 6.283185308

#define SAVE 0


world3d::world3d(QApplication*a_,QWidget *par)
 : QGLWidget(par)
{

  if ( !QGLFormat::hasOpenGL() )
    qFatal( "This system has no OpenGL support" );
  else 
    printf("OpenGL support found\n");

  a=a_;
  parent=par;

  rotSphereX  = rotSphereY  = rotSphereZ  = 0.0;
  rotOffsetZ = rotOffsetX = -180;
  rotOffsetY = 0.0;
  transWorldX = transWorldY = 0.0;
  transWorldZ = 5.0;
  rotWorldX = rotWorldZ = rotWorldY = 0.0;

  near = 0.00;
  far  = 15.0;
  aspect = 1.0;

  imset=false;
  im_arc_h = 60.0*PI/180.0;  //image width covers aprox 40 deg
  im_arc_v = 45.0*PI/180.0;  //image height covers aprox 30 deg
  fwd_angle_h = PI/2;        //0..2PI       longitudes
  fwd_angle_v = 0.0;         //-PI/2..PI/2  lattitudes
  radius = 1.0;

  centre.x = 0.0;
  centre.y = 0.0;
  centre.z = 0.0;
 
  create_slider(  parent,this, SLOT(rotateHeadZ(int)), -360,0,60,-180);
  create_slider(  parent,this, SLOT(rotateHeadX(int)), -360,0,60,-180 );
  create_slider(  parent,this, SLOT(rotateHeadY(int)), -180,180,60,0 );
  create_slider( parent,this, SLOT(translateWorldX(int)) , -100,100,10,0);
  create_slider( parent,this, SLOT(translateWorldY(int)) , -100,100,10,0);
  create_slider( parent,this, SLOT(translateWorldZ(int)) , 0,100,10,50);

  parent->show();
  parent->resize(640,640);

  pic=0;

}



world3d::~world3d()
{
  makeCurrent();


}


void world3d::initializeGL()
{

  //printf("initializeGL\n");

  glClearColor(0.0f, 0.0f, 0.0f, 0.0f); 
  glClearDepth(1.0f);                                
  glEnable(GL_LIGHTING);
  glEnable(GL_DEPTH_TEST);
  glEnable (GL_LIGHT1); 
  glEnable(GL_COLOR_MATERIAL);
  glEnable (GL_BLEND);
  glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
  glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
  glEnable( GL_TEXTURE_2D );
  glDepthFunc(GL_LEQUAL);                            
  glShadeModel(GL_SMOOTH); 
  glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_FASTEST);
  glLightfv(GL_LIGHT1, GL_POSITION, Light_Position);
  glLightfv(GL_LIGHT1, GL_AMBIENT,  Light_Ambient);
  glLightfv(GL_LIGHT1, GL_DIFFUSE,  Light_Diffuse); 
  glColorMaterial(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

}


//THIS THREAD BEGINS WHEN "update()" IS CALLED!
void world3d::paintGL()
{

  glClear( GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT );//reset depth buffer

  //DRAW SPHERE:
  glLoadIdentity();
  glTranslatef(-transWorldX,-transWorldY,-transWorldZ);
  glRotatef(rotWorldX + rotOffsetX,1.0f,0.0f,0.0f);
  glRotatef(rotWorldY + rotOffsetY,0.0f,1.0f,0.0f);
  glRotatef(rotWorldZ + rotOffsetZ,0.0f,0.0f,1.0f);

  //rotation:
  glRotatef(rotSphereX,1.0f,0.0f,0.0f);
  glRotatef(rotSphereY,0.0f,1.0f,0.0f);
  //sphere:
  CreateSphere(centre,radius,50,0,
	       fwd_angle_h - im_arc_h/2.0,
	       fwd_angle_h + im_arc_h/2.0,
	       fwd_angle_v - im_arc_v/2.0,
	       fwd_angle_v + im_arc_v/2.0);

  //GAZE:
  glBegin( GL_LINE_LOOP );
  qglColor( Qt::yellow );
  glVertex3f( 0.0, 0.0, 0.0 ); //origin
  glVertex3f( 0.0, 0.0, 1.0 ); //center of image
  glEnd();


  //DRAW AXES:
  glLoadIdentity();
  glTranslatef(-transWorldX,-transWorldY,-transWorldZ);
  glRotatef(rotWorldX + rotOffsetX,1.0f,0.0f,0.0f);
  glRotatef(rotWorldY + rotOffsetY,0.0f,1.0f,0.0f);
  glRotatef(rotWorldZ + rotOffsetZ,0.0f,0.0f,1.0f);
  //x
  glBegin( GL_LINE_LOOP );
  qglColor( Qt::white );
  glVertex3f( 0.0, 0.0, 0.0 ); 
  glVertex3f( 1.0, 0.0, 0.0 );
  glEnd();
  //y
  glBegin( GL_LINE_LOOP );
  qglColor( Qt::green );
  glVertex3f( 0.0, 0.0, 0.0 );
  glVertex3f( 0.0, 1.0, 0.0 );
  glEnd();
  //z
  glBegin( GL_LINE_LOOP );
  qglColor( Qt::red );
  glVertex3f( 0.0, 0.0, 0.0 );
  glVertex3f( 0.0, 0.0, 1.0 );
  glEnd();



#if SAVE
 //save world window:
  pic++;
  QImage snapshot = QGLWidget::grabFrameBuffer();
  snapshot.save("/home/andrew/Desktop/w/w"+QString::number(pic)+".jpg","JPEG");
#endif


}


void world3d::resizeGL( int w, int h )
{
  //printf("resizeGL\n");

  aspect = (float)w/(float)h;
  glViewport( 0, 0, w, h );
  glMatrixMode( GL_PROJECTION );
  glLoadIdentity();
  gluPerspective(30.0, aspect, near, far);
  glMatrixMode( GL_MODELVIEW );
}


void world3d::create_slider( QWidget *parent_, world3d *world_, const char *slot_,int min_, int max_, int tick_space_, int start_val_)
{
  //printf("create_slider\n");

  QSlider *slider = new QSlider(min_, max_, tick_space_, start_val_,
				 Qt::Horizontal, parent_ );
  slider->setTickmarks( QSlider::TicksBelow );
  slider->show();
  QObject::connect( slider, SIGNAL(valueChanged(int)), world_, slot_ );
}



void world3d::callback()
{

  update();

}



/*
   Create a sphere centered at c, with radius r, and precision n
   Draw a point for zero radius spheres
   Use CCW facet ordering
   "method" is 0 for quads, 1 for triangles
      (quads look nicer in wireframe mode)
   Partial spheres can be created using theta1->theta2, phi1->phi2
   in radians 0 < theta < 2pi, -pi/2 < phi < pi/2
*/
void world3d::CreateSphere(point3D c,double r,int n,int method,
   double theta1,double theta2,double phi1,double phi2)
{
   int i,j;
   double jdivn,j1divn,idivn,dosdivn,unodivn=1/(double)n,ndiv2=(double)n/2,t1,t2,t3,cost1,cost2,cte1,cte3;
   cte3 = (theta2-theta1)/n;
   cte1 = (phi2-phi1)/ndiv2;
   dosdivn = 2*unodivn;
   point3D e,p,e2,p2;

   glColor4f(1.0,1.0,1.0,1.0);




   /* Handle special cases */
   if (r < 0)
      r = -r;
   if (n < 0){
      n = -n;
      ndiv2 = -ndiv2;
   }
   if (n < 4 || r <= 0) {
      glBegin(GL_POINTS);
      glVertex3f(c.x,c.y,c.z);
      glEnd();
      return;
   }

   if (imset){
     QImage tex = QGLWidget::convertToGLFormat((const QImage&) *qim );
     glGenTextures(1, texName);
     glBindTexture(GL_TEXTURE_2D,texName[0]);
     glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP);
     glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP);
     glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
     glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
     glTexImage2D(GL_TEXTURE_2D,0,3,tex.width(),tex.height(),
		  0,GL_RGBA,GL_UNSIGNED_BYTE, tex.bits() );
     glBindTexture(GL_TEXTURE_2D, texName[0]);
   }

   t2=phi1;
   cost2=cos(phi1);
   j1divn=0;
   for (j=0;j<ndiv2;j++) {
      t1 = t2;//t1 = phi1 + j * cte1;
      t2 += cte1;//t2 = phi1 + (j + 1) * cte1;
      t3 = theta1 - cte3;
      cost1 = cost2;//cost1=cos(t1);
      cost2 = cos(t2);
      e.y = sin(t1);
      e2.y = sin(t2);
      p.y = c.y + r * e.y;
      p2.y = c.y + r * e2.y;


      if (method == 0)
	glBegin(GL_QUAD_STRIP);
      else
	glBegin(GL_TRIANGLE_STRIP);

      idivn=0;
      jdivn=j1divn;
      j1divn+=dosdivn;//=2*(j+1)/(double)n;
      for (i=0;i<=n;i++) {
       //t3 = theta1 + i * (theta2 - theta1) / n;
         t3 += cte3;
         e.x = cost1 * cos(t3);
       //e.y = sin(t1);
         e.z = cost1 * sin(t3);
         p.x = c.x + r * e.x;
       //p.y = c.y + r * e.y;
         p.z = c.z + r * e.z;

         glNormal3f(e.x,e.y,e.z);
         glTexCoord2f(idivn,jdivn);
         glVertex3f(p.x,p.y,p.z);


         e2.x = cost2 * cos(t3);
       //e.y = sin(t2);
         e2.z = cost2 * sin(t3);
         p2.x = c.x + r * e2.x;
       //p.y = c.y + r * e.y;
         p2.z = c.z + r * e2.z;
         glNormal3f(e2.x,e2.y,e2.z);
         glTexCoord2f(idivn,j1divn);
         glVertex3f(p2.x,p2.y,p2.z);
         idivn += unodivn;
      }
      glEnd();
   }
   glDeleteTextures(1, texName);  

}


void world3d::setim(QImage*qim_,double rx_,double ry_, double roll_, double pitch_, double yaw_)
{
  qim=qim_;
  imset=true;
  rotSphereY = (GLfloat)rx_;
  rotSphereX = (GLfloat)ry_; 

  rotWorldX = -pitch_;//NOD
  rotWorldY = yaw_;   //PAN
  rotWorldZ = -roll_; //LR LEAN

}


