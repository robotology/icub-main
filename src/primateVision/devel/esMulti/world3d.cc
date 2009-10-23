
/*esLog project*/

///////////////////////////////////////////////////////////////////////////////////////////
//                                                                                       // 
// 3D representation of image on half sphere surface, with logaritmically sampled pixels //
//                                                                                       //
///////////////////////////////////////////////////////////////////////////////////////////


#include "world3d.h"
#include <qimage.h>
#include <qlabel.h>
#include <qlcdnumber.h>
#include <qpushbutton.h>
#include <qcheckbox.h>


float Light_Ambient[]=  { 1.0f, 1.0f, 1.0f, 1.0f };  
float Light_Diffuse[]=  { 1.0f, 1.0f, 1.0f, 1.0f }; 
float Light_Position[]= { 1.0f, 1.0f, 1.0f, 1.0f };


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

  point_rep = true;
  filter_on = false;

  imset=false;
  radius = 1.0;

  centre.x = 0.0;
  centre.y = 0.0;
  centre.z = 0.0;

  abs_disp = 16;

  r_const = abs_disp;
  r_offset = abs_disp +1;

  phi_fov = PI/18.0;
  k_lev_angle = 40;
  log_base = 1.03;
  alpha_h = PI/4;
  real_w = 100;
  real_h = 100;
  top_bord = 0;
  left_bord = 0;
  set_deg_res(alpha_h, real_w);
  set_eye_dist(real_w, alpha_h);
  set_alpha_d(alpha_h, real_w, real_h);
  set_params(phi_fov, alpha_d, log_base, deg_res);
  SetMeridians(delta_phi, delta_fov, k_lev_angle, log_base);


  create_slider( parent,this, SLOT(rotateHeadZ(int)), -360,0,60,-180, "Rot Z");
  create_slider( parent,this, SLOT(rotateHeadX(int)), -360,0,60,-180, "Rot X");
  create_slider( parent,this, SLOT(rotateHeadY(int)), -180,180,60,0, "Rot Y");
  create_slider( parent,this, SLOT(translateWorldX(int)) , -100,100,10,0, "Transl X");
  create_slider( parent,this, SLOT(translateWorldY(int)) , -100,100,10,0, "Transl Y");
  create_slider( parent,this, SLOT(translateWorldZ(int)) , 0,100,10,50, "Transl Z");
  create_slider( parent,this, SLOT(set_phi_fov(int)) , 1,40,1,10, "Fovea Angle");
  create_slider( parent,this, SLOT(set_k_lev_angle(int)) , 0,90,1,40, "Equal Dist Angle");
  create_slider( parent,this, SLOT(set_log_base(int)) , 103,200,1,120, "Log Base");

  //Create save button and switch representation button
  Q3HBox *qhb = new Q3HBox(parent);
  qhb->setFixedHeight(25);
  qhb->setSpacing(50);
  QPushButton *pb1 = new QPushButton("Save Logpolar", qhb);
  QObject::connect( pb1, SIGNAL(clicked()), this, SLOT(save_log_im()));
  QPushButton *pb2 = new QPushButton("Switch Representation", qhb);
  QObject::connect( pb2, SIGNAL(clicked()), this, SLOT(switch_rep()));
  QCheckBox *ch1 = new QCheckBox("Apply Filter", qhb);
  QObject::connect( ch1, SIGNAL(toggled(bool)), this, SLOT(set_filter_on_off(bool)));

  parent->show();
  parent->resize(640,640);


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
  //Setting image depending values
  //printf("paintGL1\n");
  if(imset){
    set_params(phi_fov, alpha_d, log_base, deg_res);
    SetMeridians(delta_phi, delta_fov, k_lev_angle, log_base);
  }
  
  glClear( GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT );//reset depth buffer
  
  //printf("paintGL2\n");
  //DRAW SPHERE:
  glLoadIdentity();
  glTranslatef(-transWorldX,-transWorldY,-transWorldZ);
  glRotatef(rotWorldX + rotOffsetX,1.0f,0.0f,0.0f);
  glRotatef(rotWorldY + rotOffsetY,0.0f,1.0f,0.0f);
  glRotatef(rotWorldZ + rotOffsetZ,0.0f,0.0f,1.0f);

  //rotation:
  glRotatef(rotSphereX,1.0f,0.0f,0.0f);
  glRotatef(rotSphereY,0.0f,1.0f,0.0f);

  //printf("paintGL3\n");
  //sphere:
  if(imset)
    {
      draw_multisphere(qim, depth_map, abs_disp, real_w, real_h, top_bord, left_bord);
    }
	   
  //printf("paintGL4\n");    
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


void world3d::create_slider( QWidget *parent_, world3d *world_, const char *slot_,int min_, int max_, int tick_space_, int start_val_, const QString &title)
{
  //printf("create_slider\n");
  Q3HBox *hb = new Q3HBox(parent_);
  hb->setFixedHeight(25);
  QSlider *slider = new QSlider(min_, max_, tick_space_, start_val_,
				Qt::Horizontal, hb );
  slider->setTickmarks( QSlider::TicksBelow );
  slider->show();

  QLabel *lb = new QLabel(title, hb );
  QLCDNumber *lcd = new QLCDNumber( hb );
  QObject::connect( slider, SIGNAL(valueChanged(int)), world_, slot_ );
  QObject::connect( slider, SIGNAL(valueChanged(int)),lcd, SLOT(display(int)));

}



void world3d::callback()
{

  a->lock();
  update();
  a->unlock();

}

//Projects onto a sphere starting points sampled with logaritmic deresolution
void world3d::CreateSpherePoints(point3D c,double r, QImage *im)
{  
  QRgb qcolor;
  float* fcolor;

  //Setting phi and theta step
  float phi = delta_phi;
  float theta = ((2.0*PI)/n_mer);

 if(imset)
    {
      //Computing logpolar sampled image
      if(filter_on)
	{
	  set_log_im_filt(im, phi, theta, delta_fov, phi_fov, log_base, n_mer, eye_dist, n_par, n_fov);
	}
      else
	{
	  set_log_im(im, phi, theta, delta_fov, phi_fov, log_base, n_mer, eye_dist, n_par, n_fov);
	}
	

      //START DRAWING POINTS
      glBegin(GL_POINTS);

      qcolor = im->pixel((int)(im->width()/2), (int)(im->height()/2));
      fcolor = convertColor(qcolor);
      glColor4fv(fcolor);
      glVertex3f(c.x, c.y, (float)r);   //plot centre
      float x = 0.0, y = 0.0, z = 0.0;  //3D coordinates

      //Cycle1, pixel outside fovea:'i' sets the parallel, starting from outside; 'j' sets the meridian
      for( int i = 0; i <= n_par; i++)
	{
	  for( int j = 0; j < n_mer; j++)
	    {
	      qcolor = log_im->pixel(i+n_fov, j);

	      if(qAlpha(qcolor)>0)
		{
		  z = c.z + (float)((float)r*(float)cos((double)phi*(double)pow(log_base,(float)i) + (double)phi_fov - (double)delta_phi));
		  x = -c.x - (float)((float)r*(float)sin((double)phi*(double)pow(log_base,(float)i) + (double)phi_fov - (double)delta_phi)*(float)cos((double)theta*(double)j));
		  y = c.y + (float)((float)r*(float)sin((double)phi*(double)pow(log_base,(float)i) + (double)phi_fov - (double)delta_phi)*(float)sin((double)theta*(double)j));

		  //Setting color of the 3D pixel with the corresponding one stored in the logpolar image
		  fcolor = convertColor(qcolor);
		  glColor4fv(fcolor);
		  glVertex3f(x, y, z);
		}
	    }
	}

      //Cycle2, pixel inside fovea:'k' sets the parallel, starting from outside; 'j' sets the meridian
      for( int k = 1; k <= n_fov; k++)
	{
	  for( int j = 0; j < n_mer; j++)
	    {
	      qcolor = log_im->pixel(k-1, j);

	      if(qAlpha(qcolor)>0)
		{

		  z = c.z + (float)((float)r*(float)cos((double)delta_fov*(double)k));
		  x = -c.x - (float)((float)r*(float)sin((double)delta_fov*(double)k)*(float)cos((double)theta*(double)j));
		  y = c.y + (float)((float)r*(float)sin((double)delta_fov*(double)k)*(float)sin((double)theta*(double)j));

		  //Setting color of the 3D pixel with the corresponding one stored in the logpolar image
		  fcolor = convertColor(qcolor);
		  glColor4fv(fcolor);
		  glVertex3f(x, y, z);
		}
	    }
	}

	glEnd();
  		
    }
}
 

//Projects onto a sphere starting points sampled with logaritmic deresolution and reconstruct the image
void world3d::CreateSphereLog(point3D c,double r, QImage *im)
{  
  QRgb qcolor, qcolor0, qcolor1, qcolor2, qcolor3;
  float* fcolor;

  //Setting phi and theta step
  float phi = delta_phi;
  float theta = ((2.0*PI)/n_mer);

 if(imset)
    {
      //Computing logpolar sampled image
      if(filter_on)
	{
	  set_log_im_filt(im, phi, theta, delta_fov, phi_fov, log_base, n_mer, eye_dist, n_par, n_fov);
	}
      else
	{
	  set_log_im(im, phi, theta, delta_fov, phi_fov, log_base, n_mer, eye_dist, n_par, n_fov);
	}
	 

      //START DRAWING SPHERE
      glBegin(GL_QUADS);

      float x = 0.0, y = 0.0, z = 0.0, x1 = 0.0, y1 = 0.0, z1 = 0.0, x2 = 0.0, y2 = 0.0, z2 = 0.0, x3 = 0.0, y3 = 0.0, z3 = 0.0;;  //3D coordinates

      //Cycle1, pixel outside fovea:'i' sets the parallel, starting from outside; 'j' sets the meridian
      for( int i = 0; i < n_par; i++)
	{
	  for( int j = 0; j < n_mer-1; j++)
	    {
	      qcolor0 = log_im->pixel(i+n_fov, j);
	      qcolor3 = log_im->pixel(i+n_fov, j+1);
	      qcolor1 = log_im->pixel(i+n_fov+1, j);
	      qcolor2 = log_im->pixel(i+n_fov+1, j+1);

	      if((qAlpha(qcolor0) + qAlpha(qcolor1) + qAlpha(qcolor2) + qAlpha(qcolor3))>0)
		{
		  //internal ring
		  z = c.z + (float)((float)r*(float)cos((double)phi*(double)pow(log_base,(float)i) + (double)phi_fov - (double)delta_phi));
		  x = -c.x - (float)((float)r*(float)sin((double)phi*(double)pow(log_base,(float)i) + (double)phi_fov - (double)delta_phi)*(float)cos((double)theta*(double)j));
		  y = c.y + (float)((float)r*(float)sin((double)phi*(double)pow(log_base,(float)i) + (double)phi_fov - (double)delta_phi)*(float)sin((double)theta*(double)j));

		  z3 = c.z + (float)((float)r*(float)cos((double)phi*(double)pow(log_base,(float)i) + (double)phi_fov - (double)delta_phi));
		  x3 = -c.x - (float)((float)r*(float)sin((double)phi*(double)pow(log_base,(float)i) + (double)phi_fov - (double)delta_phi)*(float)cos((double)theta*(double)(j+1)));
		  y3 = c.y + (float)((float)r*(float)sin((double)phi*(double)pow(log_base,(float)i) + (double)phi_fov - (double)delta_phi)*(float)sin((double)theta*(double)(j+1)));


		  //external ring
		  z1 = c.z + (float)((float)r*(float)cos((double)phi*(double)pow(log_base,(float)(i+1)) + (double)phi_fov - (double)delta_phi));
		  x1 = -c.x - (float)((float)r*(float)sin((double)phi*(double)pow(log_base,(float)(i+1)) + (double)phi_fov - (double)delta_phi)*(float)cos((double)theta*(double)j));
		  y1 = c.y + (float)((float)r*(float)sin((double)phi*(double)pow(log_base,(float)(i+1)) + (double)phi_fov - (double)delta_phi)*(float)sin((double)theta*(double)j));

		  z2 = c.z + (float)((float)r*(float)cos((double)phi*(double)pow(log_base,(float)(i+1)) + (double)phi_fov - (double)delta_phi));
		  x2 = -c.x - (float)((float)r*(float)sin((double)phi*(double)pow(log_base,(float)(i+1)) + (double)phi_fov - (double)delta_phi)*(float)cos((double)theta*(double)(j+1)));
		  y2 = c.y + (float)((float)r*(float)sin((double)phi*(double)pow(log_base,(float)(i+1)) + (double)phi_fov - (double)delta_phi)*(float)sin((double)theta*(double)(j+1)));
	     

		  //Setting color of the 3D pixel with the corresponding one stored in the logpolar image
		  fcolor = convertColor(qcolor0);
		  glColor4fv(fcolor);
		  glVertex3f(x, y, z);

		  fcolor = convertColor(qcolor1);
		  glColor4fv(fcolor);
		  glVertex3f(x1, y1, z1);


		  fcolor = convertColor(qcolor2);
		  glColor4fv(fcolor);
		  glVertex3f(x2, y2, z2);

		  fcolor = convertColor(qcolor3);
		  glColor4fv(fcolor);
		  glVertex3f(x3, y3, z3);

		}
	    }
	}

//Cycle1, pixel outside fovea; last meridian
      for( int ii = 0; ii < n_par; ii++)
	{
	  qcolor0 = log_im->pixel(ii+n_fov, n_mer-1);
	  qcolor3 = log_im->pixel(ii+n_fov, 0);
          qcolor1 = log_im->pixel(ii+n_fov+1, n_mer-1);
	  qcolor2 = log_im->pixel(ii+n_fov+1, 0);

	  if((qAlpha(qcolor0) + qAlpha(qcolor1) + qAlpha(qcolor2) + qAlpha(qcolor3))>0)
	    {
	      //internal ring
	      z = c.z + (float)((float)r*(float)cos((double)phi*(double)pow(log_base,(float)ii) + (double)phi_fov - (double)delta_phi));
	      x = -c.x - (float)((float)r*(float)sin((double)phi*(double)pow(log_base,(float)ii) + (double)phi_fov - (double)delta_phi)*(float)cos((double)theta*(double)(n_mer-1)));
	      y = c.y + (float)((float)r*(float)sin((double)phi*(double)pow(log_base,(float)ii) + (double)phi_fov - (double)delta_phi)*(float)sin((double)theta*(double)(n_mer-1)));

	      z3 = c.z + (float)((float)r*(float)cos((double)phi*(double)pow(log_base,(float)ii) + (double)phi_fov - (double)delta_phi));
	      x3 = -c.x - (float)((float)r*(float)sin((double)phi*(double)pow(log_base,(float)ii) + (double)phi_fov - (double)delta_phi)*(float)cos((double)theta*0));
	      y3 = c.y + (float)((float)r*(float)sin((double)phi*(double)pow(log_base,(float)ii) + (double)phi_fov - (double)delta_phi)*(float)sin((double)theta*0));


	      //external ring
	      z1 = c.z + (float)((float)r*(float)cos((double)phi*(double)pow(log_base,(float)(ii+1)) + (double)phi_fov - (double)delta_phi));
	      x1 = -c.x - (float)((float)r*(float)sin((double)phi*(double)pow(log_base,(float)(ii+1)) + (double)phi_fov - (double)delta_phi)*(float)cos((double)theta*(double)(n_mer-1)));
	      y1 = c.y + (float)((float)r*(float)sin((double)phi*(double)pow(log_base,(float)(ii+1)) + (double)phi_fov - (double)delta_phi)*(float)sin((double)theta*(double)(n_mer-1)));

	      z2 = c.z + (float)((float)r*(float)cos((double)phi*(double)pow(log_base,(float)(ii+1)) + (double)phi_fov - (double)delta_phi));
	      x2 = -c.x - (float)((float)r*(float)sin((double)phi*(double)pow(log_base,(float)(ii+1)) + (double)phi_fov - (double)delta_phi)*(float)cos((double)theta*0));
	      y2 = c.y + (float)((float)r*(float)sin((double)phi*(double)pow(log_base,(float)(ii+1)) + (double)phi_fov - (double)delta_phi)*(float)sin((double)theta*0));
	     

	      //Setting color of the 3D pixel with the corresponding one stored in the logpolar image
	      fcolor = convertColor(qcolor0);
	      glColor4fv(fcolor);
	      glVertex3f(x, y, z);

	      fcolor = convertColor(qcolor1);
	      glColor4fv(fcolor);
	      glVertex3f(x1, y1, z1);


	      fcolor = convertColor(qcolor2);
	      glColor4fv(fcolor);
	      glVertex3f(x2, y2, z2);

	      fcolor = convertColor(qcolor3);
	      glColor4fv(fcolor);
	      glVertex3f(x3, y3, z3);

	    }
	}

 
      //Cycle2, pixel inside fovea:'k' sets the parallel, starting from outside; 'j' sets the meridian
      for( int k = 1; k < n_fov; k++)
	{
	  for( int j = 0; j < n_mer-1; j++)
	    {
	      qcolor0 = log_im->pixel(k-1, j);
	      qcolor3 = log_im->pixel(k-1, j+1);
	      qcolor1 = log_im->pixel(k, j);
	      qcolor2 = log_im->pixel(k, j+1);

	      if((qAlpha(qcolor0) + qAlpha(qcolor1) + qAlpha(qcolor2) + qAlpha(qcolor3))>0)
		{

		  //external ring
		  z1 = c.z + (float)((float)r*(float)cos((double)delta_fov*(double)(k+1)));
		  x1 = -c.x - (float)((float)r*(float)sin((double)delta_fov*(double)(k+1))*(float)cos((double)theta*(double)j));
		  y1 = c.y + (float)((float)r*(float)sin((double)delta_fov*(double)(k+1))*(float)sin((double)theta*(double)j));

		  z2 = c.z + (float)((float)r*(float)cos((double)delta_fov*(double)(k+1)));
		  x2 = -c.x - (float)((float)r*(float)sin((double)delta_fov*(double)(k+1))*(float)cos((double)theta*(double)(j+1)));
		  y2 = c.y + (float)((float)r*(float)sin((double)delta_fov*(double)(k+1))*(float)sin((double)theta*(double)(j+1)));

	      
		  //internal ring
		  z = c.z + (float)((float)r*(float)cos((double)delta_fov*(double)k));
		  x = -c.x - (float)((float)r*(float)sin((double)delta_fov*(double)k)*(float)cos((double)theta*(double)j));
		  y = c.y + (float)((float)r*(float)sin((double)delta_fov*(double)k)*(float)sin((double)theta*(double)j));

		  z3 = c.z + (float)((float)r*(float)cos((double)delta_fov*(double)k));
		  x3 = -c.x - (float)((float)r*(float)sin((double)delta_fov*(double)k)*(float)cos((double)theta*(double)(j+1)));
		  y3 = c.y + (float)((float)r*(float)sin((double)delta_fov*(double)k)*(float)sin((double)theta*(double)(j+1)));



		  //Setting color of the 3D pixel with the corresponding one stored in the logpolar image
		  fcolor = convertColor(qcolor0);
		  glColor4fv(fcolor);
		  glVertex3f(x, y, z);

		  fcolor = convertColor(qcolor1);
		  glColor4fv(fcolor);
		  glVertex3f(x1, y1, z1);


		  fcolor = convertColor(qcolor2);
		  glColor4fv(fcolor);
		  glVertex3f(x2, y2, z2);	    

		  fcolor = convertColor(qcolor3);
		  glColor4fv(fcolor);
		  glVertex3f(x3, y3, z3);

		}
	    }
	}
 

      //Cycle2, pixel inside fovea; last meridian
      for( int kk = 1; kk < n_fov; kk++)
	{
	  qcolor0 = log_im->pixel(kk-1, n_mer-1);
	  qcolor3 = log_im->pixel(kk-1, 0);
	  qcolor1 = log_im->pixel(kk, n_mer-1);
	  qcolor2 = log_im->pixel(kk, 0);

	  if((qAlpha(qcolor0) + qAlpha(qcolor1) + qAlpha(qcolor2) + qAlpha(qcolor3))>0)
	    {

	      //external ring
	      z1 = c.z + (float)((float)r*(float)cos((double)delta_fov*(double)(kk+1)));
	      x1 = -c.x - (float)((float)r*(float)sin((double)delta_fov*(double)(kk+1))*(float)cos((double)theta*(double)(n_mer-1))); 
	      y1 = c.y + (float)((float)r*(float)sin((double)delta_fov*(double)(kk+1))*(float)sin((double)theta*(double)(n_mer-1)));

	      z2 = c.z + (float)((float)r*(float)cos((double)delta_fov*(double)(kk+1)));
	      x2 = -c.x - (float)((float)r*(float)sin((double)delta_fov*(double)(kk+1))*(float)cos((double)theta*0));
	      y2 = c.y + (float)((float)r*(float)sin((double)delta_fov*(double)(kk+1))*(float)sin((double)theta*0));

	    
	      //internal ring
	      z = c.z + (float)((float)r*(float)cos((double)delta_fov*(double)kk));
	      x = -c.x - (float)((float)r*(float)sin((double)delta_fov*(double)kk)*(float)cos((double)theta*(double)(n_mer-1)));
	      y = c.y + (float)((float)r*(float)sin((double)delta_fov*(double)kk)*(float)sin((double)theta*(double)(n_mer-1)));

	      z3 = c.z + (float)((float)r*(float)cos((double)delta_fov*(double)kk));
	      x3 = -c.x - (float)((float)r*(float)sin((double)delta_fov*(double)kk)*(float)cos((double)theta*0));
	      y3 = c.y + (float)((float)r*(float)sin((double)delta_fov*(double)kk)*(float)sin((double)theta*0));


	      //Setting color of the 3D pixel with the corresponding one stored in the logpolar image
	      fcolor = convertColor(qcolor0);
	      glColor4fv(fcolor);
	      glVertex3f(x, y, z);

	      fcolor = convertColor(qcolor1);
	      glColor4fv(fcolor);
	      glVertex3f(x1, y1, z1);


	      fcolor = convertColor(qcolor2);
	      glColor4fv(fcolor);
	      glVertex3f(x2, y2, z2);	    

	      fcolor = convertColor(qcolor3);
	      glColor4fv(fcolor);
	      glVertex3f(x3, y3, z3);

	    }
	}


      glEnd();

 
      //Internal triangle fan
      glBegin(GL_TRIANGLES);

      //centre
      qcolor = im->pixel((int)(im->width()/2), (int)(im->height()/2));

      //other points
      for(int l = 0; l < n_mer-1; l++)
	{
	  if((qAlpha(qcolor) + qAlpha(qcolor1) + qAlpha(qcolor2))>0)
	    {
	      qcolor1 = log_im->pixel(0, l);
	      qcolor2 = log_im->pixel(0, l+1);


	      z1 = c.z + (float)((float)r*(float)cos((double)delta_fov));
	      x1 = -c.x - (float)((float)r*(float)sin((double)delta_fov)*(float)cos((double)theta*(double)l));
	      y1 = c.y + (float)((float)r*(float)sin((double)delta_fov)*(float)sin((double)theta*(double)l));

	      z2 = c.z + (float)((float)r*(float)cos((double)delta_fov));
	      x2 = -c.x - (float)((float)r*(float)sin((double)delta_fov)*(float)cos((double)theta*(double)(l+1)));
	      y2 = c.y + (float)((float)r*(float)sin((double)delta_fov)*(float)sin((double)theta*(double)(l+1)));
	  	  

	      fcolor = convertColor(qcolor);
	      glColor4fv(fcolor);
	      glVertex3f(c.x, c.y, (float)r);


	      fcolor = convertColor(qcolor1);
	      glColor4fv(fcolor);
	      glVertex3f(x1, y1, z1);

	      fcolor = convertColor(qcolor2);
	      glColor4fv(fcolor);
	      glVertex3f(x2, y2, z2);
	    }
	}

      //last triangle
      if((qAlpha(qcolor0) + qAlpha(qcolor1) + qAlpha(qcolor2) + qAlpha(qcolor3))>0)
	{
	  qcolor1 = log_im->pixel(0, n_mer-1);
	  qcolor2 = log_im->pixel(0, 0);


	  z1 = c.z + (float)((float)r*(float)cos((double)delta_fov));
	  x1 = -c.x - (float)((float)r*(float)sin((double)delta_fov)*(float)cos((double)theta*(double)(n_mer-1)));
	  y1 = c.y + (float)((float)r*(float)sin((double)delta_fov)*(float)sin((double)theta*(double)(n_mer-1)));

	  z2 = c.z + (float)((float)r*(float)cos((double)delta_fov));
	  x2 = -c.x - (float)((float)r*(float)sin((double)delta_fov)*(float)cos((double)theta*0));
	  y2 = c.y + (float)((float)r*(float)sin((double)delta_fov)*(float)sin((double)theta*0));
	  	  

	  fcolor = convertColor(qcolor);
	  glColor4fv(fcolor);
	  glVertex3f(c.x, c.y, (float)r);


	  fcolor = convertColor(qcolor1);
	  glColor4fv(fcolor);
	  glVertex3f(x1, y1, z1);

	  fcolor = convertColor(qcolor2);
	  glColor4fv(fcolor);
	  glVertex3f(x2, y2, z2);
	}


      glEnd();
  		
    }
}
 

//Sets the number of points in theta direction
void world3d::SetMeridians(float delta_out, float delta_in, int k, float b)
{
  float m;
  int lev;
  float c;
  double angle;

  if(k > alpha_d*(180/PI))
    k = alpha_d*(180/PI);
 
  if(k < deg_res*(360/PI))
    k = deg_res*(360/PI);
 
  if(k >= phi_fov*(180/PI))
    {
      lev = (int)(log(((float)(k*PI/180)-phi_fov+delta_out)/delta_out)/log(b));

      c = ((TWOPI/delta_out)*pow(b, 1-lev))/(b-1.0);
      angle = (double)(pow(b, lev)*delta_out) + (double)(phi_fov-delta_out);

      m = (c*(float)sin(angle));
    }
  else
    {
      c = TWOPI/delta_in;
      angle = (double)(k*PI/180);

      m = (c*(float)sin(angle));
    }      

  n_mer =(int) m;
}


//Set horizontal angular resolution
void world3d::set_deg_res(float a, int w)
{
  deg_res = 2*(a/(float)w);
}


//Set diagonal angle
void world3d::set_alpha_d(float a, int w, int h)
{
  float diag = (float)sqrt(pow(h,2) + pow(w,2));
  alpha_d = (float)atan(tan(alpha_h)*(double)(diag/(float)w));
}


//Set number of levels inside and outside fovea
void world3d::set_params(float f, float a, float b, float r)
{
  delta_fov = r;
  delta_phi = r/(b-1);
  n_fov = (int)(f/r);
  n_par = (int)(log((a-f+delta_phi)/delta_phi)/(log(b)));

}


//Set distance from observation point
void world3d::set_eye_dist(int w, float a)
{
  eye_dist = (float)w/(2*tan(a));
}


//Set logpolar image
void world3d::set_log_im(QImage *im, float delta_out, float theta, float delta_in, float phi_f, float b, int theta_max, float d, int outside_lev, int inside_lev)
{
  log_im = new QImage(outside_lev+inside_lev+1, theta_max, 4*8);
  log_im->setAlphaBuffer(true);

  int x, y;
  float rho;

  //Pixel sampled outside of fovea
  for( int i = 0; i <= outside_lev; i++)
    {
      rho = d*(float)tan(((double)delta_out*(double)pow(b,(float)i)) + (double)phi_f - (double)delta_out);
      for( int j = 0; j < theta_max; j++)
	{
	  x = im->width()/2 + (int)(rho*(float)cos((double)theta*(double)j));
	  y = im->height()/2 - (int)(rho*(float)sin((double)theta*(double)j));
	  log_im->setPixel(i+inside_lev, j, im->pixel(x, y));
	}
    }
   
  //Pixels sampled inside of fovea, with constant delta rho
  for( int k = 1; k <= inside_lev; k++)
    {
      rho = d*(float)tan((double)delta_in*(double)k);
      for( int j = 0; j < theta_max; j++)
       {
         x = im->width()/2 + (int)(rho*(float)cos((double)theta*(double)j));
         y = im->height()/2 - (int)(rho*(float)sin((double)theta*(double)j));
         log_im->setPixel(k-1, j, im->pixel(x, y));
       }
   }

}


//Set logpolar filtered image
void world3d::set_log_im_filt(QImage *im, float delta_out, float theta, float delta_in, float phi_f, float b, int theta_max, float d, int outside_lev, int inside_lev)
{
  log_im = new QImage(outside_lev+inside_lev+1, theta_max, 4*8);
  log_im->setAlphaBuffer(true);


  //Separating color planes
  unsigned char **red_plane, **green_plane, **blue_plane, **alpha_plane;

  red_plane = new unsigned char*[im->height()];
  green_plane = new unsigned char*[im->height()];
  blue_plane = new unsigned char*[im->height()];
  alpha_plane = new unsigned char*[im->height()];

  for(int ii = 0; ii < im->height(); ii++)
    {
      red_plane[ii] = new unsigned char[im->width()];
      green_plane[ii] = new unsigned char[im->width()];
      blue_plane[ii] = new unsigned char[im->width()];
      alpha_plane[ii] = new unsigned char[im->width()];
    }

  split_planes(im, red_plane, green_plane, blue_plane, alpha_plane, im->width(), im->height());


  int x, y;
  float rho, delta_rho;
  unsigned char rr, gg, bb, aa;

  //Pixel sampled outside of fovea, filtered
  for( int i = 0; i < outside_lev; i++)
    {
      rho = d*(float)tan(((double)delta_out*(double)pow(b,(float)i)) + (double)phi_f - (double)delta_out);
      delta_rho = (d*(float)tan(((double)delta_out*(double)pow(b,(float)(i+1))) + (double)phi_f - (double)delta_out)) - (d*(float)tan(((double)delta_out*(double)pow(b,(float)i)) + (double)phi_f - (double)delta_out));
      int dim = 2*delta_rho+1;

      //Creating a gaussian filter for each ring, with size equal to the distance between that ring and the previous one
      pointfilter *filt = new pointfilter(dim, dim, delta_rho/3, delta_rho/3);

      for( int j = 0; j < theta_max; j++)
	{
	  x = im->width()/2 + (int)(rho*(float)cos((double)theta*(double)j));
	  y = im->height()/2 - (int)(rho*(float)sin((double)theta*(double)j));

	  rr = filt->pixel_filter(red_plane, x, y);
	  gg = filt->pixel_filter(green_plane, x, y);
	  bb = filt->pixel_filter(blue_plane, x, y);
	  aa = filt->pixel_filter(alpha_plane, x, y);

	  log_im->setPixel(i+inside_lev, j, qRgba(rr,gg,bb,qAlpha(im->pixel(x,y))));
	}
    }
   
  //external level, not filtered
  rho = d*(float)tan(((double)delta_out*(double)pow(b,(float)outside_lev)) + (double)phi_f - (double)delta_out);
  for( int jj = 0; jj < theta_max; jj++)
    {
      x = im->width()/2 + (int)(rho*(float)cos((double)theta*(double)jj));
      y = im->height()/2 - (int)(rho*(float)sin((double)theta*(double)jj));

      log_im->setPixel(outside_lev+inside_lev, jj, im->pixel(x, y));
    }

  //Pixels sampled inside of fovea, with constant delta rho, not filtered
  for( int k = 1; k <= inside_lev; k++)
    {
      rho = d*(float)tan((double)delta_in*(double)k);
      for( int j = 0; j < theta_max; j++)
       {
         x = im->width()/2 + (int)(rho*(float)cos((double)theta*(double)j));
         y = im->height()/2 - (int)(rho*(float)sin((double)theta*(double)j));
         log_im->setPixel(k-1, j, im->pixel(x, y));
       }
   }

}



//Set image
void world3d::setim(QImage *qim_, QImage *depth_, double rx_, double ry_, double roll_, double pitch_, double yaw_, float a_, int w_, int h_, int left_, int top_)
{ 
  qim=qim_;
  qim->setAlphaBuffer(true);
  imset=true;
  alpha_h = a_;
  depth_map = depth_;

  rotSphereY = (GLfloat)rx_;
  rotSphereX = (GLfloat)ry_; 

  rotWorldX = -pitch_;//NOD
  rotWorldY = yaw_;   //PAN
  rotWorldZ = -roll_; //LR LEAN

  //Computing horizontal angular resolution
  set_deg_res(alpha_h, w_);

  //Computing diagonal angle
  set_alpha_d(alpha_h, w_, h_);


  //Computing distance from observation point
  set_eye_dist(w_, alpha_h);

  real_w = w_;
  real_h = h_;
  top_bord = top_;
  left_bord = left_;

  set_grey_params(depth_map, 2*abs_disp+1, real_w, real_h);


}


//Saving the logpolar image obtained with the current settings
void world3d::save_log_im()
{
  int angle = (int)(phi_fov*180/PI);
  if(imset)
    {
      set_log_im(qim, delta_phi,((2.0*PI)/n_mer), delta_fov, phi_fov, log_base, n_mer, eye_dist, n_par, n_fov);
      log_im->save("log"+QString::number(log_base)+"_fovea"+QString::number(angle)+"_m"+QString::number(n_mer)+".jpg","JPEG");
      set_log_im_filt(qim, delta_phi,((2.0*PI)/n_mer), delta_fov, phi_fov, log_base, n_mer, eye_dist, n_par, n_fov);
      log_im->save("filt_log"+QString::number(log_base)+"_fovea"+QString::number(angle)+"_m"+QString::number(n_mer)+".jpg","JPEG");
    }
}



//Convert color format RGB_8u into float
float* world3d::convertColor(QRgb qcol)
{
  float fcol[4], *color;
  fcol[0] = (float)qRed(qcol)/255;
  fcol[1] = (float)qGreen(qcol)/255;
  fcol[2] = (float)qBlue(qcol)/255;
  fcol[3] = (float)qAlpha(qcol)/255;

  color = fcol;

  return color;
}


//separate planes of a 4-channel image
void world3d::split_planes(QImage *src, unsigned char **red, unsigned char **green, unsigned char **blue, unsigned char **alpha, int w, int h)
{
  QRgb col;
  for(int x = 0; x < w; x++)
    {
      for(int y = 0; y < h; y++)
	{
	  col = src->pixel(x,y);
	  red[y][x] = qRed(col);
	  green[y][x] = qGreen(col);
	  blue[y][x] = qBlue(col);
	  alpha[y][x] = qAlpha(col);
	}
    }
}



//find minimum value of a greyscale image
int world3d::find_min_grey(QImage *src, int w, int h)
{
  int min_val = 255;
  for(int x = 0; x < w; x++)
    {
      for(int y = 0; y < h; y++)
	{
	  min_val = std::min(min_val, src->pixelIndex(x,y));
	}
    }
  return min_val;
}


//find maximum value of a greyscale image
int world3d::find_max_grey(QImage *src, int w, int h)
{
  int max_val = 0;
  for(int x = 0; x < w; x++)
    {
      for(int y = 0; y < h; y++)
	{
	  max_val = std::max(max_val, src->pixelIndex(x,y));
	}
    }
  return max_val;
}


//set min and max grey values and grey step
void world3d::set_grey_params(QImage *src, int tot_lev, int w, int h)
{
  min_grey = find_min_grey(src, w, h) - 1;
  max_grey = find_max_grey(src, w, h);

  grey_step = (float)(max_grey - min_grey +1)/(float)tot_lev;
}


//set alpha mask for current disparity level
void world3d::set_current_mask(QImage *src, QImage *mask, int lev, int w, int h)
{
  int l = lev + abs_disp + 1;

  //Creating disparity mask
  for(int x = 0; x < w; x++)
    {
      for(int y = 0; y < h; y++)
	{
	  if((src->pixelIndex(x,y)>(min_grey+(float)(l-1)*grey_step)) && (src->pixelIndex(x,y))<=(min_grey+(float)l*grey_step))
	    {
	      mask->setPixel(x,y,255);
	    }
	  else
	    {
	      mask->setPixel(x,y,0);
	    }
	}
    }
}


//mask the original image and create the disparity map corresponding to mask
void world3d::create_disparity_map(QImage *src, QImage *dest, QImage *mask, int w, int h, int left, int top)
{
  for(int x = left; x < w + left; x++)
    {
      for(int y = top; y < h + top; y++)
	{
	  QRgb col;
	  col = src->pixel(x,y);
	  dest->setPixel(x,y,qRgba(qRed(col),qGreen(col),qBlue(col),mask->pixelIndex(x-left,y-top)));
	}
    }
}


//draw the multisphere
void world3d::draw_multisphere(QImage *src, QImage *depth, int max_disp, int w, int h, int top, int left) 
{
  float r_step = radius/(float)(max_disp+1);

  QImage *disp_map = new QImage(src->width(), src->height(), 8*4);
  disp_map->setAlphaBuffer(true);

  QImage *current_mask = new QImage(w, h, 8, 256);
  for(unsigned int ui=0;ui<256;ui++) //set to B&W
    {
      current_mask->setColor(ui,qRgb(ui,ui,ui));
    }


 

  //Drawing multisphere
  if(point_rep)
    {
      for(int k = -max_disp; k <= max_disp; k++)
	{
	  set_current_mask(depth, current_mask, k, w, h);
	  create_disparity_map(src, disp_map, current_mask, w, h, left, top);
	  CreateSpherePoints(centre,(float)r_const/(float)(k + r_offset), disp_map);
	}
    }
  else
    {
      for(int k = -max_disp; k <= max_disp; k++)
	{
	  set_current_mask(depth, current_mask, k, w, h);
	  create_disparity_map(src, disp_map, current_mask, w, h, left, top);
	  CreateSphereLog(centre,(float)r_const/(float)(k + r_offset), disp_map);
	} 
    }
  
}
