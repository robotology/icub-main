/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 *  
 */
 

#include "world3d.h"
#include <qimage.h>

float Light_Ambient[]=  { 1.0f, 1.0f, 1.0f, 1.0f };  
float Light_Diffuse[]=  { 1.0f, 1.0f, 1.0f, 1.0f }; 
float Light_Position[]= { 0.0f, 0.0f, 1.0f, 1.0f };

static  GLuint texName[1]; // parameter is the number of textures in program


#define SAVE 0


world3d::world3d(QApplication*a_,QWidget *par_)
 : QGLWidget(par_)
{

  if ( !QGLFormat::hasOpenGL() )
    qFatal( "This system has no OpenGL support" );
  else 
    printf("OpenGL support found\n");

  a=a_;
  parent=par_;

  svogrid_registered=false;

  rotOffsetZ = rotOffsetX = -180;
  rotOffsetY = 0.0;
  transWorldX = transWorldY = 0.0;
  transWorldZ = 5.0;
  rotWorldX = rotWorldZ = rotWorldY = 0.0;

  near = 0.00;
  far  = 15.0;

  create_slider( parent,this, SLOT(rotateHeadZ(int)), -360,0,60,-180);
  create_slider( parent,this, SLOT(rotateHeadX(int)), -360,0,60,-180 );
  create_slider( parent,this, SLOT(rotateHeadY(int)), -180,180,60,0 );
  create_slider( parent,this, SLOT(translateWorldX(int)) , -100,100,10,0);
  create_slider( parent,this, SLOT(translateWorldY(int)) , -100,100,10,0);
  create_slider( parent,this, SLOT(translateWorldZ(int)) , 0,100,10,50);

  parent->show();
  parent->resize(640,640);

  pic = 0;

}


 
world3d::~world3d()
{
  makeCurrent();
  glDeleteLists(head,1);
}


void world3d::initializeGL()
{

  glClearColor(0.0f, 0.0f, 0.0f, 0.0f); 
  glClearDepth(1.0f);                                
  glEnable(GL_LIGHTING);
  glEnable(GL_DEPTH_TEST);
  glEnable (GL_LIGHT1); 
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
  glEnable(GL_COLOR_MATERIAL);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  head = makeHead();

}


//THIS THREAD BEGINS WHEN "update()" IS CALLED!
void world3d::paintGL()
{

  glClear( GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT );//reset depth buffer

  //Prepare:
  glLoadIdentity();
  glTranslatef(-transWorldX,-transWorldY,-transWorldZ);
  glRotatef(rotWorldX + rotOffsetX,1.0f,0.0f,0.0f);
  glRotatef(rotWorldY + rotOffsetY,0.0f,1.0f,0.0f);
  glRotatef(rotWorldZ + rotOffsetZ,0.0f,0.0f,1.0f);

  //Lighting:
  glLightfv(GL_LIGHT1, GL_POSITION, Light_Position);
  glLightfv(GL_LIGHT1, GL_AMBIENT,  Light_Ambient);
  glLightfv(GL_LIGHT1, GL_DIFFUSE,  Light_Diffuse); 


  if (svogrid_registered){
    //a->lock();
    render_OGborder();
    render_OG();
    //render_VG();
    render_h();
    //a->unlock();
  }

  //Render!
  glCallList(head);



#if SAVE
 //save world window:
  pic++;
  QImage snapshot = QGLWidget::grabFrameBuffer();
  snapshot.save("/home/andrew/Desktop/w/w"+QString::number(pic)+".jpg","JPEG");
#endif


}


void world3d::resizeGL( int w, int h )
{
  GLfloat aspect = (GLfloat) ((double)w/(double)h);
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



//RENDERING:
void world3d::render_h()
{
  glColor4f(1.0,0.5,1.0,0.1);
  draw_OGPlane(svogrid->pn);
  glColor4f(0.5,1.0,1.0,0.1);
  draw_OGPlane(svogrid->pf);
}



void world3d::render_VG()
{
  cell c;
  point3D p;

  for (int x=0;x < svogrid->numx; x++){
    for (int y=0;y < svogrid->numy; y++){
      for (int z=0;z < svogrid->numz; z++){
	if (!(svogrid->OG[x][y][z].vel.x==0.0 && 
	      svogrid->OG[x][y][z].vel.y==0.0 &&
	      svogrid->OG[x][y][z].vel.z==0.0)){
	  
	  c.x=x;
	  c.y=y;
	  c.z=z;
	  p = svogrid->cell2Point(c);
	  
	  qglColor(Qt::yellow);
	  glBegin(GL_LINES);
	  
	  glVertex3f(p.x,p.y,p.z);
	  glVertex3f(p.x+svogrid->OG[x][y][z].vel.x,
		     p.y+svogrid->OG[x][y][z].vel.y,
		     p.z+svogrid->OG[x][y][z].vel.z);
	  
	  glEnd();
	  glPointSize(2.3);
	  glBegin(GL_POINTS);
	  glColor3f(1.0,0.0,0.0);
	  glVertex3f(p.x,p.y,p.z);
	  glEnd();
	  glPointSize(3.0);
	  glBegin(GL_POINTS);
	  glColor3f(0.0,1.0,0.0);
	  glVertex3f(p.x+svogrid->OG[x][y][z].vel.x,
		     p.y+svogrid->OG[x][y][z].vel.y,
		     p.z+svogrid->OG[x][y][z].vel.z);
	  glEnd();
	}
      }
    }
  }
  
}

void world3d::render_OG()
{

  //only draw faces that have image/depth data:
  for (int x=svogrid->xmin;x <= svogrid->xmax; x++){
    for (int y=svogrid->ymin;y <= svogrid->ymax; y++){
      

	if (svogrid->rear_wall==0 && svogrid->get_ray_max_cell(x,y)==(svogrid->numz-1)){
	  //draw nothing
	}
	else{
	  if (svogrid->project_texture==1){
	    //draw_OGFaceTex(x,y);
	    draw_OGCubeTex(x,y);
	  }
	  else{
	    //draw_OGFace(x,y);
	    draw_OGCube(x,y);
	  }
	}
      

    }
  }
  
}


void world3d::draw_OGFace(int x, int y)
{  
  
  double z_front;
  double len;
  int z = svogrid->get_ray_max_cell(x,y);
  svogrid->get_cell_params(z,&z_front,&len);

  double r=1.0-(((double)svogrid->get_ray_max_cell(x,y))/((double)svogrid->numz));
  double g=svogrid->get_ray_max_cell(x,y)/svogrid->numz;
  double b=0.5;
  double br=1.0;
  
  //DRAW FACE:
  glColor4f(r,b,g,br);

  typedef GLfloat point3[3];
  point3 vertices[4]={{x*len - svogrid->numx/2*len, y*len - svogrid->numy/2*len, z_front},
		      {x*len - svogrid->numx/2*len, (y+1)*len - svogrid->numy/2*len, z_front},
		      {(x+1)*len - svogrid->numx/2*len, (y+1)*len - svogrid->numy/2*len, z_front},
		      {(x+1)*len - svogrid->numx/2*len, y*len - svogrid->numy/2*len, z_front}};            
  point3 normal={ 0.0, 0.0, -1.0};
  glBegin(GL_QUADS);
  glNormal3fv(normal);
  glTexCoord2f(0.0, 0.0); glVertex3fv(vertices[3]);
  glTexCoord2f(0.0, 1.0); glVertex3fv(vertices[2]);
  glTexCoord2f(1.0, 1.0); glVertex3fv(vertices[1]);
  glTexCoord2f(1.0, 0.0); glVertex3fv(vertices[0]);
  glEnd();
  
}


void world3d::draw_OGCube(int x, int y)
{

  double z_front;
  double len;
  int z = svogrid->get_ray_max_cell(x,y);
  svogrid->get_cell_params(z,&z_front,&len);
  double r=1.0-(((double)svogrid->get_ray_max_cell(x,y))/((double)svogrid->numz));
  double g=svogrid->get_ray_max_cell(x,y)/svogrid->numz;
  double b=0.5;
  double br=1.0;


  //DRAW CUBE:
  glColor4f(r,g,b,br);

  GLfloat n[6][3] = {  /* Normals for the 6 faces of a cube. */
    {-1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {1.0, 0.0, 0.0},
    {0.0, -1.0, 0.0}, {0.0, 0.0, 1.0}, {0.0, 0.0, -1.0} };

  GLint faces[6][4] = {  /* Vertex indices for the 6 faces of a cube. */
    {0, 1, 2, 3}, {3, 2, 6, 7}, {7, 6, 5, 4},
    {4, 5, 1, 0}, {5, 6, 2, 1}, {7, 4, 0, 3} };

  GLfloat v[8][3];  /* Setup cube vertex data. */
  v[0][0] = v[1][0] = v[2][0] = v[3][0] = x*len - svogrid->numx/2*len;
  v[4][0] = v[5][0] = v[6][0] = v[7][0] = (x+1)*len - svogrid->numx/2*len;
  v[0][1] = v[1][1] = v[4][1] = v[5][1] = y*len - svogrid->numy/2*len;
  v[2][1] = v[3][1] = v[6][1] = v[7][1] = (y+1)*len - svogrid->numy/2*len;
  v[0][2] = v[3][2] = v[4][2] = v[7][2] = z_front+len;
  v[1][2] = v[2][2] = v[5][2] = v[6][2] = z_front;
  
  for (int i = 0; i < 6; i++) {
    glBegin(GL_QUADS);
    glNormal3fv(&n[i][0]);
    glVertex3fv(&v[faces[i][0]][0]);
    glVertex3fv(&v[faces[i][1]][0]);
    glVertex3fv(&v[faces[i][2]][0]);
    glVertex3fv(&v[faces[i][3]][0]);
    glEnd();
  }

}




void world3d::draw_OGCubeTex(int x, int y)
{

  double z_front;
  double len;
  int z = svogrid->get_ray_max_cell(x,y);
  svogrid->get_cell_params(z,&z_front,&len);
  double r=1.0-(((double)svogrid->get_ray_max_cell(x,y))/((double)svogrid->numz));
  double g=svogrid->get_ray_max_cell(x,y)/svogrid->numz;
  double b=0.5;
  double br=1.0;
  

  //DRAW CUBE:
  glColor4f(r,b,g,br);

  GLfloat n[6][3] = {  /* Normals for the 6 faces of a cube. */
    {-1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {1.0, 0.0, 0.0},
    {0.0, -1.0, 0.0}, {0.0, 0.0, 1.0}, {0.0, 0.0, -1.0} };

  GLint faces[6][4] = {  /* Vertex indices for the 6 faces of a cube. */
    {0, 1, 2, 3}, {3, 2, 6, 7}, {7, 6, 5, 4},
    {4, 5, 1, 0}, {5, 6, 2, 1}, {7, 4, 0, 3} };

  GLfloat v[8][3];  /* Setup cube vertex data. */
  v[0][0] = v[1][0] = v[2][0] = v[3][0] = x*len - svogrid->numx/2*len;
  v[4][0] = v[5][0] = v[6][0] = v[7][0] = (x+1)*len - svogrid->numx/2*len;
  v[0][1] = v[1][1] = v[4][1] = v[5][1] = y*len - svogrid->numy/2*len;
  v[2][1] = v[3][1] = v[6][1] = v[7][1] = (y+1)*len - svogrid->numy/2*len;
  v[0][2] = v[3][2] = v[4][2] = v[7][2] = z_front+len;
  v[1][2] = v[2][2] = v[5][2] = v[6][2] = z_front;
  
  for (int i = 0; i < 6; i++) {
    glBegin(GL_QUADS);
    glNormal3fv(&n[i][0]);
    glVertex3fv(&v[faces[i][0]][0]);
    glVertex3fv(&v[faces[i][1]][0]);
    glVertex3fv(&v[faces[i][2]][0]);
    glVertex3fv(&v[faces[i][3]][0]);
    glEnd();
  }

  
  //TEXTURE infront of front face:
  glColor4f(1.0,1.0,1.0,1.0);
  
  cell c;
  c.x = x;
  c.y = y;
  c.z = z;
  
  QImage tex = QGLWidget::convertToGLFormat( (const QImage&) *svogrid->get_tex_rgba(c));
  
  typedef GLfloat point3[3];
  point3 vertices[4]={{x*len - svogrid->numx/2*len, y*len - svogrid->numy/2*len, z_front},
		      {x*len - svogrid->numx/2*len, (y+1)*len - svogrid->numy/2*len, z_front},
		      {(x+1)*len - svogrid->numx/2*len, (y+1)*len - svogrid->numy/2*len, z_front},
		      {(x+1)*len - svogrid->numx/2*len, y*len - svogrid->numy/2*len, z_front}};   
  point3 normal={ 0.0, 0.0, -1.0};
  glGenTextures(1, texName);
  glBindTexture(GL_TEXTURE_2D,texName[0]);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
  glTexImage2D(GL_TEXTURE_2D,0,3,tex.width(),tex.height(),
	       0,GL_RGBA,GL_UNSIGNED_BYTE, tex.bits() );
  glBindTexture(GL_TEXTURE_2D, texName[0]);
  glBegin(GL_QUADS);
  glNormal3fv(normal);
  glTexCoord2f(0.0, 0.0); glVertex3fv(vertices[3]);
  glTexCoord2f(0.0, 1.0); glVertex3fv(vertices[2]);
  glTexCoord2f(1.0, 1.0); glVertex3fv(vertices[1]);
  glTexCoord2f(1.0, 0.0); glVertex3fv(vertices[0]);
  glEnd();
  glDeleteTextures(1, texName);  
  

}










void world3d::draw_OGFaceTex(int x, int y)
{

  double z_front;
  double len;
  int z = svogrid->get_ray_max_cell(x,y);
  svogrid->get_cell_params(z,&z_front,&len);

  //TEXTURE:

  glColor4f(1.0,1.0,1.0,1.0);

  cell c;
  c.x = x;
  c.y = y;
  c.z = z;

  QImage tex = QGLWidget::convertToGLFormat( (const QImage&) *svogrid->get_tex_rgba(c));
  
  typedef GLfloat point3[3];
  point3 vertices[4]={{x*len - svogrid->numx/2*len, y*len - svogrid->numy/2*len, z_front},
		      {x*len - svogrid->numx/2*len, (y+1)*len - svogrid->numy/2*len, z_front},
		      {(x+1)*len - svogrid->numx/2*len,(y+1)*len - svogrid->numy/2*len, z_front},
		      {(x+1)*len - svogrid->numx/2*len, y*len - svogrid->numy/2*len, z_front}};
  point3 normal={ 0.0, 0.0, -1.0};
  glGenTextures(1, texName);
  glBindTexture(GL_TEXTURE_2D,texName[0]);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
  glTexImage2D(GL_TEXTURE_2D,0,3,tex.width(),tex.height(),
	       0,GL_RGBA,GL_UNSIGNED_BYTE, tex.bits() );
  glBindTexture(GL_TEXTURE_2D, texName[0]);
  glBegin(GL_QUADS);
  glNormal3fv(normal);
  glTexCoord2f(0.0, 0.0); glVertex3fv(vertices[3]);
  glTexCoord2f(0.0, 1.0); glVertex3fv(vertices[2]);
  glTexCoord2f(1.0, 1.0); glVertex3fv(vertices[1]);
  glTexCoord2f(1.0, 0.0); glVertex3fv(vertices[0]);
  glEnd();
  glDeleteTextures(1, texName); 

  
}




void world3d::draw_OGPlane(plane3D p)
{
  
  GLfloat n[3] = {0.0, 0.0, -1.0};
  GLfloat v[4][3];
  //bl
  v[0][0]=p.bl.x;
  v[0][1]=p.bl.y;
  v[0][2]=p.bl.z;
  //tl
  v[1][0]=p.tl.x;
  v[1][1]=p.tl.y;
  v[1][2]=p.tl.z;
  //tr:
  v[2][0]=p.tr.x;
  v[2][1]=p.tr.y;
  v[2][2]=p.tr.z;
  //br:
  v[3][0]=p.br.x;
  v[3][1]=p.br.y;
  v[3][2]=p.br.z;
  
  glBegin(GL_QUADS);
  glNormal3fv(n);
  glVertex3fv(v[0]);
  glVertex3fv(v[1]);
  glVertex3fv(v[2]);
  glVertex3fv(v[3]);
  glEnd();
}

void world3d::render_OGborder()
{
  GLfloat f[4][3];  
  //br
  point3D p;
  cell c = {0,0,0};
  p = svogrid->cell2Point(c);
  f[0][0]=p.x;
  f[0][1]=p.y;
  f[0][2]=p.z;

  //tr
  c.x=0;
  c.y=svogrid->numy-1;
  p = svogrid->cell2Point(c);
  f[1][0]=p.x;
  f[1][1]=p.y;
  f[1][2]=p.z;

  //tl:
  c.x=svogrid->numx-1;
  p = svogrid->cell2Point(c);
  f[2][0]=p.x;
  f[2][1]=p.y;
  f[2][2]=p.z;

  //bl:
  c.y=0;
  p = svogrid->cell2Point(c);
  f[3][0]=p.x;
  f[3][1]=p.y;
  f[3][2]=p.z;


  GLfloat n[4][3];
  //br
  c.x=0;
  c.y=0;
  c.z=svogrid->numz-1;
  p = svogrid->cell2Point(c);
  n[0][0]=p.x;
  n[0][1]=p.y;
  n[0][2]=p.z;

  //tr
  c.x=0;
  c.y=svogrid->numy-1;
  p = svogrid->cell2Point(c);
  n[1][0]=p.x;
  n[1][1]=p.y;
  n[1][2]=p.z;

  //tl:
  c.x=svogrid->numx-1;
  p = svogrid->cell2Point(c);
  n[2][0]=p.x;
  n[2][1]=p.y;
  n[2][2]=p.z;

  //bl:
  c.y=0;
  p = svogrid->cell2Point(c);
  n[3][0]=p.x;
  n[3][1]=p.y;
  n[3][2]=p.z;

  
  glColor4f(1.0,0.5,0.5,1.0);
  glBegin(GL_LINE_LOOP);
  glVertex3fv(f[0]);
  glVertex3fv(f[1]);
  glVertex3fv(f[2]);
  glVertex3fv(f[3]);
  glEnd();

  glColor4f(0.5,1.0,0.5,1.0);
  glBegin(GL_LINE_LOOP);
  glVertex3fv(n[0]);
  glVertex3fv(n[1]);
  glVertex3fv(n[2]);
  glVertex3fv(n[3]);
  glEnd();

  glColor4f(1.0,1.0,1.0,0.5);
  glBegin(GL_LINES);
  glVertex3fv(n[0]);
  glVertex3fv(f[0]);
  glVertex3fv(n[1]);
  glVertex3fv(f[1]);
  glVertex3fv(n[2]);
  glVertex3fv(f[2]);
  glVertex3fv(n[3]);
  glVertex3fv(f[3]);
  glEnd();
}









GLuint world3d::makeHead()
{
  GLuint hlist = glGenLists( 1 );
  glNewList( hlist, GL_COMPILE );
  qglColor( Qt::yellow );
  glLineWidth( 2.0 );
  
  //axis:
  //x
  glBegin( GL_LINE_LOOP );
  qglColor( Qt::white );
  glVertex3f( 0.0, 0.0, 0.0 );
  glVertex3f( 0.5, 0.0, 0.0 );
  glEnd();
  //y
  glBegin( GL_LINE_LOOP );
  qglColor( Qt::white );
  glVertex3f( 0.0, 0.0, 0.0 );
  glVertex3f( 0.0, 0.5, 0.0 );
  glEnd();
  //z
  glBegin( GL_LINE_LOOP );
  qglColor( Qt::white );
  glVertex3f( 0.0, 0.0, 0.0 );
  glVertex3f( 0.0, 0.0, 0.5 );
  glEnd();
  
  //cam L:
  glBegin( GL_QUADS );
  glVertex3f( +0.15+0.03, +0.03, +0.04 );
  glVertex3f( +0.15+0.03, -0.03, +0.04 );
  glVertex3f( +0.15-0.03, -0.03, +0.04 );
  glVertex3f( +0.15-0.03, +0.03, +0.04 );
  
  qglColor( Qt::red );
  glVertex3f( +0.15+0.03, +0.03, +0.04 );
  glVertex3f( +0.15+0.03, -0.03, +0.04 );
  glVertex3f( +0.15+0.03, -0.03, -0.04 );
  glVertex3f( +0.15+0.03, +0.03, -0.04 );
  
  glVertex3f( +0.15+0.03, +0.03, +0.04 );
  glVertex3f( +0.15-0.03, +0.03, +0.04 );
  glVertex3f( +0.15-0.03, +0.03, -0.04 );
  glVertex3f( +0.15+0.03, +0.03, -0.04 );
  
  glVertex3f( +0.15-0.03, +0.03, +0.04 );
  glVertex3f( +0.15-0.03, -0.03, +0.04 );
  glVertex3f( +0.15-0.03, -0.03, -0.04 );
  glVertex3f( +0.15-0.03, +0.03, -0.04 );
  
  glVertex3f( +0.15+0.03, -0.03, +0.04 );
  glVertex3f( +0.15-0.03, -0.03, +0.04 );
  glVertex3f( +0.15-0.03, -0.03, -0.04 );
  glVertex3f( +0.15+0.03, -0.03, -0.04 );
  
  qglColor( Qt::yellow );
  glVertex3f( +0.15+0.03, +0.03, -0.04 );
  glVertex3f( +0.15+0.03, -0.03, -0.04 );
  glVertex3f( +0.15-0.03, -0.03, -0.04 );
  glVertex3f( +0.15-0.03, +0.03, -0.04 );
  
  //cam R:
  glVertex3f( -0.15+0.03, +0.03, +0.04 );
  glVertex3f( -0.15+0.03, -0.03, +0.04 );
  glVertex3f( -0.15-0.03, -0.03, +0.04 );
  glVertex3f( -0.15-0.03, +0.03, +0.04 );
  
  qglColor( Qt::red );
  glVertex3f( -0.15+0.03, +0.03, +0.04 );
  glVertex3f( -0.15+0.03, -0.03, +0.04 );
  glVertex3f( -0.15+0.03, -0.03, -0.04 );
  glVertex3f( -0.15+0.03, +0.03, -0.04 );
  
  glVertex3f( -0.15+0.03, +0.03, +0.04 );
  glVertex3f( -0.15-0.03, +0.03, +0.04 );
  glVertex3f( -0.15-0.03, +0.03, -0.04 );
  glVertex3f( -0.15+0.03, +0.03, -0.04 );
  
  glVertex3f( -0.15-0.03, +0.03, +0.04 );
  glVertex3f( -0.15-0.03, -0.03, +0.04 );
  glVertex3f( -0.15-0.03, -0.03, -0.04 );
  glVertex3f( -0.15-0.03, +0.03, -0.04 );
  
  glVertex3f( -0.15+0.03, -0.03, +0.04 );
  glVertex3f( -0.15-0.03, -0.03, +0.04 );
  glVertex3f( -0.15-0.03, -0.03, -0.04 );
  glVertex3f( -0.15+0.03, -0.03, -0.04 );
  
  qglColor( Qt::yellow );
  glVertex3f( -0.15+0.03, +0.03, -0.04 );
  glVertex3f( -0.15+0.03, -0.03, -0.04 );
  glVertex3f( -0.15-0.03, -0.03, -0.04 );
  glVertex3f( -0.15-0.03, +0.03, -0.04 );
  glEnd();
  
  
  //top brace:
  glBegin( GL_LINE_LOOP );
  qglColor( Qt::green );
  glVertex3f( +0.15, +0.03, 0.0 );
  glVertex3f( +0.15, +0.06, 0.0 );
  glVertex3f( -0.15, +0.06, 0.0 );
  glVertex3f( -0.15, +0.03, 0.0 );
  
  glVertex3f( +0.15, +0.03, 0.025 );
  glVertex3f( +0.15, +0.06, 0.025 );
  glVertex3f( -0.15, +0.06, 0.025 );
  glVertex3f( -0.15, +0.03, 0.025 );
  
  glVertex3f( +0.15, +0.03, 0.025 );
  glVertex3f( +0.15, +0.03, 0.0 );
  glVertex3f( +0.15, +0.06, 0.0 );
  glVertex3f( +0.15, +0.06, 0.025 );
  
  glVertex3f( -0.15, +0.03, 0.025 );
  glVertex3f( -0.15, +0.03, 0.0 );
  glVertex3f( -0.15, +0.06, 0.0 );
  glVertex3f( -0.15, +0.06, 0.025 );
  glEnd();
  
  //neck:
  glBegin( GL_QUADS );
  qglColor( Qt::magenta );
  glVertex3f( +0.05, +0.03, +0.02 );
  glVertex3f( +0.05, -0.06, +0.02 );
  glVertex3f( -0.05, -0.06, +0.02 );
  glVertex3f( -0.05, +0.03, +0.02 );
  
  glVertex3f( +0.05, +0.03, +0.02 );
  glVertex3f( +0.05, -0.06, -0.04 );
  glVertex3f( -0.05, -0.06, -0.04 );
  glVertex3f( -0.05, +0.03, +0.02 );
  
  qglColor( Qt::green );
  glVertex3f( +0.05, +0.03, +0.02 );
  glVertex3f( +0.05, -0.06, -0.04 );
  glVertex3f( +0.05, -0.06, +0.02 );
  glVertex3f( +0.05, +0.03, +0.02 );
  
  glVertex3f( -0.05, +0.03, +0.02 );
  glVertex3f( -0.05, -0.06, -0.04 );
  glVertex3f( -0.05, -0.06, +0.02 );
  glVertex3f( -0.05, +0.03, +0.02 );
  
  //base:
  qglColor( Qt::yellow );
  glVertex3f( +0.08, -0.06, +0.05 );
  glVertex3f( -0.08, -0.06, +0.05 );
  glVertex3f( -0.08, -0.09, +0.05 );
  glVertex3f( +0.08, -0.09, +0.05 );
  
  glVertex3f( +0.08, -0.06, -0.07 );
  glVertex3f( -0.08, -0.06, -0.07 );
  glVertex3f( -0.08, -0.09, -0.07 );
  glVertex3f( +0.08, -0.09, -0.07 );
  
  qglColor( Qt::magenta );
  glVertex3f( +0.08, -0.06, +0.05 );
  glVertex3f( +0.08, -0.06, -0.07 );
  glVertex3f( +0.08, -0.09, -0.07 );
  glVertex3f( +0.08, -0.09, +0.05 );
  
  glVertex3f( -0.08, -0.06, +0.05 );
  glVertex3f( -0.08, -0.06, -0.07 );
  glVertex3f( -0.08, -0.09, -0.07 );
  glVertex3f( -0.08, -0.09, +0.05 );
  
  qglColor( Qt::red );
  glVertex3f( +0.08, -0.06, +0.05 );
  glVertex3f( +0.08, -0.06, -0.07 );
  glVertex3f( -0.08, -0.06, -0.07 );
  glVertex3f( -0.08, -0.06, +0.05 );
  
  glVertex3f( +0.08, -0.09, +0.05 );
  glVertex3f( +0.08, -0.09, -0.07 );
  glVertex3f( -0.08, -0.09, -0.07 );
  glVertex3f( -0.08, -0.09, +0.05 );
  glEnd();

  glEndList();

  return hlist;
}
