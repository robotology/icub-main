/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */


#ifndef WORLD3D_H
#define WORLD3D_H

class world3d;

#include <qgl.h>
#include <qslider.h>
#include <qapplication.h>

#include <math.h>


struct point3D{ 
  double x;
  double y;
  double z;
};

struct plane3D{ 
  point3D bl;
  point3D tl;
  point3D tr;
  point3D br;
};

struct cell{ 
  int x;
  int y;
  int z;
};



#include "svogrid.h"




class world3d : public QGLWidget
{
  Q_OBJECT

public:
  world3d(QApplication*a,QWidget *parent);
  ~world3d();
  void callback();
  void reg_svogrid(SVOGRID*s){svogrid=s;svogrid_registered=true;};
  
  
public slots:
  //if you change these, don't forget to
  //type: "moc-qt3 -o mworld3d.cc world3d.h"
  void rotateHeadX( int deg ) {rotOffsetX = (GLfloat)deg;}
  void rotateHeadY( int deg ) {rotOffsetY = (GLfloat)deg;}
  void rotateHeadZ( int deg ) {rotOffsetZ = (GLfloat)deg;}

  void translateWorldX( int td )  {transWorldX = (GLfloat)td/10.0;}
  void translateWorldY( int td )  {transWorldY = (GLfloat)td/10.0;}
  void translateWorldZ( int td )  {transWorldZ = (GLfloat)td/10.0;}

signals:

protected:
  virtual void initializeGL();
  virtual void paintGL();
  virtual void resizeGL( int w, int h );

private:
  void create_slider( QWidget *par, world3d *w, const char *s ,
		      int min_, int max_, int tick_space_, int start_val_);
  void render_h();
  void render_OG();
  void render_VG();
  void draw_OGFace(int x, int y);
  void draw_OGFaceTex(int x, int y);
  void draw_OGCube(int x, int y);
  void draw_OGCubeTex(int x, int y);
  void draw_OGPlane(plane3D p);
  void render_OGborder();
  GLuint makeHead();

  GLfloat transWorldX, transWorldY, transWorldZ;
  GLfloat rotWorldX,rotWorldY,rotWorldZ;
  GLfloat rotOffsetX,rotOffsetY,rotOffsetZ;
  GLfloat near,far,zstartoffset;

  QWidget *parent;
  QApplication*a;
  SVOGRID*svogrid;

  GLuint head;
  int pic;
  bool svogrid_registered;
  
};

#endif

