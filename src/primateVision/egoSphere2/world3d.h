/*
 * Copyright (C) 2007 Andrew Dankers
 * 
 */


#ifndef WORLD3D_H
#define WORLD3D_H

#include <qgl.h>
#include <qslider.h>
#include <qapplication.h>
#include <math.h>


struct point3D{ 
  double x;
  double y;
  double z;
};


class world3d : public QGLWidget
{
  Q_OBJECT

public:
  world3d(QApplication*a,QWidget *parent);
  ~world3d();
  void callback();
  void setim(QImage*qim_,double rx_,double ry_, double roll_, double pitch_, double yaw_);
  
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
  void CreateSphere(point3D c,double r,int n,int method,
		    double theta1,double theta2,double phi1,double phi2);
  
private:
  void create_slider( QWidget *par, world3d *w, const char *s ,
		      int min_, int max_, int tick_space_, int start_val_);

  GLfloat rotSphereX, rotSphereY, rotSphereZ;
  GLfloat transWorldX, transWorldY, transWorldZ;
  GLfloat rotWorldX,rotWorldY,rotWorldZ;
  GLfloat rotOffsetX,rotOffsetY,rotOffsetZ;
  GLfloat near,far,aspect;

  QWidget *parent;
  QApplication*a;

  point3D centre;
  QImage *qim;

  bool imset;

  double im_arc_h;
  double im_arc_v;
  double fwd_angle_h;
  double fwd_angle_v;
  double radius;

  int pic;
  
};

#endif

