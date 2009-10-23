
/*esLog project*/

///////////////////////////////////////////////////////////////////////////////////////////
//                                                                                       // 
// 3D representation of image on half sphere surface, with logaritmically sampled pixels //
//                                                                                       //
///////////////////////////////////////////////////////////////////////////////////////////

#ifndef WORLD3D_H
#define WORLD3D_H

#include <qgl.h>
#include <qslider.h>
#include <qapplication.h>
#include <math.h>
#include <q3hbox.h>
#include <qimage.h>
#include "pointfilter.h"


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
  void setim(QImage*qim_,double rx_,double ry_, double roll_, double pitch_, double yaw_, float a_, int w_, int h_);
  
public slots:
  //if you change these, don't forget to
  //type: "moc-qt3 -o mworld3d.cc world3d.h"
  void rotateHeadX( int deg ) {rotOffsetX = (GLfloat)deg;}
  void rotateHeadY( int deg ) {rotOffsetY = (GLfloat)deg;}
  void rotateHeadZ( int deg ) {rotOffsetZ = (GLfloat)deg;}

  void translateWorldX( int td )  {transWorldX = (GLfloat)td/10.0;}
  void translateWorldY( int td )  {transWorldY = (GLfloat)td/10.0;}
  void translateWorldZ( int td )  {transWorldZ = (GLfloat)td/10.0;}

  void set_k_lev_angle( int val ){k_lev_angle = val;}
  void set_phi_fov( int val ){phi_fov = (float)val*(PI/180.0);}
  void set_log_base( int val ){log_base = (float)val/100;}

  void save_log_im();
  void switch_rep(){point_rep = !point_rep;}
  void set_filter_on_off(bool state){filter_on = state;}

signals:

protected:
  virtual void initializeGL();
  virtual void paintGL();
  virtual void resizeGL( int w, int h );
  void CreateSphereLog(point3D c,double r, QImage *im); 
  void CreateSpherePoints(point3D c,double r, QImage *im); 
  void SetMeridians(float delta_out, float delta_in, int k, float b);
  void set_deg_res(float a, int w);
  void set_alpha_d(float a, int w, int h);
  void set_params(float f, float a, float b, float r);
  void set_eye_dist(int w, float a);
  void set_log_im(QImage *im, float delta_out, float theta, float delta_in, float phi_f, float b, int theta_max, float d, int inside_lev, int outside_lev);
  void set_log_im_filt(QImage *im, float delta_out, float theta, float delta_in, float phi_f, float b, int theta_max, float d, int inside_lev, int outside_lev);
  float* convertColor(QRgb qcol);
  void split_planes(QImage *src, unsigned char **red, unsigned char **green, unsigned char **blue, unsigned char **alpha, int w, int h);

private:
  void create_slider( QWidget *par, world3d *w, const char *s ,
		      int min_, int max_, int tick_space_, int start_val_, const QString &title);

  GLfloat rotSphereX, rotSphereY, rotSphereZ;
  GLfloat transWorldX, transWorldY, transWorldZ;
  GLfloat rotWorldX,rotWorldY,rotWorldZ;
  GLfloat rotOffsetX,rotOffsetY,rotOffsetZ;
  GLfloat near,far,aspect;

  QWidget *parent;
  QApplication*a;

  point3D centre;
  QImage *qim, *log_im, *transp_im;

  bool imset;

  bool point_rep;
  bool filter_on;

  double radius;


  int n_par;              //number of rings outside the fovea
  int n_fov;              //number of rings inside the fovea
  int n_mer;              //number of points in each ring
  float log_base;         //base of the logaritmic deresolution
  int k_lev_angle;        //angle with z-axe chosen to compare distances in phi and theta directions
  float deg_res;          //angular pixel resolution of the original image
  float delta_phi;        //angular step outside the fovea that changes logaritmically
  float delta_fov;        //constant angular step inside the fovea
  float phi_fov;          //angle of fovea  
  float eye_dist;         //distance (in pixels) of the image from the observation point
  float alpha_h;          //half horizontal angle of the image
  float alpha_d;          //half diagonal angle of the image
  int real_w;             //width of the image without borders
  int real_h;             //height of the image without borders 

};

#endif

