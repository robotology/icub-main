 /*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */


#ifndef SVOGRID_H
#define SVOGRID_H

class SVOGRID;

#include <qimage.h>
#include <qthread.h>
#include <qapplication.h>

#include <ipp.h>
#include "world3d.h"
#include <string>

#include <yarp/os/Property.h>

using namespace yarp::os;
using namespace std;


struct og_data{ 
  float occupancy;
  point3D vel;
  bool ray_max;
};




class SVOGRID : public QThread
{

 private:
  void ray_occupied(cell co);
  void ray_empty(cell ce);
  og_data*** make_og();
  void run();
  QImage* get_tex_y(cell c);
  QImage* get_tex_u(cell c);
  QImage* get_tex_v(cell c);

  QApplication *a;
  string*cfg;
  world3d*world;
  bool world_registered;
  int hd,px,py,psb_cell;
  point3D nbl,ntl,ntr,nbr,fbl,ftl,ftr,fbr;
  Ipp8u* ray_max_cell;
  double ray_max;
  double max_z,min_z, mind,maxd;
  double box_l_min, box_l_max;
  double baseline;
  double min_l,max_l;
  double occ,emp;
  int width,height,mos_width,mos_height,psb,psb32f,focus,offset,range,flow_scale;
  int numy_frame,numx_frame;
  double io_ll;

  Ipp8u* rec_im_ly;
  Ipp8u* rec_im_lu;
  Ipp8u* rec_im_lv;

    
 public:
  SVOGRID(QApplication* a_,string*cfg_);
  ~SVOGRID();
  void reg_world(world3d*w_){world = w_; world_registered=true;};
  cell point2Cell(point3D p);
  point3D cell2Point(cell c);
  void cell2im(cell c,IppiPoint* p);
  int get_ray_max_cell(int x, int y){return ray_max_cell[x+y*psb_cell];};
  void get_cell_params(int cell_z, double *z_front, double *len);
  point3D depth23D(int x, int y, double D_);
  QImage* get_tex_rgba(cell c);

  plane3D pn,pf;
  int numx,numy,numz;
  og_data*** OG;
  int zmin,ymin,xmin,zmax,ymax,xmax,pixels_per_box;
  int project_texture,rear_wall;

  QImage* tex_r;
  QImage* tex_g;
  QImage* tex_b;
  QImage* tex_a;
  QImage* tex_y;
  QImage* tex_u;
  QImage* tex_v;
  QImage*tex_rgba;
  IppiSize texsize;

};
#endif
