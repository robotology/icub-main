// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Alex Bernardino, ISR-IST
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


#include <iCub/spherical_projection.h>

/* 
Computes spherical projection parameters, from pinhole projection parameters.
Considers possible radial distortion in the pinhole model.
WARNING: The function do not validate input parameters. These should be checked previously (see check_sp_params).
*/
void compute_sp_params(int input_lines, int input_cols,
                       int output_lines, int output_cols,
                       double fx, double fy,
                       double cx, double cy,
                       double k1, double k2,
                       double p1, double p2, 
                       double *fa, double *fe,
                       double *ca, double *ce)
{
   double max_azimuth = atan((input_cols-1-cx)/fx);
   double min_azimuth = atan(-cx/fx);
   double max_elevation = atan((input_lines-1-cy)/fy);
   double min_elevation = atan(-cy/fy);
   *fa = output_cols/(max_azimuth-min_azimuth);
   *fe = output_lines/(max_elevation-min_elevation);
   *ca = -min_azimuth*(*fa);
   *ce = -min_elevation*(*fe);
}

/*
Checks validity of spherical projection input parameters.
*/
bool check_sp_params(int input_lines, int input_cols,
                     int output_lines, int output_cols,
                     double fx, double fy,
                     double cx, double cy,
                     double k1, double k2,
                     double p1, double p2,
                     float *mapx, float *mapy)
{ 
   if(input_lines <= 1) return false;
   if(input_cols <= 1) return false;
   if(output_lines <= 1) return false;
   if(output_cols <= 1) return false;
   if(fx <= 0) return false;
   if(fy <= 0) return false;
   if(mapx == 0) return false;
   if(mapy == 0) return false;    
   return true;
}

/* Computes the warping map for spherical projection.
   To actually perform the image spherical projection, the computed map
   (mapx, mapy) must be used in conjuction with opencv funtion cvRemap.
*/
bool compute_sp_map(int input_lines, int input_cols,
                    int output_lines, int output_cols,
                    double fx, double fy,
                    double cx, double cy,
                    double k1, double k2,
                    double p1, double p2,
                    float *mapx, float *mapy)
{
   double fa, fe, ca, ce, a, e, x, y, r2, xd, yd;
   
   if(!check_sp_params(input_lines, input_cols, output_lines, output_cols, 
                        fx, fy, cx, cy, k1, k2, p1, p2, mapx, mapy))
        return false;

   compute_sp_params(input_lines, input_cols, output_lines, output_cols,
                     fx, fy, cx, cy, k1, k2, p1, p2, 
                     &fa, &fe, &ca, &ce);
   int i, j;
   for(i=0;i<output_lines;i++)
   {
      for(j=0;j<output_cols;j++)
      {
         a=(j-ca)/fa;
         e=(i-ce)/fe;
         x=tan(a);
         y=tan(e);
         r2 = x*x+y*y;
         xd = x*(1+k1*r2+k2*r2*r2)+2*p1*x*y+p2*(r2+2*x*x);
         yd = y*(1+k1*r2+k2*r2*r2)+p1*(r2+2*y*y)+2*p2*x*y;
         *mapx++ = (float)(fx*xd+cx);
         *mapy++ = (float)(fy*yd+cy);
      }
   }
   return true;
}

/* Computes the warping map for egosphere projection.
   To actually perform the image projection, the computed map
   (mapx, mapy) must be used in conjuction with opencv funtion cvRemap.

   Assumes iCub's specific reference frames:
   - World Reference:    X axis vertical pointing up
                         Y axis horizontal pointing backward
                         Z axis horizontal pointing right
  
   - Camera Reference:   X axis aligned with optical axis, pointing forward
                         Y axis aligned with image plane pointing right(increasing column index)
                         Z axis aligned with image plane pointing down (increasing line index)
                         
*/
bool compute_icub_egosp_map( int input_lines, int input_cols,
                        int output_lines, int output_cols,
                        double fx, double fy,
                        double cx, double cy,
                        double *R, //camera to world rotation matrix
                        float *mapx, float *mapy)
{
   // converts to intuitive reference frames:
   /* 

   - World Reference:    X axis horizontal pointing front
                         Y axis horizontal pointing left
                         Z axis vertical pointing up
  
   - Camera Reference:   X axis aligned with image plane pointing right(increasing column index)
                         Y axis aligned with image plane pointing down (increasing line index)
                         Z axis aligned with optical axis, pointing forward
   */

   double Rt[9];
   Rt[0] = -R[4];
   Rt[1] = -R[7];
   Rt[2] = R[1];
   Rt[3] = -R[5];
   Rt[4] = -R[8];
   Rt[5] = R[2];
   Rt[6] = -R[3];
   Rt[7] = -R[6];
   Rt[8] = R[0];
   compute_egosp_map(input_lines, input_cols, output_lines, output_cols, fx, fy, cx, cy, Rt, mapx, mapy);
   return true;
}
    

/* Computes the warping map for egosphere projection.
   To actually perform the image projection, the computed map
   (mapx, mapy) must be used in conjuction with opencv funtion cvRemap.

   Assumes the following reference frames:

   - World Reference:    X axis horizontal pointing front
                         Y axis horizontal pointing left
                         Z axis vertical pointing up
  
   - Camera Reference:   X axis aligned with image plane pointing right(increasing column index)
                         Y axis aligned with image plane pointing down (increasing line index)
                         Z axis aligned with optical axis, pointing forward
                         
*/
bool compute_egosp_map( int input_lines, int input_cols,
                        int output_lines, int output_cols,
                        double fx, double fy,
                        double cx, double cy,
                        double *R, //world to camera rotation matrix
                        float *mapx, float *mapy)
{
   double flongitude, flatitude, clongitude, clatitude, longitude, latitude, x, y;
   double Xworld,Yworld,Zworld,Xcamera,Ycamera,Zcamera;
  
   clongitude = output_cols/2;
   clatitude = output_lines/2;
   flongitude = output_cols/2/3.141529;
   flatitude = output_lines/3.141529;
   int i, j;
   for(i=0;i<output_lines;i++)
   {
      for(j=0;j<output_cols;j++)
      {
         latitude=-(i-clatitude)/flatitude;
         longitude=-(j-clongitude)/flongitude;
         //world coordinates x,y,z
         Xworld = cos(longitude)*cos(latitude);
         Yworld = sin(longitude)*cos(latitude);
         Zworld = sin(latitude);
         // convert to camera frame
         Xcamera= R[0]*Xworld+R[1]*Yworld+R[2]*Zworld;
         Ycamera= R[3]*Xworld+R[4]*Yworld+R[5]*Zworld;
         Zcamera= R[6]*Xworld+R[7]*Yworld+R[8]*Zworld;
         //compute the image projection
         if(Zcamera > 0)
         {
            x = Xcamera/Zcamera;
            y = Ycamera/Zcamera;
            *mapx++ = (float)(x*fx+cx);
            *mapy++ = (float)(y*fy+cy);
         }
         else
         {
             *mapx++ = -1;
             *mapy++ = -1;
         }
      }
   }
   return true;
}
