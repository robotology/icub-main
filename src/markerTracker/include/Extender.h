#ifndef YARP2_EXTENDER
#define YARP2_EXTENDER

#include <yarp/sig/Image.h>

#include <math.h>
#include <stdio.h>

namespace yarp {
  namespace sig {
    class Extender;
  }
}

class yarp::sig::Extender
{
public:

  Extender() {
  }
  
  double dist(double x1, double y1, double x2, double y2) {
    return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
  }


  float total_len;

  void Reset()
    {
      total_len = 0;
    }

  int  Move(ImageOf<PixelRgb>& src,
	    ImageOf<PixelRgb>& dest,
	    ImageOf<PixelRgb>& code,
	    ImageOf<PixelFloat>& src_x,
	    ImageOf<PixelFloat>& src_y,
	    ImageOf<PixelFloat>& src_m,
	    int ox, int oy, int& nx, int& ny, float dx, float dy, int del)
    {
      nx = ox;
      ny = oy;
      // scan around from ox, oy to find somewhere else going in the
      // same direction
      
      int done = 0;
      int sx = (int)(ox+dx*3.5+0.5);
      int sy = (int)(oy+dy*3.5+0.5);
      if (src_m.isPixel(sx,sy))
	{
	  if (src_m(sx,sy)>0.01)
	    {
	      if (fabs(src_x(sx,sy)*dx+src_y(sx,sy)*dy)>0.85)
		{
		  nx = sx;
		  ny = sy;
		  done = 1;
		}
	    }
	}
      if (!done)
	{
	  for (int ii=sx-2; ii<=sx+2 && !done; ii++)
	    {
	      for (int jj=sy-2; jj<=sy+2 && !done; jj++)
		{
		  if (src_m.isPixel(ii,jj))
		    {
		      if (src_m(ii,jj)>0.01)
			{
			  if (fabs(src_x(ii,jj)*dx+src_y(ii,jj)*dy)>0.85)
			    {
			      nx = ii;
			      ny = jj;
			      done = 1;
			    }
			}
		    }
		}
	    }
	}
      return done;
    }

  void Extend(ImageOf<PixelRgb>& src,
	      ImageOf<PixelRgb>& dest,
	      ImageOf<PixelRgb>& code,
	      ImageOf<PixelFloat>& src_x,
	      ImageOf<PixelFloat>& src_y,
	      ImageOf<PixelFloat>& src_m,
	      ImageOf<PixelFloat>& dest_x,
	      ImageOf<PixelFloat>& dest_y,
	      ImageOf<PixelFloat>& dest_m,
	      int ox, int oy, int sense, int show)
    {
      float m = src_m(ox,oy);
      int nx = ox, ny = oy;
      if (m>0.01)
	{
	  float dx = src_x(ox,oy)*sense;
	  float dy = src_y(ox,oy)*sense;
	  for (int i=0; i<100; i++)
	    {
	      int cx = nx, cy = ny;
	      int ok = 
		Move(src,dest,code,src_x,src_y,src_m,nx,ny,cx,cy,dx,dy,3);
	      if (!ok) break;
	      nx = cx;
	      ny = cy;
	      if (show)
		{
		  dest(nx,ny) = code(ox,oy);
		}
	    }
	  float len = dist(ox,oy,nx,ny);
	  total_len += len;
	}
    }
  double GetLength() { return total_len; }





  ImageOf<PixelInt> id;
  
};



#endif

