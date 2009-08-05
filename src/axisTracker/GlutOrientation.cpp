#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#include <yarp/sig/GlutOrientation.h>
#include <yarp/sig/ImageFile.h>
#include <yarp/sig/ImageDraw.h>
#include <yarp/sig/IntegralImage.h>

#ifdef __QNX__
#ifndef for
#define for if (1) for
#endif
#endif

#define ORIENT_SRC "/yarp/conf/orient.txt"

#define NOISE_LEVEL (0)
#define USE_LUMINANCE_FILTER
//#define USE_MASK
#define USE_DEMOCRACY

using namespace yarp::sig;

ImageOf<PixelRgb> GlutOrientation::shifts[FINE_ORIENTATION_SHIFTS], GlutOrientation::mean;

static ImageOf<PixelMono> mshifts[FINE_ORIENTATION_SHIFTS], 
		mmean;

#define SatisfySize(x,y) (y).resize(x)

class FineOrientationData
{
public:
  int active;
  ImageOf<PixelFloat> orient;
  
  FineOrientationData() { active = 0; }

  void Read(const char *fname = NULL)
    {
      if (!active)
	{
	  printf("Catalog loading from %s...\n",
		 (fname!=NULL)?fname:ORIENT_SRC);
	  ImageOf<PixelFloat> orient_pre;
	  bool ok = yarp::sig::file::read(orient_pre,
					  (fname!=NULL)?fname:ORIENT_SRC);
	  if (!ok) {
	    printf("*** error: cannot find %s\n",
		   (fname!=NULL)?fname:ORIENT_SRC);
	    exit(1);
	  }

	  orient.resize(4,orient_pre.height());
	  for (int i=0; i<orient_pre.height(); i++)
	    {
	      for (int j=0; j<4; j++)
		{
		  orient(j,i) = orient_pre(j,i);
		}
	    }

	  printf("Catalog loaded\n");
	  active = 1;
	}
    }

  ImageOf<PixelFloat>& Orient()
    {
      if (!active)
	{
	  Read();
	}
      return orient;
    }
} fine_orientation_data;


void GlutOrientation::Init(const char *fname)
{
  fine_orientation_data.Read(fname);
}


#define SHIFTS FINE_ORIENTATION_SHIFTS






static float sq(float x)
{
  return x*x;
}

static float pdist(PixelRgb& pix0, PixelRgb& pix1)
{
  double d1 = fabs(pix0.r-pix1.r);
  double d2 = fabs(pix0.g-pix1.g);
  double d3 = fabs(pix0.b-pix1.b);
  if (d2>d1) d1=d2;
  if (d3>d1) d1=d3;
  return d1;

  //return sqrt(fabs(sq(pix0.r-pix1.r)+sq(pix0.g-pix1.g)+sq(pix0.b-pix1.b)));
}

#define OBINS 64


//#define USE_COLOR_INFO


#ifdef USE_COLOR_INFO

void GlutOrientation::Apply(ImageOf<PixelRgb>& src,
				ImageOf<PixelRgb>& dest,
				ImageOf<PixelFloat>& xdata, 
				ImageOf<PixelFloat>& ydata, 
				ImageOf<PixelFloat>& mdata)
{
  int view = 1; //(dest.width()>0);
  ImageOf<PixelFloat>& orient = fine_orientation_data.Orient();
  static ImageOf<PixelMono> mask;
  static IntegralImage ii;
  if (view)
    {
      dest.resize(src);
    }
  for (int i=0;i<SHIFTS;i++)
    {
      shifts[i].resize(src);
    }
  mean.resize(src);

  static int shifts_x[SHIFTS], shifts_y[SHIFTS];
  
  int ct = 0;
  for (int dx=-1; dx<=2; dx++)
    {
      for (int dy=-1; dy<=2; dy++)
	{
	  assert(ct<SHIFTS);
	  ii.Offset(src,shifts[ct],dx,dy,1);
	  shifts_x[ct] = dx;
	  shifts_y[ct] = dy;
	  ct++;
	}
    }

  static ImageOf<PixelFloat> mean, var, lx, ly, agree;
  ImageOf<PixelRgb>& mono = src;
  SatisfySize(mono,mean);
  SatisfySize(mono,var);
  SatisfySize(mono,lx);
  SatisfySize(mono,ly);
  SatisfySize(mono,agree);
  SatisfySize(mono,xdata);
  SatisfySize(mono,ydata);
  SatisfySize(mono,mdata);
  int response_ct = 0;
  IMGFOR(mono,x,y)
    {
      float total = 0;
      float total2 = 0;
      PixelRgb& pix0 = src(x,y);
      for (int k=0; k<SHIFTS; k++)
	{
	  PixelRgb& pix1 = shifts[k](x,y);
	  float v = pdist(pix0,pix1);
	  total += v;
#ifdef USE_LUMINANCE_FILTER
	  total2 += v*v;
#endif
	}
      mean(x,y) = total/SHIFTS;
#ifdef USE_LUMINANCE_FILTER
      var(x,y) = total2/SHIFTS - (total/SHIFTS)*(total/SHIFTS);
#endif
    }
  //printf(">>> %d\n", __LINE__); fflush(stdout);
  int quotient = 16;
  if (use_quotient>0)
    {
      quotient = use_quotient;
    }
  if (1) IMGFOR(mono,x,y)
    {
      float theta = 0;
      float strength = 0;
      float number = 0;
      if (use_democracy)
	{
	  float qx = 0;
	  float qy = 0;
	  int qct = 0;
	  {
	    for (int dx=-1; dx<=2; dx++)
	      {
		for (int dy=-1; dy<=2; dy++)
		  {
		    float total = 0;
		    float total2 = 0;
		    PixelRgb& pix0 = src.SafePixel(x+dx,y+dy);
		    for (int k=0; k<SHIFTS; k++)
		      {
			PixelRgb& pix1 = shifts[k](x,y);
			float v = pdist(pix0,pix1);
			total += v;
			total2 += v*v;
		      }
		    
		    float v = total/SHIFTS; //mean.SafePixel(x+dx,y+dy);
		    int m = 0;
		    if (1) for (int k=0; k<SHIFTS; k++)
		      {
			m *= 2;
			if (pdist(src.SafePixel(x+dx,y+dy),shifts[k](x,y))>=v)
			  {
			    m += 1;
			  }
		      }
		    assert(orient(0,m) == m);
		    float theta1 = orient(1,m)*4;
		    float strength1 = orient(2,m);
		    float number1 = orient(3,m);
		    if (strength1>0.001)
		      {
			qx += cos(theta1);
			qy += sin(theta1);
			strength += strength1;
			number += number1;
			qct++;
		      }
		  }
	      }
	  }
	  if (qct>0)
	    {
	      float d = sqrt(qx*qx+qy*qy);
	      if (d>0.0001 && d>8)
		{
		  theta = atan2(qy,qx)/2;
		  strength /= qct;
		  number /= qct;
		}
	      else
		{
		  theta = 0;
		  strength = 0;
		  number = 0;
		}
	    }
	}
      else
	{
	  float v = mean(x,y);
	  int m = 0;
	  PixelRgb& ref = src(x,y);
	  for (int k=0; k<SHIFTS; k++)
	    {
	      m *= 2;
	      if (pdist(ref,shifts[k](x,y))>=v)
		{
		  m += 1;
		}
	    }
	  //assert(orient(0,m) == m);
	  theta = orient(1,m)*2;
	  strength = orient(2,m);
	  number = orient(3,m);
	}
      float mtheta = theta*2;
      //lx(x,y) = cos(theta*2);
      //ly(x,y) = sin(theta*2);


      // moving this experimentally
      float dx = sin(theta);
      float dy = cos(theta);

      if (theta>M_PI)
	{
	  theta -= M_PI;
	}
      if (theta<-M_PI)
	{
	  theta += M_PI;
	}
      theta += M_PI/2;
      //float dx = sin(theta);
      //float dy = cos(theta);
      //xdata(x,y) = dx;
      //ydata(x,y) = dy;
      xdata(x,y) = cos(theta);
      ydata(x,y) = sin(theta);
      float md = number;
      int itheta = (int)(255*(theta/M_PI)+0.5);
      if (itheta>255) itheta = 255;
      if (itheta<0) itheta = 0;
      float lf = -1;
      if (use_luminance_filter>0)
	{
	  lf = use_luminance_filter*use_luminance_filter;
	}
      if (1)
	{
	  //number = log(number+1);
	  number /= quotient;
	  //number /= 2000;
	  number *= strength;
#ifdef USE_LUMINANCE_FILTER
	  // really don't need this
	  //float scale = (var(x,y)>25);
	  if (use_luminance_filter)
	    {
	      float scale = (var(x,y)>lf);
	      if (scale>1) scale = 1;
	      number *= scale;
	    }
#endif
	  if (number>1) number = 1;
	  dx *= number;
	  dy *= number;
	  //lx(x,y) *= number;  //PFADD3
	  //ly(x,y) *= number;  //PFADD3
	}
      mdata(x,y) = number;
      if (view)
	{
	  int ix = (int)fabs(dx*255);
	  int iy = (int)fabs(dy*255);
	  int iz = ((dx*dy)<-0.001);
	  iz = iz*255;

	  PixelRgb pix(ix,iy,iz);
	  dest(x,y) = pix;
	}
    }
}


#else


void GlutOrientation::Apply(ImageOf<PixelRgb>& src,
				ImageOf<PixelRgb>& dest,
				ImageOf<PixelFloat>& xdata, 
				ImageOf<PixelFloat>& ydata, 
				ImageOf<PixelFloat>& mdata)
{
  int view = 1; //(dest.width()>0);
  ImageOf<PixelFloat>& orient = fine_orientation_data.Orient();
  static ImageOf<PixelMono> mask, rmono;
  static IntegralImage ii;
  rmono.copy(src);
  if (view)
    {
      SatisfySize(src,dest);
    }
  for (int i=0;i<SHIFTS;i++)
    {
      SatisfySize(src,mshifts[i]);
    }
  SatisfySize(src,mmean);
  static int shifts_x[SHIFTS], shifts_y[SHIFTS];
  
  int ct = 0;
  for (int dx=-1; dx<=2; dx++)
    {
      for (int dy=-1; dy<=2; dy++)
	{
	  assert(ct<SHIFTS);
	  ii.Offset(rmono,mshifts[ct],dx,dy,1);
	  shifts_x[ct] = dx;
	  shifts_y[ct] = dy;
	  ct++;
	}
    }

  static ImageOf<PixelFloat> mean, var, lx, ly, agree;
  ImageOf<PixelMono>& mono = rmono;
  SatisfySize(mono,mean);
  SatisfySize(mono,var);
  SatisfySize(mono,lx);
  SatisfySize(mono,ly);
  SatisfySize(mono,agree);
  SatisfySize(mono,xdata);
  SatisfySize(mono,ydata);
  SatisfySize(mono,mdata);
  int response_ct = 0;
  IMGFOR(mono,x,y)
    {
      float total = 0;
      float total2 = 0;
      PixelMono& pix0 = rmono(x,y);
      for (int k=0; k<SHIFTS; k++)
	{
	  PixelMono& pix1 = mshifts[k](x,y);
	  float v = fabs(pix0-pix1);
	  total += v;
#ifdef USE_LUMINANCE_FILTER
	  total2 += v*v;
#endif
	}
      mean(x,y) = total/SHIFTS;
#ifdef USE_LUMINANCE_FILTER
      var(x,y) = total2/SHIFTS - (total/SHIFTS)*(total/SHIFTS);
#endif
    }
  //printf(">>> %d\n", __LINE__); fflush(stdout);
  int quotient = 16;
  if (use_quotient>0)
    {
      quotient = use_quotient;
    }
  if (1) IMGFOR(mono,x,y)
    {
      float theta = 0;
      float strength = 0;
      float number = 0;
	{
	  float v = mean(x,y);
	  int m = 0;
	  PixelMono& ref = rmono(x,y);
	  for (int k=0; k<SHIFTS; k++)
	    {
	      m *= 2;
	      if (fabs(ref-mshifts[k](x,y))>=v)
		{
		  m += 1;
		}
	    }
	  //assert(orient(0,m) == m);
	  theta = orient(1,m)*2;
	  strength = orient(2,m);
	  number = orient(3,m);
	}
      float mtheta = theta*2;
      //lx(x,y) = cos(theta*2);
      //ly(x,y) = sin(theta*2);


      // moving this experimentally
      float dx = sin(theta);
      float dy = cos(theta);

      if (theta>M_PI)
	{
	  theta -= M_PI;
	}
      if (theta<-M_PI)
	{
	  theta += M_PI;
	}
      theta += M_PI/2;
      //float dx = sin(theta);
      //float dy = cos(theta);
      //xdata(x,y) = dx;
      //ydata(x,y) = dy;
      xdata(x,y) = cos(theta);
      ydata(x,y) = sin(theta);
      float md = number;
      int itheta = (int)(255*(theta/M_PI)+0.5);
      if (itheta>255) itheta = 255;
      if (itheta<0) itheta = 0;
      float lf = -1;
      if (use_luminance_filter>0)
	{
	  lf = use_luminance_filter*use_luminance_filter;
	}
      if (1)
	{
	  //number = log(number+1);
	  number /= quotient;
	  //number /= 2000;
	  number *= strength;
#ifdef USE_LUMINANCE_FILTER
	  // really don't need this
	  //float scale = (var(x,y)>25);
	  if (use_luminance_filter)
	    {
	      float scale = (var(x,y)>lf);
	      if (scale>1) scale = 1;
	      number *= scale;
	    }
#endif
	  if (number>1) number = 1;
	  dx *= number;
	  dy *= number;
	  //lx(x,y) *= number;  //PFADD3
	  //ly(x,y) *= number;  //PFADD3
	}
      mdata(x,y) = number;
      if (view)
	{
	  int ix = (int)fabs(dx*255);
	  int iy = (int)fabs(dy*255);
	  int iz = ((dx*dy)<-0.001);
	  iz = iz*255;

	  PixelRgb pix(ix,iy,iz);
	  dest(x,y) = pix;
	}
    }
}

#endif





void GlutOrientation::Apply(ImageOf<PixelRgb>& src,
				ImageOf<PixelFloat>& delx,
				ImageOf<PixelFloat>& dely,
				ImageOf<PixelFloat>& strength)
{
  ImageOf<PixelRgb> dest;
  Apply(src,dest,delx,dely,strength);
}

