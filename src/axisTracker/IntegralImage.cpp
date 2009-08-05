
#include <math.h>
#include <stdlib.h>

#include <yarp/sig/Image.h>
#include <yarp/sig/ImageDraw.h>
#include <yarp/sig/IntegralImage.h>

#define SatisfySize(x,y) (y).resize(x)

using namespace yarp::sig;

// Sweep summation horizontally and vertically
static void GenerateSum(ImageOf<PixelFloat>& src, ImageOf<PixelFloat>& dest)
{
  int i, j;
  float total, v;
  int h = src.height();
  int w = src.width();
 
  for (i=0; i<h; i++)
    {
      total = 0;
      for (j=0; j<w; j++)
		{
			v = src(j,i);
			total += v;
			dest(j,i) = total;
		}
    }

  for (j=0; j<w; j++)
    {
      total = 0;
      for (i=0; i<h; i++)
		{
		  total += dest(j,i);
		  dest(j,i) = total;
		}
    }
}

static void GenerateSum2(ImageOf<PixelFloat>& src, ImageOf<PixelFloat>& dest, 
		  ImageOf<PixelFloat>& dest2)
{
  int i, j;
  float total, total2, v;
  int h = src.height();
  int w = src.width();

  for (i=0; i<h; i++)
    {
      total = 0;
      total2 = 0;
      for (j=0; j<w; j++)
	{
	  v = src(j,i);
	  total += v;
	  total2 += v*v;
	  dest(j,i) = total;
	  dest2(j,i) = total2;
	}
    }

  for (j=0; j<w; j++)
    {
      total = 0;
      total2 = 0;
      for (i=0; i<h; i++)
	{
	  total += dest(j,i);
	  total2 += dest2(j,i);
	  dest(j,i) = total;
	  dest2(j,i) = total2;
	}
    }
}

/* Optimized, deals with src==dest and dx or dy == 0 case */

template <class T>
static void Offset(ImageOf<T>& src, ImageOf<T>& dest, 
	    int dx, int dy, int clean, const T& zero)
{
  int w, h, i, j;
  T *p, *p2;
  int H = src.height();
  int W = src.width();
  w = W-abs(dx);
  h = H-abs(dy);
  if (dx<0)
    {
      if (dy>0)
	{
	  for (i=dy; i<H; i++)
	    {
	      p = &src(-dx,i);
	      p2 = &dest(0,i-dy);
	      for (j=-dx; j<W; j++)
		{
		  *p2 = *p;
		  p++;
		  p2++;
		}
	    }
	}
      else
	{
	  for (i=H+dy-1; i>=0; i--)
	    {
	      p = &src(-dx,i);
	      p2 = &dest(0,i-dy);
	      for (j=-dx; j<W; j++)
		{
		  *p2 = *p;
		  p++;
		  p2++;
		}
	    }
	}
    }
  else
    {
      if (dy>0)
	{
	  for (i=dy; i<H; i++)
	    {
	      p = &src(W-dx-1,i);
	      p2 = &dest(W-1,i-dy);
	      for (j=dx; j<W; j++)
		{
		  *p2 = *p;
		  p--;
		  p2--;
		}
	    }
	}
      else
	{
	  for (i=H+dy-1; i>=0; i--)
	    {
	      p = &src(W-dx-1,i);
	      p2 = &dest(W-1,i-dy);
	      for (j=dx; j<W; j++)
		{
		  *p2 = *p;
		  p--;
		  p2--;
		}
	    }
	}      
    }
  if (clean)
    {
      if (dx>0)
	{
	  for (int i=0;i<dx;i++)
	    {
	      for (int j=0; j<H; j++)
		{
		  dest(i,j) = zero;
		}
	    }
	}
      else
	{
	  for (int i=W+dx; i<W; i++)
	    {
	      for (int j=0; j<H; j++)
		{
		  dest(i,j) = zero;
		}
	    }
	}
      if (dy<0)
	{
	  for (int j=0;j<-dy;j++)
	    {
	      for (int i=0; i<W; i++)
		{
		  dest(i,j) = zero;
		}
	    }
	}
      else
	{
	  for (int j=H-dy;j<H;j++)
	    {
	      for (int i=0; i<W; i++)
		{
		  dest(i,j) = zero;
		}
	    }
	}
    }
}

void IntegralImage::Offset(ImageOf<PixelFloat>& src, 
			       ImageOf<PixelFloat>& dest, 
			       int dx, int dy, int clean, float nulval)
{
  SatisfySize(src,dest);	
  ::Offset(src,dest,dx,dy,clean,(PixelFloat)nulval);
}


void IntegralImage::Offset(ImageOf<PixelMono>& src, 
			       ImageOf<PixelMono>& dest, 
			       int dx, int dy, int clean, int nulval)
{
  SatisfySize(src,dest);	
  ::Offset(src,dest,dx,dy,clean,(PixelMono)nulval);
}


void IntegralImage::Offset(ImageOf<PixelRgb>& src, 
			       ImageOf<PixelRgb>& dest, 
			       int dx, int dy, int clean)
{
  SatisfySize(src,dest);
  PixelRgb zero(0,0,0);
  ::Offset(src,dest,dx,dy,clean,zero);
}


void AddOffset(ImageOf<PixelFloat>& src, ImageOf<PixelFloat>& dest, 
	       int dx, int dy)
{
  int w, h, i, j;
  float *p, *p2;
  int H = src.height();
  int W = src.width();
  w = W-abs(dx);
  h = H-abs(dy);
  if (dx<0)
    {
      if (dy>0)
	{
	  for (i=dy; i<H; i++)
	    {
	      p = &src(-dx,i);
	      p2 = &dest(0,i-dy);
	      for (j=-dx; j<W; j++)
		{
		  *p2 += *p;
		  p++;
		  p2++;
		}
	    }
	}
      else
	{
	  for (i=H+dy-1; i>=0; i--)
	    {
	      p = &src(-dx,i);
	      p2 = &dest(0,i-dy);
	      for (j=-dx; j<W; j++)
		{
		  *p2 += *p;
		  p++;
		  p2++;
		}
	    }
	}
    }
  else
    {
      if (dy>0)
	{
	  for (i=dy; i<H; i++)
	    {
	      p = &src(W-dx-1,i);
	      p2 = &dest(W-1,i-dy);
	      for (j=dx; j<W; j++)
		{
		  *p2 += *p;
		  p--;
		  p2--;
		}
	    }
	}
      else
	{
	  for (i=H+dy-1; i>=0; i--)
	    {
	      p = &src(W-dx-1,i);
	      p2 = &dest(W-1,i-dy);
	      for (j=dx; j<W; j++)
		{
		  *p2 += *p;
		  p--;
		  p2--;
		}
	    }
	}      
    }
}

void SubOffset(ImageOf<PixelFloat>& src, ImageOf<PixelFloat>& dest, 
	       int dx, int dy)
{
  int w, h, i, j;
  float *p, *p2;
  int H = src.height();
  int W = src.width();
  w = W-abs(dx);
  h = H-abs(dy);
  if (dx<0)
    {
      if (dy>0)
	{
	  for (i=dy; i<H; i++)
	    {
	      p = &src(-dx,i);
	      p2 = &dest(0,i-dy);
	      for (j=-dx; j<W; j++)
		{
		  *p2 -= *p;
		  p++;
		  p2++;
		}
	    }
	}
      else
	{
	  for (i=H+dy-1; i>=0; i--)
	    {
	      p = &src(-dx,i);
	      p2 = &dest(0,i-dy);
	      for (j=-dx; j<W; j++)
		{
		  *p2 -= *p;
		  p++;
		  p2++;
		}
	    }
	}
    }
  else
    {
      if (dy>0)
	{
	  for (i=dy; i<H; i++)
	    {
	      p = &src(W-dx-1,i);
	      p2 = &dest(W-1,i-dy);
	      for (j=dx; j<W; j++)
		{
		  *p2 -= *p;
		  p--;
		  p2--;
		}
	    }
	}
      else
	{
	  for (i=H+dy-1; i>=0; i--)
	    {
	      p = &src(W-dx-1,i);
	      p2 = &dest(W-1,i-dy);
	      for (j=dx; j<W; j++)
		{
		  *p2 -= *p;
		  p--;
		  p2--;
		}
	    }
	}      
    }
}

void MaskOffset(ImageOf<PixelFloat>& dest, int delta)
{
  int i, j;
  int H = dest.height();
  int W = dest.width();
  for (i=0; i<delta; i++)
    {
      for (j=0; j<W; j++)
	{
	  dest(j,i) = 0;
	  dest(j,H-i-1) = 0;
	}
    }
  for (j=0; j<delta; j++)
    {
      for (i=0; i<H; i++)
	{
	  dest(j,i) = 0;
	  dest(W-j-1,i) = 0;
	}
    }
}

template <class T>
void MaskOffsetT(ImageOf<T>& dest, int delta, const T& zero)
{
  int i, j;
  int H = dest.height();
  int W = dest.width();
  for (i=0; i<delta; i++)
    {
      for (j=0; j<W; j++)
	{
	  dest(j,i) = zero;
	  dest(j,H-i-1) = zero;
	}
    }
  for (j=0; j<delta; j++)
    {
      for (i=0; i<H; i++)
	{
	  dest(j,i) = zero;
	  dest(W-j-1,i) = zero;
	}
    }
}


// Should optimize this
void AssignDelta(ImageOf<PixelFloat>& src, ImageOf<PixelFloat>& dest, 
		 int dx, int dy)
{
  int i, j;
  int H = dest.height();
  int W = dest.width();
  for (i=0; i<H; i++)
    {
      for (j=0; j<W; j++)
	{
	  dest(j,i) = 0;
	}
    }
  Offset(src,dest,dx,dy,0,(PixelFloat)0);
}

void TotalFromSum(ImageOf<PixelFloat>& src, ImageOf<PixelFloat>& dest, 
		  int delta)
{
  AssignDelta(src,dest,-delta,delta);
  AddOffset(src,dest,delta+1,-delta-1);
  SubOffset(src,dest,-delta,-delta-1);
  SubOffset(src,dest,delta+1,delta);
  MaskOffset(dest,delta);
}



void GenerateVariance(ImageOf<PixelFloat>& src, ImageOf<PixelFloat>& dest, 
		      int delta)
{
  float factor = (delta*2+1)*(delta*2+1);
  static ImageOf<PixelFloat> sum1;
  static ImageOf<PixelFloat> sum2;
  static ImageOf<PixelFloat> tot1;
  static ImageOf<PixelFloat> tot2;
  SatisfySize(src,sum1);
  SatisfySize(src,sum2);
  SatisfySize(src,tot1);
  SatisfySize(src,tot2);
  GenerateSum2(src,sum1,sum2);
  TotalFromSum(sum1,tot1,delta);
  TotalFromSum(sum2,tot2,delta);
  IMGFOR(dest,j,i)	
	{
	  dest(j,i) = (tot2(j,i)-tot1(j,i)*(tot1(j,i)/factor))/factor;
	}
  //  MakeLog(dest,dest,1);
  //  ApplyThreshold(dest,dest,3000);
}


void GenerateMean(ImageOf<PixelFloat>& src, ImageOf<PixelFloat>& dest, int delta)
{
  float factor = (delta*2+1)*(delta*2+1);
  static ImageOf<PixelFloat> sum1;
  static ImageOf<PixelFloat> sum2;
  static ImageOf<PixelFloat> tot1;
  static ImageOf<PixelFloat> tot2;
  SatisfySize(src,sum1);
  SatisfySize(src,sum2);
  SatisfySize(src,tot1);
  SatisfySize(src,tot2);
  GenerateSum(src,sum1);
  TotalFromSum(sum1,tot1,delta);
  IMGFOR(dest,j,i)
  {
	  dest(j,i) = tot1(j,i)/factor;
  }
  //  MakeLog(dest,dest,1);
  //  ApplyThreshold(dest,dest,3000);
}



void IntegralImage::GetMean(ImageOf<PixelFloat>& src, 
				ImageOf<PixelFloat>& dest, 
				int size)
{
  SatisfySize(src,dest);
  GenerateMean(src,dest,size);
}


void IntegralImage::GetVariance(ImageOf<PixelFloat>& src, 
				    ImageOf<PixelFloat>& dest, 
				    int size)
{
  SatisfySize(src,dest);
  GenerateVariance(src,dest,size);
}


void IntegralImage::MaskOffset(ImageOf<PixelFloat>& dest, int delta, float zero)
{
  ::MaskOffsetT(dest,delta,zero);
}

void IntegralImage::MaskOffset(ImageOf<PixelMono>& dest, int delta, int zero)
{
  ::MaskOffsetT(dest,delta,(PixelMono)zero);
}

void IntegralImage::MaskOffset(ImageOf<PixelRgb>& dest, int delta)
{
  PixelRgb zero(0,0,0);
  ::MaskOffsetT(dest,delta,zero);
}

