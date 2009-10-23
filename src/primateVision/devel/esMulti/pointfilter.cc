
/*esLog project*/

///////////////////////////////////////////////////////////////////////////////////////////
//                                                                                       // 
// 3D representation of image on half sphere surface, with logaritmically sampled pixels //
//                                                                                       //
///////////////////////////////////////////////////////////////////////////////////////////


#include "pointfilter.h"


pointfilter::pointfilter(int w, int h, float sx, float sy)
{
  width = w;
  height = h;
  sigma_x = sx;
  sigma_y = sy;
  
  set_profile();
}


pointfilter::~pointfilter()
{
  for (int i = 0; i < height; i++)
    {
      delete[] profile[i];
    }

  delete[] profile;
}


unsigned char pointfilter::pixel_filter(unsigned char **src, int xc, int yc)
{
  float val = 0;
  float correction_factor = 0;

  for(int h = 0; h < height; h++)
    {
      for(int w = 0; w < width; w++)
	{
	  val += profile[h][w]*(float)(src[yc-(height-1)/2+h][xc-(width-1)/2+w]);
	  correction_factor += profile[h][w];
	}
    }

  return (unsigned char)(val/correction_factor);
}


void pointfilter::set_profile()
{
  profile = new float*[height];
  
  for(int i = 0; i < height; i++)
    {
      profile[i] = new float[width];
    }

  float norm = 1/(2*PI*sigma_x*sigma_y);
  float ee = 0;

  for(int h = 0; h < height; h++)
    {
      for(int w = 0; w < width; w++)
	{
	  ee = -(pow((float)w - (float)(width/2),2)/pow(sigma_x,2) + pow((float)h - (float)(height/2),2)/pow(sigma_y,2))/2;
	  profile[h][w] = norm*exp(ee);
	}
    }

}
