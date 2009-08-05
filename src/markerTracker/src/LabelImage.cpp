#include <assert.h>
#include <stdio.h>

#include "LabelImage.h"

#ifndef __LINUX__
#define for if (1) for
#endif

using namespace yarp::sig;

// this is deliberately inefficient for various reasons,
// there are much faster ways of doing this.

int LabelImage::Apply(ImageOf<PixelMono>& src, 
		      ImageOf<PixelMono>& dest)
{
  int IMG_H = src.width();  // swapped role of H and W
  int IMG_W = src.height();
  int nalloc_len = IMG_H*IMG_W*4;
  if (nalloc_len>alloc_len)
    {
      Clean();
      alloc_len = nalloc_len;
      xstack = new int [alloc_len];
      ystack = new int [alloc_len];
    }
  assert(xstack!=NULL&&ystack!=NULL);
  int i, j;
  int ii, jj;
  int id = 1;
  int top = 0;
  int maxCount = 0;
  int maxCountId = 0;
  int count;

  for (i=0; i<IMG_H; i++)
    {
      for (j=0; j<IMG_W; j++)
	{
	  dest(i,j) = (src(i,j)==0);
	}
    }
  for (i=0; i<IMG_H; i++)
    {
      for (j=0; j<IMG_W; j++)
	{
	  if (dest(i,j) == 0)
	    {
	      id++;
	      if (id>255)
		{
		  id = 2;
		}
	      count = 0;
	      top = 1;
	      ystack[0] = i;
	      xstack[0] = j;
	      while (top>0)
		{
		  top--;
		  ii = ystack[top];
		  jj = xstack[top];
		  if (dest(ii,jj) == 0)
		    {
		      dest(ii,jj) = id;
		      count++;
		      if (ii>0)
			{
			  if (dest(ii-1,jj) == 0)
			    {
			      ystack[top] = ii-1;
			      xstack[top] = jj;
			      top++;
			    }
			}
		      if (ii+1<IMG_H)
			{
			  if (dest(ii+1,jj) == 0)
			    {
			      ystack[top] = ii+1;
			      xstack[top] = jj;
			      top++;
			    }
			}
		      if (jj>0)
			{
			  if (dest(ii,jj-1) == 0)
			    {
			      ystack[top] = ii;
			      xstack[top] = jj-1;
			      top++;
			    }
			}
		      if (jj+1<IMG_W)
			{
			  if (dest(ii,jj+1) == 0)
			    {
			      ystack[top] = ii;
			      xstack[top] = jj+1;
			      top++;
			    }
			}
		    }
		}
	      if (count>maxCount)
		{
		  maxCount = count;
		  maxCountId = id;
		}
	    }
	}
    }
  bestId = maxCountId;
  return maxCountId;
}


int LabelImage::Apply(ImageOf<PixelInt>& src, 
			  ImageOf<PixelInt>& dest)
{
  int IMG_H = src.width();  // swapped role of H and W
  int IMG_W = src.height();
  int nalloc_len = IMG_H*IMG_W*4;
  if (nalloc_len>alloc_len)
    {
      Clean();
      alloc_len = nalloc_len;
      xstack = new int [alloc_len];
      ystack = new int [alloc_len];
    }
  assert(xstack!=NULL&&ystack!=NULL);
  int i, j;
  int ii, jj;
  int id = 1;
  int top = 0;
  int maxCount = 0;
  int maxCountId = 0;
  int count;

  for (i=0; i<IMG_H; i++)
    {
      for (j=0; j<IMG_W; j++)
	{
	  dest(i,j) = (src(i,j)==0);
	}
    }
  for (i=0; i<IMG_H; i++)
    {
      for (j=0; j<IMG_W; j++)
	{
	  if (dest(i,j) == 0)
	    {
	      id++;
	      count = 0;
	      top = 1;
	      ystack[0] = i;
	      xstack[0] = j;
	      while (top>0)
		{
		  top--;
		  ii = ystack[top];
		  jj = xstack[top];
		  if (dest(ii,jj) == 0)
		    {
		      dest(ii,jj) = id;
		      count++;
		      if (ii>0)
			{
			  if (dest(ii-1,jj) == 0)
			    {
			      ystack[top] = ii-1;
			      xstack[top] = jj;
			      top++;
			    }
			}
		      if (ii+1<IMG_H)
			{
			  if (dest(ii+1,jj) == 0)
			    {
			      ystack[top] = ii+1;
			      xstack[top] = jj;
			      top++;
			    }
			}
		      if (jj>0)
			{
			  if (dest(ii,jj-1) == 0)
			    {
			      ystack[top] = ii;
			      xstack[top] = jj-1;
			      top++;
			    }
			}
		      if (jj+1<IMG_W)
			{
			  if (dest(ii,jj+1) == 0)
			    {
			      ystack[top] = ii;
			      xstack[top] = jj+1;
			      top++;
			    }
			}
		    }
		}
	      if (count>maxCount)
		{
		  maxCount = count;
		  maxCountId = id;
		}
	    }
	}
    }
  bestId = maxCountId;
  return maxCountId;
}



int LabelImage::ApplySimilarity(ImageOf<PixelInt>& src, 
				    ImageOf<PixelInt>& dest)
{
  int IMG_H = src.width();  // swapped role of H and W
  int IMG_W = src.height();
  int nalloc_len = IMG_H*IMG_W*4;
  if (nalloc_len>alloc_len)
    {
      Clean();
      alloc_len = nalloc_len;
      xstack = new int [alloc_len];
      ystack = new int [alloc_len];
    }
  assert(xstack!=NULL&&ystack!=NULL);
  int i, j;
  int ii, jj;
  int id = 1;
  int top = 0;
  int maxCount = 0;
  int maxCountId = 0;
  int count;

  for (i=0; i<IMG_H; i++)
    {
      for (j=0; j<IMG_W; j++)
	{
	  dest(i,j) = 0;
	}
    }
  for (i=0; i<IMG_H; i++)
    {
      for (j=0; j<IMG_W; j++)
	{
	  if (dest(i,j) == 0)
	    {
	      int ref = src(i,j);
	      id++;
	      //if (id>255)
	      //{
	      //  id = 2;
	      //}
	      count = 0;
	      top = 1;
	      ystack[0] = i;
	      xstack[0] = j;
	      while (top>0)
		{
		  top--;
		  ii = ystack[top];
		  jj = xstack[top];
		  if (dest(ii,jj) == 0 && src(ii,jj)==ref)
		    {
		      dest(ii,jj) = id;
		      count++;
		      if (ii>0)
			{
			  if (dest(ii-1,jj) == 0 && src(ii-1,jj)==ref)
			    {
			      ystack[top] = ii-1;
			      xstack[top] = jj;
			      top++;
			    }
			}
		      if (ii+1<IMG_H)
			{
			  if (dest(ii+1,jj) == 0 && src(ii+1,jj)==ref)
			    {
			      ystack[top] = ii+1;
			      xstack[top] = jj;
			      top++;
			    }
			}
		      if (jj>0)
			{
			  if (dest(ii,jj-1) == 0 && src(ii,jj-1)==ref)
			    {
			      ystack[top] = ii;
			      xstack[top] = jj-1;
			      top++;
			    }
			}
		      if (jj+1<IMG_W)
			{
			  if (dest(ii,jj+1) == 0 && src(ii,jj+1)==ref)
			    {
			      ystack[top] = ii;
			      xstack[top] = jj+1;
			      top++;
			    }
			}
		    }
		}
	      if (count>maxCount)
		{
		  maxCount = count;
		  maxCountId = id;
		}
	    }
	}
    }
  bestId = maxCountId;
  return maxCountId;
}




// overload IsCompatible to use this
int LabelImage::Apply(int x, int y, ImageOf<PixelInt>& dest)
{
  int IMG_H = x;
  int IMG_W = y;
  int nalloc_len = IMG_H*IMG_W*4;
  if (nalloc_len>alloc_len)
    {
      Clean();
      alloc_len = nalloc_len;
      xstack = new int [alloc_len];
      ystack = new int [alloc_len];
    }
  assert(xstack!=NULL&&ystack!=NULL);
  int i, j;
  int ii, jj;
  int id = 1;
  int top = 0;
  int maxCount = 0;
  int maxCountId = 0;
  int count;
  int stop = 0;

  for (i=0; i<IMG_H; i++)
    {
      for (j=0; j<IMG_W; j++)
	{
	  dest(i,j) = 0;
	}
    }
  for (i=0; i<IMG_H; i++)
    {
      for (j=0; j<IMG_W; j++)
	{
	  if (dest(i,j) == 0)
	    {
	      //int ref = src(i,j);
	      if (stop)
		{
		  Notify(id,count,1);
		}
	      id++;
	      //printf(":: %d %d (%d)\n", i, j, id);
	      //if (id>255)
	      //{
	      //  id = 2;
	      //}
	      count = 0;
	      Notify(id,count,0);
	      stop = 1;
	      top = 1;
	      ystack[0] = i;
	      xstack[0] = j;
	      while (top>0)
		{
		  top--;
		  ii = ystack[top];
		  jj = xstack[top];
		  if (dest(ii,jj) == 0)
		    if (IsCompatible(i,j,ii,jj))
		    {
		      dest(ii,jj) = id;
		      Notify(ii,jj);
		      count++;
		      if (ii>0)
			{
			  if (dest(ii-1,jj) == 0)
			    if (IsCompatible(i,j,ii-1,jj))
			    {
			      ystack[top] = ii-1;
			      xstack[top] = jj;
			      top++;
			    }
			}
		      if (ii+1<IMG_H)
			{
			  if (dest(ii+1,jj) == 0) 
			    if (IsCompatible(i,j,ii+1,jj))
			    {
			      ystack[top] = ii+1;
			      xstack[top] = jj;
			      top++;
			    }
			}
		      if (jj>0)
			{
			  if (dest(ii,jj-1) == 0)
			    if (IsCompatible(i,j,ii,jj-1))
			    {
			      ystack[top] = ii;
			      xstack[top] = jj-1;
			      top++;
			    }
			}
		      if (jj+1<IMG_W)
			{
			  if (dest(ii,jj+1) == 0)
			    if (IsCompatible(i,j,ii,jj+1))
			    {
			      ystack[top] = ii;
			      xstack[top] = jj+1;
			      top++;
			    }
			}
		    }
		}
	      if (count>maxCount)
		{
		  maxCount = count;
		  maxCountId = id;
		}
	    }
	}
    }
  bestId = maxCountId;
  return maxCountId;
}


