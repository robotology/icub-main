/*
 *  rimage.h
 *
 *  Created by Rhan Dahl on Thu Apr 18 2002.
 *  Fixes: removed bounds checking,
 *         improved corner caching - Ian Fasel, July 24, 2002
 * 	   fixed bounds checking and properly templatized, 
 *	   added RIntegral class   - John Hershey, Sept 5, 2002
 * 
 *  Copyright (c) 2002 Machine Perception Laboratory 
 *  University of California San Diego.
 * 
 * Please read the disclaimer and notes about redistribution 
 * at the end of this file.
 *
 * Benchmark on June 2002 on my 700 MHz iBook:
 *    Jan 10 2003 -- spock.gif = 1.25, dude = 0.3, people = 7.86
 *    Apr 19 2003 -- spock.gif = 0.82, dude = 0.3, people = 5.24 -- about 33% improvement!
 *    Jul 29 2003 -- spock.gif = 0.35, dude = 0.055, people = 3.1 -- 40-60% or more improvement!
 *
 */
#ifndef _RIMAGE_H_
#define _RIMAGE_H_

#include "roi.h"
//#include <iostream>
#ifndef ASSERT
#ifdef _DEBUG
#include <assert.h>
#define ASSERT(x) assert(x)
#else
#define ASSERT(x) 
#endif
#endif

template <class T>
class RImage 
{
  public:
    RImage (const T *array_, const int, const int);
    RImage (const int, const int);
    RImage (const int, const int, T);
    RImage (const RImage &other){
      width = other.width;
      height = other.height;
      numpixels = other.numpixels;
      array = other.array;
    }
    RImage (){};
    inline void setSize(const int width_, const int height_){
      width = width_;
      height = height_;
      numpixels = width * height;
      array = new T[numpixels];
      my_memory = 1;
      // keeps track of use of the image. The destructor will only
      //erase the image if my_memory == 0
      
      //  cout << "RImage<T>:
      //allocated RImage with width = " << width << " height = " <<
      //height << endl;
    }
    inline void deleteArray(){
      delete [] array;
    }
    virtual ~RImage ( );
    virtual T getPixel (const int x, const int y) const;
    virtual T getPixel (const int index) const;
    void setPixel (const int x, const int y, const T &value);
    void setPixel (const int index, const T &value);
    void copyFromRawImage(float *rawImage, int step);
    void print ( ) const;
    void print ( const int m ) const;

   public:

    T *array;
    
    int width;
    int height;
    int numpixels;

    /** my_memory keeps track of use of the image. The destructor will
	only erase the image if my_memory == 0*/
    int my_memory;
      
};

template <class T>
inline RImage<T>::RImage (const T *array_, const int width_, const int height_ )
{
  width = width_;
  height = height_;
  numpixels = width * height;
  array = const_cast<T *>(array_);
  my_memory = 0;
}
  
template <class T>
inline RImage<T>::RImage ( const int width_, const int height_ )
{
	setSize(width_,height_);
}

template <class T>
inline RImage<T>::RImage ( const int width_, const int height_ , const T val)
{
    setSize(width_,height_);
    T *ptr = array;
    for(int y=0; y <height_; y++)
      for(int x=0; x <width_; x++)	
	   *(ptr++) = val; 
}

template <class T>
inline RImage<T>::~RImage ()
{
  if(my_memory){
    //cout << "RImage::~RImage(): deleting" << endl;
    delete [] array;
  }
}

template <class T>
inline T RImage<T>::getPixel ( const int x, const int y ) const
{
  if(x >= 0 && y >= 0 && x < width && y < height)
    return array[width * y + x]; 
  else 
#ifdef WIN32
	return array[0];
#else
    return 0;
#endif
}

template <class T>
inline T RImage<T>::getPixel ( const int index ) const
{
  if((index >= 0) && (index < numpixels ))
    return array[index];
  else
#ifdef WIN32
	return array[0];
#else
    return 0;
#endif
}

template <class T>
inline void RImage<T>::setPixel ( const int x, const int y, const T &value )
{
  if(x >= 0 && y >= 0 && x < width && y < height)
    array[width * y + x] = value;
}

template <class T>
inline void RImage<T>::setPixel ( const int index, const T &value )
{
  if((index >= 0) && (index < numpixels ))
    array[index] = value;
}
template <class T>
inline void RImage<T>::copyFromRawImage(float *rawImage, int step)
{
	int contCol,contRow;
	for( contRow = 0; contRow < height; contRow++, rawImage += step )
		for( contCol = 0; contCol < width; contCol++ )
			array[step*contRow+contCol]=static_cast<float>(rawImage[contCol]);
}
template <class T>
void RImage<T>::print ( ) const 
{
  for(int y = 0; y < this->height; y++) {
    for(int x = 0; x < this->width; x++) {
      std::cout << getPixel(x,y) << " ";
    }
    std::cout << std::endl;
  }
}

template <class T>
void RImage<T>::print ( const int m ) const 
{
  for(int y = 0; y < ((m < height) ? m : height) ; y++) {
    for(int x = 0; x < ((m < width) ? m : width) ; x++) {
      std::cout << getPixel(x,y) << " ";
    }
    std::cout << std::endl;
  }
}

//template class RImage<float>; 

//  RImage<T> returns the wrong values if we scan too far
//  so we override getPixel in derived class and provide constructor.


template <class T>
class RIntegral : public RImage<T>
{
public:
  RIntegral(const int width_, const int height_){
      this->setSize(width_+1,height_+1);
  }

  RIntegral(const RImage<T> &image) {
      setSize(image.width+1,image.height+1);
    integrate(image);
  }

  RIntegral(const RImage<T> &image, ROI &roi){
      setSize(image.width+1,image.height+1);
    integrate(image, roi);
  }
  
  void integrate(const RImage<T> &image){
      T * p = this->array;
      for(int x = 0; x < this->width; ++x) // create top row of zeros
        *p++ = 0;
      for(int y = 0; y < this->height-1; ++y){
        *p++ = 0;                    // create left column of zeros
        for(int x = 0; x < this->width-1; ++x)
          *p++ =
            image.getPixel(x,y) + getPixel(x, y+1) +
            getPixel(x+1, y) - getPixel(x, y);
      }
  }
  void integrate(const RImage<T> &image, ROI roi){
	//cout << "RImage<T>::integrate: " << roi << endl;
    //roi.fitImage(*this);
    //cout << "RImage<T>::integrate: " << roi << endl;

    //cout << "roi: (" << roi.m_min_x << ", " << roi.m_min_y << ", "<< roi.m_max_x << ", "<< roi.m_max_y << ")" << endl;
      T *p = this->array + (this->width*roi.m_min_y + roi.m_min_x);
      T *pxy, *px1y, *pxy1, *q;
      for(int x = roi.m_min_x; x < roi.m_max_x+1; ++x) // create top row of zeros
        *p++ = 0;
      for(int y = roi.m_min_y; y < roi.m_max_y; ++y){
        //cout << "integrating line " << y << " of " << image.height << endl;
        p = this->array + (this->width*(y+1) + roi.m_min_x);
        pxy = this->array + (this->width * y +  roi.m_min_x);
        px1y = this->array + (this->width * y +  roi.m_min_x + 1);
        pxy1 = this->array + (this->width * (y+1) +  roi.m_min_x);
        q = image.array + (image.width * y +  roi.m_min_x);
        *p++ = 0;                    // create left column of zeros
        for(int x = roi.m_min_x; x < roi.m_max_x; ++x)
          *p++ =
            //image.getPixel(x,y) + getPixel(x, y+1) +
            //getPixel(x+1, y) - getPixel(x, y);
            *q++ + *pxy1++ + *px1y++ - *pxy++;
      }
  }
  
  T getPixel (const int x, const int y) const{
    int lx = x, ly = y;
    if(lx < 0 || ly < 0) {
      return 0;
    }
    if(lx >= this->width) {
      ASSERT(false);
      lx = this->width - 1;
    }
    if(ly >= this->height) {
      ASSERT(false);
      ly = this->height - 1;
    }
    return this->array[this->width * ly + lx];
  };

  T getPixel (const int index) const{
    if(index < 0)
      return 0;
    if(index >= this->numpixels)
      return this->array[this->numpixels-1];
    else
      return this->array[index];
  };

	inline void setPixel (const int x, const int y, T val)
	{
		if(x >= 0 && y >= 0 && x < this->width && y < this->height)
			this->array[this->width * y + x] = val;
	};
};

//template class RIntegral<float>; 

#endif


/*
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * 
 *    1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *    3. The name of the author may not be used to endorse or promote products derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */







