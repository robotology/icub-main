// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Authors: 2008 Dario Figueira
 *          2007 Bruno Damas
 *      Institute for Systems and Robotics (IST)
 *      IST, TULisbon, Portugal
 */


#ifndef __IMG_WRAPPER_HPP__B
#define __IMG_WRAPPER_HPP__B

#include <cv.h>


float getFloat( IplImage const *img, int row, int col );
double getDouble( IplImage const *img, int row, int col );

void  setFloat( IplImage *img, float val, int row, int col );
void  setDouble( IplImage *img, float val, int row, int col );

float getFloatROI( IplImage const *img, int row, int col );
double getDoubleROI( IplImage const *img, int row, int col );

void  setFloatROI( IplImage *img, float val, int row, int col );
void  setDoubleROI( IplImage *img, float val, int row, int col );


class UImage
{
	public:
		UImage(IplImage *img = 0);
		~UImage();
		
		void operator = (IplImage *img);
		
		double get2D( int row, int col, int channel = 0 );
		void   set2D( double val, int row, int col, int channel = 0 );
		void   set2D( CvScalar data, int row, int col, int channel = 0 );

		IplImage* imgp;
	private:
		
		void init();

		double (UImage::* get)(int, int, int);
		void (UImage::* set)(CvScalar const &, int, int, int);
		
		double RGBGet8U( int row, int col, int ch );
		double BwGet8U( int row, int col, int ch );
		double RGBGet8S( int row, int col, int ch );
		double BwGet8S( int row, int col, int ch );
		
		double RGBGet16U( int row, int col, int ch );
		double BwGet16U( int row, int col, int ch );
		double RGBGet16S( int row, int col, int ch );
		double BwGet16S( int row, int col, int ch );
		
		double RGBGet32S( int row, int col, int ch );
		double BwGet32S( int row, int col, int ch );
		
		double RGBGet32F( int row, int col, int ch );
		double BwGet32F( int row, int col, int ch );
		double RGBGet64F( int row, int col, int ch );
		double BwGet64F( int row, int col, int ch );
		
		void RGBSet8U( CvScalar const &val, int row, int col, int ch );
		void BwSet8U( CvScalar const &val, int row, int col, int ch );
		void RGBSet8S( CvScalar const &val, int row, int col, int ch );
		void BwSet8S( CvScalar const &val, int row, int col, int ch );
		
		void RGBSet16U( CvScalar const &val, int row, int col, int ch );
		void BwSet16U( CvScalar const &val, int row, int col, int ch );
		void RGBSet16S( CvScalar const &val, int row, int col, int ch );
		void BwSet16S( CvScalar const &val, int row, int col, int ch );
		
		void RGBSet32S( CvScalar const &val, int row, int col, int ch );
		void BwSet32S( CvScalar const &val, int row, int col, int ch );
		
		void RGBSet32F( CvScalar const &val, int row, int col, int ch );
		void BwSet32F( CvScalar const &val, int row, int col, int ch );
		void RGBSet64F( CvScalar const &val, int row, int col, int ch );
		void BwSet64F( CvScalar const &val, int row, int col, int ch );
};



/*  NOT USED  */


template<class T> class Image
{
	public:
		Image(IplImage *img = 0)
		{
			imgp = img;
		}
		~Image()
		{}
		void operator=(IplImage *img)
		{
			imgp = img;
		}
		T* operator[](const int rowIndx)
  		{
    		return (static_cast<T *>(imgp->imageData + rowIndx*imgp->widthStep));
		}
	private:
		IplImage* imgp;
};

typedef struct
{
	unsigned char b,g,r;
}	RgbPixel8U;

typedef struct
{
	char b,g,r;
}	RgbPixel8S;

typedef struct
{
	unsigned int b,g,r;
}	RgbPixel16U;

typedef struct
{
	int b,g,r;
}	RgbPixel16S;

typedef struct
{
	long b,g,r;
}	RgbPixel32S;

typedef struct
{
	float b,g,r;
}	RgbPixel32F;

typedef struct
{
	float b,g,r;
}	RgbPixel64F;

typedef Image<RgbPixel8U>			RgbImage8U;
typedef Image<RgbPixel8S>			RgbImage8S;
typedef Image<RgbPixel16U>			RgbImage16U;
typedef Image<RgbPixel16S>			RgbImage16S;
typedef Image<RgbPixel32S>			RgbImage32S;
typedef Image<RgbPixel32F>			RgbImage32F;
typedef Image<RgbPixel64F>			RgbImage64F;

typedef Image<unsigned char>	BwImage8U;
typedef Image<char>				BwImage8S;
typedef Image<unsigned int>	BwImage16U;
typedef Image<int>				BwImage16S;
typedef Image<long>				BwImage32S;
typedef Image<float>				BwImage32F;
typedef Image<double>			BwImage64F;

#endif
