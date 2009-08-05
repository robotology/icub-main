#include "image_wrapper.hpp"

#include <iostream>
using namespace std;

float getFloat( IplImage const *img, int row, int col )
{
	return ( reinterpret_cast<float *>(img->imageData + row*img->widthStep)[col] );
}

double getDouble( IplImage const *img, int row, int col )
{
	return ( reinterpret_cast<double *>(img->imageData + row*img->widthStep)[col] );
}

void setFloat( IplImage *img, float val, int row, int col )
{
	reinterpret_cast<float *>(img->imageData+row*img->widthStep)[col] = val;
}

void setDouble( IplImage *img, float val, int row, int col )
{
	reinterpret_cast<double *>(img->imageData+row*img->widthStep)[col] = val;
}


float getFloatROI( IplImage const *img, int row, int col )
{
	return ( reinterpret_cast<float *>(img->imageData +
	(row+cvGetImageROI(img).y)*img->widthStep)[col+cvGetImageROI(img).x] );
}

double getDoubleROI( IplImage const *img, int row, int col )
{
	return ( reinterpret_cast<double *>(img->imageData +
	(row+cvGetImageROI(img).y)*img->widthStep)[col+cvGetImageROI(img).x] );
}

void setFloatROI( IplImage *img, float val, int row, int col )
{
	reinterpret_cast<float *>(img->imageData +
	(row+cvGetImageROI(img).y)*img->widthStep)[col+cvGetImageROI(img).x] = val;
}

void setDoubleROI( IplImage *img, float val, int row, int col )
{
	reinterpret_cast<double *>(img->imageData +
	(row+cvGetImageROI(img).y)*img->widthStep)[col+cvGetImageROI(img).x] = val;
}



inline UImage::UImage(IplImage *img)
{
	imgp = img;
	init();
}

inline UImage::~UImage()
{
	if( imgp )
		cvReleaseImage( &imgp );
}

inline void UImage::operator = (IplImage *img)
{
	imgp = img;
	init();
}

void UImage::init()
{
	if( imgp == 0 )
		return;
	else if( imgp->nChannels == 1)		
		switch( imgp->depth )
		{
			case IPL_DEPTH_8U:
				get = &UImage::BwGet8U;
				set = &UImage::BwSet8U;
				break;
			case IPL_DEPTH_8S:
				get = &UImage::BwGet8S;
				set = &UImage::BwSet8S;
				break;
			case IPL_DEPTH_16U:
				get = &UImage::BwGet16U;
				set = &UImage::BwSet16U;
				break;
			case IPL_DEPTH_16S:
				get = &UImage::BwGet16S;
				set = &UImage::BwSet16S;
				break;
			case IPL_DEPTH_32S:
				get = &UImage::BwGet32S;
				set = &UImage::BwSet32S;
				break;
			case IPL_DEPTH_32F:
				get = &UImage::BwGet32F;
				set = &UImage::BwSet32F;
				break;
			case IPL_DEPTH_64F:
				get = &UImage::BwGet32F;
				set = &UImage::BwSet32F;
				break;
			default:
				cerr << "UImage::init(): IplImage.depth not recognized !!" << endl << "Quitting..." << endl;
				exit(1);
		}
	else
		switch( imgp->depth )
		{
			case IPL_DEPTH_8U:
				get = &UImage::RGBGet8U;
				set = &UImage::RGBSet8U;
				break;
			case IPL_DEPTH_8S:
				get = &UImage::RGBGet8S;
				set = &UImage::RGBSet8S;
				break;
			case IPL_DEPTH_16U:
				get = &UImage::RGBGet16U;
				set = &UImage::RGBSet16U;
				break;
			case IPL_DEPTH_16S:
				get = &UImage::RGBGet16S;
				set = &UImage::RGBSet16S;
				break;
			case IPL_DEPTH_32S:
				get = &UImage::RGBGet32S;
				set = &UImage::RGBSet32S;
				break;
			case IPL_DEPTH_32F:
				get = &UImage::RGBGet32F;
				set = &UImage::RGBSet32F;
				break;
			case IPL_DEPTH_64F:
				get = &UImage::RGBGet32F;
				set = &UImage::RGBSet32F;
				break;
			default:
				cerr << "UImage::init(): IplImage.depth not recognized !!" << endl << "Quitting..." << endl;
				exit(1);
		}
}

double UImage::get2D( int row, int col, int channel )
{
	if( imgp == 0 || row >= imgp->height || col >= imgp->width || channel >= imgp->nChannels )
		return cvGet2D(imgp,row,col).val[channel];	// let openCV do the error control

	return (this->*get)(row, col, channel);
}	

void UImage::set2D( CvScalar data, int row, int col, int channel )
{
	if( imgp == 0 || row >= imgp->height || col >= imgp->width || channel >= imgp->nChannels )
		cvSet2D(imgp, row, col, data);	// let openCV do the error control

	(this->*set)(data, row, col, channel);
}

inline void UImage::set2D( double val, int row, int col, int channel )
{
	set2D( cvRealScalar(val), row, col, channel );
}




/*************** GET's ****************/

double UImage::RGBGet8U( int row, int col, int ch )
{
	return static_cast<double>( reinterpret_cast<unsigned char *>(imgp->imageData+row*imgp->widthStep)[col*imgp->nChannels + ch] );
}

double UImage::BwGet8U( int row, int col, int ch )
{
	return static_cast<double>( reinterpret_cast<unsigned char *>(imgp->imageData+row*imgp->widthStep)[col] );
}

double UImage::RGBGet8S( int row, int col, int ch )
{
	return static_cast<double>( reinterpret_cast<char *>(imgp->imageData+row*imgp->widthStep)[col*imgp->nChannels + ch] );
}

double UImage::BwGet8S( int row, int col, int ch )
{
	return static_cast<double>( reinterpret_cast<char *>(imgp->imageData+row*imgp->widthStep)[col] );
}

double UImage::RGBGet16U( int row, int col, int ch )
{
	return static_cast<double>( reinterpret_cast<unsigned int *>(imgp->imageData+row*imgp->widthStep)[col*imgp->nChannels + ch] );
}

double UImage::BwGet16U( int row, int col, int ch )
{
	return static_cast<double>( reinterpret_cast<unsigned int *>(imgp->imageData+row*imgp->widthStep)[col] );
}

double UImage::RGBGet16S( int row, int col, int ch )
{
	return static_cast<double>( reinterpret_cast<int *>(imgp->imageData+row*imgp->widthStep)[col*imgp->nChannels + ch] );
}

double UImage::BwGet16S( int row, int col, int ch )
{
	return static_cast<double>( reinterpret_cast<int *>(imgp->imageData+row*imgp->widthStep)[col] );
}

double UImage::RGBGet32S( int row, int col, int ch )
{
	return static_cast<double>( reinterpret_cast<long *>(imgp->imageData+row*imgp->widthStep)[col*imgp->nChannels + ch] );
}

double UImage::BwGet32S( int row, int col, int ch )
{
	return static_cast<double>( reinterpret_cast<long *>(imgp->imageData+row*imgp->widthStep)[col] );
}

double UImage::RGBGet32F( int row, int col, int ch )
{
	return static_cast<double>( reinterpret_cast<float *>(imgp->imageData+row*imgp->widthStep)[col*imgp->nChannels + ch] );
}

double UImage::BwGet32F( int row, int col, int ch )
{
	return static_cast<double>( reinterpret_cast<float *>(imgp->imageData+row*imgp->widthStep)[col] );
}

double UImage::RGBGet64F( int row, int col, int ch )
{
	return static_cast<double>( reinterpret_cast<double *>(imgp->imageData+row*imgp->widthStep)[col*imgp->nChannels + ch] );
}

double UImage::BwGet64F( int row, int col, int ch )
{
	return static_cast<double>( reinterpret_cast<double *>(imgp->imageData+row*imgp->widthStep)[col] );
}



/*************** SET's ****************/

void UImage::RGBSet8U( CvScalar const &data, int row, int col, int ch )
{
	for( int i = 0; i < ch; i++ )
		reinterpret_cast<unsigned char *>(imgp->imageData+row*imgp->widthStep)[col*imgp->nChannels + ch] = 
			static_cast<unsigned char>( data.val[ch] );
}

void UImage::BwSet8U( CvScalar const &data, int row, int col, int ch )
{
	reinterpret_cast<unsigned char *>(imgp->imageData+row*imgp->widthStep)[col] = 
		static_cast<unsigned char>( data.val[0] );
}

void UImage::RGBSet8S( CvScalar const &data, int row, int col, int ch )
{
	for( int i = 0; i < ch; i++ )
		reinterpret_cast<char *>(imgp->imageData+row*imgp->widthStep)[col*imgp->nChannels + ch] = 
			static_cast<char>( data.val[ch] );
}

void UImage::BwSet8S( CvScalar const &data, int row, int col, int ch )
{
	reinterpret_cast<char *>(imgp->imageData+row*imgp->widthStep)[col] = 
		static_cast<char>( data.val[0] );
}

void UImage::RGBSet16U( CvScalar const &data, int row, int col, int ch )
{
	for( int i = 0; i < ch; i++ )
		reinterpret_cast<unsigned int *>(imgp->imageData+row*imgp->widthStep)[col*imgp->nChannels + ch] = 
			static_cast<unsigned int>( data.val[ch] );
}

void UImage::BwSet16U( CvScalar const &data, int row, int col, int ch )
{
	reinterpret_cast<unsigned int *>(imgp->imageData+row*imgp->widthStep)[col] = 
		static_cast<unsigned int>( data.val[0] );
}

void UImage::RGBSet16S( CvScalar const &data, int row, int col, int ch )
{
	for( int i = 0; i < ch; i++ )
		reinterpret_cast<int *>(imgp->imageData+row*imgp->widthStep)[col*imgp->nChannels + ch] = 
			static_cast<int>( data.val[ch] );
}

void UImage::BwSet16S( CvScalar const &data, int row, int col, int ch )
{
	reinterpret_cast<int *>(imgp->imageData+row*imgp->widthStep)[col] = 
		static_cast<int>( data.val[0] );
}

void UImage::RGBSet32S( CvScalar const &data, int row, int col, int ch )
{
	for( int i = 0; i < ch; i++ )
		reinterpret_cast<long *>(imgp->imageData+row*imgp->widthStep)[col*imgp->nChannels + ch] = 
			static_cast<long>( data.val[ch] );
}

void UImage::BwSet32S( CvScalar const &data, int row, int col, int ch )
{
	reinterpret_cast<long *>(imgp->imageData+row*imgp->widthStep)[col] = 
		static_cast<long>( data.val[0] );
}

void UImage::RGBSet32F( CvScalar const &data, int row, int col, int ch )
{
	for( int i = 0; i < ch; i++ )
		reinterpret_cast<float *>(imgp->imageData+row*imgp->widthStep)[col*imgp->nChannels + ch] = 
			static_cast<float>( data.val[ch] );
}

void UImage::BwSet32F( CvScalar const &data, int row, int col, int ch )
{
	reinterpret_cast<float *>(imgp->imageData+row*imgp->widthStep)[col] = 
		static_cast<float>( data.val[0] );
}

void UImage::RGBSet64F( CvScalar const &data, int row, int col, int ch )
{
	for( int i = 0; i < ch; i++ )
		reinterpret_cast<double *>(imgp->imageData+row*imgp->widthStep)[col*imgp->nChannels + ch] = 
			static_cast<double>( data.val[ch] );
}

void UImage::BwSet64F( CvScalar const &data, int row, int col, int ch )
{
	reinterpret_cast<double *>(imgp->imageData+row*imgp->widthStep)[col] = 
		static_cast<double>( data.val[0] );
}

