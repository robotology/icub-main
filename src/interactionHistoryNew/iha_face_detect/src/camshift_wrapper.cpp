// modification of existing code by Frank Broz, 2009
// camshift_wrapper.c - by Robin Hewitt, 2007
// http://www.cognotics.com/opencv/downloads/camshift_wrapper
// This is free software. See License.txt, in the download
// package, for details.
//

#include <cv.h>
#include <stdio.h>
#include <iCub/iha/camshift_wrapper.h>


CamshiftWrapper::CamshiftWrapper(){
  nHistBins = 30;
  vmin = 65;
  vmax = 256;
  smin = 55;
  nFrames = 0;
  initialized = false;
  rangesArr[0]=0;
  rangesArr[1]=180;

}


CamshiftWrapper::~CamshiftWrapper(){


}


//////////////////////////////////
// createTracker()
//
int CamshiftWrapper::createTracker(const IplImage * pImg)
{

 // Allocate the main data structures ahead of time
  float * pRanges = rangesArr;
  //IhaDebug::pmesg(DBGL_DEBUG1,"In createTracker %d %d %d \n", pImg->width, pImg->height, cvSize(pImg->width, pImg->height));
  IhaDebug::pmesg(DBGL_DEBUG1,"In createTracker \n");
  pHSVImg  = cvCreateImage( cvGetSize(pImg),  IPL_DEPTH_8U, 3 );
  IhaDebug::pmesg(DBGL_DEBUG1,"1 images allocated \n");
  pHueImg  = cvCreateImage( cvGetSize(pImg),  IPL_DEPTH_8U, 1 );
  IhaDebug::pmesg(DBGL_DEBUG1,"2 images allocated \n");
  pMask    = cvCreateImage( cvGetSize(pImg),  IPL_DEPTH_8U, 1 );
  IhaDebug::pmesg(DBGL_DEBUG1,"3 images allocated \n");
  pProbImg = cvCreateImage( cvGetSize(pImg),  IPL_DEPTH_8U, 1 );
  IhaDebug::pmesg(DBGL_DEBUG1,"All images allocated \n");

  pHist = cvCreateHist( 1, &nHistBins, CV_HIST_ARRAY, &pRanges, 1 );
  IhaDebug::pmesg(DBGL_DEBUG1,"histograms created \n");
  
  
  initialized = true;
  return 1;
}


//////////////////////////////////
// releaseTracker()
//
void CamshiftWrapper::releaseTracker()
{
	// Release all tracker resources
	cvReleaseImage( &pHSVImg );
	cvReleaseImage( &pHueImg );
	cvReleaseImage( &pMask );
	cvReleaseImage( &pProbImg );

	cvReleaseHist( &pHist );
	initialized = false;
}


//////////////////////////////////
// startTracking()
//
void CamshiftWrapper::startTracking(IplImage * pImg, CvRect * pFaceRect)
{
	float maxVal = 0.f;

	// Make sure internal data structures have been allocated
	if( !pHist ) createTracker(pImg);

	// Create a new hue image
	updateHueImage(pImg);

	// Create a histogram representation for the face
    cvSetImageROI( pHueImg, *pFaceRect );
    cvSetImageROI( pMask,   *pFaceRect );
    cvCalcHist( &pHueImg, pHist, 0, pMask );
    cvGetMinMaxHistValue( pHist, 0, &maxVal, 0, 0 );
    cvConvertScale( pHist->bins, pHist->bins, maxVal? 255.0/maxVal : 0, 0 );
    cvResetImageROI( pHueImg );
    cvResetImageROI( pMask );

	// Store the previous face location
	prevFaceRect = *pFaceRect;
}


//////////////////////////////////
// track()
//
CvRect CamshiftWrapper::track(IplImage * pImg)
{
	CvConnectedComp components;

	// Create a new hue image
	updateHueImage(pImg);

	// Create a probability image based on the face histogram
	cvCalcBackProject( &pHueImg, pProbImg, pHist );
    cvAnd( pProbImg, pMask, pProbImg, 0 );

	// Use CamShift to find the center of the new face probability
    cvCamShift( pProbImg, prevFaceRect,
                cvTermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ),
                &components, &faceBox );

	// Update face location and angle
    prevFaceRect = components.rect;
	faceBox.angle = -faceBox.angle;

	return components.rect;
}


//////////////////////////////////
// updateHueImage()
//
void CamshiftWrapper::updateHueImage(const IplImage * pImg)
{
	// Convert to HSV color model
	cvCvtColor( pImg, pHSVImg, CV_BGR2HSV );

	// Mask out-of-range values
	cvInRangeS( pHSVImg, cvScalar(0, smin, MIN(vmin,vmax), 0),
	            cvScalar(180, 256, MAX(vmin,vmax) ,0), pMask );

	// Extract the hue channel
	cvSplit( pHSVImg, pHueImg, 0, 0, 0 );
}


//////////////////////////////////
// setVmin()
//
void CamshiftWrapper::setVmin(int _vmin)
{ vmin = _vmin; }


//////////////////////////////////
// setSmin()
//
void CamshiftWrapper::setSmin(int _smin)
{ smin = _smin; }


