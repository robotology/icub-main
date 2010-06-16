/**
*
@ingroup icub_module
\defgroup icub_findMovements findMovements

This module finds movements in both cameras and sends out the movement positions


\section lib_sec Libraries
YARP, OPEN_CV
 
\section portsa_sec Ports Accessed
/findmovements/image/left/in
/findmovements/image/right/in

\section portsc_sec Ports Created
/findmovements/left/out
/findmovements/right/out


\section tested_os_sec Tested OS
Linux 

\section example_sec Example Instantiation of the Module
findMovements

\author Zenon Mathews

Copyright (C) 2010 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at findMovements/findMovements.cpp.
**/





#include <stdio.h>

// Get all OS and signal processing YARP classes

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

#include <cv.h>
#include <cvaux.h>
#include <highgui.h>

#include "torsodet.h"

#include <vector>
#include <math.h>

#include <time.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

using namespace std;


struct Point{
  int x;
  int y;
};

vector<Point> leftPoints;
vector<Point> rightPoints;

bool left = true;


//----------------------------------------------------------
// various tracking parameters (in seconds)
const double MHI_DURATION = 0.5;//1;
const double MAX_TIME_DELTA = 0.2; //0.5;
const double MIN_TIME_DELTA = 0.01;//0.05;


// number of cyclic frame buffer used for motion detection
// (should, probably, depend on FPS)
const int N = 4;


// ring image buffer
IplImage **buf = 0;
int last = 0;

// temporary images
IplImage *mhi = 0; // MHI
IplImage *orient = 0; // orientation
IplImage *mask = 0; // valid orientation mask
IplImage *segmask = 0; // motion segmentation map
CvMemStorage* storage = 0; // temporary storage



// same for right images
int last_right = 0;
IplImage **buf_right = 0;

IplImage *mhi_right = 0; // MHI
IplImage *orient_right = 0; // orientation
IplImage *mask_right = 0; // valid orientation mask
IplImage *segmask_right = 0; // motion segmentation map
CvMemStorage* storage_right = 0; // temporary storage

//--------------------------------------------------------



/* ------------------------------------------------------------

                     MOTION GRADIENT DETECTION RIGHT IMAGE

--------------------------------------------------------------
*/



// parameters:
//  img - input video frame
//  dst - resultant motion picture
//  args - optional parameters
void  update_mhi_right( IplImage* img,/* IplImage* dst,*/ vector<Point>* vec, int diff_threshold )
{
    double timestamp = (double)clock()/CLOCKS_PER_SEC; // get current time in seconds
    CvSize size = cvSize(img->width,img->height); // get current frame size
    //printf("width, height = %d, %d \n",img->width,img->height );
    int i, idx1 = last_right, idx2;
    IplImage* silh;
    CvSeq* seq;
    CvRect comp_rect;
    double count;
    double angle;
    CvPoint center;
    double magnitude;          
    CvScalar color;
    CvScalar colorG;

    // allocate images at the beginning or
    // reallocate them if the frame size is changed
    if( !mhi_right || mhi_right->width != size.width || mhi_right->height != size.height ) {
        if( buf_right == 0 ) {
            buf_right = (IplImage**)malloc(N*sizeof(buf_right[0]));
            memset( buf_right, 0, N*sizeof(buf_right[0]));
        }
        
        for( i = 0; i < N; i++ ) {
            cvReleaseImage( &buf_right[i] );
            buf_right[i] = cvCreateImage( size, IPL_DEPTH_8U, 1 );
            cvZero( buf_right[i] );
        }
        cvReleaseImage( &mhi_right );
        cvReleaseImage( &orient_right );
        cvReleaseImage( &segmask_right );
        cvReleaseImage( &mask_right );
        
        mhi_right = cvCreateImage( size, IPL_DEPTH_32F, 1 );
        cvZero( mhi_right ); // clear MHI at the beginning
        orient_right = cvCreateImage( size, IPL_DEPTH_32F, 1 );
        segmask_right = cvCreateImage( size, IPL_DEPTH_32F, 1 );
        mask_right = cvCreateImage( size, IPL_DEPTH_8U, 1 );
    }

    cvCvtColor( img, buf_right[last_right], CV_BGR2GRAY ); // convert frame to grayscale

    idx2 = (last_right + 1) % N; // index of (last - (N-1))th frame
    last_right = idx2;

    silh = buf_right[idx2];
    cvAbsDiff( buf_right[idx1], buf_right[idx2], silh ); // get difference between frames
    
    cvThreshold( silh, silh, diff_threshold, 1, CV_THRESH_BINARY ); // and threshold it
    cvUpdateMotionHistory( silh, mhi_right, timestamp, MHI_DURATION ); // update MHI

    // convert MHI to blue 8u image
    cvCvtScale( mhi_right, mask_right, 255./MHI_DURATION,
                (MHI_DURATION - timestamp)*255./MHI_DURATION );
    //cvZero( dst );
    //cvCvtPlaneToPix( mask, 0, 0, 0, dst );

    // calculate motion gradient orientation and valid orientation mask
    cvCalcMotionGradient( mhi_right, mask_right, orient_right, MAX_TIME_DELTA, MIN_TIME_DELTA, 3 );
    
    if( !storage_right )
        storage_right = cvCreateMemStorage(0);
    else
        cvClearMemStorage(storage_right);
    
    // segment motion: get sequence of motion components
    // segmask is marked motion components map. It is not used further
    seq = cvSegmentMotion( mhi_right, segmask_right, storage_right, timestamp, MAX_TIME_DELTA );

    // iterate through the motion components,
    // One more iteration (i == -1) corresponds to the whole image (global motion)
    for( i = 0; i < seq->total; i++ ) {

        if( i < 0 ) { // case of the whole image
            comp_rect = cvRect( 0, 0, size.width, size.height );
            color = CV_RGB(255,255,255);
            magnitude = 100;
        }
        else { // i-th motion component
            comp_rect = ((CvConnectedComp*)cvGetSeqElem( seq, i ))->rect;
            if( comp_rect.width + comp_rect.height < 100 ) // reject very small components
                continue;
            color = CV_RGB(255,0,0);
            magnitude = 30;
        }

        // select component ROI
        cvSetImageROI( silh, comp_rect );
        cvSetImageROI( mhi_right, comp_rect );
        cvSetImageROI( orient_right, comp_rect );
        cvSetImageROI( mask_right, comp_rect );

        // calculate orientation
        angle = cvCalcGlobalOrientation( orient_right, mask_right, mhi_right, timestamp, MHI_DURATION);
        angle = 360.0 - angle;  // adjust for images with top-left origin

        count = cvNorm( silh, 0, CV_L1, 0 ); // calculate number of points within silhouette ROI

        cvResetImageROI( mhi_right );
        cvResetImageROI( orient_right );
        cvResetImageROI( mask_right );
        cvResetImageROI( silh );

        // check for the case of little motion
        if( count < comp_rect.width*comp_rect.height * 0.05 )
            continue;

        // draw a clock with arrow indicating the direction
        center = cvPoint( (comp_rect.x + comp_rect.width/2),
                          (comp_rect.y + comp_rect.height/2) );

	Point p;
	p.x = center.x;
	p.y = center.y;
	vec->push_back(p);
	colorG = CV_RGB(0,255,0); 
	//cvCircle( dst, center, cvRound(magnitude*1.2), colorG, 3, CV_AA, 0 );          printf("center = %d, %d \n", center.x, center.y); 

        //cvCircle( dst, center, cvRound(magnitude*1.2), color, 3, CV_AA, 0 );
        //cvLine( dst, center, cvPoint( cvRound( center.x + magnitude*cos(angle*CV_PI/180)),
	//      cvRound( center.y - magnitude*sin(angle*CV_PI/180))), color, 3, CV_AA, 0 );
    }
}




/* ------------------------------------------------------------

                     MOTION GRADIENT DETECTION LEFT IMAGE

--------------------------------------------------------------
*/



// parameters:
//  img - input video frame
//  dst - resultant motion picture
//  args - optional parameters
void  update_mhi( IplImage* img,/* IplImage* dst,*/ vector<Point>* vec, int diff_threshold )
{
    double timestamp = (double)clock()/CLOCKS_PER_SEC; // get current time in seconds
    CvSize size = cvSize(img->width,img->height); // get current frame size
    //printf("width, height = %d, %d \n",img->width,img->height );
    int i, idx1 = last, idx2;
    IplImage* silh;
    CvSeq* seq;
    CvRect comp_rect;
    double count;
    double angle;
    CvPoint center;
    double magnitude;          
    CvScalar color;
    CvScalar colorG;

    // allocate images at the beginning or
    // reallocate them if the frame size is changed
    if( !mhi || mhi->width != size.width || mhi->height != size.height ) {
        if( buf == 0 ) {
            buf = (IplImage**)malloc(N*sizeof(buf[0]));
            memset( buf, 0, N*sizeof(buf[0]));
        }
        
        for( i = 0; i < N; i++ ) {
            cvReleaseImage( &buf[i] );
            buf[i] = cvCreateImage( size, IPL_DEPTH_8U, 1 );
            cvZero( buf[i] );
        }
        cvReleaseImage( &mhi );
        cvReleaseImage( &orient );
        cvReleaseImage( &segmask );
        cvReleaseImage( &mask );
        
        mhi = cvCreateImage( size, IPL_DEPTH_32F, 1 );
        cvZero( mhi ); // clear MHI at the beginning
        orient = cvCreateImage( size, IPL_DEPTH_32F, 1 );
        segmask = cvCreateImage( size, IPL_DEPTH_32F, 1 );
        mask = cvCreateImage( size, IPL_DEPTH_8U, 1 );
    }

    cvCvtColor( img, buf[last], CV_BGR2GRAY ); // convert frame to grayscale

    idx2 = (last + 1) % N; // index of (last - (N-1))th frame
    last = idx2;

    silh = buf[idx2];
    cvAbsDiff( buf[idx1], buf[idx2], silh ); // get difference between frames
    
    cvThreshold( silh, silh, diff_threshold, 1, CV_THRESH_BINARY ); // and threshold it
    cvUpdateMotionHistory( silh, mhi, timestamp, MHI_DURATION ); // update MHI

    // convert MHI to blue 8u image
    cvCvtScale( mhi, mask, 255./MHI_DURATION,
                (MHI_DURATION - timestamp)*255./MHI_DURATION );
    //cvZero( dst );
    //cvCvtPlaneToPix( mask, 0, 0, 0, dst );

    // calculate motion gradient orientation and valid orientation mask
    cvCalcMotionGradient( mhi, mask, orient, MAX_TIME_DELTA, MIN_TIME_DELTA, 3 );
    
    if( !storage )
        storage = cvCreateMemStorage(0);
    else
        cvClearMemStorage(storage);
    
    // segment motion: get sequence of motion components
    // segmask is marked motion components map. It is not used further
    seq = cvSegmentMotion( mhi, segmask, storage, timestamp, MAX_TIME_DELTA );

    // iterate through the motion components,
    // One more iteration (i == -1) corresponds to the whole image (global motion)
    for( i = 0; i < seq->total; i++ ) {

        if( i < 0 ) { // case of the whole image
            comp_rect = cvRect( 0, 0, size.width, size.height );
            color = CV_RGB(255,255,255);
            magnitude = 100;
        }
        else { // i-th motion component
            comp_rect = ((CvConnectedComp*)cvGetSeqElem( seq, i ))->rect;
            if( comp_rect.width + comp_rect.height < 100 ) // reject very small components
                continue;
            color = CV_RGB(255,0,0);
            magnitude = 30;
        }

        // select component ROI
        cvSetImageROI( silh, comp_rect );
        cvSetImageROI( mhi, comp_rect );
        cvSetImageROI( orient, comp_rect );
        cvSetImageROI( mask, comp_rect );

        // calculate orientation
        angle = cvCalcGlobalOrientation( orient, mask, mhi, timestamp, MHI_DURATION);
        angle = 360.0 - angle;  // adjust for images with top-left origin

        count = cvNorm( silh, 0, CV_L1, 0 ); // calculate number of points within silhouette ROI

        cvResetImageROI( mhi );
        cvResetImageROI( orient );
        cvResetImageROI( mask );
        cvResetImageROI( silh );

        // check for the case of little motion
        if( count < comp_rect.width*comp_rect.height * 0.05 )
            continue;

        // draw a clock with arrow indicating the direction
        center = cvPoint( (comp_rect.x + comp_rect.width/2),
                          (comp_rect.y + comp_rect.height/2) );

	Point p;
	p.x = center.x;
	p.y = center.y;
	vec->push_back(p);
	colorG = CV_RGB(0,255,0); 
	//cvCircle( dst, center, cvRound(magnitude*1.2), colorG, 3, CV_AA, 0 );          printf("center = %d, %d \n", center.x, center.y); 

        //cvCircle( dst, center, cvRound(magnitude*1.2), color, 3, CV_AA, 0 );
        //cvLine( dst, center, cvPoint( cvRound( center.x + magnitude*cos(angle*CV_PI/180)),
	//      cvRound( center.y - magnitude*sin(angle*CV_PI/180))), color, 3, CV_AA, 0 );
    }
}





int main(){

 Network yarp; // set up yarp
  
  // for IQR
  BufferedPort<Vector> movements_left_outPort;
  movements_left_outPort.open("/findmovements/left/out");
  BufferedPort<Vector> movements_right_outPort;
  movements_right_outPort.open("/findmovements/right/out");

 

  
  bool foundleftmove = false;
  bool foundrightmove = false;

  

  BufferedPort<ImageOf<PixelRgb> > imageLeftPort; // port for reading in images
  BufferedPort<ImageOf<PixelRgb> > imageRightPort; // port for reading in image
  imageLeftPort.open("/findmovements/image/left/in"); // give a name for port
  imageRightPort.open("/findmovements/image/right/in"); // give a name for port
  

  ImageOf<PixelRgb> *imageLeft = imageLeftPort.read(); // read an image
  ImageOf<PixelRgb> *imageRight = imageRightPort.read(); // read an image
  IplImage *cvLeftImage = cvCreateImage(cvSize(imageLeft->width(), imageLeft->height()),IPL_DEPTH_8U, 3);
  IplImage *cvRightImage = cvCreateImage(cvSize(imageRight->width(), imageRight->height()),IPL_DEPTH_8U, 3);

  
  //IplImage *cvLeftImageCopy = cvCreateImage(cvSize(imageLeft->width(), imageLeft->height()),IPL_DEPTH_8U, 3);
  //IplImage *cvRightImageCopy = cvCreateImage(cvSize(imageRight->width(), imageRight->height()),IPL_DEPTH_8U, 3);
    
  //cvNamedWindow("left-movements", 1);
  //cvNamedWindow("right-movements", 1);

  while(1){ // repeat forever
    
    
    if (left){

    imageLeft = imageLeftPort.read(); // read an image
    imageRight = imageRightPort.read(); // read an image
    if (imageLeft  != NULL){
      
      cvCvtColor((IplImage*)imageLeft->getIplImage(), cvLeftImage, CV_RGB2BGR);
      //cvCvtColor((IplImage*)imageLeft->getIplImage(), cvLeftImageCopy, CV_RGB2BGR);
      
      // do whatever in opencv with the image
      //CvSeq* torsosleft = torsoDetector.detect(cvLeftImage);
      // find left movements
      leftPoints.clear();
      if (cvLeftImage != NULL){
	update_mhi( cvLeftImage, &leftPoints, 5);
	//update_mhi( cvLeftImage, cvLeftImageCopy, &leftPoints, 5);
      }
      
           
     // for IQR
     
      Vector& leftMovements = movements_left_outPort.prepare();
      int countIQRt = 0;
      
      if (leftPoints.size() > 0){
	leftMovements.resize( leftPoints.size()* 2);
      
      }
      
      
      // draw circle on found left movements
      float scaleT = 1.0;
      for(int i = 0; i < leftPoints.size(); i++){
	
	CvPoint center;
	center.x = leftPoints.at(i).x;
	center.y = leftPoints.at(i).y - 20;
	
	//printf("drawing at: %d, %d\n", center.x, center.y);

	//cvCircle(cvLeftImage, center, 50, cvScalar(0.5), 2);


	// to send to IQR
	leftMovements[countIQRt] = center.x;
	countIQRt++;
	leftMovements[countIQRt] = center.y - 20;
	countIQRt++;

	foundleftmove = true;

      }

      

       // send to IQR
       if (foundleftmove){			
	
	 movements_left_outPort.write();
	 
      }

      // show the image
      //cvShowImage("left-movements", cvLeftImage);
      //cvWaitKey(10);
    }

    left = false;

    } // if left
    //
    // ----------------------- right image ---------------------------//
    //
    //
    else{
    
    if (imageRight  != NULL){
      cvCvtColor((IplImage*)imageRight->getIplImage(), cvRightImage, CV_RGB2BGR);     //cvCvtColor((IplImage*)imageRight->getIplImage(), cvRightImageCopy, CV_RGB2BGR);     

      rightPoints.clear();
      update_mhi_right( cvRightImage, &rightPoints, 5);


      // simulaneously send right faces to IQR
     
       Vector& rightMovements = movements_right_outPort.prepare();
      int countIQRrt = 0;
      if ( rightPoints.size() > 0){
	rightMovements.resize(rightPoints.size() * 2);
      }

      
      // draw circle on torsos
      float scalef = 1.0;
      for(int i = 0; i < rightPoints.size(); i++){
	
	CvPoint center;
	center.x = rightPoints.at(i).x;
	center.y = rightPoints.at(i).y - 20;
	
	//cvCircle(cvRightImage, center, 10.0, cvScalar(0.5));

	// to send to IQR
	rightMovements[countIQRrt] = center.x;
	countIQRrt++;
	rightMovements[countIQRrt] = center.y - 20;
	countIQRrt++;

	foundrightmove = true;
      }
      

      // show the image
      //cvShowImage("right-movements", cvRightImage);
      //cvWaitKey(10);
    }


   
 
  // send to IQR
   if (foundrightmove){		  
     movements_right_outPort.write();
    
    }
   
      
    foundleftmove = false;
    foundrightmove = false;

    left = true;
 
    } // end if !left
  }


  cvDestroyWindow("left-movements");
  cvDestroyWindow("right-movements");
  return 0;
  
}
