// modification of existing code by Frank Broz, 2009
// camshift_wrapper.h - by Robin Hewitt, 2007
// http://www.cognotics.com/opencv/downloads/camshift_wrapper
// This is free software. See License.txt, in the download
// package, for details.
//
//
// Public interface for the Simple Camshift Wrapper

#ifndef __SIMPLE_CAMSHIFT_WRAPPER_H
#define __SIMPLE_CAMSHIFT_WRAPPER_H

#include <iCub/iha/debug.h>
#include <cv.h>


using namespace std;

namespace iCub {
    namespace contrib {
        class CamshiftWrapper;
    }
}

using namespace iCub::contrib;
using namespace iCub::iha;


class iCub::contrib::CamshiftWrapper{
  
 private:

  // Parameters
  int   nHistBins;                 // number of histogram bins
  float rangesArr[2];          // histogram range
  int vmin, vmax, smin; // limits for calculating hue
  bool initialized;
  
  // File-level variables
  IplImage * pHSVImg; // the input image converted to HSV color mode
  IplImage * pHueImg; // the Hue channel of the HSV image
  IplImage * pMask; // this image is used for masking pixels
  IplImage * pProbImg; // the face probability estimates for each pixel
  CvHistogram * pHist; // histogram of hue in the original face image
  
  CvRect prevFaceRect;  // location of face in previous frame
  CvBox2D faceBox;      // current face-location estimate
  
  int nFrames;
  
  // Declarations for internal functions
  void updateHueImage(const IplImage * pImg);
  
  
 public:
  
  CamshiftWrapper();
  virtual ~CamshiftWrapper();
  
  int createTracker(const IplImage * pImg);
  void releaseTracker();
  void startTracking(IplImage * pImg, CvRect * pRect);
  CvRect track(IplImage *);
  bool isInitialized() { return initialized; }
  
  // Parameter settings
  void setVmin(int vmin);
  void setSmin(int smin);
};

#endif
