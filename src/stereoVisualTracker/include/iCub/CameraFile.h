#ifndef _CAMERAFILE_H_
#define _CAMERAFILE_H_

#include "opencv/cv.h"

#include "Camera.h"

class CameraFile: public Camera
{
 public:
  
  /**
   * The default constructor.
   * @param name the name of the camera.
   * @param fileName the name of the file from which grabbing image.
   */
  CameraFile(char* name, char* fileName); 

  /**
   * A destructor.
   */
  virtual ~CameraFile();

  /**
   * Open the device.
   * Call this function before beginning to work with this camera.
   */
  virtual bool open();

  /**
   * Close the device.
   * Call this function when you have finished working with this camera.
   */
  virtual void close();

  /**
   * Fills the img with the data from the file (YCbCr format).
   * If one or more of the param are NULL, then the correspondig plane
   * will not be retrieved.
   * The image passed as parameter should be just header image. Don't free the
   * image passed as parameter with cvReleaseImage, but use cvReleaseImageHeader instead.
   * @param YImg a valid image pointer that will receive the Y plane.
   * @param CbImg a valid image pointer that will receive the Cv plane.
   * @param CrImg a valid image pointer that will receive the Cr plane.
   */
  virtual void grabImage(IplImage* YImg, IplImage* CbImg, IplImage* CrImg);

 protected:
  
  char* fileName;
  IplImage* src;
  IplImage* tmp;

  IplImage* iY;
  IplImage* iCb;
  IplImage* iCr;

  
};

#endif/*_CAMERAFILE_H_*/
