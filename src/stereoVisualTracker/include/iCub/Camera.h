#ifndef _CAMERA_H_
#define _CAMERA_H_

#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include "public.h"
#ifndef NEW_VERSION
#include "frameGrabber.h"
#endif
class Camera
{
 public:
  
  /**
   * The default constructor.
   * @param name the name of the camera
   * This constructor should be only called by subclass of Camera. You should
   * always call createCameras() in the sub-classes to get the cameras present in the system.
   * @see createCameras()
   */
  Camera(char* name);
  /**
   * A destructor.
   */
  virtual ~Camera();

  /**
   * Open the device.
   * Call this function before beginning to work with this camera.
   * @return true if open succeed
   */
  virtual bool open();

  /**
   * Close the device.
   * Call this function when you have finished working with this camera.
   */
  virtual void close();

  /**
   * Check if the camera is open.
   * @return true if camera is open
   * @see open
   */
  bool isOpen() { return opened; };

  /**
   * Fills the img with the data from the camera (YCbCr format).
   * If one or more of the param are NULL, then the correspondig plane
   * will not be retrieved.
   * The image passed as parameter should be just header image. Don't free the
   * image passed as parameter with cvReleaseImage, but use cvReleaseImageHeader instead.
   * @param YImg a valid image pointer that will receive the Y plane.
   * @param CbImg a valid image pointer that will receive the Cv plane.
   * @param CrImg a valid image pointer that will receive the Cr plane.
   */
  virtual void grabImage(IplImage* YImg, IplImage* CbImg, IplImage* CrImg) = 0;


  /**
   * Get the size of the Y plane.
   * @return The size in pixel of the Y plane.
   */
  CvSize* getYSize() { return &YSize; };

  /**
   * Get the size of the Cb or Cr plane.
   * The Cb and Cr planes have the same size.
   * @return The size in pixel of the Cr or Cb plane.
   */
  CvSize* getCbCrSize() { return &CbCrSize; };
  
  /**
   * Get the camera intinsic parameters.
   * @return camera intinsic parameters.
   */
  Cv3dTrackerCameraIntrinsics* getCameraIntrinsics() {return &intParams;};

  /**
   * Set the camera intrinsic parameters.
   * @param matrix intinsic camera parameters:  [ fx 0 cx; 0 fy cy; 0 0 1 ]
   */
  void setCameraIntrinsics(CvCamera* cam);

  /**
   * Get the camera parameters.
   * @return camera parameters (include intrinsic and extrinsic parameters).
   */
    Cv3dTrackerCameraInfo* getCameraExtrinsics() {return &extParams;};

  /**
   * Set the camera parameters.
   * @param the camera parameters to set.
   */
  void setCameraExtrinsics(Cv3dTrackerCameraInfo* param);

  
  /**
   * Save the calibration parameters to a file.
   * @param filename the name of the file.
   */
  void saveCalibrationParams(char* filename);
  
  /**
   * Load the calibration parameters from a file.
   * @param filename the name of the file.
   */
  void loadCalibrationParams(char* filename);
   

 protected:

  char* name;	

  CvSize YSize;
  CvSize CbCrSize;

  bool opened;
  
  /* Camera intrinsic parameters */
  Cv3dTrackerCameraIntrinsics intParams;

  /* Camera extrinsic parameters */
  Cv3dTrackerCameraInfo extParams;

};

class CameraImage : public FrameGrabber 
{
 public:
                  CameraImage(Camera* cam, int width, int height);
  virtual         ~CameraImage();

  void            Update();
  unsigned char * GetDataPtr();
  unsigned char * GetUnDistortDataPtr();

  IplImage*       GetImage();
  IplImage*       GetUnDistortImage();
  void            GetImageYCbCr(IplImage** _YImg, IplImage** _CbImg, IplImage** _CrImg);

#ifdef NEW_VERSION
  CvSize         GetSize() ;
#else
  CvSize*         GetSize() ;
#endif

  Camera*         m_Camera; 

  void            SetCameraParams(const CvCamera* cam);
  void            SetCameraReferential(float* transMat, float* rotMat);
  void            SetCameraReferential(CvCamera* cam);

  void            LoadCameraParams(char* filename);
  void            SaveCameraParams(char* filename);
  void            GrabFrame(IplImage **frame, u32 index=0);
  // void            Kill();
  // s32             Config(char * configFile);

  float           m_TransMat[4][4];
  float           m_RotMat[4][4];

 protected:

  CvCamera        m_CameraParams;

  unsigned char * m_Data;
  unsigned char * m_DData;

  int             m_Width;
  int             m_Height;
  CvSize          m_Size;

  
  IplImage*       YCamImg;
  IplImage*       CbCamImg;
  IplImage*       CrCamImg;

  IplImage*       YImg;
  IplImage*       CbImg;
  IplImage*       CrImg;
  IplImage*       YCbCrImg;
  IplImage*       DRGBImg;
  IplImage*       RGBImg;

  IplImage*       DistMap;


};

#endif/*_CAMERA_H_*/
